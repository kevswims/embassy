//! USB high-speed device driver.
//!
//! Wraps [`embassy_usb_chipidea`], which implements the actual
//! `embassy-usb-driver` traits; everything MCXN-specific - clock gating, PHY
//! and PLL bring-up, DMA address translation - lives here.
//!
//! ```rust,ignore
//! bind_interrupts!(struct Irqs {
//!     USB1_HS => usb::InterruptHandler<usb::UsbHs>;
//! });
//!
//! let mut cfg = embassy_mcxn::config::Config::default();
//! cfg.clock_cfg.sosc = Some(SoscConfig { hz: 24_000_000, mode: SoscMode::Crystal });
//! let p = embassy_mcxn::init(cfg);
//!
//! let driver = usb::Driver::new(
//!     p.USB1, p.USB1_HS_PHY, p.USB1_NC, Irqs, usb::Config::default(),
//! )
//! .unwrap();
//! ```
//!
//! Requires `ClocksConfig::sosc`: the PHY PLL has no reference other than
//! `clk_in`, so USB cannot come up without a crystal configured.

mod phy;

use embassy_hal_internal::{Peri, PeripheralType};
pub use embassy_usb_chipidea::{Bus, Config, ControlPipe, Endpoint, In, Out};
use embassy_usb_chipidea::{ChipideaInstance, State, StateStorage, no_cache_op, regs};

use crate::clocks::{ClockError, Gate};
use crate::interrupt::typelevel::{Binding, Handler, Interrupt};
use crate::{interrupt, pac, peripherals};

/// The MCXN947 has eight endpoint pairs. Cross-checked against
/// `DCCPARAMS.DEN` by the driver at construction.
const MAX_EP_COUNT: usize = 8;

/// Convenience alias for the awkwardly spelled `peripherals::USB1`.
///
/// The singleton name is generated from PAC metadata, so it cannot be renamed
/// at the source; this makes `bind_interrupts!` readable.
pub type UsbHs = peripherals::USB1;

/// Errors that can occur bringing the USB controller up.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// A clock the driver needs was not brought up by `clocks::init()`.
    ///
    /// Almost always a missing [`crate::clocks::config::SoscConfig`]: the PHY
    /// PLL runs off `clk_in` and nothing else.
    Clock(ClockError),

    /// `clk_in` is running, but at a frequency the PHY PLL cannot divide from.
    UnsupportedRefClock(u32),

    /// The PHY PLL never reported lock.
    PllLock,
}

impl From<ClockError> for Error {
    fn from(e: ClockError) -> Self {
        Self::Clock(e)
    }
}

/// Translate a CPU address for the USB bus master.
///
/// The examples link RAM at `0x3000_0000`, the secure alias of the physical
/// `0x2000_0000`; the two differ in bit 28 alone. Identity is the expected
/// answer on this part, because the AHB matrix - not just the CPU's IDAU -
/// decodes the alias.
///
/// If the first SETUP never arrives (`ENDPTSETUPSTAT` stays 0) despite a clean
/// reset and correct port speed, the bus master is not decoding the alias.
/// Change this to `addr & !(1 << 28)` and retry; that is the *only* thing this
/// hook is for.
///
/// Note the obvious-looking `addr & 0x0fff_ffff` is wrong: it maps
/// `0x3000_0000` to `0x0000_0000`, which is the flash alias, so the controller
/// would DMA into flash.
fn to_dma_addr(p: *const ()) -> u32 {
    // Clear the secure-alias bit. The examples link RAM at 0x3000_0000, the
    // secure alias of the physical 0x2000_0000, and the two differ in bit 28
    // alone. `AHBSC.MASTER_SEC_LEVEL.usb_hs` reads `NonsecureNonprivMaster`,
    // so the controller cannot reach the alias and sees nothing at all there.
    //
    // Note the obvious-looking `& 0x0fff_ffff` is wrong: it maps 0x3000_0000
    // to 0x0000_0000, the *flash* alias, so the controller would DMA into
    // flash.
    (p as u32) & !(1 << 28)
}

/// Interrupt handler for [`Instance::Interrupt`].
pub struct InterruptHandler<T: Instance> {
    _phantom: core::marker::PhantomData<T>,
}

impl<T: Instance> Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        unsafe { embassy_usb_chipidea::on_interrupt(T::regs(), &T::state()) }
    }
}

trait SealedInstance {
    fn regs() -> regs::Usb;
    fn state() -> State<'static>;
}

/// A USB high-speed device controller instance.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + Gate + PeripheralType + 'static + Send {
    /// The interrupt this controller raises.
    type Interrupt: Interrupt;
}

impl SealedInstance for peripherals::USB1 {
    fn regs() -> regs::Usb {
        // SAFETY: the PAC const is the controller's base address.
        unsafe { regs::Usb::from_ptr(pac::USB1.as_ptr() as *mut ()) }
    }

    fn state() -> State<'static> {
        static STATE: StateStorage<MAX_EP_COUNT> = StateStorage::new();
        STATE.as_state()
    }
}

impl Instance for peripherals::USB1 {
    type Interrupt = interrupt::typelevel::USB1_HS;
}

/// USB high-speed device driver.
pub struct Driver<'d, T: Instance> {
    inner: embassy_usb_chipidea::Driver<'d>,
    _phantom: core::marker::PhantomData<&'d mut T>,
}

impl<'d, T: Instance> Driver<'d, T> {
    /// Bring the PHY and controller up and create the driver.
    ///
    /// Takes `USB1_HS_PHY` and `USB1_NC` as well as the controller because it
    /// writes all three; owning them stops a future charger-detect or VBUS
    /// driver from racing this one. `USBHS1_PHY_DCD` is deliberately left
    /// free.
    ///
    /// No pins: USB1_DP and USB1_DM are dedicated analog pads with no PORT
    /// mux.
    pub fn new(
        _peri: Peri<'d, T>,
        _phy: Peri<'d, peripherals::USB1_HS_PHY>,
        _usbnc: Peri<'d, peripherals::USB1_NC>,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Result<Self, Error> {
        let clk_in =
            crate::clocks::with_clocks(|c| c.ensure_clk_in()).ok_or(Error::Clock(ClockError::NeverInitialized))??;

        #[cfg(feature = "defmt")]
        defmt::info!("usb: clk_in {} Hz", clk_in);

        phy::init(clk_in)?;

        let instance = ChipideaInstance {
            regs: T::regs(),
            state: T::state(),
            endpoint_count: MAX_EP_COUNT,
            to_dma_addr,
            // Cortex-M33 with no data cache: nothing to maintain.
            cache_clean: no_cache_op,
            cache_invalidate: no_cache_op,
        };

        let inner = embassy_usb_chipidea::Driver::new(instance, config);

        T::Interrupt::unpend();
        // SAFETY: the handler is bound by the `Binding` argument.
        unsafe { T::Interrupt::enable() };

        Ok(Self {
            inner,
            _phantom: core::marker::PhantomData,
        })
    }
}

impl<'d, T: Instance> embassy_usb_driver::Driver<'d> for Driver<'d, T> {
    type EndpointOut = Endpoint<'d, Out>;
    type EndpointIn = Endpoint<'d, In>;
    type ControlPipe = ControlPipe<'d>;
    type Bus = Bus<'d>;

    fn alloc_endpoint_out(
        &mut self,
        config: &embassy_usb_driver::EndpointConfig,
    ) -> Result<Self::EndpointOut, embassy_usb_driver::EndpointAllocError> {
        self.inner.alloc_endpoint_out(config)
    }

    fn alloc_endpoint_in(
        &mut self,
        config: &embassy_usb_driver::EndpointConfig,
    ) -> Result<Self::EndpointIn, embassy_usb_driver::EndpointAllocError> {
        self.inner.alloc_endpoint_in(config)
    }

    fn start(self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        self.inner.start(control_max_packet_size)
    }
}
