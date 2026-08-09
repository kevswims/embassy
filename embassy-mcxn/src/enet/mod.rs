//! Ethernet MAC (ENET0).
//!
//! MCXN947's ENET0 is a Synopsys DesignWare Ethernet QoS core: a MAC, a
//! multi-queue MTL layer, and a descriptor DMA with its own bus master. It has
//! nothing to do with the eDMA controllers in [`crate::dma`].
//!
//! Only RMII is wired up today, which is what the FRDM-MCXN947 board supports -
//! it routes TXD0/TXD1 only. The 50 MHz RMII reference clock must be supplied
//! externally on `ENET0_TX_CLK`; see [`RmiiRefClock`].
//!
//! # Example
//!
//! ```ignore
//! bind_interrupts!(struct Irqs {
//!     ETHERNET => enet::InterruptHandler<peripherals::ENET0>;
//! });
//!
//! static PACKETS: StaticCell<PacketQueue<4, 4>> = StaticCell::new();
//! let device = Enet::new(
//!     PACKETS.init(PacketQueue::new()),
//!     p.ENET0, Irqs,
//!     p.P1_4, p.P1_13, p.P1_14, p.P1_15, p.P1_6, p.P1_7, p.P1_5,
//!     p.P1_21, p.P1_20,
//!     mac_addr,
//!     enet::Config::default(),
//! ).unwrap();
//! ```

mod descriptors;
mod generic_phy;
mod sma;

use core::marker::PhantomData;
use core::mem::MaybeUninit;
use core::sync::atomic::{Ordering, fence};
use core::task::Context;

use embassy_hal_internal::{Peri, PeripheralType};
use embassy_net_driver::{Capabilities, Checksum, HardwareAddress, LinkState};
use embassy_sync::waitqueue::AtomicWaker;

use self::descriptors::{RDes, RDesRing, TDes, TDesRing};
pub use self::generic_phy::GenericPhy;
pub use self::sma::{Sma, StationManagement};
use crate::clocks::Gate;
use crate::interrupt::typelevel::{self, Handler, Interrupt as _};
use crate::pac;
// This PAC block keeps its field enums at the module root rather than in a
// `vals` submodule; alias it so the register writes read the usual way.
use crate::pac::enet as vals;

/// Largest Ethernet frame the driver hands to, or accepts from, the stack.
const MTU: usize = 1514;
const TX_BUFFER_SIZE: usize = 1514;
/// Must be a multiple of 4: `DMA_CH0_RX_CONTROL.RBSZ`'s low two bits are not
/// programmable, so the size is written as `RX_BUFFER_SIZE >> 2`.
const RX_BUFFER_SIZE: usize = 1536;

const _: () = assert!(RX_BUFFER_SIZE % 4 == 0);

/// A single packet buffer.
///
/// 8-byte aligned: the DMA requires at least 4-byte alignment for buffer
/// addresses, and over-aligning costs nothing.
#[repr(C, align(8))]
#[derive(Copy, Clone)]
pub(crate) struct Packet<const N: usize>([u8; N]);

/// Ethernet descriptor rings and packet buffers.
///
/// This owns all the memory the MAC's DMA reads and writes. `TX` and `RX` are
/// the number of packets in each ring; larger rings let the hardware absorb
/// more traffic while the CPU is busy, at the cost of RAM.
pub struct PacketQueue<const TX: usize, const RX: usize> {
    tx_desc: [TDes; TX],
    rx_desc: [RDes; RX],
    tx_buf: [Packet<TX_BUFFER_SIZE>; TX],
    rx_buf: [Packet<RX_BUFFER_SIZE>; RX],
}

impl<const TX: usize, const RX: usize> PacketQueue<TX, RX> {
    /// Create a new packet queue.
    pub const fn new() -> Self {
        Self {
            tx_desc: [const { TDes::new() }; TX],
            rx_desc: [const { RDes::new() }; RX],
            tx_buf: [Packet([0; TX_BUFFER_SIZE]); TX],
            rx_buf: [Packet([0; RX_BUFFER_SIZE]); RX],
        }
    }

    /// Initialise a packet queue in place.
    ///
    /// [`PacketQueue::new`] returns a large value, and the compiler may
    /// materialise it on the stack before moving it into its final home - which
    /// for a realistic ring size overflows the stack. Building it through a
    /// `MaybeUninit` static avoids that entirely. After this returns, calling
    /// `assume_init` on the `MaybeUninit` is sound.
    pub fn init(this: &mut MaybeUninit<Self>) {
        unsafe {
            this.as_mut_ptr().write_bytes(0u8, 1);
        }
    }
}

impl<const TX: usize, const RX: usize> Default for PacketQueue<TX, RX> {
    fn default() -> Self {
        Self::new()
    }
}

/// Source of the 50 MHz RMII reference clock.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum RmiiRefClock {
    /// Supplied from off-chip on `ENET0_TX_CLK`, by the PHY or an oscillator.
    ///
    /// This is how the FRDM-MCXN947 is wired, and it is what NXP's own SDK
    /// selects (`CLOCK_AttachClk(kNONE_to_ENETRMII)`).
    ///
    /// Sourcing the reference internally is not offered: `ENETRMIICLKSEL` can
    /// only select PLL0, CLKIN or PLL1_clk0, and [`crate::clocks`] cannot bring
    /// any of those up yet.
    #[default]
    External,
}

/// Ethernet driver configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// Where the RMII reference clock comes from.
    pub rmii_ref_clock: RmiiRefClock,
    /// Receive all frames regardless of destination address.
    ///
    /// Defaults to `true`, matching the STM32 driver: the MAC's single unicast
    /// address filter would otherwise drop the multicast traffic that IPv6 and
    /// mDNS depend on.
    pub promiscuous: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            rmii_ref_clock: RmiiRefClock::External,
            promiscuous: true,
        }
    }
}

/// Ethernet driver error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// [`crate::clocks::init`] has not run, so the CSR clock frequency needed
    /// to derive the MDIO divider is unknown.
    ClockNotConfigured,
}

/// Trait for an Ethernet PHY.
pub trait Phy {
    /// Reset the PHY and wait for it to come out of reset.
    fn phy_reset(&mut self);
    /// Initialise the PHY.
    fn phy_init(&mut self);
    /// Poll the link. Returns `None` if there is no new information yet.
    fn poll_link(&mut self, cx: &mut Context) -> Option<bool>;
}

static WAKER: AtomicWaker = AtomicWaker::new();

/// ENET interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let regs = T::info().regs();

        // Normal completion interrupts are level-driven by these W1C flags, so
        // they must be cleared here or the line re-asserts immediately. The
        // rings themselves are inspected from the poll side.
        regs.dma_ch0_status().write(|w| {
            w.set_ti(true);
            w.set_ri(true);
            w.set_nis(true);
        });

        WAKER.wake();
    }
}

/// Translate a CPU pointer into the address the ENET bus master must be given.
///
/// Secure firmware links its buffers into the secure SRAM alias
/// (`0x3000_0000`), but ENET's master port decodes addresses against the
/// non-secure alias (`0x2000_0000`) - the same physical SRAM, seen through a
/// different window. Handing the DMA a secure-alias pointer makes its very
/// first descriptor fetch fail with `DMA_CH0_STATUS.FBE`, `REB` reporting a
/// descriptor read error, and leaves the receive process stopped.
///
/// Clearing bit 28 maps either alias onto the one the master can reach, and is
/// a no-op for a non-secure build that already links there.
pub(crate) fn dma_addr<T>(ptr: *const T) -> u32 {
    (ptr as u32) & !0x1000_0000
}

/// Bring up only the MDIO interface, without starting the MAC.
///
/// Useful for identifying a PHY during board bring-up: MDC is derived from the
/// CSR (bus) clock, so this works even when the RMII reference clock is absent
/// and no cable is plugged in.
///
/// This consumes `ENET0`, so it cannot be combined with [`Enet::new`] - it is a
/// diagnostic entry point, not a step on the way to a running MAC.
pub fn sma<'d, T: Instance>(
    _peri: Peri<'d, T>,
    mdio: Peri<'d, impl MdioPin<T>>,
    mdc: Peri<'d, impl MdcPin<T>>,
) -> Result<Sma<T>, Error> {
    let csr_clk_hz = crate::clocks::with_clocks(|c| c.system_clk_hz).ok_or(Error::ClockNotConfigured)?;

    mdio.mux();
    mdc.mux();
    unsafe { crate::clocks::enable_and_reset::<T>() };

    Ok(Sma::new(csr_clk_hz))
}

/// Ethernet driver.
pub struct Enet<'d, T: Instance, P: Phy> {
    _peri: Peri<'d, T>,
    /// Ties the pins' lifetime to the driver's without needing a type-erased
    /// pin: `'d` is unified across every pin argument and the driver, so the
    /// borrow checker rejects reusing a pad while the MAC is running.
    _pins: PhantomData<&'d mut ()>,
    tx: TDesRing<'d>,
    rx: RDesRing<'d>,
    phy: P,
    mac_addr: [u8; 6],
    link_state: LinkState,
}

impl<'d, T: Instance> Enet<'d, T, GenericPhy<Sma<T>>> {
    /// Create an RMII Ethernet driver with a generic clause-22 PHY.
    ///
    /// The PHY's SMI address is discovered by probing. Use
    /// [`Enet::new_with_phy`] for a non-standard PHY or a known address.
    ///
    /// `ref_clk` is `ENET0_TX_CLK`, which in RMII carries the 50 MHz reference
    /// clock *into* the MCU, and `rx_dv` doubles as RMII's CRS_DV.
    pub fn new<const TX: usize, const RX: usize>(
        queue: &'d mut PacketQueue<TX, RX>,
        peri: Peri<'d, T>,
        irq: impl typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        ref_clk: Peri<'d, impl TxClkPin<T>>,
        rx_dv: Peri<'d, impl RxDvPin<T>>,
        rx_d0: Peri<'d, impl Rxd0Pin<T>>,
        rx_d1: Peri<'d, impl Rxd1Pin<T>>,
        tx_d0: Peri<'d, impl Txd0Pin<T>>,
        tx_d1: Peri<'d, impl Txd1Pin<T>>,
        tx_en: Peri<'d, impl TxEnPin<T>>,
        mdio: Peri<'d, impl MdioPin<T>>,
        mdc: Peri<'d, impl MdcPin<T>>,
        mac_addr: [u8; 6],
        config: Config,
    ) -> Result<Self, Error> {
        let csr_clk_hz = crate::clocks::with_clocks(|c| c.system_clk_hz).ok_or(Error::ClockNotConfigured)?;

        Self::init_hardware(
            config,
            &[
                ref_clk.mux(),
                rx_dv.mux(),
                rx_d0.mux(),
                rx_d1.mux(),
                tx_d0.mux(),
                tx_d1.mux(),
                tx_en.mux(),
                mdio.mux(),
                mdc.mux(),
            ],
        );

        let phy = GenericPhy::new_auto(Sma::new(csr_clk_hz));
        Ok(Self::finish(queue, peri, irq, phy, mac_addr, config))
    }
}

impl<'d, T: Instance, P: Phy> Enet<'d, T, P> {
    /// Create an RMII Ethernet driver with a caller-supplied PHY.
    ///
    /// Build the PHY from an [`Sma`] obtained via [`Enet::sma`] when a
    /// non-default address or a vendor-specific PHY driver is needed.
    pub fn new_with_phy<const TX: usize, const RX: usize>(
        queue: &'d mut PacketQueue<TX, RX>,
        peri: Peri<'d, T>,
        irq: impl typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        ref_clk: Peri<'d, impl TxClkPin<T>>,
        rx_dv: Peri<'d, impl RxDvPin<T>>,
        rx_d0: Peri<'d, impl Rxd0Pin<T>>,
        rx_d1: Peri<'d, impl Rxd1Pin<T>>,
        tx_d0: Peri<'d, impl Txd0Pin<T>>,
        tx_d1: Peri<'d, impl Txd1Pin<T>>,
        tx_en: Peri<'d, impl TxEnPin<T>>,
        mdio: Peri<'d, impl MdioPin<T>>,
        mdc: Peri<'d, impl MdcPin<T>>,
        mac_addr: [u8; 6],
        config: Config,
        phy: P,
    ) -> Result<Self, Error> {
        // Resolved even though this path does not build an `Sma`, so that a
        // missing clock setup fails the same way on both constructors.
        crate::clocks::with_clocks(|c| c.system_clk_hz).ok_or(Error::ClockNotConfigured)?;

        Self::init_hardware(
            config,
            &[
                ref_clk.mux(),
                rx_dv.mux(),
                rx_d0.mux(),
                rx_d1.mux(),
                tx_d0.mux(),
                tx_d1.mux(),
                tx_en.mux(),
                mdio.mux(),
                mdc.mux(),
            ],
        );

        Ok(Self::finish(queue, peri, irq, phy, mac_addr, config))
    }

    /// Access the PHY.
    pub fn phy(&self) -> &P {
        &self.phy
    }

    /// Mutably access the PHY.
    pub fn phy_mut(&mut self) -> &mut P {
        &mut self.phy
    }

    /// Clock, reset and configure the MAC, up to but not including starting it.
    fn init_hardware(config: Config, _muxed_pins: &[()]) {
        match config.rmii_ref_clock {
            // Selecting "no clock" is what tells the RMII block to take its
            // reference from the pad rather than from a SYSCON source.
            RmiiRefClock::External => pac::SYSCON0
                .enetrmiiclksel()
                .modify(|w| w.set_sel(pac::syscon::EnetrmiiclkselSel::Enum0)),
        }

        // The PHY interface must be selected before the MAC leaves reset.
        pac::SYSCON0
            .enet_phy_intf_sel()
            .modify(|w| w.set_phy_sel(pac::syscon::PhySel::Rmii));

        unsafe { crate::clocks::enable_and_reset::<T>() };

        let regs = T::info().regs();

        // Software reset clears every DMA/MTL/MAC register bank.
        regs.dma_mode().modify(|w| w.set_swr(true));
        while regs.dma_mode().read().swr() {}

        regs.mac_configuration().modify(|w| {
            w.set_ipg(vals::Ipg::Ipg96);
            // ACS strips pad+FCS from *length* frames (length field <= 1500);
            // CST strips the FCS from *type* frames. IP traffic is all type
            // frames, so without CST every received packet arrives with four
            // trailing CRC bytes.
            w.set_acs(true);
            w.set_cst(true);
            // PS selects the 10/100 port; FES then picks 100 over 10. FES is
            // meaningless without PS, so the two must be set together.
            w.set_ps(vals::Ps::M10100m);
            w.set_fes(vals::Fes::M1002500m);
            w.set_dm(vals::Dm::Fduplx);
            // RX checksum offload. Requires RX store-and-forward, set below.
            // TX insertion is requested per frame through TDES3.CIC.
            w.set_ipc(true);
        });

        // Enable RX queue 0 for generic (non-AV) traffic. This MAC is
        // multi-queue, so unlike single-queue DWMAC variants the queues come
        // out of reset disabled.
        regs.mac_rxq_ctrl0().modify(|w| w.set_rxq0en(vals::Rxq0en::EnDcbGen));

        regs.mac_packet_filter().modify(|w| w.set_pm(config.promiscuous));

        regs.mtl_rxq0_operation_mode().modify(|w| {
            w.set_rsf(true); // store and forward
            w.set_rqs(7); // 2 KB receive queue
        });
        regs.mtl_txq0_operation_mode().modify(|w| {
            w.set_tsf(true); // store and forward
            w.set_txqen(vals::MtlTxq0OperationModeTxqen::Enable);
            w.set_tqs(7); // 2 KB transmit queue
        });

        // Map RX queue 0 to DMA channel 0. This is the reset value; set it
        // explicitly so the mapping is visible in one place.
        regs.mtl_rxq_dma_map0().modify(|w| w.set_q0mdmach(false));

        regs.dma_sysbus_mode().modify(|w| {
            w.set_fb(true); // fixed burst length
            w.set_aal(true); // address-aligned beats
        });

        regs.dma_ch0_tx_control().modify(|w| w.set_tx_pbl(32));
        regs.dma_ch0_rx_control().modify(|w| {
            w.set_rx_pbl(32);
            // RBSZ is split: bits [2:1] hold RBSZ[1:0] and must stay zero, and
            // bits [14:3] hold RBSZ[13:2]. So the field takes the size shifted
            // right by two, which is why RX_BUFFER_SIZE must be a multiple of 4.
            w.set_rbsz_x_0(0);
            w.set_rbsz_13_y((RX_BUFFER_SIZE >> 2) as u16);
        });
    }

    fn finish<const TX: usize, const RX: usize>(
        queue: &'d mut PacketQueue<TX, RX>,
        peri: Peri<'d, T>,
        _irq: impl typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        phy: P,
        mac_addr: [u8; 6],
        _config: Config,
    ) -> Self {
        let regs = T::info().regs();

        // Writing the low half is what latches both halves into the MAC core,
        // so the high half must be written first.
        regs.mac_address0_high()
            .modify(|w| w.set_addrhi(u16::from(mac_addr[4]) | (u16::from(mac_addr[5]) << 8)));
        regs.mac_address0_low().write(|w| {
            w.set_addrlo(
                u32::from(mac_addr[0])
                    | (u32::from(mac_addr[1]) << 8)
                    | (u32::from(mac_addr[2]) << 16)
                    | (u32::from(mac_addr[3]) << 24),
            )
        });

        let mut this = Self {
            _peri: peri,
            _pins: PhantomData,
            tx: TDesRing::new(regs, &mut queue.tx_desc, &mut queue.tx_buf),
            rx: RDesRing::new(regs, &mut queue.rx_desc, &mut queue.rx_buf),
            phy,
            mac_addr,
            link_state: LinkState::Down,
        };

        // Every descriptor write must be visible before the engines start.
        fence(Ordering::SeqCst);

        // Clear any status latched while the rings were being set up. These are
        // all write-1-to-clear and several - FBE in particular - stay asserted
        // until acknowledged, so leaving them set would make a stale error look
        // like a live one for the rest of the driver's life.
        regs.dma_ch0_status().write(|w| w.0 = !0);

        regs.mac_configuration().modify(|w| {
            w.set_re(true);
            w.set_te(true);
        });
        regs.mtl_txq0_operation_mode().modify(|w| w.set_ftq(true));

        regs.dma_ch0_tx_control()
            .modify(|w| w.set_st(vals::DmaCh0TxControlSt::Start));
        regs.dma_ch0_rx_control()
            .modify(|w| w.set_sr(vals::DmaCh0RxControlSr::Start));

        regs.dma_ch0_interrupt_enable().modify(|w| {
            w.set_nie(true);
            w.set_rie(true);
            w.set_tie(true);
        });

        this.phy.phy_reset();
        this.phy.phy_init();

        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        this
    }
}

impl<'d, T: Instance, P: Phy> Drop for Enet<'d, T, P> {
    fn drop(&mut self) {
        let regs = T::info().regs();

        // Let anything already queued drain before tearing the MAC down.
        regs.dma_ch0_tx_control()
            .modify(|w| w.set_st(vals::DmaCh0TxControlSt::Stop));
        while {
            let d = regs.mtl_txq0_debug().read();
            d.trcsts() == vals::MtlTxq0DebugTrcsts::Read || d.txqsts()
        } {}

        regs.mac_configuration().modify(|w| {
            w.set_re(false);
            w.set_te(false);
        });

        while {
            let d = regs.mtl_rxq0_debug().read();
            d.rxqsts() != vals::MtlRxq0DebugRxqsts::Empty || d.prxq() != 0
        } {}
        regs.dma_ch0_rx_control()
            .modify(|w| w.set_sr(vals::DmaCh0RxControlSr::Stop));
    }
}

impl<'d, T: Instance, P: Phy> embassy_net_driver::Driver for Enet<'d, T, P> {
    type RxToken<'a>
        = RxToken<'a, 'd>
    where
        Self: 'a;
    type TxToken<'a>
        = TxToken<'a, 'd>
    where
        Self: 'a;

    fn receive(&mut self, cx: &mut Context) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        WAKER.register(cx.waker());

        // Only hand out an RX token together with a TX token, so the stack can
        // always answer the packet it just received.
        // The token holds a raw pointer rather than a slice so the ring can be
        // borrowed mutably alongside it; both point into the same `self`, and
        // the token's lifetime keeps that borrow alive.
        let rx: *mut [u8] = self.rx.available()?;
        let tx: *mut [u8] = self.tx.available()?;

        Some((
            RxToken {
                pkt: rx,
                rx: &mut self.rx,
            },
            TxToken {
                pkt: tx,
                tx: &mut self.tx,
            },
        ))
    }

    fn transmit(&mut self, cx: &mut Context) -> Option<Self::TxToken<'_>> {
        WAKER.register(cx.waker());
        let pkt: *mut [u8] = self.tx.available()?;
        Some(TxToken { pkt, tx: &mut self.tx })
    }

    fn capabilities(&self) -> Capabilities {
        let mut caps = Capabilities::default();
        caps.max_transmission_unit = MTU;
        caps.max_burst_size = Some(self.tx.len());
        // The MAC computes and verifies these in hardware, so the stack can
        // skip them: MAC_CONFIGURATION.IPC on receive, TDES3.CIC on transmit.
        // Frames that fail verification never reach the stack - the descriptor
        // ring drops them.
        caps.checksum.ipv4 = Checksum::None;
        caps.checksum.tcp = Checksum::None;
        caps.checksum.udp = Checksum::None;
        caps
    }

    fn link_state(&mut self, cx: &mut Context) -> LinkState {
        if let Some(up) = self.phy.poll_link(cx) {
            self.link_state = if up { LinkState::Up } else { LinkState::Down };
        }
        self.link_state
    }

    fn hardware_address(&self) -> HardwareAddress {
        HardwareAddress::Ethernet(self.mac_addr)
    }
}

/// `embassy-net` receive token.
pub struct RxToken<'a, 'd> {
    pkt: *mut [u8],
    rx: &'a mut RDesRing<'d>,
}

impl<'a, 'd> embassy_net_driver::RxToken for RxToken<'a, 'd> {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        // SAFETY: `pkt` points into the RX ring this token exclusively borrows.
        let r = f(unsafe { &mut *self.pkt });
        self.rx.pop_packet();
        r
    }
}

/// `embassy-net` transmit token.
pub struct TxToken<'a, 'd> {
    pkt: *mut [u8],
    tx: &'a mut TDesRing<'d>,
}

impl<'a, 'd> embassy_net_driver::TxToken for TxToken<'a, 'd> {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        // SAFETY: `pkt` points into the TX ring this token exclusively borrows.
        let pkt = unsafe { &mut *self.pkt };
        let r = f(&mut pkt[..len]);
        self.tx.transmit(len);
        r
    }
}

pub(crate) struct Info {
    pub(crate) regs: pac::enet::Enet,
}

impl Info {
    #[inline(always)]
    fn regs(&self) -> pac::enet::Enet {
        self.regs
    }
}

unsafe impl Sync for Info {}

pub(crate) trait SealedInstance: Gate {
    fn info() -> &'static Info;
}

/// ENET instance.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + 'static + Send {
    /// Interrupt for this instance.
    type Interrupt: typelevel::Interrupt;
}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_enet_instance {
    ($n:literal) => {
        paste::paste! {
            impl crate::enet::SealedInstance for crate::peripherals::[<ENET $n>] {
                fn info() -> &'static crate::enet::Info {
                    static INFO: crate::enet::Info = crate::enet::Info {
                        regs: crate::pac::[<ENET $n>],
                    };
                    &INFO
                }
            }

            impl crate::enet::Instance for crate::peripherals::[<ENET $n>] {
                // The NVIC vector is named after the function, not the
                // peripheral: ENET0's data-path interrupt is `ETHERNET`.
                type Interrupt = crate::interrupt::typelevel::ETHERNET;
            }
        }
    };
}

/// Declare one pin trait and its sealed counterpart.
macro_rules! enet_pin_trait {
    ($(#[$attr:meta])* $name:ident, $sealed:ident) => {
        pub(crate) trait $sealed<T: Instance> {}

        $(#[$attr])*
        #[allow(private_bounds)]
        pub trait $name<T: Instance>: crate::gpio::GpioPin + $sealed<T> + PeripheralType {
            /// Mux this pad to its ENET function.
            fn mux(&self);
        }
    };
}

enet_pin_trait!(/// MDIO management clock.
    MdcPin, SealedMdcPin);
enet_pin_trait!(/// MDIO management data.
    MdioPin, SealedMdioPin);
enet_pin_trait!(/// MII transmit clock; in RMII, the 50 MHz reference clock input.
    TxClkPin, SealedTxClkPin);
enet_pin_trait!(/// Transmit enable.
    TxEnPin, SealedTxEnPin);
enet_pin_trait!(/// Transmit data 0.
    Txd0Pin, SealedTxd0Pin);
enet_pin_trait!(/// Transmit data 1.
    Txd1Pin, SealedTxd1Pin);
enet_pin_trait!(/// Transmit data 2 (MII only).
    Txd2Pin, SealedTxd2Pin);
enet_pin_trait!(/// Transmit data 3 (MII only).
    Txd3Pin, SealedTxd3Pin);
enet_pin_trait!(/// Transmit error (MII only).
    TxErPin, SealedTxErPin);
enet_pin_trait!(/// Receive clock (MII only).
    RxClkPin, SealedRxClkPin);
enet_pin_trait!(/// Receive data valid; in RMII, CRS_DV.
    RxDvPin, SealedRxDvPin);
enet_pin_trait!(/// Receive data 0.
    Rxd0Pin, SealedRxd0Pin);
enet_pin_trait!(/// Receive data 1.
    Rxd1Pin, SealedRxd1Pin);
enet_pin_trait!(/// Receive data 2 (MII only).
    Rxd2Pin, SealedRxd2Pin);
enet_pin_trait!(/// Receive data 3 (MII only).
    Rxd3Pin, SealedRxd3Pin);
enet_pin_trait!(/// Receive error (MII only).
    RxErPin, SealedRxErPin);
enet_pin_trait!(/// Carrier sense (MII only).
    CrsPin, SealedCrsPin);
enet_pin_trait!(/// Collision detect (MII only).
    ColPin, SealedColPin);

#[doc(hidden)]
#[macro_export]
macro_rules! impl_enet_pin {
    ($pin:ident, $enet:ident, $mux:ident, $signal:ident) => {
        paste::paste! {
            impl crate::enet::[<Sealed $signal>]<crate::peripherals::$enet> for crate::peripherals::$pin {}

            impl crate::enet::$signal<crate::peripherals::$enet> for crate::peripherals::$pin {
                #[inline(always)]
                fn mux(&self) {
                    use crate::gpio::SealedPin;
                    self.set_pull(crate::gpio::Pull::Disabled);
                    self.set_slew_rate(crate::gpio::SlewRate::Fast);
                    self.set_drive_strength(crate::gpio::DriveStrength::Normal);
                    self.set_function(crate::pac::port::Mux::$mux);
                    // Every ENET pad runs with the input buffer enabled,
                    // outputs included - this matches NXP's own pin_mux for the
                    // FRDM-MCXN947, and MDIO is bidirectional besides.
                    self.set_enable_input_buffer(true);
                }
            }
        }
    };
}
