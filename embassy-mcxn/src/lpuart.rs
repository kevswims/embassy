//! LP_FLEXCOMM-hosted LPUART driver.
//!
//! Each `LP_FLEXCOMMn` is a shared block backing one LPUART, one LPSPI and one
//! LPI2C, all overlaid on the same register page. Nothing about the
//! peripheral is usable until its own init sequence runs (RM 66.2.4): bring
//! up the flexcomm's bus clock and functional clock, then write
//! `PSELID[PERSEL]` to claim the UART personality - out of reset `PERSEL` is
//! 0 ("none"), and the clock to whichever function isn't selected is
//! internally gated, so touching an LPUART register first just reads/writes
//! nothing. [`Uart::new`] does this in the required order: clock, then
//! `PERSEL`, then the LPUART's own registers.
//!
//! This driver holds the flexcomm's `Peri<'d, T>` for its whole lifetime, the
//! same shape as [`crate::ctimer::MatchOutput`]/[`crate::ctimer::capture::Capture`],
//! which is what stops a second driver from claiming this instance for a
//! different personality while a `Uart` is still using it as one; nothing at
//! the register level enforces that on its own.
//!
//! **Status:** LPUART5 only, interrupt-driven async TX/RX, no flow control
//! (RTS_b/CTS_b pins aren't generated yet) and no DMA.

use core::future::Future;
use core::task::Poll;

use embassy_hal_internal::{Peri, PeripheralType};
use embassy_sync::waitqueue::AtomicWaker;

use crate::clocks::{ClockError, FunctionalClock, Gate, enable_and_reset_with_clock};
use crate::gpio::GpioPin;
use crate::interrupt::typelevel::{Binding, Handler, Interrupt};
use crate::pac;
use crate::pac::lp_flexcomm::Persel;
use crate::pac::lpuart::{Ilt, M, Osr, Pt, Rst, Rxflush, Sbns, Txflush};

/// Parity mode.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Parity {
    Even,
    Odd,
}

/// Number of stop bits.
#[derive(Copy, Clone, Eq, PartialEq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StopBits {
    #[default]
    One,
    Two,
}

/// LPUART configuration.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    pub baudrate: u32,
    /// `None` disables the parity bit entirely, rather than selecting a parity.
    pub parity: Option<Parity>,
    pub stop_bits: StopBits,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            baudrate: 115_200,
            parity: None,
            stop_bits: StopBits::default(),
        }
    }
}

/// Errors from constructing or using a [`Uart`].
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    Clock(ClockError),
    /// No `(OSR, SBR)` pair reproduces the requested baud rate within 3% from
    /// this functional clock.
    UnsupportedBaudrate,
    /// `STAT[OR]`: a byte arrived while the receive FIFO had no room for it.
    Overrun,
    /// `STAT[NF]`: the receiver's majority-vote sampling disagreed within a bit.
    Noise,
    /// `STAT[FE]`: no valid stop bit.
    Framing,
    /// `STAT[PF]`: parity mismatch.
    Parity,
    /// The DMA channel rejected the transfer parameters (e.g. `buf` longer
    /// than `dma::DMA_MAX_TRANSFER_SIZE`).
    InvalidDmaParameters,
    /// The DMA transfer itself faulted - see [`crate::dma::TransferErrors`].
    Dma(crate::dma::TransferErrors),
    /// A [`RingBufferedUartRx`] wasn't read fast enough and eDMA overwrote
    /// data before it was consumed. Distinct from [`Error::Overrun`]
    /// (`STAT[OR]`), which is the LPUART's own hardware FIFO overrunning.
    RingOverrun,
}

impl From<ClockError> for Error {
    fn from(e: ClockError) -> Self {
        Error::Clock(e)
    }
}

impl From<crate::dma::InvalidParameters> for Error {
    fn from(_: crate::dma::InvalidParameters) -> Self {
        Error::InvalidDmaParameters
    }
}

impl From<crate::dma::TransferErrors> for Error {
    fn from(e: crate::dma::TransferErrors) -> Self {
        Error::Dma(e)
    }
}

impl From<crate::dma::Error> for Error {
    fn from(_: crate::dma::Error) -> Self {
        // `RingBuffer::read`, this crate's only source of `dma::Error`, only
        // ever produces `Overrun`.
        Error::RingOverrun
    }
}

pub(crate) trait SealedInstance {
    /// Index into this module's per-instance waker arrays. Not the same thing
    /// as an AHB/SYSCON register index - it is just this LPUART's own number.
    const INDEX: u8;

    /// This instance's flexcomm's DMA request number for transmit.
    const TX_DMA_REQUEST: crate::dma::DmaRequest;
    /// This instance's flexcomm's DMA request number for receive.
    const RX_DMA_REQUEST: crate::dma::DmaRequest;

    fn regs() -> pac::lpuart::Lpuart;

    /// The same physical block's flexcomm-wrapper register view, for `PSELID`.
    fn flexcomm() -> pac::lp_flexcomm::LpFlexcomm;
}

/// An LPUART peripheral.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + 'static {
    /// Interrupt for this instance - shared with every other personality of
    /// the same flexcomm, though only one personality is ever selected at a
    /// time.
    type Interrupt: crate::interrupt::typelevel::Interrupt;
}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_lpuart_instance {
    ($n:literal) => {
        ::paste::paste! {
            impl crate::lpuart::SealedInstance for crate::peripherals::[<LPUART $n>] {
                const INDEX: u8 = $n;
                const TX_DMA_REQUEST: crate::dma::DmaRequest = crate::dma::DmaRequest::[<LpFlexcomm $n Tx>];
                const RX_DMA_REQUEST: crate::dma::DmaRequest = crate::dma::DmaRequest::[<LpFlexcomm $n Rx>];

                #[inline(always)]
                fn regs() -> crate::pac::lpuart::Lpuart {
                    crate::pac::[<LPUART $n>]
                }

                #[inline(always)]
                fn flexcomm() -> crate::pac::lp_flexcomm::LpFlexcomm {
                    crate::pac::[<LP_FLEXCOMM $n>]
                }
            }

            impl crate::lpuart::Instance for crate::peripherals::[<LPUART $n>] {
                type Interrupt = crate::interrupt::typelevel::[<LP_FLEXCOMM $n>];
            }
        }
    };
}

pub(crate) trait SealedTxPin<T: Instance> {}
pub(crate) trait SealedRxPin<T: Instance> {}

/// A pad that can drive an LPUART's transmit signal.
#[allow(private_bounds)]
pub trait TxPin<T: Instance>: GpioPin + SealedTxPin<T> + PeripheralType {
    /// Configure this pad to act as the UART transmit output.
    fn mux(&self);
}

/// A pad that can carry an LPUART's receive signal.
#[allow(private_bounds)]
pub trait RxPin<T: Instance>: GpioPin + SealedRxPin<T> + PeripheralType {
    /// Configure this pad to act as the UART receive input.
    fn mux(&self);
}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_lpuart_pin {
    ($lpuart:ident, $pin:ident, $mux:ident, RXD) => {
        impl crate::lpuart::SealedRxPin<crate::peripherals::$lpuart> for crate::peripherals::$pin {}

        impl crate::lpuart::RxPin<crate::peripherals::$lpuart> for crate::peripherals::$pin {
            #[inline(always)]
            fn mux(&self) {
                use crate::gpio::SealedPin;
                self.set_pull(crate::gpio::Pull::Disabled);
                self.set_slew_rate(crate::gpio::SlewRate::Fast);
                self.set_drive_strength(crate::gpio::DriveStrength::Normal);
                self.set_function(crate::pac::port::Mux::$mux);
                self.set_enable_input_buffer(true);
            }
        }
    };
    ($lpuart:ident, $pin:ident, $mux:ident, TXD) => {
        impl crate::lpuart::SealedTxPin<crate::peripherals::$lpuart> for crate::peripherals::$pin {}

        impl crate::lpuart::TxPin<crate::peripherals::$lpuart> for crate::peripherals::$pin {
            #[inline(always)]
            fn mux(&self) {
                use crate::gpio::SealedPin;
                self.set_pull(crate::gpio::Pull::Disabled);
                self.set_slew_rate(crate::gpio::SlewRate::Fast);
                self.set_drive_strength(crate::gpio::DriveStrength::Normal);
                self.set_function(crate::pac::port::Mux::$mux);
                self.set_enable_input_buffer(false);
            }
        }
    };
}

/// An LPUART, with its TX and RX pins.
pub struct Uart<'d, T: Instance + Gate, TxP: PeripheralType, RxP: PeripheralType> {
    _peri: Peri<'d, T>,
    _tx_pin: Peri<'d, TxP>,
    _rx_pin: Peri<'d, RxP>,
    hz: u32,
}

impl<'d, T: Instance + Gate + FunctionalClock, TxP: PeripheralType, RxP: PeripheralType> Uart<'d, T, TxP, RxP> {
    /// Bring up `peri`'s flexcomm as an LPUART on `tx_pin`/`rx_pin`.
    ///
    /// This enables and resets the flexcomm's functional clock, claims the
    /// UART personality (`PSELID[PERSEL]`), and must not be called while
    /// another driver already owns `peri` - including a driver for one of
    /// this flexcomm's other personalities (SPI/I2C), since they are the same
    /// physical block.
    pub fn new(
        peri: Peri<'d, T>,
        tx_pin: Peri<'d, TxP>,
        rx_pin: Peri<'d, RxP>,
        clock: &T::Config,
        config: Config,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>> + 'd,
    ) -> Result<Self, Error>
    where
        TxP: TxPin<T>,
        RxP: RxPin<T>,
    {
        // SAFETY: `peri` is owned, so nothing else is driving this peripheral.
        let hz = unsafe { enable_and_reset_with_clock::<T>(clock) }?;

        // RM 66.2.4: PERSEL must be written after the flexcomm's clock/reset
        // comes up and before any LPUART register access - out of reset it is
        // 0 ("none"), and the sub-block's clock is gated until selected.
        T::flexcomm().pselid().write(|w| w.set_persel(Persel::Uart));

        let regs = T::regs();

        // Software-reset the LPUART sub-block. This is scoped to LPUART's own
        // registers, not the flexcomm wrapper, so it does not disturb PERSEL.
        regs.global().write(|w| w.set_rst(Rst::Reset));
        regs.global().write(|w| w.set_rst(Rst::NoEffect));

        let (osr, sbr) = calculate_baud(hz, config.baudrate).ok_or(Error::UnsupportedBaudrate)?;

        regs.baud().write(|w| {
            w.set_sbr(sbr as u16);
            w.set_osr(Osr::from_bits((osr - 1) as u8));
            // Required whenever OSR is 4..=7 (RM 66.5.2.1); harmless otherwise.
            w.set_bothedge(osr < 8);
            w.set_sbns(match config.stop_bits {
                StopBits::One => Sbns::One,
                StopBits::Two => Sbns::Two,
            });
        });

        regs.ctrl().write(|w| {
            match config.parity {
                Some(parity) => {
                    w.set_pe(true);
                    w.set_pt(match parity {
                        Parity::Even => Pt::Even,
                        Parity::Odd => Pt::Odd,
                    });
                    // 8 data bits + 1 parity bit = a 9-bit frame.
                    w.set_m(M::Data9);
                }
                None => {
                    w.set_pe(false);
                    w.set_m(M::Data8);
                }
            }
            w.set_ilt(Ilt::FromStop);
        });

        regs.fifo().modify(|w| {
            w.set_txfe(true);
            w.set_rxfe(true);
        });
        regs.fifo().modify(|w| {
            w.set_txflush(Txflush::TxfifoRst);
            w.set_rxflush(Rxflush::RxfifoRst);
        });

        tx_pin.mux();
        rx_pin.mux();

        regs.ctrl().modify(|w| {
            w.set_te(true);
            w.set_re(true);
        });

        let this = Self {
            _peri: peri,
            _tx_pin: tx_pin,
            _rx_pin: rx_pin,
            hz,
        };

        T::Interrupt::unpend();
        // Safety: binding the interrupt below ensures a handler exists.
        unsafe { T::Interrupt::enable() };

        Ok(this)
    }
}

impl<T: Instance + Gate, TxP: PeripheralType, RxP: PeripheralType> Uart<'_, T, TxP, RxP> {
    /// Reprogram the baud rate on a running UART, returning the rate actually
    /// achieved.
    ///
    /// The achieved rate is `frequency() / (OSR * SBR)` and is within 3% of
    /// `baudrate` or the call fails, which is the same tolerance
    /// [`Uart::new`] applies.
    ///
    /// RM 66.5.2.1: `BAUD[SBR]` must not be written while the transmitter or
    /// receiver is enabled, so both are stopped for the update and the FIFOs are
    /// flushed before they come back. A byte arriving mid-call is lost, and a
    /// [`RingBufferedUartRx`] over this UART should be cleared afterwards -
    /// anything already in its buffer was framed at the old rate.
    pub fn set_baudrate(&self, baudrate: u32) -> Result<u32, Error> {
        let (osr, sbr) = calculate_baud(self.hz, baudrate).ok_or(Error::UnsupportedBaudrate)?;
        let regs = T::regs();

        let ctrl = regs.ctrl().read();
        regs.ctrl().modify(|w| {
            w.set_te(false);
            w.set_re(false);
        });

        regs.baud().modify(|w| {
            w.set_sbr(sbr as u16);
            w.set_osr(Osr::from_bits((osr - 1) as u8));
            // Required whenever OSR is 4..=7 (RM 66.5.2.1); harmless otherwise.
            w.set_bothedge(osr < 8);
        });

        regs.fifo().modify(|w| {
            w.set_txflush(Txflush::TxfifoRst);
            w.set_rxflush(Rxflush::RxfifoRst);
        });

        // Restore whatever was enabled rather than assuming both were.
        regs.ctrl().modify(|w| {
            w.set_te(ctrl.te());
            w.set_re(ctrl.re());
        });

        Ok(self.hz / (osr * sbr))
    }

    /// The functional clock feeding the baud-rate generator, in Hz.
    pub fn frequency(&self) -> u32 {
        self.hz
    }

    /// Write `buf`, one byte at a time, waiting for FIFO room as needed.
    ///
    /// Takes `&self`, not `&mut self`: nothing here touches instance state,
    /// only `T`'s registers and its static waker, so this can run
    /// concurrently with [`Uart::read`] (e.g. via `embassy_futures::join`)
    /// without needing a `split()` into separate TX/RX halves. That matters
    /// once a transfer is longer than the FIFO (8 words): with RX not being
    /// drained at all until a strictly sequential write finished, the excess
    /// would overrun before a subsequent `read()` ever ran.
    pub async fn write(&self, buf: &[u8]) {
        for &b in buf {
            wait_tx_ready::<T>().await;
            T::regs().data().write(|w| w.0 = u32::from(b));
        }
    }

    /// Fill `buf`, one byte at a time, waiting for data as needed.
    ///
    /// Stops at the first error, with `buf` filled up to that point. See
    /// [`Uart::write`] for why this takes `&self`.
    pub async fn read(&self, buf: &mut [u8]) -> Result<(), Error> {
        for slot in buf.iter_mut() {
            *slot = wait_rx_byte::<T>().await?;
        }
        Ok(())
    }

    /// Write `buf` in one DMA transfer, rather than interrupt-driven
    /// byte-at-a-time like [`Uart::write`].
    ///
    /// `dma` is a channel the caller already owns and constructed
    /// (`DmaChannel::new`) - not one held by this `Uart` - so a `write_dma`
    /// and a concurrent [`Uart::read`]/[`Uart::read_dma`] (with a *different*
    /// channel) compose the same way the plain interrupt-driven methods do.
    pub async fn write_dma(&self, dma: &mut crate::dma::DmaChannel<'_>, buf: &[u8]) -> Result<(), Error> {
        let peri_addr = T::regs().data().as_ptr() as *mut u8;

        // SAFETY: `peri_addr` is this instance's own DATA register, valid for
        // as long as `T::regs()` is (the whole program); `buf` outlives the
        // transfer since it is borrowed for this whole async fn.
        let transfer = unsafe {
            dma.write_to_peripheral(buf, peri_addr, T::TX_DMA_REQUEST, crate::dma::TransferOptions::COMPLETE_INTERRUPT)
        }?;

        // Only after the TCD is armed: the DMA channel is ready to service a
        // request before the LPUART is told to start making them.
        T::regs().baud().modify(|w| w.set_tdmae(true));
        let _guard = clear_tdmae_on_drop::<T>();

        transfer.await?;
        Ok(())
    }

    /// Fill `buf` in one DMA transfer, rather than interrupt-driven
    /// byte-at-a-time like [`Uart::read`].
    ///
    /// Fixed-length: this resolves only once `buf` is completely full. See
    /// [`Uart::write_dma`] for the `dma` parameter.
    pub async fn read_dma(&self, dma: &mut crate::dma::DmaChannel<'_>, buf: &mut [u8]) -> Result<(), Error> {
        let peri_addr = T::regs().data().as_ptr() as *const u8;

        // SAFETY: as `write_dma`, mirrored for the read direction.
        let transfer = unsafe {
            dma.read_from_peripheral(peri_addr, buf, T::RX_DMA_REQUEST, crate::dma::TransferOptions::COMPLETE_INTERRUPT)
        }?;

        T::regs().baud().modify(|w| w.set_rdmae(true));
        let _guard = clear_rdmae_on_drop::<T>();

        transfer.await?;
        Ok(())
    }

    /// Receive continuously into `buf` via DMA, wrapping back to the start
    /// every time it fills, rather than one fixed-length transfer like
    /// [`Uart::read_dma`].
    ///
    /// Unlike [`Uart::write_dma`]/[`Uart::read_dma`], `BAUD[RDMAE]` stays set
    /// for as long as the returned [`RingBufferedUartRx`] lives - this is a
    /// long-running receive, not a single bounded transfer, so there is no
    /// single call to scope a guard to. The returned value's `Drop` clears it.
    pub fn into_ring_rx<'ch, 'buf>(
        &self,
        dma: &'ch mut crate::dma::DmaChannel<'_>,
        buf: &'buf mut [u8],
    ) -> Result<RingBufferedUartRx<'ch, 'buf, T>, Error> {
        let peri_addr = T::regs().data().as_ptr() as *const u8;

        // SAFETY: as `read_dma`; `buf` is exclusively borrowed by the
        // returned `RingBufferedUartRx` for as long as it exists.
        let ring = unsafe { dma.setup_circular_read(peri_addr, buf, T::RX_DMA_REQUEST) }?;

        T::regs().baud().modify(|w| w.set_rdmae(true));

        Ok(RingBufferedUartRx {
            ring,
            _instance: core::marker::PhantomData,
        })
    }
}

/// A continuous DMA receive into a wrapping buffer, from [`Uart::into_ring_rx`].
pub struct RingBufferedUartRx<'a, 'buf, T: Instance> {
    ring: crate::dma::RingBuffer<'a, 'buf, u8>,
    _instance: core::marker::PhantomData<T>,
}

impl<T: Instance> RingBufferedUartRx<'_, '_, T> {
    /// Wait for at least one byte, then copy out as much as is available.
    ///
    /// Also checks/clears the LPUART error flags the way [`Uart::read`] does,
    /// since DMA moving the bytes doesn't stop the LPUART from raising them.
    pub async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        check_and_clear_rx_errors::<T>()?;
        Ok(self.ring.read(buf).await?)
    }

    /// Copy out whatever is already available, without waiting.
    pub fn read_immediate(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        check_and_clear_rx_errors::<T>()?;
        Ok(self.ring.read_immediate(buf))
    }

    /// Discard whatever is currently unread.
    pub fn clear(&mut self) {
        self.ring.clear();
    }
}

impl<T: Instance> Drop for RingBufferedUartRx<'_, '_, T> {
    fn drop(&mut self) {
        // The `RingBuffer`'s own `Drop` stops the DMA side (ERQ/EARQ); this
        // stops the LPUART side, which nothing else would otherwise do.
        T::regs().baud().modify(|w| w.set_rdmae(false));
    }
}

/// Clears `BAUD[TDMAE]`/`[RDMAE]` on every exit path from `write_dma`/
/// `read_dma` - success, an `Error`, or the `await` being dropped early by
/// cancellation. Neither `Transfer::drop` (which only knows about the DMA
/// side) nor `Uart::drop` clears these, so without this guard an aborted
/// transfer would leave a live, un-serviced DMA request enabled on the
/// LPUART side.
struct ClearOnDrop<F: FnMut()>(F);

impl<F: FnMut()> Drop for ClearOnDrop<F> {
    fn drop(&mut self) {
        (self.0)();
    }
}

fn clear_tdmae_on_drop<T: Instance>() -> ClearOnDrop<impl FnMut()> {
    ClearOnDrop(move || T::regs().baud().modify(|w| w.set_tdmae(false)))
}

fn clear_rdmae_on_drop<T: Instance>() -> ClearOnDrop<impl FnMut()> {
    ClearOnDrop(move || T::regs().baud().modify(|w| w.set_rdmae(false)))
}

impl<T: Instance + Gate, TxP: PeripheralType, RxP: PeripheralType> Drop for Uart<'_, T, TxP, RxP> {
    fn drop(&mut self) {
        // Also clears every interrupt-enable bit this driver may have set.
        T::regs().ctrl().write(|w| w.0 = 0);
        // SAFETY: this is the last user of `T`, since dropping it is what
        // releases the `Peri` that gave this driver exclusive access.
        unsafe { crate::clocks::disable::<T>() };
    }
}

fn wait_tx_ready<T: Instance>() -> impl Future<Output = ()> {
    core::future::poll_fn(move |cx| {
        let regs = T::regs();
        let index = T::INDEX as usize;

        // Register before re-enabling the interrupt and sampling the flag:
        // TDRE is a level (true for as long as the FIFO has room), not an
        // edge, so there is nothing to lose by re-checking after registering.
        tx_waker(index).register(cx.waker());
        regs.ctrl().modify(|w| w.set_tie(true));

        if regs.stat().read().tdre().to_bits() != 0 {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    })
}

/// Check `STAT[OR]/[NF]/[FE]/[PF]`, clearing whichever fired.
///
/// Shared by the interrupt-driven and DMA-backed RX paths - DMA moving the
/// bytes themselves doesn't stop the LPUART from raising these on its own.
fn check_and_clear_rx_errors<T: Instance>() -> Result<(), Error> {
    let regs = T::regs();
    let stat = regs.stat().read();

    if !(stat.or() || stat.nf() || stat.fe() || stat.pf()) {
        return Ok(());
    }

    // `.write()`, not `.modify()`: STAT is W1C, and writing only the bits
    // actually seen here - rather than the whole just-read value back -
    // leaves any other pending flag (e.g. IDLE) alone.
    regs.stat().write(|w| {
        w.set_or(stat.or());
        w.set_nf(stat.nf());
        w.set_fe(stat.fe());
        w.set_pf(stat.pf());
    });

    // Priority order matches embassy-imxrt's flexcomm UART: a byte that is
    // both framing-broken and overrun is reported as the condition most
    // likely to explain the others.
    Err(if stat.or() {
        Error::Overrun
    } else if stat.fe() {
        Error::Framing
    } else if stat.pf() {
        Error::Parity
    } else {
        Error::Noise
    })
}

fn wait_rx_byte<T: Instance>() -> impl Future<Output = Result<u8, Error>> {
    core::future::poll_fn(move |cx| {
        let regs = T::regs();
        let index = T::INDEX as usize;

        rx_waker(index).register(cx.waker());
        regs.ctrl().modify(|w| {
            w.set_rie(true);
            w.set_orie(true);
            w.set_neie(true);
            w.set_feie(true);
            w.set_peie(true);
        });

        if let Err(e) = check_and_clear_rx_errors::<T>() {
            return Poll::Ready(Err(e));
        }

        if regs.stat().read().rdrf() {
            let byte = (regs.data().read().0 & 0xFF) as u8;
            Poll::Ready(Ok(byte))
        } else {
            Poll::Pending
        }
    })
}

const INSTANCES: usize = 10;

struct State {
    tx_waker: AtomicWaker,
    rx_waker: AtomicWaker,
}

impl State {
    const fn new() -> Self {
        Self {
            tx_waker: AtomicWaker::new(),
            rx_waker: AtomicWaker::new(),
        }
    }
}

static STATES: [State; INSTANCES] = [const { State::new() }; INSTANCES];

fn tx_waker(index: usize) -> &'static AtomicWaker {
    &STATES[index].tx_waker
}

fn rx_waker(index: usize) -> &'static AtomicWaker {
    &STATES[index].rx_waker
}

/// LPUART interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: core::marker::PhantomData<T>,
}

impl<T: Instance> Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let regs = T::regs();
        let index = T::INDEX as usize;
        let ctrl = regs.ctrl().read();
        let stat = regs.stat().read();

        // Disable-on-fire, re-enable-on-poll: STAT's flags are levels that
        // stay asserted until the condition they describe changes (FIFO
        // drained, byte consumed, error cleared), not one-shot pulses, so
        // leaving the matching *IE bit set would re-fire this interrupt
        // forever until something polls again.
        if ctrl.tie() && stat.tdre().to_bits() != 0 {
            regs.ctrl().modify(|w| w.set_tie(false));
            tx_waker(index).wake();
        }

        let rx_ready = ctrl.rie() && stat.rdrf();
        let rx_error = stat.or() || stat.nf() || stat.fe() || stat.pf();
        if rx_ready || rx_error {
            regs.ctrl().modify(|w| {
                w.set_rie(false);
                w.set_orie(false);
                w.set_neie(false);
                w.set_feie(false);
                w.set_peie(false);
            });
            rx_waker(index).wake();
        }
    }
}

/// Search `osr in 4..=32` for the `(OSR, SBR)` pair closest to `baudrate`,
/// within 3%, breaking ties toward the larger `OSR` (better bit-sampling
/// margin). Mirrors embassy-imxrt's flexcomm UART search, narrowed to a small
/// window around the ideal `SBR` per `OSR` rather than a full sweep.
///
/// Returns `(osr, sbr)` as the oversampling *count* and divisor - not the
/// register values, which are each `-1` of these.
fn calculate_baud(clk_hz: u32, baudrate: u32) -> Option<(u32, u32)> {
    if baudrate == 0 {
        return None;
    }

    let mut best: Option<(u32, u32, u32)> = None; // (error, osr, sbr)

    for osr in 4..=32u32 {
        let ideal_sbr = clk_hz / (baudrate * osr);
        if ideal_sbr == 0 {
            continue;
        }

        for sbr in ideal_sbr.saturating_sub(1)..=ideal_sbr + 1 {
            if !(1..=8191).contains(&sbr) {
                continue;
            }

            let actual = clk_hz / (sbr * osr);
            let error = actual.abs_diff(baudrate);
            if error * 100 > baudrate * 3 {
                continue;
            }

            let better = match best {
                None => true,
                Some((best_error, best_osr, _)) => error < best_error || (error == best_error && osr > best_osr),
            };
            if better {
                best = Some((error, osr, sbr));
            }
        }
    }

    best.map(|(_, osr, sbr)| (osr, sbr))
}
