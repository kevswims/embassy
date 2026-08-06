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
}

impl From<ClockError> for Error {
    fn from(e: ClockError) -> Self {
        Error::Clock(e)
    }
}

pub(crate) trait SealedInstance {
    /// Index into this module's per-instance waker arrays. Not the same thing
    /// as an AHB/SYSCON register index - it is just this LPUART's own number.
    const INDEX: u8;

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

        let stat = regs.stat().read();
        if stat.or() || stat.nf() || stat.fe() || stat.pf() {
            // `.write()`, not `.modify()`: STAT is W1C, and writing only the
            // bits actually seen here - rather than the whole just-read value
            // back - leaves any other pending flag (e.g. IDLE) alone.
            regs.stat().write(|w| {
                w.set_or(stat.or());
                w.set_nf(stat.nf());
                w.set_fe(stat.fe());
                w.set_pf(stat.pf());
            });

            // Priority order matches embassy-imxrt's flexcomm UART: a byte
            // that is both framing-broken and overrun is reported as the
            // condition most likely to explain the others.
            let err = if stat.or() {
                Error::Overrun
            } else if stat.fe() {
                Error::Framing
            } else if stat.pf() {
                Error::Parity
            } else {
                Error::Noise
            };
            return Poll::Ready(Err(err));
        }

        if stat.rdrf() {
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
