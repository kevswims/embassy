//! CTIMER capture-input driver.
//!
//! A capture channel timestamps the counter value on an edge of its input.
//! Routing that input is a full crossbar through INPUTMUX0: any of the 20
//! shared `CT_INPn` pads can feed any timer's any capture channel, so
//! [`Capture::new`] just writes the pin's [`InputPin::INP`] index into the
//! target channel's `ctimerXcap` register - there is no restricted
//! `ValidMatchConfig`-style marker here because every combination is valid.
//!
//! [`PwmInput`] builds on the same primitives to measure an external PWM
//! signal: since INPUTMUX routing is per-channel, one pad's `CT_INPn` can feed
//! two capture channels at once, one watching only rising edges and the other
//! only falling, without ever needing to poll the pin's level directly.

use core::future::Future;
use core::sync::atomic::{AtomicU8, Ordering};
use core::task::Poll;

use embassy_hal_internal::{Peri, PeripheralType};
use embassy_sync::waitqueue::AtomicWaker;

use super::{Channel, CtimerChannel, InputPin, Instance};
use crate::clocks::{ClockError, FunctionalClock, Gate, disable, enable, enable_and_reset_with_clock};
use crate::interrupt::typelevel::{Binding, Handler, Interrupt};
use crate::pac;
use crate::peripherals::INPUTMUX0;

/// Which edges a capture channel timestamps.
#[derive(Copy, Clone, Eq, PartialEq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Edge {
    Rising,
    Falling,
    #[default]
    Both,
}

/// A CTIMER capture channel.
///
/// Await [`Capture::capture`] to get the counter value at the next edge.
pub struct Capture<'d, T: Instance, Ch: PeripheralType, P: PeripheralType> {
    /// `Some` only when this instance brought the timer up (via [`Capture::new`])
    /// and so is the one responsible for stopping it again on drop. A second
    /// channel attached with [`Capture::new_channel`] must not stop a counter
    /// some other driver still needs.
    _timer: Option<Peri<'d, T>>,
    _channel: Peri<'d, Ch>,
    _pin: Peri<'d, P>,
    channel: Channel,
    /// Functional clock feeding the counter, before the prescaler. `None`
    /// when attached to a timer already running under another driver's clock.
    hz: Option<u32>,
}

impl<'d, T: Instance + Gate + FunctionalClock, Ch: PeripheralType, P: PeripheralType> Capture<'d, T, Ch, P> {
    /// Capture `channel` of `timer` from the edges of `pin`.
    ///
    /// `pin` need not belong to `timer` in any physical sense: the pad's
    /// `CT_INPn` signal is routed to this specific channel through INPUTMUX0,
    /// independent of which timer's own match/capture pins the pad happens to
    /// carry at other ALT settings.
    ///
    /// This enables and resets the timer's functional clock and starts its
    /// counter, so - like [`super::MatchOutput::new`] - it must not be called
    /// while another driver already owns `timer`. To add a second capture
    /// channel to a timer already running (whether started by this method or
    /// by [`super::MatchOutput`]/[`super::pwm`]), use [`Capture::new_channel`]
    /// instead.
    pub fn new(
        timer: Peri<'d, T>,
        channel_peri: Peri<'d, Ch>,
        pin: Peri<'d, P>,
        edge: Edge,
        clock: &T::Config,
        irq: impl Binding<T::Interrupt, InterruptHandler<T>> + 'd,
    ) -> Result<Self, ClockError>
    where
        Ch: CtimerChannel<T>,
        P: InputPin<T>,
    {
        // SAFETY: `timer` is owned, so nothing else is driving this peripheral.
        let hz = unsafe { enable_and_reset_with_clock::<T>(clock) }?;

        let mut this = Self::new_channel(channel_peri, pin, edge, irq);
        this._timer = Some(timer);
        this.hz = Some(hz);

        T::regs().tcr().modify(|w| w.set_cen(true));

        Ok(this)
    }

    /// The functional clock feeding the counter, in Hz.
    ///
    /// `None` for a [`Capture::new_channel`] instance: attaching to a timer
    /// already running does not itself determine that timer's rate.
    pub fn frequency(&self) -> Option<u32> {
        self.hz
    }
}

impl<'d, T: Instance, Ch: PeripheralType, P: PeripheralType> Capture<'d, T, Ch, P> {
    /// Add a capture channel to a timer some other driver already started.
    ///
    /// Unlike [`Capture::new`], this does not touch the timer's clock or
    /// counter - the timer must already be running, or edges will never
    /// advance a timestamp for this channel to capture. Dropping this
    /// instance releases only this channel, leaving the timer running for
    /// whatever else is using it.
    pub fn new_channel(
        channel_peri: Peri<'d, Ch>,
        pin: Peri<'d, P>,
        edge: Edge,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>> + 'd,
    ) -> Self
    where
        Ch: CtimerChannel<T>,
        P: InputPin<T>,
    {
        let channel = channel_peri.number();
        let this = Self {
            _timer: None,
            _channel: channel_peri,
            _pin: pin,
            channel,
            hz: None,
        };

        this._pin.mux();

        // INPUTMUX0 only needs its clock ungated for this one write; the
        // register itself holds its value with the clock off. Plain `enable`,
        // not `enable_and_reset`: INPUTMUX0 is shared across every capture
        // channel on every timer, so pulsing its reset here would wipe out
        // whatever routing another channel already configured.
        unsafe {
            enable::<INPUTMUX0>();
            write_inp_select(T::TIMER_INDEX, channel, P::INP);
            disable::<INPUTMUX0>();
        }

        configure_edges::<T>(channel, edge);
        enable_channel_interrupt::<T>(channel);

        T::Interrupt::unpend();
        // Safety: `_irq` ensures an Interrupt Handler exists.
        unsafe { T::Interrupt::enable() };

        this
    }
}

impl<'d, T: Instance, Ch: PeripheralType, P: PeripheralType> Capture<'d, T, Ch, P> {
    /// The counter value at the next captured edge.
    pub fn capture(&mut self) -> impl Future<Output = u32> + '_ {
        wait_edge::<T>(self.channel)
    }
}

impl<T: Instance, Ch: PeripheralType, P: PeripheralType> Drop for Capture<'_, T, Ch, P> {
    fn drop(&mut self) {
        if self._timer.is_some() {
            T::regs().tcr().modify(|w| w.set_cen(false));
        }
        disable_channel::<T>(self.channel);
    }
}

/// A period/duty measurement from [`PwmInput::measure`].
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Measurement {
    /// Ticks between consecutive rising edges.
    pub period: u32,
    /// Ticks the input was high within that period.
    pub high_ticks: u32,
}

/// Measures an external PWM signal's period and duty cycle.
///
/// Uses two capture channels on the same pad: one watching only rising edges,
/// the other only falling, both fed the same [`InputPin::INP`] index through
/// INPUTMUX0. This is what lets [`PwmInput::measure`] compute a duty cycle
/// without ever sampling the pin's level directly, unlike a single
/// [`Capture`] channel watching [`Edge::Both`].
pub struct PwmInput<'d, T: Instance, ChR: PeripheralType, ChF: PeripheralType, P: PeripheralType> {
    _timer: Peri<'d, T>,
    _rising_channel: Peri<'d, ChR>,
    _falling_channel: Peri<'d, ChF>,
    _pin: Peri<'d, P>,
    rising: Channel,
    falling: Channel,
    /// The rising edge that starts the *next* measurement's period. `None`
    /// only before the first [`PwmInput::measure`] call.
    last_rising: Option<u32>,
    hz: u32,
}

impl<'d, T: Instance + Gate + FunctionalClock, ChR: PeripheralType, ChF: PeripheralType, P: PeripheralType>
    PwmInput<'d, T, ChR, ChF, P>
{
    /// Measure the signal on `pin` using `rising_channel` and `falling_channel`.
    ///
    /// Like [`Capture::new`], this brings up the timer's functional clock and
    /// starts its counter, so it must not be called while another driver
    /// already owns `timer`.
    pub fn new(
        timer: Peri<'d, T>,
        rising_channel: Peri<'d, ChR>,
        falling_channel: Peri<'d, ChF>,
        pin: Peri<'d, P>,
        clock: &T::Config,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>> + 'd,
    ) -> Result<Self, ClockError>
    where
        ChR: CtimerChannel<T>,
        ChF: CtimerChannel<T>,
        P: InputPin<T>,
    {
        // SAFETY: `timer` is owned, so nothing else is driving this peripheral.
        let hz = unsafe { enable_and_reset_with_clock::<T>(clock) }?;

        let rising = rising_channel.number();
        let falling = falling_channel.number();
        let this = Self {
            _timer: timer,
            _rising_channel: rising_channel,
            _falling_channel: falling_channel,
            _pin: pin,
            rising,
            falling,
            last_rising: None,
            hz,
        };

        this._pin.mux();

        // Both channels observe the same pad - routed to two independent
        // capture registers, one per channel, is what lets one watch just the
        // rising edge and the other just the falling edge of the same signal.
        unsafe {
            enable::<INPUTMUX0>();
            write_inp_select(T::TIMER_INDEX, rising, P::INP);
            write_inp_select(T::TIMER_INDEX, falling, P::INP);
            disable::<INPUTMUX0>();
        }

        configure_edges::<T>(rising, Edge::Rising);
        configure_edges::<T>(falling, Edge::Falling);
        enable_channel_interrupt::<T>(rising);
        enable_channel_interrupt::<T>(falling);

        T::Interrupt::unpend();
        // Safety: `_irq` ensures an Interrupt Handler exists.
        unsafe { T::Interrupt::enable() };

        T::regs().tcr().modify(|w| w.set_cen(true));

        Ok(this)
    }
}

impl<T: Instance, ChR: PeripheralType, ChF: PeripheralType, P: PeripheralType> PwmInput<'_, T, ChR, ChF, P> {
    /// The functional clock feeding the counter, in Hz.
    pub fn frequency(&self) -> u32 {
        self.hz
    }

    /// The next period/duty measurement.
    ///
    /// The first call waits out a full rising-to-rising period plus whatever
    /// is left of the current cycle; every call after that needs only the
    /// remaining falling and rising edge, reusing the previous measurement's
    /// closing rising edge as this one's opening edge.
    pub async fn measure(&mut self) -> Measurement {
        if self.last_rising.is_none() {
            self.last_rising = Some(wait_edge::<T>(self.rising).await);
        }

        let high_edge = wait_edge::<T>(self.falling).await;
        let next_rising = wait_edge::<T>(self.rising).await;

        // SAFETY of the unwrap: set above, and every path below re-sets it.
        let r0 = self.last_rising.unwrap();
        let measurement = Measurement {
            period: next_rising.wrapping_sub(r0),
            high_ticks: high_edge.wrapping_sub(r0),
        };
        self.last_rising = Some(next_rising);

        measurement
    }
}

impl<T: Instance, ChR: PeripheralType, ChF: PeripheralType, P: PeripheralType> Drop for PwmInput<'_, T, ChR, ChF, P> {
    fn drop(&mut self) {
        T::regs().tcr().modify(|w| w.set_cen(false));
        disable_channel::<T>(self.rising);
        disable_channel::<T>(self.falling);
    }
}

/// Set a channel's `CAPnRE`/`CAPnFE` edge-enable bits.
fn configure_edges<T: Instance>(channel: Channel, edge: Edge) {
    let (rising, falling) = match edge {
        Edge::Rising => (true, false),
        Edge::Falling => (false, true),
        Edge::Both => (true, true),
    };
    T::regs().ccr().modify(|w| match channel {
        Channel::Zero => {
            w.set_cap0re(rising);
            w.set_cap0fe(falling);
        }
        Channel::One => {
            w.set_cap1re(rising);
            w.set_cap1fe(falling);
        }
        Channel::Two => {
            w.set_cap2re(rising);
            w.set_cap2fe(falling);
        }
        Channel::Three => {
            w.set_cap3re(rising);
            w.set_cap3fe(falling);
        }
    });
}

/// Set a channel's `CAPnI` interrupt-enable bit.
fn enable_channel_interrupt<T: Instance>(channel: Channel) {
    T::regs().ccr().modify(|w| match channel {
        Channel::Zero => w.set_cap0i(true),
        Channel::One => w.set_cap1i(true),
        Channel::Two => w.set_cap2i(true),
        Channel::Three => w.set_cap3i(true),
    });
}

/// Clear a channel's edge-enable and interrupt-enable bits.
fn disable_channel<T: Instance>(channel: Channel) {
    T::regs().ccr().modify(|w| match channel {
        Channel::Zero => {
            w.set_cap0re(false);
            w.set_cap0fe(false);
            w.set_cap0i(false);
        }
        Channel::One => {
            w.set_cap1re(false);
            w.set_cap1fe(false);
            w.set_cap1i(false);
        }
        Channel::Two => {
            w.set_cap2re(false);
            w.set_cap2fe(false);
            w.set_cap2i(false);
        }
        Channel::Three => {
            w.set_cap3re(false);
            w.set_cap3fe(false);
            w.set_cap3i(false);
        }
    });
}

/// The counter value at `channel`'s next captured edge.
///
/// The interrupt must already be enabled for `channel` for this to ever
/// resolve - see [`Capture::new`]/[`Capture::new_channel`]/[`PwmInput::new`].
fn wait_edge<T: Instance>(channel: Channel) -> impl Future<Output = u32> {
    core::future::poll_fn(move |cx| {
        let index = T::TIMER_INDEX as usize;
        let n = channel.index();
        let bit = 1 << n;

        // Register before sampling the flag. The interrupt is enabled for
        // this channel's whole lifetime, so `FLAGS` is sticky: an edge that
        // lands between two polls, or before the first one, is still
        // recorded and this returns Ready for it immediately rather than
        // waiting for a fresh edge that may never come. Registering first
        // only matters for the case where the edge lands after this poll -
        // the same reasoning as the DMA transfer future.
        waker(index).register(cx.waker());

        let prev = FLAGS[index].fetch_and(!bit, Ordering::AcqRel);
        if prev & bit != 0 {
            Poll::Ready(T::regs().cr(n).read().cap())
        } else {
            Poll::Pending
        }
    })
}

struct State {
    waker: AtomicWaker,
}

impl State {
    const fn new() -> Self {
        Self {
            waker: AtomicWaker::new(),
        }
    }
}

const TIMERS: usize = 5;

static STATES: [State; TIMERS] = [const { State::new() }; TIMERS];
static FLAGS: [AtomicU8; TIMERS] = [const { AtomicU8::new(0) }; TIMERS];

fn waker(timer: usize) -> &'static AtomicWaker {
    &STATES[timer].waker
}

/// Write `inp` into INPUTMUX0's capture-select register for `(timer, channel)`.
///
/// Each of the five timers has its own `CtimerXCapInp` enum (chiptool gives
/// every register its own type even though all five share the same 20
/// variants), which is why this is a match over timer index rather than a
/// single generic write.
///
/// # Safety
///
/// INPUTMUX0's clock must be enabled.
unsafe fn write_inp_select(timer: u8, channel: Channel, inp: u8) {
    let n = channel.index();
    macro_rules! write {
        ($t:literal) => {
            ::paste::paste! {
                match n {
                    0 => pac::INPUTMUX0.[<ctimer $t cap0>]().modify(|w| w.set_inp(crate::bits::flag(inp))),
                    1 => pac::INPUTMUX0.[<ctimer $t cap1>]().modify(|w| w.set_inp(crate::bits::flag(inp))),
                    2 => pac::INPUTMUX0.[<ctimer $t cap2>]().modify(|w| w.set_inp(crate::bits::flag(inp))),
                    3 => pac::INPUTMUX0.[<ctimer $t cap3>]().modify(|w| w.set_inp(crate::bits::flag(inp))),
                    _ => unreachable!(),
                }
            }
        };
    }

    match timer {
        0 => write!(0),
        1 => write!(1),
        2 => write!(2),
        3 => write!(3),
        4 => write!(4),
        _ => unreachable!(),
    }
}

/// CTIMER interrupt handler for capture channels.
pub struct InterruptHandler<T: Instance> {
    _phantom: core::marker::PhantomData<T>,
}

impl<T: Instance> Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let regs = T::regs();
        let ir = regs.ir().read();

        let mut mask = 0u8;
        if ir.cr0int() {
            mask |= 1 << 0;
        }
        if ir.cr1int() {
            mask |= 1 << 1;
        }
        if ir.cr2int() {
            mask |= 1 << 2;
        }
        if ir.cr3int() {
            mask |= 1 << 3;
        }

        // IR is W1C, and cr0int..cr3int sit at bits 4..7. Deriving the write
        // from `mask` rather than the raw read means this can never disturb
        // mr0int..mr3int, whatever else happens to be pending there.
        regs.ir().write(|w| w.0 = (mask as u32) << 4);

        if mask != 0 {
            FLAGS[T::TIMER_INDEX as usize].fetch_or(mask, Ordering::Release);
            waker(T::TIMER_INDEX as usize).wake();
        }
    }
}
