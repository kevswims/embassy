//! CTIMER PWM driver.
//!
//! PWM mode repurposes up to three match channels as single-edge outputs: one
//! match register - the "period" channel - resets the counter on match and
//! never drives a pin; each other channel, once switched into PWM mode via
//! `PWMC[PWMENn]`, drives its pin low at reset and high when its own match
//! value is reached, so its duty cycle is how far from the reset boundary
//! that match sits (RM 48.7.13). A companion shadow register per duty channel
//! (`MSRn`) lets a new duty value be queued without touching the register
//! actively being compared this cycle, so [`SinglePwm::set_duty`] and friends
//! never glitch a cycle already in progress.
//!
//! Reuses [`super::ValidMatchConfig`]: a duty channel drives the same `MATn`
//! pad a match-mode channel would, so the same pin-mux routing check applies.
//! The period channel drives no pin and needs no such check.

use embassy_hal_internal::{Peri, PeripheralType};

use super::{Channel, CtimerChannel, Instance, OutputPin, ValidMatchConfig};
use crate::clocks::{ClockError, FunctionalClock, Gate, enable_and_reset_with_clock};
use crate::pac::ctimer::Pwmen;

fn configure_period<T: Instance>(channel: Channel, period: u32) {
    let regs = T::regs();
    regs.mr(channel.index()).write(|w| w.set_match_(period));
    regs.mcr().modify(|w| match channel {
        Channel::Zero => w.set_mr0r(true),
        Channel::One => w.set_mr1r(true),
        Channel::Two => w.set_mr2r(true),
        Channel::Three => w.set_mr3r(true),
    });
}

fn configure_duty<T: Instance>(channel: Channel, duty: u32) {
    let regs = T::regs();

    regs.pwmc().modify(|w| match channel {
        Channel::Zero => w.set_pwmen0(Pwmen::Pwm),
        Channel::One => w.set_pwmen1(Pwmen::Pwm),
        Channel::Two => w.set_pwmen2(Pwmen::Pwm),
        Channel::Three => w.set_pwmen3(Pwmen::Pwm),
    });

    // This channel's match must not reset, stop or interrupt the counter -
    // only the period channel does that - and MRnRL is what lets a duty
    // update queued in MSRn take effect on the next reset without touching
    // the register actively being compared this cycle.
    regs.mcr().modify(|w| match channel {
        Channel::Zero => {
            w.set_mr0r(false);
            w.set_mr0s(false);
            w.set_mr0i(false);
            w.set_mr0rl(true);
        }
        Channel::One => {
            w.set_mr1r(false);
            w.set_mr1s(false);
            w.set_mr1i(false);
            w.set_mr1rl(true);
        }
        Channel::Two => {
            w.set_mr2r(false);
            w.set_mr2s(false);
            w.set_mr2i(false);
            w.set_mr2rl(true);
        }
        Channel::Three => {
            w.set_mr3r(false);
            w.set_mr3s(false);
            w.set_mr3i(false);
            w.set_mr3rl(true);
        }
    });

    // Both the active and shadow registers get the initial value, so the
    // very first cycle already runs at the requested duty rather than
    // whatever MRn happened to hold before PWM mode was enabled.
    regs.mr(channel.index()).write(|w| w.set_match_(duty));
    regs.msr(channel.index()).write(|w| w.set_match_shadow(duty));
}

fn set_duty<T: Instance>(channel: Channel, duty: u32) {
    // Only the shadow register: MRnRL (set once in `configure_duty`) copies
    // this into the active register at the next counter reset, so a change
    // never disturbs the comparison already running this cycle.
    T::regs().msr(channel.index()).write(|w| w.set_match_shadow(duty));
}

fn release_duty_channel<T: Instance>(channel: Channel) {
    T::regs().pwmc().modify(|w| match channel {
        Channel::Zero => w.set_pwmen0(Pwmen::Match),
        Channel::One => w.set_pwmen1(Pwmen::Match),
        Channel::Two => w.set_pwmen2(Pwmen::Match),
        Channel::Three => w.set_pwmen3(Pwmen::Match),
    });
}

/// Convert a "ticks high" duty value into the register value PWM mode
/// actually compares against.
///
/// RM 48.7.13: a duty channel's pin goes *high* on its own match and is
/// *cleared low* when the counter resets - backwards from the intuitive
/// "counts up to the on-time" reading - so a *smaller* register value means
/// *more* time high. `period + 1` is outside the counter's reachable
/// `0..=period` range, so it never matches (0 ticks high); `0` matches
/// immediately after reset, giving very close to the full period high.
fn duty_register_value(period: u32, ticks_high: u32) -> u32 {
    (period + 1).saturating_sub(ticks_high)
}

/// A single PWM output: one period channel, one duty channel, one pin.
pub struct SinglePwm<'d, T: Instance, PeriodCh: PeripheralType, DutyCh: PeripheralType, P: PeripheralType> {
    _timer: Peri<'d, T>,
    _period_channel: Peri<'d, PeriodCh>,
    _duty_channel: Peri<'d, DutyCh>,
    _pin: Peri<'d, P>,
    duty_channel: Channel,
    period: u32,
    hz: u32,
}

impl<'d, T: Instance + Gate + FunctionalClock, PeriodCh: PeripheralType, DutyCh: PeripheralType, P: PeripheralType>
    SinglePwm<'d, T, PeriodCh, DutyCh, P>
{
    /// `period_channel` resets the counter every `period` ticks and drives no
    /// pin; `duty_channel` drives `pin` high for `initial_duty` of those
    /// ticks, out of `period + 1` (see [`SinglePwm::max_duty`]).
    ///
    /// This enables and resets the timer's functional clock and starts its
    /// counter, so - like [`super::MatchOutput::new`] - it must not be called
    /// while another driver already owns `timer`.
    pub fn new(
        timer: Peri<'d, T>,
        period_channel: Peri<'d, PeriodCh>,
        duty_channel: Peri<'d, DutyCh>,
        pin: Peri<'d, P>,
        clock: &T::Config,
        period: u32,
        initial_duty: u32,
    ) -> Result<Self, ClockError>
    where
        PeriodCh: CtimerChannel<T>,
        DutyCh: CtimerChannel<T>,
        P: OutputPin<T>,
        (T, DutyCh, P): ValidMatchConfig,
    {
        // SAFETY: `timer` is owned, so nothing else is driving this peripheral.
        let hz = unsafe { enable_and_reset_with_clock::<T>(clock) }?;

        let period_channel_n = period_channel.number();
        let duty_channel_n = duty_channel.number();
        let this = Self {
            _timer: timer,
            _period_channel: period_channel,
            _duty_channel: duty_channel,
            _pin: pin,
            duty_channel: duty_channel_n,
            period,
            hz,
        };

        configure_period::<T>(period_channel_n, period);
        configure_duty::<T>(duty_channel_n, duty_register_value(period, initial_duty));
        this._pin.mux();

        Ok(this)
    }
}

impl<T: Instance, PeriodCh: PeripheralType, DutyCh: PeripheralType, P: PeripheralType>
    SinglePwm<'_, T, PeriodCh, DutyCh, P>
{
    /// The functional clock feeding the counter, in Hz.
    pub fn frequency(&self) -> u32 {
        self.hz
    }

    /// The duty value, in ticks, corresponding to 100%.
    pub fn max_duty(&self) -> u32 {
        self.period + 1
    }

    /// Start the counter.
    pub fn start(&mut self) {
        T::regs().tcr().modify(|w| w.set_cen(true));
    }

    /// Stop the counter, leaving the output pin at its current level.
    pub fn stop(&mut self) {
        T::regs().tcr().modify(|w| w.set_cen(false));
    }

    /// Set how many ticks, out of [`SinglePwm::max_duty`], the pin is high.
    ///
    /// Takes effect at the next counter reset, so it never disturbs the cycle
    /// already in progress.
    pub fn set_duty(&mut self, ticks_high: u32) {
        set_duty::<T>(self.duty_channel, duty_register_value(self.period, ticks_high));
    }
}

impl<T: Instance, PeriodCh: PeripheralType, DutyCh: PeripheralType, P: PeripheralType> Drop
    for SinglePwm<'_, T, PeriodCh, DutyCh, P>
{
    fn drop(&mut self) {
        T::regs().tcr().modify(|w| w.set_cen(false));
        release_duty_channel::<T>(self.duty_channel);
    }
}

impl<T: Instance, PeriodCh: PeripheralType, DutyCh: PeripheralType, P: PeripheralType> embedded_hal_1::pwm::ErrorType
    for SinglePwm<'_, T, PeriodCh, DutyCh, P>
{
    type Error = core::convert::Infallible;
}

impl<T: Instance, PeriodCh: PeripheralType, DutyCh: PeripheralType, P: PeripheralType> embedded_hal_1::pwm::SetDutyCycle
    for SinglePwm<'_, T, PeriodCh, DutyCh, P>
{
    /// Truncated to `u16` by the trait; a `period` beyond 65535 ticks loses
    /// range here even though [`SinglePwm::set_duty`] itself does not.
    fn max_duty_cycle(&self) -> u16 {
        self.max_duty().min(u32::from(u16::MAX)) as u16
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        self.set_duty(u32::from(duty));
        Ok(())
    }
}

/// Two PWM outputs sharing one period channel, so both run at the same
/// frequency with independently settable duty cycles.
pub struct DualPwm<
    'd,
    T: Instance,
    PeriodCh: PeripheralType,
    DutyCh0: PeripheralType,
    P0: PeripheralType,
    DutyCh1: PeripheralType,
    P1: PeripheralType,
> {
    _timer: Peri<'d, T>,
    _period_channel: Peri<'d, PeriodCh>,
    _duty_channel0: Peri<'d, DutyCh0>,
    _pin0: Peri<'d, P0>,
    _duty_channel1: Peri<'d, DutyCh1>,
    _pin1: Peri<'d, P1>,
    duty_channel0: Channel,
    duty_channel1: Channel,
    period: u32,
    hz: u32,
}

impl<
    'd,
    T: Instance + Gate + FunctionalClock,
    PeriodCh: PeripheralType,
    DutyCh0: PeripheralType,
    P0: PeripheralType,
    DutyCh1: PeripheralType,
    P1: PeripheralType,
> DualPwm<'d, T, PeriodCh, DutyCh0, P0, DutyCh1, P1>
{
    /// As [`SinglePwm::new`], with a second independent duty channel and pin
    /// sharing the same period.
    pub fn new(
        timer: Peri<'d, T>,
        period_channel: Peri<'d, PeriodCh>,
        duty_channel0: Peri<'d, DutyCh0>,
        pin0: Peri<'d, P0>,
        duty_channel1: Peri<'d, DutyCh1>,
        pin1: Peri<'d, P1>,
        clock: &T::Config,
        period: u32,
        initial_duty0: u32,
        initial_duty1: u32,
    ) -> Result<Self, ClockError>
    where
        PeriodCh: CtimerChannel<T>,
        DutyCh0: CtimerChannel<T>,
        P0: OutputPin<T>,
        DutyCh1: CtimerChannel<T>,
        P1: OutputPin<T>,
        (T, DutyCh0, P0): ValidMatchConfig,
        (T, DutyCh1, P1): ValidMatchConfig,
    {
        // SAFETY: `timer` is owned, so nothing else is driving this peripheral.
        let hz = unsafe { enable_and_reset_with_clock::<T>(clock) }?;

        let period_channel_n = period_channel.number();
        let duty_channel0_n = duty_channel0.number();
        let duty_channel1_n = duty_channel1.number();
        let this = Self {
            _timer: timer,
            _period_channel: period_channel,
            _duty_channel0: duty_channel0,
            _pin0: pin0,
            _duty_channel1: duty_channel1,
            _pin1: pin1,
            duty_channel0: duty_channel0_n,
            duty_channel1: duty_channel1_n,
            period,
            hz,
        };

        configure_period::<T>(period_channel_n, period);
        configure_duty::<T>(duty_channel0_n, duty_register_value(period, initial_duty0));
        configure_duty::<T>(duty_channel1_n, duty_register_value(period, initial_duty1));
        this._pin0.mux();
        this._pin1.mux();

        Ok(this)
    }
}

impl<
    T: Instance,
    PeriodCh: PeripheralType,
    DutyCh0: PeripheralType,
    P0: PeripheralType,
    DutyCh1: PeripheralType,
    P1: PeripheralType,
> DualPwm<'_, T, PeriodCh, DutyCh0, P0, DutyCh1, P1>
{
    /// The functional clock feeding the counter, in Hz.
    pub fn frequency(&self) -> u32 {
        self.hz
    }

    /// The duty value, in ticks, corresponding to 100%.
    pub fn max_duty(&self) -> u32 {
        self.period + 1
    }

    /// Start the counter.
    pub fn start(&mut self) {
        T::regs().tcr().modify(|w| w.set_cen(true));
    }

    /// Stop the counter, leaving both output pins at their current level.
    pub fn stop(&mut self) {
        T::regs().tcr().modify(|w| w.set_cen(false));
    }

    /// Set channel 0's duty, in ticks out of [`DualPwm::max_duty`].
    pub fn set_duty0(&mut self, ticks_high: u32) {
        set_duty::<T>(self.duty_channel0, duty_register_value(self.period, ticks_high));
    }

    /// Set channel 1's duty, in ticks out of [`DualPwm::max_duty`].
    pub fn set_duty1(&mut self, ticks_high: u32) {
        set_duty::<T>(self.duty_channel1, duty_register_value(self.period, ticks_high));
    }
}

impl<
    T: Instance,
    PeriodCh: PeripheralType,
    DutyCh0: PeripheralType,
    P0: PeripheralType,
    DutyCh1: PeripheralType,
    P1: PeripheralType,
> Drop for DualPwm<'_, T, PeriodCh, DutyCh0, P0, DutyCh1, P1>
{
    fn drop(&mut self) {
        T::regs().tcr().modify(|w| w.set_cen(false));
        release_duty_channel::<T>(self.duty_channel0);
        release_duty_channel::<T>(self.duty_channel1);
    }
}

/// Three PWM outputs sharing one period channel; see [`DualPwm`].
pub struct TriplePwm<
    'd,
    T: Instance,
    PeriodCh: PeripheralType,
    DutyCh0: PeripheralType,
    P0: PeripheralType,
    DutyCh1: PeripheralType,
    P1: PeripheralType,
    DutyCh2: PeripheralType,
    P2: PeripheralType,
> {
    _timer: Peri<'d, T>,
    _period_channel: Peri<'d, PeriodCh>,
    _duty_channel0: Peri<'d, DutyCh0>,
    _pin0: Peri<'d, P0>,
    _duty_channel1: Peri<'d, DutyCh1>,
    _pin1: Peri<'d, P1>,
    _duty_channel2: Peri<'d, DutyCh2>,
    _pin2: Peri<'d, P2>,
    duty_channel0: Channel,
    duty_channel1: Channel,
    duty_channel2: Channel,
    period: u32,
    hz: u32,
}

#[allow(clippy::too_many_arguments)]
impl<
    'd,
    T: Instance + Gate + FunctionalClock,
    PeriodCh: PeripheralType,
    DutyCh0: PeripheralType,
    P0: PeripheralType,
    DutyCh1: PeripheralType,
    P1: PeripheralType,
    DutyCh2: PeripheralType,
    P2: PeripheralType,
> TriplePwm<'d, T, PeriodCh, DutyCh0, P0, DutyCh1, P1, DutyCh2, P2>
{
    /// As [`DualPwm::new`], with a third independent duty channel and pin.
    ///
    /// A timer has exactly four match channels, so this uses every one of
    /// them: one period channel plus all three duty channels.
    pub fn new(
        timer: Peri<'d, T>,
        period_channel: Peri<'d, PeriodCh>,
        duty_channel0: Peri<'d, DutyCh0>,
        pin0: Peri<'d, P0>,
        duty_channel1: Peri<'d, DutyCh1>,
        pin1: Peri<'d, P1>,
        duty_channel2: Peri<'d, DutyCh2>,
        pin2: Peri<'d, P2>,
        clock: &T::Config,
        period: u32,
        initial_duty0: u32,
        initial_duty1: u32,
        initial_duty2: u32,
    ) -> Result<Self, ClockError>
    where
        PeriodCh: CtimerChannel<T>,
        DutyCh0: CtimerChannel<T>,
        P0: OutputPin<T>,
        DutyCh1: CtimerChannel<T>,
        P1: OutputPin<T>,
        DutyCh2: CtimerChannel<T>,
        P2: OutputPin<T>,
        (T, DutyCh0, P0): ValidMatchConfig,
        (T, DutyCh1, P1): ValidMatchConfig,
        (T, DutyCh2, P2): ValidMatchConfig,
    {
        // SAFETY: `timer` is owned, so nothing else is driving this peripheral.
        let hz = unsafe { enable_and_reset_with_clock::<T>(clock) }?;

        let period_channel_n = period_channel.number();
        let duty_channel0_n = duty_channel0.number();
        let duty_channel1_n = duty_channel1.number();
        let duty_channel2_n = duty_channel2.number();
        let this = Self {
            _timer: timer,
            _period_channel: period_channel,
            _duty_channel0: duty_channel0,
            _pin0: pin0,
            _duty_channel1: duty_channel1,
            _pin1: pin1,
            _duty_channel2: duty_channel2,
            _pin2: pin2,
            duty_channel0: duty_channel0_n,
            duty_channel1: duty_channel1_n,
            duty_channel2: duty_channel2_n,
            period,
            hz,
        };

        configure_period::<T>(period_channel_n, period);
        configure_duty::<T>(duty_channel0_n, duty_register_value(period, initial_duty0));
        configure_duty::<T>(duty_channel1_n, duty_register_value(period, initial_duty1));
        configure_duty::<T>(duty_channel2_n, duty_register_value(period, initial_duty2));
        this._pin0.mux();
        this._pin1.mux();
        this._pin2.mux();

        Ok(this)
    }
}

impl<
    T: Instance,
    PeriodCh: PeripheralType,
    DutyCh0: PeripheralType,
    P0: PeripheralType,
    DutyCh1: PeripheralType,
    P1: PeripheralType,
    DutyCh2: PeripheralType,
    P2: PeripheralType,
> TriplePwm<'_, T, PeriodCh, DutyCh0, P0, DutyCh1, P1, DutyCh2, P2>
{
    /// The functional clock feeding the counter, in Hz.
    pub fn frequency(&self) -> u32 {
        self.hz
    }

    /// The duty value, in ticks, corresponding to 100%.
    pub fn max_duty(&self) -> u32 {
        self.period + 1
    }

    /// Start the counter.
    pub fn start(&mut self) {
        T::regs().tcr().modify(|w| w.set_cen(true));
    }

    /// Stop the counter, leaving all three output pins at their current level.
    pub fn stop(&mut self) {
        T::regs().tcr().modify(|w| w.set_cen(false));
    }

    /// Set channel 0's duty, in ticks out of [`TriplePwm::max_duty`].
    pub fn set_duty0(&mut self, ticks_high: u32) {
        set_duty::<T>(self.duty_channel0, duty_register_value(self.period, ticks_high));
    }

    /// Set channel 1's duty, in ticks out of [`TriplePwm::max_duty`].
    pub fn set_duty1(&mut self, ticks_high: u32) {
        set_duty::<T>(self.duty_channel1, duty_register_value(self.period, ticks_high));
    }

    /// Set channel 2's duty, in ticks out of [`TriplePwm::max_duty`].
    pub fn set_duty2(&mut self, ticks_high: u32) {
        set_duty::<T>(self.duty_channel2, duty_register_value(self.period, ticks_high));
    }
}

impl<
    T: Instance,
    PeriodCh: PeripheralType,
    DutyCh0: PeripheralType,
    P0: PeripheralType,
    DutyCh1: PeripheralType,
    P1: PeripheralType,
    DutyCh2: PeripheralType,
    P2: PeripheralType,
> Drop for TriplePwm<'_, T, PeriodCh, DutyCh0, P0, DutyCh1, P1, DutyCh2, P2>
{
    fn drop(&mut self) {
        T::regs().tcr().modify(|w| w.set_cen(false));
        release_duty_channel::<T>(self.duty_channel0);
        release_duty_channel::<T>(self.duty_channel1);
        release_duty_channel::<T>(self.duty_channel2);
    }
}
