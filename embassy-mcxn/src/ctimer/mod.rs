//! Standard counter/timer (CTIMER) driver.
//!
//! Each CTIMER has a 32-bit counter, a prescaler, and four match registers.
//! This module currently implements only match outputs: a match can drive its
//! external match pin, which produces a waveform in hardware with no CPU
//! involvement.
//!
//! Which pad a match can reach is fixed by the chip's pin-mux table and comes
//! from the PAC metadata, so [`MatchOutput::new`] will only accept a
//! timer/channel/pin combination the hardware can actually route - see
//! [`ValidMatchConfig`].
//!
//! **Status:** match outputs only. Capture inputs have their pin traits
//! generated ([`InputPin`]) but no driver, and there is no interrupt-driven or
//! PWM support yet.

use embassy_hal_internal::{Peri, PeripheralType};

use crate::clocks::{ClockError, FunctionalClock, Gate, enable_and_reset_with_clock};
use crate::gpio::GpioPin;

/// One of a CTIMER's four match channels.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Channel {
    Zero,
    One,
    Two,
    Three,
}

impl Channel {
    #[inline]
    fn index(self) -> usize {
        match self {
            Channel::Zero => 0,
            Channel::One => 1,
            Channel::Two => 2,
            Channel::Three => 3,
        }
    }
}

pub(crate) trait SealedInstance {
    fn regs() -> crate::pac::ctimer::Ctimer;
}

/// A CTIMER peripheral.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + 'static {}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_ctimer_instance {
    ($n:literal) => {
        ::paste::paste! {
            impl crate::ctimer::SealedInstance for crate::peripherals::[<CTIMER $n>] {
                #[inline(always)]
                fn regs() -> crate::pac::ctimer::Ctimer {
                    crate::pac::[<CTIMER $n>]
                }
            }

            impl crate::ctimer::Instance for crate::peripherals::[<CTIMER $n>] {}

            crate::impl_ctimer_channel!([<CTIMER $n _CH0>], [<CTIMER $n>], Zero);
            crate::impl_ctimer_channel!([<CTIMER $n _CH1>], [<CTIMER $n>], One);
            crate::impl_ctimer_channel!([<CTIMER $n _CH2>], [<CTIMER $n>], Two);
            crate::impl_ctimer_channel!([<CTIMER $n _CH3>], [<CTIMER $n>], Three);
        }
    };
}

pub(crate) trait SealedCtimerChannel<T: Instance> {
    fn number(&self) -> Channel;
}

/// A match channel belonging to CTIMER `T`.
#[allow(private_bounds)]
pub trait CtimerChannel<T: Instance>: SealedCtimerChannel<T> + PeripheralType + 'static {}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_ctimer_channel {
    ($ch:ident, $peri:ident, $n:ident) => {
        impl crate::ctimer::SealedCtimerChannel<crate::peripherals::$peri> for crate::peripherals::$ch {
            #[inline(always)]
            fn number(&self) -> crate::ctimer::Channel {
                crate::ctimer::Channel::$n
            }
        }

        impl crate::ctimer::CtimerChannel<crate::peripherals::$peri> for crate::peripherals::$ch {}
    };
}

pub(crate) trait SealedInputPin<T: Instance> {}
pub(crate) trait SealedOutputPin<T: Instance> {}

/// A pad that can be routed to a CTIMER capture input.
///
/// The 20 capture inputs are shared: any of them can be routed to any timer
/// through INPUTMUX, so every CTIMER implements this for the same set of pads.
#[allow(private_bounds)]
pub trait InputPin<T: Instance>: GpioPin + SealedInputPin<T> + PeripheralType {
    /// Configure this pad to act as the capture input.
    fn mux(&self);
}

/// A pad that can be driven by a CTIMER match output.
#[allow(private_bounds)]
pub trait OutputPin<T: Instance>: GpioPin + SealedOutputPin<T> + PeripheralType {
    /// Configure this pad to be driven by the match output.
    fn mux(&self);
}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_ctimer_input_pin {
    ($pin:ident, $peri:ident, $fn:ident) => {
        impl crate::ctimer::SealedInputPin<crate::peripherals::$peri> for crate::peripherals::$pin {}

        impl crate::ctimer::InputPin<crate::peripherals::$peri> for crate::peripherals::$pin {
            #[inline(always)]
            fn mux(&self) {
                use crate::gpio::SealedPin;
                self.set_pull(crate::gpio::Pull::Disabled);
                self.set_slew_rate(crate::gpio::SlewRate::Fast);
                self.set_drive_strength(crate::gpio::DriveStrength::Double);
                self.set_function(crate::pac::port::Mux::$fn);
                self.set_enable_input_buffer(true);
            }
        }
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_ctimer_output_pin {
    ($pin:ident, $peri:ident, $fn:ident) => {
        impl crate::ctimer::SealedOutputPin<crate::peripherals::$peri> for crate::peripherals::$pin {}

        impl crate::ctimer::OutputPin<crate::peripherals::$peri> for crate::peripherals::$pin {
            #[inline(always)]
            fn mux(&self) {
                use crate::gpio::SealedPin;
                self.set_pull(crate::gpio::Pull::Disabled);
                self.set_slew_rate(crate::gpio::SlewRate::Fast);
                self.set_drive_strength(crate::gpio::DriveStrength::Normal);
                self.set_function(crate::pac::port::Mux::$fn);
                self.set_enable_input_buffer(false);
            }
        }
    };
}

pub(crate) trait SealedValidMatchConfig {}

/// Marker for a timer/channel/pin combination the pin-mux can route.
///
/// A match drives one specific external match pin, so a channel and a pad are
/// only usable together if the chip wires them together. This is implemented for
/// the `(timer, channel, pin)` tuples in the PAC pin-mux metadata, which makes
/// an unroutable combination a compile error rather than a silent no-op.
#[allow(private_bounds)]
pub trait ValidMatchConfig: SealedValidMatchConfig {}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_ctimer_match {
    ($peri:ident, $ch:ident, $pin:ident) => {
        impl crate::ctimer::SealedValidMatchConfig
            for (
                crate::peripherals::$peri,
                crate::peripherals::$ch,
                crate::peripherals::$pin,
            )
        {
        }

        impl crate::ctimer::ValidMatchConfig
            for (
                crate::peripherals::$peri,
                crate::peripherals::$ch,
                crate::peripherals::$pin,
            )
        {
        }
    };
}

/// A CTIMER match channel driving its external match pin.
///
/// The pin toggles on every match and the counter resets, so the output is a
/// square wave at half the match rate, generated entirely in hardware.
pub struct MatchOutput<'d, T: Instance, Ch: PeripheralType, P: PeripheralType> {
    /// Held to keep exclusive use of the timer, channel and pin for this
    /// driver's lifetime - dropping any of them early would let something else
    /// reconfigure hardware this driver is still actively using.
    _timer: Peri<'d, T>,
    _channel: Peri<'d, Ch>,
    _pin: Peri<'d, P>,
    channel: Channel,
    /// Functional clock feeding the counter, before the prescaler.
    hz: u32,
}

impl<'d, T: Instance + Gate + FunctionalClock, Ch: PeripheralType, P: PeripheralType> MatchOutput<'d, T, Ch, P> {
    /// Configure `pin` to be toggled by `channel` every `period` counter ticks.
    ///
    /// This enables and resets the timer's functional clock, so it must not be
    /// called while another driver is using the same timer. The counter starts
    /// stopped; call [`MatchOutput::start`].
    ///
    /// A `period` of 0 toggles on every tick.
    pub fn new(
        timer: Peri<'d, T>,
        channel_peri: Peri<'d, Ch>,
        pin: Peri<'d, P>,
        clock: &T::Config,
        period: u32,
    ) -> Result<Self, ClockError>
    where
        Ch: CtimerChannel<T>,
        P: OutputPin<T>,
        (T, Ch, P): ValidMatchConfig,
    {
        // SAFETY: `timer` is owned, so nothing else is driving this peripheral.
        let hz = unsafe { enable_and_reset_with_clock::<T>(clock) }?;

        let channel = channel_peri.number();
        let mut this = Self {
            _timer: timer,
            _channel: channel_peri,
            _pin: pin,
            channel,
            hz,
        };

        let regs = T::regs();
        let n = channel.index();

        // Count the functional clock directly.
        regs.pr().write(|w| w.set_prval(0));
        regs.mr(n).write(|w| w.set_match_(period));

        // Reset the counter on match, so matches repeat, and toggle the pin.
        // RM 48.3.4: EMC selects what the match does to the external match pin.
        this.set_reset_on_match(true);
        this.set_external_match(crate::pac::ctimer::Emc::Toggle);

        this._pin.mux();

        Ok(this)
    }
}

impl<T: Instance, Ch: PeripheralType, P: PeripheralType> MatchOutput<'_, T, Ch, P> {
    /// The functional clock feeding the counter, in Hz.
    ///
    /// The output toggles every `period + 1` of these ticks.
    pub fn frequency(&self) -> u32 {
        self.hz
    }

    /// Start the counter.
    pub fn start(&mut self) {
        T::regs().tcr().modify(|w| w.set_cen(true));
    }

    /// Stop the counter, leaving the output pin at its current level.
    pub fn stop(&mut self) {
        T::regs().tcr().modify(|w| w.set_cen(false));
    }

    /// The current state of the external match output.
    ///
    /// This is the timer's own view of the output, before the pin-mux, so it
    /// tracks the match logic whether or not a pad is routed to it.
    pub fn output_level(&self) -> bool {
        let emr = T::regs().emr().read();
        match self.channel {
            Channel::Zero => emr.em0(),
            Channel::One => emr.em1(),
            Channel::Two => emr.em2(),
            Channel::Three => emr.em3(),
        }
    }

    /// Change the match value, in counter ticks.
    pub fn set_period(&mut self, period: u32) {
        T::regs().mr(self.channel.index()).write(|w| w.set_match_(period));
    }

    fn set_reset_on_match(&mut self, reset: bool) {
        T::regs().mcr().modify(|w| match self.channel {
            Channel::Zero => w.set_mr0r(reset),
            Channel::One => w.set_mr1r(reset),
            Channel::Two => w.set_mr2r(reset),
            Channel::Three => w.set_mr3r(reset),
        });
    }

    fn set_external_match(&mut self, emc: crate::pac::ctimer::Emc) {
        T::regs().emr().modify(|w| match self.channel {
            Channel::Zero => w.set_emc0(emc),
            Channel::One => w.set_emc1(emc),
            Channel::Two => w.set_emc2(emc),
            Channel::Three => w.set_emc3(emc),
        });
    }
}

impl<T: Instance, Ch: PeripheralType, P: PeripheralType> Drop for MatchOutput<'_, T, Ch, P> {
    fn drop(&mut self) {
        let regs = T::regs();
        regs.tcr().modify(|w| w.set_cen(false));
        // Release the pin, which keeps the pad from being driven by a timer
        // whose driver no longer exists.
        regs.emr().modify(|w| match self.channel {
            Channel::Zero => w.set_emc0(crate::pac::ctimer::Emc::DoNothing),
            Channel::One => w.set_emc1(crate::pac::ctimer::Emc::DoNothing),
            Channel::Two => w.set_emc2(crate::pac::ctimer::Emc::DoNothing),
            Channel::Three => w.set_emc3(crate::pac::ctimer::Emc::DoNothing),
        });
    }
}
