//! GPIO driver.
//!
//! PORTn is a shared MetaPAC block (`data/metadata/peripherals/mcx/PORT.yaml`
//! in `nxp-pac`, reused from MCXA) with one runtime-indexed `Pcr` register
//! (`PORTn.pcr(pin)`), same shape as MCXA's own GPIO/PORT driver.
//!
//! GPIO data registers (`PDOR`/`PSOR`/`PCOR`/`PTOR`/`PDIR`/`PDDR`/`PIDR`) are
//! ordinary bit-per-pin registers (bit N is pin N, uniformly), so [`Flex`]'s
//! operations on those just shift a runtime pin index directly rather than
//! going through the (also now available) typed per-pin field accessors.
//!
//! Pad configuration lives on the sealed pin trait as provided methods over the
//! pin's `PCR` register, so peripherals other than GPIO configure pads through
//! the same path - notably `set_function`, which the generated per-peripheral pin
//! traits use to route a pad to their own ALT setting.
//!
//! **Status:** blocking only. There's no type-erased `AnyPin`, and no
//! interrupt-driven async `wait_for_*` yet - that needs a GPIO
//! `Instance`/interrupt story like embassy-mcxa's.

use core::convert::Infallible;

use embassy_hal_internal::{Peri, PeripheralType};

/// Open-drain for GPIO pins.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OpenDrain {
    /// Output is push-pull (not open-drain).
    No,
    /// Output is open-drain.
    Yes,
}

/// Logical level for GPIO pins.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Level {
    Low,
    High,
}

impl From<bool> for Level {
    fn from(val: bool) -> Self {
        match val {
            true => Self::High,
            false => Self::Low,
        }
    }
}

/// Pull-up/down configuration for GPIO pins.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Pull {
    Disabled,
    Up,
    Down,
}

/// Output slew rate.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SlewRate {
    Fast,
    Slow,
}

/// Output drive strength.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DriveStrength {
    Normal,
    Double,
}

/// Input logic inversion.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Inverter {
    Disabled,
    Enabled,
}

/// Pad configuration, shared by every pin.
///
/// Only the four accessors are per-pin; everything else is written in terms of
/// [`SealedPin::pcr_reg`], so [`impl_gpio_pin!`] generates no register code and
/// other drivers reach pad settings the same way GPIO does.
pub(crate) trait SealedPin {
    /// The PORT/GPIO index (0-5) this pin belongs to.
    fn port(&self) -> u8;

    /// The bit position (0-31) of this pin within its PORT/GPIO.
    fn pin(&self) -> u8;

    /// The GPIO data-register block for this pin's bank.
    fn gpio(&self) -> crate::pac::gpio::Gpio;

    /// The PORT pad-control block for this pin's bank.
    fn port_reg(&self) -> crate::pac::port::Port;

    /// This pin's pad control register.
    #[inline(always)]
    fn pcr_reg(&self) -> crate::pac::common::Reg<crate::pac::port::Pcr, crate::pac::common::RW> {
        self.port_reg().pcr(self.pin() as usize)
    }

    /// Select which peripheral function drives this pin.
    ///
    /// The mux value for a given pin and peripheral comes from the PAC pin-mux
    /// metadata (RM Table 119), which is what the per-peripheral pin traits are
    /// generated from.
    #[inline(always)]
    fn set_function(&self, mux: crate::pac::port::Mux) {
        self.pcr_reg().modify(|w| w.set_mux(mux));
    }

    /// Select GPIO, which is ALT0 on every pin of this chip.
    #[inline(always)]
    fn set_function_gpio(&self) {
        self.set_function(crate::pac::port::Mux::Mux0);
    }

    #[inline(always)]
    fn set_pull(&self, pull: Pull) {
        let (pe, ps) = match pull {
            Pull::Disabled => (crate::pac::port::Pe::Pe0, crate::pac::port::Ps::Ps0),
            Pull::Up => (crate::pac::port::Pe::Pe1, crate::pac::port::Ps::Ps1),
            Pull::Down => (crate::pac::port::Pe::Pe1, crate::pac::port::Ps::Ps0),
        };
        self.pcr_reg().modify(|w| {
            w.set_pe(pe);
            w.set_ps(ps);
        });
    }

    #[inline(always)]
    fn set_drive_strength(&self, strength: DriveStrength) {
        let val = match strength {
            DriveStrength::Normal => crate::pac::port::Dse::Dse0,
            DriveStrength::Double => crate::pac::port::Dse::Dse1,
        };
        self.pcr_reg().modify(|w| w.set_dse(val));
    }

    #[inline(always)]
    fn set_slew_rate(&self, slew_rate: SlewRate) {
        let val = match slew_rate {
            SlewRate::Fast => crate::pac::port::Sre::Sre0,
            SlewRate::Slow => crate::pac::port::Sre::Sre1,
        };
        self.pcr_reg().modify(|w| w.set_sre(val));
    }

    #[inline(always)]
    fn set_enable_input_buffer(&self, enabled: bool) {
        let val = if enabled {
            crate::pac::port::Ibe::Ibe1
        } else {
            crate::pac::port::Ibe::Ibe0
        };
        self.pcr_reg().modify(|w| w.set_ibe(val));
    }

    #[inline(always)]
    fn set_open_drain(&self, open_drain: OpenDrain) {
        let val = match open_drain {
            OpenDrain::No => crate::pac::port::Ode::Ode0,
            OpenDrain::Yes => crate::pac::port::Ode::Ode1,
        };
        self.pcr_reg().modify(|w| w.set_ode(val));
    }

    #[inline(always)]
    fn set_input_inversion(&self, invert: Inverter) {
        let val = match invert {
            Inverter::Disabled => crate::pac::port::Inv::Inv0,
            Inverter::Enabled => crate::pac::port::Inv::Inv1,
        };
        self.pcr_reg().modify(|w| w.set_inv(val));
    }
}

/// GPIO pin trait, implemented for each concrete pin singleton by [`impl_gpio_pin!`].
#[allow(private_bounds)]
pub trait GpioPin: SealedPin + PeripheralType + Sized + 'static {}

pub(crate) trait SealedInstance {}

/// Marker trait for a GPIO bank (`GPIO0`..`GPIO5`), implemented by [`impl_gpio_instance!`].
///
/// This is currently just a marker: embassy-mcxn's build.rs (mirroring
/// embassy-mcxa's) requires a `GPIOn` `Instance` to exist once GPIO has a
/// `block` in its metadata, but there's no interrupt-driven async support
/// (`wait_for_*`) yet - that needs its own `Instance`/interrupt story like
/// embassy-mcxa's, at which point this trait grows real content.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType {}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_gpio_instance {
    ($n:expr) => {
        ::paste::paste! {
            impl crate::gpio::SealedInstance for crate::peripherals::[<GPIO $n>] {}
            impl crate::gpio::Instance for crate::peripherals::[<GPIO $n>] {}
        }
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_gpio_pin {
    ($peri:ident, $port:expr, $pin:expr, $block:ident) => {
        ::paste::paste! {
            impl crate::gpio::SealedPin for crate::peripherals::$peri {
                #[inline(always)]
                fn port(&self) -> u8 {
                    $port
                }

                #[inline(always)]
                fn pin(&self) -> u8 {
                    $pin
                }

                #[inline(always)]
                fn gpio(&self) -> crate::pac::gpio::Gpio {
                    crate::pac::$block
                }

                #[inline(always)]
                fn port_reg(&self) -> crate::pac::port::Port {
                    crate::pac::[<PORT $port>]
                }
            }

            impl crate::gpio::GpioPin for crate::peripherals::$peri {}
        }
    };
}

/// A flexible pin that can be configured as input or output.
pub struct Flex<'d, P: GpioPin> {
    pin: Peri<'d, P>,
}

impl<'d, P: GpioPin> Flex<'d, P> {
    /// Wrap the pin in a `Flex`.
    ///
    /// The pin remains unmodified. The initial output level is unspecified, but
    /// can be changed before the pin is put into output mode.
    pub fn new(pin: Peri<'d, P>) -> Self {
        pin.set_function_gpio();
        Self { pin }
    }

    #[inline]
    fn gpio(&self) -> crate::pac::gpio::Gpio {
        self.pin.gpio()
    }

    #[inline]
    fn bit(&self) -> u32 {
        1 << self.pin.pin()
    }

    /// The PORT/GPIO index (0-5) this pin belongs to.
    #[inline]
    pub fn port_index(&self) -> u8 {
        self.pin.port()
    }

    /// The bit position (0-31) of this pin within its PORT/GPIO.
    #[inline]
    pub fn pin_index(&self) -> u8 {
        self.pin.pin()
    }

    /// Put the pin into input mode.
    pub fn set_as_input(&mut self) {
        self.set_enable_input_buffer(true);
        let bit = self.bit();
        // Clear PIDR to enable pin input.
        self.gpio().pidr().modify(|w| w.0 &= !bit);
        self.gpio().pddr().modify(|w| w.0 &= !bit);
    }

    /// Put the pin into output mode.
    pub fn set_as_output(&mut self) {
        self.set_pull(Pull::Disabled);
        let bit = self.bit();
        // Clear PIDR in case this pin was previously disabled.
        self.gpio().pidr().modify(|w| w.0 &= !bit);
        self.gpio().pddr().modify(|w| w.0 |= bit);
    }

    /// Disable the pin: direction input, input buffer off, GPIO mux, marked disabled.
    fn set_as_disabled(&mut self) {
        let bit = self.bit();
        self.gpio().pddr().modify(|w| w.0 &= !bit);
        self.set_enable_input_buffer(false);
        self.pin.set_function_gpio();
        self.gpio().pidr().modify(|w| w.0 |= bit);
    }

    /// Set output level to High.
    #[inline]
    pub fn set_high(&mut self) {
        let bit = self.bit();
        self.gpio().psor().write(|w| w.0 = bit);
    }

    /// Set output level to Low.
    #[inline]
    pub fn set_low(&mut self) {
        let bit = self.bit();
        self.gpio().pcor().write(|w| w.0 = bit);
    }

    /// Set output level to the given `Level`.
    #[inline]
    pub fn set_level(&mut self, level: Level) {
        match level {
            Level::High => self.set_high(),
            Level::Low => self.set_low(),
        }
    }

    /// Toggle output level.
    #[inline]
    pub fn toggle(&mut self) {
        let bit = self.bit();
        self.gpio().ptor().write(|w| w.0 = bit);
    }

    /// Get whether the pin input level is high.
    #[inline]
    pub fn is_high(&self) -> bool {
        (self.gpio().pdir().read().0 & self.bit()) != 0
    }

    /// Get whether the pin input level is low.
    #[inline]
    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    /// Is the output pin set as high?
    #[inline]
    pub fn is_set_high(&self) -> bool {
        (self.gpio().pdor().read().0 & self.bit()) != 0
    }

    /// Is the output pin set as low?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        !self.is_set_high()
    }

    /// Get pin level.
    pub fn get_level(&self) -> Level {
        self.is_high().into()
    }

    /// Configure open-drain output.
    #[inline]
    pub fn set_open_drain(&mut self, open_drain: OpenDrain) {
        self.pin.set_open_drain(open_drain);
    }

    /// Configure the input logic inversion of this pin.
    #[inline]
    pub fn set_input_inversion(&mut self, invert: Inverter) {
        self.pin.set_input_inversion(invert);
    }

    /// Configure the pin pull up/down level.
    pub fn set_pull(&mut self, pull: Pull) {
        self.pin.set_pull(pull);
    }

    /// Configure the pin drive strength.
    pub fn set_drive_strength(&mut self, strength: DriveStrength) {
        self.pin.set_drive_strength(strength);
    }

    /// Configure the pin slew rate.
    pub fn set_slew_rate(&mut self, slew_rate: SlewRate) {
        self.pin.set_slew_rate(slew_rate);
    }

    /// Enable the input buffer for the pin.
    pub fn set_enable_input_buffer(&mut self, enabled: bool) {
        self.pin.set_enable_input_buffer(enabled);
    }
}

impl<'d, P: GpioPin> Drop for Flex<'d, P> {
    #[inline]
    fn drop(&mut self) {
        self.set_as_disabled();
    }
}

/// GPIO output driver that owns a `Flex` pin.
pub struct Output<'d, P: GpioPin> {
    flex: Flex<'d, P>,
}

impl<'d, P: GpioPin> Output<'d, P> {
    /// Create a GPIO output driver for a [`GpioPin`] with the provided [`Level`].
    pub fn new(pin: Peri<'d, P>, initial: Level, strength: DriveStrength, slew_rate: SlewRate) -> Self {
        let mut flex = Flex::new(pin);
        flex.set_level(initial);
        flex.set_as_output();
        flex.set_drive_strength(strength);
        flex.set_slew_rate(slew_rate);
        flex.set_open_drain(OpenDrain::No);
        Self { flex }
    }

    /// Set the output as high.
    #[inline]
    pub fn set_high(&mut self) {
        self.flex.set_high();
    }

    /// Set the output as low.
    #[inline]
    pub fn set_low(&mut self) {
        self.flex.set_low();
    }

    /// Set the output level.
    #[inline]
    pub fn set_level(&mut self, level: Level) {
        self.flex.set_level(level);
    }

    /// Toggle the output level.
    #[inline]
    pub fn toggle(&mut self) {
        self.flex.toggle();
    }

    /// Is the output pin set as high?
    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.flex.is_set_high()
    }

    /// Is the output pin set as low?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        self.flex.is_set_low()
    }

    /// Expose the inner `Flex` if callers need to reconfigure the pin.
    #[inline]
    pub fn into_flex(self) -> Flex<'d, P> {
        self.flex
    }

    /// Convert this output pin into an open-drain output pin.
    #[inline]
    pub fn into_open_drain(mut self) -> OutputOpenDrain<'d, P> {
        self.flex.set_open_drain(OpenDrain::Yes);
        OutputOpenDrain { flex: self.flex }
    }
}

/// GPIO open-drain output driver that owns a `Flex` pin.
pub struct OutputOpenDrain<'d, P: GpioPin> {
    flex: Flex<'d, P>,
}

impl<'d, P: GpioPin> OutputOpenDrain<'d, P> {
    /// Create a GPIO open-drain output driver for a [`GpioPin`] with the provided [`Level`].
    pub fn new(pin: Peri<'d, P>, initial: Level, strength: DriveStrength, slew_rate: SlewRate) -> Self {
        let mut flex = Flex::new(pin);
        flex.set_level(initial);
        flex.set_as_output();
        flex.set_drive_strength(strength);
        flex.set_slew_rate(slew_rate);
        flex.set_enable_input_buffer(true);
        flex.set_open_drain(OpenDrain::Yes);
        Self { flex }
    }

    /// Get whether the pin level is high.
    #[inline]
    pub fn is_high(&self) -> bool {
        self.flex.is_high()
    }

    /// Get whether the pin level is low.
    #[inline]
    pub fn is_low(&self) -> bool {
        self.flex.is_low()
    }

    /// Set the output as high (open-drain high is just letting go of the line).
    #[inline]
    pub fn set_high(&mut self) {
        self.flex.set_high();
    }

    /// Set the output as low (open-drain low is driving the line low).
    #[inline]
    pub fn set_low(&mut self) {
        self.flex.set_low();
    }

    /// Set the output level.
    #[inline]
    pub fn set_level(&mut self, level: Level) {
        self.flex.set_level(level);
    }

    /// Get the pin level.
    pub fn get_level(&self) -> Level {
        self.flex.get_level()
    }

    /// Toggle the output level.
    #[inline]
    pub fn toggle(&mut self) {
        if self.flex.is_set_low() {
            self.set_high();
        } else {
            self.set_low();
        }
    }

    /// Configure the input logic inversion of this pin.
    #[inline]
    pub fn set_inversion(&mut self, invert: Inverter) {
        self.flex.set_input_inversion(invert);
    }

    /// Expose the inner `Flex` if callers need to reconfigure the pin.
    #[inline]
    pub fn into_flex(self) -> Flex<'d, P> {
        self.flex
    }

    /// Convert this output pin into a push-pull output pin.
    #[inline]
    pub fn into_push_pull(mut self) -> Output<'d, P> {
        self.flex.set_open_drain(OpenDrain::No);
        Output { flex: self.flex }
    }
}

/// GPIO input driver that owns a `Flex` pin.
pub struct Input<'d, P: GpioPin> {
    flex: Flex<'d, P>,
}

impl<'d, P: GpioPin> Input<'d, P> {
    /// Create a GPIO input driver for a [`GpioPin`].
    pub fn new(pin: Peri<'d, P>, pull: Pull) -> Self {
        let mut flex = Flex::new(pin);
        flex.set_as_input();
        flex.set_pull(pull);
        Self { flex }
    }

    /// Get whether the pin input level is high.
    #[inline]
    pub fn is_high(&self) -> bool {
        self.flex.is_high()
    }

    /// Get whether the pin input level is low.
    #[inline]
    pub fn is_low(&self) -> bool {
        self.flex.is_low()
    }

    /// Get the pin level.
    pub fn get_level(&self) -> Level {
        self.flex.get_level()
    }

    /// Configure the input logic inversion of this pin.
    #[inline]
    pub fn set_inversion(&mut self, invert: Inverter) {
        self.flex.set_input_inversion(invert);
    }

    /// Expose the inner `Flex` if callers need to reconfigure the pin.
    ///
    /// Since drive strength and slew rate are not set when creating the
    /// `Input` pin, they need to be set when converting.
    #[inline]
    pub fn into_flex(mut self, strength: DriveStrength, slew_rate: SlewRate) -> Flex<'d, P> {
        self.flex.set_drive_strength(strength);
        self.flex.set_slew_rate(slew_rate);
        self.flex
    }
}

// Both embedded_hal 0.2 and 1.0 must be supported by embassy HALs.
impl<P: GpioPin> embedded_hal_02::digital::v2::InputPin for Flex<'_, P> {
    // GPIO operations on this block cannot fail, therefore we set the error type
    // to Infallible to guarantee that we can only produce Ok variants.
    type Error = Infallible;

    #[inline]
    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.is_high())
    }

    #[inline]
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(self.is_low())
    }
}

impl<P: GpioPin> embedded_hal_02::digital::v2::InputPin for Input<'_, P> {
    type Error = Infallible;

    #[inline]
    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.is_high())
    }

    #[inline]
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(self.is_low())
    }
}

impl<P: GpioPin> embedded_hal_02::digital::v2::OutputPin for Output<'_, P> {
    type Error = Infallible;

    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_high();
        Ok(())
    }

    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_low();
        Ok(())
    }
}

impl<P: GpioPin> embedded_hal_02::digital::v2::OutputPin for Flex<'_, P> {
    type Error = Infallible;

    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_high();
        Ok(())
    }

    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_low();
        Ok(())
    }
}

impl<P: GpioPin> embedded_hal_02::digital::v2::StatefulOutputPin for Flex<'_, P> {
    #[inline]
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.is_set_high())
    }

    #[inline]
    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(self.is_set_low())
    }
}

impl<P: GpioPin> embedded_hal_02::digital::v2::ToggleableOutputPin for Flex<'_, P> {
    type Error = Infallible;

    #[inline]
    fn toggle(&mut self) -> Result<(), Self::Error> {
        self.toggle();
        Ok(())
    }
}

impl<P: GpioPin> embedded_hal_1::digital::ErrorType for Flex<'_, P> {
    type Error = Infallible;
}

impl<P: GpioPin> embedded_hal_1::digital::ErrorType for Input<'_, P> {
    type Error = Infallible;
}

impl<P: GpioPin> embedded_hal_1::digital::ErrorType for Output<'_, P> {
    type Error = Infallible;
}

impl<P: GpioPin> embedded_hal_1::digital::InputPin for Input<'_, P> {
    #[inline]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_high())
    }

    #[inline]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_low())
    }
}

impl<P: GpioPin> embedded_hal_1::digital::OutputPin for Output<'_, P> {
    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_high();
        Ok(())
    }

    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_low();
        Ok(())
    }
}

impl<P: GpioPin> embedded_hal_1::digital::OutputPin for Flex<'_, P> {
    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_high();
        Ok(())
    }

    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_low();
        Ok(())
    }
}

impl<P: GpioPin> embedded_hal_1::digital::StatefulOutputPin for Flex<'_, P> {
    #[inline]
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_high())
    }

    #[inline]
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_low())
    }
}
