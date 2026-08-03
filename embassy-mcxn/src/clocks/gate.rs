//! Clock gate trait and free functions for enabling/disabling peripheral
//! clocks through SYSCON.
//!
//! Unlike embassy-mcxa (which gates peripheral clocks through MRCC), MCXN947
//! gates AHB peripheral clocks through `SYSCON0.AHBCLKCTRLx` and resets them
//! through `SYSCON0.PRESETCTRLx` (RM "Chapter 17 System Controller (SYSCON)").
//! Clock-enable and reset bits share the same bit position within their
//! respective register index for a given peripheral.

/// Trait describing an AHB clock gate that can be toggled through SYSCON.
pub trait Gate {
    /// Enable the clock gate.
    ///
    /// # Safety
    ///
    /// The current peripheral must be disabled prior to calling this method.
    unsafe fn enable_clock();

    /// Disable the clock gate.
    ///
    /// # Safety
    ///
    /// There must be no active user of this peripheral when calling this method.
    unsafe fn disable_clock();

    /// Drive the peripheral into reset.
    ///
    /// # Safety
    ///
    /// There must be no active user of this peripheral when calling this method.
    unsafe fn assert_reset();

    /// Drive the peripheral out of reset.
    ///
    /// # Safety
    ///
    /// There must be no active user of this peripheral when calling this method.
    unsafe fn release_reset();

    /// Return whether the clock gate for this peripheral is currently enabled.
    fn is_clock_enabled() -> bool;

    /// Return whether the peripheral is currently held in reset.
    fn is_reset_released() -> bool;
}

/// Enable the clock gate for the given peripheral, then pulse its reset line.
///
/// # Safety
///
/// This peripheral must not yet be in use prior to calling `enable_and_reset`.
#[inline]
pub unsafe fn enable_and_reset<G: Gate>() {
    unsafe {
        enable::<G>();
        pulse_reset::<G>();
    }
}

/// Enable the clock gate for the given peripheral.
///
/// Prefer [`enable_and_reset`] unless you are specifically avoiding a pulse
/// of the reset, or need to control the duration of the pulse more directly.
///
/// # Safety
///
/// This peripheral must not yet be in use prior to calling `enable`.
#[inline]
pub unsafe fn enable<G: Gate>() {
    unsafe {
        G::enable_clock();
        while !G::is_clock_enabled() {}
        core::arch::asm!("dsb sy; isb sy", options(nomem, nostack, preserves_flags));
    }
}

/// Disable the clock gate for the given peripheral.
///
/// # Safety
///
/// This peripheral must no longer be in use prior to calling `disable`.
#[allow(dead_code)]
#[inline]
pub unsafe fn disable<G: Gate>() {
    unsafe {
        G::disable_clock();
    }
}

/// Check whether a gate is currently enabled.
#[allow(dead_code)]
#[inline]
pub fn is_clock_enabled<G: Gate>() -> bool {
    G::is_clock_enabled()
}

/// Release a reset line for the given peripheral.
///
/// Prefer [`enable_and_reset`].
///
/// # Safety
///
/// This peripheral must not yet be in use prior to calling `release_reset`.
#[inline]
pub unsafe fn release_reset<G: Gate>() {
    unsafe {
        G::release_reset();
    }
}

/// Assert a reset line for the given peripheral.
///
/// Prefer [`enable_and_reset`].
///
/// # Safety
///
/// This peripheral must not yet be in use prior to calling `assert_reset`.
#[inline]
pub unsafe fn assert_reset<G: Gate>() {
    unsafe {
        G::assert_reset();
    }
}

/// Check whether the peripheral is held in reset.
#[allow(dead_code)]
#[inline]
pub fn is_reset_released<G: Gate>() -> bool {
    G::is_reset_released()
}

/// Pulse a reset line (assert then release).
///
/// Prefer [`enable_and_reset`].
///
/// # Safety
///
/// This peripheral must not yet be in use prior to calling `pulse_reset`.
#[inline]
pub unsafe fn pulse_reset<G: Gate>() {
    unsafe {
        G::assert_reset();
        cortex_m::asm::nop();
        cortex_m::asm::nop();
        G::release_reset();
        while !G::is_reset_released() {}
    }
}

/// Implements [`Gate`] for a peripheral gated through a `SYSCON0.AHBCLKCTRLn`
/// clock-enable bit and a `SYSCON0.PRESETCTRLn` reset bit, at the same
/// array-indexed field in both (e.g. `port(0)`/`port_rst(0)` at bit 13 of
/// index 0).
///
/// `$rst_ty` is the reset value enum SYSCON's register definitions use for
/// this field family (e.g. `PortRst`, `GpioRst`) - shared across every index
/// in the family since they all have the same `Released = 0`/`Asserted = 1`
/// shape.
macro_rules! impl_syscon_gate {
    ($name:ident, $clk_reg:ident, $clk_field:ident, $rst_reg:ident, $rst_field:ident, $rst_ty:ident, $idx:expr) => {
        impl crate::clocks::Gate for crate::peripherals::$name {
            paste::paste! {
                #[inline]
                unsafe fn enable_clock() {
                    crate::pac::SYSCON0.$clk_reg().modify(|w| w.[<set_ $clk_field>]($idx, true));
                }

                #[inline]
                unsafe fn disable_clock() {
                    crate::pac::SYSCON0.$clk_reg().modify(|w| w.[<set_ $clk_field>]($idx, false));
                }

                #[inline]
                unsafe fn release_reset() {
                    use crate::pac::syscon::$rst_ty;
                    crate::pac::SYSCON0
                        .$rst_reg()
                        .modify(|w| w.[<set_ $rst_field>]($idx, $rst_ty::Released));
                }

                #[inline]
                unsafe fn assert_reset() {
                    use crate::pac::syscon::$rst_ty;
                    crate::pac::SYSCON0
                        .$rst_reg()
                        .modify(|w| w.[<set_ $rst_field>]($idx, $rst_ty::Asserted));
                }
            }

            #[inline]
            fn is_clock_enabled() -> bool {
                crate::pac::SYSCON0.$clk_reg().read().$clk_field($idx)
            }

            #[inline]
            fn is_reset_released() -> bool {
                use crate::pac::syscon::$rst_ty;
                crate::pac::SYSCON0.$rst_reg().read().$rst_field($idx) == $rst_ty::Released
            }
        }
    };
}

// PORT0-4 / GPIO0-4: gated via SYSCON0.AHBCLKCTRL0 / PRESETCTRL0 (RM Chapter
// 17). GPIO5/PORT5 are deliberately not included here - RM "Chapter 33
// Clocking" (Table 319, `slow_clk`) documents them as always clocked (they
// live in the always-on VBAT domain, for wake-up support), with no AHB clock
// gate or reset bit exposed for them at all.
impl_syscon_gate!(PORT0, ahbclkctrl0, port, presetctrl0, port_rst, PortRst, 0);
impl_syscon_gate!(PORT1, ahbclkctrl0, port, presetctrl0, port_rst, PortRst, 1);
impl_syscon_gate!(PORT2, ahbclkctrl0, port, presetctrl0, port_rst, PortRst, 2);
impl_syscon_gate!(PORT3, ahbclkctrl0, port, presetctrl0, port_rst, PortRst, 3);
impl_syscon_gate!(PORT4, ahbclkctrl0, port, presetctrl0, port_rst, PortRst, 4);
impl_syscon_gate!(GPIO0, ahbclkctrl0, gpio, presetctrl0, gpio_rst, GpioRst, 0);
impl_syscon_gate!(GPIO1, ahbclkctrl0, gpio, presetctrl0, gpio_rst, GpioRst, 1);
impl_syscon_gate!(GPIO2, ahbclkctrl0, gpio, presetctrl0, gpio_rst, GpioRst, 2);
impl_syscon_gate!(GPIO3, ahbclkctrl0, gpio, presetctrl0, gpio_rst, GpioRst, 3);
impl_syscon_gate!(GPIO4, ahbclkctrl0, gpio, presetctrl0, gpio_rst, GpioRst, 4);
