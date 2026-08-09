//! Busy-wait delays measured in wall-clock time rather than core cycles.
//!
//! `cortex_m::asm::delay` counts core cycles, so a literal cycle count encodes
//! one specific core frequency. Raising `main_clk` then silently shortens every
//! such delay - and the hardware these delays exist for (analog settling times,
//! regulator ramps) does not get faster to match. The failures that produces are
//! quiet ones: an unsettled VREF biases the ADC wrong rather than reporting an
//! error.
//!
//! [`delay_us`] resolves the cycle count from the core clock recorded by
//! [`crate::clocks::init()`] instead, so a settling time stated in microseconds
//! stays that many microseconds at any supported frequency.
//!
//! Callers state the time they actually want to wait. Where that is much longer
//! than the hardware requires, the margin belongs in the caller's constant and
//! its comment - not hidden in here.

/// Core frequency assumed when the clock tree has not been recorded yet.
///
/// Deliberately the *highest* `system_clk` the part supports (RM Table 320:
/// 150 MHz in overdrive) rather than the lowest, so the unknown case overshoots
/// instead of under-delaying.
const FALLBACK_HZ: u32 = 150_000_000;

/// Block for at least `us` microseconds.
///
/// Reads `system_clk` - the post-`AHBCLKDIV` clock the core actually executes
/// at, which is what `asm::delay`'s cycle count is denominated in - not
/// `main_clk`.
///
/// Every caller is reachable only after [`crate::init()`], so the clock tree is
/// always recorded in practice; see [`FALLBACK_HZ`] for the fallback.
pub(crate) fn delay_us(us: u32) {
    let hz = crate::clocks::with_clocks(|c| c.system_clk_hz).unwrap_or(FALLBACK_HZ);
    cortex_m::asm::delay(us.saturating_mul(hz / 1_000_000));
}
