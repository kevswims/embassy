//! # Clock module
//!
//! **Status: minimal bring-up.** [`init()`] does not change the main clock
//! source away from its reset default - RM "Chapter 33 Clocking" documents
//! `fro_hf` (SCG's FIRC, `SCG0.RCCR.SCS == Firc`) as the default main clock,
//! at 48 MHz (`SCG0.FIRCCFG.RANGE == Firc48mhz`), with `AHBCLKDIV` reset to
//! divide-by-1. [`init()`] verifies those reset-default conditions still
//! hold and records the resulting frequencies; it returns
//! [`ClockError::UnsupportedMainClockSource`] if the main clock has already
//! been switched to anything else (SOSC, PLL0, PLL1, ROSC, ...) - selecting
//! those, and configuring `AHBCLKDIV` to something other than 1, isn't
//! implemented yet.
//!
//! Peripheral clock gating (SYSCON `AHBCLKCTRLx`/`PRESETCTRLx`) is handled by
//! the [`gate`] module and does not depend on the above.

use core::cell::RefCell;

use config::ClocksConfig;

use crate::pac;

pub mod config;
mod gate;
mod types;

pub use gate::{Gate, assert_reset, disable, enable, enable_and_reset, is_reset_released, release_reset};
pub use types::{Clocks, ClockError};

/// The state of system core clocks.
///
/// Initialized by [`init()`], and then unchanged for the remainder of the program.
pub(super) static CLOCKS: critical_section::Mutex<RefCell<Option<Clocks>>> =
    critical_section::Mutex::new(RefCell::new(None));

/// Initialize the core system clocks with the given [`ClocksConfig`].
///
/// This function should be called EXACTLY once at start-up, usually via a
/// call to [`embassy_mcxn::init()`](crate::init()). Subsequent calls will
/// return an error.
pub fn init(_settings: ClocksConfig) -> Result<(), ClockError> {
    critical_section::with(|cs| {
        if CLOCKS.borrow_ref(cs).is_some() {
            Err(ClockError::AlreadyInitialized)
        } else {
            Ok(())
        }
    })?;

    use pac::scg::{FirccfgRange, RccrScs};

    if pac::SCG0.rccr().read().scs() != RccrScs::Firc {
        return Err(ClockError::UnsupportedMainClockSource);
    }

    let main_clk_hz = match pac::SCG0.firccfg().read().range() {
        FirccfgRange::Firc48mhz => 48_000_000,
        FirccfgRange::Firc144mhz => 144_000_000,
    };

    // AHBCLKDIV.DIV divides by DIV + 1; reset value is 0 (divide-by-1).
    let ahb_div = pac::SYSCON0.ahbclkdiv().read().div() as u32 + 1;
    let system_clk_hz = main_clk_hz / ahb_div;

    let clocks = Clocks {
        main_clk_hz,
        system_clk_hz,
    };

    critical_section::with(|cs| {
        let mut clks = CLOCKS.borrow_ref_mut(cs);
        assert!(clks.is_none(), "Clock setup race!");
        *clks = Some(clocks);
    });

    Ok(())
}

/// Obtain the full clocks structure, calling the given closure in a critical section.
///
/// The given closure will be called with read-only access to the state of the system
/// clocks. This can be used to query and return the state of a given clock.
///
/// As the caller's closure will be called in a critical section, care must be taken
/// not to block or cause any other undue delays while accessing.
///
/// Calls to this function will not succeed until after a successful call to `init()`,
/// and will always return None.
pub fn with_clocks<R: 'static, F: FnOnce(&Clocks) -> R>(f: F) -> Option<R> {
    critical_section::with(|cs| {
        let c = CLOCKS.borrow_ref(cs);
        let c = c.as_ref()?;
        Some(f(c))
    })
}
