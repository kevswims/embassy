//! # Clock module
//!
//! [`init()`] brings up the clock sources and records the resulting tree in
//! [`Clocks`], which peripheral drivers then read to resolve their own
//! functional clock. Four things are worth knowing about how it is put
//! together.
//!
//! **`main_clk` is selected by [`config::MainClock`].** It defaults to FIRC at
//! 48 MHz, the reset value, which costs no register writes at all. Selecting
//! [`config::MainClock::FroHf144m`] instead moves it to 144 MHz, and [`init()`]
//! owns the whole sequence that requires: park the core on SIRC, raise the
//! `SPC0` core rail, retime the SRAM, raise the `FMU0` flash wait states, then
//! the FIRC range, then attach. The core voltage is derived from the requested
//! frequency rather than chosen by the caller, so the two cannot be mismatched;
//! the result is readable as [`Clocks::core_voltage`]. See `main_clk.rs` for
//! why each step sits where it does, and `DRIVERS.md`'s hazards section for
//! what happens when they are reordered.
//!
//! [`config::MainClock::Pll0At150m`] goes one step further, to the fastest rate
//! the part supports, and takes the same prologue - but sources `main_clk` from
//! PLL0 with FIRC left at 48 MHz as its reference.
//!
//! That difference matters beyond the 6 MHz: raising the FIRC range also raises
//! [`Clocks::fro_hf`], which is what every peripheral mux's `fro_hf` input
//! receives, so dividers sized for 48 MHz need revisiting at 144 MHz. The PLL0
//! path leaves `fro_hf` alone, so it costs no divider rework.
//!
//! `AHBCLKDIV` is division only, so `system_clk` can sit below `main_clk` but
//! never above it.
//!
//! **Sources must be enabled before a peripheral can select them.** Most of the
//! fan-out gates in `SYSCON0.CLOCK_CTRL` are clear at reset, and `FROHFDIV`
//! resets halted, so a `*CLKSEL` pointing at `clk_1m` or `fro_hf_div` receives
//! nothing until [`init()`] has enabled it. [`Clocks`] records only what was
//! actually brought up, and a driver selecting an unavailable source is
//! rejected with [`ClockError::BadConfig`] naming the config field to set -
//! rather than silently running at no clock. Drivers never enable a source
//! themselves: several are shared across peripherals, and one
//! (`xtal32k`) can take hundreds of milliseconds to settle.
//!
//! **`CLKUNLOCK` is never written.** It resets to `Unlock::Enable`, meaning
//! clock configuration registers are already writable. Freezing it would break
//! every later driver, because `*CLKSEL`/`*CLKDIV` are programmed at
//! driver-construction time, not here.
//!
//! **[`Clocks`] is written once, by [`init()`].** That is what makes handing
//! out a shared `&Clocks` sound, and it is also how the genuinely shared nodes
//! stay coherent - [`Clocks::pll_div`] in particular is one select and one
//! divider, so the rate chosen for it constrains every consumer at once.
//!
//! Note which consumers those are: `pll_div` feeds the LP_FLEXCOMM mux
//! (`FCCLKSEL`), and nothing else on this chip. Peripherals whose own mux offers
//! a PLL - ADC, FlexCAN, CTIMER, ENET - take PLL0 or `pll1_clk0` *directly*, so
//! they are unaffected by `PLLCLKDIV`.
//!
//! Peripheral clock gating (SYSCON `AHBCLKCTRLn`/`PRESETCTRLn`) lives in the
//! `gate` submodule, and the per-peripheral mux/divider layer in `functional`.

use core::cell::RefCell;

use config::ClocksConfig;

pub mod config;
mod functional;
mod gate;
mod main_clk;
mod pll;
mod source;
mod types;

pub use functional::{FunctionalClock, enable_and_reset_with_clock};
#[cfg(mcx_ctimer)]
pub use functional::{Ctimer0ClockConfig, Ctimer0ClockSource, Ctimer1ClockConfig, Ctimer1ClockSource};
#[cfg(mcx_wwdt)]
pub use functional::WwdtClockConfig;
#[cfg(mcxn_ostimer)]
pub use functional::{OstimerClockConfig, OstimerClockSource};
#[cfg(mcxn_lpuart)]
pub use functional::{
    Lpuart0ClockConfig, Lpuart0ClockSource, Lpuart1ClockConfig, Lpuart1ClockSource, Lpuart2ClockConfig,
    Lpuart2ClockSource, Lpuart3ClockConfig, Lpuart3ClockSource, Lpuart4ClockConfig, Lpuart4ClockSource,
    Lpuart5ClockConfig, Lpuart5ClockSource, Lpuart6ClockConfig, Lpuart6ClockSource, Lpuart7ClockConfig,
    Lpuart7ClockSource, Lpuart8ClockConfig, Lpuart8ClockSource, Lpuart9ClockConfig, Lpuart9ClockSource,
};
#[cfg(mcxn_adc)]
pub use functional::{Adc0ClockConfig, Adc0ClockSource, Adc1ClockConfig, Adc1ClockSource};
#[cfg(mcxn_can)]
pub use functional::{Can0ClockConfig, Can0ClockSource, Can1ClockConfig, Can1ClockSource};
pub use config::{MainClock, PllDivConfig, PllDivSource, SoscConfig, SoscMode};
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
pub fn init(settings: ClocksConfig) -> Result<(), ClockError> {
    critical_section::with(|cs| {
        if CLOCKS.borrow_ref(cs).is_some() {
            Err(ClockError::AlreadyInitialized)
        } else {
            Ok(())
        }
    })?;

    // First: it rewrites `FIRCCSR` (so it must precede `record_firc`) and it
    // settles the rate every later step derives from.
    let outcome = main_clk::switch(settings.main_clk)?;
    let main_clk_hz = outcome.main_clk_hz;

    let mut clocks = Clocks {
        main_clk_hz,
        core_voltage: outcome.core_voltage,
        // Filled in below, once AHBCLKDIV is known.
        system_clk_hz: main_clk_hz,
        fro_12m: None,
        // Overwritten by `record_firc` below, which reads `FIRCCFG.RANGE`.
        fro_hf: main_clk_hz,
        fro_hf_48m: None,
        fro_hf_144m: None,
        fro_hf_div: None,
        clk_1m: None,
        clk_in: None,
        clk_16k: None,
        xtal32k: None,
        pll0_clk: outcome.pll0_hz,
        pll1_clk0: None,
        pll1_clk1: None,
        pll_div: None,
    };

    source::record_sirc(&mut clocks);
    source::record_firc(&mut clocks);
    // Before `configure_clock_ctrl`: that enables the `clk_in` fan-out gates,
    // which want a source already running behind them.
    source::configure_sosc(&settings, &mut clocks)?;
    source::configure_clock_ctrl(&settings, &mut clocks);
    // After `main_clk::switch`, which is what brings PLL0 up at all.
    source::configure_pll_div(&settings, &mut clocks)?;
    // Needs main_clk_hz already recorded: FROHFDIV divides `fro_hf`.
    source::configure_fro_hf_div(&settings, &mut clocks);
    source::configure_vbat(&settings, &mut clocks);

    clocks.system_clk_hz = main_clk_hz / source::configure_ahb_div(&settings);

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
