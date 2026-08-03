//! Clock state tracking.

/// Errors that can occur while configuring or querying clocks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClockError {
    /// [`super::init()`] has already been called.
    AlreadyInitialized,
    /// A clock query was made before [`super::init()`] was called.
    NeverInitialized,
    /// `main_clk` is not currently sourced from `fro_hf` (the reset default).
    ///
    /// Selecting/configuring other main clock sources (SOSC, PLL0, PLL1,
    /// ROSC, ...) isn't implemented yet.
    UnsupportedMainClockSource,
}

/// State of the core system clocks, as observed after [`super::init()`].
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Clocks {
    /// Frequency of `main_clk`, in Hz.
    ///
    /// See RM "Chapter 33 Clocking" - the main clock feeds the CPU and AHB
    /// bus (via `AHBCLKDIV`), and most peripherals derive their clock from
    /// it (directly, or via a peripheral-specific mux/divider).
    pub main_clk_hz: u32,

    /// Frequency of `system_clk` (`main_clk` divided by `AHBCLKDIV`), in Hz.
    pub system_clk_hz: u32,
}
