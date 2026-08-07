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
    /// A peripheral asked for a clock that [`super::init()`] did not bring up.
    ///
    /// `clock` names the tree node and `reason` says what to do about it -
    /// usually which [`super::config::ClocksConfig`] field to set.
    BadConfig {
        /// The tree node that is unavailable.
        clock: &'static str,
        /// Why it is unavailable, and how to make it available.
        reason: &'static str,
    },
}

/// State of the core system clocks, as configured by [`super::init()`].
///
/// A node is `Some(hz)` only if `init()` brought that clock up (or verified it
/// was already running) *and* enabled whatever fan-out gate it needs to reach a
/// peripheral mux. `None` means a peripheral selecting it will be rejected with
/// [`ClockError::BadConfig`] rather than silently receiving no clock.
///
/// Every field is written exactly once, by `init()`, and is immutable
/// afterwards. That is what makes a shared `&Clocks` a sound read-only view for
/// driver constructors, including from the second core.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Clocks {
    /// Frequency of `main_clk`, in Hz.
    ///
    /// See RM "Chapter 33 Clocking" - the main clock feeds the CPU and AHB
    /// bus (via `AHBCLKDIV`), and most peripherals derive their clock from
    /// it (directly, or via a peripheral-specific mux/divider).
    ///
    /// Not optional: if the CPU is executing, `main_clk` is running.
    pub main_clk_hz: u32,

    /// Frequency of `system_clk` (`main_clk` divided by `AHBCLKDIV`), in Hz.
    ///
    /// This is the clock the bus-clocked peripherals run at - PORT, GPIO, DMA,
    /// CRC and the like, which have no functional-clock mux of their own.
    ///
    /// Not optional, for the same reason as [`Self::main_clk_hz`].
    pub system_clk_hz: u32,

    /// `fro_12m`, SCG's SIRC. Fixed 12 MHz.
    ///
    /// `Some` iff `SCG0.SIRCCSR.SIRC_CLK_PERIPH_EN` is set, which it is at
    /// reset. SIRC has no enable bit at all on this chip - it is always
    /// running.
    pub fro_12m: Option<u32>,

    /// `fro_hf` at 48 MHz, from `SCG0.FIRCCSR.FIRC_SCLK_PERIPH_EN`.
    ///
    /// This is the fan-out that also feeds the PLLs' reference input.
    pub fro_hf_48m: Option<u32>,

    /// `fro_hf` at 144 MHz, from `SCG0.FIRCCSR.FIRC_FCLK_PERIPH_EN`.
    ///
    /// Available as a peripheral source regardless of what `main_clk` runs at:
    /// the two FIRC fan-outs are independent of `FIRCCFG.RANGE`.
    pub fro_hf_144m: Option<u32>,

    /// `fro_hf_div`, `fro_hf` through `FROHFDIV`.
    ///
    /// `None` unless [`super::config::ClocksConfig::fro_hf_div`] was set: the
    /// divider is *halted* at reset and a halted divider produces nothing.
    pub fro_hf_div: Option<u32>,

    /// `clk_1m`, 1 MHz derived from FRO12M inside the clock generator.
    ///
    /// `None` unless [`super::config::ClocksConfig::enable_clk_1m`] was set,
    /// because both `CLOCK_CTRL.FRO1MHZ_ENA` and `FRO1MHZ_CLK_ENA` are clear
    /// at reset.
    pub clk_1m: Option<u32>,

    /// `clk_in`, the SOSC output (external crystal or clock input).
    ///
    /// Always `None`: SOSC bring-up is not implemented.
    pub clk_in: Option<u32>,

    /// `clk_16k`, FRO16K from the VBAT domain.
    ///
    /// `None` unless [`super::config::ClocksConfig::enable_clk_16k`] was set.
    pub clk_16k: Option<u32>,

    /// `xtal32k`, OSC32K from the VBAT domain.
    ///
    /// Always `None`: not implemented.
    pub xtal32k: Option<u32>,

    /// `pll0_clk`, the APLL output.
    ///
    /// Always `None`: PLL bring-up is not implemented.
    pub pll0_clk: Option<u32>,

    /// `pll1_clk0`, the first SPLL output divider.
    ///
    /// Always `None`: PLL bring-up is not implemented.
    pub pll1_clk0: Option<u32>,

    /// `pll1_clk1`, the second SPLL output divider.
    ///
    /// Always `None`: PLL bring-up is not implemented.
    pub pll1_clk1: Option<u32>,

    /// The "PLL divided clock" node: `PLLCLKDIVSEL` picks PLL0 or `pll1_clk0`,
    /// `PLLCLKDIV` divides it.
    ///
    /// This is a single, globally shared resource - it is the source that every
    /// peripheral mux sees as its "PLL divided clock" input, so selecting a PLL
    /// for one peripheral constrains all of them. It is therefore chosen once
    /// by `init()` and never written by a driver.
    ///
    /// Always `None`: PLL bring-up is not implemented.
    pub pll_div: Option<u32>,
}

impl Clocks {
    /// Resolve an optional node, or explain why it is unavailable.
    fn ensure(node: Option<u32>, clock: &'static str, reason: &'static str) -> Result<u32, ClockError> {
        node.ok_or(ClockError::BadConfig { clock, reason })
    }

    /// `fro_12m` (SCG SIRC), 12 MHz.
    pub fn ensure_fro_12m(&self) -> Result<u32, ClockError> {
        Self::ensure(self.fro_12m, "fro_12m", "SCG0.SIRCCSR.SIRC_CLK_PERIPH_EN is clear")
    }

    /// `fro_hf` at 48 MHz.
    pub fn ensure_fro_hf_48m(&self) -> Result<u32, ClockError> {
        Self::ensure(
            self.fro_hf_48m,
            "fro_hf_48m",
            "SCG0.FIRCCSR.FIRC_SCLK_PERIPH_EN is clear",
        )
    }

    /// `fro_hf` at 144 MHz.
    pub fn ensure_fro_hf_144m(&self) -> Result<u32, ClockError> {
        Self::ensure(
            self.fro_hf_144m,
            "fro_hf_144m",
            "SCG0.FIRCCSR.FIRC_FCLK_PERIPH_EN is clear",
        )
    }

    /// `fro_hf_div`.
    pub fn ensure_fro_hf_div(&self) -> Result<u32, ClockError> {
        Self::ensure(
            self.fro_hf_div,
            "fro_hf_div",
            "FROHFDIV is halted; set ClocksConfig::fro_hf_div",
        )
    }

    /// `clk_1m`, 1 MHz.
    pub fn ensure_clk_1m(&self) -> Result<u32, ClockError> {
        Self::ensure(self.clk_1m, "clk_1m", "set ClocksConfig::enable_clk_1m")
    }

    /// `clk_in`, the SOSC output.
    pub fn ensure_clk_in(&self) -> Result<u32, ClockError> {
        Self::ensure(self.clk_in, "clk_in", "SOSC bring-up is not implemented")
    }

    /// `clk_16k`, FRO16K from the VBAT domain.
    pub fn ensure_clk_16k(&self) -> Result<u32, ClockError> {
        Self::ensure(self.clk_16k, "clk_16k", "set ClocksConfig::enable_clk_16k")
    }

    /// `xtal32k`, OSC32K from the VBAT domain.
    pub fn ensure_xtal32k(&self) -> Result<u32, ClockError> {
        Self::ensure(self.xtal32k, "xtal32k", "VBAT0 OSC32K is not implemented")
    }

    /// `pll0_clk`.
    pub fn ensure_pll0_clk(&self) -> Result<u32, ClockError> {
        Self::ensure(self.pll0_clk, "pll0_clk", "PLL bring-up is not implemented")
    }

    /// `pll1_clk0`.
    pub fn ensure_pll1_clk0(&self) -> Result<u32, ClockError> {
        Self::ensure(self.pll1_clk0, "pll1_clk0", "PLL bring-up is not implemented")
    }

    /// `pll1_clk1`.
    pub fn ensure_pll1_clk1(&self) -> Result<u32, ClockError> {
        Self::ensure(self.pll1_clk1, "pll1_clk1", "PLL bring-up is not implemented")
    }

    /// The shared "PLL divided clock" node.
    pub fn ensure_pll_div(&self) -> Result<u32, ClockError> {
        Self::ensure(self.pll_div, "pll_div", "PLL bring-up is not implemented")
    }
}
