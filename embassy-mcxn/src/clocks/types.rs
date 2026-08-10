//! Clock state tracking.

/// Errors that can occur while configuring or querying clocks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClockError {
    /// [`super::init()`] has already been called.
    AlreadyInitialized,
    /// A clock query was made before [`super::init()`] was called.
    NeverInitialized,
    /// `main_clk` is on a source this crate cannot describe.
    ///
    /// [`super::init()`] switches between FIRC's two ranges and parks on SIRC
    /// to do it, so it tolerates finding either of those. Anything else (SOSC,
    /// PLL0, PLL1, ROSC) means a bootloader moved the main clock somewhere
    /// unexpected, and switching out from under that would be worse than
    /// refusing.
    UnsupportedMainClockSource,

    /// An SPC operation failed while moving the core rail.
    Power(crate::power::PowerError),

    /// The core rail sits on a reserved encoding, so no frequency limit can be
    /// derived from it.
    UnknownCoreVoltage,

    /// No core voltage on this part supports the requested `main_clk`, or no
    /// flash wait-state encoding exists for it (RM Tables 320 and 37).
    MainClockUnsupportedFrequency(u32),

    /// `SCG0.CSR[SCS]` never followed the source written to `RCCR[SCS]`.
    ///
    /// The source is selected but is not producing a clock.
    MainClockNotStable,

    /// FIRC was configured but never reported `FIRCCSR.FIRCVLD`.
    FircNotValid,

    /// PLL0 (APLL) never reported `APLLCSR.APLL_LOCK`.
    Pll0NotLocked,

    /// PLL0 locked, but its dividers read back a rate other than the one this
    /// crate configures - most likely a `*REQ` latch write that did not take.
    /// Carries the rate actually computed from the registers, in Hz.
    Pll0UnexpectedRate(u32),

    /// `SCG0.FIRCCSR.LK` is set and could not be cleared, so FIRC cannot be
    /// reconfigured. Nothing in this crate sets it; a bootloader must have.
    FircLocked,

    /// The requested SOSC frequency cannot be encoded in `SOSCCFG.RANGE`,
    /// which only spans 16-66 MHz.
    SoscUnsupportedFrequency(u32),

    /// `SCG0.SOSCCSR.LK` is set, so SOSC cannot be configured.
    ///
    /// Nothing in this crate sets it; a bootloader running before `init()`
    /// must have.
    SoscLocked,

    /// SOSC was enabled but never reported `SOSCVLD`.
    ///
    /// Usually a crystal that is absent, damaged, or mismatched against the
    /// configured frequency - or [`super::config::SoscMode::Crystal`] selected
    /// on a board that actually drives `EXTAL` with an external clock.
    SoscNotValid,
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

    /// The core rail voltage `init()` settled on.
    ///
    /// Derived from [`super::config::MainClock`], never chosen by the caller:
    /// the lowest level that supports the requested frequency. Read-only here,
    /// but [`crate::power`] can report and change it.
    pub core_voltage: crate::power::CoreVoltage,

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

    /// `fro_hf`, the FIRC output at whatever rate `SCG0.FIRCCFG.RANGE`
    /// selects - 48 or 144 MHz.
    ///
    /// **This is what a peripheral mux's `fro_hf` input receives**, and it is
    /// *not* either of the two gated taps below. A `*CLKSEL` selecting `fro_hf`
    /// follows `FIRCCFG.RANGE`, so raising the FIRC range raises this node and
    /// every peripheral selecting it. NXP's own `CLOCK_GetAdcClkFreq` resolves
    /// mux value 2 through `CLOCK_GetFroHfFreq`, which reads `FIRCCFG.RANGE`;
    /// the taps have their own separate accessors.
    ///
    /// Not optional: FIRC is running whenever the CPU is, either as `main_clk`
    /// itself or as the PLLs' reference.
    pub fro_hf: u32,

    /// The 48 MHz FIRC tap, gated by `SCG0.FIRCCSR.FIRC_SCLK_PERIPH_EN`.
    ///
    /// A dedicated fan-out held at 48 MHz independently of `FIRCCFG.RANGE`,
    /// and the reference the PLLs take. It is a distinct node from
    /// [`Self::fro_hf`] - only consumers documented as taking *this* tap should
    /// use it, not peripherals whose mux offers a generic "fro_hf".
    pub fro_hf_48m: Option<u32>,

    /// The 144 MHz FIRC tap, gated by `SCG0.FIRCCSR.FIRC_FCLK_PERIPH_EN`.
    ///
    /// Like [`Self::fro_hf_48m`], a fixed-rate fan-out independent of
    /// `FIRCCFG.RANGE` - not what a peripheral mux's `fro_hf` input sees.
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
    /// `None` unless [`super::config::ClocksConfig::sosc`] was set. Note this
    /// records the source itself; reaching a peripheral mux additionally needs
    /// the matching `CLOCK_CTRL` fan-out gate
    /// ([`super::config::ClocksConfig::enable_clk_in_periph`] or
    /// [`super::config::ClocksConfig::enable_clk_in_fm_usbh_lpt`]).
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
    /// `Some` only when [`super::config::MainClock::Pll0At150m`] brought it up;
    /// PLL0 is not configured for any other purpose yet.
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
    /// A single, globally shared resource, so it would be chosen once by
    /// `init()` rather than by a driver.
    ///
    /// Always `None`: `PLLCLKDIVSEL`/`PLLCLKDIV` are not programmed yet.
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

    /// `fro_hf`, the rate a peripheral mux's `fro_hf` input actually receives.
    ///
    /// Infallible - [`Clocks::fro_hf`] is not optional - but shaped like the
    /// other accessors so the `impl_functional_clock!` source tables can call
    /// it uniformly.
    pub fn ensure_fro_hf(&self) -> Result<u32, ClockError> {
        Ok(self.fro_hf)
    }

    /// The 48 MHz FIRC tap.
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
        Self::ensure(self.clk_in, "clk_in", "set ClocksConfig::sosc")
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
        Self::ensure(
            self.pll0_clk,
            "pll0_clk",
            "set ClocksConfig::main_clk to MainClock::Pll0At150m",
        )
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
        Self::ensure(self.pll_div, "pll_div", "pll_div is not implemented")
    }
}
