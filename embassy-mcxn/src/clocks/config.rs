//! Clock configuration surface.
//!
//! Pure data: nothing in this module touches a register. [`super::init()`]
//! consumes a [`ClocksConfig`] and performs the bring-up.

/// A SYSCON clock divider with a `BITS`-wide `div` field.
///
/// The hardware divides by `div + 1`, so the representable divisors are
/// `1..=2^BITS`. `BITS` binds a value to one register width - `Div<6>` for
/// `WDT1CLKDIV`, `Div<8>` for `AHBCLKDIV` and `FROHFDIV` - so a divisor too
/// large for a given register cannot be constructed for it.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Div<const BITS: u32>(u8);

impl<const BITS: u32> Div<BITS> {
    /// Divide by one.
    pub const fn no_div() -> Self {
        Self(0)
    }

    /// Divide by `d`, where `1 <= d <= 2^BITS`.
    pub const fn from_divisor(d: u32) -> Option<Self> {
        if d == 0 || d > (1u32 << BITS) {
            None
        } else {
            Some(Self((d - 1) as u8))
        }
    }

    /// Divide by `d`, panicking if it is out of range for this register width.
    ///
    /// Intended for `const` contexts, where an out-of-range divisor is a
    /// compile error rather than a panic. Use [`Self::from_divisor`] when the
    /// divisor is computed at run time.
    pub const fn new(d: u32) -> Self {
        match Self::from_divisor(d) {
            Some(v) => v,
            None => panic!("clock divisor out of range for this register"),
        }
    }

    /// Raw `div` field value.
    pub(crate) const fn into_bits(self) -> u8 {
        self.0
    }

    /// The divisor this represents.
    pub(crate) const fn into_divisor(self) -> u32 {
        self.0 as u32 + 1
    }
}

/// How the SOSC input is driven.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SoscMode {
    /// A crystal across `EXTAL`/`XTAL`, driven by the on-chip oscillator
    /// (`SOSCCFG.EREFS = Internal`).
    Crystal,

    /// A square-wave clock driven into `EXTAL` from off-chip
    /// (`SOSCCFG.EREFS = External`). The internal oscillator, and its LDO,
    /// stay powered down.
    ExternalClock,
}

/// SOSC (system oscillator) bring-up request, producing `clk_in`.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SoscConfig {
    /// Input frequency, in Hz.
    ///
    /// `SOSCCFG.RANGE` can only encode 16-66 MHz, so anything outside that is
    /// rejected with [`super::ClockError::SoscUnsupportedFrequency`]. This
    /// must match the crystal actually fitted to the board: nothing measures
    /// it, and every consumer's divider is computed from this number.
    pub hz: u32,

    /// Whether the input is a crystal or an externally driven clock.
    pub mode: SoscMode,
}

/// Configuration for the core system clocks, passed to [`super::init()`].
///
/// The defaults enable every clock source that costs a single register write,
/// needs no pin, and has no start-up latency, so that a peripheral selecting
/// one does not have to think about it. Sources that need a crystal or a PLL
/// lock are off by default: SOSC because it is board-specific, the PLLs
/// because they are not implemented at all - see the module docs.
///
/// [`super::init()`] does not change the main clock source. It stays on its
/// reset default of FIRC at 48 MHz, because raising `main_clk` requires
/// raising the core LDO voltage (`SPC0`) and the flash read wait states
/// (`FMU0`) in a specific order first.
#[non_exhaustive]
#[derive(Debug, Clone)]
pub struct ClocksConfig {
    /// Enable `clk_1m` (`CLOCK_CTRL.FRO1MHZ_ENA` + `FRO1MHZ_CLK_ENA`).
    ///
    /// Defaults to `true`: it is one register write, derives from the
    /// always-running FRO12M, and is the clock the OSTIMER time driver and
    /// WWDT select.
    pub enable_clk_1m: bool,

    /// Enable `fro_12m` to the flash controller, LPTMR0/1 and FREQME
    /// (`CLOCK_CTRL.FRO12MHZ_ENA`).
    ///
    /// Defaults to `true`. Note this is a consumer-side gate: `fro_12m` as a
    /// *mux input* comes from `SCG0.SIRCCSR.SIRC_CLK_PERIPH_EN`, which is set
    /// at reset.
    pub enable_fro_12m_periph: bool,

    /// Enable `fro_hf` to FREQME (`CLOCK_CTRL.FRO_HF_ENA`).
    ///
    /// Defaults to `false` - the frequency-measurement block is its only
    /// consumer.
    pub enable_fro_hf_measure: bool,

    /// Enable `clk_in` fan-out to MICFIL, CAN0/1, I3C0/1, SAI0/1 and CLKOUT
    /// (`CLOCK_CTRL.CLKIN_ENA`).
    ///
    /// Defaults to `false`, and has no effect unless [`Self::sosc`] brought
    /// the source up.
    pub enable_clk_in_periph: bool,

    /// Enable `clk_in` fan-out to FREQME, USB HS and LPTMR0/1
    /// (`CLOCK_CTRL.CLKIN_ENA_FM_USBH_LPT`).
    ///
    /// Defaults to `false`, and has no effect unless [`Self::sosc`] brought
    /// the source up. The USB HS driver sets this itself, so it only needs
    /// setting here for FREQME or LPTMR0/1.
    pub enable_clk_in_fm_usbh_lpt: bool,

    /// Bring SOSC up from an external crystal or clock, producing `clk_in`.
    ///
    /// `None` (the default) leaves SOSC disabled, so [`super::Clocks::clk_in`]
    /// stays `None` and any peripheral selecting it is rejected. There is no
    /// usable default because the value has to match the crystal fitted to the
    /// board.
    ///
    /// The USB HS PHY requires this: its 480 MHz PLL has no reference other
    /// than `clk_in`.
    pub sosc: Option<SoscConfig>,

    /// Bring `fro_hf_div` up with this divisor (`FROHFDIV`).
    ///
    /// `None` (the default) leaves the divider in its reset state, which is
    /// *halted* - so `Clocks::fro_hf_div` stays `None` and any peripheral
    /// selecting it gets [`super::ClockError::BadConfig`].
    pub fro_hf_div: Option<Div<8>>,

    /// `system_clk` = `main_clk` / this divisor (`AHBCLKDIV`).
    ///
    /// `None` (the default) leaves the reset value of divide-by-1. Only
    /// division is available: raising `main_clk` above its 48 MHz reset value
    /// needs `SPC0` voltage and `FMU0` wait-state changes that are not
    /// implemented.
    pub ahb_div: Option<Div<8>>,

    /// Enable `clk_16k`, the VBAT domain's FRO16K (`VBAT0.FROCTLA.FRO_EN` +
    /// `VBAT0.FROCLKE.CLKE`).
    ///
    /// Defaults to `false` - nothing needs it until a driver (RTC) selects
    /// it. Unlike `xtal32k`, this is a free-running RC oscillator with no
    /// ready flag to poll, so enabling it costs one register write and takes
    /// effect immediately.
    pub enable_clk_16k: bool,
}

impl Default for ClocksConfig {
    fn default() -> Self {
        Self {
            enable_clk_1m: true,
            enable_fro_12m_periph: true,
            enable_fro_hf_measure: false,
            enable_clk_in_periph: false,
            enable_clk_in_fm_usbh_lpt: false,
            sosc: None,
            fro_hf_div: None,
            ahb_div: None,
            enable_clk_16k: false,
        }
    }
}
