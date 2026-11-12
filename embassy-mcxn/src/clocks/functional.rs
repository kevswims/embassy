//! The per-peripheral functional-clock layer.
//!
//! Between a [`Clocks`] tree node and a peripheral sit a `*CLKSEL` mux and
//! usually a `*CLKDIV` divider. SYSCON has 54 of the former and 58 of the
//! latter, and they are not interchangeable: each selector has its own enum
//! type whose variants are *anonymous* (`Enum0`, or `Enum0x0` - the two
//! spellings appear at the same field width) and whose ordering differs between
//! peripherals. `Adc0clkselSel` is `{1=PLL0, 2=fro_hf, 3=fro_12m, 4=clk_in}`
//! while `FlexioclkselSel` is `{1=PLL0, 2=clk_in, 3=fro_hf, 4=fro_12m}`.
//!
//! So there can be no shared mux enum, and every peripheral needs its own
//! mapping from "source I want" to "raw value for this register". That mapping
//! is what [`impl_functional_clock!`] declares, and each row carries the raw
//! value it expects so that a PAC regeneration renumbering the anonymous
//! variants breaks the build instead of silently selecting a different clock.
//!
//! Bus-clocked peripherals - PORT, GPIO, DMA, CRC and friends - have no mux and
//! no divider. They do not implement [`FunctionalClock`]; they use
//! [`super::enable_and_reset`] and read [`Clocks::system_clk_hz`] if they need
//! a rate.

use super::CLOCKS;
use super::gate::Gate;
use super::types::{ClockError, Clocks};

/// A peripheral's functional-clock configuration: which source, and how far
/// divided.
pub trait FunctionalClock {
    /// Source selection, plus a divisor for peripherals that have a `*CLKDIV`.
    type Config;

    /// Program the mux and divider, returning the resulting frequency in Hz.
    ///
    /// # Contract
    ///
    /// Called with this peripheral's functional clock gated *off*, inside a
    /// critical section, with a read-only view of [`Clocks`]. Must not touch
    /// core system clock configuration and must not block for long.
    fn configure(cfg: &Self::Config, clocks: &Clocks) -> Result<u32, ClockError>;
}

/// Enable a peripheral's clock gate, program its functional clock, and pulse
/// its reset. Returns the functional clock frequency in Hz.
///
/// The ordering is load-bearing. The mux and divider are programmed while the
/// peripheral's clock is gated off, because switching a live mux or reloading a
/// live divider glitches. And the `?` sits before the gate is enabled, so if
/// the requested source turns out to be unavailable the peripheral is left
/// disabled rather than half-configured.
///
/// # Safety
///
/// This peripheral must not already be in use.
pub unsafe fn enable_and_reset_with_clock<G>(cfg: &G::Config) -> Result<u32, ClockError>
where
    G: Gate + FunctionalClock,
{
    unsafe {
        G::disable_clock();

        let hz = critical_section::with(|cs| {
            let clocks = CLOCKS.borrow_ref(cs);
            let clocks = clocks.as_ref().ok_or(ClockError::NeverInitialized)?;
            G::configure(cfg, clocks)
        })?;

        G::enable_clock();
        while !G::is_clock_enabled() {}
        core::arch::asm!("dsb sy; isb sy", options(nomem, nostack, preserves_flags));

        super::gate::pulse_reset::<G>();

        Ok(hz)
    }
}

/// Program a SYSCON divider whose register has `div`/`reset`/`halt` fields.
///
/// The divisor is written while the divider is halted and held in reset, then
/// both are released.
///
/// This does not wait on `UNSTAB`. `WDT0CLKDIV` and `WDT1CLKDIV` latch `UNSTAB`
/// high the moment `HALT` is released and never clear it, even though the
/// divider output is correct - measured against `embassy-time`, `wdt0_clk`
/// comes out at 249,993 Hz for the expected 250,000 (`clk_1m` / 1 / 4).
/// `FROHFDIV` and `AHBCLKDIV` do clear it, so the flag is not usable uniformly
/// and polling it would hang forever.
macro_rules! apply_div_no_poll {
    ($reg:expr, $div:expr) => {{
        use crate::bits::flag;

        let r = $reg;
        r.modify(|w| {
            w.set_div($div.into_bits());
            w.set_halt(flag(1));
            w.set_reset(flag(1));
        });
        r.modify(|w| {
            w.set_halt(flag(0));
            w.set_reset(flag(0));
        });
    }};
}

/// Implement [`FunctionalClock`] for a peripheral, generating its source enum,
/// its config struct, and the `configure()` body.
///
/// Each row of `sources` reads
/// `Variant => PacVariant @ raw : tree_accessor`, where `raw` is the register
/// value that `PacVariant` must encode and `tree_accessor` is a [`Clocks`]
/// `ensure_*` method. The `raw` value is asserted against the PAC at compile
/// time - that check is the reason the anonymous, per-peripheral-permuted
/// selector enums are safe to use.
///
/// `sel:` takes the register accessor expression, so array-indexed selectors
/// work by writing the index at the invocation site
/// (`pac::SYSCON0.fcclksel(3)`).
///
/// `div:` is either `none` for a peripheral with no `*CLKDIV`, or
/// `full(accessor, EnumPrefix, BITS)`.
macro_rules! impl_functional_clock {
    // No divider: the config carries just a source.
    (
        $periph:ident,
        name: $src_enum:ident / $cfg_struct:ident,
        sel: $sel:expr,
        div: none,
        sources: {
            $( $variant:ident => $raw:literal : $accessor:ident ),+ $(,)?
        }
    ) => {
        impl_functional_clock!(@sources $periph, $src_enum, { $($variant => $raw : $accessor),+ });

        #[doc = concat!("Functional clock configuration for `", stringify!($periph), "`.")]
        #[derive(Copy, Clone, Debug, PartialEq, Eq)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        pub struct $cfg_struct {
            /// Which source to select.
            pub source: $src_enum,
        }

        impl crate::clocks::FunctionalClock for crate::peripherals::$periph {
            type Config = $cfg_struct;

            fn configure(
                cfg: &Self::Config,
                clocks: &crate::clocks::Clocks,
            ) -> Result<u32, crate::clocks::ClockError> {
                let src_hz = impl_functional_clock!(@select $sel, cfg, clocks, $src_enum, { $($variant => $raw : $accessor),+ })?;
                Ok(src_hz)
            }
        }
    };

    // With a divider: the config carries a width-bound divisor too, applied
    // with `apply_div_no_poll!` - not `apply_div_full!`, since WDT0CLKDIV and
    // WDT1CLKDIV proved a SYSCON divider's UNSTAB does not reliably clear (see
    // that macro's doc), and nothing has verified any of the other 55 do
    // either, so no caller here should be left spinning on a bit that may
    // never move.
    (
        $periph:ident,
        name: $src_enum:ident / $cfg_struct:ident,
        sel: $sel:expr,
        div: full($div_reg:expr, $bits:expr),
        sources: {
            $( $variant:ident => $raw:literal : $accessor:ident ),+ $(,)?
        }
    ) => {
        impl_functional_clock!(@sources $periph, $src_enum, { $($variant => $raw : $accessor),+ });

        #[doc = concat!("Functional clock configuration for `", stringify!($periph), "`.")]
        #[derive(Copy, Clone, Debug, PartialEq, Eq)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        pub struct $cfg_struct {
            /// Which source to select.
            pub source: $src_enum,
            /// Divisor applied to the selected source.
            pub div: crate::clocks::config::Div<$bits>,
        }

        impl crate::clocks::FunctionalClock for crate::peripherals::$periph {
            type Config = $cfg_struct;

            fn configure(
                cfg: &Self::Config,
                clocks: &crate::clocks::Clocks,
            ) -> Result<u32, crate::clocks::ClockError> {
                let src_hz = impl_functional_clock!(@select $sel, cfg, clocks, $src_enum, { $($variant => $raw : $accessor),+ })?;
                apply_div_no_poll!($div_reg, cfg.div);
                Ok(src_hz / cfg.div.into_divisor())
            }
        }
    };

    // As `full`, for the CTIMER dividers.
    (
        $periph:ident,
        name: $src_enum:ident / $cfg_struct:ident,
        sel: $sel:expr,
        div: ctimer($div_reg:expr, $bits:expr),
        sources: {
            $( $variant:ident => $raw:literal : $accessor:ident ),+ $(,)?
        }
    ) => {
        impl_functional_clock!(@sources $periph, $src_enum, { $($variant => $raw : $accessor),+ });

        #[doc = concat!("Functional clock configuration for `", stringify!($periph), "`.")]
        #[derive(Copy, Clone, Debug, PartialEq, Eq)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        pub struct $cfg_struct {
            /// Which source to select.
            pub source: $src_enum,
            /// Divisor applied to the selected source.
            pub div: crate::clocks::config::Div<$bits>,
        }

        impl crate::clocks::FunctionalClock for crate::peripherals::$periph {
            type Config = $cfg_struct;

            fn configure(
                cfg: &Self::Config,
                clocks: &crate::clocks::Clocks,
            ) -> Result<u32, crate::clocks::ClockError> {
                let src_hz = impl_functional_clock!(@select $sel, cfg, clocks, $src_enum, { $($variant => $raw : $accessor),+ })?;
                apply_div_no_poll!($div_reg, cfg.div);
                Ok(src_hz / cfg.div.into_divisor())
            }
        }
    };

    // Shared: the source enum plus its compile-time pin against the PAC's raw
    // values (see the doc comment above).
    (@sources $periph:ident, $src_enum:ident, { $( $variant:ident => $raw:literal : $accessor:ident ),+ }) => {
        #[doc = concat!("Functional clock sources selectable for `", stringify!($periph), "`.")]
        #[derive(Copy, Clone, Debug, PartialEq, Eq)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        pub enum $src_enum {
            $(
                #[doc = concat!("Source ", stringify!($variant), ".")]
                $variant,
            )+
        }

    };

    // Shared: resolve the selected source's rate, then write the mux. Rate
    // resolution happens first so a failure leaves the mux untouched.
    (@select $sel:expr, $cfg:ident, $clocks:ident, $src_enum:ident, { $( $variant:ident => $raw:literal : $accessor:ident ),+ }) => {{
        let src_hz = match $cfg.source {
            $( $src_enum::$variant => $clocks.$accessor()?, )+
        };
        $sel.modify(|w| {
            w.set_sel(match $cfg.source {
                $( $src_enum::$variant => crate::bits::flag($raw), )+
            })
        });
        Result::<u32, crate::clocks::ClockError>::Ok(src_hz)
    }};
}

// LPUARTn's mux and divider actually belong to LP_FLEXCOMMn (RM 66.2.4: a
// flexcomm's bus clock, functional clock and PSELID must be brought up before
// its LPUART sub-block is touched at all), but FCCLKSEL/FLEXCOMMCLKDIV are
// array-indexed by instance number, which is the same number either name
// carries - so this is written directly against `LPUARTn` rather than adding
// a separate `LP_FLEXCOMMn` clock owner. `clk_1m` is deliberately absent: at
// 1 MHz the baud-rate generator can't reach any usable baud rate above a
// handful of bit/s (SBR would need to be well under 1).
macro_rules! impl_lpuart_functional_clock {
    ($n:literal) => {
        ::paste::paste! {
            #[cfg(mcx_n_lpuart)]
            impl_functional_clock! {
                [<LPUART $n>],
                name: [<Lpuart $n ClockSource>] / [<Lpuart $n ClockConfig>],
                sel: crate::pac::SYSCON0.fcclksel($n),
                div: full(crate::pac::SYSCON0.flexcommclkdiv($n), 8),
                sources: {
                    PllDiv => 1 : ensure_pll_div,
                    Fro12m => 2 : ensure_fro_12m,
                    FroHfDiv => 3 : ensure_fro_hf_div,
                }
            }
        }
    };
}

impl_lpuart_functional_clock!(0);
impl_lpuart_functional_clock!(1);
impl_lpuart_functional_clock!(2);
impl_lpuart_functional_clock!(3);
impl_lpuart_functional_clock!(4);
impl_lpuart_functional_clock!(5);
impl_lpuart_functional_clock!(6);
impl_lpuart_functional_clock!(7);
impl_lpuart_functional_clock!(8);
impl_lpuart_functional_clock!(9);

// ADC0/ADC1's mux and divider are SYSCON-owned (`ADCnCLKSEL`/`ADCnCLKDIV`),
// unlike RTC0's `Ctrl.clk_sel` or LPTMR's `PSR.PCS` - both of those live in
// the peripheral's own register block and can't be touched from inside
// `FunctionalClock::configure()` (see `crate::rtc`, `crate::lptmr`). ADC has
// no such restriction. `clk_1m` is deliberately absent: at 1 MHz the ADC's
// conversion clock would be far below its usable range.
//
// `FroHf` is `Clocks::fro_hf` - the FIRC output at whatever `FIRCCFG.RANGE`
// selects - not the fixed 48 MHz tap. At a 144 MHz range this mux delivers
// 144 MHz, so the divider has to bring ADCK back inside RM 41.6.1's window;
// `crate::adc` rejects a resolved rate outside it rather than converting out
// of spec.
macro_rules! impl_adc_functional_clock {
    ($n:literal) => {
        ::paste::paste! {
            #[cfg(mcx_n_adc)]
            impl_functional_clock! {
                [<ADC $n>],
                name: [<Adc $n ClockSource>] / [<Adc $n ClockConfig>],
                sel: crate::pac::SYSCON0.[<adc $n clksel>](),
                div: full(crate::pac::SYSCON0.[<adc $n clkdiv>](), 3),
                sources: {
                    Pll0 => 1 : ensure_pll0_clk,
                    Fro12m => 3 : ensure_fro_12m,
                    FroHf => 2 : ensure_fro_hf,
                }
            }
        }
    };
}

impl_adc_functional_clock!(0);
impl_adc_functional_clock!(1);

// FlexCAN. The SYSCON accessors are named `flexcan{n}clksel`/`clkdiv`, not
// `can{n}...`, and the mux's `Enum3` is `fro_hf`.
//
// NXP's own board example drives FlexCAN from PLL1 at 50 MHz, but this crate
// has no PLL bring-up (see `ClocksConfig`), so `fro_hf` is the usable source.
// `crate::can` computes its bit timing against whatever rate this returns
// rather than assuming NXP's 50 MHz - which matters because `fro_hf` follows
// `FIRCCFG.RANGE` and so is 144 MHz, not 48, once the FIRC range is raised.
macro_rules! impl_can_functional_clock {
    ($n:literal) => {
        ::paste::paste! {
            #[cfg(mcx_n_can)]
            impl_functional_clock! {
                [<CAN $n>],
                name: [<Can $n ClockSource>] / [<Can $n ClockConfig>],
                sel: crate::pac::SYSCON0.[<flexcan $n clksel>](),
                div: full(crate::pac::SYSCON0.[<flexcan $n clkdiv>](), 8),
                sources: {
                    Pll0 => 1 : ensure_pll0_clk,
                    FroHf => 3 : ensure_fro_hf,
                }
            }
        }
    };
}

impl_can_functional_clock!(0);
impl_can_functional_clock!(1);

// FlexSPI's serial root clock. The SYSCON accessors are spelled with an
// underscore the other peripherals' are not - `flex_spiclksel`/`flex_spiclkdiv`
// - and the divider field is only 3 bits wide, so the reachable range is /1../8.
//
// The mux has four real sources (RM "Chapter 33 Clocking", figure 136); PLL1 and
// the USB PLL are absent here because this crate cannot bring either up. Its
// reset state is `SEL = no clock` *and* `CLKDIV.HALT = 1`, and none of the board
// clock presets change that, so unlike most peripherals FlexSPI arrives with no
// functional clock at all - `enable_and_reset_with_clock` is what first routes
// one.
//
// `fro_hf` follows `SCG0.FIRCCFG.RANGE`, so it delivers 144 MHz once the FIRC
// range is raised; the divider has to bring the result back inside RM Table
// 320's window (300/200/100 MHz for OD/SD/MD), which `crate::flexspi` range
// checks rather than assuming.
#[cfg(mcx_n_flexspi)]
impl_functional_clock! {
    FLEXSPI0,
    name: FlexspiClockSource / FlexspiClockConfig,
    sel: crate::pac::SYSCON0.flex_spiclksel(),
    div: full(crate::pac::SYSCON0.flex_spiclkdiv(), 3),
    sources: {
        Pll0 => 1 : ensure_pll0_clk,
        FroHf => 3 : ensure_fro_hf,
    }
}

// FlexIO's shifter/timer clock. All four implemented sources are reachable; the
// mux also lists `pll1_clk0`, which PLL1 bring-up would have to exist for.
//
// The choice matters more here than for most peripherals: FlexIO samples input
// pins at a quarter of this rate (RM 71.6.3), so it caps how fast anything
// bidirectional built on FlexIO can run, and PLL0 at 150 MHz is the only source
// that gets that ceiling above 12 MHz.
#[cfg(mcx_n_flexio)]
impl_functional_clock! {
    FLEXIO0,
    name: Flexio0ClockSource / Flexio0ClockConfig,
    sel: crate::pac::SYSCON0.flexioclksel(),
    div: full(crate::pac::SYSCON0.flexioclkdiv(), 8),
    sources: {
        Pll0 => 1 : ensure_pll0_clk,
        ClkIn => 2 : ensure_clk_in,
        FroHf => 3 : ensure_fro_hf,
        Fro12m => 4 : ensure_fro_12m,
    }
}

// OSTIMER0's mux has no divider. `clk_1m` is the source the time driver wants:
// it is exactly 1 MHz, matching embassy-time's configured tick rate 1:1, and
// unlike the other two options it needs nothing from the VBAT power domain.
#[cfg(mcx_n_ostimer)]
impl_functional_clock! {
    OSTIMER0,
    name: OstimerClockSource / OstimerClockConfig,
    sel: crate::pac::SYSCON0.ostimerclksel(),
    div: none,
    sources: {
        Clk16k => 0 : ensure_clk_16k,
        Xtal32k => 1 : ensure_xtal32k,
        Clk1M => 2 : ensure_clk_1m,
    }
}

/// Functional clock configuration shared by `WWDT0` and `WWDT1`.
///
/// Both are driven from `clk_1m` (RM "Chapter 33 Clocking", figures 147 and
/// 148), then through the SYSCON divider below, then through a fixed /4 inside
/// the watchdog itself. WWDT0 has no `*CLKSEL` at all, its source being
/// hardwired; WWDT1 has one, and its reset value already selects `clk_1m`.
///
/// WWDT1's other options (`clk_16k` from the VBAT domain, `fro_hf_div`) would
/// need this to grow a source field. Neither is useful yet: the VBAT sources are
/// not implemented and `fro_hf_div` is halted unless asked for.
#[cfg(mcx_n_wwdt)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct WwdtClockConfig {
    /// Divisor applied to `clk_1m` ahead of the watchdog's own fixed /4.
    pub div: crate::clocks::config::Div<6>,
}

#[cfg(mcx_n_wwdt)]
impl crate::clocks::FunctionalClock for crate::peripherals::WWDT0 {
    type Config = WwdtClockConfig;

    fn configure(cfg: &Self::Config, clocks: &Clocks) -> Result<u32, ClockError> {
        // No WDT0CLKSEL exists: the source is hardwired to clk_1m, so there is
        // only a divider to program.
        let src_hz = clocks.ensure_clk_1m()?;
        apply_div_no_poll!(crate::pac::SYSCON0.wdt0clkdiv(), cfg.div);
        Ok(src_hz / cfg.div.into_divisor())
    }
}

#[cfg(mcx_n_wwdt)]
impl crate::clocks::FunctionalClock for crate::peripherals::WWDT1 {
    type Config = WwdtClockConfig;

    fn configure(cfg: &Self::Config, clocks: &Clocks) -> Result<u32, ClockError> {
        // Both 2 and 3 select clk_1m on this mux; 2 is canonical.
        let src_hz = clocks.ensure_clk_1m()?;
        crate::pac::SYSCON0
            .wdt1clksel()
            .modify(|w| w.set_sel(crate::bits::flag(2)));
        apply_div_no_poll!(crate::pac::SYSCON0.wdt1clkdiv(), cfg.div);
        Ok(src_hz / cfg.div.into_divisor())
    }
}

// CTIMER0/1. `CTIMERCLKSEL`/`CTIMERCLKDIV` are arrays of five, one per instance,
// so this is generic in the instance number rather than hand-written per
// instance like WWDT0/WWDT1.
//
// `CTIMERCLKSEL` resets to 0xF ("no clock"), so the mux has to be written even
// to get the nominal default - which the generated `configure` always does.
//
// `PLL1_clk0` (`Enum0x2`) is omitted because PLL1 is not implemented, and the
// SAI MCLK and LP-oscillator options because neither is a node `Clocks` models.
macro_rules! impl_ctimer_functional_clock {
    ($n:literal) => {
        ::paste::paste! {
            #[cfg(mcx_n_ctimer)]
            impl_functional_clock! {
                [<CTIMER $n>],
                name: [<Ctimer $n ClockSource>] / [<Ctimer $n ClockConfig>],
                sel: crate::pac::SYSCON0.ctimerclksel($n),
                div: ctimer(crate::pac::SYSCON0.ctimerclkdiv($n), 8),
                sources: {
                    Clk1M => 0 : ensure_clk_1m,
                    Pll0 => 1 : ensure_pll0_clk,
                    FroHf => 3 : ensure_fro_hf,
                    Fro12m => 4 : ensure_fro_12m,
                }
            }

            // `clk_1m` undivided, which is what this config resolved to before
            // it grew a source field. Kept so existing callers that only care
            // about a 1 MHz tick keep working unchanged.
            #[cfg(mcx_n_ctimer)]
            impl Default for [<Ctimer $n ClockConfig>] {
                fn default() -> Self {
                    Self {
                        source: [<Ctimer $n ClockSource>]::Clk1M,
                        div: crate::clocks::config::Div::no_div(),
                    }
                }
            }
        }
    };
}

impl_ctimer_functional_clock!(0);
impl_ctimer_functional_clock!(1);
