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

use super::gate::Gate;
use super::types::{ClockError, Clocks};
use super::CLOCKS;

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

/// Program a divider whose register has the full `div`/`reset`/`halt`/`unstab`
/// field set. That is 55 of SYSCON's 58 dividers, including every
/// per-peripheral one.
///
/// The divisor is written while the divider is halted and held in reset, then
/// both are released, then we wait for the output to report stable.
///
/// Note the field polarity here reads the way it is named - `Halt::Run` is 0,
/// `Reset::Released` is 0, `Unstab::Stable` is 0. embassy-mcxa's equivalent
/// macro works against inverted `On`/`Off` naming, so do not pattern-match the
/// two against each other.
// Currently unused: of the per-peripheral dividers wired up so far, WWDT's need
// `apply_div_no_poll!` and CTIMER's has a different field shape entirely. Kept
// as the correct sequence for a divider whose `UNSTAB` does clear.
#[allow(unused_macros)]
macro_rules! apply_div_full {
    ($reg:expr, $divty:ident, $div:expr) => {{
        ::paste::paste! {
            use crate::pac::syscon::{[<$divty Halt>], [<$divty Reset>], [<$divty Unstab>]};

            let r = $reg;
            r.modify(|w| {
                w.set_div($div.into_bits());
                w.set_halt([<$divty Halt>]::Halt);
                w.set_reset([<$divty Reset>]::Asserted);
            });
            r.modify(|w| {
                w.set_halt([<$divty Halt>]::Run);
                w.set_reset([<$divty Reset>]::Released);
            });
            while r.read().unstab() == [<$divty Unstab>]::Ongoing {}
        }
    }};
}

/// Program a full-shape divider without waiting for `UNSTAB` to clear.
///
/// `WDT0CLKDIV` and `WDT1CLKDIV` latch `UNSTAB` high the moment `HALT` is
/// released and never clear it, even though the divider output is correct -
/// measured against `embassy-time`, `wdt0_clk` comes out at 249,993 Hz for the
/// expected 250,000 (`clk_1m` / 1 / 4). `FROHFDIV` and `AHBCLKDIV` do clear it,
/// so the flag is not usable uniformly and polling it here would hang forever.
macro_rules! apply_div_no_poll {
    ($reg:expr, $divty:ident, $div:expr) => {{
        ::paste::paste! {
            use crate::pac::syscon::{[<$divty Halt>], [<$divty Reset>]};

            let r = $reg;
            r.modify(|w| {
                w.set_div($div.into_bits());
                w.set_halt([<$divty Halt>]::Halt);
                w.set_reset([<$divty Reset>]::Asserted);
            });
            r.modify(|w| {
                w.set_halt([<$divty Halt>]::Run);
                w.set_reset([<$divty Reset>]::Released);
            });
        }
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
        sel: $sel:expr, $sel_ty:ident,
        div: none,
        sources: {
            $( $variant:ident => $pac_variant:ident @ $raw:literal : $accessor:ident ),+ $(,)?
        }
    ) => {
        impl_functional_clock!(@sources $periph, $src_enum, $sel_ty, { $($variant => $pac_variant @ $raw : $accessor),+ });

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
                let src_hz = impl_functional_clock!(@select $sel, $sel_ty, cfg, clocks, $src_enum, { $($variant => $pac_variant : $accessor),+ })?;
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
        sel: $sel:expr, $sel_ty:ident,
        div: full($div_reg:expr, $divty:ident, $bits:expr),
        sources: {
            $( $variant:ident => $pac_variant:ident @ $raw:literal : $accessor:ident ),+ $(,)?
        }
    ) => {
        impl_functional_clock!(@sources $periph, $src_enum, $sel_ty, { $($variant => $pac_variant @ $raw : $accessor),+ });

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
                let src_hz = impl_functional_clock!(@select $sel, $sel_ty, cfg, clocks, $src_enum, { $($variant => $pac_variant : $accessor),+ })?;
                apply_div_no_poll!($div_reg, $divty, cfg.div);
                Ok(src_hz / cfg.div.into_divisor())
            }
        }
    };

    // Shared: the source enum plus its compile-time pin against the PAC's raw
    // values (see the doc comment above).
    (@sources $periph:ident, $src_enum:ident, $sel_ty:ident, { $( $variant:ident => $pac_variant:ident @ $raw:literal : $accessor:ident ),+ }) => {
        #[doc = concat!("Functional clock sources selectable for `", stringify!($periph), "`.")]
        #[derive(Copy, Clone, Debug, PartialEq, Eq)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        pub enum $src_enum {
            $(
                #[doc = concat!("Source ", stringify!($variant), ".")]
                $variant,
            )+
        }

        const _: () = {
            $( assert!(crate::pac::syscon::$sel_ty::$pac_variant.to_bits() == $raw); )+
        };
    };

    // Shared: resolve the selected source's rate, then write the mux. Rate
    // resolution happens first so a failure leaves the mux untouched.
    (@select $sel:expr, $sel_ty:ident, $cfg:ident, $clocks:ident, $src_enum:ident, { $( $variant:ident => $pac_variant:ident : $accessor:ident ),+ }) => {{
        let src_hz = match $cfg.source {
            $( $src_enum::$variant => $clocks.$accessor()?, )+
        };
        $sel.modify(|w| {
            w.set_sel(match $cfg.source {
                $( $src_enum::$variant => crate::pac::syscon::$sel_ty::$pac_variant, )+
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
            #[cfg(mcxn_lpuart)]
            impl_functional_clock! {
                [<LPUART $n>],
                name: [<Lpuart $n ClockSource>] / [<Lpuart $n ClockConfig>],
                sel: crate::pac::SYSCON0.fcclksel($n), FcclkselSel,
                div: full(crate::pac::SYSCON0.flexcommclkdiv($n), Flexcommclkdiv, 8),
                sources: {
                    Fro12m   => Enum0x2 @ 2 : ensure_fro_12m,
                    FroHfDiv => Enum0x3 @ 3 : ensure_fro_hf_div,
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

// OSTIMER0's mux has no divider. `clk_1m` is the source the time driver wants:
// it is exactly 1 MHz, matching embassy-time's configured tick rate 1:1, and
// unlike the other two options it needs nothing from the VBAT power domain.
#[cfg(mcxn_ostimer)]
impl_functional_clock! {
    OSTIMER0,
    name: OstimerClockSource / OstimerClockConfig,
    sel: crate::pac::SYSCON0.ostimerclksel(), OstimerclkselSel,
    div: none,
    sources: {
        Clk16k  => Enum0 @ 0 : ensure_clk_16k,
        Xtal32k => Enum1 @ 1 : ensure_xtal32k,
        Clk1M   => Enum2 @ 2 : ensure_clk_1m,
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
#[cfg(mcx_wwdt)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct WwdtClockConfig {
    /// Divisor applied to `clk_1m` ahead of the watchdog's own fixed /4.
    pub div: crate::clocks::config::Div<6>,
}

#[cfg(mcx_wwdt)]
impl crate::clocks::FunctionalClock for crate::peripherals::WWDT0 {
    type Config = WwdtClockConfig;

    fn configure(cfg: &Self::Config, clocks: &Clocks) -> Result<u32, ClockError> {
        // No WDT0CLKSEL exists: the source is hardwired to clk_1m, so there is
        // only a divider to program.
        let src_hz = clocks.ensure_clk_1m()?;
        apply_div_no_poll!(crate::pac::SYSCON0.wdt0clkdiv(), Wdt0clkdiv, cfg.div);
        Ok(src_hz / cfg.div.into_divisor())
    }
}

#[cfg(mcx_wwdt)]
impl crate::clocks::FunctionalClock for crate::peripherals::WWDT1 {
    type Config = WwdtClockConfig;

    fn configure(cfg: &Self::Config, clocks: &Clocks) -> Result<u32, ClockError> {
        use crate::pac::syscon::Wdt1clkselSel;

        // Both Enum2 and Enum3 select clk_1m on this mux; Enum2 is canonical.
        const _: () = assert!(Wdt1clkselSel::Enum2.to_bits() == 2);

        let src_hz = clocks.ensure_clk_1m()?;
        crate::pac::SYSCON0
            .wdt1clksel()
            .modify(|w| w.set_sel(Wdt1clkselSel::Enum2));
        apply_div_no_poll!(crate::pac::SYSCON0.wdt1clkdiv(), Wdt1clkdiv, cfg.div);
        Ok(src_hz / cfg.div.into_divisor())
    }
}

/// Functional clock configuration for a `CTIMERn`.
///
/// Sourced from `clk_1m`. `CTIMERCLKSEL` offers PLL and `fro_hf` options too,
/// but those are not brought up yet, so this only needs a divisor.
///
/// `CTIMERCLKSEL`/`CTIMERCLKDIV` are arrays of five, one per instance, so
/// [`impl_ctimer_functional_clock!`] is generic in the instance number rather
/// than this being hand-written per instance like WWDT0/WWDT1.
macro_rules! impl_ctimer_functional_clock {
    ($n:literal) => {
        ::paste::paste! {
            #[cfg(mcx_ctimer)]
            #[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
            #[cfg_attr(feature = "defmt", derive(defmt::Format))]
            pub struct [<Ctimer $n ClockConfig>] {
                /// Divisor applied to `clk_1m`.
                pub div: crate::clocks::config::Div<8>,
            }

            #[cfg(mcx_ctimer)]
            impl crate::clocks::FunctionalClock for crate::peripherals::[<CTIMER $n>] {
                type Config = [<Ctimer $n ClockConfig>];

                fn configure(cfg: &Self::Config, clocks: &Clocks) -> Result<u32, ClockError> {
                    use crate::pac::syscon::{CtimerclkdivHalt, CtimerclkselSel};

                    // CTIMERCLKSEL resets to 0xF ("no clock"), so the mux must
                    // be written even to get the nominal default. Enum0x0 is
                    // FRO_1M, i.e. clk_1m.
                    const _: () = assert!(CtimerclkselSel::Enum0x0.to_bits() == 0);

                    let src_hz = clocks.ensure_clk_1m()?;

                    crate::pac::SYSCON0
                        .ctimerclksel($n)
                        .modify(|w| w.set_sel(CtimerclkselSel::Enum0x0));

                    // This divider has its own shape: `reset` is a bare bool
                    // rather than an enum, and `halt`/`unstab` are spelled
                    // Enable/Disable rather than Run/Halt and Stable/Ongoing -
                    // `Enable` meaning running and stable respectively. So
                    // none of the shared divider macros fit. Like every other
                    // SYSCON divider it also resets halted.
                    let d = crate::pac::SYSCON0.ctimerclkdiv($n);
                    d.modify(|w| {
                        w.set_div(cfg.div.into_bits());
                        w.set_halt(CtimerclkdivHalt::Disable);
                        w.set_reset(true);
                    });
                    d.modify(|w| {
                        w.set_halt(CtimerclkdivHalt::Enable);
                        w.set_reset(false);
                    });

                    Ok(src_hz / cfg.div.into_divisor())
                }
            }
        }
    };
}

impl_ctimer_functional_clock!(0);
impl_ctimer_functional_clock!(1);
