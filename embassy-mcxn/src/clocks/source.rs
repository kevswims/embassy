//! Bring-up of the clock sources and the core dividers.
//!
//! Each function here follows the same shape: read the request out of
//! [`ClocksConfig`], return early if it wasn't asked for, poke registers, wait
//! for the hardware to report the result stable, then record what happened into
//! [`Clocks`]. Nothing else in the crate writes these registers.

use super::config::{ClocksConfig, Div, SoscMode};
use super::types::{ClockError, Clocks};
use crate::pac;

/// SCG's SIRC runs at a fixed 12 MHz on this chip - there is no range select.
const SIRC_HZ: u32 = 12_000_000;

/// The two FIRC fan-out taps to peripherals, independently gated by
/// `FIRCCSR.FIRC_SCLK_PERIPH_EN` and `FIRC_FCLK_PERIPH_EN`.
const FIRC_SCLK_HZ: u32 = 48_000_000;
const FIRC_FCLK_HZ: u32 = 144_000_000;

/// Program a divider whose register has `div`, `halt` and `unstab` but no
/// `reset` field. `FROHFDIV` is the only one this driver touches.
///
/// The divisor is written while the divider is halted, then the halt is
/// released, then we wait for the hardware to report the output stable.
macro_rules! apply_div_no_reset {
    ($reg:expr, $divty:ident, $div:expr) => {{
        ::paste::paste! {
            use crate::pac::syscon::{[<$divty Halt>], [<$divty Unstab>]};

            let r = $reg;
            r.modify(|w| {
                w.set_div($div.into_bits());
                w.set_halt([<$divty Halt>]::Halt);
            });
            r.modify(|w| w.set_halt([<$divty Halt>]::Run));
            while r.read().unstab() == [<$divty Unstab>]::Ongoing {}
        }
    }};
}

/// Program a divider whose register has only `div` and `unstab` - no `halt`,
/// no `reset`. `AHBCLKDIV` is the only one of these.
///
/// With nothing to halt, the divisor is written directly and we wait for the
/// output to settle.
macro_rules! apply_div_no_halt {
    ($reg:expr, $divty:ident, $div:expr) => {{
        ::paste::paste! {
            use crate::pac::syscon::[<$divty Unstab>];

            let r = $reg;
            r.modify(|w| w.set_div($div.into_bits()));
            while r.read().unstab() == [<$divty Unstab>]::Ongoing {}
        }
    }};
}

/// Confirm `main_clk` is still on its reset-default source and report its rate.
///
/// Switching `main_clk` is not implemented, so this verifies rather than
/// configures. Raising the FIRC range would additionally require raising the
/// core LDO level in `SPC0` and the flash read wait states in `FMU0`, in that
/// order, which is why the range is only read here.
pub(super) fn verify_main_clock() -> Result<u32, ClockError> {
    use pac::scg::{FirccfgRange, RccrScs};

    if pac::SCG0.rccr().read().scs() != RccrScs::Firc {
        return Err(ClockError::UnsupportedMainClockSource);
    }

    Ok(match pac::SCG0.firccfg().read().range() {
        FirccfgRange::Firc48mhz => FIRC_SCLK_HZ,
        FirccfgRange::Firc144mhz => FIRC_FCLK_HZ,
    })
}

/// Record SIRC's peripheral fan-out.
///
/// SIRC has no enable bit - it is always running - so only the fan-out gate to
/// peripherals decides whether `fro_12m` is selectable. It is set at reset.
pub(super) fn record_sirc(clocks: &mut Clocks) {
    if pac::SCG0.sirccsr().read().sirc_clk_periph_en() {
        clocks.fro_12m = Some(SIRC_HZ);
    }
}

/// Record FIRC's two peripheral fan-out taps.
///
/// These are independent of `FIRCCFG.RANGE`, which selects what `main_clk`
/// sees: the 48 MHz and 144 MHz taps are separately gated and both are enabled
/// at reset.
pub(super) fn record_firc(clocks: &mut Clocks) {
    let csr = pac::SCG0.firccsr().read();

    if csr.firc_sclk_periph_en() {
        clocks.fro_hf_48m = Some(FIRC_SCLK_HZ);
    }
    if csr.firc_fclk_periph_en() {
        clocks.fro_hf_144m = Some(FIRC_FCLK_HZ);
    }
}

/// Pick `SOSCCFG.RANGE` for an input frequency.
///
/// The ranges the field can encode are contiguous and cover 16-66 MHz only, so
/// anything outside that - including the 12 MHz crystals some NXP parts use -
/// has no representation. Range boundaries overlap by one endpoint in the RM's
/// wording; 20/30/50 MHz are assigned to the lower of the two candidates.
fn sosc_range(hz: u32) -> Result<pac::scg::SosccfgRange, ClockError> {
    use pac::scg::SosccfgRange;

    Ok(match hz {
        16_000_000..=20_000_000 => SosccfgRange::Freq16to20mhz,
        20_000_001..=30_000_000 => SosccfgRange::LowFreq,
        30_000_001..=50_000_000 => SosccfgRange::MediumFreq,
        50_000_001..=66_000_000 => SosccfgRange::HighFreq,
        _ => return Err(ClockError::SoscUnsupportedFrequency(hz)),
    })
}

/// Bring SOSC up, if asked for, and record `clk_in`.
///
/// SOSC is off at reset and is the only source of `clk_in`. It is left to
/// explicit configuration because the frequency is a property of the board,
/// not the chip: nothing on-chip measures the crystal, and every consumer
/// derives its divider from the number recorded here.
///
/// `SOSCCFG` is only writable while `SOSCEN` is clear, hence the
/// disable/configure/enable order rather than a single write.
pub(super) fn configure_sosc(cfg: &ClocksConfig, clocks: &mut Clocks) -> Result<(), ClockError> {
    use pac::scg::{Erefs, SosccsrLk};

    let Some(sosc) = cfg.sosc else {
        return Ok(());
    };

    let range = sosc_range(sosc.hz)?;
    let csr = pac::SCG0.sosccsr();

    // `LK` freezes all of SOSCCSR, including SOSCEN. Nothing here sets it, but
    // a bootloader can have, and the writes below would otherwise be dropped
    // silently and leave SOSCVLD never setting - which looks identical to a
    // dead crystal.
    if csr.read().lk() == SosccsrLk::WriteDisabled {
        return Err(ClockError::SoscLocked);
    }

    // Clear SOSCEN before touching SOSCCFG. It is clear at reset, but `init()`
    // is not guaranteed to be running from reset.
    csr.modify(|w| w.set_soscen(false));

    pac::SCG0.sosccfg().modify(|w| {
        w.set_range(range);
        w.set_erefs(match sosc.mode {
            SoscMode::Crystal => Erefs::Internal,
            SoscMode::ExternalClock => Erefs::External,
        });
    });

    csr.modify(|w| w.set_soscen(true));

    // Enable the SOSC clock monitor. Nominally only loss-of-clock detection,
    // but NXP's working USB firmware sets it and the USB HS PHY PLL does not
    // lock without it.
    csr.modify(|w| w.set_sosccm(true));

    // A crystal takes on the order of a millisecond to start; an externally
    // driven clock is valid almost at once. The cap is deliberately generous -
    // this runs once, before the executor exists, and the failure it guards
    // against (absent or mismatched crystal) would otherwise hang the boot.
    let mut spins = 0u32;
    while !csr.read().soscvld() {
        spins += 1;
        if spins > 5_000_000 {
            return Err(ClockError::SoscNotValid);
        }
    }

    clocks.clk_in = Some(sosc.hz);

    Ok(())
}

/// Enable the source fan-outs owned by `SYSCON0.CLOCK_CTRL`.
///
/// Every bit here is clear at reset, so a peripheral mux selecting `clk_1m` or
/// `clk_in` produces nothing until this runs.
///
/// `CLOCK_CTRL` resets to `0x81`, and bits 0 and 7 of that value are not
/// modelled by the PAC fieldset - so this must `modify`, never `write`, or it
/// would silently clear them.
pub(super) fn configure_clock_ctrl(cfg: &ClocksConfig, clocks: &mut Clocks) {
    pac::SYSCON0.clock_ctrl().modify(|w| {
        // Two separate bits: one enables FRO_1MHz for RTC and UTICK, the other
        // routes it into the clock generator so that a `*CLKSEL` selecting
        // clk_1m actually receives it. Both are needed.
        w.set_fro1mhz_ena(cfg.enable_clk_1m);
        w.set_fro1mhz_clk_ena(cfg.enable_clk_1m);

        w.set_fro12mhz_ena(cfg.enable_fro_12m_periph);
        w.set_fro_hf_ena(cfg.enable_fro_hf_measure);
        w.set_clkin_ena(cfg.enable_clk_in_periph);
        w.set_clkin_ena_fm_usbh_lpt(cfg.enable_clk_in_fm_usbh_lpt);
    });

    if cfg.enable_clk_1m {
        clocks.clk_1m = Some(1_000_000);
    }
}

/// Bring `fro_hf_div` up, if asked for.
///
/// `FROHFDIV` resets halted, so the node stays unavailable unless a divisor is
/// requested. The input is `fro_hf` at whatever rate `FIRCCFG.RANGE` selects -
/// the same value `main_clk` runs at.
pub(super) fn configure_fro_hf_div(cfg: &ClocksConfig, clocks: &mut Clocks) {
    let Some(div) = cfg.fro_hf_div else {
        return;
    };

    apply_div_no_reset!(pac::SYSCON0.frohfdiv(), Frohfdiv, div);

    clocks.fro_hf_div = Some(clocks.main_clk_hz / div.into_divisor());
}

/// Program `AHBCLKDIV` if asked for, and report the resulting divisor.
///
/// Division only. `main_clk` cannot currently be raised, so `system_clk` can
/// only be taken below 48 MHz, which needs no voltage or wait-state change.
pub(super) fn configure_ahb_div(cfg: &ClocksConfig) -> u32 {
    match cfg.ahb_div {
        Some(div) => {
            apply_div_no_halt!(pac::SYSCON0.ahbclkdiv(), Ahbclkdiv, div);
            div.into_divisor()
        }
        // Reset value divides by 1. `AHBCLKDIV.div` divides by `div + 1`.
        None => pac::SYSCON0.ahbclkdiv().read().div() as u32 + 1,
    }
}

/// FRO16K's fixed rate.
const FRO_16K_HZ: u32 = 16_384;

/// Bring `clk_16k` up, if asked for.
///
/// `VBAT0.FROCTLA.FRO_EN` starts the oscillator; `VBAT0.FROCLKE.CLKE` then
/// fans it out to consumers. Unlike `xtal32k`, FRO16K is a free-running RC
/// oscillator with no crystal settling time, so there is no ready flag to
/// poll.
///
/// This crate does not model the VBAT domain's per-power-domain gating (the
/// `CLKE` nibble has one bit per domain on chips with more than one), so all
/// four bits are set: RTC's tick clock has to survive every low-power mode it
/// runs through, and there is no correctness downside to leaving unused
/// domains enabled here.
///
/// UNVERIFIED ON HARDWARE: the one attempt to run an `RTC0` example that
/// exercises this path left the target unresponsive to the debug probe (not
/// a normal crash - `probe-rs` could not reconnect at all, needing a full
/// power cycle). Root cause not confirmed; this VBAT0 write is one of several
/// candidates (the others being RTC0's own write-unlock sequence). Do not
/// trust this path until it has been re-run and diagnosed.
pub(super) fn configure_vbat(cfg: &ClocksConfig, clocks: &mut Clocks) {
    if !cfg.enable_clk_16k {
        return;
    }

    pac::VBAT0.froctla().modify(|w| w.set_fro_en(true));
    pac::VBAT0.froclke().modify(|w| w.set_clke(0b1111));

    clocks.clk_16k = Some(FRO_16K_HZ);
}

/// Compile-time guard: `Div<8>` must cover the full `AHBCLKDIV`/`FROHFDIV`
/// range, and the encoding must be divide-by-`n+1`.
const _: () = {
    assert!(Div::<8>::no_div().into_divisor() == 1);
    assert!(Div::<8>::new(256).into_bits() == 255);
    assert!(Div::<8>::from_divisor(257).is_none());
};
