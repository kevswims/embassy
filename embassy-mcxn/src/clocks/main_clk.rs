//! Selecting what drives `main_clk`.
//!
//! # Park low, program, attach
//!
//! Raising the core clock touches three subsystems whose ordering is
//! unforgiving, so the whole move is done from a slow, always-valid source:
//!
//! 1. **Park** `main_clk` on SIRC (12 MHz). SIRC is running at every core
//!    voltage and every wait-state count, so from here nothing that follows can
//!    under-clock the flash or overload the regulator.
//! 2. **Raise the core voltage** ([`crate::power`]), then the SRAM timing
//!    margin, then the flash wait states.
//! 3. **Bring the target source up** — it is not feeding the core yet, so a
//!    `FIRCCFG` rewrite is harmless here and would not be if FIRC were still
//!    attached.
//! 4. **Attach** it and wait for `CSR[SCS]` to report the handover done.
//!
//! Parking first is what makes this correct in both directions without
//! branching on which way the frequency is going, and it is what NXP's
//! `BOARD_BootClock*` functions do. It is also load-bearing for a reason that
//! has nothing to do with clocks: raising the rail at 48 MHz **wedges the
//! part** rather than merely resetting it. See `DRIVERS.md`'s hazards section.
//!
//! # Why no RAM function is needed
//!
//! The `FCTRL[RWSC]` and `RCCR[SCS]` writes are peripheral stores; the
//! instructions around them are fetched from flash. The hazard would be running
//! flash code with too *few* wait states — but `RWSC` is only ever written while
//! parked at 12 MHz, and only ever to a value at least as large as 12 MHz
//! needs. The dangerous direction, lowering `RWSC` ahead of a frequency drop,
//! never occurs because the voltage is never lowered. So no `.data`-resident
//! trampoline: NXP run the same sequence from flash.

use super::types::ClockError;
use crate::pac;
use crate::power::{self, ActiveModeConfig, ClockNode, CoreVoltage, DriveStrength};

use super::config::MainClock;

/// Bounded spin, matching `configure_sosc`. This runs once, before the executor
/// exists, and a hang here happens before a probe can attach.
const SPIN_CAP: u32 = 5_000_000;

/// What a completed switch settled on.
pub(super) struct Outcome {
    /// `main_clk`, in Hz.
    pub main_clk_hz: u32,
    /// The core voltage the rail ended up at.
    pub core_voltage: CoreVoltage,
    /// PLL0's rate, if this configuration brought it up.
    pub pll0_hz: Option<u32>,
}

/// Point `main_clk` at `target`, taking the core voltage and flash wait states
/// with it.
pub(super) fn switch(target: MainClock) -> Result<Outcome, ClockError> {
    let current_scs = pac::SCG0.csr().read().scs();

    // A source this crate cannot describe means a bootloader moved `main_clk`
    // somewhere unexpected. Switching out from under that is more likely to
    // break the caller's assumptions than to help.
    if crate::bits::raw(current_scs) != 3 && crate::bits::raw(current_scs) != 2 {
        return Err(ClockError::UnsupportedMainClockSource);
    }

    let range = pac::SCG0.firccfg().read().range();

    match target {
        // The default costs no register writes at all when the part is already
        // where it resets to, which keeps `MainClock::FroHf48m` byte-identical
        // to the previous verify-only behaviour.
        MainClock::FroHf48m if crate::bits::raw(current_scs) == 3 && crate::bits::raw(range) == 0 => Ok(Outcome {
            main_clk_hz: super::source::FIRC_SCLK_HZ,
            core_voltage: observed_core_voltage()?,
            pll0_hz: None,
        }),
        MainClock::FroHf48m => configure(0, super::source::FIRC_SCLK_HZ),
        MainClock::FroHf144m => configure(1, super::source::FIRC_FCLK_HZ),
        MainClock::Pll0At150m => configure_pll0(),
    }
}

/// The rail voltage as read back, rejected if it sits on a reserved encoding.
fn observed_core_voltage() -> Result<CoreVoltage, ClockError> {
    power::active_mode_status()
        .effective_core_voltage()
        .ok_or(ClockError::UnknownCoreVoltage)
}

fn configure(range: u8, target_hz: u32) -> Result<Outcome, ClockError> {
    // What this frequency needs. Computed from `main_clk`, not `system_clk`:
    // `AHBCLKDIV` only divides, so sizing the wait states for the undivided
    // rate is always conservative however the divider is later set.
    let voltage = required_voltage(target_hz, &[ClockNode::SystemClk, ClockNode::FroHf])?;
    let rwsc =
        power::flash_wait_states(voltage, target_hz).ok_or(ClockError::MainClockUnsupportedFrequency(target_hz))?;

    // 1. Park. Everything after this runs at 12 MHz.
    attach(2, 2)?;

    // 2. Rail, then SRAM timing, then flash wait states - each one ahead of the
    //    frequency that will need it.
    power::set_active_mode(&ActiveModeConfig {
        core_voltage: voltage,
        core_drive_strength: DriveStrength::Normal,
        dcdc_drive_strength: DriveStrength::Normal,
    })
    .map_err(ClockError::Power)?;

    power::set_sram_margin(power::sram_margin_for(voltage)).map_err(ClockError::Power)?;

    // Raising `RWSC` is safe at any frequency; it is only ever lowered after a
    // frequency drop, which this path never performs.
    power::set_flash_wait_states(rwsc);

    // 3. Bring FIRC up at the requested range. Safe to rewrite `FIRCCFG` only
    //    because the core is parked on SIRC.
    configure_firc(range)?;

    // 4. Attach.
    attach(3, 3)?;

    Ok(Outcome {
        main_clk_hz: target_hz,
        core_voltage: voltage,
        pll0_hz: None,
    })
}

/// Take `main_clk` to PLL0 at 150 MHz.
///
/// Same park/rail/wait-state prologue as [`configure`], then PLL0 replaces the
/// FIRC-range step. FIRC itself stays at 48 MHz throughout - it is the PLL's
/// reference, so it has to keep running, and leaving it at 48 MHz is what keeps
/// every peripheral mux's `fro_hf` input where it was.
fn configure_pll0() -> Result<Outcome, ClockError> {
    let target_hz = super::pll::PLL0_HZ;

    // `fro_hf` is deliberately absent: it stays at 48 MHz here, so its ceiling
    // does not bind. `pll0_clk` does.
    let voltage = required_voltage(target_hz, &[ClockNode::SystemClk, ClockNode::Pll0Clk])?;
    let rwsc =
        power::flash_wait_states(voltage, target_hz).ok_or(ClockError::MainClockUnsupportedFrequency(target_hz))?;

    attach(2, 2)?;

    power::set_active_mode(&ActiveModeConfig {
        core_voltage: voltage,
        core_drive_strength: DriveStrength::Normal,
        dcdc_drive_strength: DriveStrength::Normal,
    })
    .map_err(ClockError::Power)?;

    power::set_sram_margin(power::sram_margin_for(voltage)).map_err(ClockError::Power)?;
    power::set_flash_wait_states(rwsc);

    // The reference. `configure_firc` also sets `FIRC_SCLK_PERIPH_EN`, which
    // `ApllctrlSource::Firc` requires, and it is safe to (re)write the range
    // here because the core is parked on SIRC.
    configure_firc(0)?;

    let pll0_hz = super::pll::configure_pll0_150m()?;

    attach(5, 5)?;

    Ok(Outcome {
        main_clk_hz: pll0_hz,
        core_voltage: voltage,
        pll0_hz: Some(pll0_hz),
    })
}

/// The lowest core voltage at which `hz` is legal on every node in `nodes`.
///
/// Picking the lowest rather than always going to overdrive keeps the part at
/// the voltage it resets to whenever the frequency does not demand more.
///
/// `nodes` matters because the ceilings are not uniform (RM Table 320). Driving
/// `main_clk` from FIRC makes the rate a `fro_hf` rate *and* a `system_clk`
/// rate, so both limits bind; driving it from PLL0 leaves `fro_hf` at 48 MHz
/// and constrains `pll0_clk` instead.
fn required_voltage(hz: u32, nodes: &[ClockNode]) -> Result<CoreVoltage, ClockError> {
    for v in [CoreVoltage::Mid, CoreVoltage::Normal, CoreVoltage::Over] {
        if nodes.iter().all(|&n| hz <= power::max_hz(n, v)) {
            return Ok(v);
        }
    }
    Err(ClockError::MainClockUnsupportedFrequency(hz))
}

/// Program `FIRCCFG.RANGE` and wait for FIRC to report itself valid.
///
/// Mirrors `CLOCK_SetupFROHFClocking`. `FIRCCFG` is written whole, as the SDK
/// does - it carries only `RANGE`.
fn configure_firc(range: u8) -> Result<(), ClockError> {
    let csr = pac::SCG0.firccsr();

    // `LK` freezes FIRCCSR. Nothing here sets it, but a bootloader can have,
    // and the enables below would then be dropped silently.
    if crate::bits::raw(csr.read().lk()) == 1 {
        csr.modify(|w| w.set_lk(crate::bits::flag(0)));
        if crate::bits::raw(csr.read().lk()) == 1 {
            return Err(ClockError::FircLocked);
        }
    }

    pac::SCG0.firccfg().write(|w| w.set_range(crate::bits::flag(range)));

    csr.modify(|w| {
        w.set_firc_sclk_periph_en(crate::bits::from_bool(true));
        w.set_firc_fclk_periph_en(crate::bits::from_bool(true));
        w.set_fircen(crate::bits::from_bool(true));
    });

    let mut spins = 0u32;
    while crate::bits::raw(csr.read().fircvld()) != 1 {
        spins += 1;
        if spins > SPIN_CAP {
            return Err(ClockError::FircNotValid);
        }
    }

    Ok(())
}

/// Request a `main_clk` source and wait for the mux to actually follow.
///
/// `RCCR[SCS]` is the request; `CSR[SCS]` is read-only and reports the source in
/// use, so polling it is what makes the handover observed rather than assumed.
fn attach(request: u8, expect: u8) -> Result<(), ClockError> {
    pac::SCG0.rccr().modify(|w| w.set_scs(crate::bits::flag(request)));

    let mut spins = 0u32;
    while crate::bits::raw(pac::SCG0.csr().read().scs()) != expect {
        spins += 1;
        if spins > SPIN_CAP {
            return Err(ClockError::MainClockNotStable);
        }
    }

    // The next instruction fetch runs at the new rate.
    unsafe { core::arch::asm!("dsb sy", "isb sy", options(nomem, nostack, preserves_flags)) };

    Ok(())
}
