#![no_std]
#![no_main]

//! Find the SPC write that resets the board.
//!
//! `power_overdrive.rs`, `clocks_144m.rs` and `clocks_150m.rs` all reset partway
//! through raising the core rail, looping forever. RTT cannot tell us where,
//! because the reset takes the log with it.
//!
//! So this leaves a breadcrumb in **uninitialised SRAM**, which a warm reset does
//! not clear and which `cortex-m-rt` does not zero (`.uninit`). Before every SPC
//! write the step number and a snapshot of `VD_STAT`/`ACTIVE_CFG` are stored;
//! after the reset the next boot reads them back and reports exactly which write
//! died and what the detectors had latched at the time.
//!
//! It also logs `CMC0`'s reset reason, which says *why* — in particular whether
//! `is_voltage_detect()` is set, which would confirm a detector tripped despite
//! being muted.
//!
//! The SPC sequence below is `power::set_active_mode` unrolled inline, so each
//! individual register write can be bracketed. Keep it in step with `src/power.rs`.

use defmt::{info, warn};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn as hal;
use embassy_time::Timer;
use hal::config::Config;
use hal::pac;
use panic_probe as _;

/// Settling delay inserted after each SPC write. The sequence passes with 40 ms
/// between writes; 0 tests whether `SC[BUSY]` alone is sufficient.
const STEP_DELAY_MS: u64 = 0;

/// `true` calls `power::set_active_mode` — the path `power_overdrive.rs` takes,
/// which resets the board. `false` performs the same register writes inline,
/// which does not. This is the A/B that isolates the two.
const USE_HAL_SET_ACTIVE_MODE: bool = true;

/// Announce the step about to run, with the detector state going into it, then
/// pause long enough for RTT to drain. If the board resets on the next write,
/// the last line the host received names it.
///
/// Retention in `.uninit` was tried first and is not usable here: the stack
/// overlaps it, so the marker came back holding spilled register values.
async fn step(n: u32) {
    info!(
        "step {}: {} | VD_STAT 0x{=u32:08x} ACTIVE_CFG 0x{=u32:08x}",
        n,
        step_name(n),
        pac::SPC0.vd_stat().read().0,
        pac::SPC0.active_cfg().read().0,
    );
    if STEP_DELAY_MS > 0 {
        Timer::after_millis(STEP_DELAY_MS).await;
    }
}

/// Names for the step numbers, so the post-reset report is readable.
fn step_name(step: u32) -> &'static str {
    match step {
        0 => "start (nothing written yet)",
        1 => "mute detectors: ACTIVE_CFG LVDE/HVDE -> 0",
        2 => "set DCDC drive strength",
        3 => "set CORELDO drive strength -> Normal",
        10..=19 => "level walk: DCDC_VDD_LVL step up",
        20..=29 => "level walk: CORELDO_VDD_LVL step up",
        30 => "restore CORELDO drive strength",
        31 => "clear VD_STAT",
        32 => "restore detector enables",
        33 => "done - rail is up",
        40 => "set_sram_margin (retimes SRAM under the live stack)",
        41 => "set_flash_wait_states",
        _ => "unknown",
    }
}

fn attach_main_clk(scs: u8, expect: u8) {
    pac::SCG0.rccr().modify(|w| w.set_scs(embassy_mcxn::bits::flag(scs)));
    let mut spins = 0u32;
    while embassy_mcxn::bits::raw(pac::SCG0.csr().read().scs()) != expect {
        spins += 1;
        if spins > 1_000_000 {
            warn!("main_clk never reported the requested source");
            return;
        }
    }
}

fn wait_not_busy() {
    let mut spins = 0u32;
    while embassy_mcxn::bits::raw(pac::SPC0.sc().read().busy()) == 1 {
        spins += 1;
        if spins > 1_000_000 {
            warn!("SPC stayed busy");
            return;
        }
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let _p = hal::init(Config::default());

    // Read this before anything else touches SPC.
    let reason = hal::reset_reason::reset_reason();

    info!("=== SPC reset probe ===");
    info!("reset reason: {}", reason);
    info!(
        "  por {} / voltage_detect {} / warm {} / fatal {} / pin {} / dap {} / lockup {} / reset_ack_timeout {}",
        reason.is_por(),
        reason.is_voltage_detect(),
        reason.is_warm(),
        reason.is_fatal(),
        reason.is_pin(),
        reason.is_dap(),
        reason.is_lockup(),
        reason.is_reset_ack_timeout(),
    );

    info!("(a reset truncates the log; the last step logged is the one that died)");

    info!("initial SPC state:");
    info!("  {}", hal::power::active_mode_status());
    info!(
        "  CNTRL: coreldo_en {} dcdc_en {}",
        pac::SPC0.cntrl().read().coreldo_en(),
        pac::SPC0.cntrl().read().dcdc_en()
    );
    info!("  VD_STAT 0x{=u32:08x}", pac::SPC0.vd_stat().read().0);
    info!("  ACTIVE_CFG 0x{=u32:08x}", pac::SPC0.active_cfg().read().0);

    // Give the log time to drain before anything risky.
    Timer::after_millis(500).await;

    info!("parking main_clk on SIRC");
    attach_main_clk(embassy_mcxn::bits::flag(2), embassy_mcxn::bits::flag(2));
    info!("parked; CSR.SCS {}", pac::SCG0.csr().read().scs());

    if USE_HAL_SET_ACTIVE_MODE {
        info!("calling power::set_active_mode(Over, Normal, Normal) - result logged, not unwrapped");
        Timer::after_millis(300).await;
        let r = hal::power::set_active_mode(&hal::power::ActiveModeConfig {
            core_voltage: hal::power::CoreVoltage::Over,
            core_drive_strength: hal::power::DriveStrength::Normal,
            dcdc_drive_strength: hal::power::DriveStrength::Normal,
        });
        match r {
            Ok(()) => info!("set_active_mode returned Ok"),
            Err(e) => warn!("set_active_mode returned Err: {}", e),
        }
        Timer::after_millis(300).await;
        info!("  {}", hal::power::active_mode_status());
        info!("  VD_STAT 0x{=u32:08x}", pac::SPC0.vd_stat().read().0);
        Timer::after_millis(300).await;
        info!("survived the HAL call");
        attach_main_clk(embassy_mcxn::bits::flag(3), embassy_mcxn::bits::flag(3));
        loop {
            Timer::after_secs(2).await;
            info!("idle (HAL path)");
        }
    }

    let cntrl = pac::SPC0.cntrl().read();
    let dcdc_en = cntrl.dcdc_en();
    let coreldo_en = cntrl.coreldo_en();

    step(0).await;
    wait_not_busy();

    // Step 1: mute every detector.
    step(1).await;
    let saved = pac::SPC0.active_cfg().read();
    pac::SPC0.active_cfg().modify(|w| {
        w.set_core_lvde(embassy_mcxn::bits::from_bool(false));
        w.set_sys_lvde(embassy_mcxn::bits::from_bool(false));
        w.set_io_lvde(embassy_mcxn::bits::from_bool(false));
        w.set_core_hvde(embassy_mcxn::bits::from_bool(false));
        w.set_sys_hvde(embassy_mcxn::bits::from_bool(false));
        w.set_io_hvde(embassy_mcxn::bits::from_bool(false));
    });
    wait_not_busy();

    // Step 2: DCDC drive strength.
    if embassy_mcxn::bits::is_set(dcdc_en) {
        step(2).await;
        pac::SPC0
            .active_cfg()
            .modify(|w| w.set_dcdc_vdd_ds(embassy_mcxn::bits::flag(2)));
        wait_not_busy();
    }

    // Step 3: CORELDO drive strength must be Normal before the level moves.
    if embassy_mcxn::bits::is_set(coreldo_en) {
        step(3).await;
        pac::SPC0
            .active_cfg()
            .modify(|w| w.set_coreldo_vdd_ds(embassy_mcxn::bits::flag(1)));
        wait_not_busy();
    }

    // Steps 10+/20+: walk both regulators up to Over, one level per round.
    let dcdc_levels = [embassy_mcxn::bits::flag(2), embassy_mcxn::bits::flag(3)];
    let ldo_levels = [embassy_mcxn::bits::flag(2), embassy_mcxn::bits::flag(3)];

    for (round, (dcdc_lvl, ldo_lvl)) in dcdc_levels.iter().zip(ldo_levels.iter()).enumerate() {
        if embassy_mcxn::bits::is_set(dcdc_en) {
            step(10 + round as u32).await;
            pac::SPC0.active_cfg().modify(|w| w.set_dcdc_vdd_lvl(*dcdc_lvl));
            wait_not_busy();
        }
        if embassy_mcxn::bits::is_set(coreldo_en) {
            step(20 + round as u32).await;
            pac::SPC0.active_cfg().modify(|w| w.set_coreldo_vdd_lvl(*ldo_lvl));
            wait_not_busy();
        }
    }

    // Step 30: requested drive strength is only safe now.
    if embassy_mcxn::bits::is_set(coreldo_en) {
        step(30).await;
        pac::SPC0
            .active_cfg()
            .modify(|w| w.set_coreldo_vdd_ds(embassy_mcxn::bits::flag(1)));
        wait_not_busy();
    }

    // Step 31: discard anything latched while muted.
    step(31).await;
    let flags = pac::SPC0.vd_stat().read();
    pac::SPC0.vd_stat().write_value(flags);

    // Step 32: restore the enables.
    step(32).await;
    pac::SPC0.active_cfg().modify(|w| {
        w.set_core_lvde(saved.core_lvde());
        w.set_sys_lvde(saved.sys_lvde());
        w.set_io_lvde(saved.io_lvde());
        w.set_core_hvde(saved.core_hvde());
        w.set_sys_hvde(saved.sys_hvde());
        w.set_io_hvde(saved.io_hvde());
    });

    step(33).await;
    info!("SPC sequence COMPLETED without reset");
    info!("  {}", hal::power::active_mode_status());
    info!("  VD_STAT 0x{=u32:08x}", pac::SPC0.vd_stat().read().0);

    // The step power_overdrive.rs does next, and the scariest one: retiming the
    // SRAM array that the live stack is sitting in.
    step(40).await;
    match hal::power::set_sram_margin(hal::power::sram_margin_for(hal::power::CoreVoltage::Over)) {
        Ok(()) => info!("step 40 OK: sram margin set"),
        Err(e) => warn!("step 40 returned Err: {}", e),
    }
    if STEP_DELAY_MS > 0 {
        Timer::after_millis(STEP_DELAY_MS).await;
    }

    step(41).await;
    let rwsc = hal::power::flash_wait_states(hal::power::CoreVoltage::Over, 48_000_000);
    info!("step 41: flash_wait_states for Over @48MHz = {}", rwsc);
    if let Some(rwsc) = rwsc {
        hal::power::set_flash_wait_states(rwsc);
    }
    if STEP_DELAY_MS > 0 {
        Timer::after_millis(STEP_DELAY_MS).await;
    }
    info!("step 42: both post-rail steps survived");

    attach_main_clk(embassy_mcxn::bits::flag(3), embassy_mcxn::bits::flag(3));
    info!("main_clk restored; CSR.SCS {}", pac::SCG0.csr().read().scs());

    loop {
        Timer::after_secs(2).await;
        info!("idle");
    }
}
