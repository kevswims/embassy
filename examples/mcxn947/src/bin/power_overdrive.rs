#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn as hal;
use embassy_time::{Duration, Timer};
use hal::pac;
use hal::pac::scg::{CsrScs, RccrScs};
use hal::power::{self, ActiveModeConfig, ClockNode, CoreVoltage, DriveStrength};
use panic_probe as _;

/// Bounded spin, matching the convention in the HAL's own clock bring-up.
const SPIN_CAP: u32 = 5_000_000;

/// Point `main_clk` at a clock source and wait for the mux to actually follow.
///
/// `RCCR[SCS]` is the request; `CSR[SCS]` is read-only and reports the source
/// in use, so the poll is what makes the switch observed rather than assumed.
fn attach_main_clk(scs: RccrScs, expect: CsrScs) {
    pac::SCG0.rccr().modify(|w| w.set_scs(scs));

    let mut spins = 0u32;
    while pac::SCG0.csr().read().scs() != expect {
        spins += 1;
        if spins > SPIN_CAP {
            defmt::panic!("main_clk never switched to {}", expect);
        }
    }
    unsafe { core::arch::asm!("dsb sy", "isb sy", options(nomem, nostack, preserves_flags)) };
}

/// SIRC is 12 MHz, always running, and valid at every core voltage.
fn park_on_sirc() {
    attach_main_clk(RccrScs::Sirc, CsrScs::Sirc);
}

fn restore_firc() {
    attach_main_clk(RccrScs::Firc, CsrScs::Firc);
}

/// Raise the core rail to overdrive with `main_clk` left at its reset default.
///
/// This isolates the SPC sequence from the clock switch: raising the voltage is
/// safe at any frequency the *lower* voltage already supported, so if the part
/// stops responding here, the fault is in the regulator sequencing and not in
/// anything to do with clocks.
///
/// The part is expected to keep running normally throughout. `embassy-time`
/// runs on `clk_1m`, which is independent of the core clock, so the tick below
/// staying at one second is itself a check that nothing was disturbed.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let _p = hal::init(hal::config::Config::default());

    defmt::info!("SPC overdrive probe (main_clk untouched)");

    // A boot-time write that wedges the part is unrecoverable over SWD: the
    // firmware re-wedges on every reset before `--connect-under-reset` can win
    // the race, leaving the ISP button as the only way back. An earlier version
    // of this example did exactly that. The pause gives a probe time to attach
    // and halt the core before anything risky happens.
    //
    // Keep this ahead of every SPC write in any example that changes the rails.
    defmt::warn!("pausing 5s before touching SPC - halt now to recover a wedged board");
    Timer::after(Duration::from_secs(5)).await;

    // RTT is polled, not pushed, so anything logged in the last few
    // milliseconds before a reset never reaches the host. Each marker in the
    // critical path below is followed by a drain pause so that the last line
    // seen actually identifies the step that failed.
    macro_rules! mark {
        ($($arg:tt)*) => {{
            defmt::info!($($arg)*);
            Timer::after(Duration::from_millis(300)).await;
        }};
    }

    let before = power::active_mode_status();
    mark!("  before: {}", before);

    let core_hz = hal::clocks::with_clocks(|c| c.system_clk_hz).unwrap();
    let fro_hf = hal::clocks::with_clocks(|c| c.fro_hf).unwrap();

    // The reset rail turns out to be 1.0 V, where RM Table 320 caps `fro_hf` at
    // 48 MHz - exactly where it already is. Report the headroom that leaves.
    if let Some(v) = before.effective_core_voltage() {
        defmt::info!(
            "  at {} the ceilings are system_clk {} Hz / fro_hf {} Hz (currently {} / {})",
            v,
            power::max_hz(ClockNode::SystemClk, v),
            power::max_hz(ClockNode::FroHf, v),
            core_hz,
            fro_hf,
        );
    }

    // Park on SIRC before touching the rails. Doing this at 48 MHz wedged the
    // board hard enough to need an ISP recovery: the core draws several times
    // the current there, so the regulator transient is correspondingly worse,
    // and every LVD/HVD detector is armed. NXP park on FRO12M in every
    // `BOARD_BootClock*` for the same reason.
    //
    // Raw PAC because `clocks` does not own a main-clock switch yet; that
    // arrives with the 144 MHz work and subsumes this.
    mark!("  parking main_clk on SIRC");
    park_on_sirc();
    mark!("  parked; CSR.SCS now {}", pac::SCG0.csr().read().scs());

    // `set_active_mode` mutes the voltage detectors across the transition and
    // walks both regulators up in lockstep; both are load-bearing, and the
    // history is in `DRIVERS.md`'s hazards section.
    power::set_active_mode(&ActiveModeConfig {
        core_voltage: CoreVoltage::Over,
        core_drive_strength: DriveStrength::Normal,
        dcdc_drive_strength: DriveStrength::Normal,
    })
    .unwrap();
    mark!("  rails raised: {}", power::active_mode_status());

    // Conservative by design: the RM documents no 1.2 V margin, so overdrive
    // pairs with the 1.1 V setting. See `power::sram_margin_for`.
    power::set_sram_margin(power::sram_margin_for(CoreVoltage::Over)).unwrap();
    mark!("  sram margin set");

    // Back to FIRC. Still 48 MHz - the point of this probe is the rail, not the
    // frequency - but now with 1.2 V of headroom under it.
    mark!("  restoring main_clk to FIRC");
    restore_firc();
    mark!("  restored; CSR.SCS now {}", pac::SCG0.csr().read().scs());

    let after = power::active_mode_status();
    defmt::info!("  after:  {}", after);

    defmt::assert_eq!(
        after.effective_core_voltage(),
        Some(CoreVoltage::Over),
        "core rail did not reach overdrive"
    );

    // Wait states are left alone here: nothing has gone faster, and RWSC is
    // only ever raised ahead of a frequency increase.
    defmt::assert_eq!(
        after.flash_wait_states,
        before.flash_wait_states,
        "flash wait states changed unexpectedly"
    );

    defmt::info!(
        "  headroom now: system_clk {} Hz / fro_hf {} Hz",
        power::max_hz(ClockNode::SystemClk, CoreVoltage::Over),
        power::max_hz(ClockNode::FroHf, CoreVoltage::Over),
    );
    defmt::info!("SPC overdrive probe passed");

    // Still executing from flash and still scheduling: if the rail change had
    // disturbed either, we would not get here.
    let mut n = 0u32;
    loop {
        Timer::after(Duration::from_secs(1)).await;
        n += 1;
        defmt::info!("alive at overdrive, {}s", n);
    }
}
