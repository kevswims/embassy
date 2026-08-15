#![no_std]
#![no_main]

//! Capture the error that `hal::init` swallows when asked for 150 MHz.
//!
//! `chips/mcxn947.rs` does `clocks::init(cfg).unwrap()` before interrupts or the
//! time driver exist. If that errors, the core panics into `udf` at once and
//! probe-rs reports an access-port fault long before it reads RTT, so the actual
//! `ClockError` is never seen. `clocks::init` also refuses a second call with
//! `AlreadyInitialized`, so it cannot be retried after a working boot either.
//!
//! Hence: no `hal::init` at all. A bare `cortex_m_rt::entry` calls
//! `clocks::init` exactly once with `main_clk = Pll0At150m` and logs the
//! `Result`. `defmt-rtt` needs no HAL setup, and delays use `asm::delay` since
//! there is no time driver.
//!
//! Reading the trace:
//! * `Err(..)` names the failure outright;
//! * `Ok` means the sequence itself is sound and the fault is elsewhere in `init`;
//! * output stopping at "calling" means it wedged inside rather than returning.

use defmt::{error, info, warn};
use defmt_rtt as _;
use embassy_mcxn as hal;
use hal::clocks::MainClock;
use hal::pac;
use panic_probe as _;

/// Roughly one second at the 48 MHz reset clock.
const ONE_SEC: u32 = 48_000_000;

fn dump(tag: &str) {
    info!(
        "  {}: SCG0.CSR 0x{=u32:08x} APLLCSR 0x{=u32:08x} ACTIVE_CFG 0x{=u32:08x} FMU0.FCTRL 0x{=u32:08x}",
        tag,
        pac::SCG0.csr().read().0,
        pac::SCG0.apllcsr().read().0,
        pac::SPC0.active_cfg().read().0,
        pac::FMU0.fctrl().read().0,
    );
}

#[cortex_m_rt::entry]
fn main() -> ! {
    info!("=== 150 MHz boot probe (no hal::init) ===");
    dump("at reset");

    // Recovery window: a wedge here is otherwise unrecoverable over SWD, since
    // the firmware re-wedges on every reset before a probe can halt the core.
    warn!("pausing 3s before clocks::init - halt now to recover a wedged board");
    cortex_m::asm::delay(ONE_SEC * 3);

    let mut cfg = hal::config::Config::default().clock_cfg;
    cfg.main_clk = MainClock::Pll0At150m;

    info!("calling clocks::init(Pll0At150m)");
    cortex_m::asm::delay(ONE_SEC / 4);

    match hal::clocks::init(cfg) {
        Ok(()) => {
            info!("clocks::init returned Ok");
            dump("after Ok");
            let hz = hal::clocks::with_clocks(|c| (c.main_clk_hz, c.system_clk_hz));
            match hz {
                Some((m, s)) => info!("  main_clk {} Hz, system_clk {} Hz", m, s),
                None => warn!("  Clocks not readable"),
            }
        }
        Err(e) => {
            error!("clocks::init returned Err: {}", e);
            dump("after Err");
        }
    }

    loop {
        cortex_m::asm::delay(ONE_SEC * 2);
        info!("idle");
    }
}
