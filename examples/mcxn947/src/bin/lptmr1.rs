#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn as hal;
use embassy_time::Instant;
use hal::bind_interrupts;
use hal::lptmr::{Config, InterruptHandler, Lptmr, LptmrClockConfig, LptmrClockSource};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    LPTMR1 => InterruptHandler<hal::peripherals::LPTMR1>;
});

bind_interrupts!(struct Irqs16k {
    LPTMR0 => InterruptHandler<hal::peripherals::LPTMR0>;
});

/// As lptmr.rs, on LPTMR1 - confirms the gate/clock generalization holds for
/// an arbitrary instance, not just LPTMR0. No jumper needed.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = hal::config::Config::default();
    config.clock_cfg.enable_clk_16k = true;
    let p = hal::init(config);

    defmt::info!("LPTMR1 example");

    let lptmr = Lptmr::new(p.LPTMR1, Irqs, Config::default()).unwrap();

    const FRO_12M_HZ: u32 = 12_000_000;

    for round in 0..3u32 {
        let ticks = FRO_12M_HZ / 4;
        let start = Instant::now();
        lptmr.wait_for_timeout(ticks).await;
        let elapsed = Instant::now() - start;

        defmt::info!("[{}] {} ticks elapsed in {} ms", round, ticks, elapsed.as_millis());
        defmt::assert!(
            elapsed.as_millis() >= 200 && elapsed.as_millis() <= 300,
            "timeout duration inconsistent with fro_12m @ 12 MHz"
        );
    }

    // fro_16k @ ~16384 Hz: ~250ms is ~4096 ticks.
    let lptmr16k = Lptmr::new(
        p.LPTMR0,
        Irqs16k,
        Config {
            clock: LptmrClockConfig {
                source: LptmrClockSource::Fro16k,
            },
        },
    )
    .unwrap();
    let start = Instant::now();
    lptmr16k.wait_for_timeout(4096).await;
    let elapsed = Instant::now() - start;
    defmt::info!("fro_16k: 4096 ticks elapsed in {} ms", elapsed.as_millis());
    defmt::assert!(
        elapsed.as_millis() >= 200 && elapsed.as_millis() <= 300,
        "timeout duration inconsistent with fro_16k @ ~16384 Hz"
    );

    defmt::info!("LPTMR1 example passed");

    loop {
        embassy_time::Timer::after_secs(1).await;
        defmt::info!("idle");
    }
}
