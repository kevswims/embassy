#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn as hal;
use embassy_time::Instant;
use hal::bind_interrupts;
use hal::lptmr::{Config, InterruptHandler, Lptmr};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    LPTMR0 => InterruptHandler<hal::peripherals::LPTMR0>;
});

/// No jumper needed - LPTMR0 has no pins in this mode. Times a known number
/// of `fro_12m` cycles and checks the elapsed wall-clock time matches 12 MHz
/// - this is the empirical confirmation that PSR.PCS's Pcs00 really is
/// fro_12m (see lptmr.rs's module docs; not RM-confirmed otherwise).
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(hal::config::Config::default());

    defmt::info!("LPTMR0 example");

    let lptmr = Lptmr::new(p.LPTMR0, Irqs, Config::default()).unwrap();

    const FRO_12M_HZ: u32 = 12_000_000;

    for round in 0..3u32 {
        let ticks = FRO_12M_HZ / 4; // 250 ms, if Pcs00 really is fro_12m.
        let start = Instant::now();
        lptmr.wait_for_timeout(ticks).await;
        let elapsed = Instant::now() - start;

        defmt::info!("[{}] {} ticks elapsed in {} ms", round, ticks, elapsed.as_millis());
        defmt::assert!(
            elapsed.as_millis() >= 200 && elapsed.as_millis() <= 300,
            "timeout duration inconsistent with fro_12m @ 12 MHz - Pcs00 guess may be wrong"
        );
    }

    defmt::info!("LPTMR0 example passed");

    loop {
        embassy_time::Timer::after_secs(1).await;
        defmt::info!("idle");
    }
}
