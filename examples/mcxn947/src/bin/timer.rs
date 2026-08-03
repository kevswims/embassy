#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn as hal;
use embassy_time::{Duration, Instant, Timer};
use hal::gpio::{DriveStrength, Level, Output, SlewRate};
use panic_probe as _;

/// Blink on a timer, concurrently with the main task, to show the driver
/// servicing more than one waker.
#[embassy_executor::task]
async fn blink(mut led: Output<'static, hal::peripherals::P0_10>) {
    loop {
        led.toggle();
        Timer::after(Duration::from_millis(250)).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = hal::init(hal::config::Config::default());

    defmt::info!("embassy-time on OSTIMER");

    let led = Output::new(p.P0_10, Level::High, DriveStrength::Normal, SlewRate::Fast);
    spawner.spawn(blink(led).unwrap());

    // A tick is a microsecond, so `Instant` deltas are directly comparable
    // against the requested duration. Anything far off means the OSTIMER
    // functional clock is not the 1 MHz the driver assumes.
    for ms in [1_u64, 10, 100, 1000] {
        let start = Instant::now();
        Timer::after(Duration::from_millis(ms)).await;
        let elapsed = start.elapsed();

        defmt::info!(
            "requested {} ms, slept {} us",
            ms,
            elapsed.as_micros(),
        );

        // Sleeps are allowed to overshoot but never to finish early.
        let requested_us = ms * 1000;
        defmt::assert!(
            elapsed.as_micros() >= requested_us,
            "Timer returned early"
        );
        // Generous upper bound: the 1 ms case is dominated by fixed overhead.
        defmt::assert!(
            elapsed.as_micros() <= requested_us + 1000,
            "Timer overslept badly"
        );
    }

    defmt::info!("all timer checks passed");

    let mut ticks = 0u32;
    loop {
        Timer::after(Duration::from_secs(1)).await;
        ticks += 1;
        defmt::info!("tick {} at {} ms", ticks, Instant::now().as_millis());
    }
}
