#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn as hal;
use embassy_time::{Duration, Timer};
use hal::reset_reason::reset_reason;
use hal::wwdt::{Config, InterruptHandler, Watchdog};
use panic_probe as _;

hal::bind_interrupts!(struct Irqs {
    WWDT0 => InterruptHandler<hal::peripherals::WWDT0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(hal::config::Config::default());

    // Reading this latches and clears CMC0.SRS, so it reflects why *this* boot
    // happened. On the second pass it is the watchdog.
    let reason = reset_reason();

    if reason.is_watchdog0() {
        defmt::info!("booted from a WWDT0 reset - the watchdog fired as intended");
        defmt::info!("watchdog test passed");
        loop {
            defmt::info!("idle");
            Timer::after(Duration::from_secs(1)).await;
        }
    }

    defmt::info!("watchdog example (reset reason: {})", reason);

    let config = Config {
        timeout: Duration::from_millis(500),
        ..Default::default()
    };

    let mut wdt = Watchdog::new(p.WWDT0, Irqs, config).unwrap();
    wdt.start();
    defmt::info!("watchdog started, 500 ms timeout");

    // Feeding well inside the window must keep the system alive.
    for i in 1..=10 {
        Timer::after(Duration::from_millis(100)).await;
        wdt.feed();
        defmt::info!("fed {} of 10", i);
    }
    defmt::info!("survived 1 s of feeding");

    // Now stop feeding. The watchdog should reset the chip within ~500 ms, and
    // the next boot takes the branch at the top.
    defmt::info!("no longer feeding; expecting a reset in ~500 ms");
    loop {
        Timer::after(Duration::from_millis(100)).await;
    }
}
