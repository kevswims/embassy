#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn as hal;
use embassy_time::Instant;
use hal::bind_interrupts;
use hal::rtc::{Config, DateTime, InterruptHandler, Rtc, Weekday};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    RTC => InterruptHandler<hal::peripherals::RTC0>;
});

/// No jumper needed - RTC0 has no pins. Sets a known date/time, reads it
/// back, then arms the alarm a few seconds out and waits for it to fire.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = hal::config::Config::default();
    config.clock_cfg.enable_clk_16k = true;
    let p = hal::init(config);

    defmt::info!("RTC0 example");

    let rtc = Rtc::new(p.RTC0, Irqs, Config::default()).unwrap();

    let set = DateTime {
        year: 2026,
        month: 3,
        day: 5,
        dow: Weekday::Thursday,
        hour: 12,
        minute: 0,
        second: 0,
    };
    rtc.set_datetime(set).unwrap();

    let read_back = rtc.now();
    defmt::info!(
        "set {}-{}-{} {}:{}:{}, read back {}-{}-{} {}:{}:{}",
        set.year,
        set.month,
        set.day,
        set.hour,
        set.minute,
        set.second,
        read_back.year,
        read_back.month,
        read_back.day,
        read_back.hour,
        read_back.minute,
        read_back.second,
    );
    defmt::assert_eq!(read_back, set, "now() did not round-trip set_datetime()");

    const ALARM_IN_SECONDS: u8 = 3;
    let mut alarm = set;
    alarm.second += ALARM_IN_SECONDS;
    rtc.set_alarm(alarm).unwrap();

    defmt::info!("waiting for alarm in {} seconds", ALARM_IN_SECONDS);
    let start = Instant::now();
    rtc.wait_for_alarm().await;
    let elapsed = Instant::now() - start;

    defmt::info!("alarm fired after {} ms", elapsed.as_millis());
    // RTC0's counters only tick in whole seconds, and set_datetime/set_alarm
    // don't know the phase within the current tick, so up to one second of
    // slack either side of ALARM_IN_SECONDS is expected, not a bug.
    defmt::assert!(
        elapsed.as_millis() >= ((ALARM_IN_SECONDS as u64 - 1) * 1000),
        "alarm fired too early"
    );

    defmt::info!("RTC0 example passed");

    loop {
        embassy_time::Timer::after_secs(1).await;
        defmt::info!("idle");
    }
}
