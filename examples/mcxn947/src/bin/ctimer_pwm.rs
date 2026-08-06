#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn as hal;
use embassy_time::{Duration, Timer};
use hal::bind_interrupts;
use hal::clocks::{Ctimer0ClockConfig, Ctimer1ClockConfig};
use hal::ctimer::capture::{Capture, Edge};
use hal::ctimer::pwm::SinglePwm;
use hal::pac;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    CTIMER1 => hal::ctimer::capture::InterruptHandler<hal::peripherals::CTIMER1>;
});

/// Drive a PWM output on CTIMER0 and measure its duty cycle with a capture
/// channel on CTIMER1.
///
/// PWM and capture can't share one timer instance here: both `SinglePwm::new`
/// and `Capture::new` bring up their timer's functional clock and counter
/// from scratch, so calling both on the same instance would have the second
/// one reset what the first just configured. Capture's INPUTMUX0 routing is a
/// full crossbar regardless - any `CT_INPn` pad reaches any timer's capture
/// channel - so putting them on separate timers costs nothing but a second
/// timer instance.
///
/// CTIMER0_CH2 is the PWM period channel (drives no pin); CTIMER0_CH0 is the
/// duty channel, driving P0_10 (the red LED, `CT0_MAT0`). CTIMER1_CH1
/// captures P1_8 (`CT_INP8`) - jumper P0_10 to P1_8 before running this.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(hal::config::Config::default());

    defmt::info!("CTIMER0 PWM example (CTIMER1 capture verifies the duty cycle)");

    // 1 kHz PWM at the 1 MHz functional clock.
    const PERIOD: u32 = 1000;
    const DUTY_PERCENT: u32 = 30;

    let mut pwm = SinglePwm::new(
        p.CTIMER0,
        p.CTIMER0_CH2,
        p.CTIMER0_CH0,
        p.P0_10,
        &Ctimer0ClockConfig::default(),
        PERIOD,
        0,
    )
    .unwrap();

    defmt::info!(
        "CTIMER0 functional clock {} Hz, max_duty {}",
        pwm.frequency(),
        pwm.max_duty()
    );
    defmt::assert_eq!(pwm.frequency(), 1_000_000);

    let duty_ticks = pwm.max_duty() * DUTY_PERCENT / 100;
    pwm.set_duty(duty_ticks);
    pwm.start();

    let mut capture = Capture::new(
        p.CTIMER1,
        p.CTIMER1_CH1,
        p.P1_8,
        Edge::Both,
        &Ctimer1ClockConfig::default(),
        Irqs,
    )
    .unwrap();

    // One warm-up edge outside the measurement window - see ctimer_capture.rs.
    let mut last_ticks = capture.capture().await;
    let mut last_level = pad_high();

    let mut high_ticks = 0u64;
    let mut low_ticks = 0u64;
    let mut high_intervals = 0u32;
    let mut low_intervals = 0u32;

    for _ in 0..40 {
        let ticks = capture.capture().await;
        let level = pad_high();
        let delta = ticks.wrapping_sub(last_ticks) as u64;

        // The interval that just ended is classified by the level it was
        // *at* - the level before this edge - which is `last_level`, not the
        // level the pad just transitioned to.
        if last_level {
            high_ticks += delta;
            high_intervals += 1;
        } else {
            low_ticks += delta;
            low_intervals += 1;
        }

        last_ticks = ticks;
        last_level = level;
    }

    let avg_high = (high_ticks / u64::from(high_intervals.max(1))) as u32;
    let avg_low = (low_ticks / u64::from(low_intervals.max(1))) as u32;
    defmt::info!(
        "measured {} high-intervals averaging {} ticks, {} low-intervals averaging {} ticks (requested {}/{})",
        high_intervals,
        avg_high,
        low_intervals,
        avg_low,
        duty_ticks,
        PERIOD
    );

    defmt::assert!(
        avg_high.abs_diff(duty_ticks) < PERIOD / 10,
        "measured high time too far from the requested duty cycle"
    );
    defmt::assert!(
        avg_low.abs_diff(PERIOD - duty_ticks) < PERIOD / 10,
        "measured low time too far from the requested duty cycle"
    );
    defmt::info!("P0_10's measured duty cycle matches SinglePwm::set_duty");

    loop {
        Timer::after(Duration::from_secs(1)).await;
        defmt::info!("idle");
    }
}

/// Sample P1_8's physical level through GPIO1's input register.
fn pad_high() -> bool {
    pac::GPIO1.pdir().read().0 & (1 << 8) != 0
}
