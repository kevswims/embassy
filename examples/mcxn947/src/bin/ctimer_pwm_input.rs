#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn as hal;
use embassy_time::{Duration, Timer};
use hal::bind_interrupts;
use hal::clocks::{Ctimer0ClockConfig, Ctimer1ClockConfig};
use hal::ctimer::capture::PwmInput;
use hal::ctimer::pwm::SinglePwm;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    CTIMER1 => hal::ctimer::capture::InterruptHandler<hal::peripherals::CTIMER1>;
});

/// Measure an external PWM signal's period and duty cycle with `PwmInput`,
/// entirely from interrupt-driven capture edges - no polling the pin's level.
///
/// Reuses ctimer_pwm.rs's setup as the signal source: CTIMER0 drives P0_10
/// (red LED, `CT0_MAT0`) at a known period and duty. CTIMER1's PwmInput
/// watches P1_8 (`CT_INP8`, the same jumper as ctimer_pwm.rs) with two
/// channels - CTIMER1_CH1 for rising edges, CTIMER1_CH2 for falling - and
/// reports back the period and duty it measures, which should match what
/// CTIMER0 was actually configured to produce.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(hal::config::Config::default());

    defmt::info!("PwmInput example");

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

    let duty_ticks = pwm.max_duty() * DUTY_PERCENT / 100;
    pwm.set_duty(duty_ticks);
    pwm.start();

    let mut input = PwmInput::new(
        p.CTIMER1,
        p.CTIMER1_CH1,
        p.CTIMER1_CH2,
        p.P1_8,
        &Ctimer1ClockConfig::default(),
        Irqs,
    )
    .unwrap();

    defmt::info!("PwmInput functional clock {} Hz", input.frequency());
    defmt::assert_eq!(input.frequency(), 1_000_000);

    for i in 0..10 {
        let m = input.measure().await;
        defmt::info!(
            "[{}] measured period={} high_ticks={} (source: period={} high_ticks={})",
            i,
            m.period,
            m.high_ticks,
            PERIOD,
            duty_ticks
        );

        if i > 0 {
            defmt::assert!(m.period.abs_diff(PERIOD) < PERIOD / 20, "measured period too far off");
            defmt::assert!(
                m.high_ticks.abs_diff(duty_ticks) < PERIOD / 20,
                "measured duty too far off"
            );
        }
    }

    defmt::info!("PwmInput measured the source signal's period and duty cycle correctly");

    loop {
        Timer::after(Duration::from_secs(1)).await;
        defmt::info!("idle");
    }
}
