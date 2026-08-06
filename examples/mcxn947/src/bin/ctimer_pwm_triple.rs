#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn as hal;
use embassy_time::{Duration, Timer};
use hal::bind_interrupts;
use hal::clocks::{Ctimer0ClockConfig, Ctimer1ClockConfig};
use hal::ctimer::capture::{Capture, Edge};
use hal::ctimer::pwm::TriplePwm;
use hal::pac;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    CTIMER1 => hal::ctimer::capture::InterruptHandler<hal::peripherals::CTIMER1>;
});

/// As ctimer_pwm_dual.rs, with a third independent duty channel.
///
/// This uses every one of CTIMER0's four match channels: CTIMER0_CH2 is the
/// shared period channel, and CTIMER0_CH0/CH3/CH1 are the three duty
/// channels, driving P0_10 (red LED, `CT0_MAT0`), P0_27 (green LED,
/// `CT0_MAT3`) and P0_11 (`CT0_MAT1`) respectively. CTIMER1_CH1/CH2/CH3
/// capture P1_8 (`CT_INP8`), P1_9 (`CT_INP9`) and P1_14 (`CT_INP10`) -
/// jumper P0_10-P1_8, P0_27-P1_9 and P0_11-P1_14 before running this.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(hal::config::Config::default());

    defmt::info!("CTIMER0 triple-PWM example (CTIMER1 verifies all three duty cycles)");

    const PERIOD: u32 = 1000;
    const DUTY0_PERCENT: u32 = 20;
    const DUTY1_PERCENT: u32 = 50;
    const DUTY2_PERCENT: u32 = 80;

    let mut pwm = TriplePwm::new(
        p.CTIMER0,
        p.CTIMER0_CH2,
        p.CTIMER0_CH0,
        p.P0_10,
        p.CTIMER0_CH3,
        p.P0_27,
        p.CTIMER0_CH1,
        p.P0_11,
        &Ctimer0ClockConfig::default(),
        PERIOD,
        0,
        0,
        0,
    )
    .unwrap();

    defmt::info!("CTIMER0 functional clock {} Hz, max_duty {}", pwm.frequency(), pwm.max_duty());
    defmt::assert_eq!(pwm.frequency(), 1_000_000);

    let duty0_ticks = pwm.max_duty() * DUTY0_PERCENT / 100;
    let duty1_ticks = pwm.max_duty() * DUTY1_PERCENT / 100;
    let duty2_ticks = pwm.max_duty() * DUTY2_PERCENT / 100;
    pwm.set_duty0(duty0_ticks);
    pwm.set_duty1(duty1_ticks);
    pwm.set_duty2(duty2_ticks);
    pwm.start();

    let mut capture0 = Capture::new(
        p.CTIMER1,
        p.CTIMER1_CH1,
        p.P1_8,
        Edge::Both,
        &Ctimer1ClockConfig::default(),
        Irqs,
    )
    .unwrap();
    let mut capture1 = Capture::new_channel(p.CTIMER1_CH2, p.P1_9, Edge::Both, Irqs);
    let mut capture2 = Capture::new_channel(p.CTIMER1_CH3, p.P1_14, Edge::Both, Irqs);

    let high0 = measure_high_ticks(&mut capture0, || pad_high(8)).await;
    let high1 = measure_high_ticks(&mut capture1, || pad_high(9)).await;
    let high2 = measure_high_ticks(&mut capture2, || pad_high(14)).await;

    defmt::info!(
        "measured high ticks: ch0={} (want {}) ch1={} (want {}) ch2={} (want {})",
        high0,
        duty0_ticks,
        high1,
        duty1_ticks,
        high2,
        duty2_ticks
    );

    defmt::assert!(high0.abs_diff(duty0_ticks) < PERIOD / 10, "channel 0 duty mismatch");
    defmt::assert!(high1.abs_diff(duty1_ticks) < PERIOD / 10, "channel 1 duty mismatch");
    defmt::assert!(high2.abs_diff(duty2_ticks) < PERIOD / 10, "channel 2 duty mismatch");
    defmt::info!("all three TriplePwm channels measured correctly and independently");

    loop {
        Timer::after(Duration::from_secs(1)).await;
        defmt::info!("idle");
    }
}

/// As ctimer_pwm_dual.rs's helper of the same name.
async fn measure_high_ticks<T, Ch, P>(capture: &mut hal::ctimer::capture::Capture<'_, T, Ch, P>, level: impl Fn() -> bool) -> u32
where
    T: hal::ctimer::Instance,
    Ch: hal::PeripheralType,
    P: hal::PeripheralType,
{
    let mut last_ticks = capture.capture().await;
    let mut last_level = level();

    let mut high_ticks = 0u64;
    let mut high_intervals = 0u32;

    for _ in 0..40 {
        let ticks = capture.capture().await;
        let this_level = level();
        let delta = ticks.wrapping_sub(last_ticks) as u64;

        if last_level {
            high_ticks += delta;
            high_intervals += 1;
        }

        last_ticks = ticks;
        last_level = this_level;
    }

    (high_ticks / u64::from(high_intervals.max(1))) as u32
}

/// Sample GPIO1 bit `n`'s physical level.
fn pad_high(n: u8) -> bool {
    pac::GPIO1.pdir().read().0 & (1 << n) != 0
}
