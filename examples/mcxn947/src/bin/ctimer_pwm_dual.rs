#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn as hal;
use embassy_time::{Duration, Timer};
use hal::bind_interrupts;
use hal::clocks::{Ctimer0ClockConfig, Ctimer1ClockConfig};
use hal::ctimer::capture::{Capture, Edge};
use hal::ctimer::pwm::DualPwm;
use hal::pac;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    CTIMER1 => hal::ctimer::capture::InterruptHandler<hal::peripherals::CTIMER1>;
});

/// As ctimer_pwm.rs, but both DualPwm outputs are independently measured.
///
/// CTIMER0_CH2 is the shared period channel; CTIMER0_CH0 drives P0_10 (red
/// LED, `CT0_MAT0`) and CTIMER0_CH3 drives P0_27 (green LED, `CT0_MAT3`), at
/// independent duty cycles on the same 1 kHz period. CTIMER1_CH1 captures
/// P1_8 (`CT_INP8`, jumpered to P0_10) and CTIMER1_CH2 captures P1_9
/// (`CT_INP9`, jumpered to P0_27).
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(hal::config::Config::default());

    defmt::info!("CTIMER0 dual-PWM example (CTIMER1 verifies both duty cycles)");

    const PERIOD: u32 = 1000;
    const DUTY0_PERCENT: u32 = 30;
    const DUTY1_PERCENT: u32 = 70;

    let mut pwm = DualPwm::new(
        p.CTIMER0,
        p.CTIMER0_CH2,
        p.CTIMER0_CH0,
        p.P0_10,
        p.CTIMER0_CH3,
        p.P0_27,
        &Ctimer0ClockConfig::default(),
        PERIOD,
        0,
        0,
    )
    .unwrap();

    defmt::info!("CTIMER0 functional clock {} Hz, max_duty {}", pwm.frequency(), pwm.max_duty());
    defmt::assert_eq!(pwm.frequency(), 1_000_000);

    let duty0_ticks = pwm.max_duty() * DUTY0_PERCENT / 100;
    let duty1_ticks = pwm.max_duty() * DUTY1_PERCENT / 100;
    pwm.set_duty0(duty0_ticks);
    pwm.set_duty1(duty1_ticks);
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

    let high0 = measure_high_ticks(&mut capture0, || pad_high(8)).await;
    let high1 = measure_high_ticks(&mut capture1, || pad_high(9)).await;

    defmt::info!(
        "channel 0: measured {} ticks high (requested {}); channel 1: measured {} ticks high (requested {})",
        high0,
        duty0_ticks,
        high1,
        duty1_ticks
    );

    defmt::assert!(
        high0.abs_diff(duty0_ticks) < PERIOD / 10,
        "channel 0's measured high time too far from its requested duty cycle"
    );
    defmt::assert!(
        high1.abs_diff(duty1_ticks) < PERIOD / 10,
        "channel 1's measured high time too far from its requested duty cycle"
    );
    defmt::info!("both DualPwm channels measured correctly and independently");

    loop {
        Timer::after(Duration::from_secs(1)).await;
        defmt::info!("idle");
    }
}

/// Average ticks spent high over 20 cycles of `capture`, classifying each
/// interval by the pad level (from `level`) at its start - see ctimer_pwm.rs.
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
