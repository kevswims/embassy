#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn as hal;
use embassy_time::Timer;
use hal::adc::{Adc, Config, InterruptHandler};
use hal::bind_interrupts;
use hal::clocks::config::Div;
use hal::clocks::{Adc0ClockConfig, Adc0ClockSource};
use hal::vref::Vref;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    ADC0 => InterruptHandler<hal::peripherals::ADC0>;
});

/// Channel 28 is `VREF BG+`, an internal bandgap reference (RM Table 358),
/// so it reads without any external wiring. Channel 2 (`ADC0_A2`, pin
/// `P4_23`) expects an external source - with 1.0 V applied it reads about
/// `1.0 / 3.3 * 65536` = ~19860.
///
/// Note the [`Vref`] below: the ADC cannot convert without it, even though
/// this example references `VDD_ANA` rather than `VREFO`. See the
/// [`hal::vref`] module docs.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(hal::config::Config::default());

    defmt::info!("ADC0 example");

    // Supplies the ADC's bias current - must outlive the `Adc`.
    let _vref = Vref::new_bandgap_only(p.VREF0);

    let clock = Adc0ClockConfig {
        source: Adc0ClockSource::FroHf48m,
        div: Div::no_div(),
    };
    let adc = Adc::new(p.ADC0, Irqs, Config::default(), &clock).unwrap();

    for round in 0..5u32 {
        let value = adc.read_channel(28).await;
        defmt::info!("[{}] channel 28 (VREF BG+) = {}", round, value);
        Timer::after_millis(100).await;
    }

    for round in 0..5u32 {
        let value = adc.read_channel(2).await;
        defmt::info!("[{}] channel 2 (ADC0_A2, expect ~19860 at 1.0V) = {}", round, value);
        Timer::after_millis(100).await;
    }

    defmt::info!("ADC0 example passed");

    loop {
        Timer::after_secs(1).await;
        defmt::info!("idle");
    }
}
