//! Reads the supply rails through the ADCs' internal monitor channels, which
//! need no external wiring (RM Table 358):
//!
//! | Channel | ADC0         | ADC1          |
//! | ------- | ------------ | ------------- |
//! | 25      | `VDD_CORE/4` | `VDD_SYS/4`   |
//! | 29      | `VBAT/4`     | `VDD_DCDC/4`  |
//!
//! The ADC is referenced to `VDD_ANA`, taken here as 3.3 V as on the
//! FRDM-MCXN947, so the result is only as accurate as that supply. The monitor
//! channels are high impedance and read low at the default sample time, so this
//! uses the longest.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn as hal;
use embassy_time::Timer;
use hal::adc::{Adc, Config, InterruptHandler, SampleTime};
use hal::bind_interrupts;
use hal::clocks::config::Div;
use hal::clocks::{Adc0ClockConfig, Adc0ClockSource, Adc1ClockConfig, Adc1ClockSource};
use hal::vref::Vref;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    ADC0 => InterruptHandler<hal::peripherals::ADC0>;
    ADC1 => InterruptHandler<hal::peripherals::ADC1>;
});

/// `VDD_ANA`, the ADC reference.
const VDD_ANA_MV: u32 = 3300;
const CORE_OR_SYS_CHANNEL: u8 = 25;
const VBAT_OR_DCDC_CHANNEL: u8 = 29;
/// The monitor channels sense a quarter of the rail.
const DIVIDER: u32 = 4;
/// Conversions averaged per reading.
const SAMPLES: u32 = 16;

/// Average `SAMPLES` conversions of `channel` and convert to the rail voltage.
async fn rail_mv<T: hal::adc::Instance>(adc: &Adc<'_, T>, channel: u8) -> u32 {
    // The first conversions after power-up read low while the sampling
    // capacitor settles.
    for _ in 0..4 {
        let _ = adc.read_channel(channel).await;
    }
    let mut sum = 0u32;
    for _ in 0..SAMPLES {
        sum += adc.read_channel(channel).await as u32;
    }
    (sum / SAMPLES) * VDD_ANA_MV * DIVIDER / 65536
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(hal::config::Config::default());

    defmt::info!("ADC supply-rail monitor");

    // Supplies the ADC's bias current - must outlive the `Adc`s.
    let _vref = Vref::new_bandgap_only(p.VREF0);

    // `fro_hf` follows `FIRCCFG.RANGE`; keep ADCK within 6-48 MHz.
    const ADCK_MAX_HZ: u32 = 48_000_000;
    let fro_hf = hal::clocks::with_clocks(|c| c.fro_hf).unwrap();
    let div = Div::from_divisor(fro_hf.div_ceil(ADCK_MAX_HZ)).unwrap();

    let mut config = Config::default();
    config.sample_time = SampleTime::Adck131_5;

    let adc0 = Adc::new(
        p.ADC0,
        Irqs,
        config,
        &Adc0ClockConfig {
            source: Adc0ClockSource::FroHf,
            div,
        },
    )
    .unwrap();
    let adc1 = Adc::new(
        p.ADC1,
        Irqs,
        config,
        &Adc1ClockConfig {
            source: Adc1ClockSource::FroHf,
            div,
        },
    )
    .unwrap();

    loop {
        let core = rail_mv(&adc0, CORE_OR_SYS_CHANNEL).await;
        let vbat = rail_mv(&adc0, VBAT_OR_DCDC_CHANNEL).await;
        let sys = rail_mv(&adc1, CORE_OR_SYS_CHANNEL).await;
        let dcdc = rail_mv(&adc1, VBAT_OR_DCDC_CHANNEL).await;
        defmt::info!(
            "VDD_CORE {} mV, VBAT {} mV, VDD_SYS {} mV, VDD_DCDC {} mV",
            core,
            vbat,
            sys,
            dcdc
        );
        Timer::after_secs(1).await;
    }
}
