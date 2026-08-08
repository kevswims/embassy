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
use hal::pac;
use hal::vref::{BufferPower, Vref};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    ADC0 => InterruptHandler<hal::peripherals::ADC0>;
});

/// Confirms that `VREF_OUT` is only actually driven in buffered mode.
///
/// Per RM: `Csr[LPBGEN]` alone enables the low-power bandgap bias current
/// for ADC use, while `Csr[LPBGEN]` + `Csr[HCBGEN]` enables the VREF supply
/// voltage on `VREF_OUT`. Channel 28 (`VREF BG+`) measures that output, so
/// it should read far below nominal in bandgap-only mode and rise to a
/// sensible reference voltage here.
///
/// Channel 2 (`ADC0_A2`, 1.0 V external source) is read alongside as a
/// control - it should be unaffected, since it never depended on `VREF_OUT`.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(hal::config::Config::default());

    defmt::info!("VREF buffered-mode probe");

    let mut vref = Vref::new_buffered(p.VREF0, BufferPower::High);
    defmt::info!(
        "VREF0.CSR = {:#010x}, UTRIM = {:#010x}",
        pac::VREF0.csr().read().0,
        pac::VREF0.utrim().read().0
    );

    let clock = Adc0ClockConfig {
        source: Adc0ClockSource::FroHf48m,
        div: Div::no_div(),
    };
    let adc = Adc::new(p.ADC0, Irqs, Config::default(), &clock).unwrap();

    // Sweep TRIM2V1 across the range the SDK's vref_trim2V1_adc example
    // uses (0..0xC) and watch both channels. NOTE: that example measures
    // VREF_OUT on channel *2* on this board, not 28 - so ch2 here is
    // whatever is wired to P4_23 (currently an external 1V source), and
    // ch28 is a separate internal node whose relationship to VREF_OUT is
    // what we are trying to establish.
    for trim in 0..0xCu8 {
        vref.set_trim_2v1(trim);
        let bg = adc.read_channel(28).await;
        let ch2 = adc.read_channel(2).await;
        defmt::info!(
            "trim={:#03x} ch28 = {} ({} mV) | ch2 = {} ({} mV)",
            trim,
            bg,
            (bg as u32 * 3300) / 65536,
            ch2,
            (ch2 as u32 * 3300) / 65536
        );
        Timer::after_millis(50).await;
    }

    defmt::info!("probe done");

    loop {
        Timer::after_secs(1).await;
        defmt::info!("idle");
    }
}
