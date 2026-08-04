#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn as hal;
use panic_probe as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Exercises clocks::init() (verifies the reset-default main clock) and
    // enables the PORT0-4/GPIO0-4 clock gates.
    let _p = hal::init(hal::config::Config::default());

    defmt::info!("Hello, world!");

    hal::clocks::with_clocks(|c| {
        defmt::info!("main_clk = {} Hz, system_clk = {} Hz", c.main_clk_hz, c.system_clk_hz);
    });

    // Why the MCU last came up. Reading this latches and clears CMC0.SRS, so
    // the value is cached for the rest of the boot and repeat calls are cheap.
    let reason = hal::reset_reason::reset_reason();
    defmt::info!("reset reason: {}", reason);
    for r in reason {
        defmt::info!("  - {}", r);
    }

    loop {
        defmt::info!("tick");
        cortex_m::asm::delay(48_000_000);
    }
}
