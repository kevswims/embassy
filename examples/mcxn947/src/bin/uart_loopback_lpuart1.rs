#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_mcxn as hal;
use embassy_time::{Duration, Timer};
use hal::bind_interrupts;
use hal::clocks::config::Div;
use hal::clocks::{Lpuart1ClockConfig, Lpuart1ClockSource};
use hal::lpuart::{Config, InterruptHandler, Uart};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    LP_FLEXCOMM1 => InterruptHandler<hal::peripherals::LPUART1>;
});

/// As uart_loopback.rs, on a second instance - confirms the clock/gate
/// generalization from LPUART5-only to all 10 works for an arbitrary `n`,
/// not just by code inspection.
///
/// P0_24 is LPUART1's RXD (`LP_FLEXCOMM1.P0`, ALT2) and P0_25 is its TXD
/// (`LP_FLEXCOMM1.P1`, ALT2) - jumper the two together before running this.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(hal::config::Config::default());

    defmt::info!("LPUART1 loopback example");

    let clock = Lpuart1ClockConfig {
        source: Lpuart1ClockSource::Fro12m,
        div: Div::no_div(),
    };

    let uart = Uart::new(p.LPUART1, p.P0_25, p.P0_24, &clock, Config::default(), Irqs).unwrap();

    defmt::info!("LPUART1 functional clock {} Hz", uart.frequency());
    defmt::assert_eq!(uart.frequency(), 12_000_000);

    const MESSAGE: &[u8] = b"the quick brown fox jumps over the lazy dog";

    for round in 0..5u32 {
        let mut rx = [0u8; MESSAGE.len()];

        let (_, result) = join(uart.write(MESSAGE), uart.read(&mut rx)).await;
        result.unwrap();

        defmt::assert_eq!(&rx[..], MESSAGE, "[{}] loopback data mismatch", round);
        defmt::info!("[{}] round ok", round);

        Timer::after(Duration::from_millis(50)).await;
    }

    defmt::info!("LPUART1 loopback passed");

    loop {
        Timer::after(Duration::from_secs(1)).await;
        defmt::info!("idle");
    }
}
