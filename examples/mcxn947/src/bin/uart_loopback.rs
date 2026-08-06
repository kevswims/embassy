#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_mcxn as hal;
use embassy_time::{Duration, Timer};
use hal::bind_interrupts;
use hal::clocks::{Lpuart5ClockConfig, Lpuart5ClockSource};
use hal::clocks::config::Div;
use hal::lpuart::{Config, InterruptHandler, Uart};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    LP_FLEXCOMM5 => InterruptHandler<hal::peripherals::LPUART5>;
});

/// Bring up LPUART5 and confirm it with a physical loopback.
///
/// P1_16 is LPUART5's RXD (`LP_FLEXCOMM5.P0`, ALT2) and P1_17 is its TXD
/// (`LP_FLEXCOMM5.P1`, ALT2) - jumper the two together before running this.
/// RM 66.2.3's pad-letter mapping is backwards from the intuitive reading:
/// the low-numbered pad is the *input*.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(hal::config::Config::default());

    defmt::info!("LPUART5 loopback example");

    let clock = Lpuart5ClockConfig {
        source: Lpuart5ClockSource::Fro12m,
        div: Div::no_div(),
    };

    let uart = Uart::new(p.LPUART5, p.P1_17, p.P1_16, &clock, Config::default(), Irqs).unwrap();

    defmt::info!("LPUART5 functional clock {} Hz", uart.frequency());
    defmt::assert_eq!(uart.frequency(), 12_000_000);

    const MESSAGE: &[u8] = b"the quick brown fox jumps over the lazy dog";

    for round in 0..5u32 {
        let mut rx = [0u8; MESSAGE.len()];

        // MESSAGE is longer than the 8-word FIFO, so write and read must run
        // concurrently - a strictly sequential write-then-read would overrun
        // RX long before `read()` ever got a chance to start draining it.
        let (_, result) = join(uart.write(MESSAGE), uart.read(&mut rx)).await;
        result.unwrap();

        defmt::assert_eq!(&rx[..], MESSAGE, "[{}] loopback data mismatch", round);
        defmt::info!("[{}] round ok", round);

        Timer::after(Duration::from_millis(50)).await;
    }

    defmt::info!("LPUART5 loopback passed");

    loop {
        Timer::after(Duration::from_secs(1)).await;
        defmt::info!("idle");
    }
}
