#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_mcxn as hal;
use embassy_time::{Duration, Timer};
use hal::bind_interrupts;
use hal::clocks::config::Div;
use hal::clocks::{Lpuart5ClockConfig, Lpuart5ClockSource};
use hal::dma::DmaChannel;
use hal::lpuart::{Config, InterruptHandler, Uart};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    LP_FLEXCOMM5 => InterruptHandler<hal::peripherals::LPUART5>;
});

/// As uart_loopback.rs, but TX and RX move through DMA instead of
/// interrupt-driven byte-at-a-time transfers.
///
/// Same jumper as uart_loopback.rs: P1_16 (RXD) to P1_17 (TXD) on LPUART5.
/// `write_dma`/`read_dma` each take a separately-owned `DmaChannel`, so - like
/// the plain `write`/`read` - the two directions can still run concurrently
/// via `join`.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(hal::config::Config::default());

    defmt::info!("LPUART5 DMA loopback example");

    let clock = Lpuart5ClockConfig {
        source: Lpuart5ClockSource::Fro12m,
        div: Div::no_div(),
    };

    let uart = Uart::new(p.LPUART5, p.P1_17, p.P1_16, &clock, Config::default(), Irqs).unwrap();

    defmt::info!("LPUART5 functional clock {} Hz", uart.frequency());
    defmt::assert_eq!(uart.frequency(), 12_000_000);

    let mut tx_dma = DmaChannel::new(p.DMA0_CH0);
    let mut rx_dma = DmaChannel::new(p.DMA0_CH1);

    const MESSAGE: &[u8] = b"the quick brown fox jumps over the lazy dog";

    for round in 0..5u32 {
        let mut rx = [0u8; MESSAGE.len()];

        let (write, read) = join(
            uart.write_dma(&mut tx_dma, MESSAGE),
            uart.read_dma(&mut rx_dma, &mut rx),
        )
        .await;
        write.unwrap();
        read.unwrap();

        defmt::assert_eq!(&rx[..], MESSAGE, "[{}] DMA loopback data mismatch", round);
        defmt::info!("[{}] round ok", round);

        Timer::after(Duration::from_millis(50)).await;
    }

    defmt::info!("LPUART5 DMA loopback passed");

    loop {
        Timer::after(Duration::from_secs(1)).await;
        defmt::info!("idle");
    }
}
