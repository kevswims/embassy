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

const RING_LEN: usize = 16;
const BURSTS: u32 = 10;
const BURST_LEN: u32 = 8;

/// Continuous DMA reception via `Uart::into_ring_rx`, verified across
/// multiple wraps of a ring buffer smaller than the total data sent.
///
/// Same jumper as uart_loopback.rs: P1_16 (RXD) to P1_17 (TXD) on LPUART5. TX
/// stays interrupt-driven (`write`), to isolate what's actually being tested
/// - RX draining a wrapping DMA buffer while more data keeps arriving
/// concurrently, not anything about the TX path.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(hal::config::Config::default());

    defmt::info!("LPUART5 ring-buffered RX example");

    let clock = Lpuart5ClockConfig {
        source: Lpuart5ClockSource::Fro12m,
        div: Div::no_div(),
    };

    let uart = Uart::new(p.LPUART5, p.P1_17, p.P1_16, &clock, Config::default(), Irqs).unwrap();

    defmt::info!("LPUART5 functional clock {} Hz", uart.frequency());

    // BURSTS * BURST_LEN bytes total, forcing several full wraps of a
    // RING_LEN-byte ring.
    let mut rx_dma = DmaChannel::new(p.DMA0_CH1);
    let mut ring_storage = [0u8; RING_LEN];
    let mut rx = uart.into_ring_rx(&mut rx_dma, &mut ring_storage).unwrap();

    let tx = async {
        for burst in 0..BURSTS {
            let chunk: [u8; BURST_LEN as usize] = core::array::from_fn(|i| (burst * BURST_LEN + i as u32) as u8);
            uart.write(&chunk).await;
            Timer::after(Duration::from_millis(20)).await;
        }
    };

    let verify = async {
        let mut expected: u8 = 0;
        let mut total_received: u32 = 0;
        let mut buf = [0u8; 5];

        while total_received < BURSTS * BURST_LEN {
            let n = rx.read(&mut buf).await.unwrap();
            for &b in &buf[..n] {
                defmt::assert_eq!(b, expected, "byte {} out of sequence", total_received);
                expected = expected.wrapping_add(1);
            }
            total_received += n as u32;
        }
    };

    join(tx, verify).await;

    defmt::info!(
        "received {} bytes across {} wraps of a {}-byte ring, all in sequence",
        BURSTS * BURST_LEN,
        (BURSTS * BURST_LEN) / RING_LEN as u32,
        RING_LEN
    );
    defmt::info!("LPUART5 ring RX passed");

    loop {
        Timer::after(Duration::from_secs(1)).await;
        defmt::info!("idle");
    }
}
