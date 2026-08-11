#![no_std]
#![no_main]

//! eDMA-backed FlexSPI self-test against the FRDM-MCXN947's W25Q64.
//!
//! Identical to `flexspi_quad_interrupt.rs` except that read and program payloads
//! are moved between memory and the IP FIFO windows by two eDMA channels instead
//! of by the CPU. Transfers too short or too unaligned for the FIFO's 8-byte
//! window fall back to the CPU path automatically, which is what the odd read
//! lengths in the self-test exercise.

use defmt::{info, unwrap};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn as hal;
use embassy_time::Timer;
use hal::config::Config;
use hal::flexspi::{self, Flexspi, NorFlash};
use hal::{bind_interrupts, peripherals};
use panic_probe as _;

#[path = "../flexspi_common.rs"]
mod flexspi_common;

use flexspi_common::{FLASH_CONFIG, async_self_test, flexspi_clock};

bind_interrupts!(struct Irqs {
    FLEXSPI0 => flexspi::InterruptHandler<peripherals::FLEXSPI0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(Config::default());

    info!("FlexSPI eDMA self-test");

    let flexspi = unwrap!(Flexspi::new_with_dma(
        p.FLEXSPI0,
        p.P3_0,
        p.P3_7,
        p.P3_6,
        p.P3_8,
        p.P3_9,
        p.P3_10,
        p.P3_11,
        p.DMA0_CH0,
        p.DMA0_CH1,
        Irqs,
        &flexspi_clock(),
        FLASH_CONFIG,
    ));

    let mut flash = NorFlash::new(flexspi);

    async_self_test(&mut flash).await;

    info!("FlexSPI eDMA self-test PASSED");

    loop {
        Timer::after_secs(1).await;
        info!("FlexSPI demo heartbeat");
    }
}
