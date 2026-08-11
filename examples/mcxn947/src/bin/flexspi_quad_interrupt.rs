#![no_std]
#![no_main]

//! Interrupt-driven FlexSPI self-test against the FRDM-MCXN947's W25Q64.
//!
//! The same erase/program/verify sequence as `flexspi_quad_polling.rs`, but every
//! command completion arrives as a `FLEXSPI0` interrupt rather than a spin on
//! `INTR.IPCMDDONE`, so the executor is free to run other tasks in between.
//! Payloads still move through the CPU; see `flexspi_quad_dma.rs` for the eDMA
//! variant.

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

    info!("FlexSPI interrupt async self-test");

    let flexspi = unwrap!(Flexspi::new_async(
        p.FLEXSPI0,
        p.P3_0,
        p.P3_7,
        p.P3_6,
        p.P3_8,
        p.P3_9,
        p.P3_10,
        p.P3_11,
        Irqs,
        &flexspi_clock(),
        FLASH_CONFIG,
    ));

    let mut flash = NorFlash::new(flexspi);

    async_self_test(&mut flash).await;

    info!("FlexSPI interrupt async self-test PASSED");

    loop {
        Timer::after_secs(1).await;
        info!("FlexSPI demo heartbeat");
    }
}
