#![no_std]
#![no_main]

//! Blocking FlexSPI self-test against the FRDM-MCXN947's on-board W25Q64.
//!
//! Reads the JEDEC identification, then erases, programs and verifies a span of
//! sectors. The identification alone proves the clock is routed, the pads are
//! muxed and the LUT uploaded, so a failure there is the first thing to look at:
//! `0x00` or `0xff` usually means `FlexSPICLKSEL` is still on "no clock" or
//! `FlexSPICLKDIV.HALT` is still set.

use defmt::{info, unwrap};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn as hal;
use embassy_time::Timer;
use hal::config::Config;
use hal::flexspi::{Flexspi, NorFlash};
use panic_probe as _;

#[path = "../flexspi_common.rs"]
mod flexspi_common;

use flexspi_common::{FLASH_CONFIG, blocking_self_test, flexspi_clock};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(Config::default());

    info!("FlexSPI blocking self-test");

    let flexspi = unwrap!(Flexspi::new_blocking(
        p.FLEXSPI0,
        p.P3_0,
        p.P3_7,
        p.P3_6,
        p.P3_8,
        p.P3_9,
        p.P3_10,
        p.P3_11,
        &flexspi_clock(),
        FLASH_CONFIG,
    ));

    let mut flash = NorFlash::new(flexspi);

    blocking_self_test(&mut flash);

    info!("FlexSPI blocking self-test PASSED");

    loop {
        Timer::after_secs(1).await;
        info!("FlexSPI demo heartbeat");
    }
}
