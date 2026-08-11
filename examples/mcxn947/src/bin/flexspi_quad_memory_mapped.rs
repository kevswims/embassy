#![no_std]
#![no_main]

//! FlexSPI memory-mapped (XIP-style) reads on the FRDM-MCXN947.
//!
//! Once the driver has configured `FLEXSPI0` the flash is directly readable
//! through the AHB window - an ordinary load runs the configured read sequence
//! against the device, with no driver call involved. This example programs a
//! sector with IP commands, then reads it back purely through the window and
//! checks the two agree.
//!
//! This is also the cache test. The window sits behind CACHE64, so an IP-command
//! write is invisible through the window until the cache is invalidated; the
//! driver does that at the end of every erase and program. A mismatch here where
//! `flexspi_quad_polling.rs` passes points at that invalidation rather than at
//! the flash.
//!
//! The window is read through its Secure alias, since this HAL runs entirely
//! Secure. The read sequence carries a 3-byte address, so only the low 16 MiB is
//! reachable - the offset below stays well under that.

use defmt::{info, panic, unwrap};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn as hal;
use embassy_time::Timer;
use hal::config::Config;
use hal::flexspi::{Flexspi, NorFlash};
use panic_probe as _;

#[path = "../flexspi_common.rs"]
mod flexspi_common;

use flexspi_common::{FLASH_CONFIG, FLASH_PAGE_SIZE, FLASH_SECTOR_SIZE, check_pattern, fill_pattern, flexspi_clock};

/// Flash offset to exercise (sector aligned, under the 16 MiB 3-byte reach).
const FLASH_OFFSET: u32 = 0x0040_0000;

/// Read `buf.len()` bytes from the memory-mapped window at `offset`.
///
/// # Safety
///
/// `base` must be the window pointer of a live [`NorFlash`], and
/// `offset + buf.len()` must stay inside the configured device size.
unsafe fn memory_mapped_read(base: *const u8, offset: u32, buf: &mut [u8]) {
    for (i, b) in buf.iter_mut().enumerate() {
        *b = unsafe { core::ptr::read_volatile(base.add(offset as usize + i)) };
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(Config::default());

    info!("FlexSPI memory-mapped read example");

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
    let window = flash.memory_mapped_ptr();

    info!("Memory-mapped window at 0x{=u32:08x}", window as u32);

    // Program a known pattern into one sector using IP commands.
    info!(
        "Programming sector at flash offset 0x{=u32:08x} via IP commands",
        FLASH_OFFSET
    );
    unwrap!(flash.blocking_erase_sector(FLASH_OFFSET));
    let mut page = [0u8; FLASH_PAGE_SIZE];
    for pg in 0..(FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE) as u32 {
        let addr = FLASH_OFFSET + pg * FLASH_PAGE_SIZE as u32;
        fill_pattern(addr, &mut page);
        unwrap!(flash.blocking_page_program(addr, &page));
    }

    // Read the whole sector back through the memory-mapped window only.
    info!(
        "Reading it back through the window at 0x{=u32:08x}",
        window as u32 + FLASH_OFFSET
    );
    let mut readback = [0u8; FLASH_SECTOR_SIZE];
    // SAFETY: `window` came from the live `flash` above, and the sector at
    // `FLASH_OFFSET` is inside the configured 8 MiB device.
    unsafe { memory_mapped_read(window, FLASH_OFFSET, &mut readback) };

    match check_pattern(FLASH_OFFSET, &readback) {
        None => info!(
            "Memory-mapped read PASSED: {=usize} bytes match the IP-programmed pattern",
            readback.len()
        ),
        Some(bad) => panic!("memory-mapped read mismatch at flash offset 0x{=u32:08x}", bad),
    }

    loop {
        Timer::after_secs(1).await;
        info!("FlexSPI memory-mapped demo heartbeat");
    }
}
