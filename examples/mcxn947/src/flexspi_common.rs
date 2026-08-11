// This module is shared via `#[path]` between several flexspi_* example
// binaries; each one only references a subset of the helpers below.
#![allow(dead_code)]

use defmt::{assert, assert_eq, info, panic, unwrap};
use embassy_mcxn::clocks::config::Div;
use embassy_mcxn::clocks::{FlexspiClockConfig, FlexspiClockSource};
use embassy_mcxn::flexspi::lookup::opcodes::sdr::{CMD, DUMMY, MODE8, RADDR, READ, WRITE};
use embassy_mcxn::flexspi::lookup::{Command, Instr, LookupTable, Pads, SequenceBuilder};
use embassy_mcxn::flexspi::{Async, Blocking, FlashConfig, NorFlash};

pub const FLASH_PAGE_SIZE: usize = 256;
pub const FLASH_SECTOR_SIZE: usize = 4096;

/// Number of contiguous sectors the self-test will erase, program, and verify.
pub const SELF_TEST_SECTORS: u32 = 4;
/// Total byte span exercised by the self-test.
pub const SELF_TEST_BYTES: u32 = SELF_TEST_SECTORS * FLASH_SECTOR_SIZE as u32;
/// Number of pages in the self-test span.
pub const SELF_TEST_PAGES: u32 = SELF_TEST_BYTES / FLASH_PAGE_SIZE as u32;

/// Where in the flash the self-tests operate. Far enough in to stay clear of
/// anything a boot image would occupy.
pub const FLASH_BASE: u32 = 1000 * FLASH_SECTOR_SIZE as u32;

/// Lengths used to exercise the read path with a variety of (mostly) odd sizes.
/// They include sub-byte-bus, sub-FIFO, full page, and across-page values.
pub const READ_LEN_PROBES: &[usize] = &[1, 3, 7, 17, 33, 63, 127, 256, 257, 511, 1024];

/// Serial root clock the examples aim for.
///
/// The board's own boot configuration runs the flash at 75 MHz from PLL0, but
/// PLL0 only comes up in this HAL as a side effect of selecting it for
/// `main_clk`, which also moves the core to 150 MHz and raises the core rail.
/// These examples deliberately stay on the reset clock tree and drive FlexSPI
/// from `fro_hf` instead, which keeps them well inside the mid-drive 100 MHz
/// ceiling and leaves one variable to debug rather than three.
pub const FLEXSPI_TARGET_HZ: u32 = 50_000_000;

/// A FlexSPI clock configuration at or below [`FLEXSPI_TARGET_HZ`].
///
/// `fro_hf` follows `SCG0.FIRCCFG.RANGE`, so it is 48 MHz at the reset default
/// and 144 MHz once the FIRC range is raised. Sizing the divider from the
/// measured rate keeps this correct either way.
///
/// Panics if the clock tree has not been initialised, or if the required divisor
/// exceeds the 3-bit divider.
pub fn flexspi_clock() -> FlexspiClockConfig {
    let fro_hf = embassy_mcxn::clocks::with_clocks(|c| c.fro_hf).expect("clocks not initialised");
    let divisor = fro_hf.div_ceil(FLEXSPI_TARGET_HZ);

    FlexspiClockConfig {
        source: FlexspiClockSource::FroHf,
        div: Div::from_divisor(divisor).expect("fro_hf too fast for the FlexSPI divider"),
    }
}

/// Deterministic pseudo-random byte for a given flash address.
///
/// Picks bytes with a Knuth multiplicative hash so that adjacent addresses
/// look uncorrelated; this catches off-by-one address arithmetic that an
/// `i as u8` ramp would silently mask (since 256 = page size).
#[inline]
pub fn pattern_byte(address: u32) -> u8 {
    let h = address.wrapping_mul(2_654_435_761);
    (h ^ (h >> 16)) as u8
}

pub fn fill_pattern(base: u32, buffer: &mut [u8]) {
    for (i, b) in buffer.iter_mut().enumerate() {
        *b = pattern_byte(base + i as u32);
    }
}

/// Returns the absolute address of the first mismatch, or `None` on match.
pub fn check_pattern(base: u32, buffer: &[u8]) -> Option<u32> {
    for (i, b) in buffer.iter().enumerate() {
        if *b != pattern_byte(base + i as u32) {
            return Some(base + i as u32);
        }
    }
    None
}

/// Returns the absolute address of the first non-`0xFF` byte, or `None`.
pub fn check_erased(base: u32, buffer: &[u8]) -> Option<u32> {
    for (i, b) in buffer.iter().enumerate() {
        if *b != 0xff {
            return Some(base + i as u32);
        }
    }
    None
}

/// Flash configuration for the FRDM-MCXN947's on-board NOR flash.
///
/// The board carries a Winbond **W25Q64JVSSIQ** (64 Mbit = 8 MiB) on FlexSPI
/// port A, wired for quad operation: `P3_0` SS0, `P3_7` SCLK, `P3_6` DQS and
/// `P3_8`-`P3_11` DATA0-3. Reads use the 1-4-4 `0xEB` fast-read-quad-I/O
/// sequence; erase and program stay in single-pad framing.
///
/// The sequences all use a 3-byte (24-bit) address, which reaches 16 MiB and so
/// covers this part in full; a flash larger than 16 MiB would need 4-byte-address
/// sequences instead. The controller is not the limit: the memory-mapped window
/// spans 256 MiB and IP commands carry a full 32-bit address in `IPCR0.SFAR`.
///
/// Cross-checked against NXP's own board configuration for this part
/// (`mcxn_flexspi_nor_config.c` and the FlexSPI driver example's `customLUT`).
pub const FLASH_CONFIG: FlashConfig = FlashConfig {
    // W25Q64 == 8 MiB == 8192 KiB. `flash_size_kbytes` programs FLSHA1CR0.FLSHSZ,
    // which bounds every IP and memory-mapped access; sizing it to the real chip
    // makes the controller reject out-of-range accesses instead of wrapping.
    flash_size_kbytes: 0x2000,
    page_size: FLASH_PAGE_SIZE,
    // SR1 bit 0 is BUSY, active high.
    busy_status_polarity: true,
    busy_status_offset: 0,
    lookup_table: LookupTable::new()
        .command(
            Command::Read,
            // 1-4-4 fast read quad I/O. The mode byte plus four dummy cycles are
            // what give the flash time to turn the data lines around; a read with
            // no gap between the controller driving the address and the flash
            // driving data is not supported by the hardware.
            SequenceBuilder::new()
                .instr(Instr::new(CMD, Pads::One, 0xEB))
                .instr(Instr::new(RADDR, Pads::Four, 0x18))
                .instr(Instr::new(MODE8, Pads::Four, 0xF0))
                .instr(Instr::new(DUMMY, Pads::Four, 0x04))
                .instr(Instr::new(READ, Pads::Four, 0x00))
                .build(),
        )
        .command(
            Command::ReadStatus,
            SequenceBuilder::new()
                .instr(Instr::new(CMD, Pads::One, 0x05))
                .instr(Instr::new(READ, Pads::One, 0x00))
                .build(),
        )
        .command(
            Command::WriteEnable,
            SequenceBuilder::new().instr(Instr::new(CMD, Pads::One, 0x06)).build(),
        )
        .command(
            Command::ReadId,
            SequenceBuilder::new()
                .instr(Instr::new(CMD, Pads::One, 0x9F))
                .instr(Instr::new(READ, Pads::One, 0x00))
                .build(),
        )
        .command(
            Command::EraseSector,
            SequenceBuilder::new()
                .instr(Instr::new(CMD, Pads::One, 0x20))
                .instr(Instr::new(RADDR, Pads::One, 0x18))
                .build(),
        )
        .command(
            Command::PageProgram,
            SequenceBuilder::new()
                .instr(Instr::new(CMD, Pads::One, 0x02))
                .instr(Instr::new(RADDR, Pads::One, 0x18))
                .instr(Instr::new(WRITE, Pads::One, 0x00))
                .build(),
        ),
    read_seq: Command::Read as u8,
    read_status_seq: Command::ReadStatus as u8,
    write_enable_seq: Command::WriteEnable as u8,
    read_id_seq: Command::ReadId as u8,
    erase_sector_seq: Command::EraseSector as u8,
    page_program_seq: Command::PageProgram as u8,
    // Enable Reset (0x66) then Reset Device (0x99), so a part left in
    // continuous-read mode by a previous boot starts from a known state.
    reset_sequence: Some(
        SequenceBuilder::new()
            .instr(Instr::new(CMD, Pads::One, 0x66))
            .instr(Instr::new(CMD, Pads::One, 0x99))
            .build(),
    ),
    // No mode-entry command. The 1-4-4 read above needs the quad-enable bit
    // (SR2 bit 1) set, and the part fitted to this board ships with it already
    // programmed non-volatile - confirmed by these examples' quad reads working
    // with no mode entry at all, which is also why NXP's own example never writes
    // it. If `0xEB` reads come back wrong on some other board, add a sequence
    // issuing 0x06 write-enable then 0x31 (write SR2) with SR2 | 0x02, and point a
    // `DeviceCommand` at it.
    device_mode_command: None,
};

/// Erase / program / read-back self-test over [`SELF_TEST_SECTORS`] sectors,
/// through the blocking API.
///
/// Panics via `defmt` on the first disagreement, naming the flash address.
pub fn blocking_self_test(flash: &mut NorFlash<'_, Blocking>) {
    // 1) Vendor ID is idempotent.
    let id_a = unwrap!(flash.blocking_vendor_id());
    let id_b = unwrap!(flash.blocking_vendor_id());
    assert_eq!(id_a, id_b, "vendor id changed between back-to-back reads");
    info!("Vendor ID: 0x{=u8:02x} (Winbond is 0xef)", id_a);

    // 2) Erase the test span sector by sector.
    info!(
        "Erasing {=u32} sectors at 0x{=u32:08x} ({=u32} bytes)",
        SELF_TEST_SECTORS, FLASH_BASE, SELF_TEST_BYTES
    );
    for s in 0..SELF_TEST_SECTORS {
        unwrap!(flash.blocking_erase_sector(FLASH_BASE + s * FLASH_SECTOR_SIZE as u32));
    }

    // 3) Verify all-0xFF using a sector-sized read, repeated for the whole span.
    let mut sector = [0u8; FLASH_SECTOR_SIZE];
    for s in 0..SELF_TEST_SECTORS {
        let addr = FLASH_BASE + s * FLASH_SECTOR_SIZE as u32;
        sector.fill(0);
        unwrap!(flash.blocking_read(addr, &mut sector));
        if let Some(bad) = check_erased(addr, &sector) {
            panic!("erase verify failed at 0x{=u32:08x}", bad);
        }
    }

    // 3a) Read with a variety of small/odd lengths from the start of the span,
    //     all of which must still see 0xFF. These straddle the driver's
    //     word-, FIFO- and page-sized boundaries.
    let mut probe = [0u8; 1024];
    for &len in READ_LEN_PROBES {
        probe[..len].fill(0);
        unwrap!(flash.blocking_read(FLASH_BASE, &mut probe[..len]));
        if let Some(bad) = check_erased(FLASH_BASE, &probe[..len]) {
            panic!("erased read len {=usize} failed at 0x{=u32:08x}", len, bad);
        }
    }

    // 3b) Read at unaligned offsets within the span.
    for &off in &[1u32, 3, 17, 255, 256, 257, 4095, 4096, 4097] {
        let addr = FLASH_BASE + off;
        probe[..64].fill(0);
        unwrap!(flash.blocking_read(addr, &mut probe[..64]));
        if let Some(bad) = check_erased(addr, &probe[..64]) {
            panic!(
                "unaligned erased read at 0x{=u32:08x} failed at 0x{=u32:08x}",
                addr, bad
            );
        }
    }
    info!("Erase verified across {=u32} bytes", SELF_TEST_BYTES);

    // 4) Program every page in the span with a deterministic pattern.
    info!("Programming {=u32} pages", SELF_TEST_PAGES);
    let mut page = [0u8; FLASH_PAGE_SIZE];
    for p_idx in 0..SELF_TEST_PAGES {
        let addr = FLASH_BASE + p_idx * FLASH_PAGE_SIZE as u32;
        fill_pattern(addr, &mut page);
        unwrap!(flash.blocking_page_program(addr, &page));
    }

    // 5) Read back the entire span sector by sector and verify the pattern.
    for s in 0..SELF_TEST_SECTORS {
        let addr = FLASH_BASE + s * FLASH_SECTOR_SIZE as u32;
        sector.fill(0);
        unwrap!(flash.blocking_read(addr, &mut sector));
        if let Some(bad) = check_pattern(addr, &sector) {
            panic!(
                "pattern verify failed at 0x{=u32:08x} (got 0x{=u8:02x}, want 0x{=u8:02x})",
                bad,
                sector[(bad - addr) as usize],
                pattern_byte(bad),
            );
        }
    }

    // 5a) Re-read with the same length probes to make sure the read path
    //     works at non-sector sizes after programming.
    for &len in READ_LEN_PROBES {
        probe[..len].fill(0);
        unwrap!(flash.blocking_read(FLASH_BASE, &mut probe[..len]));
        if let Some(bad) = check_pattern(FLASH_BASE, &probe[..len]) {
            panic!("post-program read len {=usize} mismatch at 0x{=u32:08x}", len, bad);
        }
    }

    // 5b) Read across a sector boundary.
    let cross_addr = FLASH_BASE + FLASH_SECTOR_SIZE as u32 - 32;
    let mut cross = [0u8; 64];
    unwrap!(flash.blocking_read(cross_addr, &mut cross));
    if let Some(bad) = check_pattern(cross_addr, &cross) {
        panic!("cross-sector read mismatch at 0x{=u32:08x}", bad);
    }
    info!("Pattern verified across {=u32} bytes", SELF_TEST_BYTES);

    // 6) Partial-page write: erase the last sector, write only part of its first
    //    page, and verify the tail stays erased.
    let last_sector = FLASH_BASE + (SELF_TEST_SECTORS - 1) * FLASH_SECTOR_SIZE as u32;
    unwrap!(flash.blocking_erase_sector(last_sector));
    // 8-byte aligned, per the driver's write-granularity contract.
    let partial_len = 96usize;
    let mut partial = [0u8; FLASH_PAGE_SIZE];
    fill_pattern(last_sector, &mut partial);
    unwrap!(flash.blocking_page_program(last_sector, &partial[..partial_len]));

    sector.fill(0);
    unwrap!(flash.blocking_read(last_sector, &mut sector));
    if let Some(bad) = check_pattern(last_sector, &sector[..partial_len]) {
        panic!("partial-page write mismatch at 0x{=u32:08x}", bad);
    }
    if let Some(bad) = check_erased(last_sector + partial_len as u32, &sector[partial_len..]) {
        panic!("partial-page tail not erased at 0x{=u32:08x}", bad);
    }

    // 7) Erase only the middle sector and confirm its neighbours survive.
    let middle = FLASH_BASE + FLASH_SECTOR_SIZE as u32;
    unwrap!(flash.blocking_erase_sector(middle));
    sector.fill(0);
    unwrap!(flash.blocking_read(middle, &mut sector));
    assert!(
        check_erased(middle, &sector).is_none(),
        "middle sector not fully erased"
    );
    sector.fill(0);
    unwrap!(flash.blocking_read(FLASH_BASE, &mut sector));
    assert!(
        check_pattern(FLASH_BASE, &sector).is_none(),
        "sector 0 corrupted after erasing sector 1"
    );
    let third = FLASH_BASE + 2 * FLASH_SECTOR_SIZE as u32;
    sector.fill(0);
    unwrap!(flash.blocking_read(third, &mut sector));
    assert!(
        check_pattern(third, &sector).is_none(),
        "sector 2 corrupted after erasing sector 1"
    );
}

/// [`blocking_self_test`], through the async API.
///
/// Shared by the interrupt-driven and DMA examples, which differ only in how the
/// driver was constructed.
pub async fn async_self_test(flash: &mut NorFlash<'_, Async>) {
    let id_a = unwrap!(flash.read_vendor_id_async().await);
    let id_b = unwrap!(flash.read_vendor_id_async().await);
    assert_eq!(id_a, id_b, "vendor id changed between back-to-back reads");
    info!("Vendor ID: 0x{=u8:02x} (Winbond is 0xef)", id_a);

    info!(
        "Erasing {=u32} sectors at 0x{=u32:08x} ({=u32} bytes)",
        SELF_TEST_SECTORS, FLASH_BASE, SELF_TEST_BYTES
    );
    for s in 0..SELF_TEST_SECTORS {
        unwrap!(
            flash
                .erase_sector_async(FLASH_BASE + s * FLASH_SECTOR_SIZE as u32)
                .await
        );
    }

    let mut sector = [0u8; FLASH_SECTOR_SIZE];
    for s in 0..SELF_TEST_SECTORS {
        let addr = FLASH_BASE + s * FLASH_SECTOR_SIZE as u32;
        sector.fill(0);
        unwrap!(flash.read_async(addr, &mut sector).await);
        if let Some(bad) = check_erased(addr, &sector) {
            panic!("erase verify failed at 0x{=u32:08x}", bad);
        }
    }

    let mut probe = [0u8; 1024];
    for &len in READ_LEN_PROBES {
        probe[..len].fill(0);
        unwrap!(flash.read_async(FLASH_BASE, &mut probe[..len]).await);
        if let Some(bad) = check_erased(FLASH_BASE, &probe[..len]) {
            panic!("erased read len {=usize} failed at 0x{=u32:08x}", len, bad);
        }
    }

    for &off in &[1u32, 3, 17, 255, 256, 257, 4095, 4096, 4097] {
        let addr = FLASH_BASE + off;
        probe[..64].fill(0);
        unwrap!(flash.read_async(addr, &mut probe[..64]).await);
        if let Some(bad) = check_erased(addr, &probe[..64]) {
            panic!(
                "unaligned erased read at 0x{=u32:08x} failed at 0x{=u32:08x}",
                addr, bad
            );
        }
    }
    info!("Erase verified across {=u32} bytes", SELF_TEST_BYTES);

    info!("Programming {=u32} pages", SELF_TEST_PAGES);
    let mut page = [0u8; FLASH_PAGE_SIZE];
    for p_idx in 0..SELF_TEST_PAGES {
        let addr = FLASH_BASE + p_idx * FLASH_PAGE_SIZE as u32;
        fill_pattern(addr, &mut page);
        unwrap!(flash.page_program_async(addr, &page).await);
    }

    for s in 0..SELF_TEST_SECTORS {
        let addr = FLASH_BASE + s * FLASH_SECTOR_SIZE as u32;
        sector.fill(0);
        unwrap!(flash.read_async(addr, &mut sector).await);
        if let Some(bad) = check_pattern(addr, &sector) {
            panic!(
                "pattern verify failed at 0x{=u32:08x} (got 0x{=u8:02x}, want 0x{=u8:02x})",
                bad,
                sector[(bad - addr) as usize],
                pattern_byte(bad),
            );
        }
    }

    for &len in READ_LEN_PROBES {
        probe[..len].fill(0);
        unwrap!(flash.read_async(FLASH_BASE, &mut probe[..len]).await);
        if let Some(bad) = check_pattern(FLASH_BASE, &probe[..len]) {
            panic!("post-program read len {=usize} mismatch at 0x{=u32:08x}", len, bad);
        }
    }

    let cross_addr = FLASH_BASE + FLASH_SECTOR_SIZE as u32 - 32;
    let mut cross = [0u8; 64];
    unwrap!(flash.read_async(cross_addr, &mut cross).await);
    if let Some(bad) = check_pattern(cross_addr, &cross) {
        panic!("cross-sector read mismatch at 0x{=u32:08x}", bad);
    }
    info!("Pattern verified across {=u32} bytes", SELF_TEST_BYTES);

    let last_sector = FLASH_BASE + (SELF_TEST_SECTORS - 1) * FLASH_SECTOR_SIZE as u32;
    unwrap!(flash.erase_sector_async(last_sector).await);
    let partial_len = 96usize;
    let mut partial = [0u8; FLASH_PAGE_SIZE];
    fill_pattern(last_sector, &mut partial);
    unwrap!(flash.page_program_async(last_sector, &partial[..partial_len]).await);

    sector.fill(0);
    unwrap!(flash.read_async(last_sector, &mut sector).await);
    if let Some(bad) = check_pattern(last_sector, &sector[..partial_len]) {
        panic!("partial-page write mismatch at 0x{=u32:08x}", bad);
    }
    if let Some(bad) = check_erased(last_sector + partial_len as u32, &sector[partial_len..]) {
        panic!("partial-page tail not erased at 0x{=u32:08x}", bad);
    }

    let middle = FLASH_BASE + FLASH_SECTOR_SIZE as u32;
    unwrap!(flash.erase_sector_async(middle).await);
    sector.fill(0);
    unwrap!(flash.read_async(middle, &mut sector).await);
    assert!(
        check_erased(middle, &sector).is_none(),
        "middle sector not fully erased"
    );
    sector.fill(0);
    unwrap!(flash.read_async(FLASH_BASE, &mut sector).await);
    assert!(
        check_pattern(FLASH_BASE, &sector).is_none(),
        "sector 0 corrupted after erasing sector 1"
    );
    let third = FLASH_BASE + 2 * FLASH_SECTOR_SIZE as u32;
    sector.fill(0);
    unwrap!(flash.read_async(third, &mut sector).await);
    assert!(
        check_pattern(third, &sector).is_none(),
        "sector 2 corrupted after erasing sector 1"
    );
}
