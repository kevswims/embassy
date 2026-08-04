#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn as hal;
use hal::crc::Crc;
use panic_probe as _;

/// The check value every entry in the CRC catalogue is defined against.
const CHECK: &[u8] = b"123456789";

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(hal::config::Config::default());
    let mut crc0 = p.CRC0;

    defmt::info!("CRC example");

    // Catalogue check values for the ASCII string "123456789".
    let mut failures = 0;

    macro_rules! check32 {
        ($ctor:ident, $expected:expr) => {{
            let mut crc = Crc::$ctor(crc0.reborrow());
            crc.feed(CHECK);
            let got = crc.finalize();
            let expected: u32 = $expected;
            if got == expected {
                defmt::info!("  {=str} = {=u32:#010x}", stringify!($ctor), got);
            } else {
                defmt::error!(
                    "  {=str} = {=u32:#010x}, expected {=u32:#010x}",
                    stringify!($ctor),
                    got,
                    expected
                );
                failures += 1;
            }
        }};
    }

    macro_rules! check16 {
        ($ctor:ident, $expected:expr) => {{
            let mut crc = Crc::$ctor(crc0.reborrow());
            crc.feed(CHECK);
            let got = crc.finalize();
            let expected: u16 = $expected;
            if got == expected {
                defmt::info!("  {=str} = {=u16:#06x}", stringify!($ctor), got);
            } else {
                defmt::error!(
                    "  {=str} = {=u16:#06x}, expected {=u16:#06x}",
                    stringify!($ctor),
                    got,
                    expected
                );
                failures += 1;
            }
        }};
    }

    defmt::info!("CRC-32 algorithms:");
    check32!(new_iso_hdlc, 0xcbf4_3926);
    check32!(new_bzip2, 0xfc89_1918);
    check32!(new_mpeg2, 0x0376_e6e7);
    check32!(new_jam_crc, 0x340b_c6d9);
    check32!(new_c, 0xe306_9283);
    check32!(new_d, 0x8731_5576);
    check32!(new_posix, 0x765e_7680);
    check32!(new_q, 0x3010_bf7f);
    check32!(new_xfer, 0xbd0b_e338);

    defmt::info!("CRC-16 algorithms:");
    check16!(new_arc, 0xbb3d);
    check16!(new_ccitt_false, 0x29b1);
    check16!(new_kermit, 0x2189);
    check16!(new_modbus, 0x4b37);
    check16!(new_xmodem, 0x31c3);
    check16!(new_a, 0xbf05);
    check16!(new_aug_ccitt, 0xe5cc);
    check16!(new_buypass, 0xfee8);
    check16!(new_ccitt_zero, 0x31c3);
    check16!(new_cdma_2000, 0x4c06);
    check16!(new_dds_110, 0x9ecf);
    check16!(new_dect_x, 0x007f);
    check16!(new_mcrf4xx, 0x6f91);
    check16!(new_riello, 0x63d0);
    check16!(new_t10_dif, 0xd0db);
    check16!(new_teledisk, 0x0fb3);
    check16!(new_tms_37157, 0x26b1);
    // The remainder all apply a final XOR through CTRL[FXOR].
    check16!(new_dnp, 0xea82);
    check16!(new_en13757, 0xc2b7);
    check16!(new_genibus, 0xd64e);
    check16!(new_maxim, 0x44c2);
    check16!(new_usb, 0xb4c8);
    check16!(new_x25, 0x906e);

    // Feeding the same bytes at different access widths must land on the same
    // checksum. This is what the DATA8/DATA16/DATA32 register aliases exist
    // for: an 8-bit write clocks in one byte, not four.
    defmt::info!("access widths:");
    let words: [u32; 2] = [0x0403_0201, 0x0807_0605];
    let halves: [u16; 4] = [0x0201, 0x0403, 0x0605, 0x0807];
    let bytes: [u8; 8] = [1, 2, 3, 4, 5, 6, 7, 8];

    let mut crc = Crc::new_iso_hdlc(crc0.reborrow());
    crc.feed(&words);
    let via_words = crc.finalize();

    let mut crc = Crc::new_iso_hdlc(crc0.reborrow());
    crc.feed(&halves);
    let via_halves = crc.finalize();

    let mut crc = Crc::new_iso_hdlc(crc0.reborrow());
    crc.feed(&bytes);
    let via_bytes = crc.finalize();

    defmt::info!(
        "  u32 {=u32:#010x}  u16 {=u32:#010x}  u8 {=u32:#010x}",
        via_words,
        via_halves,
        via_bytes
    );
    if via_words != via_halves || via_words != via_bytes {
        defmt::error!("  access widths disagree");
        failures += 1;
    }

    if failures == 0 {
        defmt::info!("all CRC checks passed");
    } else {
        defmt::panic!("{} CRC check(s) failed", failures);
    }

    loop {
        defmt::info!("idle");
        cortex_m::asm::delay(48_000_000);
    }
}
