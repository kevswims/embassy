//! ENET0 MDIO bring-up check for the FRDM-MCXN947.
//!
//! Scans all 32 SMI addresses and prints the PHY identifier registers. This
//! needs no Ethernet cable and no RMII reference clock - MDC is derived from
//! the CSR (bus) clock - so it isolates pin muxing, clock gating and the MDIO
//! state machine from everything else.
//!
//! The board's LAN8741A should answer at address 0 with ID1 `0x0007` and ID2
//! `0xC12x` (the low nibble is the silicon revision).

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn::enet::{self, StationManagement};
use panic_probe as _;

/// PHY identifier registers, IEEE 802.3 clause 22.
const PHY_REG_ID1: u8 = 0x02;
const PHY_REG_ID2: u8 = 0x03;
/// Basic control and status, used as a second opinion on whether anything is
/// really there: an unterminated MDIO bus reads back as all-ones.
const PHY_REG_BCR: u8 = 0x00;
const PHY_REG_BSR: u8 = 0x01;

/// LAN8741A: OUI 0x1F0, model 0x12, revision in the low nibble.
const LAN8741A_ID1: u16 = 0x0007;
const LAN8741A_ID2_MASKED: u16 = 0xC120;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mcxn::init(Default::default());
    info!("ENET0 MDIO probe");

    let mut sma = unwrap!(enet::sma(p.ENET0, p.P1_21 /* MDIO */, p.P1_20 /* MDC */));

    let mut found = 0u32;
    for addr in 0..32u8 {
        let id1 = sma.smi_read(addr, PHY_REG_ID1);
        let id2 = sma.smi_read(addr, PHY_REG_ID2);

        // A floating bus reads 0xFFFF, and an address with nothing at it reads
        // 0x0000. Neither is a PHY.
        if (id1 == 0xFFFF && id2 == 0xFFFF) || (id1 == 0x0000 && id2 == 0x0000) {
            continue;
        }

        found += 1;
        let bcr = sma.smi_read(addr, PHY_REG_BCR);
        let bsr = sma.smi_read(addr, PHY_REG_BSR);
        info!(
            "addr {}: ID1={:#06x} ID2={:#06x} BCR={:#06x} BSR={:#06x}",
            addr, id1, id2, bcr, bsr
        );

        if id1 == LAN8741A_ID1 && (id2 & 0xFFF0) == LAN8741A_ID2_MASKED {
            info!("  -> LAN8741A (revision {})", id2 & 0xF);
        } else {
            warn!("  -> responded, but does not match LAN8741A");
        }
    }

    if found == 0 {
        error!("no PHY responded on any address - check MDC/MDIO mux and the ENET0 clock gate");
    } else {
        info!("probe complete, {} device(s) responded", found);
    }

    loop {
        embassy_time::Timer::after_secs(60).await;
    }
}
