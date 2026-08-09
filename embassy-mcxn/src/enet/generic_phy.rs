//! Generic clause-22 SMI Ethernet PHY.
//!
//! Register-agnostic: it talks to the PHY only through [`StationManagement`],
//! so it works with any MDIO transport. The FRDM-MCXN947's LAN8741A is a
//! standard clause-22 PHY and needs nothing beyond this.

use core::future::Future;
use core::pin::Pin;
use core::task::Context;

use embassy_time::{Duration, Timer, block_for};

use super::{Phy, StationManagement};

#[allow(dead_code)]
mod phy_consts {
    pub const PHY_REG_BCR: u8 = 0x00;
    pub const PHY_REG_BSR: u8 = 0x01;
    pub const PHY_REG_ID1: u8 = 0x02;
    pub const PHY_REG_ID2: u8 = 0x03;
    pub const PHY_REG_ANTX: u8 = 0x04;
    pub const PHY_REG_ANRX: u8 = 0x05;
    /// Ethernet PHY register control (MMD access).
    pub const PHY_REG_CTL: u8 = 0x0D;
    /// Ethernet PHY address or data (MMD access).
    pub const PHY_REG_ADDAR: u8 = 0x0E;

    /// Wake-up control and status, in MMD address space.
    pub const PHY_REG_WUCSR: u16 = 0x8010;

    pub const PHY_REG_BCR_ANRST: u16 = 1 << 9;
    pub const PHY_REG_BCR_AN: u16 = 1 << 12;
    pub const PHY_REG_BCR_RESET: u16 = 1 << 15;

    pub const PHY_REG_ANTX_100BTX_FD: u16 = 0b1 << 8;
    pub const PHY_REG_ANTX_TECH: u16 = 0b11111 << 5;

    pub const PHY_REG_BSR_UP: u16 = 1 << 2;
    pub const PHY_REG_BSR_ANDONE: u16 = 1 << 5;
}
use self::phy_consts::*;

/// Address used to mean "probe every address during reset".
const PHY_ADDR_AUTO: u8 = 0xFF;

/// Generic SMI Ethernet PHY.
pub struct GenericPhy<SM: StationManagement> {
    phy_addr: u8,
    sm: SM,
    poll_interval: Duration,
    timer: Timer,
}

impl<SM: StationManagement> GenericPhy<SM> {
    /// Construct a PHY at a known SMI address.
    ///
    /// On the FRDM-MCXN947 the LAN8741A is strapped to address 0.
    ///
    /// # Panics
    /// `phy_addr` must be in `0..32`.
    pub fn new(sm: SM, phy_addr: u8) -> Self {
        assert!(phy_addr < 32);
        Self {
            phy_addr,
            sm,
            poll_interval: Duration::from_millis(500),
            timer: Timer::after_ticks(0),
        }
    }

    /// Construct a PHY whose address is discovered by probing 0..32.
    ///
    /// # Panics
    /// [`Phy::phy_reset`] panics if no PHY responds on any address.
    pub fn new_auto(sm: SM) -> Self {
        Self {
            sm,
            phy_addr: PHY_ADDR_AUTO,
            poll_interval: Duration::from_millis(500),
            timer: Timer::after_ticks(0),
        }
    }

    /// Set how often the link state is re-read over MDIO.
    pub fn set_poll_interval(&mut self, poll_interval: Duration) {
        self.poll_interval = poll_interval;
    }

    /// The SMI address in use. Only meaningful after [`Phy::phy_reset`] when
    /// the PHY was built with [`GenericPhy::new_auto`].
    pub fn address(&self) -> u8 {
        self.phy_addr
    }

    /// Access the underlying station management, e.g. to read vendor registers.
    pub fn station_management(&mut self) -> &mut SM {
        &mut self.sm
    }

    /// Write an extended register in MMD address space.
    fn smi_write_ext(&mut self, reg_addr: u16, reg_data: u16) {
        self.sm.smi_write(self.phy_addr, PHY_REG_CTL, 0x0003); // set address
        self.sm.smi_write(self.phy_addr, PHY_REG_ADDAR, reg_addr);
        self.sm.smi_write(self.phy_addr, PHY_REG_CTL, 0x4003); // set data
        self.sm.smi_write(self.phy_addr, PHY_REG_ADDAR, reg_data);
    }
}

impl<SM: StationManagement> Phy for GenericPhy<SM> {
    fn phy_reset(&mut self) {
        if self.phy_addr == PHY_ADDR_AUTO {
            for addr in 0..32 {
                self.sm.smi_write(addr, PHY_REG_BCR, PHY_REG_BCR_RESET);
                // Give each address a total of 100ms to drop the self-clearing
                // reset bit before moving on.
                for _ in 0..10 {
                    if self.sm.smi_read(addr, PHY_REG_BCR) & PHY_REG_BCR_RESET != PHY_REG_BCR_RESET {
                        #[cfg(feature = "defmt")]
                        defmt::trace!("Found ETH PHY at address {}", addr);
                        self.phy_addr = addr;
                        return;
                    }
                    block_for(Duration::from_millis(10));
                }
            }
            panic!("PHY did not respond on any SMI address");
        }

        self.sm.smi_write(self.phy_addr, PHY_REG_BCR, PHY_REG_BCR_RESET);
        while self.sm.smi_read(self.phy_addr, PHY_REG_BCR) & PHY_REG_BCR_RESET == PHY_REG_BCR_RESET {}
    }

    fn phy_init(&mut self) {
        // Clear the wake-up control and status register.
        self.smi_write_ext(PHY_REG_WUCSR, 0);

        // Advertise 100BASE-TX full duplex only, then restart autonegotiation.
        let antx = self.sm.smi_read(self.phy_addr, PHY_REG_ANTX);
        let antx = (antx & !PHY_REG_ANTX_TECH) | PHY_REG_ANTX_100BTX_FD;
        self.sm.smi_write(self.phy_addr, PHY_REG_ANTX, antx);
        self.sm
            .smi_write(self.phy_addr, PHY_REG_BCR, PHY_REG_BCR_AN | PHY_REG_BCR_ANRST);
    }

    fn poll_link(&mut self, cx: &mut Context) -> Option<bool> {
        // MDIO reads are slow and blocking, so rate-limit them: return `None`
        // ("no new information") until the poll interval elapses.
        if Pin::new(&mut self.timer).poll(cx).is_pending() {
            return None;
        }
        self.timer = Timer::after(self.poll_interval);
        let _ = Pin::new(&mut self.timer).poll(cx);

        let bsr = self.sm.smi_read(self.phy_addr, PHY_REG_BSR);

        // The link only counts as up once autonegotiation has also completed.
        Some(bsr & PHY_REG_BSR_ANDONE != 0 && bsr & PHY_REG_BSR_UP != 0)
    }
}
