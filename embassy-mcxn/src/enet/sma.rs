//! Station Management Agent (MDIO/SMI).
//!
//! MDIO lives inside the `ENET0` register block on MCXN, not in a separate
//! peripheral the way it does on STM32, so [`Sma`] borrows the MAC's registers
//! rather than owning a singleton of its own.

use super::Instance;

/// Clause-22 station management, used to read and write PHY registers.
pub trait StationManagement {
    /// Read a PHY register.
    fn smi_read(&mut self, phy_addr: u8, reg: u8) -> u16;
    /// Write a PHY register.
    fn smi_write(&mut self, phy_addr: u8, reg: u8, val: u16);
}

/// MDIO bus over the `ENET0` MAC's management interface.
pub struct Sma<T: Instance> {
    /// `MAC_MDIO_ADDRESS.CR`, the CSR-clock divider that lands MDC under the
    /// clause-22 maximum of 2.5 MHz.
    clock_range: u8,
    _marker: core::marker::PhantomData<T>,
}

impl<T: Instance> Sma<T> {
    /// Build the MDIO interface.
    ///
    /// The caller is responsible for having muxed the MDC/MDIO pads and for
    /// having ungated `ENET0` - [`super::Enet`] does both before constructing
    /// this.
    pub(crate) fn new(csr_clk_hz: u32) -> Self {
        Self {
            clock_range: clock_range_for(csr_clk_hz),
            _marker: core::marker::PhantomData,
        }
    }
}

/// Pick `MAC_MDIO_ADDRESS.CR` for the CSR (bus) clock frequency.
///
/// Boundaries are the DWMAC CSR-clock ranges. The MCXN947 MAC has the extended
/// divider set, but `system_clk` tops out at 48 MHz today so only the low rows
/// are reachable in practice; the rest are here so this stays correct if the
/// clock tree grows PLL support.
fn clock_range_for(csr_clk_hz: u32) -> u8 {
    match csr_clk_hz / 1_000_000 {
        0..=34 => 2,    // CSR / 16
        35..=59 => 3,   // CSR / 26
        60..=99 => 0,   // CSR / 42
        100..=149 => 1, // CSR / 62
        150..=249 => 4, // CSR / 102
        250..=299 => 5, // CSR / 124
        300..=499 => 6, // CSR / 204
        _ => 7,         // CSR / 324
    }
}

impl<T: Instance> StationManagement for Sma<T> {
    fn smi_read(&mut self, phy_addr: u8, reg: u8) -> u16 {
        let regs = T::info().regs();

        regs.mac_mdio_address().modify(|w| {
            w.set_pa(phy_addr);
            w.set_rda(reg);
            // GOC is modelled as two single-bit fields rather than one 2-bit
            // field: 0b11 = read, 0b01 = write.
            w.set_goc_0(true);
            w.set_goc_1(true);
            w.set_cr(self.clock_range);
            w.set_gb(true);
        });

        while regs.mac_mdio_address().read().gb() {}

        regs.mac_mdio_data().read().gd()
    }

    fn smi_write(&mut self, phy_addr: u8, reg: u8, val: u16) {
        let regs = T::info().regs();

        regs.mac_mdio_data().modify(|w| w.set_gd(val));
        regs.mac_mdio_address().modify(|w| {
            w.set_pa(phy_addr);
            w.set_rda(reg);
            w.set_goc_0(true);
            w.set_goc_1(false);
            w.set_cr(self.clock_range);
            w.set_gb(true);
        });

        while regs.mac_mdio_address().read().gb() {}
    }
}
