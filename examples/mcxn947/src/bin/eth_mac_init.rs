//! ENET0 MAC bring-up check for the FRDM-MCXN947.
//!
//! Constructs the driver and then periodically dumps the MAC/MTL/DMA state,
//! without involving a network stack. Run it with no cable to confirm the init
//! sequence completes and the engines are running, then plug a cable in and
//! watch the link and the RX counters move.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxn::enet::{self, Enet, PacketQueue, StationManagement};
use embassy_mcxn::pac;
use embassy_time::Timer;
use panic_probe as _;
use static_cell::StaticCell;

/// Basic status register, IEEE 802.3 clause 22.
const PHY_REG_BSR: u8 = 0x01;

embassy_mcxn::bind_interrupts!(struct Irqs {
    ETHERNET => enet::InterruptHandler<embassy_mcxn::peripherals::ENET0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mcxn::init(Default::default());
    info!("ENET0 MAC init check");

    let mac_addr = [0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];

    static PACKETS: StaticCell<PacketQueue<4, 4>> = StaticCell::new();
    info!("constructing driver (this resets the MAC and the PHY)...");
    let mut device = unwrap!(Enet::new(
        PACKETS.init(PacketQueue::<4, 4>::new()),
        p.ENET0,
        Irqs,
        p.P1_4,  // TX_CLK / RMII reference clock in
        p.P1_13, // RXDV (CRS_DV)
        p.P1_14, // RXD0
        p.P1_15, // RXD1
        p.P1_6,  // TXD0
        p.P1_7,  // TXD1
        p.P1_5,  // TXEN
        p.P1_21, // MDIO
        p.P1_20, // MDC
        mac_addr,
        enet::Config::default(),
    ));
    info!("driver constructed, PHY at address {}", device.phy().address());

    let regs = pac::ENET0;

    // Confirm the AHB Secure Controller actually took the ENET master security
    // level the driver asked for - the register can be locked by a bootloader.
    let msl = pac::AHBSC.master_sec_level().read();
    let anti = pac::AHBSC.master_sec_anti_pol_reg().read();
    info!(
        "AHBSC master_sec_level={:#010x} (ethernet={:?}) anti_pol={:#010x} (ethernet={:?}) lock={:?}",
        msl.0,
        msl.ethernet(),
        anti.0,
        anti.ethernet(),
        msl.master_sec_level_lock()
    );

    // Where the DMA thinks the rings are, read back from the hardware.
    info!(
        "TX list={:#010x} ring_len={} | RX list={:#010x} ring_len={}",
        regs.dma_ch0_txdesc_list_address().read().0,
        regs.dma_ch0_txdesc_ring_length().read().tdrl(),
        regs.dma_ch0_rxdesc_list_address().read().0,
        regs.dma_ch0_rx_control2().read().rdrl(),
    );

    // The driver clears stale status before starting the engines, so anything
    // reported below is a fresh error.

    // If the RMII reference clock is missing, the transmit engine never leaves
    // the stopped state - so this is the check that proves the PHY is really
    // driving ENET0_TX_CLK.
    let mac_cfg = regs.mac_configuration().read();
    info!(
        "MAC_CONFIGURATION: re={} te={} dm={:?} fes={:?} ps={:?} ipc={}",
        mac_cfg.re(),
        mac_cfg.te(),
        mac_cfg.dm(),
        mac_cfg.fes(),
        mac_cfg.ps(),
        mac_cfg.ipc()
    );

    // Dump the RX ring as the DMA sees it. The list address is the non-secure
    // alias; add it back to read the same memory from secure code.
    {
        let rx_list = regs.dma_ch0_rxdesc_list_address().read().0 | 0x1000_0000;
        for i in 0..4usize {
            let d = (rx_list as *const u32).wrapping_add(i * 4);
            info!(
                "RDES[{}] @ {:#010x}: {:#010x} {:#010x} {:#010x} {:#010x}",
                i,
                d as u32,
                unsafe { core::ptr::read_volatile(d) },
                unsafe { core::ptr::read_volatile(d.wrapping_add(1)) },
                unsafe { core::ptr::read_volatile(d.wrapping_add(2)) },
                unsafe { core::ptr::read_volatile(d.wrapping_add(3)) },
            );
        }
        // Does the non-secure alias really mirror the same physical SRAM?
        let ns = regs.dma_ch0_rxdesc_list_address().read().0 as *const u32;
        let sec = (ns as u32 | 0x1000_0000) as *const u32;
        info!(
            "alias check: NS {:#010x} -> [{:#010x} {:#010x}] | S {:#010x} -> [{:#010x} {:#010x}]",
            ns as u32,
            unsafe { core::ptr::read_volatile(ns) },
            unsafe { core::ptr::read_volatile(ns.wrapping_add(3)) },
            sec as u32,
            unsafe { core::ptr::read_volatile(sec) },
            unsafe { core::ptr::read_volatile(sec.wrapping_add(3)) },
        );

        let rxc = regs.dma_ch0_rx_control().read();
        let txc = regs.dma_ch0_tx_control().read();
        let mtl_tx = regs.mtl_txq0_operation_mode().read();
        let mtl_rx = regs.mtl_rxq0_operation_mode().read();
        info!(
            "DMA rx_control={:#010x} sr={:?} rbsz_13_y={} rx_pbl={} | tx_control={:#010x} st={:?} tx_pbl={}",
            rxc.0,
            rxc.sr(),
            rxc.rbsz_13_y(),
            rxc.rx_pbl(),
            txc.0,
            txc.st(),
            txc.tx_pbl(),
        );
        info!(
            "MTL txq0={:#010x} txqen={:?} tsf={} tqs={} | rxq0={:#010x} rsf={} rqs={}",
            mtl_tx.0,
            mtl_tx.txqen(),
            mtl_tx.tsf(),
            mtl_tx.tqs(),
            mtl_rx.0,
            mtl_rx.rsf(),
            mtl_rx.rqs(),
        );
        info!(
            "current_app_rxdesc={:#010x} rx_tail={:#010x}",
            regs.dma_ch0_current_app_rxdesc().read().0,
            regs.dma_ch0_rxdesc_tail_pointer().read().0,
        );
    }

    loop {
        let status = regs.dma_ch0_status().read();
        let tx_dbg = regs.mtl_txq0_debug().read();
        let rx_dbg = regs.mtl_rxq0_debug().read();
        let dma_dbg = regs.dma_debug_status0().read();

        // Read the PHY's basic status directly rather than going through
        // `Driver::link_state`, which rate-limits itself with an
        // `embassy_time::Timer` and so needs a real executor waker.
        let addr = device.phy().address();
        let bsr = device.phy_mut().station_management().smi_read(addr, PHY_REG_BSR);
        let link_up = bsr & (1 << 2) != 0;
        let an_done = bsr & (1 << 5) != 0;

        info!(
            "link_up={} an_done={} bsr={:#06x} | DMA_CH0_STATUS: ti={} ri={} rbu={} tbu={} fbe={} ais={} teb={} reb={} | TXQ trcsts={:?} txqsts={} | RXQ rxqsts={:?} prxq={} | FSM rps0={:?} tps0={:?} rps={} tps={}",
            link_up,
            an_done,
            bsr,
            status.ti(),
            status.ri(),
            status.rbu(),
            status.tbu(),
            status.fbe(),
            status.ais(),
            status.teb(),
            status.reb(),
            tx_dbg.trcsts(),
            tx_dbg.txqsts(),
            rx_dbg.rxqsts(),
            rx_dbg.prxq(),
            dma_dbg.rps0(),
            dma_dbg.tps0(),
            status.rps(),
            status.tps(),
        );

        if embassy_mcxn::bits::is_set(status.fbe()) {
            error!(
                "fresh fatal bus error: teb={} reb={} (bit2=read/write, bit1=descriptor/data)",
                status.teb(),
                status.reb()
            );
            regs.dma_ch0_status()
                .write(|w| w.set_fbe(embassy_mcxn::bits::from_bool(true)));
        }

        Timer::after_secs(2).await;
    }
}
