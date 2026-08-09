//! DWMAC ring-mode descriptors.
//!
//! Both rings are "ring mode": a flat array of 4-word descriptors whose length
//! is given to the DMA once, with wraparound handled by the hardware. Handing a
//! descriptor to the DMA is a two-step protocol - fill it in, then write the
//! tail pointer, which is what actually wakes the engine.

use core::sync::atomic::{Ordering, fence};

use vcell::VolatileCell;

use super::{Packet, RX_BUFFER_SIZE, TX_BUFFER_SIZE, dma_addr};
use crate::pac::enet::Enet as Regs;

/// Descriptor bit definitions. These come from the DWMAC IP, not from any
/// vendor's register map, so they are identical to the STM32 `eth_v2` set.
#[allow(dead_code)]
mod desc_consts {
    /// Descriptor is owned by the DMA.
    pub const DES3_OWN: u32 = 0x8000_0000;
    /// Descriptor is a context descriptor, not a packet.
    pub const DES3_CTXT: u32 = 0x4000_0000;
    /// Contains the first buffer of the packet.
    pub const DES3_FD: u32 = 0x2000_0000;
    /// Contains the last buffer of the packet.
    pub const DES3_LD: u32 = 0x1000_0000;
    /// Error summary.
    pub const DES3_ES: u32 = 0x0000_8000;

    /// Interrupt on completion.
    pub const TDES2_IOC: u32 = 0x8000_0000;
    /// Buffer 1 length.
    pub const TDES2_B1L: u32 = 0x0000_3FFF;

    /// Checksum insertion control, bits [17:16]. `0b11` inserts the IP header
    /// and payload checksums with the pseudo-header computed in hardware.
    pub const TDES3_CIC_FULL: u32 = 0x0003_0000;

    /// Frame length, TDES3[14:0]. Distinct from TDES2's buffer length: the
    /// buffer length says how much to fetch, the frame length says how long the
    /// packet is. The MAC transmits nothing at all if this is left zero.
    pub const TDES3_FL: u32 = 0x0000_7FFF;

    /// Interrupt on completion.
    pub const RDES3_IOC: u32 = 0x4000_0000;
    /// Buffer 1 address is valid.
    pub const RDES3_BUF1V: u32 = 0x0100_0000;
    /// Packet length, in the write-back format.
    pub const RDES3_PKTLEN: u32 = 0x0000_7FFF;

    /// IP header checksum error (RDES1, write-back format).
    pub const RDES1_IPHE: u32 = 0x0000_0008;
    /// IP payload checksum error (RDES1, write-back format).
    pub const RDES1_IPCE: u32 = 0x0000_0080;
    /// Payload type.
    pub const RDES1_PT: u32 = 0x0000_0007;
    pub const RDES1_PT_UDP: u32 = 1;
    pub const RDES1_PT_TCP: u32 = 2;
}
use desc_consts::*;

/// Transmit descriptor.
#[repr(C)]
pub(crate) struct TDes {
    tdes0: VolatileCell<u32>,
    tdes1: VolatileCell<u32>,
    tdes2: VolatileCell<u32>,
    tdes3: VolatileCell<u32>,
}

impl TDes {
    pub const fn new() -> Self {
        Self {
            tdes0: VolatileCell::new(0),
            tdes1: VolatileCell::new(0),
            tdes2: VolatileCell::new(0),
            tdes3: VolatileCell::new(0),
        }
    }

    /// True if the DMA has handed this descriptor back to us.
    fn available(&self) -> bool {
        self.tdes3.get() & DES3_OWN == 0
    }
}

pub(crate) struct TDesRing<'a> {
    regs: Regs,
    descriptors: &'a mut [TDes],
    buffers: &'a mut [Packet<TX_BUFFER_SIZE>],
    index: usize,
}

impl<'a> TDesRing<'a> {
    /// Initialise the TX ring and register it with the DMA.
    ///
    /// The DMA is still stopped at this point; the caller issues a barrier
    /// before starting it.
    pub(crate) fn new(regs: Regs, descriptors: &'a mut [TDes], buffers: &'a mut [Packet<TX_BUFFER_SIZE>]) -> Self {
        assert!(!descriptors.is_empty());
        assert_eq!(descriptors.len(), buffers.len());

        for td in descriptors.iter_mut() {
            *td = TDes::new();
        }

        // These four registers hold plain byte addresses, but the PAC models
        // them as fields at bit offset 2 - so `set_tdesla(a)` would write
        // `a << 2` and point the DMA at nonsense. Write the raw value instead;
        // the low two bits are read-only zero and descriptors are 16-byte
        // aligned anyway.
        regs.dma_ch0_txdesc_list_address()
            .write(|w| w.0 = dma_addr(descriptors.as_ptr()));
        regs.dma_ch0_txdesc_ring_length()
            .write(|w| w.set_tdrl((descriptors.len() as u16) - 1));
        // Tail == head means "nothing to send"; transmit() moves it forward.
        regs.dma_ch0_txdesc_tail_pointer()
            .write(|w| w.0 = dma_addr(descriptors.as_ptr()));

        Self {
            regs,
            descriptors,
            buffers,
            index: 0,
        }
    }

    pub(crate) fn len(&self) -> usize {
        self.descriptors.len()
    }

    /// The next free transmit buffer, or `None` if the ring is full.
    pub(crate) fn available(&mut self) -> Option<&mut [u8]> {
        if self.descriptors[self.index].available() {
            Some(&mut self.buffers[self.index].0)
        } else {
            None
        }
    }

    /// Hand the buffer previously returned by [`Self::available`] to the DMA.
    pub(crate) fn transmit(&mut self, len: usize) {
        let index = self.index;
        let td = &self.descriptors[index];
        debug_assert!(td.available());
        assert!(len as u32 <= TDES2_B1L);

        td.tdes0.set(dma_addr(self.buffers[index].0.as_ptr()));
        td.tdes2.set((len as u32 & TDES2_B1L) | TDES2_IOC);
        // Single-buffer packet, owned by the DMA, with full checksum insertion.
        td.tdes3
            .set((len as u32 & TDES3_FL) | DES3_FD | DES3_LD | DES3_OWN | TDES3_CIC_FULL);

        // The descriptor writes must land before the DMA is told to look at it.
        fence(Ordering::Release);

        self.index = (index + 1) % self.descriptors.len();

        // The tail pointer addresses the slot *after* the last one owned by the
        // DMA, and writing it is what restarts a suspended transmit engine.
        self.regs
            .dma_ch0_txdesc_tail_pointer()
            .write(|w| w.0 = dma_addr(&self.descriptors[self.index] as *const TDes));
    }
}

/// Receive descriptor.
#[repr(C)]
pub(crate) struct RDes {
    rdes0: VolatileCell<u32>,
    rdes1: VolatileCell<u32>,
    rdes2: VolatileCell<u32>,
    rdes3: VolatileCell<u32>,
}

/// A snapshot of the write-back status words, taken once so the descriptor is
/// not re-read field by field while the DMA may be writing it.
struct RDesInfo {
    rdes1: u32,
    rdes3: u32,
}

impl RDesInfo {
    /// True if this descriptor holds a complete, error-free packet we can pass
    /// to the network stack.
    const fn valid(&self) -> bool {
        // Must be a whole packet (first and last buffer), not an error, and not
        // a context descriptor.
        if self.rdes3 & (DES3_FD | DES3_LD | DES3_ES | DES3_CTXT) != (DES3_FD | DES3_LD) {
            return false;
        }

        // The MAC verified the IPv4 header and TCP/UDP payload checksums, and
        // `capabilities()` tells the stack not to re-verify them. Those results
        // are not folded into the error summary, so check them here or bad
        // frames would be accepted silently.
        let pt = self.rdes1 & RDES1_PT;
        let tcp_or_udp = pt == RDES1_PT_TCP || pt == RDES1_PT_UDP;
        if self.rdes1 & RDES1_IPHE != 0 || (tcp_or_udp && self.rdes1 & RDES1_IPCE != 0) {
            return false;
        }

        true
    }

    /// True if the DMA has handed this descriptor back to us.
    const fn available(&self) -> bool {
        self.rdes3 & DES3_OWN == 0
    }

    /// True if this is a context descriptor we should skip over.
    const fn context_available(&self) -> bool {
        self.rdes3 & (DES3_OWN | DES3_CTXT) == DES3_CTXT
    }

    const fn len(&self) -> u32 {
        self.rdes3 & RDES3_PKTLEN
    }
}

impl RDes {
    pub const fn new() -> Self {
        Self {
            rdes0: VolatileCell::new(0),
            rdes1: VolatileCell::new(0),
            rdes2: VolatileCell::new(0),
            rdes3: VolatileCell::new(0),
        }
    }

    fn info(&self) -> RDesInfo {
        RDesInfo {
            rdes1: self.rdes1.get(),
            rdes3: self.rdes3.get(),
        }
    }

    /// Return this descriptor to the DMA, pointed at `buf`.
    fn set_ready(&self, buf: *mut u8) {
        self.rdes0.set(dma_addr(buf));
        self.rdes3.set(RDES3_BUF1V | RDES3_IOC | DES3_OWN);
    }
}

pub(crate) struct RDesRing<'a> {
    regs: Regs,
    descriptors: &'a mut [RDes],
    buffers: &'a mut [Packet<RX_BUFFER_SIZE>],
    index: usize,
}

impl<'a> RDesRing<'a> {
    /// Initialise the RX ring, arm every descriptor, and register it with the DMA.
    pub(crate) fn new(regs: Regs, descriptors: &'a mut [RDes], buffers: &'a mut [Packet<RX_BUFFER_SIZE>]) -> Self {
        assert!(descriptors.len() > 1);
        assert_eq!(descriptors.len(), buffers.len());

        for (i, desc) in descriptors.iter_mut().enumerate() {
            *desc = RDes::new();
            desc.set_ready(buffers[i].0.as_mut_ptr());
        }

        regs.dma_ch0_rxdesc_list_address()
            .write(|w| w.0 = dma_addr(descriptors.as_ptr()));
        regs.dma_ch0_rx_control2()
            .modify(|w| w.set_rdrl((descriptors.len() as u16) - 1));
        // Every descriptor is owned by the DMA, so the tail is one past the end.
        let end = unsafe { descriptors.as_ptr().add(descriptors.len()) };
        regs.dma_ch0_rxdesc_tail_pointer().write(|w| w.0 = dma_addr(end));

        Self {
            regs,
            descriptors,
            buffers,
            index: 0,
        }
    }

    /// Advance past any context or invalid descriptors, returning the first
    /// usable packet's status.
    fn fast_forward(&mut self) -> Option<RDesInfo> {
        loop {
            let info = self.descriptors[self.index].info();
            if !info.available() {
                break None;
            }

            if info.context_available() {
                self.pop_current();
                continue;
            }

            if !info.valid() {
                #[cfg(feature = "defmt")]
                defmt::warn!("enet: dropping invalid rx descriptor {}", self.index);
                self.pop_current();
                continue;
            }

            break Some(info);
        }
    }

    /// The oldest received packet, or `None`.
    pub(crate) fn available(&mut self) -> Option<&mut [u8]> {
        // Pair with the DMA's descriptor write-back before reading status.
        fence(Ordering::SeqCst);

        let info = self.fast_forward()?;

        Some(&mut self.buffers[self.index].0[..info.len() as usize])
    }

    /// Release the packet previously returned by [`Self::available`].
    pub(crate) fn pop_packet(&mut self) {
        self.pop_current();
        self.fast_forward();
    }

    fn pop_current(&mut self) {
        let index = self.index;
        let rd = &self.descriptors[index];
        debug_assert!(rd.info().available());

        rd.set_ready(self.buffers[index].0.as_mut_ptr());

        fence(Ordering::Release);

        self.index = (index + 1) % self.descriptors.len();

        // Hand the re-armed descriptor back by moving the tail past it, which
        // also restarts the receive engine if it had suspended on an empty ring.
        self.regs
            .dma_ch0_rxdesc_tail_pointer()
            .write(|w| w.0 = dma_addr(&self.descriptors[self.index] as *const RDes));
    }
}
