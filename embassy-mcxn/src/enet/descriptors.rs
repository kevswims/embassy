//! DWMAC ring-mode descriptors, and the transmit and receive rings built on them.
//!
//! Both rings are "ring mode": a flat array of 4-word descriptors whose length
//! is given to the DMA once, with wraparound handled by the hardware. Handing a
//! descriptor to the DMA is a two-step protocol - fill it in, then write the
//! tail pointer, which is what actually wakes the engine.
//!
//! The frames live in the network stack's packet pool, not here. The receive
//! ring holds one pool buffer per descriptor, which the DMA fills in place; the
//! transmit ring holds each frame's buffer until the DMA has read it.
//!
//! The DMA stops fetching at the tail pointer and prefetches descriptors, so
//! the tail must never land on a descriptor the DMA may be waiting on. The
//! receive ring keeps its most recently rearmed descriptor as the tail, and the
//! transmit ring never fills its last descriptor. This is the same layout
//! `embassy-stm32` uses for this IP.

use core::sync::atomic::{Ordering, fence};

#[cfg(feature = "ptp")]
use heapless::deque::DequeView;
use vcell::VolatileCell;
use xarxa_driver::PacketBuf;
#[cfg(feature = "ptp")]
use xarxa_driver::{Timestamp, TxTimestamp};

use super::dma_addr;
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

    /// Transmit timestamp enable, TDES2 (read format). Set per packet to ask
    /// the MAC to capture an egress timestamp.
    pub const TDES2_TTSE: u32 = 0x4000_0000;

    /// Transmit timestamp status, TDES3 (write-back format). When set, TDES1
    /// and TDES0 hold the egress timestamp instead of their read-format
    /// meanings.
    pub const TDES3_TTSS: u32 = 0x0002_0000;

    /// RDES1 status word is valid, RDES3 (write-back format). Without this,
    /// RDES1 holds stale data - descriptors handed back to the DMA are not
    /// required to clear it.
    pub const RDES3_RS1V: u32 = 0x0400_0000;

    /// Timestamp available for this packet, RDES1 (write-back format).
    pub const RDES1_TSA: u32 = 0x0000_4000;

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

/// Alignment of receive buffers, and the granularity of their usable size.
///
/// The DMA needs 4-byte aligned buffer addresses, and `RBSZ` cannot express a
/// size that is not a multiple of 4. Requiring 8 keeps the packet pool
/// definition the same as for the STM32 driver.
const BUF_ALIGN: usize = 8;

/// The part of a buffer of `len` bytes the DMA can use.
fn rx_buf_size(len: usize) -> usize {
    len / BUF_ALIGN * BUF_ALIGN
}

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

    /// Program the read format: buffer address, length, and interrupt on
    /// completion.
    fn set_frame(&self, buf: *const u8, len: usize) {
        assert!(len as u32 <= TDES2_B1L);
        self.tdes0.set(dma_addr(buf));
        self.tdes2.set((len as u32 & TDES2_B1L) | TDES2_IOC);
    }

    /// Ask the MAC to capture an egress timestamp for this frame.
    ///
    /// Per packet: hardware has few timestamp slots, so timestamping
    /// everything would lose most of them. `set_frame` rewrites TDES2 on every
    /// transmit, so a frame that does not ask needs no explicit clear.
    #[cfg(feature = "ptp")]
    fn request_timestamp(&self) {
        self.tdes2.set(self.tdes2.get() | TDES2_TTSE);
    }

    /// Hand the descriptor to the DMA as a single-buffer frame of `len` bytes,
    /// with full checksum insertion.
    fn set_owned(&self, len: usize) {
        self.tdes3
            .set((len as u32 & TDES3_FL) | DES3_FD | DES3_LD | DES3_OWN | TDES3_CIC_FULL);
    }

    /// The egress timestamp, if the MAC captured one for this frame.
    ///
    /// Valid only in the write-back format, where TDES1/TDES0 alias the
    /// seconds/nanoseconds in place of the buffer address and lengths.
    #[cfg(feature = "ptp")]
    fn timestamp(&self) -> Option<Timestamp> {
        (self.tdes3.get() & TDES3_TTSS != 0)
            .then(|| Timestamp::from_seconds_and_nanos(self.tdes1.get(), self.tdes0.get()))
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

    /// True if the hardware captured an ingress timestamp for this packet.
    ///
    /// Both bits matter: RDES1 is only meaningful when RS1V says the status
    /// word was actually written back, since descriptors returned to the DMA
    /// do not clear it.
    #[cfg(feature = "ptp")]
    const fn has_timestamp(&self) -> bool {
        self.rdes3 & RDES3_RS1V != 0 && self.rdes1 & RDES1_TSA != 0
    }

    /// True if this is a context descriptor we should skip over.
    const fn context_available(&self) -> bool {
        self.rdes3 & (DES3_OWN | DES3_CTXT) == DES3_CTXT
    }

    const fn len(&self) -> usize {
        (self.rdes3 & RDES3_PKTLEN) as usize
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

    /// The timestamp carried by a context descriptor.
    ///
    /// All-ones in both words is the hardware's "timestamp corrupted or
    /// unavailable" marker.
    #[cfg(feature = "ptp")]
    fn context_timestamp(&self) -> Option<Timestamp> {
        let nanos = self.rdes0.get();
        let seconds = self.rdes1.get();

        (!(nanos == u32::MAX && seconds == u32::MAX)).then(|| Timestamp::from_seconds_and_nanos(seconds, nanos))
    }

    /// Return this descriptor to the DMA, pointed at `buf`.
    fn set_ready(&self, buf: *mut u8) {
        self.rdes0.set(dma_addr(buf));
        // The address must be visible before ownership is: the slot's buffer
        // changes every time a frame is handed to the stack, and a DMA that saw
        // the old address would write into a buffer the stack now owns.
        fence(Ordering::Release);
        self.rdes3.set(RDES3_BUF1V | RDES3_IOC | DES3_OWN);
    }
}

pub(crate) struct RDesRing<'a> {
    regs: Regs,
    descriptors: &'a mut [RDes],
    /// One buffer per descriptor, DMA'd into in place. `None` for a slot whose
    /// buffer was handed to the stack and has not been replaced by `give` yet.
    buffers: &'a mut [Option<PacketBuf>],
    /// Next descriptor to receive from.
    index: usize,
    /// Next empty slot to give a buffer to.
    fill: usize,
    /// Number of empty slots.
    empty: usize,
    /// Receive buffer size programmed into `RBSZ`, 0 before the first buffer.
    buf_size: usize,
}

impl<'a> RDesRing<'a> {
    /// Register the RX ring with the DMA, with every slot empty.
    ///
    /// The stack fills the slots through [`Self::give`] before the first
    /// receive.
    pub(crate) fn new(regs: Regs, descriptors: &'a mut [RDes], buffers: &'a mut [Option<PacketBuf>]) -> Self {
        assert!(descriptors.len() > 1);
        // A timestamped frame takes a frame descriptor and a context
        // descriptor, and the tail guard takes a third.
        #[cfg(feature = "ptp")]
        assert!(descriptors.len() > 2);
        assert_eq!(descriptors.len(), buffers.len());

        for (desc, buf) in descriptors.iter_mut().zip(buffers.iter_mut()) {
            *desc = RDes::new();
            *buf = None;
        }

        regs.dma_ch0_rxdesc_list_address()
            .write(|w| w.0 = dma_addr(descriptors.as_ptr()));
        regs.dma_ch0_rx_control2()
            .modify(|w| w.set_rdrl((descriptors.len() as u16) - 1));
        // Nothing is armed yet. Start the tail on the last descriptor, so that
        // arming the first one moves it.
        let last = &descriptors[descriptors.len() - 1] as *const RDes;
        regs.dma_ch0_rxdesc_tail_pointer().write(|w| w.0 = dma_addr(last));

        let empty = descriptors.len();
        Self {
            regs,
            descriptors,
            buffers,
            index: 0,
            fill: 0,
            empty,
            buf_size: 0,
        }
    }

    /// Number of empty slots, waiting for a buffer.
    pub(crate) fn wanted(&self) -> usize {
        self.empty
    }

    /// Give a buffer to the next empty slot and hand it to the DMA.
    pub(crate) fn give(&mut self, mut buf: PacketBuf) {
        if self.empty == 0 {
            // Not asked for: dropping it returns it to the pool.
            return;
        }

        let storage = buf.storage_mut();
        assert!(
            storage.as_ptr() as usize % BUF_ALIGN == 0,
            "ENET RX buffers must be 8-byte aligned; use a pool with ALIGN = 8"
        );
        let size = rx_buf_size(storage.len());

        if size != self.buf_size {
            self.buf_size = size;
            // RBSZ is split: bits [2:1] hold RBSZ[1:0] and must stay zero, and
            // bits [14:3] hold RBSZ[13:2]. So the field takes the size shifted
            // right by two.
            self.regs.dma_ch0_rx_control().modify(|w| {
                w.set_rbsz_x_0(0);
                w.set_rbsz_13_y((size >> 2) as u16);
            });
        }

        let i = self.fill;
        self.descriptors[i].set_ready(storage.as_mut_ptr());
        self.buffers[i] = Some(buf);
        self.fill = (i + 1) % self.descriptors.len();
        self.empty -= 1;
        self.rearmed(i);
    }

    /// Tell the DMA descriptor `i` was handed back to it.
    fn rearmed(&self, i: usize) {
        fence(Ordering::Release);
        // The rearmed descriptor becomes the tail, releasing the previous one.
        // Writing the tail is also what restarts a receive engine that had
        // suspended on an empty ring.
        self.regs
            .dma_ch0_rxdesc_tail_pointer()
            .write(|w| w.0 = dma_addr(&self.descriptors[i] as *const RDes));
    }

    /// Take a received frame, if any.
    ///
    /// The buffer the frame was DMA'd into is handed out, and its slot stays
    /// empty until `give` fills it again.
    pub(crate) fn receive(&mut self) -> Option<PacketBuf> {
        // Pair with the DMA's descriptor write-back before reading status.
        fence(Ordering::SeqCst);

        let info = self.fast_forward()?;

        // If the timestamp context has not landed yet, leave the frame queued
        // rather than deliver it without the timestamp that is about to arrive.
        #[cfg(feature = "ptp")]
        let timestamp = self.timestamp(&info)?;

        let len = info.len();
        let fits = self.buffers[self.index].as_ref().is_some_and(|b| len <= b.capacity());
        if !fits {
            #[cfg(feature = "defmt")]
            defmt::warn!("enet: dropping oversized rx frame of {} bytes", len);
            self.pop_current();
            return None;
        }

        let mut buf = self.buffers[self.index].take()?;
        buf.set_len(len);
        #[cfg(feature = "ptp")]
        {
            buf.meta_mut().timestamp = timestamp;
        }

        self.index = (self.index + 1) % self.descriptors.len();
        self.empty += 1;
        Some(buf)
    }

    /// Advance past context and invalid descriptors, returning the first
    /// usable frame's status, or `None` if there is none.
    fn fast_forward(&mut self) -> Option<RDesInfo> {
        loop {
            // An empty slot's descriptor is CPU-owned, but holds no frame.
            self.buffers[self.index].as_ref()?;

            let info = self.descriptors[self.index].info();
            if !info.available() {
                return None;
            }

            // Timestamp context descriptor for the previous frame.
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

            return Some(info);
        }
    }

    /// The ingress timestamp for the frame at the head of the ring.
    ///
    /// Three-state, and the distinction matters:
    ///
    /// - `Some(Some(ts))` - the following context descriptor holds a timestamp.
    /// - `Some(None)` - this frame has no timestamp, and never will.
    /// - `None` - the context descriptor has not been written back yet. The
    ///   caller must leave the frame queued and retry.
    #[cfg(feature = "ptp")]
    fn timestamp(&self, info: &RDesInfo) -> Option<Option<Timestamp>> {
        if !info.has_timestamp() {
            return Some(None);
        }

        let next = (self.index + 1) % self.descriptors.len();
        // An empty slot's descriptor holds stale write-back data. The DMA
        // writes the context descriptor there once the slot gets a buffer.
        self.buffers[next].as_ref()?;

        let context = &self.descriptors[next];
        let next_info = context.info();
        let timestamp = if next_info.context_available() {
            context.context_timestamp()
        } else if next_info.available() {
            // A normal frame followed instead, so the timestamp context never
            // came - do not stall the ring waiting for it.
            None
        } else {
            return None;
        };

        // The MAC also reports a missing timestamp as all zeros.
        Some(timestamp.filter(|ts| *ts != Timestamp::default()))
    }

    /// Give the descriptor at the head back to the DMA, keeping its buffer.
    fn pop_current(&mut self) {
        let index = self.index;
        debug_assert!(self.descriptors[index].info().available());

        if let Some(buf) = self.buffers[index].as_mut() {
            self.descriptors[index].set_ready(buf.storage_mut().as_mut_ptr());
            self.rearmed(index);
        }

        self.index = (index + 1) % self.descriptors.len();
    }
}

pub(crate) struct TDesRing<'a> {
    regs: Regs,
    descriptors: &'a mut [TDes],
    /// The buffer of each frame in flight, held until the DMA is done with it.
    buffers: &'a mut [Option<PacketBuf>],
    /// Egress timestamps waiting for `Driver::poll_tx_timestamp`.
    #[cfg(feature = "ptp")]
    timestamps: &'a mut DequeView<TxTimestamp>,
    /// Next descriptor to submit.
    index: usize,
    /// Submitted descriptors not yet reclaimed.
    in_flight: usize,
}

impl<'a> TDesRing<'a> {
    /// Initialise the TX ring and register it with the DMA.
    ///
    /// The DMA is still stopped at this point; the caller issues a barrier
    /// before starting it.
    pub(crate) fn new(
        regs: Regs,
        descriptors: &'a mut [TDes],
        buffers: &'a mut [Option<PacketBuf>],
        #[cfg(feature = "ptp")] timestamps: &'a mut DequeView<TxTimestamp>,
    ) -> Self {
        // One descriptor always stays free as the tail guard.
        assert!(descriptors.len() > 1);
        assert_eq!(descriptors.len(), buffers.len());

        #[cfg(feature = "ptp")]
        timestamps.clear();
        for (desc, buf) in descriptors.iter_mut().zip(buffers.iter_mut()) {
            *desc = TDes::new();
            *buf = None;
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
            #[cfg(feature = "ptp")]
            timestamps,
            index: 0,
            in_flight: 0,
        }
    }

    pub(crate) fn len(&self) -> usize {
        self.descriptors.len()
    }

    /// How many frames can be in flight at once: all but the tail guard.
    fn capacity(&self) -> usize {
        self.len() - 1
    }

    /// The oldest submitted descriptor not yet reclaimed.
    fn completion_index(&self) -> usize {
        (self.index + self.len() - self.in_flight) % self.len()
    }

    /// Reclaim every descriptor the DMA has finished with, freeing its buffer
    /// and queueing its egress timestamp, if it has one.
    pub(crate) fn reclaim(&mut self) {
        while self.in_flight > 0 {
            let i = self.completion_index();
            if !self.descriptors[i].available() {
                break;
            }

            // Observe the DMA's write-back before reading the timestamp or
            // releasing the buffer.
            fence(Ordering::Acquire);

            #[cfg(feature = "ptp")]
            if let Some(timestamp) = self.descriptors[i].timestamp()
                && let Some(buf) = self.buffers[i].as_ref()
            {
                let id = buf.meta().id;
                if self.timestamps.push_back(TxTimestamp { id, timestamp }).is_err() {
                    #[cfg(feature = "defmt")]
                    defmt::warn!("enet: dropping tx timestamp for packet id {}", id);
                }
            }

            // Dropping the buffer returns it to the pool.
            self.buffers[i] = None;
            self.in_flight -= 1;
        }
    }

    #[cfg(feature = "ptp")]
    pub(crate) fn poll_timestamp(&mut self) -> Option<TxTimestamp> {
        self.timestamps.pop_front()
    }

    /// Whether the next `transmit` will be accepted.
    pub(crate) fn can_transmit(&self) -> bool {
        self.in_flight < self.capacity() && self.descriptors[self.index].available()
    }

    /// Hand a frame to the DMA. `can_transmit` must have returned `true`.
    pub(crate) fn transmit(&mut self, buf: PacketBuf) {
        debug_assert!(self.can_transmit());
        let index = self.index;
        let td = &self.descriptors[index];
        let len = buf.len();

        td.set_frame(buf.as_ptr(), len);
        #[cfg(feature = "ptp")]
        if buf.meta().request_timestamp {
            td.request_timestamp();
        }

        // The DMA reads the frame from the buffer, so it must stay alive until
        // the descriptor is reclaimed.
        self.buffers[index] = Some(buf);

        // The frame and the rest of the descriptor must be visible before the
        // DMA is handed ownership, or it transmits whatever the buffer held
        // before.
        fence(Ordering::Release);
        td.set_owned(len);

        // The descriptor writes must land before the DMA is told to look at it.
        fence(Ordering::Release);

        self.index = (index + 1) % self.descriptors.len();
        self.in_flight += 1;

        // The tail pointer addresses the slot *after* the last one owned by the
        // DMA, and writing it is what restarts a suspended transmit engine.
        self.regs
            .dma_ch0_txdesc_tail_pointer()
            .write(|w| w.0 = dma_addr(&self.descriptors[self.index] as *const TDes));
    }
}
