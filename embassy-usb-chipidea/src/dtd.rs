//! Device queue heads and transfer descriptors - the memory the controller
//! DMAs against.
//!
//! These are hardware-defined layouts, not registers, so they are plain
//! `#[repr(C)]` structs rather than fieldsets. Every access goes through
//! `read_volatile`/`write_volatile`: the controller writes back into the same
//! words the CPU wrote, and the compiler must not cache or reorder either
//! side. Volatile alone does not order these against *non-volatile* accesses
//! to the caller's data buffer, which is what the `compiler_fence`es in
//! `endpoint.rs` are for.

use core::cell::UnsafeCell;
use core::ptr::{addr_of, addr_of_mut, read_volatile, write_volatile};

/// Chipidea supports at most 8 endpoint pairs: `ENDPTPRIME` and friends have
/// one byte per direction. `DCCPARAMS.DEN` is the per-instance truth and is
/// checked against this at init.
pub const MAX_EP: usize = 8;

// ---------------------------------------------------------------------------
// dQH capability word
// ---------------------------------------------------------------------------

/// Transactions per (micro)frame, isochronous only. `1` elsewhere.
pub const CAPS_MULT_SHIFT: u32 = 30;
/// Zero-length termination *disable*.
///
/// Set on transmitting endpoints, where `embassy-usb` drives ZLPs explicitly
/// via `needs_zlp` and hardware auto-termination would duplicate them. It must
/// stay clear on a receiving non-control endpoint, which otherwise halts with
/// a data buffer error on the first packet it is sent.
pub const CAPS_ZLT: u32 = 1 << 29;
/// Maximum packet size, 11 bits.
pub const CAPS_MPS_SHIFT: u32 = 16;
/// Mask of the max-packet-size field, before shifting.
pub const CAPS_MPS_MASK: u32 = 0x7ff;
/// Interrupt on setup. Set on the endpoint 0 OUT queue head only.
pub const CAPS_IOS: u32 = 1 << 15;

// ---------------------------------------------------------------------------
// dTD / dQH-overlay token
// ---------------------------------------------------------------------------

/// Bytes remaining, 15 bits. The controller decrements this, so
/// `requested - total_bytes` is what actually moved.
pub const TOKEN_TOTAL_SHIFT: u32 = 16;
/// Mask of the total-bytes field, before shifting.
pub const TOKEN_TOTAL_MASK: u32 = 0x7fff;
/// Interrupt on complete.
pub const TOKEN_IOC: u32 = 1 << 15;
/// Multiplier override, isochronous only. Zero means no override.
pub const TOKEN_MULTO_SHIFT: u32 = 10;

/// The controller owns this descriptor.
pub const STATUS_ACTIVE: u32 = 0x80;
/// The endpoint halted - a stall, or an error the controller could not retry.
pub const STATUS_HALTED: u32 = 0x40;
/// The buffer could not keep up, or was too small for the packet received.
pub const STATUS_DATA_BUFFER_ERROR: u32 = 0x20;
/// CRC, timeout, bit-stuff or PID error on the wire.
pub const STATUS_TRANSACTION_ERROR: u32 = 0x08;
/// Any status bit that means the transfer did not complete cleanly.
pub const STATUS_ERROR_MASK: u32 = STATUS_HALTED | STATUS_DATA_BUFFER_ERROR | STATUS_TRANSACTION_ERROR;

/// Next-pointer terminate bit: there is no following descriptor.
pub const TERMINATE: u32 = 1;

/// How many bytes a single dTD can describe starting at `addr`.
///
/// A dTD carries five buffer pointers. The first is a full address, so its
/// low 12 bits eat into the first page; the rest contribute only a page base.
/// The full 20 KiB is therefore only available to a page-aligned buffer.
#[inline]
pub const fn max_dtd_len(addr: u32) -> usize {
    5 * 4096 - (addr as usize & 0xfff)
}

// ---------------------------------------------------------------------------
// Layouts
// ---------------------------------------------------------------------------

/// Device queue head.
///
/// Hardware indexes the array as `2 * endpoint + direction`, with OUT at the
/// even slot. `next_dtd` through `buffer` form the "transfer overlay" the
/// controller copies the active descriptor into, so those words are shared
/// between software and hardware and are only safe to touch when no transfer
/// is primed.
#[repr(C, align(64))]
pub struct Dqh {
    /// Max packet size, mult, ZLT and IOS. See the `CAPS_*` constants.
    pub caps: u32,
    /// Hardware-owned: the descriptor currently being executed.
    pub cur_dtd: u32,
    /// Head of the descriptor list. Software writes this to arm a transfer.
    pub next_dtd: u32,
    /// Overlay status word, mirroring the active descriptor's token.
    pub token: u32,
    /// Overlay buffer pointers.
    pub buffer: [u32; 5],
    /// Reserved.
    pub reserved: u32,
    /// Where the controller deposits the 8 SETUP bytes. Endpoint 0 OUT only.
    pub setup: [u32; 2],
    /// Padding to the 64-byte stride hardware indexes this array by.
    pub _pad: [u32; 4],
}

// `::core::assert!` explicitly: `fmt.rs` shadows `assert!` with defmt's,
// which is not const-evaluable.
const _: () = ::core::assert!(core::mem::size_of::<Dqh>() == 64);
// `::core::assert!` explicitly: `fmt.rs` shadows `assert!` with defmt's,
// which is not const-evaluable.
const _: () = ::core::assert!(core::mem::align_of::<Dqh>() == 64);

/// Device transfer descriptor.
#[repr(C, align(32))]
pub struct Dtd {
    /// Next descriptor, or [`TERMINATE`].
    pub next: u32,
    /// Byte count, interrupt-on-complete, and status. See the `TOKEN_*` and
    /// `STATUS_*` constants.
    pub token: u32,
    /// Buffer pointers: `[0]` is a full address, the rest are page bases.
    pub buffer: [u32; 5],
    /// Reserved.
    pub _reserved: u32,
}

// `::core::assert!` explicitly: `fmt.rs` shadows `assert!` with defmt's,
// which is not const-evaluable.
const _: () = ::core::assert!(core::mem::size_of::<Dtd>() == 32);
// `::core::assert!` explicitly: `fmt.rs` shadows `assert!` with defmt's,
// which is not const-evaluable.
const _: () = ::core::assert!(core::mem::align_of::<Dtd>() == 32);

impl Dqh {
    /// An all-zero queue head, for `const` array initialisation.
    pub const fn new() -> Self {
        Self {
            caps: 0,
            cur_dtd: 0,
            next_dtd: 0,
            token: 0,
            buffer: [0; 5],
            reserved: 0,
            setup: [0; 2],
            _pad: [0; 4],
        }
    }

    /// Program the capability word and retire any stale overlay.
    ///
    /// # Safety
    /// No transfer may be primed on this endpoint.
    #[inline]
    pub unsafe fn configure(p: *mut Dqh, max_packet_size: u16, ios: bool, mult: u8, zlt_disable: bool) {
        let caps = ((mult as u32 & 3) << CAPS_MULT_SHIFT)
            | if zlt_disable { CAPS_ZLT } else { 0 }
            | ((max_packet_size as u32 & CAPS_MPS_MASK) << CAPS_MPS_SHIFT)
            | if ios { CAPS_IOS } else { 0 };

        write_volatile(addr_of_mut!((*p).caps), caps);
        Self::retire(p);
    }

    /// Point the queue head at `dtd_dma` and clear the overlay status, which
    /// is what makes the controller fetch it on the next prime.
    ///
    /// # Safety
    /// No transfer may be primed on this endpoint.
    #[inline]
    pub unsafe fn arm(p: *mut Dqh, dtd_dma: u32) {
        write_volatile(addr_of_mut!((*p).next_dtd), dtd_dma);
        // Clearing the overlay token drops both ACTIVE and any sticky HALTED
        // left by a previous stall.
        write_volatile(addr_of_mut!((*p).token), 0);
    }

    /// Drop any descriptor the queue head still refers to.
    ///
    /// # Safety
    /// No transfer may be primed on this endpoint - flush first.
    #[inline]
    pub unsafe fn retire(p: *mut Dqh) {
        write_volatile(addr_of_mut!((*p).next_dtd), TERMINATE);
        write_volatile(addr_of_mut!((*p).token), 0);
    }

    /// Read the 8 SETUP bytes the controller deposited.
    ///
    /// The caller is responsible for the `USBCMD.SUTW` tripwire around this:
    /// a newer SETUP can overwrite the landing zone mid-read.
    ///
    /// # Safety
    /// `p` must point at the endpoint 0 OUT queue head.
    #[inline]
    pub unsafe fn read_setup(p: *const Dqh) -> [u32; 2] {
        [
            read_volatile(addr_of!((*p).setup[0])),
            read_volatile(addr_of!((*p).setup[1])),
        ]
    }
}

impl Default for Dqh {
    fn default() -> Self {
        Self::new()
    }
}

impl Dtd {
    /// An all-zero transfer descriptor, for `const` array initialisation.
    pub const fn new() -> Self {
        Self {
            next: 0,
            token: 0,
            buffer: [0; 5],
            _reserved: 0,
        }
    }

    /// Describe a transfer of `len` bytes to or from bus address `dma_addr`.
    ///
    /// `len` must not exceed [`max_dtd_len`] for this address.
    ///
    /// # Safety
    /// The controller must not currently own this descriptor.
    #[inline]
    pub unsafe fn arm(p: *mut Dtd, dma_addr: u32, len: usize, ioc: bool, mult_override: u8) {
        debug_assert!(len <= max_dtd_len(dma_addr));

        write_volatile(addr_of_mut!((*p).next), TERMINATE);

        // buffer[0] carries the byte offset; the rest are page bases the
        // controller advances through as the transfer crosses boundaries.
        //
        // Only the pages this transfer actually reaches are filled. Pointing
        // the unused ones past the end of the buffer - and, for a buffer near
        // the top of RAM, past the end of RAM itself - makes the controller
        // fault when it prefetches them, which surfaces as a data buffer
        // error rather than anything that names the real cause.
        write_volatile(addr_of_mut!((*p).buffer[0]), dma_addr);

        let page0 = dma_addr & !0xfff;
        let pages = (((dma_addr as usize & 0xfff) + len) + 4095) / 4096;
        let mut i = 1;
        while i < 5 {
            // Unused slots repeat the first page rather than continuing past
            // the buffer or reading as zero: both are addresses the controller
            // may not be able to reach, and it need never dereference these.
            let v = if i < pages {
                page0.wrapping_add(i as u32 * 4096)
            } else {
                page0
            };
            write_volatile(addr_of_mut!((*p).buffer[i]), v);
            i += 1;
        }

        write_volatile(addr_of_mut!((*p)._reserved), 0);

        // The token, and so ACTIVE, goes last: it is what hands ownership over.
        let token = ((len as u32 & TOKEN_TOTAL_MASK) << TOKEN_TOTAL_SHIFT)
            | if ioc { TOKEN_IOC } else { 0 }
            | ((mult_override as u32 & 3) << TOKEN_MULTO_SHIFT)
            | STATUS_ACTIVE;
        write_volatile(addr_of_mut!((*p).token), token);
    }

    /// The raw token, for status and residue.
    ///
    /// # Safety
    /// `p` must point at a live descriptor.
    #[inline]
    pub unsafe fn token(p: *const Dtd) -> u32 {
        read_volatile(addr_of!((*p).token))
    }

    /// Bytes the controller did *not* move.
    #[inline]
    pub const fn remaining(token: u32) -> usize {
        ((token >> TOKEN_TOTAL_SHIFT) & TOKEN_TOTAL_MASK) as usize
    }
}

impl Default for Dtd {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Arena
// ---------------------------------------------------------------------------

/// The queue-head array plus one transfer descriptor per endpoint direction.
///
/// 2048-byte aligned because `ENDPTLISTADDR` cannot represent the low 11 bits
/// of the queue-head list address. Sized for [`MAX_EP`] rather than the
/// instance's actual endpoint count: array lengths computed from a const
/// generic need `generic_const_exprs`, and the waste is at most ~1 KiB.
///
/// One descriptor per direction is enough because `EndpointIn::write` and
/// `EndpointOut::read` take `&mut self`, so a direction can only have one
/// transfer in flight.
#[repr(C, align(2048))]
pub struct DmaArena {
    /// Queue heads, indexed by [`crate::dqh_index`]. Must come first: its
    /// address is what goes into `ENDPTLISTADDR`.
    pub dqh: [UnsafeCell<Dqh>; 2 * MAX_EP],
    /// One transfer descriptor per endpoint direction, same indexing.
    pub dtd: [UnsafeCell<Dtd>; 2 * MAX_EP],
}

// `::core::assert!` explicitly: `fmt.rs` shadows `assert!` with defmt's,
// which is not const-evaluable.
const _: () = ::core::assert!(core::mem::align_of::<DmaArena>() == 2048);

// SAFETY: each slot belongs to one endpoint direction, and the driver hands
// out at most one endpoint object per direction, whose `&mut self` methods are
// the only things that write it. The interrupt handler never touches the arena
// - it works purely from `ENDPTCOMPLETE`. Access races against the controller
// are ordered by the ACTIVE bit and the fences around priming.
unsafe impl Send for DmaArena {}
unsafe impl Sync for DmaArena {}

impl DmaArena {
    /// An arena with every descriptor zeroed, for `const` initialisation.
    pub const fn new() -> Self {
        Self {
            dqh: [const { UnsafeCell::new(Dqh::new()) }; 2 * MAX_EP],
            dtd: [const { UnsafeCell::new(Dtd::new()) }; 2 * MAX_EP],
        }
    }
}

impl Default for DmaArena {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dtd_len_limits() {
        // A page-aligned buffer gets the full five pages.
        assert_eq!(max_dtd_len(0x2000_0000), 20480);
        // One byte in costs one byte of reach.
        assert_eq!(max_dtd_len(0x2000_0001), 20479);
        // Worst case: all but one byte of the first page is unusable.
        assert_eq!(max_dtd_len(0x2000_0fff), 5 * 4096 - 4095);
    }

    #[test]
    fn dtd_buffer_pages() {
        let d = UnsafeCell::new(Dtd::new());
        unsafe { Dtd::arm(d.get(), 0x2000_1234, 8192, true, 0) };
        let d = unsafe { &*d.get() };

        // 0x234 into the page plus 8192 bytes spans three pages, so only two
        // continuation pointers are needed. The rest must stay zero rather
        // than pointing past the buffer - see `arm`.
        assert_eq!(d.buffer[0], 0x2000_1234);
        assert_eq!(d.buffer[1], 0x2000_2000);
        assert_eq!(d.buffer[2], 0x2000_3000);
        assert_eq!(d.buffer[3], 0x2000_1000);
        assert_eq!(d.buffer[4], 0x2000_1000);
        assert_eq!(d.next, TERMINATE);

        assert_eq!(d.token & 0xff, STATUS_ACTIVE);
        assert!(d.token & TOKEN_IOC != 0);
        assert_eq!(Dtd::remaining(d.token), 8192);
    }

    /// A packet-sized buffer near the top of RAM must not generate page
    /// pointers past the end of it - the controller prefetches them and
    /// reports a data buffer error.
    #[test]
    fn single_page_transfer_leaves_pages_zero() {
        let d = UnsafeCell::new(Dtd::new());
        // 64 bytes at the top of a 256 KiB RAM at 0x20000000.
        unsafe { Dtd::arm(d.get(), 0x2003_f62c, 64, true, 0) };
        let d = unsafe { &*d.get() };

        assert_eq!(d.buffer[0], 0x2003_f62c);
        for (i, b) in d.buffer.iter().enumerate().skip(1) {
            assert_eq!(*b, 0x2003_f000, "buffer[{i}] must stay within the buffer's page");
        }
    }

    #[test]
    fn residue_is_shortfall() {
        // A 512-byte request that moved 200 bytes reports 312 remaining.
        let token = (312u32 << TOKEN_TOTAL_SHIFT) | TOKEN_IOC;
        assert_eq!(512 - Dtd::remaining(token), 200);
    }

    #[test]
    fn dqh_caps_layout() {
        let q = UnsafeCell::new(Dqh::new());
        unsafe { Dqh::configure(q.get(), 512, true, 1, true) };
        let q = unsafe { &*q.get() };

        assert_eq!((q.caps >> CAPS_MPS_SHIFT) & CAPS_MPS_MASK, 512);
        assert!(q.caps & CAPS_IOS != 0);
        assert!(q.caps & CAPS_ZLT != 0, "auto-ZLT must be disabled");
        assert_eq!(q.caps >> CAPS_MULT_SHIFT, 1);
        assert_eq!(q.next_dtd, TERMINATE);
    }

    /// The arena's alignment is what `ENDPTLISTADDR` requires.
    #[test]
    fn arena_alignment() {
        let a = DmaArena::new();
        assert_eq!(core::ptr::addr_of!(a) as usize & 0x7ff, 0);
    }
}
