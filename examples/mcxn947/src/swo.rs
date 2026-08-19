//! SWO capture for the CMSIS-DAP probe.
//!
//! SWO in UART mode is plain asynchronous NRZ, so this is an LPUART receiving
//! into a DMA ring — no new peripheral driver, and the same hardware path the
//! planned virtual COM ports will use.
//!
//! [`SwoControl`] is the sync surface `dap.rs` drives, deliberately `&self` with
//! interior mutability so it can be held as `&dyn SwoControl` alongside the
//! wire engine. [`Swo`] implements it over any [`SwoUart`], and the bytes
//! themselves are drained by whoever owns the DMA ring — see `dap_probe.rs`.
//!
//! # What probe-rs requires
//!
//! Advertising SWO makes probe-rs issue commands at *open* time, before any
//! target is touched: it queries `DAP_Info(0xFD)` for the trace buffer size and
//! parses it strictly as a u32, so a wrong length byte there breaks the whole
//! probe rather than just trace. It then drives Control(Stop), Transport, Mode,
//! Baudrate, Control(Start), and treats a **zero baud as a hard error**
//! (`SwoBaudrateNotConfigured`).
//!
//! Streaming transport is raw bytes on the trace endpoint with no framing of any
//! kind; the polled `DAP_SWO_Data` path carries the same bytes in-band. Both are
//! implemented, because probe-rs picks between them from the capability bits and
//! whether the endpoint exists.
//!
//! Manchester mode is not implemented. It is in the specification and probe-rs
//! supports it, but essentially no probe implements it, and the capability bit
//! stays clear so it is never requested.

#![allow(dead_code)]

use core::cell::{Cell, RefCell};

/// `DAP_SWO_Transport` values.
pub const TRANSPORT_NONE: u8 = 0;
/// Trace is read back in-band with `DAP_SWO_Data`.
pub const TRANSPORT_DATA: u8 = 1;
/// Trace is streamed on the dedicated bulk IN endpoint.
pub const TRANSPORT_ENDPOINT: u8 = 2;

/// `DAP_SWO_Mode` values.
pub const MODE_OFF: u8 = 0;
/// Asynchronous NRZ, which is what an LPUART receives.
pub const MODE_UART: u8 = 1;
/// Manchester — advertised nowhere and rejected here.
pub const MODE_MANCHESTER: u8 = 2;

/// `DAP_SWO_Status` bit: capture is running.
pub const STATUS_ACTIVE: u8 = 1 << 0;
/// `DAP_SWO_Status` bit: a line error (framing, noise, parity) was seen.
pub const STATUS_ERROR: u8 = 1 << 6;
/// `DAP_SWO_Status` bit: bytes were dropped.
pub const STATUS_OVERRUN: u8 = 1 << 7;

/// Bytes held for the polled transport.
///
/// Only used when the host chooses `DAP_SWO_Data` over the trace endpoint. The
/// DMA ring is the real buffer; this is a staging queue in front of it.
pub const POLL_QUEUE_LEN: usize = 4096;

/// Reprograms the SWO line rate, returning what was achieved, or `None` if no
/// divisor pair reaches it from the UART's functional clock.
///
/// A closure rather than a trait so that [`Swo`] carries no generic parameters
/// and can be handed to the dispatcher as a plain `&dyn SwoControl`.
pub type SetBaudrate<'u> = &'u dyn Fn(u32) -> Option<u32>;

/// A fixed-capacity byte queue that drops the oldest byte when full.
///
/// Dropping the oldest is the right way round for trace: the newest bytes are the
/// ones a profiler still has use for, and the overrun flag tells the host the
/// stream has a hole in it either way.
struct ByteQueue {
    buf: [u8; POLL_QUEUE_LEN],
    head: usize,
    len: usize,
}

impl ByteQueue {
    const fn new() -> Self {
        Self {
            buf: [0; POLL_QUEUE_LEN],
            head: 0,
            len: 0,
        }
    }

    fn len(&self) -> usize {
        self.len
    }

    /// Returns true if a byte had to be dropped to make room.
    fn push(&mut self, byte: u8) -> bool {
        let dropped = self.len == POLL_QUEUE_LEN;
        if dropped {
            self.head = (self.head + 1) % POLL_QUEUE_LEN;
            self.len -= 1;
        }
        self.buf[(self.head + self.len) % POLL_QUEUE_LEN] = byte;
        self.len += 1;
        dropped
    }

    fn pop(&mut self) -> Option<u8> {
        if self.len == 0 {
            return None;
        }
        let b = self.buf[self.head];
        self.head = (self.head + 1) % POLL_QUEUE_LEN;
        self.len -= 1;
        Some(b)
    }
}

/// The synchronous surface the CMSIS-DAP dispatcher drives.
///
/// Every method takes `&self` so that the dispatcher can hold this as a shared
/// `&dyn` reference while the capture loop independently owns the DMA ring.
pub trait SwoControl {
    /// Reported through `DAP_Info(0xFD)`, and what probe-rs sizes its polling
    /// interval from.
    fn trace_buffer_size(&self) -> u32;

    /// `DAP_SWO_Transport`. False if the transport is not available.
    fn set_transport(&self, transport: u8) -> bool;

    /// `DAP_SWO_Mode`. False for a mode this probe does not implement.
    fn set_mode(&self, mode: u8) -> bool;

    /// `DAP_SWO_Baudrate`. Returns the rate achieved, or 0 if none is — which
    /// probe-rs treats as a hard failure rather than a warning.
    fn set_baudrate(&self, baud: u32) -> u32;

    /// `DAP_SWO_Control(Start)`. False if mode or baud were never configured.
    fn start(&self) -> bool;

    /// `DAP_SWO_Control(Stop)`.
    fn stop(&self) -> bool;

    /// `DAP_SWO_Status`: the status byte and the count of bytes available.
    fn status(&self) -> (u8, u32);

    /// `DAP_SWO_Data`: drain up to `buf.len()` bytes of the polled transport.
    fn read(&self, buf: &mut [u8]) -> usize;
}

/// SWO capture state, shared between the dispatcher and the capture loop.
pub struct Swo<'u> {
    set_baud: SetBaudrate<'u>,
    /// Size of the DMA ring behind this, which is the honest answer to
    /// `DAP_Info(0xFD)`.
    ring_len: u32,
    running: Cell<bool>,
    mode: Cell<u8>,
    transport: Cell<u8>,
    /// Achieved baud, zero until configured.
    baud: Cell<u32>,
    /// Raised when the rate changed, so the capture loop discards anything the
    /// ring framed at the old rate before streaming again.
    stale: Cell<bool>,
    error: Cell<bool>,
    overrun: Cell<bool>,
    queue: RefCell<ByteQueue>,
}

impl<'u> Swo<'u> {
    /// `ring_len` is the length of the DMA buffer the capture loop reads.
    pub fn new(set_baud: SetBaudrate<'u>, ring_len: u32) -> Self {
        Self {
            set_baud,
            ring_len,
            running: Cell::new(false),
            mode: Cell::new(MODE_OFF),
            transport: Cell::new(TRANSPORT_NONE),
            baud: Cell::new(0),
            stale: Cell::new(false),
            error: Cell::new(false),
            overrun: Cell::new(false),
            queue: RefCell::new(ByteQueue::new()),
        }
    }

    /// Whether capture is running, for the capture loop.
    pub fn is_running(&self) -> bool {
        self.running.get()
    }

    /// Whether the host wants trace on the dedicated endpoint rather than in-band.
    pub fn is_streaming(&self) -> bool {
        self.transport.get() == TRANSPORT_ENDPOINT
    }

    /// Take and clear the "rate changed" flag. The capture loop uses this to
    /// drop bytes that were framed at the previous baud.
    pub fn take_stale(&self) -> bool {
        self.stale.replace(false)
    }

    /// Hand captured bytes to the polled transport.
    pub fn push_polled(&self, data: &[u8]) {
        let mut q = self.queue.borrow_mut();
        for &b in data {
            if q.push(b) {
                self.overrun.set(true);
            }
        }
    }

    /// Record that the ring overran, i.e. that trace bytes were lost.
    pub fn set_overrun(&self) {
        self.overrun.set(true);
    }

    /// Record a line error - framing, noise or parity on the SWO line.
    pub fn set_error(&self) {
        self.error.set(true);
    }

    /// The configured baud, for logging.
    pub fn baud(&self) -> u32 {
        self.baud.get()
    }
}

impl SwoControl for Swo<'_> {
    fn trace_buffer_size(&self) -> u32 {
        self.ring_len
    }

    fn set_transport(&self, transport: u8) -> bool {
        match transport {
            TRANSPORT_NONE | TRANSPORT_DATA | TRANSPORT_ENDPOINT => {
                self.transport.set(transport);
                true
            }
            _ => false,
        }
    }

    fn set_mode(&self, mode: u8) -> bool {
        match mode {
            MODE_OFF => {
                self.mode.set(mode);
                self.running.set(false);
                true
            }
            MODE_UART => {
                self.mode.set(mode);
                true
            }
            // Manchester is not implemented, and the capability bit for it is
            // clear, so this should never be asked for.
            _ => false,
        }
    }

    fn set_baudrate(&self, baud: u32) -> u32 {
        match (self.set_baud)(baud) {
            Some(actual) => {
                self.baud.set(actual);
                // Whatever is already buffered was framed at the old rate.
                self.stale.set(true);
                actual
            }
            None => {
                self.baud.set(0);
                0
            }
        }
    }

    fn start(&self) -> bool {
        if self.mode.get() != MODE_UART || self.baud.get() == 0 {
            return false;
        }
        self.error.set(false);
        self.overrun.set(false);
        self.stale.set(true);
        self.running.set(true);
        true
    }

    fn stop(&self) -> bool {
        self.running.set(false);
        true
    }

    fn status(&self) -> (u8, u32) {
        let mut status = 0;
        if self.running.get() {
            status |= STATUS_ACTIVE;
        }
        if self.error.get() {
            status |= STATUS_ERROR;
        }
        if self.overrun.get() {
            status |= STATUS_OVERRUN;
        }
        (status, self.queue.borrow().len() as u32)
    }

    fn read(&self, buf: &mut [u8]) -> usize {
        let mut q = self.queue.borrow_mut();
        let mut n = 0;
        while n < buf.len() {
            match q.pop() {
                Some(b) => {
                    buf[n] = b;
                    n += 1;
                }
                None => break,
            }
        }
        n
    }
}
