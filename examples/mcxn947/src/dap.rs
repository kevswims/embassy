//! A CMSIS-DAP v2 command dispatcher over [`SwdBus`].
//!
//! Implements the commands probe-rs actually issues, and nothing else. Verified
//! against probe-rs's own parsers in `probe-rs/src/probe/cmsisdap/`, which is
//! also where the framing rules below come from:
//!
//! - Every command produces **exactly one** response whose first byte is the
//!   command ID. probe-rs only checks that byte, so one dropped or extra response
//!   desynchronises the stream permanently. Unknown commands answer `0xFF`.
//! - `DAP_Info` length bytes are checked exactly: 1 for packet count, 2 for
//!   packet size, and strings carry `len + 1` with a NUL terminator.
//! - `DAP_Transfer` reports the number of transfers **attempted**, with response
//!   data present only for the ones that succeeded. probe-rs assumes every
//!   transfer before the last one returned OK, so the batch must stop at the
//!   first failure.
//!
//! Commands deliberately absent, having been confirmed unreachable from
//! probe-rs: `DAP_TransferAbort`, `DAP_WriteABORT`, `DAP_Delay`,
//! `DAP_JTAG_IDCODE`, `DAP_SWD_Sequence`, `DAP_QueueCommands` /
//! `DAP_ExecuteCommands`, and the SWO and UART families. Value match, match mask
//! and test-domain timestamps are parsed but refused — see
//! [`crate::swd::SwdBus::transfer`].

#![allow(dead_code)]

use crate::swd::{A2, A3, ACK_OK, APNDP, MATCH_MASK, MATCH_VALUE, PROTOCOL_ERROR, RDBUFF_READ, RNW, SwdBus, TIMESTAMP};
use crate::swo::{SwoControl, TRANSPORT_ENDPOINT};

/// `DAP_Info`.
const ID_INFO: u8 = 0x00;
/// `DAP_HostStatus`.
const ID_HOST_STATUS: u8 = 0x01;
/// `DAP_Connect`.
const ID_CONNECT: u8 = 0x02;
/// `DAP_Disconnect`.
const ID_DISCONNECT: u8 = 0x03;
/// `DAP_TransferConfigure`.
const ID_TRANSFER_CONFIGURE: u8 = 0x04;
/// `DAP_Transfer`.
const ID_TRANSFER: u8 = 0x05;
/// `DAP_TransferBlock`.
const ID_TRANSFER_BLOCK: u8 = 0x06;
/// `DAP_SWJ_Pins`.
const ID_SWJ_PINS: u8 = 0x10;
/// `DAP_SWJ_Clock`.
const ID_SWJ_CLOCK: u8 = 0x11;
/// `DAP_SWJ_Sequence`.
const ID_SWJ_SEQUENCE: u8 = 0x12;
/// `DAP_SWD_Configure`.
const ID_SWD_CONFIGURE: u8 = 0x13;
/// `DAP_SWO_Transport`.
const ID_SWO_TRANSPORT: u8 = 0x17;
/// `DAP_SWO_Mode`.
const ID_SWO_MODE: u8 = 0x18;
/// `DAP_SWO_Baudrate`.
const ID_SWO_BAUDRATE: u8 = 0x19;
/// `DAP_SWO_Control`.
const ID_SWO_CONTROL: u8 = 0x1A;
/// `DAP_SWO_Status`.
const ID_SWO_STATUS: u8 = 0x1B;
/// `DAP_SWO_Data`.
const ID_SWO_DATA: u8 = 0x1C;
/// `DAP_SWO_ExtendedStatus`.
const ID_SWO_EXTENDED_STATUS: u8 = 0x1E;
/// The response ID for a command this firmware does not implement.
const ID_INVALID: u8 = 0xFF;

/// `DAP_Status` OK.
const DAP_OK: u8 = 0x00;
/// `DAP_Status` error.
const DAP_ERROR: u8 = 0xFF;

/// `DAP_Connect` port values.
const PORT_DEFAULT: u8 = 0;
const PORT_SWD: u8 = 1;

/// `DAP_Info(0xF0)` bit 0. Must be set or probe-rs refuses to select SWD.
const CAP_SWD: u8 = 1 << 0;
/// Bit 2: SWO in UART mode.
const CAP_SWO_UART: u8 = 1 << 2;
/// Bit 6: trace streamed on the dedicated endpoint rather than polled in-band.
///
/// This and the endpoint have to agree. Setting it without the endpoint, or the
/// endpoint without it, silently falls back to polled `DAP_SWO_Data`.
const CAP_SWO_STREAMING: u8 = 1 << 6;

/// Responses in flight at once. One, because this dispatcher is synchronous.
const PACKET_COUNT: u8 = 1;

const INFO_VENDOR: &str = "Embassy";
const INFO_PRODUCT: &str = "MCXN947 CMSIS-DAP";
const INFO_PROTOCOL_VERSION: &str = "2.1.0";

/// CMSIS-DAP command dispatcher.
pub struct Dap<'d, B: SwdBus> {
    bus: B,
    /// `None` leaves the SWO capability bits clear, so probe-rs never asks.
    swo: Option<&'d dyn SwoControl>,
    /// Whether the trace endpoint exists. Only meaningful with `swo` set.
    swo_streaming: bool,
    /// Advertised and actual bulk endpoint size; must match the descriptor.
    packet_size: u16,
    serial: &'static str,
    connected: bool,
    /// WAIT responses to retry before giving up, from `DAP_TransferConfigure`.
    wait_retry: u16,
    /// Last rate `DAP_SWJ_Clock` actually achieved.
    clock_hz: u32,
}

impl<'d, B: SwdBus> Dap<'d, B> {
    /// Wrap a bus. `packet_size` must equal the bulk endpoint's size.
    ///
    /// `swo` enables the SWO capability bits and command set; `swo_streaming`
    /// says whether the trace endpoint was actually built, which decides whether
    /// the streaming bit is advertised alongside them.
    pub fn new(
        bus: B,
        packet_size: u16,
        serial: &'static str,
        swo: Option<&'d dyn SwoControl>,
        swo_streaming: bool,
    ) -> Self {
        Self {
            bus,
            swo,
            swo_streaming,
            packet_size,
            serial,
            connected: false,
            wait_retry: 100,
            clock_hz: 0,
        }
    }

    /// The rate the bus last reported for a `DAP_SWJ_Clock` request.
    pub fn clock_hz(&self) -> u32 {
        self.clock_hz
    }

    /// Handle one command packet, writing its response into `resp`.
    ///
    /// Returns the response length, which is never zero for a well-formed
    /// request: silence desynchronises the host permanently.
    pub fn process(&mut self, req: &[u8], resp: &mut [u8]) -> usize {
        if req.is_empty() || resp.is_empty() {
            return 0;
        }

        let cmd = req[0];
        let payload = &req[1..];
        let out = &mut resp[1..];

        let len = match cmd {
            ID_INFO => self.info(payload, out),
            ID_HOST_STATUS => Self::host_status(out),
            ID_CONNECT => self.connect(payload, out),
            ID_DISCONNECT => self.disconnect(out),
            ID_TRANSFER_CONFIGURE => self.transfer_configure(payload, out),
            ID_TRANSFER => self.transfer(payload, out),
            ID_TRANSFER_BLOCK => self.transfer_block(payload, out),
            ID_SWJ_PINS => self.swj_pins(payload, out),
            ID_SWJ_CLOCK => self.swj_clock(payload, out),
            ID_SWJ_SEQUENCE => self.swj_sequence(payload, out),
            ID_SWD_CONFIGURE => self.swd_configure(payload, out),
            ID_SWO_TRANSPORT => self.swo_transport(payload, out),
            ID_SWO_MODE => self.swo_mode(payload, out),
            ID_SWO_BAUDRATE => self.swo_baudrate(payload, out),
            ID_SWO_CONTROL => self.swo_control(payload, out),
            ID_SWO_STATUS => self.swo_status(out),
            ID_SWO_DATA => self.swo_data(payload, out),
            ID_SWO_EXTENDED_STATUS => self.swo_extended_status(out),
            _ => {
                resp[0] = ID_INVALID;
                return 1;
            }
        };

        resp[0] = cmd;
        1 + len
    }

    /// Length-prefixed, NUL-terminated string, as `DAP_Info` defines it. The
    /// length byte counts the terminator.
    fn info_string(out: &mut [u8], s: &str) -> usize {
        let bytes = s.as_bytes();
        if out.len() < bytes.len() + 2 {
            out[0] = 0;
            return 1;
        }
        out[0] = bytes.len() as u8 + 1;
        out[1..1 + bytes.len()].copy_from_slice(bytes);
        out[1 + bytes.len()] = 0;
        2 + bytes.len()
    }

    fn info(&mut self, req: &[u8], out: &mut [u8]) -> usize {
        match req.first().copied().unwrap_or(0) {
            0x01 => Self::info_string(out, INFO_VENDOR),
            0x02 => Self::info_string(out, INFO_PRODUCT),
            0x03 => Self::info_string(out, self.serial),
            0x04 => Self::info_string(out, INFO_PROTOCOL_VERSION),
            0xF0 => {
                let mut caps = CAP_SWD;
                if self.swo.is_some() {
                    caps |= CAP_SWO_UART;
                    if self.swo_streaming {
                        caps |= CAP_SWO_STREAMING;
                    }
                }
                out[0] = 1;
                out[1] = caps;
                2
            }
            // SWO trace buffer size. probe-rs asks for this while *opening* the
            // probe, as soon as either SWO capability bit is set, and parses it
            // strictly as a u32 - so a wrong length byte here breaks SWD too.
            0xFD => match self.swo {
                Some(swo) => {
                    out[0] = 4;
                    out[1..5].copy_from_slice(&swo.trace_buffer_size().to_le_bytes());
                    5
                }
                None => {
                    out[0] = 0;
                    1
                }
            },
            0xFE => {
                out[0] = 1;
                out[1] = PACKET_COUNT;
                2
            }
            0xFF => {
                out[0] = 2;
                out[1..3].copy_from_slice(&self.packet_size.to_le_bytes());
                3
            }
            // Everything else, including the test-domain timer, is absent. A zero
            // length is the defined way to say so.
            _ => {
                out[0] = 0;
                1
            }
        }
    }

    /// Host connect/running indication. Nothing is wired to it yet.
    fn host_status(out: &mut [u8]) -> usize {
        out[0] = DAP_OK;
        1
    }

    fn connect(&mut self, req: &[u8], out: &mut [u8]) -> usize {
        let port = req.first().copied().unwrap_or(PORT_DEFAULT);
        out[0] = if port == PORT_DEFAULT || port == PORT_SWD {
            self.connected = true;
            PORT_SWD
        } else {
            // JTAG is a later phase, and capabilities do not advertise it.
            0
        };
        1
    }

    fn disconnect(&mut self, out: &mut [u8]) -> usize {
        self.connected = false;
        out[0] = DAP_OK;
        1
    }

    fn transfer_configure(&mut self, req: &[u8], out: &mut [u8]) -> usize {
        if req.len() < 5 {
            out[0] = DAP_ERROR;
            return 1;
        }
        self.bus.set_idle_cycles(req[0]);
        self.wait_retry = u16::from_le_bytes([req[1], req[2]]);
        // Match retry is only meaningful for value-match reads, which are refused.
        out[0] = DAP_OK;
        1
    }

    fn swj_clock(&mut self, req: &[u8], out: &mut [u8]) -> usize {
        if req.len() < 4 {
            out[0] = DAP_ERROR;
            return 1;
        }
        let hz = u32::from_le_bytes([req[0], req[1], req[2], req[3]]);
        self.clock_hz = self.bus.set_clock_hz(hz);
        out[0] = DAP_OK;
        1
    }

    fn swd_configure(&mut self, req: &[u8], out: &mut [u8]) -> usize {
        let cfg = req.first().copied().unwrap_or(0);
        self.bus.set_turnaround((cfg & 0x03) + 1);
        self.bus.set_data_phase(cfg & 0x04 != 0);
        out[0] = DAP_OK;
        1
    }

    fn swj_sequence(&mut self, req: &[u8], out: &mut [u8]) -> usize {
        let Some(&count_byte) = req.first() else {
            out[0] = DAP_ERROR;
            return 1;
        };
        // Zero means 256 bits, which is the maximum.
        let bits = if count_byte == 0 { 256 } else { count_byte as usize };
        let bytes = bits.div_ceil(8);
        if req.len() < 1 + bytes {
            out[0] = DAP_ERROR;
            return 1;
        }
        self.bus.swj_sequence(bits, &req[1..1 + bytes]);
        out[0] = DAP_OK;
        1
    }

    fn swj_pins(&mut self, req: &[u8], out: &mut [u8]) -> usize {
        if req.len() < 6 {
            out[0] = 0;
            return 1;
        }
        let wait_us = u32::from_le_bytes([req[2], req[3], req[4], req[5]]);
        out[0] = self.bus.swj_pins(req[0], req[1], wait_us);
        1
    }

    /// One SWD transaction, retrying while the target answers WAIT.
    fn wire(&mut self, request: u8, data: Option<u32>) -> Result<u32, u8> {
        let mut retries = self.wait_retry;
        loop {
            match self.bus.transfer(request, data) {
                Err(crate::swd::ACK_WAIT) if retries > 0 => retries -= 1,
                other => return other,
            }
        }
    }

    /// `DAP_Transfer`: a list of independent transactions.
    ///
    /// The posted-read handling is the subtle part. In SWD an AP read returns the
    /// *previous* read's value, so an AP read is issued once to post it and its
    /// data collected either by the next read of the same AP register or by a read
    /// of `DP.RDBUFF`. That keeps n AP reads to n+1 transactions while still
    /// handing back values in the order the host asked for them.
    fn transfer(&mut self, req: &[u8], out: &mut [u8]) -> usize {
        if req.len() < 2 || out.len() < 2 {
            if out.len() >= 2 {
                out[0] = 0;
                out[1] = 0;
                return 2;
            }
            return 0;
        }

        let count = req[1] as usize;
        let mut r = 2;
        let mut w = 2;
        let mut executed = 0usize;
        let mut last = ACK_OK;
        let mut post_read = false;
        let mut posted_reg = 0u8;

        'transfers: for _ in 0..count {
            if r >= req.len() {
                break;
            }
            let request = req[r];
            r += 1;

            // An operand follows every write, and every read that carries a value
            // match. Consume it before anything else can fail, so that refusing a
            // transfer cannot desynchronise the rest of the packet.
            let is_read = request & RNW != 0;
            let operand = if !is_read || request & MATCH_VALUE != 0 {
                if r + 4 > req.len() {
                    break;
                }
                let v = u32::from_le_bytes([req[r], req[r + 1], req[r + 2], req[r + 3]]);
                r += 4;
                Some(v)
            } else {
                None
            };

            if request & (MATCH_VALUE | MATCH_MASK | TIMESTAMP) != 0 {
                executed += 1;
                last = PROTOCOL_ERROR;
                break;
            }

            let is_ap = request & APNDP != 0;
            let reg = request & (APNDP | A2 | A3);

            // A repeat read of the same AP register both collects the posted value
            // and posts the next one.
            if is_read && is_ap && post_read && posted_reg == reg {
                match self.wire(request, None) {
                    Ok(v) => {
                        if !write_word(out, &mut w, v) {
                            last = PROTOCOL_ERROR;
                            break 'transfers;
                        }
                    }
                    Err(e) => {
                        executed += 1;
                        last = e;
                        break 'transfers;
                    }
                }
                executed += 1;
                last = ACK_OK;
                continue;
            }

            // Anything else has to collect an outstanding posted read first. A
            // failure here belongs to the *previous* transfer, which is why
            // `executed` is not incremented: it already counts that transfer, and
            // reporting it as the failing one keeps the host's data alignment.
            if post_read {
                match self.wire(RDBUFF_READ, None) {
                    Ok(v) => {
                        post_read = false;
                        if !write_word(out, &mut w, v) {
                            last = PROTOCOL_ERROR;
                            break 'transfers;
                        }
                    }
                    Err(e) => {
                        last = e;
                        break 'transfers;
                    }
                }
            }

            if is_read && is_ap {
                // Post it; the value arrives later.
                match self.wire(request, None) {
                    Ok(_) => {
                        post_read = true;
                        posted_reg = reg;
                    }
                    Err(e) => {
                        executed += 1;
                        last = e;
                        break 'transfers;
                    }
                }
            } else {
                match self.wire(request, operand) {
                    Ok(v) => {
                        if is_read && !write_word(out, &mut w, v) {
                            last = PROTOCOL_ERROR;
                            break 'transfers;
                        }
                    }
                    Err(e) => {
                        executed += 1;
                        last = e;
                        break 'transfers;
                    }
                }
            }

            executed += 1;
            last = ACK_OK;
        }

        // Collect a read still posted when the list ran out. It belongs to the
        // last transfer, so a failure here reports against it without changing the
        // count.
        if post_read && last == ACK_OK {
            match self.wire(RDBUFF_READ, None) {
                Ok(v) => {
                    if !write_word(out, &mut w, v) {
                        last = PROTOCOL_ERROR;
                    }
                }
                Err(e) => last = e,
            }
        }

        self.bus.flush();

        out[0] = executed as u8;
        out[1] = last;
        w
    }

    /// `DAP_TransferBlock`: many accesses to one register, which is what carries
    /// bulk memory reads and writes.
    fn transfer_block(&mut self, req: &[u8], out: &mut [u8]) -> usize {
        if req.len() < 4 || out.len() < 3 {
            if out.len() >= 3 {
                out[0] = 0;
                out[1] = 0;
                out[2] = 0;
                return 3;
            }
            return 0;
        }

        let count = u16::from_le_bytes([req[1], req[2]]) as usize;
        let request = req[3];
        let is_read = request & RNW != 0;
        let is_ap = request & APNDP != 0;

        let mut done = 0usize;
        let mut last = ACK_OK;
        let mut w = 3;
        let mut r = 4;

        if request & (MATCH_VALUE | MATCH_MASK | TIMESTAMP) != 0 {
            last = PROTOCOL_ERROR;
        } else if is_read {
            // Post the first AP read. DP reads are not posted, so they need none.
            let posted = if is_ap {
                match self.wire(request, None) {
                    Ok(_) => true,
                    Err(e) => {
                        last = e;
                        false
                    }
                }
            } else {
                true
            };

            if posted {
                for i in 0..count {
                    // Every read but the last returns the previous value and posts
                    // the next; the last is collected from RDBUFF.
                    let rd = if is_ap && i + 1 == count { RDBUFF_READ } else { request };
                    match self.wire(rd, None) {
                        Ok(v) => {
                            if !write_word(out, &mut w, v) {
                                last = PROTOCOL_ERROR;
                                break;
                            }
                            done += 1;
                        }
                        Err(e) => {
                            last = e;
                            break;
                        }
                    }
                }
            }
        } else {
            for _ in 0..count {
                if r + 4 > req.len() {
                    break;
                }
                let v = u32::from_le_bytes([req[r], req[r + 1], req[r + 2], req[r + 3]]);
                r += 4;
                match self.wire(request, Some(v)) {
                    Ok(_) => done += 1,
                    Err(e) => {
                        last = e;
                        break;
                    }
                }
            }
        }

        self.bus.flush();

        out[0..2].copy_from_slice(&(done as u16).to_le_bytes());
        out[2] = last;
        w
    }

    fn swo_transport(&mut self, req: &[u8], out: &mut [u8]) -> usize {
        let transport = req.first().copied().unwrap_or(0);
        let ok = match self.swo {
            // Refusing the endpoint transport when there is no endpoint makes
            // probe-rs fall back to polling rather than wait on a dead pipe.
            Some(_) if transport == TRANSPORT_ENDPOINT && !self.swo_streaming => false,
            Some(swo) => swo.set_transport(transport),
            None => false,
        };
        out[0] = if ok { DAP_OK } else { DAP_ERROR };
        1
    }

    fn swo_mode(&mut self, req: &[u8], out: &mut [u8]) -> usize {
        let mode = req.first().copied().unwrap_or(0);
        let ok = matches!(self.swo, Some(swo) if swo.set_mode(mode));
        out[0] = if ok { DAP_OK } else { DAP_ERROR };
        1
    }

    /// Returns the rate achieved. Zero is a hard error to probe-rs rather than a
    /// warning, so it is only reported when nothing was reachable at all.
    fn swo_baudrate(&mut self, req: &[u8], out: &mut [u8]) -> usize {
        let requested = if req.len() >= 4 {
            u32::from_le_bytes([req[0], req[1], req[2], req[3]])
        } else {
            0
        };
        let actual = match self.swo {
            Some(swo) => swo.set_baudrate(requested),
            None => 0,
        };
        out[0..4].copy_from_slice(&actual.to_le_bytes());
        4
    }

    fn swo_control(&mut self, req: &[u8], out: &mut [u8]) -> usize {
        let start = req.first().copied().unwrap_or(0) == 1;
        let ok = match self.swo {
            Some(swo) => {
                if start {
                    swo.start()
                } else {
                    swo.stop()
                }
            }
            None => false,
        };
        out[0] = if ok { DAP_OK } else { DAP_ERROR };
        1
    }

    fn swo_status(&mut self, out: &mut [u8]) -> usize {
        let (status, count) = self.swo.map_or((0, 0), |swo| swo.status());
        out[0] = status;
        out[1..5].copy_from_slice(&count.to_le_bytes());
        5
    }

    /// Never issued by probe-rs, which defines the request but has no call site.
    /// Implemented anyway, because an unanswered command desynchronises a host for
    /// good; the index and timestamp fields are reported as zero.
    fn swo_extended_status(&mut self, out: &mut [u8]) -> usize {
        let (status, count) = self.swo.map_or((0, 0), |swo| swo.status());
        out[0] = status;
        out[1..5].copy_from_slice(&count.to_le_bytes());
        out[5..9].copy_from_slice(&0u32.to_le_bytes());
        out[9..13].copy_from_slice(&0u32.to_le_bytes());
        13
    }

    /// The polled transport: status, a 16-bit count, then the bytes.
    fn swo_data(&mut self, req: &[u8], out: &mut [u8]) -> usize {
        let max = if req.len() >= 2 {
            u16::from_le_bytes([req[0], req[1]]) as usize
        } else {
            0
        };

        let (status, _) = self.swo.map_or((0, 0), |swo| swo.status());
        // Three bytes of header, and never more than the response buffer holds.
        let room = out.len().saturating_sub(3);
        let want = max.min(room);

        let n = match self.swo {
            Some(swo) => swo.read(&mut out[3..3 + want]),
            None => 0,
        };

        out[0] = status;
        out[1..3].copy_from_slice(&(n as u16).to_le_bytes());
        3 + n
    }
}

/// Append a little-endian word, or report that the response buffer is full.
fn write_word(out: &mut [u8], w: &mut usize, value: u32) -> bool {
    if *w + 4 > out.len() {
        return false;
    }
    out[*w..*w + 4].copy_from_slice(&value.to_le_bytes());
    *w += 4;
    true
}
