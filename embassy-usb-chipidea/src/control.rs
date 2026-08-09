//! The control pipe (endpoint 0).

use core::future::poll_fn;
use core::sync::atomic::Ordering;
use core::task::Poll;

use embassy_usb_driver::{ControlPipe as ControlPipeTrait, Direction, EndpointError};

use crate::endpoint::{Endpoint, In, Out, flush, set_endpoint_stalled};
use crate::fmt::Bytes;
use crate::regs::{Endptsetupstat, Usb};
use crate::{State, dqh_index, ep_mask};

/// Endpoint 0, both directions.
pub struct ControlPipe<'d> {
    pub(crate) max_packet_size: u16,
    pub(crate) regs: Usb,
    pub(crate) state: State<'d>,
    pub(crate) ep_in: Endpoint<'d, In>,
    pub(crate) ep_out: Endpoint<'d, Out>,
}

impl ControlPipe<'_> {
    /// Both endpoint-0 bits, for a flush covering the whole control pipe.
    const EP0_MASK: u32 = ep_mask(0, Direction::In) | ep_mask(0, Direction::Out);
}

impl ControlPipeTrait for ControlPipe<'_> {
    fn max_packet_size(&self) -> usize {
        self.max_packet_size as usize
    }

    async fn setup(&mut self) -> [u8; 8] {
        // Wait for the controller to report a SETUP. `ENDPTSETUPSTAT` is the
        // authority; `setup_pending` is only the software latch the interrupt
        // handler uses to abort in-flight data stages.
        poll_fn(|cx| {
            self.state.cp_state.setup_waker.register(cx.waker());
            if self.regs.endptsetupstat().read().0 & 1 != 0 {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;

        let dqh = self.state.arena.dqh[dqh_index(0, Direction::Out)].get();

        // Acknowledge *before* reading the buffer, per the RM's control
        // endpoint sequence. Clearing afterwards would discard the status of a
        // SETUP that arrived during the copy, and the controller flushes
        // pending endpoint 0 transfers whenever this bit is set - so leaving
        // it set while the data stage is primed kills that transfer.
        self.regs.endptsetupstat().write_value(Endptsetupstat(1));

        // The setup tripwire. A newer SETUP can overwrite the landing zone
        // while we are copying out of it, so the copy is only trustworthy if
        // SUTW survived it. This needs `USBMODE.SLOM` set (lockouts off),
        // which `Bus` does at init.
        let data = loop {
            self.regs.usbcmd().modify(|w| w.set_sutw(true));

            (self.ep_out.cache_invalidate)(dqh as *const u8, size_of::<crate::dtd::Dqh>());
            let words = unsafe { crate::dtd::Dqh::read_setup(dqh) };

            if self.regs.usbcmd().read().sutw() {
                break words;
            }
        };
        self.regs.usbcmd().modify(|w| w.set_sutw(false));

        // The acknowledge above is a posted write; wait for it to land before
        // arming anything on endpoint 0.
        while self.regs.endptsetupstat().read().0 & 1 != 0 {}

        // A SETUP cancels whatever endpoint 0 was doing. Drop any stale
        // descriptor before the data stage arms a new one, or it would
        // consume the data stage's packet.
        flush(self.regs, Self::EP0_MASK);
        unsafe {
            crate::dtd::Dqh::retire(self.state.arena.dqh[dqh_index(0, Direction::In)].get());
            crate::dtd::Dqh::retire(dqh);
        }
        self.state.complete.fetch_and(!Self::EP0_MASK, Ordering::AcqRel);

        // Re-arm: the handler masked UE and latched the abort flag.
        self.state.cp_state.setup_pending.store(false, Ordering::Release);
        self.regs.usbintr().modify(|w| w.set_ue(true));

        let mut buf = [0u8; 8];
        buf[0..4].copy_from_slice(&data[0].to_le_bytes());
        buf[4..8].copy_from_slice(&data[1].to_le_bytes());

        trace!("SETUP {:?}", Bytes(&buf));
        buf
    }

    async fn data_out(&mut self, buf: &mut [u8], _first: bool, _last: bool) -> Result<usize, EndpointError> {
        let n = unsafe { self.ep_out.transfer(buf.as_mut_ptr(), buf.len()).await? };
        trace!("control out {} bytes", n);
        Ok(n)
    }

    async fn data_in(&mut self, data: &[u8], _first: bool, last: bool) -> Result<(), EndpointError> {
        trace!("control in {} bytes, last={}", data.len(), last);

        let mps = self.max_packet_size as usize;
        let mut sent = 0;

        // The stack hands us at most one transfer's worth, but it can exceed
        // one packet; the controller packetises, so this is one descriptor
        // unless the buffer straddles more pages than a descriptor can reach.
        while sent < data.len() {
            let chunk = &data[sent..];
            let n = unsafe { self.ep_in.transfer(chunk.as_ptr() as *mut u8, chunk.len()).await? };
            if n == 0 {
                return Err(EndpointError::Disabled);
            }
            sent += n;
        }

        if data.is_empty() {
            unsafe { self.ep_in.transfer(core::ptr::NonNull::dangling().as_ptr(), 0).await? };
        }

        if last {
            // `last` implies acceptance: no `accept()` follows, so the status
            // stage is ours to run. Consume the host's zero-length OUT.
            let _ = sent % mps;
            unsafe { self.ep_out.transfer(core::ptr::NonNull::dangling().as_ptr(), 0).await? };
        }

        Ok(())
    }

    async fn accept(&mut self) {
        trace!("control accept");
        // Zero-length IN is the status acknowledgement. A SETUP arriving here
        // is not an error - the stack will just call `setup()` again.
        let _ = unsafe { self.ep_in.transfer(core::ptr::NonNull::dangling().as_ptr(), 0).await };
    }

    async fn reject(&mut self) {
        trace!("control reject");
        // Stalling endpoint 0 stalls both directions; hardware clears it on
        // the next SETUP, so there is nothing to undo.
        set_endpoint_stalled(self.regs, 0, Direction::In, true);
    }

    async fn accept_set_address(&mut self, addr: u8) {
        trace!("control set address {}", addr);

        // `USBADRA` tells the controller to latch the address when the status
        // stage completes rather than immediately, which is exactly the
        // timing the spec wants - so the address goes in *before* the status
        // stage, the reverse of what most drivers do, and no delay is needed.
        self.regs.deviceaddr().write(|w| {
            w.set_usbadr(addr);
            w.set_usbadra(true);
        });

        self.accept().await;
    }
}
