//! Endpoints and the transfer state machine.

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{Ordering, compiler_fence};
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver::{
    Direction, Endpoint as EndpointTrait, EndpointError, EndpointIn, EndpointInfo, EndpointOut, EndpointType,
};
use portable_atomic::AtomicBool;

use crate::dtd::{self, Dqh, Dtd, max_dtd_len};
use crate::regs::{EpType, Usb};
use crate::{ChipideaInstance, EpState, State, dqh_index, ep_mask};

/// Direction marker, so the transfer machine is written once.
#[allow(private_bounds)]
pub trait Dir: sealed::Sealed {
    /// The direction this marker stands for.
    fn dir() -> Direction;

    #[doc(hidden)]
    fn waker(ep: &EpState) -> &AtomicWaker;

    #[doc(hidden)]
    fn enabled(ep: &EpState) -> &AtomicBool;
}

mod sealed {
    pub trait Sealed {}
}

/// Marker for device-to-host endpoints.
pub enum In {}
/// Marker for host-to-device endpoints.
pub enum Out {}

impl sealed::Sealed for In {}
impl sealed::Sealed for Out {}

impl Dir for In {
    fn dir() -> Direction {
        Direction::In
    }
    fn waker(ep: &EpState) -> &AtomicWaker {
        &ep.in_waker
    }
    fn enabled(ep: &EpState) -> &AtomicBool {
        &ep.in_enabled
    }
}

impl Dir for Out {
    fn dir() -> Direction {
        Direction::Out
    }
    fn waker(ep: &EpState) -> &AtomicWaker {
        &ep.out_waker
    }
    fn enabled(ep: &EpState) -> &AtomicBool {
        &ep.out_enabled
    }
}

/// Set or clear an endpoint's enable bit, keeping the hardware's endpoint-0
/// special case in one place.
///
/// `ENDPTCTRL0` has no enable bits worth writing - endpoint 0 is permanently
/// enabled - and `endptctrl(n)` addresses hardware endpoint `n + 1`.
pub(crate) fn set_endpoint_enabled(regs: Usb, index: usize, dir: Direction, ep_type: EndpointType, enabled: bool) {
    if index == 0 {
        return;
    }

    let r = regs.endptctrl(index - 1);
    let ty = EpType::from_bits(ep_type as u8);

    critical_section::with(|_| {
        r.modify(|w| match dir {
            Direction::In => {
                w.set_txt(ty);
                w.set_txe(enabled);
                w.set_txs(false);
                // Resetting the data toggle is mandatory on enable; hardware
                // will not do it for us.
                w.set_txr(enabled);
            }
            Direction::Out => {
                w.set_rxt(ty);
                w.set_rxe(enabled);
                w.set_rxs(false);
                w.set_rxr(enabled);
            }
        })
    });
}

/// Set or clear an endpoint's stall.
///
/// Clearing also resets the data toggle, which `ClearFeature(ENDPOINT_HALT)`
/// requires and the controller does not do on its own. Stalling endpoint 0
/// stalls both directions, and hardware clears it on the next SETUP.
pub(crate) fn set_endpoint_stalled(regs: Usb, index: usize, dir: Direction, stalled: bool) {
    critical_section::with(|_| {
        if index == 0 {
            regs.endptctrl0().modify(|w| {
                w.set_txs(stalled);
                w.set_rxs(stalled);
            });
            return;
        }

        regs.endptctrl(index - 1).modify(|w| match dir {
            Direction::In => {
                w.set_txs(stalled);
                if !stalled {
                    w.set_txr(true);
                }
            }
            Direction::Out => {
                w.set_rxs(stalled);
                if !stalled {
                    w.set_rxr(true);
                }
            }
        });
    });
}

/// Is this endpoint stalled?
pub(crate) fn endpoint_is_stalled(regs: Usb, index: usize, dir: Direction) -> bool {
    if index == 0 {
        let r = regs.endptctrl0().read();
        return match dir {
            Direction::In => r.txs(),
            Direction::Out => r.rxs(),
        };
    }

    let r = regs.endptctrl(index - 1).read();
    match dir {
        Direction::In => r.txs(),
        Direction::Out => r.rxs(),
    }
}

/// Flush an endpoint until the controller confirms nothing is primed.
///
/// `ENDPTFLUSH` is best-effort: a prime can land between the write and the
/// status read, so this loops. Used on cancellation, disable, and reset.
pub(crate) fn flush(regs: Usb, mask: u32) {
    loop {
        regs.endptflush().write_value(crate::regs::Endptflush(mask));
        while regs.endptflush().read().0 & mask != 0 {}
        if regs.endptstat().read().0 & mask == 0 {
            break;
        }
    }
}

/// Abandons an in-flight transfer if the future is dropped.
///
/// Without this, a cancelled `read`/`write` leaves the controller owning a
/// descriptor that points at a buffer whose lifetime has ended, and it will
/// happily DMA into it later.
struct CancelGuard {
    regs: Usb,
    mask: u32,
    dqh: *mut Dqh,
    armed: bool,
}

impl Drop for CancelGuard {
    fn drop(&mut self) {
        if !self.armed {
            return;
        }

        flush(self.regs, self.mask);
        // Retire the descriptor too, so a later prime cannot resurrect it.
        unsafe { Dqh::retire(self.dqh) };
        compiler_fence(Ordering::Acquire);
    }
}

/// A non-control endpoint.
pub struct Endpoint<'d, D: Dir> {
    pub(crate) info: EndpointInfo,
    pub(crate) regs: Usb,
    pub(crate) state: State<'d>,
    pub(crate) to_dma_addr: fn(*const ()) -> u32,
    pub(crate) cache_clean: fn(*const u8, usize),
    pub(crate) cache_invalidate: fn(*const u8, usize),
    pub(crate) _phantom: PhantomData<D>,
}

impl<'d, D: Dir> Endpoint<'d, D> {
    pub(crate) fn new(instance: &ChipideaInstance<'d>, info: EndpointInfo) -> Self {
        Self {
            info,
            regs: instance.regs,
            state: instance.state,
            to_dma_addr: instance.to_dma_addr,
            cache_clean: instance.cache_clean,
            cache_invalidate: instance.cache_invalidate,
            _phantom: PhantomData,
        }
    }

    fn index(&self) -> usize {
        self.info.addr.index()
    }

    fn ep_state(&self) -> &EpState {
        &self.state.ep_states[self.index()]
    }

    fn is_enabled(&self) -> bool {
        // Endpoint 0 is permanently enabled; nothing ever sets its mirror.
        self.index() == 0 || D::enabled(self.ep_state()).load(Ordering::Acquire)
    }

    /// A new SETUP abandons whatever endpoint 0 was doing.
    fn aborted_by_setup(&self) -> bool {
        self.index() == 0 && self.state.cp_state.setup_pending.load(Ordering::Acquire)
    }

    /// Prime the endpoint, working around a prime that races the controller's
    /// own queue-head fetch and gets lost.
    fn prime(&self, mask: u32, dtd: *const Dtd) {
        loop {
            self.regs.endptprime().write_value(crate::regs::Endptprime(mask));
            while self.regs.endptprime().read().0 & mask != 0 {}

            if self.regs.endptstat().read().0 & mask != 0 {
                // Armed.
                break;
            }
            if unsafe { Dtd::token(dtd) } & dtd::STATUS_ACTIVE == 0 {
                // Completed before we could observe it primed.
                break;
            }
            // Otherwise the prime was lost; try again.
        }
    }

    /// Move up to `len` bytes to or from `ptr`, returning how many moved.
    ///
    /// # Safety
    /// `ptr` must be valid for `len` bytes for the direction of this endpoint,
    /// and must stay valid until this future completes or is dropped.
    pub(crate) async unsafe fn transfer(&mut self, ptr: *mut u8, len: usize) -> Result<usize, EndpointError> {
        if !self.is_enabled() {
            trace!("ep{}{} transfer: not enabled", self.index(), dir_str(D::dir()));
            return Err(EndpointError::Disabled);
        }
        if self.aborted_by_setup() {
            trace!("ep{}{} transfer: setup pending at entry", self.index(), dir_str(D::dir()));
            return Err(EndpointError::Disabled);
        }

        let index = self.index();
        let dir = D::dir();
        let mask = ep_mask(index, dir);
        let slot = dqh_index(index, dir);

        let dqh = self.state.arena.dqh[slot].get();
        let dtd = self.state.arena.dtd[slot].get();

        // Zero-length transfers still hand the controller a buffer pointer.
        // Callers pass a dangling pointer for those, which the bus master
        // would fault on, so point it at the descriptor's own storage - valid,
        // DMA-reachable, and nothing is moved to or from it.
        let dma = if len == 0 {
            (self.to_dma_addr)(dtd as *const ())
        } else {
            (self.to_dma_addr)(ptr as *const ())
        };
        let n = len.min(max_dtd_len(dma));

        if dir == Direction::In {
            (self.cache_clean)(ptr as *const u8, n);
        }

        unsafe {
            Dtd::arm(dtd, dma, n, true, 0);
            (self.cache_clean)(dtd as *const u8, size_of::<Dtd>());

            Dqh::arm(dqh, (self.to_dma_addr)(dtd as *const ()));
            (self.cache_clean)(dqh as *const u8, size_of::<Dqh>());
        }

        // Drop any completion left over from a previous transfer on this
        // endpoint, so we cannot mistake it for ours.
        self.state.complete.fetch_and(!mask, Ordering::AcqRel);

        // Keep the stores above from sinking past the prime.
        compiler_fence(Ordering::Release);

        self.prime(mask, dtd);


        let mut guard = CancelGuard {
            regs: self.regs,
            mask,
            dqh,
            armed: true,
        };

        let token = poll_fn(|cx| {
            D::waker(self.ep_state()).register(cx.waker());

            if !self.is_enabled() || self.aborted_by_setup() {
                trace!(
                    "ep{}{} abort: setup={} dtd.token={:#010x} stat={:#010x} comp={:#010x} complete={:#010x}",
                    index,
                    dir_str(dir),
                    self.aborted_by_setup(),
                    unsafe { Dtd::token(dtd) },
                    self.regs.endptstat().read().0,
                    self.regs.endptcomplete().read().0,
                    self.state.complete.load(Ordering::Acquire)
                );
                return Poll::Ready(Err(EndpointError::Disabled));
            }

            if self.state.complete.load(Ordering::Acquire) & mask != 0 {
                (self.cache_invalidate)(dtd as *const u8, size_of::<Dtd>());
                let token = unsafe { Dtd::token(dtd) };
                // The completion bit can be set before the descriptor is
                // fully retired; wait for the controller to release it.
                if token & dtd::STATUS_ACTIVE == 0 {
                    return Poll::Ready(Ok(token));
                }
            }

            Poll::Pending
        })
        .await;

        let token = match token {
            Ok(t) => t,
            Err(e) => return Err(e),
        };

        // Completed normally, so there is nothing to cancel.
        guard.armed = false;
        self.state.complete.fetch_and(!mask, Ordering::AcqRel);

        compiler_fence(Ordering::Acquire);
        if dir == Direction::Out {
            (self.cache_invalidate)(ptr as *const u8, n);
        }

        if token & dtd::STATUS_HALTED != 0 {
            warn!(
                "ep{}{} halted, token {:#010x} dma={:#010x} n={} caps={:#010x}",
                index,
                dir_str(dir),
                token,
                dma,
                n,
                unsafe { core::ptr::read_volatile(core::ptr::addr_of!((*dqh).caps)) }
            );
            return Err(EndpointError::Disabled);
        }
        if token & dtd::STATUS_DATA_BUFFER_ERROR != 0 {
            warn!("ep{}{} buffer error", index, dir_str(dir));
            return Err(EndpointError::BufferOverflow);
        }
        if token & dtd::STATUS_TRANSACTION_ERROR != 0 {
            warn!("ep{}{} transaction error", index, dir_str(dir));
            return Err(EndpointError::Disabled);
        }

        Ok(n - Dtd::remaining(token))
    }
}

fn dir_str(dir: Direction) -> &'static str {
    match dir {
        Direction::In => "in",
        Direction::Out => "out",
    }
}

impl<D: Dir> EndpointTrait for Endpoint<'_, D> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        poll_fn(|cx| {
            D::waker(self.ep_state()).register(cx.waker());
            if self.is_enabled() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await
    }
}

impl EndpointOut for Endpoint<'_, Out> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        // A single packet. Provisioning less than a full packet risks the host
        // sending more than fits, which surfaces as a buffer error.
        let n = buf.len().min(self.info.max_packet_size as usize);
        unsafe { self.transfer(buf.as_mut_ptr(), n).await }
    }

    async fn read_transfer(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        // The controller retires a descriptor when the byte count is
        // exhausted *or* a short packet arrives, reporting the shortfall in
        // the token - which is exactly this method's contract. So a whole
        // multi-packet transfer costs one descriptor and one interrupt,
        // instead of one per packet as the default implementation would.
        let mut total = 0;

        while total < buf.len() {
            let chunk = &mut buf[total..];
            let n = unsafe { self.transfer(chunk.as_mut_ptr(), chunk.len()).await? };
            total += n;

            // Short packet: the transfer is over.
            if n % self.info.max_packet_size as usize != 0 || n == 0 {
                break;
            }
        }

        Ok(total)
    }
}

impl EndpointIn for Endpoint<'_, In> {
    async fn write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        if buf.len() > self.info.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }
        unsafe { self.transfer(buf.as_ptr() as *mut u8, buf.len()).await? };
        Ok(())
    }

    async fn write_transfer(&mut self, buf: &[u8], needs_zlp: bool) -> Result<(), EndpointError> {
        let mps = self.info.max_packet_size as usize;
        let mut sent = 0;

        while sent < buf.len() {
            let chunk = &buf[sent..];
            let n = unsafe { self.transfer(chunk.as_ptr() as *mut u8, chunk.len()).await? };
            if n == 0 {
                return Err(EndpointError::Disabled);
            }
            sent += n;
        }

        // Auto-ZLT is disabled on every endpoint, so a terminating zero-length
        // packet is ours to send.
        if needs_zlp && buf.len() % mps == 0 {
            unsafe { self.transfer(core::ptr::NonNull::dangling().as_ptr(), 0).await? };
        }

        Ok(())
    }
}
