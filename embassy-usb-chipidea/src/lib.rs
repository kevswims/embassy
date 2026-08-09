#![cfg_attr(not(test), no_std)]
#![allow(async_fn_in_trait)]
#![allow(unsafe_op_in_unsafe_fn)]
#![doc = include_str!("../README.md")]
#![warn(missing_docs)]

// This must go FIRST so that all the other modules see its macros.
mod fmt;

mod bus;
mod control;
pub mod dtd;
mod endpoint;
pub mod regs;

pub use bus::Bus;
pub use control::ControlPipe;
pub use endpoint::{Dir, Endpoint, In, Out};

use core::cell::UnsafeCell;

use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver::{
    Direction, EndpointAddress, EndpointAllocError, EndpointInfo, EndpointType,
};
use portable_atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};

use crate::dtd::{DmaArena, MAX_EP};
use crate::regs::{PortSpeed, Usb};

// ---------------------------------------------------------------------------
// Endpoint bit vocabulary
// ---------------------------------------------------------------------------

/// The bit for `(index, dir)` in `ENDPTPRIME`, `ENDPTFLUSH`, `ENDPTSTAT` and
/// `ENDPTCOMPLETE`.
///
/// All four registers share one layout - IN in the high half, OUT in the low -
/// so a single mask works against any of them, and against the `complete`
/// atomic the interrupt handler stashes raw register values into.
#[inline]
pub const fn ep_mask(index: usize, dir: Direction) -> u32 {
    match dir {
        Direction::Out => 1 << index,
        Direction::In => 1 << (index + 16),
    }
}

/// Every endpoint bit in both directions, for a full flush.
pub const ALL_EP_MASK: u32 = 0x00ff_00ff;

/// The queue-head slot hardware uses for `(index, dir)`: OUT even, IN odd.
#[inline]
pub const fn dqh_index(index: usize, dir: Direction) -> usize {
    2 * index
        + match dir {
            Direction::Out => 0,
            Direction::In => 1,
        }
}

// ---------------------------------------------------------------------------
// Chip hooks
// ---------------------------------------------------------------------------

/// `to_dma_addr` for chips where CPU and bus-master addresses coincide.
pub fn identity_dma_addr(p: *const ()) -> u32 {
    p as u32
}

/// `cache_clean`/`cache_invalidate` for cores with no data cache.
pub fn no_cache_op(_addr: *const u8, _len: usize) {}

/// Everything the HAL supplies to construct a [`Driver`].
#[derive(Copy, Clone)]
pub struct ChipideaInstance<'d> {
    /// The controller's register block.
    pub regs: Usb,

    /// Borrow of the HAL's [`StateStorage`].
    pub state: State<'d>,

    /// Endpoint pairs to use. Clamped to `DCCPARAMS.DEN` and [`MAX_EP`].
    pub endpoint_count: usize,

    /// Translate a CPU address into what the USB bus master must see.
    ///
    /// Applied to every address handed to the controller: the queue-head
    /// list, descriptor links, and data buffers. Use [`identity_dma_addr`]
    /// when they coincide.
    pub to_dma_addr: fn(*const ()) -> u32,

    /// Make CPU writes visible to the controller. [`no_cache_op`] on
    /// uncached cores.
    pub cache_clean: fn(*const u8, usize),

    /// Make controller writes visible to the CPU. [`no_cache_op`] on
    /// uncached cores.
    pub cache_invalidate: fn(*const u8, usize),
}

/// Driver configuration.
#[non_exhaustive]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Allow high-speed operation.
    ///
    /// `false` sets `PORTSC1.PFSC`, forcing full speed. This must agree with
    /// `embassy_usb::Config::max_speed`, which is where the stack actually
    /// gets its packet sizes - see the crate docs.
    pub high_speed: bool,

    /// `USBCMD.ITC`, interrupt coalescing in micro-frames. Zero, the default,
    /// is the lowest-latency setting.
    pub interrupt_threshold: u8,

    /// `BURSTSIZE` as `(rx, tx)`. `None` leaves the reset value.
    pub burst_size: Option<(u8, u8)>,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            high_speed: true,
            interrupt_threshold: 0,
            burst_size: None,
        }
    }
}

// ---------------------------------------------------------------------------
// Shared state
// ---------------------------------------------------------------------------

/// What an endpoint slot was allocated as.
#[derive(Copy, Clone)]
pub(crate) struct EndpointData {
    pub(crate) ep_type: EndpointType,
    pub(crate) max_packet_size: u16,
}

/// Per-endpoint-pair state.
pub struct EpState {
    pub(crate) in_waker: AtomicWaker,
    pub(crate) out_waker: AtomicWaker,

    /// RAM mirror of `ENDPTCTRLn.TXE`/`RXE`. Futures test this rather than
    /// the register so that a disabled endpoint fails fast without a bus
    /// access, and so a reset can unblock them all at once.
    pub(crate) in_enabled: AtomicBool,
    pub(crate) out_enabled: AtomicBool,

    /// Written only during allocation, before `start()` hands out the bus.
    pub(crate) in_alloc: UnsafeCell<Option<EndpointData>>,
    pub(crate) out_alloc: UnsafeCell<Option<EndpointData>>,
}

// SAFETY: the `UnsafeCell` allocation slots are written only during endpoint
// allocation, which happens before `Driver::start` and is `&mut self`-gated;
// afterwards they are read-only. Everything else is atomic.
unsafe impl Send for EpState {}
unsafe impl Sync for EpState {}

impl EpState {
    const fn new() -> Self {
        Self {
            in_waker: AtomicWaker::new(),
            out_waker: AtomicWaker::new(),
            in_enabled: AtomicBool::new(false),
            out_enabled: AtomicBool::new(false),
            in_alloc: UnsafeCell::new(None),
            out_alloc: UnsafeCell::new(None),
        }
    }
}

/// Control-pipe state shared with the interrupt handler.
pub struct ControlPipeState {
    pub(crate) setup_waker: AtomicWaker,

    /// A SETUP has arrived and not yet been consumed. In-flight endpoint 0
    /// transfers poll this so they can abandon the old control transfer, as
    /// the `ControlPipe` contract requires.
    pub(crate) setup_pending: AtomicBool,
}

impl ControlPipeState {
    const fn new() -> Self {
        Self {
            setup_waker: AtomicWaker::new(),
            setup_pending: AtomicBool::new(false),
        }
    }
}

/// Storage for the driver's shared state, instantiated as a `static` by the HAL.
///
/// Contains the queue-head array the controller DMAs against, so it must live
/// in RAM the USB bus master can reach - see the crate docs on placement.
///
/// `EP_COUNT` sizes the per-endpoint state. The DMA arena is always sized for
/// [`MAX_EP`].
pub struct StateStorage<const EP_COUNT: usize> {
    arena: DmaArena,
    ep_states: [EpState; EP_COUNT],
    cp_state: ControlPipeState,
    bus_waker: AtomicWaker,

    /// `ENDPTCOMPLETE` bits accumulated by the interrupt handler.
    ///
    /// The handler must clear that register to deassert `USBSTS.UI`, so the
    /// bits have to be preserved here or completions would be lost.
    complete: AtomicU32,

    /// Negotiated [`PortSpeed`], as raw bits.
    speed: AtomicU8,
}

impl<const EP_COUNT: usize> StateStorage<EP_COUNT> {
    /// Create the storage. Intended for a `static` initialiser.
    pub const fn new() -> Self {
        ::core::assert!(EP_COUNT <= MAX_EP, "EP_COUNT exceeds the Chipidea maximum of 8");

        Self {
            arena: DmaArena::new(),
            ep_states: [const { EpState::new() }; EP_COUNT],
            cp_state: ControlPipeState::new(),
            bus_waker: AtomicWaker::new(),
            complete: AtomicU32::new(0),
            speed: AtomicU8::new(PortSpeed::Undefined as u8),
        }
    }

    /// Borrow the storage as a type-erased [`State`].
    pub fn as_state(&self) -> State<'_> {
        State {
            arena: &self.arena,
            ep_states: &self.ep_states,
            cp_state: &self.cp_state,
            bus_waker: &self.bus_waker,
            complete: &self.complete,
            speed: &self.speed,
        }
    }
}

impl<const EP_COUNT: usize> Default for StateStorage<EP_COUNT> {
    fn default() -> Self {
        Self::new()
    }
}

/// A type-erased borrow of [`StateStorage`].
///
/// `Copy`, so it can be handed to the driver and the interrupt handler alike
/// without the const generic leaking into either.
#[derive(Clone, Copy)]
pub struct State<'d> {
    pub(crate) arena: &'d DmaArena,
    pub(crate) ep_states: &'d [EpState],
    pub(crate) cp_state: &'d ControlPipeState,
    pub(crate) bus_waker: &'d AtomicWaker,
    pub(crate) complete: &'d AtomicU32,
    pub(crate) speed: &'d AtomicU8,
}

impl State<'_> {
    /// Endpoint pairs this storage was sized for.
    pub fn endpoint_count(&self) -> usize {
        self.ep_states.len()
    }

    /// Speed negotiated with the host. Only meaningful after `Event::Reset`.
    pub fn speed(&self) -> PortSpeed {
        PortSpeed::from_bits(self.speed.load(Ordering::Acquire))
    }
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

/// The Chipidea `embassy-usb-driver` implementation.
pub struct Driver<'d> {
    instance: ChipideaInstance<'d>,
    config: Config,
    /// Endpoint pairs actually usable: the smallest of what the HAL asked
    /// for, what the storage was sized for, and what `DCCPARAMS.DEN` reports.
    endpoint_count: usize,
}

impl<'d> Driver<'d> {
    /// Create the driver.
    ///
    /// Does not touch the controller beyond reading its capability register;
    /// bring-up happens on the first `Bus::poll`, after the HAL has had a
    /// chance to finish whatever chip-level setup it owns.
    pub fn new(instance: ChipideaInstance<'d>, config: Config) -> Self {
        let den = instance.regs.dccparams().read().den() as usize;

        let endpoint_count = instance
            .endpoint_count
            .min(instance.state.endpoint_count())
            .min(MAX_EP)
            .min(if den == 0 {
                // A zero here means the block is unclocked or still in reset -
                // trust the HAL's number rather than allocating nothing, and
                // let the bring-up traces show the problem.
                warn!("DCCPARAMS.DEN reads 0; is the controller clocked?");
                MAX_EP
            } else {
                den
            });

        if instance.endpoint_count > endpoint_count {
            warn!(
                "requested {} endpoint pairs, using {}",
                instance.endpoint_count, endpoint_count
            );
        }

        Self {
            instance,
            config,
            endpoint_count,
        }
    }

    /// Claim an endpoint slot and record what it was allocated as.
    fn alloc_endpoint<D: Dir>(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Endpoint<'d, D>, EndpointAllocError> {
        // The capability field is 11 bits; the top bits of wMaxPacketSize are
        // the high-bandwidth multiplier, which only isochronous endpoints use.
        if max_packet_size & 0x7ff > 1024 {
            error!("max packet size {} out of range", max_packet_size);
            return Err(EndpointAllocError);
        }

        let dir = D::dir();
        let taken = |s: &Self, i: usize| s.alloc_slot(i, dir).is_some();

        let index = match ep_addr {
            Some(addr) => {
                let i = addr.index();
                if i >= self.endpoint_count || addr.direction() != dir {
                    return Err(EndpointAllocError);
                }
                // Endpoint 0 is the control pipe's, and nothing else's.
                if i == 0 && ep_type != EndpointType::Control {
                    return Err(EndpointAllocError);
                }
                if taken(self, i) {
                    return Err(EndpointAllocError);
                }
                i
            }
            None => {
                let first = if ep_type == EndpointType::Control { 0 } else { 1 };
                (first..self.endpoint_count)
                    .find(|&i| !taken(self, i))
                    .ok_or(EndpointAllocError)?
            }
        };

        let data = EndpointData {
            ep_type,
            max_packet_size,
        };

        let ep = &self.instance.state.ep_states[index];
        // SAFETY: allocation happens through `&mut self` and only before
        // `start()` hands the state to the bus; from then on these are read
        // only. No interrupt touches them.
        unsafe {
            match dir {
                Direction::In => *ep.in_alloc.get() = Some(data),
                Direction::Out => *ep.out_alloc.get() = Some(data),
            }
        }

        debug!(
            "alloc ep{} {:?} {:?} mps {}",
            index,
            dir,
            ep_type,
            max_packet_size & 0x7ff
        );

        Ok(Endpoint::new(
            &self.instance,
            EndpointInfo {
                addr: EndpointAddress::from_parts(index, dir),
                ep_type,
                max_packet_size,
                interval_ms,
            },
        ))
    }

    fn alloc_slot(&self, index: usize, dir: Direction) -> Option<EndpointData> {
        let ep = &self.instance.state.ep_states[index];
        // SAFETY: see `alloc_endpoint`.
        unsafe {
            match dir {
                Direction::In => *ep.in_alloc.get(),
                Direction::Out => *ep.out_alloc.get(),
            }
        }
    }
}

impl<'d> embassy_usb_driver::Driver<'d> for Driver<'d> {
    type EndpointOut = Endpoint<'d, Out>;
    type EndpointIn = Endpoint<'d, In>;
    type ControlPipe = ControlPipe<'d>;
    type Bus = Bus<'d>;

    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        self.alloc_endpoint(ep_type, ep_addr, max_packet_size, interval_ms)
    }

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        self.alloc_endpoint(ep_type, ep_addr, max_packet_size, interval_ms)
    }

    fn start(mut self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        let ep_out = self
            .alloc_endpoint::<Out>(
                EndpointType::Control,
                Some(EndpointAddress::from_parts(0, Direction::Out)),
                control_max_packet_size,
                0,
            )
            .expect("endpoint 0 OUT must be free");
        let ep_in = self
            .alloc_endpoint::<In>(
                EndpointType::Control,
                Some(EndpointAddress::from_parts(0, Direction::In)),
                control_max_packet_size,
                0,
            )
            .expect("endpoint 0 IN must be free");

        let bus = Bus {
            regs: self.instance.regs,
            state: self.instance.state,
            config: self.config,
            endpoint_count: self.endpoint_count,
            control_max_packet_size,
            to_dma_addr: self.instance.to_dma_addr,
            inited: false,
            pending_reset: false,
            suspended: false,
        };

        let control = ControlPipe {
            max_packet_size: control_max_packet_size,
            regs: self.instance.regs,
            state: self.instance.state,
            ep_in,
            ep_out,
        };

        (bus, control)
    }
}

// ---------------------------------------------------------------------------
// Interrupt handler
// ---------------------------------------------------------------------------

/// Handle a USB interrupt.
///
/// The HAL's interrupt handler calls this and nothing else.
///
/// # Safety
/// Must be called only from the interrupt for the controller at `regs`, with
/// the `state` that was given to the matching [`ChipideaInstance`].
pub unsafe fn on_interrupt(regs: Usb, state: &State<'_>) {
    let sts = regs.usbsts().read();
    let intr = regs.usbintr().read();

    // ---- endpoint completion ----
    //
    // `USBSTS.UI` is driven by `ENDPTCOMPLETE` having any bit set, so that
    // register must be cleared here or the interrupt re-fires forever. Read
    // once and write the same value back: that clears exactly the bits
    // observed and cannot drop one the controller sets concurrently.
    if sts.ui() || sts.uei() {
        let comp = regs.endptcomplete().read();
        if comp.0 != 0 {
            regs.endptcomplete().write_value(comp);
            state.complete.fetch_or(comp.0, Ordering::AcqRel);

            for (i, ep) in state.ep_states.iter().enumerate() {
                if comp.0 & ep_mask(i, Direction::Out) != 0 {
                    ep.out_waker.wake();
                }
                if comp.0 & ep_mask(i, Direction::In) != 0 {
                    ep.in_waker.wake();
                }
            }
        }

        // `ENDPTSETUPSTAT` also drives `UI`, but it can only be cleared by the
        // tripwire sequence, which spins and so belongs in task context. Mask
        // `UE` instead; `ControlPipe::setup` re-enables it.
        if regs.endptsetupstat().read().0 != 0 {
            // Latch once. `ENDPTSETUPSTAT` stays set until task context runs
            // the tripwire, so masking UE on every interrupt would also mask
            // the completion interrupts the in-flight data stage is waiting
            // on - and only `setup()` re-enables it, which cannot run until
            // that data stage finishes. Mask only on the transition.
            if !state.cp_state.setup_pending.swap(true, Ordering::AcqRel) {
                regs.usbintr().modify(|w| w.set_ue(false));
            }
            state.cp_state.setup_waker.wake();

            // A new SETUP abandons whatever endpoint 0 was doing.
            if let Some(ep0) = state.ep_states.first() {
                ep0.in_waker.wake();
                ep0.out_waker.wake();
            }
        }

        regs.usbsts().write(|w| {
            w.set_ui(sts.ui());
            w.set_uei(sts.uei());
        });
    }

    // ---- bus events ----
    //
    // Masked rather than cleared, so the status bit survives for `Bus::poll`
    // to read; `poll` does the write-1-to-clear and re-enables these.
    let bus_events = sts.uri() || sts.pci() || sts.sli() || sts.sri() || sts.sei();
    let bus_enabled = intr.ure() || intr.pce() || intr.sle() || intr.sre() || intr.see();

    if bus_events && bus_enabled {
        regs.usbintr().modify(|w| {
            w.set_ure(false);
            w.set_pce(false);
            w.set_sle(false);
            w.set_sre(false);
            w.set_see(false);
        });
        state.bus_waker.wake();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn endpoint_masks() {
        assert_eq!(ep_mask(0, Direction::Out), 1);
        assert_eq!(ep_mask(0, Direction::In), 1 << 16);
        assert_eq!(ep_mask(7, Direction::Out), 1 << 7);
        assert_eq!(ep_mask(7, Direction::In), 1 << 23);

        // Every endpoint bit, and nothing else.
        let mut all = 0;
        for i in 0..MAX_EP {
            all |= ep_mask(i, Direction::Out) | ep_mask(i, Direction::In);
        }
        assert_eq!(all, ALL_EP_MASK);
    }

    #[test]
    fn queue_head_slots() {
        assert_eq!(dqh_index(0, Direction::Out), 0);
        assert_eq!(dqh_index(0, Direction::In), 1);
        assert_eq!(dqh_index(3, Direction::Out), 6);
        assert_eq!(dqh_index(3, Direction::In), 7);
    }

    /// The arena inside a `static`-shaped storage still meets the hardware's
    /// 2048-byte requirement.
    #[test]
    fn storage_arena_alignment() {
        static STORAGE: StateStorage<8> = StateStorage::new();
        let state = STORAGE.as_state();
        assert_eq!(core::ptr::from_ref(state.arena) as usize & 0x7ff, 0);
        assert_eq!(state.endpoint_count(), 8);
    }
}
