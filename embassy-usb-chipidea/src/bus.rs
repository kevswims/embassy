//! Bus-level control: bring-up, the event loop, endpoint enable and stall.

use core::future::poll_fn;
use core::sync::atomic::Ordering;
use core::task::Poll;

use embassy_time::Timer;
use embassy_usb_driver::{Bus as BusTrait, Direction, EndpointAddress, Event, Unsupported};

use crate::endpoint::{endpoint_is_stalled, flush, set_endpoint_enabled, set_endpoint_stalled};
use crate::regs::{ControllerMode, Endptcomplete, Endptlistaddr, Endptsetupstat, PortSpeed, Usb};
use crate::{ALL_EP_MASK, Config, EndpointData, State, dqh_index, ep_mask};

/// The Chipidea bus.
pub struct Bus<'d> {
    pub(crate) regs: Usb,
    pub(crate) state: State<'d>,
    pub(crate) config: Config,
    pub(crate) endpoint_count: usize,
    pub(crate) control_max_packet_size: u16,
    pub(crate) to_dma_addr: fn(*const ()) -> u32,

    /// Deferred until the first `poll`, matching how the stack expects a
    /// `PowerDetected` to open the sequence.
    pub(crate) inited: bool,

    /// A bus reset has been handled but not yet reported: the stack only gets
    /// `Event::Reset` once the port is enabled and the speed is known.
    pub(crate) pending_reset: bool,
    pub(crate) suspended: bool,
}

impl Bus<'_> {
    /// Speed negotiated with the host. Only meaningful after `Event::Reset`.
    ///
    /// `embassy-usb-driver` 0.2.2 has no way to report this through the trait,
    /// so the packet sizes in your descriptors still come from
    /// `embassy_usb::Config::max_speed` - see the crate docs.
    pub fn speed(&self) -> PortSpeed {
        self.state.speed()
    }

    /// Re-enable the bus interrupt sources the handler masked.
    fn restore_irqs(&self) {
        self.regs.usbintr().modify(|w| {
            w.set_ue(true);
            w.set_uee(true);
            w.set_pce(true);
            w.set_ure(true);
            w.set_sle(true);
            w.set_see(true);
        });
    }

    /// Bring the controller up in device mode and attach.
    pub(crate) fn init(&mut self) {
        let r = self.regs;

        // Controller reset first: `USBMODE.CM` is write-once after reset on
        // some revisions, so it must be written to a freshly reset block.
        r.usbcmd().modify(|w| w.set_rst(true));
        while r.usbcmd().read().rst() {}

        let dcc = r.dccparams().read();
        debug!(
            "chipidea: dciversion {:#06x}, den {}, device-capable {}",
            r.dciversion().read().dciversion(),
            dcc.den(),
            dcc.dc()
        );

        // Bits 14 and 12 are set in NXP's working configuration and have no
        // field in the PAC. As with PLL_SIC's bits 26:25, they are written
        // through the raw word.
        const USBMODE_UNDOCUMENTED: u32 = (1 << 14) | (1 << 12);

        // `write`, not `modify`: see above.
        r.usbmode().write(|w| {
            w.0 |= USBMODE_UNDOCUMENTED;
            w.set_cm(ControllerMode::Device);
            // Setup lockouts off, so `ENDPTSETUPSTAT` can be cleared from
            // task context via the tripwire instead of deadlocking the
            // control pipe.
            w.set_slom(true);
            w.set_es(false);
            // Stream mode stays *enabled*. With `SDIS` set the controller
            // primes an endpoint but never transmits, which looks exactly
            // like a dead data stage: the host times out and resets.
            w.set_sdis(false);
        });

        let list = (self.to_dma_addr)(core::ptr::from_ref(self.state.arena) as *const ());
        debug_assert!(
            list & 0x7ff == 0,
            "queue head list must be 2048-byte aligned, got {:#010x}",
            list
        );
        debug!("chipidea: ENDPTLISTADDR {:#010x}", list);
        r.endptlistaddr().write_value(Endptlistaddr(list));

        // Vendor firmware uses 8 words in each direction; the reset value is
        // not necessarily sane on every part.
        let (rx, tx) = self.config.burst_size.unwrap_or((8, 8));
        r.burstsize().write(|w| {
            w.set_rxpburst(rx);
            w.set_txpburst(tx);
        });

        r.usbcmd().modify(|w| w.set_itc(self.config.interrupt_threshold));

        if !self.config.high_speed {
            r.portsc1().modify(|w| w.set_pfsc(true));
        }

        r.deviceaddr().write(|w| {
            w.set_usbadr(0);
            w.set_usbadra(false);
        });

        self.configure_control_queue_heads();
        self.restore_irqs();

        // Attach.
        r.usbcmd().modify(|w| w.set_rs(true));
    }

    /// Program the endpoint 0 queue heads.
    ///
    /// Endpoint 0 is permanently enabled in hardware, so the stack never calls
    /// `endpoint_set_enabled` for it and nothing else would ever set its
    /// packet size. Left at zero, the controller cannot receive SETUP at all -
    /// the port resets forever with no other symptom. `IOS` goes on the OUT
    /// queue head so SETUP raises an interrupt.
    fn configure_control_queue_heads(&self) {
        let mps = self.control_max_packet_size;
        unsafe {
            crate::dtd::Dqh::configure(
                self.state.arena.dqh[dqh_index(0, Direction::Out)].get(),
                mps,
                true,
                0,
                // Control endpoints keep it set in both directions.
                true,
            );
            crate::dtd::Dqh::configure(self.state.arena.dqh[dqh_index(0, Direction::In)].get(), mps, false, 0, true);
        }
    }

    /// Handle a bus reset: drop all in-flight work and unblock every waiter.
    fn handle_reset(&mut self) {
        let r = self.regs;

        // Order matters, and is the one the RM prescribes.
        r.endptsetupstat().write_value(Endptsetupstat(r.endptsetupstat().read().0));
        r.endptcomplete().write_value(Endptcomplete(r.endptcomplete().read().0));
        while r.endptprime().read().0 != 0 {}
        flush(r, ALL_EP_MASK);

        if !r.portsc1().read().pr() {
            // We took too long to get here and the reset window closed. The
            // host will reset us again, so this recovers on its own.
            warn!("missed the USB reset window");
        }

        r.deviceaddr().write(|w| {
            w.set_usbadr(0);
            w.set_usbadra(false);
        });

        // Retire every queue head, and drop stale completions.
        for i in 0..self.state.endpoint_count() {
            for dir in [Direction::Out, Direction::In] {
                unsafe { crate::dtd::Dqh::retire(self.state.arena.dqh[dqh_index(i, dir)].get()) };
            }
        }
        self.state.complete.store(0, Ordering::Release);
        self.state.cp_state.setup_pending.store(false, Ordering::Release);

        // Retiring the queue heads above cleared endpoint 0's too, and nothing
        // else will put it back.
        self.configure_control_queue_heads();

        // Wake everything, so in-flight futures observe `Disabled` rather than
        // hanging on a transfer the reset just destroyed.
        for ep in self.state.ep_states {
            ep.in_enabled.store(false, Ordering::Release);
            ep.out_enabled.store(false, Ordering::Release);
            ep.in_waker.wake();
            ep.out_waker.wake();
        }
        self.state.cp_state.setup_waker.wake();

        self.suspended = false;
        self.pending_reset = true;
    }

    fn alloc_for(&self, index: usize, dir: Direction) -> Option<EndpointData> {
        let ep = &self.state.ep_states[index];
        // SAFETY: allocation slots are written only before `start()`, and are
        // read-only from then on.
        unsafe {
            match dir {
                Direction::In => *ep.in_alloc.get(),
                Direction::Out => *ep.out_alloc.get(),
            }
        }
    }
}

impl BusTrait for Bus<'_> {
    async fn enable(&mut self) {
        self.regs.usbcmd().modify(|w| w.set_rs(true));
    }

    async fn disable(&mut self) {
        self.regs.usbcmd().modify(|w| w.set_rs(false));
    }

    async fn poll(&mut self) -> Event {
        poll_fn(|cx| {
            if !self.inited {
                self.inited = true;
                self.init();
                // There is no VBUS detection in scope, so synthesise the
                // power event the stack opens on.
                return Poll::Ready(Event::PowerDetected);
            }

            self.state.bus_waker.register(cx.waker());

            let r = self.regs;
            let sts = r.usbsts().read();

            // Reset: clean up here, but do not report yet. The stack enables
            // every endpoint the moment it sees `Reset`, and that must not
            // happen before the port is enabled and the speed is settled.
            if sts.uri() {
                r.usbsts().write(|w| w.set_uri(true));
                trace!("bus reset");
                self.handle_reset();
                self.restore_irqs();
            }

            if sts.pci() {
                r.usbsts().write(|w| w.set_pci(true));
                let p = r.portsc1().read();

                if p.fpr() && self.suspended {
                    self.suspended = false;
                    self.restore_irqs();
                    return Poll::Ready(Event::Resume);
                }

                if p.pe() {
                    let speed = p.pspd();
                    self.state.speed.store(speed.to_bits(), Ordering::Release);

                    if core::mem::take(&mut self.pending_reset) {
                        info!("port enabled at {:?}", speed);

                        if self.config.high_speed && speed != PortSpeed::High {
                            warn!(
                                "configured for high speed but enumerated at {:?}; descriptors \
                                 advertising high-speed packet sizes will be rejected",
                                speed
                            );
                        }

                        self.restore_irqs();
                        return Poll::Ready(Event::Reset);
                    }
                }
            }

            if sts.sli() {
                r.usbsts().write(|w| w.set_sli(true));
                self.suspended = true;
                self.restore_irqs();
                return Poll::Ready(Event::Suspend);
            }

            if sts.uei() {
                r.usbsts().write(|w| w.set_uei(true));
                trace!("usb error interrupt");
            }
            if sts.sei() {
                r.usbsts().write(|w| w.set_sei(true));
                error!("USB system (AHB) error - check the DMA address translation");
            }

            self.restore_irqs();
            Poll::Pending
        })
        .await
    }

    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
        let index = ep_addr.index();
        let dir = ep_addr.direction();

        if index >= self.endpoint_count {
            warn!("endpoint_set_enabled on out-of-range ep{}", index);
            return;
        }

        let Some(data) = self.alloc_for(index, dir) else {
            warn!("endpoint_set_enabled on unallocated ep{}", index);
            return;
        };

        let ep = &self.state.ep_states[index];

        if enabled {
            if self.config.high_speed
                && self.state.speed() == PortSpeed::Full
                && data.max_packet_size > 64
                && data.ep_type != embassy_usb_driver::EndpointType::Isochronous
            {
                error!(
                    "ep{} wants {} byte packets but the bus enumerated at full speed",
                    index, data.max_packet_size
                );
            }

            // Already enabled: leave it alone. The stack re-enables endpoints
            // on every SET_CONFIGURATION, and reconfiguring a queue head
            // retires it - which, with a receive already primed, halts the
            // endpoint with a data buffer error mid-fetch. Flushing first
            // avoids the halt but silently drops a transfer whose future is
            // still waiting on it, so the only safe answer is to do nothing.
            let already = match dir {
                Direction::In => ep.in_enabled.load(Ordering::Acquire),
                Direction::Out => ep.out_enabled.load(Ordering::Acquire),
            };
            if already {
                return;
            }

            let slot = dqh_index(index, dir);
            // Isochronous high-bandwidth endpoints encode extra transactions
            // per frame in the top bits of wMaxPacketSize.
            // `MULT` is only meaningful for isochronous endpoints, where it
            // carries the high-bandwidth transactions-per-frame count from the
            // top bits of wMaxPacketSize. It must be zero everywhere else -
            // a non-zero value on a control or bulk endpoint leaves the
            // endpoint primed but never executed.
            let mult = if data.ep_type == embassy_usb_driver::EndpointType::Isochronous {
                ((data.max_packet_size >> 11) & 3) as u8 + 1
            } else {
                0
            };

            unsafe {
                crate::dtd::Dqh::configure(
                    self.state.arena.dqh[slot].get(),
                    data.max_packet_size & 0x7ff,
                    index == 0 && dir == Direction::Out,
                    mult,
                    // Auto-ZLP off in both directions. On IN the stack emits
                    // zero-length packets itself, so hardware would duplicate
                    // them; on OUT, leaving it on makes a transfer whose
                    // length is an exact multiple of the packet size wait for
                    // a terminating ZLP the host has no reason to send.
                    true,
                )
            };

            set_endpoint_enabled(self.regs, index, dir, data.ep_type, true);
        } else {
            flush(self.regs, ep_mask(index, dir));
            unsafe { crate::dtd::Dqh::retire(self.state.arena.dqh[dqh_index(index, dir)].get()) };
            set_endpoint_enabled(self.regs, index, dir, data.ep_type, false);
        }

        match dir {
            Direction::In => {
                ep.in_enabled.store(enabled, Ordering::Release);
                ep.in_waker.wake();
            }
            Direction::Out => {
                ep.out_enabled.store(enabled, Ordering::Release);
                ep.out_waker.wake();
            }
        }
    }

    fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        let index = ep_addr.index();
        if index >= self.endpoint_count {
            return;
        }

        set_endpoint_stalled(self.regs, index, ep_addr.direction(), stalled);

        let ep = &self.state.ep_states[index];
        match ep_addr.direction() {
            Direction::In => ep.in_waker.wake(),
            Direction::Out => ep.out_waker.wake(),
        }
    }

    fn endpoint_is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
        if ep_addr.index() >= self.endpoint_count {
            return false;
        }
        endpoint_is_stalled(self.regs, ep_addr.index(), ep_addr.direction())
    }

    fn force_reset(&mut self) -> Result<(), Unsupported> {
        // Detach and re-attach; the host sees a disconnect and re-enumerates.
        self.regs.usbcmd().modify(|w| w.set_rs(false));
        self.regs.usbcmd().modify(|w| w.set_rs(true));
        Ok(())
    }

    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        if !self.suspended {
            return Err(Unsupported);
        }

        // The PHY clock is gated while suspended; it has to come back before
        // the resume signalling can be driven.
        if self.regs.portsc1().read().phcd() {
            self.regs.portsc1().modify(|w| w.set_phcd(false));
        }

        self.regs.portsc1().modify(|w| w.set_fpr(true));
        // Hardware clears FPR once it has driven resume for long enough.
        Timer::after_millis(5).await;

        Ok(())
    }
}
