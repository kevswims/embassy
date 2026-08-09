# embassy-usb-chipidea

An [`embassy-usb-driver`](https://crates.io/crates/embassy-usb-driver) implementation for
the Chipidea USB **device** controller, the EHCI-derived "device controller interface"
found in NXP MCX N, i.MX RT and LPC parts.

This crate is chip-agnostic. It carries its own register definitions and depends on no
PAC, so a HAL wires it up by handing over a base pointer, some statics, and a couple of
function pointers. Host mode is not implemented.

## Using it from a HAL

```rust,ignore
use embassy_usb_chipidea::{ChipideaInstance, Config, Driver, State, StateStorage, regs};

fn state() -> State<'static> {
    static STATE: StateStorage<8> = StateStorage::new();
    STATE.as_state()
}

let instance = ChipideaInstance {
    regs: unsafe { regs::Usb::from_ptr(0x4010_b000 as _) },
    state: state(),
    endpoint_count: 8,
    to_dma_addr: embassy_usb_chipidea::identity_dma_addr,
    cache_clean: embassy_usb_chipidea::no_cache_op,
    cache_invalidate: embassy_usb_chipidea::no_cache_op,
};
let driver = Driver::new(instance, Config::default());
```

and from the interrupt handler:

```rust,ignore
unsafe { embassy_usb_chipidea::on_interrupt(REGS, &state()) }
```

## Memory placement

`StateStorage` contains the device queue-head array the controller DMAs into. It is
`#[repr(align(2048))]` because `ENDPTLISTADDR` cannot represent the low 11 bits of an
address, and it must live in RAM the USB bus master can reach. If the default `.bss`
bank is not reachable, place it explicitly:

```rust,ignore
#[unsafe(link_section = ".usb_ram")]
static STATE: StateStorage<8> = StateStorage::new();
```

`to_dma_addr` translates a CPU address into what the USB master must see. Use
`identity_dma_addr` when they coincide. It is applied to *every* address handed to the
controller — the queue-head list, transfer-descriptor links, and data buffers — so a
partial translation cannot produce a half-working device.

`cache_clean` / `cache_invalidate` are no-ops (`no_cache_op`) on cores without a data
cache. On a cached core they must be real, and data buffers additionally have to be
cache-line aligned and padded, which this crate cannot enforce.

## Speed

`embassy-usb-driver` 0.2.2 has no way for a device-side driver to report the negotiated
speed, so the packet sizes in your descriptors come solely from
`embassy_usb::Config::max_speed`, which defaults to `Full`.

Set that to `High` **and** leave [`Config::high_speed`] `true` to run at high speed; set
both to their full-speed values to force full speed (which drives `PORTSC1.PFSC`). If
they disagree, the driver logs an error and rejects endpoints whose packet size the
negotiated speed cannot carry, rather than enumerating something illegal.

`Bus::speed()` reports what was actually negotiated, once `Event::Reset` has been
delivered.
