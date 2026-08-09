//! Register definitions for the Chipidea USB device controller.
//!
//! This is the EHCI-derived "device controller interface" block: NXP call it
//! `USBHS1__USBC` on MCX N and `USB_OTGn` on i.MX RT, at the same offsets in
//! both. Only the device-mode registers are modelled - the host-mode schedule
//! registers (`PERIODICLISTBASE`, `ASYNCLISTADDR`, `CONFIGFLAG`) alias some of
//! these addresses and would only invite mistakes.
//!
//! The accessor shape (`read`/`write`/`write_value`/`modify`, fieldset
//! newtypes over `u32`) matches chiptool's output, so a generated block could
//! replace this file without touching the driver.

#![allow(missing_docs)]

use core::marker::PhantomData;

#[derive(Copy, Clone, PartialEq, Eq)]
pub struct RW;
#[derive(Copy, Clone, PartialEq, Eq)]
pub struct R;
#[derive(Copy, Clone, PartialEq, Eq)]
pub struct W;

mod sealed {
    use super::*;
    pub trait Access {}
    impl Access for R {}
    impl Access for W {}
    impl Access for RW {}
}

pub trait Access: sealed::Access + Copy {}
impl Access for R {}
impl Access for W {}
impl Access for RW {}

pub trait Read: Access {}
impl Read for RW {}
impl Read for R {}

pub trait Write: Access {}
impl Write for RW {}
impl Write for W {}

#[derive(Copy, Clone, PartialEq, Eq)]
pub struct Reg<T: Copy, A: Access> {
    ptr: *mut u8,
    phantom: PhantomData<*mut (T, A)>,
}
unsafe impl<T: Copy, A: Access> Send for Reg<T, A> {}
unsafe impl<T: Copy, A: Access> Sync for Reg<T, A> {}

impl<T: Copy, A: Access> Reg<T, A> {
    #[allow(clippy::missing_safety_doc)]
    #[inline(always)]
    pub const unsafe fn from_ptr(ptr: *mut T) -> Self {
        Self {
            ptr: ptr as _,
            phantom: PhantomData,
        }
    }

    #[inline(always)]
    pub const fn as_ptr(&self) -> *mut T {
        self.ptr as _
    }
}

impl<T: Copy, A: Read> Reg<T, A> {
    #[inline(always)]
    pub fn read(&self) -> T {
        unsafe { (self.ptr as *mut T).read_volatile() }
    }
}

impl<T: Copy, A: Write> Reg<T, A> {
    #[inline(always)]
    pub fn write_value(&self, val: T) {
        unsafe { (self.ptr as *mut T).write_volatile(val) }
    }
}

impl<T: Default + Copy, A: Write> Reg<T, A> {
    #[inline(always)]
    pub fn write<R>(&self, f: impl FnOnce(&mut T) -> R) -> R {
        let mut val = Default::default();
        let res = f(&mut val);
        self.write_value(val);
        res
    }
}

impl<T: Copy, A: Read + Write> Reg<T, A> {
    #[inline(always)]
    pub fn modify<R>(&self, f: impl FnOnce(&mut T) -> R) -> R {
        let mut val = self.read();
        let res = f(&mut val);
        self.write_value(val);
        res
    }
}

// ---------------------------------------------------------------------------
// Fieldset plumbing
// ---------------------------------------------------------------------------

/// Declare a fieldset newtype over `u32`.
macro_rules! fieldset {
    ($(#[$m:meta])* $name:ident) => {
        $(#[$m])*
        #[repr(transparent)]
        #[derive(Copy, Clone, Default, Eq, PartialEq)]
        pub struct $name(pub u32);
    };
}

/// Single-bit accessors. Setter names are spelled out rather than derived, so
/// this needs no proc-macro and every generated name is greppable.
macro_rules! bits {
    ($ty:ident; $($(#[$m:meta])* $get:ident / $set:ident @ $shift:expr;)*) => {
        impl $ty {
            $(
                $(#[$m])*
                #[must_use]
                #[inline(always)]
                pub const fn $get(&self) -> bool {
                    (self.0 >> $shift) & 1 != 0
                }
                #[inline(always)]
                pub const fn $set(&mut self, val: bool) {
                    self.0 = (self.0 & !(1 << $shift)) | ((val as u32) << $shift);
                }
            )*
        }
    };
}

/// Multi-bit integer accessors.
///
/// The shift follows the type after a comma rather than an `@`, because `@` is
/// not a legal follow token for a `:ty` fragment.
macro_rules! uints {
    ($ty:ident; $($(#[$m:meta])* $get:ident / $set:ident : $fty:ty, $shift:expr, $width:expr;)*) => {
        impl $ty {
            $(
                $(#[$m])*
                #[must_use]
                #[inline(always)]
                pub const fn $get(&self) -> $fty {
                    ((self.0 >> $shift) & ((1 << $width) - 1)) as $fty
                }
                #[inline(always)]
                pub const fn $set(&mut self, val: $fty) {
                    let mask: u32 = ((1 << $width) - 1) << $shift;
                    self.0 = (self.0 & !mask) | (((val as u32) << $shift) & mask);
                }
            )*
        }
    };
}

/// Enum-typed field accessors. The enum must be `#[repr(u8)]` with a
/// `from_bits`/`to_bits` pair covering every encoding of its width.
macro_rules! enums {
    ($ty:ident; $($(#[$m:meta])* $get:ident / $set:ident : $fty:ty, $shift:expr, $width:expr;)*) => {
        impl $ty {
            $(
                $(#[$m])*
                #[must_use]
                #[inline(always)]
                pub const fn $get(&self) -> $fty {
                    <$fty>::from_bits((((self.0 >> $shift) & ((1 << $width) - 1)) as u8))
                }
                #[inline(always)]
                pub const fn $set(&mut self, val: $fty) {
                    let mask: u32 = ((1 << $width) - 1) << $shift;
                    self.0 = (self.0 & !mask) | (((val.to_bits() as u32) << $shift) & mask);
                }
            )*
        }
    };
}

// ---------------------------------------------------------------------------
// Enums
// ---------------------------------------------------------------------------

/// `USBMODE.CM`, the controller's operating mode.
#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ControllerMode {
    Idle = 0,
    Reserved = 1,
    Device = 2,
    Host = 3,
}

impl ControllerMode {
    #[inline(always)]
    pub const fn from_bits(v: u8) -> Self {
        match v & 3 {
            0 => Self::Idle,
            1 => Self::Reserved,
            2 => Self::Device,
            _ => Self::Host,
        }
    }

    #[inline(always)]
    pub const fn to_bits(self) -> u8 {
        self as u8
    }
}

/// `PORTSC1.PSPD`, the speed the port negotiated.
///
/// Only meaningful once the port is enabled, i.e. after the reset handshake
/// has completed.
#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PortSpeed {
    Full = 0,
    Low = 1,
    High = 2,
    Undefined = 3,
}

impl PortSpeed {
    #[inline(always)]
    pub const fn from_bits(v: u8) -> Self {
        match v & 3 {
            0 => Self::Full,
            1 => Self::Low,
            2 => Self::High,
            _ => Self::Undefined,
        }
    }

    #[inline(always)]
    pub const fn to_bits(self) -> u8 {
        self as u8
    }
}

/// `ENDPTCTRLn.RXT`/`TXT`, the endpoint's transfer type.
///
/// The encoding is identical to [`embassy_usb_driver::EndpointType`], so
/// conversion is a cast.
#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EpType {
    Control = 0,
    Isochronous = 1,
    Bulk = 2,
    Interrupt = 3,
}

impl EpType {
    #[inline(always)]
    pub const fn from_bits(v: u8) -> Self {
        match v & 3 {
            0 => Self::Control,
            1 => Self::Isochronous,
            2 => Self::Bulk,
            _ => Self::Interrupt,
        }
    }

    #[inline(always)]
    pub const fn to_bits(self) -> u8 {
        self as u8
    }
}

// ---------------------------------------------------------------------------
// Fieldsets
// ---------------------------------------------------------------------------

fieldset!(
    /// Identification register.
    Id
);
uints!(Id;
    /// Configuration number, complement of `nid`.
    id / set_id: u8, 0, 6;
    /// Complement of `id`; reads `!id` on a working block.
    nid / set_nid: u8, 8, 6;
    revision / set_revision: u8, 16, 8;
);

fieldset!(
    /// Device hardware parameters.
    Hwdevice
);
bits!(Hwdevice;
    /// Device capable.
    dc / set_dc @ 0;
);
uints!(Hwdevice;
    /// Number of endpoint *directions*, i.e. twice the endpoint-pair count.
    devep / set_devep: u8, 1, 5;
);

fieldset!(
    /// Device controller interface version.
    Dciversion
);
uints!(Dciversion;
    dciversion / set_dciversion: u16, 0, 16;
);

fieldset!(
    /// Device controller capability parameters.
    Dccparams
);
bits!(Dccparams;
    /// Device capable.
    dc / set_dc @ 7;
    /// Host capable.
    hc / set_hc @ 8;
);
uints!(Dccparams;
    /// Endpoint pairs supported. The authority on how many endpoints this
    /// instance really has - do not assume 8.
    den / set_den: u8, 0, 5;
);

fieldset!(
    /// USB command register.
    Usbcmd
);
bits!(Usbcmd;
    /// Run/Stop. Clearing it detaches (drops the pull-up).
    rs / set_rs @ 0;
    /// Controller reset. Self-clearing.
    rst / set_rst @ 1;
    /// Setup tripwire. Guards a SETUP read against being overwritten
    /// mid-copy; see `ControlPipe::setup`.
    sutw / set_sutw @ 13;
    /// Add-dTD tripwire. Guards appending a descriptor to a running endpoint.
    atdtw / set_atdtw @ 14;
);
uints!(Usbcmd;
    /// Interrupt threshold, in micro-frames. Zero means no coalescing, which
    /// is the lowest-latency setting.
    itc / set_itc: u8, 16, 8;
);

fieldset!(
    /// USB status register. Every bit modelled here is write-1-to-clear.
    Usbsts
);
bits!(Usbsts;
    /// USB interrupt: a transfer completed. Level-driven by `ENDPTCOMPLETE`,
    /// so clearing this without also clearing that re-fires immediately.
    ui / set_ui @ 0;
    /// USB error interrupt.
    uei / set_uei @ 1;
    /// Port change detect.
    pci / set_pci @ 2;
    /// Frame list rollover.
    fri / set_fri @ 3;
    /// System error - an AHB transaction from the USB master faulted.
    sei / set_sei @ 4;
    /// USB reset received.
    uri / set_uri @ 6;
    /// Start of frame.
    sri / set_sri @ 7;
    /// Device controller suspend.
    sli / set_sli @ 8;
    /// NAK interrupt.
    naki / set_naki @ 16;
);

fieldset!(
    /// USB interrupt enable register. Bit positions mirror [`Usbsts`].
    Usbintr
);
bits!(Usbintr;
    ue / set_ue @ 0;
    uee / set_uee @ 1;
    pce / set_pce @ 2;
    fre / set_fre @ 3;
    see / set_see @ 4;
    ure / set_ure @ 6;
    sre / set_sre @ 7;
    sle / set_sle @ 8;
    nake / set_nake @ 16;
);

fieldset!(
    /// Current frame index.
    Frindex
);
uints!(Frindex;
    frindex / set_frindex: u16, 0, 14;
);

fieldset!(
    /// Device address.
    Deviceaddr
);
bits!(Deviceaddr;
    /// Advance on status: latch `usbadr` when the next status stage
    /// completes, rather than immediately. This is what makes SET_ADDRESS
    /// timing correct without a software delay.
    usbadra / set_usbadra @ 24;
);
uints!(Deviceaddr;
    usbadr / set_usbadr: u8, 25, 7;
);

fieldset!(
    /// Device queue-head list base address.
    ///
    /// The address field starts at bit 11, so the queue-head array must be
    /// 2048-byte aligned. Prefer `write_value(Endptlistaddr(addr))` over
    /// `set_epbase`, which takes an already-shifted value.
    Endptlistaddr
);
uints!(Endptlistaddr;
    epbase / set_epbase: u32, 11, 21;
);

fieldset!(
    /// Burst size for the USB bus master.
    Burstsize
);
uints!(Burstsize;
    rxpburst / set_rxpburst: u8, 0, 8;
    txpburst / set_txpburst: u8, 8, 8;
);

fieldset!(
    /// Port status and control.
    Portsc1
);
bits!(Portsc1;
    /// Current connect status - a cable is attached.
    ccs / set_ccs @ 0;
    /// Connect status change.
    csc / set_csc @ 1;
    /// Port enabled. Set by hardware at the end of the reset handshake.
    pe / set_pe @ 2;
    /// Port enable/disable change.
    pec / set_pec @ 3;
    /// Force port resume.
    fpr / set_fpr @ 6;
    /// Suspend.
    susp / set_susp @ 7;
    /// Port reset - the host is resetting us.
    pr / set_pr @ 8;
    /// High-speed port.
    hsp / set_hsp @ 9;
    /// PHY low-power clock disable.
    phcd / set_phcd @ 23;
    /// Port force full-speed connect.
    pfsc / set_pfsc @ 24;
);
enums!(Portsc1;
    /// Negotiated port speed. Only valid once [`Portsc1::pe`] is set.
    pspd / set_pspd: PortSpeed, 26, 2;
);

fieldset!(
    /// USB device mode.
    Usbmode
);
bits!(Usbmode;
    /// Setup lockout mode. Must be set (lockouts *off*) so that
    /// `ENDPTSETUPSTAT` can be cleared from task context via the tripwire.
    slom / set_slom @ 3;
    /// Stream disable.
    sdis / set_sdis @ 4;
    /// Big-endian descriptor and data structures. Leave clear.
    es / set_es @ 2;
);
enums!(Usbmode;
    cm / set_cm: ControllerMode, 0, 2;
);

fieldset!(
    /// Endpoint setup status. Bit `n` means a SETUP landed on endpoint `n`.
    Endptsetupstat
);
uints!(Endptsetupstat;
    endptsetupstat / set_endptsetupstat: u16, 0, 16;
);

fieldset!(
    /// Endpoint prime.
    ///
    /// Like [`Endptflush`], [`Endptstat`] and [`Endptcomplete`], the raw value
    /// is `IN << 16 | OUT`, so `.0` can be masked directly with the bit
    /// vocabulary the driver uses everywhere else.
    Endptprime
);
uints!(Endptprime;
    /// Prime OUT (receive) endpoints.
    perb / set_perb: u8, 0, 8;
    /// Prime IN (transmit) endpoints.
    petb / set_petb: u8, 16, 8;
);

fieldset!(
    /// Endpoint flush.
    Endptflush
);
uints!(Endptflush;
    ferb / set_ferb: u8, 0, 8;
    fetb / set_fetb: u8, 16, 8;
);

fieldset!(
    /// Endpoint status - which endpoints are currently primed.
    Endptstat
);
uints!(Endptstat;
    erbr / set_erbr: u8, 0, 8;
    etbr / set_etbr: u8, 16, 8;
);

fieldset!(
    /// Endpoint complete. Write-1-to-clear.
    Endptcomplete
);
uints!(Endptcomplete;
    erce / set_erce: u8, 0, 8;
    etce / set_etce: u8, 16, 8;
);

fieldset!(
    /// Endpoint control for hardware endpoint 0.
    ///
    /// A distinct fieldset from [`Endptctrl`]: endpoint 0 is permanently
    /// enabled and permanently a control endpoint, so it has no
    /// enable-controlling, data-toggle or stall-reset bits.
    Endptctrl0
);
bits!(Endptctrl0;
    /// RX stall. Hardware clears it on the next SETUP.
    rxs / set_rxs @ 0;
    /// RX enable. Read-only 1.
    rxe / set_rxe @ 7;
    /// TX stall. Hardware clears it on the next SETUP.
    txs / set_txs @ 16;
    /// TX enable. Read-only 1.
    txe / set_txe @ 23;
);
enums!(Endptctrl0;
    rxt / set_rxt: EpType, 2, 2;
    txt / set_txt: EpType, 18, 2;
);

fieldset!(
    /// Endpoint control for hardware endpoints 1 through 7.
    Endptctrl
);
bits!(Endptctrl;
    /// RX stall.
    rxs / set_rxs @ 0;
    /// RX data sink - for debug only, leave clear.
    rxd / set_rxd @ 1;
    /// RX data toggle inhibit. Leave clear outside of test modes.
    rxi / set_rxi @ 5;
    /// RX data toggle reset. Write 1 to reset the toggle to DATA0; required
    /// whenever an endpoint is enabled or a halt is cleared.
    rxr / set_rxr @ 6;
    /// RX enable.
    rxe / set_rxe @ 7;
    /// TX stall.
    txs / set_txs @ 16;
    /// TX data source - for debug only, leave clear.
    txd / set_txd @ 17;
    /// TX data toggle inhibit.
    txi / set_txi @ 21;
    /// TX data toggle reset.
    txr / set_txr @ 22;
    /// TX enable.
    txe / set_txe @ 23;
);
enums!(Endptctrl;
    rxt / set_rxt: EpType, 2, 2;
    txt / set_txt: EpType, 18, 2;
);

// ---------------------------------------------------------------------------
// Register block
// ---------------------------------------------------------------------------

/// Chipidea USB device controller register block.
#[derive(Copy, Clone, Eq, PartialEq)]
pub struct Usb {
    ptr: *mut u8,
}
unsafe impl Send for Usb {}
unsafe impl Sync for Usb {}

impl Usb {
    /// # Safety
    /// `ptr` must be the base address of a Chipidea USB controller, valid for
    /// the lifetime of the program.
    #[inline(always)]
    pub const unsafe fn from_ptr(ptr: *mut ()) -> Self {
        Self { ptr: ptr as _ }
    }

    #[inline(always)]
    pub const fn as_ptr(&self) -> *mut () {
        self.ptr as _
    }

    #[inline(always)]
    pub const fn id(self) -> Reg<Id, R> {
        unsafe { Reg::from_ptr(self.ptr.add(0x000) as _) }
    }

    #[inline(always)]
    pub const fn hwdevice(self) -> Reg<Hwdevice, R> {
        unsafe { Reg::from_ptr(self.ptr.add(0x00c) as _) }
    }

    #[inline(always)]
    pub const fn dciversion(self) -> Reg<Dciversion, R> {
        unsafe { Reg::from_ptr(self.ptr.add(0x120) as _) }
    }

    #[inline(always)]
    pub const fn dccparams(self) -> Reg<Dccparams, R> {
        unsafe { Reg::from_ptr(self.ptr.add(0x124) as _) }
    }

    #[inline(always)]
    pub const fn usbcmd(self) -> Reg<Usbcmd, RW> {
        unsafe { Reg::from_ptr(self.ptr.add(0x140) as _) }
    }

    #[inline(always)]
    pub const fn usbsts(self) -> Reg<Usbsts, RW> {
        unsafe { Reg::from_ptr(self.ptr.add(0x144) as _) }
    }

    #[inline(always)]
    pub const fn usbintr(self) -> Reg<Usbintr, RW> {
        unsafe { Reg::from_ptr(self.ptr.add(0x148) as _) }
    }

    #[inline(always)]
    pub const fn frindex(self) -> Reg<Frindex, R> {
        unsafe { Reg::from_ptr(self.ptr.add(0x14c) as _) }
    }

    #[inline(always)]
    pub const fn deviceaddr(self) -> Reg<Deviceaddr, RW> {
        unsafe { Reg::from_ptr(self.ptr.add(0x154) as _) }
    }

    #[inline(always)]
    pub const fn endptlistaddr(self) -> Reg<Endptlistaddr, RW> {
        unsafe { Reg::from_ptr(self.ptr.add(0x158) as _) }
    }

    #[inline(always)]
    pub const fn burstsize(self) -> Reg<Burstsize, RW> {
        unsafe { Reg::from_ptr(self.ptr.add(0x160) as _) }
    }

    #[inline(always)]
    pub const fn portsc1(self) -> Reg<Portsc1, RW> {
        unsafe { Reg::from_ptr(self.ptr.add(0x184) as _) }
    }

    #[inline(always)]
    pub const fn usbmode(self) -> Reg<Usbmode, RW> {
        unsafe { Reg::from_ptr(self.ptr.add(0x1a8) as _) }
    }

    #[inline(always)]
    pub const fn endptsetupstat(self) -> Reg<Endptsetupstat, RW> {
        unsafe { Reg::from_ptr(self.ptr.add(0x1ac) as _) }
    }

    #[inline(always)]
    pub const fn endptprime(self) -> Reg<Endptprime, RW> {
        unsafe { Reg::from_ptr(self.ptr.add(0x1b0) as _) }
    }

    #[inline(always)]
    pub const fn endptflush(self) -> Reg<Endptflush, RW> {
        unsafe { Reg::from_ptr(self.ptr.add(0x1b4) as _) }
    }

    /// Read-only: the controller owns this, software only observes it.
    #[inline(always)]
    pub const fn endptstat(self) -> Reg<Endptstat, R> {
        unsafe { Reg::from_ptr(self.ptr.add(0x1b8) as _) }
    }

    #[inline(always)]
    pub const fn endptcomplete(self) -> Reg<Endptcomplete, RW> {
        unsafe { Reg::from_ptr(self.ptr.add(0x1bc) as _) }
    }

    #[inline(always)]
    pub const fn endptctrl0(self) -> Reg<Endptctrl0, RW> {
        unsafe { Reg::from_ptr(self.ptr.add(0x1c0) as _) }
    }

    /// Endpoint control for hardware endpoint `n + 1`.
    ///
    /// The off-by-one is the hardware's: endpoint 0 lives in the separate
    /// [`Usb::endptctrl0`]. Callers should go through the driver's helper
    /// rather than indexing this directly.
    #[inline(always)]
    pub const fn endptctrl(self, n: usize) -> Reg<Endptctrl, RW> {
        // `::core::assert!` explicitly: `fmt.rs` shadows `assert!` with
        // defmt's, which is not const-evaluable.
        ::core::assert!(n < 7);
        unsafe { Reg::from_ptr(self.ptr.add(0x1c4 + n * 4) as _) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every accessor round-trips and lands on the bit the PAC documents.
    #[test]
    fn field_positions() {
        let mut c = Usbcmd(0);
        c.set_rs(true);
        assert_eq!(c.0, 1 << 0);
        c = Usbcmd(0);
        c.set_sutw(true);
        assert_eq!(c.0, 1 << 13);
        c = Usbcmd(0);
        c.set_atdtw(true);
        assert_eq!(c.0, 1 << 14);
        c = Usbcmd(0);
        c.set_itc(0xff);
        assert_eq!(c.0, 0xff << 16);
        assert_eq!(c.itc(), 0xff);

        let mut s = Usbsts(0);
        s.set_uri(true);
        assert_eq!(s.0, 1 << 6);
        assert!(s.uri());

        let mut d = Deviceaddr(0);
        d.set_usbadr(0x7f);
        d.set_usbadra(true);
        assert_eq!(d.0, (0x7f << 25) | (1 << 24));
        assert_eq!(d.usbadr(), 0x7f);

        let mut m = Usbmode(0);
        m.set_cm(ControllerMode::Device);
        m.set_slom(true);
        assert_eq!(m.0, 0b1010);
        assert_eq!(m.cm(), ControllerMode::Device);

        let mut e = Endptctrl(0);
        e.set_txe(true);
        e.set_txt(EpType::Bulk);
        e.set_txr(true);
        assert_eq!(e.0, (1 << 23) | (2 << 18) | (1 << 22));
        assert_eq!(e.txt(), EpType::Bulk);

        let mut p = Portsc1(0);
        p.set_pspd(PortSpeed::High);
        assert_eq!(p.0, 2 << 26);
        assert_eq!(p.pspd(), PortSpeed::High);
    }

    /// The IN-high/OUT-low layout the driver's `ep_mask` relies on.
    #[test]
    fn endpoint_bitmap_layout() {
        let mut p = Endptprime(0);
        p.set_perb(1 << 3);
        p.set_petb(1 << 5);
        assert_eq!(p.0, (1 << 3) | (1 << (16 + 5)));
    }

    /// `ENDPTLISTADDR` cannot represent the low 11 bits.
    #[test]
    fn endptlistaddr_alignment() {
        let a = Endptlistaddr(0x2000_0800);
        assert_eq!(a.epbase() << 11, 0x2000_0800);
    }

    /// Matches `embassy_usb_driver::EndpointType`'s encoding.
    #[test]
    fn ep_type_encoding() {
        use embassy_usb_driver::EndpointType;
        assert_eq!(EpType::Control.to_bits(), EndpointType::Control as u8);
        assert_eq!(EpType::Isochronous.to_bits(), EndpointType::Isochronous as u8);
        assert_eq!(EpType::Bulk.to_bits(), EndpointType::Bulk as u8);
        assert_eq!(EpType::Interrupt.to_bits(), EndpointType::Interrupt as u8);
    }
}
