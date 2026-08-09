//! USB high-speed bulk loopback, with no USB class involved.
//!
//! A vendor-specific (class `0xFF`) interface with one bulk OUT and one bulk
//! IN endpoint, echoing whatever the host sends. Deliberately class-free, so
//! that a failure points at the driver rather than at a class implementation -
//! but still built through `embassy_usb::Builder`, because enumeration is what
//! exercises the hard parts of the Chipidea control pipe (the SETUP tripwire,
//! zero-length status stages, and `SET_ADDRESS` timing).
//!
//! Requires an external crystal: the USB HS PHY's 480 MHz PLL has no reference
//! other than `clk_in`, so [`SoscConfig`] must match the board's crystal.
//!
//! Test it from the host with pyusb:
//!
//! ```ignore
//! import usb.core, usb.util
//! dev = usb.core.find(idVendor=0xc0de, idProduct=0xcafe)
//! print("speed:", dev.speed)          # 3 == high speed
//! dev.set_configuration()
//! intf = dev.get_active_configuration()[(0, 0)]
//! out = usb.util.find_descriptor(intf, custom_match=lambda e:
//!     usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT)
//! inp = usb.util.find_descriptor(intf, custom_match=lambda e:
//!     usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN)
//!
//! for payload in [b"hello world", bytes(range(64)), bytes(out.wMaxPacketSize)]:
//!     out.write(payload)
//!     got = bytes(inp.read(len(payload) or 1, timeout=1000))
//!     assert got == payload, (len(got), len(payload))
//!     print("ok", len(payload))
//! ```
//!
//! The third payload is the one that matters: a transfer that is an exact
//! multiple of the packet size is the zero-length-packet case, and short
//! packet termination is what Chipidea drivers most often get wrong.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_mcxn as hal;
use embassy_usb::driver::{Endpoint, EndpointIn, EndpointOut};
use embassy_usb::msos::{self, windows_version};
use embassy_usb::{Builder, Config, UsbDeviceSpeed};
use hal::bind_interrupts;
use hal::clocks::{SoscConfig, SoscMode};
use hal::usb::{self, UsbHs};
use panic_probe as _;

/// Randomly generated, so Windows can bind WinUSB to the device.
const DEVICE_INTERFACE_GUIDS: &[&str] = &["{B7B0B4D0-3E5F-4C1E-9A2B-1D9E9C0A55F3}"];

/// The FRDM-MCXN947 fits a 24 MHz crystal. Change this to match your board -
/// nothing measures it, and a wrong value shows up as a PHY PLL lock failure.
const CRYSTAL_HZ: u32 = 24_000_000;

/// Raise to 512 once high speed is confirmed working; 64 is valid at either
/// speed, so it keeps first bring-up honest. A device that enumerates at full
/// speed while advertising 512-byte bulk endpoints is illegal, and Windows
/// rejects it in a way that is tedious to diagnose.
const BULK_PACKET_SIZE: u16 = 64;

bind_interrupts!(struct Irqs {
    USB1_HS => usb::InterruptHandler<UsbHs>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut cfg = hal::config::Config::default();
    cfg.clock_cfg.sosc = Some(SoscConfig {
        hz: CRYSTAL_HZ,
        mode: SoscMode::Crystal,
    });

    let p = hal::init(cfg);

    defmt::info!("USB HS bulk loopback example");

    let driver = match usb::Driver::new(
        p.USBHS1__USBC,
        p.USBPHY,
        p.USBHS1__USBNC,
        Irqs,
        usb::Config::default(),
    ) {
        Ok(d) => d,
        Err(e) => {
            defmt::panic!("USB bring-up failed: {:?}", e);
        }
    };

    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("MCXN947 USB HS bulk loopback");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    // High-speed control endpoints are always 64 bytes.
    config.max_packet_size_0 = 64;
    // The driver cannot tell the stack what speed was negotiated, so this has
    // to agree with `usb::Config::high_speed` (which defaults to true).
    config.max_speed = UsbDeviceSpeed::High;

    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    // Tell Windows the whole device is WinUSB-compatible, so pyusb/libusb can
    // open it without a custom driver or INF.
    builder.msos_descriptor(windows_version::WIN8_1, 0);
    builder.msos_feature(msos::CompatibleIdFeatureDescriptor::new("WINUSB", ""));
    builder.msos_feature(msos::RegistryPropertyFeatureDescriptor::new(
        "DeviceInterfaceGUIDs",
        msos::PropertyData::RegMultiSz(DEVICE_INTERFACE_GUIDS),
    ));

    let mut function = builder.function(0xFF, 0, 0);
    let mut interface = function.interface();
    let mut alt = interface.alt_setting(0xFF, 0, 0, None);
    let mut read_ep = alt.endpoint_bulk_out(None, BULK_PACKET_SIZE);
    let mut write_ep = alt.endpoint_bulk_in(None, BULK_PACKET_SIZE);
    drop(function);

    let mut usb = builder.build();
    let usb_fut = usb.run();

    let echo_fut = async {
        let mut data = [0u8; BULK_PACKET_SIZE as usize];
        loop {
            read_ep.wait_enabled().await;
            defmt::info!("connected");

            loop {
                match read_ep.read(&mut data).await {
                    Ok(n) => {
                        defmt::info!("got {} bytes", n);
                        if write_ep.write(&data[..n]).await.is_err() {
                            break;
                        }
                    }
                    Err(e) => {
                        defmt::warn!("read failed: {:?}", e);
                        break;
                    }
                }
            }

            defmt::info!("disconnected");
        }
    };

    join(usb_fut, echo_fut).await;
}
