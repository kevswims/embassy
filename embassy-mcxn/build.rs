use std::collections::HashMap;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::{env, fs};

use build_common::CfgSet;
use convert_case::{Casing, ccase};
use indexmap::IndexMap;
use nxp_pac::metadata::METADATA;
use proc_macro2::TokenStream;
use quote::{ToTokens, format_ident, quote};
use regex::Regex;

mod build_common;

fn main() {
    let mut cfgs = CfgSet::new();
    build_common::set_target_cfgs(&mut cfgs);

    fn driver_to_cfg_name(name: &str) -> String {
        match name.split_once("::") {
            Some((path, _block)) => path,
            None => name,
        }
        .replace("/", "_")
        .to_lowercase()
    }

    // Declare all drivers in nxp-pac (used or unused)
    for peripheral in nxp_pac::metadata::META_PERIPHERALS {
        cfgs.declare(&driver_to_cfg_name(peripheral));
    }

    // Enable all drivers for this chip
    for peripheral in METADATA.peripherals {
        if peripheral.driver_name.is_empty() {
            continue;
        }

        cfgs.enable(&driver_to_cfg_name(peripheral.driver_name));
    }

    let mut generated = vec![
        generate_peripherals_call(),
        generate_interrupt_mod_call(),
        generate_cc_gates(),
        generate_dma_channel_impls(),
        generate_instance_calls(),
        generate_gpio_pin_impls(),
        generate_dma_impls(),
    ];

    for (family, generate) in PIN_IMPLS {
        if PIN_FAMILIES.contains(family) {
            generated.push(generate());
        }
    }

    let out_dir = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let out_file = out_dir.join("_generated.rs");
    fs::write(&out_file, generated.into_iter().collect::<TokenStream>().to_string()).unwrap();
    rustfmt(&out_file);
}

/// Pin-trait families whose `impl_*_pin!` macros exist in the HAL.
///
/// The PAC metadata carries pin-mux signals for every peripheral in RM Table
/// 119, but a generator emits macro calls that only compile once the driver
/// defining them lands. Adding a family here is the last step of implementing
/// its pin traits, not the first.
const PIN_FAMILIES: &[&str] = &["ctimer", "lpuart"];

/// Pin-impl generators, keyed by the family name checked against
/// [`PIN_FAMILIES`]. GPIO is absent because it is unconditional.
#[allow(clippy::type_complexity)]
const PIN_IMPLS: &[(&str, fn() -> TokenStream)] = &[
    ("adc", generate_adc_pin_impls),
    ("clkout", generate_clkout_impls),
    ("ctimer", generate_ctimer_pin_impls),
    ("flexcan", generate_flexcan_pin_impls),
    ("flexspi", generate_flexspi_pin_impls),
    ("i3c", generate_i3c_pin_impls),
    ("lpi2c", generate_lpi2c_pin_impls),
    ("lpuart", generate_lpuart_pin_impls),
    ("spi", generate_spi_pin_impls),
];

fn generate_peripherals_call() -> TokenStream {
    struct Singleton {
        name: String,
        feature: Option<String>,
    }

    impl Singleton {
        pub fn just_name(name: impl ToString) -> Self {
            Self {
                name: name.to_string(),
                feature: None,
            }
        }
    }

    let mut singletons: Vec<Singleton> = Vec::new();
    // Add pins
    singletons.extend(METADATA.pins.iter().map(|pin| Singleton {
        name: pin.name.to_owned(),
        feature: pin.feature.map(|str| str.to_owned()),
    }));

    // Add peripherals
    singletons.extend(
        METADATA
            .peripherals
            .iter()
            .map(|peripheral| Singleton::just_name(peripheral.name)),
    );

    // Add DMA channels
    let dma_regex = Regex::new(r"(?i:^DMA)(\d+)").unwrap();
    let dma_channels_regex = Regex::new(r"(?i:DMA)(\d+)").unwrap();
    for (dma_num, dma_peripheral) in METADATA
        .peripherals
        .iter()
        .filter_map(|p| get_regex_num(p.name, &dma_regex).map(|dma_num| (dma_num, p)))
    {
        let channels_num = get_regex_num(dma_peripheral.driver_name, &dma_channels_regex).unwrap();

        for channel in 0..channels_num {
            singletons.push(Singleton::just_name(format!("DMA{dma_num}_CH{channel}")));
        }
    }

    // Add CTIMER channels
    let ctimer_regex = Regex::new(r"(?i:^CTIMER)(\d+)").unwrap();
    for ctimer_num in METADATA
        .peripherals
        .iter()
        .filter_map(|p| get_regex_num(p.name, &ctimer_regex))
    {
        for num in 0..4 {
            singletons.push(Singleton::just_name(format!("CTIMER{ctimer_num}_CH{num}")));
        }
    }

    // Output the singletons
    let singleton_tokens: Vec<_> = singletons
        .iter()
        .map(|s| {
            let feature = s
                .feature
                .as_ref()
                .map_or(TokenStream::default(), |feature| quote! { #[cfg(feature = #feature)] });
            let name = format_ident!("{}", s.name);
            quote! { #feature #name }
        })
        .collect();

    quote! {
        embassy_hal_internal::peripherals!(#(#singleton_tokens),*);
    }
}

fn generate_interrupt_mod_call() -> TokenStream {
    let mut irqs = Vec::new();
    for (name, _) in METADATA.interrupts {
        irqs.push(format_ident!("{}", name));
    }

    quote! {
        embassy_hal_internal::interrupt_mod!(
            #(
                #irqs,
            )*
        );
    }
}

fn generate_cc_gates() -> TokenStream {
    let mut generated = TokenStream::new();

    for peripheral in METADATA.peripherals {
        let Some(gate) = peripheral.gate.as_ref() else {
            continue;
        };

        let peripheral_name = format_ident!("{}", peripheral.name);
        let enable = format_ident!("{}", gate.enable);
        let reset = gate.reset.map(|reset| format_ident!("{reset}"));
        let config = gate
            .config
            .map(|config| format_ident!("{config}"))
            .unwrap_or_else(|| format_ident!("NoConfig"));
        let bit = format_ident!("{}", gate.bit);

        match reset {
            Some(reset) => generated.extend(quote! {
                crate::impl_cc_gate!(#peripheral_name, #enable, #reset, #bit, #config);
            }),
            None => generated.extend(quote! {
                crate::impl_cc_gate!(#peripheral_name, #enable, #bit, #config);
            }),
        }
    }

    generated
}

fn pin_feature_gate(pin_name: &str) -> TokenStream {
    let pin = METADATA
        .pins
        .iter()
        .find(|pin| pin.name == pin_name)
        .expect(&format!("Failed to find pin {pin_name}"));
    pin.feature
        .as_ref()
        .map_or(TokenStream::default(), |feature| quote! { #[cfg(feature = #feature)] })
}

fn generate_instance_calls() -> TokenStream {
    let mut generated = TokenStream::new();

    // Peripherals whose driver defines an `impl_<name>_instance!` macro. Adding
    // a name here without the matching macro breaks the build, so this list
    // grows as drivers land.
    const REQUIRES_INSTANCE: &[&str] = &["adc", "crc", "ctimer", "gpio", "lptmr", "lpuart", "rtc", "vref", "wwdt"];

    let peripheral_regex = Regex::new(r"(^.*\D)(\d+)?").unwrap();

    for peripheral in METADATA.peripherals {
        if peripheral.driver_name.is_empty() {
            // Only use peripherals that have a driver
            continue;
        }

        let Some(captures) = peripheral_regex.captures(peripheral.name) else {
            panic!("Weird peripheral name: {}", peripheral.name);
        };

        let peripheral_name = captures.get(1).unwrap().as_str().to_lowercase();
        let peripheral_number = captures.get(2).map(|m| m.as_str().parse::<u32>().unwrap());

        if REQUIRES_INSTANCE.contains(&peripheral_name.as_str()) {
            let arg = if let Some(peripheral_number) = peripheral_number {
                proc_macro2::Literal::u32_unsuffixed(peripheral_number).into_token_stream()
            } else {
                TokenStream::new()
            };

            let macro_ident = format_ident!("impl_{peripheral_name}_instance");

            generated.extend(quote! {
                crate::#macro_ident!(#arg);
            });
        }
    }

    generated
}

fn generate_gpio_pin_impls() -> TokenStream {
    let mut generated = TokenStream::new();

    let gpio_regex = Regex::new(r"(?i:^GPIO)(\d+)").unwrap();
    for (gpio_num, gpio_peripheral) in METADATA
        .peripherals
        .iter()
        .filter_map(|p| get_regex_num(p.name, &gpio_regex).map(|gpio_num| (gpio_num, p)))
    {
        let peripheral = format_ident!("{}", gpio_peripheral.name);
        let gpio_num = proc_macro2::Literal::u32_unsuffixed(gpio_num);

        for s in gpio_peripheral.signals {
            assert_eq!(s.pins.len(), 1, "Each gpio signal should only have 1 pin: {s:?}");
            let pin = format_ident!("{}", s.pins[0].pin);
            let pin_num = proc_macro2::Literal::u32_unsuffixed(s.name.parse().unwrap());
            let feature_gate = pin_feature_gate(s.pins[0].pin);

            generated.extend(quote! {
                #feature_gate
                crate::impl_gpio_pin!(#pin, #gpio_num, #pin_num, #peripheral);
            });
        }
    }

    generated
}

fn generate_adc_pin_impls() -> TokenStream {
    let mut generated = TokenStream::new();

    let adc_regex = Regex::new(r"^ADC\d+").unwrap();
    let adc_channel_regex = Regex::new(r"(?:^A)(\d+)").unwrap();
    for adc in METADATA.peripherals.iter().filter(|p| adc_regex.is_match(p.name)) {
        let adc_name = format_ident!("{}", adc.name);
        for signal in adc.signals {
            let channel: u8 = get_regex_num(signal.name, &adc_channel_regex)
                .expect(&format!("Could not get ADC channel from: {}", signal.name))
                .try_into()
                .unwrap();
            for pin in signal.pins {
                let pin_name = format_ident!("{}", pin.pin);
                let feature_gate = pin_feature_gate(pin.pin);

                generated.extend(quote! {
                    #feature_gate
                    crate::impl_adc_pin!(#pin_name, #adc_name, #channel);
                });
            }
        }
    }

    generated
}

fn generate_clkout_impls() -> TokenStream {
    let mut generated = TokenStream::new();

    for clkout in METADATA
        .peripherals
        .iter()
        .filter(|p| p.name.to_ascii_lowercase() == "clkout")
    {
        for signal in clkout.signals {
            for pin in signal.pins {
                let pin_name = format_ident!("{}", pin.pin);
                let mux = format_ident!("Mux{}", pin.alt);
                let feature_gate = pin_feature_gate(pin.pin);

                generated.extend(quote! {
                    #feature_gate
                    crate::impl_clkout_pin!(#pin_name, #mux);
                });
            }
        }
    }

    generated
}

fn generate_lpi2c_pin_impls() -> TokenStream {
    let mut generated = TokenStream::new();

    let lpi2c_regex = Regex::new(r"^LPI2C\d+").unwrap();
    for lpi2c in METADATA.peripherals.iter().filter(|p| lpi2c_regex.is_match(p.name)) {
        let lpi2c_name = format_ident!("{}", lpi2c.name);
        for signal in lpi2c.signals {
            let signal_pin = format_ident!("{}Pin", ccase!(pascal, signal.name));
            for pin in signal.pins {
                let pin_name = format_ident!("{}", pin.pin);
                let mux = format_ident!("Mux{}", pin.alt);
                let feature_gate = pin_feature_gate(pin.pin);

                generated.extend(quote! {
                    #feature_gate
                    crate::impl_lpi2c_pin!(#pin_name, #lpi2c_name, #mux, #signal_pin);
                });
            }
        }
    }

    generated
}

fn generate_i3c_pin_impls() -> TokenStream {
    let mut generated = TokenStream::new();

    let i3c_regex = Regex::new(r"^I3C\d+").unwrap();
    for i3c in METADATA.peripherals.iter().filter(|p| i3c_regex.is_match(p.name)) {
        let i3c_name = format_ident!("{}", i3c.name);
        for signal in i3c.signals {
            let signal_pin = format_ident!("{}Pin", ccase!(pascal, signal.name));
            for pin in signal.pins {
                let pin_name = format_ident!("{}", pin.pin);
                let mux = format_ident!("Mux{}", pin.alt);
                let feature_gate = pin_feature_gate(pin.pin);

                generated.extend(quote! {
                    #feature_gate
                    crate::impl_i3c_pin!(#pin_name, #i3c_name, #mux, #signal_pin);
                });
            }
        }
    }

    generated
}

fn generate_spi_pin_impls() -> TokenStream {
    let mut generated = TokenStream::new();

    let spi_regex = Regex::new(r"^LPSPI\d+").unwrap();
    for spi in METADATA.peripherals.iter().filter(|p| spi_regex.is_match(p.name)) {
        let spi_name = format_ident!("{}", spi.name);
        for signal in spi.signals {
            let signal_pin = format_ident!("{}Pin", ccase!(pascal, signal.name));
            for pin in signal.pins {
                let pin_name = format_ident!("{}", pin.pin);
                let mux = format_ident!("Mux{}", pin.alt);
                let feature_gate = pin_feature_gate(pin.pin);

                generated.extend(quote! {
                    #feature_gate
                    crate::impl_spi_pin!(#pin_name, #spi_name, #mux, #signal_pin);
                });
            }
        }
    }

    generated
}

fn generate_flexspi_pin_impls() -> TokenStream {
    let mut generated = TokenStream::new();

    let flexspi_regex = Regex::new(r"^FLEXSPI\d+").unwrap();
    for flexspi in METADATA.peripherals.iter().filter(|p| flexspi_regex.is_match(p.name)) {
        let flexspi_name = format_ident!("{}", flexspi.name);

        for signal in flexspi.signals {
            let mut name_split = signal.name.split("_");
            let port = format_ident!("{}", name_split.next().unwrap());
            let signal_name = format_ident!("{}", name_split.next().unwrap().to_case(convert_case::Case::Pascal));

            for pin in signal.pins {
                let pin_name = format_ident!("{}", pin.pin);
                let feature_gate = pin_feature_gate(pin.pin);

                generated.extend(quote! {
                    #feature_gate
                    crate::impl_flexspi_pin!(#pin_name, #flexspi_name, #port, #signal_name);
                });
            }
        }
    }

    generated
}

fn generate_ctimer_pin_impls() -> TokenStream {
    let mut generated = TokenStream::new();

    let ctimer_regex = Regex::new(r"^CTIMER\d+").unwrap();
    let input_regex = Regex::new(r"(?:^INP)(\d+)").unwrap();
    let match_channel_regex = Regex::new(r"(?:^MAT)(\d+)").unwrap();

    for ctimer in METADATA.peripherals.iter().filter(|p| ctimer_regex.is_match(p.name)) {
        let ctimer_name = format_ident!("{}", ctimer.name);

        let mut inp_pins = IndexMap::new();
        let mut mat_pins = IndexMap::new();

        // There are multiple INPn and MATn signals
        // And they share pins. We don't use that, so we need to filter out duplicates to prevent duplicate trait impls
        for signal in ctimer.signals {
            if let Some(inp_index) = get_regex_num(signal.name, &input_regex) {
                for pin in signal.pins {
                    let pin_name = format_ident!("{}", pin.pin);
                    let mux = format_ident!("Mux{}", pin.alt);
                    // Every INPn signal on this timer lists the same pins (RM:
                    // the 20 capture inputs are shared across all five CTIMERs
                    // through INPUTMUX), so a pin's INP index is a property of
                    // the pin, not of which timer we happen to be iterating.
                    inp_pins.insert(pin_name, (mux, inp_index as u8));
                }
            } else if let Some(match_index) = get_regex_num(signal.name, &match_channel_regex) {
                for pin in signal.pins {
                    let pin_name = format_ident!("{}", pin.pin);
                    let mux = format_ident!("Mux{}", pin.alt);
                    let feature_gate = pin_feature_gate(pin.pin);

                    mat_pins.insert(pin_name.clone(), mux);

                    let ctimer_channel = format_ident!("{}_CH{}", ctimer_name, match_index);
                    generated.extend(quote! {
                        #feature_gate
                        crate::impl_ctimer_match!(#ctimer_name, #ctimer_channel, #pin_name);
                    });
                }
            } else {
                unreachable!()
            }
        }

        for (pin_name, (mux, inp_index)) in inp_pins {
            let feature_gate = pin_feature_gate(&pin_name.to_string());
            generated.extend(quote! {
                #feature_gate
                crate::impl_ctimer_input_pin!(#pin_name, #ctimer_name, #mux, #inp_index);
            });
        }
        for (pin_name, mux) in mat_pins {
            let feature_gate = pin_feature_gate(&pin_name.to_string());
            generated.extend(quote! {
                #feature_gate
                crate::impl_ctimer_output_pin!(#pin_name, #ctimer_name, #mux);
            });
        }
    }

    generated
}

fn generate_lpuart_pin_impls() -> TokenStream {
    let mut generated = TokenStream::new();

    // Pin-mux signals live on LP_FLEXCOMMn, not LPUARTn - LPUART's own
    // `signals` are empty, since a flexcomm's pads are shared raw material for
    // whichever of UART/SPI/I2C/I2S its PSELID selects. RM 66.2.3's pure-UART
    // column (not the combined UART+I2C one) says P0 is RXD and P1 is TXD -
    // backwards from the intuitive reading, since the low-numbered pad is the
    // input. RTS_b/CTS_b (P2/P3) and the modem-control pads (P4-P6) are left
    // unmapped until a driver actually uses them.
    let flexcomm_regex = Regex::new(r"^LP_FLEXCOMM\d+").unwrap();
    let signal_names: &[(&str, &str)] = &[("P0", "RXD"), ("P1", "TXD")];

    for flexcomm in METADATA.peripherals.iter().filter(|p| flexcomm_regex.is_match(p.name)) {
        let n = &flexcomm.name["LP_FLEXCOMM".len()..];
        let lpuart_name = format_ident!("LPUART{n}");

        for signal in flexcomm.signals {
            let Some((_, uart_signal)) = signal_names.iter().find(|(pad, _)| *pad == signal.name) else {
                continue;
            };
            let signal_name = format_ident!("{uart_signal}");

            for pin in signal.pins {
                let pin_name = format_ident!("{}", pin.pin);
                let mux = format_ident!("Mux{}", pin.alt);
                let feature_gate = pin_feature_gate(pin.pin);

                generated.extend(quote! {
                    #feature_gate
                    crate::impl_lpuart_pin!(#lpuart_name, #pin_name, #mux, #signal_name);
                });
            }
        }
    }

    generated
}

fn generate_flexcan_pin_impls() -> TokenStream {
    let mut generated = TokenStream::new();
    let can_regex = Regex::new(r"^CAN\d+").unwrap();
    for can in METADATA.peripherals.iter().filter(|p| can_regex.is_match(p.name)) {
        let can_name = format_ident!("{}", can.name);
        for signal in can.signals {
            // Signal names from the PAC metadata are `TXD` / `RXD`, which the
            // `impl_flexcan_pin!` macro matches on to pick the right trait.
            let signal_name = format_ident!("{}", signal.name);
            for pin in signal.pins {
                let pin_name = format_ident!("{}", pin.pin);
                let mux = format_ident!("Mux{}", pin.alt);
                let feature_gate = pin_feature_gate(pin.pin);

                generated.extend(quote! {
                    #feature_gate
                    crate::impl_flexcan_pin!(#can_name, #pin_name, #mux, #signal_name);
                });
            }
        }
    }

    generated
}

fn generate_dma_channel_impls() -> TokenStream {
    let mut generated = TokenStream::new();

    let dma_regex = Regex::new(r"(?:^DMA)(\d+)").unwrap();
    let dma_driver_regex = Regex::new(r"(?:DMA)(\d+)").unwrap();

    for (dma, dma_num) in METADATA
        .peripherals
        .iter()
        .filter_map(|p| get_regex_num(p.name, &dma_regex).map(|num| (p, num)))
    {
        // The channel count is encoded in the block name, e.g. `mcxn/DMA::DMA16`.
        let num_channels = get_regex_num(dma.driver_name, &dma_driver_regex).unwrap();

        for channel_num in 0..num_channels {
            // The HAL singleton is `DMA{n}_CH{c}`, matching the `DMA{n}` parent
            // peripheral. The NVIC vector for that channel is spelled
            // `EDMA_{n}_CH{c}` on MCXN947, so the two idents differ - which is
            // why `impl_dma_channel!` takes the interrupt separately. A wrong
            // vector name is a compile error, not a dead handler: the
            // `#[cortex_m_rt::interrupt]` expansion checks the name against
            // `interrupt::*`.
            let channel_ident = format_ident!("DMA{dma_num}_CH{channel_num}");
            let irq_ident = format_ident!("EDMA_{dma_num}_CH{channel_num}");

            let dma_num = proc_macro2::Literal::u32_unsuffixed(dma_num);
            let channel_num = proc_macro2::Literal::u32_unsuffixed(channel_num);

            generated.extend(quote! {
                crate::impl_dma_channel!(#channel_ident, #dma_num, #channel_num, #irq_ident);
                crate::impl_dma_interrupt_handler!(#irq_ident, #dma_num, #channel_num);
            });
        }
    }

    generated
}

fn generate_dma_impls() -> TokenStream {
    let mut dma_requests = HashMap::new();
    for dma_mux in METADATA.peripherals.iter().flat_map(|p| p.dma_muxing) {
        dma_requests.insert(dma_mux.signal, dma_mux.request);
    }
    // The empty case is unreachable for mcxn947, which carries all 117 DMAMUX
    // request sources, but is kept for a chip whose metadata has none. Two
    // things must differ from the populated case there:
    //
    //   * `#[repr(u8)]` is rejected on a zero-variant enum, so it is omitted.
    //   * `from_number_unchecked` is omitted: `transmute`ing a `u8` into an
    //     uninhabited (zero-sized) type cannot compile.
    //
    // `number()` is still emitted, because `dma::set_request_source` calls it.
    // With no variants there is no way to obtain a `DmaRequest` to pass, so
    // that path is unreachable rather than broken.
    //
    // Note this map is keyed on the signal name alone: two entries sharing a
    // signal collapse silently, keeping whichever peripheral appears later in
    // the metadata. Duplicate *request* numbers are caught, but only as an
    // E0081 in the generated code rather than here.
    let mut sorted_dma_requests = dma_requests.into_iter().collect::<Vec<_>>();
    sorted_dma_requests.sort_unstable_by_key(|(_, request)| *request);
    if sorted_dma_requests.is_empty() {
        return quote! {
            /// DMA request sources
            #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
            #[allow(dead_code)]
            pub enum DmaRequest {}

            impl DmaRequest {
                /// Convert enumerated value into a raw integer
                #[allow(dead_code)]
                pub const fn number(self) -> u8 {
                    match self {}
                }
            }
        };
    }
    let enum_variants = sorted_dma_requests.into_iter().map(|(name, value)| {
        use convert_case::ccase;
        let name = format_ident!("{}", ccase!(pascal, name));
        quote! { #name = #value }
    });
    quote! {
        /// DMA request sources
        #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
        #[repr(u8)]
        #[allow(dead_code)]
        pub enum DmaRequest {
            #(#enum_variants),*
        }

        impl DmaRequest {
            /// Convert enumerated value into a raw integer
            pub const fn number(self) -> u8 {
                self as u8
            }

            /// Convert a raw integer into an enumerated value
            ///
            /// ## SAFETY
            ///
            /// The given number MUST be one of the defined variant, e.g. a number
            /// derived from [`Self::number()`], otherwise it is immediate undefined behavior.
            pub unsafe fn from_number_unchecked(num: u8) -> Self {
                unsafe { core::mem::transmute(num) }
            }
        }
    }
}

/// rustfmt a given path.
/// Failures are logged to stderr and ignored.
fn rustfmt(path: impl AsRef<Path>) {
    let path = path.as_ref();
    match Command::new("rustfmt").args([path]).output() {
        Err(e) => {
            eprintln!("failed to exec rustfmt {:?}: {:?}", path, e);
        }
        Ok(out) => {
            if !out.status.success() {
                eprintln!("rustfmt {:?} failed:", path);
                eprintln!("=== STDOUT:");
                std::io::stderr().write_all(&out.stdout).unwrap();
                eprintln!("=== STDERR:");
                std::io::stderr().write_all(&out.stderr).unwrap();
            }
        }
    }
}

fn get_regex_num(string: &str, regex: &Regex) -> Option<u32> {
    regex
        .captures(string)
        .map(|cap| cap.extract::<1>().1[0].parse::<u32>().unwrap())
}
