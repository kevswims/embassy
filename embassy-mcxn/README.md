# Embassy NXP MCX-N MCUs HAL

A Hardware Abstraction Layer (HAL) for the NXP MCX-N family of
microcontrollers using the Embassy async framework.

**Status: early bring-up scaffold.** This crate currently only wires up
the `nxp-pac`-driven codegen (peripherals, interrupts, singletons) for
MCXN947. There are no drivers yet — no clock tree, no GPIO, no UART.
See `embassy-mcxa` for the pattern this crate is following as support
for MCXN947 (and siblings) is filled in.
