# Embassy NXP MCX-N MCUs HAL

A Hardware Abstraction Layer (HAL) for the NXP MCX-N family of
microcontrollers using the Embassy async framework.

**Status: early bring-up.** MCXN947 (core 0) only. The `nxp-pac`-driven codegen
— peripherals, interrupts, pin and channel singletons — is wired up, along with
drivers for GPIO, eDMA, LPUART, FlexCAN, USB HS (device), LPADC, VREF, CTIMER
(match/PWM/capture), LPTMR, OSTIMER (the `embassy-time` driver), RTC, WWDT, CRC,
and reset-reason reporting. Clocks come from the FRO tree plus SOSC (needed by
the USB PHY). The main clock runs at its 48 MHz reset default, at 144 MHz from
the FRO, or at 150 MHz from PLL0, with the core voltage and flash wait states
moved to match; PLL0 is not yet selectable as a peripheral clock source.

There are no SPI, I2C, I3C, or flash drivers yet. See `embassy-mcxa`
for the pattern this crate is following as support for MCXN947 (and siblings)
is filled in.

**See [`DRIVERS.md`](DRIVERS.md)** for the full per-driver inventory: which
instances are supported, what each driver does and does not implement, the
peripherals with no driver at all, and the known unverified hardware paths.
