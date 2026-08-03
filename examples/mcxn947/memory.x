MEMORY
{
    /* RM "Table 16. System memory map": 0x0000_0000/0x2000_0000 are the
     * Non-Secure flash/RAM aliases (IDAU-fixed default at reset); SYSCON0/
     * SCG0 (and other peripherals) default to Secure-only access per the AHB
     * Secure Controller, so Non-Secure code faults touching them. Until we
     * set up TrustZone properly, run entirely as Secure code via the Secure
     * aliases (+0x1000_0000) instead.
     */
    FLASH : ORIGIN = 0x10000000, LENGTH = 2M
    RAM   : ORIGIN = 0x30000000, LENGTH = 256K
}
