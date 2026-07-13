/* Bootloader layout: lower 16K of flash.

   RAM is carved at BOTH ends so the toolchain never places .data/.bss/stack
   over our fixed-address debug words:
   - Bottom 0x40 bytes (0x20000000..0x2000003F) are reserved for the status /
     download-telemetry block (boot markers at 0x20000000, FlashBackend
     offset/stage at 0x20000020/0x20000024). CRITICAL: the HAL's flash
     program/erase routines are RAM-resident (.data); if .data started at
     0x20000000 the marker writes would corrupt that code mid-erase ->
     UNDEFINSTR HardFault. Reserving the low 0x40 keeps them clear.
   - Top 8 bytes hold the warm-reset boot flag (survives reset, never clobbered).
*/
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 16K
  RAM   : ORIGIN = 0x20000040, LENGTH = 24K - 8 - 0x40
}
_boot_flag = ORIGIN(RAM) + LENGTH(RAM); /* = 0x20005FF8, just past usable RAM */
