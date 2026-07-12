/* Bootloader layout: lower 16K of flash. RAM shrunk by 4 bytes so the boot
   flag sits ABOVE the region the stack/.bss/.data can use -> survives warm
   reset and is never clobbered by either bootloader or app. */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 16K
  RAM   : ORIGIN = 0x20000000, LENGTH = 24K - 8
}
_boot_flag = ORIGIN(RAM) + LENGTH(RAM); /* = 0x20005FFC, just past usable RAM */
