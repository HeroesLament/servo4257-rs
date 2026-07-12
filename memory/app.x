/* Application layout: flash after the 16K bootloader. The app region is
   108K (0x08004000..0x0801F7FF); the final 2K page (0x0801F800..0x0801FFFF)
   is reserved for APP-META (magic + CRC32 + length + version), erasable
   independently of the app image. RAM shrunk by 8 to reserve the shared
   boot flag (same addr as boot.x). */
MEMORY
{
  FLASH : ORIGIN = 0x08004000, LENGTH = 108K
  META  : ORIGIN = 0x0801F800, LENGTH = 2K
  RAM   : ORIGIN = 0x20000000, LENGTH = 24K - 8
}
_boot_flag = ORIGIN(RAM) + LENGTH(RAM);

/* Address of the APP-META page, for code that reads/writes the record.
   Mirrors app_meta::META_BASE — keep in sync. */
_app_meta_base = ORIGIN(META);
