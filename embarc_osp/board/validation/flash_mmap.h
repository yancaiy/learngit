#ifndef FLASH_MMAP_H
#define FLASH_MMAP_H

/* the memory mapping address for flash */
#define FLASH_MMAP_FIRMWARE_BASE    (0x300000)

/* flash address */
/* If you want change the start address of "FLASH_HEADER_BASE" or "FLASH_BOOT_BASE" or "FLASH_FIRMWARE_BASE",
   Don't forget to change the value in "settings" file under "C:\Calterah_downloader" accordingly */
#define FLASH_HEADER_BASE     (0x000000)
#define FLASH_DBG_CERT_BASE   (0x004000)
#define FLASH_HEADER_BK_BASE  (0x008000)

#define FLASH_ANTENNA_INFO_BASE  (0x010000)
#define FLASH_BOOT_BASE	       (0x020000)
#define FLASH_FIRMWARE_BASE	   (0x030000)

#define FLASH_ANGLE_CALIBRATION_INFO_BASE	(FLASH_FIRMWARE_BASE+0x100000) //ensure firmware space 1MB

#endif
