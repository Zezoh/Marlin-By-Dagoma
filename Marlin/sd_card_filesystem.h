/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * Arduino SdFat Library
 * Copyright (C) 2009 by William Greiman
 *
 * This file is part of the Arduino Sd2Card Library
 */

#include "Marlin.h"
#if ENABLED(SDSUPPORT)

#ifndef sd_card_filesystem_h
#define sd_card_filesystem_h

//==============================================================================
// SDFATCONFIG - Configuration defines
//==============================================================================
  #include <stdint.h>
  //------------------------------------------------------------------------------
  /**
  * To use multiple SD cards set USE_MULTIPLE_CARDS nonzero.
  *
  * Using multiple cards costs 400 - 500  bytes of flash.
  *
  * Each card requires about 550 bytes of SRAM so use of a Mega is recommended.
  */
  #define USE_MULTIPLE_CARDS 0
  //------------------------------------------------------------------------------
  /**
  * Call flush for endl if ENDL_CALLS_FLUSH is nonzero
  *
  * The standard for iostreams is to call flush.  This is very costly for
  * SdFat.  Each call to flush causes 2048 bytes of I/O to the SD.
  *
  * SdFat has a single 512 byte buffer for SD I/O so it must write the current
  * data block to the SD, read the directory block from the SD, update the
  * directory entry, write the directory block to the SD and read the data
  * block back into the buffer.
  *
  * The SD flash memory controller is not designed for this many rewrites
  * so performance may be reduced by more than a factor of 100.
  *
  * If ENDL_CALLS_FLUSH is zero, you must call flush and/or close to force
  * all data to be written to the SD.
  */
  #define ENDL_CALLS_FLUSH 0
  //------------------------------------------------------------------------------
  /**
  * Allow use of deprecated functions if ALLOW_DEPRECATED_FUNCTIONS is nonzero
  */
  #define ALLOW_DEPRECATED_FUNCTIONS 1
  //------------------------------------------------------------------------------
  /**
  * Allow FAT12 volumes if FAT12_SUPPORT is nonzero.
  * FAT12 has not been well tested.
  */
  #define FAT12_SUPPORT 0
  //------------------------------------------------------------------------------
  /**
  * SPI init rate for SD initialization commands. Must be 5 (F_CPU/64)
  * or 6 (F_CPU/128).
  */
  #define SPI_SD_INIT_RATE 5
  //------------------------------------------------------------------------------
  /**
  * Set the SS pin high for hardware SPI.  If SS is chip select for another SPI
  * device this will disable that device during the SD init phase.
  */
  #define SET_SPI_SS_HIGH 1
  //------------------------------------------------------------------------------
  /**
  * Define MEGA_SOFT_SPI nonzero to use software SPI on Mega Arduinos.
  * Pins used are SS 10, MOSI 11, MISO 12, and SCK 13.
  *
  * MEGA_SOFT_SPI allows an unmodified Adafruit GPS Shield to be used
  * on Mega Arduinos.  Software SPI works well with GPS Shield V1.1
  * but many SD cards will fail with GPS Shield V1.0.
  */
  #define MEGA_SOFT_SPI 0
  //------------------------------------------------------------------------------
  /**
  * Set USE_SOFTWARE_SPI nonzero to always use software SPI.
  */
  #define USE_SOFTWARE_SPI 0
  // define software SPI pins so Mega can use unmodified 168/328 shields
  /** Software SPI chip select pin for the SD */
  uint8_t const SOFT_SPI_CS_PIN = 10;
  /** Software SPI Master Out Slave In pin */
  uint8_t const SOFT_SPI_MOSI_PIN = 11;
  /** Software SPI Master In Slave Out pin */
  uint8_t const SOFT_SPI_MISO_PIN = 12;
  /** Software SPI Clock pin */
  uint8_t const SOFT_SPI_SCK_PIN = 13;
  //------------------------------------------------------------------------------
  /**
  * The __cxa_pure_virtual function is an error handler that is invoked when
  * a pure virtual function is called.
  */
  #define USE_CXA_PURE_VIRTUAL 1

  /** Number of UTF-16 characters per entry */
  #define FILENAME_LENGTH 13

  /**
  * Defines for long (vfat) filenames
  */
  /** Number of VFAT entries used. Every entry has 13 UTF-16 characters */
  #define MAX_VFAT_ENTRIES (2)
  /** Total size of the buffer used to store the long filenames */
  #define LONG_FILENAME_LENGTH (FILENAME_LENGTH*MAX_VFAT_ENTRIES+1)


//==============================================================================
// SDINFO - SD card information structures
//==============================================================================
#include <stdint.h>
// Based on the document:
//
// SD Specifications
// Part 1
// Physical Layer
// Simplified Specification
// Version 3.01
// May 18, 2010
//
// http://www.sdcard.org/developers/tech/sdcard/pls/simplified_specs
//------------------------------------------------------------------------------
// SD card commands
/** GO_IDLE_STATE - init card in spi mode if CS low */
uint8_t const CMD0 = 0X00;
/** SEND_IF_COND - verify SD Memory Card interface operating condition.*/
uint8_t const CMD8 = 0X08;
/** SEND_CSD - read the Card Specific Data (CSD register) */
uint8_t const CMD9 = 0X09;
/** SEND_CID - read the card identification information (CID register) */
uint8_t const CMD10 = 0X0A;
/** STOP_TRANSMISSION - end multiple block read sequence */
uint8_t const CMD12 = 0X0C;
/** SEND_STATUS - read the card status register */
uint8_t const CMD13 = 0X0D;
/** READ_SINGLE_BLOCK - read a single data block from the card */
uint8_t const CMD17 = 0X11;
/** READ_MULTIPLE_BLOCK - read a multiple data blocks from the card */
uint8_t const CMD18 = 0X12;
/** WRITE_BLOCK - write a single data block to the card */
uint8_t const CMD24 = 0X18;
/** WRITE_MULTIPLE_BLOCK - write blocks of data until a STOP_TRANSMISSION */
uint8_t const CMD25 = 0X19;
/** ERASE_WR_BLK_START - sets the address of the first block to be erased */
uint8_t const CMD32 = 0X20;
/** ERASE_WR_BLK_END - sets the address of the last block of the continuous
    range to be erased*/
uint8_t const CMD33 = 0X21;
/** ERASE - erase all previously selected blocks */
uint8_t const CMD38 = 0X26;
/** APP_CMD - escape for application specific command */
uint8_t const CMD55 = 0X37;
/** READ_OCR - read the OCR register of a card */
uint8_t const CMD58 = 0X3A;
/** SET_WR_BLK_ERASE_COUNT - Set the number of write blocks to be
     pre-erased before writing */
uint8_t const ACMD23 = 0X17;
/** SD_SEND_OP_COMD - Sends host capacity support information and
    activates the card's initialization process */
uint8_t const ACMD41 = 0X29;
//------------------------------------------------------------------------------
/** status for card in the ready state */
uint8_t const R1_READY_STATE = 0X00;
/** status for card in the idle state */
uint8_t const R1_IDLE_STATE = 0X01;
/** status bit for illegal command */
uint8_t const R1_ILLEGAL_COMMAND = 0X04;
/** start data token for read or write single block*/
uint8_t const DATA_START_BLOCK = 0XFE;
/** stop token for write multiple blocks*/
uint8_t const STOP_TRAN_TOKEN = 0XFD;
/** start data token for write multiple blocks*/
uint8_t const WRITE_MULTIPLE_TOKEN = 0XFC;
/** mask for data response tokens after a write block operation */
uint8_t const DATA_RES_MASK = 0X1F;
/** write data accepted token */
uint8_t const DATA_RES_ACCEPTED = 0X05;
//------------------------------------------------------------------------------
/** Card IDentification (CID) register */
typedef struct CID {
  // byte 0
  /** Manufacturer ID */
  unsigned char mid;
  // byte 1-2
  /** OEM/Application ID */
  char oid[2];
  // byte 3-7
  /** Product name */
  char pnm[5];
  // byte 8
  /** Product revision least significant digit */
  unsigned char prv_m : 4;
  /** Product revision most significant digit */
  unsigned char prv_n : 4;
  // byte 9-12
  /** Product serial number */
  uint32_t psn;
  // byte 13
  /** Manufacturing date year low digit */
  unsigned char mdt_year_high : 4;
  /** not used */
  unsigned char reserved : 4;
  // byte 14
  /** Manufacturing date month */
  unsigned char mdt_month : 4;
  /** Manufacturing date year low digit */
  unsigned char mdt_year_low : 4;
  // byte 15
  /** not used always 1 */
  unsigned char always1 : 1;
  /** CRC7 checksum */
  unsigned char crc : 7;
} cid_t;
//------------------------------------------------------------------------------
/** CSD for version 1.00 cards */
typedef struct CSDV1 {
  // byte 0
  unsigned char reserved1 : 6;
  unsigned char csd_ver : 2;
  // byte 1
  unsigned char taac;
  // byte 2
  unsigned char nsac;
  // byte 3
  unsigned char tran_speed;
  // byte 4
  unsigned char ccc_high;
  // byte 5
  unsigned char read_bl_len : 4;
  unsigned char ccc_low : 4;
  // byte 6
  unsigned char c_size_high : 2;
  unsigned char reserved2 : 2;
  unsigned char dsr_imp : 1;
  unsigned char read_blk_misalign : 1;
  unsigned char write_blk_misalign : 1;
  unsigned char read_bl_partial : 1;
  // byte 7
  unsigned char c_size_mid;
  // byte 8
  unsigned char vdd_r_curr_max : 3;
  unsigned char vdd_r_curr_min : 3;
  unsigned char c_size_low : 2;
  // byte 9
  unsigned char c_size_mult_high : 2;
  unsigned char vdd_w_cur_max : 3;
  unsigned char vdd_w_curr_min : 3;
  // byte 10
  unsigned char sector_size_high : 6;
  unsigned char erase_blk_en : 1;
  unsigned char c_size_mult_low : 1;
  // byte 11
  unsigned char wp_grp_size : 7;
  unsigned char sector_size_low : 1;
  // byte 12
  unsigned char write_bl_len_high : 2;
  unsigned char r2w_factor : 3;
  unsigned char reserved3 : 2;
  unsigned char wp_grp_enable : 1;
  // byte 13
  unsigned char reserved4 : 5;
  unsigned char write_partial : 1;
  unsigned char write_bl_len_low : 2;
  // byte 14
  unsigned char reserved5: 2;
  unsigned char file_format : 2;
  unsigned char tmp_write_protect : 1;
  unsigned char perm_write_protect : 1;
  unsigned char copy : 1;
  /** Indicates the file format on the card */
  unsigned char file_format_grp : 1;
  // byte 15
  unsigned char always1 : 1;
  unsigned char crc : 7;
} csd1_t;
//------------------------------------------------------------------------------
/** CSD for version 2.00 cards */
typedef struct CSDV2 {
  // byte 0
  unsigned char reserved1 : 6;
  unsigned char csd_ver : 2;
  // byte 1
  /** fixed to 0X0E */
  unsigned char taac;
  // byte 2
  /** fixed to 0 */
  unsigned char nsac;
  // byte 3
  unsigned char tran_speed;
  // byte 4
  unsigned char ccc_high;
  // byte 5
  /** This field is fixed to 9h, which indicates READ_BL_LEN=512 Byte */
  unsigned char read_bl_len : 4;
  unsigned char ccc_low : 4;
  // byte 6
  /** not used */
  unsigned char reserved2 : 4;
  unsigned char dsr_imp : 1;
  /** fixed to 0 */
  unsigned char read_blk_misalign : 1;
  /** fixed to 0 */
  unsigned char write_blk_misalign : 1;
  /** fixed to 0 - no partial read */
  unsigned char read_bl_partial : 1;
  // byte 7
  /** not used */
  unsigned char reserved3 : 2;
  /** high part of card size */
  unsigned char c_size_high : 6;
  // byte 8
  /** middle part of card size */
  unsigned char c_size_mid;
  // byte 9
  /** low part of card size */
  unsigned char c_size_low;
  // byte 10
  /** sector size is fixed at 64 KB */
  unsigned char sector_size_high : 6;
  /** fixed to 1 - erase single is supported */
  unsigned char erase_blk_en : 1;
  /** not used */
  unsigned char reserved4 : 1;
  // byte 11
  unsigned char wp_grp_size : 7;
  /** sector size is fixed at 64 KB */
  unsigned char sector_size_low : 1;
  // byte 12
  /** write_bl_len fixed for 512 byte blocks */
  unsigned char write_bl_len_high : 2;
  /** fixed value of 2 */
  unsigned char r2w_factor : 3;
  /** not used */
  unsigned char reserved5 : 2;
  /** fixed value of 0 - no write protect groups */
  unsigned char wp_grp_enable : 1;
  // byte 13
  unsigned char reserved6 : 5;
  /** always zero - no partial block read*/
  unsigned char write_partial : 1;
  /** write_bl_len fixed for 512 byte blocks */
  unsigned char write_bl_len_low : 2;
  // byte 14
  unsigned char reserved7: 2;
  /** Do not use always 0 */
  unsigned char file_format : 2;
  unsigned char tmp_write_protect : 1;
  unsigned char perm_write_protect : 1;
  unsigned char copy : 1;
  /** Do not use always 0 */
  unsigned char file_format_grp : 1;
  // byte 15
  /** not used always 1 */
  unsigned char always1 : 1;
  /** checksum */
  unsigned char crc : 7;
} csd2_t;
//------------------------------------------------------------------------------
/** union of old and new style CSD register */
union csd_t {
  csd1_t v1;
  csd2_t v2;
};


//==============================================================================
// SDFATSTRUCTS - FAT filesystem data structures
//==============================================================================

#define PACKED __attribute__((__packed__))
/**
 * \file
 * \brief FAT file structures
 */
/**
 * mostly from Microsoft document fatgen103.doc
 * http://www.microsoft.com/whdc/system/platform/firmware/fatgen.mspx
 */
//------------------------------------------------------------------------------
/** Value for byte 510 of boot block or MBR */
uint8_t const BOOTSIG0 = 0X55;
/** Value for byte 511 of boot block or MBR */
uint8_t const BOOTSIG1 = 0XAA;
/** Value for bootSignature field int FAT/FAT32 boot sector */
uint8_t const EXTENDED_BOOT_SIG = 0X29;
//------------------------------------------------------------------------------
/**
 * \struct partitionTable
 * \brief MBR partition table entry
 *
 * A partition table entry for a MBR formatted storage device.
 * The MBR partition table has four entries.
 */
struct partitionTable {
          /**
           * Boot Indicator . Indicates whether the volume is the active
           * partition.  Legal values include: 0X00. Do not use for booting.
           * 0X80 Active partition.
           */
  uint8_t  boot;
          /**
           * Head part of Cylinder-head-sector address of the first block in
           * the partition. Legal values are 0-255. Only used in old PC BIOS.
           */
  uint8_t  beginHead;
          /**
           * Sector part of Cylinder-head-sector address of the first block in
           * the partition. Legal values are 1-63. Only used in old PC BIOS.
           */
  unsigned beginSector : 6;
           /** High bits cylinder for first block in partition. */
  unsigned beginCylinderHigh : 2;
          /**
           * Combine beginCylinderLow with beginCylinderHigh. Legal values
           * are 0-1023.  Only used in old PC BIOS.
           */
  uint8_t  beginCylinderLow;
          /**
           * Partition type. See defines that begin with PART_TYPE_ for
           * some Microsoft partition types.
           */
  uint8_t  type;
          /**
           * head part of cylinder-head-sector address of the last sector in the
           * partition.  Legal values are 0-255. Only used in old PC BIOS.
           */
  uint8_t  endHead;
          /**
           * Sector part of cylinder-head-sector address of the last sector in
           * the partition.  Legal values are 1-63. Only used in old PC BIOS.
           */
  unsigned endSector : 6;
           /** High bits of end cylinder */
  unsigned endCylinderHigh : 2;
          /**
           * Combine endCylinderLow with endCylinderHigh. Legal values
           * are 0-1023.  Only used in old PC BIOS.
           */
  uint8_t  endCylinderLow;
           /** Logical block address of the first block in the partition. */
  uint32_t firstSector;
           /** Length of the partition, in blocks. */
  uint32_t totalSectors;
} PACKED;
/** Type name for partitionTable */
typedef struct partitionTable part_t;
//------------------------------------------------------------------------------
/**
 * \struct masterBootRecord
 *
 * \brief Master Boot Record
 *
 * The first block of a storage device that is formatted with a MBR.
 */
struct masterBootRecord {
           /** Code Area for master boot program. */
  uint8_t  codeArea[440];
           /** Optional Windows NT disk signature. May contain boot code. */
  uint32_t diskSignature;
           /** Usually zero but may be more boot code. */
  uint16_t usuallyZero;
           /** Partition tables. */
  part_t   part[4];
           /** First MBR signature byte. Must be 0X55 */
  uint8_t  mbrSig0;
           /** Second MBR signature byte. Must be 0XAA */
  uint8_t  mbrSig1;
} PACKED;
/** Type name for masterBootRecord */
typedef struct masterBootRecord mbr_t;
//------------------------------------------------------------------------------
/**
 * \struct fat_boot
 *
 * \brief Boot sector for a FAT12/FAT16 volume.
 *
 */
struct fat_boot {
         /**
          * The first three bytes of the boot sector must be valid,
          * executable x 86-based CPU instructions. This includes a
          * jump instruction that skips the next nonexecutable bytes.
          */
  uint8_t jump[3];
         /**
          * This is typically a string of characters that identifies
          * the operating system that formatted the volume.
          */
  char    oemId[8];
          /**
           * The size of a hardware sector. Valid decimal values for this
           * field are 512, 1024, 2048, and 4096. For most disks used in
           * the United States, the value of this field is 512.
           */
  uint16_t bytesPerSector;
          /**
           * Number of sectors per allocation unit. This value must be a
           * power of 2 that is greater than 0. The legal values are
           * 1, 2, 4, 8, 16, 32, 64, and 128.  128 should be avoided.
           */
  uint8_t  sectorsPerCluster;
          /**
           * The number of sectors preceding the start of the first FAT,
           * including the boot sector. The value of this field is always 1.
           */
  uint16_t reservedSectorCount;
          /**
           * The number of copies of the FAT on the volume.
           * The value of this field is always 2.
           */
  uint8_t  fatCount;
          /**
           * For FAT12 and FAT16 volumes, this field contains the count of
           * 32-byte directory entries in the root directory. For FAT32 volumes,
           * this field must be set to 0. For FAT12 and FAT16 volumes, this
           * value should always specify a count that when multiplied by 32
           * results in a multiple of bytesPerSector.  FAT16 volumes should
           * use the value 512.
           */
  uint16_t rootDirEntryCount;
          /**
           * This field is the old 16-bit total count of sectors on the volume.
           * This count includes the count of all sectors in all four regions
           * of the volume. This field can be 0; if it is 0, then totalSectors32
           * must be nonzero.  For FAT32 volumes, this field must be 0. For
           * FAT12 and FAT16 volumes, this field contains the sector count, and
           * totalSectors32 is 0 if the total sector count fits
           * (is less than 0x10000).
           */
  uint16_t totalSectors16;
          /**
           * This dates back to the old MS-DOS 1.x media determination and is
           * no longer usually used for anything.  0xF8 is the standard value
           * for fixed (nonremovable) media. For removable media, 0xF0 is
           * frequently used. Legal values are 0xF0 or 0xF8-0xFF.
           */
  uint8_t  mediaType;
          /**
           * Count of sectors occupied by one FAT on FAT12/FAT16 volumes.
           * On FAT32 volumes this field must be 0, and sectorsPerFat32
           * contains the FAT size count.
           */
  uint16_t sectorsPerFat16;
           /** Sectors per track for interrupt 0x13. Not used otherwise. */
  uint16_t sectorsPerTrack;
           /** Number of heads for interrupt 0x13.  Not used otherwise. */
  uint16_t headCount;
          /**
           * Count of hidden sectors preceding the partition that contains this
           * FAT volume. This field is generally only relevant for media
           * visible on interrupt 0x13.
           */
  uint32_t hiddenSectors;
          /**
           * This field is the new 32-bit total count of sectors on the volume.
           * This count includes the count of all sectors in all four regions
           * of the volume.  This field can be 0; if it is 0, then
           * totalSectors16 must be nonzero.
           */
  uint32_t totalSectors32;
           /**
            * Related to the BIOS physical drive number. Floppy drives are
            * identified as 0x00 and physical hard disks are identified as
            * 0x80, regardless of the number of physical disk drives.
            * Typically, this value is set prior to issuing an INT 13h BIOS
            * call to specify the device to access. The value is only
            * relevant if the device is a boot device.
            */
  uint8_t  driveNumber;
           /** used by Windows NT - should be zero for FAT */
  uint8_t  reserved1;
           /** 0X29 if next three fields are valid */
  uint8_t  bootSignature;
           /**
            * A random serial number created when formatting a disk,
            * which helps to distinguish between disks.
            * Usually generated by combining date and time.
            */
  uint32_t volumeSerialNumber;
           /**
            * A field once used to store the volume label. The volume label
            * is now stored as a special file in the root directory.
            */
  char     volumeLabel[11];
           /**
            * A field with a value of either FAT, FAT12 or FAT16,
            * depending on the disk format.
            */
  char     fileSystemType[8];
           /** X86 boot code */
  uint8_t  bootCode[448];
           /** must be 0X55 */
  uint8_t  bootSectorSig0;
           /** must be 0XAA */
  uint8_t  bootSectorSig1;
} PACKED;
/** Type name for FAT Boot Sector */
typedef struct fat_boot fat_boot_t;
//------------------------------------------------------------------------------
/**
 * \struct fat32_boot
 *
 * \brief Boot sector for a FAT32 volume.
 *
 */
struct fat32_boot {
         /**
          * The first three bytes of the boot sector must be valid,
          * executable x 86-based CPU instructions. This includes a
          * jump instruction that skips the next nonexecutable bytes.
          */
  uint8_t jump[3];
         /**
          * This is typically a string of characters that identifies
          * the operating system that formatted the volume.
          */
  char    oemId[8];
          /**
           * The size of a hardware sector. Valid decimal values for this
           * field are 512, 1024, 2048, and 4096. For most disks used in
           * the United States, the value of this field is 512.
           */
  uint16_t bytesPerSector;
          /**
           * Number of sectors per allocation unit. This value must be a
           * power of 2 that is greater than 0. The legal values are
           * 1, 2, 4, 8, 16, 32, 64, and 128.  128 should be avoided.
           */
  uint8_t  sectorsPerCluster;
          /**
           * The number of sectors preceding the start of the first FAT,
           * including the boot sector. Must not be zero
           */
  uint16_t reservedSectorCount;
          /**
           * The number of copies of the FAT on the volume.
           * The value of this field is always 2.
           */
  uint8_t  fatCount;
          /**
           * FAT12/FAT16 only. For FAT32 volumes, this field must be set to 0.
           */
  uint16_t rootDirEntryCount;
          /**
           * For FAT32 volumes, this field must be 0.
           */
  uint16_t totalSectors16;
          /**
           * This dates back to the old MS-DOS 1.x media determination and is
           * no longer usually used for anything.  0xF8 is the standard value
           * for fixed (nonremovable) media. For removable media, 0xF0 is
           * frequently used. Legal values are 0xF0 or 0xF8-0xFF.
           */
  uint8_t  mediaType;
          /**
           * On FAT32 volumes this field must be 0, and sectorsPerFat32
           * contains the FAT size count.
           */
  uint16_t sectorsPerFat16;
           /** Sectors per track for interrupt 0x13. Not used otherwise. */
  uint16_t sectorsPerTrack;
           /** Number of heads for interrupt 0x13.  Not used otherwise. */
  uint16_t headCount;
          /**
           * Count of hidden sectors preceding the partition that contains this
           * FAT volume. This field is generally only relevant for media
           * visible on interrupt 0x13.
           */
  uint32_t hiddenSectors;
          /**
           * Contains the total number of sectors in the FAT32 volume.
           */
  uint32_t totalSectors32;
         /**
           * Count of sectors occupied by one FAT on FAT32 volumes.
           */
  uint32_t sectorsPerFat32;
          /**
           * This field is only defined for FAT32 media and does not exist on
           * FAT12 and FAT16 media.
           * Bits 0-3 -- Zero-based number of active FAT.
           *             Only valid if mirroring is disabled.
           * Bits 4-6 -- Reserved.
           * Bit 7  -- 0 means the FAT is mirrored at runtime into all FATs.
           *        -- 1 means only one FAT is active; it is the one referenced
           *             in bits 0-3.
           * Bits 8-15  -- Reserved.
           */
  uint16_t fat32Flags;
          /**
           * FAT32 version. High byte is major revision number.
           * Low byte is minor revision number. Only 0.0 define.
           */
  uint16_t fat32Version;
          /**
           * Cluster number of the first cluster of the root directory for FAT32.
           * This usually 2 but not required to be 2.
           */
  uint32_t fat32RootCluster;
          /**
           * Sector number of FSINFO structure in the reserved area of the
           * FAT32 volume. Usually 1.
           */
  uint16_t fat32FSInfo;
          /**
           * If nonzero, indicates the sector number in the reserved area
           * of the volume of a copy of the boot record. Usually 6.
           * No value other than 6 is recommended.
           */
  uint16_t fat32BackBootBlock;
          /**
           * Reserved for future expansion. Code that formats FAT32 volumes
           * should always set all of the bytes of this field to 0.
           */
  uint8_t  fat32Reserved[12];
           /**
            * Related to the BIOS physical drive number. Floppy drives are
            * identified as 0x00 and physical hard disks are identified as
            * 0x80, regardless of the number of physical disk drives.
            * Typically, this value is set prior to issuing an INT 13h BIOS
            * call to specify the device to access. The value is only
            * relevant if the device is a boot device.
            */
  uint8_t  driveNumber;
           /** used by Windows NT - should be zero for FAT */
  uint8_t  reserved1;
           /** 0X29 if next three fields are valid */
  uint8_t  bootSignature;
           /**
            * A random serial number created when formatting a disk,
            * which helps to distinguish between disks.
            * Usually generated by combining date and time.
            */
  uint32_t volumeSerialNumber;
           /**
            * A field once used to store the volume label. The volume label
            * is now stored as a special file in the root directory.
            */
  char     volumeLabel[11];
           /**
            * A text field with a value of FAT32.
            */
  char     fileSystemType[8];
           /** X86 boot code */
  uint8_t  bootCode[420];
           /** must be 0X55 */
  uint8_t  bootSectorSig0;
           /** must be 0XAA */
  uint8_t  bootSectorSig1;
} PACKED;
/** Type name for FAT32 Boot Sector */
typedef struct fat32_boot fat32_boot_t;
//------------------------------------------------------------------------------
/** Lead signature for a FSINFO sector */
uint32_t const FSINFO_LEAD_SIG = 0x41615252;
/** Struct signature for a FSINFO sector */
uint32_t const FSINFO_STRUCT_SIG = 0x61417272;
/**
 * \struct fat32_fsinfo
 *
 * \brief FSINFO sector for a FAT32 volume.
 *
 */
struct fat32_fsinfo {
           /** must be 0X52, 0X52, 0X61, 0X41 */
  uint32_t  leadSignature;
           /** must be zero */
  uint8_t  reserved1[480];
           /** must be 0X72, 0X72, 0X41, 0X61 */
  uint32_t  structSignature;
          /**
           * Contains the last known free cluster count on the volume.
           * If the value is 0xFFFFFFFF, then the free count is unknown
           * and must be computed. Any other value can be used, but is
           * not necessarily correct. It should be range checked at least
           * to make sure it is <= volume cluster count.
           */
  uint32_t freeCount;
          /**
           * This is a hint for the FAT driver. It indicates the cluster
           * number at which the driver should start looking for free clusters.
           * If the value is 0xFFFFFFFF, then there is no hint and the driver
           * should start looking at cluster 2.
           */
  uint32_t nextFree;
           /** must be zero */
  uint8_t  reserved2[12];
           /** must be 0X00, 0X00, 0X55, 0XAA */
  uint8_t  tailSignature[4];
} PACKED;
/** Type name for FAT32 FSINFO Sector */
typedef struct fat32_fsinfo fat32_fsinfo_t;
//------------------------------------------------------------------------------
// End Of Chain values for FAT entries
/** FAT12 end of chain value used by Microsoft. */
uint16_t const FAT12EOC = 0XFFF;
/** Minimum value for FAT12 EOC.  Use to test for EOC. */
uint16_t const FAT12EOC_MIN = 0XFF8;
/** FAT16 end of chain value used by Microsoft. */
uint16_t const FAT16EOC = 0XFFFF;
/** Minimum value for FAT16 EOC.  Use to test for EOC. */
uint16_t const FAT16EOC_MIN = 0XFFF8;
/** FAT32 end of chain value used by Microsoft. */
uint32_t const FAT32EOC = 0X0FFFFFFF;
/** Minimum value for FAT32 EOC.  Use to test for EOC. */
uint32_t const FAT32EOC_MIN = 0X0FFFFFF8;
/** Mask a for FAT32 entry. Entries are 28 bits. */
uint32_t const FAT32MASK = 0X0FFFFFFF;
//------------------------------------------------------------------------------
/**
 * \struct directoryEntry
 * \brief FAT short directory entry
 *
 * Short means short 8.3 name, not the entry size.
 *
 * Date Format. A FAT directory entry date stamp is a 16-bit field that is
 * basically a date relative to the MS-DOS epoch of 01/01/1980. Here is the
 * format (bit 0 is the LSB of the 16-bit word, bit 15 is the MSB of the
 * 16-bit word):
 *
 * Bits 9-15: Count of years from 1980, valid value range 0-127
 * inclusive (1980-2107).
 *
 * Bits 5-8: Month of year, 1 = January, valid value range 1-12 inclusive.
 *
 * Bits 0-4: Day of month, valid value range 1-31 inclusive.
 *
 * Time Format. A FAT directory entry time stamp is a 16-bit field that has
 * a granularity of 2 seconds. Here is the format (bit 0 is the LSB of the
 * 16-bit word, bit 15 is the MSB of the 16-bit word).
 *
 * Bits 11-15: Hours, valid value range 0-23 inclusive.
 *
 * Bits 5-10: Minutes, valid value range 0-59 inclusive.
 *
 * Bits 0-4: 2-second count, valid value range 0-29 inclusive (0 - 58 seconds).
 *
 * The valid time range is from Midnight 00:00:00 to 23:59:58.
 */
struct directoryEntry {
           /** Short 8.3 name.
            *
            * The first eight bytes contain the file name with blank fill.
            * The last three bytes contain the file extension with blank fill.
            */
  uint8_t  name[11];
          /** Entry attributes.
           *
           * The upper two bits of the attribute byte are reserved and should
           * always be set to 0 when a file is created and never modified or
           * looked at after that.  See defines that begin with DIR_ATT_.
           */
  uint8_t  attributes;
          /**
           * Reserved for use by Windows NT. Set value to 0 when a file is
           * created and never modify or look at it after that.
           */
  uint8_t  reservedNT;
          /**
           * The granularity of the seconds part of creationTime is 2 seconds
           * so this field is a count of tenths of a second and its valid
           * value range is 0-199 inclusive. (WHG note - seems to be hundredths)
           */
  uint8_t  creationTimeTenths;
           /** Time file was created. */
  uint16_t creationTime;
           /** Date file was created. */
  uint16_t creationDate;
          /**
           * Last access date. Note that there is no last access time, only
           * a date.  This is the date of last read or write. In the case of
           * a write, this should be set to the same date as lastWriteDate.
           */
  uint16_t lastAccessDate;
          /**
           * High word of this entry's first cluster number (always 0 for a
           * FAT12 or FAT16 volume).
           */
  uint16_t firstClusterHigh;
           /** Time of last write. File creation is considered a write. */
  uint16_t lastWriteTime;
           /** Date of last write. File creation is considered a write. */
  uint16_t lastWriteDate;
           /** Low word of this entry's first cluster number. */
  uint16_t firstClusterLow;
           /** 32-bit unsigned holding this file's size in bytes. */
  uint32_t fileSize;
} PACKED;
/**
 * \struct directoryVFATEntry
 * \brief VFAT long filename directory entry
 *
 * directoryVFATEntries are found in the same list as normal directoryEntry.
 * But have the attribute field set to DIR_ATT_LONG_NAME.
 *
 * Long filenames are saved in multiple directoryVFATEntries.
 * Each entry containing 13 UTF-16 characters.
 */
struct directoryVFATEntry {
  /**
   * Sequence number. Consists of 2 parts:
   *  bit 6:   indicates first long filename block for the next file
   *  bit 0-4: the position of this long filename block (first block is 1)
   */
  uint8_t  sequenceNumber;
  /** First set of UTF-16 characters */
  uint16_t name1[5];//UTF-16
  /** attributes (at the same location as in directoryEntry), always 0x0F */
  uint8_t  attributes;
  /** Reserved for use by Windows NT. Always 0. */
  uint8_t  reservedNT;
  /** Checksum of the short 8.3 filename, can be used to checked if the file system as modified by a not-long-filename aware implementation. */
  uint8_t  checksum;
  /** Second set of UTF-16 characters */
  uint16_t name2[6];//UTF-16
  /** firstClusterLow is always zero for longFilenames */
  uint16_t firstClusterLow;
  /** Third set of UTF-16 characters */
  uint16_t name3[2];//UTF-16
} PACKED;
//------------------------------------------------------------------------------
// Definitions for directory entries
//
/** Type name for directoryEntry */
typedef struct directoryEntry dir_t;
/** Type name for directoryVFATEntry */
typedef struct directoryVFATEntry vfat_t;
/** escape for name[0] = 0XE5 */
uint8_t const DIR_NAME_0XE5 = 0X05;
/** name[0] value for entry that is free after being "deleted" */
uint8_t const DIR_NAME_DELETED = 0XE5;
/** name[0] value for entry that is free and no allocated entries follow */
uint8_t const DIR_NAME_FREE = 0X00;
/** file is read-only */
uint8_t const DIR_ATT_READ_ONLY = 0X01;
/** File should hidden in directory listings */
uint8_t const DIR_ATT_HIDDEN = 0X02;
/** Entry is for a system file */
uint8_t const DIR_ATT_SYSTEM = 0X04;
/** Directory entry contains the volume label */
uint8_t const DIR_ATT_VOLUME_ID = 0X08;
/** Entry is for a directory */
uint8_t const DIR_ATT_DIRECTORY = 0X10;
/** Old DOS archive bit for backup support */
uint8_t const DIR_ATT_ARCHIVE = 0X20;
/** Test value for long name entry.  Test is
  (d->attributes & DIR_ATT_LONG_NAME_MASK) == DIR_ATT_LONG_NAME. */
uint8_t const DIR_ATT_LONG_NAME = 0X0F;
/** Test mask for long name entry */
uint8_t const DIR_ATT_LONG_NAME_MASK = 0X3F;
/** defined attribute bits */
uint8_t const DIR_ATT_DEFINED_BITS = 0X3F;
/** Directory entry is part of a long name
 * \param[in] dir Pointer to a directory entry.
 *
 * \return true if the entry is for part of a long name else false.
 */
static inline uint8_t DIR_IS_LONG_NAME(const dir_t* dir) {
  return (dir->attributes & DIR_ATT_LONG_NAME_MASK) == DIR_ATT_LONG_NAME;
}
/** Mask for file/subdirectory tests */
uint8_t const DIR_ATT_FILE_TYPE_MASK = (DIR_ATT_VOLUME_ID | DIR_ATT_DIRECTORY);
/** Directory entry is for a file
 * \param[in] dir Pointer to a directory entry.
 *
 * \return true if the entry is for a normal file else false.
 */
static inline uint8_t DIR_IS_FILE(const dir_t* dir) {
  return (dir->attributes & DIR_ATT_FILE_TYPE_MASK) == 0;
}
/** Directory entry is for a subdirectory
 * \param[in] dir Pointer to a directory entry.
 *
 * \return true if the entry is for a subdirectory else false.
 */
static inline uint8_t DIR_IS_SUBDIR(const dir_t* dir) {
  return (dir->attributes & DIR_ATT_FILE_TYPE_MASK) == DIR_ATT_DIRECTORY;
}
/** Directory entry is for a file or subdirectory
 * \param[in] dir Pointer to a directory entry.
 *
 * \return true if the entry is for a normal file or subdirectory else false.
 */
static inline uint8_t DIR_IS_FILE_OR_SUBDIR(const dir_t* dir) {
  return (dir->attributes & DIR_ATT_VOLUME_ID) == 0;
}



//==============================================================================
// SDFATUTIL - Utility functions
//==============================================================================
/**
 * \file
 * \brief Useful utility functions.
 */
#include "MarlinSerial.h"
/** Store and print a string in flash memory.*/
#define PgmPrint(x) SerialPrint_P(PSTR(x))
/** Store and print a string in flash memory followed by a CR/LF.*/
#define PgmPrintln(x) SerialPrintln_P(PSTR(x))

namespace SdFatUtil {
  int FreeRam();
  void print_P(PGM_P str);
  void println_P(PGM_P str);
  void SerialPrint_P(PGM_P str);
  void SerialPrintln_P(PGM_P str);
}

using namespace SdFatUtil;  // NOLINT



//==============================================================================
// SDVOLUME - FAT volume management
//==============================================================================
/**
 * \file
 * \brief SdVolume class
 */
#include "sd_card_hardware.h"

//==============================================================================
// SdVolume class
/**
 * \brief Cache for an SD data block
 */
union cache_t {
           /** Used to access cached file data blocks. */
  uint8_t  data[512];
           /** Used to access cached FAT16 entries. */
  uint16_t fat16[256];
           /** Used to access cached FAT32 entries. */
  uint32_t fat32[128];
           /** Used to access cached directory entries. */
  dir_t    dir[16];
           /** Used to access a cached Master Boot Record. */
  mbr_t    mbr;
           /** Used to access to a cached FAT boot sector. */
  fat_boot_t fbs;
           /** Used to access to a cached FAT32 boot sector. */
  fat32_boot_t fbs32;
           /** Used to access to a cached FAT32 FSINFO sector. */
  fat32_fsinfo_t fsinfo;
};
//------------------------------------------------------------------------------
/**
 * \class SdVolume
 * \brief Access FAT16 and FAT32 volumes on SD and SDHC cards.
 */
class SdVolume {
 public:
  /** Create an instance of SdVolume */
  SdVolume() : fatType_(0) {}
  /** Clear the cache and returns a pointer to the cache.  Used by the WaveRP
   * recorder to do raw write to the SD card.  Not for normal apps.
   * \return A pointer to the cache buffer or zero if an error occurs.
   */
  cache_t* cacheClear() {
    if (!cacheFlush()) return 0;
    cacheBlockNumber_ = 0XFFFFFFFF;
    return &cacheBuffer_;
  }
  /** Initialize a FAT volume.  Try partition one first then try super
   * floppy format.
   *
   * \param[in] dev The Sd2Card where the volume is located.
   *
   * \return The value one, true, is returned for success and
   * the value zero, false, is returned for failure.  Reasons for
   * failure include not finding a valid partition, not finding a valid
   * FAT file system or an I/O error.
   */
  bool init(Sd2Card* dev) { return init(dev, 1) ? true : init(dev, 0);}
  bool init(Sd2Card* dev, uint8_t part);

  // inline functions that return volume info
  /** \return The volume's cluster size in blocks. */
  uint8_t blocksPerCluster() const {return blocksPerCluster_;}
  /** \return The number of blocks in one FAT. */
  uint32_t blocksPerFat()  const {return blocksPerFat_;}
  /** \return The total number of clusters in the volume. */
  uint32_t clusterCount() const {return clusterCount_;}
  /** \return The shift count required to multiply by blocksPerCluster. */
  uint8_t clusterSizeShift() const {return clusterSizeShift_;}
  /** \return The logical block number for the start of file data. */
  uint32_t dataStartBlock() const {return dataStartBlock_;}
  /** \return The number of FAT structures on the volume. */
  uint8_t fatCount() const {return fatCount_;}
  /** \return The logical block number for the start of the first FAT. */
  uint32_t fatStartBlock() const {return fatStartBlock_;}
  /** \return The FAT type of the volume. Values are 12, 16 or 32. */
  uint8_t fatType() const {return fatType_;}
  int32_t freeClusterCount();
  /** \return The number of entries in the root directory for FAT16 volumes. */
  uint32_t rootDirEntryCount() const {return rootDirEntryCount_;}
  /** \return The logical block number for the start of the root directory
       on FAT16 volumes or the first cluster number on FAT32 volumes. */
  uint32_t rootDirStart() const {return rootDirStart_;}
  /** Sd2Card object for this volume
   * \return pointer to Sd2Card object.
   */
  Sd2Card* sdCard() {return sdCard_;}
  /** Debug access to FAT table
   *
   * \param[in] n cluster number.
   * \param[out] v value of entry
   * \return true for success or false for failure
   */
  bool dbgFat(uint32_t n, uint32_t* v) {return fatGet(n, v);}
  //------------------------------------------------------------------------------
 private:
  // Allow SdBaseFile access to SdVolume private data.
  friend class SdBaseFile;

  // value for dirty argument in cacheRawBlock to indicate read from cache
  static bool const CACHE_FOR_READ = false;
  // value for dirty argument in cacheRawBlock to indicate write to cache
  static bool const CACHE_FOR_WRITE = true;

#if USE_MULTIPLE_CARDS
  cache_t cacheBuffer_;        // 512 byte cache for device blocks
  uint32_t cacheBlockNumber_;  // Logical number of block in the cache
  Sd2Card* sdCard_;            // Sd2Card object for cache
  bool cacheDirty_;            // cacheFlush() will write block if true
  uint32_t cacheMirrorBlock_;  // block number for mirror FAT
#else  // USE_MULTIPLE_CARDS
  static cache_t cacheBuffer_;        // 512 byte cache for device blocks
  static uint32_t cacheBlockNumber_;  // Logical number of block in the cache
  static Sd2Card* sdCard_;            // Sd2Card object for cache
  static bool cacheDirty_;            // cacheFlush() will write block if true
  static uint32_t cacheMirrorBlock_;  // block number for mirror FAT
#endif  // USE_MULTIPLE_CARDS
  uint32_t allocSearchStart_;   // start cluster for alloc search
  uint8_t blocksPerCluster_;    // cluster size in blocks
  uint32_t blocksPerFat_;       // FAT size in blocks
  uint32_t clusterCount_;       // clusters in one FAT
  uint8_t clusterSizeShift_;    // shift to convert cluster count to block count
  uint32_t dataStartBlock_;     // first data block number
  uint8_t fatCount_;            // number of FATs on volume
  uint32_t fatStartBlock_;      // start block for first FAT
  uint8_t fatType_;             // volume type (12, 16, OR 32)
  uint16_t rootDirEntryCount_;  // number of entries in FAT16 root dir
  uint32_t rootDirStart_;       // root start block for FAT16, cluster for FAT32
  //----------------------------------------------------------------------------
  bool allocContiguous(uint32_t count, uint32_t* curCluster);
  uint8_t blockOfCluster(uint32_t position) const {
    return (position >> 9) & (blocksPerCluster_ - 1);
  }
  uint32_t clusterStartBlock(uint32_t cluster) const {
    return dataStartBlock_ + ((cluster - 2) << clusterSizeShift_);
  }
  uint32_t blockNumber(uint32_t cluster, uint32_t position) const {
    return clusterStartBlock(cluster) + blockOfCluster(position);
  }
  cache_t* cache() {return &cacheBuffer_;}
  uint32_t cacheBlockNumber() {return cacheBlockNumber_;}
#if USE_MULTIPLE_CARDS
  bool cacheFlush();
  bool cacheRawBlock(uint32_t blockNumber, bool dirty);
#else  // USE_MULTIPLE_CARDS
  static bool cacheFlush();
  static bool cacheRawBlock(uint32_t blockNumber, bool dirty);
#endif  // USE_MULTIPLE_CARDS
  // used by SdBaseFile write to assign cache to SD location
  void cacheSetBlockNumber(uint32_t blockNumber, bool dirty) {
    cacheDirty_ = dirty;
    cacheBlockNumber_  = blockNumber;
  }
  void cacheSetDirty() {cacheDirty_ |= CACHE_FOR_WRITE;}
  bool chainSize(uint32_t beginCluster, uint32_t* size);
  bool fatGet(uint32_t cluster, uint32_t* value);
  bool fatPut(uint32_t cluster, uint32_t value);
  bool fatPutEOC(uint32_t cluster) {
    return fatPut(cluster, 0x0FFFFFFF);
  }
  bool freeChain(uint32_t cluster);
  bool isEOC(uint32_t cluster) const {
    if (FAT12_SUPPORT && fatType_ == 12) return  cluster >= FAT12EOC_MIN;
    if (fatType_ == 16) return cluster >= FAT16EOC_MIN;
    return  cluster >= FAT32EOC_MIN;
  }
  bool readBlock(uint32_t block, uint8_t* dst) {
    return sdCard_->readBlock(block, dst);
  }
  bool writeBlock(uint32_t block, const uint8_t* dst) {
    return sdCard_->writeBlock(block, dst);
  }
  //------------------------------------------------------------------------------
  // Deprecated functions  - suppress cpplint warnings with NOLINT comment
#if ALLOW_DEPRECATED_FUNCTIONS && !defined(DOXYGEN)
 public:
  /** \deprecated Use: bool SdVolume::init(Sd2Card* dev);
   * \param[in] dev The SD card where the volume is located.
   * \return true for success or false for failure.
   */
  bool init(Sd2Card& dev) {return init(&dev);}  // NOLINT
  /** \deprecated Use: bool SdVolume::init(Sd2Card* dev, uint8_t vol);
   * \param[in] dev The SD card where the volume is located.
   * \param[in] part The partition to be used.
   * \return true for success or false for failure.
   */
  bool init(Sd2Card& dev, uint8_t part) {  // NOLINT
    return init(&dev, part);
  }
#endif  // ALLOW_DEPRECATED_FUNCTIONS
};

#endif // sd_card_filesystem_h

#endif
