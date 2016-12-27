/*
 * @brief Secondary loader protocol structures and manifest constants
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2014
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */
#ifndef _SL_PROTOCOL_H_
#define _SL_PROTOCOL_H_

#ifndef __KERNEL__
#include <stdint.h>
#endif

#ifdef __CC_ARM
#pragma anon_unions
#endif


#define BUS_PROBE_TIMEOUT_MS (5000)
#define BUS_COMMAND_TIMEOUT_MS (1000)

#define SL_FLASH_BLOCK_SZ		0x200	/*!< Block size */
#define SL_FLASH_SECT_SZ		0x8000	/*!< Size of a flash sector */
#define SL_FLASH_END			0x80000	/*!< Total size of the flash. */
#define SL_FLASH_PAGE_SZ		0x100	/*!< Size of the flash page. */
/* FIXME here, wangjianping, if use qualcomm i2c bus v3 protocol, SL_MAX_I2C_READ_SIZE maybe change to 128 */
#define SL_MAX_I2C_READ_SIZE (512)
#define SL_MAX_I2C_WRITE_SIZE (512 + 4) /*4 bytes header*/

/* Address of where the boot-able application is located in FLASH */
#define SL_BOOTAPP_ADDR				0x00008000
#define SL_BOOTAPP_START_BLOCK		(SL_BOOTAPP_ADDR/SL_FLASH_BLOCK_SZ)
#define SL_BOOTAPP_START_PAGE		(SL_BOOTAPP_ADDR/SL_FLASH_PAGE_SZ)

/* Offset in application where the IMG_HEADER_T is stored */
#define SL_BOOTAPP_IMGHDR_OFS		0x100
#define IMG_HEADER_MARKER			0xFEEDA5A5
#define SL_KEY_BYTES				0x40

/*!< Pin PIO0_11 has SPI0_SCK (1) function. */
#define PIN_SPI0_SCK_1		((0 << 5) | 11)
/*!< Pin PIO1_3 has	SPI0_SCK (5) function. */
#define PIN_SPI0_SCK_2		((1 << 5) | 3)
/*!< Pin PIO0_12 has SPI0_MOSI (1) function. */
#define PIN_SPI0_MOSI_1		((0 << 5) | 12)
/*!< Pin PIO1_9 has	SPI0_MOSI (2) function. */
#define PIN_SPI0_MOSI_2		((1 << 5) | 9)
/*!< Pin PIO0_13 has SPI0_MISO (1) function. */
#define PIN_SPI0_MISO_1		((0 << 5) | 13)
/*!< Pin PIO1_4 has	SPI0_MISO (5) function. */
#define PIN_SPI0_MISO_2		((1 << 5) | 4)
/*!< Pin PIO0_0 has	SPI0_SSEL0 (2) function. */
#define PIN_SPI0_SEL0_1		((0 << 5) | 0)
/*!< Pin PIO0_9 has	SPI0_SSEL0 (5) function. */
#define PIN_SPI0_SEL0_2		((0 << 5) | 9)
/*!< Pin PIO0_14 has SPI0_SSEL0 (1) function. */
#define PIN_SPI0_SEL0_3		((0 << 5) | 14)

/*!< Pin PIO1_6 has	SPI1_SCK (2) function. */
#define PIN_SPI1_SCK_1		((1 << 5) | 6)
/*!< Pin PIO1_12 has SPI1_SCK (4) function. */
#define PIN_SPI1_SCK_2		((1 << 5) | 12)
/*!< Pin PIO1_7 has SPI1_MOSI (2) function. */
#define PIN_SPI1_MOSI_1		((1 << 5) | 7)
/*!< Pin PIO1_13 has SPI1_MOSI (4) function. */
#define PIN_SPI1_MOSI_2		((1 << 5) | 13)
/*!< Pin PIO1_8 has	SPI1_MISO (2) function. */
#define PIN_SPI1_MISO_1		((1 << 5) | 8)
/*!< Pin PIO1_14 has SPI1_MISO (4) function. */
#define PIN_SPI1_MISO_2		((1 << 5) | 14)
/*!< Pin PIO1_5 has	SPI1_SSEL0 (2) function. */
#define PIN_SPI1_SEL0_1		((1 << 5) | 5)
/*!< Pin PIO1_15 has SPI1_SSEL0 (4) function. */
#define PIN_SPI1_SEL0_2		((1 << 5) | 15)
#define SH_RESP_HDR_SOP			0x55

/* Bootloader specific commands */
#define SH_CMD_WHO_AM_I				0xA0
#define SH_CMD_GET_VERSION			0xA1
#define SH_CMD_RESET				0xA2
 /*!< Command to boot the flashed image */
#define SH_CMD_BOOT					0xA3
/*!< Command to check image integrity.
	Return 0 if matches else computed CRC32 value
	(not include CRC field in mage).
*/
#define SH_CMD_CHECK_IMAGE			0xA4
#define SH_CMD_PROBE				0xA5
/*!< Command to write a block.
	Each block is 512 bytes.
	Check CmdRWBlockParam_t for message format.
*/
#define SH_CMD_WRITE_BLOCK			0xA6
/*!< Command to Read a block.
	Each block is 512 bytes.
	Check CmdRWBlockParam_t for message format.
*/
#define SH_CMD_READ_BLOCK			0xA7
#define SH_CMD_SECTOR_ERASE			0xA8
#define SH_CMD_PAGE_ERASE			0xA9
#define SH_CMD_PAGE_WRITE			0xAA
#define SH_CMD_PAGE_READ			0xAB
#define SH_CMD_WRITE_SUB_BLOCK       0xAC
#define SH_CMD_READ_SUB_BLOCK        0xAD
#define SH_CMD_BULK_ERASE           0xAE
/* Command codes 0xAF - 0xBF are reserved for roms that have integrated sec loader functionality */
/* The rom and flash version should use same command codes */
/* Secure firmware update related commands */
#define SH_CMD_ENABLE_SECURE        0xC0
#define SH_CMD_DISABLE_SECURE       0xC1
/* RAM dump commands */
/*!< get the summary about what can be dumpped */
#define SH_CMD_DUMP_SUMMARY             0xC8
/*!< dump the CPU context */
#define SH_CMD_DUMP_CPU_CONTEXT             0xC9
/*!< dump the stack used when HardFault or NMI happens */
#define SH_CMD_DUMP_STACK             0xCA
/*!< dump arbitrary address range */
#define SH_CMD_DUMP_MEMORY             0xCB
/* RW command parameter defines */
#define SH_RW_PARAM_SKIP_CRC        1
/* SH_CMD_PROBE command constants */
#define SH_PROBE_LEN                8
#define SH_PROBE_XOR_OFS            5

/* Response field size constants */
#define SH_RESP_HEADER_SIZE         4
#define SH_RESP_ERROR_CODE_SZ       4
#define SH_RESP_CRC32_SZ            4
#define SH_RESP_VERSION_SZ          2

/** Structure describing response packet format. */
struct _CmdResponse_t {
	/*!< Start of packet = 0x55 for boatloader */
	uint8_t sop;
	/*!< Response to the Command ID. For notification use 0xFF. */
	uint8_t cmd;
	/*!< Response data length not including the header. */
	uint16_t length;
};

/** Structure describing Read/Write block command packet format. */
struct _CmdRWBlockParam_t {
	uint8_t cmd; /*!< Command ID */
	/*!< specifies if we need to do CRC check before processing */
	uint8_t crc_check;
	/*!< Block/page number starting from 0x8000 offset.*/
	uint16_t block_page_nr;
	uint32_t data[SL_FLASH_BLOCK_SZ / 4];	/*!< Data */
	uint32_t crc32;							/*!< CRC32 of command header and data */
};

/** Structure decribring Read/Write block command packet format. */
struct _CmdRWSubblockParam_t {
	uint8_t cmd;											/*!< Command ID */
	uint8_t crc_check;										/*!< specifies if we need to do CRC check before processing */
	uint16_t block_page_nr;									/*!< Block number.*/
	uint32_t data[SL_MAX_I2C_READ_SIZE/4];		/*!< Data */
	uint32_t crc32;											/*!< CRC32 of command header and data */
};
/** Structure describing Read sub-block command packet format. */
struct _CmdReadSubBlockParam_t {
	uint8_t cmd;											/*!< Command ID */
	uint8_t sub_block_nr;									/*!< specifies the sub-block number.
															    0 - Skip crc;
															    5-1: sub block nr;
															    7-6: sub-block size. 0 - 32, 1 - 64, 2 - 128, 3 - 256 */
	uint16_t block_nr;										/*!< block number.*/
	uint32_t data[SL_MAX_I2C_READ_SIZE / 4];		/*!< Data */
};

/** Structure describing Sector erase command packet format. */
struct _CmdEraseSectorParam_t {
	uint8_t cmd;							/*!< Command ID */
	uint8_t reserved;						/*!< Should be zero. */
	uint16_t sec_nr;						/*!< Sector number.*/
};
/** Structure describing Bulk erase command packet format. */
struct _CmdBulkEraseParam_t {
	uint8_t cmd;						/*!< Command ID */
	uint8_t reserved;					/*!< Should be zero. */
	uint8_t start_sec;					/*!< Start Sector number.*/
	uint8_t end_sec;					/*!< End Sector number.*/
};

/** Structure describing response packet with data. */
struct _CmdDataResp_t {
	struct _CmdResponse_t hdr;				/*!< Response header. */
	uint32_t data[SL_FLASH_BLOCK_SZ / 4];	/*!< Data */
	uint32_t crc32;							/*!< CRC32 of response packet. */
};

/** Structure describing Enable secure command packet format. */
struct _CmdEnableSecureParam_t {
	uint8_t cmd;							/*!< Command ID */
	uint8_t crc_check;						/*!< specifies if we need to do CRC check before processing */
	uint16_t reserved;						/*!< Should be zero. */
	uint32_t data[SL_KEY_BYTES / 4];		/*!< Key */
	uint32_t crc32;							/*!< CRC32 of command header and data */
};

/*--------------------------------------------------------------------------------*/
union _U32FINE {
	uint8_t u8s[4];
	uint16_t u16s[2];
	uint32_t u32s[1];
	uint8_t *pu8;
	uint32_t *pu32;
	volatile uint32_t *pvu32;
};

/*-------------------------------------------------------------------------------*/
enum _enum_DumpTriggerSrc {
	dump_trg_none = 0,
	dump_trg_rst_ext = 1,		/* MCU is reset by AP using reset pin, context is lost */
	dump_trg_rst_wdt = 2,		/* MCU is reset by watch dog, context is lost */
	dump_trg_rst_usr = 3,		/* MCU is reset by user code manually, used when user code detects some errors. */
	dump_trg_rst_bod = 4,		/* MCU is reset by brown out, not sure whether this is required for dump*/
	dump_trg_irq_wdt = 5,		/* Trigger by Watchdog IRQ, it connects to NMI, full context is available to dump */
	dump_trg_irq_nmi = dump_trg_irq_wdt,
	dump_trg_irq_hft = 6,		/* Trigger by HardFault handler, full context is available to dump */
	dump_trg_irq_pin = 7,		/* Trigger by IRQ from AP, full context is available but CPU regs are meaningless.*/
	dump_trg_source_cnt,
};

enum _enum_DumpNvicIrqCnt {
	dump_nvic_irq_64 = 0,
	dump_nvic_irq_128 = 1,
	dump_nvic_irq_192 = 2,
	dump_nvic_irq_240 = 3,
	dump_nvic_irq_full = dump_nvic_irq_240,
};

/* Tags for user code to imply that dump is supported and / or requsted */
/* 12 bytes */
/* there is a flash tag and a RAM tag. */
/* flash tag locates at vector 4,5,6 of user image */
/* RAM tag locates at SL_ADDRESS_DUMP_TAG_IN_RAM */
struct _DS_DumpTag {
	union {
		char c1Magics[4];		/* 'D', 'U', 'M', 'P' */
		uint32_t c4Magic;
	};
	/* ofs 4 */
	union _U32FINE userFmwrVersion;
	/* ofs 8 */
	uint16_t c8MainStackCap;		/* main stack capacity of user code, in 8 bytes (DWORD). Stack bottom can be read from user vector table */
	/* ofs 10 */
	uint8_t b00_isCtxValid:1;		/* reserved for user code */
	uint8_t b01_b2NvicIRQCnt:2;	// enum_DumpNvicIrqCnt values, reserved for user code
	uint8_t b03_b3TrgSrc:3;		/* trigger source of dump, enum values of enum_DumpResetSrc, filled by user code */
	/* ofs 11 */
	uint8_t cksum8;

};

/* Summary to dump, 24 bytes. The first 12 bytes are request tag. */
struct _DS_SummaryToDump {
	/* >>> below are the same structure as dump tag */
	union {
		char c1Magics[4];		/* 'D', 'U', 'M', 'P' */
		uint32_t c4Magic;
	};
	/* ofs 4 */
	union _U32FINE userFmwrVersion;
	/* ofs 8 */
	uint16_t c8MainStackCap;		/* main stack capacity of user code, in 8 bytes (DWORD). Stack bottom can be read from user vector table*/
	/* ofs 10 */
	uint8_t b00_isCtxValid:1;		/* reserved for user code */
	uint8_t b01_b2NvicIRQCnt:2;	/* enum_DumpNvicIrqCnt values */
	uint8_t b03_b3TrgSrc:3;		/* trigger source of dump, enum values of enum_DumpResetSrc, filled by user code */
	/* ofs 11 */
	uint8_t cksum8;
	/* <<< above are the same structure as dump tag */

	/* ofs 12 */
	/* User code can optional fill the pExtra field to denote there are extra data saved */
	/* customer need to according to userFmwrVersion to interpret this field */
	void *pExtra;				/* optional extra data block that user code saved, RAM DUMP feature process it as black box */

	/* below fields are filled by dumper */
	/* ofs 16 */
	/* >>> rky: 150128_2050 : add initial main stack and firmware version */
	uint32_t intialMSP;			/* initial MSP for user firmware, since CRP disables flash read, user code provide the value here */
	/* ofs 20 */
	uint32_t pad32;

};

/*--------------------------------------------------------------------------------*/
/* saved CPU regs, 19 32-bit regs, 4 8-bit regs, 20x4 = 80 bytes */
struct _DS_SavedCPURegs {
	uint8_t CONTROL, PRIMASK, FAULTMASK, BASEPRI;
	uint32_t MSP, PSP;
	uint32_t SP;	/* could be MSP or PSP, ease for use */
	uint32_t R4, R5, R6, R7, R8, R9, R10, R11;
	/* Below are automatic stacked registers by Cortex-M core */
	uint32_t R0, R1, R2, R3, IP, LR, PC;
	union {
		uint32_t PSR;
		struct {
			uint32_t psr00_b10ExpNdx:10;
			uint32_t psr10_b6ICIIT_A:6;
			uint32_t psr16_b4GE:4;
			uint32_t psr20_b4zzz:4;
			uint32_t psr24_T:1;
			uint32_t psr25_b2ICIIT_B:2;
			uint32_t psr27_Q:1;
			uint32_t psr28_V:1;
			uint32_t psr29_C:1;
			uint32_t psr30_Z:1;
			uint32_t psr31_N:1;
		};
	};
};
/*-------------------------------------------------------------------------------*/
/* Saved System Control Block (SCB) regs, 140 bytes.*/
struct _DS_SavedSCBRegs {
	/* >>> System Control Block (SCB) structures below (22 registers) */
	uint32_t CPUID;		/* Offset: 0x000 (R/ )  CPUID Base Register */
	uint32_t ICSR;		/* Offset: 0x004 (R/W)  Interrupt Control and State Register */
	uint32_t VTOR;		/* Offset: 0x008 (R/W)  Vector Table Offset Register */
	uint32_t AIRCR;		/* Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register*/
	uint32_t SCR;		/* Offset: 0x010 (R/W)  System Control Register */
	uint32_t CCR;		/* Offset: 0x014 (R/W)  Configuration Control Register */
	uint8_t SHP[12];	/* Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
	uint32_t SHCSR;		/* Offset: 0x024 (R/W)  System Handler Control and State Register */
	                    /* Offset: 0x028 (R/W)  Configurable Fault Status Register */
	union {
		uint32_t CFSR;
		struct {
			uint32_t cfsr00_IAccViol:1;
			uint32_t cfsr01_DAccViol:1;
			uint32_t cfsr02_zzz02:1;
			uint32_t cfsr03_MUnStkErr:1;
			uint32_t cfsr04_MStkErr:1;
			uint32_t cfsr05_MLazyFPErr:1;
			uint32_t cfsr06_zzz:1;
			uint32_t cfsr07_MMAR_Valid:1;

			uint32_t cfsr08_IBusErr:1;
			uint32_t cfsr09_PreciseErr:1;
			uint32_t cfsr10_ImpreciseErr:1;
			uint32_t cfsr11_UnStkErr:1;
			uint32_t cfsr12_StkErr:1;
			uint32_t cfsr13_LazyFPErr:1;
			uint32_t cfsr14_zzz:1;
			uint32_t cfsr15_BFAR_Valid:1;

			uint32_t cfsr16_UndefIns:1;
			uint32_t cfsr17_InvState:1;
			uint32_t cfsr18_InvPC:1;
			uint32_t cfsr19_NoCop:1;
			uint32_t cfsr20_zzzTo23:4;

			uint32_t cfsr24_Unaligned:1;
			uint32_t cfsr25_DivBy0:1;
		};
	};
	/* Offset: 0x02C (R/W)  HardFault Status Register */
	union {
		uint32_t HFSR;
		struct {
			uint32_t hfsr00_zzz:1;
			uint32_t hfsr01_VectTbl:1;
			uint32_t hfsr02_zzzTo29:1;
			uint32_t hfsr30_Forced:1;
			uint32_t hfsr31_DebugEvt:1;
		};
	};
	uint32_t DFSR;		/* Offset: 0x030 (R/W)  Debug Fault Status Register */
	uint32_t MMFAR;		/* Offset: 0x034 (R/W)  MemManage Fault Address Register */
	uint32_t BFAR;		/* Offset: 0x038 (R/W)  BusFault Address Register */
	uint32_t AFSR;		/* Offset: 0x03C (R/W)  Auxiliary Fault Status Register */
	uint32_t PFR[2];	/* Offset: 0x040 (R/ )  Processor Feature Register */
	uint32_t DFR;		/* Offset: 0x048 (R/ )  Debug Feature Register */
	uint32_t ADR;		/* Offset: 0x04C (R/ )  Auxiliary Feature Register */
	uint32_t MMFR[4];	/* Offset: 0x050 (R/ )  Memory Model Feature Register */
	uint32_t ISAR[5];	/* Offset: 0x060 (R/ )  Instruction Set Attributes Register */
	uint32_t RESERVED0[5];
	uint32_t CPACR;		/* Offset: 0x088 (R/W)  Coprocessor Access Control Register */
};
/*--------------------------------------------------------------------------------*/
struct _DS_SavedNVICRegs_64IRQ {
	uint32_t ISER[2];	/* Interrupt Set Enable Register, one bit per IRQ line. */
	uint32_t ICER[2];	/* Interrupt Clear Enable Register, one bit per IRQ line. */
	uint32_t ISPR[2];	/* Interrupt Set Pending Register, one bit per IRQ line. */
	uint32_t ICPR[2];	/* Interrupt Clear Pending Register, one bit per IRQ line. */
	uint32_t IABR[2];	/* Interrupt Active bit Register, one bit per IRQ line. */
	uint8_t IP[64];		/* Interrupt Priority Register, one byte per IRQ line */
};
/*----------------------------------------------------------------------------------------*/
/* CPU context to dump, summary, cpu registers, SCB, and pointer to saved NVIC registers */
/* fault handler, NMI handler, and AP dump pin IRQ handler ssaves this structure to SL_ADDRESS_DUMP_SAVED_CONTEXT */
/* must be <=256 bytes long */
struct _DS_CtxToDump {
	struct _DS_SummaryToDump sum;			/* 24 bytes */
	struct _DS_SavedCPURegs cpu;			/* 80 bytes */
	struct _DS_SavedSCBRegs scb;			/* 140 bytes */
	struct _DS_SavedNVICRegs_64IRQ nvic;	/* 104 bytes */
};
#pragma pack(1)
struct _DS_HostDumpCmdCommon {
	uint8_t cmdCode;		/* SH_CMD_DUMP_XXX */
	uint8_t c4ToReply;		/* how many 32-bit words (4 bytes) to reply. By this means host can ask for partial context data */
};

struct _DS_HostDumpCmdStk {
	uint8_t cmdCode;		/* SH_CMD_DUMP_XXX */
	uint8_t c4ToReply;		/* how many 32-bit words (4 bytes) to reply */
	/* determines from where to dump : how many 32-bit words to stack top */
	/* e.g. dump address from = saved SP + c4ToTop * 4. This limits stack max to 256kB */
	uint16_t c4ToTop;
};

struct _DS_HostDumpCmdRng {
	uint8_t cmdCode;		/* SH_CMD_DUMP_XXX */
	uint8_t c4ToReply;		/* how many 32-bit words (4 bytes) to reply */
	uint16_t addrL16;		/* low 16 bit of address to dump from */
	uint16_t addrH16;		/* high 16 bit of address to dump from */
};



/* define responses of dump */

/* response structure for dump summary */
struct _DS_HostDumpResSum {
	struct _CmdResponse_t hdr;
	struct _DS_SummaryToDump sum;
};

/* response structure for context */
struct _DS_HostDumpResCtx {
	struct _CmdResponse_t hdr;
	/* same definition as DS_CtxToDump */
	struct _DS_SummaryToDump sum;
	struct _DS_SavedCPURegs reg;
	struct _DS_SavedSCBRegs scb;
	/* >>> NVIC is too large to embed in this structure, so only save first 64 IRQs */
	/* >>> host should use SH_CMD_DUMP_RNG to dump the full NVIC registers (not apply to LPC54101/2) */
	struct _DS_SavedNVICRegs_64IRQ nvic;
};

struct _DS_HostDumpResStk {
	struct _CmdResponse_t hdr;
	/* define maximum possible data */
	union _U32FINE stackedAry[SL_MAX_I2C_WRITE_SIZE];	
};

struct _DS_HostDumpResRng {
	struct _CmdResponse_t hdr;
	/* define payload array as 1 is just for demo, the real count depends on host's request. */
	union _U32FINE payloads[SL_MAX_I2C_WRITE_SIZE];
};

#pragma pack()

extern int SLBoot_xfer(struct device *dev, u8 *txbuffer, int txlength,
					   u8 *rxbuffer, int rxlength);

extern int SLBoot_send_cmd(struct device *dev, u8 *txbuffer,
						   int txlength, u8 *rxbuffer, int rxlength, int gpio);
#endif	/* SL_PROTOCOL_H_ */

