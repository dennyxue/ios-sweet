/*
 * SWEET / OCEAN internal eMMC root device.
 *
 * Shared declarations for the SDHCI (SDHC) host controller driver.
 */

#ifndef _SWEET_EMMC_H_
#define _SWEET_EMMC_H_

#include <sys/types.h>

/*
 * SDM710 (Snapdragon 710) eMMC controller, as used by the Smartisan
 * Nut Pro 2S (codename "ocean").
 *
 * From arch/arm64/boot/dts/qcom/sdm670.dtsi (LineageOS sdm710 tree):
 *
 *   sdhc_1: sdhci@7c4000 {
 *           compatible = "qcom,sdhci-msm-v5";
 *           reg = <0x7C4000 0x1000>, <0x7C5000 0x1000>;
 *           reg-names = "hc_mem", "cmdq_mem";
 *           interrupts = <0 641 0>, <0 644 0>;
 *           qcom,bus-width = <8>;
 *           qcom,large-address-bus;
 *   };
 *
 * SDCC5 ("v5") removed the MCI register block: every vendor register the
 * driver needs lives inside the 4 KiB host-controller window at 0x7c4000,
 * so there is no second mapping and no CORE_* bank to program.
 *
 * The controller is left powered and clocked by the bootloader (it read
 * our boot image out of this very eMMC), so - unlike a cold UFS host - we
 * have to bring the card side up ourselves.  See docs/DESIGN.md.
 */

/* Host controller window (4 KiB). */
#define EMMC_HC_BASE		0x07c4000
#define EMMC_HC_SIZE		0x1000

/*
 * SDHCI 3.x register offsets (SD Host Controller Specification v3.0).
 * Only the ones this driver touches are named.
 */
#define SDHCI_SYSAD		0x00	/* vendor id / system address */
#define SDHCI_BLKG_ATT		0x04
#define SDHCI_PRNSTS		0x08	/* present state */
#define SDHCI_CMD		0x0c	/* command register */
#define SDHCI_TRANSFER_MODE	0x0e
#define SDHCI_DATA		0x10
#define SDHCI_RSPREG		0x14	/* response, 4 x u32 */
#define SDHCI_BLKATT		0x18
#define SDHCI_HOSTCTL		0x1c	/* host control */
#define SDHCI_PWRCON		0x20	/* power control */
#define SDHCI_CLK_CONTROL	0x24	/* clock control */
#define SDHCI_SWRST		0x28	/* software reset */
#define SDHCI_INTSTATUS		0x2c
#define SDHCI_INTENABLE		0x30
#define SDHCI_CAPABILITIES	0x34
#define SDHCI_CAPABILITIES_1	0x38
#define SDHCI_MAXBLOCK		0x3c
#define SDHCI_ADMA_DESC_ADDR	0x40
#define SDHCI_HOST_VERSION	0x44

/* Transfer mode (0x0e) */
#define SDHCI_TRNS_READ		0x01
#define SDHCI_TRNS_WRITE	0x02
#define SDHCI_TRNS_DMA		0x20
#define SDHCI_TRNS_BLOCK_CNT	0x04
#define SDHCI_TRNS_BLK_CNT_E	0x02	/* enable block count */
#define SDHCI_TRNS_BLK_CNT_S	0x01	/* 512-byte block count */

/* Software reset (0x28) */
#define SDHCI_RESET_CMD		0x01
#define SDHCI_RESET_DATA	0x02
#define SDHCI_RESET_ALL		0x04
#define SDHCI_RESET_OFFSET	24

/* Power control (0x20) */
#define SDHCI_POWER_BUS_OFF	0x00
#define SDHCI_POWER_BUS_ON	0x01
#define SDHCI_POWER_SD_BUS_ON	0x03
#define SDHCI_POWER_SD_BUS_OFF	0x02

/* Host control (0x1c) */
#define SDHCI_CTRL_4BIT		0x02
#define SDHCI_CTRL_8BIT		0x04
#define SDHCI_CTRL_SDMA		0x00
#define SDHCI_CTRL_ADMA		0x08
#define SDHCI_CTRL_1V8		0x10	/* 1.8 V signalling */

/*
 * Clock control (0x24).
 *
 * In the SDHCI 3.0 layout the bus clock divider is 10 bits wide:
 * bits 8-15 hold the low 8 bits and bits 6-7 the high 2.  The value
 * written is the divider minus one, and it must be even, because the
 * controller divides by (value + 1) and the specification requires the
 * divisor to be a multiple of two above 25 MHz.
 */
#define SDHCI_CLK_DIV_MASK	0x00ff		/* bits 8-15 */
#define SDHCI_CLK_DIV_SHIFT	8
#define SDHCI_CLK_DIV_MASK_LEN	8
#define SDHCI_CLK_DIV_HI_MASK	0x0300		/* bits 6-7 */
#define SDHCI_CLK_DIV_HI_SHIFT	6
#define SDHCI_CLK_INT_EN	0x0004
#define SDHCI_CLK_STABLE	0x0008
#define SDHCI_CLK_SDCLK_EN	0x0010

/* Present state (0x08) */
#define SDHCI_CMD_INHIBIT	0x0001
#define SDHCI_DATA_INHIBIT	0x0002
#define SDHCI_DAT_LINE_ACTIVE	0x0004
#define SDHCI_WRITE_XFER_ACTIVE	0x0008
#define SDHCI_READ_XFER_ACTIVE	0x0010
#define SDHCI_CLOCK_STABLE	0x0200
#define SDHCI_DATA0_LEVEL	0x0800
#define SDHCI_DATA1_LEVEL	0x1000
#define SDHCI_DATA2_LEVEL	0x2000
#define SDHCI_DATA3_LEVEL	0x4000
#define SDHCI_CMD_LINE_LEVEL	0x8000

/* Interrupt status / enable (0x2c / 0x30) */
#define SDHCI_INT_RESPONSE	0x00000001
#define SDHCI_INT_CMD_COMPLETE	0x00000002
#define SDHCI_INT_DATA_COMPLETE	0x00000004
#define SDHCI_INT_BLOCK_GAP	0x00000008
#define SDHCI_INT_TRANSFER_COMPLETE 0x00000020
#define SDHCI_INT_ADMA_ERROR	0x00000040
#define SDHCI_INT_TIMEOUT	0x00010000
#define SDHCI_INT_CRC_ERROR	0x00020000
#define SDHCI_INT_CARD_REMOVE	0x00040000
#define SDHCI_INT_CARD_INSERT	0x00080000
#define SDHCI_INT_ERROR_MASK	(SDHCI_INT_ADMA_ERROR | \
				 SDHCI_INT_TIMEOUT | \
				 SDHCI_INT_CRC_ERROR)

/*
 * Command register flags (0x0c).
 *
 * The response type lives in bits 1:0, the CRC check in bit 2, the index
 * in bit 3, "data present" in bit 4 and the direction in bit 5.  Note
 * that the response-type encodings overlap with the flag bits above, so
 * the two must never be OR'ed into the same word -- the driver keeps
 * them apart and assembles the final value in one place.
 *
 * There is no SDHCI_CMD_READ: the direction of a data transfer is taken
 * from the transfer mode register (0x0e), not from the command
 * register.  Linux's sdhci.c works the same way.
 */
#define SDHCI_CMD_RESP_NONE	0x00000000
#define SDHCI_CMD_RESP_SHORT	0x00000004	/* bit 2: 35 bit response */
#define SDHCI_CMD_RESP_LONG	0x00000008	/* bit 3: 64 bit response */
#define SDHCI_CMD_RESP_SHORT_BUSY 0x0000000c	/* bits 3:2: with busy */
#define SDHCI_CMD_CRC		0x00000004	/* bit 2: CRC check present */
#define SDHCI_CMD_INDEX		0x00000008	/* bit 3: response carries index */
#define SDHCI_CMD_DATA		0x00000010	/* bit 4: data present */

/*
 * Response kinds, as used by the driver's emmc_cmd() resp_type argument.
 * These are internal to the driver and never written to the register
 * directly.
 */
#define EMMC_RSP_NONE		0
#define EMMC_RSP_SHORT		1	/* 35 bit, no busy */
#define EMMC_RSP_LONG		2	/* 64 bit */
#define EMMC_RSP_SHORT_BUSY	3	/* 35 bit with busy (R1b) */

/* ADMA descriptor sizes */
#define ADMA_DESC_SZ_32		12
#define ADMA_DESC_SZ_64		16

/* ADMA descriptor attributes */
#define ADMA_VALID		0x00000001
#define ADMA_END		0x00000002
#define ADMA_INT		0x00000004
#define ADMA_NOP		0x00000008
#define ADMA_DESC_64BIT		0x00000010
#define ADMA_LEN_SHIFT		16
#define ADMA_LEN_MASK		0x0000ffff

/*
 * MMC / eMMC command indices and arguments.
 */
#define MMC_GO_IDLE		0
#define MMC_SEND_OP_COND	1
#define MMC_ALL_SEND_CID	2
#define MMC_SET_RELATIVE_ADDR	3
#define MMC_SELECT_CARD		7
#define MMC_SEND_EXT_CSD	8
#define MMC_SEND_CSD		9
#define MMC_STOP_TRANSMISSION	12
#define MMC_SEND_STATUS		13
#define MMC_SET_BLOCKLEN	16
#define MMC_READ_SINGLE_BLOCK	17
#define MMC_READ_MULTIPLE_BLOCK	18
#define MMC_WRITE_MULTIPLE_BLOCK	25
#define MMC_APP_CMD		55

#define MMC_RSP_SPI_R1		1
#define MMC_RSP_R2		2
#define MMC_RSP_R3		3
#define MMC_RSP_R1B		4
#define MMC_RSP_R2R3		5
#define MMC_RSP_R1B2		5

/* OCR bits */
#define OCR_BUSY		0x80000000
#define OCR_HCS			0x40000000	/* high capacity */
#define OCR_SECTORS		0x20000000	/* sector addressing */
#define OCR_ACCESS_MODE		0x00000030
#define OCR_POWERUP		0x00000001
#define OCR_VOLTAGE_SHIFT	8
#define OCR_VOLTAGE_MASK	0x00ff0000

/*
 * EXT_CSD indices we care about (JEDEC JESD84-B51).
 */
#define EXT_CSD_REV		192
#define EXT_CSD_SEC_CNT		212	/* 4-byte, in 512-byte units */
#define EXT_CSD_DEV_TYPE	196
#define EXT_CSD_PREP_EOL		241
#define EXT_CSD_HC_ERASE_GRP_SIZE	221
#define EXT_CSD_BKOPS		192
#define EXT_CSD_CACHE_SIZE	249	/* 4-byte */
#define EXT_CSD_FS_TYPES		254

/*
 * Device geometry as reported by EXT_CSD.
 */
#define EMMC_ADDR_SHIFT		9	/* 512-byte blocks -> sectors */
#define EMMC_SEC_SIZE		512

#endif /* _SWEET_EMMC_H_ */
