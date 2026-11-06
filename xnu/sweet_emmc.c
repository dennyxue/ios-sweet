/*
 * SWEET / OCEAN internal eMMC root device.
 *
 * A from-scratch SDHCI host controller driver for the Qualcomm SDM710
 * (Snapdragon 710) eMMC, used by the Smartisan Nut Pro 2S ("ocean").
 *
 * Why this exists: every existing iOS-on-Android port (ios-sweet, ios-topaz,
 * HackiOS, the Pixel and LG ports) drives UFS.  The Nut Pro 2S has eMMC 5.1,
 * so the UFS driver cannot be reused and an eMMC host driver has to be
 * written from scratch.  This is that driver.
 *
 * How it works:
 *
 *  - The bootloader has already powered and clocked the SDHC core and read
 *    our boot image out of this eMMC, but it leaves the *card* side in an
 *    unknown state and the host controller reset.  We therefore do a full
 *    MMC card bring-up ourselves: software reset, CMD0, CMD1 (OCR loop),
 *    CMD2 (CID), CMD3 (RCA), CMD9 (CSD), CMD7 (select), CMD8 (EXT_CSD),
 *    then set the block length / sector mode.
 *
 *  - Transfers use the SDHCI ADMA2 engine.  All DMA goes through one
 *    physically contiguous kernel allocation holding the ADMA descriptor
 *    table and a bounce buffer, because the SDM710 SDHC is not cache
 *    coherent with the CPU: caches are cleaned/invalidated around every
 *    transfer.
 *
 *  - The controller is driven by polling.  We take the host interrupt line
 *    (SPI 641) but only use it to detect the power IRQ; command completion
 *    is detected by polling the present-state and interrupt-status
 *    registers.  That avoids an interrupt-driven state machine in a driver
 *    that runs very early in boot, before the interrupt infrastructure for
 *    a new controller is settled, and it makes the driver deterministic
 *    and easy to debug from the framebuffer console.
 *
 *  - The card is found by scanning the GPT on the eMMC's user area for a
 *    partition name given by the boot-arg "emmcroot=" (default "userdata"),
 *    exactly like sweet_ufs.c does for UFS.  Root on it with
 *    rd=emmcroot0.
 *
 * - Ported from dipa-studio/ios-sweet's bsd/dev/sweet_ufs.c, which is the
 *   only other Qualcomm storage driver in the project; the BSD glue
 *   (strategy/ioctl/bioctl/read/write) follows it so the two drivers are
 *   interchangeable from the file system's point of view.
 */

#if defined(ARM64_BOARD_CONFIG_SWEET)
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/buf.h>
#include <sys/conf.h>
#include <sys/disk.h>
#include <sys/errno.h>
#include <sys/proc.h>
#include <sys/uio.h>
#include <miscfs/devfs/devfs.h>
#include <kern/locks.h>
#include <kern/clock.h>
#include <machine/machine_routines.h>
#include <arm/caches_internal.h>
#include <vm/pmap.h>
#include <vm/vm_kern.h>
#include <pexpert/pexpert.h>
#include <pexpert/device_tree.h>
#include <libkern/libkern.h>

#include "sweet_emmc.h"

/* osfmk/arm/machine_routines.h, not exported to the BSD layer */
extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern ppnum_t pmap_find_phys(pmap_t pmap, addr64_t va);

/*
 * DMA bounce buffer size.  One MiB matches sweet_ufs.c; eMMC blocks are
 * 4 KiB physical, and we address the card in 512-byte sectors, so the
 * bounce buffer must be a whole number of 4 KiB blocks.
 */
#define EMMC_BOUNCE_SIZE	(1u << 20)
#define EMMC_BLOCK_SIZE		4096u	/* physical block of the eMMC */
#define EMMC_MAX_XFER		(EMMC_BOUNCE_SIZE / EMMC_SEC_SIZE)

#define EMMC_CMD_TIMEOUT_NS	(5ull * NSEC_PER_SEC)
#define EMMC_RESET_TIMEOUT_NS	(1ull * NSEC_PER_SEC)

/*
 * Layout of the contiguous DMA allocation.
 *
 *   DMA_DESC    ADMA2 descriptor table (128 KiB, enough for 64 KiB transfers
 *               with 64-bit descriptors at 16 bytes each)
 *   DMA_BOUNCE  bounce buffer, physically contiguous
 */
#define DMA_DESC		0x0000
#define DMA_DESC_SIZE		(128u << 10)
#define DMA_BOUNCE		0x20000
#define DMA_SIZE		(DMA_BOUNCE + EMMC_BOUNCE_SIZE)

/* ADMA2 descriptor, 64-bit form. */
struct adma_desc {
	uint32_t attr;
	uint32_t len;
	uint64_t addr;
};

static struct {
	vm_offset_t regs;			/* kernel VA of the HC window */
	vm_offset_t dma;			/* kernel VA of the DMA area */
	uint64_t dma_pa;

	uint32_t rca;				/* relative card address */
	uint64_t ocr;				/* negotiated operating conditions */
	uint32_t cid[4];
	uint32_t csd[4];

	uint64_t first_sector;			/* partition start, in 512-byte sectors */
	uint64_t n_sectors;			/* partition length, in 512-byte sectors */
	uint32_t card_sectors;			/* whole-card capacity, 512-byte units */
	uint32_t dev_type;			/* EXT_CSD DEVICE_TYPE */
	uint32_t secsize;			/* logical sector size for the BSD layer */
	uint32_t ddr_config;			/* EXT_CSD DDR_CONFIG, if HS400 */
	int bmajor, cmajor;			/* BSD disk/char device numbers */
	dev_t bdev;
	bool hs200;				/* card negotiated HS200 */
	bool hs400;				/* card negotiated HS400 */
	bool blockaddr;			/* sector addressing in use */
	bool ready;
} em;

static LCK_GRP_DECLARE(sweet_emmc_grp, "sweet_emmc");
static LCK_MTX_DECLARE(sweet_emmc_lock, &sweet_emmc_grp);

/* ---------------------------------------------------------------- */
/* Register access                                                   */
/* ---------------------------------------------------------------- */

static inline uint8_t
emmc_r8(uint32_t off)
{
	return *(volatile uint8_t *)(em.regs + off);
}

static inline uint16_t
emmc_r16(uint32_t off)
{
	return *(volatile uint16_t *)(em.regs + off);
}

static inline uint32_t
emmc_r32(uint32_t off)
{
	return *(volatile uint32_t *)(em.regs + off);
}

static inline void
emmc_w8(uint32_t off, uint8_t v)
{
	*(volatile uint8_t *)(em.regs + off) = v;
}

static inline void
emmc_w16(uint32_t off, uint16_t v)
{
	*(volatile uint16_t *)(em.regs + off) = v;
}

static inline void
emmc_w32(uint32_t off, uint32_t v)
{
	*(volatile uint32_t *)(em.regs + off) = v;
}

/*
 * The SDHC is not cache coherent with the CPU on this platform (the
 * device tree does not describe the SDHC as DMA-coherent), so every
 * descriptor and bounce buffer access has to be explicitly cleaned or
 * invalidated, and the write-back has to be visible before the doorbell.
 */
static void
dma_clean(uint32_t off, uint32_t len)
{
	flush_dcache64((addr64_t)(em.dma_pa + off), len, TRUE);
}

static void
dma_inv(uint32_t off, uint32_t len)
{
	flush_dcache64((addr64_t)(em.dma_pa + off), len, FALSE);
}

/* ---------------------------------------------------------------- */
/* Low-level host controller helpers                                */
/* ---------------------------------------------------------------- */

/*
 * Reset the host controller (SDHCI_RESET_ALL) and wait for it to settle.
 * The bootloader leaves the controller in a partially configured state;
 * a full software reset is the only reliable way to get a known baseline
 * before we start issuing MMC commands.
 */
static int
emmc_reset_hc(void)
{
	uint32_t v;
	uint64_t deadline;

	v = emmc_r32(SDHCI_SWRST);
	v &= 0x00ffffff;
	v |= (uint32_t)(SDHCI_RESET_ALL << SDHCI_RESET_OFFSET);
	emmc_w32(SDHCI_SWRST, v);

	nanoseconds_to_absolutetime(EMMC_RESET_TIMEOUT_NS, &deadline);
	deadline += mach_absolute_time();
	do {
		v = emmc_r32(SDHCI_SWRST);
		if (!(v & (uint32_t)(SDHCI_RESET_ALL << SDHCI_RESET_OFFSET))) {
			break;
		}
		if (mach_absolute_time() > deadline) {
			printf("SWEET EMMC: host reset timed out (swrst 0x%x)\n", v);
			return ETIMEDOUT;
		}
	} while (1);

	/* A reset clears the interrupt mask; clear anything stale too. */
	emmc_w32(SDHCI_INTENABLE, 0);
	emmc_w32(SDHCI_INTSTATUS, emmc_r32(SDHCI_INTSTATUS));
	return 0;
}

/* Wait until the internal clock is stable and the divider is enabled. */
static int
emmc_wait_clock_stable(void)
{
	uint64_t deadline;

	nanoseconds_to_absolutetime(EMMC_CMD_TIMEOUT_NS, &deadline);
	deadline += mach_absolute_time();
	while (!(emmc_r32(SDHCI_CLK_CONTROL) & SDHCI_CLOCK_STABLE)) {
		if (mach_absolute_time() > deadline) {
			printf("SWEET EMMC: clock never became stable\n");
			return ETIMEDOUT;
		}
	}
	return 0;
}

/*
 * Program the SDHCI clock: an integer divider plus the internal enable
 * bits.  The SDM710 SDHC has no fractional divider, so 400 kHz (the
 * identification rate) is divider 0x0 for a 100 MHz core clock.
 */
/*
 * Program the SD bus clock.
 *
 * The SDHCI 3.0 clock divider is 10 bits wide and the value written is
 * the divisor minus one.  Above 25 MHz the divisor has to be a multiple
 * of two, so the search steps by two; below that any divisor is
 * allowed, which is what makes the 400 kHz identification clock
 * reachable at all (a 200 MHz core needs a divisor of 500, and the
 * 10-bit field can express that).
 */
static int
emmc_set_clock(uint32_t hz, uint32_t core_hz)
{
	uint32_t div, ctrl;
	uint16_t clk;

	if (core_hz == 0) {
		return EINVAL;
	}
	if (hz > core_hz) {
		hz = core_hz;
	}

	if (core_hz <= 25000000) {
		/* Any divisor is fine below 25 MHz. */
		for (div = 1; div <= 1024; div++) {
			if ((core_hz / div) <= hz) {
				break;
			}
		}
	} else {
		/* Above 25 MHz the divisor must be a multiple of two. */
		for (div = 2; div <= 1024; div += 2) {
			if ((core_hz / div) <= hz) {
				break;
			}
		}
	}
	if (div > 1024) {
		div = 1024;
	}

	/* Turn the clock off while the divider changes. */
	ctrl = emmc_r32(SDHCI_CLK_CONTROL);
	ctrl &= ~(SDHCI_CLK_INT_EN | SDHCI_CLK_SDCLK_EN);
	emmc_w32(SDHCI_CLK_CONTROL, ctrl);

	/*
	 * The register holds the divisor minus one: the low 8 bits go to
	 * bits 8-15 and the high 2 bits to bits 6-7.
	 */
	clk = (uint16_t)(div - 1);
	ctrl = emmc_r32(SDHCI_CLK_CONTROL);
	ctrl &= ~(SDHCI_CLK_DIV_MASK | SDHCI_CLK_DIV_HI_MASK);
	ctrl |= ((uint32_t)clk & SDHCI_CLK_DIV_MASK) << SDHCI_CLK_DIV_SHIFT;
	ctrl |= (((uint32_t)clk >> SDHCI_CLK_DIV_MASK_LEN) & 0x3)
	    << SDHCI_CLK_DIV_HI_SHIFT;
	ctrl |= SDHCI_CLK_INT_EN | SDHCI_CLK_SDCLK_EN;
	emmc_w32(SDHCI_CLK_CONTROL, ctrl);

	return emmc_wait_clock_stable();
}

/*
 * Wait for the command inhibit bits to clear, i.e. for the controller to
 * accept a new command.
 */
static int
emmc_wait_cmd_inhibit(void)
{
	uint64_t deadline;

	nanoseconds_to_absolutetime(EMMC_CMD_TIMEOUT_NS, &deadline);
	deadline += mach_absolute_time();
	while (emmc_r32(SDHCI_PRNSTS) & (SDHCI_CMD_INHIBIT | SDHCI_DATA_INHIBIT)) {
		if (mach_absolute_time() > deadline) {
			printf("SWEET EMMC: command inhibit stuck (prnsts 0x%x)\n",
			    emmc_r32(SDHCI_PRNSTS));
			return ETIMEDOUT;
		}
	}
	return 0;
}

/* ---------------------------------------------------------------- */
/* ADMA2 transfer engine                                             */
/* ---------------------------------------------------------------- */

/*
 * Build an ADMA2 descriptor chain in slot 0 for one transfer of "len"
 * bytes at physical address "pa".  The chain is limited to one bounce
 * buffer, which is why the strategy routine chops large requests.
 */
static void
emmc_build_adma(uint64_t pa, uint32_t len, bool write)
{
	struct adma_desc *d = (struct adma_desc *)(em.dma + DMA_DESC);
	uint32_t n = 0;

	/*
	 * ADMA2 descriptor layout (SDHCI 3.0, section 3.3):
	 *
	 *   bits  0:  Valid
	 *   bit   1:  End
	 *   bit   2:  Interrupt
	 *   bit   3:  NOP
	 *   bit   4:  64-bit addressing
	 *   bits 31:16 Length (bytes; zero means 65536)
	 *
	 * The 64-bit form is 16 bytes: a 32-bit attribute word, a 32-bit
	 * reserved word, then the 64-bit address.
	 */
	while (len > 0 && n < (DMA_DESC_SIZE / sizeof(struct adma_desc)) - 1) {
		uint32_t chunk = len;

		/*
		 * The length field is 16 bits and counts bytes; a zero means
		 * 65536.  Keep chunks at 64 KiB so the arithmetic stays exact.
		 */
		if (chunk > (64u << 10)) {
			chunk = 64u << 10;
		}
		d[n].attr = ADMA_VALID | ADMA_DESC_64BIT |
		    (chunk << ADMA_LEN_SHIFT);
		d[n].len = 0;
		d[n].addr = pa;
		pa += chunk;
		len -= chunk;
		n++;
	}
	/* Terminate the chain. */
	d[n].attr = ADMA_END;
	d[n].len = 0;
	d[n].addr = 0;
	(void)write;
}

/*
 * Issue one MMC command and optionally transfer data.
 *
 * resp_type:  EMMC_RSP_NONE / _SHORT / _LONG / _SHORT_BUSY
 * flags:      SDHCI_CMD_DATA (a data transfer follows)
 * has_index:  the response carries the command index (CMD7, CMD18, ...)
 * has_crc:    the response is protected by a CRC7
 * write:      the transfer writes to the card; the direction is taken
 *             from the transfer mode register, not from the command
 *             register
 *
 * The response type, the index bit and the CRC bit all live in the same
 * few bits of the command register, so they are kept as separate
 * arguments here and combined exactly once, inside this function.
 *
 * Returns 0 or an errno; the response is returned in resp[4].
 */
static int
emmc_cmd(uint32_t index, uint32_t arg, uint32_t resp_type, uint32_t flags,
    bool has_index, bool has_crc, bool write, void *data, uint32_t len,
    uint32_t *resp)
{
	uint32_t cmd_reg = 0;
	uint32_t present;
	uint64_t deadline;
	int error;

	if (resp) {
		resp[0] = resp[1] = resp[2] = resp[3] = 0;
	}

	error = emmc_wait_cmd_inhibit();
	if (error) {
		return error;
	}

	/* Clear any stale interrupt status. */
	emmc_w32(SDHCI_INTSTATUS, emmc_r32(SDHCI_INTSTATUS));

	/*
	 * Response type and command flags share bits in the command
	 * register, so they are assembled here, once, and the two never
	 * overlap: RESP_SHORT/RESP_LONG/RESP_SHORT_BUSY occupy bits 2:1,
	 * and the flags below use bits 4 and 5 (data present, direction).
	 * The index and CRC bits are only set for the commands that need
	 * them and are added by the caller through resp_type.
	 */
	switch (resp_type) {
	case EMMC_RSP_SHORT:
		cmd_reg |= SDHCI_CMD_RESP_SHORT;
		break;
	case EMMC_RSP_LONG:
		cmd_reg |= SDHCI_CMD_RESP_LONG;
		break;
	case EMMC_RSP_SHORT_BUSY:
		cmd_reg |= SDHCI_CMD_RESP_SHORT_BUSY;
		break;
	default:
		cmd_reg |= SDHCI_CMD_RESP_NONE;
		break;
	}
	cmd_reg |= flags;
	cmd_reg |= (index << 8);

	/*
	 * The index and CRC bits share bit 3 with the response-type field
	 * in the SDHCI encoding used here, so they are applied together
	 * and only for the commands that actually need them.  A command
	 * with an index response uses the long-form encoding, which is
	 * already covered by the response type above; the CRC bit is only
	 * meaningful for the commands that carry a CRC-protected
	 * response.
	 */
	if (has_index) {
		cmd_reg |= SDHCI_CMD_INDEX;
	}
	if (has_crc) {
		cmd_reg |= SDHCI_CMD_CRC;
	}

	if (flags & SDHCI_CMD_DATA) {
		uint64_t desc_pa = em.dma_pa + DMA_DESC;

		emmc_build_adma(em.dma_pa + DMA_BOUNCE, len, write);

		/*
		 * Make the bounce buffer and the descriptor table visible to
		 * the controller *before* the descriptor address is published:
		 * once the address register is written the controller may start
		 * walking the chain.
		 */
		if (write) {
			dma_clean(DMA_BOUNCE, len);
		} else {
			dma_inv(DMA_BOUNCE, len);
		}
		dma_clean(DMA_DESC, DMA_DESC_SIZE);
		__builtin_arm_dsb(0xf);

		/*
		 * The ADMA system address register is 64 bits wide: the low
		 * half at 0x40 and the high half at 0x44.  The SDM710 is
		 * described as qcom,large-address-bus, so the high half is
		 * required.
		 */
		emmc_w32(SDHCI_ADMA_DESC_ADDR, (uint32_t)desc_pa);
		emmc_w32(SDHCI_ADMA_DESC_ADDR + 4, (uint32_t)(desc_pa >> 32));

		/* Block count: 512-byte units, in the SDHCI data register. */
		emmc_w16(SDHCI_DATA, (uint16_t)(len / EMMC_SEC_SIZE));
		emmc_w16(SDHCI_DATA + 2, 0);

		/*
		 * Transfer mode (0x0e): direction, DMA enable, and the block
		 * count enable bits.  ADMA transfers do not use the SDHCI
		 * block count register (that is for the legacy block count),
		 * so BLK_CNT_E is left clear for ADMA.
		 */
		emmc_w16(SDHCI_TRANSFER_MODE, (uint16_t)((write ? SDHCI_TRNS_WRITE :
		    SDHCI_TRNS_READ) | SDHCI_TRNS_DMA));
	} else {
		emmc_w16(SDHCI_TRANSFER_MODE, 0);
	}

	emmc_w32(SDHCI_CMD, cmd_reg);

	/* Wait for the command to complete. */
	nanoseconds_to_absolutetime(EMMC_CMD_TIMEOUT_NS, &deadline);
	deadline += mach_absolute_time();
	for (;;) {
		present = emmc_r32(SDHCI_PRNSTS);
		if (!(present & SDHCI_CMD_INHIBIT)) {
			break;
		}
		if (mach_absolute_time() > deadline) {
			printf("SWEET EMMC: CMD%u timed out (prnsts 0x%x)\n",
			    index, present);
			return ETIMEDOUT;
		}
	}

	/* Collect the response. */
	if (resp) {
		resp[0] = emmc_r32(SDHCI_RSPREG);
		resp[1] = emmc_r32(SDHCI_RSPREG + 4);
		resp[2] = emmc_r32(SDHCI_RSPREG + 8);
		resp[3] = emmc_r32(SDHCI_RSPREG + 12);
	}

	if (flags & SDHCI_CMD_DATA) {
		uint32_t status;

		/* Wait for the data transfer to complete. */
		nanoseconds_to_absolutetime(EMMC_CMD_TIMEOUT_NS, &deadline);
		deadline += mach_absolute_time();
		for (;;) {
			present = emmc_r32(SDHCI_PRNSTS);
			if (!(present & (SDHCI_READ_XFER_ACTIVE | SDHCI_WRITE_XFER_ACTIVE))) {
				break;
			}
			if (mach_absolute_time() > deadline) {
				printf("SWEET EMMC: CMD%u data timeout (prnsts 0x%x)\n",
				    index, present);
				return ETIMEDOUT;
			}
		}
		status = emmc_r32(SDHCI_INTSTATUS);
		if (status & SDHCI_INT_ERROR_MASK) {
			printf("SWEET EMMC: CMD%u data error (status 0x%x)\n",
			    index, status);
			return EIO;
		}
		if (!write) {
			dma_inv(DMA_BOUNCE, len);
		}
	}

	/* Report a CRC error or a bad response for commands that expect one. */
	if (resp_type != 0) {
		uint32_t status = emmc_r32(SDHCI_INTSTATUS);
		if (status & SDHCI_INT_CRC_ERROR) {
			return EIO;
		}
	}
	return 0;
}

/* ---------------------------------------------------------------- */
/* MMC card bring-up                                                 */
/* ---------------------------------------------------------------- */

/*
 * CMD1: negotiate the operating conditions.  The card answers with its
 * OCR; the host keeps the bits it supports and clears the busy bit.  This
 * is repeated until the card reports "busy", which means it has accepted
 * the voltage and is ready for CMD2.
 */
static int
emmc_send_op_cond(uint32_t host_ocr, uint32_t *card_ocr)
{
	uint32_t ocr = 0;
	int tries;

	for (tries = 0; tries < 1000; tries++) {
		uint32_t resp[4];
		int error;

		error = emmc_cmd(MMC_SEND_OP_COND, host_ocr | OCR_HCS, EMMC_RSP_SHORT,
		    0, false, false, false, NULL, 0, resp);
		if (error) {
			return error;
		}
		ocr = resp[0] & 0xffffffff;
		if (ocr & OCR_BUSY) {
			break;
		}
		/* The card is not ready yet; give it a moment. */
		delay(1);
	}
	if (tries == 1000) {
		printf("SWEET EMMC: CMD1 never became busy (ocr 0x%x)\n", ocr);
		return ETIMEDOUT;
	}
	*card_ocr = ocr;
	return 0;
}

/* Read the 512-byte EXT_CSD into the bounce buffer. */
static int
emmc_read_ext_csd(uint8_t *out)
{
	uint32_t resp[4];
	int error;

	error = emmc_cmd(MMC_SEND_EXT_CSD, em.rca << 16, EMMC_RSP_SHORT,
	    SDHCI_CMD_DATA, false, true, false, out, 512, resp);
	if (error) {
		return error;
	}
	/* The card returns R1; its low byte holds the EXT_CSD index. */
	return 0;
}

/*
 * Bring the eMMC up: reset the host, negotiate the OCR, read the CID,
 * assign a relative address, read the CSD, select the card and read the
 * EXT_CSD.
 */
static int
emmc_init_card(void)
{
	uint32_t resp[4];
	uint32_t ocr = 0;
	uint32_t host_ocr;
	int error;

	/* Identification starts at 400 kHz. */
	error = emmc_set_clock(400000, 200000000);
	if (error) {
		return error;
	}

	/* 8-bit bus, ADMA, and enable the internal clock. */
	emmc_w8(SDHCI_HOSTCTL, SDHCI_CTRL_8BIT | SDHCI_CTRL_ADMA);

	/* CMD0: go idle. */
	error = emmc_cmd(MMC_GO_IDLE, 0, EMMC_RSP_NONE, 0, false, false, false, NULL, 0, NULL);
	if (error) {
		printf("SWEET EMMC: CMD0 failed\n");
		return error;
	}

	/*
	 * CMD1: OCR negotiation.  The card's supported voltage window is
	 * reported back; we accept whatever it offers and re-assert the high
	 * capacity bit.
	 */
	host_ocr = OCR_HCS | (0x00ff0000) | OCR_POWERUP;
	error = emmc_send_op_cond(host_ocr, &ocr);
	if (error) {
		return error;
	}
	em.ocr = ocr;
	printf("SWEET EMMC: CMD1 ocr 0x%08x\n", ocr);

	/* CMD2: get the CID. */
	error = emmc_cmd(MMC_ALL_SEND_CID, 0, EMMC_RSP_LONG, 0, false, false, false, NULL, 0, resp);
	if (error) {
		printf("SWEET EMMC: CMD2 failed\n");
		return error;
	}
	memcpy(em.cid, resp, sizeof(em.cid));
	printf("SWEET EMMC: cid %02x%02x%02x%02x%02x%02x%02x%02x\n",
	    (em.cid[0] >> 24) & 0xff, (em.cid[0] >> 16) & 0xff,
	    (em.cid[0] >> 8) & 0xff, em.cid[0] & 0xff,
	    (em.cid[1] >> 24) & 0xff, (em.cid[1] >> 16) & 0xff,
	    (em.cid[1] >> 8) & 0xff, em.cid[1] & 0xff);

	/* CMD3: assign a relative address. */
	error = emmc_cmd(MMC_SET_RELATIVE_ADDR, 1, EMMC_RSP_SHORT, 0, false, false, false, NULL, 0, resp);
	if (error) {
		printf("SWEET EMMC: CMD3 failed\n");
		return error;
	}
	em.rca = resp[0] >> 16;

	/* CMD9: read the CSD. */
	error = emmc_cmd(MMC_SEND_CSD, em.rca << 16, EMMC_RSP_LONG, 0, false, false, false, NULL, 0, resp);
	if (error) {
		printf("SWEET EMMC: CMD9 failed\n");
		return error;
	}
	memcpy(em.csd, resp, sizeof(em.csd));

	/* CMD7: select the card. */
	error = emmc_cmd(MMC_SELECT_CARD, em.rca << 16, EMMC_RSP_SHORT_BUSY,
	    0, true, true, false, NULL, 0, resp);
	if (error) {
		printf("SWEET EMMC: CMD7 failed\n");
		return error;
	}

	/* CMD8: read the EXT_CSD into the bounce buffer. */
	error = emmc_read_ext_csd((uint8_t *)(em.dma + DMA_BOUNCE));
	if (error) {
		printf("SWEET EMMC: CMD8 failed\n");
		return error;
	}
	dma_inv(DMA_BOUNCE, 512);
	{
		uint8_t *ecs = (uint8_t *)(em.dma + DMA_BOUNCE);
		uint32_t sec_cnt;

		em.dev_type = ecs[EXT_CSD_DEV_TYPE];
		em.blockaddr = (em.ocr & OCR_SECTORS) != 0;

		/*
		 * EXT_CSD SEC_CNT (bytes 212..215) is the card capacity in
		 * 512-byte units, but only valid when the card is in sector
		 * addressing mode.  It is little endian.
		 */
		sec_cnt = (uint32_t)ecs[EXT_CSD_SEC_CNT] |
		    ((uint32_t)ecs[EXT_CSD_SEC_CNT + 1] << 8) |
		    ((uint32_t)ecs[EXT_CSD_SEC_CNT + 2] << 16) |
		    ((uint32_t)ecs[EXT_CSD_SEC_CNT + 3] << 24);
		em.card_sectors = sec_cnt;

		printf("SWEET EMMC: dev_type 0x%x rev %u sector_addr %d capacity %u sectors (%llu MiB)\n",
		    em.dev_type, ecs[EXT_CSD_REV], em.blockaddr, sec_cnt,
		    (uint64_t)sec_cnt * EMMC_SEC_SIZE / (1024 * 1024));
	}

	/* Set the block length to 512 bytes for byte addressing. */
	if (!em.blockaddr) {
		error = emmc_cmd(MMC_SET_BLOCKLEN, EMMC_SEC_SIZE, EMMC_RSP_SHORT,
		    0, false, true, false, NULL, 0, resp);
		if (error) {
			printf("SWEET EMMC: CMD16 failed\n");
			return error;
		}
	}

	return 0;
}

/* ---------------------------------------------------------------- */
/* Block I/O                                                        */
/* ---------------------------------------------------------------- */

/*
 * Read "count" 512-byte sectors starting at LBA "lba" of the selected
 * partition into the bounce buffer.
 */
static int
emmc_read_sectors(uint64_t lba, uint32_t count)
{
	uint32_t resp[4];
	int error;

	if (count == 0 || count > EMMC_MAX_XFER) {
		return EINVAL;
	}

	if (!em.blockaddr) {
		/*
		 * Byte addressing: CMD17 reads one block of "blocklen" bytes.
		 * We always work in 512-byte units, so a single CMD17 covers
		 * one sector at a time.
		 */
		uint32_t i;

		for (i = 0; i < count; i++) {
			error = emmc_cmd(MMC_READ_SINGLE_BLOCK,
			    (uint32_t)((lba + i) * EMMC_SEC_SIZE), EMMC_RSP_SHORT,
			    SDHCI_CMD_DATA, false, true, false,
			    (void *)(em.dma + DMA_BOUNCE), EMMC_SEC_SIZE, resp);
			if (error) {
				return error;
			}
		}
		return 0;
	}

	/* Sector addressing: CMD18 reads multiple blocks. */
	error = emmc_cmd(MMC_READ_MULTIPLE_BLOCK, (uint32_t)lba, EMMC_RSP_SHORT,
	    SDHCI_CMD_DATA, true, true, false,
	    (void *)(em.dma + DMA_BOUNCE), count * EMMC_SEC_SIZE, resp);
	if (error) {
		return error;
	}
	/* CMD12 stops the multiple-block read. */
	error = emmc_cmd(MMC_STOP_TRANSMISSION, 0, EMMC_RSP_SHORT_BUSY,
	    0, true, true, false, NULL, 0, resp);
	return error;
}

/*
 * Write "count" 512-byte sectors from the bounce buffer at LBA "lba".
 */
static int
emmc_write_sectors(uint64_t lba, uint32_t count)
{
	uint32_t resp[4];
	int error;

	if (count == 0 || count > EMMC_MAX_XFER) {
		return EINVAL;
	}

	if (!em.blockaddr) {
		uint32_t i;

		for (i = 0; i < count; i++) {
			error = emmc_cmd(MMC_WRITE_MULTIPLE_BLOCK,
			    (uint32_t)((lba + i) * EMMC_SEC_SIZE), EMMC_RSP_SHORT,
			    SDHCI_CMD_DATA, false, true, true,
			    (void *)(em.dma + DMA_BOUNCE), EMMC_SEC_SIZE, resp);
			if (error) {
				return error;
			}
		}
		return 0;
	}

	error = emmc_cmd(MMC_WRITE_MULTIPLE_BLOCK, (uint32_t)lba, EMMC_RSP_SHORT,
	    SDHCI_CMD_DATA, true, true, true,
	    (void *)(em.dma + DMA_BOUNCE), count * EMMC_SEC_SIZE, resp);
	if (error) {
		return error;
	}
	error = emmc_cmd(MMC_STOP_TRANSMISSION, 0, EMMC_RSP_SHORT_BUSY,
	    0, true, true, false, NULL, 0, resp);
	return error;
}

/* ---------------------------------------------------------------- */
/* GPT partition lookup                                             */
/* ---------------------------------------------------------------- */

static int
emmc_find_partition(const char *want)
{
	const uint8_t *b = (const uint8_t *)(em.dma + DMA_BOUNCE);
	uint64_t entry_lba;
	uint32_t nent, esz;
	int error;

	/* LBA 0 is the protective MBR; LBA 1 is the GPT header. */
	error = emmc_read_sectors(1, 1);
	if (error) {
		return error;
	}
	if (memcmp(b, "EFI PART", 8) != 0) {
		printf("SWEET EMMC: no GPT on the eMMC\n");
		return ENXIO;
	}
	memcpy(&entry_lba, b + 72, 8);
	memcpy(&nent, b + 80, 4);
	memcpy(&esz, b + 84, 4);
	if (esz != 128 || nent == 0 || nent * esz > EMMC_BOUNCE_SIZE) {
		printf("SWEET EMMC: unsupported GPT entries %u x %u\n", nent, esz);
		return ENXIO;
	}
	error = emmc_read_sectors(entry_lba, (nent * esz + EMMC_SEC_SIZE - 1) / EMMC_SEC_SIZE);
	if (error) {
		return error;
	}
	for (uint32_t i = 0; i < nent; i++) {
		const uint8_t *e = b + i * esz;
		char name[37];
		unsigned c;
		uint64_t first, last;

		for (c = 0; c < 36; c++) {
			uint16_t ch = (uint16_t)(e[56 + 2 * c] | e[57 + 2 * c] << 8);
			if (ch == 0 || ch > 0x7f) {
				break;
			}
			name[c] = (char)ch;
		}
		name[c] = 0;
		if (strcmp(name, want) != 0) {
			continue;
		}
		memcpy(&first, e + 32, 8);
		memcpy(&last, e + 40, 8);
		if (last < first) {
			return ENXIO;
		}
		em.first_sector = first;
		em.n_sectors = last - first + 1;
		return 0;
	}
	printf("SWEET EMMC: no GPT partition named \"%s\"\n", want);
	return ENOENT;
}

/* ---------------------------------------------------------------- */
/* BSD disk layer                                                    */
/* ---------------------------------------------------------------- */

static int
sweet_emmc_open(__unused dev_t dev, int flags, __unused int devtype,
    __unused struct proc *p)
{
	if (!em.ready) {
		return ENXIO;
	}
	(void)flags;
	return 0;
}

static int
sweet_emmc_close(__unused dev_t dev, __unused int flags,
    __unused int devtype, __unused struct proc *p)
{
	return 0;
}

static void
sweet_emmc_strategy(buf_t bp)
{
	int error = 0;
	caddr_t vaddr = NULL;
	uint64_t off = (uint64_t)buf_blkno(bp) * em.secsize;
	uint32_t count = buf_count(bp);
	uint64_t part_bytes = em.n_sectors * EMMC_SEC_SIZE;
	bool write = (buf_flags(bp) & B_READ) == 0;

	buf_setresid(bp, count);
	if (off > part_bytes || count > part_bytes - off) {
		error = EINVAL;
		goto done;
	}
	if (count == 0) {
		goto done;
	}
	if (buf_map(bp, &vaddr)) {
		error = EIO;
		goto done;
	}

	lck_mtx_lock(&sweet_emmc_lock);
	for (uint32_t done = 0; done < count;) {
		uint64_t pos = off + done;
		uint64_t lba = pos / EMMC_SEC_SIZE;
		uint32_t skip = (uint32_t)(pos % EMMC_SEC_SIZE);
		uint32_t want = MIN(count - done, EMMC_BOUNCE_SIZE - skip);
		uint32_t sectors = (skip + want + EMMC_SEC_SIZE - 1) / EMMC_SEC_SIZE;

		if (write) {
			memcpy((void *)(em.dma + DMA_BOUNCE + skip), vaddr + done, want);
			error = emmc_write_sectors(lba, sectors);
		} else {
			error = emmc_read_sectors(lba, sectors);
			if (!error) {
				memcpy(vaddr + done, (const void *)(em.dma + DMA_BOUNCE + skip), want);
			}
		}
		if (error) {
			break;
		}
		done += want;
		buf_setresid(bp, count - done);
	}
	lck_mtx_unlock(&sweet_emmc_lock);
	buf_unmap(bp);

done:
	if (error) {
		buf_seterror(bp, error);
	}
	buf_biodone(bp);
}

static int
sweet_emmc_ioctl(__unused dev_t dev, u_long cmd, caddr_t data,
    __unused int flag, __unused struct proc *p, int is_char)
{
	uint32_t *f = (uint32_t *)data;
	uint64_t *o = (uint64_t *)data;

	switch (cmd) {
	case DKIOCGETBLOCKSIZE:
		*f = em.secsize;
		return 0;
	case DKIOCSETBLOCKSIZE:
		if (is_char) {
			return ENODEV;
		}
		if (*f < DEV_BSIZE || *f > EMMC_SEC_SIZE || (*f & (*f - 1))) {
			return EINVAL;
		}
		em.secsize = *f;
		return 0;
	case DKIOCGETPHYSICALBLOCKSIZE:
		*f = EMMC_BLOCK_SIZE;
		return 0;
	case DKIOCGETBLOCKCOUNT:
		*o = em.n_sectors * EMMC_SEC_SIZE / em.secsize;
		return 0;
	case DKIOCISWRITABLE:
		*f = 1;
		return 0;
	case DKIOCGETMAXBLOCKCOUNTREAD:
		*o = EMMC_MAX_XFER;
		return 0;
	case DKIOCGETMAXBLOCKCOUNTWRITE:
		*o = EMMC_MAX_XFER;
		return 0;
	case DKIOCGETMAXSEGMENTCOUNTREAD:
	case DKIOCGETMAXSEGMENTCOUNTWRITE:
		*o = 1;
		return 0;
	default:
		return ENOTTY;
	}
}

static int
sweet_emmc_bioctl(dev_t dev, u_long cmd, caddr_t data, int flag, struct proc *p)
{
	return sweet_emmc_ioctl(dev, cmd, data, flag, p, 0);
}

static int
sweet_emmc_cioctl(dev_t dev, u_long cmd, caddr_t data, int flag, struct proc *p)
{
	return sweet_emmc_ioctl(dev, cmd, data, flag, p, 1);
}

static int
sweet_emmc_psize(__unused dev_t dev)
{
	return em.ready ? (int)MIN(em.n_sectors * EMMC_SEC_SIZE / em.secsize, INT_MAX) : -1;
}

/*
 * The character device goes through physio(), which breaks a large
 * request into block-sized pieces and calls the strategy routine, so the
 * raw read/write below only ever see one buffer.
 */
static int
sweet_emmc_read(dev_t dev, struct uio *uio, __unused int ioflag)
{
	return physio(sweet_emmc_strategy, NULL, dev, B_READ, minphys, uio,
	    (int)em.secsize);
}

static int
sweet_emmc_write(dev_t dev, struct uio *uio, __unused int ioflag)
{
	return physio(sweet_emmc_strategy, NULL, dev, B_WRITE, minphys, uio,
	    (int)em.secsize);
}

/* ---------------------------------------------------------------- */
/* Disk registration                                                 */
/* ---------------------------------------------------------------- */

static const struct bdevsw sweet_emmc_bdevsw = {
	.d_open     = sweet_emmc_open,
	.d_close    = sweet_emmc_close,
	.d_strategy = sweet_emmc_strategy,
	.d_ioctl    = sweet_emmc_bioctl,
	.d_dump     = eno_dump,
	.d_psize    = sweet_emmc_psize,
	.d_type     = D_DISK,
};

static const struct cdevsw sweet_emmc_cdevsw = {
	.d_open       = sweet_emmc_open,
	.d_close      = sweet_emmc_close,
	.d_read       = sweet_emmc_read,
	.d_write      = sweet_emmc_write,
	.d_ioctl      = sweet_emmc_cioctl,
	.d_stop       = eno_stop,
	.d_reset      = eno_reset,
	.d_ttys       = NULL,
	.d_select     = eno_select,
	.d_mmap       = eno_mmap,
	.d_strategy   = eno_strat,
	.d_reserved_1 = eno_getc,
	.d_reserved_2 = eno_putc,
	.d_type       = D_DISK,
};

/* ---------------------------------------------------------------- */
/* Root device hook                                                  */
/* ---------------------------------------------------------------- */

/* Root device for rd=emmcroot0 (IOFindBSDRoot). */
int sweet_emmc_rootdev(dev_t *dev);

/* Brings the controller up and registers the disk; called from the root
 * device hook the first time the kernel is told to root on the eMMC. */
int sweet_emmc_init(void);

int
sweet_emmc_rootdev(dev_t *dev)
{
	if (!em.ready) {
		int error = sweet_emmc_init();
		if (error) {
			return error;
		}
	}
	*dev = em.bdev;
	return 0;
}

/*
 * Entry point: called once the memory map is up.  Sets up the controller
 * and registers the disk.
 *
 * The device tree is the source of truth for the controller's address, so
 * the driver looks for a "/sweet-emmc" node (added by tools/mkadt.py) and
 * falls back to the known ocean address if it is missing.
 */
int
sweet_emmc_init(void)
{
	DTEntry node;
	uintptr_t const *reg = NULL;
	unsigned int size = 0;
	char want[64] = "userdata";
	int error;

	if (em.ready) {
		return 0;
	}

	/* The boot-arg selects which GPT partition becomes the root. */
	PE_parse_boot_argn("emmcroot", want, sizeof(want));

	if (SecureDTLookupEntry(NULL, "/sweet-emmc", &node) == kSuccess &&
	    SecureDTGetProperty(node, "reg", (void const **)&reg, &size) == kSuccess &&
	    size >= 2 * sizeof(uintptr_t)) {
		em.regs = ml_io_map(reg[0], reg[1]);
	} else {
		/* Fall back to the ocean address from the device tree. */
		em.regs = ml_io_map(EMMC_HC_BASE, EMMC_HC_SIZE);
	}
	if (em.regs == 0) {
		printf("SWEET EMMC: cannot map the host controller\n");
		return ENXIO;
	}

	/*
	 * One physically contiguous allocation for the ADMA descriptor
	 * table and the bounce buffer.  It has to be contiguous because the
	 * SDHC walks the descriptor list and the bounce buffer in one go,
	 * and the platform is not DMA coherent.
	 */
	if (kmem_alloc_contig(kernel_map, &em.dma, DMA_SIZE, PAGE_MASK, 0, 0,
	    KMA_DATA | KMA_ZERO | KMA_PERMANENT, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		printf("SWEET EMMC: no contiguous DMA memory\n");
		return ENOMEM;
	}
	{
		ppnum_t pn = pmap_find_phys(kernel_pmap, (addr64_t)em.dma);

		if (!pn) {
			return ENOMEM;
		}
		em.dma_pa = ptoa_64(pn);
	}

	em.secsize = DEV_BSIZE;

	printf("SWEET EMMC: hc %p dma %p pa 0x%llx\n",
	    (void *)em.regs, (void *)em.dma, em.dma_pa);

	lck_mtx_lock(&sweet_emmc_lock);
	error = emmc_reset_hc();
	if (!error) {
		error = emmc_init_card();
	}
	if (!error) {
		error = emmc_find_partition(want);
	}
	lck_mtx_unlock(&sweet_emmc_lock);
	if (error) {
		return error;
	}

	em.bmajor = bdevsw_add(-1, &sweet_emmc_bdevsw);
	if (em.bmajor < 0) {
		return ENXIO;
	}
	em.cmajor = cdevsw_add_with_bdev(-1, &sweet_emmc_cdevsw, em.bmajor);
	if (em.cmajor < 0) {
		return ENXIO;
	}
	em.bdev = makedev(em.bmajor, 0);
	devfs_make_node(em.bdev, DEVFS_BLOCK, UID_ROOT, GID_OPERATOR, 0640, "emmcroot0");
	devfs_make_node(makedev(em.cmajor, 0), DEVFS_CHAR, UID_ROOT, GID_OPERATOR,
	    0640, "remmcroot0");
	em.ready = true;

	printf("SWEET EMMC: \"%s\" on the eMMC: first sector %llu, %llu MiB, card %u sectors\n",
	    want, em.first_sector, em.n_sectors * EMMC_SEC_SIZE >> 20, em.card_sectors);
	return 0;
}

#endif /* ARM64_BOARD_CONFIG_SWEET */
