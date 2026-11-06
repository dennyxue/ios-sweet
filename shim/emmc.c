/*
 * OCEAN (Smartisan Nut Pro 2S, SDM710) eMMC probe for the boot shim.
 *
 * The shim runs before XNU exists and its only job here is to find out
 * what state the bootloader left the SDHC host controller in, and to
 * print it, so the XNU driver can decide whether it has to do a full
 * card bring-up.
 *
 * This mirrors shim/ufs.c, which does the same for the UFS host on the
 * sweet board: both drivers run with the MMU off (all memory Device, so
 * DMA is coherent without cache maintenance) and only read registers.
 *
 * Register layout: Qualcomm SDHCI ("sdhci-msm-v5"), the same window the
 * XNU driver uses.
 *
 *   sdhc_1: sdhci@7c4000 {
 *           compatible = "qcom,sdhci-msm-v5";
 *           reg = <0x7C4000 0x1000>, <0x7C5000 0x1000>;
 *           reg-names = "hc_mem", "cmdq_mem";
 *           interrupts = <0 641 0>, <0 644 0>;
 *           interrupt-names = "hc_irq", "pwr_irq";
 *           qcom,bus-width = <8>;
 *   };
 */

#include <stdint.h>
#include "console.h"

/* Host controller window. */
#define EMMC_BASE	0x07c4000UL

/* SDHCI registers. */
#define REG_SYSAD	0x00
#define REG_PRNSTS	0x08
#define REG_HOSTCTL	0x1c
#define REG_PWRCON	0x20
#define REG_CLK_CONTROL	0x24
#define REG_SWRST	0x28
#define REG_INTSTATUS	0x2c
#define REG_CAPABILITIES	0x34
#define REG_CAPABILITIES_1	0x38
#define REG_HOST_VERSION	0x44

/* Present state bits. */
#define PRNSTS_CMD_INHIBIT	0x0001
#define PRNSTS_DATA_INHIBIT	0x0002
#define PRNSTS_CLOCK_STABLE	0x0200
#define PRNSTS_DATA0_LEVEL	0x0800
#define PRNSTS_CMD_LINE_LEVEL	0x8000

/* Capabilities bits. */
#define CAP_8BIT		0x00040000
#define CAP_ADMA		0x00080000
#define CAP_64BIT		0x00100000
#define CAP_HS400		0x10000000

static inline uint8_t
rd8(uint32_t off)
{
	return *(volatile uint8_t *)(EMMC_BASE + off);
}

static inline uint16_t
rd16(uint32_t off)
{
	return *(volatile uint16_t *)(EMMC_BASE + off);
}

static inline uint32_t
rd32(uint32_t off)
{
	return *(volatile uint32_t *)(EMMC_BASE + off);
}

/*
 * Probe the host controller and report its state.
 *
 * We deliberately do not touch the controller here: the shim only reads,
 * because the XNU driver wants to see exactly what the bootloader left
 * behind (a host that still has the clock running and the bus powered is
 * in a much better starting position than one that was reset).
 */
void
emmc_probe(void)
{
	uint32_t prnsts, caps, caps1, ver, pwrc, clk;
	uint16_t rver;
	uint8_t hostctl;

	prnsts = rd32(REG_PRNSTS);
	caps = rd32(REG_CAPABILITIES);
	caps1 = rd32(REG_CAPABILITIES_1);
	ver = rd32(REG_HOST_VERSION);
	pwrc = rd32(REG_PWRCON);
	clk = rd32(REG_CLK_CONTROL);
	hostctl = rd8(REG_HOSTCTL);
	rver = rd16(REG_SYSAD);

	/*
	 * The vendor id at 0x00 tells us whether this really is an SDHC
	 * window and which generation it is.  On the SDM710 it reads back
	 * as the SDHCI specification version.
	 */
	con_hex2("EMMC", "ver=", (ver >> 16) & 0xff, "vendor=", rver);
	con_hex2("EMMC", "prnsts=", prnsts, "pwrc=", pwrc);
	con_hex2("EMMC", "clkctl=", clk, "hostctl=", hostctl);
	con_hex2("EMMC", "caps=", caps, "caps1=", caps1);

	/*
	 * A card that is present leaves DAT0 low (the card pulls the data
	 * line down) and the command line high.  That is the cheapest
	 * presence check available before any command is issued.
	 */
	con_str("EMMC", "card looks ");
	if ((prnsts & PRNSTS_DATA0_LEVEL) == 0) {
		con_str("", "PRESENT");
	} else {
		con_str("", "ABSENT");
	}
	con_str("EMMC", "cmd_inhibit=");
	con_hex("", (prnsts & PRNSTS_CMD_INHIBIT) ? 1 : 0);
	con_str("EMMC", "clk_stable=");
	con_hex("", (prnsts & PRNSTS_CLOCK_STABLE) ? 1 : 0);

	con_str("EMMC", "features:");
	if (caps & CAP_8BIT) {
		con_str("", " 8bit");
	}
	if (caps & CAP_ADMA) {
		con_str("", " adma");
	}
	if (caps & CAP_64BIT) {
		con_str("", " dma64");
	}
	if (caps1 & CAP_HS400) {
		con_str("", " hs400");
	}
	con_str("EMMC", "probe done");
}
