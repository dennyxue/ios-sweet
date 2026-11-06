/*
 * OCEAN eMMC probe, shim side.
 */

#ifndef _OCEAN_EMMC_H_
#define _OCEAN_EMMC_H_

/*
 * Read-only probe of the SDM710 SDHC host controller.  Prints what the
 * bootloader left behind so the XNU driver knows whether it has to do a
 * full MMC card bring-up.
 */
void emmc_probe(void);

#endif /* _OCEAN_EMMC_H_ */
