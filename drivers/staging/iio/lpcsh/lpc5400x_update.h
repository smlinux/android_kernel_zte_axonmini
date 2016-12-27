#ifndef LPC5400X_FIRMWARE_UPDATE_H
#define LPC5400X_FIRMWARE_UPDATE_H

#include <linux/types.h>
#include <linux/device.h>

#define FWUPDATE_TRANSFER_RETRY_MAX 5

enum _SL_IFSEL_T {
	SL_AUTO = 0,	/*!< Auto-detect 2nd boot loader host interface */
	SL_I2C0,		/*!< I2C0 used for secondary loader host interface */
	SL_I2C1,		/*!< I2C1 used for secondary loader host interface */
	SL_I2C2,		/*!< I2C2 used for secondary loader host interface */
	SL_SPI0,		/*!< SPI0 used for secondary loader host interface */
	SL_SPI1			/*!< SPI1 used for secondary loader host interface */
};

int lpc5400x_updateFirmware(struct device *dev, int gpio);
int lpc5400x_probebus(struct device *dev, enum _SL_IFSEL_T busport, int gpio);
int lpc5400x_romVersion(struct device *dev, int gpio);
bool lpc5400x_checkImage(struct device *dev, int gpio);
bool lpc5400x_bootFirmware(struct device *dev, int gpio);
int lpc5400x_writeSubBlock(struct device *dev, const u8 *data, size_t size, int blocknum, int gpio);
int lpc5400x_enable_secure(struct device *dev, const u8 *data, size_t size, int gpio);
int lpc5400x_disable_secure(struct device *dev, int gpio);
#endif	/* LPC5400X_FIRMWARE_UPDATE_H */

