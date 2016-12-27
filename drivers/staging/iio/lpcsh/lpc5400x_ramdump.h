#ifndef LPC5400X_RAMDUMP_H_
#define LPC5400X_RAMDUMP_H_

#include <linux/types.h>
#include <linux/device.h>

#define RAMDUMP_RESP_HEADER_SIZE 4

int lpc5400x_dumpLunchMenu(
	struct device *dev,
	uint8_t *data,
	uint32_t maxsize,
	int gpio);

int lpc5400x_dumpCPUContext(
	struct device *dev,
	uint8_t *data,
	uint32_t maxsize,
	int gpio);

int lpc5400x_dumpStack(
	struct device *dev,
	uint8_t *data,
	uint32_t maxsize,
	uint32_t startAddr,
	int gpio);

int lpc5400x_dumpMemory(
	struct device *dev,
	uint8_t *data,
	uint32_t maxsize,
	uint32_t startAddr,
	int gpio);

int lpc5400x_dumpTestSample(struct device *dev, int gpio);

#endif	/* LPC5400X_RAMDUMP_H */
