/*  drivers/input/misc/lpc5400x_update.c
 *
 *  Copyright (C) 2014 NXP semiconductors
 *
 *  Helper functions parts of LPC Sensor Hub driver to update firmware.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#define DEBUG
#include <linux/crc32.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include "sl_protocol.h"
#include "lpc5400x_update.h"

static uint8_t update_sendbuf[SL_FLASH_BLOCK_SZ * 2];
bool lpc5400x_checkImage(struct device *dev, int gpio)
{
	uint32_t response[2];
	uint8_t cmd = SH_CMD_CHECK_IMAGE;
	int ret;
	struct _CmdResponse_t *pRhdr;

	ret = SLBoot_send_cmd(dev, &cmd, 1, (uint8_t *)&response, 8, gpio);

	dev_dbg(dev, "lpc5400x_checkImage: gpio = %d ret = %d, response: 0x%x, 0x%x\n",
		gpio, ret, response[0], response[1]);
	if (ret == 0) {
		pRhdr = (struct _CmdResponse_t *)&response[0];
		if ((pRhdr->sop == SH_RESP_HDR_SOP) && (pRhdr->cmd == cmd)
			&& (response[1] == 0))
			return true;
	}

	return false;
}

bool lpc5400x_bootFirmware(struct device *dev, int gpio)
{
	uint32_t response;
	uint8_t cmd = SH_CMD_BOOT;
	int ret;

	ret = SLBoot_send_cmd(dev, &cmd, 1, (uint8_t *)&response, 4, gpio);

	dev_dbg(dev, "lpc5400x_bootFirmware: gpio = %d ret = %d\n",
			gpio, ret);
	if (ret == 0)
		return true;

	return false;
}

int lpc5400x_writeSubBlock(struct device *dev, const u8 *data, size_t size, int blocknum, int gpio)
{
	struct _CmdRWSubblockParam_t *datablock =
		(struct _CmdRWSubblockParam_t *)&update_sendbuf;
	uint32_t response[2];
	unsigned long crc = 0 /* crc32(~0, NULL, 0) */;
	int ret;
	struct _CmdResponse_t *pRhdr;
	u8 *pU8 = (u8 *)&datablock->data[0];

	datablock->cmd = SH_CMD_WRITE_SUB_BLOCK;
#if 0
	memset(pU8, 0xFF, SL_MAX_I2C_READ_SIZE);
	memcpy(pU8, data, size);
#else
	memcpy(pU8, data, size);
	if(SL_MAX_I2C_READ_SIZE > size) {
		memset(pU8 + size, 0xFF, SL_MAX_I2C_READ_SIZE - size);
		pr_debug("Stuffing 0xFF %d, %d\n", pU8[0], pU8[SL_MAX_I2C_READ_SIZE - size - 1]);
	}
#endif

	//pr_debug("data2 %d, %d, %d, %d\n", data[0], data[1], data[2], data[3]);
	//pr_debug("data3 0x%x, size = %d\n", datablock->data[0], (int)size);

	datablock->crc_check = 0;	/* CRC is checked */
	datablock->block_page_nr =
		blocknum + SL_BOOTAPP_START_BLOCK;

	crc = crc32(0xFFFFFFFF,
		(unsigned char const *)datablock,
				SL_MAX_I2C_READ_SIZE + 4);
	crc ^= 0xFFFFFFFF;
	datablock->crc32 = crc;
	ret = SLBoot_send_cmd(dev, (uint8_t *)datablock,
		sizeof(struct _CmdRWSubblockParam_t),
		(uint8_t *)&response, 8, gpio);

	if (ret == 0) {
		pRhdr = (struct _CmdResponse_t *)&response[0];
		if ((pRhdr->sop == SH_RESP_HDR_SOP) && (pRhdr->cmd == SH_CMD_WRITE_SUB_BLOCK)
			&& (response[1] == 0))
			return 0;
		else
			return -response[1];
	}

	return ret;
}


int lpc5400x_updateFirmware(struct device *dev, int gpio)
{
	const struct firmware *fw_entry;
	size_t offset = 0;
	int ret, sendsize = 0, blocknum = 0, retry = 0, trannum = 0;

	dev_info(dev, "device name: %s\n", dev_name(dev));
	ret = request_firmware(&fw_entry, "fw_niobe_lpcsh.hex", dev);
	if (ret == 0) {
		dev_dbg(dev, "request firmware return %d\n", ret);
		dev_dbg(dev, "request_firmware firmwork size = %d\n",
			(int)fw_entry->size);
		dev_dbg(dev, "data: %X, %X, %X, %X\n",
			fw_entry->data[0], fw_entry->data[1],
			fw_entry->data[2], fw_entry->data[3]);
		dev_dbg(dev, "last_data: %X, %X, %X, %X\n",
			fw_entry->data[fw_entry->size - 4],
			fw_entry->data[fw_entry->size - 3],
			fw_entry->data[fw_entry->size - 2],
			fw_entry->data[fw_entry->size - 1]);

		while (offset < fw_entry->size) {
			sendsize = fw_entry->size - offset;
			if (sendsize > SL_MAX_I2C_READ_SIZE)
				sendsize = SL_MAX_I2C_READ_SIZE;

			if(trannum == SL_FLASH_BLOCK_SZ/SL_MAX_I2C_READ_SIZE) {
				trannum = 0;
				blocknum++;
			}

			ret = lpc5400x_writeSubBlock(dev, &fw_entry->data[offset], sendsize, blocknum, gpio);
			/* Need to check the ret and response from Niobe */
			if (ret < 0) {
				if (retry == FWUPDATE_TRANSFER_RETRY_MAX) {
					dev_warn(dev,
							"Failed to upgrade firmware, bus error\n");
					break;
				} else {
					retry++;
					continue;
				}
			}
			offset += sendsize;
			retry = 0;
			trannum++;
		}

		if (ret == 0) {
			if (lpc5400x_checkImage(dev, gpio))
				lpc5400x_bootFirmware(dev, gpio);
			else
				dev_warn(dev, "Check firmware image failed\n");
		}

		release_firmware(fw_entry);
	} else
		dev_warn(dev, "Failed to get firmware image from vendor path\n");

	return ret;
}

int lpc5400x_probebus(struct device *dev, enum _SL_IFSEL_T busport, int gpio)
{
	int val = 1;
	unsigned long orig_jiffies;
#if 0
	uint8_t bus_probe_cmd[8] = { 0xA5, /* auto detect command */
	/* hostIRQ_pin: P0_30 used as host IRQ line on LPCXpresso board */
	0x55, (0 << 5) | 18,
	PIN_SPI0_MISO_2, /* MISO_pin: P1_4 */
	PIN_SPI0_MOSI_1, /* MOSI_pin: P0_12 */
	PIN_SPI0_SEL0_3, /* SEL_pin: P0_14 */
	PIN_SPI1_MISO_2, /* SCK_pin: P1_3 */
	0x55, };
#else
	uint8_t bus_probe_cmd[8] = { 0xA5, /* auto detect command */
	/* hostIRQ_pin: P0_30 used as host IRQ line on LPCXpresso board */
	0x55, (0 << 5) | 20,
	PIN_SPI0_MISO_2, /* MISO_pin: P1_4 */
	PIN_SPI0_MOSI_1, /* MOSI_pin: P0_12 */
	PIN_SPI0_SEL0_3, /* SEL_pin: P0_14 */
	PIN_SPI1_MISO_2, /* SCK_pin: P1_3 */
	0x55, };
#endif
	uint8_t rx_buff[8];

	bus_probe_cmd[1] = busport;
	/* XOR pin information */
	bus_probe_cmd[7] =
		bus_probe_cmd[0] ^ bus_probe_cmd[1] ^ bus_probe_cmd[2]
		^ bus_probe_cmd[3] ^ bus_probe_cmd[4] ^ bus_probe_cmd[5]
		^ bus_probe_cmd[6];

	orig_jiffies = jiffies;

	val = gpio_get_value_cansleep(gpio);

	dev_dbg(dev,
			"Bus Probe: gpio_get_value_cansleep return %d\n", val);
	if (val == 0)
		return -EBUSY;

	while (val == 1) {
		/* send FW update command in loop */
		SLBoot_xfer(dev, &bus_probe_cmd[0], 8, 0, 0);
		msleep(20);
		if (time_after(jiffies,
				orig_jiffies +
				msecs_to_jiffies(BUS_PROBE_TIMEOUT_MS)))
			break;
		val = gpio_get_value_cansleep(gpio);
	}

	dev_dbg(dev,
			"Bus Probe: gpio_get_value_cansleep2 return %d\n", val);

	if (val == 0) {
		SLBoot_xfer(dev, NULL, 0, &rx_buff[0], 4);
		dev_dbg(dev, "Bus Probe response: 0x%x, 0x%x, 0x%x, 0x%x\n",
			rx_buff[0], rx_buff[1], rx_buff[2], rx_buff[3]);
	} else {
    	dev_err(dev,
    			"lpc5400x_probebus Bus Probe error return %d\n", val);	
	}

	return 0;
}

int lpc5400x_romVersion(struct device *dev, int gpio)
{
	uint8_t response[6];
	uint8_t cmd = SH_CMD_GET_VERSION;
	int ret;

	ret = SLBoot_send_cmd(dev, &cmd, 1, (uint8_t *)&response, 6, gpio);
	if (ret == 0) {
		printk("ROM Version: %d:%d", response[4], response[5]);
		return 0;
	}

	return ret;
}

static uint32_t secureResp[2];	/* Use static memeory to avoid side attack */
int lpc5400x_enable_secure(struct device *dev, const u8 *data, size_t size, int gpio)
{
	/* SH_CMD_ENABLE_SECURE */
	struct _CmdEnableSecureParam_t *param =
		(struct _CmdEnableSecureParam_t *)&update_sendbuf;
	unsigned long crc = 0 /* crc32(~0, NULL, 0) */;
	int ret;
	struct _CmdResponse_t *pRhdr;

	u8 *pU8 = (u8 *)&param->data[0];

	param->cmd = SH_CMD_ENABLE_SECURE;
	param->crc_check = 0;	/* CRC is checked */
	memcpy(pU8, data, SL_KEY_BYTES);

	pr_debug("key data2 %d, %d, %d, %d", data[0], data[1], data[2], data[3]);
	pr_debug("key data3 0x%x 0x%x, size = %d", param->data[0],
		param->data[SL_KEY_BYTES / 4 - 1], (int)size);


	crc = crc32(0xFFFFFFFF,
		(unsigned char const *)param,
		SL_KEY_BYTES + 4);
	crc ^= 0xFFFFFFFF;
	param->crc32 = crc;

	ret = SLBoot_send_cmd(dev, (u8 *)param, sizeof(struct _CmdEnableSecureParam_t),
		(u8 *)&secureResp[0], 8, gpio);
	if (ret < 0) {
		dev_dbg(dev, "lpc5400x_enable_secure bus error %d", ret);
		return ret;
	}

	pr_debug("secure 0x%x 0x%x", secureResp[0], secureResp[1]);

	if (ret == 0) {
		pRhdr = (struct _CmdResponse_t *)&secureResp[0];
		if ((pRhdr->sop == SH_RESP_HDR_SOP) && (pRhdr->cmd == SH_CMD_ENABLE_SECURE)
			&& (secureResp[1] == 0))
			return 0;
		else
			return -secureResp[1];
	}

	return ret;
}


int lpc5400x_disable_secure(struct device *dev, int gpio)
{
	int ret;
	u8 cmd = SH_CMD_DISABLE_SECURE;

	ret = SLBoot_send_cmd(dev, &cmd, 1, (u8 *)&secureResp[0], 4, gpio);
	if (ret < 0) {
		dev_dbg(dev, "lpc5400x_disable_secure bus error %d", ret);
		return ret;
	}

	return ret;
}
