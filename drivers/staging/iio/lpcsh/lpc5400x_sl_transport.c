#define DEBUG
#include <linux/crc32.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include "sl_protocol.h"

#define SENCORD_BOOTLOADER_I2C_ADDR   (0x18)

int SLBoot_xfer(struct device *dev, u8 *txbuffer, int txlength,
				u8 *rxbuffer, int rxlength)
{
	struct i2c_msg msg[2];
	int num = 0;
	struct i2c_client *client = i2c_verify_client(dev);

	if (txbuffer && txlength > 0) {
		#ifdef SENCORD_BOOTLOADER_I2C_ADDR
		msg[num].addr = SENCORD_BOOTLOADER_I2C_ADDR;
		#else
		msg[num].addr = client->addr;
		#endif
		msg[num].flags = 0;
		msg[num].len = txlength;
		msg[num].buf = txbuffer;
		num++;
	}

	if (rxbuffer && rxlength > 0) {
		#ifdef SENCORD_BOOTLOADER_I2C_ADDR
		msg[num].addr = SENCORD_BOOTLOADER_I2C_ADDR;
		#else
		msg[num].addr = client->addr;
		#endif
		msg[num].flags = I2C_M_RD;
		msg[num].len = rxlength;
		msg[num].buf = rxbuffer;
		num++;
	}

	return i2c_transfer(client->adapter, msg, num);
}

int SLBoot_send_cmd(struct device *dev, u8 *txbuffer,
					int txlength, u8 *rxbuffer, int rxlength, int gpio)
{
	unsigned long orig_jiffies;
	int ret = 0, val;
	struct _CmdResponse_t *resp;

	resp = (struct _CmdResponse_t *)rxbuffer;

	if (!txbuffer || !rxbuffer) {
		dev_warn(dev, "SLBoot_send_cmd, wrong parameter\n");
		return -EINVAL;
	}

	if (txbuffer[0] != SH_CMD_WRITE_BLOCK && txbuffer[0] != SH_CMD_WRITE_SUB_BLOCK){
	//dev_info(dev, "SLBoot_send_cmd: 0x%x\n", txbuffer[0]);
	}
	
	val = gpio_get_value_cansleep(gpio);
	dev_dbg(dev, "SLBoot_send_cmd 0x%x GPIO(%d) Value %d txlength:%d before sending\n", txbuffer[0], gpio, val, txlength);

	ret = SLBoot_xfer(dev, txbuffer, txlength, NULL, 0);
	if (ret < 0) {
		dev_err(dev, "SLBoot_send_cmd bus error %d\n", ret);
		return ret;
	}

	if (txbuffer[0] == SH_CMD_BOOT)
		return 0;

	orig_jiffies = jiffies;

	val = gpio_get_value_cansleep(gpio);
	//dev_dbg(dev, "SLBoot_send_cmd GPIO(%d) Value %d\n", gpio, val);

	while (val == 1) {
		if (time_after(jiffies,
				orig_jiffies +
				msecs_to_jiffies(BUS_COMMAND_TIMEOUT_MS))) {
			dev_warn(dev, "SLBoot_send_cmd, timeout\n");
			ret = -ETIMEDOUT;
			return ret;
		}

		val = gpio_get_value_cansleep(gpio);
	}

	ret = SLBoot_xfer(dev, NULL, 0, rxbuffer, rxlength);
	if (ret < 0) {
		dev_err(dev, "SLBoot_send_cmd bus error %d\n", ret);
		return ret;
	}

	dev_dbg(dev, "SL Response 0x%x, 0x%x, length: %d\n", resp->cmd, resp->sop, resp->length);

	return 0;
}


