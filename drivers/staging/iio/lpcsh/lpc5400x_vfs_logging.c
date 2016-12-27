/* #define DEBUG */
#include <linux/errno.h>
#include <linux/kfifo.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/lpcsh/lpc5400x.h>
#include "hostif_protocol.h"

#include "lpc5400x_vfs.h"
#include "lpc5400x_vfs_logging.h"

int lpc5400x_send_buf(struct i2c_client *client, u8 *buffer, int length);
//void lpc5400x_lock_i2c_client(struct i2c_client *client);
//void lpc5400x_unlock_i2c_client(struct i2c_client *client);

struct _lpc5400x_vfs_logging_pri {
	struct kfifo datafifo;
	int datafifolen;
	struct mutex logging_mutex;	/* logging_mutex protecting buffer */
	struct device *dev_weakref;
	struct lpc5400x_platform_data *platform_data_weakref;
	bool eventLost;
};

void lpc5400x_vfs_logging_save_events(
	void *vfsP,
	const void *pBuf, int size)
{
	struct _lpc5400x_vfs *vfs = (struct _lpc5400x_vfs *)vfsP;
	int n;
	uint16_t header;
	struct _lpc5400x_vfs_logging_pri *pri =
		(struct _lpc5400x_vfs_logging_pri *)vfs->devices[SENSORHUB_LOGGING]->private;
	struct _lpc5400x_vfs_devices *vfs_device =
		dev_get_drvdata(pri->dev_weakref);

	mutex_lock(&pri->logging_mutex);

	n = kfifo_avail(&pri->datafifo);

	if ((pri->eventLost) && (n >= sizeof(header))) {
		header = PACK_LOG_SERV_HEADER(0, 0, 0, false, false);
		kfifo_in(&pri->datafifo, &header, sizeof(header));
		pri->eventLost = false;
		pr_err("kFifo full header sent");

		n -= sizeof(header);
	}

	if (n < size) {
		if (n >= sizeof(header)) {
			header = PACK_LOG_SERV_HEADER(0, 0, 0, false, false);
			kfifo_in(&pri->datafifo, &header, sizeof(header));
			pr_err("kFifo full header sent");
		} else
			pri->eventLost = true;

		pr_err("kFifo full avail [%d] needs [%d]", n, size);
	} else {
        pr_err("kFifo avail = %d, sizein = %d\n", n, size);

		n = kfifo_in(&pri->datafifo, pBuf, size);

		n = kfifo_avail(&pri->datafifo);
        pr_err("kFifo avail after [%d]\n", n);
	}
	mutex_unlock(&pri->logging_mutex);

	vfs_device->busy = false;
}



int lpc5400x_vfs_logging_prepare(struct device *dev, void **pPrivate)
{
	struct _lpc5400x_vfs_logging_pri *p =
		(struct _lpc5400x_vfs_logging_pri *)
		kmalloc(sizeof(struct _lpc5400x_vfs_logging_pri), GFP_KERNEL);
	struct _lpc5400x_vfs_devices *vfs_device = dev_get_drvdata(dev);
	int ret;

	if (!p) {
		*pPrivate = NULL;
		return -ENOMEM;
	}
	p->eventLost = false;
	p->datafifolen = LPC5400X_MAX_I2C_RX_PAYLOAD_SIZE;
	ret = kfifo_alloc(&p->datafifo, p->datafifolen, GFP_KERNEL);
	if (ret) {
		pr_warn("lpc5400x_vfs_logging_prepare kfifo allocation failed...");
		kfree(p);
		return -ENOMEM;
	}
	/*
	    I assume I don't need a wake lock here,
	    as logging runs in ADB and USB is connected
	 */
	p->dev_weakref = dev;
	p->platform_data_weakref = vfs_device->pPlatformData;

	mutex_init(&p->logging_mutex);

	*pPrivate = p;

	pr_debug("lpc5400x_vfs_logging_prepare complete\n");
	return 0;
}

int lpc5400x_vfs_logging_write_hdl(struct device *dev,
								   void *pBuf, int size, void *pPrivate)
{
	struct _lpc5400x_vfs_logging_pri *pRdPri =
		(struct _lpc5400x_vfs_logging_pri *)pPrivate;

	struct _lpc5400x_vfs_devices *vfs_device =
		dev_get_drvdata(pRdPri->dev_weakref);

	int ret, n;
	struct i2c_client *client;

	switch (*((uint8_t *)pBuf)) {
	case LPCSH_CMD_LOG_SERV_CONFIGURE:
	{
		struct lpcsh_logServConfigure_cmd_t *pConfigCommand =
			(struct lpcsh_logServConfigure_cmd_t *)pBuf;		/* Logging parameters */

		pr_debug("Logging Enable Command %d, bufferSize [%d], logLevel[%d]",
			pConfigCommand->command, pConfigCommand->bufferSize, pConfigCommand->logLevel);

		if ((int)pConfigCommand->bufferSize > pRdPri->datafifolen) {
			kfifo_free(&pRdPri->datafifo);
			pRdPri->datafifolen = 0;
			ret = kfifo_alloc(
				&pRdPri->datafifo,
				roundup_pow_of_two(pConfigCommand->bufferSize),
				GFP_KERNEL);

			if (ret < 0) {
				pr_warn("kfifo_alloc error %d", ret);
				return ret;
			}

			pRdPri->datafifolen =
				roundup_pow_of_two(pConfigCommand->bufferSize);

			n = kfifo_avail(&pRdPri->datafifo);

			pr_debug("Reallocate DATA FIFO size = %d, avail = %d", pRdPri->datafifolen, n);
		}
		kfifo_reset(&pRdPri->datafifo);
	}
		client = i2c_verify_client(dev);

		//lpc5400x_lock_i2c_client(client);
		ret = lpc5400x_send_buf(client, (uint8_t *)pBuf, size);
		//lpc5400x_unlock_i2c_client(client);

		if (ret < 0)
			ret = 0;
		else
			ret = sizeof(struct lpcsh_logServConfigure_cmd_t);
		break;
	case LPCSH_CMD_LOG_SERV_CONTROL:
	{
		struct lpcsh_logServEnable_cmd_t *pEnableCommand =
			(struct lpcsh_logServEnable_cmd_t *)pBuf;	/* Logging parameters */

		pr_debug("Logging Enable Command %d, moduleId [%d], logLevel[%d]",
			pEnableCommand->command, pEnableCommand->moduleId, pEnableCommand->logLevel);

		client = i2c_verify_client(dev);

		//lpc5400x_lock_i2c_client(client);
		ret = lpc5400x_send_buf(client, (uint8_t *)pBuf, size);
		//lpc5400x_unlock_i2c_client(client);

		if (ret < 0)
			ret = 0;
		else
			ret = sizeof(struct lpcsh_logServEnable_cmd_t);
	}
	break;
	default:
		ret = 0;
	}

	vfs_device->busy = false;

	return ret;
}

int lpc5400x_vfs_logging_read_hdl(
	struct device *dev,
	void *pBuf,
	int size,
	void *pPrivate)
{
	int ret, readsize;
	struct _lpc5400x_vfs_logging_pri *pRdPri = (struct _lpc5400x_vfs_logging_pri *)pPrivate;

	mutex_lock(&pRdPri->logging_mutex);

	ret = kfifo_to_user(&pRdPri->datafifo, pBuf, size, &readsize);
	if (ret < 0) {
		pr_err("Read VFS data fifo error, ret = %d\n", ret);
		mutex_unlock(&pRdPri->logging_mutex);
		return ret;
	}
	mutex_unlock(&pRdPri->logging_mutex);

    pr_debug(
        "lpc5400x_vfs_read...	logging data ret = %d, size = %d, readsize %d\n",
        ret,
        size,
        readsize);

	return readsize;
}


int lpc5400x_vfs_logging_poll_hdl(struct device *dev, void *pPrivate)
{
	struct _lpc5400x_vfs_logging_pri *pRdPri = (struct _lpc5400x_vfs_logging_pri *)pPrivate;
	int n;

	mutex_lock(&pRdPri->logging_mutex);
	n = kfifo_len(&pRdPri->datafifo);
	mutex_unlock(&pRdPri->logging_mutex);

	return n;
}

void lpc5400x_vfs_logging_clean_up(void *pPrivate)
{
	struct _lpc5400x_vfs_logging_pri *pRdPri = (struct _lpc5400x_vfs_logging_pri *)pPrivate;

	mutex_destroy(&pRdPri->logging_mutex);
	kfree(pPrivate);
}

