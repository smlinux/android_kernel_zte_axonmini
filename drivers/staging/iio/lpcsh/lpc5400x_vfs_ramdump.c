/* #define DEBUG */
#include <linux/errno.h>
#include <linux/kfifo.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/lpcsh/lpc5400x.h>

#include "lpc5400x_vfs.h"
#include "lpc5400x_vfs_ramdump.h"
#include "lpc5400x_ramdump.h"

enum Sesnorhub_VFS_RamDump_Cmd {
	SENSORHUB_RAMDUMP_SUMMARY,
	SENSORHUB_RAMDUMP_REGISTERS,
	SENSORHUB_RAMDUMP_STACK,
	SENSORHUB_RAMDUMP_MEMORY,
};

struct _lpc5400x_RamDump_Param {
	uint32_t command;
	uint32_t startAddr;
	uint32_t size;
};

struct _lpc5400x_vfs_ramdump_pri {
	struct kfifo datafifo;
	int datafifolen;
	char *pRxBuf;
	struct work_struct work;
	struct mutex mutex;	/* mutex protecting buffer */
	struct _lpc5400x_RamDump_Param param;
	struct device *dev_weakref;
	struct lpc5400x_platform_data *platform_data_weakref;
};

void lpc5400x_vfs_ramdump_readbus(struct work_struct *work)
{
	struct _lpc5400x_vfs_ramdump_pri *pri =
		container_of(work, struct _lpc5400x_vfs_ramdump_pri, work);
	struct _lpc5400x_vfs_devices *vfs_device = dev_get_drvdata(pri->dev_weakref);

	int maxreadsize = 0, rxpos = 0, ret, n;
	bool done = false;

	while (!done) {
		maxreadsize = LPC5400X_MAX_I2C_RX_TRANSFER_SIZE;

		if ((int)pri->param.size > 0)
			if (maxreadsize > pri->param.size - rxpos + RAMDUMP_RESP_HEADER_SIZE)
				maxreadsize = pri->param.size - rxpos + RAMDUMP_RESP_HEADER_SIZE;

		switch (pri->param.command) {
		case SENSORHUB_RAMDUMP_SUMMARY:
			ret = lpc5400x_dumpLunchMenu(vfs_device->parent->dev,
				pri->pRxBuf,
				maxreadsize,
				pri->platform_data_weakref->gpio);
			done = true;
			break;
		case SENSORHUB_RAMDUMP_REGISTERS:
			ret = lpc5400x_dumpCPUContext(vfs_device->parent->dev,
				pri->pRxBuf,
				maxreadsize,
				pri->platform_data_weakref->gpio);
			done = true;
			break;
		case SENSORHUB_RAMDUMP_STACK:
			if (pri->param.size <= 0) {
				ret = -EINVAL;
				break;
			}
			ret = lpc5400x_dumpStack(
				vfs_device->parent->dev,
				pri->pRxBuf,
				maxreadsize,
				pri->param.startAddr + rxpos,
				pri->platform_data_weakref->gpio);
			break;
		case SENSORHUB_RAMDUMP_MEMORY:
			if (pri->param.size <= 0) {
				ret = -EINVAL;
				break;
			}
			ret = lpc5400x_dumpMemory(
				vfs_device->parent->dev,
				pri->pRxBuf,
				maxreadsize,
				pri->param.startAddr + rxpos,
				pri->platform_data_weakref->gpio);
			break;
		default:
			pr_debug("Ramdump Unknown command");
			ret = -EINVAL;
			break;
		}
		if (ret > 0) {
			mutex_lock(&pri->mutex);

			n = kfifo_avail(&pri->datafifo);
/*			pr_debug("kFifo avail = %d, sizein = %d", n, ret - 4); */

			n = kfifo_in(&pri->datafifo, pri->pRxBuf + 4, ret - 4);

			mutex_unlock(&pri->mutex);

			rxpos += (ret - 4);
/*			pr_debug("rxpos = %d, ret = %d, n = %d", rxpos, ret, n); */

			if (rxpos >= (int)pri->param.size)
				done = true;
		} else
			break;
	}

	vfs_device->busy = false;

	pr_debug("lpc5400x_vfs_ramdump_readbus complete\n");
}



int lpc5400x_vfs_ramdump_prepare(struct device *dev, void **pPrivate)
{
	struct _lpc5400x_vfs_ramdump_pri *p =
		(struct _lpc5400x_vfs_ramdump_pri *)
		kmalloc(sizeof(struct _lpc5400x_vfs_ramdump_pri), GFP_KERNEL);
	struct _lpc5400x_vfs_devices *vfs_device = dev_get_drvdata(dev);
	int ret;

	if (!p) {
		*pPrivate = NULL;
		return -ENOMEM;
	}

	p->pRxBuf = kmalloc(LPC5400X_MAX_I2C_RX_TRANSFER_SIZE, GFP_KERNEL);
	if (p->pRxBuf == NULL) {
		kfree(p);
		*pPrivate = NULL;
		return -ENOMEM;
	}

	p->datafifolen = LPC5400X_MAX_I2C_RX_PAYLOAD_SIZE;
	ret = kfifo_alloc(&p->datafifo, p->datafifolen, GFP_KERNEL);
	if (ret) {
		pr_warn("lpc5400x_vfs_ramdump_prepare kfifo allocation failed...");
		kfree(p->pRxBuf);
		kfree(p);
		return -ENOMEM;
	}
	/*
	    I assume I don't need a wake lock here,
	    as ramdump runs in ADB and USB is connected
	 */
	p->dev_weakref = dev;
	p->platform_data_weakref = vfs_device->pPlatformData;

	INIT_WORK(&p->work, lpc5400x_vfs_ramdump_readbus);

	mutex_init(&p->mutex);

	*pPrivate = p;

	pr_debug("lpc5400x_vfs_ramdump_prepare complete\n");
	return 0;
}

int lpc5400x_vfs_ramdump_write_hdl(struct device *dev,
								   void *pBuf, int size, void *pPrivate)
{
	struct _lpc5400x_RamDump_Param *pRdParam =
		(struct _lpc5400x_RamDump_Param *)pBuf;	/* Ramdump parameters */

	struct _lpc5400x_vfs_ramdump_pri *pRdPri =
		(struct _lpc5400x_vfs_ramdump_pri *)pPrivate;

/*	struct _lpc5400x_vfs_devices *vfs_device = dev_get_drvdata(dev); */
/*	struct lpc5400x_platform_data * platformData = vfs_device->pPlatformData; */

	int ret, n;

	pr_debug("Ramdump Command %d, %d, %d", pRdParam->command, pRdParam->startAddr, pRdParam->size);

	if ((int)pRdParam->size > pRdPri->datafifolen) {
		kfifo_free(&pRdPri->datafifo);
		pRdPri->datafifolen = 0;
		ret = kfifo_alloc(
			&pRdPri->datafifo,
			roundup_pow_of_two(pRdParam->size),
			GFP_KERNEL);
		if (ret < 0) {
			pr_warn("kfifo_alloc error %d", ret);
			return ret;
		}
		pRdPri->datafifolen = roundup_pow_of_two(pRdParam->size);
		n = kfifo_avail(&pRdPri->datafifo);

		pr_debug(
			"Reallocate DATA FIFO size = %d, avail = %d\n",
			pRdPri->datafifolen,
			n);
	}
	kfifo_reset(&pRdPri->datafifo);

	pRdPri->param.size = pRdParam->size;
	pRdPri->param.startAddr = pRdParam->startAddr;
	pRdPri->param.command = pRdParam->command;

	schedule_work(&pRdPri->work);

	return sizeof(struct _lpc5400x_RamDump_Param);
}

int lpc5400x_vfs_ramdump_read_hdl(struct device *dev,
								  void *pBuf, int size, void *pPrivate)
{
	int ret, readsize;
	struct _lpc5400x_vfs_ramdump_pri *pRdPri =
		(struct _lpc5400x_vfs_ramdump_pri *)pPrivate;

	mutex_lock(&pRdPri->mutex);

	ret = kfifo_to_user(&pRdPri->datafifo, pBuf, size, &readsize);
	if (ret < 0) {
		pr_debug("Read VFS data fifo error, ret = %d", ret);
		return ret;
	}
	mutex_unlock(&pRdPri->mutex);


	pr_debug(
		"lpc5400x_vfs_read...	ramdump data ret = %d, size = %d, readsize %d\n",
		ret,
		size,
		readsize);

	return readsize;
}


int lpc5400x_vfs_ramdump_poll_hdl(struct device *dev, void *pPrivate)
{
	struct _lpc5400x_vfs_ramdump_pri *pRdPri =
		(struct _lpc5400x_vfs_ramdump_pri *)pPrivate;
	int n;

	mutex_lock(&pRdPri->mutex);
	n = kfifo_len(&pRdPri->datafifo);
	mutex_unlock(&pRdPri->mutex);

	return n;
}

void lpc5400x_vfs_ramdump_clean_up(void *pPrivate)
{
	struct _lpc5400x_vfs_ramdump_pri *pRdPri =
		(struct _lpc5400x_vfs_ramdump_pri *)pPrivate;

	mutex_destroy(&pRdPri->mutex);
	kfree(pRdPri->pRxBuf);
	kfree(pPrivate);
}

