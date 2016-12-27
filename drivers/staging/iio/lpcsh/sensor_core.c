#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#define DEBUG		/* enable dev_dbg */
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/lpcsh/lpc5400x.h>
//#include "../buffer.h"
#include <linux/iio/buffer.h>
//#include "../iio.h"
#include <linux/iio/iio.h>

#include "sensor_core.h"

struct sensor_base {
	struct i2c_client *client;
	struct iio_dev *indio;
	struct mutex mutex;	/* reentrant protection for struct */
	char devicename[32];
	bool enabled;		/* P: mutex */
	u32 polldelay;
	unsigned model;
	unsigned int_mask;
	s64 lastsamplets;
	u8 sensorId;
	u8 samplebytes;

	struct mutex iiomutex;
};


int lpc_sensor_get_result_length(void *senbase)
{
	if (senbase)
		return ((struct sensor_base *)senbase)->samplebytes;

	return 0;
}

int lpc_sensor_configure_ring_length(
	int sensorId,
	struct iio_dev *indio_dev,
	size_t ringLength)
{
	int err = 0;

	indio_dev->buffer->access->set_length(
		indio_dev->buffer,
		ringLength);
	err = indio_dev->buffer->access->request_update(indio_dev->buffer);

	if (err < 0)
		pr_err(
			"lpc_sensor_configure_ring_length FAILED (%d) "
			"sensorId (%d) set_length(%d)\n",
			err,
			sensorId,
			(int)ringLength);
	else
		pr_info(
			"lpc_sensor_configure_ring_length sensorId (%d) set_length(%d)\n",
			sensorId,
			(int)ringLength);

	return err;
}


int lpc_sensor_configure_ring(
	struct iio_dev *indio_dev,
	int samplebytes,
	size_t ringLength)
{
	int err = 0;

	struct iio_buffer *ring;

#ifdef CONFIG_IIO_KFIFO_BUF
	ring = iio_kfifo_allocate(indio_dev);
	if (!ring)
		return -ENOMEM;
#else
	ring = iio_sw_rb_allocate(indio_dev);
	if (!ring)
		return -ENOMEM;
#endif

	indio_dev->buffer = ring;

	/* setup ring buffer */
	ring->scan_timestamp = true;
	ring->access->set_bytes_per_datum(ring, samplebytes);
	ring->access->set_length(ring, ringLength);
	err = ring->access->request_update(ring);

	/*scan count double count timestamp. should subtract 1. but
	   number of channels still includes timestamp*/
	indio_dev->modes |= INDIO_BUFFER_TRIGGERED;

	if (err < 0)
		pr_err(
			"lpc_sensor_configure_ring FAILED (%d) "
			"set_bytes_per_datum(%d) set_length(%d)\n",
			err,
			samplebytes,
			(int)ringLength);
/*			
	else
		pr_info(
			"lpc_sensor_configure_ring set_bytes_per_datum(%d) set_length(%d)\n",
			samplebytes,
			ringLength);
*/
	return err;
}

static void lpc_sensor_unconfigure_ring(struct iio_dev *indio_dev)
{
#ifdef CONFIG_IIO_KFIFO_BUF
	iio_kfifo_free(indio_dev->buffer);
#else
	iio_sw_rb_free(indio_dev->buffer);
#endif
}

static ssize_t sensor_base_enable_show(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct sensor_base *senbase = iio_priv(indio_dev);

	dev_dbg(dev, "sensor_base_enable_show type = %d, senbase->enabled %d\n",
		senbase->sensorId, senbase->enabled);

#if 1
	dev_dbg(dev, "sensor_base_enable_show type = %d, val = %u\n",
		senbase->sensorId, senbase->enabled ? 1 : 0);
#endif

	return sprintf(buf, "%d\n", senbase->enabled);
}



static ssize_t sensor_base_enable_store(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct sensor_base *senbase = iio_priv(indio_dev);

	unsigned long val;
	int err = 0;

	err = strict_strtoul(buf, 10, &val);
	if (err)
		return 0;

	dev_dbg(
		&senbase->client->dev,
		"sensor_base_enable_store type = %d, val = %lu\n",
		senbase->sensorId,
		val);

	if ((val) && (!senbase->enabled))
		dev_dbg(
			&senbase->client->dev,
			"*** sensor_base_enable_store to enable sensorId [%-2.2d]\n",
			senbase->sensorId);
	else
	if ((!val) && (senbase->enabled))
		dev_dbg(
			&senbase->client->dev,
			"*** sensor_base_enable_store to disable sensorId [%-2.2d]\n",
			senbase->sensorId);
	else
		dev_dbg(
			&senbase->client->dev,
			"*** sensor_base_enable_store unchanged state %s sensorId [%-2.2d]\n",
			senbase->enabled ? "enabled" : "disabled",
			senbase->sensorId);

	senbase->enabled = val ? true : false;
	err = lpc5400x_sensor_enable(
		senbase->client,
		senbase->sensorId,
		senbase->enabled);
	if (err < 0) {
		dev_err(
			&senbase->client->dev,
			"Sensor enable failed, err : %d",
			err);
		count = 0;
	}

	return count;
}

static DEVICE_ATTR(enable, 0666,
	sensor_base_enable_show,
	sensor_base_enable_store);

static ssize_t sensor_base_polldelay_show(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct sensor_base *senbase = iio_priv(indio_dev);

	dev_dbg(dev, "sensor_base_polldelay_show type = %d, polldelay %d\n",
		senbase->sensorId, senbase->polldelay);

	return sprintf(buf, "%d\n", senbase->polldelay);

}

static ssize_t sensor_base_polldelay_store(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct sensor_base *senbase = iio_priv(indio_dev);
	unsigned long val_us = 0;
	int err;

	dev_dbg(&senbase->client->dev, "sensor_base_polldelay_store\n");
	dev_dbg(&senbase->client->dev, "senbase = %p, type = %d, buf = %p, count = %d\n",
		senbase, senbase->sensorId, buf, (int)count);

	err = strict_strtoul(buf, 10, &val_us);
	dev_dbg(&senbase->client->dev, "value = %lu, err = %d\n", val_us, err);
	if (err)
		return 0;

	if (senbase->polldelay != val_us) {
		senbase->polldelay = val_us;
		err = lpc5400x_sensor_samplerate(senbase->client,
			senbase->sensorId, val_us);
		if (err < 0) {
			dev_err(
				&senbase->client->dev,
				"lpc5400x_sensor_samplerate failed,err = %d",
				err);
			count = 0;
		}
	}
	dev_dbg(
		&senbase->client->dev,
		"sensor_base_polldelay_store val %lu\n",
		val_us);

	return count;
}

static DEVICE_ATTR(
	polldelay,
	0666,
	sensor_base_polldelay_show,
	sensor_base_polldelay_store);


static ssize_t sensor_base_batch_store(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct sensor_base *senbase = iio_priv(indio_dev);
	int err;
	struct batchCommandParams *cmd = (struct batchCommandParams *)buf;

	if (count != sizeof(struct batchCommandParams)) {
		dev_err(
			dev,
			"invalid batch command size senbase = %p, type = %d, buf = %p, count = %d != %ld\n",
			senbase,
			senbase->sensorId,
			buf,
			(int)count,
			sizeof(struct batchCommandParams));
		return 0;
	}

	dev_dbg(
		&senbase->client->dev,
		"sensorId [%-2.2d] flag [%d] samplerate [%d] timeoutenc_us [%d] enable [%d -> %d] count [%d]\n",
		senbase->sensorId,
		cmd->flag,
		cmd->samplerate_us,
		cmd->timeoutenc_us,
		senbase->enabled,
		cmd->samplerate_us ? true : false,
		(int)count);

	senbase->polldelay = cmd->samplerate_us;

	err = lpc5400x_sensor_batch(
		senbase->client,
		senbase->sensorId,
		cmd->flag,
		cmd->samplerate_us,
		cmd->timeoutenc_us);

	if ((err >= 0)) {
#if 0
		if ((senbase != NULL) && (senbase->indio != NULL)) {
			size_t ringLength = DEFAULT_RINGBUFFERLENGTH;

			if ((cmd->samplerate_us > 0) &&
				(cmd->timeoutenc_us > cmd->samplerate_us)) {
				ringLength =
					(cmd->timeoutenc_us /
					 cmd->samplerate_us) +
					DEFAULT_RINGBUFFERLENGTH;

				if (ringLength > MAX_RINGBUFFERLENGTH)
					ringLength = MAX_RINGBUFFERLENGTH;
			}
			ringLength = MAX_RINGBUFFERLENGTH;
			err =  lpc_sensor_configure_ring_length(
				senbase->sensorId,
				senbase->indio,
				ringLength);
		}
#endif
	}

	return count;
}

static ssize_t sensor_base_batch_show(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	return 0;
}

static DEVICE_ATTR(batch, 0666,
	sensor_base_batch_show,
	sensor_base_batch_store);



static ssize_t sensor_base_flushfifo_store(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	int err;

	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct sensor_base *senbase = iio_priv(indio_dev);

	dev_dbg(
		&senbase->client->dev,
		"sensorId [%-2.2d] flushfifo_store\n",
		senbase->sensorId);

	err = flushfifo_request(senbase->client, senbase->sensorId);

	if (err < 0) {
		dev_err(
			&senbase->client->dev,
			"flushfifo_request failed,err = %d",
			err);
		count = 0;
	}

	return count;
}

static ssize_t sensor_base_flushfifo_show(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{

	return 0;
}

static DEVICE_ATTR(
	flushfifo,
	0666,
	sensor_base_flushfifo_show,
	sensor_base_flushfifo_store);

static struct attribute *sensor_base_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_polldelay.attr,
	&dev_attr_batch.attr,
	&dev_attr_flushfifo.attr,
	NULL
};

static const struct attribute_group sensor_base_attr_group = {
	.attrs	= sensor_base_attributes,
};

static const struct iio_info sensor_info = {
	.driver_module	= THIS_MODULE,
	.attrs			= &sensor_base_attr_group,
};

static const struct iio_chan_spec base_channels[] = {
	IIO_CHAN_SOFT_TIMESTAMP(0)
};

int lpc_sensor_register(
	void **pSenbase,
	struct i2c_client *client,
	struct sensor_config *sencfg)
{
	struct iio_dev *indio_dev;
	int result;
	struct sensor_base *senbase = NULL;

	//indio_dev = iio_allocate_device(sizeof(struct sensor_base));
    indio_dev =  iio_device_alloc(sizeof(struct sensor_base));
	if (indio_dev == NULL) {
		pr_warn("lpc_sensor_register: memory allocation failed\n");
		result = -ENOMEM;
		goto out_no_free;
	}

	senbase = iio_priv(indio_dev);

	senbase->client = client;
	indio_dev->name = sencfg->name;
	indio_dev->channels = base_channels;
	indio_dev->num_channels = ARRAY_SIZE(base_channels);

	indio_dev->info = &sensor_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->currentmode = INDIO_DIRECT_MODE;

	/* add bytes for converting 32 bit time stamp into 64 bit,
	    when reporting to HAL */
	result = lpc_sensor_configure_ring(
		indio_dev,
		sencfg->samplebytes +
		sizeof(struct lpcsh_hal_sensor_node) -
		sizeof(struct lpcsh_sensor_node),
		MAX_RINGBUFFERLENGTH * 3);
	if (result) {
		dev_warn(&senbase->client->dev,
			"lpc_sensor_register: configure ring buffer fail\n");
		goto out_free;
	}

	result = iio_device_register(indio_dev);
	if (result) {
		dev_warn(&senbase->client->dev,
			"lpc_sensor_register: IIO device register fail\n");
		goto out_unreg_iio;
	}

	mutex_init(&senbase->mutex);
	mutex_init(&senbase->iiomutex);

	senbase->indio = indio_dev;
	senbase->sensorId = sencfg->sensorId;
	senbase->enabled = false;
	senbase->lastsamplets = 0;
	senbase->samplebytes = sencfg->samplebytes;

	senbase->polldelay = 0xffff;
/*
	dev_dbg(&senbase->client->dev, "lpc_sensor_register: senbase= %p indio = %p\n",
			senbase, indio_dev);
*/
	*pSenbase = senbase;

	return 0;
out_unreg_iio:
	iio_device_unregister(indio_dev);
	lpc_sensor_unconfigure_ring(indio_dev);
out_free:
	//iio_free_device(indio_dev);
	iio_device_free(indio_dev);
out_no_free: dev_dbg(&senbase->client->dev, "sensors_register error.\n");
	return -EIO;
}

void lpc_sensor_unregister(void *senbaseP)
{
	struct iio_dev *indio_dev;

	struct sensor_base *senbase = (struct sensor_base *)senbaseP;


	mutex_destroy(&senbase->mutex);
	mutex_destroy(&senbase->iiomutex);

	indio_dev = senbase->indio;
	lpc_sensor_unconfigure_ring(indio_dev);
	iio_device_unregister(indio_dev);
	//iio_free_device(indio_dev);
	iio_device_free(indio_dev);

	dev_dbg(&senbase->client->dev, "lpc_sensor_unregister: sensors_unregister exit\n");
}
extern void mdss_hbm_set_state(int enable);
int lpc_sensor_newdata(
	struct i2c_client *client,
	void *senbaseP,
	struct lpcsh_sensor_node *node)
{
	struct lpcsh_hal_sensor_node halNode;
	struct sensor_base *senbase;
	struct iio_buffer *iio_buf;
	static uint64_t timestampUpperBits;
	uint64_t timestamp;
	int ret = 0;

	if (node->header.sensorId == LPCSH_SENSOR_ID_TIMESTAMP) {
		/* Warning: senbaseP == NULL for this sensor id*/
		timestampUpperBits =
			((uint64_t)node->data.fullTimestamp.timestampUpper32) <<
			16L;
		return 0;
	}
	if (senbaseP == NULL) {
		dev_err(
			&client->dev,
			"senbaseP == NULL  for sensorId [%-2.2d]\n",
			node->header.sensorId);
		return -1;
	}
	timestamp = lpc5400x_get_synced_timestamp(
		client,
		timestampUpperBits | (uint64_t)node->header.timeStamp);
	senbase = (struct sensor_base *)senbaseP;
	iio_buf = senbase->indio->buffer;

	if (node->header.sensorId == LPCSH_SENSOR_ID_META_SENSOR_FLUSH_DONE)
		dev_dbg(
			&client->dev,
			"End of flush fifo for sensorId [%-2.2d] IIO buffer\n",
			senbase->sensorId);
	if(node->header.sensorId == LPCSH_SENSOR_ID_LIGHT)
	{
		int lightdata =  node->data.lightData.Data  /  (1 << 8);
		//dev_err(	&client->dev,"mdss hbm [%d]\n", lightdata );	
		
		if(lightdata > 40000)
			mdss_hbm_set_state(1);
		else
			mdss_hbm_set_state(0);
	}
	if (senbase->enabled) {
#if 0
		dev_dbg(&client->dev,
			"sensorId [%-2.2d] sts [%-10.10llu]  lts [%-10.10llu]\n",
			senbase->sensorId,
			timestamp,
			timestampUpperBits | (uint64_t)node->header.timeStamp);
#endif
		memcpy(&halNode.node, node, senbase->samplebytes);
		halNode.timeStamp = timestamp;

		mutex_lock(&senbase->iiomutex);
		//ret = iio_buf->access->store_to(iio_buf, cp, t);
		ret = iio_buf->access->store_to(
			iio_buf,
			(char *)&halNode);  /* wangjianping modified here, NEED TO CONFIRM */
		if (ret != 0)
			dev_err(
				&client->dev,
				"Error on store data to sensor [%d] IIO buffer, %d\n",
				node->header.sensorId,
				ret);
		mutex_unlock(&senbase->iiomutex);
	}

	return 0;
}
