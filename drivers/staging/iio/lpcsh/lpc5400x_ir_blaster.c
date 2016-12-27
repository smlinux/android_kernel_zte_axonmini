#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/kfifo.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>

#ifdef CONFIG_IIO_KFIFO_BUF

    #include <linux/iio/kfifo_buf.h>
#else
 #include "../ring_sw.h"
#endif


#include "lpc5400x_ir_blaster.h"

#include "hostif_protocol.h"

#include <linux/lpcsh/lpc5400x.h>

#ifdef CONFIG_LPC5400X_SENSORHUB_I2C

int lpc5400x_send_buf(
	struct i2c_client *client,
	u8 *buffer,
	int length);

int lpc5400x_read_buf(
	struct i2c_client *client,
	u8 cmd,
	u8 *buffer,
	int length);

static int lpc_sensor_configure_ring(
	struct iio_dev *indio_dev,
	int samplebytes,
	size_t ringLength);

#endif

struct ir_blaster_iio {
	struct i2c_client *client;
	struct iio_dev *indio;
	char devicename[32];
	bool learningEnabled;
	bool abortlearn;
	bool abortsend;
	unsigned model;
	struct mutex iiomutex;
};

struct lpc5400x_ir_blaster {
	dev_t ir_blaster_tx_dev_numeber;
	struct cdev ir_blaster_tx_dev;
	struct class *ir_blaster_tx_class;
	struct i2c_client *client;
	struct device *ir_blaster_tx_device;

    //struct mutex ir_blaster_rx_mutex;
    //struct mutex ir_blaster_status_mutex;

	uint16_t learning_result_record_size;
	uint8_t learndata[IR_BLASTER_LEARN_DATA_LENGTH_MAX];

	struct ir_blaster_iio *iio;

	struct lpcsh_ir_blaster_tx_command_t hostTxCommand;
};

static int lpc5400x_ir_blaster_open(
	struct inode *pInode,
	struct file *flip)
{
	struct lpc5400x_ir_blaster *ir_blaster =
		container_of(
			pInode->i_cdev,
			struct lpc5400x_ir_blaster,
			ir_blaster_tx_dev);

	flip->private_data = ir_blaster;

	dev_info(
		&ir_blaster->client->dev,
		"lpc5400x_ir_blaster_open: ir_blaster = %p, name = %s\n",
		ir_blaster,
		IR_BLASTER_TX_FILE_NAME);

	return 0;
}

static int lpc5400x_ir_blaster_release(
	struct inode *pInode,
	struct file *flip)
{
	struct lpc5400x_ir_blaster *ir_blaster =
		container_of(
			pInode->i_cdev,
			struct lpc5400x_ir_blaster,
			ir_blaster_tx_dev);

	dev_info(
		&ir_blaster->client->dev,
		"lpc5400x_ir_blaster_release: ir_blaster = %p, name = %s\n",
		ir_blaster,
		IR_BLASTER_TX_FILE_NAME);

	return 0;
}

static ssize_t lpc5400x_ir_blaster_write(
	struct file *flip,
	const char __user *pUser,
	size_t size,
	loff_t *offset)
{
	struct lpc5400x_ir_blaster *ir_blaster = flip->private_data;


	int ret = 0;
	bool bret = true;

	uint16_t packet_offset = 0;
	uint16_t packet_size = 0;

	dev_info(
		&ir_blaster->client->dev,
		"lpc5400x_ir_blaster_write: size = %d\n",
		(int)size);

	ir_blaster->hostTxCommand.hdr.command = LPCSH_IR_BLASTER_START_TX;

	while(packet_offset < size) {

        if ((size - packet_offset) > IR_BLASTER_TX_SUB_BLOCK_MAX_SIZE)
			packet_size = IR_BLASTER_TX_SUB_BLOCK_MAX_SIZE;
		else
            packet_size  = size - packet_offset;


		if (packet_offset == 0)	/* report full size for first packet */
			ir_blaster->hostTxCommand.hdr.size  = size;
		else					/* report actual size for all other packets */
			ir_blaster->hostTxCommand.hdr.size = packet_size;


		if (copy_from_user(
				ir_blaster->hostTxCommand.data,
				pUser + packet_offset,
				packet_size))
			return -EFAULT;

		if (packet_size < IR_BLASTER_TX_SUB_BLOCK_MAX_SIZE) {
			memset(
				ir_blaster->hostTxCommand.data + packet_size,
				0,
				IR_BLASTER_RX_SUB_BLOCK_MAX_SIZE - packet_size);
		}

		dev_info(
			&ir_blaster->client->dev,
			"lpc5400x_ir_blaster_write: command %d  offset %d, size = %d\n",
			ir_blaster->hostTxCommand.hdr.command,
			packet_offset,
			ir_blaster->hostTxCommand.hdr.size);



		/* Need to check the ret and response from Niobe */

		ret = lpc5400x_send_buf(
			ir_blaster->client,
			(u8 *)&ir_blaster->hostTxCommand,
			sizeof(ir_blaster->hostTxCommand));

		if (ret < 0) {
			dev_info(
				&ir_blaster->client->dev,
				"Failed to upgrade firmware, Bus error\n");
			bret = false;
			break;
		}
		packet_offset += packet_size;
		//size -= packet_size;
		ir_blaster->hostTxCommand.hdr.command = LPCSH_IR_BLASTER_NEXT_TX;

	}
	return packet_offset;
}


static const struct iio_chan_spec base_channels[] = {
	IIO_CHAN_SOFT_TIMESTAMP(0)
};

static void lpc5400x_sensor_unconfigure_ring(struct iio_dev *indio_dev)
{
#ifdef CONFIG_IIO_KFIFO_BUF
	iio_kfifo_free(indio_dev->buffer);
#else
	iio_sw_rb_free(indio_dev->buffer);
#endif
}



static ssize_t  lpc5400x_ir_learn_show(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ir_blaster_iio *iio = iio_priv(indio_dev);

	dev_dbg(dev, "lpc5400x_ir_learn_show  val = %u\n",
		iio->learningEnabled ? 1 : 0);

	return sprintf(buf, "%d\n", iio->learningEnabled ? 1 : 0);
}

static ssize_t lpc5400x_ir_learn_store(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{

	struct ShIrBlasterCommand_t ir_command;

	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ir_blaster_iio *ir_blaster_iio = iio_priv(indio_dev);

	unsigned long val;
	int err = 0;

	err = strict_strtoul(buf, 10, &val);
	if (err)
		return 0;

	dev_dbg(
		&ir_blaster_iio->client->dev,
		"lpc5400x_ir_learn_store val = [%lu]\n", val);


#ifdef CONFIG_LPC5400X_SENSORHUB_FIRMWARE_UPDATE_I2C
	/*err = lpc5400x_check_firmware(iio->client);
	if (err < 0) {
	    return 0;
	}*/
#endif	    
	if ((val) && (!ir_blaster_iio->learningEnabled))
		dev_dbg(
			&ir_blaster_iio->client->dev,
			"*** lpc5400x_ir_learn_store to enable\n");
	else
	if ((!val) && (ir_blaster_iio->learningEnabled))
		dev_dbg(&ir_blaster_iio->client->dev, "*** lpc5400x_ir_learn_store to disable\n");
	else
		dev_dbg(
			&ir_blaster_iio->client->dev,
			"*** lpc5400x_ir_learn_store unchanged state %s \n",
			ir_blaster_iio->learningEnabled ? "enabled" : "disabled");

	ir_blaster_iio->learningEnabled = val ? true : false;

	ir_command.command = LPCSH_IR_BLASTER_LEARN_START;

	err = lpc5400x_send_buf(
		ir_blaster_iio->client,
		(u8 *)&ir_command,
		sizeof(ir_command));

	if (err < 0)
		dev_info(
			&ir_blaster_iio->client->dev,
			"lpc5400x_ir_learn_store Failed to send command , error [%d]\n",
			err);

	return count;
}

static ssize_t  lpc5400x_ir_abortsending_show(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ir_blaster_iio *ir_blaster_iio = iio_priv(indio_dev);

	dev_dbg(
		dev,
		"lpc5400x_ir_abortsending_show  val = %u\n",
		ir_blaster_iio->abortsend ? 1 : 0);
	return sprintf(buf, "%d\n", ir_blaster_iio->abortsend ? 1 : 0);
}


static ssize_t lpc5400x_ir_abortsending_store(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{

	struct ShIrBlasterCommand_t ir_command;

	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ir_blaster_iio *ir_blaster_iio = iio_priv(indio_dev);

	unsigned long val;
	int err = 0;

	err = strict_strtoul(buf, 5, &val);
	if (err)
		return 0;

	dev_dbg(
		&ir_blaster_iio->client->dev,
		"lpc5400x_ir_abortsending_store val = [%lu]\n",
		val);

#ifdef CONFIG_LPC5400X_SENSORHUB_FIRMWARE_UPDATE_I2C
/*	err = lpc5400x_check_firmware(iio->client);
	if (err < 0) {
	    return 0;
	}*/	
#endif	    
	if ((val) && (!ir_blaster_iio->abortsend))
		dev_dbg(
			&ir_blaster_iio->client->dev,
			"*** lpc5400x_ir_abortsending_store to enable\n");
	else
	if ((!val) && (ir_blaster_iio->abortsend))
		dev_dbg(
			&ir_blaster_iio->client->dev,
			"*** lpc5400x_ir_abortsending_store to disable\n");
	else
		dev_dbg(
			&ir_blaster_iio->client->dev,
			"*** lpc5400x_ir_abortsending_store unchanged state %s\n",
			ir_blaster_iio->abortsend ? "enabled" : "disabled");

	ir_blaster_iio->abortsend = val ? true : false;

	ir_command.command = LPCSH_IR_BLASTER_CANCEL_SENDING;

	err = lpc5400x_send_buf(
		ir_blaster_iio->client,
		(u8 *)&ir_command,
		sizeof(ir_command));

	if (err < 0)
		dev_info(
			&ir_blaster_iio->client->dev,
			"lpc5400x_ir_abortsending_store Failed to send command , error [%d]\n",
			err);

	return count;
}

static ssize_t  lpc5400x_ir_abortlearning_show(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ir_blaster_iio *ir_blaster_iio = iio_priv(indio_dev);

	dev_dbg(
		dev,
		"lpc5400x_ir_abortlearning_show  val = %u\n",
		ir_blaster_iio->abortlearn ? 1 : 0);

	return sprintf(
		buf,
		"%d\n",
		ir_blaster_iio->abortlearn ? 1 : 0);
}


static ssize_t lpc5400x_ir_abortlearning_store(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{

	struct ShIrBlasterCommand_t ir_command;

	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ir_blaster_iio *ir_blaster_iio = iio_priv(indio_dev);

	unsigned long val;
	int err = 0;

	err = strict_strtoul(buf, 5, &val);
	if (err)
		return 0;

	dev_dbg(
		&ir_blaster_iio->client->dev,
		"lpc5400x_ir_abortlearning_store val = [%lu]\n",
		val);

#ifdef CONFIG_LPC5400X_SENSORHUB_FIRMWARE_UPDATE_I2C
/*	err = lpc5400x_check_firmware(iio->client);
	if (err < 0) {
	    return 0;
	}*/
#endif	    
	if ((val) && (!ir_blaster_iio->abortlearn))
		dev_dbg(
			&ir_blaster_iio->client->dev,
			"*** lpc5400x_ir_abortlearning_store to enable\n");
	else
	if ((!val) && (ir_blaster_iio->abortlearn))
		dev_dbg(
			&ir_blaster_iio->client->dev,
			"*** lpc5400x_ir_abortlearning_store to disable\n");
	else
		dev_dbg(
			&ir_blaster_iio->client->dev,
			"*** lpc5400x_ir_abortlearning_store unchanged state %s\n",
			ir_blaster_iio->abortlearn ? "enabled" : "disabled");

	ir_blaster_iio->abortlearn = val ? true : false;

	/* when we set abortlearn, we should be clear learningEnabled. */
	if (ir_blaster_iio->abortlearn)
		ir_blaster_iio->learningEnabled = false;

	ir_command.command = LPCSH_IR_BLASTER_CANCEL_LEARNING;

	err = lpc5400x_send_buf(
		ir_blaster_iio->client,
		(u8 *)&ir_command,
		sizeof(ir_command));

	if (err < 0)
		dev_info(
			&ir_blaster_iio->client->dev,
			"lpc5400x_ir_abortlearning_store Failed to send command , error [%d]\n",
			err);

	return count;
}

static DEVICE_ATTR(learn, 0666,
	lpc5400x_ir_learn_show,
	lpc5400x_ir_learn_store);
static DEVICE_ATTR(abortsending, 0666,
	lpc5400x_ir_abortsending_show,
	lpc5400x_ir_abortsending_store);
static DEVICE_ATTR(abortlearning, 0666,
	lpc5400x_ir_abortlearning_show,
	lpc5400x_ir_abortlearning_store);

static struct attribute *ir_blaster_attributes[] = {
	&dev_attr_learn.attr,
	&dev_attr_abortsending.attr,
	&dev_attr_abortlearning.attr,
	NULL,
};

static const struct attribute_group ir_blaster_attr_group = {
	.attrs	= ir_blaster_attributes,
};

static const struct iio_info ir_blaster_info = {
	.driver_module	= THIS_MODULE,
	.attrs			= &ir_blaster_attr_group,
};

static const struct file_operations ir_blaster_tx_ops = {
	.owner		= THIS_MODULE,
	.open		= lpc5400x_ir_blaster_open,
	.write		= lpc5400x_ir_blaster_write,
	.release	= lpc5400x_ir_blaster_release,
};

int lpc5400x_create_ir_blaster_device(
	void **pHandle,
	struct i2c_client *client)
{
	struct lpc5400x_ir_blaster *ir_blaster =
		kzalloc(sizeof(struct lpc5400x_ir_blaster), GFP_KERNEL);
	int ret = 0;
	struct iio_dev *indio_dev;

	*pHandle = NULL;

	if (ir_blaster == NULL) {
		ret = -ENOMEM;
		goto exit_3;
	}

	//mutex_init(&ir_blaster->ir_blaster_rx_mutex);
	//mutex_init(&ir_blaster->ir_blaster_status_mutex);


	ir_blaster->client = client;

	ret = alloc_chrdev_region(
		&ir_blaster->ir_blaster_tx_dev_numeber,
		0,
		1,
		IR_BLASTER_TX_FILE_NAME);

	if (ret < 0) {
		dev_err(&ir_blaster->client->dev, "lpc5400x failed to alloc_chrdev_region for ir_blaster\n");
		goto exit_3;
	}

	ir_blaster->ir_blaster_tx_class =
		class_create(
			THIS_MODULE,
			IR_BLASTER_TX_FILE_NAME);
	if (IS_ERR(ir_blaster->ir_blaster_tx_class)) {
		dev_err(
			&ir_blaster->client->dev,
			"Lpc5400x ir_blaster class create failed\n");
		ret = PTR_ERR(ir_blaster->ir_blaster_tx_class);
		goto exit_2;
	}

	cdev_init(&ir_blaster->ir_blaster_tx_dev, &ir_blaster_tx_ops);

	ir_blaster->ir_blaster_tx_dev.owner = THIS_MODULE;

	ret = cdev_add(
		&ir_blaster->ir_blaster_tx_dev,
		ir_blaster->ir_blaster_tx_dev_numeber,
		1);
	if (ret) {
		dev_err(&ir_blaster->client->dev, "Lpc5400x vfs fail to add cdev: %d\n", ret);
		goto exit_1;
	}

	ir_blaster->ir_blaster_tx_device =
		device_create(
			ir_blaster->ir_blaster_tx_class,
			&ir_blaster->client->dev,
			ir_blaster->ir_blaster_tx_dev_numeber,
			ir_blaster, IR_BLASTER_TX_FILE_NAME);

	if (IS_ERR(ir_blaster->ir_blaster_tx_device)) {
		ret = PTR_ERR(ir_blaster->ir_blaster_tx_device);
		goto exit_1;
	}

	dev_info(&ir_blaster->client->dev, "lpc5400x_create_sysfsnode %s, ir_blaster = %p, device = %p\n",
		IR_BLASTER_TX_FILE_NAME, ir_blaster, ir_blaster->ir_blaster_tx_device);


	indio_dev = iio_device_alloc(sizeof(struct ir_blaster_iio));
	if (indio_dev == NULL) {
		pr_warn("lpc5400x_create_ir_blaster_device: memory allocation failed\n");
		ret = -ENOMEM;
		goto exit_0;
	}

	ir_blaster->iio = iio_priv(indio_dev);
	ir_blaster->iio->client = client;
	mutex_init(&ir_blaster->iio->iiomutex);


	indio_dev->name = IR_BLASTER_LEARNING_DATA_NAME;
	indio_dev->channels = base_channels;
	indio_dev->num_channels = ARRAY_SIZE(base_channels);
	indio_dev->info = &ir_blaster_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->currentmode = INDIO_DIRECT_MODE;

	ret = lpc_sensor_configure_ring(indio_dev, IR_BLASTER_LEARN_DATA_LENGTH_MAX, 5);
	if (ret) {
		dev_warn(&indio_dev->dev,
			"lpc_sensor_register: configure ring buffer fail\n");
		goto out_free;
	}

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_warn(&indio_dev->dev,
			"lpc_sensor_register: IIO device register fail\n");
		goto out_unreg_iio;
	}
	ir_blaster->iio->indio = indio_dev;

	*pHandle = ir_blaster;

	return 0;
out_unreg_iio:
	iio_device_unregister(indio_dev);
	lpc5400x_sensor_unconfigure_ring(indio_dev);
out_free:
	iio_device_free(indio_dev);
exit_0:
	device_unregister(ir_blaster->ir_blaster_tx_device);
exit_1:
	class_destroy(ir_blaster->ir_blaster_tx_class);
exit_2:
	unregister_chrdev_region(ir_blaster->ir_blaster_tx_dev_numeber, 1);
	//mutex_destroy(&ir_blaster->ir_blaster_rx_mutex);
	//mutex_destroy(&ir_blaster->ir_blaster_status_mutex);
	mutex_destroy(&ir_blaster->iio->iiomutex);
exit_3:
	kfree(ir_blaster);

	*pHandle = NULL;

	return ret;
}


int lpc5400x_destory_ir_blaster_tx_device(void *handle)
{
	struct lpc5400x_ir_blaster *ir_blaster = handle;

	device_unregister(ir_blaster->ir_blaster_tx_device);
	class_destroy(ir_blaster->ir_blaster_tx_class);
	unregister_chrdev_region(ir_blaster->ir_blaster_tx_dev_numeber, 1);
	//mutex_destroy(&ir_blaster->ir_blaster_rx_mutex);
	//mutex_destroy(&ir_blaster->ir_blaster_status_mutex);
	mutex_destroy(&ir_blaster->iio->iiomutex);

	kfree(ir_blaster);
	return 0;
}

int lpc5400x_report_ir_blaster_learning_record(
	void *handle,
	const char *data,
	uint8_t size,
	uint16_t sequenceNumber)//zhaoheping 0727 32->16,ffff
{
	int err = 0;
	int isSend = 0;
	struct lpc5400x_ir_blaster *ir_blaster = handle;
	 //s64 timestamp = timeStamp32_to_int64(sequenceNumber);

	dev_info(
		&ir_blaster->client->dev,
		"lpc5400x_report_ir_blaster_learning_record sequenceNumber=%d, size=%d \n",
		sequenceNumber,
		size);
	//zhaoheping nxp <=,other project use IR need comfirm
    if ((ir_blaster->learning_result_record_size + size) < 
		IR_BLASTER_LEARN_DATA_LENGTH_MAX) {
		/* buffering all data. */
		memcpy(
			ir_blaster->learndata +
			ir_blaster->learning_result_record_size,
			data,
			size);
		ir_blaster->learning_result_record_size += size;
		
		/* when we received last package, we should wirte data to IIO buffer. */
		//if (0xffffffff == sequenceNumber){  
		//	isSend = 1;
		if (0xffff == sequenceNumber){  
			isSend = 1;
		}//zhaoheping 16bit ffff
	} else {
		int availabesize = IR_BLASTER_LEARN_DATA_LENGTH_MAX -
						   ir_blaster->learning_result_record_size;

		/* fill the buffer and save to IIO buffer. */
		memcpy(
			ir_blaster->learndata + ir_blaster->learning_result_record_size,
			data,
			availabesize);

		mutex_lock(&ir_blaster->iio->iiomutex);
		err = ir_blaster->iio->indio->buffer->access->store_to(
			ir_blaster->iio->indio->buffer,
			 ir_blaster->learndata);
		mutex_unlock(&ir_blaster->iio->iiomutex);

        	/* tolerance fault program  */
		ir_blaster->learning_result_record_size = size - availabesize;
        	if (ir_blaster->learning_result_record_size > 0){
		memcpy(ir_blaster->learndata, 
			   data, 
			   ir_blaster->learning_result_record_size);
		
			if (0xffff == sequenceNumber){
				isSend = 1;
			}
			else {
				isSend = 0;
			}
    	}else{
			isSend = 0;
    		}
	}

    if ((0xffff == sequenceNumber) && (1 == isSend))//A-mini not use IR,other project if use,need confirm
    {
		if (ir_blaster->learning_result_record_size <
			IR_BLASTER_LEARN_DATA_LENGTH_MAX) {
			memset(
				ir_blaster->learndata +
				ir_blaster->learning_result_record_size,
				0x00,
				(IR_BLASTER_LEARN_DATA_LENGTH_MAX -
				 ir_blaster->learning_result_record_size));
		}

		dev_info(
			&ir_blaster->client->dev,
			"lpc5400x_report_ir_blaster_learning_record write to IIO buffer [%d]\n",
			ir_blaster->learning_result_record_size);
		/* save IBR header and raw data to IIO buffer. */
		mutex_lock(&ir_blaster->iio->iiomutex);
		/*err = ir_blaster->iio->indio->buffer->access->store_to(
		ir_blaster->iio->indio->buffer, 
		ir_blaster->learndata, 
		timestamp);*/

		/*Qualcomm platform api only 2 params. */
		err = ir_blaster->iio->indio->buffer->access->store_to
			(ir_blaster->iio->indio->buffer, 
			ir_blaster->learndata);
		ir_blaster->learning_result_record_size = 0;
		mutex_unlock(&ir_blaster->iio->iiomutex);
		if (err != 0)
			dev_err(&ir_blaster->client->dev, "lpc5400x_report_ir_blaster_learning_record() Error on store data IIO buffer, err=%d \n", err);
	}

	return err;
}

