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
#include <linux/delay.h>
#include <linux/i2c.h>

#include "lpc5400x_voice_trigger.h"

#include "hostif_protocol.h"

#ifdef CONFIG_LPC5400X_SENSORHUB_I2C

int lpc5400x_send_buf(struct i2c_client *client, u8 *buffer, int length);
void lpc5400x_lock_i2c_client(struct i2c_client *client);
void lpc5400x_unlock_i2c_client(struct i2c_client *client);

#endif


struct lpc5400x_vt {
	struct mutex mutex;
	dev_t dev_numeber;
	struct cdev vt_dev;
	struct class *vt_class;
	struct i2c_client *client;
	struct device *vt_device;


	struct ShVoiceTriggerWriteUserDataCommand_t hostCommand;

};


int lpc5400x_voicetrigger_control(void *handle, bool enable)
{
	struct lpc5400x_vt *vt = handle;
	int ret;

	vt->hostCommand.command = enable ?
							  LPCSH_VOICE_TRIGGER_ENABLE : LPCSH_VOICE_TRIGGER_DISABLE;

	//lpc5400x_lock_i2c_client(vt->client);
	ret = lpc5400x_send_buf(vt->client, (u8 *)&vt->hostCommand, 1);
	//lpc5400x_unlock_i2c_client(vt->client);

	if (ret < 0)
		dev_info(
			&vt->client->dev,
			"lpc5400x_vt_control: Failed command 0x%x , Bus error",
			vt->hostCommand.command);
	return ret;
}


static int lpc5400x_vt_open(struct inode *pInode, struct file *flip)
{
	struct lpc5400x_vt *vt =
		container_of(pInode->i_cdev, struct lpc5400x_vt, vt_dev);

	flip->private_data = vt;

	dev_info(
		&vt->client->dev,
		"lpc5400x_vt_open: vt = %p, name = %s\n",
		vt,
		VOICE_TRIGGER_FILE_NAME);

	return 0;
}

static int lpc5400x_vt_release(struct inode *pInode, struct file *flip)
{
	struct lpc5400x_vt *vt = container_of(pInode->i_cdev, struct lpc5400x_vt, vt_dev);

	dev_info(
		&vt->client->dev,
		"lpc5400x_vt_release: vt = %p, name = %s\n",
		vt,
		VOICE_TRIGGER_FILE_NAME);

	return 0;
}

static ssize_t lpc5400x_vt_write(struct file *flip,
								 const char __user *pUser,
								 size_t size,
								 loff_t *offset)
{
	struct lpc5400x_vt *vt = flip->private_data;


	int ret = 0;
	bool bret = true;


	dev_info(&vt->client->dev, "lpc5400x_vt_write: size = %d", size);

	ret = lpc5400x_voicetrigger_control(vt, false);
	if (ret < 0)
		bret = false;
	else {
		msleep(15);
		vt->hostCommand.offset = 0;
		vt->hostCommand.command = LPCSH_VOICE_TRIGGER_WRITE;

		while (vt->hostCommand.offset < size) {
			if ((size - vt->hostCommand.offset) > VOICE_TRIGGER_SUB_BLOCK_MAX_SIZE)
				vt->hostCommand.size = VOICE_TRIGGER_SUB_BLOCK_MAX_SIZE;
			else
				vt->hostCommand.size  = size - vt->hostCommand.offset;


			if ((vt->hostCommand.size  + vt->hostCommand.offset) == size)
				vt->hostCommand.command = LPCSH_VOICE_TRIGGER_WRITE_LAST;

			if (copy_from_user(
					vt->hostCommand.data,
					pUser + vt->hostCommand.offset,
					vt->hostCommand.size))
				return -EFAULT;

			if (vt->hostCommand.size < VOICE_TRIGGER_SUB_BLOCK_MAX_SIZE) {
				memset(
					vt->hostCommand.data + vt->hostCommand.size,
					0,
					VOICE_TRIGGER_SUB_BLOCK_MAX_SIZE - vt->hostCommand.size);
			}

			dev_info(
				&vt->client->dev,
				"lpc5400x_vt_write: command 0x%x  offset %d, size = %d",
				vt->hostCommand.command,
				vt->hostCommand.offset,
				vt->hostCommand.size);



			/* Need to check the ret and response from Niobe */

			//lpc5400x_lock_i2c_client(vt->client);
			ret = lpc5400x_send_buf(
				vt->client,
				(u8 *)&vt->hostCommand,
				sizeof(vt->hostCommand));
			//lpc5400x_unlock_i2c_client(vt->client);

			if (ret < 0) {
				dev_info(
					&vt->client->dev,
					"lpc5400x_vt_write: Failed command 0x%x , Bus error",
					vt->hostCommand.command);
				bret = false;
				break;
			}
			vt->hostCommand.offset += vt->hostCommand.size;
		}
		ret = lpc5400x_voicetrigger_control(vt, true);
		if (ret < 0)
			bret = false;
	}
	if (bret)
		dev_info(
			&vt->client->dev,
			"lpc5400x_vt_write: Written %d\n",
			vt->hostCommand.offset);
	else
		dev_err(
			&vt->client->dev,
			"Error in program sensory data failed"\n);
	return size;
}

static const struct file_operations vt_ops {
	.owner = THIS_MODULE,
	.open = lpc5400x_vt_open,
	.write = lpc5400x_vt_write,
	.release = lpc5400x_vt_release,
};

int lpc5400x_create_vt_device(void **pHandle, struct i2c_client *client)
{
	struct lpc5400x_vt *vt =
		kzalloc(sizeof(struct lpc5400x_vt), GFP_KERNEL);
	int ret = 0;

	*pHandle = NULL;

	if (vt == NULL) {
		ret = -ENOMEM;
		goto exit_1;
	}

	mutex_init(&vt->mutex);
	vt->client = client;

	ret = alloc_chrdev_region(
		&vt->dev_numeber,
		0,
		1,
		VOICE_TRIGGER_FILE_NAME);
	if (ret < 0) {
		dev_err(
			&vt->client->dev,
			"lpc5400x failed to alloc_chrdev_region for vt");
		goto exit_3;
	}

	vt->vt_class = class_create(THIS_MODULE, VOICE_TRIGGER_FILE_NAME);
	if (IS_ERR(vt->vt_class)) {
		dev_err(
			&vt->client->dev,
			"Lpc5400x vt class create failed");
		ret = PTR_ERR(vt->vt_class);
		goto exit_2;
	}

	cdev_init(&vt->vt_dev, &vt_ops);

	vt->vt_dev.owner = THIS_MODULE;

	ret = cdev_add(&vt->vt_dev, vt->dev_numeber, 1);
	if (ret) {
		dev_err(
			&vt->client->dev,
			"Lpc5400x vfs fail to add cdev: %d",
			ret);
		goto exit_1;
	}

	vt->vt_device = device_create(vt->vt_class, &vt->client->dev,
		vt->dev_numeber, vt, VOICE_TRIGGER_FILE_NAME);

	if (IS_ERR(vt->vt_device)) {
		ret = PTR_ERR(vt->vt_device);
		goto exit_1;
	}

	dev_info(
		&vt->client->dev,
		"lpc5400x_create_sysfsnode %s, vt = %p, device = %p",
		VOICE_TRIGGER_FILE_NAME,
		vt,
		vt->vt_device);

	*pHandle = vt;

	return 0;

exit_1:
	class_destroy(vt->vt_class);
exit_2:
	unregister_chrdev_region(vt->dev_numeber, 1);
exit_3:
	kfree(vt);

	*pHandle = NULL;

	return ret;
}


int lpc5400x_destory_vt_device(void *handle)
{
	struct lpc5400x_vt *vt = handle;

	device_unregister(vt->vt_device);
	class_destroy(vt->vt_class);
	unregister_chrdev_region(vt->dev_numeber, 1);
	mutex_destroy(&vt->mutex);
	kfree(vt);
	return 0;
}




