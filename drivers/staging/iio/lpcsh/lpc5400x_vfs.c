#define DEBUG
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
#include <linux/lpcsh/lpc5400x.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <linux/log2.h>
#define LPC5400X_VFS_AIO
#ifdef LPC5400X_VFS_AIO
#include <linux/aio.h>
#endif

#include "lpc5400x_vfs.h"
#include "lpc5400x_update.h"
#include "lpc5400x_ramdump.h"
#include "hostif_protocol.h"
#include "lpc5400x_vfs_ramdump.h"
//zhangji
#include <linux/proc_fs.h>
#include "lpc5400x_vfs_logging.h"

#define DATA_KFIFO_LENGTH (512)
/* FIXME here, wangjianping, if use qualcomm i2c bus v3 protocol, LPC5400X_MAX_I2C_TRANSFER maybe change to 128 */
#define LPC5400X_MAX_I2C_TRANSFER (512) 
#define LPC5400X_MAX_I2C_RX (DATA_KFIFO_LENGTH + 4) /*Header size = 4*/
//nxp34663
static char commandbuffer[10];

int lpc5400x_getVersion(
	struct device *dev,
	uint8_t *major_version,
	uint8_t *minor_version);

int lpc5400x_getCalibrationData(
	struct device *dev,
	unsigned char *pData,
	int size);

int lpc5400x_startCalibration(
	struct device *dev,
	unsigned char *pData,
	int size);

#define SL_FLASH_BLOCK_SZ (0x200)
#define SENSORY_DATA_OFFSET (0x40000)

#define LPC5400X_KEY_LENGTH (64)
#define LPC5400X_DUMMY_KEY_LENGTH (1)
#define LPC5400X_DUMMY_KEY (0xBA)

#define LPC5400X_VERSION_LENGTH (2)
#define LPC5400X_CALIBRATION_LENGTH (32)
#define LPC5400X_CROSSTALK_LENGTH (4)

#ifdef CONFIG_LPC5400X_SENSORHUB_AUDIO_SENSE
/*Dmic clock configuraion info length(bytes)*/
#define LPC5400X_AUDIOSENSE_DMIC_CONFIGURATION_INFO_LENGHT (4)
#endif

//zhangji
char *sensor_proc_info[]=
{
	"BMI-BMI160-NA-NA-0x68",
	"ST-LSM6DS3-NA-NA-0x6A",
	"AKM-AK09911-NA-NA-0x0C",
	"STK-STK331128A-NA-NA-0x38",
	"ERR_GET_INFO"
};


enum LPC5400X_VFS_State {
	LPC5400X_VFS_STATE_NULL,
	LPC5400X_VFS_STATE_OPEN,
	LPC5400X_VFS_STATE_PROCESSING,
};



enum Sesnorhub_VFS_RamDump_Cmd {
	SENSORHUB_RAMDUMP_SUMMARY,
	SENSORHUB_RAMDUMP_REGISTERS,
	SENSORHUB_RAMDUMP_STACK,
	SENSORHUB_RAMDUMP_MEMORY,
};

struct Sensorhub_RamDump_Param {
	uint32_t command;
	uint32_t startAddr;
	uint32_t size;
};


static int lpc5400x_vfsdevice_command_handler(
	struct _lpc5400x_vfs_devices *vfs_device,
	const char *pData,
	size_t size,
	bool fromUser);


static ssize_t vfs_status_show(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct _lpc5400x_vfs_devices *vfsnode = NULL;

	vfsnode = (struct _lpc5400x_vfs_devices *)
			  dev_get_drvdata(dev);

	*buf = vfsnode->busy;

	pr_debug("vfs_status_show %d", vfsnode->busy);

	return 1;
}

static DEVICE_ATTR(status, 0444,
	vfs_status_show,
	NULL);


static struct attribute *lpc5400x_vfs_attributes[] = {
	&dev_attr_status.attr,
	NULL
};

static const struct attribute_group lpc5400x_vfs_attr_group = {
	.attrs = lpc5400x_vfs_attributes,
};

static struct _lpc5400x_vfs_device_info
	vfs_device_entities[SENSORHUB_DEVICE_NUM] = {
	{
		"lpcsh",
		NULL,
		NULL,
		NULL,
		NULL,
		NULL
	},
#ifdef CONFIG_LPC5400X_SENSORHUB_VOICE_TRIGGER
	{
		"voicetrigger",
		NULL,
		NULL,
		NULL,
		NULL,
		NULL
	},
#endif
#ifdef CONFIG_LPC5400X_SENSORHUB_RAMDUMP
	{
		"ramdump",
		lpc5400x_vfs_ramdump_prepare,
		lpc5400x_vfs_ramdump_write_hdl,
		lpc5400x_vfs_ramdump_read_hdl,
		lpc5400x_vfs_ramdump_poll_hdl,
		lpc5400x_vfs_ramdump_clean_up
	},
#endif
#ifdef CONFIG_LPC5400X_SENSORHUB_LOGGING_SERVICE
	{
		"lpcshlog", 
		lpc5400x_vfs_logging_prepare,
		lpc5400x_vfs_logging_write_hdl,
		lpc5400x_vfs_logging_read_hdl,
		lpc5400x_vfs_logging_poll_hdl,
		lpc5400x_vfs_logging_clean_up
	},
/*#else
	{
		"lpcshlog",
		NULL,
		NULL,
		NULL,
		NULL,
		NULL
	},*/
#endif
	{
		"crypt",
		NULL,
		NULL,
		NULL,
		NULL,
		NULL
	},
	{
		"Sensorlist",
		NULL,
		NULL,
		NULL,
		NULL,
		NULL
	},
	{
		"calibration",
		NULL,
		NULL,
		NULL,
		NULL,
		NULL
	},
#ifdef CONFIG_LPC5400X_SENSORHUB_AUDIO_SENSE
	{
		"audiosense",
		NULL,
		NULL,
		NULL,
		NULL,
		NULL
	},
#endif
    {
		"lpcsh_registerdump",
		NULL,
		NULL,
		NULL,
		NULL,
		NULL
	},
};

static int lpc5400x_vfs_open(struct inode *pInode, struct file *flip)
{
	struct _lpc5400x_vfs_devices *vfsdevice;

	vfsdevice = container_of(
		pInode->i_cdev,
		struct _lpc5400x_vfs_devices,
		vfs_dev);

	flip->private_data = vfsdevice;

	pr_debug(
		"lpc5400x_vfs_open: vfs = %p, name = %d\n",
		vfsdevice,
		vfsdevice->name);

	return 0;
}

static int lpc5400x_vfs_release(
	struct inode *pInode,
	struct file *flip)
{
/*	struct _lpc5400x_vfs_devices *vfsdevice = flip->private_data; */

	return 0;
}

static ssize_t lpc5400x_vfs_read(
	struct file *flip,
	char __user *pUser,
	size_t size,
	loff_t *offset)
{
	struct _lpc5400x_vfs_devices *vfsdevice = flip->private_data;
	unsigned int ret = 0, readsize = 0;
	uint8_t fwverion[LPC5400X_VERSION_LENGTH] = { 0xFF, 0xFF };

	switch (vfsdevice->name) {
	#ifdef CONFIG_LPC5400X_SENSORHUB_RAMDUMP
	case SENSORHUB_RAMDUMP:
		if (vfs_device_entities[vfsdevice->name].write_handler)
			readsize =
				vfs_device_entities[vfsdevice->name].read_handler(
					vfsdevice->parent->dev,
					pUser,
					size,
					vfsdevice->private);
/* pr_debug("lpc5400x_vfs_read...  ramdump data ret = %d, size = %d, readsize %d\n", ret, size, readsize); */
		break;
    #endif

	case SENSORHUB_LOGGING:
		if (vfs_device_entities[vfsdevice->name].write_handler)
			readsize =
				vfs_device_entities[vfsdevice->name].read_handler(
					vfsdevice->parent->dev,
					pUser,
					size,
					vfsdevice->private);
		pr_debug("lpc5400x_vfs_read...  logging data ret = %d, size = %d, readsize %d\n",
			ret,
			(int)size,
			readsize);
		break;

	case SENSORHUB_FIRMWAREUPDATE:
		lpc5400x_getVersion(
			vfsdevice->parent->dev,
			&fwverion[0],
			&fwverion[1]);
		printk("%s major_version.minor_version:%d.%d\n", __func__, fwverion[0], fwverion[1]);
		if (size != LPC5400X_VERSION_LENGTH)
			return -EINVAL;

		ret = copy_to_user(pUser, &fwverion, size);
		if (ret != 0) {
			pr_debug("copy_to_user error, ret = %d", ret);
			return size - ret;
		}
		readsize = size;
		break;

	case SENSORHUB_CALIBRATION:
	{
		u8 cal_data[LPC5400X_CROSSTALK_LENGTH];
		if (size != LPC5400X_CROSSTALK_LENGTH) {
			return -EINVAL;
		}

		ret = lpc5400x_getCalibrationData(vfsdevice->parent->dev,
			&cal_data[0], LPC5400X_CROSSTALK_LENGTH);
		if (ret < 0) {
			pr_err("lpc5400x_getCalibrationData error, ret = %d", ret);
			readsize = 0;
			break;
		}

		ret = copy_to_user(pUser, cal_data, LPC5400X_CROSSTALK_LENGTH);
		if (ret != 0) {
			pr_err("lpc5400x copy_to_user error, ret = %d", ret);
			return size - ret;
		}
		readsize = size;
		break;
	}
#ifdef CONFIG_LPC5400X_SENSORHUB_AUDIO_SENSE
	case SENSORHUB_AUDIOSENSE:
	{
		u16 cal_data[2] = { 0 };
		/* check params length. */
		if (size != sizeof(cal_data)) {
			pr_err("Invalid DMIC clock params length %d\n", (int)size);
			ret = -EINVAL;
			return ret;
		}

		/* send read data command to sensor hub, and get dmic configuration info. */
		ret = lpc5400x_getDmicClock(
			vfsdevice->parent->dev,
			(void *)&cal_data[0],
			size);
		if (ret < 0) {
			pr_debug("lpc5400x_getDmicClock error, ret = %d\n", ret);
			readsize = 0;
			break;
		}
		/* copy data to user space. */
		ret = copy_to_user(pUser, cal_data, sizeof(cal_data));
		if (ret != 0) {
			pr_debug("copy_to_user error, ret = %d\n", ret);
			return size - ret;
		}
		readsize = size;
		break;
	}
#endif

	case SENSORHUB_SENSORDUMP:
	{
		u8 cal_data[50];
		if (1 == commandbuffer[0])
		{
			memcpy(cal_data, &commandbuffer[1], 4);
			memset(commandbuffer, 0x00, sizeof(commandbuffer)); //clear pending flag
			ret= lpc5400x_read_sensor_register(vfsdevice->parent->dev, (void*)&cal_data[0], (int)size);
			if (ret < 0) {
				pr_err("lpc5400x_read_sensor_register error, ret = %d\n", ret);
				readsize = 0;
				break;
			}
		
			ret = copy_to_user(pUser, cal_data, sizeof(cal_data));
			if (ret != 0) {
				pr_err("lpc5400x_read_sensor_register copy_to_user error, ret = %d\n", ret);
				return size - ret;
			}
		}
		/* the first byte is buf length */
		readsize = size + 1;
		break;
	}
	default:
		break;
	}
	return readsize;
}

static ssize_t lpc5400x_vfs_write(
	struct file *flip,
	const char __user *pUser,
	size_t size,
	loff_t *offset)
{
	struct _lpc5400x_vfs_devices *vfsdevice = flip->private_data;
	int ret;

	pr_debug("lpc5400x_vfs_write: size = %d", (int)size);

	if (vfsdevice->busy)
		return -EAGAIN;

	vfsdevice->busy = true;

	ret = lpc5400x_vfsdevice_command_handler(
		vfsdevice, pUser, size, true);

	if (ret < 0) {
		pr_debug("Error in program data failed ret = %d", ret);
		vfsdevice->busy = false;
		return ret;
	}

#ifdef CONFIG_LPC5400X_SENSORHUB_RAMDUMP
	if (SENSORHUB_RAMDUMP != vfsdevice->name)
#endif
		vfsdevice->busy = false;

	pr_debug("lpc5400x_vfs_write: return = %d", (int)size);

	return size;
}

#ifdef LPC5400X_VFS_AIO
void lpc5400x_vfs_aio_task(struct work_struct *work)
{
	struct _lpc5400x_vfs_devices *vfsdevice =
		container_of(work, struct _lpc5400x_vfs_devices, aio_work);

	int ret;

	ret = lpc5400x_vfsdevice_command_handler(vfsdevice,
		vfsdevice->aio_tmpbuf,
		vfsdevice->aio_tmpbufsize,
		false);

#ifdef CONFIG_LPC5400X_SENSORHUB_RAMDUMP
	if (SENSORHUB_RAMDUMP != vfsdevice->name)
#endif
	 	vfsdevice->busy = false;

	aio_complete(vfsdevice->aio_iocb, ret, 0);
}

static ssize_t lpc5400x_vfs_aio_write(
	struct kiocb *iocb,
	const struct iovec *iov,
	unsigned long niov,
	loff_t offset)
{
	struct file *flip = iocb->ki_filp;
	struct _lpc5400x_vfs_devices *vfsdevice = flip->private_data;
	unsigned long i = 0, bufsize = 0, pos = 0;

	if (is_sync_kiocb(iocb)) {
		pr_info("Kernel asks for Synchronized Write");
		return lpc5400x_vfs_write(
			flip,
			iov->iov_base,
			iov->iov_len, &offset);
	}

	if (vfsdevice->busy)
		return -EAGAIN;

	vfsdevice->busy = true;

	pr_info("Asynchronous Write niov = %ld", niov);

	while (i < niov)
		bufsize += iov[i].iov_len;

	vfsdevice->aio_tmpbuf = (char *)kmalloc(bufsize, GFP_KERNEL);
	if (!vfsdevice->aio_tmpbuf)
		return -ENOMEM;

	i = 0;
	vfsdevice->aio_tmpbufsize = bufsize;
	while (i < niov) {
		if (copy_from_user(
				vfsdevice->aio_tmpbuf + pos,
				iov[i].iov_base,
				iov[i].iov_len)) {
			pos += iov[i].iov_len;
			i++;
		}
	}

	vfsdevice->aio_iocb = iocb;
	schedule_work(&vfsdevice->aio_work);

	return 0;
}
#endif


unsigned lpc5400x_vfs_poll(
	struct file *flip,
	struct poll_table_struct *wait)
{
	struct _lpc5400x_vfs_devices *vfsdevice = flip->private_data;
	unsigned int datasize = 0, ret = 0;

	poll_wait(flip, &vfsdevice->vfs_wq, wait);

	if (vfs_device_entities[vfsdevice->name].poll_handler)
		datasize =
			vfs_device_entities[vfsdevice->name].poll_handler(
				vfsdevice->parent->dev,
				vfsdevice->private);

	if (datasize > 0)
		ret = POLLIN | POLLRDNORM;
	else
		ret = 0;

	return ret;
}

int lpc5400x_probe_bus_for_update(struct device *dev, int gpio)
{
	int ret = 0;

	/* Probe the bus for firmware update */
	if (lpc5400x_probebus(dev, SL_I2C2, gpio)) {
		ret = -ENXIO;
		dev_err(dev, "Firmware update bus detection failed");
		return ret;
	}

	lpc5400x_romVersion(dev, gpio);

	dev_dbg(dev, "lpc5400x_probebus Success");

	return ret;
}

long lpc5400x_vfs_ioctl(struct file *flip, unsigned int request,
						unsigned long pUser)
{
	struct _lpc5400x_vfs_devices *vfsdevice = flip->private_data;
	struct lpc5400x_platform_data *pPlatformData;
	int ret = 0;
	int sensor_id_vfs = 0;
	struct i2c_client *client = i2c_verify_client(vfsdevice->parent->dev);

	pPlatformData = 
		(struct lpc5400x_platform_data *)vfsdevice->parent->dev->platform_data;

	pr_debug(
		"lpc5400x_vfs_ioctl %d, request = %d\n",
		vfsdevice->name,
		request);

	switch (vfsdevice->name) {
#ifdef CONFIG_LPC5400X_SENSORHUB_VOICE_TRIGGER		
	case SENSORHUB_UDTDATA:
		gpio_direction_output(pPlatformData->irq_gpio, 1);
		msleep(200);
		gpio_set_value_cansleep(pPlatformData->irq_gpio, 0);
		msleep(3000);
		break;
#endif
	case SENSORHUB_FIRMWAREUPDATE:
    {
		pr_err("SENSORHUB_FIRMWAREUPDATE 001.\n");

        /* FIXME here:Delay time should be confirmed */
        if(client->irq){
        	irq_set_irq_wake(client->irq, 0);  /*disable irq*/     				
        	disable_irq(client->irq);  /*disable irq*/
		}
        /* wangjianping added this for firmware update */
        //lpc5400x_cancel_timestamp_delayed_work(vfsdevice->parent->dev);
        gpio_direction_output(pPlatformData->wakeup_gpio, 0);
        gpio_set_value_cansleep(pPlatformData->wakeup_gpio, 0);
		msleep(20);
        gpio_direction_output(pPlatformData->reset_gpio, 1);
        gpio_set_value_cansleep(pPlatformData->reset_gpio, 0);
		msleep(20);
        gpio_set_value_cansleep(pPlatformData->reset_gpio, 1);
        msleep(100);
        gpio_direction_input(pPlatformData->wakeup_gpio);

        ret = lpc5400x_probe_bus_for_update(vfsdevice->parent->dev, 
				pPlatformData->wakeup_gpio);

		break;
    }
#ifdef CONFIG_LPC5400X_SENSORHUB_RAMDUMP
	case SENSORHUB_RAMDUMP:
		break;
#endif
	//zhangji
	case SENSORHUB_SENSORLIST:
	{		
		sensor_id_vfs=lpc5400x_report_sensorlistinfo(request,client);
		ret = copy_to_user((void __user *)pUser, (void*)(&sensor_id_vfs), sizeof(sensor_id_vfs));
		if (ret){
			pr_err("lpc5400x_vfs_ioctl copy sensor ID failed.");
		}
		break;
		break;
	}
	//zhangji
	case SENSORHUB_LOGGING:
		break;
	default:
		pr_debug("Unsupport vfsdevice->name %d", vfsdevice->name);
		break;
	}

	return ret;
}

const struct file_operations vfs_ops = {
	.owner			= THIS_MODULE,
	.open			= lpc5400x_vfs_open,
#ifdef LPC5400X_VFS_AIO
	.aio_write		= lpc5400x_vfs_aio_write,
#else
	.write			= lpc5400x_vfs_write,
#endif
	.read			= lpc5400x_vfs_read,
	.poll			= lpc5400x_vfs_poll,
	.unlocked_ioctl = lpc5400x_vfs_ioctl,
	.release		= lpc5400x_vfs_release,
};


static int lpc5400x_create_sysfsnode(
	struct _lpc5400x_vfs *vfs,
	struct _lpc5400x_vfs_devices **vfs_device_p,
	enum Sensorhub_VFS_DeviceName devicename)
{
	struct _lpc5400x_vfs_devices *vfs_device =
		kzalloc(sizeof(struct _lpc5400x_vfs_devices), GFP_KERNEL);

	int ret = 0;
	const char *strname = vfs_device_entities[devicename].name;

	*vfs_device_p = NULL;

	if (!vfs_device)
		return -ENOMEM;

	vfs_device->transbuffer = kmalloc(
		LPC5400X_MAX_I2C_TRANSFER, GFP_KERNEL);

	if (!vfs_device->transbuffer) {
		pr_warn("lpc5400x_create_sysfsnode mem allocation failed");
		kfree(vfs_device);
		return -ENOMEM;
	}

	init_waitqueue_head(&vfs_device->vfs_wq);

	ret = alloc_chrdev_region(&vfs_device->dev_numeber, 0, 1, strname);
	if (ret < 0) {
		pr_warn("lpc5400x vfsno chrdev region");
		kfree(vfs_device->transbuffer);
		kfree(vfs_device);
		return ret;
	}

	vfs_device->vfs_class = class_create(THIS_MODULE, strname);
	if (IS_ERR(vfs_device->vfs_class)) {
		pr_warn("Lpc5400x vfs class create failed");
		ret = PTR_ERR(vfs_device->vfs_class);
		goto exit2;
	}

	cdev_init(&vfs_device->vfs_dev, &vfs_ops);

	vfs_device->vfs_dev.owner = THIS_MODULE;
	vfs_device->parent = vfs;
	vfs_device->name = devicename;
	vfs_device->pPlatformData = vfs->dev->platform_data;  /* FIXME HERE */

	ret = cdev_add(&vfs_device->vfs_dev, vfs_device->dev_numeber, 1);
	if (ret) {
		pr_warn("Lpc5400x vfs fail to add cdev: %d", ret);
		goto exit1;
	}

	vfs_device->device = device_create(vfs_device->vfs_class, vfs->dev,
		vfs_device->dev_numeber, vfs_device, strname);
	if (!IS_ERR(vfs_device->device)) {
		ret = sysfs_create_group(&vfs_device->device->kobj,
			&lpc5400x_vfs_attr_group);
		if (ret != 0)
			device_unregister(vfs_device->device);
	} else
		ret = PTR_ERR(vfs_device->device);

	/*pr_err("lpc5400x_create_sysfsnode %s, vfs = %p, device = %p",
		strname, vfs_device, vfs_device->device);*/

	vfs_device->busy = false;
#ifdef LPC5400X_VFS_AIO
	vfs_device->aio_tmpbuf = NULL;
	vfs_device->aio_tmpbufsize = 0;
	vfs_device->aio_iocb = NULL;
	printk("lpc5400x_create_sysfsnode\n");
	INIT_WORK(&vfs_device->aio_work, lpc5400x_vfs_aio_task);
#endif

	if (vfs_device_entities[devicename].prepare_private_data)
		vfs_device_entities[devicename].prepare_private_data(
			vfs_device->device,
			&vfs_device->private);

	*vfs_device_p = vfs_device;

	return ret;
exit1:
	class_destroy(vfs_device->vfs_class);
exit2:
	unregister_chrdev_region(vfs_device->dev_numeber, 1);
	kfree(vfs_device->transbuffer);
	kfree(vfs_device);

	return ret;
}

static void lpc5400x_destory_sysfsnode
	(struct _lpc5400x_vfs_devices *vfs_device)
{
	if (vfs_device_entities[vfs_device->name].clean_up)
		vfs_device_entities[vfs_device->name].clean_up(
			vfs_device->private);

#ifdef LPC5400X_VFS_AIO
	kfree(vfs_device->aio_tmpbuf);
#endif
	class_destroy(vfs_device->vfs_class);
	unregister_chrdev_region(vfs_device->dev_numeber, 1);
	kfree(vfs_device->transbuffer);
	kfree(vfs_device);
}

//zhangji add for proc start
static int show_mag_id(struct seq_file *m, void *v)
{

	seq_printf(m, "%s\n", sensor_proc_info[2]);

	return 0;
}


static int mag_id_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, show_mag_id, PDE_DATA(inode));
}

static const struct file_operations mag = {
	.open		= mag_id_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void create_mag_proc_file(struct _lpc5400x_vfs* vfs)
{
	struct proc_dir_entry *accel_proc_file;
	accel_proc_file = proc_create_data("driver/mag_id", 0664, NULL, &mag, vfs);
	if (accel_proc_file) {
		printk(KERN_INFO "mag_proc_file create success!\n");
	}else{
		printk(KERN_INFO "mag_proc_file create failed!\n");
	}
}

static int show_light_id(struct seq_file *m, void *v)
{
	
	seq_printf(m, "%s\n", sensor_proc_info[3]);

	return 0;
}


static int light_id_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, show_light_id, PDE_DATA(inode));
}

static const struct file_operations light = {
	.open		= light_id_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void create_light_proc_file(struct _lpc5400x_vfs* vfs)
{
	struct proc_dir_entry *accel_proc_file;
	accel_proc_file = proc_create_data("driver/light_id", 0664, NULL, &light, vfs);
	if (accel_proc_file) {
		printk(KERN_INFO "light_proc_file create success!\n");
	}else{
		printk(KERN_INFO "light_proc_file create failed!\n");
	}
}


static int show_accel_id(struct seq_file *m, void *v)
{
	struct _lpc5400x_vfs *vfs = m->private;
	struct i2c_client *client = i2c_verify_client(vfs->dev);
	char *accel_id;
	int   sensor_id = 0;

	sensor_id = lpc5400x_report_sensorlistinfo((LPCSH_SENSOR_ID_ACCELEROMETER+20),client);
	if(sensor_id == 0x0D)
		accel_id = sensor_proc_info[0];
	else if(sensor_id == 0x1C)
		accel_id = sensor_proc_info[1];
	else 
		accel_id = sensor_proc_info[4];
	
	seq_printf(m, "%s\n", accel_id);

	return 0;
}


static int accel_id_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, show_accel_id, PDE_DATA(inode));
}

static const struct file_operations accels = {
	.open		= accel_id_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void create_accel_proc_file(struct _lpc5400x_vfs* vfs)
{
	struct proc_dir_entry *accel_proc_file;
	accel_proc_file = proc_create_data("driver/accel_id", 0664, NULL, &accels, vfs);
	if (accel_proc_file) {
		printk(KERN_INFO "accel_proc_file create success!\n");
	}else{
		printk(KERN_INFO "accel_proc_file create failed!\n");
	}
}

static void lpc5400x_create_proc_node (struct _lpc5400x_vfs* vfs)
{
	create_accel_proc_file(vfs);
	create_mag_proc_file(vfs);
	create_light_proc_file(vfs);
}
//zhangji add for proc end

int lpc5400x_create_vfsdevice(
	void **pHandle,
	struct device *parent,
	struct vfs_callback_t *callback)
{
	struct _lpc5400x_vfs *vfs =
		kzalloc(sizeof(struct _lpc5400x_vfs), GFP_KERNEL);
	int ret = 0, i = 0;

	*pHandle = NULL;

	if (vfs == NULL)
		return -ENOMEM;

	vfs->dev = parent;
	vfs->notifier.vfs_notify = callback->vfs_notify;

	for (i = 0; i < SENSORHUB_DEVICE_NUM; i++) {
		ret = lpc5400x_create_sysfsnode(vfs, &vfs->devices[i], i);
		if (ret)
			break;
	}

	//zhangji add for proc node
	lpc5400x_create_proc_node(vfs);
	*pHandle = vfs;

	memset(commandbuffer, 0x00, sizeof(commandbuffer));

	return 0;
}

int lpc5400x_destory_vfsdevice(void *handle)
{
	struct _lpc5400x_vfs *vfs = handle;
	int i = 0;

	while (i < SENSORHUB_DEVICE_NUM && vfs->devices[i]) {
		lpc5400x_destory_sysfsnode(vfs->devices[i]);
		++i;
	}

	kfree(handle);
	return 0;
}

/*
static void nxp_mcu_reset_after_calibration(struct lpc5400x_platform_data *pPlatformData)
{
	gpio_direction_output(pPlatformData->wakeup_gpio, 1);
	gpio_direction_output(pPlatformData->reset_gpio, 1);

	gpio_set_value(pPlatformData->reset_gpio, 0);
	msleep(3);
	gpio_set_value(pPlatformData->reset_gpio, 1);
	msleep(450);
}
*/


int lpc5400x_vfsdevice_command_handler(
	struct _lpc5400x_vfs_devices *vfsdevice,
	const char *pData,
	size_t size,
	bool fromUser)
{
	int transize = 0, txpos = 0, trannum = 0, ret = 0, retry = 0;
	int blocknum = 0;
	bool bboot = false, bcheckimage = false;
	struct lpc5400x_platform_data *pPlatformData =
		(struct lpc5400x_platform_data *)vfsdevice->pPlatformData;
	struct i2c_client *client = i2c_verify_client(vfsdevice->parent->dev);

	pr_err("lpc5400x_vfsdevice_command_handler vfsdevice->name: %d, size = %d\n",
		vfsdevice->name,
		(int)size);

    //lpc5400x_romVersion(vfsdevice->parent->dev, pPlatformData->irq_gpio);

	//pr_err("lpc5400x_romVersion \n\n");

	do {
		if(size - txpos > LPC5400X_MAX_I2C_TRANSFER)
			transize = LPC5400X_MAX_I2C_TRANSFER;
		else
			transize = size - txpos;
		if (fromUser) {
			if (copy_from_user(vfsdevice->transbuffer, pData + txpos, transize))
				return -EFAULT;
		} else
			memcpy(vfsdevice->transbuffer, pData + txpos, transize);

		switch (vfsdevice->name) {
			#ifdef CONFIG_LPC5400X_SENSORHUB_VOICE_TRIGGER	
		case SENSORHUB_UDTDATA:
			bboot = true;
			blocknum = (SENSORY_DATA_OFFSET - 0x8000) / SL_FLASH_BLOCK_SZ;
			if(trannum == SL_FLASH_BLOCK_SZ / LPC5400X_MAX_I2C_TRANSFER) {
				trannum = 0;
				blocknum++;
			}
			/* Need to check the ret and response from Niobe */

			ret = lpc5400x_writeSubBlock(vfsdevice->parent->dev,
				vfsdevice->transbuffer, transize, blocknum, pPlatformData->wakeup_gpio);
			trannum++;
			txpos += transize;
			break;
			#endif

		case SENSORHUB_FWKEY:
			if (size != LPC5400X_KEY_LENGTH && size != LPC5400X_DUMMY_KEY_LENGTH) {
				pr_err("Invalid crypto %d", (int)size);
				return -EINVAL;
			}

			if (size == LPC5400X_KEY_LENGTH) {
#if 0
				ret = lpc5400x_disable_secure(vfsdevice->parent->dev, pPlatformData->gpio);
				if (ret < 0) {
					dev_warn(vfsdevice->parent->dev,
						"lpc5400x_disable_secure error %d", ret);
					return ret;
				}
#endif
				ret = lpc5400x_enable_secure(
					vfsdevice->parent->dev,
					vfsdevice->transbuffer,
					size,
					pPlatformData->wakeup_gpio);

				if (ret < 0) {
					dev_warn(
						vfsdevice->parent->dev,
						"lpc5400x_enable_secure error %d", ret);
					return ret;
				}
			} else {
				if (vfsdevice->transbuffer[0] == LPC5400X_DUMMY_KEY) {
					lpc5400x_disable_secure(
						vfsdevice->parent->dev,
						pPlatformData->wakeup_gpio);
					dev_warn(
						vfsdevice->parent->dev,
						"Dangerous!! You disable the secure firmware");
				}
			}
			txpos += transize;
			break;

		case SENSORHUB_FIRMWAREUPDATE:
			bboot = true;
			bcheckimage = true;
			/* Check the F/W version here to control the update process*/
			if(trannum == SL_FLASH_BLOCK_SZ / LPC5400X_MAX_I2C_TRANSFER) {
				trannum = 0;
				blocknum++;
			}

			ret = lpc5400x_writeSubBlock(vfsdevice->parent->dev,
				vfsdevice->transbuffer,
				transize,
						blocknum, pPlatformData->wakeup_gpio);
			/* Need to check the ret and response from Niobe */
			if (ret < 0) {
				if (retry == FWUPDATE_TRANSFER_RETRY_MAX) {
					dev_warn(vfsdevice->parent->dev,
						"Failed to upgrade firmware, bus error");
					break;
				} else {
					retry++;
					continue;
				}
			}
			retry = 0;
			trannum++;
			txpos += transize;
			break;

        #ifdef CONFIG_LPC5400X_SENSORHUB_RAMDUMP
		case SENSORHUB_RAMDUMP:
			if (vfs_device_entities[vfsdevice->name].write_handler)
				txpos = vfs_device_entities[vfsdevice->name].write_handler(
					vfsdevice->parent->dev,
					vfsdevice->transbuffer,
					transize,
					vfsdevice->private);
			break;
		#endif	
		
		case SENSORHUB_LOGGING:
			if (vfs_device_entities[vfsdevice->name].write_handler)
				txpos = vfs_device_entities[vfsdevice->name].write_handler(
					vfsdevice->parent->dev,
					vfsdevice->transbuffer,
					transize,
					vfsdevice->private);
			break;
		case SENSORHUB_CALIBRATION:
			if (size > LPCSH_SENSORCALIBRATION_DATA_SIZE) {
				pr_err("lpc5400x Invalid Calibration %d", (int)size);
				ret = -EINVAL;
				break;
			}

			printk("lpc5400x_vfsdevice_command_handler SENSORHUB_CALIBRATION size:%d\n", (int)size);

			lpc5400x_startCalibration(
				vfsdevice->parent->dev,
				vfsdevice->transbuffer,
				size);
			//msleep(500);
			//nxp_mcu_reset_after_calibration(pPlatformData);
			//lpc5400x_startCalibration(vfsdevice->parent->dev, vfsdevice->transbuffer, size);
			txpos = (int)size;
			break;

#ifdef CONFIG_LPC5400X_SENSORHUB_AUDIO_SENSE
		case SENSORHUB_AUDIOSENSE:
		{
			/* check params length. */
			if (size != LPC5400X_AUDIOSENSE_DMIC_CONFIGURATION_INFO_LENGHT) {
				pr_err("Invalid DMIC clock params length %d\n", (int)size);
				ret = -EINVAL;
				break;
			}

			/*when we write I2C command sucessed, will return 0.*/
			ret = lpc5400x_setDmicClock(
				vfsdevice->parent->dev,
				vfsdevice->transbuffer,
				size);
			if (0 == ret)
				txpos = (int)size;
			break;
		}
#endif
		case SENSORHUB_SENSORDUMP:
		{
			if ((*vfsdevice->transbuffer) == 'w'){
				ret = lpc5400x_write_sensor_register(vfsdevice->parent->dev, (void*)vfsdevice->transbuffer, (int)size);
				pr_err("lpc5400x_write_sensor_register ret = %d\n", ret);
				if (0 == ret)
					txpos = (int)size;
			}
			else{
				ret = 0;
				commandbuffer[0] = 1;
				memcpy(&commandbuffer[1], vfsdevice->transbuffer, 4);
				txpos = (int)size;
				pr_err("lpc5400x_write_sensor_register before read \n");
			}

			break;
		}
		default:
			ret = -ENOTSUPP;
			break;
		}

		if (ret < 0) {
			pr_debug("Failed to perform command, Bus error ret = %d\n", ret);
			bboot = false;
			break;
		}
	} while ((txpos < (int)size) && (ret >= 0));

	if (ret >= 0) {
		if (bcheckimage) {
			if (!lpc5400x_checkImage(
					vfsdevice->parent->dev,
					pPlatformData->wakeup_gpio)) {
				dev_err(
					vfsdevice->parent->dev,
					"check image failure boot is cancelled\n");
				return -EINVAL;
			}
		}

		if (bboot) {
			pr_err("blocks = %d, pos =%d", blocknum, txpos);
			pr_err("Download Complete, boot the device");

			
			lpc5400x_bootFirmware(vfsdevice->parent->dev, pPlatformData->wakeup_gpio);

			if (vfsdevice->name == SENSORHUB_FIRMWAREUPDATE)
				vfsdevice->parent->notifier.vfs_notify(vfsdevice->parent->dev,
					VFS_NOTIFY_UPDATE_COMPLETE, &bboot);

			msleep(20);

			//gpio_direction_output(pPlatformData->wakeup_gpio, 1);
			if(client->irq){
				enable_irq(client->irq);  /* enable irq*/
 				irq_set_irq_wake(client->irq, 1);  /*enbale irq*/     				
			}
            {
                /* only for debug */
                uint8_t fwverion[LPC5400X_VERSION_LENGTH] = {0xFF, 0xFF};
                
                lpc5400x_getVersion(vfsdevice->parent->dev, &fwverion[0], &fwverion[1]);
                printk("%s major_version.minor_version:%d.%d\n", __func__, fwverion[0], fwverion[1]);
            }

			//gpio_direction_input(pPlatformData->irq_gpio);
		}

		pr_err("lpc5400x_vfs_write: Written %d", txpos);
		return txpos;
	}

	return ret;
}


