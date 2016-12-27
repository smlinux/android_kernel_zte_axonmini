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

#include "lpc5400x_voicetrigger.h"

#define DEVICE_NAME "lpc5400x_vt"
#define MAX_COMMAND_LENGTH (256)

char *demo_command_list[11] = {
	"pause music",
	"stop music",
	"play music",
	"previous song",
	"next song",
	"song information",
	"equalizer",	   
	"cancel",
	"settings",
	"help me",
	"hello blue genie"
};

struct _lpc5400x_vt {
	dev_t dev_numeber;
	struct cdev vt_dev;
	struct class *vt_class;
	wait_queue_head_t vt_wq;
	struct kfifo vt_cmdfifo;
	struct mutex mutex;
	bool isopen;
};

static char *lpc_vt_make_cmd(int mode, int index)
{
	if (mode == 0)	/* fix mode */
		return demo_command_list[index];
	else if (mode == 1)
		return demo_command_list[jiffies % 11];
	else
		return "unsupported mode";
}

static int lpc_vt_open(struct inode *pInode, struct file *flip)
{
	struct _lpc5400x_vt *vt;

	vt = container_of(pInode->i_cdev, struct _lpc5400x_vt, vt_dev);

	flip->private_data = vt;

	vt->isopen = true;

	pr_debug("lpc_vt_open: vt = %p\n", vt);

	return 0;
}

static unsigned int lpc_vt_poll(
	struct file *flip,
	struct poll_table_struct *wait)
{
	struct _lpc5400x_vt *vt = flip->private_data;
	unsigned int datasize, ret;

	poll_wait(flip, &vt->vt_wq, wait);

	datasize = kfifo_len(&vt->vt_cmdfifo);
	pr_debug("lpc_vt_poll... data size = %d\n", datasize);

	if (datasize > 0)
		ret = POLLIN | POLLRDNORM;
	else
		ret = 0;

	return ret;
}

static ssize_t lpc_vt_read(
	struct file *flip,
	char __user *pUserMem,
	size_t copysize,
	loff_t *pOffset)
{
	struct _lpc5400x_vt *vt = flip->private_data;
	int ret, val = 0;
	char *pCmd = NULL;	/* lpc_vt_make_cmd(1); */

	/*
	   ret = wait_event_interruptible(vt->vt_wq, 1);
	   if (ret)
	   {
	   return 0;
	   }
	 */
	mutex_lock(&vt->mutex);

	ret = kfifo_out(&vt->vt_cmdfifo, &pCmd, sizeof(char *));

	mutex_unlock(&vt->mutex);

	pr_debug("lpc_vt_read..., pCmd = %s, ret = %d\n", pCmd, ret);
	val = strlen(pCmd);

	ret = copy_to_user(pUserMem, pCmd, val + 1);

	pr_debug("lpc_vt_read:ret = %d", ret);

	return val + 1;
}

static int lpc_vt_release(struct inode *pInode, struct file *flip)
{
	struct _lpc5400x_vt *vt = flip->private_data;

	vt->isopen = false;
	kfifo_reset(&vt->vt_cmdfifo);

	return 0;
}

const struct file_operations vt_ops = {
	.owner		= THIS_MODULE;
	.open		= lpc_vt_open;
	.poll		= lpc_vt_poll;
	.read		= lpc_vt_read;
	.release	= lpc_vt_release;
};


int lpc_voicetrigger_init(void **pHandle, void *client, void *pInitCfg)
{
	struct _lpc5400x_vt *vt =
		kzalloc(sizeof(struct _lpc5400x_vt), GFP_KERNEL);
	int ret = 0;

	*pHandle = NULL;

	if (vt == NULL)
		return -ENOMEM;

	ret = kfifo_alloc(&vt->vt_cmdfifo, MAX_COMMAND_LENGTH, GFP_KERNEL);
	if (ret) {
		pr_warn("lpc_voicetrigger_init kfifo allocation failed...");
		kfree(vt);
		return ret;
	}


	ret = alloc_chrdev_region(&vt->dev_numeber, 0, 1, DEVICE_NAME);
	if (ret < 0) {
		kfifo_free(&vt->vt_cmdfifo);
		kfree(vt);
		return ret;
	}

	vt->vt_class = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(vt->vt_class)) {
		pr_warn("Lpc5400x Voice Command Bad Class create...");
		goto exit2;
	}

	cdev_init(&vt->vt_dev, &vt_ops);

	vt->vt_dev.owner = THIS_MODULE;

	ret = cdev_add(&vt->vt_dev, vt->dev_numeber, 1);
	if (ret) {
		pr_warn("Lpc5400x Voice Command fail to add cdev: %d", ret);
		goto exit1;
	}

	device_create(vt->vt_class, NULL, vt->dev_numeber, NULL, DEVICE_NAME);

	init_waitqueue_head(&vt->vt_wq);

	vt->isopen = false;
	mutex_init(&vt->mutex);

	pr_debug("lpc_voicetrigger_init vt = %p", vt);

	*pHandle = vt;

	return 0;
exit1:
	class_destroy(vt->vt_class);
exit2:
	unregister_chrdev_region(vt->dev_numeber, 1);
	kfifo_free(&vt->vt_cmdfifo);
	kfree(vt);
	return ret;
}

int lpc_voicetrigger_cleanup(void *handle)
{
	struct _lpc5400x_vt *vt = handle;

	class_destroy(vt->vt_class);
	cdev_del(&vt->vt_dev);
	unregister_chrdev_region(vt->dev_numeber, 1);
	kfifo_free(&vt->vt_cmdfifo);
	kfree(handle);
	return 0;
}

int lpc_voicetrigger_newCommand(void *handle, int index)
{
	struct _lpc5400x_vt *vt = handle;
	char  *pCmd;

	/* int length = 0; */

	pr_debug(
		"lpc_voicetrigger_newCommand index = %d, isopen = %d",
		index,
		vt->isopen);

	if (vt->isopen) {
		pCmd = lpc_vt_make_cmd(0, index);
		/* length = strlen(pCmd) + 1; */

		mutex_lock(&vt->mutex);
		kfifo_in(&vt->vt_cmdfifo, &pCmd, sizeof(char *));
		mutex_unlock(&vt->mutex);

		pr_debug("lpc_voicetrigger_newCommand put %s in FIFO", pCmd);

		/*  wake_up_interruptible(&vt->vt_wq); */
	}
	return 0;
}

