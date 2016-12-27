/* drivers/android/bootloader_logger.c
 *
 * Copyright (C) 2008-2013 li.xingyuan@zte.com.cn.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/console.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/of.h>
#include <linux/of_address.h>

struct bootloader_logger_buffer {
	uint32_t    sig;
	uint32_t    start;
	uint32_t    size;
	uint8_t     data[0];
};

#define bootloader_logger_SIG (0x43474244) /* DBGC */

struct bootloader_logger {
	char *tag;
	char *old_log;
	size_t old_log_size;
	struct bootloader_logger_buffer *buffer;
	size_t buffer_size;
};

#if 0
static char *bootloader_logger_old_log;
static size_t bootloader_logger_old_log_size;

static struct bootloader_logger_buffer *bootloader_logger_buffer;
static size_t bootloader_logger_buffer_size;
#endif

static struct bootloader_logger logger_main;
static struct bootloader_logger logger_last;

#if 0
static void notrace
bootloader_logger_write(const void *s, unsigned int count)
{
	int rem;
	struct bootloader_logger *logger = &logger_main;
	struct bootloader_logger_buffer *buffer = logger->buffer;
	static DEFINE_SPINLOCK(ram_lock);
	unsigned long flags;

	spin_lock_irqsave(&ram_lock, flags);
	if (unlikely(count > logger->buffer_size)) {
		s += count - logger->buffer_size;
		count = logger->buffer_size;
	}
	rem = logger->buffer_size - buffer->start;
	if (rem < count) {
		memcpy(buffer->data + buffer->start, s, rem);
		s += rem;
		count -= rem;
		buffer->start = 0;
		buffer->size = logger->buffer_size;
	}
	memcpy(buffer->data + buffer->start, s, count);

	buffer->start += count;
	if (buffer->size < logger->buffer_size)
		buffer->size += count;
	spin_unlock_irqrestore(&ram_lock, flags);
}
#endif

static void __init
bootloader_logger_save_old(struct bootloader_logger *logger, char *dest)
{
	struct bootloader_logger_buffer *buffer = logger->buffer;
	size_t old_log_size = buffer->size;
	size_t total_size = old_log_size;

	if (dest == NULL) {
		dest = kmalloc(total_size, GFP_KERNEL);
		if (dest == NULL) {
			printk(KERN_ERR
			       "bootloader_logger_%s: failed to allocate buffer\n",
			       logger->tag);
			return;
		}
	}

	logger->old_log = dest;
	logger->old_log_size = total_size;
	memcpy(logger->old_log,
	       &buffer->data[buffer->start], buffer->size - buffer->start);
	memcpy(logger->old_log + buffer->size - buffer->start,
	       &buffer->data[0], buffer->start);
}

static int bootloader_logger_init(struct bootloader_logger *logger, char *old_buf)
{
	struct bootloader_logger_buffer *buffer = logger->buffer;

	printk(KERN_INFO "bootloader_logger_%s: buffer=%p, size=0x%zx\n", 
			logger->tag, logger->buffer, logger->buffer_size);
	
	if (buffer->sig == bootloader_logger_SIG) {
		if (buffer->size > logger->buffer_size
		    || buffer->start > buffer->size)
			printk(KERN_INFO "bootloader_logger_%s: found existing invalid "
			       "buffer, size %d, start %d\n",
			       logger->tag, buffer->size, buffer->start);
		else {
			printk(KERN_INFO "bootloader_logger_%s: found existing buffer, "
			       "size %d, start %d\n",
			       logger->tag, buffer->size, buffer->start);
			if (buffer->size > 0)
				bootloader_logger_save_old(logger, old_buf);
		}
	} else {
		printk(KERN_INFO "bootloader_logger_%s: no valid data in buffer "
		       "(sig = 0x%08x)\n", logger->tag, buffer->sig);
	}

	/* Erase old bootloader log ?? */
	/*
	buffer->sig = 0;
	buffer->start = 0;
	buffer->size = 0;
	*/
	
	return 0;
}

static int bootloader_logger_dt_init(struct platform_device *pdev)
{
	struct device_node *bootloader_logger_pnode = NULL;
	struct resource *res = pdev->resource;

	
	bootloader_logger_pnode = of_parse_phandle(pdev->dev.of_node,
			"linux,contiguous-region", 0);
	if (!bootloader_logger_pnode) {
		dev_err(&pdev->dev, "memory is not reserved for %s\n", pdev->name);
		return -ENOMEM;
	} else {
		const u32 *addr;
		u64 len;

		addr = of_get_address(bootloader_logger_pnode, 0, &len, NULL);
		if (!addr) {
			dev_err(&pdev->dev, "fail to parse the bootloader logger memory\n");
			of_node_put(bootloader_logger_pnode);
			return -ENOMEM;
		}
		res->start = of_read_ulong(addr, 2);;
		res->end = (size_t)(res->start + len) - 1;
		of_node_put(bootloader_logger_pnode);
	}

	pdev->num_resources = 1;
	return 0;

}


static int bootloader_logger_driver_probe(struct platform_device *pdev)
{
	struct resource *res = pdev->resource;
	size_t start;
	size_t buffer_size;
	void *buffer;

	if(bootloader_logger_dt_init(pdev) != 0 ){
		dev_err(&pdev->dev, "no memory for bootloader logger\n");
		return -ENOMEM;
	}

	if (res == NULL || pdev->num_resources != 1 ||
	    !(res->flags & IORESOURCE_MEM)) {
		printk(KERN_ERR "bootloader_logger: invalid resource, %p %d flags "
		       "%lx\n", res, pdev->num_resources, res ? res->flags : 0);
		return -ENXIO;
	}
	buffer_size = res->end - res->start + 1;
	start = res->start;
	printk(KERN_INFO "bootloader_logger: got buffer at %zx, size %zx\n",
	       start, buffer_size);
	buffer = ioremap_cache(res->start, buffer_size);
	if (buffer == NULL) {
		printk(KERN_ERR "bootloader_logger: failed to map memory\n");
		return -ENOMEM;
	}
	printk(KERN_INFO "bootloader_logger: buffer mapped at %p\n", buffer);

	logger_main.tag = "main";
	logger_main.buffer = buffer;
	logger_main.buffer_size = buffer_size / 2 - sizeof(struct bootloader_logger_buffer);
	
	logger_last.tag = "last";
	logger_last.buffer = buffer + buffer_size / 2;
	logger_last.buffer_size = buffer_size / 2 - sizeof(struct bootloader_logger_buffer);

	bootloader_logger_init(&logger_main, NULL/* allocate */);
	bootloader_logger_init(&logger_last, NULL/* allocate */);

	return 0;
}

static const struct of_device_id of_bootloader_logger_match[] = {
	{ .compatible = "zte,bootloader_logger", },
	{},
};
MODULE_DEVICE_TABLE(of, of_bootloader_logger_match);


static struct platform_driver bootloader_logger_driver = {
	.probe = bootloader_logger_driver_probe,
	.driver		= {
		.name	= "bootloader_logger",
		.of_match_table = of_match_ptr(of_bootloader_logger_match),
	},
};

static int __init bootloader_logger_module_init(void)
{
	return platform_driver_register(&bootloader_logger_driver);
}
device_initcall(bootloader_logger_module_init);

static ssize_t bootloader_logger_main_read_old(struct file *file, char __user *buf,
				    size_t len, loff_t *offset)
{
	loff_t pos = *offset;
	ssize_t count;

	if (pos >= logger_main.old_log_size)
		return 0;

	count = min(len, (size_t)(logger_main.old_log_size - pos));
	if (copy_to_user(buf, logger_main.old_log + pos, count))
		return -EFAULT;

	*offset += count;
	return count;
}

static const struct file_operations bootloader_logger_main_file_ops = {
	.owner = THIS_MODULE,
	.read = bootloader_logger_main_read_old,
};

static ssize_t bootloader_logger_last_read_old(struct file *file, char __user *buf,
				    size_t len, loff_t *offset)
{
	loff_t pos = *offset;
	ssize_t count;

	if (pos >= logger_last.old_log_size)
		return 0;

	count = min(len, (size_t)(logger_last.old_log_size - pos));
	if (copy_to_user(buf, logger_last.old_log + pos, count))
		return -EFAULT;

	*offset += count;
	return count;
}

static const struct file_operations bootloader_logger_last_file_ops = {
	.owner = THIS_MODULE,
	.read = bootloader_logger_last_read_old,
};

static void bootloader_logger_dump_old(struct bootloader_logger *logger)
{
	int i;
	char *p = logger->old_log;
	
	if (!p)
		return;
	
	printk(KERN_INFO "*****************************"
			"bootloader_log_%s begin"
			"*****************************\n", logger->tag);
	for (i = 0; i < logger->old_log_size; i++) {
		if (logger->old_log[i] == '\0')
			logger->old_log[i] = ' ';
		if (logger->old_log[i] == '\n') {
			logger->old_log[i] = '\0';
			printk(KERN_INFO "bootloader_log_%s: %s\n", logger->tag, p);
			logger->old_log[i] = '\n';
			p = &logger->old_log[i + 1];
		}
	}
	printk(KERN_INFO "******************************"
			"bootloader_log_%s end"
			"******************************\n", logger->tag);
}

static int __init bootloader_logger_late_init(void)
{
	struct proc_dir_entry *entry;

	if (logger_main.old_log != NULL) {
		entry = proc_create("bootloader_log", S_IFREG | S_IRUGO, NULL, &bootloader_logger_main_file_ops);
		if (!entry) {
			printk(KERN_ERR "bootloader_logger: failed to create proc entry\n");
			kfree(logger_main.old_log);
			logger_main.old_log = NULL;
			return 0;
		}
		//entry->proc_fops = &bootloader_logger_main_file_ops;
		//entry->size = logger_main.old_log_size;
		bootloader_logger_dump_old(&logger_main);
	}

	if (logger_last.old_log != NULL) {
		entry = proc_create("bootloader_log_last", S_IFREG | S_IRUGO, NULL, &bootloader_logger_last_file_ops);
		if (!entry) {
			printk(KERN_ERR "bootloader_log_last: failed to create proc entry\n");
			kfree(logger_last.old_log);
			logger_last.old_log = NULL;
			return 0;
		}
		//entry->proc_fops = &bootloader_logger_last_file_ops;
		//entry->size = logger_last.old_log_size;
		bootloader_logger_dump_old(&logger_last);
	}

	return 0;
}
late_initcall(bootloader_logger_late_init);
