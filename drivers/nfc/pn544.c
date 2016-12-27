/*
 * Copyright (C) 2010 Trusted Logic S.A.
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
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/nfc/pn544.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <asm-generic/gpio.h>
#include <linux/mfd/pm8xxx/pm8921.h>

#include <linux/string.h>	/* zte-ccb-20130128 */
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
//zte-modify by zuojianfang
//#include <linux/pn544.h>
//#include <plat/gpio-core.h>
//#include <plat/gpio-cfg.h>
//#include <plat/gpio-cfg-helpers.h>
#include <linux/clk.h>
#include <linux/wakelock.h>//FIXED Screen off transcation proformance issue

#include <linux/proc_fs.h>
#include <linux/seq_file.h>

//#define pr_err NFC_DBG_MSG
//#define pr_debug NFC_DBG_MSG
//#define pr_warning NFC_DBG_MSG
#define DBG_MODULE 0
#if DBG_MODULE
#define NFC_DBG_MSG(fmt,msg...) printk(KERN_ERR " %s: "fmt,__func__,##msg);
#define NFC_ERR_MSG(fmt,msg...) printk(KERN_ERR " %s: "fmt,__func__,##msg);
#else
#define NFC_DBG_MSG(fmt,msg...)
#define NFC_ERR_MSG(fmt,msg...)
#endif
//#define NFC_ERR_MSG(fmt,msg...) printk(KERN_ERR " %s: "fmt,__func__,##msg);

#define MAX_BUFFER_SIZE	512

#define PN544_DRIVER_NAME         "pn544"

#define NFC_RF_CLK_FREQ			(19200000)



struct pn544_dev	{
	wait_queue_head_t	read_wq;
	struct mutex		read_mutex;
	struct mutex		write_mutex;
	struct i2c_client	*client;
	struct miscdevice	pn544_device;
	unsigned int 		ven_gpio;
	unsigned int 		firm_gpio;
	unsigned int		irq_gpio;
	unsigned int		vddio_en;
	unsigned int 		clk_gpio;
	bool			irq_enabled;
	spinlock_t		irq_enabled_lock;
};

static struct pn544_dev    *pn544_dev = NULL;
static struct clk *bb_clk2;
bool          clk_run = false;
static struct wakeup_source *nfc_wake_lock;
static int pn544_regulator_enable(bool enable);

/*add nfc id information begin*/

static char nfc_id[51] = "NXP-PN65T-NA-NA-NA";
static int nfc_id_read_proc(struct seq_file *m, void *v)
{
	return seq_printf(m, "%s\n", nfc_id);
}

static int nfc_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, nfc_id_read_proc, NULL);
}

static const struct file_operations nfc_proc_fops = {
	.open		= nfc_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static struct proc_dir_entry *nfc_id_proc_file;

static void create_nfc_info_proc_file(void)
{
  nfc_id_proc_file = proc_create("driver/nfc_id", 0777, NULL,&nfc_proc_fops);
  NFC_ERR_MSG("goes to create_nfc_info_proc_file\n");
  if (nfc_id_proc_file) {
   } else
	NFC_ERR_MSG( "proc file create failed!\n");
}

static void remove_nfc_info_proc_file(void)
{
	NFC_ERR_MSG("goes to remove_nfc_info_proc_file\n");
	if(nfc_id_proc_file){
		remove_proc_entry("driver/nfc_id", NULL);
		nfc_id_proc_file = NULL;
	}	
}
/*add nfc id information end*/

static void pn544_disable_irq(struct pn544_dev *pn544_dev)
{
	unsigned long flags;
    NFC_DBG_MSG("%s start\n", __func__);
	spin_lock_irqsave(&pn544_dev->irq_enabled_lock, flags);
	if (pn544_dev->irq_enabled) {
		disable_irq_nosync(pn544_dev->client->irq);
		pn544_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&pn544_dev->irq_enabled_lock, flags);
	NFC_DBG_MSG("%s end\n", __func__);
}

static irqreturn_t pn544_dev_irq_handler(int irq, void *dev_id)
{
	struct pn544_dev *pn544_dev = dev_id;
    NFC_DBG_MSG("start\n");
	if (!gpio_get_value(pn544_dev->irq_gpio)) {
		return IRQ_HANDLED;
	}
	pn544_disable_irq(pn544_dev);

	/* Wake up waiting readers */
	wake_up(&pn544_dev->read_wq);
    NFC_DBG_MSG("end\n");
	return IRQ_HANDLED;
}

static ssize_t pn544_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn544_dev *pn544_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
	int ret;
	//int i;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;
	NFC_DBG_MSG("%s start\n");
	
	__pm_wakeup_event(nfc_wake_lock, 5 * MSEC_PER_SEC);
	mutex_lock(&pn544_dev->read_mutex);

	if (!gpio_get_value(pn544_dev->irq_gpio)) {
		if (filp->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto fail;
		}

		pn544_dev->irq_enabled = true;
		enable_irq(pn544_dev->client->irq);
		NFC_DBG_MSG("wait_event_interruptible start\n");
		ret = wait_event_interruptible(pn544_dev->read_wq,
				gpio_get_value(pn544_dev->irq_gpio));

		pn544_disable_irq(pn544_dev);
        NFC_DBG_MSG("wait_event_interruptible end \n");
		if (ret)
			goto fail;
	}

	/* Read data */
	ret = i2c_master_recv(pn544_dev->client, tmp, count);
	NFC_DBG_MSG("i2c_master_recv end \n");
	mutex_unlock(&pn544_dev->read_mutex);

	if (ret < 0) {
		pr_err("%s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
	}
	if (ret > count) {
		pr_err("%s: received too many bytes from i2c (%d)\n",
			__func__, ret);
		return -EIO;
	}
	if (copy_to_user(buf, tmp, ret)) {
		pr_warning("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}
	//NFC_ERR_MSG("reading %zu bytes.\n",count);
	/*printk("NFCC->DH:");
	for(i = 0; i < ret; i++){
		printk(" %02X", tmp[i]);
	}
	printk("\n");*/
	NFC_DBG_MSG("%s end\n");
	return ret;

fail:
	mutex_unlock(&pn544_dev->read_mutex);
	return ret;
}

static ssize_t pn544_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn544_dev  *pn544_dev;
	char tmp[MAX_BUFFER_SIZE];
	int ret;
	//int i;

	pn544_dev = filp->private_data;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;
	
	NFC_DBG_MSG("%s start\n", __func__);

	if (copy_from_user(tmp, buf, count)) {
		pr_err("%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	//NFC_ERR_MSG("writing %zu bytes.\n", count);
	/*printk("DH->NFCC:");
	for(i = 0; i < count; i++){
		printk(" %02X", tmp[i]);
	}
	printk("\n");*/
	/* Write data */
	ret = i2c_master_send(pn544_dev->client, tmp, count);
	if (ret != count) {
		pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
	}
	NFC_DBG_MSG("%s end\n");
	return ret;
}

static int pn544_dev_open(struct inode *inode, struct file *filp)
{
	struct pn544_dev *pn544_dev = container_of(filp->private_data,
						struct pn544_dev,
						pn544_device);

	filp->private_data = pn544_dev;
	pr_debug("%s : %d,%d\n", __func__, imajor(inode), iminor(inode));

	return 0;
}

static long pn544_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct pn544_dev *pn544_dev = filp->private_data;
	int ret = 0;

	switch (cmd) {
	case PN544_SET_PWR:
		if (arg == 2) {
			/* power on with firmware download (requires hw reset)
			 */
			NFC_ERR_MSG("power on with firmware\n");
			gpio_set_value(pn544_dev->ven_gpio, 1);
			gpio_set_value(pn544_dev->firm_gpio, 1);
			msleep(10);
			gpio_set_value(pn544_dev->ven_gpio, 0);
			msleep(50);
			gpio_set_value(pn544_dev->ven_gpio, 1);
			msleep(10);
		} else if (arg == 0) {
			/* power off */
			NFC_ERR_MSG("power off\n");
			gpio_set_value(pn544_dev->firm_gpio, 0);
			gpio_set_value(pn544_dev->ven_gpio, 0);
			msleep(10);
			irq_set_irq_wake(pn544_dev->client->irq,0);
			if(clk_run == true){
			   msleep(10);
			   clk_disable_unprepare(bb_clk2);
			   clk_run = false;
			   NFC_ERR_MSG("%s BBCLK2 OFF\n", __func__);
			}
		} else  if (arg == 1) {
			/* power on */
			NFC_ERR_MSG("power on\n");
			gpio_set_value(pn544_dev->firm_gpio, 0);
			gpio_set_value(pn544_dev->ven_gpio, 1);
			msleep(10);
			irq_set_irq_wake(pn544_dev->client->irq,1);
			if(clk_run == false){
			   msleep(10);
			   ret = clk_prepare_enable(bb_clk2);
		       if(ret){
		          NFC_ERR_MSG(KERN_ERR "%s: prepare bb_clk2 failed ret:%d\n", __func__, ret);
	           }
			   clk_run = true;
			   NFC_ERR_MSG("%s BBCLK2 ON\n", __func__);
			}
 
		} else  if (arg == 3) {
			/* power on pvdd*/
			NFC_ERR_MSG("pvdd on\n");
			pn544_regulator_enable(true);
		} else  if (arg == 4) {
			/* power off pvdd*/
			NFC_ERR_MSG("pvdd off\n");
			pn544_regulator_enable(false);
		} else {
		//	printk("%s bad arg %x\n", __func__, arg);
			return -EINVAL;
		}
		break;
	default:
		NFC_ERR_MSG("bad ioctl %x\n", cmd);
		return -EINVAL;
	}

	return 0;
}

static int nxp_pn544_reset(void)
{
	int rc;
	/*rc = gpio_tlmm_config(GPIO_CFG(pn544_dev->irq_gpio, 0,
				GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN,
				GPIO_CFG_2MA), GPIO_CFG_ENABLE);*/				
    if (!gpio_is_valid(pn544_dev->irq_gpio)) {
        NFC_ERR_MSG( "Could not configure nfc gpio %d\n",
                pn544_dev->irq_gpio);
        return -EIO;
    }
    rc = gpio_request(pn544_dev->irq_gpio, "nxp_pn544_IRQ");
    if (rc) {
        NFC_ERR_MSG( "unable to request nfc gpio %d (%d)\n",
                pn544_dev->irq_gpio, rc);
        return -EIO;
    }
    rc = gpio_direction_input(pn544_dev->irq_gpio);
    if (rc) {
        NFC_ERR_MSG( "unable to set input nfc gpio %d (%d)\n",
                pn544_dev->irq_gpio, rc);
        return -EIO;
    } 
    /*rc = gpio_tlmm_config(GPIO_CFG(pn544_dev->firm_gpio, 0,
				GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN,
				GPIO_CFG_2MA), GPIO_CFG_ENABLE);*/
    //printk("pn544 config firmgpio pull down\n");	  
    if (!gpio_is_valid(pn544_dev->firm_gpio)) {
        NFC_ERR_MSG( "Could not configure nfc gpio %d\n",
                pn544_dev->firm_gpio);
        return -EIO;
    }
    
    rc = gpio_request(pn544_dev->firm_gpio, "nxp_pn544_download");
    if (rc) {
        NFC_ERR_MSG( "unable to request nfc gpio %d (%d)\n",
                pn544_dev->firm_gpio, rc);
        return -EIO;
    }
    rc = gpio_direction_output(pn544_dev->firm_gpio, 0);
    if (rc) {
        NFC_ERR_MSG( "unable to set output nfc gpio %d (%d)\n",
                pn544_dev->firm_gpio, rc);
        return -EIO;
    }
    /*ven gpio out*/
    /*rc = gpio_tlmm_config(GPIO_CFG(pn544_dev->ven_gpio, 0,
	                                   GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
	                                   GPIO_CFG_2MA), GPIO_CFG_ENABLE);*/
    //printk("pn544 config vengpio out put no pull\n");
    if (!gpio_is_valid(pn544_dev->ven_gpio)) {
        NFC_ERR_MSG( "Could not configure nfc gpio %d\n",
                pn544_dev->ven_gpio);
        return -EIO;
    }
    
	rc = gpio_request(pn544_dev->ven_gpio, "nxp_pn544_en");
	if (rc) {
        NFC_ERR_MSG( "unable to request nfc gpio %d (%d)\n",
                pn544_dev->ven_gpio, rc);
        return -EIO;
	}

	rc = gpio_direction_output(pn544_dev->ven_gpio, 0);
	if (rc) {
        NFC_ERR_MSG( "unable to set output nfc gpio %d (%d)\n",
                pn544_dev->ven_gpio, rc);
        return -EIO;
    }	

    if (!gpio_is_valid(pn544_dev->vddio_en)) {
        NFC_ERR_MSG( "Could not configure nfc gpio %d\n",
                pn544_dev->vddio_en);
        return -EIO;
    }

	rc = gpio_request(pn544_dev->vddio_en, "nxp_pn544_vddio_en");
	if (rc) {
        NFC_ERR_MSG( "unable to request nfc gpio %d (%d)\n",
                pn544_dev->vddio_en, rc);
        return -EIO;
	}

	rc = gpio_direction_output(pn544_dev->vddio_en, 0);
	if (rc) {
        NFC_ERR_MSG( "unable to set output nfc gpio %d (%d)\n",
                pn544_dev->vddio_en, rc);
        return -EIO;
    }	

    if (!gpio_is_valid(pn544_dev->clk_gpio)) {
        NFC_ERR_MSG( "Could not configure nfc gpio %d\n",
                pn544_dev->clk_gpio);
        return -EIO;
    }

	rc = gpio_request(pn544_dev->clk_gpio, "nxp_pn544_clk_en");
	if (rc) {
        NFC_ERR_MSG( "unable to request nfc gpio %d (%d)\n",
                pn544_dev->clk_gpio, rc);
        return -EIO;
	}

	rc = gpio_direction_input(pn544_dev->clk_gpio);
	if (rc) {
        NFC_ERR_MSG( "unable to set output nfc gpio %d (%d)\n",
                pn544_dev->clk_gpio, rc);
	}  
	return 0;
}

static const struct file_operations pn544_dev_fops = {
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.read	= pn544_dev_read,
	.write	= pn544_dev_write,
	.open	= pn544_dev_open,
	.unlocked_ioctl  = pn544_dev_ioctl,
	.compat_ioctl  = pn544_dev_ioctl,
};


static int pn544_regulator_enable(bool enable)
{
	int rc = 0;

	if (enable) {
		NFC_DBG_MSG("PVDD enable\n");
		if (!gpio_is_valid(pn544_dev->vddio_en)) {
			NFC_ERR_MSG("vddio_en gpio is not valid\n");
			return -EINVAL;
		}
		gpio_set_value(pn544_dev->vddio_en, 1);
	} else {
		NFC_DBG_MSG("PVDD disable\n");
		if (!gpio_is_valid(pn544_dev->vddio_en)) {
			NFC_ERR_MSG("vddio_en gpio is not valid\n");
			return -EINVAL;
		}
		gpio_set_value(pn544_dev->vddio_en, 0);
	}	
	return rc;
}

static int pn544_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
  // u32 	temp_val;
   struct device_node *of_node = NULL;
   //static struct clk *bb_clk2;
   /*
	platform_data = client->dev.platform_data;
	if (platform_data == NULL) {
		pr_err("%s : nfc probe fail\n", __func__);
		return  -ENODEV;
	}
    */ 

   NFC_ERR_MSG("pn544_probe(): start\n");   
   if (pn544_dev != NULL) {
      NFC_ERR_MSG("pn544_probe: multiple devices NOT supported\n");
      ret = -ENODEV;
      goto err_single_device;
   }
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s : need I2C_FUNC_I2C\n", __func__);
		return  -ENODEV;
	}
	/*//zte-modify by zuojianfang
	ret = gpio_request(platform_data->irq_gpio, "nfc_int");
	if (ret)
		return  -ENODEV;
	ret = gpio_request(platform_data->ven_gpio, "nfc_ven");
	if (ret)
		goto err_ven;
	ret = gpio_request(platform_data->firm_gpio, "nfc_firm");
	if (ret)
		goto err_firm;
    */
	pn544_dev = kzalloc(sizeof(*pn544_dev), GFP_KERNEL);
	if (pn544_dev == NULL) {
		dev_err(&client->dev,
				"failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
	}

   	/*//zte-modify by zuojianfang
	pn544_dev->irq_gpio = platform_data->irq_gpio;
	pn544_dev->ven_gpio  = platform_data->ven_gpio;
	pn544_dev->firm_gpio  = platform_data->firm_gpio;
	*/
	pn544_dev->client   = client;
	
   //zte-modify by zuojianfang for gpio
   if (client->dev.of_node) {
   	
   	of_node = client->dev.of_node;

	ret = of_get_named_gpio(of_node, "nfc,irq_gpio", 0);
	if (ret>0) {
		pn544_dev->irq_gpio=ret;
	} else{
		NFC_ERR_MSG(" of_property_read(irq_gpio) fail:%d\n",ret);
		goto err_device_create_failed;
	}

	ret = of_get_named_gpio(of_node, "nfc,firm_gpio", 0);
	if (ret>0) {
		pn544_dev->firm_gpio=ret;
	} else{
		NFC_ERR_MSG("of_property_read(firm_gpio) fail:%d\n",ret);
		goto err_device_create_failed;
	}

	ret = of_get_named_gpio(of_node, "nfc,ven_gpio", 0);	//8974 mpp7
	if (ret>0) {
		pn544_dev->ven_gpio=ret;
	} else{
		NFC_ERR_MSG("of_property_read(ven_gpio) fail:%d\n",ret);
		goto err_device_create_failed;
	}
	ret = of_get_named_gpio(of_node, "nfc,vddio_en", 0);	
	if (ret>0) {
		pn544_dev->vddio_en=ret;
	} else{
		NFC_ERR_MSG("of_property_read(vddio_en) fail:%d\n",ret);
		goto err_device_create_failed;
	}
	  
	ret = of_get_named_gpio(of_node, "nfc,clk_gpio", 0);	
	if (ret>0) {
		pn544_dev->clk_gpio=ret;
	} else{
		NFC_ERR_MSG("of_property_read(clk_gpio) fail:%d\n",ret);
		goto err_device_create_failed;
	}
	/* init mutex and queues */
	init_waitqueue_head(&pn544_dev->read_wq);
	mutex_init(&pn544_dev->read_mutex);
	mutex_init(&pn544_dev->write_mutex);
	spin_lock_init(&pn544_dev->irq_enabled_lock);

	nfc_wake_lock = wakeup_source_register("nfctimer");
	pn544_dev->pn544_device.minor = MISC_DYNAMIC_MINOR;
	pn544_dev->pn544_device.name = "pn544";
	pn544_dev->pn544_device.fops = &pn544_dev_fops;

	ret = misc_register(&pn544_dev->pn544_device);
	if (ret) {
		pr_err("%s : misc_register failed\n", __FILE__);
		goto err_misc_register;
	}

	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */
	 /*//zte-modify by zuojianfang
	s3c_gpio_cfgpin(platform_data->ven_gpio, (0x1 << ((0) * 4)));//output
	s3c_gpio_cfgpin(platform_data->firm_gpio, (0x1 << ((1) * 4)));//output
	s3c_gpio_setpull(platform_data->ven_gpio, S3C_GPIO_PULL_UP);
	s3c_gpio_setpull(platform_data->firm_gpio, S3C_GPIO_PULL_DOWN);

	//eint16 setting
	s3c_gpio_cfgpin(platform_data->irq_gpio, 3);//input
	s3c_gpio_setpull(platform_data->irq_gpio, S3C_GPIO_PULL_UP);
    */
    ret = nxp_pn544_reset();
    NFC_DBG_MSG("pn544 reset\n");
    if (ret < 0) {
        NFC_ERR_MSG(  "can't reset device\n");
        goto err_device_create_file_failed;
    }
	
	pn544_regulator_enable(true);
	
    bb_clk2 = clk_get(&client->dev, "bb_clk2");
	//bb_clk2 = devm_clk_get(&client->dev, "bb_clk2");
	if (IS_ERR(bb_clk2)) {
		NFC_ERR_MSG("Error getting bb_clk2\n");
		bb_clk2 = NULL;
		//return -ENOENT;
	}
	else{
		NFC_ERR_MSG("start prepare bb_clk2\n");
		/*
		ret = clk_set_rate(bb_clk2,NFC_RF_CLK_FREQ);
		if(ret){
		    NFC_DBG_MSG("set bb_clk2 rate failed ret:%d\n", ret);
	    }		
        //ret = clk_prepare_enable(bb_clk2);
		if(ret){
		    NFC_DBG_MSG("prepare bb_clk2 failed ret:%d\n",ret);
	    }*/
	}

	//pr_info("%s : requesting IRQ %d\n", __func__, client->irq);
	pn544_dev->irq_enabled = true;

	ret = request_irq(client->irq, pn544_dev_irq_handler,
			  IRQF_TRIGGER_HIGH, client->name, pn544_dev);
	if (ret) {
		NFC_ERR_MSG("request_irq failed\n");
		goto err_request_irq_failed;
	}else{
		/* test to see if irq is really wakeup capable */
		ret = irq_set_irq_wake(client->irq, 1);
		if (ret) {
            NFC_ERR_MSG("irq %d cannot set wakeup (%d)\n",client->irq,ret);
            ret = 0;
 		} else {
            irq_set_irq_wake(client->irq, 0);
		}
		NFC_DBG_MSG("request_irq ok\n");
	}

	pn544_disable_irq(pn544_dev);
	i2c_set_clientdata(client, pn544_dev);
	create_nfc_info_proc_file();
	NFC_ERR_MSG("nfc probe is ok\n");

	return 0;
   }
err_request_irq_failed:
	misc_deregister(&pn544_dev->pn544_device);
err_misc_register:
	mutex_destroy(&pn544_dev->read_mutex);
	mutex_destroy(&pn544_dev->write_mutex);
	kfree(pn544_dev);
err_device_create_failed:
   kfree(pn544_dev);
   pn544_dev = NULL;
err_device_create_file_failed:
    //device_destroy(pn544_dev_class, MKDEV(pn544_major, pn544_minor));
err_exit:
	//gpio_free(platform_data->firm_gpio);
err_single_device:
	return ret;

}

static int pn544_remove(struct i2c_client *client)
{
	//struct pn544_dev *pn544_dev;
	NFC_ERR_MSG("start\n");
    //irq_set_irq_wake(client->irq,0);
	if(clk_run == true){
	   clk_disable_unprepare(bb_clk2);
	   clk_run = false;
	}
	pn544_dev = i2c_get_clientdata(client);
	pn544_disable_irq(pn544_dev);
	free_irq(client->irq, pn544_dev);
	misc_deregister(&pn544_dev->pn544_device);
	mutex_destroy(&pn544_dev->read_mutex);
	mutex_destroy(&pn544_dev->write_mutex);
	gpio_free(pn544_dev->irq_gpio);
	gpio_free(pn544_dev->ven_gpio);
	gpio_free(pn544_dev->firm_gpio);
	gpio_free(pn544_dev->vddio_en);
	gpio_free(pn544_dev->clk_gpio);
	kfree(pn544_dev);
	pn544_dev = NULL;
	
	remove_nfc_info_proc_file();

	return 0;
}

static const struct i2c_device_id pn544_id[] = {  //zuojianfang
   	{ PN544_DRIVER_NAME, 0 },
   	{ }
};

static struct of_device_id nfc_match_table[] = { //zuojianfang
	{.compatible = "nxp,pn544",},
	{ },
};


static struct i2c_driver pn544_driver = {
	.id_table	= pn544_id,
	.probe		= pn544_probe,
	.remove		= pn544_remove,
	.driver		= {
		.name   = "pn544",
		.owner = THIS_MODULE,
		.of_match_table = nfc_match_table,
	},
};

/*
 * module load/unload record keeping
 */
 
static int __init pn544_dev_init(void)
{
	pr_info("Loading pn544 driver\n");
	return i2c_add_driver(&pn544_driver);
}
module_init(pn544_dev_init);

static void __exit pn544_dev_exit(void)
{
	pr_info("Unloading pn544 driver\n");
	i2c_del_driver(&pn544_driver);
}
module_exit(pn544_dev_exit);

MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_DESCRIPTION("NFC PN544 driver");
MODULE_LICENSE("GPL");
