#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
//#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/proc_fs.h>
#include <linux/power_supply.h>
#include <linux/qpnp/qpnp-adc.h>

#define LC709203_BUSNUM  1
#define LC709203_SLAVE_ADDR   (0x0B )
#define LC709203_SLAVE_ADDR_WR (0x16)
#define LC709203_SLAVE_ADDR_RD (0x17)
#define LC709203_TEMP_OFFSET 2732

#define BATT_SETUP_INITIAL_RSOC_ADDR 0x04
#define INITIAL_RSOC_THERMISTOR_ADDR 0x06
#define INITIAL_RSOC_ADDR 0x07 
#define CELL_TEMP_ADDR 0x08
#define CELL_VOLTAGE_ADDR 0x09

#define ADJUST_PACK_APPI_ADDR 0x0B
#define ADJUST_PACK_THERMISTOR_ADDR 0x0C
#define RSOC_ADDR  0x0D

#define INDICATOR_TO_EMPTY_ADDR 0x0F

#define IC_VERSION_ADDR 0x11
#define ALARM_LOW_RSOC_ADDR  0x13
#define ALARM_LOW_CELL_VOLTAGE_ADDR 0x14
#define IC_POWER_MODE_ADDR 0x15
#define SATAUS_BIT_ADDR 0x16

#define RSOC_INIT 0xAA55
#define POWER_MODE_ACCURATE_ECO 0x1
#define BATT_OEM_10 0x0023
#define BATT_OEM_30 0x0018
#define BATT_OEM_UNKNOWN 0x0034
#define FLAG_BATT_SETUP 3400


#define LC709203_REG_NUM 10
#define	dPOLYNOMIAL8			0x8380
#define  IIC_WRRD_ERR -1
#define  IIC_WRRD_SUCCESS 0

//extern int g_batt_temp;

struct onsemi_chip {
	struct device			*dev;
       struct power_supply		*bms_psy;
	struct power_supply		batt_psy;

	struct qpnp_adc_tm_chip		*adc_tm_dev;

       struct delayed_work set_battery_temp_work;
       struct delayed_work get_capacity_work;
};

static struct i2c_client *new_client = NULL;

static int lc709203_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);

u8 g_lc709203_driver_init_done = 0;

u16 lc709203_reg[LC709203_REG_NUM] = {0};

static DEFINE_MUTEX(lc709203_i2c_access);


static unsigned char u1_CRC_8_u1u1( unsigned char u1ArgBeforeData , unsigned char u1ArgAfterData)
{
	unsigned char	u1TmpLooper = 0;
	unsigned char	u1TmpOutData = 0;
	unsigned short	 u2TmpValue = 0;

	u2TmpValue = (unsigned short)(u1ArgBeforeData ^ u1ArgAfterData);
	u2TmpValue <<= 8;

	for( u1TmpLooper = 0 ; u1TmpLooper < 8 ; u1TmpLooper++ ){
		if( u2TmpValue & 0x8000 ){
			u2TmpValue ^= dPOLYNOMIAL8;
		}
		u2TmpValue <<= 1;
	}

	u1TmpOutData = (unsigned char)(u2TmpValue >> 8);

	return( u1TmpOutData );
}

u8 lc709203_write_crc_calc(u8 cmd, u8 low,u8 high)
{
	u8 data[4], crc8=0;
	int i;
    
       data[0] =LC709203_SLAVE_ADDR_WR;
       data[1] = cmd;
       data[2]=low; 
       data[3]=high; 
	for (i = 0; i < 4; i++) {
		crc8 = u1_CRC_8_u1u1(crc8, data[i]);
	}
	pr_debug("%s: lc709203_crc_calc 0x%02x\n", __func__, crc8);
       return crc8;
}

u8 lc709203_read_crc_calc(u8 cmd, u8 low,u8 high)
{
	u8 data[5], crc8=0;
	int i;
    
       data[0] =LC709203_SLAVE_ADDR_WR;
       data[1] = cmd;
       data[2]= LC709203_SLAVE_ADDR_RD;
       data[3]=low; 
       data[4]=high; 
	for (i = 0; i < 5; i++) {
		crc8 = u1_CRC_8_u1u1(crc8, data[i]);
	}
	pr_debug("%s: lc709203_crc_calc 0x%02x\n", __func__, crc8);
       return crc8;
}

int lc709203_read_word(u8 cmd, u16 *returnData)
{
    u8     buf[3]={0x00,0x00,0x00};
    u16     readData = 0;
    u16      ret=0;
    u16     crc8 =0;
    u8  reg = cmd;
    
    struct i2c_msg msg[] = {
    	{
    		.addr = new_client->addr,
    		.flags = 0,
    		.len = 1,
    		.buf = &reg,
    	},
    	{
    		.addr = new_client->addr,
    		.flags = I2C_M_RD,
    		.len = 3,
    		.buf = buf,
    	},
    };
    
    mutex_lock(&lc709203_i2c_access);

    reg = reg & 0xFF;

    ret = i2c_transfer(new_client->adapter, msg, 2);

    if (ret < 2) {
    	dev_err(&new_client->dev, "%s: i2c_transfer() i2c read error, reg: %x\n", __func__, cmd);
       mutex_unlock(&lc709203_i2c_access);
    	return ret < 0 ? ret : -EIO;
    }

    pr_debug("%s: lc709203_read_word 0x%x, 0x%x, 0x%x, 0x%x\n", __func__, cmd, buf[0], buf[1], buf[2]);
   
    crc8 = lc709203_read_crc_calc(cmd,buf[0], buf[1]);
   
    if(buf[2] != crc8 )
    {
        pr_err("%s: \tlc709203_read_word crc8 error\n", __func__);
        mutex_unlock(&lc709203_i2c_access);    
        return IIC_WRRD_ERR;
    }
    
    readData = buf[0] | (buf[1] << 8);
    *returnData = readData;
    
    mutex_unlock(&lc709203_i2c_access);    
    return IIC_WRRD_SUCCESS;
}

int lc709203_write_word(u8 cmd, u16 writeData)
{
    u8    write_data[4] = {0};
    int     ret=0;
    
    mutex_lock(&lc709203_i2c_access);
    
    write_data[0] = cmd;
    write_data[1] = writeData & 0x00ff;
    write_data[2] = (writeData & 0xff00) >> 8;
    write_data[3] = lc709203_write_crc_calc(cmd,write_data[1], write_data[2]);
    
    ret = i2c_master_send(new_client, write_data, 4);
    if (ret < 0) 
    {
        printk(KERN_ERR"%s: i2c_master_send() error, %d\n", __func__, ret);
        mutex_unlock(&lc709203_i2c_access);
        return IIC_WRRD_ERR;
    }

    pr_debug("%s: write success\n", __func__);
    mutex_unlock(&lc709203_i2c_access);
    return IIC_WRRD_SUCCESS;
}

u32 lc709203_read_interface (u8 RegNum, u16 *val, u16 MASK, u16 SHIFT)
{
    u16 lc709203_reg = 0;
    int ret = 0;

    ret = lc709203_read_word(RegNum, &lc709203_reg);

    lc709203_reg &= MASK ;
    *val = (lc709203_reg >> SHIFT);
	
    pr_debug("%s: read Reg[%x]=0x%x val=0x%x\n",  __func__, RegNum,lc709203_reg, *val);
    return ret;
}

u32 lc709203_config_interface (u8 RegNum, u16 val, u16 MASK, u16 SHIFT)
{
    u16 lc709203_reg = 0;
    int ret = 0;

    ret = lc709203_read_word(RegNum, &lc709203_reg);
    
    lc709203_reg &= (~ MASK );
    lc709203_reg |= (val << SHIFT);

    ret = lc709203_write_word(RegNum, lc709203_reg);
    pr_debug("%s:write Reg[%x]=0x%x\n", __func__, RegNum, lc709203_reg);

    return ret;
}

#define DEFAULT_TEMP		250
static int get_prop_batt_temp(struct onsemi_chip*chip)
{
	int batt_therm;
       union power_supply_propval ret = {0,};

       chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_TEMP, &ret);
	batt_therm = ret.intval;

	return batt_therm;
}

#define CYCLE_SET_TEMP_MS  5000
void  set_battery_temp_work(struct work_struct *work)
{
    u16 temperature=0;
    static u16 batt_temp_last = 0;
    int batt_temp;

    struct onsemi_chip *chip =
		container_of(work, struct onsemi_chip, set_battery_temp_work.work);

    batt_temp = get_prop_batt_temp(chip);

    printk(KERN_ERR"lc709203 batt_tem=%d\n", batt_temp);
    
    if(g_lc709203_driver_init_done)
    {
        
        if (batt_temp != batt_temp_last)
        {
            temperature = batt_temp+LC709203_TEMP_OFFSET;
            pr_debug("%s: batt temp = %d, write reg[0x%x] = 0x%x\n",  __func__, batt_temp, CELL_TEMP_ADDR, temperature);
            
            lc709203_write_word(CELL_TEMP_ADDR, temperature);
        }

        batt_temp_last = batt_temp;
    }
    else
    {
        printk(KERN_ERR"lc709203 driver init not done ,please waiting\n");
    }
    
    schedule_delayed_work(&chip->set_battery_temp_work, msecs_to_jiffies(CYCLE_SET_TEMP_MS));
}

void lc709203_dump_register(void)
{
    int i=0;
    printk(KERN_DEBUG"%s **********************\n", __func__);
    for (i=0;i<LC709203_REG_NUM;i++)
    {
        lc709203_read_word(i, &lc709203_reg[i]);
        printk(KERN_DEBUG"[0x%x]=0x%x \n", i, lc709203_reg[i]);        
    }
}


int lc709203_power_on_init(struct onsemi_chip *chip)
{
    int ret = -1;

    mdelay(3);
    ret = lc709203_write_word(IC_POWER_MODE_ADDR, POWER_MODE_ACCURATE_ECO);
    if ( ret < 0 )
    {
        printk(KERN_ERR"%s: #1 write reg[0x%x] failed, try read agin!\n", __func__, IC_POWER_MODE_ADDR); 
    }
    
    mdelay(500);
    ret = lc709203_write_word(IC_POWER_MODE_ADDR, POWER_MODE_ACCURATE_ECO);
    if ( ret < 0 )
    {
        printk(KERN_ERR"%s: #2 write reg[0x%x] failed, try read agin!! \n", __func__, IC_POWER_MODE_ADDR);
    }

    mdelay(3);
    //ret = lc709203_write_word(ADJUST_PACK_APPI_ADDR, BATT_OEM_10); 
    ret = lc709203_write_word(ADJUST_PACK_APPI_ADDR, BATT_OEM_30); 
    if ( ret < 0 )
    {
        printk(KERN_ERR"%s: write reg[0x%x] failed!\n", __func__, ADJUST_PACK_APPI_ADDR);
        goto init_err;
    }

    schedule_delayed_work(&chip->set_battery_temp_work, msecs_to_jiffies(1000));
    schedule_delayed_work(&chip->get_capacity_work, msecs_to_jiffies(3000));
    
    printk(KERN_DEBUG"%s: init success!\n", __func__);
    return 0;
    
init_err:
    printk(KERN_ERR"%s: lc709203 init failed!\n", __func__);
    return ret;
}


u16 lc709203_get_capacity(void)
{
     u16 rsoc=0;
     u16 ite=0;
     u16 corrected_capcity =0;
     u16 reg_volt = 0;
     u16 reg_temp = 0;

    if(g_lc709203_driver_init_done)
    {
        mdelay(3);
        lc709203_read_word(RSOC_ADDR,&rsoc);
        mdelay(3);
        lc709203_read_word(INDICATOR_TO_EMPTY_ADDR,&ite);
        mdelay(3);
        lc709203_read_word(CELL_VOLTAGE_ADDR,&reg_volt);
        mdelay(3);
        lc709203_read_word(CELL_TEMP_ADDR,&reg_temp);
    }

    corrected_capcity = (ite - 19)*100/(1000-19);
     
    printk(KERN_ERR"LC709203: addr temp =%d, vbat=%d, RSOC=%d, ITE=%d, Corrected capcity=%d\n", 
                               reg_temp-2732, reg_volt, rsoc, ite, corrected_capcity); 

    return corrected_capcity;
}

#define CYCLE_GET_SOC 5000
void get_capacity_work(struct work_struct *work)
{
    struct onsemi_chip *chip =
		container_of(work, struct onsemi_chip, get_capacity_work.work);
    
    lc709203_get_capacity();
    schedule_delayed_work(&chip->get_capacity_work, msecs_to_jiffies(CYCLE_GET_SOC));
}

void lc709203_get_verison_ic(void)
{

    u16 reg_value=0;
    
    lc709203_read_word(IC_VERSION_ADDR,&reg_value);
    printk(KERN_ERR"lc709203 version is 0x%x\n", reg_value);
}
/*
static ssize_t lc709203_proc_write(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	char messages[128];

	if (len > 128)
		len = 128;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;
       lc709203_get_verison_ic();
       return len;
}

static struct file_operations lc709203_proc_ops = {
    .owner   = THIS_MODULE,
    .write =lc709203_proc_write,
};

static int create_lc709203_proc_file(void)
{
    struct proc_dir_entry *entry;

    entry = create_proc_entry("driver/lc709203", 0, NULL);
    if (entry)
        entry->proc_fops = &lc709203_proc_ops;
    return 0;
}
*/
static ssize_t show_lc709203_access(struct device *dev,struct device_attribute *attr, char *buf)
{
    lc709203_get_capacity();
    
    return 0;
}

static ssize_t store_lc709203_access(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    int ret=0;
    char *pvalue = NULL;
    u16 reg_value = 0;
    u8 reg_addr = 0;

    
    if(buf != NULL && size != 0)
    {
        //printk(KERN_ERR"buf is %s and size is %d \n",buf,size);
        reg_addr = simple_strtoul(buf,&pvalue,16);
        
        if( size > 4 )
        {        
            reg_value = simple_strtoul((pvalue+1),NULL,16);        
            printk(KERN_ERR" write reg[0x%x] = 0x%x\n",reg_addr,reg_value);
            ret=lc709203_config_interface(reg_addr, reg_value, 0xFFFF, 0x0);
        }
        else
        {    
            ret=lc709203_read_interface(reg_addr, &reg_value, 0xFFFF, 0x0);
            printk(KERN_ERR"read reg[0x%x] = 0x%x\n",reg_addr, reg_value);
        }        
    }    
    return size;
}
static DEVICE_ATTR(lc709203_access, 0664, show_lc709203_access, store_lc709203_access); //664

static int lc709203_user_space_probe(struct platform_device *dev)    
{    
    int ret_device_file = 0;

    printk(KERN_DEBUG"lc709203_user_space_probe!!\n" );
    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_lc709203_access);
    
    return 0;
}

struct platform_device lc709203_user_space_device = {
    .name   = "lc709203-user",
    .id     = -1,
};

static struct platform_driver lc709203_user_space_driver = {
    .probe      = lc709203_user_space_probe,
    .driver     = {
        .name = "lc709203-user",
    },
};

static int lc709203_driver_probe(struct i2c_client *client, const struct i2c_device_id *id) 
{             
    int err=0; 
    struct onsemi_chip *chip;

    printk(KERN_ERR"%s\n", __func__);

    chip = kzalloc(sizeof(struct onsemi_chip), GFP_KERNEL);
    if (!chip) {
    	pr_err("%s: memory allocation failed.\n", __func__);
    	return -ENOMEM;
    }

    if (!(new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL))) {
        err = -ENOMEM;
        goto exit;
    }    
    memset(new_client, 0, sizeof(struct i2c_client));

    new_client = client;

    chip->bms_psy = power_supply_get_by_name("bms");
    if (!chip->bms_psy) {
        pr_err("bms supply not found deferring probe\n");        		
        err = -EPROBE_DEFER;        		
        goto exit;
    }

    INIT_DELAYED_WORK(&chip->set_battery_temp_work, set_battery_temp_work);
    INIT_DELAYED_WORK(&chip->get_capacity_work, get_capacity_work);
    
    // lc709203_dump_register();
    lc709203_power_on_init(chip);
    //create_lc709203_proc_file();

    err = platform_device_register(&lc709203_user_space_device);
    if (err) {
        printk(KERN_ERR"lc709203 Unable to register device (%d)\n", err);
        return err;
    }
    
    err = platform_driver_register(&lc709203_user_space_driver);
    if (err) {
        printk(KERN_ERR"lc709203 Unable to register driver (%d)\n", err);
        return err;
    }

    g_lc709203_driver_init_done = 1;
    
    lc709203_get_verison_ic();
    lc709203_get_capacity();
    
    return 0;                                                                                       

exit:
    return err;

}

#ifdef CONFIG_OF
static struct of_device_id lc709203_match_table[] = {
	{ .compatible = "lc709203,on-fuel-gauge",},
	{ },
};
#else
#define synaptics_match_table NULL
#endif

static const struct i2c_device_id lc709203_i2c_id[] = {{"lc709203",0},{}};   

static struct i2c_driver lc709203_driver = {
	.driver = {
		.name = "lc709203",
		.owner = THIS_MODULE,
		.of_match_table = lc709203_match_table,
	},
	.probe = lc709203_driver_probe,
	.id_table = lc709203_i2c_id,
};

int lc709203_init(void)
{
	return i2c_add_driver(&lc709203_driver);
}

static void __exit lc709203_exit(void)
{
    i2c_del_driver(&lc709203_driver);
}

late_initcall(lc709203_init);
module_exit(lc709203_exit);
   
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C lc709203 Driver");
MODULE_AUTHOR("ZTE <jiang.zhifeng1@zte.com.cn>");
