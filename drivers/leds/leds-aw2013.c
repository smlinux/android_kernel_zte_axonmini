#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/param.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/debugfs.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/ctype.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include	<linux/regulator/consumer.h>

#define AW2013_DRIVER_NAME "aw2013"

#define AW2013_REG_RSTR     0x00
#define AW2013_REG_GCR      0x01
#define AW2013_REG_ISR      0x02
#define AW2013_REG_LCTR     0x30

#define AW2013_REG_LCFG0    0x31
#define AW2013_REG_LCFG1    0x32
#define AW2013_REG_LCFG2    0x33

#define AW2013_REG_PWM0     0x34
#define AW2013_REG_PWM1     0x35
#define AW2013_REG_PWM2     0x36

#define AW2013_REG_LED0T0   0x37
#define AW2013_REG_LED1T0   0x3A
#define AW2013_REG_LED2T0   0x3D

#define AW2013_REG_LED0T1   0x38
#define AW2013_REG_LED1T1   0x3B
#define AW2013_REG_LED2T1   0x3E

#define AW2013_REG_LED0T2   0x39
#define AW2013_REG_LED1T2   0x3C
#define AW2013_REG_LED2T2   0x3F

#define AW2013_REG_IADR     0x77

#define Imax          0x1   //0x00=omA,0x01=5mA,0x02=10mA,0x03=15mA,
#define Rise_time   0x4   //0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define Hold_time   0x3  //0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s
#define Fall_time     0x4   //0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define Off_time      0x3   //0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define Delay_time   0x1   //0x00=0s,0x01=0.13s,0x02=0.26s,0x03=0.52s,0x04=1.04s,0x05=2.08s,0x06=4.16s,0x07=8.32s,0x08=16.64s
#define Period_Num  0x00   //0x00=repeat allways, 0x01=1time,0x02=2times,.....0x0f=15times

struct aw2013_Time {
    u8 T1;
    u8 T2;
    u8 T3;
    u8 T4;
};

struct aw2013_Time aw2013_times[] = {
                                        {0x0 , 0x2 , 0x0 , 0x2},//blink
                                        {0x3 , 0x3 , 0x3 , 0x3},//breath slow
                                        {0x4 , 0x3 , 0x4 , 0x3},//breath mid
                                        {0x5 , 0x3 , 0x5 , 0x3},//breath fast
                                    };

enum aw2013_led_blink_type{
       AW2013_LED_OFF,
       AW2013_LED_BLINK,
       AW2013_LED_BREATH_SLOW,
       AW2013_LED_BREATH_MID,
       AW2013_LED_BREATH_FAST,
};

struct aw2013_debug_reg {
	char  *name;
	u8  reg;
};

#define AW2013_DEBUG_REG(x, y) {#x#y, AW2013_REG_##y}

static struct aw2013_debug_reg aw2013_debug_regs[] = {
	AW2013_DEBUG_REG(00_, RSTR),
	AW2013_DEBUG_REG(01_, GCR),
	AW2013_DEBUG_REG(02_, ISR),
	AW2013_DEBUG_REG(30_, LCTR),
	AW2013_DEBUG_REG(31_, LCFG0),
	AW2013_DEBUG_REG(32_, LCFG1),
	AW2013_DEBUG_REG(33_, LCFG2),
	AW2013_DEBUG_REG(34_, PWM0),
	AW2013_DEBUG_REG(35_, PWM1),
	AW2013_DEBUG_REG(36_, PWM2),
	AW2013_DEBUG_REG(37_, LED0T0),
	AW2013_DEBUG_REG(38_, LED0T1),
	AW2013_DEBUG_REG(39_, LED0T2),
       AW2013_DEBUG_REG(3A_, LED1T0),
       AW2013_DEBUG_REG(3B_, LED1T1),
       AW2013_DEBUG_REG(3C_, LED1T2),
       AW2013_DEBUG_REG(3D_, LED2T0),        
       AW2013_DEBUG_REG(3E_, LED2T1),
       AW2013_DEBUG_REG(3F_, LED2T2),
       AW2013_DEBUG_REG(77_, IADR),
};

static bool has_red = 0;
static bool has_green = 0;
static bool has_blue = 0;
static bool has_buttonlight = 0;

struct breath_light_data{
    struct led_classdev led;
    int blink_flag;
    int pwm_flag;
};

struct breaths_light_dev {
    struct breath_light_data	breath_light[3];
    struct i2c_client	*client;
    struct dentry  *dent;
    unsigned int irq_gpio;
    struct work_struct	work;
    struct mutex		lock ;
};

static struct breaths_light_dev * breath_dev = NULL;
enum aw2013_command{
    AW2013_READ_THE_CHIP_ID,
    AW2013_SW_RESET,
    AW2013_GLOBAL_CONTROL, //GCR,
    AW2013_LED_CONTROL_WRITE, //LCTR
    AW2013_LED_READ_ISR,
    AW2013_MODE_CONFIG_WRITE_LCFG0,
    AW2013_MODE_CONFIG_WRITE_LCFG1,
    AW2013_MODE_CONFIG_WRITE_LCFG2,
    AW2013_PWM_CONFIG_WRITE_PWM0,
    AW2013_PWM_CONFIG_WRITE_PWM1,
    AW2013_PWM_CONFIG_WRITE_PWM2,
    AW2013_T1_AND_T2_WRITE_LED0T0,
    AW2013_T1_AND_T2_WRITE_LED1T0,
    AW2013_T1_AND_T2_WRITE_LED2T0,
    AW2013_T3_AND_T4_WRITE_LED0T1,
    AW2013_T3_AND_T4_WRITE_LED1T1,
    AW2013_T3_AND_T4_WRITE_LED2T1,
    AW2013_T0_AND_REPEAT_WRITE_LED0T2,
    AW2013_T0_AND_REPEAT_WRITE_LED1T2,
    AW2013_T0_AND_REPEAT_WRITE_LED2T2,
};

static int aw2013_i2c_read(struct breaths_light_dev *breaths_dev, u8 reg)
{
    struct i2c_client *client =breaths_dev -> client;
    struct i2c_msg msg[2];
    u8 val;
    int ret;

    msg[0].addr = client -> addr;
    msg[0].flags = 0;
    msg[0].buf = &reg;
    msg[0].len = sizeof(reg);

    msg[1].addr = client -> addr;
    msg[1].flags = I2C_M_RD;
    msg[1].buf = &val;
    msg[1].len = sizeof(val);

    mutex_lock(&breath_dev->lock );
    ret = i2c_transfer(client -> adapter , msg ,ARRAY_SIZE(msg));
    pr_debug("reg == 0x%x  , val =  0x%x\n " ,reg , val);
    mutex_unlock(&breath_dev->lock );
    udelay(5);
    if (ret < 0)
        return ret;
    return val;
}

static int aw2013_i2c_write(struct breaths_light_dev *breaths_dev, u8 reg, u8 val)
{
    struct i2c_client *client =breaths_dev -> client;
    struct i2c_msg msg[1];
    u8 data[2];
    int ret ;

    data[0] = reg;
    data[1] = val;

    msg[0].addr = client -> addr;
    msg[0].flags = 0;
    msg[0].buf = data;
    msg[0].len = ARRAY_SIZE(data);

    mutex_lock(&breaths_dev->lock );
    ret = i2c_transfer(client -> adapter , msg ,ARRAY_SIZE(msg));
    mutex_unlock(&breaths_dev->lock );

    if (ret != 1) {
        dev_err(&breaths_dev->client->dev, "write transfer error\n");
        ret = -EIO;
    } else {
        ret = 0;
    }
    udelay(5);
    return 0;
}

static int aw2013_exec_command(struct breaths_light_dev *breaths_dev , enum aw2013_command command ,u8 val)
{
    int ret =1;
    switch (command){
    case  AW2013_READ_THE_CHIP_ID:
        ret = aw2013_i2c_read(breaths_dev,AW2013_REG_RSTR);
        break;
    case  AW2013_SW_RESET:
        ret = aw2013_i2c_write(breaths_dev , AW2013_REG_RSTR , val);
        break;
    case   AW2013_GLOBAL_CONTROL:
        ret = aw2013_i2c_write(breaths_dev , AW2013_REG_GCR , val);
        break;
    case   AW2013_LED_READ_ISR:
        ret = aw2013_i2c_read(breaths_dev,AW2013_REG_ISR);
        break;
    case     AW2013_LED_CONTROL_WRITE:
        ret = aw2013_i2c_write(breaths_dev , AW2013_REG_LCTR , val);
        break;
    case     AW2013_MODE_CONFIG_WRITE_LCFG0:
        ret = aw2013_i2c_write(breaths_dev , AW2013_REG_LCFG0 , val);
        break;
    case     AW2013_MODE_CONFIG_WRITE_LCFG1:
        ret = aw2013_i2c_write(breaths_dev , AW2013_REG_LCFG1 , val);
        break;
    case     AW2013_MODE_CONFIG_WRITE_LCFG2:
        ret = aw2013_i2c_write(breaths_dev , AW2013_REG_LCFG2 , val);
        break;
    case     AW2013_PWM_CONFIG_WRITE_PWM0:
        ret = aw2013_i2c_write(breaths_dev , AW2013_REG_PWM0 , val);
        break;
    case     AW2013_PWM_CONFIG_WRITE_PWM1:
        ret = aw2013_i2c_write(breaths_dev , AW2013_REG_PWM1 , val);
        break;
    case     AW2013_PWM_CONFIG_WRITE_PWM2:
        ret = aw2013_i2c_write(breaths_dev , AW2013_REG_PWM2 , val);
        break;
    case     AW2013_T1_AND_T2_WRITE_LED0T0:
        ret = aw2013_i2c_write(breaths_dev , AW2013_REG_LED0T0 , val);
        break;
    case     AW2013_T1_AND_T2_WRITE_LED1T0:
        ret = aw2013_i2c_write(breaths_dev , AW2013_REG_LED1T0 , val);
        break;
    case     AW2013_T1_AND_T2_WRITE_LED2T0:
        ret = aw2013_i2c_write(breaths_dev , AW2013_REG_LED2T0 , val);
        break;
    case   AW2013_T3_AND_T4_WRITE_LED0T1:
        ret = aw2013_i2c_write(breaths_dev , AW2013_REG_LED0T1 , val);
        break;
    case     AW2013_T3_AND_T4_WRITE_LED1T1:
        ret = aw2013_i2c_write(breaths_dev , AW2013_REG_LED1T1 , val);
        break;
    case     AW2013_T3_AND_T4_WRITE_LED2T1:
        ret = aw2013_i2c_write(breaths_dev , AW2013_REG_LED2T1 , val);
        break;
    case     AW2013_T0_AND_REPEAT_WRITE_LED0T2:
        ret = aw2013_i2c_write(breaths_dev , AW2013_REG_LED0T2 , val);
        break;
    case     AW2013_T0_AND_REPEAT_WRITE_LED1T2:
        ret = aw2013_i2c_write(breaths_dev , AW2013_REG_LED1T2 , val);
        break;
    case     AW2013_T0_AND_REPEAT_WRITE_LED2T2:
        ret = aw2013_i2c_write(breaths_dev , AW2013_REG_LED2T2 , val);
        break;
    default :
        ret = -1;
    }
    if (ret < 0) {
        pr_err("I2C read or write err");
    }
    return ret;
}

static int enable_breath_light(void){
    int  err = aw2013_exec_command(breath_dev , AW2013_GLOBAL_CONTROL,0x01);
    if (err < 0){
        pr_err("AW2013_GLOBAL_CONTROL is failed");
        return err;
    }
    return err;
}

void channel_enable_breath_light(void){
    aw2013_exec_command(breath_dev , AW2013_LED_CONTROL_WRITE,0x07);
}
void breath_light_ctr(int id){
    pr_debug("breath_light_ctr %d" ,id);
    aw2013_exec_command(breath_dev , AW2013_MODE_CONFIG_WRITE_LCFG0+id,0x0);
    //aw2013_exec_command(breath_dev , AW2013_PWM_CONFIG_WRITE_PWM0+id,0x3f);
    aw2013_exec_command(breath_dev , AW2013_T1_AND_T2_WRITE_LED0T0+id,Rise_time<<4 | Hold_time);
    aw2013_exec_command(breath_dev , AW2013_T3_AND_T4_WRITE_LED0T1+id,Fall_time<<4 | Off_time);
    aw2013_exec_command(breath_dev , AW2013_T0_AND_REPEAT_WRITE_LED0T2+id, Delay_time<<4| Period_Num);
}

void set_breath_light_brightness(void){
    pr_debug("set_breath_light_brightness ");

    aw2013_exec_command(breath_dev , AW2013_PWM_CONFIG_WRITE_PWM0,0xB8);
    aw2013_exec_command(breath_dev , AW2013_PWM_CONFIG_WRITE_PWM0+1,0x1f);
    aw2013_exec_command(breath_dev , AW2013_PWM_CONFIG_WRITE_PWM0+2,0x5f);

}

void set_blink(int id, u32 state){
    //u32 pwms_flag = breath_dev->breath_light[id].pwm_flag;
    int ret;
    u32 pwms_flag = state -1;
    if ( pwms_flag > sizeof(aw2013_times)/sizeof(aw2013_times[0]) )
         pwms_flag = sizeof(aw2013_times)/sizeof(aw2013_times[0]) - 1;
    aw2013_exec_command(breath_dev , AW2013_T1_AND_T2_WRITE_LED0T0+id,aw2013_times[pwms_flag].T1 << 4 | aw2013_times[pwms_flag].T2);
    aw2013_exec_command(breath_dev , AW2013_T3_AND_T4_WRITE_LED0T1+id,aw2013_times[pwms_flag].T3 << 4 | aw2013_times[pwms_flag].T4);
    aw2013_exec_command(breath_dev , AW2013_MODE_CONFIG_WRITE_LCFG0+id,Imax|0x01<<6|0x01<<5|0x01<<4);
    ret = aw2013_i2c_read(breath_dev,AW2013_REG_LCTR);
    aw2013_exec_command(breath_dev , AW2013_LED_CONTROL_WRITE, ret | (1<<id) );
}
void set_off(int id){
    int ret;
    aw2013_exec_command(breath_dev , AW2013_MODE_CONFIG_WRITE_LCFG0+id,0x0);
    ret = aw2013_i2c_read(breath_dev,AW2013_REG_LCTR);
    aw2013_exec_command(breath_dev , AW2013_LED_CONTROL_WRITE, ret & (~(1<<id)) );
}
void set_brightness(int id){
    int ret;
    aw2013_exec_command(breath_dev , AW2013_MODE_CONFIG_WRITE_LCFG0+id,Imax);
    ret = aw2013_i2c_read(breath_dev,AW2013_REG_LCTR);
    aw2013_exec_command(breath_dev , AW2013_LED_CONTROL_WRITE, ret | (1<<id) );
}

struct aw2013_regulator {
    struct regulator *vreg;
    const char *name;
    u32	min_uV;
    u32	max_uV;
};
struct aw2013_regulator aw2013_vreg[] = {
                                            {NULL, "vdd", 1700000, 3600000},
                                            {NULL, "vio", 1700000, 3600000},
                                        };


static int  aw2013_power_on(void){
    int rc = 0,i;
    int num_reg = sizeof(aw2013_vreg) / sizeof(struct aw2013_regulator);

    for (i = 0; i < num_reg; i++) {
        aw2013_vreg[i].vreg =
            regulator_get(&breath_dev->client->dev,
                          aw2013_vreg[i].name);
        if (IS_ERR(aw2013_vreg[i].vreg)) {
            rc = PTR_ERR(aw2013_vreg[i].vreg);
            pr_err("%s:regulator get failed rc=%d\n",
                   __func__, rc);
            aw2013_vreg[i].vreg = NULL;
            return -1;
        }

        if (regulator_count_voltages(
                    aw2013_vreg[i].vreg) > 0) {
            rc = regulator_set_voltage(
                     aw2013_vreg[i].vreg,
                     aw2013_vreg[i].min_uV,
                     aw2013_vreg[i].max_uV);
            if (rc) {
                pr_err("%s: set voltage failed rc=%d\n",
                       __func__, rc);
                regulator_put(aw2013_vreg[i].vreg);
                aw2013_vreg[i].vreg = NULL;
                return -1;
            }
        }

        rc = regulator_enable(aw2013_vreg[i].vreg);
        if (rc) {
            pr_err("%s: regulator_enable failed rc =%d\n",
                   __func__, rc);
            if (regulator_count_voltages(
                        aw2013_vreg[i].vreg) > 0) {
                regulator_set_voltage(
                    aw2013_vreg[i].vreg, 0,
                    aw2013_vreg[i].max_uV);
            }
            regulator_put(aw2013_vreg[i].vreg);
            aw2013_vreg[i].vreg = NULL;
            return -1;
        }
    }
    return rc;

}

static ssize_t led_blink_solid_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
    struct breaths_light_dev *breath_dev;
    int idx = 1 ;
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    ssize_t ret = 0;

    if (!strcmp(led_cdev->name , "green"))
        idx = 1;
    else if (!strcmp(led_cdev->name , "blue"))
        idx = 0;
    else
        idx = 2;

    breath_dev = container_of(led_cdev , struct breaths_light_dev , breath_light[idx].led);

    sprintf(buf, "%u\n", breath_dev->breath_light[idx].blink_flag);
    ret = strlen(buf) + 1;


    return ret;
}

static ssize_t led_blink_solid_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t size)
{

    struct breaths_light_dev *breaths_dev;
    int idx = 1;
    char *after;
    unsigned long state = 0;
    ssize_t ret = -EINVAL;
    size_t count = 0;

    struct led_classdev *led_cdev = dev_get_drvdata(dev);

    if (!strcmp(led_cdev->name, "green"))
        idx = 1;
    else if (!strcmp(led_cdev->name, "blue"))
        idx = 0;
    else
        idx =2;
    breaths_dev = container_of(led_cdev, struct breaths_light_dev, breath_light[idx].led);
    state = simple_strtoul(buf, &after, 10);
    count = after - buf;
    if (*after && isspace(*after))
        count++;

    if (count == size) {
        ret = count;
        if (AW2013_LED_OFF == state)
        {
            set_off(idx);
            breath_dev->breath_light[idx].blink_flag =0;
        }
        else
        {
            if (state >  AW2013_LED_BREATH_FAST || state < AW2013_LED_OFF)
                state = AW2013_LED_BREATH_MID;
            set_blink(idx, state);
            breath_dev->breath_light[idx].blink_flag = state;
        }

    }

    return ret;


}

static DEVICE_ATTR(blink, 0644, led_blink_solid_show, led_blink_solid_store);

static ssize_t led_pwm_solid_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
    struct breaths_light_dev *breath_dev;
    int idx = 1 ;
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    ssize_t ret = 0;

    if (!strcmp(led_cdev->name , "green"))
        idx = 1;
    else if (!strcmp(led_cdev->name , "blue"))
        idx = 0;
    else
        idx = 2;

    breath_dev = container_of(led_cdev , struct breaths_light_dev , breath_light[idx].led);

    sprintf(buf, "%u\n", breath_dev->breath_light[idx].pwm_flag);
    ret = strlen(buf) + 1;


    return ret;
}



static ssize_t led_pwm_solid_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf, size_t size)
{
    struct breaths_light_dev *breaths_dev;
    int idx = 1;
    char *after;
    unsigned long state = 0;
    ssize_t ret = -EINVAL;
    size_t count = 0;

    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    if (!strcmp(led_cdev->name, "green"))
        idx = 1;
    else if (!strcmp(led_cdev->name, "blue"))
        idx = 0;
    else
        idx =2;
    breaths_dev = container_of(led_cdev, struct breaths_light_dev, breath_light[idx].led);
    state = simple_strtoul(buf, &after, 10);
    count = after - buf;
    if (*after && isspace(*after))
        count++;
    if (count == size) {
        ret = count;
        pr_err("state : %lu  idx :%d" ,state ,idx);
        if (state > 3)
            state =2 ;
        breath_dev->breath_light[idx].pwm_flag = state;

    }
    return ret;

}



static DEVICE_ATTR(pwm, 0644, led_pwm_solid_show, led_pwm_solid_store);

static void mem_pmic_breath_light_set(struct led_classdev *led_cdev,
                                      enum led_brightness value){

    int idx = 1;
    pr_err( "Enter mem_pmic_breath_light_set\n");
    if (!strcmp(led_cdev->name, "green"))
        idx = 1;
    else if (!strcmp(led_cdev->name, "blue"))
        idx = 0;
    else if(!strcmp(led_cdev->name, "red"))
        idx = 2;
    else 
    	  idx = 0;
    if (value == 0)
    {
        set_off(idx);
        if(!strcmp(led_cdev->name, "button-backlight"))
        	set_off(2);
    } else {
        set_brightness(idx);
        if(!strcmp(led_cdev->name, "button-backlight"))
        	set_brightness(2);
    }

}

static int set_reg(void *data, u64 val)
{
	u8 addr = (u8) *((u8*)data);
	int ret;

       printk(KERN_ERR"data=%s, *data=0x%x\n", (u8*)data, *(u32*)data);

	ret = aw2013_i2c_write(breath_dev, addr, (u8) val);

	return ret;
}

static int get_reg(void *data, u64 *val)
{
	u8 addr = (u8) *((u8*)data);
	
	int ret;

       printk(KERN_ERR"data=%s, *data=0x%x\n", (u8*)data, *(u32*)data);
       
	ret = aw2013_i2c_read(breath_dev, addr);
	if (ret < 0)
		return ret;

	*val = ret;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(aw2013_reg_fops, get_reg, set_reg, "0x%02llx\n");

static int aw2013_create_debugfs_entries(struct breaths_light_dev  *dev)
{
	int i;
       struct dentry *file;

	dev->dent = debugfs_create_dir(AW2013_DRIVER_NAME, NULL);
	if (IS_ERR(dev->dent)) {
		pr_err("aw2013 driver couldn't create debugfs dir\n");
		return -EFAULT;
	}

	for (i = 0 ; i < ARRAY_SIZE(aw2013_debug_regs) ; i++) {
		char *name = aw2013_debug_regs[i].name;
            
		file = debugfs_create_file(name, 0644, dev->dent,
					(void *) (&(aw2013_debug_regs[i].reg)), &aw2013_reg_fops);
		if (IS_ERR(file)) {
			pr_err("debugfs_create_file %s failed.\n", name);
			return -EFAULT;
		}
	}

	return 0;
}

static int aw2013_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

    int err = -1;
    int i , j;
    struct device_node *node = NULL;

    printk(KERN_ERR"%s: enter\n", __func__);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        err = -ENODEV;
        goto exit_check_functionality_failed;
    }
    breath_dev = kzalloc(sizeof(struct breaths_light_dev),GFP_KERNEL);
    if (breath_dev == NULL){
        err = -ENOMEM;
        goto free_breath_dev;
    }

    mutex_init(&breath_dev->lock );

    breath_dev->client = client;

    if (0){
        err = aw2013_power_on();
        if (err < 0) {
            dev_err(&client->dev, "power on failed: %d\n", err);
            goto free_breath_dev;
        }
    }

    if (client->dev.of_node) {
        node = client->dev.of_node;
    
        has_green = of_property_read_bool(node,
                                        "awinic,has_green");
        if (has_green){
            breath_dev->breath_light[1].led.name = "green";
            breath_dev->breath_light[1].led.brightness_set = mem_pmic_breath_light_set;
            breath_dev->breath_light[1].led.brightness = LED_OFF;
            breath_dev->breath_light[1].blink_flag = 0;
            breath_dev->breath_light[1].pwm_flag = 0;
        }
        has_buttonlight = of_property_read_bool(node,
                                        "awinic,has_buttonlight");
        if(!has_buttonlight){
            has_blue = of_property_read_bool(node,
                                              "awinic,has_blue");
            if (has_blue){
                breath_dev->breath_light[0].led.name = "blue";
                breath_dev->breath_light[0].led.brightness_set = mem_pmic_breath_light_set;
                breath_dev->breath_light[0].led.brightness = LED_OFF;
                breath_dev->breath_light[0].blink_flag = 0;
                breath_dev->breath_light[0].pwm_flag = 0;
            }

            has_red = of_property_read_bool(node,
                                             "awinic,has_red");
            if (has_red){
                breath_dev->breath_light[2].led.name = "red";
                breath_dev->breath_light[2].led.brightness_set = mem_pmic_breath_light_set;
                breath_dev->breath_light[2].led.brightness = LED_OFF;
                breath_dev->breath_light[2].blink_flag = 0;
                breath_dev->breath_light[2].pwm_flag = 0;
            }
        }else{
            breath_dev->breath_light[0].led.name = "button-backlight";
            breath_dev->breath_light[0].led.brightness_set = mem_pmic_breath_light_set;
            breath_dev->breath_light[0].led.brightness = LED_OFF;
            breath_dev->breath_light[0].blink_flag = 0;
            breath_dev->breath_light[0].pwm_flag = 0;
        }
        for ( i = 0 ; i < 3 ; i++){
            if ((i == 1)&&(!has_red))
                continue;
            else if (((i == 0)&&(!has_green))&&((i == 0) && (!has_buttonlight)))
                continue;
            else if ((i == 2)&&(!has_blue))
                continue;
            err = led_classdev_register(&client -> dev , &breath_dev -> breath_light[i].led );
            if (err){
                dev_err(&client -> dev  ,"breath_dev : led_classdev_register failed\n");
                goto err_led_classdev_register_failed;
            }
        }

        for ( i = 0 ; i < 3 ; i++){
            if ((i == 1)&&(!has_red))
                continue;
            else if ((i == 0)&&(!has_green))
                continue;
            else if ((i == 2)&&(!has_blue))
                continue;
            err = device_create_file(breath_dev->breath_light[i].led.dev, &dev_attr_blink);
            if (err) {
                dev_err(&client->dev,"breath_dev: create dev_attr_blink failed\n");
                goto err_out_attr_blink;
            }
        }
		
        for ( i = 0 ; i < 3 ; i++){
            if ((i == 1)&&(!has_red))
                continue;
            else if ((i == 0)&&(!has_green))
                continue;
            else if ((i == 2)&&(!has_blue))
                continue;
            err = device_create_file(breath_dev->breath_light[i].led.dev, &dev_attr_pwm);
            if (err) {
                dev_err(&client->dev,"breath_dev: create dev_attr_pwm failed\n");
                goto err_out_attr_pwm;
            }
        }
		
        dev_set_drvdata(&client->dev, breath_dev);

        aw2013_create_debugfs_entries( breath_dev);

	 aw2013_exec_command(breath_dev,AW2013_SW_RESET,0x55);	
        err = aw2013_exec_command(breath_dev,AW2013_READ_THE_CHIP_ID,0);
        printk(KERN_ERR"%s: read aw2013 chip id=0x%x\n", __func__, err);

        enable_breath_light();
        channel_enable_breath_light();
        breath_light_ctr(0);
        breath_light_ctr(1);
        breath_light_ctr(2);
        set_breath_light_brightness();


        return 0;
    }

err_out_attr_pwm :
	
    for (j = 0; j < i; i++){
        if ((j == 1)&&(!has_red))
            continue;
        else if ((j == 0)&&(!has_green))
            continue;
        else if ((j == 2)&&(!has_blue))
            continue;
        device_remove_file(breath_dev->breath_light[i].led.dev, &dev_attr_pwm);
    }
	
err_out_attr_blink:
    for (j = 0; j < i; i++){
        if ((j == 1)&&(!has_red))
            continue;
        else if ((j == 0)&&(!has_green))
            continue;
        else if ((j == 2)&&(!has_blue))
            continue;
        device_remove_file(breath_dev->breath_light[i].led.dev, &dev_attr_blink);
    }
err_led_classdev_register_failed:
    for ( j = 0; j < i; i++){
        if ((j == 1)&&(!has_red))
            continue;
        else if (((i == 0)&&(!has_green))&&((i == 0) && (!has_buttonlight)))
            continue;
        else if ((j == 2)&&(!has_blue))
            continue;
        led_classdev_unregister(&breath_dev -> breath_light[j].led);
    }

free_breath_dev:
    kfree(breath_dev);
exit_check_functionality_failed:
    dev_err(&client->dev, "%s: Driver Init failed\n", AW2013_DRIVER_NAME);

    return err;
}
static int aw2013_remove(struct i2c_client *client)
{

    int i;

    for (i = 0; i < 3; i++){
        if ((i == 1)&&(!has_red))
            continue;
        else if (((i == 0)&&(!has_green))&&((i == 0) && (!has_buttonlight)))
            continue;
        else if ((i == 2)&&(!has_blue))
            continue;
        if((i != 0)||((i ==0)&&(!has_buttonlight))){
        	   device_remove_file(breath_dev->breath_light[i].led.dev, &dev_attr_blink);
             device_remove_file(breath_dev->breath_light[i].led.dev, &dev_attr_pwm);
      }
        led_classdev_unregister(&breath_dev -> breath_light[i].led);
    }
    kfree(breath_dev);
    return 0;
}

static const struct i2c_device_id aw2013_id[] = {
            { AW2013_DRIVER_NAME,0 },
            { }
        };

static struct of_device_id nfc_match_table[] = {
            {.compatible = "awin,led-control",},
                           { },
                       };

static struct i2c_driver aw2013_driver = {
            .id_table	= aw2013_id,
            .probe       = aw2013_probe,
            .remove	= aw2013_remove,
            .driver	= {
                                 .name   = "aw2013",
                                 .owner = THIS_MODULE,
                                  .of_match_table = nfc_match_table,
                                   },
             };

static int __init aw2013_dev_init(void)
{
    pr_err("Loading aw2013 driver\n");
    printk(KERN_ERR"Loading aw2013 driver\n");
    return i2c_add_driver(&aw2013_driver);
}
//module_init(aw2013_dev_init);
__define_initcall(aw2013_dev_init, 7s);

static void __exit aw2013_dev_exit(void)
{
    i2c_del_driver(&aw2013_driver);
}
module_exit(aw2013_dev_exit);

MODULE_AUTHOR("ZTE");
MODULE_DESCRIPTION("AW2013 LED control");
MODULE_LICENSE("GPL");

