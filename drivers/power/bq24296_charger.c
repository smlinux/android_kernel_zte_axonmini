/* Copyright (c) 2013 LGE Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/power_supply.h>
#include <linux/bitops.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>
#include <linux/qpnp/qpnp-adc.h>

#define BQ24296_NAME "BQ24296-CHG"

#ifndef BIT
#define BIT(x)	(1 << (x))
#endif

/* Register definitions */
#define BQ00_INPUT_SRC_CONT_REG              0X00
#define BQ01_PWR_ON_CONF_REG                 0X01
#define BQ02_CHARGE_CUR_CONT_REG             0X02
#define BQ03_PRE_CHARGE_TERM_CUR_REG         0X03
#define BQ04_CHARGE_VOLT_CONT_REG            0X04
#define BQ05_CHARGE_TERM_TIMER_CONT_REG      0X05
#define BQ06_IR_COMP_THERM_CONT_REG          0X06
#define BQ07_MISC_OPERATION_CONT_REG         0X07
#define BQ08_SYSTEM_STATUS_REG               0X08
#define BQ09_FAULT_REG                       0X09
#define BQ0A_VENDOR_PART_REV_STATUS_REG      0X0A

/* BQ00 Input Source Control Register MASK */
#define EN_HIZ			BIT(7)
#define VINDPM_MASK 		(BIT(6)|BIT(5)|BIT(4)|BIT(3))
#define IINLIM_MASK 		(BIT(2)|BIT(1)|BIT(0))
#define INPUT_VOLTAGE_LIMIT_MAX 4400

/* BQ01 Power-On Configuration  Register MASK */
#define RESET_REG_MASK		BIT(7)
#define CHG_CONFIG_MASK 	(BIT(5)|BIT(4))
#define OTG_ENABLE_MASK         BIT(5)

/* #define SYSTEM_MIN_VOLTAGE_MASK    0x0E */
#define SYS_MIN_VOL_MASK	(BIT(3)|BIT(2)|BIT(1))
#define BOOST_LIM 		BIT(0)

/* BQ02 Charge Current Control Register MASK */
#define ICHG_MASK 		(BIT(7)|BIT(6)|BIT(5)|BIT(4)|BIT(3)|BIT(2))
#define FORCE_20PCT_MASK	BIT(0)

/* BQ03 Pre-Charge, Termination Current Control Register MASK */
#define IPRECHG_MASK 		(BIT(7)|BIT(6)|BIT(5)|BIT(4))
#define ITERM_MASK		(BIT(3)|BIT(2)|BIT(1)|BIT(0))

/* BQ04 Charge Voltage Control Register MASK */
#define CHG_VOLTAGE_LIMIT_MASK 	(BIT(7)|BIT(6)|BIT(5)|BIT(4)|BIT(3)|BIT(2))
#define BATLOWV_MASK 		BIT(1)
#define VRECHG_MASK 		BIT(0)

/* BQ05 Charge Termination, Timer-Control Register MASK */
#define EN_CHG_TERM_MASK 	BIT(7)
#define EN_CHG_BATFET_RST_MASK 	BIT(6)
#define I2C_TIMER_MASK          (BIT(5)|BIT(4))
#define EN_CHG_TIMER_MASK	BIT(3)
#define CHG_TIMER_MASK 		(BIT(2)|BIT(1))

/* BQ06 IR Compensation, Thermal Regulation Control Register MASK */
#define IR_COMP_R_MASK		(BIT(7)|BIT(6)|BIT(5))
#define IR_COMP_VCLAMP_MASK 	(BIT(4)|BIT(3)|BIT(2))

/* BQ07 Misc-Operation Control Register MASK */
#define BATFET_DISABLE_MASK 	BIT(5)

/* BQ08 SYSTEM_STATUS_REG Mask */
#define VBUS_STAT_MASK 		(BIT(7)|BIT(6))
#define PRE_CHARGE_MASK 	BIT(4)
#define FAST_CHARGE_MASK 	BIT(5)
#define CHRG_STAT_MASK		(FAST_CHARGE_MASK|PRE_CHARGE_MASK)
#define DPM_STAT_MASK		BIT(3)
#define PG_STAT_MASK		BIT(2)
#define THERM_STAT_MASK 	BIT(1)
#define VSYS_STAT_MASK 		BIT(0)

/* BQ09 FAULT_REG Mask */
#define CHRG_FAULT_MASK 	(BIT(5)|BIT(4))


#define _WIRELESS_ "wireless"
#define _BATTERY_     "battery"
#define _USB_		"usb"
#define _CN_		"cn"
#define _THIS_		"ac"

#define MONITOR_BATTEMP_POLLING_PERIOD      msecs_to_jiffies(2000)

#define DISABLE_CHG_TIMER_FOR_MTBF 1

enum bq24296_chg_status {
	BQ_CHG_STATUS_NONE 		= 0,
	BQ_CHG_STATUS_PRE_CHARGE	= 1,
	BQ_CHG_STATUS_FAST_CHARGE 	= 2,
	BQ_CHG_STATUS_FULL 		= 3,
	BQ_CHG_STATUS_EXCEPTION		= 4,
};

static const char * const bq24296_chg_status[] = {
	"none",
	"pre-charge",
	"fast-charge",
	"full"
	"exception"
};
#define NULL_CHECK_VOID(p)  \
			if (!(p)) { \
				pr_err("FATAL (%s)\n", __func__); \
				return ; \
			}
#define NULL_CHECK(p, err)  \
			if (!(p)) { \
				pr_err("FATAL (%s)\n", __func__); \
				return err; \
			}

struct bq24296_chip {
	struct i2c_client  *client;
	struct dentry  *dent;
	//struct switch_dev batt_removed;

       int input_current_force;
       int input_current_max;
	int chg_current_ma;
	int term_current_ma;
	int vbat_max_mv;
	int pre_chg_current_ma;
	int sys_vmin_mv;
	int vin_limit_mv;
	int int_gpio;
       int chg_en_gpio;
	int ext_chg_en;
	int otg_en;
	int irq;
	int usb_present;
	int usb_online;
	int ac_present;
	int ac_online;
	int chg_type;
	bool charging_disabled;
	int full_design;
	bool chg_timeout;
	int icl_vbus_mv;
	int icl_idx;
	bool icl_first;
	int icl_fail_cnt;
	int set_icl_idx;
	struct wake_lock icl_wake_lock;
	enum bq24296_chg_status	chg_status;

	struct delayed_work  irq_work;

	struct wake_lock  chg_wake_lock;

	struct power_supply  *usb_psy;
       struct power_supply  *bms_psy;
	struct power_supply  ac_psy;
	struct power_supply  batt_psy;
	struct power_supply  *psy_this;
	struct power_supply  *cn_psy;

       struct qpnp_vadc_chip		*vadc_dev;
    
	/* TODO: should be verify */
	struct wake_lock uevent_wake_lock;

	int  set_chg_current_ma;
	struct wake_lock battgone_wake_lock;
	struct wake_lock chg_timeout_lock;

	bool watchdog;

	struct delayed_work usbin_mon;
       struct delayed_work battemp_work;
	struct mutex usbin_lock;
	int			usbin_ref_count;
	int			last_usbin_mv;

	bool batt_present;

};

#ifdef CONFIG_SENSORS_QPNP_ADC_VOLTAGE
int last_batt_temp;
#endif

struct bq24296_chip *the_bq24296_chip;

struct debug_reg {
	char  *name;
	u8  reg;
};

#define BQ24296_DEBUG_REG(x, y) {#x#y, y##_REG}

static struct debug_reg bq24296_debug_regs[] = {
	BQ24296_DEBUG_REG(00_, BQ00_INPUT_SRC_CONT),
	BQ24296_DEBUG_REG(01_, BQ01_PWR_ON_CONF),
	BQ24296_DEBUG_REG(02_, BQ02_CHARGE_CUR_CONT),
	BQ24296_DEBUG_REG(03_, BQ03_PRE_CHARGE_TERM_CUR),
	BQ24296_DEBUG_REG(04_, BQ04_CHARGE_VOLT_CONT),
	BQ24296_DEBUG_REG(05_, BQ05_CHARGE_TERM_TIMER_CONT),
	BQ24296_DEBUG_REG(06_, BQ06_IR_COMP_THERM_CONT),
	BQ24296_DEBUG_REG(07_, BQ07_MISC_OPERATION_CONT),
	BQ24296_DEBUG_REG(08_, BQ08_SYSTEM_STATUS),
	BQ24296_DEBUG_REG(09_, BQ09_FAULT),
	BQ24296_DEBUG_REG(0A_, BQ0A_VENDOR_PART_REV_STATUS),
};

enum dwc3_chg_type {
	DWC3_INVALID_CHARGER = 0,
	DWC3_SDP_CHARGER,
	DWC3_DCP_CHARGER,
	DWC3_CDP_CHARGER,
	DWC3_PROPRIETARY_CHARGER,
	DWC3_FLOATED_CHARGER,
};

bool globle_charging_done = false;
bool globle_usb_present = false;

extern int input_current_mA_globle;
extern int globle_vm_bms_soc;

static int bq24296_enable_charging(struct bq24296_chip *chip, bool enable);
static bool bq24296_is_charger_present(struct bq24296_chip *chip);

enum batt_id_type batt_id = BATT_ID_UNKNOWN;
bool usb_inserted_while_booting=false;

extern int qpnp_set_usb_suspend( bool enable);

int g_boot_ffbm = 0;
int g_boot_charger = 0;
static int __init bootmode_set(char *str)
{
    if(strstr(str,"ffbm-01")) 
    {
        g_boot_ffbm = 1;
    }
    else if (strstr(str,"charger"))
    {
        g_boot_charger = 1;
    }

    return 1;
}
__setup("androidboot.mode=", bootmode_set);

static int bq24296_read_reg(struct i2c_client *client, int reg, u8 *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev,
			"i2c read fail: can't read from %02x: %d\n",
			reg, ret);
		return ret;
	} else {
		*val = ret;
	}

	return 0;
}

static int bq24296_write_reg(struct i2c_client *client, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		dev_err(&client->dev,
			"i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);
		return ret;
	}
	return 0;
}

static int bq24296_masked_write(struct i2c_client *client, int reg,
			       u8 mask, u8 val)
{
	s32 rc;
	u8 temp;

	rc = bq24296_read_reg(client, reg, &temp);
	if (rc) {
		pr_err("bq24296_read_reg failed: reg=%03X, rc=%d\n",
				reg, rc);
		return rc;
	}

	temp &= ~mask;
	temp |= val & mask;

	rc = bq24296_write_reg(client, reg, temp);
	if (rc) {
		pr_err("bq24296_write failed: reg=%03X, rc=%d\n",
				reg, rc);
		return rc;
	}

	return 0;
}

static int set_reg(void *data, u64 val)
{
	int addr = (u32) *((u8*)data);
	int ret;
	struct i2c_client *client = the_bq24296_chip->client;

       printk(KERN_ERR"data=%s, *data=0x%x\n", (u8*)data, *(u32*)data);
       
       //ret = kstrtoul((u8*)data, 16, &addr);
	ret = bq24296_write_reg(client, (int)addr, (u8) val);

	return ret;
}

static int get_reg(void *data, u64 *val)
{
	int addr = (u32) *((u8*)data);
	
	u8 temp;
	int ret;
	struct i2c_client *client = the_bq24296_chip->client;

       printk(KERN_ERR"data=%s, *data=0x%x\n", (u8*)data, *(u32*)data);
       
       //ret = kstrtoul((u8*)data, 16, &addr);
       
	ret = bq24296_read_reg(client, (int)addr, &temp);
	if (ret < 0)
		return ret;

	*val = temp;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(reg_fops, get_reg, set_reg, "0x%02llx\n");

static int bq24296_create_debugfs_entries(struct bq24296_chip *chip)
{
	int i;
       //char* addr;
       struct dentry *file;
	NULL_CHECK(chip, -EINVAL);
	chip->dent = debugfs_create_dir(BQ24296_NAME, NULL);
	if (IS_ERR(chip->dent)) {
		pr_err("bq24296 driver couldn't create debugfs dir\n");
		return -EFAULT;
	}

	for (i = 0 ; i < ARRAY_SIZE(bq24296_debug_regs) ; i++) {
		char *name = bq24296_debug_regs[i].name;
		//u32 reg = bq24296_debug_regs[i].reg;

              //snprintf(&addr, sizeof(reg), "%u\n", reg);
              //addr = (u32*)reg;
            
		file = debugfs_create_file(name, 0644, chip->dent,
					(void *) (&(bq24296_debug_regs[i].reg)), &reg_fops);
		if (IS_ERR(file)) {
			pr_err("debugfs_create_file %s failed.\n", name);
			return -EFAULT;
		}
	}

	return 0;
}

static void bq24296_reginfo(struct bq24296_chip *chip)
{
	int i;
	int cnt = ARRAY_SIZE(bq24296_debug_regs);
	u8 val[cnt];

	NULL_CHECK_VOID(chip);

	for (i = 0; i < cnt; i++)
		bq24296_read_reg(chip->client,
			bq24296_debug_regs[i].reg, &val[i]);

	pr_debug("bq24296_reginfo 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
		val[0], val[1], val[2], val[3], val[4],
		val[5], val[6], val[7], val[8], val[9]);
}

#define IBAT_MAX_MA 3008
#define IBAT_MIN_MA  512
#define IBAT_DIS_MA  (100)
#define IBAT_STEP_MA  64
#define IBAT_DEFAULT  2048
#define IBAT_NORMAL  2496

/* ~0.5C bit6 1024 + bit4 256*/
#define IBAT_05C 1280

/* 0.2C=2800*0.2=560mA, set 512mA */
#define IBAT_02C 512

/* ~0.3C  bit5 512 + bit4 256*/
#define IBAT_03C 768

static int bq24296_set_ibat_max(struct bq24296_chip *chip, int ma)
{
	u8 reg_val = 0;
	int set_ibat = 0;

	NULL_CHECK(chip, -EINVAL);

	if (ma < IBAT_MIN_MA) {
		bq24296_enable_charging(chip,false);
		ma = IBAT_MIN_MA;
	} else {
		bq24296_enable_charging(chip, true);
	}
	if (ma > IBAT_MAX_MA) {
		ma = IBAT_MAX_MA;
	}

	reg_val = (ma - IBAT_MIN_MA)/IBAT_STEP_MA;
	set_ibat = reg_val * IBAT_STEP_MA + IBAT_MIN_MA;
	reg_val = reg_val << 2;
	chip->set_chg_current_ma = set_ibat;
	pr_debug("req_ibat = %d set_ibat = %d reg_val = 0x%02x\n",
				ma, set_ibat, reg_val);

	return bq24296_masked_write(chip->client, BQ02_CHARGE_CUR_CONT_REG,
			ICHG_MASK, reg_val);
}

#define VBAT_MAX_MV  4400
#define VBAT_NORMAL_MV  4352
#define VBAT_MIN_MV  3504
#define VBAT_STEP_MV  16
static int bq24296_set_vbat_max(struct bq24296_chip *chip, int mv)
{
	u8 reg_val = 0;
	int set_vbat = 0;
	NULL_CHECK(chip, -EINVAL);
	if (mv < VBAT_MIN_MV)
		mv = VBAT_MIN_MV;
	if (mv > VBAT_MAX_MV)
		mv = VBAT_MAX_MV;

	reg_val = (mv - VBAT_MIN_MV)/VBAT_STEP_MV;
	set_vbat = reg_val * VBAT_STEP_MV + VBAT_MIN_MV;
	reg_val = reg_val << 2;

	pr_debug("req_vbat = %d set_vbat = %d reg_val = 0x%02x\n",
				mv, set_vbat, reg_val);

	return bq24296_masked_write(chip->client, BQ04_CHARGE_VOLT_CONT_REG,
			CHG_VOLTAGE_LIMIT_MASK, reg_val);
}

struct input_ma_limit_entry {
	int  icl_ma;
	u8  value;
};

static struct input_ma_limit_entry icl_ma_table[] = {
	{100, 0x00},
	{150, 0x01},
	{500, 0x02},
	{900, 0x03},
	{1000, 0x04},
	{1500, 0x05},
	{2000, 0x06},
	{3000, 0x07},
};

#define INPUT_CURRENT_LIMIT_MIN_MA  100
#define INPUT_CURRENT_LIMIT_MAX_MA  2000
#define INPUT_CURRENT_LIMIT_DC      1500
#define INPUT_CURRENT_LIMIT_USB20   500
#define INPUT_CURRENT_LIMIT_USB30   900

static int bq24296_set_input_i_limit(struct bq24296_chip *chip, int ma)
{
	int i;
	u8 temp;
	NULL_CHECK(chip, -EINVAL);

	for (i = ARRAY_SIZE(icl_ma_table) - 1; i > 0; i--) {
		if (ma >= icl_ma_table[i].icl_ma)
			break;
	}
	temp = icl_ma_table[i].value;

	printk(KERN_ERR"input current limit=%d setting 0x%02x\n", ma, temp);
	return bq24296_masked_write(chip->client, BQ00_INPUT_SRC_CONT_REG,
			IINLIM_MASK, temp);
}

struct input_mv_limit_entry {
	int  ivl_mv;
	u8  value;
};

static struct input_mv_limit_entry ivl_mv_table[] = {
	/*{3880, 0x00},
	{3960, 0x08},*/
	{4040, 0x10},
	{4120, 0x18},
	{4200, 0x20},
	{4280, 0x28},
	{4360, 0x30},
	{4440, 0x38},
	{4520, 0x40},
	{4600, 0x48},
	{4680, 0x50},
	{4760, 0x58},
};

#define CHG_DELTA_MV 400

static int bq24296_set_input_v_limit(struct bq24296_chip *chip, int mv)
{
       int i;
	u8 temp;
    
	NULL_CHECK(chip, -EINVAL);

       for (i = ARRAY_SIZE(ivl_mv_table) - 1; i > 0; i--) {
		if (mv + CHG_DELTA_MV >= ivl_mv_table[i].ivl_mv)
			break;
	}
	temp = ivl_mv_table[i].value;

	pr_err("input voltage set %dmv, vindpm %d, set reg value 0x%x\n", mv, ivl_mv_table[i].ivl_mv, temp);
	return bq24296_masked_write(chip->client, BQ00_INPUT_SRC_CONT_REG,
			VINDPM_MASK, temp);
}

int bq24296_set_input_current(int ma)
{
       int ma_set = ma;
       int ret;

       pr_err("%s: input current max=%d, input current force =%d\n", __func__, 
                          the_bq24296_chip->input_current_max, the_bq24296_chip->input_current_force);
       if ( the_bq24296_chip->input_current_max >= 0 )
       {
            ma_set = min(the_bq24296_chip->input_current_max, input_current_mA_globle);
            ma_set = min(ma_set, ma);
       }
       
       if ( the_bq24296_chip->input_current_force >= 0 )
            ma_set = the_bq24296_chip->input_current_force;

       ret = bq24296_set_input_i_limit(the_bq24296_chip, ma_set);

       return ret;
}

#define IPRECHG_MIN_MA  128
#define IPRECHG_MAX_MA  2048
#define IPRECHG_STEP_MA  128
#define IPRECHG_DEFAULT_MA  512
static int bq24296_set_prechg_i_limit(struct bq24296_chip *chip, int ma)
{
	u8 reg_val = 0;
	int set_ma = 0;
	NULL_CHECK(chip, -EINVAL);

	if (ma < IPRECHG_MIN_MA)
		ma = IPRECHG_MIN_MA;
	if (ma > IPRECHG_MAX_MA)
		ma = IPRECHG_MAX_MA;


	reg_val = (ma)/IPRECHG_STEP_MA;
	set_ma = reg_val * IPRECHG_STEP_MA;
	reg_val = reg_val << 4;

	pr_info("req_i = %d set_i = %d reg_val = 0x%02x\n",
				ma, set_ma, reg_val);

	return bq24296_masked_write(chip->client, BQ03_PRE_CHARGE_TERM_CUR_REG,
			IPRECHG_MASK, reg_val);
}

/*
#define VIN_LIMIT_MIN_MV  3880
#define VIN_LIMIT_MAX_MV  5080
#define VIN_LIMIT_STEP_MV  80
static int bq24296_get_input_vin_limit(struct bq24296_chip *chip, int *mv)
{
	u8 reg_val = 0;
	int ret;
	NULL_CHECK(chip, -EINVAL);

	ret = bq24296_read_reg(chip->client, BQ00_INPUT_SRC_CONT_REG, &reg_val);
	if (ret) {
		pr_debug("failed to read BQ00_SYSTEM_STATUS_REG ret=%d\n", ret);
		return ret;
	}
	*mv = (reg_val & VINDPM_MASK >> 3) * VIN_LIMIT_STEP_MV +
		VIN_LIMIT_MIN_MV;
	return ret;
}
*/
#define CHG_ENABLE_SHIFT  4
static int bq24296_enable_charging(struct bq24296_chip *chip, bool enable)
{
	//int ret;
	//u8 val = (u8)(!!enable << CHG_ENABLE_SHIFT);
	NULL_CHECK(chip, -EINVAL);

	pr_debug("charging enable=%d\n", enable);

	if (chip->chg_timeout) {
		pr_err("charging timeout state, never enabel charging\n");
		return 0;
	}
/*  not disable charging by register, if system shutdown by very-low-battery when over temp, sbl1 will not charge
	ret = bq24296_masked_write(chip->client, BQ01_PWR_ON_CONF_REG,
						CHG_CONFIG_MASK, val);
	if (ret) {
		pr_err("failed to set CHG_CONFIG ret=%d\n", ret);
		return ret;
	}
*/
       if ( !chip->charging_disabled )
       {
            //0: OE low, enable charging;  1: OE high, disable charging
            gpio_direction_output(chip->chg_en_gpio, !enable);
       }

	return 0;
}

static int get_battery_voltage(struct bq24296_chip *chip);
#define BATT_CHG_HIGH_VOLTAGE_UV 4000000

int bq24296_enable_charging_globle(int batt_temp)
{
        int batt_voltage;
        int usb_present;

        usb_present = bq24296_is_charger_present(the_bq24296_chip);
        batt_voltage = get_battery_voltage(the_bq24296_chip);

        if ( usb_present == 1)
        {
        
            if ( batt_temp >= 0 && batt_temp <= 450)
            {
                pr_debug("%s batt_temp=%d,  set ibat=%dmA\n", __func__, batt_temp, IBAT_NORMAL);
                bq24296_set_ibat_max(the_bq24296_chip, IBAT_NORMAL);
                bq24296_enable_charging(the_bq24296_chip, true);
            }
            else if (batt_temp > 450 && batt_temp <= 550)
            {                
                pr_err("%s batt_temp=%d,  set ibat=%dmA\n", __func__, batt_temp, IBAT_05C);
                bq24296_set_ibat_max(the_bq24296_chip, IBAT_05C);
                /* 
                if ( batt_voltage > BATT_CHG_HIGH_VOLTAGE_UV )
                {
                      pr_err("%s batt_temp=%d,  high vol=%d stop charging \n", __func__, batt_temp, batt_voltage);
                      bq24296_enable_charging(the_bq24296_chip, false);
                }
                else */
                bq24296_enable_charging(the_bq24296_chip, true);
            }
            else
            {
                pr_err("%s batt_temp=%d,   disable charging\n", __func__, batt_temp);
                bq24296_enable_charging(the_bq24296_chip, false);
            }
#if 1/* enable it for MTBF test if using old testing box with 500mA charger */
            pr_err("%s vbat=%dmv, set vindpm\n", __func__, batt_voltage/1000);
            bq24296_set_input_v_limit(the_bq24296_chip, batt_voltage/1000);
#endif
        }
        power_supply_changed(&the_bq24296_chip->batt_psy);
        return 0;
}

#define WATCHDOG_SHIFT  4
static int bq24296_enable_watchdog(struct bq24296_chip *chip, bool enable)
{
	int ret;
       u8 val;

       if ( enable == 0)
              val = 0;
       else
              val = I2C_TIMER_MASK;
       
	NULL_CHECK(chip, -EINVAL);

	printk(KERN_ERR"watchdog enable=%d\n", enable);

	ret = bq24296_masked_write(chip->client, BQ05_CHARGE_TERM_TIMER_CONT_REG,
						I2C_TIMER_MASK, val);
	if (ret) {
		pr_err("failed to set I2C watchdog ret=%d\n", ret);
		return ret;
	}

	chip->watchdog = enable;

	return 0;
}

static int bq24296_enable_longpress_reset_batfet(struct bq24296_chip *chip, bool enable)
{
	int ret;
       u8 val;

       if ( enable == 0)
              val = 0;
       else
              val = EN_CHG_BATFET_RST_MASK;
       
	NULL_CHECK(chip, -EINVAL);

	printk(KERN_ERR"watchdog enable=%d\n", enable);

	ret = bq24296_masked_write(chip->client, BQ05_CHARGE_TERM_TIMER_CONT_REG,
						EN_CHG_BATFET_RST_MASK, val);
	if (ret) {
		pr_err("failed to set I2C batfet reset. ret=%d\n", ret);
		return ret;
	}

	chip->watchdog = enable;

	return 0;
}

#ifdef DISABLE_CHG_TIMER_FOR_MTBF
#define CHG_TIMER_SHIFT  3
static int bq24296_enable_chg_timer(struct bq24296_chip *chip, bool enable)
{
	int ret;
	u8 val = (u8)(!!enable << CHG_TIMER_SHIFT);
	NULL_CHECK(chip, -EINVAL);

	printk(KERN_ERR"charge timer enable=%d\n", enable);

	ret = bq24296_masked_write(chip->client, BQ05_CHARGE_TERM_TIMER_CONT_REG,
						EN_CHG_TIMER_MASK, val);
	if (ret) {
		pr_err("failed to set charge timer ret=%d\n", ret);
		return ret;
	}

	return 0;
}
#endif

#define DEFAULT_TEMP		250
int bq24296_get_batt_temp_origin(void)
{
//#ifdef CONFIG_SENSORS_QPNP_ADC_VOLTAGE
#if 0
	int rc = 0;
	struct qpnp_vadc_result results;

	NULL_CHECK(the_bq24296_chip, -EINVAL);

	rc = qpnp_vadc_read(the_bq24296_chip->vadc_dev, LR_MUX1_BATT_THERM, &results);
	if (rc) {
		pr_debug("Unable to read batt temperature rc=%d\n", rc);
		pr_debug("Report last_bat_temp %d again\n", last_batt_temp);
		return last_batt_temp;
	} else {
		pr_debug("get_bat_temp %d %lld\n", results.adc_code, results.physical);
		last_batt_temp = (int)results.physical;
		return (int)results.physical;
	}
#else
	pr_err("CONFIG_SENSORS_QPNP_ADC_VOLTAGE is not defined.\n");
	return DEFAULT_TEMP;
#endif
}

EXPORT_SYMBOL(bq24296_get_batt_temp_origin);

static int __bq24296_get_prop_batt_present(struct bq24296_chip *chip)
{
	int temp = 0;
	bool batt_present;
	NULL_CHECK(chip, -EINVAL);

	temp = bq24296_get_batt_temp_origin();

	if (temp <= -300 || temp >= 790) {
		pr_err("\n\n  battery missing(%d) \n\n", temp);
		batt_present = 0;
	} else
		batt_present = 1;

	pr_debug("present=%d, chip->batt_present=%d\n",
		batt_present ? 1 : 0, chip->batt_present);

	return batt_present ? 1 : 0;
}

static int bq24296_get_prop_batt_present(struct bq24296_chip *chip)
{
	return chip->batt_present ? 1 :
		__bq24296_get_prop_batt_present(chip);
}

#define OTG_ENABLE_SHIFT  5
static int bq24296_enable_otg(struct bq24296_chip *chip, bool enable)
{
	int ret;
	u8 val = (u8)(!!enable << OTG_ENABLE_SHIFT);

	pr_err("otg enable = %d\n", enable);
	NULL_CHECK(chip, -EINVAL);

      if ( g_boot_charger )
      {
              pr_err("Poweroff charging, disable otg\n");
              return 0;
      }

	ret = bq24296_masked_write(chip->client, BQ01_PWR_ON_CONF_REG,
					OTG_ENABLE_MASK, val);
	if (ret) {
		pr_err("failed to set OTG_ENABLE_MASK rc=%d\n", ret);
		return ret;
	}

       //power_supply_set_present(chip->usb_psy, chip->usb_present);
	//power_supply_changed(&chip->batt_psy);
    
	return 0;
}

int bq24296_enable_otg_globe(bool enable)
{
	return bq24296_enable_otg(the_bq24296_chip, enable);
}
EXPORT_SYMBOL(bq24296_enable_otg_globe);

static bool bq24296_is_otg_mode(struct bq24296_chip *chip)
{
	u8 temp;
	int ret;
	NULL_CHECK(chip, -EINVAL);

	ret = bq24296_read_reg(chip->client, BQ01_PWR_ON_CONF_REG, &temp);
	if (ret) {
		pr_err("failed to read OTG_ENABLE_MASK rc=%d\n", ret);
		return false;
	}

	return !!(temp & OTG_ENABLE_MASK);
}

static int bq24296_get_prop_charge_type(struct bq24296_chip *chip)
{
	int ret = 0;
	u8 sys_status;
	enum bq24296_chg_status status;
	int chg_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	NULL_CHECK(chip, -EINVAL);
	ret = bq24296_read_reg(chip->client, BQ08_SYSTEM_STATUS_REG, &sys_status);
	if (ret) {
		pr_err("fail to read BQ08_SYSTEM_STATUS_REG. ret=%d\n", ret);
		chg_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		goto exception_handling;
	}

	sys_status &= CHRG_STAT_MASK;
	if (sys_status == 0x10) {
		chg_type = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		status = BQ_CHG_STATUS_PRE_CHARGE;
	} else if (sys_status == 0x20) {
		chg_type = POWER_SUPPLY_CHARGE_TYPE_FAST;
		status = BQ_CHG_STATUS_FAST_CHARGE;
	} else if (sys_status == 0x30) {
	       if ( globle_vm_bms_soc != 100 )
              {   
                    chg_type = POWER_SUPPLY_CHARGE_TYPE_FAST;                    
              }
              else
              {
		      chg_type = POWER_SUPPLY_CHARGE_TYPE_NONE;
              }

              pr_debug("%s: charge termited, SOC=%d\n", __func__, globle_vm_bms_soc);
		status = BQ_CHG_STATUS_FULL;
	} else {
		chg_type = POWER_SUPPLY_CHARGE_TYPE_NONE;
		status = BQ_CHG_STATUS_NONE;
	}
	pr_debug("bq-chg-status (%d=%s).\n", status, bq24296_chg_status[status]);
	if (chip->chg_status != status) {
		if (status == BQ_CHG_STATUS_NONE
			|| status == BQ_CHG_STATUS_FULL) {
			pr_debug("Charging stopped.\n");

			//wake_unlock(&chip->chg_wake_lock);
		} else {
			pr_debug("Charging started.\n");
			//wake_lock(&chip->chg_wake_lock);
		}
		chip->chg_status = status;
	}

	return chg_type;

exception_handling:
	chip->chg_status = BQ_CHG_STATUS_EXCEPTION;
	//if (wake_lock_active(&chip->chg_wake_lock)) {
	//	pr_debug("exception_handling : unlock chg_wake_lock.\n");
	//	wake_unlock(&chip->chg_wake_lock);
	//}
	return chg_type;

}

static int bq24296_get_prop_batt_capacity(struct bq24296_chip *chip)
{
#if 0
       int batt_soc;
       union power_supply_propval ret = {0,};
       
	NULL_CHECK(chip, -EINVAL);       

       if (chip->bms_psy) {
              chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, &ret);
		batt_soc = ret.intval;
        }
	return batt_soc;
#else
       return globle_vm_bms_soc;
#endif
}

static enum batt_id_type bq24296_get_prop_batt_id(struct bq24296_chip *chip)
{
       int batt_id;
       union power_supply_propval ret = {0,};
       
	NULL_CHECK(chip, -EINVAL);       

       if (chip->bms_psy) {
              chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_BATTERY_ID, &ret);
		batt_id = ret.intval;
       }

       if (batt_id == 1 )
	      return BATT_ID_GY;
       else if (batt_id == 2 )
	      return BATT_ID_LG;
       else if (batt_id == 3 )
	      return BATT_ID_BAK;
       else
	      return BATT_ID_UNKNOWN;
}

static bool bq24296_is_charger_present(struct bq24296_chip *chip)
{
	int ret = 0;
	u8 sys_status, power_good;
	bool power_ok;

	NULL_CHECK(chip, false);
	ret = bq24296_read_reg(chip->client, BQ08_SYSTEM_STATUS_REG, &sys_status);
	if (ret) {
		pr_err("failed to read BQ08_SYSTEM_STATUS_REG ret=%d\n", ret);
		return false;
	}

	power_good = (sys_status & PG_STAT_MASK);
	sys_status &= VBUS_STAT_MASK;

	if ((power_good == 0) && (sys_status == 0 || sys_status == 0xC0)) {
		power_ok = false;
		//pr_err("DC is missing.\n");
	} else {
		power_ok = true;
		//pr_err("DC is present.\n");
	}

	return power_ok;
}

#define ITERM_MIN_MA  128
#define ITERM_MAX_MA  2048
#define ITERM_STEP_MA  128

static int bq24296_set_term_current(struct bq24296_chip *chip, int ma)
{
	u8 reg_val = 0;
	int set_ma = 0;
	NULL_CHECK(chip, -EINVAL);
	if (ma < ITERM_MIN_MA)
		ma = ITERM_MIN_MA;
	if (ma > ITERM_MAX_MA)
		ma = ITERM_MAX_MA;

	reg_val = (ma - ITERM_MIN_MA)/ITERM_STEP_MA;
	set_ma = reg_val * ITERM_STEP_MA + ITERM_MIN_MA;

	pr_info("req_i = %d set_i = %d reg_val = 0x%02x\n",
				ma, set_ma, reg_val);

	return bq24296_masked_write(chip->client, BQ03_PRE_CHARGE_TERM_CUR_REG,
			ITERM_MASK, reg_val);
}

static int bq24296_get_ibat_max(struct bq24296_chip *chip, int *mv)
{
	u8 reg_val = 0;
	int ret;

	NULL_CHECK(chip, -EINVAL);
	ret = bq24296_read_reg(chip->client, BQ02_CHARGE_CUR_CONT_REG, &reg_val);
	if (ret) {
		pr_err("failed to read BQ08_SYSTEM_STATUS_REG ret=%d\n", ret);
		return -EIO;
	}
	reg_val = reg_val >> 2;
	*mv = reg_val * IBAT_STEP_MA + IBAT_MIN_MA;
	return ret;
}

static int bq24296_get_force_ichg_decrease(struct bq24296_chip *chip, int *enable)
{
	int ret;
	u8 val;
	NULL_CHECK(chip, -EINVAL);

	ret = bq24296_read_reg(chip->client, BQ02_CHARGE_CUR_CONT_REG, &val);
	if (ret) {
		pr_err("failed to get FORCE_20PCT ret=%d\n", ret);
		return ret;
	}
	*enable = (val & FORCE_20PCT_MASK) ? 1 : 0;

	return 0;
}

static int bq24296_get_input_i_limit(struct bq24296_chip *chip, int *ma)
{
	u8 reg_val = 0;
	int ret;
	NULL_CHECK(chip, -EINVAL);

	ret = bq24296_read_reg(chip->client, BQ00_INPUT_SRC_CONT_REG, &reg_val);
	if (ret) {
		pr_err("failed to read BQ08_SYSTEM_STATUS_REG ret=%d\n", ret);
		return -EIO;
	}
	*ma = icl_ma_table[reg_val & IINLIM_MASK].icl_ma;
	return ret;
}

#define VIN_LIMIT_MIN_MV  3880
#define VIN_LIMIT_MAX_MV  5080
#define VIN_LIMIT_STEP_MV  80
static int bq24296_get_input_vin_limit(struct bq24296_chip *chip, int *mv)
{
	u8 reg_val = 0;
	int ret;
	NULL_CHECK(chip, -EINVAL);

	ret = bq24296_read_reg(chip->client, BQ00_INPUT_SRC_CONT_REG, &reg_val);
	if (ret) {
		pr_err("failed to read BQ00_SYSTEM_STATUS_REG ret=%d\n", ret);
		return ret;
	}
	*mv = (reg_val & VINDPM_MASK >> 3) * VIN_LIMIT_STEP_MV +
		VIN_LIMIT_MIN_MV;
	return ret;
}

static int bq24296_get_adjust_ibat(struct bq24296_chip *chip, int *mv)
{
	int ret, enable;
	NULL_CHECK(chip, -EINVAL);
	ret = bq24296_get_ibat_max(chip, mv);
	if (ret)
		return ret;
	bq24296_get_force_ichg_decrease(chip, &enable);
	if (enable)
		*mv /= 5;
	return 0;
}

static int bq24296_force_ichg_decrease(struct bq24296_chip *chip, bool enable)
{
	int ret;
	u8 val = (u8)(!!enable);
	NULL_CHECK(chip, -EINVAL);

	pr_debug("enable=%d\n", enable);

	ret = bq24296_masked_write(chip->client, BQ02_CHARGE_CUR_CONT_REG,
			FORCE_20PCT_MASK, val);
	if (ret) {
		pr_err("failed to set FORCE_20PCT ret=%d\n", ret);
		return ret;
	}

	return 0;
}

/* Use bq24296_set_adjust_ibat() instead of bq24296_set_ibat_max() */
#define MIN_CHG_CURRENT (100)
static int bq24296_set_adjust_ibat(struct bq24296_chip *chip, int ma)
{
	bool is_charger;
	NULL_CHECK(chip, -EINVAL);
	is_charger = bq24296_is_charger_present(chip);

	if (!is_charger)
		return 0;

	if (ma < IBAT_MIN_MA) {
		bq24296_set_ibat_max(chip, ma * 5);
		bq24296_force_ichg_decrease(chip, 1);
	} else {
		bq24296_set_ibat_max(chip, ma);
		bq24296_force_ichg_decrease(chip, 0);
	}
	pr_info("charging current limit=%d\n", ma);
	return 0;
}

static int get_batt_therm(struct bq24296_chip *chip);

static void bq24296_monitor_batt_temp(struct work_struct *work)
{
       //int batt_therm;
       
       //struct bq24296_chip *chip = container_of(work, struct bq24296_chip, battemp_work.work);
       //wake_lock(&chip->chg_wake_lock);

       //batt_therm = get_batt_therm(chip);
       //pr_debug("%s : batt_temp = %d\n", __func__, batt_therm);
       
       //bq24296_enable_charging_globle( batt_therm );

       //wake_unlock(&chip->chg_wake_lock);

       //schedule_delayed_work(&chip->battemp_work, MONITOR_BATTEMP_POLLING_PERIOD);
}

static void bq24296_irq_worker(struct work_struct *work)
{
	struct bq24296_chip *chip =
		container_of(work, struct bq24296_chip, irq_work.work);
	u8 reg08, reg09;
	int ret = 0, usb_present = 0;
       bool charger = false;
       int batt_temp;
       bool is_charging = true;

	pr_err("%s: occured\n", __func__);
	NULL_CHECK_VOID(chip);

       if ( usb_inserted_while_booting )
        {
               pr_err("%s : inform usb\n", __func__);
               usb_inserted_while_booting = false;
               power_supply_set_present(chip->usb_psy, chip->usb_present);
		 power_supply_changed(&chip->batt_psy);
        }

	ret = bq24296_read_reg(chip->client, BQ08_SYSTEM_STATUS_REG, &reg08);
	if (ret)
		goto exit;
	ret = bq24296_read_reg(chip->client, BQ09_FAULT_REG, &reg09);
	if (ret)
		goto exit;

	pr_err("bq24296_irq_worker 08:0x%02X, 09:0x%02X\n", reg08, reg09);

	if ((reg08 & VBUS_STAT_MASK) == VBUS_STAT_MASK)
		pr_err("otg detection!\n");
	else if (reg08 & BIT(7))
		pr_err("adapter port detected!\n");
	else if (reg08 & BIT(6))
		pr_err("usb host detected!\n");
	if ((reg08 & CHRG_STAT_MASK) == CHRG_STAT_MASK)
       {   
              globle_charging_done = true;
              if ( !wake_lock_active(&chip->chg_wake_lock)) {
                    wake_lock_timeout(&chip->chg_wake_lock, msecs_to_jiffies(30000));
                    pr_err("lock chg_wake_lock for 30s!\n");
              }

		pr_err("charging done!\n");
       }
	else if (reg08 & FAST_CHARGE_MASK)
       {   
		pr_err("fast charging!\n");
              if ( !wake_lock_active(&chip->chg_wake_lock)) {
                    wake_lock_timeout(&chip->chg_wake_lock, msecs_to_jiffies(5000));
                    pr_err("lock chg_wake_lock for 5s!\n");
              }
       }
	else if (reg08 & PRE_CHARGE_MASK)
       {   
		pr_err("pre-charging!\n");
              if ( !wake_lock_active(&chip->chg_wake_lock)) {
                    wake_lock_timeout(&chip->chg_wake_lock, msecs_to_jiffies(5000));
                    pr_err("lock chg_wake_lock for 5s!\n");
              }
       }
	else
       {
              if ( !wake_lock_active(&chip->chg_wake_lock)) {
                     wake_lock_timeout(&chip->chg_wake_lock, msecs_to_jiffies(5000));
                     pr_err("lock chg_wake_lock for 5s!\n");
              }
              globle_charging_done = false;
              //power_supply_changed(&chip->batt_psy);
              is_charging = false;
		pr_err("not charging!\n");
       }

	if (!bq24296_get_prop_batt_present(chip)) {		

		//wake_lock_timeout(&chip->battgone_wake_lock, HZ*10);
		//msleep(2000); /* makes top-half i2c time margin */
		charger = bq24296_is_charger_present(chip);
		pr_debug("battery removed %d\n", charger);
		if (charger ) {
			cancel_delayed_work(&chip->irq_work);
			bq24296_enable_charging(chip, 0);
			//switch_set_state(&chip->batt_removed, 1);
			power_supply_changed(&chip->batt_psy);
		}
		bq24296_set_vbat_max(chip, 3600);
	}

	globle_usb_present = bq24296_is_charger_present(chip);
       usb_present = globle_usb_present;
       batt_temp = get_batt_therm(chip);

       pr_err("%s usb_present=%d, chip->usb_present=%d\n", __func__, usb_present, chip->usb_present);

	if (chip->usb_present ^ usb_present) {
	       if ( ( is_charging && usb_present) || ( !is_charging && !usb_present ))
              {   
		    chip->usb_present = usb_present;
                  power_supply_set_present(chip->usb_psy, chip->usb_present);
              }

              if ( is_charging && usb_present)
              {
                  pr_err("enable charging!\n");
                  bq24296_set_vbat_max(chip, VBAT_MAX_MV);
                  bq24296_enable_charging_globle( batt_temp );    		    
              }
		power_supply_changed(&chip->batt_psy);
	}

exit:
       enable_irq(chip->irq);
       printk(KERN_ERR"%s exit\n", __func__);
}

static irqreturn_t bq24296_irq(int irq, void *dev_id)
{
	struct bq24296_chip *chip = dev_id;
	NULL_CHECK(chip, IRQ_NONE);

       disable_irq_nosync(chip->irq);
       //pr_err("%s triggered\n", __func__);
	schedule_delayed_work(&chip->irq_work, msecs_to_jiffies(0));

	return IRQ_HANDLED;
}

static char *bq24296_power_supplied_to[] = {
	"battery",
};

static enum power_supply_property bq24296_batt_power_props[] = {
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_BATTERY_ID,
};



static enum power_supply_property bq24296_power_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
       POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_FORCE,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED,
	//POWER_SUPPLY_PROP_CHARGING_COMPLETE,
	//POWER_SUPPLY_PROP_SAFTETY_CHARGER_TIMER,
};

static int bq24296_get_prop_batt_status(struct bq24296_chip *chip)
{
	int chg_type = bq24296_get_prop_charge_type(chip);
	int batt_present = bq24296_get_prop_batt_present(chip);
	int capacity = bq24296_get_prop_batt_capacity(chip);

	if (capacity >= 100 && batt_present
		&& bq24296_is_charger_present(chip))
		return POWER_SUPPLY_STATUS_FULL;

	if (chg_type == POWER_SUPPLY_CHARGE_TYPE_TRICKLE ||
		chg_type == POWER_SUPPLY_CHARGE_TYPE_FAST)
		return POWER_SUPPLY_STATUS_CHARGING;

	if (chip->usb_present)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;

	return POWER_SUPPLY_STATUS_DISCHARGING;
}

static int bq24296_get_prop_batt_health(struct bq24296_chip *chip)
{
       int batt_temp;
       
	NULL_CHECK(chip, -EINVAL);

       batt_temp = get_batt_therm(chip);

	if (batt_temp>= 600)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (batt_temp <= 0)
		return POWER_SUPPLY_HEALTH_COLD;
	else
		return POWER_SUPPLY_HEALTH_GOOD;
}

#define DEFAULT_CURRENT		200000
static int bq24296_get_prop_batt_current_now(struct bq24296_chip *chip)
{
	int batt_current = 0;
	if (bq24296_get_prop_charge_type(chip) >
		POWER_SUPPLY_CHARGE_TYPE_NONE) {
		bq24296_get_adjust_ibat(chip, &batt_current);
	} else {
		batt_current = 0;
	}
    
	return batt_current;
}

#define DEFAULT_FULL_DESIGN	2800
static int bq24296_get_prop_batt_full_design(struct bq24296_chip *chip)
{
	NULL_CHECK(chip, -EINVAL);
	return DEFAULT_FULL_DESIGN;
}

#define DEFAULT_BATT_VOL		4000000
static int get_battery_voltage(struct bq24296_chip *chip)
{
	int batt_vol;
       union power_supply_propval ret = {0,};

       if (chip->bms_psy) {
                chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &ret);
	         batt_vol = ret.intval;
                return batt_vol;
       }
	return DEFAULT_BATT_VOL;
}

static int bq24296_get_battery_id(struct bq24296_chip *chip)
{
	int batt_id = 0;
       union power_supply_propval ret = {0,};

       if (chip->bms_psy) {
                chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_BATTERY_ID, &ret);
	         batt_id = ret.intval;
       }
	return batt_id;
}

#define DEFAULT_BATT_TEMP		250
static int get_batt_therm(struct bq24296_chip *chip)
{
	   int batt_therm;
       union power_supply_propval ret = {0,};

       if(g_boot_ffbm)
       {
            return DEFAULT_BATT_TEMP;
       }
       if (chip->bms_psy) {
              chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_TEMP, &ret);
		batt_therm = ret.intval;
              return batt_therm;
       }
       else
	       return DEFAULT_BATT_TEMP;
}

int get_batt_therm_globle(void)
{
	return get_batt_therm(the_bq24296_chip);
}

static int bq24296_batt_power_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct bq24296_chip *chip = container_of(psy,
					struct bq24296_chip, batt_psy);
	NULL_CHECK(chip, -EINVAL);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bq24296_get_prop_batt_status(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = bq24296_get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = bq24296_get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq24296_get_prop_batt_present(chip);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = 4400 * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = 4400 * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = get_battery_voltage(chip);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = get_batt_therm(chip);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bq24296_get_prop_batt_capacity(chip);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = bq24296_get_prop_batt_current_now(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = bq24296_get_prop_batt_full_design(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = !chip->charging_disabled;
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		/* it makes ibat max set following themral mitigation.
		 * But, SMB349 cannot control ibat current like PMIC.
		 * if LGE charging scenario make charging thermal control,
		 * it is good interface to use LG mitigation level.
		 */
		val->intval = 0;
		break;
       case POWER_SUPPLY_PROP_BATTERY_ID:
		val->intval = bq24296_get_battery_id(chip);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bq24296_batt_power_set_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	struct bq24296_chip *chip = container_of(psy,
					struct bq24296_chip, batt_psy);
	NULL_CHECK(chip, -EINVAL);
	switch (psp) {
       case POWER_SUPPLY_PROP_STATUS:
              printk("%s: set status %d. do nothing\n", __func__, val->intval);
              break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
              chip->charging_disabled = !!!(val->intval);
              printk("%s: CHARGING_ENABLED = %d\n", __func__, val->intval);
		//0: OE low, enable charging;  1: OE high, disable charging
              gpio_direction_output(chip->chg_en_gpio, chip->charging_disabled);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		chip->batt_present = !!(val->intval);
		printk("\nBattery %s!!!\n\n",
			val->intval ? "inserted" : "removed");
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		/* it makes ibat max set following themral mitigation.
		 * But, SMB349 cannot control ibat current like PMIC.
		 * if LGE charging scenario make charging thermal control,
		 * it is good interface to use LG mitigation level.
		 */
		break;

	default:
		return -EINVAL;
	}

	power_supply_changed(&chip->batt_psy);

	return 0;
}


static int
bq24296_batt_power_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		return 1;
	default:
		break;
	}

	return 0;
}
#if 0
static void bq24296_batt_external_power_changed(struct power_supply *psy)
{
	struct bq24296_chip *chip = container_of(psy,
					struct bq24296_chip, batt_psy);
	union power_supply_propval ret = {0,};

	int	busy_wait = 20;
	NULL_CHECK_VOID(chip);

	pr_debug("\n");
	/*
	     Kernel crash patch due to i2c failure
	     i2c oeration is established by someone before
		 resuming i2c client.
		     <workQ pending -> kernel resume -> WorkQ resumt
		     -> invoke power supply changed
	*/
	if(chip->suspend) {
		/* In suspend, busy waiting... till resume */
		do{
			if(!chip->suspend || busy_wait == 0)
				break;
			--busy_wait;
			/* max 400ms */
			msleep(20);
			pr_info("\n\n busy waiting.... till resume \n\n");
		} while(true);
	}

	bq24296_charger_psy_getprop(chip, usb_psy, CURRENT_MAX, &ret);

	ret.intval = ret.intval / 1000; /* dwc3 treats uA */
	pr_info("dwc3 result=%dmA\n", ret.intval);

	bq24296_charger_psy_setprop(chip, psy_this, CURRENT_MAX, chip->chg_current_ma);

	chip->usb_psy = _psy_check_ext(chip->usb_psy, _USB_);
	NULL_CHECK_VOID(chip->usb_psy);

	/*
	  * FLOATED_CHARGER is under control of USB
	  * below function is only for	TA (D+/D- short) -> online  is 1
	  * Floated charger -> online is ZERO
	*/
	bq24296_charger_psy_getprop(chip, usb_psy, ONLINE, &ret);
	if (_psy_check_ext(chip->psy_this, _THIS_) > 0) {
		if (ret.intval) {
			if (likely(delayed_work_pending(&chip->usbin_mon))) {
				cancel_delayed_work_sync(&chip->usbin_mon);
			}
			schedule_delayed_work(&chip->usbin_mon, msecs_to_jiffies(10));
		}
	}
	bq24296_decide_otg_mode(chip);

       bq24296_set_input_vin_limit(chip, chip->vin_limit_mv);

	power_supply_changed(&chip->batt_psy);
}
#endif

static int bq24296_power_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct bq24296_chip *chip = container_of(psy,
						struct bq24296_chip, ac_psy);
	NULL_CHECK(chip, -EINVAL);
	switch (psp) {

	case POWER_SUPPLY_PROP_CURRENT_MAX:
		bq24296_get_adjust_ibat(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
              if (chip->input_current_max == -1)
                    val->intval = -1;
              else
                    bq24296_get_input_i_limit(chip, &val->intval);
		break;
       case POWER_SUPPLY_PROP_INPUT_CURRENT_FORCE:
              if (chip->input_current_force == -1)
                    val->intval = -1;
              else
                    bq24296_get_input_i_limit(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
       case POWER_SUPPLY_PROP_ONLINE:
              val->intval = 0;
/*		if (chip->charging_disabled)
			return 0;
		val->intval = bq24296_is_charger_present(chip);
*/
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = bq24296_get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = bq24296_get_prop_charge_type(chip)
				!= POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED:
		bq24296_get_input_vin_limit(chip, &val->intval);
		break;
/*	case POWER_SUPPLY_PROP_SAFTETY_CHARGER_TIMER:
	{
		int ret;
		u8 value = 0;
		ret = bq24296_read_reg(chip->client, BQ05_CHARGE_TERM_TIMER_CONT_REG, &value);
		if (ret) {
			pr_err("failed to read BQ05_CHARGE_TERM_TIMER_CONT_REG ret=%d\n", ret);
			return -EINVAL;
		}
		val->intval = (value >> 3) & 0x01;
		pr_info("get charger_timeout : %d[D]\n", val->intval);
	}
		break;
	case POWER_SUPPLY_PROP_CHARGING_COMPLETE:
		if (bq24296_get_prop_batt_capacity(chip) == 100)
			val->intval = 0;
		else
			val->intval = 1;

		break;
		*/
	default:
		return -EINVAL;
	}
	return 0;
}

static int bq24296_power_set_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	struct bq24296_chip *chip = container_of(psy,
						struct bq24296_chip,
						ac_psy);
	NULL_CHECK(chip, -EINVAL);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		bq24296_enable_charging(chip, val->intval);
		chip->ac_online = val->intval;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		bq24296_set_adjust_ibat(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
              chip->input_current_max = val->intval;
              
              if ( val->intval == -1 )
              {
                    pr_err("set input current max=-1,  set current according usb type\n");
                    bq24296_set_input_current(input_current_mA_globle);
              }
              else
                    bq24296_set_input_current(val->intval);
		break;
       case POWER_SUPPLY_PROP_INPUT_CURRENT_FORCE:
              chip->input_current_force = val->intval;
              
              if ( val->intval == -1 )
              {
                    pr_err("set input current force=-1,  set current according usb type\n");
                    bq24296_set_input_current(input_current_mA_globle);
              }
              else
                    bq24296_set_input_i_limit(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED:
		bq24296_force_ichg_decrease(chip, val->intval);
		break;
	//case POWER_SUPPLY_PROP_SAFTETY_CHARGER_TIMER:
	//	break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int
bq24296_power_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	//case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
       case POWER_SUPPLY_PROP_INPUT_CURRENT_FORCE:
	//case POWER_SUPPLY_PROP_SAFTETY_CHARGER_TIMER:
		return 1;
	default:
		break;
	}

	return 0;
}

static int  bq24296_init_batt_psy(struct bq24296_chip *chip)
{
	int ret;
	NULL_CHECK(chip, -EINVAL);

	chip->batt_psy.name = "battery";
	chip->batt_psy.type = POWER_SUPPLY_TYPE_BATTERY;
	chip->batt_psy.properties = bq24296_batt_power_props;
	chip->batt_psy.num_properties =
					ARRAY_SIZE(bq24296_batt_power_props);
	chip->batt_psy.get_property = bq24296_batt_power_get_property;
	chip->batt_psy.set_property = bq24296_batt_power_set_property;
	chip->batt_psy.property_is_writeable =
					bq24296_batt_power_property_is_writeable;
	//chip->batt_psy.external_power_changed = bq24296_batt_external_power_changed;

	ret = power_supply_register(&chip->client->dev,
				&chip->batt_psy);
	if (ret) {
		pr_err("failed to register power_supply. ret=%d.\n", ret);
		return ret;
	}

	return 0;
}

static int bq24296_init_ac_psy(struct bq24296_chip *chip)
{
	int ret = 0;
	NULL_CHECK(chip, -EINVAL);

	chip->ac_psy.name = "ac";
	chip->ac_psy.type = POWER_SUPPLY_TYPE_MAINS;
	chip->ac_psy.supplied_to = bq24296_power_supplied_to;
	chip->ac_psy.num_supplicants = ARRAY_SIZE(bq24296_power_supplied_to);
	chip->ac_psy.properties = bq24296_power_props;
	chip->ac_psy.num_properties = ARRAY_SIZE(bq24296_power_props);
	chip->ac_psy.get_property = bq24296_power_get_property;
	chip->ac_psy.set_property = bq24296_power_set_property;
	//chip->ac_psy.get_event_property = bq24296_power_get_event_property;
	//chip->ac_psy.set_event_property = bq24296_power_set_event_property;
	chip->ac_psy.property_is_writeable =
				bq24296_power_property_is_writeable;
	ret = power_supply_register(&chip->client->dev,
				&chip->ac_psy);
	if (ret) {
		pr_err("failed to register power_supply. ret=%d.\n", ret);
		return ret;
	}
	chip->psy_this = &chip->ac_psy;

	return 0;
}

static int bq24296_hw_init(struct bq24296_chip *chip)
{
	int ret = 0;
       u8 temp;
	u8 enable = 0; 

	NULL_CHECK(chip, -EINVAL);

       pr_err("bq24296 bq24296_hw_init : start\n");

       bq24296_masked_write(chip->client, BQ00_INPUT_SRC_CONT_REG, EN_HIZ, enable ? EN_HIZ : 0);

       ret = bq24296_read_reg(chip->client, BQ0A_VENDOR_PART_REV_STATUS_REG, &temp);
       if (ret) {
		pr_err("bq24296_read_reg failed: reg=%03X, rc=%d\n",
				BQ0A_VENDOR_PART_REV_STATUS_REG, ret);
		return ret;
	}
       printk(KERN_ERR"bq24296_hw_init  read reg [%03x] = %03x\n",
                            BQ0A_VENDOR_PART_REV_STATUS_REG, temp);

       ret = bq24296_read_reg(chip->client, BQ08_SYSTEM_STATUS_REG, &temp);
       if (ret) {
		pr_err("bq24296_read_reg failed: reg=%03X, rc=%d\n",
				BQ08_SYSTEM_STATUS_REG, ret);
		return ret;
	}

       if ( temp & (BIT(6) | BIT(7) ))
       {
              printk(KERN_ERR"bq24296_hw_init  usb is inserted while booting\n");
              usb_inserted_while_booting = true;
              schedule_delayed_work(&chip->irq_work, msecs_to_jiffies(500));
       }
       printk(KERN_ERR"bq24296_hw_init  read reg [%03x] = %03x\n",
                            BQ08_SYSTEM_STATUS_REG, temp);
        
        bq24296_set_ibat_max(chip, IBAT_NORMAL);
        bq24296_set_input_i_limit(chip, INPUT_CURRENT_LIMIT_USB20);
        printk(KERN_ERR"%s: set input current 500\n", __func__);
        
        bq24296_set_input_v_limit(chip, INPUT_VOLTAGE_LIMIT_MAX);

        batt_id = bq24296_get_prop_batt_id(chip);
        pr_err("batt_id = %d\n", batt_id);

        bq24296_set_vbat_max(chip, VBAT_MAX_MV);
        
        bq24296_enable_watchdog(chip, 0);
        bq24296_enable_longpress_reset_batfet(chip, 0);
        
#ifdef DISABLE_CHG_TIMER_FOR_MTBF
        bq24296_enable_chg_timer(chip, 0);
#endif
        bq24296_set_term_current(chip, ITERM_MIN_MA);

        ret = bq24296_set_prechg_i_limit(chip, chip->pre_chg_current_ma);
	if (ret) {
		pr_err("failed to set pre-charge current\n");
		return ret;
	}

    
#if 0
	ret = bq24296_set_input_vin_limit(chip, chip->vin_limit_mv);
	if (ret) {
		pr_err("failed to set input voltage limit\n");
		return ret;
	}

	ret = bq24296_set_system_vmin(chip, chip->sys_vmin_mv);
	if (ret) {
		pr_err("failed to set system min voltage\n");
		return ret;
	}

	ret = bq24296_force_ichg_decrease(chip, 0);
	if (ret) {
		pr_err("failed to set charging current as reg[ICHG] programmed\n");
		return ret;
	}
#endif
	return 0;
}

static ssize_t at_chg_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int r;
	bool b_chg_ok = false;
	int chg_type;
	struct bq24296_chip *chip = dev_get_drvdata(dev);
	NULL_CHECK(chip, -EINVAL);

	chg_type = bq24296_get_prop_charge_type(chip);
	if (chg_type != POWER_SUPPLY_CHARGE_TYPE_NONE) {
		b_chg_ok = true;
		r = snprintf(buf, 3, "%d\n", b_chg_ok);
		pr_err("[Diag] true ! buf = %s, charging=1\n", buf);
	} else {
		b_chg_ok = false;
		r = snprintf(buf, 3, "%d\n", b_chg_ok);
		pr_err("[Diag] false ! buf = %s, charging=0\n", buf);
	}

	return r;
}

static ssize_t at_chg_status_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret = 0;
	struct bq24296_chip *chip = dev_get_drvdata(dev);

	NULL_CHECK(chip, -EINVAL);

	NULL_CHECK(count, -EINVAL);

	if (strncmp(buf, "0", 1) == 0) {
		/* stop charging */
		pr_err("[Diag] stop charging start\n");
		ret = bq24296_enable_charging(chip, false);

	} else if (strncmp(buf, "1", 1) == 0) {
		/* start charging */
		pr_err("[Diag] start charging start\n");
		ret = bq24296_enable_charging(chip, true);
	}

	if (ret)
		return -EINVAL;

	return 1;
}

static ssize_t at_chg_complete_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int guage_level = 0;
	int r = 0;

	struct bq24296_chip *chip = dev_get_drvdata(dev);

	NULL_CHECK(chip, -EINVAL);

	guage_level = bq24296_get_prop_batt_capacity(chip);

	if (guage_level == 100) {
		r = snprintf(buf, 3, "%d\n", 0);
		pr_err("[Diag] buf = %s, gauge==100\n", buf);
	} else {
		r = snprintf(buf, 3, "%d\n", 1);
		pr_err("[Diag] buf = %s, gauge<=100\n", buf);
	}

	return r;
}

static ssize_t at_chg_complete_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret = 0;

	struct bq24296_chip *chip = dev_get_drvdata(dev);

	NULL_CHECK(chip, -EINVAL);

	NULL_CHECK(count, -EINVAL);

	if (strncmp(buf, "0", 1) == 0) {
		/* charging not complete */
		pr_err("[Diag] charging not complete start \n");
		ret = bq24296_enable_charging(chip, true);
	} else if (strncmp(buf, "1", 1) == 0) {
		/* charging complete */
		pr_err("[Diag] charging complete start\n");
		ret = bq24296_enable_charging(chip, false);
	}

	if (ret)
		return -EINVAL;

	return 1;
}

static ssize_t at_otg_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int otg_mode;
	int r = 0;

	struct bq24296_chip *chip = dev_get_drvdata(dev);

	NULL_CHECK(chip, -EINVAL);

	otg_mode = bq24296_is_otg_mode(chip);
	if (otg_mode) {
		otg_mode = 1;
		r = snprintf(buf, 3, "%d\n", otg_mode);
		pr_err("[Diag] true ! buf = %s, OTG Enabled\n", buf);
	} else {
		otg_mode = 0;
		r = snprintf(buf, 3, "%d\n", otg_mode);
		pr_err("[Diag] false ! buf = %s, OTG Disabled\n", buf);
	}
	return r;
}

static ssize_t at_otg_status_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret = 0;
	struct bq24296_chip *chip = dev_get_drvdata(dev);

	NULL_CHECK(chip, -EINVAL);

	NULL_CHECK(count, -EINVAL);

	if (strncmp(buf, "0", 1) == 0) {
		pr_err("[Diag] OTG Disable start\n");
		if (bq24296_is_otg_mode(chip))
			ret = bq24296_enable_otg(chip, false);

	} else if (strncmp(buf, "1", 1) == 0) {
		pr_err("[Diag] OTG Enable start\n");
		if (!bq24296_is_otg_mode(chip))
			ret = bq24296_enable_otg(chip, true);
	}

	if (ret)
		return -EINVAL;
	return 1;
}
DEVICE_ATTR(at_charge, 0644, at_chg_status_show, at_chg_status_store);
DEVICE_ATTR(at_chcomp, 0644, at_chg_complete_show, at_chg_complete_store);
DEVICE_ATTR(at_otg, 0644, at_otg_status_show, at_otg_status_store);


#define bq24296_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0644,			\
	},					\
	.show	= _name##_show,					\
	.store	= _name##_store,			\
}

static ssize_t  bq24296_state_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{

	//sprintf(buf, "%s\n", "jiangzf");
       return 0;

}

static ssize_t  bq24296_state_store(struct kobject *kobj,
			struct kobj_attribute *attr, const char *buf, size_t count)
{
       u8 read_val;
       int ret;

       unsigned long  addr_rx=0;
       unsigned long  data_tx = 0;

       char messages[32] = {0};

       struct i2c_client *client = the_bq24296_chip->client;

       if (count > 31)
            count = 31;
       
       memcpy(messages, buf, count);

       printk(KERN_ERR"message=%s\n", messages);

       if (strncmp(buf, "messages", 2) == 0)
       {
           ret = kstrtoul(&messages[2], 16, &data_tx);
           printk(KERN_ERR"addr&data=0x%x\n", (int)data_tx);
      
           bq24296_write_reg(client, (data_tx>>8)&0xffff , (u8)(data_tx&0xff));
           return 0;
       }     
       else if (strncmp(messages, "rd", 2) == 0)
       {
           ret = kstrtoul(&messages[2], 16, &addr_rx);
      
           printk(KERN_ERR"addr=0x%x\n", (u32)addr_rx);
      
            bq24296_read_reg(client, (u32)addr_rx, &read_val);
            printk(KERN_ERR"reg[0x%x]=0x%x\n", (u32)addr_rx, read_val);
            return 0;
       }

        return -1;
}

bq24296_attr(bq24296_state);

static struct attribute *bq24296_state_att[] = {
	&bq24296_state_attr.attr,
	NULL
};

static struct attribute_group bq24296_state_gr = {
	//.name = "hotplug",
	.attrs = bq24296_state_att
};

struct kobject *bq24296_state_kobj;

#define DELAY_CHECK_BATT_TEMP msecs_to_jiffies(4000)

static int bq24296_probe(struct i2c_client *client,		
		  const struct i2c_device_id *id)
{
	
        struct bq24296_chip *chip;
        int ret = 0;

        struct device_node *dev_node = client->dev.of_node;
        
        pr_debug("bq24296 charger probe : start\n");
          
        printk(KERN_ERR"bq24296_probe\n");
        	
        if (!i2c_check_functionality(client->adapter,I2C_FUNC_SMBUS_BYTE_DATA)) {        		
                   pr_err("i2c func fail.\n");		
                   return -EIO;
        }

        chip = kzalloc(sizeof(struct bq24296_chip), GFP_KERNEL);
        if (!chip) {        		
                    pr_err("failed to alloc memory\n");        		
                    return -ENOMEM;
         }

        chip->client = client;
        chip->batt_present = true;
        	
        chip->usb_psy = power_supply_get_by_name("usb");
        if (!chip->usb_psy) {        		
            pr_err("usb supply not found deferring probe\n");        		
            ret = -EPROBE_DEFER;        		
            goto probe_defer_error;
        }

        chip->bms_psy = power_supply_get_by_name("bms");
        if (!chip->bms_psy) {
            pr_err("bms supply not found deferring probe\n");        		
            ret = -EPROBE_DEFER;
            goto probe_defer_error;
        }

        chip->ext_chg_en = 1;
        chip->otg_en = 120;
        chip->chg_current_ma = 500; 
        chip->term_current_ma = 100;
        chip->vbat_max_mv = 4350000;
        chip->pre_chg_current_ma = 512;
        chip->sys_vmin_mv = 3300000;
        chip->vin_limit_mv = 4200000;
        chip->icl_vbus_mv = 3000000;
        chip->set_chg_current_ma = chip->chg_current_ma;
        chip->charging_disabled = 0;
        chip->input_current_force = -1;
        chip->input_current_max = -1;

        i2c_set_clientdata(client, chip);
        mutex_init(&chip->usbin_lock);
        
        bq24296_init_batt_psy(chip);
        bq24296_init_ac_psy(chip);

        INIT_DELAYED_WORK(&chip->irq_work, bq24296_irq_worker);
        INIT_DELAYED_WORK(&chip->battemp_work, bq24296_monitor_batt_temp);
        
        the_bq24296_chip = chip;
        wake_lock_init(&chip->chg_wake_lock, WAKE_LOCK_SUSPEND, BQ24296_NAME);
	 wake_lock_init(&chip->uevent_wake_lock, WAKE_LOCK_SUSPEND, "bq24296_chg_uevent");
	 //wake_lock_init(&chip->battgone_wake_lock, WAKE_LOCK_SUSPEND, "batt removed");
	 wake_lock_init(&chip->chg_timeout_lock,  WAKE_LOCK_SUSPEND, "chg timeout");
	 wake_lock_init(&chip->icl_wake_lock, WAKE_LOCK_SUSPEND, "icl_wake_lock");

        if ( dev_node )
        {
              if (of_find_property(dev_node, "qcom,bq24296-irq-gpio", NULL))
                   chip->int_gpio = of_get_named_gpio_flags(dev_node, "qcom,bq24296-irq-gpio", 0, NULL);
              if (!gpio_is_valid(chip->int_gpio))
              {
                   pr_err("gpio bq24296-irq is not valid\n");
                   goto gpio_request_error;
              }
              pr_debug("int_gpio = %d\n", chip->int_gpio);

              if ( chip->int_gpio > 0 )
                     ret = gpio_request(chip->int_gpio, "bq24296_int");
        	 if (ret<0) {
        		pr_err("failed to request int_gpio\n");
        		goto gpio_request_error;
        	 }
               gpio_direction_input(chip->int_gpio);
        	 chip->irq = gpio_to_irq(chip->int_gpio);
        	 pr_debug("int_gpio irq#=%d.\n", chip->irq);

               if (chip->irq) {
        		ret = request_irq(chip->irq, bq24296_irq, IRQF_TRIGGER_FALLING,	"bq24296_irq", chip);
        		if (ret) {
        			pr_err("request_irq %d failed\n", chip->irq);
                            gpio_free(chip->int_gpio);
        			goto gpio_request_error;
        		}

                     enable_irq_wake(chip->irq);
        	}

              if (of_find_property(dev_node, "qcom,bq24296-chg-en-gpio", NULL))
                   chip->chg_en_gpio= of_get_named_gpio_flags(dev_node, "qcom,bq24296-chg-en-gpio", 0, NULL);
              if (!gpio_is_valid(chip->chg_en_gpio))
              {
                   pr_err("gpio chg-en is not valid\n");
              }
              
              pr_err("chg_en_gpio = %d\n", chip->chg_en_gpio);

              if ( chip->chg_en_gpio > 0 )
                     ret = gpio_request(chip->chg_en_gpio, "bq24296_chg_en");
        	 if (ret<0) {
        		pr_err("failed to request int_gpio\n");
        	 }
               gpio_direction_output(chip->chg_en_gpio, 0);//0: OE low, enable charging
        }

 /*        
        ret = bq24296_create_debugfs_entries(chip);
        if (ret) {        		
            pr_err("bq24296_create_debugfs_entries failed ret=%d\n", ret);        		
            goto err_debugfs;
        }
*/
        bq24296_state_kobj = kobject_create_and_add("bq24296", NULL);
	
	if (!bq24296_state_kobj)
	{
		printk("bq24296_state_kobj: NULL!\n");
	}
	
	ret = sysfs_create_group(bq24296_state_kobj, &bq24296_state_gr);
	
	if (ret) {
		printk("bq24296_state: device create file failed!!\n");		
	}

       bq24296_create_debugfs_entries(chip);

        bq24296_reginfo(chip);

       ret = device_create_file(&client->dev, &dev_attr_at_charge);
	if (ret < 0) {
		pr_err("%s:File dev_attr_at_charge creation failed: %d\n",
				__func__, ret);
		ret = -ENODEV;
	}

	ret = device_create_file(&client->dev, &dev_attr_at_chcomp);
	if (ret < 0) {
		pr_err("%s:File dev_attr_at_chcomp creation failed: %d\n",
				__func__, ret);
		ret = -ENODEV;
	}

	ret = device_create_file(&client->dev, &dev_attr_at_otg);
	if (ret < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
	}

       globle_usb_present = bq24296_is_charger_present(chip);
	chip->usb_present = globle_usb_present;

	power_supply_set_present(chip->usb_psy, chip->usb_present);
    
        ret = bq24296_hw_init(chip); 
        if (ret) {        		
            pr_err("bq24296_hwinit failed.ret=%d\n", ret);        		
            goto error;
        }	

        schedule_delayed_work(&chip->battemp_work, DELAY_CHECK_BATT_TEMP);
                  
        pr_err("%s success\n", __func__);
        //enable_irq(chip->irq);
        return 0;

gpio_request_error:
       mutex_destroy(&chip->usbin_lock);
error:
       wake_lock_destroy(&chip->chg_wake_lock);	
       wake_lock_destroy(&chip->uevent_wake_lock);
       wake_lock_destroy(&chip->chg_timeout_lock);
       wake_lock_destroy(&chip->icl_wake_lock);
probe_defer_error:
       kfree(chip);
	chip = NULL;
	pr_err("fail to probe\n");
	the_bq24296_chip = NULL;
	return ret;
}


static int bq24296_remove(struct i2c_client *client)
{
	struct bq24296_chip *chip = i2c_get_clientdata(client);

       printk(KERN_ERR"bq24296_remove\n");
	mutex_destroy(&chip->usbin_lock);
       kfree(chip);
	chip = NULL;
	the_bq24296_chip = NULL;
	return 0;
}

static const struct i2c_device_id bq24296_id[] = {
	{BQ24296_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, bq24296_id);

static const struct of_device_id bq24296_match[] = {
	{ .compatible = "ti,bq24296-charger", },
	{ },
};


#ifdef CONFIG_PM
static int bq24296_resume(struct i2c_client *client)
{
	//struct bq24296_chip *chip = i2c_get_clientdata(client);
	printk(KERN_ERR"bq24296_resume\n");
	return 0;
}

static int bq24296_suspend(struct i2c_client *client, pm_message_t mesg)
{
	//struct bq24296_chip *chip = i2c_get_clientdata(client);
	printk(KERN_ERR"bq24296_suspend\n");
	return 0;
}
#endif

static struct i2c_driver bq24296_driver = {
	.driver	= {
                                 .name	= BQ24296_NAME,		
	                          .owner	= THIS_MODULE,		
	                          .of_match_table = of_match_ptr(bq24296_match),
	},
	.probe		= bq24296_probe,
	.remove		= bq24296_remove,
	.id_table	= bq24296_id,
#ifdef CONFIG_PM
	.resume		= bq24296_resume,
	.suspend	= bq24296_suspend,
#endif
};


static int __init bq24296_init(void)
{
  
       printk(KERN_ERR"bq24296_init\n");
	return i2c_add_driver(&bq24296_driver);
}
module_init(bq24296_init);

static void __exit bq24296_exit(void)
{
	return i2c_del_driver(&bq24296_driver);
}
module_exit(bq24296_exit);

MODULE_DESCRIPTION("Driver for BQ24296 charger chip");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:" BQ24296_NAME);

