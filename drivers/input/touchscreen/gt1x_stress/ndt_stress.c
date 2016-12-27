/*
 * Driver for Pixcir I2C touchscreen controllers.
 *
 * Copyright (C) 2010-2011 Pixcir, Inc.
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
 * You should have received a copy of the GNU General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/input.h>
#include <linux/input/pixcir_ts.h>
#include <linux/proc_fs.h>
#include <linux/completion.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

//---------------------------------------------------------------------
#define I2C_DRIVER_NAME	 	"ndt_stress_i2c"
#define NDT_DRIVER_VERSION	"zte version v0.1"
#define NDT_FW_PATH			"/etc/firmware/pressure_dev_30.nfw"
#define NDT_CHAR_MAJOR        	103 
#define NDT_DEVICE_NAME        	"ndt_i2c_node"

#define NDT_IOC_MAGIC			'x'
#define NDT_IOC_HW_RESET   	_IO(NDT_IOC_MAGIC, 1)
#define NDT_IOC_READ 			_IOR(NDT_IOC_MAGIC, 2, int)
#define NDT_IOC_WRITE 			_IOW(NDT_IOC_MAGIC, 3, int)
//---------------------------------------------------------------------
#define CONFIG_NDT_USE_DEBUG
#define CONFIG_NDT_USE_PV_TEST
#define CONFIG_NDT_USE_TEST_I2C
#define NDT_PRESS_USE_PM
#define CONFIG_NDT_USE_UPDATE
#define CONFIG_NDT_USE_CHAR_DEV
//---------------------------------------------------------------------
#define NDT_DEBUG_ON 				1
#define NDT_DEBUG_FUNC_ON 		0
#define IIC_EEPROM 					0
//---------------------------------------------------------------------
// ZTE_modify add custom gain control begin
#define FORCE_MAX_VALUE 512
// ZTE_modify end

#define NDT_INFO(fmt,arg...)           printk("<<NDT-INF>>[%s:%d] "fmt"\n", __func__, __LINE__, ##arg)
#define NDT_ERROR(fmt,arg...)          printk("<<NDT-ERR>>[%s:%d] "fmt"\n", __func__, __LINE__, ##arg)
#define NDT_DEBUG(fmt,arg...)          do{\
                                         if(NDT_DEBUG_ON)\
                                         printk("<<NDT-DBG>>[%s:%d]"fmt"\n",__func__, __LINE__, ##arg);\
                                       }while(0)
                                       
#define NDT_DEBUG_FUNC()               do{\
                                         if(NDT_DEBUG_FUNC_ON)\
                                         printk("<<NDT-FUNC>> Func:%s@Line:%d\n",__func__,__LINE__);\
                                       }while(0)
//---------------------------------------------------------------------
static const struct i2c_device_id ndt_stress_id[] = {
	{I2C_DRIVER_NAME, 0},
	{}
};

static struct of_device_id ndt_match_table[] = {
	{.compatible = "ndt,stress-i2c",},
	{ },
};

typedef enum {
	DEBUG_MODE_WEIGHT	= -1,
	DEBUG_MODE_NORMAL 	= 0,
	DEBUG_MODE_DDATA 	= 1,
	DEBUG_MODE_LOG 		= 2,	
	
} DEBUG_TYPE_T;

struct ndt_stress_board_data{
	const char *pwr_reg_name;
	const char *bus_reg_name;
	unsigned int power_delay_ms;
	unsigned int reset_delay_ms;
	unsigned int reset_active_ms;
	int irq_gpio;
	int irq_on_state;
	int irq_flags;
	int power_gpio;
	int power_on_state;
	int reset_gpio;
	int reset_on_state;
	int max_y_for_2d;
};

struct ndt_stress_data {
	struct i2c_client *client;
	struct ndt_stress_board_data bdata;
	struct delayed_work int_work;
	struct delayed_work debug_work;
	struct delayed_work update_work;
	struct delayed_work watchdog_work;
	struct delayed_work send_coord_work;
	struct workqueue_struct *ndt_queue;
	struct workqueue_struct *ndt_debug_queue;
	struct workqueue_struct *ndt_coord_handle;
	struct class *i2c_class;
	struct mutex stress_value_mutex;
	struct mutex i2c_mutex;
	//struct completion coord_thread_sync;
	char update_file[128];
	unsigned short value1;
	unsigned short value2;
	unsigned short FirmwareVer;
	short sensor[4];
	short dvalue1;
	short dvalue2;
	unsigned short x1;
	unsigned short y1;
	unsigned char status;
	char dmode;
	bool appear;
	bool suspend;
	bool flashprog;
	bool devbusy;
	bool force_update;
	bool power_stat;
	bool alive_flag;
	// ZTE_modify add custom gain control begin
	unsigned int custom_gain_value;
	// ZTE_end
};

struct ndt_stress_data *gNdtData = NULL;

void ndt_i2c_comm_lock(void);
void ndt_i2c_comm_unlock(void);
extern int get_batt_therm_globle(void);

inline void print_mem(char *buffer, unsigned int len)
{
	unsigned int i = 0;

	printk("\n<<NDT-INF>>");

	for(i = 0; i < len; i++)
		{
		printk("0x%02X ", buffer[i]);

		if((i !=0) && ((i+1) % 8 == 0))	 printk("	");
		
		if((i !=0) && ((i+1) % 16 == 0)) printk("\n<<NDT-INF>>");
		}

	printk("\n");
}

static inline int ndt_Bon(void *p)
{
	if(p == NULL)
		return 1;

	return 0;
}

static int ndt_stress_gpio_setup(int gpio, bool config, int dir, int state)
{
	int retval = 0;
	unsigned char buf[16];

	NDT_DEBUG_FUNC();

	if (config) {
		snprintf(buf, PAGE_SIZE, "ndt_gpio_%u\n", gpio);

		retval = gpio_request(gpio, buf);
		if (retval) {
			pr_err("%s: Failed to get gpio %d (code: %d)",
					__func__, gpio, retval);
			return retval;
		}

		if (dir == 0)
			retval = gpio_direction_input(gpio);
		else
			retval = gpio_direction_output(gpio, state);
		if (retval) {
			pr_err("%s: Failed to set gpio %d direction",
					__func__, gpio);
			return retval;
		}
	} else {
		gpio_free(gpio);
	}

	NDT_DEBUG_FUNC();

	return retval;
}

static int parse_dt(struct device *dev, struct ndt_stress_board_data *bdata)
{
	int retval;
	u32 value;
	const char *name;
	struct device_node *np = dev->of_node;

	NDT_DEBUG_FUNC();

	if (of_find_property(np, "ndt,irq-gpio", NULL)) {
		bdata->irq_gpio = of_get_named_gpio_flags(np,
				"ndt,irq-gpio", 0, NULL);
	}
	else
		bdata->irq_gpio = -1;

	NDT_INFO("irq_gpio: %d", bdata->irq_gpio);

	retval = of_property_read_u32(np, "ndt,irq-on-state",
			&value);
	if (retval < 0)
		bdata->irq_on_state = 0;
	else
		bdata->irq_on_state = value;

	NDT_INFO("irq_on_state: %d", bdata->irq_on_state);

	retval = of_property_read_u32(np, "ndt,irq-flags", &value);
	if (retval < 0)
		return retval;
	else
		bdata->irq_flags = value;

	NDT_INFO("irq_flags: %d", bdata->irq_flags);

	retval = of_property_read_string(np, "ndt,pwr-reg-name", &name);
	if (retval == -EINVAL)
		bdata->pwr_reg_name = NULL;
	else if (retval < 0)
		return retval;
	else
		bdata->pwr_reg_name = name;

	NDT_INFO("pwr_reg_name: %s", bdata->pwr_reg_name);

	retval = of_property_read_string(np, "ndt,bus-reg-name", &name);
	if (retval == -EINVAL)
		bdata->bus_reg_name = NULL;
	else if (retval < 0)
		return retval;
	else
		bdata->bus_reg_name = name;

	NDT_INFO("bus_reg_name: %s", bdata->bus_reg_name);

	if (of_find_property(np, "ndt,power-gpio", NULL)) {
		bdata->power_gpio = of_get_named_gpio_flags(np,
				"ndt,power-gpio", 0, NULL);
		retval = of_property_read_u32(np, "ndt,power-on-state",
				&value);
		if (retval < 0)
			return retval;
		else
			bdata->power_on_state = value;
	} else {
		bdata->power_gpio = -1;
	}

	NDT_INFO("power_gpio: %d, power_on_state %d", bdata->power_gpio, bdata->power_on_state);

	if (of_find_property(np, "ndt,power-delay-ms", NULL)) {
		retval = of_property_read_u32(np, "ndt,power-delay-ms",
				&value);
		if (retval < 0)
			return retval;
		else
			bdata->power_delay_ms = value;
	} else {
		bdata->power_delay_ms = 0;
	}

	NDT_INFO("power_delay_ms: %d", bdata->power_delay_ms);

	if (of_find_property(np, "ndt,reset-gpio", NULL)) {
		bdata->reset_gpio = of_get_named_gpio_flags(np,
				"ndt,reset-gpio", 0, NULL);
		retval = of_property_read_u32(np, "ndt,reset-on-state",
				&value);
		if (retval < 0)
			return retval;
		else
			bdata->reset_on_state = value;
		retval = of_property_read_u32(np, "ndt,reset-active-ms",
				&value);
		if (retval < 0)
			return retval;
		else
			bdata->reset_active_ms = value;
	} else {
		bdata->reset_gpio = -1;
	}

	if (of_find_property(np, "ndt,reset-delay-ms", NULL)) {
		retval = of_property_read_u32(np, "ndt,reset-delay-ms",
				&value);
		if (retval < 0)
			return retval;
		else
			bdata->reset_delay_ms = value;
	} else {
		bdata->reset_delay_ms = 0;
	}

	NDT_INFO("reset_gpio: %d, reset_on_state: %d, reset_active_ms %d, reset_delay_ms %d", 
					bdata->reset_gpio, bdata->reset_on_state, bdata->reset_active_ms, bdata->reset_delay_ms);

	NDT_DEBUG_FUNC();

	return 0;
}

static int ndt_hw_reset(void)
{
	const struct ndt_stress_board_data *bdata = &gNdtData->bdata;
	
	if (bdata->reset_gpio >= 0) {
		gpio_set_value(bdata->reset_gpio, bdata->reset_on_state);
		mdelay(bdata->reset_active_ms);
		gpio_set_value(bdata->reset_gpio, !bdata->reset_on_state);
		mdelay(bdata->reset_delay_ms);
		return 0;
	}

	return -1;
}

static int ndt_stress_i2c_write( struct i2c_client *client, u16 addr, int len, void *txbuf )
{
	int ret;

	NDT_DEBUG_FUNC();

	if((txbuf == NULL) || (len == 0))
		{
		NDT_ERROR("txbuf is null, len is 0. write 0x%04X", addr);
		return 0;
		}

	ndt_i2c_comm_lock();
	
	ret = i2c_smbus_write_i2c_block_data(client, addr, len, txbuf); 	
	
	if(ret<0)
		{
		NDT_ERROR("ndt_stress_i2c_write addr 0x%04X", addr);
		ndt_i2c_comm_unlock();
		return -1;
		}

	ndt_i2c_comm_unlock();

	NDT_DEBUG_FUNC();

	return len;
}

static int ndt_stress_i2c_read(struct i2c_client *client, u16 addr, u16 len, void *rxbuf)
{
	int ret = -1;
	//int i; 	
	struct i2c_msg msg[2];
	char i2c_buffer[128] = {0};

	NDT_DEBUG_FUNC();

	if (( rxbuf == NULL ) ||(len == 0))
		{
		NDT_ERROR("rxbuf is null, len is 0. read 0x%04X", addr);
		return -1;
		}

	memset(msg, 0, sizeof(msg));
	memset(i2c_buffer, 0, sizeof(i2c_buffer));

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = (unsigned char *)&addr;

	msg[1].addr = client->addr;//client->addr | I2C_DMA_FLAG;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = i2c_buffer;

	ndt_i2c_comm_lock();

	ret = i2c_transfer(client->adapter, msg, 2);

	if(ret<0)
		{
		NDT_ERROR("ndt_stress_i2c_read addr 0x%04X", addr);
		ndt_i2c_comm_unlock();
		return -1;
		}

	ndt_i2c_comm_unlock();

	memcpy(rxbuf, i2c_buffer, len);

	NDT_DEBUG_FUNC();

	return len;
}

static int ndt_stress_write_eeprom(struct i2c_client *client,unsigned short reg,unsigned char* datbuf,int ByteNo)
{	
	unsigned char* buf;
	struct i2c_msg msg;
	int ret;
//	int count=0;

	//NDT_INFO("i2c_client addr: 0x%02X",client->addr);

	if(!datbuf) return -1;
	buf = (unsigned char*)kmalloc(ByteNo+2,GFP_KERNEL);
	if(!buf) return -1;
	
	memset(buf,0,ByteNo+2);
	buf[0]=(reg>>8)&0xff;
	buf[1]=reg&0xff;
	memcpy(buf+2,datbuf,ByteNo);

	//NDT_INFO("reg addr: 0x%02x,0x%02X",buf[0],buf[1]);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = ByteNo+2;
	msg.buf = buf;

	ndt_i2c_comm_lock();

	ret = i2c_transfer(client->adapter, &msg, 1);

	ndt_i2c_comm_unlock();

	if(ret<0)
		NDT_ERROR("i2c_master_send Error ! err_code:%d, reg: 0x%04X",ret, reg);
	//else
		//NDT_INFO("i2c_master_send OK !");

	kfree(buf);

	return ret;
}

static int ndt_stress_read_eeprom(struct i2c_client *client,unsigned short reg,unsigned char* datbuf,int ByteNo)
{
	struct i2c_msg msg[2];
	int ret;
	unsigned char reg16[2];
//	int count=0;

	//NDT_INFO("i2c_client addr: 0x%02X",client->addr);

	if(!datbuf) return -1;
	
	reg16[0] = (reg>>8)&0xff;
	reg16[1] = reg&0xff;
	
	//NDT_INFO("reg addr: 0x%02X,0x%02X",reg16[0],reg16[1]);

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = reg16;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = ByteNo;
	msg[1].buf = datbuf;

	ndt_i2c_comm_lock();

	ret = i2c_transfer(client->adapter, msg, 2);

	ndt_i2c_comm_unlock();
	
	if(ret<0)
		NDT_ERROR("i2c_master_send Error ! err_code:%d, reg: 0x%04X",ret, reg);
	//else
		//NDT_INFO("i2c_transfer OK !\n");

	return ret;
}

static int ndt_stress_jump_wait(void)
{
	unsigned char erase_cmd[10];
	unsigned short reg;
	int ByteNo = 0;

	reg = IIC_EEPROM;
	//erase_cmd[ByteNo++] = 0;
	//erase_cmd[ByteNo++] = 0;
	erase_cmd[ByteNo++] = 0x7e;
	erase_cmd[ByteNo++] = 0xe7;
	erase_cmd[ByteNo++] = 0xee;
	erase_cmd[ByteNo++] = 0x77;

	if(ndt_stress_write_eeprom(gNdtData->client,reg,erase_cmd,ByteNo) < 0)
		return -1;

	return 0;	
}

static int ndt_stress_power_switch(bool SwFlag, bool ForceFlag)
{

	struct ndt_stress_board_data *bdata = NULL;
	
	if(gNdtData == NULL) 
		return -1;

	bdata = &gNdtData->bdata;

	if(gNdtData->power_stat == SwFlag)
		{
		NDT_ERROR("power mode already in power %s", SwFlag? "on":"down");
		return 0;
		}

	if(SwFlag == true)
		{
		if (bdata->power_gpio >= 0) {
			gpio_set_value(bdata->power_gpio, bdata->power_on_state);
			msleep(bdata->power_delay_ms);
			}
		
		if (bdata->reset_gpio >= 0) {
			gpio_set_value(bdata->reset_gpio, !bdata->reset_on_state);
			//mdelay(bdata->reset_active_ms);
			}
		
		gNdtData->power_stat = true;

		NDT_INFO("+++++++++power on");
		}
	else if((gNdtData->flashprog == false) || (ForceFlag == true))
		{
		if (bdata->reset_gpio >= 0) {
			gpio_set_value(bdata->reset_gpio, bdata->reset_on_state);
			mdelay(bdata->reset_active_ms);
			}

		if (bdata->power_gpio >= 0) {
			gpio_set_value(bdata->power_gpio, !bdata->power_on_state);
			msleep(bdata->power_delay_ms);
			}

		gNdtData->power_stat = false;

		NDT_INFO("------------power down");
		}
	
	return 0;
}


#ifdef NDT_PRESS_USE_PM
int ndt_stress_suspend(void)
{
	struct ndt_stress_board_data *bdata = NULL;
	
	NDT_INFO("#########");

	if(ndt_Bon(gNdtData) || (gNdtData->appear == false))
		{
		//NDT_ERROR("gNdtData is NULL & appear is false");
		return -1;
		}
	
	if(gNdtData->suspend == true)
		{
		NDT_INFO("device already in suspended!!!!");
		return -1;
		}

	if(gNdtData->flashprog == true)
		{
		NDT_INFO("NDT do not suspend in prog flash mode !!!!");
		return 0;
		}

	if(gNdtData->devbusy == true)
		{
		NDT_INFO("suspend and close dev node!!!!");
		gNdtData->devbusy = false;
		}
	
	bdata = &gNdtData->bdata;

	ndt_stress_power_switch(false, false);

	gNdtData->suspend = true;
	gNdtData->dmode = DEBUG_MODE_NORMAL;

	return 0;
}

EXPORT_SYMBOL(ndt_stress_suspend);

int ndt_stress_resume(void)
{
	struct ndt_stress_board_data *bdata = NULL;
	
	NDT_INFO("#########");
	
	if(ndt_Bon(gNdtData) || (gNdtData->appear == false))
		{
		//NDT_ERROR("gNdtData is NULL & appear is false");
		return -1;
		}
	
	if(gNdtData->suspend == false)
		{
		NDT_INFO("device already in resume!!!!");
		return -1;
		}
	
	bdata = &gNdtData->bdata;
		
	ndt_stress_power_switch(true, false);

	ndt_hw_reset();

	ndt_stress_jump_wait();

	gNdtData->suspend = false;

	return 0;
}

EXPORT_SYMBOL(ndt_stress_resume);

#endif

static int ndt_stress_set_gpio(struct ndt_stress_data *ndt_data)
{
	int retval;
	const struct ndt_stress_board_data *bdata = &ndt_data->bdata;

	NDT_DEBUG_FUNC();

	retval = ndt_stress_gpio_setup(
                       	bdata->irq_gpio,
                     	true, 1, 0);
	if (retval < 0) {
		NDT_ERROR("Failed to configure attention GPIO");
		goto err_gpio_irq;
	}

	if (bdata->power_gpio >= 0) {
		retval = ndt_stress_gpio_setup(
				bdata->power_gpio,
				true, 1, !bdata->power_on_state);
		if (retval < 0) {
			NDT_ERROR("Failed to configure power GPIO");
			goto err_gpio_power;
		}
	}

	if (bdata->reset_gpio >= 0) {
		retval = ndt_stress_gpio_setup(
				bdata->reset_gpio,
				true, 1, !bdata->reset_on_state);
		if (retval < 0) {
			NDT_ERROR("Failed to configure reset GPIO");
			goto err_gpio_reset;
		}
	}

	if (bdata->power_gpio >= 0) {
		gpio_set_value(bdata->power_gpio, bdata->power_on_state);
		mdelay(bdata->power_delay_ms);
	}

	if (bdata->reset_gpio >= 0) {
		gpio_set_value(bdata->reset_gpio, bdata->reset_on_state);
		mdelay(bdata->reset_active_ms);
		gpio_set_value(bdata->reset_gpio, !bdata->reset_on_state);
		mdelay(bdata->reset_delay_ms);
	}

	gNdtData->power_stat = true;

	NDT_DEBUG_FUNC();
	
	return 0;

err_gpio_irq:
	if(bdata->irq_gpio >= 0)
		ndt_stress_gpio_setup(bdata->irq_gpio, false, 0, 0);
err_gpio_reset:
	if(bdata->power_gpio >= 0)
		ndt_stress_gpio_setup(bdata->power_gpio, false, 0, 0);

err_gpio_power:
	ndt_stress_gpio_setup(bdata->irq_gpio, false, 0, 0);

	return retval;
}

void ndt_i2c_comm_lock(void)
{
	NDT_DEBUG_FUNC();
	
	if((gNdtData == NULL) || (gNdtData->appear == false))
		return;

	//NDT_INFO("lock ++++++++++++");
	
	mutex_lock(&(gNdtData->i2c_mutex));

	NDT_DEBUG_FUNC();

	return;
}

EXPORT_SYMBOL(ndt_i2c_comm_lock);

void ndt_i2c_comm_unlock(void)
{
	NDT_DEBUG_FUNC();
	
	if((gNdtData == NULL) || (gNdtData->appear == false))
		return;
	
	mutex_unlock(&(gNdtData->i2c_mutex));

	//NDT_INFO("lock ----------------");

	NDT_DEBUG_FUNC();

	return;
}

EXPORT_SYMBOL(ndt_i2c_comm_unlock);

static void ndt_update_send_coord_work(struct work_struct *work)
{
	unsigned char data_no =0;
	unsigned char data = 1;
	unsigned char buffer[5] = {0};
	int batt_temp = 0;
	unsigned short press_buf1 = 0;
	unsigned short x1 = 0, y1 = 0;
	unsigned char status = 0;

//------------------------------------------------------------------------------------
//get data
	x1 = gNdtData->x1;
	y1 = gNdtData->y1;
	status = gNdtData->status;
//------------------------------------------------------------------------------------
//send coord
	memcpy(buffer, (char *)&x1, sizeof(x1));
	
	memcpy(buffer + sizeof(x1), (char *)&y1, sizeof(y1));
	
	memcpy(buffer + sizeof(x1)+ sizeof(y1), &status, sizeof(status));

	ndt_stress_i2c_write(gNdtData->client, 0x11, sizeof(buffer), buffer);
//------------------------------------------------------------------------------------
//send coord num
	data = status ? 1 : 0;

	ndt_stress_i2c_write(gNdtData->client, 0x10, sizeof(char),  &data);

    /* Clean the press value after finger up */
	if(!status){
        gNdtData->value1 = 0;
	 gNdtData->value2 = 0;
	}

	if(gNdtData->dmode == DEBUG_MODE_LOG)
		NDT_INFO("###### x1:0x%04X y1:0x%04X status:%u", x1, y1, status);
//------------------------------------------------------------------------------------
	if(status)
		{
		batt_temp = get_batt_therm_globle();//send therm
		ndt_stress_i2c_write(gNdtData->client, 0x52, sizeof(batt_temp), (u8 *)&batt_temp);

		ndt_stress_i2c_read(gNdtData->client, 0x20, 1, &data_no);

		if(gNdtData->dmode == DEBUG_MODE_LOG)
			NDT_INFO("###### data ready:%u",data_no);
		
		if(data_no > 0)//get stress
			{
			ndt_stress_i2c_read(gNdtData->client, 0x21, sizeof(press_buf1), &press_buf1);
			mutex_lock(&(gNdtData->stress_value_mutex));
			gNdtData->value1 = press_buf1;
			
			if(gNdtData->dmode == DEBUG_MODE_LOG)
				NDT_INFO("###### read press value:%u", press_buf1);
	
			mutex_unlock(&(gNdtData->stress_value_mutex));

			data_no = 0;
			ndt_stress_i2c_write(gNdtData->client, 0x20, 1, &data_no);
			}
		else
			NDT_INFO("###### data ready: %u", data_no);
		}
//------------------------------------------------------------------------------------

}

// ZTE_modify add press status begin
#define PRESS_STATUS_BUF_SIZE 5
unsigned char press_status_buf[PRESS_STATUS_BUF_SIZE] = {0};
//ZTE_modify end
// ZTE_modify add for stress calibration begin
#define NDT_X_MAX_RANGE 1080
#define NDT_Y_MAX_RANGE 1920
extern u32 gt1x_abs_x_max;
extern u32 gt1x_abs_y_max;
//ZTE_modify end
int ndt_stress_set_vlaue(unsigned short x1,  unsigned short y1, unsigned char status)
{
	// ZTE_modify add for stress calibration begin
	unsigned short x_temp, y_temp;
	//ZTE_modify end
	NDT_DEBUG_FUNC();
	// ZTE_modify add for stress calibration begin
	if ((gt1x_abs_x_max==NDT_X_MAX_RANGE) || (gt1x_abs_x_max==0)){
		x_temp = x1;
	}
        else{
		x_temp =(unsigned short) ((u32)x1 * NDT_X_MAX_RANGE / gt1x_abs_x_max);
	}

	if ((gt1x_abs_y_max==NDT_Y_MAX_RANGE) || (gt1x_abs_y_max==0)){
		y_temp = y1;
	}
        else{
		y_temp =(unsigned short) ((u32)y1 * NDT_Y_MAX_RANGE / gt1x_abs_y_max);
	}

	memcpy(press_status_buf, (char *)&x_temp, sizeof(x_temp));
	memcpy(press_status_buf + sizeof(x_temp), (char *)&y_temp, sizeof(y_temp));
	memcpy(press_status_buf + sizeof(x_temp)+ sizeof(y_temp), &status, sizeof(status));

	//ZTE_modify end
	if((gNdtData == NULL) || (gNdtData->appear == false) ||(gNdtData->dmode == DEBUG_MODE_DDATA)
		||(gNdtData->flashprog == true) || (gNdtData->devbusy == true))
		return -1;

	if(gNdtData->dmode == DEBUG_MODE_LOG)
		NDT_INFO("*************x1:0x%04X y1:0x%04X status:%u", x1, y1, status);

	gNdtData->x1 = x1;
	gNdtData->y1 = y1;
	gNdtData->status = status;

	queue_delayed_work(gNdtData->ndt_coord_handle, &gNdtData->send_coord_work, 0);

	NDT_DEBUG_FUNC();

	return 0;
}

EXPORT_SYMBOL(ndt_stress_set_vlaue);

int ndt_stress_get_vlaue(unsigned int *value)
{
	NDT_DEBUG_FUNC();

	if((gNdtData == NULL) || (gNdtData->appear == false) ||(gNdtData->dmode == DEBUG_MODE_DDATA)
		||(gNdtData->flashprog == true) ||(gNdtData->devbusy == true))
		return -1;

	if(value == NULL) 
	{
		NDT_ERROR("value == NULL");
		goto ExitLoop;
	}

	mutex_lock(&(gNdtData->stress_value_mutex));
	
       // ZTE_modify add custom gain control begin
	*value = gNdtData->custom_gain_value * (unsigned int) (gNdtData->value1)/100;
	   
	if(*value > FORCE_MAX_VALUE)
		*value = FORCE_MAX_VALUE;

	*value = (*value) >> 1;

	*value = (*value) ? (*value) : 1;

	gNdtData->value2 = *value;
	
      //pr_debug("%s value1=%d, value=%d\n", __func__, gNdtData->value1, *value);
      // ZTE_modify end
      
	mutex_unlock(&(gNdtData->stress_value_mutex));
	
	if(gNdtData->dmode == DEBUG_MODE_LOG)
		NDT_INFO("*************get value: %u", *value);

ExitLoop:
	NDT_DEBUG_FUNC();

	return 0;
}

EXPORT_SYMBOL(ndt_stress_get_vlaue);

#ifdef CONFIG_NDT_USE_UPDATE
int ndt_fwu_burn_fw(unsigned char* buf,unsigned int len)
{
	struct i2c_client *force_key_client = gNdtData->client;
	unsigned char erase_cmd[10];
	unsigned short reg;
	int ByteNo = 0;
	int pos = 0;
	unsigned char* read_buf = NULL;
	int ret = 1;
	int i=0;
	int number=0;

	NDT_INFO("burn fw start");
	
	if(len % 128 != 0)
		{
		NDT_ERROR("burn len is not 128*");
		return -2;//no need remove device, can not return -1
		}

	read_buf = (unsigned char*)kmalloc(len,GFP_KERNEL);
	if(read_buf == NULL)
		{
		NDT_ERROR("kmalloc failed");
		return -2;//no need remove device, can not return -1
		}
	
	do{
		NDT_INFO("burn eeprom number: %d",number+1);

		ret = 0;
		ByteNo = 0;
			
		//reset
		ndt_hw_reset();
		
		//erase flash
		reg = IIC_EEPROM;
		//erase_cmd[ByteNo++] = 0;
		//erase_cmd[ByteNo++] = 0;
		erase_cmd[ByteNo++] = 0xaa;
		erase_cmd[ByteNo++] = 0x55;
		erase_cmd[ByteNo++] = 0xa5;
		erase_cmd[ByteNo++] = 0x5a;
		
		if(ndt_stress_write_eeprom(force_key_client,reg,erase_cmd,ByteNo)>0)
			NDT_INFO("reg=0x%02x,ByteNo=%d",reg,ByteNo);
	  	else
	  		{
	  		ret = -1;
			NDT_ERROR("erase flash failed!");
			goto failedloop;
	  		}
		
		mdelay(2000);
		
		//write eeprom
		pos = 0;
		reg = IIC_EEPROM;
		ByteNo = 128;
		while(pos < len)
			{
			if(ndt_stress_write_eeprom(force_key_client,reg,buf+pos,ByteNo) < 0)
				{
				ret = -1;
				NDT_ERROR("write eeprom failed!");
				goto failedloop;
				}
			
			pos += ByteNo;
			reg += ByteNo;
			mdelay(10);
			}

		NDT_INFO("write reg=0x%02x,ByteNo=%d",reg,ByteNo);

		//print_mem(buf, 512);
	
		//read eeprom and check
		pos = 0;
		reg = IIC_EEPROM;
		ByteNo = 128;
		while(pos < len)
			{
			if(ndt_stress_read_eeprom(force_key_client,reg,read_buf+pos,ByteNo) < 0)
				{
				ret = -1;
				NDT_ERROR("read eeprom failed!\n");
				goto failedloop;
				}
			
			pos += ByteNo;
			reg += ByteNo;
			mdelay(10);
			}

		NDT_INFO("read reg=0x%02x,ByteNo=%d",reg,ByteNo);
		//print_mem(read_buf, 512);
	
		//check
		for(i = 0;i < len;i++)
			{
			if(buf[i] != read_buf[i])
				{
				ret = -1;
				NDT_ERROR("[0x%08X][0x%08X][0x%08X]burn check error!\n", i, buf[i], read_buf[i]);
				break;
				}
			}
		number++;
	}while(number < 3 && -1 == ret);

	if(-1 == ret)
		{
		NDT_ERROR("burn eeprom fail!\n");
		goto failedloop;
		}
	else
		NDT_INFO("burn eeprom succeed!\n");
	
	//exit burn
	ndt_hw_reset();

	reg = IIC_EEPROM;
	ByteNo = 0;
	//erase_cmd[ByteNo++] = 0;
	//erase_cmd[ByteNo++] = 0;
	erase_cmd[ByteNo++] = 0x7e;
	erase_cmd[ByteNo++] = 0xe7;
	erase_cmd[ByteNo++] = 0xee;
	erase_cmd[ByteNo++] = 0x77;
	
	if(ndt_stress_write_eeprom(force_key_client,reg,erase_cmd,ByteNo)>0)
		NDT_INFO("reg=0x%02x,ByteNo=%d\n",reg,ByteNo);
	else
		{
		ret = -1;
		NDT_ERROR("exit burn failed!\n");
		}

failedloop:	
	if(read_buf != NULL)
		{
		kfree(read_buf);
		read_buf = NULL;
		}

	NDT_INFO("burn fw end\n");

	return ret;
}
static int ndt_fwu_start_reflash(void)
{
	struct file *fp = NULL;
	mm_segment_t fs;
	loff_t pos;
	unsigned short file_fw_ver;
	int ret = 0;
	int len=0;
	char fw_data_len[4];
	char *fw_data = NULL;
	char *filename = NULL;

	if(gNdtData->force_update == true)
		filename = gNdtData->update_file;
	else
		filename = NDT_FW_PATH;

	NDT_INFO("#### force_update %u, update FW name \"%s\"",gNdtData->force_update, filename);
	
	fp = filp_open(filename, O_RDONLY, 0);
	if (IS_ERR(fp))
		{
		NDT_ERROR("open \"%s\" failed !!!", filename);
		goto ExitLoop;
		}
	
	fs = get_fs();
	set_fs(KERNEL_DS);

	pos = 8;
	vfs_read(fp, (char *)&file_fw_ver, sizeof(file_fw_ver), &pos);
	NDT_INFO("read file_fw_ver: 0x%04X", ntohs(file_fw_ver));

	if((gNdtData->FirmwareVer <= 0x0C) && 0)
		{
		NDT_INFO("Old sensor,No need to update FW !!!");
		goto ExitLoop;
		}

	if((gNdtData->force_update == false) && (gNdtData->FirmwareVer < 0x100) && (ntohs(file_fw_ver) <= gNdtData->FirmwareVer))
		{
		NDT_INFO("No need to update FW");
		goto ExitLoop;
		}

	pos = 0x0c;
	vfs_read(fp, fw_data_len, 4, &pos);
	NDT_INFO("read fw_data_len: 0x%02x%02x%02x%02x", fw_data_len[0],fw_data_len[1],fw_data_len[2],fw_data_len[3]);
	len = (int)((unsigned int)fw_data_len[3]<<0)|((unsigned int)fw_data_len[2]<<8)|((unsigned int)fw_data_len[1]<<16)|(fw_data_len[0]<<24);

	fw_data = (char*)kmalloc(len,GFP_KERNEL);
	if(fw_data == NULL)
		{
		NDT_INFO("kmalloc failed!!!");
		goto ExitLoop;
		}
	
	pos = 0x100;
	vfs_read(fp, fw_data, len, &pos);

	ret = ndt_fwu_burn_fw(fw_data,len);
	if(ret == -2)
		{
		ret = 0;//no need remove device
		NDT_ERROR("Burn FW read or write file failed!!!");
		}
	else if(ret == -1)
		NDT_ERROR("Burn FW i2c communicate failed!!!");
	else if(ret == 0)
		gNdtData->FirmwareVer = ntohs(file_fw_ver);
	else
		NDT_ERROR("en....,bug bug bug!!!");
	
ExitLoop:
	if (!IS_ERR(fp))
		{
		filp_close(fp,NULL);
		fp = NULL;
		set_fs(fs);
		}

	if(fw_data)
		{
		kfree(fw_data);
		fw_data = NULL;
		}

	return ret;
}

static void ndt_update_handle_work(struct work_struct *work)
{
	int ret = 0;
	
	NDT_DEBUG_FUNC();
	
	NDT_INFO("update handle work into");

	if(gNdtData->flashprog == true)
		{
		NDT_ERROR("device already in flash prog mode or i2c char dev opened!!!");
		return;
		}

	ndt_stress_power_switch(true, false);
	
	gNdtData->flashprog = true;

	ret = ndt_fwu_start_reflash();
	if(ret == -1)
		NDT_ERROR("Can not find the i2c slave device!!!");
	else if((gNdtData->force_update == true) && (gNdtData->appear == false))
		{
		gNdtData->force_update = false;
		gNdtData->appear = true;
		NDT_INFO("Bad firmware, force update sucessed!!!");
		}

	gNdtData->flashprog = false;

	if(ret == -1)//not found i2c slave device, power down
		ndt_stress_power_switch(false, false);

	NDT_DEBUG_FUNC();

	return;
}

static void ndt_force_firmware_update(void)
{
	NDT_INFO("!!!-_-!!!force update firmware !!!-_-!!!");

	memset(gNdtData->update_file, 0, sizeof(gNdtData->update_file));
	strcpy(gNdtData->update_file, NDT_FW_PATH);
	
	gNdtData->dmode = 0;
	gNdtData->force_update = true;

	queue_delayed_work(gNdtData->ndt_debug_queue, &gNdtData->update_work, msecs_to_jiffies(10000));
}

#endif

#ifdef CONFIG_NDT_USE_DEBUG

static void ndt_debug_handle_work(struct work_struct *work)
{
	struct i2c_client *client = gNdtData->client;
	unsigned char ReadFlag = 0, i;
	short *ptr = NULL;
	short buffer[16] = {0} ;
	int ret, temp = 0;

	NDT_DEBUG_FUNC();
	
	if((gNdtData != NULL) && (gNdtData->dmode) && (gNdtData->appear == true) 
		&& (gNdtData->suspend == false) && (gNdtData->flashprog == false) && (gNdtData->devbusy == false))
		{
		if(gNdtData->dmode == DEBUG_MODE_DDATA)
			{
			ret = ndt_stress_i2c_read(client, 0x61, 1, &ReadFlag);
			if(ret < 0 )	goto retry_loop;

			if((ReadFlag > 0) && (ReadFlag <= 16))
				{
				ptr = buffer;
				for(i = 0; i < ReadFlag; i++)
					{
					ret = ndt_stress_i2c_read(client, 0x62 + i, sizeof(short), (u8 *)ptr);
					if(ret < 0 )	goto retry_loop;
					
					ptr++;
					}
				}

				//print_mem((char *)buffer, ReadFlag * sizeof(short));

				if(ReadFlag == 1)
					{
					NDT_INFO("***************** Debug value: (%05d) - (%05d) = %d!!!", buffer[0], gNdtData->dvalue1, buffer[0] - gNdtData->dvalue1);
					mutex_lock(&(gNdtData->stress_value_mutex));
					gNdtData->dvalue2 = buffer[0];
					mutex_unlock(&(gNdtData->stress_value_mutex));
					}
				else if(ReadFlag == 4)
					{		
					mutex_lock(&(gNdtData->stress_value_mutex));
					memcpy((char *)gNdtData->sensor, (char *)buffer, ReadFlag * sizeof(short));
					mutex_unlock(&(gNdtData->stress_value_mutex));
					temp = get_batt_therm_globle();
					printk("ndt: %d\t%d\t%d\t%d\t%d\n",temp, buffer[0], buffer[1], buffer[2], buffer[3]);
					}
			
				ReadFlag = 0;
				
				ndt_stress_i2c_write(gNdtData->client, 0x61, 1, &ReadFlag);
				
			}
		}
	
retry_loop:

	NDT_DEBUG_FUNC();

	if(gNdtData->dmode == DEBUG_MODE_DDATA)
		queue_delayed_work(gNdtData->ndt_debug_queue, &gNdtData->debug_work, msecs_to_jiffies(10));
}

static int debug_proc_show(struct seq_file *m, void *v)
{
	NDT_DEBUG_FUNC();

	seq_printf(m, "NDT Driver Built:	%s, %s\n",__DATE__ ,__TIME__ );
	seq_printf(m, "NDT data init: 		%s\n", gNdtData != NULL ? "yes": "no");
	
	if(gNdtData == NULL)
		return 0;
	
	seq_printf(m, "NDT firmware versions: 	0x%04X\n", gNdtData->FirmwareVer);
	seq_printf(m, "NDT appear: 		%s\n", gNdtData->appear ? "yes": "no");
	seq_printf(m, "NDT debug mode: 	%u\n", gNdtData->dmode);
	seq_printf(m, "NDT suspend: 		%s\n", gNdtData->suspend ? "yes": "no");
	seq_printf(m, "NDT flashprog: 		%s\n", gNdtData->flashprog? "yes": "no");
	seq_printf(m, "NDT devbusy: 		%s\n", gNdtData->devbusy? "yes": "no");

	mutex_lock(&(gNdtData->stress_value_mutex));
	seq_printf(m, "NDT Press value:	%u\n", gNdtData->value2);
	mutex_unlock(&(gNdtData->stress_value_mutex));
	
	seq_printf(m, "NDT gain value:		%u\n", gNdtData->custom_gain_value);
	

	NDT_DEBUG_FUNC();

	return 0;
}

static int debug_proc_open(struct inode *inode, struct file *file)
{
	NDT_DEBUG_FUNC();
	
	return single_open(file, debug_proc_show, NULL);
}

static ssize_t debug_proc_write(struct file *file, const char __user *buffer,
				    size_t count, loff_t *pos)
{
	unsigned int input = 0;
	unsigned char debug_mode = 0;
	
	NDT_DEBUG_FUNC();

	if (sscanf(buffer, "%u", &input) != 1)
		{
		NDT_ERROR("input sscanf failed !!!");
		return -EINVAL;
		}

	NDT_INFO("input %u", input);
	
	if ((gNdtData == NULL) ||(input == gNdtData->dmode) 
		||(gNdtData->flashprog == true) || (gNdtData->devbusy == true))
		{
		NDT_ERROR("dmode = input || flashprog = true || dev busy");
		return count;
		}

	if(input >= DEBUG_MODE_LOG) 
		{
		mutex_lock(&(gNdtData->stress_value_mutex));

		if(input == DEBUG_MODE_LOG)
			gNdtData->dmode = DEBUG_MODE_LOG;
		else
			gNdtData->dvalue1 = gNdtData->dvalue2;
		
		mutex_unlock(&(gNdtData->stress_value_mutex));
		
		return count;
		}

	gNdtData->dmode = input;

	NDT_INFO("set dmode %u", gNdtData->dmode);

	if(gNdtData->suspend == true)
		{
		NDT_INFO("suspend is true");
		return count;
		}

	debug_mode = (gNdtData->dmode == DEBUG_MODE_NORMAL)? 0 : 1;
	ndt_stress_i2c_write(gNdtData->client, 0x60, sizeof(debug_mode), &debug_mode);

	if(gNdtData->dmode == DEBUG_MODE_DDATA)
		queue_delayed_work(gNdtData->ndt_debug_queue, &gNdtData->debug_work, msecs_to_jiffies(10));

	NDT_DEBUG_FUNC();

	return count;
}


static const struct file_operations debug_proc_fops = {
	.open		= debug_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release		= single_release,
	.write		= debug_proc_write,
};

static int debug_proc_init(void)
{
       struct proc_dir_entry *res;
	   
	NDT_DEBUG_FUNC();
	
	res = proc_create("driver/ndt_debug", 0x664, NULL,
			  &debug_proc_fops);
	if (!res)
		{
		NDT_ERROR("failed to create ndt_debug!!!");
		return -ENOMEM;
		}

	NDT_INFO("created ndt_debug");

	NDT_DEBUG_FUNC();
	
	return 0;
}

#endif

#ifdef CONFIG_NDT_USE_PV_TEST
static int test_proc_show(struct seq_file *m, void *v)
{
	short *SenData = gNdtData->sensor;
	unsigned int i, count;
	
	NDT_DEBUG_FUNC();

	count = sizeof(gNdtData->sensor)/sizeof(gNdtData->sensor[0]);

	mutex_lock(&(gNdtData->stress_value_mutex));
	
	for(i = 0; i < count; i++)
		{
		seq_printf(m, "%d",SenData[i]);

		if(i != count -1)
			seq_printf(m, ",");
		}

	seq_printf(m, "\n");
	
	mutex_unlock(&(gNdtData->stress_value_mutex));

	NDT_DEBUG_FUNC();

	return 0;
}

void test_check_file_name(char *buffer, unsigned int len)
{
	unsigned int i;
	
	for(i = 0; i < len; i++)
		{
		if((buffer[i] == '\n') || (buffer[i] == '\r') || (buffer[i] == '\t'))
			buffer[i] = '\0';
		}

	return;
}

static ssize_t test_proc_write(struct file *file, const char __user *buffer,
				    size_t count, loff_t *pos)
{
	NDT_DEBUG_FUNC();

	NDT_INFO("write file name \"%s\"", buffer);

	if((gNdtData == NULL) ||(gNdtData->suspend == true))
		{
		NDT_ERROR("gNdtData NULL, suspend is true");
		return count;
		}

	if((count > sizeof(gNdtData->update_file)) || (count == 0))
		{
		NDT_ERROR("write count %u, update file name len %u", (unsigned int)count,(unsigned int)sizeof(gNdtData->update_file));
		return count;
		}
	
	memset(gNdtData->update_file, 0, sizeof(gNdtData->update_file));

	if(copy_from_user(gNdtData->update_file, buffer, count) != 0)
		{
		memset(gNdtData->update_file, 0, sizeof(gNdtData->update_file));
		NDT_ERROR("copy_from_user file name failed !!!");
		return count;
		}

	test_check_file_name(gNdtData->update_file, sizeof(gNdtData->update_file));
	
#ifdef CONFIG_NDT_USE_UPDATE
	gNdtData->dmode = 0;
	gNdtData->force_update = true;
	queue_delayed_work(gNdtData->ndt_debug_queue, &gNdtData->update_work, msecs_to_jiffies(100));
#endif


	NDT_DEBUG_FUNC();

	return count;
}


static int test_proc_open(struct inode *inode, struct file *file)
{
	NDT_DEBUG_FUNC();
	
	return single_open(file, test_proc_show, NULL);
}

static const struct file_operations test_proc_fops = {
	.open		= test_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release		= single_release,
	.write		= test_proc_write,
};

static int test_proc_init(void)
{
       struct proc_dir_entry *res;
	   
	NDT_DEBUG_FUNC();
	
	res = proc_create("driver/press_test", 0x664, NULL,
			  &test_proc_fops);
	if (!res)
		{
		NDT_ERROR("failed to create press_test!!!");
		return -ENOMEM;
		}

	NDT_INFO("created press_test");

	NDT_DEBUG_FUNC();
	
	return 0;
}

#endif

#ifdef CONFIG_NDT_USE_CHAR_DEV
static void ndt_dev_watchdog_work(struct work_struct *work)
{
	NDT_DEBUG_FUNC();

	if((gNdtData->devbusy == true) && (gNdtData->alive_flag == true))
		{
		NDT_INFO("ndt dev alive!!!");
		gNdtData->alive_flag = false;
		queue_delayed_work(gNdtData->ndt_debug_queue, &gNdtData->watchdog_work, msecs_to_jiffies(15000));
		}
	else
		{
		NDT_INFO("ndt dev timeout found!!!");
		if(gNdtData->devbusy == true)
			{
			gNdtData->devbusy = false;
			ndt_stress_power_switch(false, false);
			msleep(20);
			ndt_stress_power_switch(true, false);
			}
		}

	return;
}

static ssize_t dev_i2c_read(struct file *filp, char __user *usr_p,
		size_t count, loff_t *pos)
{
	char *buffer = NULL;
	int ret = 0;
	
	NDT_DEBUG_FUNC();

	if (count == 0)
		return 0;

	buffer = kzalloc(count + 1, GFP_KERNEL);
	if(buffer == NULL)
		{
		NDT_ERROR("kzalloc failed.\n");
		return -ENOMEM;
		}

	ret = ndt_stress_i2c_read(gNdtData->client, *pos, count, buffer);

	if (copy_to_user(usr_p, buffer, count))
		ret = -EFAULT;
	else
		*pos += ret;

	NDT_DEBUG_FUNC();

	if(buffer)
		{
		kfree(buffer);
		buffer = NULL;
		}

	return ret;
}

static ssize_t dev_i2c_write(struct file *file, const char __user *usr_p,
				    size_t count, loff_t *pos)
{
	char *buffer = NULL;
	int ret = 0;
	
	NDT_DEBUG_FUNC();
	
	if (count == 0)
		return 0;

	buffer = kzalloc(count + 1, GFP_KERNEL);
	if(buffer == NULL)
		{
		NDT_ERROR("kzalloc failed.\n");
		return -ENOMEM;
		}

	if (copy_from_user(buffer, usr_p, count))
		{
		NDT_ERROR("copy from user failed!!!");
		goto ExitLoop;
		}

	ret = ndt_stress_i2c_write(gNdtData->client, *pos, count, buffer);

	if(ret >= 0)
		pos += ret;

	NDT_DEBUG_FUNC();

ExitLoop:

	if(buffer)
		{
		kfree(buffer);
		buffer = NULL;
		}

	return ret;
}

static int dev_i2c_open(struct inode *inode, struct file *filp)
{
	NDT_DEBUG_FUNC();

	if (gNdtData->suspend) {
		NDT_ERROR("ndt sensor in suspend");
		return -ENOSPC;
	}

	NDT_INFO("current pid %d", current->pid);

	if((gNdtData->dmode == DEBUG_MODE_DDATA) || (gNdtData->flashprog == true) || (gNdtData->devbusy == true)){
		NDT_ERROR("ndt sensor in debug or reduplicate open");
		return -EBUSY;
	}
	
	queue_delayed_work(gNdtData->ndt_debug_queue, &gNdtData->watchdog_work, msecs_to_jiffies(15000));
	
	gNdtData->devbusy = true;
	
	NDT_DEBUG_FUNC();
	
	return 0;
}

static loff_t dev_i2c_llseek(struct file *filp, loff_t off, int whence)
{
	loff_t newpos;

	if(gNdtData->alive_flag == false)//feed dog
			gNdtData->alive_flag = true;

	switch (whence) {
	case SEEK_SET:
		newpos = off;
		break;
	case SEEK_CUR:
		newpos = filp->f_pos + off;
		break;
	case SEEK_END:
		newpos = 0xFF + off;
		break;
	default:
		newpos = -EINVAL;
		goto clean_up;
	}

	if (newpos < 0 || newpos > 0xFF) {
		newpos = -EINVAL;
		goto clean_up;
	}

	filp->f_pos = newpos;

clean_up:

	return newpos;
}


static int dev_i2c_release(struct inode *inode, struct file *file)
{
	NDT_DEBUG_FUNC();

	gNdtData->devbusy = false;
	
	return 0;
}

static long dev_i2c_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int data = 0;

	if (_IOC_TYPE(cmd) != NDT_IOC_MAGIC)
        return -EINVAL;

	if ((_IOC_DIR(cmd) & _IOC_READ) && (!access_ok(VERIFY_WRITE, (void *)arg, _IOC_SIZE(cmd))))
       	return -EFAULT;

	if((_IOC_DIR(cmd) & _IOC_WRITE) && (!access_ok(VERIFY_READ, (void *)arg, _IOC_SIZE(cmd))))
        	 return -EFAULT;

	switch(cmd) 
		{
		case NDT_IOC_HW_RESET:
			NDT_INFO("NDT_IOC_HW_RESET");
			ndt_stress_power_switch(false, true);
			msleep(20);
			ndt_stress_power_switch(true, false);
			ndt_hw_reset();
			ndt_stress_jump_wait();
			msleep(50);
			break;
		case NDT_IOC_READ:
			NDT_INFO("NDT_IOC_READ");
			if(copy_to_user((int *)arg, &data, sizeof(data)) != 0)
				return -EINVAL;
			break;
		case NDT_IOC_WRITE:
			NDT_INFO("NDT_IOC_WRITE");
			if(copy_from_user(&data, (int *)arg, sizeof(data)) != 0)
				return -EINVAL;
			break;
		default: 
			return -EINVAL;
		}
		
    return ret;
}

static const struct file_operations app_proc_fops = {
	.owner 			= THIS_MODULE,
	.open			= dev_i2c_open,
	.read			= dev_i2c_read,
	.write			= dev_i2c_write,
	.llseek 			= dev_i2c_llseek,
	.unlocked_ioctl 	= dev_i2c_ioctl,
	.release			= dev_i2c_release,
};

static int dev_i2c_init(void)
{
	int ret;

	NDT_DEBUG_FUNC();
	
	ret = register_chrdev(NDT_CHAR_MAJOR, NDT_DEVICE_NAME, &app_proc_fops);
	if(ret < 0)
		{
		NDT_ERROR("can't register major number\n");
		return ret;
		}

	gNdtData->i2c_class = class_create(THIS_MODULE, NDT_DEVICE_NAME);
	if(IS_ERR(gNdtData->i2c_class))
		{
		NDT_ERROR("failed in My_led class.\n");
		return -1;
		}
	
	device_create(gNdtData->i2c_class, NULL, MKDEV(NDT_CHAR_MAJOR,0), NULL, NDT_DEVICE_NAME);

	INIT_DELAYED_WORK(&gNdtData->watchdog_work, ndt_dev_watchdog_work);

	NDT_DEBUG_FUNC();
	
	return 0;
}

#endif


#ifdef CONFIG_NDT_USE_TEST_I2C
static int ndt_i2c_test(void)
{
	unsigned short FirmwareVer = 0;
	unsigned char i = 0;
	
	NDT_DEBUG_FUNC();
	
	do
		{
		mdelay(15 + i * 30);
		
		if(ndt_stress_i2c_read(gNdtData->client, 0x05, sizeof(FirmwareVer), (unsigned char *)&FirmwareVer) != -1)
			{
			gNdtData->FirmwareVer = FirmwareVer;
			gNdtData->appear = true;
			NDT_INFO("Firmware Version: 0x%04x\n", FirmwareVer);
			return 0;
			}
		}while(i++ < 11);

	gNdtData->appear = false;
	
	NDT_ERROR("NTD device test failed!!!!\n");

	NDT_DEBUG_FUNC();
	
	return -1;
}
#endif

// ZTE_modify add proc/driver/pressure_id begin
static int pressure_proc_show(struct seq_file *m, void *v)
{
	NDT_DEBUG_FUNC();

	if(gNdtData == NULL)
		return 0;

	seq_printf(m, "NDT-KL26Z-NA-NA-NA\n");

	return 0;
}

static int pressure_proc_open(struct inode *inode, struct file *file)
{
	NDT_DEBUG_FUNC();

	return single_open(file, pressure_proc_show, NULL);
}

static const struct file_operations pressure_proc_fops = {
	.open		= pressure_proc_open,
	.read		= seq_read,
	.release		= single_release,
};

static int pressure_proc_init(void)
{
       struct proc_dir_entry *res;

	NDT_DEBUG_FUNC();

	res = proc_create("driver/pressure_id", 0444, NULL,
			  &pressure_proc_fops);
	if (!res)
	{
		NDT_ERROR("failed to create ndt_debug!!!");
		return -ENOMEM;
	}

	return 0;
}
// ZTE_modify end

// ZTE_modify add custom gain control begin
static int custom_gain_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "Force screen gain=%u\n", gNdtData->custom_gain_value);

	return 0;
}

static int custom_gain_proc_open(struct inode *inode, struct file *file)
{
	NDT_DEBUG_FUNC();
	
	return single_open(file, custom_gain_proc_show, NULL);
}

static ssize_t custom_gain_proc_write(struct file *file, const char __user *buffer,
				    size_t count, loff_t *pos)
{
	NDT_DEBUG_FUNC();

	if (sscanf(buffer, "%u", &(gNdtData->custom_gain_value)) != 1)
	{
		NDT_ERROR("input custom gain failed !!!");
		return -EINVAL;
	}

	NDT_INFO("custom gain %u", gNdtData->custom_gain_value);

	return count;
}


static const struct file_operations custom_gain_proc_fops = {
	.open		= custom_gain_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release		= single_release,
	.write		= custom_gain_proc_write,
};

static int custom_gain_proc_init(void)
{
       struct proc_dir_entry *res;
	   
	NDT_DEBUG_FUNC();
	
	res = proc_create("driver/custom_gain_set", S_IWUGO | S_IRUGO, NULL,
			  &custom_gain_proc_fops);
	if (!res)
	{
		NDT_ERROR("failed to create custom_gain_set!!!");
		return -ENOMEM;
	}

	return 0;
}
// ZTE_modify end

// ZTE_modify add press status begin
static int press_status_open(struct inode *inode, struct file *file)
{
      pr_debug("%s\n", __func__);
	return 0;
}

static int press_status_release(struct inode *inode, struct file *file)
{
	pr_debug("%s\n", __func__);
	
	return 0;
}

static ssize_t press_status_read(struct file *filp, char __user *usr_p,
		size_t count, loff_t *pos)
{
	int ret = count;
       pr_debug("%s count=%ld, *pos=%lld\n", __func__, count, *pos);
	if ((count == 0) || (count > PRESS_STATUS_BUF_SIZE))
		return 0;

	if (copy_to_user(usr_p, &press_status_buf[*pos], count))
		ret = -EFAULT;

       pr_debug("%s ret=%d\n", __func__, ret);
	return ret;
}

static loff_t press_status_llseek(struct file *filp, loff_t off, int whence)
{
	loff_t newpos;

	switch (whence) {
	case SEEK_SET:
		newpos = off;
		break;
	case SEEK_CUR:
		newpos = filp->f_pos + off;
		break;
	case SEEK_END:
		newpos = PRESS_STATUS_BUF_SIZE;
		break;
	default:
		newpos = -EINVAL;
		goto clean_up;
	}

	if (newpos < 0 || newpos > PRESS_STATUS_BUF_SIZE-1) {
		newpos = -EINVAL;
		goto clean_up;
	}

	filp->f_pos = newpos;

clean_up:

	return newpos;
}

static const struct file_operations press_status_fops = {
	.owner 			= THIS_MODULE,
	.open			= press_status_open,
	.read			= press_status_read,
	.llseek 			= press_status_llseek,
	.release			= press_status_release,
};

#define PRESS_STATUS_MAJOR 104 
#define PRESS_STATUS_NAME "press_status"
static int press_status_init(void)
{
	int ret;
       struct class *status_class;

	ret = register_chrdev(PRESS_STATUS_MAJOR, PRESS_STATUS_NAME, &press_status_fops);
	if(ret < 0)
	{
		pr_err("can't register major number\n");
		return ret;
	}

	status_class = class_create(THIS_MODULE, PRESS_STATUS_NAME);
	if(IS_ERR(status_class))
	{
		pr_err("failed in My_led class.\n");
		return -1;
	}

	device_create(status_class, NULL, MKDEV(PRESS_STATUS_MAJOR,0), NULL, PRESS_STATUS_NAME);
	
	return 0;
}
// ZTE_modify end

static int ndt_stress_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ndt_stress_board_data *bdata;
	
	NDT_INFO("NDT Driver Version: %s", NDT_DRIVER_VERSION);
	NDT_INFO("NDT Driver Built@%s, %s", __TIME__, __DATE__);
	NDT_INFO("NDT I2C Address: 0x%02x", client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		NDT_ERROR("I2C check functionality failed.");
		return -ENODEV;
	}

	gNdtData = kzalloc(sizeof(struct ndt_stress_data), GFP_KERNEL);
	if (!gNdtData) {
		NDT_ERROR("Failed to alloc mem for rmi4_data");
		return -ENOMEM;
	}

	gNdtData->client = client;
	
	bdata = &gNdtData->bdata;

	if(parse_dt(&client->dev, bdata))
		{
		NDT_ERROR("Failed to parse dt");
		goto err_parse_dt;
		}

	if(ndt_stress_set_gpio(gNdtData))
		{
		NDT_ERROR("Failed to set up GPIO's");
		goto err_set_gpio;
		}

	if(ndt_stress_jump_wait() == -1)
		{
		NDT_ERROR("Failed to jump wait");
		//goto err_set_gpio;
		}

	 msleep(50);

	 gNdtData->ndt_coord_handle = create_singlethread_workqueue("ndt_coord_handle");
	 INIT_DELAYED_WORK(&gNdtData->send_coord_work, ndt_update_send_coord_work);

#if (defined CONFIG_NDT_USE_DEBUG) || (defined CONFIG_NDT_USE_UPDATE)
	gNdtData->ndt_debug_queue = create_singlethread_workqueue("ndt_debug_workqueue");
#endif

#ifdef CONFIG_NDT_USE_UPDATE
	INIT_DELAYED_WORK(&gNdtData->update_work, ndt_update_handle_work);
#endif

#ifdef CONFIG_NDT_USE_TEST_I2C
	if(ndt_i2c_test())
		{
		NDT_ERROR("Failed to test ndt i2c");

		ndt_stress_power_switch(false, false);//  tp can't init when stress sensor firmware is bad and i2c is noisy. we must pown down

		ndt_force_firmware_update();//delay 10s force update firmware when filesystem have mounted
		}
#else
	gNdtData->appear = true;
#endif

	mutex_init(&(gNdtData->stress_value_mutex));

	mutex_init(&(gNdtData->i2c_mutex));

	//init_completion(&gNdtData->coord_thread_sync);

#ifdef CONFIG_NDT_USE_IRQ
	gNdtData->ndt_queue = create_singlethread_workqueue("ndt_irq_workqueue");
	
	INIT_DELAYED_WORK(&gNdtData->int_work, ndt_irq_handle_work);

	if(ndt_stress_request_irq(gNdtData))
		{
		NDT_ERROR("Failed to request irq");
		goto err_request_irq;
		}
#endif

#ifdef CONFIG_NDT_USE_DEBUG
	INIT_DELAYED_WORK(&gNdtData->debug_work, ndt_debug_handle_work);

	if(debug_proc_init())
		goto err_debug;
#endif

#ifdef CONFIG_NDT_USE_PV_TEST
	if(test_proc_init())
		goto err_pv_test;
#endif

      // ZTE_modify set custom gain proc command begin
      gNdtData->custom_gain_value = 100;
      custom_gain_proc_init();
      // ZTE_modify end
      // ZTE_modify add proc/driver/pressure_id begin
      pressure_proc_init();
      // ZTE_modify end
      // ZTE_modify get press status begin
      press_status_init();
      //ZTE_modify
#ifdef CONFIG_NDT_USE_CHAR_DEV
	dev_i2c_init();
#endif

#ifdef CONFIG_NDT_USE_UPDATE
	gNdtData->dmode = 0;
	queue_delayed_work(gNdtData->ndt_debug_queue, &gNdtData->update_work, msecs_to_jiffies(10000));
#endif

	return 0;

#ifdef CONFIG_NDT_USE_PV_TEST
err_pv_test:
	remove_proc_entry("driver/press_test", NULL);
#endif

#ifdef CONFIG_NDT_USE_DEBUG
err_debug:
	remove_proc_entry("driver/ndt_debug", NULL);
#endif

#ifdef CONFIG_NDT_USE_IRQ
err_request_irq:
#endif

#ifdef CONFIG_NDT_USE_TEST_I2C
//err_test_i2c:
#endif

err_set_gpio:
	ndt_stress_gpio_setup(bdata->irq_gpio, false, 0, 0);

	if (bdata->reset_gpio >= 0)
		ndt_stress_gpio_setup(bdata->reset_gpio, false, 0, 0);

	if (bdata->power_gpio >= 0)
		ndt_stress_gpio_setup(bdata->power_gpio, false, 0, 0);

err_parse_dt:
	kfree(gNdtData);
	gNdtData = NULL;

	return 0;
}

static int ndt_stress_remove(struct i2c_client *client)
{
	NDT_DEBUG_FUNC();

 	return 0;
}

static struct i2c_driver ndt_stress_driver = {
	.probe = ndt_stress_probe,
	.remove = ndt_stress_remove,
	.id_table = ndt_stress_id,
	.driver = {
		   .name = I2C_DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = ndt_match_table,
		   },
};

static int __init ndt_stress_init(void)
{
	NDT_DEBUG_FUNC();
	
	return i2c_add_driver(&ndt_stress_driver);
}

static void __exit ndt_stress_exit(void)
{
	NDT_DEBUG_FUNC();
	
	return i2c_del_driver(&ndt_stress_driver);
}

module_init(ndt_stress_init);
module_exit(ndt_stress_exit);

MODULE_AUTHOR("cc <cui.xiaochuan@zte.com.cn>");
MODULE_DESCRIPTION("NDT Stress Driver For P839A01");
MODULE_LICENSE("GPL");
