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
#include <linux/ctype.h>
#include <linux/printk.h>
#include <linux/leds.h>
#include "lpc5400x_leds.h"
#include <linux/workqueue.h>
#include "hostif_protocol.h"

#include <linux/lpcsh/lpc5400x.h>

static int log_flag = 1;

#define pr_leds(mask,args...) \
	do { \
		   if(mask <= log_flag) \
			printk("LED_DEBUG_ZJ: lpc5400x "args); \
	} while (0)

#define lpc5400x_breathled "lpc5400x_leds"


int lpc5400x_send_buf(struct i2c_client *client, u8 *buffer, int length);
int lpc5400x_read_buf(struct i2c_client *client, u8 cmd, u8 *buffer, int length);

struct lpc5400x_bright_led{
        struct mutex						led_lock ;
        struct led_classdev  					led;
	 struct work_struct  					control_work;
	 struct i2c_client                                      *client;
	 struct lpcsh_breathing_led_command_t     led_command;
	 int 						breath_set;
	 int						blink_set;
	 int 						brightness;
	 int						on_light;
	 int 						t0;
	 int 						t1;
	 int 						t2;
	 int 						t3;
};

static struct lpc5400x_bright_led *breath_led=NULL;

static int device_addzj_attributes(struct device *dev,
				 struct device_attribute *attrs)
{
	int error = 0;
	int i;

	if (attrs) {
		for (i = 0; attr_name(attrs[i]); i++) {
			error = device_create_file(dev, &attrs[i]);
			if (error)
				break;
		}
		if (error)
			while (--i >= 0)
				device_remove_file(dev, &attrs[i]);
	}
	return error;
}

static void device_removezj_attributes(struct device *dev,
				     struct device_attribute *attrs)
{
	int i;

	if (attrs)
		for (i = 0; attr_name(attrs[i]); i++)
			device_remove_file(dev, &attrs[i]);
}

static void Breath_led_control(struct work_struct *work)
{
//  struct lpc5400x_bright_led *breath_led =
//  	container_of(work, struct lpc5400x_bright_led, control_work);
  int ret =0;

    pr_leds(2,"enter led control\n");

	breath_led->led_command.t0 = breath_led->t0;
	breath_led->led_command.t1 = breath_led->t1;
	breath_led->led_command.t2 = breath_led->t2;
    	breath_led->led_command.t3 = breath_led->t3;
    	breath_led->led_command.brightness = breath_led->brightness;
	if((breath_led->led_command.t0==0)
			&&(breath_led->led_command.t1==0)
			&&(breath_led->led_command.t2==0)
			&&(breath_led->led_command.t3==0))
		{
			breath_led->led_command.channelMask = 0;
			breath_led->led_command.brightness = 0;
		}
    pr_leds(1,"the command:0x%x brigtness: %d, the mask:%d the t0/t1/t2/t3:%d /%d /%d /%d\n",
				breath_led->led_command.command,
				breath_led->led_command.brightness,
				breath_led->led_command.channelMask,
				breath_led->led_command.t0,
				breath_led->led_command.t1,
				breath_led->led_command.t2,
				breath_led->led_command.t3
				);
	
  //  mutex_lock(&breath_led->led_lock);
    ret = lpc5400x_send_buf(breath_led->client, (u8 *) &breath_led->led_command, sizeof(breath_led->led_command));
    if (ret < 0) {
			pr_leds(0,"Failed to set leds, Bus error\n");
		}
  //  mutex_unlock(&breath_led->led_lock );
    pr_leds(2,"control out\n");
	
}


static ssize_t show_breath_led(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
//	struct lpc5400x_bright_led *breath_led;
	//struct led_classdev *led = dev_get_drvdata(dev);
//	 breath_led = container_of(led, struct lpc5400x_bright_led, led);
	 
	 sprintf(buf, "%d\n",breath_led->led_command.channelMask);
	  ret = strlen(buf) + 1;
        return ret;

}


static ssize_t store_breath_led(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
       int value = 0;
	char *endp;
	ssize_t ret = 0;
	//struct lpc5400x_bright_led *breath_led;
	//struct led_classdev *led = dev_get_drvdata(dev);

	pr_leds(2," enter store_breath_led\n");
//	 breath_led = container_of(led, struct lpc5400x_bright_led, led);
	 
	  value = simple_strtoul(buf, &endp, 0);
		pr_leds(2," enter store_breath_led value:%d\n",value);
          if(value == 0)
	       {
		breath_led->led_command.channelMask = 0;
		breath_led->led_command.brightness     =0;
		   }
	   else{
		   	breath_led->led_command.channelMask = 1;
			breath_led->led_command.brightness = breath_led->brightness;
		   	breath_led->led_command.t0 = breath_led->t0;
			breath_led->led_command.t1 = breath_led->t1;
			breath_led->led_command.t2 = breath_led->t2;
			breath_led->led_command.t3 = breath_led->t3;
		   }
	   
	   	if((breath_led->led_command.t0==0)
			&&(breath_led->led_command.t1==0)
			&&(breath_led->led_command.t2==0)
			&&(breath_led->led_command.t3==0))
		{
			breath_led->led_command.channelMask = 0;
			breath_led->led_command.brightness = 0;
		}

		pr_leds(2," will enter breath_led control_work\n");
		//schedule_work(&breath_led->control_work);
	    pr_leds(1,"the command:0x%x brigtness: %d, the mask:%d the t0/t1/t2/t3:%d /%d /%d /%d\n",
				breath_led->led_command.command,
				breath_led->led_command.brightness,
				breath_led->led_command.channelMask,
				breath_led->led_command.t0,
				breath_led->led_command.t1,
				breath_led->led_command.t2,
				breath_led->led_command.t3
				);
//	    mutex_lock(&breath_led->led_lock);
   		 ret = lpc5400x_send_buf(breath_led->client, (u8 *) &breath_led->led_command, sizeof(breath_led->led_command));
	    if (ret < 0) {
				pr_leds(0,"Failed to set leds, Bus error\n");
			}
//	    mutex_unlock(&breath_led->led_lock );
             pr_leds(2,"store out\n");
        return count;

}

static ssize_t show_breath_time(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
		
	ret = sprintf(buf, "%d  %d  %d  %d\n", 
				breath_led->t0,breath_led->t1,breath_led->t2,breath_led->t3);
	ret = strlen(buf) + 1;
	return ret;

}


static ssize_t store_breath_time(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
       int lenth=0;
	char *endp;
	int step=0;
	if(buf == NULL)
		return count;

	if(endp == NULL)
	{
		pr_leds(2,"endp NULL");
		return count;
		}
	while((*buf)!='\0')
	{
		if((*buf)!=' ')
		{
			if(lenth==0)
			{
				step++;
				pr_leds(2,"test step :%d\n",step);
				if(step==1)
					breath_led->t0=simple_strtoul(buf,&endp,10);
				else if(step==2)
					breath_led->t1=simple_strtoul(buf,&endp,10);
				else if(step==3)
					breath_led->t2=simple_strtoul(buf,&endp,10);
				else if(step==4)
					{
					   breath_led->t3=simple_strtoul(buf,&endp,10);
					   pr_leds(2," for 4 parameter get success\n");
					   break;
					}
				else if(step > 4)
					{
					   pr_leds(2," over the get range\n ");
					   break;
					}
			}
			lenth++;
		}else
		{
			lenth=0;
		}
		pr_leds(2,"lenth:%d,endp:%s,buf:%s\n",lenth,endp,buf);
		buf++;
	}
	return count;
}

static ssize_t show_breath_led_set(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
		
	ret = sprintf(buf, "%d\n", 
				breath_led->breath_set);
	ret = strlen(buf) + 1;
	return ret;

}


static ssize_t store_breath_led_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	char *endp;

	int value = 0;

	value = simple_strtoul(buf, &endp, 0);
	pr_leds(2,"store breath_set %d\n",value);

	breath_led->breath_set = value*255;
	if(breath_led->breath_set <= 0)
		{	
			breath_led->breath_set = 0;
			return count;
		}
	breath_led->t0 = breath_led->breath_set;
	breath_led->t1 = 100;
	breath_led->t2 = breath_led->breath_set;
	breath_led->t3 = 100;
	
	return count;

}

static ssize_t show_blink_led_set(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
		
	ret = sprintf(buf, "%d\n", 
				breath_led->blink_set);
	ret = strlen(buf) + 1;
	return ret;

}


static ssize_t store_blink_led_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	char *endp;

	int value = 0;

	value = simple_strtoul(buf, &endp, 0);
	pr_leds(2,"store breath_set %d\n",value);

	breath_led->blink_set = value*255;

	if(breath_led->blink_set <= 0)
		{	
			breath_led->blink_set = 0;
			return count;
		}
	breath_led->t0 = 0;
	breath_led->t1 = breath_led->blink_set;
	breath_led->t2 = 0;
	breath_led->t3 = breath_led->blink_set;
	
	return count;

}

static ssize_t show_on_light_set(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
		
	ret = sprintf(buf, "%d\n", 
				breath_led->on_light);
	ret = strlen(buf) + 1;
	return ret;

}


static ssize_t store_on_light_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	char *endp;

	int value = 0;

	value = simple_strtoul(buf, &endp, 0);
	pr_leds(2,"store breath_set %d\n",value);

	breath_led->on_light = value*255;

	if(breath_led->on_light <= 0)
		{	
			breath_led->on_light = 0;
			return count;
		}
	breath_led->t0 = 0;
	breath_led->t1 = breath_led->on_light;
	breath_led->t2 = 0;
	breath_led->t3 = 0;
	
	return count;

}
struct device_attribute attrs[] = {
	__ATTR(mask_breathled,S_IRWXU|S_IRGRP|S_IROTH, show_breath_led, store_breath_led),
	__ATTR(t0_1_2_3_set,S_IRWXU|S_IRGRP|S_IROTH, show_breath_time, store_breath_time),	
	__ATTR(breath_led_set,S_IRWXU|S_IRGRP|S_IROTH, show_breath_led_set, store_breath_led_set),
	__ATTR(blink_led_set,S_IRWXU|S_IRGRP|S_IROTH, show_blink_led_set, store_blink_led_set),
	__ATTR(on_light_set,S_IRWXU|S_IRGRP|S_IROTH, show_on_light_set, store_on_light_set),
	__ATTR_NULL,
};

static void lpc5400x_set_breathled_fun(struct led_classdev *led,
                                      enum led_brightness value)
{
 //    struct lpc5400x_bright_led *breath_led;
 //    breath_led = container_of(led, struct lpc5400x_bright_led, led);
	 pr_leds(1," enter lpc5400x_set_breathled_fun\n");
 	if(value < 0)
 		{
		value =0;
 	  	pr_leds(0,"get  value error\n ");
 		}
         if(value == 0)
	       {
			breath_led->led_command.channelMask = 0;
			breath_led->brightness = 0;
		   }
	   else{
	   		breath_led->led_command.channelMask = 1;
			breath_led->brightness = value;
		   }
	schedule_work(&breath_led->control_work);
	 	
}

void lpc5400x_breath_led_init(void)
{
	pr_leds(1,"enter led init\n");
/*********************************
    uint8_t  command;				enum LPCSH_BREATHING_LED  
    uint8_t  brightness;                            0x00(off) to 0xff(full)
    uint8_t  channelMask;                     0 to stop breathing LED, 1-7 to enable breathing LED on channel 0/1/2 
    uint16_t t0;                                    ramp-up time in ms 
    uint16_t t1;                                    on time in ms 
    uint16_t t2;                                    ramp-down time in ms 
    uint16_t t3;                                    off time in ms 
***********************************/
	breath_led->led_command.command = LPCSH_BREATHING_LED;
	breath_led->led_command.channelMask = 0;
	breath_led->led_command.brightness = 0x00;
	breath_led->led_command.t0 = 10;
	breath_led->led_command.t1 = 1000;
	breath_led->led_command.t2 = 10;
	breath_led->led_command.t3 = 1000;
	breath_led->t0 = 10;
	breath_led->t1 = 1000;
	breath_led->t2 = 10;
	breath_led->t3 = 1000;
	
}

void led_class_init(void)
{
	breath_led->led.name= lpc5400x_breathled;
	breath_led->led.brightness_set = lpc5400x_set_breathled_fun;
	breath_led->led.brightness = LED_OFF;

}

int lpc5400x_create_leds_device(void **pHandle, struct i2c_client *client)
{
  //   struct lpc5400x_bright_led *breath_led =
//		kzalloc(sizeof(struct lpc5400x_bright_led), GFP_KERNEL);
	int rc = 0;
	*pHandle = NULL;
	breath_led =
		kzalloc(sizeof(struct lpc5400x_bright_led), GFP_KERNEL);

	
	pr_leds(1,"enter create  device\n");
	if(!breath_led){
		rc = -ENOMEM;
		printk(KERN_ERR"lpc5400x leds kzalloc failed\n");
		goto  kzalloc_failed;
		}
//	mutex_init(&breath_led->led_lock);
	breath_led->client = client;
	
	lpc5400x_breath_led_init();
	led_class_init();
	INIT_WORK(&breath_led->control_work, Breath_led_control);
	rc= led_classdev_register(&client -> dev, &breath_led->led);
	if (rc) {
			printk(KERN_ERR
			       "breath_led: led_classdev_register failed\n");
			goto err_led_classdev_register_failed;
		}
	
	rc =  device_addzj_attributes(breath_led->led.dev, attrs);
	if (rc) {
			printk(KERN_ERR
			       "breath_led: create dev_attr_breathled failed\n");
			goto err_out_attr;
		}
	
//	dev_set_drvdata(&client->dev, breath_led);
	*pHandle = breath_led;
	pr_leds(1,"breath leds init ok\n");
	
	//schedule_work(&breath_led->control_work);

	return 0;

	
err_out_attr:
	//device_removezj_attributes(breath_led->led.dev, &attrs);
err_led_classdev_register_failed:
	led_classdev_unregister(&breath_led ->led);
//	mutex_destroy(&breath_led->led_lock);
kzalloc_failed:
	
	kfree(breath_led);
	
	*pHandle = NULL;
	pr_leds(0,"breath leds init failed!!\n");
	return rc;
	
}

int lpc5400x_destory_leds_device(void *handle)
{
	struct lpc5400x_bright_led *breath_leds = handle;

    	device_removezj_attributes(breath_leds->led.dev, attrs);
	led_classdev_unregister(&breath_leds ->led);
//	mutex_destroy(&breath_leds->led_lock);

	kfree(breath_leds);
	return 0;
}

