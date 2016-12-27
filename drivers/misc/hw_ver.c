/***********************************************************************
* Copyright (C) 2001, ZTE Corporation.
* 
* File Name:    hw_ver.c
* Description:  hw version process code
* Author:       liuzhongzhi
* Date:         2011-07-11
* 
**********************************************************************/
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/stat.h>
#include <linux/module.h> 

#define GPIO_VALUE_0    0
#define GPIO_VALUE_1    1
#define GPIO_VALUE_2    2
#define GPIO_VALUE_3    3
#define GPIO_VALUE_4    4
#define GPIO_VALUE_5    5
#define GPIO_VALUE_6    6
#define GPIO_VALUE_7    7

#define VERSION_STR_P839A01_1   "MBV1.0"
#define VERSION_STR_P839A01_2   "MBV2.0"
#define VERSION_STR_P839A01_3   "MBV3.0"
#define VERSION_STR_P839A01_4   "MBV4.0"
#define VERSION_STR_P839A01_5   "MBV5.0"
#define VERSION_STR_P839A01_6   "MBV6.0"
#define VERSION_STR_P839A01_7   "MBV7.0"
#define VERSION_STR_P839A01_8   "MBV8.0"

struct hw_ver_name
{
    int hw_ver;     /* hw version num */
    char *hw_name;  /* hw version name */
};

int hw_ver = -1;

static char *hw_version="noversion";


static struct hw_ver_name hw_ver_names[] = {
    {GPIO_VALUE_0,  VERSION_STR_P839A01_1},
    {GPIO_VALUE_1,  VERSION_STR_P839A01_2},
    {GPIO_VALUE_2,  VERSION_STR_P839A01_3},
    {GPIO_VALUE_3,  VERSION_STR_P839A01_4},
    {GPIO_VALUE_4,  VERSION_STR_P839A01_5},
    {GPIO_VALUE_5,  VERSION_STR_P839A01_6},
    {GPIO_VALUE_6,  VERSION_STR_P839A01_7},
    {GPIO_VALUE_7,  VERSION_STR_P839A01_8},
};

/*
 * Get the hw_ver value
 */
static int __init hw_ver_setup(char *str)
{
    char *after;

    printk(KERN_NOTICE "%s @str:%s\n", __func__, str);
    
    hw_ver = simple_strtoul(str, &after, 16);
    
	return 1;
}
__setup("hw_ver=", hw_ver_setup);

static int get_hw_version(void)
{
	int i = 0;
 
       /* find the hw version string according to hw_ver value */
	for (i = 0; i < sizeof(hw_ver_names)/sizeof(hw_ver_names[0]); i++) {
              /* not use board bits from hw_ver, use  flag_p897s11_or_p897s15 from qpnp-adc-voltage.c */
		if(hw_ver_names[i].hw_ver == ( 0x00FF & hw_ver) ){
                      hw_version = hw_ver_names[i].hw_name;
                      break;
		}
	}
    
	printk(KERN_NOTICE "@hw_version: %s\n",hw_version);
    
	return 0;
}

/*
 * show_hw_version() - Show public hw version to user space
 */
static ssize_t show_hw_version(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	return sprintf(buf, "%s\n", hw_version);
}

static struct device_attribute hw_version_attr =
	__ATTR(hw_version, S_IRUGO, show_hw_version, NULL);

static int  hw_ver_probe(struct platform_device *pdev)
{
    int ret;   

    get_hw_version();

    /* create attr file */
    ret = device_create_file(&pdev->dev, &hw_version_attr);
	if (ret) {
		dev_err(&pdev->dev, "failed: create hw_version file\n");
	}
    return 0;
}

static int hw_ver_remove(struct platform_device *pdev)
{
    device_remove_file(&pdev->dev, &hw_version_attr);	

	return 0;
}

static struct platform_driver hw_ver_driver = {
	.probe	= hw_ver_probe,
	.remove	= hw_ver_remove,
	.driver = {
		.name	= "hw_version",
		.owner	= THIS_MODULE,
	},
};

static struct platform_device hw_ver_device = {
	.name = "hw_version",
	.id = -1,
};

static int __init hw_ver_init(void)
{
    int ret;

    /* register device */
    ret = platform_device_register(&hw_ver_device); 
    if (ret){
        printk(KERN_NOTICE "Unable to register hw version Device!\n");
        goto out;
    }

    /* register driver */
    ret = platform_driver_register(&hw_ver_driver);
out:
	return ret;
}


static void __exit hw_ver_exit(void)
{
    platform_device_unregister(&hw_ver_device);
	platform_driver_unregister(&hw_ver_driver);
}

late_initcall(hw_ver_init);
__exitcall(hw_ver_exit);

