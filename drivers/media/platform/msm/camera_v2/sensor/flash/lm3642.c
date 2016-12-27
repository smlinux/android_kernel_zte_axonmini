/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
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
#include <linux/module.h>
#include <linux/export.h>
#include "msm_led_flash.h"

//#include <mach/gpiomux.h>
#include "msm_camera_io_util.h"
#include "../cci/msm_cci.h"

#define FLASH_NAME "qcom-flash,lm3642"

#define CONFIG_MSMB_CAMERA_DEBUG
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

static struct msm_led_flash_ctrl_t fctrl;
static struct i2c_driver lm3642_i2c_driver;


static struct msm_camera_i2c_reg_array lm3642_init_array[] = {
	//{0x00, 0x00},
};

static struct msm_camera_i2c_reg_array lm3642_off_array[] = {
	{0x01, 0xE0},
};

static struct msm_camera_i2c_reg_array lm3642_release_array[] = {
	{0x01, 0xE0},
};

static struct msm_camera_i2c_reg_array lm3642_low_array[] = {
    {0x01,  0xE2},
   // {0x04,  0x5f},
     {0x05,  0x7f},
     {0x07,  0x3f},
};

static struct msm_camera_i2c_reg_array lm3642_high_array[] = {
    //{0x08,  0x17},
    //{0x09,  0x3f},
    //{0x0A,  0x23},
    {0x01,  0xE3},
    {0x04,  0x5f},
     {0x05,  0x7f},
     {0x06,  0x3f},
    
};

static void __exit msm_flash_lm3642_i2c_remove(void)
{
	i2c_del_driver(&lm3642_i2c_driver);
	return;
}

static const struct of_device_id lm3642_trigger_dt_match[] = {
	{.compatible = "qcom-flash,lm3642", .data = &fctrl},
	{}
};

MODULE_DEVICE_TABLE(of, lm3642_trigger_dt_match);

static const struct i2c_device_id flash_i2c_id[] = {
	{"qcom-flash,lm3642", (kernel_ulong_t)&fctrl},
	{ }
};

static const struct i2c_device_id lm3642_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&fctrl},
	{ }
};

static int msm_flash_lm3642_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	if (!id) {
		pr_err("msm_flash_lm3642_i2c_probe: id is NULL");
		id = lm3642_i2c_id;
	}

	return msm_flash_i2c_probe(client, id);
}

static struct i2c_driver lm3642_i2c_driver = {
	.id_table = lm3642_i2c_id,
	.probe  = msm_flash_lm3642_i2c_probe,
	.remove = __exit_p(msm_flash_lm3642_i2c_remove),
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lm3642_trigger_dt_match,
	},
};

static int msm_flash_lm3642_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
CDBG(" msm_flash_lm3642_platform_probe");
	match = of_match_device(lm3642_trigger_dt_match, &pdev->dev);
	if (!match)
		return -EFAULT;
	return msm_flash_probe(pdev, match->data);
}

static struct platform_driver lm3642_platform_driver = {
	.probe = msm_flash_lm3642_platform_probe,
	.driver = {
		.name = "qcom-flash,lm3642",
		.owner = THIS_MODULE,
		.of_match_table = lm3642_trigger_dt_match,
	},
};

static int __init msm_flash_lm3642_init_module(void)
{
	int32_t rc = 0;

	rc = platform_driver_register(&lm3642_platform_driver);
	if (!rc)
		return rc;
	pr_debug("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&lm3642_i2c_driver);
}

static void __exit msm_flash_lm3642_exit_module(void)
{
	if (fctrl.pdev)
		platform_driver_unregister(&lm3642_platform_driver);
	else
		i2c_del_driver(&lm3642_i2c_driver);
}

static int msm_flash_lm3642_led_init(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0, ret = 0;
       uint16_t flag_reg = 0x0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG(" %s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	if (power_info->gpio_conf->cam_gpiomux_conf_tbl != NULL) {
		pr_err("%s:%d mux install\n", __func__, __LINE__);
		/* 
		///////duyuerong delete it because add #include <mach/gpiomux.h> handle ---error!
		msm_gpiomux_install(
			(struct msm_gpiomux_config *)
			power_info->gpio_conf->cam_gpiomux_conf_tbl,
			power_info->gpio_conf->cam_gpiomux_conf_tbl_size);
		*/
	}

	/* CCI Init */
	if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
			fctrl->flash_i2c_client, MSM_CCI_INIT);
		if (rc < 0)
			pr_err("cci_init failed\n");
	}

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}
	msleep(20);

	/*gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);*/
			gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);
		gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_HIGH);
//gpio_set_value_cansleep(55,GPIO_OUT_HIGH);
        ret = fctrl->flash_i2c_client->i2c_func_tbl->i2c_read(fctrl->flash_i2c_client, 
                0x01, &flag_reg , MSM_CAMERA_I2C_BYTE_DATA);
      
         if(ret < 0)
             pr_err("%s read flag register failed, rc = %d \n",__func__, rc);
         
         pr_err("%s read flag register = %d \n",__func__, flag_reg);
         pr_err("sid=0x%x",fctrl->flash_i2c_client->cci_client->sid );
   // gpio_set_value_cansleep(55,GPIO_OUT_LOW);
    
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_LOW);
	return rc;
}

static int msm_flash_lm3642_led_release(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	CDBG(" %s:%d called\n", __func__, __LINE__);
	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
    
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);
    
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);
		
		gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_LOW);

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

	/* CCI deInit */
	if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
			fctrl->flash_i2c_client, MSM_CCI_RELEASE);
		if (rc < 0)
			pr_err("cci_deinit failed\n");
	}

	return 0;
}

static int msm_flash_lm3642_led_off(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	CDBG(" %s:%d called\n", __func__, __LINE__);
	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
		/* CCI Init */
	if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
			fctrl->flash_i2c_client, MSM_CCI_INIT);
		if (rc < 0)
			pr_err("cci_init failed\n");
	}
    
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->off_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	//////add 20150807 
	if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
			fctrl->flash_i2c_client, MSM_CCI_RELEASE);
		if (rc < 0)
			pr_err("cci_deinit failed\n");
	}
	//////add end 

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);
    
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);
		
			gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_LOW);

	return rc;
}
/*zoupeng modify start*/
static struct msm_camera_i2c_reg_array lm3642_config_low_array[] = {
    {0x01,  0xE2},
     {0x05,  0x7f},
     {0x07,  0x3f},
};
static struct msm_camera_i2c_reg_setting lm3642_config_low_setting = {
	.reg_setting = lm3642_config_low_array,
	.size = ARRAY_SIZE(lm3642_config_low_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};
static void lm3642_buildConfigLowSetting(struct msm_camera_led_cfg_t *flash_data)
{
    switch(flash_data->torch_current[0])
    {
        case 1:
        {
            lm3642_config_low_setting.reg_setting[1].reg_data = 0x1f;//max:46.48ma
            lm3642_config_low_setting.reg_setting[2].reg_data = 0x0f;//led1:23.24ma  ((23.24-2.53)/1.46) +1 = 15.18
        }break;
        case 2:
        {
            lm3642_config_low_setting.reg_setting[1].reg_data = 0x2f;//max:69.91ma
            lm3642_config_low_setting.reg_setting[2].reg_data = 0x17;//led1:34.955ma ((34.955-2.53)/1.46) +1 = 23.209
        }break;
        case 3:
        {
            lm3642_config_low_setting.reg_setting[1].reg_data = 0x3f;//max:93.35ma
            lm3642_config_low_setting.reg_setting[2].reg_data = 0x1f;//led1:46.675ma ((46.675-2.53)/1.46) +1 = 31.236
        }break;
        case 4:
        {
            lm3642_config_low_setting.reg_setting[1].reg_data = 0x4f;//max:116.79ma
            lm3642_config_low_setting.reg_setting[2].reg_data = 0x27;//led1:58.395ma ((58.395-2.53)/1.46) +1 = 39.236
        }break;
        case 5:
        {
            lm3642_config_low_setting.reg_setting[1].reg_data = 0x5f;//max:140.23ma
            lm3642_config_low_setting.reg_setting[2].reg_data = 0x2f;//led1:70.115ma ((70.115-2.53)/1.46) +1 = 47.291
        }break;
        case 6:
        {
            lm3642_config_low_setting.reg_setting[1].reg_data = 0x6f;//max:163.66ma
            lm3642_config_low_setting.reg_setting[2].reg_data = 0x37;//led1:81.83ma ((81.83-2.53)/1.46) +1 = 55.315
        }break;
        case 7:
        {
            lm3642_config_low_setting.reg_setting[1].reg_data = 0x7f;//max:187.10ma
            lm3642_config_low_setting.reg_setting[2].reg_data = 0x3f;//led1:93.55ma ((34.955-2.53)/1.46) +1 = 63.342
        }break;
        default:
        {
            lm3642_config_low_setting.reg_setting[1].reg_data = lm3642_low_array[1].reg_data;
            lm3642_config_low_setting.reg_setting[2].reg_data = lm3642_low_array[2].reg_data;
        }break;
    }
    CDBG("zoupeng lm3642_buildConfigLowSetting,torch0:0x%x,torch1 = 0x%x\n",
        lm3642_config_low_setting.reg_setting[1].reg_data,
        lm3642_config_low_setting.reg_setting[2].reg_data);
}
/*zoupeng modify end*/
static int msm_flash_lm3642_led_low(struct msm_led_flash_ctrl_t *fctrl,
    struct msm_camera_led_cfg_t *flash_data)//zoupeng modify
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG(" %s:%d called zx!\n", __func__, __LINE__);
        if(NULL != flash_data)
        {
            CDBG("zoupeng lm3642_low,torch0:%d,torch1 = %d\n",flash_data->torch_current[0],flash_data->torch_current[1]);
        }
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);
		gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_HIGH);

        /*zoupeng modify start*/
        if (fctrl->flash_i2c_client && fctrl->reg_setting) 
        {
            if(NULL != flash_data)
            {
                lm3642_buildConfigLowSetting(flash_data);
                rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
                    fctrl->flash_i2c_client,
                    &(lm3642_config_low_setting));
            }else
            {
                rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
                    fctrl->flash_i2c_client,
                    fctrl->reg_setting->low_setting);
            }
            if (rc < 0)
            {
                pr_err("%s:%d failed\n", __func__, __LINE__);
            }
        }
        /*zoupeng modify end*/

	return rc;
}

static int msm_flash_lm3642_led_high(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG(" %s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);
		gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_HIGH);
		pr_err("HWEN =%d",power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET]);
    
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->high_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}

static struct msm_camera_i2c_client lm3642_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static struct msm_camera_i2c_reg_setting lm3642_init_setting = {
	.reg_setting = lm3642_init_array,
	.size = ARRAY_SIZE(lm3642_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_off_setting = {
	.reg_setting = lm3642_off_array,
	.size = ARRAY_SIZE(lm3642_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};


static struct msm_camera_i2c_reg_setting lm3642_release_setting = {
	.reg_setting = lm3642_release_array,
	.size = ARRAY_SIZE(lm3642_release_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_low_setting = {
	.reg_setting = lm3642_low_array,
	.size = ARRAY_SIZE(lm3642_low_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_high_setting = {
	.reg_setting = lm3642_high_array,
	.size = ARRAY_SIZE(lm3642_high_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_led_flash_reg_t lm3642_regs = {
	.init_setting = &lm3642_init_setting,
	.off_setting = &lm3642_off_setting,
	.low_setting = &lm3642_low_setting,
	.high_setting = &lm3642_high_setting,
	.release_setting = &lm3642_release_setting,
};

static struct msm_flash_fn_t lm3642_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = msm_flash_lm3642_led_init,
	.flash_led_release = msm_flash_lm3642_led_release,
	.flash_led_off = msm_flash_lm3642_led_off,
	.flash_led_low = msm_flash_lm3642_led_low,
	.flash_led_high = msm_flash_lm3642_led_high,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.flash_i2c_client = &lm3642_i2c_client,
	.reg_setting = &lm3642_regs,
	.func_tbl = &lm3642_func_tbl,
};

/*subsys_initcall(msm_flash_i2c_add_driver);*/
module_init(msm_flash_lm3642_init_module);
module_exit(msm_flash_lm3642_exit_module);
MODULE_DESCRIPTION("lm3642 FLASH");
MODULE_LICENSE("GPL v2");
