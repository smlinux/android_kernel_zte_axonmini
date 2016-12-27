#include <linux/slab.h>
#define DEBUG					/* enable dev_dbg */
#include <linux/device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/gpio.h>
//#include <linux/earlysuspend.h>
#include <linux/suspend.h>
#include <linux/kfifo.h>
#include <linux/notifier.h>
#include <linux/mutex.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/time.h>
#include <linux/of_gpio.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include "sensor_core.h"

#ifdef CONFIG_LPC5400X_SENSORHUB_VOICE_TRIGGER
#include "lpc5400x_voice_trigger.h"
#endif

#ifdef CONFIG_LPC5400X_SENSORHUB_IR_BLASTER
#include "lpc5400x_ir_blaster.h"
#endif

#ifdef CONFIG_LPC5400X_SENSORHUB_BREATH_LED
#include "lpc5400x_leds.h"
#endif

#ifdef CONFIG_LPC5400X_SENSORHUB_FIRMWARE_UPDATE_I2C
#include "lpc5400x_update.h"
#endif

#ifdef CONFIG_LPC5400X_SENSORHUB_FIRMWARE_UPDATE_I2C
#include "lpc5400x_vfs.h"
#endif
#include <linux/workqueue.h>
#include <linux/alarmtimer.h>
#include <linux/time.h>

#ifdef CONFIG_LPC5400X_SENSORHUB_LOGGING_SERVICE
#include "lpc5400x_vfs_logging.h"
#endif

/* dragon board 8074 - SNS_TEST1 */

#ifdef CONFIG_LPC5400X_SENSORHUB_FIRMWARE_UPDATE_I2C
#define I2C_WHO_AM_I_ID 0x54
#endif

#define SENSOR_WAKELOCK_TIMEOUT (200)
#define SENSORHUB_RESUME_TIME (300)

/* try to sync sensorhub timestamp to linux time every 5 minutes */
#define LPC_TS_SYNC_NS (5 * 60 * 1000000000LL)

/* Scale factor (number of shifts) */
#define LPC_TS_Q       6

/* wangjianping add for MCU id proc file */
#define NXP_MCU_ZTE_PROC_FLAG

#ifdef NXP_MCU_ZTE_PROC_FLAG
struct proc_dir_entry *mcu_id_entry = NULL;
uint8_t nxp_fw_version_id[2];
#endif

/*
   Meatured in Logical analyzer, the latency between
   time 0 (command sends from AP -> MCU) and
   time 1 (Sensorhub execute the command) in nanosecond
 */
#define LPC5400X_CMD_RESPONSE_LATENCY (100 * 1000)
struct _lpc5400x {
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	struct notifier_block pm_nb;
	u8 i2c_databuf[HOSTIF_MAX_BUFFER_SIZE];
	void *sensorHandle[LPCSH_SENSOR_ID_COUNT];
	struct i2c_client *client;
	enum rpm_status pm_state;	/* dev->pm_state */

#ifdef CONFIG_LPC5400X_SENSORHUB_FIRMWARE_UPDATE_I2C
	bool isValidVersion;
	bool isFirmwareChecked;
#endif
#ifdef CONFIG_LPC5400X_SENSORHUB_VOICE_TRIGGER
	void *vt;
#endif

	void *vfs;
	struct vfs_callback_t vfscallback;

#ifdef CONFIG_LPC5400X_SENSORHUB_IR_BLASTER
	void *ir;
#endif
#ifdef CONFIG_LPC5400X_SENSORHUB_BREATH_LED
	void* breath_led;
#endif
	struct lpc5400x_platform_data *pdata;

	struct ShCmdGetHeader_get_8bits_param_t devId;
	struct lpcsh_version_t devVersion;


	/* prevents any user command to be mixed with ISR thread */
	struct mutex sensorhub_mutex;

	/*for use ISR thread when  the high freq irq coming*/
	struct mutex sensorhub_irqmutex;

	/* Hold a timeout wake lock for wake up sensor */
	struct wake_lock wakelk;

	/* Hold a wake lock for irq handle to resume func */
	struct wake_lock resumelock;
	/* last moment of timesync in local sensorhub time */
	uint64_t timesync_lpc_sync_time;

	/* second last moment of timesync in local sensorhub time */
	uint64_t timesync_lpc_sync_time_prev;

	/* last moment of timesync in linux time */
	uint64_t timesync_linux_sync_time;

	/* second last moment of timesync in linux time */
	uint64_t timesync_linux_sync_time_prev;

	/* multiplier (scaled ratio between linux time and sensorhub time)
	    calculated during last timesync */
	uint32_t timesync_mult;

	/* multiplier (scaled ratio between linux time and sensorhub time)
	    calculated during second last timesync */
	uint32_t timesync_mult_prev;

	/* counter for synchronized time */
	uint64_t timesync_calibrated_lpc_time;

	/* counter for synchronized time from before the last sync */
	uint64_t timesync_calibrated_lpc_time_prev;

    /* occur interrupt after system suspend, need clear after resume */
	bool interrupt_pending;
};

/** @brief Environmental sensor special calibration record.

*

* An environmental sensor may need a set of calibration parameters specific to
* that make/model of sensor. Specific sensor drivers use these parameters to tune
* the operation of the sensor. Since each sensors may have different parameters
* there is no single structure to hold these parameters. However, a common header
* is used for each record. This structure defines the common header portion of
* the record. The actual record consists of the header followed by one or more
* 32-bit words or parameters. This typedef provides a common type that can be
* used in generic function calls.

*/
#if 0
typedef struct envHubSpecCalRecord_s {
    uint32_t sensorGuid;                     /**< Sensor GUID. */
    uint32_t paramLength;                    /**< Bits 7:0 - length in words of parameters section. Bits 31:8 - reserved */
} envHubSpecCalRecord_t;
#endif

packed_struct leStk3311PsSpecCal_s {
    //envHubSpecCalRecord_t header;           /**< Special environmental calibration record header. */
    uint16_t              hiGain1;          /**< Region 1 high threshold gain. Q4 */
    uint16_t              hiGain2;          /**< Region 2 high threshold gain. Q4 */
    uint16_t              hiGain3;          /**< Region 3 high threshold gain. Q4 */
    uint16_t              loGain1;          /**< Region 1 low threshold gain.  Q4 */
    uint16_t              loGain2;          /**< Region 2 low threshold gain.  Q4 */
    uint16_t              loGain3;          /**< Region 3 low threshold gain.  Q4 */
    uint16_t              maxThresholdHi;   /**< Maximum high interrupt threshold to use. ADCs. */
    uint16_t              maxThresholdLo;   /**< Maximum low interrupt threshold to use.  ADCs. */
    uint16_t              minThresholdHi;   /**< Minimum high interrupt threshold to use. ADCs. */
    uint16_t              minThresholdLo;   /**< Minimum high interrupt threshold to use. ADCs. */
    uint16_t              calParam1;        /**< Boundary between region 1 and region 2.  ADCs */
    uint16_t              calParam2;        /**< Boundary between region 2 and region 3.  ADCs */
    uint16_t              deviate;          /**< Maximum deviation of current output from previous crosstalk allowed in order to run calibration. ADCs */
    uint16_t              crosstalk;        /**< Measured crosstalk from a successful calibration. ADCs */
    uint8_t               ledDutyCycle;     /**< The value to use in the DT_LED field of register 0x03. */
    uint8_t               ledCurrent;       /**< The value to use in the IRDR field of register 0x03 */
    uint8_t               integrationTime;  /**< The value to use in the IT_PS field of register 0x01 */
    uint8_t               gain;             /**< The value to use in the GAIN_PS field of register 0x01 */
    uint8_t               outputSelection;  /**< The output selection. 0 - centimeters, 1 - ADCs */
    uint8_t               reserved1;        /**< Reserved */
    uint8_t               reserved2;        /**< Reserved */
    uint8_t               reserved3;        /**< Reserved */
} leStk3311PsSpecCal_t;


typedef struct Sensor_CalibrationData_s{
	struct leStk3311PsSpecCal_s stk3311;
	
    uint8_t writeflash;
}
Sensor_CalibrationData_t;


struct lpcsh_get_sensor_list_t             sensorlist_info; //zhangji
bool                                       get_sensor_list_ok;//zhangji

struct sensor_config sensorList[] = {
	{
		ACCEL_DATA_NAME,
		LPCSH_SENSOR_ID_ACCELEROMETER,
		sizeof(struct lpcsh_motion_sensor_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		UNCALIBRATED_ACCEL_DATA_NAME,
		LPCSH_SENSOR_ID_ACCELEROMETER_UNCALIBRATED,
		sizeof(struct lpcsh_motion_uncal_sensor_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		MAGNETIC_DATA_NAME,
		LPCSH_SENSOR_ID_MAGNETIC_FIELD,
		sizeof(struct lpcsh_motion_sensor_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		UNCAL_MAGNETIC_DATA_NAME,
		LPCSH_SENSOR_ID_MAGNETIC_FIELD_UNCALIBRATED,
		sizeof(struct lpcsh_motion_uncal_sensor_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		ORIENTATION_DATA_NAME,
		LPCSH_SENSOR_ID_ORIENTATION,
		sizeof(struct lpcsh_quaternion_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		GYROSCOPE_DATA_NAME,
		LPCSH_SENSOR_ID_GYROSCOPE,
		sizeof(struct lpcsh_motion_sensor_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		UNCAL_GYROSCOPE_DATA_NAME,
		LPCSH_SENSOR_ID_GYROSCOPE_UNCALIBRATED,
		sizeof(struct lpcsh_motion_uncal_sensor_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		ROTATIONVECTOR_DATA_NAME,
		LPCSH_SENSOR_ID_ROTATION_VECTOR,
		sizeof(struct lpcsh_quaternion_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		GAME_ROTATIONVECTOR_DATA_NAME,
		LPCSH_SENSOR_ID_GAME_ROTATION_VECTOR,
		sizeof(struct lpcsh_quaternion_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		GEO_MAGNETIC_ROTATION_VECTOR_DATA_NAME,
		LPCSH_SENSOR_ID_GEOMAGNETIC_ROTATION_VECTOR,
		sizeof(struct lpcsh_quaternion_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		LIGHT_DATA_NAME,
		LPCSH_SENSOR_ID_LIGHT,
		sizeof(struct lpcsh_light_sensor_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		PROXIMITY_DATA_NAME,
		LPCSH_SENSOR_ID_PROXIMITY,
		sizeof(struct lpcsh_proximity_sensor_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		PRESSURE_DATA_NAME,
		LPCSH_SENSOR_ID_PRESSURE,
		sizeof(struct lpcsh_pressure_sensor_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		AMBIENT_TEMPERATURE_DATA_NAME,
		LPCSH_SENSOR_ID_AMBIENT_TEMPERATURE,
		sizeof(struct lpcsh_ambient_temperature_sensor_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		RELATIVE_HUMIDITY_DATA_NAME,
		LPCSH_SENSOR_ID_RELATIVE_HUMIDITY,
		sizeof(struct lpcsh_humidity_sensor_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		GRAVITY_DATA_NAME,
		LPCSH_SENSOR_ID_GRAVITY,
		sizeof(struct lpcsh_motion_sensor_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		LINEAR_ACCELERATION_DATA_NAME,
		LPCSH_SENSOR_ID_LINEAR_ACCELERATION,
		sizeof(struct lpcsh_motion_sensor_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		STEP_DETECTOR_NAME,
		LPCSH_SENSOR_ID_STEP_DETECTOR,
		sizeof(struct lpcsh_step_sensor_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		STEP_COUNTER_NAME,
		LPCSH_SENSOR_ID_STEP_COUNTER,
		sizeof(struct lpcsh_step_sensor_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		SIGNIFICANT_MOTION_DATA_NAME,
		LPCSH_SENSOR_ID_SIGNIFICANT_MOTION,
		sizeof(struct lpcsh_significant_motion_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		ACTIVITY_CLASSIFIER_UNKNOWN_DATA_NAME,
		LPCSH_SENSOR_ID_ACTIVITY_CLASSIFIER_UNKNOWN,
		sizeof(struct lpcsh_activity_classifier_probability_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		ACTIVITY_CLASSIFIER_IN_VEHICLE_DATA_NAME,
		LPCSH_SENSOR_ID_ACTIVITY_CLASSIFIER_IN_VEHICLE,
		sizeof(struct lpcsh_activity_classifier_probability_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		ACTIVITY_CLASSIFIER_ON_BICYCLE_DATA_NAME,
		LPCSH_SENSOR_ID_ACTIVITY_CLASSIFIER_ON_BICYCLE,
		sizeof(struct lpcsh_activity_classifier_probability_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		ACTIVITY_CLASSIFIER_ON_FOOT_DATA_NAME,
		LPCSH_SENSOR_ID_ACTIVITY_CLASSIFIER_ON_FOOT,
		sizeof(struct lpcsh_activity_classifier_probability_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		ACTIVITY_CLASSIFIER_STILL_DATA_NAME,
		LPCSH_SENSOR_ID_ACTIVITY_CLASSIFIER_STILL,
		sizeof(struct lpcsh_activity_classifier_probability_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		ACTIVITY_CLASSIFIER_TILTING_DATA_NAME,
		LPCSH_SENSOR_ID_ACTIVITY_CLASSIFIER_TILTING,
		sizeof(struct lpcsh_activity_classifier_probability_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		ACTIVITY_CLASSIFIER_WALKING_DATA_NAME,
		LPCSH_SENSOR_ID_ACTIVITY_CLASSIFIER_WALKING,
		sizeof(struct lpcsh_activity_classifier_probability_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		ACTIVITY_CLASSIFIER_RUNNING_DATA_NAME,
		LPCSH_SENSOR_ID_ACTIVITY_CLASSIFIER_RUNNING,
		sizeof(struct lpcsh_activity_classifier_probability_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		ACTIVITY_CLASSIFIER_ON_STAIRS_DATA_NAME,
		LPCSH_SENSOR_ID_ACTIVITY_CLASSIFIER_ON_STAIRS,
		sizeof(struct lpcsh_activity_classifier_probability_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		TAP_DETECTOR_DATA_NAME,
		LPCSH_SENSOR_ID_TAP_DETECTOR,
		sizeof(struct lpcsh_tap_detector_sensor_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		SHAKE_DETECTOR_DATA_NAME,
		LPCSH_SENSOR_ID_SHAKE_DETECTOR,
		sizeof(struct lpcsh_shake_detector_sensor_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		FLIP_DETECTOR_DATA_NAME,
		LPCSH_SENSOR_ID_FLIP_DETECTOR,
		sizeof(struct lpcsh_flip_detector_sensor_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		PICKUP_DETECTOR_DATA_NAME,
		LPCSH_SENSOR_ID_PICKUP_DETECTOR,
		sizeof(struct lpcsh_pickup_detector_sensor_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		STABILITY_DETECTOR_DATA_NAME,
		LPCSH_SENSOR_ID_STABILITY_DETECTOR,
		sizeof(struct lpcsh_stability_detector_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		SLEEP_DETECTOR_DATA_NAME,
		LPCSH_SENSOR_ID_SLEEP_DETECTOR,
		sizeof(struct lpcsh_sleep_detector_sensor_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		TILT_DETECTOR_DATA_NAME,
		LPCSH_SENSOR_ID_TILT_DETECTOR,
		sizeof(struct lpcsh_tilt_detector_sensor_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		IN_POCKET_DETECTOR_DATA_NAME,
		LPCSH_SENSOR_ID_IN_POCKET_DETECTOR,
		sizeof(struct lpcsh_tilt_detector_sensor_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
	{
		STABILITY_CLASSIFIER_DATA_NAME,
		LPCSH_SENSOR_ID_STABILITY_CLASSIFIER,
		sizeof(struct lpcsh_stability_classifier_sensor_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},

#ifdef CONFIG_LPC5400X_SENSORHUB_VOICE_TRIGGER
	{
		VOICE_TRIGGER_DATA_NAME,
		LPCSH_SENSOR_ID_VOICE_TRIGGER,
		sizeof(struct lpcsh_voice_trigger_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},

#endif
#ifdef CONFIG_LPC5400X_SENSORHUB_AUDIO_SENSE
	{
		AUDIO_SENSE_DATA_NAME,
		LPCSH_SENSOR_ID_AUDIOSENSE,
		sizeof(struct lpcsh_audiosense_node) +
		sizeof(struct lpcsh_sensor_node_header)
	},
#endif
};


static uint32_t lpc5400x_init_timeSync(struct _lpc5400x *lpc);

static irqreturn_t lpc5400x_irq_handler(int irq, void *userdata);
static irqreturn_t lpc5400x_readdata(int irq, void *userdata);
static void lpc5400x_ps_calibration_powerup(struct i2c_client *client);
static void nxp_mcu_reset(struct _lpc5400x *lpc);
static int reportdata(struct _lpc5400x *lpc,	u8 *pdata,int readsize,int totalSizeRead, bool *apply_wakelock);


int lpc5400x_read_buf_unlock(struct i2c_client *client, u8 cmd, u8 *buffer, int length)
{
	/*
	 * Annoying we can't make this const because the i2c layer doesn't
	 * declare input buffers const.
	 */
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &cmd,
		}
		,
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = buffer,
		},
	};

#if 0
	dev_dbg(&client->dev,
		"lpc5400x_read_buf, CMD = %d length = %d\n",
		cmd, length);
#endif

    //printk("lpc5400x_read_buf cmd : %d\n", cmd);

	return i2c_transfer(client->adapter, msg, 2);
}

int lpc5400x_read_buf2(struct i2c_client *client, u8* cmdbuffer, u8 cmdlength, u8 *buffer, int bufferlength) {
	/*
	 * Annoying we can't make this const because the i2c layer doesn't
	 * declare input buffers const.
	 */
	struct i2c_msg msg[] = {
			{
					.addr = client->addr,
					.flags = 0,
					.len = cmdlength,
					.buf = cmdbuffer,
			}
			,
			{
					.addr = client->addr,
					.flags = I2C_M_RD,
					.len = bufferlength,
					.buf = buffer,
			},
	};
	return i2c_transfer(client->adapter, msg, 2);
}

/**
 *	spihub_send_buf	-	Writes the sensor hub packet
 *
 *	Writes the register values in one transaction or returns a negative
 *	error code on failure.
 */

int lpc5400x_send_buf_unlock(struct i2c_client *client, u8 *buffer, int length)
{
	/*
	 * Annoying we can't make this const because the i2c layer doesn't
	 * declare input buffers const.
	 */
	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= length,
		.buf	= buffer,
	};

#if 0
	if (length == 3) {
		dev_dbg(&client->dev,
			"lpc5400x_send_buf, CMD = %d, ID = %d, value_c = %d\n",
			buffer[0], buffer[1], buffer[2]);
	} else {
		if (length == 4) {
			unsigned short value;
			memcpy(&value, &buffer[2], 2);
			dev_dbg(&client->dev,
				"lpc5400x_send_buf, CMD = %d, ID = %d, value_s = %d\n",
				buffer[0], buffer[1], value);
		}
	}
#endif
    //printk("lpc5400x_send_buf buffer : %d\n", buffer[0]);

	return i2c_transfer(client->adapter, &msg, 1);
}



int lpc5400x_read_buf(struct i2c_client *client, u8 cmd, u8 *buffer, int length) {
	/*
	 * Annoying we can't make this const because the i2c layer doesn't
	 * declare input buffers const.
	 */
	struct _lpc5400x *lpc = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&lpc->sensorhub_irqmutex); 
	mutex_lock(&lpc->sensorhub_mutex); 
	mutex_unlock(&lpc->sensorhub_irqmutex); 

	ret=lpc5400x_read_buf_unlock(client, cmd, buffer,length);
    mutex_unlock(&lpc->sensorhub_mutex); 
	return ret;
}



/**
 *	spihub_send_buf	-	Writes the sensor hub packet
 *
 *	Writes the register values in one transaction or returns a negative
 *	error code on failure.
 */

int lpc5400x_send_buf(struct i2c_client *client, u8 *buffer, int length) {
	/*
	 * Annoying we can't make this const because the i2c layer doesn't
	 * declare input buffers const.
	 */
	struct _lpc5400x *lpc = i2c_get_clientdata(client);
	int ret;
	mutex_lock(&lpc->sensorhub_irqmutex); 
	mutex_lock(&lpc->sensorhub_mutex); 
	mutex_unlock(&lpc->sensorhub_irqmutex); 

	ret=lpc5400x_send_buf_unlock(client, buffer, length);
	mutex_unlock(&lpc->sensorhub_mutex); 
	return ret;
}

#ifdef CONFIG_LPC5400X_SENSORHUB_FIRMWARE_UPDATE_I2C
int lpc5400x_check_firmware(struct i2c_client *client)
{
	struct _lpc5400x *lpc = i2c_get_clientdata(client);
	int ret;

	if (lpc->isValidVersion == false && lpc->isFirmwareChecked == false) {
		ret = lpc5400x_updateFirmware(&client->dev, lpc->pdata->gpio);
		if (ret < 0) {
			lpc->isFirmwareChecked = true;
			return 0;
		}

		ret = request_threaded_irq(
			client->irq,
			lpc5400x_irq_handler,
			lpc5400x_readdata,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			dev_name(&client->dev),
			client);

		if (ret < 0) {
			dev_err(&client->dev, "irq %d busy?\n", client->irq);
			lpc->isFirmwareChecked = true;
			ret = -EBUSY;
			return ret;
		}

		lpc->isValidVersion = true;
	}

	lpc->isFirmwareChecked = true;

	return lpc->isValidVersion ? 0 : -EINVAL;
}
#endif

int lpc5400x_getVersion(
	struct device *dev,
	uint8_t *major_version,
	uint8_t *minor_version)
{
	struct i2c_client *client = i2c_verify_client(dev);
	struct _lpc5400x *lpc = i2c_get_clientdata(client);
	int ret = 0;

   // mutex_lock(&lpc->sensorhub_mutex); 
	
	ret = lpc5400x_read_buf(
		client,
		LPCSH_CMD_GET_VERSION,
		(u8 *)&lpc->devVersion,
		sizeof(lpc->devVersion));
	
   // mutex_unlock(&lpc->sensorhub_mutex); 

	pr_err("lpc5400x_getVersion, %d.%d isValidVersion:%d\n", lpc->devVersion.major_version, lpc->devVersion.minor_version, lpc->isValidVersion);
	//pr_err("lpc5400x_getVersion, flag:%d\n", (ret > 0 && lpc->isValidVersion == true));

	if ((ret > 0)
#ifdef CONFIG_LPC5400X_SENSORHUB_FIRMWARE_UPDATE_I2C
		&& (lpc->isValidVersion == true)
#endif
		)
	{
		*major_version = lpc->devVersion.major_version;
		*minor_version = lpc->devVersion.minor_version;
		#ifdef NXP_MCU_ZTE_PROC_FLAG
		nxp_fw_version_id[0] = lpc->devVersion.major_version;
	    nxp_fw_version_id[1] = lpc->devVersion.minor_version;
		#endif
		pr_err("lpc5400x_getVersion, %d.%d\n", *major_version, *minor_version);
	} else {
		*major_version = 0;
		*minor_version = 0;
		pr_err("lpc5400x_getVersion, BUS ERROR %d\n", ret);
	}

	return 0;
}

int lpc5400x_vfs_notifer(struct device *dev, int requset, void *pData)
{
    //ktime_t kt;
	struct i2c_client *client = i2c_verify_client(dev);
	struct _lpc5400x *lpc = i2c_get_clientdata(client);

	switch (requset) {
	case VFS_NOTIFY_UPDATE_COMPLETE:
		lpc->isValidVersion = *(bool *)pData;
		pr_err("VFS_NOTIFY_UPDATE_COMPLETE, Validversion = %d\n",
			lpc->isValidVersion);

			//lpc5400x_get_timestamp_function(lpc);
    		//lpc5400x_get_timestamp_function(lpc);
			/* schedule delay work after compilete firmware update at power up */
			//schedule_delayed_work(&lpc->lpc5400x_get_timestamp_work, msecs_to_jiffies(1000 * 60 * 60));
        	//kt = ns_to_ktime(AP_GET_MCU_TIMESTAMP_PERIOD);
        	//alarm_start_relative(&lpc->get_timestamp_alarm, kt);
        	
            /* Let new calibration parameters works, need delete later */       
            nxp_mcu_reset(lpc);

            lpc5400x_ps_calibration_powerup(client);
			
			/* initialize sensorhub/linux time synchronization */
		    lpc5400x_init_timeSync(lpc);			
		break;
	default:
		break;
	}

	return 0;
}



int lpc5400x_sensor_enable(struct i2c_client *client, int sensorId, bool enable)
{
	struct ShSensorSetCmdHeader_8bits_param_t cmd;
	int err = 0;
	struct _lpc5400x *lpc = i2c_get_clientdata(client);

#ifdef CONFIG_LPC5400X_SENSORHUB_FIRMWARE_UPDATE_I2C
	if (!lpc->isValidVersion)
		return -EINVAL;
#endif

	cmd.command = LPCSH_CMD_SENSOR_ENABLE;
	cmd.sensorId = sensorId;
	cmd.param = enable ? 0xff : 0x00;

    pr_err(
		"lpc5400x_sensor_enable cmd = %d, ID = %d, value = %d\n",
		cmd.command,
		cmd.sensorId, cmd.param);

    //mutex_lock(&lpc->sensorhub_mutex); 

	err = lpc5400x_send_buf(client, (u8 *)&cmd, sizeof(cmd));

    //mutex_unlock(&lpc->sensorhub_mutex); 

	if (err < 0) {
		dev_err(&client->dev, "lpc5400x_i2c_write %d\n", err);
		return err;
	}

	return 0;
}

int lpc5400x_sensor_samplerate(
	struct i2c_client *client,
	int sensorId,
	uint32_t value)
{
	struct ShSensorSetCmdHeader_32bits_param_t cmd;
	int err = 0;
	struct _lpc5400x *lpc = i2c_get_clientdata(client);

#ifdef CONFIG_LPC5400X_SENSORHUB_FIRMWARE_UPDATE_I2C

	if (!lpc->isValidVersion)
		return -EINVAL;
#endif

	cmd.command = LPCSH_CMD_SET_DELAY;
	cmd.sensorId = sensorId;
	cmd.param = value;

	dev_dbg(&client->dev,
		"lpc5400x_sensor_samplerate cmd = %d, ID = %d, value = %d\n",
		cmd.command,
		cmd.sensorId, cmd.param);

  //  mutex_lock(&lpc->sensorhub_mutex); 

	err = lpc5400x_send_buf(client, (char *)&cmd, sizeof(cmd));
	
  //  mutex_unlock(&lpc->sensorhub_mutex); 

	if (err < 0) {
		dev_err(&client->dev, "lpc5400x_i2c_write %d\n", err);
		return err;
	}
	return 0;
}

int lpc5400x_sensor_batch(
	struct i2c_client *client,
	int sensorId,
	int flag,
	u32 batchPeriodUsec,
	u32 batchTimeoutUsec)
{
	struct ShSensorSetCmdHeader_batch_t cmd;
	int err = 0;
	struct _lpc5400x *lpc = i2c_get_clientdata(client);

#ifdef CONFIG_LPC5400X_SENSORHUB_FIRMWARE_UPDATE_I2C
	if (!lpc->isValidVersion)
		return -EINVAL;
#endif

#ifdef CONFIG_LPC5400X_SENSORHUB_VOICE_TRIGGER
	if (sensorId == LPCSH_SENSOR_ID_VOICE_TRIGGER) {
		err = lpc5400x_voicetrigger_control(
			lpc->vt,
			batchPeriodUsec != 0);
		if (err < 0)
			return err;
	} else
#endif
	{
		cmd.command = LPCSH_CMD_SET_BATCH;
		cmd.sensorId = sensorId;
		cmd.flags =  flag;
		cmd.period_us =  batchPeriodUsec;
		cmd.timeout_us = batchTimeoutUsec;

		err = lpc5400x_send_buf(client, (char *)&cmd, sizeof(cmd));
    
		if (err < 0) {
			dev_err(&client->dev, "lpc5400x_i2c_write %d\n", err);
			return err;
		}
	}

	return 0;
}

int lpc5400x_startCalibration(
	struct device *dev,
	unsigned char *pData, int size)
{
	struct i2c_client *client = i2c_verify_client(dev);
	struct ShSensorSetCalibration_param_t cal_cmd;

	cal_cmd.command = LPCSH_CMD_SET_CALIBRATE;
	memcpy(&cal_cmd.data[0], pData, size);

	lpc5400x_send_buf(client, (u8 *)&cal_cmd, sizeof(cal_cmd));

	return 0;
}

static int lpc5400x_Calibration(struct i2c_client* client, uint8_t flag)
{
	struct ShSensorSetCalibration_param_t cal_cmd;
	Sensor_CalibrationData_t CalibrationData;

	struct leStk3311PsSpecCal_s  STK331128ADriverdefaultProxCal;

    /**< Special environmental calibration record header. */
    //STK331128ADriverdefaultProxCal.header.sensorGuid = 0x20;
    //STK331128ADriverdefaultProxCal.header.paramLength = 9;
       
    STK331128ADriverdefaultProxCal.hiGain1 = 100;          /**< Region 1 high threshold gain. Q4  90 */
    STK331128ADriverdefaultProxCal.hiGain2 = 120;          /**< Region 2 high threshold gain. Q4  100 */
    STK331128ADriverdefaultProxCal.hiGain3 = 150;          /**< Region 3 high threshold gain. Q4 130 */
    STK331128ADriverdefaultProxCal.loGain1 = 50;          /**< Region 1 low threshold gain.  Q4 45 */
    STK331128ADriverdefaultProxCal.loGain2 = 70;          /**< Region 2 low threshold gain.  Q4 60 */
    STK331128ADriverdefaultProxCal.loGain3 = 100;          /**< Region 3 low threshold gain.  Q4 80 */
    STK331128ADriverdefaultProxCal.maxThresholdHi = 2000;   /**< Maximum high interrupt threshold to use. ADCs. */
    STK331128ADriverdefaultProxCal.maxThresholdLo = 1500;   /**< Maximum low interrupt threshold to use.  ADCs. */
    STK331128ADriverdefaultProxCal.minThresholdHi = 150;   /**< Minimum high interrupt threshold to use. ADCs. */
    STK331128ADriverdefaultProxCal.minThresholdLo = 100;   /**< Minimum high interrupt threshold to use. ADCs. */
    STK331128ADriverdefaultProxCal.calParam1 = 150;        /**< Boundary between region 1 and region 2.  ADCs */
    STK331128ADriverdefaultProxCal.calParam2 = 400;        /**< Boundary between region 2 and region 3.  ADCs */
    STK331128ADriverdefaultProxCal.deviate = 290;          /**< Maximum deviation of current output from previous crosstalk allowed in order to run calibration. ADCs */
    STK331128ADriverdefaultProxCal.crosstalk = 310;        /**< Measured crosstalk from a successful calibration. ADCs */
    STK331128ADriverdefaultProxCal.ledDutyCycle = 63;     /**< The value to use in the DT_LED field of register 0x03. */
    STK331128ADriverdefaultProxCal.ledCurrent = 3;       /**< The value to use in the IRDR field of register 0x03 */
    STK331128ADriverdefaultProxCal.integrationTime = 0;  /**< The value to use in the IT_PS field of register 0x01 */
    STK331128ADriverdefaultProxCal.gain = 3;             /**< The value to use in the GAIN_PS field of register 0x01 */
    STK331128ADriverdefaultProxCal.outputSelection = 0;  /**< The output selection. 0 - centimeters, 1 - ADCs */
    STK331128ADriverdefaultProxCal.reserved1 = 0;        /**< Reserved */
    STK331128ADriverdefaultProxCal.reserved2 = 0;        /**< Reserved */
    STK331128ADriverdefaultProxCal.reserved3 = 0;        /**< Reserved */

	memset((void*)(&cal_cmd), 0x00, sizeof(cal_cmd));

	cal_cmd.command = LPCSH_CMD_SET_CALIBRATE;

	//memcpy((void*)(&CalibrationData.stk3311), (void*)(&STK331128ADriverdefaultProxCal), sizeof(leStk3311PsSpecCal_t));
    /* distinguish PV & powerup calibrate, 1 for Production Version */
	CalibrationData.writeflash = flag;   
	
	memcpy(&cal_cmd.data[0], (char *)&STK331128ADriverdefaultProxCal, sizeof(leStk3311PsSpecCal_t));
	cal_cmd.data[sizeof(leStk3311PsSpecCal_t)] = CalibrationData.writeflash;
	
	printk("lpc5400x_Calibration size [%ld], flag:%d\n", sizeof(leStk3311PsSpecCal_t), flag);

	lpc5400x_send_buf(client, (u8 *) &cal_cmd, sizeof(cal_cmd));


	return 0;
}


int lpc5400x_getCalibrationData(
	struct device *dev,
	unsigned char *pData,
	int size)
{
	struct i2c_client *client = i2c_verify_client(dev);
	int ret = 0;

	ret = lpc5400x_read_buf(
		client,
		LPCSH_CMD_GET_CALIBRATE,
		pData,
		size);

	if (ret < 0)
		dev_err(dev, "lpc5400x_getCalibrationData bus error %d\n", ret);

	return ret;
}


int lpc5400x_AccelCalibration(struct device *dev)
{
	int ret = 0;
	struct i2c_client *client = i2c_verify_client(dev);
    struct ShHubCmdHeader_t cal_cmd;

	cal_cmd.command = LPCSH_CMD_ACCEL_CALIBRATE;

	ret = lpc5400x_send_buf(client, (u8 *)&cal_cmd, sizeof(cal_cmd));
	
	if (ret < 0)
		dev_err(dev, "lpc5400x_AccelCalibration bus error %d", ret);
	return 0;
}

int lpc5400x_getAccelCalibrationData(struct device *dev, void *pData, int size)
{
	struct i2c_client *client = i2c_verify_client(dev);
	int ret = 0;

	ret = lpc5400x_read_buf(client,	LPCSH_CMD_ACCEL_READ_CALIBRATEDATA, pData, size);

	if (ret < 0)
		dev_err(dev, "lpc5400x_getAccelCalibrationData bus error %d", ret);

	return ret;
}


#ifdef CONFIG_LPC5400X_SENSORHUB_AUDIO_SENSE
int lpc5400x_setDmicClock(struct device *dev, void *pData, int size)
{
	int ret = 0;
	struct i2c_client *client = i2c_verify_client(dev);
	struct ShSensorAudiosense_t cal_cmd;

	cal_cmd.command = LPCSH_AUDIOSENSE_SET_DMIC_CLOCK;
	memcpy(&cal_cmd.data[0], pData, size);
    //lpc5400x_lock_i2c_client(client);
	ret = lpc5400x_send_buf(client, (u8 *)&cal_cmd, sizeof(cal_cmd));
    //lpc5400x_unlock_i2c_client(client);
	if (ret < 0)
		dev_err(dev, "lpc5400x_setDmicClock bus error %d", ret);
	return 0;
}

int lpc5400x_getDmicClock(struct device *dev, void *pData, int size)
{
	struct i2c_client *client = i2c_verify_client(dev);
	int ret = 0;

    //lpc5400x_lock_i2c_client(client);
	ret = lpc5400x_read_buf(
		client,
		LPCSH_AUDIOSENSE_GET_DMIC_CLOCK, pData, size);
    //lpc5400x_unlock_i2c_client(client);

	if (ret < 0)
		dev_err(dev, "lpc5400x_getDmicClock bus error %d", ret);

	return ret;
}
#endif

int lpc5400x_write_sensor_register(struct device *dev, void* pData, int size)
{
	int ret = 0;
	struct i2c_client* client = i2c_verify_client(dev);	
	struct ShSensor_RegisterDump_t cal_cmd;
	char* pCommand = (char*)pData;

	cal_cmd.command = LPCSH_CMD_WRITE_SENSOR_REGISTER;
	cal_cmd.i2cAddr = *(pCommand+1);
	cal_cmd.sensorRegister = *(pCommand+2);
	cal_cmd.length = *(pCommand+3);
	memcpy(&cal_cmd.data[0], (pCommand+4), cal_cmd.length);
	
    //lpc5400x_lock_i2c_client(client);
	ret = lpc5400x_send_buf(client, (u8 *)&cal_cmd, sizeof(cal_cmd));
    //lpc5400x_unlock_i2c_client(client);
	if (ret < 0)
		dev_err(dev, "lpc5400x_write_sensor_register bus error %d", ret);
	return 0;
}

int lpc5400x_read_sensor_register(struct device *dev, void* pData, int size)
{
	int ret = 0;
	struct i2c_client* client = i2c_verify_client(dev);	
	struct ShSensor_RegisterDump_t cal_cmd;
	//struct _lpc5400x *lpc = i2c_get_clientdata(client);
	char* pCommand = (char*)pData;

	cal_cmd.command = LPCSH_CMD_READ_SENSOR_REGISTER;
	cal_cmd.i2cAddr = *(pCommand+1);
	cal_cmd.sensorRegister = *(pCommand+2);
	cal_cmd.length = *(pCommand+3);

    //lpc5400x_lock_i2c_client(client);
#if 1
    //mutex_lock(&lpc->sensorhub_mutex);
    /* the first return byte is length, not the real buf data  */
	ret = lpc5400x_read_buf2(client, (u8 *)&cal_cmd, sizeof(cal_cmd), cal_cmd.data, cal_cmd.length+1);
    printk("lpc5400x_read_sensor_register  end ret = %d\n", ret);
	//mutex_unlock(&lpc->sensorhub_mutex);
#else
    ret = lpc5400x_read_buf(client, LPCSH_CMD_READ_REGISTER, pData, size);
#endif
    //lpc5400x_unlock_i2c_client(client);

	if (ret < 0)
		dev_err(dev, "lpc5400x_read_sensor_register bus error %d", ret);

	memcpy(pCommand, cal_cmd.data, cal_cmd.length+1);
	return ret;
}

static int lpc5400x_pm_suspend(struct i2c_client *client, int id)
{
	int err = 0;
	struct ShHubCmdHeader_t cmd;
	//struct _lpc5400x *lpc = i2c_get_clientdata(client);

	cmd.command = LPCSH_CMD_SET_SUSPEND;

	dev_dbg(&client->dev,
		"lpc5400x_pm_suspend cmd = %d    id %d\n",
		cmd.command, id);

   // mutex_lock(&lpc->sensorhub_mutex); 

	err = lpc5400x_send_buf(client, (u8 *)&cmd, sizeof(cmd));

   // mutex_unlock(&lpc->sensorhub_mutex); 

	if (err < 0) {
		dev_err(&client->dev, "lpc5400x_pm_suspend %d\n", err);
		return err;
	}

	return 0;
}

static int lpc5400x_pm_resume(struct i2c_client *client, int id)
{
	int err = 0;
	struct ShHubCmdHeader_t cmd;
    
//	struct _lpc5400x *lpc = i2c_get_clientdata(client);

	cmd.command = LPCSH_CMD_SET_RESUME;

	dev_dbg(&client->dev,
		"lpc5400x_pm_resume cmd = %d  id %d\n",
		cmd.command, id);

  //  mutex_lock(&lpc->sensorhub_mutex); 

	err = lpc5400x_send_buf(client, (u8 *) &cmd, sizeof(cmd));

  //  mutex_unlock(&lpc->sensorhub_mutex); 

	if (err < 0) {
		dev_err(&client->dev, "lpc5400x_pm_resume %d\n", err);
		return err;
	}
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void lpc5400x_early_suspend(struct early_suspend *handler)
{
	struct _lpc5400x *lpc;

	lpc = container_of(handler, struct _lpc5400x, early_suspend);


	dev_dbg(&lpc->client->dev, "lpc5400x_early_suspend lpc = %p\n", lpc);

}

static void lpc5400x_late_resume(struct early_suspend *handler)
{
	struct _lpc5400x *lpc;

	lpc = container_of(handler, struct _lpc5400x, early_suspend);
	dev_dbg(&lpc->client->dev, "lpc5400x_late_resume lpc = %p\n", lpc);
}
#endif

static void lpc5400x_readdata_interrupt_pending(struct i2c_client *client)
{
	int err = 0;
	struct _lpc5400x *lpc = i2c_get_clientdata(client);
	//unsigned long orig_jiffies;

	struct LPCSH_DataLen_t data_length_record = { 0 };

	u16 totalSizeRead = 0;
	u16 sizeRead = 0;
    //nxp34663 wakelock patch 2015.09.08 begin.
	bool apply_wakelock = false;
    //nxp34663 wakelock patch 2015.09.08 end.

	/*
	    Sensor hub wake up AP and IRQ is handled,
	    The bottom half should only read data from sensor hub
	    when Perpherial resume complete.
	    Normally, the sensor hub can resume after ~100ms
	    when AP wakes up
	 */
	printk("lpc5400x enter %s\n", __func__);


	do {
        mutex_lock(&lpc->sensorhub_irqmutex);
        mutex_lock(&lpc->sensorhub_mutex);
        mutex_unlock(&lpc->sensorhub_irqmutex);

        err = lpc5400x_read_buf_unlock(
			client,
			LPCSH_CMD_GET_DATA_LENGTH,
			(u8 *)&data_length_record,
			sizeof(data_length_record));
		/*pr_err("WJP lpc5400x_readdata err = %d, dataLen = %d\n",
				err, data_length_record.dataLen);*/

#ifdef TRACE_DATA
		dev_dbg(&client->dev,"lpc5400x_readdata err = %d, data_length_record.dataLen = %d\n",
				err, data_length_record.dataLen);
#endif
		if ((err >= 0) && (data_length_record.dataLen > 0)) {

			totalSizeRead = 0;
			while ((err >= 0) && (totalSizeRead < data_length_record.dataLen)) {
				if ((data_length_record.dataLen - totalSizeRead) >=
					HOSTIF_MAX_BUFFER_SIZE)
					sizeRead = HOSTIF_MAX_BUFFER_SIZE;
				else
					sizeRead = data_length_record.dataLen - totalSizeRead;

                err = lpc5400x_read_buf_unlock(
					client,
					LPCSH_CMD_GET_DATA,
					lpc->i2c_databuf,
					sizeRead);

				if (err >= 0) {
					int repostsize = reportdata(
						lpc,
						lpc->i2c_databuf,
						sizeRead,
						totalSizeRead,
						&apply_wakelock);
                    if (repostsize > 0){
                             totalSizeRead += repostsize;
                    }else{
                        pr_err("When reportdata returned 0, we will force exit interrupt function.\n");
                        break;
                    }
                } else 
					dev_err(&client->dev, "reportdata err [%d]\n", err);
            }
        } 
        mutex_unlock(&lpc->sensorhub_mutex); 

	} while (!gpio_get_value(lpc->pdata->irq_gpio));

	/* release wakelock here after read interrupt pending data */
	wake_unlock(&lpc->resumelock);
	
    //nxp34663 wakelock patch 2015.09.08 begin.
	if (apply_wakelock){
        wake_lock_timeout(
            &lpc->wakelk,
            msecs_to_jiffies(SENSOR_WAKELOCK_TIMEOUT));
	    /* printk(&client->dev, "Wakelock sensor: Apply a wakelock"); */
	}
    //nxp34663 wakelock patch 2015.09.08 end.
    
	return;
}


static int lpc5400x_pm_suspend_ops(struct device *dev)
{
	struct i2c_client *client = i2c_verify_client(dev);
	struct _lpc5400x *lpc = i2c_get_clientdata(client);

	irq_set_irq_wake(client->irq, 1);	/* make IRQ to wake AP */

	lpc5400x_pm_suspend(client, 2);

	lpc->pm_state = RPM_SUSPENDED;
	dev_info(dev, "lpc5400x_pm_suspend_ops %p, %p\n", client, lpc);

	return 0;
}

static int lpc5400x_pm_resume_ops(struct device *dev)
{
	struct i2c_client *client = i2c_verify_client(dev);
	struct _lpc5400x *lpc = i2c_get_clientdata(client);


	irq_set_irq_wake(client->irq, 0);	/* make IRQ to wake AP */

	lpc5400x_pm_resume(client, 2);
	dev_info(dev, "lpc5400x_pm_resume_ops %p\n", client);
	lpc->pm_state = RPM_ACTIVE;

	if (lpc->interrupt_pending){
		lpc5400x_readdata_interrupt_pending(client);
		lpc->interrupt_pending = false;
	}

	return 0;
}
static const struct dev_pm_ops lpc5400x_pm_ops = {
	.suspend	= lpc5400x_pm_suspend_ops,
	.resume		= lpc5400x_pm_resume_ops,
};

#if 0
static int lpc5400x_pm_notifier(
	struct notifier_block *b,
	unsigned long val, void *null)
{
	struct _lpc5400x *lpc =
		container_of(b, struct _lpc5400x, pm_nb);

	pr_debug("lpc5400x_pm_notifier: val = %ld, lpc = %p\n", val, lpc);

	switch (val) {
	case PM_SUSPEND_PREPARE:
		lpc5400x_pm_suspend(lpc->client, 3);
		break;
	case PM_POST_SUSPEND:
	case PM_RESTORE_PREPARE:
		break;
	case PM_POST_RESTORE:
		lpc5400x_pm_resume(lpc->client, 3);
		lpc->pm_state = RPM_ACTIVE;
		break;
	}
	return 0;
}
#endif


//nxp34663 wakelock patch 2015.09.08 begin.
static bool lpc5400x_apply_wakelock_flag(u8 sensorId)
{
    bool result = false;
    switch(sensorId)
    {
        case LPCSH_SENSOR_ID_PROXIMITY:
        //case LPCSH_SENSOR_ID_STEP_DETECTOR:
        //case LPCSH_SENSOR_ID_STEP_COUNTER:
        case LPCSH_SENSOR_ID_SIGNIFICANT_MOTION:
        case LPCSH_SENSOR_ID_AUG_REALITY_COMPASS:
        case LPCSH_SENSOR_ID_TAP_DETECTOR:
        case LPCSH_SENSOR_ID_SHAKE_DETECTOR:
        case LPCSH_SENSOR_ID_FLIP_DETECTOR:
        case LPCSH_SENSOR_ID_PICKUP_DETECTOR:
        case LPCSH_SENSOR_ID_STABILITY_DETECTOR:
        case LPCSH_SENSOR_ID_SLEEP_DETECTOR:
        case LPCSH_SENSOR_ID_TILT_DETECTOR:
        case LPCSH_SENSOR_ID_IN_POCKET_DETECTOR:
        case LPCSH_SENSOR_ID_IR_BLASTER_RX_PACKET:
        case LPCSH_SENSOR_ID_META_SENSOR_FLUSH_DONE:
        case LPCSH_SENSOR_ID_VOICE_TRIGGER:
        case LPCSH_SENSOR_ID_TOUCE_PANEL:
        case LPCSH_SENSOR_ID_AUDIOSENSE:
            result = true;
            break;
        default:
            result = false;
            break;
    }
	
	return result;
}
//nxp34663 wakelock patch 2015.09.08 end.


/* processes each block of samples, reports actual number of bytes processes */
static int reportdata(
	struct _lpc5400x *lpc,
	u8 *pdata,
	int readsize,
	int totalSizeRead,
    bool *apply_wakelock)
{
	int processedSize = 0, ret = 0;
	u8 sensorId = 0;
	struct lpcsh_sensor_node *pPack;
	int length;
	bool badData = false;

	while (readsize > 0) {
		badData = false;
		pPack = (struct lpcsh_sensor_node *)(pdata + processedSize);
		sensorId = pPack->header.sensorId;


	/*dev_err(&lpc->client->dev, "sensorId [%-2.2d] timeStamp [0x%-8.8x]  readsize [%-4.4d]",
			sensorId, pPack->header.timeStamp, readsize);
    */
		
#if 0
		if (processedSize == 0)
			dev_err(&lpc->client->dev,
				"sensorId [%-2.2d] totalSizeRead [%-4.4d] readsize [%-4.4d]",
				sensorId, totalSizeRead, readsize);

#endif
		if (sensorId == LPCSH_SENSOR_ID_FAKE_SENSOR)
			return processedSize;
		sensorId &= 0x7F;		/* ignore "stale" bit */

        if (apply_wakelock){
		    *apply_wakelock = (*apply_wakelock) || lpc5400x_apply_wakelock_flag(sensorId);
    	}

		switch (sensorId) {
		case LPCSH_SENSOR_ID_META_SENSOR_FLUSH_DONE:
			length = (
				sizeof(struct lpcsh_meta_sensor_flush_done_node) +
				sizeof(struct lpcsh_sensor_node_header));
			if (pPack->header.length != length) {
				/*dev_dbg(&lpc->client->dev,
					"Wrong length for sensorId [%d], header says [%d], "
					"table says [%d], using length from header\n",
					sensorId,
					pPack->header.length,
					sizeof(struct lpcsh_meta_sensor_flush_done_node) +
					sizeof(struct lpcsh_sensor_node_header));*/
				length = pPack->header.length;
				badData = true;
			} else {
				lpc_sensor_newdata(lpc->client,
					lpc->sensorHandle[pPack->data.flushDoneData.sensorId],
					pPack);
			}
			break;
		case LPCSH_SENSOR_ID_TIMESTAMP:
			length = sizeof(struct lpcsh_full_timestamp_node) +
					 sizeof(struct lpcsh_sensor_node_header);
			if (pPack->header.length != length) {
				dev_err(&lpc->client->dev,
					"Wrong length for sensorId [%d], header says [%d], "
					"table says [%d], using length from header\n",
					sensorId,
					pPack->header.length,
					length);
				length = pPack->header.length;
				badData = true;
			} else
				lpc_sensor_newdata(lpc->client, NULL, pPack);
			break;
#ifdef CONFIG_LPC5400X_SENSORHUB_IR_BLASTER
		case LPCSH_SENSOR_ID_IR_BLASTER_RX_PACKET:
			length =
				sizeof(struct lpcsh_sensor_node_header) +
				sizeof(struct lpcsh_ir_blaster_rx_node);
			if (pPack->header.length != length) {
				dev_dbg(&lpc->client->dev,
					"Wrong length for sensorId [%d], header says [%d], "
					"table says [%d], using length from header\n",
					sensorId,
					pPack->header.length,
					length);
				length = pPack->header.length;
				badData = true;
			} else {
				struct lpcsh_ir_blaster_rx_node *node =
					(struct lpcsh_ir_blaster_rx_node *)
					(pdata +
					 processedSize +
					 sizeof(struct lpcsh_sensor_node_header));

				lpc5400x_report_ir_blaster_learning_record(
					lpc->ir,
					node->data,
					node->size,
					pPack->header.timeStamp);
			}
			break;
#endif
#ifdef CONFIG_LPC5400X_SENSORHUB_LOGGING_SERVICE
		case LPCSH_SENSOR_ID_LOGGING_DATA:
		{
			struct  lpcsh_logServHeader_t *pHeader =
				(struct lpcsh_logServHeader_t *)pdata;
			lpc5400x_vfs_logging_save_events(
				lpc->vfs,
				(const uint8_t *)
				(pdata + sizeof(struct lpcsh_logServHeader_t)),
				pHeader->size - sizeof(struct lpcsh_logServHeader_t));
			length = pHeader->size;
		}
		break;
#endif
		default:
			if ((sensorId >= LPCSH_SENSOR_ID_COUNT) ||
				(lpc->sensorHandle[sensorId] == NULL)) {
				dev_dbg(&lpc->client->dev,
					"Unknown sensorId [%d], using length from header\n",
					sensorId);
				length = pPack->header.length;
				badData = true;
			} else {
				length = lpc_sensor_get_result_length(lpc->sensorHandle[sensorId]);
				if (length != pPack->header.length) {
					dev_dbg(
						&lpc->client->dev,
						"Wrong length for sensorId [%d], header says [%d], "
						"table says [%d], using length from header\n",
						sensorId, pPack->header.length, length);
					length = pPack->header.length;
					badData = true;
				} else {
					if ((!(pPack->header.sensorId & 0x80)) &&
						(badData == false)) {
						ret = lpc_sensor_newdata(
							lpc->client,
							lpc->sensorHandle[sensorId], pPack);
					}
				}
			}
		}

		if (readsize < length) {
			dev_dbg(&lpc->client->dev,
				"Partial packet received type = 0x%x, readsize = %d, length = %d\n",
				sensorId, readsize, length);
			dev_dbg(&lpc->client->dev,
				"0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n",
				*(pdata + processedSize + 0),
				*(pdata + processedSize + 1),
				*(pdata + processedSize + 2),
				*(pdata + processedSize + 3),
				*(pdata + processedSize + 4),
				*(pdata + processedSize + 5),
				*(pdata + processedSize + 6),
				*(pdata + processedSize + 7));
			break;
		}
		readsize -= length;
		processedSize += length;
	}
	return processedSize;
}

//zhangji

static void  sensorlist_info_get (struct i2c_client *client)
{
	int ret=0;
	int i;

	if(get_sensor_list_ok)
	{
		printk("lpc5400x ZJ has get sensor list\n");
		return;
	}
		
	ret = lpc5400x_read_buf(client, LPCSH_CMD_GET_SENSOR_LIST, (u8 *) &sensorlist_info,
	      sizeof(sensorlist_info));
	if (ret < 0) {
			dev_err(&client->dev, "lpc5400x zhangji failed to get sensor list.\n");
	}
	
	pr_debug("lpc5400x ZJ size: %d,command :%d\n",
			sensorlist_info.size,LPCSH_CMD_GET_SENSOR_LIST);
	
	if(sensorlist_info.size > 0){
		printk("lpc5400x ZJ get the sensor list ok!\n");
		for (i=0; i < sensorlist_info.size; i++)
	              pr_debug("lpc5400x ZJ:deviceType:%d   sensorType:%d\n",sensorlist_info.sensorInfo[i].deviceType,
											sensorlist_info.sensorInfo[i].sensorType);
		get_sensor_list_ok = true;
		}
	else
		{
		printk("lpc5400x ZJ don't get the sensor list !\n");
		get_sensor_list_ok = false;
		}

}

int lpc5400x_report_sensorlistinfo(unsigned int request,struct i2c_client *client)
{
	int i =0;
	if(request >= 20)
		request = request -20;
	sensorlist_info_get(client);
	printk("ZJ: now enter lpc5400x_report_sensorlistinfo size: %d, real type:%d\n",sensorlist_info.size,request);
		for (i=0; i < sensorlist_info.size; i++){
				    if ((sensorlist_info.sensorInfo[i].sensorType == request) && \
	                    			sensorlist_info.sensorInfo[i].deviceType != 0xff){
	                    				printk("ZJ:deviceType:%d   sensorType:%d\n",sensorlist_info.sensorInfo[i].deviceType,
											sensorlist_info.sensorInfo[i].sensorType);
						return sensorlist_info.sensorInfo[i].deviceType;
						}
		}
		return 0;
}

int flushfifo_request(struct i2c_client *client, int sensorId)
{
	int err;
	struct ShSensorCmdHeader_t cmd;
//	struct _lpc5400x *lpc = i2c_get_clientdata(client);

	cmd.command = LPCSH_CMD_SET_FLUSH;
	cmd.sensorId = sensorId;

    //mutex_lock(&lpc->sensorhub_mutex);
    
	err = lpc5400x_send_buf(client, (u8 *)&cmd, sizeof(cmd));

  //  mutex_unlock(&lpc->sensorhub_mutex); 

	if (err < 0) {
		dev_err(&client->dev, "flushfifo_request %d\n", err);
		return err;
	}
	return 0;
}

static irqreturn_t lpc5400x_readdata(int irq, void *userdata)
{
	int err = 0;
	struct i2c_client *client = (struct i2c_client *)userdata;
	struct _lpc5400x *lpc = i2c_get_clientdata(client);
	unsigned long orig_jiffies;

	struct LPCSH_DataLen_t data_length_record = { 0 };

	u16 totalSizeRead = 0;
	u16 sizeRead = 0;
    //nxp34663 wakelock patch 2015.09.08 begin.
	bool apply_wakelock = false;
    //nxp34663 wakelock patch 2015.09.08 end.

	/*
	    Sensor hub wake up AP and IRQ is handled,
	    The bottom half should only read data from sensor hub
	    when Perpherial resume complete.
	    Normally, the sensor hub can resume after ~100ms
	    when AP wakes up
	 */

	//wake_lock(&lpc->wakelk);

	orig_jiffies = jiffies;
	if(lpc->pm_state != RPM_ACTIVE)
		wake_lock(&lpc->resumelock);

	while (lpc->pm_state != RPM_ACTIVE) {
		msleep(20);
		if (time_after(jiffies,
				orig_jiffies +
				msecs_to_jiffies(SENSORHUB_RESUME_TIME))) {
			printk("Discard the interupt, Lpc state = %d\n", lpc->pm_state);
			//wake_unlock(&lpc->wakelk);
			lpc->interrupt_pending = true;
			
			return IRQ_HANDLED;
		}
	}
	//if(wake_lock_active(&lpc->resumelock)){
	//	printk("lpc5400x release the resumelock\n");
		wake_unlock(&lpc->resumelock);
	//}

	//wake_unlock(&lpc->wakelk);

	do {
        mutex_lock(&lpc->sensorhub_irqmutex);
        mutex_lock(&lpc->sensorhub_mutex);
        mutex_unlock(&lpc->sensorhub_irqmutex);

        err = lpc5400x_read_buf_unlock(
			client,
			LPCSH_CMD_GET_DATA_LENGTH,
			(u8 *)&data_length_record,
			sizeof(data_length_record));
		/*pr_err("WJP lpc5400x_readdata err = %d, dataLen = %d\n",
				err, data_length_record.dataLen);*/

#ifdef TRACE_DATA
		dev_dbg(&client->dev,"lpc5400x_readdata err = %d, data_length_record.dataLen = %d\n",
				err, data_length_record.dataLen);
#endif
		if ((err >= 0) && (data_length_record.dataLen > 0)) {

			totalSizeRead = 0;
			while ((err >= 0) && (totalSizeRead < data_length_record.dataLen)) {
				if ((data_length_record.dataLen - totalSizeRead) >=
					HOSTIF_MAX_BUFFER_SIZE)
					sizeRead = HOSTIF_MAX_BUFFER_SIZE;
				else
					sizeRead = data_length_record.dataLen - totalSizeRead;

                err = lpc5400x_read_buf_unlock(
					client,
					LPCSH_CMD_GET_DATA,
					lpc->i2c_databuf,
					sizeRead);

				if (err >= 0) {
					int repostsize = reportdata(
						lpc,
						lpc->i2c_databuf,
						sizeRead,
						totalSizeRead,
						&apply_wakelock);
                    if (repostsize > 0){
                             totalSizeRead += repostsize;
                    }else{
                        pr_err("When reportdata returned 0, we will force exit interrupt function.\n");
                        break;
                    }
                } else 
					dev_err(&client->dev, "reportdata err [%d]\n", err);
            }
        } 
        mutex_unlock(&lpc->sensorhub_mutex); 

	} while (!gpio_get_value(lpc->pdata->irq_gpio));
	//wake_unlock(&lpc->wakelk);
	
    //nxp34663 wakelock patch 2015.09.08 begin.
	if (apply_wakelock){
        wake_lock_timeout(
            &lpc->wakelk,
            msecs_to_jiffies(SENSOR_WAKELOCK_TIMEOUT));
	    /* printk(&client->dev, "Wakelock sensor: Apply a wakelock"); */
	}
    //nxp34663 wakelock patch 2015.09.08 end.
    
	return IRQ_HANDLED;
}

static irqreturn_t lpc5400x_irq_handler(int irq, void *userdata)
{
	struct _lpc5400x *lpc;

	lpc = i2c_get_clientdata((struct i2c_client *)userdata);

#ifdef CONFIG_LPC5400X_SENSORHUB_FIRMWARE_UPDATE_I2C
	if (lpc->isValidVersion == true)
		return IRQ_WAKE_THREAD;
	return IRQ_HANDLED;
#else
	return IRQ_WAKE_THREAD;
#endif
}

/* Sends LPCSH_CMD_GET_TIME_SYNCHRONIZE command and stores linux
    time in linux_sync_time and lpc time in lpc_sync_time */
static uint32_t lpc5400x_get_synced_times(
	struct i2c_client *client,
	uint64_t *linux_sync_time,
	uint64_t *lpc_sync_time)
{
	struct timespec linux_before;
	uint64_t linux_before_ns;
	uint64_t linux_t0_ns;
	struct ShHubTimeSynchronizeData_t timeStampRecord;
	int ret = 0;
	
/*  WARNING- Never lock the client when doing time sync.
    It is already locked by the IRQ thread
	lpc5400x_lock_i2c_client(client);
*/

	/* First get linux current time, then query sensorhub for its local time */
	get_monotonic_boottime(&linux_before);

	ret = lpc5400x_read_buf_unlock(
		client,
		LPCSH_CMD_GET_TIME_SYNCHRONIZE,
		(u8 *)&timeStampRecord,
		sizeof(struct ShHubTimeSynchronizeData_t));
	if (ret < 0) {
		dev_err(&client->dev,
			"Failed to get timeStamp synchronization from MCU\n");
		return 1;	/* failed */
	}
	linux_before_ns =
		((uint64_t)linux_before.tv_sec) * 1000000000LL +
		(uint64_t)linux_before.tv_nsec;
	linux_t0_ns = linux_before_ns + LPC5400X_CMD_RESPONSE_LATENCY;

	/* save results */
	*linux_sync_time = linux_t0_ns;
	*lpc_sync_time = timeStampRecord.timeStamp;

	return 0;	/* OK */
}

/* Initialize sensorhub/linux time synchronization */
static uint32_t lpc5400x_init_timeSync(struct _lpc5400x *lpc)
{
	uint32_t ret = 0;

	ret = lpc5400x_get_synced_times(
		lpc->client,
		&lpc->timesync_linux_sync_time,
		&lpc->timesync_lpc_sync_time);
	if (!ret) {
		lpc->timesync_linux_sync_time_prev = lpc->timesync_linux_sync_time;
		lpc->timesync_lpc_sync_time_prev = lpc->timesync_lpc_sync_time;

		/* linux time reported in nsec, lpc time reported in usec */
		lpc->timesync_mult = 1000 << LPC_TS_Q;

		lpc->timesync_mult_prev = lpc->timesync_mult;

		/* sync calibrated time with linux time */
		lpc->timesync_calibrated_lpc_time = lpc->timesync_linux_sync_time;

		lpc->timesync_calibrated_lpc_time_prev = lpc->timesync_calibrated_lpc_time;
#if 0
		dev_dbg(&client->dev,
			"Initial time sync on [%10.10llu]\n",
			lpc->timesync_linux_sync_time);
#endif
	}

	return ret;
}

/* Updates timesync related variables in the lpc struct */
static uint32_t lpc5400x_update_timeSync(struct i2c_client *client)
{
	uint32_t ret = 0;
	uint64_t old_linux_sync_time;
	uint64_t old_lpc_sync_time;
	uint64_t delta_time_linux;
	uint64_t delta_time_lpc;

	/* first get synchronized linux/sensorhub time */
	struct _lpc5400x *lpc = i2c_get_clientdata(client);

	old_linux_sync_time = lpc->timesync_linux_sync_time;
	old_lpc_sync_time = lpc->timesync_lpc_sync_time;
	ret = lpc5400x_get_synced_times(
		client,
		&lpc->timesync_linux_sync_time,
		&lpc->timesync_lpc_sync_time);
	if (!ret) {
		/* if successfully received time from sensorhub, continue the sync */

		/* save old linux sync time */
		lpc->timesync_linux_sync_time_prev = old_linux_sync_time;

		/* save old lpc sync time */
		lpc->timesync_lpc_sync_time_prev = old_lpc_sync_time;

		/* save old calibrated counter value */
		lpc->timesync_calibrated_lpc_time_prev = lpc->timesync_calibrated_lpc_time;

		/* Increasing calibrated counter with [delta lpc time]*[old multiplier] */
		lpc->timesync_calibrated_lpc_time +=
			(lpc->timesync_mult *
			 (lpc->timesync_lpc_sync_time -
			  lpc->timesync_lpc_sync_time_prev)) >>
			LPC_TS_Q;
		/* save old multiplier before calculating the new multiplier */
		lpc->timesync_mult_prev = lpc->timesync_mult;

		/* note: linux time in nsec, LPC time in usec */
		delta_time_linux =
			((lpc->timesync_linux_sync_time - lpc->timesync_linux_sync_time_prev) <<
			 LPC_TS_Q);

		/* note: linux time in nsec, LPC time in usec */
		delta_time_lpc =
			(lpc->timesync_lpc_sync_time - lpc->timesync_lpc_sync_time_prev);
		/* no 64-bit division avilable in kernel space, so shift until both
		    operands are 32 bit */
		while (
			(delta_time_linux & 0xFFFFFFFF00000000) ||
			(delta_time_lpc & 0xFFFFFFFF00000000)) {
			delta_time_linux = delta_time_linux >> 1;
			delta_time_lpc = delta_time_lpc >> 1;
		}
		lpc->timesync_mult = (uint32_t)delta_time_linux / (uint32_t)delta_time_lpc;
		if (
			(lpc->timesync_mult < ((99 * 1000 << LPC_TS_Q) / 100)) ||
			(lpc->timesync_mult > ((101 * 1000 << LPC_TS_Q) / 100)))
			lpc5400x_init_timeSync(lpc);
	}
#if 0
	dev_dbg(&client->dev,
		"Synced with linux time: dt linux [%10.10llu], dt lpc [%10.10llu], "
		"linux_sync_time [%10.10llu], calibrated_lpc_time [%10.10llu], "
		"new mult [%10.10u]\n",
		delta_time_linux,
		delta_time_lpc,
		lpc->timesync_linux_sync_time,
		lpc->timesync_calibrated_lpc_time,
		lpc->timesync_mult);
#endif

	return ret;
}

/* convert timestamp (reported in local sensorhub time)
    to synchronized linux time */
uint64_t lpc5400x_get_synced_timestamp(
	struct i2c_client *client,
	uint64_t timestampIn)
{
	struct timespec linux_curr_timespec;
	uint64_t linux_curr_time;
	uint64_t timestampOut;

	struct _lpc5400x *lpc = i2c_get_clientdata(client);

	/* compare current linux time with last time we've synced.
	    Resync if configured period has elapsed */
	get_monotonic_boottime(&linux_curr_timespec);
	linux_curr_time =
		((uint64_t)linux_curr_timespec.tv_sec) *
		1000000000LL +
		(uint64_t)linux_curr_timespec.tv_nsec;
	if ((linux_curr_time - lpc->timesync_linux_sync_time) > LPC_TS_SYNC_NS)
		/* configured period since last sync has elpased,
		    so sync-up with linux time */
		lpc5400x_update_timeSync(client);

	/* convert timestamp in local sensorhub time to synchronized linux time */
	/* check timestamp against last lpc sync time, as we may get samples from
	    before last sync */
	if (timestampIn < lpc->timesync_lpc_sync_time_prev) {
		/* Received timestamp is smaller than before-last sync.
		    This can happen when sensorhub got reset without resetting the host,
		    so re-initialize sync */
		lpc5400x_init_timeSync(lpc);
		/* resynced, but timestamp is still smaller time of last sync */
		timestampOut = lpc->timesync_calibrated_lpc_time;
		timestampOut -=
			(lpc->timesync_mult * (lpc->timesync_lpc_sync_time - timestampIn)) >>
			LPC_TS_Q;
	}
	else if(timestampIn < lpc->timesync_lpc_sync_time) {
		timestampOut = lpc->timesync_calibrated_lpc_time_prev;
		timestampOut +=
			(lpc->timesync_mult_prev *
			 (timestampIn - lpc->timesync_lpc_sync_time_prev)) >> LPC_TS_Q;
	}
	else {
		timestampOut = lpc->timesync_calibrated_lpc_time;
		timestampOut +=
			(lpc->timesync_mult *
			 (timestampIn - lpc->timesync_lpc_sync_time)) >> LPC_TS_Q;
	}

	return timestampOut;
}


#define NXP_MCU_VIO_MIN_UV	1750000
#define NXP_MCU_VIO_MAX_UV	1950000
#define NXP_MCU_VDD_MIN_UV	2700000
#define NXP_MCU_VDD_MAX_UV	3300000

static int nxp_mcu_power_set(struct _lpc5400x *lpc, bool on)
{
	int rc = 0;

	if (!on && lpc->pdata->power_enabled) {
		rc = regulator_disable(lpc->pdata->vdd);
		if (rc) {
			dev_err(&lpc->client->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			goto err_vdd_disable;
		}

		rc = regulator_disable(lpc->pdata->vio);
		if (rc) {
			dev_err(&lpc->client->dev,
				"Regulator vio disable failed rc=%d\n", rc);
			goto err_vio_disable;
		}
		lpc->pdata->power_enabled = false;
		return rc;
	} else if (on && !lpc->pdata->power_enabled) {
		rc = regulator_enable(lpc->pdata->vdd);
		if (rc) {
			dev_err(&lpc->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			goto err_vdd_enable;
		}

		rc = regulator_enable(lpc->pdata->vio);
		if (rc) {
			dev_err(&lpc->client->dev,
				"Regulator vio enable failed rc=%d\n", rc);
			goto err_vio_enable;
		}
		lpc->pdata->power_enabled = true;

		/*
		 * The max time for the power supply rise time is 50ms.
		 * Use 80ms to make sure it meets the requirements.
		 */
		msleep(80);
		return rc;
	} else {
		dev_warn(&lpc->client->dev,
				"Power on=%d. enabled=%d\n",
				on, lpc->pdata->power_enabled);
		return rc;
	}

err_vio_enable:
	regulator_disable(lpc->pdata->vio);
err_vdd_enable:
	return rc;

err_vio_disable:
	if (regulator_enable(lpc->pdata->vdd))
		dev_warn(&lpc->client->dev, "Regulator vdd enable failed\n");
err_vdd_disable:
	return rc;
}

static int nxp_mcu_power_init(struct _lpc5400x *lpc, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(lpc->pdata->vdd) > 0)
			regulator_set_voltage(lpc->pdata->vdd, 0,
				NXP_MCU_VDD_MAX_UV);

		regulator_put(lpc->pdata->vdd);

		if (regulator_count_voltages(lpc->pdata->vio) > 0)
			regulator_set_voltage(lpc->pdata->vio, 0,
				NXP_MCU_VIO_MAX_UV);

		regulator_put(lpc->pdata->vio);

	} else {
		lpc->pdata->vdd = regulator_get(&lpc->client->dev, "vdd");
		if (IS_ERR(lpc->pdata->vdd)) {
			rc = PTR_ERR(lpc->pdata->vdd);
			dev_err(&lpc->client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(lpc->pdata->vdd) > 0) {
			rc = regulator_set_voltage(lpc->pdata->vdd,
				NXP_MCU_VDD_MIN_UV, NXP_MCU_VDD_MAX_UV);
			if (rc) {
				dev_err(&lpc->client->dev,
					"Regulator set failed vdd rc=%d\n",
					rc);
				goto reg_vdd_put;
			}
		}

		lpc->pdata->vio = regulator_get(&lpc->client->dev, "vio");
		if (IS_ERR(lpc->pdata->vio)) {
			rc = PTR_ERR(lpc->pdata->vio);
			dev_err(&lpc->client->dev,
				"Regulator get failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(lpc->pdata->vio) > 0) {
			rc = regulator_set_voltage(lpc->pdata->vio,
				NXP_MCU_VIO_MIN_UV, NXP_MCU_VIO_MAX_UV);
			if (rc) {
				dev_err(&lpc->client->dev,
				"Regulator set failed vio rc=%d\n", rc);
				goto reg_vio_put;
			}
		}
	}

	return 0;

reg_vio_put:
	regulator_put(lpc->pdata->vio);
reg_vdd_set:
	if (regulator_count_voltages(lpc->pdata->vdd) > 0)
		regulator_set_voltage(lpc->pdata->vdd, 0, NXP_MCU_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(lpc->pdata->vdd);
	return rc;
}

static int nxp_config_gpio(struct _lpc5400x *lpc)
{
	int ret = 0;
	
	if(gpio_is_valid(lpc->pdata->wakeup_gpio)){
		ret = gpio_request(lpc->pdata->wakeup_gpio, "nxp,wakeup-gpio");
		printk("nxp,wakeup-gpio request ret:%d\n",ret);
		gpio_direction_output(lpc->pdata->wakeup_gpio, 1);
	}
	if(gpio_is_valid(lpc->pdata->irq_gpio)){
		ret = gpio_request(lpc->pdata->irq_gpio, "nxp,irq-gpio");
		printk("nxp,irq-gpio request ret:%d\n",ret);
		gpio_direction_input(lpc->pdata->irq_gpio);
	}
	if(gpio_is_valid(lpc->pdata->reset_gpio)){
		ret = gpio_request(lpc->pdata->reset_gpio, "nxp,reset-gpio");
		//gpio_direction_output(sensor->wakeup_gpio, 0);
	}

	return ret;
}
static void nxp_unconfig_gpio(struct _lpc5400x *lpc)
{
	gpio_free(lpc->pdata->wakeup_gpio);
	gpio_free(lpc->pdata->irq_gpio);
	gpio_free(lpc->pdata->reset_gpio);

    return;
}

static void nxp_mcu_reset(struct _lpc5400x *lpc)
{    
	gpio_direction_output(lpc->pdata->wakeup_gpio, 1);
	gpio_direction_output(lpc->pdata->reset_gpio, 1);

	//gpio_set_value(lpc->pdata->wakeup_gpio, 1);
	//msleep(2);
	//gpio_set_value(lpc->pdata->reset_gpio, 1);
	//msleep(5);
	gpio_set_value(lpc->pdata->reset_gpio, 0);
	msleep(3);
	gpio_set_value(lpc->pdata->reset_gpio, 1);
	msleep(450);
}


static int nxp_parse_dt(struct device *dev,
			 struct _lpc5400x *lpc)
{
	struct device_node *np = dev->of_node;
	int ret = 0;

	ret = of_get_named_gpio(np, "nxp,irq-gpio", 0);
	if (ret < 0) {
		pr_err("failed to get \"nxp,irq_gpio\"\n");
		goto err;
	}
	lpc->pdata->irq_gpio= ret;

	ret = of_get_named_gpio(np, "nxp,wakeup-gpio", 0);
	if (ret < 0) {
		pr_err("failed to get \"wakeup\"\n");
		goto err;
	}
	lpc->pdata->wakeup_gpio = ret;

	ret = of_get_named_gpio(np, "nxp,reset-gpio", 0);
	if (ret < 0) {
		pr_err("failed to get \"reset\"\n");
		goto err;
	}
	lpc->pdata->reset_gpio = ret;

	//printk("nxp,irq-gpio is %d\n", lpc->pdata->irq_gpio);
	//printk("nxp,wakeup-gpio is %d\n", lpc->pdata->wakeup_gpio);	
	//printk("nxp,reset-gpio is %d\n", lpc->pdata->reset_gpio);

err:
	return ret;
}


static int lpc5400x_Incal_enable_sensors(struct i2c_client *client)
{
   int ret = 0;
   
   /* Accel */
   ret = lpc5400x_sensor_batch(client, LPCSH_SENSOR_ID_ACCELEROMETER, 0, 40000, 0);
   if(ret != 0)
   {
       pr_err("lpc5400x_Incal_enable_sensors ACCELEROMETER batch failed\n");
       return ret;
   }
   ret = lpc5400x_sensor_enable(client, LPCSH_SENSOR_ID_ACCELEROMETER, 1);
   if(ret != 0)
   {
       pr_err("lpc5400x_Incal_enable_sensors ACCELEROMETER enable failed\n");
       return ret;
   }
   
   /* Mag */
   ret = lpc5400x_sensor_batch(client, LPCSH_SENSOR_ID_MAGNETIC_FIELD, 0, 40000, 0);
   if(ret != 0)
   {
       pr_err("lpc5400x_Incal_enable_sensors MAGNETIC batch failed\n");
       return ret;
   }
   ret = lpc5400x_sensor_enable(client, LPCSH_SENSOR_ID_MAGNETIC_FIELD, 1);
   if(ret != 0)
   {
       pr_err("lpc5400x_Incal_enable_sensors MAGNETIC enable failed\n");
       return ret;
   }
   
   /* Gyro */
   ret = lpc5400x_sensor_batch(client, LPCSH_SENSOR_ID_GYROSCOPE, 0, 40000, 0);
   if(ret != 0)
   {
       pr_err("lpc5400x_Incal_enable_sensors GYROSCOPE batch failed\n");
       return ret;
   }
   ret = lpc5400x_sensor_enable(client, LPCSH_SENSOR_ID_GYROSCOPE, 1);
   if(ret != 0)
   {
       pr_err("lpc5400x_Incal_enable_sensors GYROSCOPE enable failed\n");
       return ret;
   }
   
   /* Light */
   ret = lpc5400x_sensor_batch(client, LPCSH_SENSOR_ID_LIGHT, 0, 40000, 0);
   if(ret != 0)
   {
       pr_err("lpc5400x_Incal_enable_sensors LIGHT batch failed\n");
       return ret;
   }
   ret = lpc5400x_sensor_enable(client, LPCSH_SENSOR_ID_LIGHT, 1);
   if(ret != 0)
   {
       pr_err("lpc5400x_Incal_enable_sensors LIGHT enable failed\n");
       return ret;
   }
   
   /* PS */
   ret = lpc5400x_sensor_batch(client, LPCSH_SENSOR_ID_PROXIMITY, 0, 40000, 0);
   if(ret != 0)
   {
       pr_err("lpc5400x_Incal_enable_sensors PROXIMITY batch failed\n");
       return ret;
   }
   ret = lpc5400x_sensor_enable(client, LPCSH_SENSOR_ID_PROXIMITY, 1);
   if(ret != 0)
   {
       pr_err("lpc5400x_Incal_enable_sensors PROXIMITY enable failed\n");
       return ret;
   }
   
   
   return 0;
}

static int lpc54xx_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, NULL, NULL);
}
struct i2c_client *g_client = NULL;
static ssize_t lpc54xx_proc_write(struct file *file, const char __user *buffer,
				    size_t count, loff_t *pos)
{	

    u8 cal_data[4] = {0};
	unsigned char accel_cal_data[13] = {0};
    int ret = 0;
	uint16_t crosstalk = 0;
	uint16_t adc = 0;
	uint32_t accel_x, accel_y, accel_z;
	struct _lpc5400x *lpc;;

	if(g_client == NULL)
		return 0;
	pr_err("%s enter PV calibrate \n", __func__ );
	lpc = i2c_get_clientdata(g_client);

    /* 1. Do Accel calibration */
	ret = lpc5400x_AccelCalibration(&g_client->dev);
	if (ret < 0){
            pr_err("lpc5400x Accel Calibration I2C bus failed, ret:%d \n", ret);
            return -1;
    }
	
	msleep(50);
    ret = lpc5400x_getAccelCalibrationData(&g_client->dev,
    &accel_cal_data[0], sizeof(accel_cal_data));

    if (ret < 0){
		    pr_err("lpc5400x getAccel Calibration I2C bus failed, ret:%d \n", ret);
            return -1;
    } 
	
    memcpy(&accel_x, &accel_cal_data[1], 4);
    memcpy(&accel_y, &accel_cal_data[5], 4);
    memcpy(&accel_z, &accel_cal_data[9], 4);
    /* x/y/z value is Q15 by MCU */
	if(accel_cal_data[0] == 0){
        printk("lpc5400x Accel Calibration success, X: 0x%x, Y: 0x%x, Z:0x%x \n", accel_x, accel_y, accel_z);
	} else { 
        pr_err("lpc5400x Accel Calibration failed, X: 0x%x, Y: 0x%x, Z:0x%x \n", accel_x, accel_y, accel_z);
        return -1;
	} 

	/* 2. Reset MCU */
	pr_err("%s PV calibrate start reset MCU \n", __func__ );
    nxp_mcu_reset(lpc);

    /* 3. enable A/G/M/ALS/PS again after reset MCU */
	printk("%s enable A/G/M/ALS/PS again after reset MCU\n", __func__ );
	lpc5400x_Incal_enable_sensors(g_client);

	/* 4. Do PS calibration */
	lpc5400x_Calibration(g_client, 1);
	
	msleep(60);
    ret = lpc5400x_getCalibrationData(&g_client->dev,
    &cal_data[0], sizeof(cal_data));

	crosstalk = (cal_data[1]<<8)|cal_data[0];

	if(cal_data[2] == 0 && cal_data[3] == 0) {
		printk("lpc5400x Calibration success, Crosstalk value %d \n", crosstalk);
	} else {
		adc = (cal_data[3]<<8)|cal_data[2];
		
		if(crosstalk == 0xAA55 && adc == 0x55AA) {
			pr_err("lpc5400x Calibration is not completed, try later again please");
		} else {
			pr_err("lpc5400x Calibration failed, old crosstalk value %d, ADC value %d", crosstalk, adc);
		}

		return -1;
	}

	printk("lpc5400x Calibration success, count %d \n", (int)count);
	
	return crosstalk;
}


static const struct file_operations lpc54xx_proc_fops = {
	.open		= lpc54xx_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= lpc54xx_proc_write,
};

static int lpc54xx_proc_init(void)
{
 	struct proc_dir_entry *res;
	res = proc_create("sensor_calibration", 0664, NULL,
			  &lpc54xx_proc_fops);
	if (!res)
	{
		printk(KERN_INFO "failed to create /proc/sensor_calibration\n");
		return -ENOMEM;
	}

	printk(KERN_INFO "created /proc/sensor_calibration\n");
	return 0;
}


static void lpc5400x_ps_calibration_powerup(struct i2c_client *client)
{
    u8 cal_data[4];
    int ret = 0;
    uint16_t crosstalk = 0;
    uint16_t old_crosstalk = 0;
    uint16_t adc = 0;
    int value=0;
    struct _lpc5400x *lpc = i2c_get_clientdata(client);
	
    ret = lpc5400x_getCalibrationData(&client->dev,
    &cal_data[0], sizeof(cal_data));

    old_crosstalk = (cal_data[1]<<8)|cal_data[0];
    printk("lpc5400x get the old_value:%d\n",old_crosstalk);
    if((cal_data[2] != 0 || cal_data[3] != 0)
   			|| old_crosstalk == 0xAA55){
		pr_err("lpc5400x now is old version\n");
		return;
	}
	
    lpc5400x_Calibration(client, 0);
    msleep(50);
    ret = lpc5400x_getCalibrationData(&client->dev,
    &cal_data[0], sizeof(cal_data));

    crosstalk = (cal_data[1]<<8)|cal_data[0]; 
    if(cal_data[2] == 0 && cal_data[3] == 0) {
	    value = crosstalk - old_crosstalk;
	    printk("lpc5400x get the new_value:%d\n",crosstalk);
	    if((value < 0)&&(abs(value)>30)){
			printk("lpc5400x now cross value is smaller,need to reset mcu\n");
			nxp_mcu_reset(lpc);
	    }else if(abs(value)<290){
	    		printk("lpc5400x Calibration success!\n");
	    }else{
	    		printk("lpc5400x Calibration failed\n");
	    }
		
	} else {
		adc = (cal_data[3]<<8)|cal_data[2];
		
		if(crosstalk == 0xAA55 && adc == 0x55AA) {
			pr_err("lpc5400x Calibration is not completed, try later again please");
		} else {
			pr_err("lpc5400x Calibration failed, old crosstalk value %d, ADC value %d", crosstalk, adc);
		}
	}	

	//nxp_mcu_reset(lpc);
}


#ifdef NXP_MCU_ZTE_PROC_FLAG
static int  nxp_mcu_id_proc_read(struct seq_file *m, void *v)
{
	seq_printf(m, "NXP-LPC540x-%d.%d-0x68\n", nxp_fw_version_id[0], nxp_fw_version_id[1]);

	return 0;
}

static int nxp_mcu_id_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, nxp_mcu_id_proc_read, NULL);
}

 static const struct file_operations nxp_mcu_id = {
	.open		= nxp_mcu_id_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif

static int lpc5400x_probe(
	struct i2c_client *client,
	const struct i2c_device_id *device_id)
{
	int ret = 0, i;

	struct _lpc5400x        *lpc;
	
	//struct ShHubCmdHeader_t cmd;
	struct pinctrl *pinctrl;
	//ktime_t kt;

    printk("WJP NXP enter %s\n", __func__);

	ret = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
	if (!ret) {
	    dev_err(&client->dev,"*** lpc5400x_probe... error %d\n", ret);
		dev_err(&client->dev, "SMBUS Byte Data not Supported\n");
		ret = -EIO;
		return ret;
	}

	dev_dbg(&client->dev,
			"lpc5400x_probe lpc = %p, client = %p, Addr = %d\n",
			lpc, client, client->addr);

    pinctrl = devm_pinctrl_get_select_default(&client->dev);
    if (IS_ERR(pinctrl)) {
         dev_warn(&client->dev, "pins are not configured from the driver\n");
         return -EINVAL;
    } 

	lpc = kzalloc(sizeof(*lpc), GFP_KERNEL);
	if (!lpc) {
		dev_dbg(&client->dev, "*** lpc5400x_probe ENOMEM...\n");
		return -ENOMEM;
	}

	//lpc->pdata = client->dev.platform_data;
	lpc->client = client;
	i2c_set_clientdata(client, lpc);


	lpc->pdata = kzalloc(sizeof(*lpc->pdata), GFP_KERNEL);
	if (lpc->pdata == NULL) {
		ret = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for pdata: %d\n",
				ret);
		goto err_alloc_pdata;
	}
	
	ret = nxp_parse_dt(&client->dev, lpc);
	if (ret < 0) {
		pr_err("failed to parse device tree: %d\n", ret);
		goto err_parse_dt;
	}
	
	ret = nxp_config_gpio(lpc);
	if (ret < 0) {
		dev_err(&client->dev, "failed to request gpio %d\n", ret);
		goto err_parse_dt;
	}

	ret = nxp_mcu_power_init(lpc, 1);
	if (ret < 0)
		goto err_power_init;

	ret = nxp_mcu_power_set(lpc, 1);
	if (ret < 0)
		goto err_power_set;

    nxp_mcu_reset(lpc);
	 
	mutex_init(&lpc->sensorhub_mutex);
	mutex_init(&lpc->sensorhub_irqmutex);


#ifdef CONFIG_LPC5400X_SENSORHUB_FIRMWARE_UPDATE_I2C
	lpc->isValidVersion = false;
	lpc->isFirmwareChecked = false;
#endif

	lpc->pm_state = RPM_ACTIVE;
    lpc->interrupt_pending = false;
/*
	ret = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
	if (!ret) {
	    dev_err(&client->dev,"*** lpc5400x_probe... error %d\n", ret);
		dev_err(&client->dev, "SMBUS Byte Data not Supported\n");
		ret = -EIO;
		goto err_free_mem;
	}
*/
	dev_dbg(&client->dev,
		"lpc5400x_probe lpc = %p, client = %p, Addr = %d\n",
		lpc, client, client->addr);


#ifdef CONFIG_HAS_EARLYSUSPEND
	lpc->early_suspend.suspend = lpc5400x_early_suspend;
	lpc->early_suspend.resume = lpc5400x_late_resume;
	register_early_suspend(&(lpc->early_suspend));
#endif


#if 0
	lpc->pm_nb.notifier_call = lpc5400x_pm_notifier;
	lpc->pm_nb.priority = INT_MAX;
	register_pm_notifier(&lpc->pm_nb);
#endif


	for (i = 0; i < ARRAY_SIZE(sensorList); i++) {
		if (sensorList[i].sensorId < LPCSH_SENSOR_ID_COUNT) {
			ret = lpc_sensor_register(
				&lpc->sensorHandle[sensorList[i].sensorId],
				client,
				&sensorList[i]);
			dev_dbg(&client->dev,
				"Register sensor %d, return %d\n",
				i,
				ret);
		}
	}

	lpc->wakelk.ws.name = NULL;
	wake_lock_init(
		&lpc->wakelk,
		WAKE_LOCK_SUSPEND,
		"LPC5400x Wake lock");
	//zhangji add 
	wake_lock_init(
		&lpc->resumelock,
		WAKE_LOCK_SUSPEND,
		"LPC5400x resume lock");

	dev_dbg(
		&client->dev,
		"lpc5400x_probe irq = %d,  ret = %d\n",
		client->irq,
		ret);
	device_init_wakeup(&client->dev, 1);

    /*printk("send reset start\n");
	cmd.command = LPCSH_CMD_RESET;
	lpc5400x_send_buf(client, (u8 *) &cmd, sizeof(cmd));*/
    printk("read LPCSH_CMD_WHO_AM_I start\n");

	ret = lpc5400x_read_buf(
		client,
		LPCSH_CMD_WHO_AM_I,
		(u8 *)&lpc->devId,
		sizeof(lpc->devId));

	if (ret < 0) {
		dev_err(&client->dev, "failed to detect device\n");
#ifdef CONFIG_LPC5400X_SENSORHUB_FIRMWARE_UPDATE_I2C
		lpc->isValidVersion = false;
#else
		ret = -ENXIO;
		goto err_free_mem;
#endif
	}

	ret = lpc5400x_read_buf(
		client,
		LPCSH_CMD_GET_VERSION,
		(u8 *)&lpc->devVersion,
		sizeof(lpc->devVersion));

	if (ret < 0) {
		dev_err(&client->dev, "failed to detect device\n");
#ifdef CONFIG_LPC5400X_SENSORHUB_FIRMWARE_UPDATE_I2C
		lpc->isValidVersion = false;
#else
		ret = -ENXIO;
		goto err_free_mem;
#endif
	}

    #ifdef NXP_MCU_ZTE_PROC_FLAG
    nxp_fw_version_id[0] = lpc->devVersion.major_version;
	nxp_fw_version_id[1] = lpc->devVersion.minor_version;
	#endif
    dev_info(&client->dev, "chip id  0x%x version %d.%d\n",
                lpc->devId.param,
                lpc->devVersion.major_version,
                lpc->devVersion.minor_version);
#ifdef CONFIG_LPC5400X_SENSORHUB_VOICE_TRIGGER
	ret = lpc5400x_create_vt_device(&lpc->vt, client);
	if (ret < 0)
        goto err_free_mem;
#endif
#ifdef CONFIG_LPC5400X_SENSORHUB_IR_BLASTER
	ret = lpc5400x_create_ir_blaster_device(&lpc->ir, client);
	if (ret < 0)
        goto err_free_mem;
#endif
#ifdef CONFIG_LPC5400X_SENSORHUB_BREATH_LED
	ret = lpc5400x_create_leds_device(&lpc->breath_led, client);
	if (ret < 0)
        goto err_free_mem;
#endif

#ifdef CONFIG_LPC5400X_SENSORHUB_FIRMWARE_UPDATE_I2C
	if (I2C_WHO_AM_I_ID == lpc->devId.param) {
		lpc->isValidVersion = true;
		lpc->isFirmwareChecked = true;
	}

	//if (lpc->isValidVersion) {
#endif    

	if (lpc->isValidVersion) {
        /* initialize sensorhub/linux time synchronization */
        ret = lpc5400x_init_timeSync(lpc);
        if (ret) {
            dev_err(&client->dev, "Timesync failed\n");
            ret = -ENXIO;
			    goto err_free_mem;
		    }
	}


	i2c_set_clientdata(client, lpc);
    printk("WJP lpc->pdata :%p\n", lpc->pdata);
    //printk("WJP client->dev.platform_data :%p\n", client->dev.platform_data);
    client->dev.platform_data = lpc->pdata; /* FIXME here */

	lpc->vfscallback.vfs_notify = lpc5400x_vfs_notifer;
	lpc5400x_create_vfsdevice(&lpc->vfs, &client->dev, &lpc->vfscallback);

	//lpc5400x_pm_resume(client);                /* tell sensor hub that AP is not suspended */

//zhangji
	get_sensor_list_ok = false;
	sensorlist_info_get(client);

    lpc5400x_ps_calibration_powerup(client);
	
	g_client = client;
	lpc54xx_proc_init();

    #ifdef NXP_MCU_ZTE_PROC_FLAG
	mcu_id_entry = proc_create("driver/mcu_id", 0664, NULL, &nxp_mcu_id);
	if (mcu_id_entry == NULL) {
		pr_err("create nxp_mcu_id fail");
	}
    #endif


	lpc->client->irq = gpio_to_irq(lpc->pdata->irq_gpio);

	printk("--NXP--lpc->pdata->irq_gpio:%d, sensor->client->irq  =%d!!\n", lpc->pdata->irq_gpio, lpc->client->irq);

    if(lpc->client->irq > 0){
        ret = request_threaded_irq(lpc->client->irq, lpc5400x_irq_handler,
                    lpc5400x_readdata,
                    IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					dev_name(&client->dev), client);
    
        if (ret) {
            dev_err(&client->dev, "irq %d busy?\n", client->irq);
            ret = -EBUSY;
            goto err_free_isr;
        }
		
	    #ifdef CONFIG_LPC5400X_SENSORHUB_FIRMWARE_UPDATE_I2C
		if (lpc->isValidVersion) {
			irq_set_irq_wake(client->irq, 1);  /*enable irq*/  
			//enable_irq(client->irq);
		}
		else
		{
			irq_set_irq_wake(client->irq, 0);  /*disable irq*/     				
	        disable_irq(client->irq);  /*disable irq*/
		}
        #else
	    irq_set_irq_wake(client->irq, 1);       /* make IRQ to wake AP */
	    #endif	
	
	 //   irq_set_irq_wake(client->irq, 1);       /* make IRQ to wake AP */
    }
	else{
		pr_err("%s:Failed gpio to irq\n", __func__);
		goto err_free_mem;
	}
	
        
#ifdef CONFIG_LPC5400X_SENSORHUB_FIRMWARE_UPDATE_I2C
	//} 
#if 0
	else {
	    gpio_direction_input(lpc->pdata->wakeup_gpio);
        /* Probe the bus for firmware update */
		if (lpc5400x_probebus(&client->dev, SL_I2C2, lpc->pdata->wakeup_gpio)) {
            ret = -ENXIO;
            dev_err(&client->dev, "Firmware update bus detection failed");
            goto err_free_mem;
        }

		lpc5400x_romVersion(&client->dev, lpc->pdata->wakeup_gpio);
		
        dev_dbg(&client->dev, "lpc5400x_probebus Success\n");
        /**************************************************************/
        ret = 0;
    }
#endif	
#endif

	/* tell sensor hub that AP is not suspended */
	lpc5400x_pm_resume(client, 0);
	pr_info("*** lpc5400x_probe Done\n");
	
	return 0;

err_free_isr:
	free_irq(client->irq, client);

err_free_mem:
	if (lpc) {
		dev_err(
			&client->dev,
			"lpc5400x_probe error , lpc = %p, ret = %d",
			lpc,
			ret);

		for (i = 0; i < ARRAY_SIZE(sensorList); i++) {
			if (sensorList[i].sensorId < LPCSH_SENSOR_ID_COUNT) {
				if (lpc->sensorHandle[sensorList[i].sensorId]) {
					lpc_sensor_unregister(lpc->sensorHandle[sensorList[i].sensorId]);
					dev_dbg(
						&client->dev,
						"Unregister sensorId [%-2.2d] type [%d]\n",
						i,
						sensorList[i].sensorId);
				}
			}
		}

#ifdef LPC5400X_SENSORHUB_VOICE_TRIGGER
		if(lpc->vt)
			lpc5400x_destory_vt_device(lpc->vt);
#endif
#ifdef LPC5400X_SENSORHUB_IR_BLASTER
		if (lpc->ir)
			lpc5400x_destory_ir_blaster_device(lpc->ir);
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&(lpc->early_suspend));
#endif
		//unregister_pm_notifier(&lpc->pm_nb);
		if (lpc->vfs)		/* Need FIXME here */
			lpc5400x_destory_vfsdevice(lpc->vfs);

		if(lpc->wakelk.ws.name)
			wake_lock_destroy(&lpc->wakelk);
		//zhangji add
		if(lpc->resumelock.ws.name)
			wake_lock_destroy(&lpc->resumelock);

    mutex_destroy(&lpc->sensorhub_mutex);
	    mutex_destroy(&lpc->sensorhub_irqmutex);

	nxp_mcu_power_set(lpc, 0);

err_power_set:
	nxp_mcu_power_init(lpc, 0);
err_power_init:
	nxp_unconfig_gpio(lpc);
err_parse_dt:	
    kfree(lpc->pdata);
err_alloc_pdata:	
		kfree(lpc);
	}

	return ret;
}

static int lpc5400x_remove(struct i2c_client *client)
{
	int i;
	struct _lpc5400x *lpc;

	lpc = i2c_get_clientdata(client);


	dev_dbg(&client->dev, "lpc5400x_remove\n");

    #ifdef NXP_MCU_ZTE_PROC_FLAG
	if(mcu_id_entry){
		remove_proc_entry("driver/mcu_id", NULL);
		mcu_id_entry = NULL;
	}
    #endif

	for (i = 0; i < ARRAY_SIZE(sensorList); i++) {
		if (sensorList[i].sensorId < LPCSH_SENSOR_ID_COUNT) {
			if (sensorList[i].name) {
				lpc_sensor_unregister(lpc->sensorHandle[sensorList[i].sensorId]);
				dev_dbg(
					&client->dev,
					"Unregister sensorId [%-2.2d] type [%d]\n",
					i,
					sensorList[i].sensorId);
			}
		}
	}

	device_init_wakeup(&client->dev, 0);

	free_irq(client->irq, client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&(lpc->early_suspend));
#endif

	//unregister_pm_notifier(&(lpc->pm_nb));

    if(lpc->vfs){ /* Need FIXME here */
		lpc5400x_destory_vfsdevice(lpc->vfs);
    }


	if(lpc->wakelk.ws.name)
		wake_lock_destroy(&lpc->wakelk);
	//zhangji add
	if(lpc->resumelock.ws.name)
		wake_lock_destroy(&lpc->resumelock);

    mutex_destroy(&lpc->sensorhub_mutex);
		    mutex_destroy(&lpc->sensorhub_irqmutex);


	kfree(lpc);

	return 0;
}



/* wangjianping 20150209 add NXP sensorhub, start */
static struct of_device_id lpc5400x_match_table[] = {
	{ .compatible = "nxp,sensor-hub",},
	{ },
};
/* wangjianping 20150209 add NXP sensorhub, end */

static const struct i2c_device_id lpc5400x_id[] = { { "lpc5400x_sensorhub", 0 }, { } };

MODULE_DEVICE_TABLE(i2c, lpc5400x_id);
static struct i2c_driver lpc5400x_i2c_driver = {
	.driver		= {
		.name	= "lpc5400x_sensorhub",
		.owner	= THIS_MODULE,
		.pm		= &lpc5400x_pm_ops,
				/* wangjianping 20150209 add NXP sensorhub, start */
				.of_match_table = lpc5400x_match_table, 
				/* wangjianping 20150209 add NXP sensorhub, end */
	},
	.probe		= lpc5400x_probe,
		.remove = lpc5400x_remove,
	.id_table	= lpc5400x_id,
};

static int __init lpc5400x_init(void)
{
	int ret;

	pr_debug("lpc5400x_init\n");

	ret = i2c_add_driver(&lpc5400x_i2c_driver);
	pr_debug("i2c_add_driver(&lpc5400x_i2c_driver) returns %d\n", ret);

	return ret;
}

static void __exit lpc5400x_exit(void)
{
	pr_debug("lpc5400x_exit\n");
	i2c_del_driver(&lpc5400x_i2c_driver);
}

MODULE_AUTHOR("ER Bin");
MODULE_DESCRIPTION("NXP Sensor hub lpc5400x");
MODULE_LICENSE("GPL");

module_init(lpc5400x_init);
module_exit(lpc5400x_exit);

