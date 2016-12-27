/*
 * @brief LPC Sensor Hub communication protocol definition
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2014
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#if !defined(__HOSTIF_PROTOCOL_H__)
#define   __HOSTIF_PROTOCOL_H__

#ifndef __KERNEL__
 #include <stdint.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup SH_HOSTIF Sensor hub: Host interface
 * @ingroup SENSOR_HUB
 * @{
 */

/** Maximum data payload to be transferred to host */
#define HOSTIF_MAX_BUFFER_SIZE 252

/** Virtual Sensor IDs */
/** Must not exceed 0x7f, sincd MS bit is used to mark sample stale */
enum LPCSH_SENSOR_ID {
	LPCSH_SENSOR_ID_FIRST = 0,

	LPCSH_SENSOR_ID_ACCELEROMETER = LPCSH_SENSOR_ID_FIRST,	/*!< Calibrated Accelerometer data */
	LPCSH_SENSOR_ID_ACCELEROMETER_UNCALIBRATED,				/*!< Uncalibrated Accelerometer data */
	LPCSH_SENSOR_ID_MAGNETIC_FIELD,							/*!< Calibrated magnetometer data */
	LPCSH_SENSOR_ID_MAGNETIC_FIELD_UNCALIBRATED,			/*!< Uncalibrated magnetometer data */
	LPCSH_SENSOR_ID_GYROSCOPE,								/*!< Calibrated gyroscope data */

	LPCSH_SENSOR_ID_GYROSCOPE_UNCALIBRATED,					/*!< Uncalibrated gyroscope data */
	LPCSH_SENSOR_ID_LIGHT,									/*!< Light data */
	LPCSH_SENSOR_ID_PRESSURE,								/*!< Barometer pressure data */
	LPCSH_SENSOR_ID_PROXIMITY,								/*!< Proximity data */
	LPCSH_SENSOR_ID_RELATIVE_HUMIDITY,						/*!< Relative humidity data */

	LPCSH_SENSOR_ID_AMBIENT_TEMPERATURE,					/*!< Ambient temperature data */
	LPCSH_SENSOR_ID_GRAVITY,								/*!< Gravity part of acceleration in body frame */
	LPCSH_SENSOR_ID_LINEAR_ACCELERATION,					/*!< Dynamic acceleration */
	LPCSH_SENSOR_ID_ORIENTATION,							/*!< yaw, pitch, roll (also use this for Win8 Inclinometer) */
	LPCSH_SENSOR_ID_ROTATION_VECTOR,						/*!< accel+mag+gyro quaternion */

	LPCSH_SENSOR_ID_GEOMAGNETIC_ROTATION_VECTOR,			/*!< accel+mag quaternion */
	LPCSH_SENSOR_ID_GAME_ROTATION_VECTOR,					/*!< accel+gyro quaternion */
	LPCSH_SENSOR_ID_STEP_DETECTOR,							/*!< Precise time a step occured */
	LPCSH_SENSOR_ID_STEP_COUNTER,							/*!< Count of sensitive steps */
	LPCSH_SENSOR_ID_SIGNIFICANT_MOTION,						/*!< Significant motion detection */

	LPCSH_SENSOR_ID_ACTIVITY_CLASSIFIER_UNKNOWN,			/*!< stability Classification probability for unknown */
	LPCSH_SENSOR_ID_ACTIVITY_CLASSIFIER_IN_VEHICLE,			/*!< stability Classification probability for in vehicle */
	LPCSH_SENSOR_ID_ACTIVITY_CLASSIFIER_ON_BICYCLE,			/*!< stability Classification probability for on bicycle */
	LPCSH_SENSOR_ID_ACTIVITY_CLASSIFIER_ON_FOOT,			/*!< stability Classification probability for on foot */
	LPCSH_SENSOR_ID_ACTIVITY_CLASSIFIER_STILL,				/*!< stability Classification probability for still */

	LPCSH_SENSOR_ID_ACTIVITY_CLASSIFIER_TILTING,			/*!< stability Classification probability for tilting */
	LPCSH_SENSOR_ID_ACTIVITY_CLASSIFIER_WALKING,			/*!< stability Classification probability for walking */
	LPCSH_SENSOR_ID_ACTIVITY_CLASSIFIER_RUNNING,			/*!< stability Classification probability for running */
	LPCSH_SENSOR_ID_ACTIVITY_CLASSIFIER_ON_STAIRS,			/*!< stability Classification probability for on stairs */
	LPCSH_SENSOR_ID_AUG_REALITY_COMPASS,					/*!< heading which switches to aug-reality mode when camera towards horizon (Win8 compass) */

	LPCSH_SENSOR_ID_TAP_DETECTOR,							/*!< precise time a TAP occured */
	LPCSH_SENSOR_ID_SHAKE_DETECTOR,							/*!< precise time a SHAKE occured */
	LPCSH_SENSOR_ID_FLIP_DETECTOR,							/*!< precise time a SHAKE occured */
	LPCSH_SENSOR_ID_PICKUP_DETECTOR,						/*!< precise time a PICK occured */
	LPCSH_SENSOR_ID_STABILITY_DETECTOR,						/*!< precise time a STABILITY occured */

	LPCSH_SENSOR_ID_SLEEP_DETECTOR,							/*!< precise time a SLEEP occured */
	LPCSH_SENSOR_ID_TILT_DETECTOR,							/*!< Precise time a tilt occured */
	LPCSH_SENSOR_ID_IN_POCKET_DETECTOR,						/*!< precise time a In-Pocket occured */
	LPCSH_SENSOR_ID_STABILITY_CLASSIFIER,					/*!< precise time a Mobile Stability reported */

	/*  !!!!  These need to be AFTER all sensors  !!! */

	LPCSH_SENSOR_ID_IR_BLASTER_RX_PACKET,			/*!< IR Blaster RX packet */

	LPCSH_SENSOR_ID_IR_BLASTER_STATUS,				/*!< IR Blaster status */
	LPCSH_SENSOR_ID_META_SENSOR_FLUSH_DONE,			/*!< Meta Sensor ID mask, to indicate Sensor(s) Flush ended */
	LPCSH_SENSOR_ID_VOICE_TRIGGER,					/*!< voice trigger event */
	LPCSH_SENSOR_ID_TIMESTAMP,						/*!< upper bits of sensor timestamp */
	LPCSH_SENSOR_ID_TOUCE_PANEL,
	LPCSH_SENSOR_ID_LOGGING_DATA,					/*!< Logging Service data indicator  */
	LPCSH_SENSOR_ID_AUDIOSENSE,						/*!< audio sense event */
	LPCSH_SENSOR_ID_COUNT,							/*!< max Sensor ID */

	LPCSH_SENSOR_ID_FAKE_SENSOR  =   0x7f,			/*!<  Meta Sensor ID, for FIFO padding when needed */
};

#define ACCEL_DATA_NAME "Accelerometer"
#define UNCALIBRATED_ACCEL_DATA_NAME "Uncalibrated Accelerometer"
#define MAGNETIC_DATA_NAME "Magnetic Field"
#define UNCAL_MAGNETIC_DATA_NAME "Uncalibrated Magnetic Field"
#define ORIENTATION_DATA_NAME "Orientation"
#define GYROSCOPE_DATA_NAME "Gyroscope"
#define UNCAL_GYROSCOPE_DATA_NAME "Uncalibrated Gyroscope"
#define ROTATIONVECTOR_DATA_NAME "Rotation Vector"
#define GAME_ROTATIONVECTOR_DATA_NAME "Game Rotation Vector"
#define GEO_MAGNETIC_ROTATION_VECTOR_DATA_NAME "Geo Magnetic Rotation Vector"
#define LIGHT_DATA_NAME "Light"
#define PROXIMITY_DATA_NAME "Proximity"
#define PRESSURE_DATA_NAME "Pressure"
#define STEP_DETECTOR_NAME "Step Detector"
#define STEP_COUNTER_NAME "Step Counter"
#define LINEAR_ACCELERATION_DATA_NAME "Linear Acceleration"
#define GRAVITY_DATA_NAME "Gravity"
#define SIGNIFICANT_MOTION_DATA_NAME "Significant Motion Detector"
#define AMBIENT_TEMPERATURE_DATA_NAME "Ambient Temperature"
#define RELATIVE_HUMIDITY_DATA_NAME "Relative Humidity"
#define TAP_DETECTOR_DATA_NAME "Tap Detector"
#define SHAKE_DETECTOR_DATA_NAME "Shake Detector"
#define FLIP_DETECTOR_DATA_NAME "Flip Detector"
#define PICKUP_DETECTOR_DATA_NAME "PICKUP Detector"
#define STABILITY_DETECTOR_DATA_NAME "Stability Detector"
#define SLEEP_DETECTOR_DATA_NAME "Sleep Detector"
#define TILT_DETECTOR_DATA_NAME "tilt Detector"
#define IN_POCKET_DETECTOR_DATA_NAME "in pocket Detector"

#define ACTIVITY_CLASSIFIER_UNKNOWN_DATA_NAME "Activity Classifier unknown"
#define ACTIVITY_CLASSIFIER_IN_VEHICLE_DATA_NAME "Activity Classifier in vehicle"
#define ACTIVITY_CLASSIFIER_ON_BICYCLE_DATA_NAME "Activity Classifier on bicycle"
#define ACTIVITY_CLASSIFIER_ON_FOOT_DATA_NAME "Activity Classifier on foot"
#define ACTIVITY_CLASSIFIER_STILL_DATA_NAME "Activity Classifier still"
#define ACTIVITY_CLASSIFIER_TILTING_DATA_NAME "Activity Classifier tilting"
#define ACTIVITY_CLASSIFIER_WALKING_DATA_NAME "Activity Classifier walking"
#define ACTIVITY_CLASSIFIER_RUNNING_DATA_NAME "Activity Classifier running"
#define ACTIVITY_CLASSIFIER_ON_STAIRS_DATA_NAME "Activity Classifier on stairs"

#define STABILITY_CLASSIFIER_DATA_NAME "Stability Classifier"

#define VOICE_TRIGGER_DATA_NAME "voice trigger"
#define AUDIO_SENSE_DATA_NAME "audio sense"

#define IR_BLASTER_LEARNING_DATA_NAME "ir blaster learning"
#define IR_BLASTER_STATUS_DATA_NAME "ir blaster status"

#define SENSOR_MANUFACTURER "NXP Semiconductors"

/*-------------------------------------------------------------------------------------------------*\
|    T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

#define packed_struct  struct
#define packed_union  union
#pragma pack(push)
#pragma pack(1)

enum lpcsh_accuracy_e {
	lpcsh_accuracy_unreliable = 0,
	lpcsh_accuracy_low,
	lpcsh_accuracy_medium,
	lpcsh_accuracy_high
};

/**
 * @brief Generic sensor structure for Accel, Mag, Gyro etc
 */
packed_struct lpcsh_motion_sensor_node {
	int16_t Data[3];	/*!< Sensor data for X, Y, Z axis */
	uint8_t Accuracy;	/*!< Sensor accuracy per enum lpcsh_accuracy_e*/
};

/**
 * @brief Generic uncalibrated sensor structure with Bias
 */
packed_struct lpcsh_motion_uncal_sensor_node {
	int16_t Data[3];	/*!< Sensor data for X, Y, Z axis */
	int16_t Bias[3];	/*!< Bias data for X, Y, Z axis */
	uint8_t Accuracy;	/*!< Sensor accuracy per enum lpcsh_accuracy_e*/
};

/**
 * @brief Proximity data structure
 */
packed_struct lpcsh_proximity_sensor_node {
	int16_t Data;	/*!< Proximity distance data */
};

/**
 * @brief Ambient Light data structure
 */
packed_struct lpcsh_light_sensor_node {
	int32_t Data;	/*!< Light intensity data */
};

/**
 * @brief Temperature data structure
 */
packed_struct lpcsh_ambient_temperature_sensor_node {
	int16_t Data;	/*!< Temperature */
};

/**
 * @brief Pressure data structure
 */
packed_struct lpcsh_pressure_sensor_node {
	int32_t Data;	/*!< Pressure */
};

/**
 * @brief Humidity data structure
 */
packed_struct lpcsh_humidity_sensor_node {
	int16_t Data;	/*!< Humidity */
};

/**
 * @brief Significant motion data structure
 */
packed_struct lpcsh_significant_motion_node {
	unsigned char significantMotionDetected;	/*!< Boolean to indicate if significant motion happened */
};

/**
 * @brief Flush Done Meta data structure
 */
packed_struct lpcsh_meta_sensor_flush_done_node {
	uint8_t sensorId;	/*!< enum LPCSH_SENSOR_ID */
};
/**
 * @brief Logging Service Meta data structure
 */
packed_struct lpcsh_meta_sensor_log_serv_data_avail_node {
	uint8_t unused;
};

/**
 * @brief Rotation Vector structure
 */
packed_struct lpcsh_quaternion_node {
	int16_t W;			/*!< Rotation Vector data for W axis */
	int16_t X;			/*!< Rotation Vector data for X axis */
	int16_t Y;			/*!< Rotation Vector data for Y axis */
	int16_t Z;			/*!< Rotation Vector data for Z axis */
	int16_t E_EST;
	uint8_t Accuracy;	/*!< Sensor accuracy per enum lpcsh_accuracy_e*/
};

/**
 * @brief Step Counter data structure
 */
packed_struct lpcsh_step_sensor_node {
	uint32_t numTotalsteps;	/*!< Total number of steps counted */
};

/**
 * @brief Tap Detector data structure
 */
packed_struct lpcsh_tap_detector_sensor_node {
	uint8_t state;	/*!< Tap state */
};

/**
 * @brief mobile stability classification data structure
 */
packed_struct lpcsh_stability_classifier_sensor_node {
	uint8_t classification;	/*!< Shake state */
	uint8_t Accuracy;		/*!< Sensor accuracy per enum lpcsh_accuracy_e*/
};

/**
 * @brief Shake detector data structure
 */
packed_struct lpcsh_shake_detector_sensor_node {
	uint8_t detected;		/*!< Shake state */
};

/**
 * @brief Tilt detector data structure
 */
packed_struct lpcsh_tilt_detector_sensor_node {
	uint8_t detected;		/*!< Tilt state */
};

/**
 * @brief Flip detector data structure
 */
packed_struct lpcsh_flip_detector_sensor_node {
	uint8_t detected;		/*!< flip state */
};

/**
 * @brief Pick Up detector data structure
 */
packed_struct lpcsh_pickup_detector_sensor_node {
	uint8_t detected;		/*!< pick up  state */
};

/**
 * @brief Shake detector data structure
 */
packed_struct lpcsh_sleep_detector_sensor_node {
	uint8_t detected;		/*!< sleep state */
};

/**
 * @brief stability classifier data structure
 */
packed_struct lpcsh_stability_detector_node {
	uint8_t detected;	/*!< stability detected data */
};

/**
 * @brief stability classifier data structure
 */
packed_struct lpcsh_stability_detector_sensor_node {
	uint8_t detected;	/*!< stability detected data */
};

/**
 * @brief stability classifier data structure
 */
packed_struct lpcsh_activity_classifier_probability_node {
	uint8_t probability;	/*!< stability classifier probability */
};

/**
 * @brief Voice Trigger data structure
 */
packed_struct lpcsh_voice_trigger_node {
	uint8_t index;	/*!< Trigger index */
};

/**
 * @brief Touch Panel data event
 */
packed_struct lpcsh_touch_panel_node {
	uint8_t detected;
};

/** Max IR Blaster Sub Block data size */
#define IR_BLASTER_RX_SUB_BLOCK_MAX_SIZE   64				/* max of 255 while size is uint8_t */

/**
 * @brief IR Blaster Rx data structure
 */
packed_struct lpcsh_ir_blaster_rx_node {
	uint8_t size;											/*!< size of user data sub-block - max 255 */
	uint8_t data[IR_BLASTER_RX_SUB_BLOCK_MAX_SIZE];			/*!< sub block data array, limited to max Host xfer size */
};

/** IR status reporting enums */
enum IR_DRVR_STATUS {
	IR_DRVR_TX_DATA_REQUESTED,
	IR_DRVR_TX_DONE,
	IR_DRVR_RX_DATA_AVAILABLE,
	IR_DRVR_RX_LEARNING_SUCCESS,
	IR_DRVR_RX_LEARNING_FAILED,
};

/**
 * @brief IR Blaster Status data structure
 */
packed_struct lpcsh_ir_blaster_status_node {
	uint8_t status;			/*!< status code enum IR_DRVR_STATUS */
	uint16_t param;			/*!< status parameter */
};

/**
 * @brief In Pocket detector data structure
 */
packed_struct lpcsh_in_pocket_detector_sensor_node {
	uint8_t inPocket;		/*!< In-pocket state 1: in pocket, 2 : out of pocket*/
};

packed_struct lpcsh_full_timestamp_node {
	uint32_t timestampUpper32;	/*!< Upper 32 bits of the timestamp, lower 16 bits are embedded in the header */
};

/**
 * @brief Logging Service query service status structure
 */
packed_struct lpcsh_LogServ_query_service_node {
	uint8_t status;											/*!< status is returned  */
};

#define AUDIO_SENSE_DATA_BLOCK_MAX 2

/**
 * @brief AudioSense data structure
 */
packed_struct lpcsh_audiosense_node {
	uint32_t result[AUDIO_SENSE_DATA_BLOCK_MAX];
	int32_t ambNoiseLevel;
	uint8_t suggestedRingtoneLevel;
};

/**
 * @brief Virtual Sensor Data Header
 */
packed_struct lpcsh_sensor_node_header {
	uint8_t sensorId;	/*!< enum LPCSH_SENSOR_ID, if MS bit is set, sample is stale */
	uint8_t length;		/*!< length of data and header */
	uint16_t timeStamp;	/*!< raw time stamp */
};

/**
 * @brief Virtual Sensor Output Data for Host Interface
 */
packed_struct lpcsh_sensor_node {
	struct lpcsh_sensor_node_header header;														/*!< Header for Sensor Data */

	union {
		struct lpcsh_motion_sensor_node sensorData;												/*!< Generic sensor structure for Accel, Mag, Gyro etc */

		struct lpcsh_motion_uncal_sensor_node uncal_sensorData;									/*!< Generic uncalibrated sensor structure with Bias */

		struct lpcsh_quaternion_node quaternionData;											/*!< Rotation Vector structure */

		struct lpcsh_step_sensor_node stepData;													/*!< Step Counter data structure */

		struct lpcsh_significant_motion_node significantMotionData;								/*!< Significant motion data structure */

		struct lpcsh_light_sensor_node lightData;												/*!< Ambient Light data structure */

		struct lpcsh_pressure_sensor_node pressureData;											/*!< Pressure data structure */

		struct lpcsh_proximity_sensor_node proximityData;										/*!< Proximity data structure */

		struct lpcsh_humidity_sensor_node humidityData;											/*!< Humidity data structure */

		struct lpcsh_ambient_temperature_sensor_node ambientTemperatureData;					/*!< Temperature data structure */

		struct lpcsh_stability_detector_node stabilityDetectorData;								/*!< stability detector data structure */

		struct lpcsh_activity_classifier_probability_node stabilityClassifierProbabilityData;	/*!< stabilityity classifier data structure */

		struct lpcsh_tap_detector_sensor_node tapDetectorData;									/*!< Tap Detector data structure */

		struct lpcsh_shake_detector_sensor_node shakeDetectorData;								/*!< Shake detector data structure */

		struct lpcsh_flip_detector_sensor_node flipDetectorData;								/*!< Flip detector data structure */

		struct lpcsh_pickup_detector_sensor_node pickUpDetectorData;							/*!< Pick detector data structure */

		struct lpcsh_sleep_detector_sensor_node sleep_detectorData;								/*!< Sleep detector data structure */

		struct lpcsh_meta_sensor_flush_done_node flushDoneData;									/*!< Flush Done meta data structure */

		struct lpcsh_voice_trigger_node voiceTriggerData;										/*!< Voice Trigger data structure */

		struct lpcsh_ir_blaster_rx_node irBlasterRxData;										/*!< IR Blaster Rx data structure */

		struct lpcsh_ir_blaster_status_node irBlasterStatusData;								/*!< IR Blaster Status data structure */

		struct lpcsh_tilt_detector_sensor_node tiltDetectorData;								/*!< Tilt detector data structure */

		struct lpcsh_in_pocket_detector_sensor_node inPocketDetectorData;						/*!< Pocket detector data structure */

		struct lpcsh_stability_classifier_sensor_node stabilityClassifierData;					/*!< stability classification data structure */

		struct lpcsh_full_timestamp_node fullTimestamp;

		struct lpcsh_touch_panel_node touchpanelData;											/*!< touch panel data structure */

		struct lpcsh_audiosense_node audiosenseData;											/*!< AudioSense data structure */

	} data;

};

packed_struct lpcsh_hal_sensor_node {
	uint64_t timeStamp;
	struct lpcsh_sensor_node node;
};

packed_struct lpcsh_meta_sensor_flush_done_full_node
{
	struct lpcsh_sensor_node_header header;													/*!< Header for Sensor Data */
	struct lpcsh_meta_sensor_flush_done_node flushDoneData;									/*!< Flush Done meta data structure */
};

packed_struct ShCmdGetHeader_get_8bits_param_t {
	uint8_t param;
};

packed_struct ShCmdGetHeader_get_16bits_param_t {
	uint16_t param;
};

#define LPCSH_MAX_CMD_LENGTH            30	/*!< Maximum command packet size */

/** Host I/F command ID enums */
enum LPCSH_CMD_ID_T {
	LPCSH_CMD_WHO_AM_I = 0x00,			/*!< Provide 8 bits Device ID */
	LPCSH_CMD_GET_VERSION,				/*!< 2 byte response. Byte 0 major version, byte_! minor version */
	LPCSH_CMD_RESET,					/*!< Resets host interface */

	LPCSH_CMD_GET_DATA_LENGTH,			/*!< Command used by I2C hostinterface only to get notification data length */
	LPCSH_CMD_GET_DATA,					/*!< Command used by I2C hostinterface only to get sensor hub notification data. */

	LPCSH_CMD_SET_SUSPEND,				/*!< Command to indicate AP is suspended  */
	LPCSH_CMD_SET_RESUME,				/*!< Command to indicate AP is active     */
	LPCSH_CMD_GET_SENSOR_LIST,			/*!< gets names of sensor devices detected on i2c bus */

	LPCSH_CMD_GET_TIME_SYNCHRONIZE,		/*!< gets current micro-second timestamp */
	LPCSH_CMD_GET_SENSOR_CONFIGURATION,	/*!< gets sensor hub configuration data */

	LPCSH_CMD_SENSOR_ENABLE = 0x20,		/*!< Command to enable virtual sensors. */
	LPCSH_CMD_GET_SENSOR_STATE,			/*!< Command to get virtual sensors state. */
	LPCSH_CMD_SET_DELAY,				/*!< Command to set sensor sample rate. */
	LPCSH_CMD_GET_DELAY,				/*!< Command to get sensor sample rate. */

	LPCSH_CMD_SET_BATCH,				/*!< Command to set sensor batch Latency. */
	LPCSH_CMD_SET_FLUSH,				/*!< Command to request flush of sensor data. */

	LPCSH_CMD_SET_CALIBRATE,
	LPCSH_CMD_GET_CALIBRATE,

	LPCSH_CMD_TOUCH_PANEL_ENABLE,
	LPCSH_CMD_TOUCH_PANEL_DISABLE,

	LPCSH_CMD_ACCEL_CALIBRATE,			/*!< Command to calibrate Accelemeter sensor. */
	LPCSH_CMD_ACCEL_READ_CALIBRATEDATA,	/*!< Command to read new calibrated offset. */

	LPCSH_FORCE_APP_ENTRY = 0x30,		/*!< Forces app to start the secondary loader */

	LPCSH_VOICE_TRIGGER_WRITE = 0x40,	/*!< Command to write voice trigger user data */
	LPCSH_VOICE_TRIGGER_WRITE_LAST,		/*!< Command to write voice trigger user data last record */
	LPCSH_VOICE_TRIGGER_DISABLE,		/*!< Command to stop voice trigger process */
	LPCSH_VOICE_TRIGGER_ENABLE,			/*!< Command to start voice trigger process */

	LPCSH_IR_BLASTER_START_TX = 0x50,	/*!< Command to transmit first IR blaster block */
	LPCSH_IR_BLASTER_NEXT_TX,			/*!< Command to transmit next IR blaster block */
	LPCSH_IR_BLASTER_LEARN_START,		/*!< Command to trigger IR blaster learning process */
	LPCSH_IR_BLASTER_RESET,				/*!< Command to abort any IR blaster operation */
	LPCSH_IR_BLASTER_CANCEL_SENDING,
	LPCSH_IR_BLASTER_CANCEL_LEARNING,

	LPCSH_BREATHING_LED = 0x60,				/*!< Command to start/stop breathing LED */
	LPCSH_AUDIOSENSE_SET_DMIC_CLOCK,
	LPCSH_AUDIOSENSE_GET_DMIC_CLOCK,

	LPCSH_CMD_LOG_SERV_QUERY_SERV = 0x78,	/*!< Command to Query whether Logging service is available   */
	LPCSH_CMD_LOG_SERV_CONTROL,				/*!< Command to control Logging service. Per module, disable or set log levels */
	LPCSH_CMD_LOG_SERV_CONFIGURE,			/*!< Command to configure Logging service */

	LPCSH_CMD_WRITE_SENSOR_REGISTER,		/*!< Command to perform diagnostic sensor register(s) write sequence, and
											    read a status byte as a response. response byte will be 0 if command not executed
											    other wise will contain the 'length' value.
											    Host to perform 'write' (to pass command structure), and a 'read' of "1" byte*/
	LPCSH_CMD_READ_SENSOR_REGISTER,			/*!< Command to perform diagnostic sensor register(s) read sequence, and
											    read a status byte as a response. response byte will be 0 if command not executed
											    other wise will contain the 'length' value. data read to follow status byte
											    Host to perform 'write' (to pass command structure), and a 'read' of "length + 1"
											    bytes */
};

/**
 * @brief Host Interface Command Structure
 */
packed_struct LPCSH_CMD_t {
	uint8_t id;										/*!< Command Id from enum LPCSH_CMD_ID_T */
	uint8_t params[LPCSH_MAX_CMD_LENGTH - 1];		/*!< Command Parameters */
};

packed_struct ShHubCmdHeader_t {
	uint8_t command;	/* enum LPCSH_CMD_ID_T */
};

packed_struct ShHubCmdHeader_8bits_param_t {
	uint8_t command;	/* enum LPCSH_CMD_ID_T */
	uint8_t param;
};

packed_struct ShSensorCmdHeader_t {
	uint8_t command;	/* enum LPCSH_CMD_ID_T */
	uint8_t sensorId;	/* enum LPCSH_SENSOR_ID */
};

packed_struct ShSensorSetCmdHeader_8bits_param_t {
	uint8_t command;	/* enum LPCSH_CMD_ID_T */
	uint8_t sensorId;	/* enum LPCSH_SENSOR_ID */
	uint8_t param;
};

packed_struct ShSensorSetCmdHeader_16bits_param_t {
	uint8_t command;	/* enum LPCSH_CMD_ID_T */
	uint8_t sensorId;	/* enum LPCSH_SENSOR_ID */
	uint16_t param;
};

packed_struct ShSensorSetCmdHeader_32bits_param_t {
	uint8_t command;	/* enum LPCSH_CMD_ID_T */
	uint8_t sensorId;	/* enum LPCSH_SENSOR_ID */
	uint32_t param;
};

packed_struct ShSensorSetCmdHeader_batch_t {
	uint8_t command;	/* enum LPCSH_CMD_ID_T */
	uint8_t sensorId;	/* enum LPCSH_SENSOR_ID */
	int flags;
	uint32_t period_us;
	uint32_t timeout_us;
};

packed_struct ShSensorSetCmdHeader_flush_t {
	uint8_t command;	/* enum LPCSH_CMD_ID_T */
	uint8_t sensorId;	/* enum LPCSH_SENSOR_ID */
};

packed_struct ShSensorSetCmdHeader_enable_t {
	uint8_t command;	/* enum LPCSH_SENSOR_ID_COMMANDS */
	uint8_t sensorId;	/* enum LPCSH_SENSOR_ID_ID */
	uint8_t enable;
};

packed_struct LPCSH_DataLen_t {
	uint16_t dataLen;
};

/** Max Voice Trigger Sub Block Size */
#define VOICE_TRIGGER_SUB_BLOCK_MAX_SIZE 128			/* size aligned to flash bank size on LPC5400x */

/**
 * @brief Voice Trigger Write Data structure
 */
packed_struct ShVoiceTriggerWriteUserDataCommand_t {
	uint8_t command;								/*!< enum LPCSH_VOICE_TRIGGER_WRITE */
	uint32_t offset;								/*!< offset of user data sub-block*/
	uint16_t size;									/*!< size of user data sub-block*/
	uint8_t data[VOICE_TRIGGER_SUB_BLOCK_MAX_SIZE];	/*!< data of current sub-block */
};

packed_struct ShIrBlasterCommand_t {
	uint8_t command;					/* enum LPCSH_IR_BLASTER_TX   */
};

/**
 * @brief IR Blaster Tx header structure
 */
packed_struct lpcsh_ir_blaster_tx_data_header_t {
	uint8_t command;					/*!< enum LPCSH_IR_BLASTER_TX   */
	uint16_t size;						/*!< size of user data sub-block - max 16k*/
};

/** Max IR Blaster Tx Sub Block Size */
#define IR_BLASTER_TX_SUB_BLOCK_MAX_SIZE   (HOSTIF_MAX_BUFFER_SIZE - \
											sizeof(packed_struct lpcsh_ir_blaster_tx_data_header_t))

/**
 * @brief IR Blaster Tx Command structure
 */
packed_struct lpcsh_ir_blaster_tx_command_t {
	struct     lpcsh_ir_blaster_tx_data_header_t hdr;		/*!< IR message header   */
	uint8_t data[IR_BLASTER_TX_SUB_BLOCK_MAX_SIZE];			/*!< data array to be max Host xfer size */
};

/**
 * @brief Breathing LED start/stop command structure
 */
packed_struct lpcsh_breathing_led_command_t {
	uint8_t command;									/*!< enum LPCSH_BREATHING_LED   */
	uint8_t brightness;									/*!< brighness of breathing LED. 0x00 (off) to 0xff (full brightness) */
	uint8_t channelMask;								/*!< 0 to stop breathing LED, 1-7 to enable breathing LED on channel 0/1/2 */
	uint16_t t0;										/*!< ramp-up time in ms */
	uint16_t t1;										/*!< on time in ms */
	uint16_t t2;										/*!< ramp-down time in ms */
	uint16_t t3;										/*!< off time in ms */
};

#define LPCSH_SENSORCALIBRATION_DATA_SIZE 100	/* 25 Words */
packed_struct ShSensorSetCalibration_param_t {
	uint8_t command;							/*!< enum LPCSH_CMD_SET_CALIBRATE   */
	uint8_t data[LPCSH_SENSORCALIBRATION_DATA_SIZE];
};

#define AUDIO_SENSE_DMIC_PARAMS_MAX 2

/**
 * @brief audiosense Dmic Command structure
 */
packed_struct ShSensorAudiosense_t {
	uint8_t command;									/*!< enum LPCSH_AUDIOSENSE_GET_DMIC_CLOCK   */
	uint16_t data[AUDIO_SENSE_DMIC_PARAMS_MAX];
};

/**
 * @brief Logging Service query command structure
 */
packed_struct lpcsh_logServ_cmd_t {
	uint8_t command;					/* enum    */
};

/**
 * @brief Logging Service query response structure
 */
packed_struct lpcsh_logServ_query_resp_t {
	uint8_t status;					/* boolean    */
};

/**
 * @brief Enumerations for Logging Service logging levels
 */
typedef enum {
	LOG_SERVICE_LVL0 = 0,
	LOG_SERVICE_LVL1,
	LOG_SERVICE_LVL2,
	LOG_SERVICE_LVL3,
	LOG_SERVICE_LVL4,
	LOG_SERVICE_LVL5,

	LOG_SERVICE_LVL_COUNT,

	LOG_SERVICE_LVL_NONE = LOG_SERVICE_LVL_COUNT
} LOG_SERVICE_LogLevel;

/**
 * @brief Logging Service set module's log level (or disable it) command structure
 */
packed_struct lpcsh_logServEnable_cmd_t {
	uint8_t command;											/*!< command (LPCSH_CMD_LOG_SERV_CONTROL)  */
	uint8_t moduleId;											/*!< log service module id      */
	uint8_t logLevel;											/*!< which logLevel  to enable/disable (using LOG_SERVICE_LVL_NONE)  module  */
};

/**
 * @brief Configure Logging Service command structure
 */
packed_struct lpcsh_logServConfigure_cmd_t {
	uint8_t command;											/*!< command (LPCSH_CMD_LOG_SERV_CONFIGURE)  */
	uint16_t bufferSize;										/*!< kernel fifo size */
	uint8_t logLevel;											/*!< which logLevel  to enable/disable (using LOG_SERVICE_LVL_NONE)  all modules  */
};

/**
 * @brief header installed in front of every Logging Service sub-packet, going to Host
 */
packed_struct lpcsh_logServHeader_t
{
	uint8_t header;							/*!< must be a byte, and must be first byte to comply with sensor header */
	uint16_t size;							/*!< data + header , DOES NOT (and need not) comply with sensor header */
};

#define LOG_SERV_OPCODE_MASK    0x007f		/*!< max of 127 opcodes per module (0..127) */
#define LOG_SERV_32_BIT_PARAMS_MASK 0x0080	/*!< set if 32 bit params, otherwise 16 bit params. don't care of no params */

#define LOG_SERV_MAX_NUM_MODULES  16		/*!< max of 16 modules (0..15) */
#define LOG_SERV_MODULE_BITS 0xf
#define LOG_SERV_MODULE_POS  8
#define LOG_SERV_MODULE_MASK ((uint16_t) (LOG_SERV_MODULE_BITS << LOG_SERV_MODULE_POS))

#define  LOG_SERV_MAX_NUM_PARAMS  7			/*!< max of 7 parameters (0..7) */

#define LOG_SERV_NUM_PARAM_BITS 0x7
#define LOG_SERV_NUM_PARAM_POS  12
#define LOG_SERV_NUM_PARAM_MASK ((uint16_t) (LOG_SERV_NUM_PARAM_BITS << LOG_SERV_NUM_PARAM_POS))

#define LOG_SERV_TIMESTAMP_MASK 0x8000

static inline uint16_t PACK_LOG_SERV_HEADER(uint8_t module,
											uint8_t opcode,
											uint8_t numParams,
											bool sizeParams,
											bool timeStampFlag)
{
	uint16_t val = (uint16_t) (opcode & LOG_SERV_OPCODE_MASK);

	val |= (uint16_t) ((module & LOG_SERV_MODULE_BITS) << LOG_SERV_MODULE_POS);
	val |= (uint16_t) ((numParams & LOG_SERV_NUM_PARAM_BITS) << LOG_SERV_NUM_PARAM_POS);
	val |= (uint16_t) (sizeParams ? LOG_SERV_32_BIT_PARAMS_MASK : 0);
	val |= (uint16_t) (timeStampFlag ? LOG_SERV_TIMESTAMP_MASK : 0);
	return val;
}

static inline void UNPACK_LOG_SERV_HEADER(uint16_t header,
										  uint8_t *module,
										  uint8_t *opcode,
										  uint8_t *numParams,
										  bool *sizeParamsFlag,
										  uint8_t *timeStampArgs)
{
	*opcode = header & LOG_SERV_OPCODE_MASK;
	*module = (header & LOG_SERV_MODULE_MASK) >> LOG_SERV_MODULE_POS;
	*numParams = (header & LOG_SERV_NUM_PARAM_MASK) >> LOG_SERV_NUM_PARAM_POS;
	*sizeParamsFlag = (header & LOG_SERV_32_BIT_PARAMS_MASK) ? true : false;
	*timeStampArgs = (header & LOG_SERV_TIMESTAMP_MASK) ? 3 : 0;
}

#define LPCSH_PARAMS_SIZE 32/* 32 bytes */
packed_struct ShSensor_RegisterDump_t {
	uint8_t  command;									/*!< enum LPCSH_CMD_WRITE_REGISTER */
	uint8_t  i2cAddr;
	uint8_t  sensorRegister;
	uint8_t  length;
	uint8_t  data[LPCSH_PARAMS_SIZE];
};

packed_union ShCmdHeaderUnion {
	struct ShSensorCmdHeader_t command;

	struct ShSensorSetCmdHeader_8bits_param_t sensor_cmd_8bits_param;

	struct ShSensorSetCmdHeader_16bits_param_t sensor_cmd_16bits_param;

	struct ShSensorSetCmdHeader_32bits_param_t sensor_cmd_32bits_param;

	struct ShHubCmdHeader_t hubCmdHeader;

	struct ShHubCmdHeader_8bits_param_t hub_cmd_8bits_param;

	struct ShSensorSetCmdHeader_batch_t sensor_cmd_batch;

	struct ShVoiceTriggerWriteUserDataCommand_t voice_trigger_write_user_data_sub_block;

	struct lpcsh_ir_blaster_tx_command_t ir_blaster_tx_command;

	struct ShSensorSetCmdHeader_enable_t enable_command;

	struct ShSensorSetCalibration_param_t calibration_param;

	struct lpcsh_logServEnable_cmd_t logServ_enable;

	struct lpcsh_LogServ_query_service_node logServ_query;

	struct lpcsh_logServConfigure_cmd_t logServ_configure;

	struct ShSensorAudiosense_t audiosense_clock_param;

	struct ShSensor_RegisterDump_t dump_register_param;

};

/* defines data structure for getting framework time stamp */
packed_struct ShHubTimeSynchronizeData_t {
	uint64_t timeStamp;
};

/* defines data structure for HAL setting "batch" command via sysfs */
packed_struct batchCommandParams {
	uint32_t flag;
	uint32_t samplerate_us;
	uint32_t timeoutenc_us;
};

/**
 * @brief Sensor Hub Version structure
 */
packed_struct lpcsh_version_t {
	uint8_t major_version;			/*!< sensor hub firmware major version number */
	uint8_t minor_version;			/*!< sensor hub firmware minor version number */
};

/**
 * @brief Sensor Hub configuration data structure
 */
packed_struct lpcsh_configuration_data_t {
	uint64_t wakeup_mask;		/*!< sensor hub wakeup sensor configuration bit map per enum LPCSH_SENSOR_ID */
};

/**
 * @brief Sensor Hub Fifo Length structure
 */
packed_struct lpcsh_fifo_length_t {
	uint16_t fifo_size;				/*!< max size of sensor hub FIFO */
};

#define LPCSH_CMD_GET_SENSOR_LIST_MAX_SENSOR_LIST_SIZE (10)

packed_struct lpcsh_sensor_info_t {
	uint8_t deviceType;		/* per enum sensorSetGuid_e */
	uint8_t sensorType;		/* per enum LPCSH_SENSOR_ID */
};

packed_struct lpcsh_get_sensor_list_t {
	uint8_t size;
	struct lpcsh_sensor_info_t sensorInfo[LPCSH_CMD_GET_SENSOR_LIST_MAX_SENSOR_LIST_SIZE];		/*!< List of sensor names */
};

#pragma pack(pop)

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif	/* __HOSTIF_PROTOCOL_H__ */

