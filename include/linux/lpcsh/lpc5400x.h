#ifndef LPC5400X_H_
#define LPC5400X_H_

#include <linux/i2c.h>
#include <linux/regulator/consumer.h>

#define LPC5400X_MAX_I2C_TX_TRANSFER_SIZE (252)
#define LPC5400X_MAX_I2C_TX_PAYLOAD_SIZE (128)
#define LPC5400X_MAX_I2C_RX_TRANSFER_SIZE (252)
#define LPC5400X_MAX_I2C_RX_PAYLOAD_SIZE (128)


struct lpc5400x_platform_data
{
    int gpio;
    int irq_gpio;
	int wakeup_gpio;
	int reset_gpio;
	int ir_en_gpio;
	int ir_sw_gpio;
    int support_firmware;
	int	power_enabled;
    struct regulator	*vio;
    struct regulator	*vdd;
};


struct lpc5400x_bus_ops {
	u16 bustype;
	int (*read)(struct i2c_client *client, u8 reg);
	int (*read_block)(struct i2c_client *client, u8 reg, u8 *buffer,
			u8 length);
	int (*write)(struct i2c_client *client, u8 reg, u8 val);
	int (*write_block)(struct i2c_client *client, u8 reg, u8 *buffer,
			u8 length);
};

enum lpc5400x_event {
	LPC5400X_EVENT_TOUCH_PANEL,
};


struct _lpc5400x_listener_block {
	struct list_head link;
	int (*notify)(void* pContext, int message, void *data);
	void *pContext;
};


int flushfifo_request(struct i2c_client *i2cclient, int sensortype);

#define LPC_READ(sensor, reg) \
	((sensor)->bops->read((sensor)->client, reg))
#define LPC_READ_BLOCK(sensor, buf, size) \
	((sensor)->bops->read_block((sensor)->client, buf, size))
#define LPC_WRITE(sensor, reg, val) \
	((sensor)->bops->write((sensor)->client, reg, val))

int lpc5400x_sensor_samplerate(
    struct i2c_client *client,
    int sensorType,
    uint32_t value);


int lpc5400x_sensor_batch(
    struct i2c_client *client,
    int sensorType,
    int flag,
    u32 batchPeriodUsec,
    u32 batchTimeoutUsec);

int lpc5400x_check_firmware(
    struct i2c_client *pClient);

int lpc5400x_irblaster_transmit(
    struct i2c_client *client,
    u8 *pData,
    u16 size,
    bool last);

int lpc5400x_irblaster_receive(
    struct i2c_client *client,
    u8* pdata,
    u16 size);

int lpc5400x_irblaster_checkRX(struct i2c_client *client);
int lpc5400x_sensor_enable(
    struct i2c_client *client,
    int sensorType,
    bool enable);

int lpc5400x_report_sensorlistinfo(unsigned int request,struct i2c_client *client);
int lpc5400x_getVersion(struct device *dev,
						uint8_t *major_version, uint8_t *minor_version);

int lpc5400x_startCalibration(struct device *dev, unsigned char *pData, int size);
int lpc5400x_getCalibrationData(struct device *dev, unsigned char *pData, int size);

uint64_t lpc5400x_get_synced_timestamp(struct i2c_client *client, uint64_t timestampIn);
int lpc5400x_registerListener(struct _lpc5400x_listener_block *listener);
int lpc5400x_unregisterListener(struct _lpc5400x_listener_block *listener);
#ifdef CONFIG_LPC5400X_SENSORHUB_AUDIO_SENSE
int lpc5400x_setDmicClock(struct device *dev, void* pData, int size);
int lpc5400x_getDmicClock(struct device *dev, void* pData, int size);
#endif

int lpc5400x_write_sensor_register(struct device *dev, void* pData, int size);
int lpc5400x_read_sensor_register(struct device *dev, void* pData, int size);


#endif /*  LPC5400X_H_ */

