#ifndef SENSOR_CORE_H_
#define SENSOR_CORE_H_

#include <linux/lpcsh/lpc5400x.h>
#include "hostif_protocol.h"

#ifdef CONFIG_IIO_KFIFO_BUF

 #include <linux/iio/kfifo_buf.h>
 #define DEFAULT_RINGBUFFERLENGTH 1000

#else

 #include "../ring_sw.h"
 #define DEFAULT_RINGBUFFERLENGTH 10

#endif


#define MAX_RINGBUFFERLENGTH 5000


#define SENSOR_FLAG_METADATA 0xFF

struct sensor_config {
	const char *name;
	u8 sensorId;
	int samplebytes;
};


int lpc_sensor_register(void **pSenbase, struct i2c_client *client, struct sensor_config *sencfg);
void lpc_sensor_unregister(void *senbaseP);
int lpc_sensor_newdata(struct i2c_client *client, void *senbaseP, struct lpcsh_sensor_node  *node);
int lpc_sensor_get_result_length(void *senbaseP);


#endif	/* SENSOR_CORE_H_ */
