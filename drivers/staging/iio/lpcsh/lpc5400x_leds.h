#ifndef LPC5400X_LED_H_
#define LPC5400X_LED_H_

#include <linux/device.h>
#include "hostif_protocol.h"



int lpc5400x_create_leds_device(void **pHandle, struct i2c_client *client);
int lpc5400x_destory_leds_device(void *handle);



#endif /* LPC5400X_LED_H_ */


