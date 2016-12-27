#ifndef LPC5400X_VOICE_TRIGGER_H_
#define LPC5400X_VOICE_TRIGGER_H_

#include <linux/device.h>


#define VOICE_TRIGGER_FILE_NAME "vt_user_data"

int lpc5400x_create_vt_device(void **pHandle, struct i2c_client *client);
int lpc5400x_destory_vt_device(void *handle);
int lpc5400x_voicetrigger_control(void *handle, bool enable);

#endif	/* LPC5400X_VOICE_TRIGGER_H_ */

