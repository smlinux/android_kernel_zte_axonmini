#ifndef LPC5400X_IR_BLASTER_H_
#define LPC5400X_IR_BLASTER_H_

#include <linux/device.h>
#include "hostif_protocol.h"


#define IR_BLASTER_TX_FILE_NAME "ir_blaster_tx"
#define IR_BLASTER_LEARN_DATA_LENGTH_MAX    1024

int lpc5400x_create_ir_blaster_device(
	void **pHandle,
	struct i2c_client *client);

int lpc5400x_destory_ir_blaster_device(void *handle);

int lpc5400x_tx_next_ir_blaster_buffer(
	void *pHandle,
	struct i2c_client *client);

int lpc5400x_rx_next_ir_blaster_buffer(
	void *pHandle,
	struct i2c_client *client);

int lpc5400x_report_ir_blaster_learning_record(
	void *handle,
	const char *data,
	uint8_t size,
	uint16_t sequenceNumber);//zhaoheping set 32->16

#endif	/* LPC5400X_IR_BLASTER_H_ */

