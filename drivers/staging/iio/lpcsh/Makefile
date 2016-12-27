#
# Makefile for the input misc drivers.
#

# Each configuration option enables a list of files.

obj-$(CONFIG_LPC5400X_SENSORHUB_I2C)	+= lpc5400x.o sensor_core.o
obj-$(CONFIG_LPC5400X_SENSORHUB_FIRMWARE_UPDATE_I2C) += lpc5400x_update.o lpc5400x_sl_transport.o lpc5400x_vfs.o 
obj-$(CONFIG_LPC5400X_SENSORHUB_VOICE_TRIGGER) += lpc5400x_voice_trigger.o
obj-$(CONFIG_LPC5400X_SENSORHUB_IR_BLASTER) += lpc5400x_ir_blaster.o lpc5400x_vfs.o
obj-$(CONFIG_LPC5400X_SENSORHUB_BREATH_LED) += lpc5400x_leds.o
obj-$(CONFIG_LPC5400X_SENSORHUB_LOGGING_SERVICE) +=  lpc5400x_vfs_logging.o
obj-$(CONFIG_LPC5400X_SENSORHUB_RAMDUMP) += lpc5400x_ramdump.o lpc5400x_sl_transport.o lpc5400x_vfs.o lpc5400x_update.o lpc5400x_vfs_ramdump.o
