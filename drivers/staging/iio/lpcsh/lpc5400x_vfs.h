#ifndef LPC5400X_DEVFS_H_
#define LPC5400X_DEVFS_H_

#define LPC5400X_VFS_AIO
#ifdef LPC5400X_VFS_AIO
#include <linux/aio.h>
#endif

#include <linux/device.h>
#include <linux/sched.h>
#include <linux/kfifo.h>
#include <linux/cdev.h>
#include <linux/fs.h>

struct vfs_callback_t {
	int (*vfs_notify)(
		struct device *dev,
		int code,
		void *pUser);
};

#define VFS_NOTIFY_UPDATE_COMPLETE (0)
struct _lpc5400x_vfs_device_info {
	const char *name;
	int (*prepare_private_data)(
		struct device *dev,
		void **pPrivate);

	int (*write_handler)(
		struct device *dev,
		void *pBuf,
		int size,
		void *pPrivate);

	int (*read_handler)(
		struct device *dev,
		void *pBuf,
		int size,
		void *pPrivate);

	int (*poll_handler)(
		struct device *dev,
		void *pPrivate);

	void (*clean_up)(void *pPrivate);
};

enum Sensorhub_VFS_DeviceName {
	SENSORHUB_FIRMWAREUPDATE,
#ifdef CONFIG_LPC5400X_SENSORHUB_VOICE_TRIGGER	
	SENSORHUB_UDTDATA,
#endif
#ifdef CONFIG_LPC5400X_SENSORHUB_RAMDUMP
	SENSORHUB_RAMDUMP,
#endif	
#ifdef CONFIG_LPC5400X_SENSORHUB_LOGGING_SERVICE
	SENSORHUB_LOGGING,
#endif
	SENSORHUB_FWKEY,
	SENSORHUB_SENSORLIST,  //zhangji
	SENSORHUB_CALIBRATION,
#ifdef CONFIG_LPC5400X_SENSORHUB_AUDIO_SENSE
	SENSORHUB_AUDIOSENSE,
#endif
	SENSORHUB_SENSORDUMP,
	SENSORHUB_DEVICE_NUM
};


struct _lpc5400x_vfs {
	struct device *dev;
	struct _lpc5400x_vfs_devices *devices[SENSORHUB_DEVICE_NUM];
	struct vfs_callback_t notifier;
};

struct _lpc5400x_vfs_devices {
	dev_t dev_numeber;
	struct cdev vfs_dev;
	struct class *vfs_class;
	struct device *device;
	enum Sensorhub_VFS_DeviceName name;
	wait_queue_head_t vfs_wq;
	struct lpc5400x_platform_data *pPlatformData;
	u8 *transbuffer;
	struct _lpc5400x_vfs *parent;
	bool busy;
	void *private;
#ifdef LPC5400X_VFS_AIO
	struct work_struct aio_work;
	char *aio_tmpbuf;
	unsigned long aio_tmpbufsize;
	struct kiocb *aio_iocb;
#endif
};


int lpc5400x_create_vfsdevice(
	void **pHandle,
	struct device *dev,
	struct vfs_callback_t *callback);

int lpc5400x_destory_vfsdevice(void *handle);

#endif	/* LPC5400X_DEVFS_H_ */

