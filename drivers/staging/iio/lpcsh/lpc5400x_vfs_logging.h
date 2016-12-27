#ifndef LPC5400X_VFS_LOGGING_H
#define LPC5400X_VFS_LOGGING_H

int lpc5400x_vfs_logging_prepare(
	struct device *dev,
	void **pPrivate);

int lpc5400x_vfs_logging_write_hdl(
	struct device *dev,
	void *pBuf,
	int size, 
	void *pPrivate);

int lpc5400x_vfs_logging_read_hdl(
	struct device *dev,
	void *pBuf,
	int size,
	void *pPrivate);

int lpc5400x_vfs_logging_poll_hdl(
	struct device *dev,
	void *pPrivate);

void lpc5400x_vfs_logging_clean_up(void *pPrivate);

void lpc5400x_vfs_logging_save_events(
	void *vfs,
	const void *pBuf,
	int size);

#endif

