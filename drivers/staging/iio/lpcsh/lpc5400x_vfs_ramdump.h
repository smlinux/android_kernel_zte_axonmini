#ifndef LPC5400X_VFS_RAMDUMP_H
#define LPC5400X_VFS_RAMDUMP_H

int lpc5400x_vfs_ramdump_prepare(
	struct device *dev,
	void **pPrivate);

int lpc5400x_vfs_ramdump_write_hdl(
	struct device *dev,
	void *pBuf,
	int size,
	void *pPrivate);

int lpc5400x_vfs_ramdump_read_hdl(
	struct device *dev,
	void *pBuf,
	int size,
	void *pPrivate);

int lpc5400x_vfs_ramdump_poll_hdl(
	struct device *dev,
	void *pPrivate);

void lpc5400x_vfs_ramdump_clean_up(void *pPrivate);

#endif
