#define DEBUG

#include <linux/types.h>
#include <linux/device.h>

#include "sl_protocol.h"

#define RAMDUMP_SUMMARY_SIZE (24)
#define RAMDUMP_CONTEXT_SIZE (348)	/* Summery(24B) +  CPU register(80B) + SCB(140B) + NVIC(104B) */
/*


 */
int lpc5400x_dumpLunchMenu(struct device *dev, uint8_t *data,
						   uint32_t maxsize, int gpio)
{
	struct _DS_HostDumpCmdCommon cmd;
	struct _DS_HostDumpResSum *resp = (struct _DS_HostDumpResSum *)data;
	int ret;

	cmd.cmdCode = SH_CMD_DUMP_SUMMARY;
	cmd.c4ToReply = RAMDUMP_SUMMARY_SIZE / 4;

	dev_dbg(dev, "lpc5400x_dumpLunchMenu, maxsize = %d", maxsize);

	ret = SLBoot_send_cmd(dev, (u8 *)&cmd, sizeof(cmd),
		(u8 *)resp, RAMDUMP_SUMMARY_SIZE + sizeof(struct _CmdResponse_t), gpio);
	if (ret < 0) {
		dev_dbg(dev, "lpc5400x_dumpLunchMenu send command error %d", ret);
		return ret;
	}

	ret = RAMDUMP_SUMMARY_SIZE + sizeof(struct _CmdResponse_t);

	pr_debug("lpc5400x_dumpLunchMenu return %d", ret);

	return ret;
}

/*


 */
int lpc5400x_dumpCPUContext(struct device *dev,
							uint8_t *data, uint32_t maxsize, int gpio)
{
	struct _DS_HostDumpCmdCommon cmd;
	struct _DS_HostDumpResCtx *resp = (struct _DS_HostDumpResCtx *)data;
	int ret;

	cmd.cmdCode = SH_CMD_DUMP_CPU_CONTEXT;
	cmd.c4ToReply = RAMDUMP_CONTEXT_SIZE / 4;

	dev_dbg(dev, "lpc5400x_dumpCPUContext, maxsize = %d", maxsize);

	ret = SLBoot_send_cmd(dev, (u8 *)&cmd, sizeof(cmd),
		(u8 *)resp, RAMDUMP_CONTEXT_SIZE + sizeof(struct _CmdResponse_t), gpio);
	if (ret < 0) {
		dev_dbg(dev, "lpc5400x_dumpCPUContext send command error %d", ret);
		return ret;
	}

	ret = RAMDUMP_CONTEXT_SIZE + sizeof(struct _CmdResponse_t);

	pr_debug("lpc5400x_dumpCPUContext return %d", ret);

	return ret;
}

/*

 */
int lpc5400x_dumpStack(struct device *dev,
					   uint8_t *data, uint32_t maxsize, uint32_t startAddr, int gpio)
{
	struct _DS_HostDumpCmdStk cmd;
	struct _DS_HostDumpResRng *resp = (struct _DS_HostDumpResRng *)data;
	int ret;

	cmd.cmdCode = SH_CMD_DUMP_STACK;
	cmd.c4ToReply = (maxsize - sizeof(struct _CmdResponse_t)) / 4;
	cmd.c4ToTop = startAddr;

	dev_dbg(dev, "lpc5400x_dumpStack, startAddr = %d, maxsize = %d",
		startAddr, maxsize);

	ret = SLBoot_send_cmd(dev, (u8 *)&cmd, sizeof(cmd), (u8 *)resp,
		maxsize, gpio);
	if (ret < 0) {
		dev_dbg(dev, "lpc5400x_dumpStack send command error %d", ret);
		return ret;
	}
	return maxsize;
}

int lpc5400x_dumpMemory(struct device *dev, uint8_t *data,
						uint32_t maxsize, uint32_t startAddr, int gpio)
{
	struct _DS_HostDumpCmdRng cmd;
	struct _DS_HostDumpResRng *resp = (struct _DS_HostDumpResRng *)data;
	int ret;

	dev_dbg(dev, "lpc5400x_dumpMemory, startAddr = 0x%x, maxsize = %d",
		startAddr, maxsize);

	cmd.cmdCode = SH_CMD_DUMP_MEMORY;
	cmd.c4ToReply = (maxsize - sizeof(struct _CmdResponse_t)) / 4;
	cmd.addrH16 = (startAddr >> 16) & 0xFFFF;
	cmd.addrL16 = startAddr & 0xFFFF;

	ret = SLBoot_send_cmd(dev, (u8 *)&cmd, sizeof(cmd), (u8 *)resp,
		maxsize, gpio);
	if (ret < 0) {
		dev_dbg(dev, "lpc5400x_dumpMemory send command error %d", ret);
		return ret;
	}

	return maxsize;
}

int lpc5400x_dumpTestSample(struct device *dev, int gpio)
{
	return 0;
}

