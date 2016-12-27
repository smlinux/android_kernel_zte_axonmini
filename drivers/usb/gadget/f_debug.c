/*
 * ZTE USB debug Driver for Android
 *
 * Copyright (C) 2012 ZTE, Inc.
 * Author: Li Xingyuan <li.xingyuan@zte.com.cn>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/sysrq.h>

static void dbg_sysrq_complete(struct usb_ep *ep, struct usb_request *req)
{
	int length = req->actual;
	char key;
	
	if (req->status != 0) {
		pr_err("err %d\n", req->status);
		return;
	}

	if (length >= 1) {
		key = ((char *)req->buf)[0];
		pr_info("%02x\n", key);
		handle_sysrq(key);
	}
}

#define CMD_BUFF_LEN	1024
#define CMD_ARGV_NUM	10
static char buf[CMD_BUFF_LEN + 1];
static int pending;

static void dbg_run_work(struct work_struct *bullshit)
{
	char *p = buf;
	char *argv[CMD_ARGV_NUM + 1];
	char *envp[] = {
		"HOME=/data",
		"PATH=/sbin:/vendor/bin:/system/sbin:/system/bin:/system/xbin",
		"LD_LIBRARY_PATH=/vendor/lib64:/system/lib64",
		NULL};
	int i;
	int ret;

	i = 0;
	while (p && i < CMD_ARGV_NUM) {
		argv[i] = strsep(&p, " ");
		if (strlen(argv[i])) {
			pr_info("argv[%d]=%s\n", i, argv[i]);
			i++;
		}
	}
	if (!i) {
		pr_err("nothing to done!\n");
		goto __out;
	}
	argv[i] = NULL;
	if (argv[i - 1][0] == '&') {
		pr_info("run in backgroud!\n");
		argv[i - 1] = NULL;
		ret = call_usermodehelper(argv[0], argv, envp, UMH_WAIT_EXEC);
	} else
		ret = call_usermodehelper(argv[0], argv, envp, UMH_WAIT_PROC);
	pr_info("ret=%d\n", ret);
	
__out:
	pending = 0;
}

static DECLARE_WORK(work, dbg_run_work);

static void dbg_run_complete(struct usb_ep *ep, struct usb_request *req)
{
	if (req->status != 0) {
		pr_err("err %d\n", req->status);
		return;
	}

	if (req->actual > CMD_BUFF_LEN) {
		pr_err("buffer overflow %d\n", req->actual);
		return;
	}

	memcpy(buf, req->buf, req->actual);
	buf[req->actual] = '\0';
	pr_info("p=%s\n", buf);

	pending = 1;
	schedule_work(&work);
}

static int dbg_ctrlrequest(struct usb_composite_dev *cdev,
				const struct usb_ctrlrequest *ctrl)
{
	int	value = -EOPNOTSUPP;
	u8 b_requestType = ctrl->bRequestType;
	u8 b_request = ctrl->bRequest;
	u16	w_index = le16_to_cpu(ctrl->wIndex);
	u16	w_value = le16_to_cpu(ctrl->wValue);
	u16	w_length = le16_to_cpu(ctrl->wLength);

	pr_debug("%02x %02x %04x %04x %04x\n",  
		b_requestType, b_request, w_index, w_value, w_length);

	if (b_requestType == (USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE)
			&& w_value == 0xffff && w_index == 0xffff) {
		if (b_request == 0x00) {
			pr_info("sysrq request!\n");
			cdev->req->complete = dbg_sysrq_complete;
			value = w_length;
		} else if (b_request == 0x01) {
			if (!pending) {
				pr_info("call user request!\n");
				cdev->req->complete = dbg_run_complete;
				value = w_length;
			} else {
				pr_info("request pending!\n");
			}
		}
	}
	
	if (value >= 0) {
		cdev->req->zero = 0;
		cdev->req->length = value;
		value = usb_ep_queue(cdev->gadget->ep0, cdev->req, GFP_ATOMIC);
		if (value < 0)
			pr_err("setup response queue error %d\n", value);
	}

	return value;
}
