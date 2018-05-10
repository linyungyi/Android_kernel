/************************************************************************
* Copyright (C) 2012-2015, Focaltech Systems (R)£¬All Rights Reserved.
*
* File Name: focaltech_ctl.c
*
* Author:
*
* Created: 2015-01-01
*
* Abstract: declare for IC info, Read/Write, reset
*
************************************************************************/
#ifndef TOUCHPANEL_H__
#define TOUCHPANEL_H__

#include <linux/hrtimer.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/byteorder/generic.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/rtpm_prio.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include <cust_eint.h>
#include <linux/jiffies.h>
#include <pmic_drv.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
	#include <linux/earlysuspend.h>
#endif
#include <linux/version.h>

#define TPD_POWER_SOURCE_CUSTOM         	PMIC_APP_CAP_TOUCH_VDD
#define TPD_I2C_NUMBER           				1
#define IIC_PORT                   				1				//MT6572: 1  MT6589:0 , Based on the I2C index you choose for TPM

#endif /* TOUCHPANEL_H__ */
