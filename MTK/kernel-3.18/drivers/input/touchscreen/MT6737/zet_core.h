/*
 *
 * Zeitecsemi TouchScreen driver.
 * 
 * Copyright (c) 2010-2015, Zeitecsemi Ltd. All rights reserved.
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

#ifndef __LINUX_ZET_H__
#define __LINUX_ZET_H__
 /*******************************************************************************
*
* File Name: zet_core.h
*
*    Author: Albert Lin
*
*   Created: 2016-06-03
*
*  Abstract:
*
* Reference:
*
*******************************************************************************/

/*******************************************************************************
* 1.Included header files
*******************************************************************************/
#include "tpd.h"
#include <linux/hrtimer.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/syscalls.h>
#include <linux/byteorder/generic.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/rtpm_prio.h>
#include <asm/unistd.h>
#include <mach/irqs.h>
#include <linux/jiffies.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/mount.h>
#include <linux/unistd.h>
#include <linux/proc_fs.h>
#include <linux/netdevice.h>
#include <../fs/proc/internal.h>

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/

/**********************Custom define begin**********************************************/
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 8, 0))
	#if defined(MODULE) || defined(CONFIG_HOTPLUG)
		#define __devexit_p(x) 				x
	#else
		#define __devexit_p(x) 				NULL
	#endif
	// Used for HOTPLUG
	#define __devinit        					__section(.devinit.text) __cold notrace
	#define __devinitdata    					__section(.devinit.data)
	#define __devinitconst   					__section(.devinit.rodata)
	#define __devexit        					__section(.devexit.text) __exitused __cold notrace
	#define __devexitdata    					__section(.devexit.data)
	#define __devexitconst   					__section(.devexit.rodata)
#endif

#define TPD_POWER_SOURCE_CUSTOM         	MT6323_POWER_LDO_VGP1
#define IIC_PORT                   			0			// MT6572: 1  MT6589:0 , Based on the I2C index you choose for TPM
//#define TPD_HAVE_BUTTON								// if have virtual key,need define the MACRO
//#define TPD_BUTTON_HEIGH        			(40)
//#define TPD_KEY_COUNT           			3
//#define TPD_KEYS                			{ KEY_MENU, KEY_HOMEPAGE, KEY_BACK}
//#define TPD_KEYS_DIM            			{{80,900,20,TPD_BUTTON_HEIGH}, {240,900,20,TPD_BUTTON_HEIGH}, {400,900,20,TPD_BUTTON_HEIGH}}

/*********************Custom Define end*************************************************/
#define MT_PROTOCOL_B
#define A_TYPE												0
#define TPD_TYPE_CAPACITIVE
#define TPD_TYPE_RESISTIVE
#define TPD_POWER_SOURCE    
#define TPD_NAME    					"ZET"
#define TPD_I2C_NUMBER           		0
#define TPD_WAKEUP_TRIAL         		60
#define TPD_WAKEUP_DELAY         		100
#define TPD_VELOCITY_CUSTOM_X 			15
#define TPD_VELOCITY_CUSTOM_Y 			20

#define CFG_MAX_TOUCH_POINTS			5
#define MT_MAX_TOUCH_POINTS				10

/******************************************************************************/


/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/

extern struct i2c_client *zet_i2c_client;
extern struct input_dev *zet_input_dev;
extern struct tpd_device *tpd;


#endif
