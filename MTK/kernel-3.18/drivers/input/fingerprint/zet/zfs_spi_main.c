/* ZFS1096 fingerprint sensor driver
 *
 * Copyright (c) 2017 powered by albert <albert.lin@zeitecsemi>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#define DEBUG

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/poll.h>
#include <linux/types.h>

#include <linux/file.h>
#include <asm/uaccess.h>
#include <linux/version.h>

#ifndef CONFIG_OF
/*
#include <linux/spi/fpc1020.h>
#include <linux/spi/fpc1020_regs.h>
#include <linux/spi/fpc1020_input.h>
#include <linux/spi/fpc1020_capture.h>
#include <linux/spi/fpc1020_regulator.h>
*/
#else
	
#include <linux/of.h>
#include <linux/of_irq.h>
#include "zfs.h"
#include "zfs_common.h"
#include "zfs_regs.h"
#include "zfs_input.h"
#include "zfs_capture.h"

#endif

#ifdef ZET_REE_SPI_SUPPORT
#include <linux/spi/spi.h>
#endif

/*MTK header*/
#ifdef FPC_MTK

#ifdef ZET_REE_SPI_SUPPORT
#include <mt_spi.h>
#include <mt_spi_hal.h>
#endif /* ZET_REE_SPI_SUPPORT */

#include <mt-plat/mt_gpio.h>

#else /* else FPC_MTK */
	
#include <linux/of_gpio.h>
#include <linux/of_platform.h>

#endif /* FPC_MTK */

/*
#ifndef CONFIG_OF
#include <linux/spi/fpc1020.h>

#include <linux/spi/fpc1020_regs.h>
#include <linux/spi/fpc1020_input.h>
#include <linux/spi/fpc1020_capture.h>
#include <linux/spi/fpc1020_regulator.h>
#else
#include <linux/of.h>
#include <linux/of_irq.h>
#include "fpc1020.h"
#include "fpc1020_common.h"
#include "fpc1020_regs.h"
#include "fpc1020_input.h"
#include "fpc1020_capture.h"
#include "fpc1020_regulator.h"
#endif
*/

#include <linux/regulator/consumer.h>
#include <linux/of_platform.h>

#ifdef ZET_REE_SPI_SUPPORT
	
#ifdef ZET_BOOTLOADER
#include "./bin/zfs1096_fw.h"
#endif

#endif

/* -------------------------------------------------------------------- */
/* driver constants						*/
/* -------------------------------------------------------------------- */
#define ZFS1096_CLASS_NAME              "zfs1096"   /* FPC1020_CLASS_NAME */
#define ZFS1096_WORKER_THREAD_NAME		"zfs1096_worker" /* FPC1020_WORKER_THREAD_NAME */

#define IOCTL_MAX_BUF_SIZE          			(1024)

#define FPC1020_IOCTL_START_CAPTURE 			(0x10)
#define FPC1020_IOCTL_ABORT_CAPTURE 			(0x11)
#define FPC1020_IOCTL_CAPTURE_SINGLE  			(0x12)

#define ZET_BASE_IO_OFFSET              0
#define ZFS1096_IOCTL_MAGIC_NO          0x3C
#define ZET_IOCTL_BASE(x)               ZET_BASE_IO_OFFSET+x

#define ZET_IOCTL_START_CAPTURE         _IO(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(0))
#define ZET_IOCTL_ABORT_CAPTURE         _IO(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(1))
#define ZET_IOCTL_READ_REGISTER         _IOWR(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(2), unsigned int)
#define ZET_IOCTL_WRITE_REGISTER        _IOW(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(3), unsigned int)

#define ZET_IOCTL_SOFT_RESET            _IOW(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(5), unsigned int)

#define ZET_IOCTL_READ_DATA             _IOWR(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(17), unsigned int)
#define ZET_IOCTL_WAIT_TOUCH            _IOWR(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(19), unsigned int)
#define ZET_IOCTL_CANCEL_WAIT_TOUCH     _IOWR(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(20), unsigned int)

#define ZET_IOCTL_SET_KEY_EVENT         _IOWR(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(23), unsigned int)

#define ZET_IOCTL_CHECK_FRAME_READY     _IOWR(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(29), unsigned int)
#define ZET_IOCTL_HARDWARE_RESET        _IOWR(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(30), unsigned int)
#define ZET_IOCTL_FLASH_DOWNLOAD        _IOW(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(31), unsigned int)
#define ZET_IOCTL_START_KEY_NAVI        _IOW(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(32), unsigned int)
#define ZET_IOCTL_ABORT_KEY_NAVI        _IOW(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(33), unsigned int)
#define ZET_IOCTL_ENABLE_KEY_REPORT     _IOW(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(34), unsigned int)
#define ZET_IOCTL_READ_KEY_DATA		    _IOW(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(35), unsigned int)
#define ZET_IOCTL_INPUT_SET_EVENT	    _IOW(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(36), unsigned int)
#define ZET_IOCTL_GET_SCAN_MODE		    _IOW(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(37), unsigned int)
#define ZET_IOCTL_SET_SCAN_MODE		    _IOW(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(38), unsigned int)
#define ZET_IOCTL_GET_HW_INFO			_IOW(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(39), unsigned int)
#define ZET_IOCTL_READ_IMAGE_SECTOR		_IOWR(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(40), unsigned int)
#define ZET_IOCTL_START_SECTORS			_IOW(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(41), unsigned int)
#define ZET_IOCTL_INTERRUPT_DONE		_IOR(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(42), unsigned int)
#define ZET_IOCTL_SPI_SUPPORT			_IOR(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(43), unsigned int)
#define ZET_IOCTL_INPUT_SYNC	    	_IOW(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(44), unsigned int)
#define ZET_IOCTL_FRAMEWORK_MSG	    	_IOW(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(45), unsigned int)

/**reserved begin***************************************************************************************************/


#define ZET_IOCTL_INIT_REG              _IOW(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(6), unsigned int)
#define ZET_IOCTL_READ_ATTR             _IOWR(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(9), unsigned int)
#define ZET_IOCTL_RV                    _IOWR(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(10), unsigned int)
#define ZET_IOCTL_SET_REG               _IOWR(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(11), unsigned int)
#define ZET_IOCTL_SET_OP                _IOWR(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(12), unsigned int)
#define ZET_IOCTL_SET_CLOCK_SPEED       _IOWR(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(13), unsigned int)
#define ZET_IOCTL_WRITE_ATTR            _IOWR(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(14), unsigned int)
#define ZET_IOCTL_SET_VR                _IOWR(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(15), unsigned int)
#define ZET_IOCTL_READ_RAW              _IOWR(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(16), unsigned int)

#define ZET_IOCTL_TEST_UEVENT           _IOWR(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(18), unsigned int)

#define ZET_IOCTL_SET_SUSPEND           _IOWR(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(21), unsigned int)
#define ZET_IOCTL_SYSTEM_WAKE_UP        _IOWR(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(22), unsigned int)

#define ZET_IOCTL_UEVENT_CTL         	_IOWR(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(24), unsigned int)
#define ZET_IOCTL_WAKE_LOCK             _IOWR(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(25), unsigned int)
#define ZET_IOCTL_WAKE_UNLOCK           _IOWR(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(26), unsigned int)
#define ZET_DISABLE_IRQ                 _IOWR(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(27), unsigned int)
#define ZET_ENABLE_IRQ                  _IOWR(ZFS1096_IOCTL_MAGIC_NO, ZET_IOCTL_BASE(28), unsigned int)

/**reserved end*****************************************************************************************************/

#define ZET_UEVENT_TOUCH_DOWN 	 "TOUCH_DOWN"
#define ZET_UEVENT_INTERRUPT	 "INTERRUPT"
#define ZET_UEVENT_SUSPEND		 "SUSPEND"
#define ZET_UEVENT_RESUME		 "RESUME"
#define ZET_UEVENT_TOUCH_KEY	 "TOUCH_KEY"

#define ZET_UEVENT_FRAMEWORK_MSG	 "FRAMEWORK_MSG"
#define ZET_UEVENT_FRAMEWORK_IMAGE	 "FRAMEWORK_IMAGE"
#define ZET_UEVENT_FRAMEWORK_ENROLL	 "FRAMEWORK_ENROLL"
#define ZET_UEVENT_FRAMEWORK_VERIFY	 "FRAMEWORK_VERIFY"

/* -------------------------------------------------------------------- */
/* global variables							*/
/* -------------------------------------------------------------------- */
static const char zfs1096_driver_info[] = { 0x0 , 0x3 , 0x0 , 0x0 };

/* static char zfs1096_fw_info[] = { 
0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00,
0x00
}; */

static u8 zfs1096_spi_support = 0;

static int zfs1096_device_count;  // fpc1020_device_count

#ifdef ZET_REE_SPI_SUPPORT
#ifdef FPC_MTK
static struct mt_chip_conf spi_conf;
#endif /* FPC_MTK */
#endif /* ZET_REE_SPI_SUPPORT */

char flash_path[512];

struct inode *inode 			= NULL;	
mm_segment_t old_fs;

/* -------------------------------------------------------------------- */
/* data types							*/
/* -------------------------------------------------------------------- */

/* fpc1020_attribute */
struct zfs_attribute {
	struct device_attribute attr;
	size_t offset;
};

enum {
	ZFS_WORKER_IDLE_MODE = 0,
	ZFS_WORKER_IRQ_MODE,
	ZFS_WORKER_SCAN_MODE,
	ZFS_WORKER_CAPTURE_MODE,
	ZFS_WORKER_INPUT_MODE,
	ZFS_WORKER_RESET_MODE,
	ZFS_WORKER_DOWNLOAD_MODE,
	ZFS_WORKER_EXIT
};

typedef enum {
	ZFS_FRAME_BUSY = 0,
	ZFS_FRAME_READY,
	ZFS_FRAME_FAIL,
	ZFS_FRAME_UNVIABLE,
} zfs_frame_state_t;

//albert added for hikey
#ifdef FPC_MTK 

static struct pinctrl *zfs_pinctrl;
// static struct pinctrl_state *zfs_pin_default;
static struct pinctrl_state *zfs_pin_rst_high, *zfs_pin_rst_low;
static struct pinctrl_state *zfs_pin_cs_high, *zfs_pin_cs_low;
static struct pinctrl_state *zfs_pin_eint;
static bool zfs_pin_rst_high_success;
static bool zfs_pin_rst_low_success;
static bool zfs_pin_cs_high_success;
static bool zfs_pin_cs_low_success;
static bool zfs_pin_eint_success;

#else
	
static int zet_gpio_reset 		= -1;
static int zet_gpio_irq 		= -1;
static int zet_irq_num 			= -1;
static int zet_gpio_cs	 		= -1;

#endif

#define RST_HIGh 0
#define RST_LOW  1
#define CS_HIGH  2
#define CS_LOW   3

// static u8 zet_buf[65535];

/* -------------------------------------------------------------------- */
/* function prototypes													*/
/* -------------------------------------------------------------------- */

static int __init zfs_spi_init(void);

static void __exit zfs_spi_exit(void);

static int zfs_spi_probe(struct spi_device *spi);

static int zfs_spi_remove(struct spi_device *spi);
	
static int zfs_manage_sysfs(zfs_data_t *zfs,
				struct spi_device *spi, bool create);


static int zfs_suspend(struct device *dev);

static int zfs_resume(struct device *dev);

static int zfs_param_init(zfs_data_t *zfs,
					struct zfs_platform_data *pdata);

static int zfs_irq_init(zfs_data_t *zfs,
					struct zfs_platform_data *pdata);
					
static int zfs_send_uevent(struct device* dev, char* event);			

static void zfs_interrupt_work(struct work_struct *work);

static int zfs_gpio_set(int cmd);					

static int zfs_create_class(zfs_data_t *zfs);

static int zfs_create_device(zfs_data_t *zfs);
		
/* fpc1020_show_attr_setup */
static ssize_t zfs_show_attr_setup(struct device *dev,
					struct device_attribute *attr,
					char *buf);

/* fpc1020_store_attr_setup */
static ssize_t zfs_store_attr_setup(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count);

/* fpc1020_show_attr_diag */
static ssize_t zfs_show_attr_diag(struct device *dev,
					struct device_attribute *attr,
					char *buf);

/* fpc1020_store_attr_diag */
static ssize_t zfs_store_attr_diag(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count);

static int zfs1096_open(struct inode *inode, struct file *file);

static ssize_t zfs1096_write(struct file *file, const char *buff,
					size_t count, loff_t *ppos);

static ssize_t zfs1096_read(struct file *file, char *buff,
				size_t count, loff_t *ppos);
				
static int zfs1096_release(struct inode *inode, struct file *file);

static unsigned int zfs1096_poll(struct file *file, poll_table *wait);
					
static long zfs1096_ioctl(struct file *file, unsigned int cmd,
					unsigned long arg);					
					
#ifdef ZET_REE_SPI_SUPPORT
static int zfs_spi_setup(zfs_data_t *zfs,
					struct zfs_platform_data *pdata);					
#endif /* ZET_REE_SPI_SUPPORT */
								
#ifdef ZET_CAPTURE_SUPPORT
								
static int zfs_worker_init(zfs_data_t *zfs);

static int zfs_worker_function(void *_zfs);

static int zfs_start_capture(zfs_data_t *zfs, bool wait_touch_down);

static int zfs_start_input(zfs_data_t *zfs /*fpc1020_data_t *fpc1020*/);
		
static int zfs_new_job(zfs_data_t *zfs, int new_job);
									
static int zfs_worker_goto_idle(zfs_data_t *zfs);	

static int zfs_fw_load(char *file_name, zfs_data_t *zfs_dev);

#if 0
static int zfs_fw_save(char *file_name, zfs_data_t *zfs_dev);
#endif

#endif /* ZET_CAPTURE_SUPPORT */

/* -------------------------------------------------------------------- */
/* devfs								*/
/* -------------------------------------------------------------------- */
/* FPC1020_ATTR */
#define ZFS_ATTR(__grp, __field, __mode)				\
{									\
	.attr = __ATTR(__field, (__mode),				\
	zfs_show_attr_##__grp,					\
	zfs_store_attr_##__grp),					\
	.offset = offsetof(struct zfs_##__grp, __field)		\
}

/* FPC1020_DEV_ATTR */
#define ZFS_DEV_ATTR(_grp, _field, _mode)				\
struct zfs_attribute zfs_attr_##_field =			\
					ZFS_ATTR(_grp, _field, (_mode))

/*#define DEVFS_SETUP_MODE (S_IWUSR|S_IWGRP|S_IWOTH|S_IRUSR|S_IRGRP|S_IROTH)*/
#define DEVFS_SETUP_MODE (S_IWUSR|S_IWGRP|S_IRUSR|S_IRGRP|S_IROTH)

/*
static ZFS_DEV_ATTR(setup, adc_gain,		DEVFS_SETUP_MODE);
static ZFS_DEV_ATTR(setup, adc_shift,		DEVFS_SETUP_MODE);
static ZFS_DEV_ATTR(setup, capture_mode,		DEVFS_SETUP_MODE);
*/
static ZFS_DEV_ATTR(setup, capture_count,		DEVFS_SETUP_MODE);
/*static ZFS_DEV_ATTR(setup, capture_settings_mux,	DEVFS_SETUP_MODE);
static ZFS_DEV_ATTR(setup, pxl_ctrl,		DEVFS_SETUP_MODE);
static ZFS_DEV_ATTR(setup, capture_row_start,	DEVFS_SETUP_MODE);
static ZFS_DEV_ATTR(setup, capture_row_count,	DEVFS_SETUP_MODE);
static ZFS_DEV_ATTR(setup, capture_col_start,	DEVFS_SETUP_MODE);
static ZFS_DEV_ATTR(setup, capture_col_groups,	DEVFS_SETUP_MODE);
*/

static struct attribute *zfs_setup_attrs[] = {
/*	&fpc1020_attr_adc_gain.attr.attr,
	&fpc1020_attr_adc_shift.attr.attr,
	&fpc1020_attr_capture_mode.attr.attr,
*/
	&zfs_attr_capture_count.attr.attr,
/*	&fpc1020_attr_capture_settings_mux.attr.attr,
	&fpc1020_attr_pxl_ctrl.attr.attr,
	&fpc1020_attr_capture_row_start.attr.attr,
	&fpc1020_attr_capture_row_count.attr.attr,
	&fpc1020_attr_capture_col_start.attr.attr,
	&fpc1020_attr_capture_col_groups.attr.attr,*/
	NULL
};

/* fpc1020_setup_attr_group */
static const struct attribute_group zfs_setup_attr_group = {
	.attrs = zfs_setup_attrs,
	.name = "setup"
};

#define DEVFS_DIAG_MODE_RO (S_IRUSR|S_IRGRP|S_IROTH)
/*#define DEVFS_DIAG_MODE_RW (S_IWUSR|S_IWGRP|S_IWOTH|S_IRUSR|S_IRGRP|S_IROTH)*/
#define DEVFS_DIAG_MODE_RW (S_IWUSR|S_IWGRP|S_IRUSR|S_IRGRP|S_IROTH)

static ZFS_DEV_ATTR(diag, chip_id,		DEVFS_DIAG_MODE_RO);
/*
static ZFS_DEV_ATTR(diag, selftest,		DEVFS_DIAG_MODE_RO);
static ZFS_DEV_ATTR(diag, spi_register,	DEVFS_DIAG_MODE_RW);
static ZFS_DEV_ATTR(diag, spi_regsize,	DEVFS_DIAG_MODE_RO);
static ZFS_DEV_ATTR(diag, spi_data ,	DEVFS_DIAG_MODE_RW);
static ZFS_DEV_ATTR(diag, last_capture_time, DEVFS_DIAG_MODE_RO);
static ZFS_DEV_ATTR(diag, finger_present_status, DEVFS_DIAG_MODE_RO);
*/

/* fpc1020_diag_attrs */
static struct attribute *zfs_diag_attrs[] = {
	&zfs_attr_chip_id.attr.attr,
/*	&fpc1020_attr_selftest.attr.attr,
	&fpc1020_attr_spi_register.attr.attr,
	&fpc1020_attr_spi_regsize.attr.attr,
	&fpc1020_attr_spi_data.attr.attr,
	&fpc1020_attr_last_capture_time.attr.attr,
	&fpc1020_attr_finger_present_status.attr.attr,*/
	NULL
};

/* fpc1020_diag_attr_group */
static const struct attribute_group zfs_diag_attr_group = {
	.attrs = zfs_diag_attrs,
	.name = "diag"
};
			
			
/* ------------------------------ZFS1096-------------------------------------- */

/* -------------------------------------------------------------------- */
/* function definitions							*/
/* -------------------------------------------------------------------- */

#ifdef ZET_CAPTURE_SUPPORT

/* -------------------------------------------------------------------- */
#if 0
static int zfs_fw_save(char *file_name, zfs_data_t *zfs_dev)
{
	struct file *fp;
	int flash_total_len = 0;
	
	fp = filp_open(file_name, O_RDWR | O_CREAT, 0644);
	
	if(IS_ERR(fp))
	{
		pr_err("[ZET] : Failed to open %s\n", file_name);
		return 0;
	}

	if(zfs_dev->flash_buffer == NULL)
	{
		pr_err("[ZET] : flash_buffer is null \n");
		return 0;
	}	
	
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	flash_total_len = zfs_dev->chip.flash_size;
	pr_err("[ZET] : flash_total_len = 0x%06x\n",flash_total_len );
	vfs_write(fp, zfs_dev->flash_buffer, flash_total_len, &(fp->f_pos));
	set_fs(old_fs);
	filp_close(fp, 0);
	return 1;
}
#endif

/* -------------------------------------------------------------------- */
static int zfs_fw_load(char *file_name, zfs_data_t *zfs_dev)
{
	int file_length = 0;
	struct file *fp;
	loff_t *pos;	
	
	pr_err("[ZET]: find %s\n", file_name);
	fp = filp_open(file_name, O_RDONLY, 0644);
	if(IS_ERR(fp))
	{
		pr_err("[ZET]: No firmware file detected\n");
		return 0;
	}
	
	if(zfs_dev->flash_buffer == NULL)
	{
  		zfs_dev->flash_buffer = kmalloc(MAX_FLASH_BUF_SIZE, GFP_KERNEL);
		memset(zfs_dev->flash_buffer,0xFF,MAX_FLASH_BUF_SIZE);
	}
	///----------------------------///
	/// Load from file
	///----------------------------///
	pr_err("[ZET]: Load from %s\n", file_name);
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	// Get file size
	
//#ifdef Kernel_3_18
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,19,0)
	inode = fp->f_dentry->d_inode;
#else
	inode = fp->f_path.dentry->d_inode;
#endif
	
	file_length = (int)inode->i_size;
	pos = &(fp->f_pos);
	vfs_read(fp, zfs_dev->flash_buffer, file_length, pos);
	//vfs_read(fp, &flash_buffer[0], file_length, pos);
	//file_length
	set_fs(old_fs);
	filp_close(fp, 0);
	return 1;
}

/* -------------------------------------------------------------------- */
static int zfs_worker_goto_idle(zfs_data_t *zfs /* fpc1020_data_t *fpc1020 */)
{
	const int wait_idle_us = 100;

	if (down_trylock(&zfs->worker.sem_idle)) {
		dev_dbg(&zfs->pdev->dev, "[ZET] %s, stop_request\n", __func__);

		zfs->worker.stop_request = true;
		zfs->worker.req_mode = ZFS_WORKER_IDLE_MODE;

		while (down_trylock(&zfs->worker.sem_idle))	{

			zfs->worker.stop_request = true;
			zfs->worker.req_mode = ZFS_WORKER_IDLE_MODE;

			udelay(wait_idle_us); /*original is usleep*/
		}

		up(&zfs->worker.sem_idle);

	} else {
		dev_dbg(&zfs->pdev->dev, "[ZET] %s, worker already idle\n", __func__);
		up(&zfs->worker.sem_idle);
	}	
	
	// if(clear_interrupt)
		zfs1096_clear_int(zfs, ZFS1096_IRQ_EXIT);

	return 0;
}

/* -------------------------------------------------------------------- */
static int zfs_new_job(zfs_data_t *zfs /* fpc1020_data_t *fpc1020 */, int new_job)
{
	dev_dbg(&zfs->pdev->dev, "[ZET] %s %d\n", __func__, new_job);

	if(new_job == ZFS_WORKER_RESET_MODE || new_job == ZFS_WORKER_DOWNLOAD_MODE)
	{
		
		zfs->worker.stop_request = true;
		// zfs->worker.req_mode = ZFS_WORKER_IDLE_MODE;		
		
		/// clear all interrupt lock
		zfs->interrupt_done = true;
		wake_up_interruptible(&zfs->wq_irq_return);
		
		/// unlock frame lock
		zfs->capture.state = ZFS_CAPTURE_STATE_IDLE;
		zfs->capture.available_bytes  = 0;
		zfs->capture.read_offset = 0;
		zfs->capture.read_pending_eof = false;			
		up(&zfs->sem_frame);			
		
	}//else
	//{
		zfs_worker_goto_idle(zfs);
	//}		

	zfs->worker.req_mode = new_job;
	zfs->worker.stop_request = false;

	wake_up_interruptible(&zfs->worker.wq_wait_job);

	return 0;
}

/* -------------------------------------------------------------------- */
static int zfs_start_capture(zfs_data_t *zfs /* fpc1020_data_t *fpc1020 */, bool wait_touch_down)
{
	//fpc1020_capture_mode_t mode = zfs->setup.capture_mode;
	// zfs_capture_mode_t mode = zfs->current_mode;
	int error = 0;
	int result = 0;

	dev_dbg(&zfs->pdev->dev, "[ZET] %s \n", __func__);
	
	if( zfs->worker.req_mode == ZFS_WORKER_CAPTURE_MODE)
	{
		return -EBUSY;
	}
		

	/* Mode check (and pre-conditions if required) ? */
	/*switch (mode) {
	case FPC1020_MODE_WAIT_FINGER_DOWN:
	
		break;		
	case FPC1020_MODE_WAIT_AND_CAPTURE:
	
		break;
	case FPC1020_MODE_SINGLE_CAPTURE:
	
		break;
	case FPC1020_MODE_CHECKERBOARD_TEST_NORM:
	case FPC1020_MODE_CHECKERBOARD_TEST_INV:
	case FPC1020_MODE_BOARD_TEST_ONE:
	case FPC1020_MODE_BOARD_TEST_ZERO:
	case FPC1020_MODE_WAIT_FINGER_UP:
	case FPC1020_MODE_SINGLE_CAPTURE_CAL:
	case FPC1020_MODE_CAPTURE_AND_WAIT_FINGER_UP:
		break;

	case FPC1020_MODE_IDLE:
		result = zfs1096_wait_for_irq(zfs, ZFS1096_REG_READ_INTERRUPT, ZFS1096_STATUS_REG_BIT_FINGER | ZFS1096_STATUS_REG_BIT_KEY, 0);
		
		if( result >= 0 )
		{
			
		}
		break;
	default:
		error = -EINVAL;
		break;
	}*/

	// zfs->capture.current_mode = (error >= 0) ? mode : FPC1020_MODE_IDLE;

	zfs->capture.state = ZFS_CAPTURE_STATE_STARTED;
	zfs->capture.available_bytes  = 0;
	zfs->capture.read_offset = 0;
	zfs->capture.read_pending_eof = false;

	if(wait_touch_down)
	{
		zfs_worker_goto_idle(zfs);
		//result = zfs1096_wait_for_irq(zfs, ZFS1096_REG_READ_INTERRUPT, ZFS1096_STATUS_REG_BIT_FINGER, 0);
		//printk("[ZET] %s scan_mode %d", __func__, scan_mode);
		//result = zfs1096_wait_for_irq(zfs, ZFS1096_REG_READ_INTERRUPT, scan_mode, 0);
		result = zfs1096_wait_for_irq(zfs, ZFS1096_REG_READ_INTERRUPT, 0);
	}else
	{
		result = zfs_new_job(zfs, ZFS_WORKER_CAPTURE_MODE);
	}
	
	return error;
}

/* -------------------------------------------------------------------- */
static int zfs_start_input(zfs_data_t *zfs /*fpc1020_data_t *fpc1020*/)
{	
	//int error = 0;
	
	if( zfs->worker.req_mode == ZFS_WORKER_INPUT_MODE)
	{
		return -EBUSY;
	}	
	
	//zfs_worker_goto_idle(zfs);		
	
	return zfs_new_job(zfs, ZFS_WORKER_INPUT_MODE);
}

/* -------------------------------------------------------------------- */

static int zfs_worker_function(void *_zfs)
{
	zfs_data_t *zfs = _zfs; /* fpc1020_data_t *fpc1020 = _fpc1020; */

	while (!kthread_should_stop()) {

		up(&zfs->worker.sem_idle);

		wait_event_interruptible(zfs->worker.wq_wait_job,
			zfs->worker.req_mode != ZFS_WORKER_IDLE_MODE);

		down(&zfs->worker.sem_idle);

		switch (zfs->worker.req_mode) {
			case ZFS_WORKER_CAPTURE_MODE:
				zfs->capture.state = ZFS_CAPTURE_STATE_PENDING;
				//pr_debug("[ZET] %s scan_mode = %02x\n", __func__, scan_mode);
				if(zfs->scan_mode == ZFS1096_STATUS_REG_BIT_FINGER)
					zfs1096_capture_task(zfs);		
				else
					zfs1096_capture_finger_key_navi_task(zfs);
				break;

			case ZFS_WORKER_INPUT_MODE:
				// if (fpc1020_capture_deferred_task(fpc1020) != -EINTR) {
					//zfs_input_enable(zfs, true);
					zfs1096_input_task(zfs);
				// }
				break;
				
			case ZFS_WORKER_RESET_MODE:
				zfs1096_mcu_reset(zfs);
				break;
				
			case ZFS_WORKER_DOWNLOAD_MODE:
				zfs1096_download_task(zfs);		
				break;
				
			case ZFS_WORKER_IDLE_MODE:
				
			case ZFS_WORKER_EXIT:
			default:
				break;
		}		

		if (zfs->worker.req_mode != ZFS_WORKER_EXIT)
			zfs->worker.req_mode = ZFS_WORKER_IDLE_MODE;
	}

	return 0;
}

/* -------------------------------------------------------------------- */

static int zfs_worker_init(zfs_data_t *zfs /* fpc1020_data_t *fpc1020 */)
{
	int error = 0;

	pr_debug("[ZET] %s\n", __func__);

	init_waitqueue_head(&zfs->worker.wq_wait_job);

	sema_init(&zfs->worker.sem_idle, 0);

	zfs->worker.req_mode = ZFS_WORKER_IDLE_MODE;
	
#if 1
	zfs->worker.thread = kthread_run(zfs_worker_function,
					   zfs, "%s",
					   ZFS1096_WORKER_THREAD_NAME);

	if (IS_ERR(zfs->worker.thread)) {
		dev_err(&zfs->pdev->dev, "kthread_run failed.\n");
		error = (int)PTR_ERR(zfs->worker.thread);
	}
#endif

	return error;
}

#endif /* ZET_CAPTURE_SUPPORT */

/* -------------------------------------------------------------------- */
static int zfs1096_open(struct inode *inode, struct file *file)
{
	zfs_data_t *zfs1096; /* fpc1020_data_t *fpc1020; */

	pr_debug("[ZET] %s\n", __func__);
	
	zfs1096 = container_of(inode->i_cdev, zfs_data_t, cdev);

	// if (down_interruptible(&zfs1096->mutex))
		// return -ERESTARTSYS;

	file->private_data = zfs1096;

	// up(&zfs1096->mutex);

	return 0;
}


/* -------------------------------------------------------------------- */
static ssize_t zfs1096_write(struct file *file, const char *buff,
					size_t count, loff_t *ppos)
{
	pr_debug("[ZET] %s\n", __func__);

	return -ENOTTY;
}

/* -------------------------------------------------------------------- */
static ssize_t zfs1096_read(struct file *file, char *buff,
				size_t count, loff_t *ppos)
{
	int error = 0;

#if 0	
	zfs_data_t *zfs1096 = file->private_data; /* fpc1020_data_t *fpc1020 = file->private_data; */
	u32 max_data;
	u32 avail_data;
	
	printk("[ZET] %s\n", __func__);

	// if (down_interruptible(&zfs1096->mutex))
		// return -ERESTARTSYS;

	if (zfs1096->capture.available_bytes > 0 && zfs1096->capture.read_pending_eof) {
		goto copy_data;
	} else {

		// if (zfs1096->capture.read_pending_eof) {
			// zfs1096->capture.read_pending_eof = false;
			// error = 0;
			// goto out;
		// }

		if (file->f_flags & O_NONBLOCK) {
			if (zfs_capture_check_ready(zfs1096)) {
				error = zfs_start_capture(zfs1096, false);
				if (error<0)
					goto out;
			}

			error = -EWOULDBLOCK;
			goto out;

		} else {
			error = zfs_start_capture(zfs1096, false);
			if (error<0)
				goto out;
		}
	}
	pr_err("[ZET] zfs1096_read++++\n");
	error = wait_event_interruptible(
			zfs1096->capture.wq_data_avail,
			(zfs1096->capture.available_bytes > 0));

	if (error)
		goto out;

	if (zfs1096->capture.last_error != 0) {
		error = zfs1096->capture.last_error;
		goto out;
	}

copy_data:
	avail_data = zfs1096->capture.available_bytes;
	max_data = (count > avail_data) ? avail_data : count;

	if (max_data) {
		error = copy_to_user(buff,
			&zfs1096->huge_buffer[zfs1096->capture.read_offset],
			max_data);

		if (error)
			goto out;

		zfs1096->capture.read_offset += max_data;
		zfs1096->capture.available_bytes -= max_data;

		error = max_data;

		if (zfs1096->capture.available_bytes == 0)
			zfs1096->capture.read_pending_eof = false;
	}

out:
	// up(&zfs1096->mutex);
	
#endif	

	return error;
}

/* -------------------------------------------------------------------- */
static int zfs1096_release(struct inode *inode, struct file *file)
{
	zfs_data_t *zfs1096 = file->private_data; /* fpc1020_data_t *fpc1020 = file->private_data; */
	int status = 0;
	
	if(0) pr_debug("[ZET] %s\n zfs1096:%p", __func__, zfs1096);
	pr_debug("[ZET] %s\n", __func__);

	///if (down_interruptible(&zfs1096->mutex))
	///	return -ERESTARTSYS;
/*
	fpc1020_start_input(fpc1020);
*/
	/// zfs_worker_goto_idle(zfs1096);
/*
	fpc1020_sleep(fpc1020, true);
*/
	///up(&zfs1096->mutex);

	return status;
}

/* -------------------------------------------------------------------- */
static unsigned int zfs1096_poll(struct file *file, poll_table *wait)
{
	zfs_data_t *zfs1096 = file->private_data; /* fpc1020_data_t *fpc1020 = file->private_data; */
	unsigned int ret = 0;
	// zfs_capture_mode_t mode = zfs1096->setup.capture_mode;
	bool blocking_op;

	// printk("[ZET]: fingerprint dev %s\n", __func__);
	
	// if (down_interruptible(&zfs1096->mutex))
		// return -ERESTARTSYS;

	if (zfs1096->capture.available_bytes > 0 && zfs1096->capture.read_pending_eof)
		ret |= (POLLIN | POLLRDNORM); // readable
	else if (zfs1096->capture.available_bytes > 0)
		ret |= POLLHUP; // readable
	else { /* available_bytes == 0 && !pending_eof */

		// blocking_op =
			// (mode == FPC1020_MODE_WAIT_AND_CAPTURE) ? true : false;
		blocking_op = true;

		switch (zfs1096->capture.state) {
		case ZFS_CAPTURE_STATE_IDLE:
			if (!blocking_op)
				ret |= POLLIN;
			break;

		case ZFS_CAPTURE_STATE_STARTED:
		case ZFS_CAPTURE_STATE_PENDING:
		case ZFS_CAPTURE_STATE_WRITE_SETTINGS:
		case ZFS_CAPTURE_STATE_WAIT_FOR_FINGER_DOWN:
		case ZFS_CAPTURE_STATE_ACQUIRE:
		case ZFS_CAPTURE_STATE_FETCH:
		case ZFS_CAPTURE_STATE_WAIT_FOR_FINGER_UP:
		case ZFS_CAPTURE_STATE_COMPLETED:
			ret |= POLLIN;

			poll_wait(file, &zfs1096->capture.wq_data_avail, wait);

			if (zfs1096->capture.read_pending_eof)
				ret |= POLLRDNORM;
			else if (blocking_op)
				ret = 0;

			break;

		case ZFS_CAPTURE_STATE_FAILED:
			if (!blocking_op)
				ret |= POLLIN;
			break;

		default:
			dev_err(&zfs1096->pdev->dev,
				"[ZET] %s unknown state\n", __func__);
			break;
		}
	}

	// up(&zfs1096->mutex);

	return ret;
}

static long zfs1096_ioctl(struct file *file, unsigned int cmd,
					unsigned long arg) {
						
	u8 __user * user_buf = (u8 __user *) arg;
	u8 buf[IOCTL_MAX_BUF_SIZE];
	u8 *tx_buf = NULL;
	u8 *rx_buf = NULL;
	// int index = 0;
	int max_data;
	// int image_size = 0;
	//int read_offset = 1;
	int tx_len = 0;
	int rx_len = 0;
	
	int i;
	int error;
	// unsigned long ret;
	zfs_data_t *zfs1096 = file->private_data; /* fpc1020_data_t *fpc1020 = file->private_data; */
	
	error = 0;
	tx_len = 0;
	rx_len = 0;	
	
	if(zfs1096->worker.req_mode == ZFS_WORKER_DOWNLOAD_MODE)
	{
		error = -ENOTBLK;
		goto exit;
	}
	
	// if (down_interruptible(&zfs1096->mutex))
		// return -ERESTARTSYS;
	
	if(copy_from_user(buf, user_buf, IOCTL_MAX_BUF_SIZE))
	{
		printk("[ZET]: zfs1096_ioctl: copy_from_user fail\n");
		return -ERESTARTSYS;
	}	
	
	//pr_debug("[ZET] %s %08x \n", __func__, cmd);
	
	switch (cmd) {
	case ZET_IOCTL_INPUT_SYNC:
	
		zfs_input_snyc(zfs1096);
		
		break;
		
	case ZET_IOCTL_INPUT_SET_EVENT:
		
		dev_err(&zfs1096->pdev->dev, "[ZET] %s input_set_event %02x %02x\n", __func__, buf[0], buf[1]);
		
		zfs_input_set_event(zfs1096, buf[0], buf[1]); // key_code, on_off
		
		break;

	case ZET_IOCTL_SET_KEY_EVENT:
		
		dev_err(&zfs1096->pdev->dev, "[ZET] %s ", __func__);
		
		for(i=0 ;i<8; i++)
		{
			zfs1096->key_event[i] = buf[i];
			dev_err(&zfs1096->pdev->dev, " %02x ", buf[i]);
		}
		
		dev_err(&zfs1096->pdev->dev, " \n");
		
		break;
		
	case ZET_IOCTL_ENABLE_KEY_REPORT:
		
		if(buf[0] == 0x0)
			zfs_input_enable(zfs1096, false);
		else
			zfs_input_enable(zfs1096, true);
		
		break;
		
	case ZET_IOCTL_READ_KEY_DATA:
	
		tx_len = 2;
		tx_buf = kzalloc(tx_len, GFP_KERNEL);
		
		tx_buf[0] = zfs1096->input.key_status;
		tx_buf[1] = zfs1096->input.navi_state;
		
		error = copy_to_user(user_buf, tx_buf, tx_len);
		
		break;
	
	case ZET_IOCTL_GET_SCAN_MODE:
		
		tx_len = 2;
		tx_buf = kzalloc(tx_len, GFP_KERNEL);
		
		tx_buf[0] = zfs1096->scan_mode;
		
		error = copy_to_user(user_buf, tx_buf, tx_len);		
	
		break;
		
	case ZET_IOCTL_SET_SCAN_MODE:
		
		zfs1096->scan_mode = buf[0];
	
		break;		
		
	case ZET_IOCTL_HARDWARE_RESET:
	
		/// clear all interrupt lock
		zfs1096->interrupt_done = true;
		wake_up_interruptible(&zfs1096->wq_irq_return);
		
		/// unlock frame lock
		zfs1096->capture.state = ZFS_CAPTURE_STATE_IDLE;
		zfs1096->capture.available_bytes  = 0;
		zfs1096->capture.read_offset = 0;
		zfs1096->capture.read_pending_eof = false;			
		up(&zfs1096->sem_frame);			
	
		error = zfs_gpio_set(RST_HIGh);
		if (error) {
			dev_err(&zfs1096->pdev->dev, "[ZET] Could not set gpio high.\n");
			/*return -EIO;*/
		}	
		udelay(ZFS1096_RESET_HIGH1_US);
		
		error = zfs_gpio_set(RST_LOW);
		if (error) {
			dev_err(&zfs1096->pdev->dev, "[ZET] Could not set gpio low.\n");
			/*return -EIO;*/
		}
		udelay(ZFS1096_RESET_LOW_US);
		
		error = zfs_gpio_set(RST_HIGh);
		if (error) {
			dev_err(&zfs1096->pdev->dev, "[ZET] Could not set gpio high.\n");
			/*return -EIO;*/
		}			
		mdelay(ZFS1096_RESET_HIGH2_MS);
		
		break;		
		
	case ZET_IOCTL_READ_DATA:
		//dev_err(&zfs1096->pdev->dev, "[ZET] %s ZET_IOCTL_READ_DATA \n", __func__);
		
		down(&zfs1096->sem_frame);
		
		if (zfs1096->capture.available_bytes > 0 && zfs1096->capture.read_pending_eof)
		{
			max_data = zfs1096->capture.available_bytes;
			// tx_buf = kzalloc(max_data+2, GFP_KERNEL);			
			error = copy_to_user(user_buf, zfs1096->huge_buffer, max_data);
			//zfs1096->capture.available_bytes = 0;
			//zfs1096->capture.read_pending_eof = false;
		}else
		{
			error = -ENOENT;
		}
		up(&zfs1096->sem_frame);

		break;
		
	case ZET_IOCTL_CHECK_FRAME_READY:
		//dev_err(&zfs1096->pdev->dev, "[ZET] %s ZET_IOCTL_CHECK_FRAME_READY \n", __func__);
		tx_len = 4;
		tx_buf = kzalloc(tx_len, GFP_KERNEL);
		if(down_trylock(&zfs1096->sem_frame)) // frame is in the processing task
		{
			// dev_err(&zfs1096->pdev->dev, "[ZET] %s ZET_IOCTL_CHECK_FRAME_READY processing.........\n", __func__);
			tx_buf[0] = ZFS_FRAME_BUSY & 0xFF;
			tx_buf[1] = zfs1096->zfs1096_image_header[0] & 0xFF;
			tx_buf[2] = zfs1096->zfs1096_image_header[1] & 0xFF;
			tx_buf[3] = zfs1096->zfs1096_image_header[2] & 0xFF;
			error = copy_to_user(user_buf, tx_buf, tx_len);
		}else
		{
			// dev_err(&zfs1096->pdev->dev, "[ZET] %s ZET_IOCTL_CHECK_FRAME_READY down_trylock\n", __func__);
			if (zfs1096->capture.available_bytes > 0 && zfs1096->capture.read_pending_eof)
			{
				// dev_err(&zfs1096->pdev->dev, "[ZET] %s ZET_IOCTL_CHECK_FRAME_READY good.........\n", __func__);
				tx_buf[0] = ZFS_FRAME_READY & 0xFF;
				tx_buf[1] = zfs1096->zfs1096_image_header[0] & 0xFF;
				tx_buf[2] = zfs1096->zfs1096_image_header[1] & 0xFF;
				tx_buf[3] = zfs1096->zfs1096_image_header[2] & 0xFF;
				error = copy_to_user(user_buf, tx_buf, tx_len);
			}else if( zfs1096->capture.state == ZFS_CAPTURE_STATE_FAILED )
			{
				tx_buf[0] = ZFS_FRAME_FAIL & 0xFF;		
				tx_buf[1] = zfs1096->zfs1096_image_header[0] & 0xFF;
				tx_buf[2] = zfs1096->zfs1096_image_header[1] & 0xFF;
				tx_buf[3] = zfs1096->zfs1096_image_header[2] & 0xFF;				
				error = copy_to_user(user_buf, tx_buf, tx_len);
			}else if( zfs1096->capture.state == ZFS_CAPTURE_STATE_COMPLETED )
			{				
				tx_buf[0] = ZFS_FRAME_UNVIABLE & 0xFF;
				tx_buf[1] = zfs1096->zfs1096_image_header[0] & 0xFF;
				tx_buf[2] = zfs1096->zfs1096_image_header[1] & 0xFF;
				tx_buf[3] = zfs1096->zfs1096_image_header[2] & 0xFF;				
				error = copy_to_user(user_buf, tx_buf, tx_len);				
			}else
			{
				tx_buf[0] = ZFS_FRAME_BUSY & 0xFF;
				tx_buf[1] = zfs1096->zfs1096_image_header[0] & 0xFF;
				tx_buf[2] = zfs1096->zfs1096_image_header[1] & 0xFF;
				tx_buf[3] = zfs1096->zfs1096_image_header[2] & 0xFF;
				error = copy_to_user(user_buf, tx_buf, tx_len);
			}
				
			up(&zfs1096->sem_frame);
			// dev_err(&zfs1096->pdev->dev, "[ZET] %s ZET_IOCTL_CHECK_FRAME_READY up\n", __func__);
		}
			
		break;		
		
	case ZET_IOCTL_START_SECTORS:
#ifdef ZET_CAPTURE_SUPPORT
		if(zfs1096->worker.req_mode != ZFS_WORKER_IDLE_MODE)
			zfs_worker_goto_idle(zfs1096);
#endif		
		zfs1096->capture.read_pending_eof = false;
		zfs1096->capture.available_bytes = 0;	
		zfs1096->capture.zfs_sector = 0;
		zfs1096->capture.state = ZFS_CAPTURE_STATE_FETCH;
		break;
		
	case ZET_IOCTL_INTERRUPT_DONE:		
		tx_len = 2;
		tx_buf = kzalloc(tx_len, GFP_KERNEL);
		
		if(zfs1096->interrupt_done)
			tx_buf[0] = 1;
		else
			tx_buf[0] = 0;		
		
		error = copy_to_user(user_buf, tx_buf, tx_len);	
		break;		
		
	case ZET_IOCTL_SPI_SUPPORT:
		tx_len = 2;
		tx_buf = kzalloc(tx_len, GFP_KERNEL);
		
		tx_buf[0] = zfs1096_spi_support;		
		
		error = copy_to_user(user_buf, tx_buf, tx_len);		
	
		break;
		
	case ZET_IOCTL_GET_HW_INFO:
		
		tx_len = 21;
		tx_buf = kzalloc(tx_len, GFP_KERNEL);
		
		/*for(i=0 ;i<sizeof(zfs1096_driver_info); i++)
		{
			tx_buf[i] = zfs1096_driver_info[i];
		}		
		
		for(i=0 ;i<sizeof(zfs1096->zfs1096_fw_info); i++)
		{
			tx_buf[i+sizeof(zfs1096_driver_info)] = zfs1096->zfs1096_fw_info[i];
		}	*/
		
		memcpy(tx_buf, zfs1096_driver_info, 4);
		
		memcpy(tx_buf+4, zfs1096->zfs1096_fw_info, 18);
		
		error = copy_to_user(user_buf, tx_buf, tx_len);		
		
		break;		

	case ZET_IOCTL_FRAMEWORK_MSG:
		
		zfs_send_uevent( &zfs1096->pdev->dev, ZET_UEVENT_FRAMEWORK_MSG );
		
		//if( buf[0] == FRAMEWORK_IMAGE )
		//	zfs_send_uevent( &zfs1096->pdev->dev, ZET_UEVENT_FRAMEWORK_IMAGE );
		//else if( buf[0] == FRAMEWORK_ENROLL )
		//	zfs_send_uevent( &zfs1096->pdev->dev, ZET_UEVENT_FRAMEWORK_ENROLL );
		//else if( buf[0] == FRAMEWORK_VERIFY )
		//	zfs_send_uevent( &zfs1096->pdev->dev, ZET_UEVENT_FRAMEWORK_VERIFY );
		
		break;

#ifdef ZET_REE_SPI_SUPPORT
	case ZET_IOCTL_READ_REGISTER:
		//dev_err(&zfs1096->pdev->dev, "[ZET] %s ZET_IOCTL_READ_REGISTER \n", __func__);		
		
		tx_len = 0;
		rx_len = 0;
		tx_len = buf[0] & 0xFF;
		tx_len = ( tx_len << 8 ) | ( buf[1] & 0xFF );
		rx_len = buf[2] & 0xFF;
		rx_len = ( rx_len << 8 ) | ( buf[3] & 0xFF );
		
		tx_buf = kzalloc(tx_len, GFP_KERNEL);
		rx_buf = kzalloc(tx_len+rx_len, GFP_KERNEL);
		
		// memcpy(tx_buf, &buf[4], tx_len);
		memcpy(tx_buf, buf+4, tx_len);
		
		zfs_reg_read(zfs1096, tx_len, tx_len+rx_len, tx_buf, rx_buf);
		
		error = copy_to_user(user_buf, rx_buf, tx_len+rx_len);		
				
		break;
		
	case ZET_IOCTL_WRITE_REGISTER:
		//dev_err(&zfs1096->pdev->dev, "[ZET] %s ZET_IOCTL_WRITE_REGISTER \n", __func__);

#ifdef ZET_CAPTURE_SUPPORT
		if(zfs1096->worker.req_mode != ZFS_WORKER_IDLE_MODE)
			zfs_worker_goto_idle(zfs1096);
#endif			
		
		zfs1096->interrupt_done = false;		
		
		tx_len = 0;
		rx_len = 0;
		tx_len = buf[0] & 0xFF;
		tx_len = ( tx_len << 8 ) | ( buf[1] & 0xFF );
		rx_len = buf[2] & 0xFF;
		rx_len = ( rx_len << 8 ) | ( buf[3] & 0xFF );
		
		tx_buf = kzalloc(tx_len, GFP_KERNEL);
		
		// memcpy(tx_buf, &buf[4], tx_len);
		memcpy(tx_buf, buf+4, tx_len);
		
		error = zfs_reg_write(zfs1096, tx_len, 0, tx_buf, NULL);
		
		break;
		
	case ZET_IOCTL_READ_IMAGE_SECTOR:
		
		tx_len = 0;
		rx_len = 0;
		tx_len = buf[0] & 0xFF;
		tx_len = ( tx_len << 8 ) | ( buf[1] & 0xFF );
		rx_len = buf[2] & 0xFF;
		rx_len = ( rx_len << 8 ) | ( buf[3] & 0xFF );
		
		tx_buf = kzalloc(tx_len, GFP_KERNEL);
		rx_buf = kzalloc(5, GFP_KERNEL);
		
		// memcpy(tx_buf, &buf[4], tx_len);
		memcpy(tx_buf, buf+4, tx_len);
		
		error = zfs_reg_read_sector(zfs1096, tx_len, rx_len, tx_buf, rx_buf);
		
		error = copy_to_user(user_buf, rx_buf, 5);		
		
		break;
		
#endif /* ZET_REE_SPI_SUPPORT */	

#ifdef ZET_CAPTURE_SUPPORT
		
	case ZET_IOCTL_START_CAPTURE:
		// dev_err(&zfs1096->pdev->dev, "[ZET] %s ZET_IOCTL_START_CAPTURE \n", __func__);	
		error = zfs_start_capture(zfs1096, false);
		break;
		
	case ZET_IOCTL_ABORT_CAPTURE:
		// dev_err(&zfs1096->pdev->dev, "[ZET] %s ZET_IOCTL_ABORT_CAPTURE \n", __func__);
		zfs_worker_goto_idle(zfs1096);
		break;
		
	case ZET_IOCTL_START_KEY_NAVI:
		error = zfs_start_input(zfs1096);
		break;
		
	case ZET_IOCTL_ABORT_KEY_NAVI:
		zfs_worker_goto_idle(zfs1096);
		break;
		
	case ZET_IOCTL_WAIT_TOUCH:
		dev_err(&zfs1096->pdev->dev, "[ZET] %s ZET_IOCTL_WAIT_TOUCH \n", __func__);
		error = zfs_start_capture(zfs1096, true);
		break;
	
	case ZET_IOCTL_CANCEL_WAIT_TOUCH:
		dev_err(&zfs1096->pdev->dev, "[ZET] %s ZET_IOCTL_CANCEL_WAIT_TOUCH \n", __func__);
		zfs_worker_goto_idle(zfs1096);
		break;
		
	case ZET_IOCTL_SOFT_RESET:
		zfs_new_job(zfs1096, ZFS_WORKER_RESET_MODE);
		break;

	case ZET_IOCTL_FLASH_DOWNLOAD:
	
		//memcpy(flash_path, buf, sizeof(flash_path));
		strcpy(flash_path, buf);
		dev_err(&zfs1096->pdev->dev, "[ZET] flash_path = %s \n", flash_path);

		zfs1096->flash_update = false;
		
		if(zfs_fw_load(flash_path, zfs1096))
		{
			zfs_new_job(zfs1096, ZFS_WORKER_DOWNLOAD_MODE);
		}else
		{
			error = -ENOENT;
		}			
		
		break;

#endif /* ZET_CAPTURE_SUPPORT */		
		
	default:
		dev_err(&zfs1096->pdev->dev, "[ZET] %s default \n", __func__);	
		error = -ENOIOCTLCMD;
	}		
	
exit:
	
	if(rx_buf)
		kfree(rx_buf);
	
	if(tx_buf)
		kfree(tx_buf);	
	
	// up(&zfs1096->mutex);
	
	return error;
}	

static const struct file_operations zfs_fops = {
	.owner          = THIS_MODULE,
	.open           = zfs1096_open,
	.write          = zfs1096_write,
	.read           = zfs1096_read,
	.release        = zfs1096_release,
	.poll           = zfs1096_poll,
	.unlocked_ioctl = zfs1096_ioctl,
};

/* -------------------------------------------------------------------- */
static ssize_t zfs_show_attr_setup(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	zfs_data_t *zfs1096 = dev_get_drvdata(dev); /* fpc1020_data_t *fpc1020 = dev_get_drvdata(dev) */
	struct zfs_attribute *zfs_attr; /* struct fpc1020_attribute *fpc_attr; */
	int val = -1;
	// int mux;

	zfs_attr = container_of(attr, struct zfs_attribute, attr);

	/*
	mux = zfs1096->setup.capture_settings_mux;
	
	if (fpc_attr->offset == offsetof(fpc1020_setup_t, adc_gain))
		val = fpc1020->setup.adc_gain[mux];

	else if (fpc_attr->offset == offsetof(fpc1020_setup_t, adc_shift))
		val = fpc1020->setup.adc_shift[mux];

	else if (fpc_attr->offset == offsetof(fpc1020_setup_t, pxl_ctrl))
		val = fpc1020->setup.pxl_ctrl[mux];

	else if (fpc_attr->offset == offsetof(fpc1020_setup_t, capture_mode))
		val = fpc1020->setup.capture_mode;

	else */ if (zfs_attr->offset == offsetof(zfs_setup_t, capture_count))
		val = zfs1096->setup.capture_count;

	/*
	else if (fpc_attr->offset == offsetof(fpc1020_setup_t, capture_settings_mux))
		val = fpc1020->setup.capture_settings_mux;

	else if (fpc_attr->offset == offsetof(fpc1020_setup_t, capture_row_start))
		val = fpc1020->setup.capture_row_start;

	else if (fpc_attr->offset == offsetof(fpc1020_setup_t, capture_row_count))
		val = fpc1020->setup.capture_row_count;

	else if (fpc_attr->offset == offsetof(fpc1020_setup_t, capture_col_start))
		val = fpc1020->setup.capture_col_start;

	else if (fpc_attr->offset == offsetof(fpc1020_setup_t, capture_col_groups))
		val = fpc1020->setup.capture_col_groups;
	*/

	if (val >= 0)
		return scnprintf(buf, PAGE_SIZE, "%i\n", val);

	return -ENOENT;
}

/* -------------------------------------------------------------------- */
static ssize_t zfs_store_attr_setup(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	zfs_data_t *zfs1096 = dev_get_drvdata(dev); /* fpc1020_data_t *fpc1020 = dev_get_drvdata(dev) */
	u64 val;
	int error = kstrtou64(buf, 0, &val);
	/*int mux;
	int column_groups = fpc1020->chip.pixel_columns / fpc1020->chip.adc_group_size;*/

	struct zfs_attribute *zfs_attr; /* struct fpc1020_attribute *fpc_attr; */

	zfs_attr = container_of(attr, struct zfs_attribute, attr);

	/*mux = fpc1020->setup.capture_settings_mux;*/

	if (!error) {
		/*if (fpc_attr->offset ==
			offsetof(fpc1020_setup_t, adc_gain)) {

			fpc1020->setup.adc_gain[mux] = (u8)val;

		} else if (fpc_attr->offset ==
			offsetof(fpc1020_setup_t, adc_shift)) {

			fpc1020->setup.adc_shift[mux] = (u8)val;

		} else if (fpc_attr->offset ==
			offsetof(fpc1020_setup_t, pxl_ctrl)) {

			fpc1020->setup.pxl_ctrl[mux] = (u16)val;

		} else if (fpc_attr->offset ==
			offsetof(fpc1020_setup_t, capture_mode)) {

			fpc1020->setup.capture_mode =
					(fpc1020_capture_mode_t)val;

		} else */if (zfs_attr->offset ==
			offsetof(zfs_setup_t, capture_count)) {

			if (zfs_check_in_range_u64
				(val, 1, ZFS1096_BUFFER_MAX_IMAGES)) {

				zfs1096->setup.capture_count = (u8)val;
			} else
				return -EINVAL;
		/*
		} else if (fpc_attr->offset ==
			offsetof(fpc1020_setup_t, capture_settings_mux)) {

			if (fpc1020_check_in_range_u64
				(val, 0, (FPC1020_BUFFER_MAX_IMAGES - 1))) {

				fpc1020->setup.capture_settings_mux = (u8)val;
			} else
				return -EINVAL;

		} else if (fpc_attr->offset ==
			offsetof(fpc1020_setup_t, capture_row_start)) {

			if (fpc1020_check_in_range_u64
				(val, 0, (fpc1020->chip.pixel_rows - 1))) {

				fpc1020->setup.capture_row_start = (u8)val;
			} else
				return -EINVAL;

		} else if (fpc_attr->offset ==
			offsetof(fpc1020_setup_t, capture_row_count)) {

			if (fpc1020_check_in_range_u64
				(val, 1, fpc1020->chip.pixel_rows)) {

				fpc1020->setup.capture_row_count = (u8)val;
			} else
				return -EINVAL;

		} else if (fpc_attr->offset ==
			offsetof(fpc1020_setup_t, capture_col_start)) {

			if (fpc1020_check_in_range_u64
				(val, 0, (column_groups - 1))) {

				fpc1020->setup.capture_col_start = (u8)val;
			} else
				return -EINVAL;


		} else if (fpc_attr->offset ==
			offsetof(fpc1020_setup_t, capture_col_groups)) {

			if (fpc1020_check_in_range_u64
				(val, 1, column_groups)) {

				fpc1020->setup.capture_col_groups = (u8)val;
			} else
				return -EINVAL;
		*/
		} else
			return -ENOENT;

		return strnlen(buf, count);
	}
	return error;
}

/* -------------------------------------------------------------------- */
static ssize_t zfs_show_attr_diag(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	zfs_data_t *zfs1096 = dev_get_drvdata(dev); /* fpc1020_data_t *fpc1020 = dev_get_drvdata(dev) */
	struct zfs_attribute *zfs_attr; /* struct fpc1020_attribute *fpc_attr; */
	
	u64 val = 0;
	int error = 0;
	bool is_buffer = false;
	
	/*u8 u8_buffer[FPC1020_REG_MAX_SIZE];
	char hex_string[sizeof("0x") + (FPC1020_REG_MAX_SIZE * 2)];*/

	zfs1096 = dev_get_drvdata(dev);

	zfs_attr = container_of(attr, struct zfs_attribute, attr);

	switch (zfs_attr->offset) {
	case offsetof(zfs_diag_t, chip_id):
		return scnprintf(buf,
				PAGE_SIZE,
				"%s rev.%d\n",
				zfs_hw_id_text(zfs1096),
				zfs1096->chip.revision);
		break;
	/*case offsetof(fpc1020_diag_t, selftest):
		val = (u64)fpc1020_selftest_short(fpc1020);
		break;
	case offsetof(fpc1020_diag_t, spi_register):
		val = (int)fpc1020->diag.spi_register;
		break;
	case offsetof(fpc1020_diag_t, spi_regsize):
		val = (int)fpc1020->diag.spi_regsize;
		break;
	case offsetof(fpc1020_diag_t, spi_data):
		is_buffer = (fpc1020->diag.spi_regsize > sizeof(val));

		if (!is_buffer) {
			error = fpc1020_spi_debug_value_read(fpc1020, &val);
		} else {
			error = fpc1020_spi_debug_buffer_read(fpc1020,
							u8_buffer,
							sizeof(u8_buffer));
		}
		break;
	case offsetof(fpc1020_diag_t, last_capture_time):
		val = (int)fpc1020->diag.last_capture_time;
		break;
	case offsetof(fpc1020_diag_t, finger_present_status):
		error = fpc1020_get_finger_present_status(fpc1020);
		if (error >= 0) {
			val = (int)error;
			error = 0;
		}
		break;*/
	}

	if (error >= 0 && !is_buffer) {
		return scnprintf(buf,
				PAGE_SIZE,
				"%lu\n",
				(unsigned long int)val);
	}

	/*if (error >= 0 && is_buffer) {
		fpc1020_spi_debug_buffer_to_hex_string(hex_string,
						u8_buffer,
						fpc1020->diag.spi_regsize);

		return scnprintf(buf, PAGE_SIZE, "%s\n", hex_string);
	}*/

	return error;
}

/* -------------------------------------------------------------------- */
static ssize_t zfs_store_attr_diag(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	
	// zfs_data_t *zfs1096 = dev_get_drvdata(dev); /* fpc1020_data_t *fpc1020 = dev_get_drvdata(dev) */
	
	// u64 val;
	int error = 0;

	struct zfs_attribute *zfs_attr; /* struct fpc1020_attribute *fpc_attr; */

	zfs_attr = container_of(attr, struct zfs_attribute, attr);

	/*if (fpc_attr->offset == offsetof(fpc1020_diag_t, spi_register)) {
		error = kstrtou64(buf, 0, &val);

		if (!error) {
			error = fpc1020_spi_debug_select(fpc1020,
							(fpc1020_reg_t)val);
		}
	} else if (fpc_attr->offset == offsetof(fpc1020_diag_t, spi_data)) {

		if (fpc1020->diag.spi_regsize <= sizeof(val)) {
			error = kstrtou64(buf, 0, &val);

			if (!error)
				error = fpc1020_spi_debug_value_write(fpc1020,
									 val);
		} else {
			error = fpc1020_spi_debug_buffer_write(fpc1020,
								buf,
								count);
		}
	} else
		error = -EPERM;*/

	return (error < 0) ? error : strnlen(buf, count);
}

/* -------------------------------------------------------------------- */
static int zfs_create_class(zfs_data_t *zfs /*fpc1020_data_t *fpc1020*/)
{
	int error = 0;

	dev_dbg(&zfs->pdev->dev, "[ZET] %s\n", __func__);

	zfs->class = class_create(THIS_MODULE, ZFS1096_CLASS_NAME);

	if (IS_ERR(zfs->class)) {
		dev_err(&zfs->pdev->dev, "[ZET] failed to create class.\n");
		error = PTR_ERR(zfs->class);
	}

	return error;
}

/* -------------------------------------------------------------------- */
static int zfs_create_device(zfs_data_t *zfs)
{
	int error = 0;

	dev_dbg(&zfs->pdev->dev, "[ZET] %s\n", __func__);

	if (ZFS1096_MAJOR > 0) {
		zfs->devno = MKDEV(ZFS1096_MAJOR, zfs1096_device_count++);

		error = register_chrdev_region(zfs->devno,
						1,
						ZFS1096_DEV_NAME);
	} else {
		error = alloc_chrdev_region(&zfs->devno,
					zfs1096_device_count++,
					1,
					ZFS1096_DEV_NAME);
	}

	if (error < 0) {
		dev_err(&zfs->pdev->dev,
				"[ZET] %s: FAILED %d.\n", __func__, error);
		goto out;

	} else {
		dev_info(&zfs->pdev->dev, "[ZET] %s: major=%d, minor=%d\n",
						__func__,
						MAJOR(zfs->devno),
						MINOR(zfs->devno));
	}

	zfs->device = device_create(zfs->class, NULL, zfs->devno,
						NULL, "%s", ZFS1096_DEV_NAME);

	if (IS_ERR(zfs->device)) {
		dev_err(&zfs->pdev->dev, "[ZET] device_create failed.\n");
		error = PTR_ERR(zfs->device);
	}
out:
	return error;
}

/* -------------------------------------------------------------------- */
static int zfs_manage_sysfs(zfs_data_t *zfs, 
				struct spi_device *spi, bool create)
{
	int error = 0;

	if (create) {
		dev_dbg(&zfs->pdev->dev, "[ZET] %s create\n", __func__);

		error = sysfs_create_group(&spi->dev.kobj,
					&zfs_setup_attr_group);

		if (error) {
			dev_err(&zfs->pdev->dev,
				"[ZET] sysf_create_group failed.\n");
			return error;
		}

		error = sysfs_create_group(&spi->dev.kobj,
					&zfs_diag_attr_group);

		if (error) {
			sysfs_remove_group(&spi->dev.kobj,
					&zfs_setup_attr_group);

			dev_err(&zfs->pdev->dev,
				"[ZET] sysf_create_group failed.\n");

			return error;
		}
	} else {
		dev_dbg(&zfs->pdev->dev, "[ZET] %s remove\n", __func__);

		sysfs_remove_group(&spi->dev.kobj, &zfs_setup_attr_group);
		sysfs_remove_group(&spi->dev.kobj, &zfs_diag_attr_group);
	}

	return error;
}

/* -------------------------------------------------------------------- */
int zfs_gpio_set(int cmd)
{
	switch (cmd) {
	case RST_HIGh:
		pr_err("[ZET] %s set zfs_pin_rst_high \n", __func__);
#ifdef FPC_MTK
		if(zfs_pin_rst_high_success)
			pinctrl_select_state(zfs_pinctrl, zfs_pin_rst_high);
#else
		gpio_direction_output(zet_gpio_reset,1);
#endif
		return 0;
		break;
	case RST_LOW:
		pr_err("[ZET] %s set zfs_pin_rst_low \n", __func__);
#ifdef FPC_MTK
		if(zfs_pin_rst_low_success)
			pinctrl_select_state(zfs_pinctrl, zfs_pin_rst_low);
#else
		gpio_direction_output(zet_gpio_reset,0);
#endif
		return 0;
		break;
	case CS_HIGH:
		pr_err("[ZET] %s set zfs_pin_cs_high \n", __func__);
#ifdef FPC_MTK
		if(zfs_pin_cs_high_success)
			pinctrl_select_state(zfs_pinctrl, zfs_pin_cs_high);
#else
		gpio_direction_output(zet_gpio_cs,1);
#endif
		return 0;
		break;		
	case CS_LOW:
		pr_err("[ZET] %s set zfs_pin_cs_low \n", __func__);
#ifdef FPC_MTK
		if(zfs_pin_cs_low_success)
			pinctrl_select_state(zfs_pinctrl, zfs_pin_cs_low);
#else
		gpio_direction_output(zet_gpio_cs,0);
#endif
		return 0;
		break;			
	}

	return 1;
}

/* -------------------------------------------------------------------- */

static int zfs_send_uevent(struct device* dev, char* event)
{
	int ret;
	struct kobject *zet_kobj = &dev->kobj;
	char* uevent_env_str[2] = {"WAKEUP", NULL};
	
	uevent_env_str[0] = event;

	ret = kobject_uevent_env(zet_kobj, KOBJ_CHANGE, uevent_env_str);
	
	if(ret){
		dev_err(dev, "[ZET] %s: Send uevent failed, ret=%d\n", __func__, ret);
	}

	return 0;
}

/* -------------------------------------------------------------------- */

static void zfs_interrupt_work(struct work_struct *work)
{
	zfs_data_t *zfs = container_of(work, zfs_data_t, zfs_int_work);
	
#ifdef ZET_CAPTURE_SUPPORT
	
	u8 status[10] = { 0x0 };
	
	if(zfs->worker.req_mode == ZFS_WORKER_INPUT_MODE)
	{
		zfs_send_uevent( &zfs->pdev->dev, ZET_UEVENT_TOUCH_KEY );
		
	}else
	{
		if(zfs->capture.state == ZFS_CAPTURE_STATE_STARTED)
		{
			//zfs_send_uevent( &zfs->pdev->dev, ZET_UEVENT_TOUCH_DOWN );
			
			// zfs_new_job(zfs, ZFS_WORKER_CAPTURE_MODE);
			
			zfs1096_read_irq(zfs, true, ZFS1096_REG_READ_STABLE, status);
			
			if( status[2] != 0x0 || status[3] != 0x0 )
			{
				zfs_send_uevent( &zfs->pdev->dev, ZET_UEVENT_TOUCH_KEY );
				
				zfs1096_input_report(zfs, status+2);
				
			}
			
			// if( status[1] == 0x1 )
				zfs_send_uevent( &zfs->pdev->dev, ZET_UEVENT_TOUCH_DOWN );
			
		}else
		{
			zfs_send_uevent( &zfs->pdev->dev, ZET_UEVENT_INTERRUPT );
		}		
	}
#else
	
	zfs_send_uevent( &zfs->pdev->dev, ZET_UEVENT_INTERRUPT );

#endif /* ZET_CAPTURE_SUPPORT */
	
}

/* -------------------------------------------------------------------- */
irqreturn_t zfs_interrupt(int irq, void *_zfs)
{
	zfs_data_t *zfs = _zfs; /* fpc1020 */
#ifdef FPC_MTK
	pr_err("zfs1096_interrupt enter\n");
	/*disable_irq_nosync(fpc1020->irq);*/
	zfs->interrupt_done = true;
	wake_up_interruptible(&zfs->wq_irq_return);
	//pr_err("zfs1096interrupt wake_up !!\n");
	queue_work(zfs->zfs_int_workqueue,&zfs->zfs_int_work);
	return IRQ_HANDLED;
#else
	if (gpio_get_value(zfs->irq_gpio)) {
		zfs->interrupt_done = true;
		wake_up_interruptible(&zfs->wq_irq_return);
		queue_work(zfs->zfs_int_workqueue,&zfs->zfs_int_work);
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
#endif

}

/* -------------------------------------------------------------------- */
static int zfs_param_init(zfs_data_t *zfs,
					struct zfs_platform_data *pdata)
					/*(fpc1020_data_t *fpc1020,
					struct fpc1020_platform_data *pdata)*/
{
	
#if 0
	zfs->force_hwid = pdata->force_hwid;
	zfs->use_regulator_for_bezel = pdata->use_regulator_for_bezel;
#endif

	return 0;
}

/* -------------------------------------------------------------------- */
static int zfs_irq_init(zfs_data_t *zfs,
					struct zfs_platform_data *pdata)
					/*(fpc1020_data_t *fpc1020,
					struct fpc1020_platform_data *pdata)*/
{
	int error = 0;
#ifdef FPC_MTK
	//unsigned int gpiopin, debounce;
	unsigned int zfs_irq = 0;
	//u32 ints[2] = {0, 0};
	struct device_node *node;

	//node = of_find_compatible_node(NULL, NULL, "mediatek,zfs1096");
	node = of_find_compatible_node(NULL, NULL, "mediatek,fpc1145");
	
	if (node) {		
		/*of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		gpiopin = ints[0];
		debounce = ints[1];*/
		/*mt_gpio_set_debounce(gpiopin, debounce);*/	
	
		zfs_irq = irq_of_parse_and_map(node, 0);		
		pr_err("[ZET] _irq = %u\n", zfs_irq);
		zfs->irq = zfs_irq;		
		
	} else
		pr_err("[ZET] %s can't find compatible node\n", __func__);

#else
	//albert removed for hikey
	//if (gpio_is_valid(pdata->irq_gpio)) {
	if (gpio_is_valid(zet_gpio_irq)) {

		//printk("[ZET] Assign IRQ -> GPIO%d\n",pdata->irq_gpio);

		//albert removed for hikey
		//error = gpio_request(pdata->irq_gpio, "zfs_irq");
		error = gpio_request(zet_gpio_irq, "irq-gpio");

		if (error) {
			dev_err(&zfs->pdev->dev,
				"[ZET] gpio_request (irq) failed.\n");

			return error;
		}

		//albert removed for hikey
		//zfs->irq_gpio = pdata->irq_gpio;

		error = gpio_direction_input(zet_gpio_irq);

		if (error) {
			dev_err(&zfs->pdev->dev,
				"[ZET] gpio_direction_input (irq) failed.\n");
			return error;
		}
	} else {
		return -EINVAL;
	}

	zet_irq_num = gpio_to_irq(zet_gpio_irq);
	zfs->irq = zet_irq_num;

	if (zfs->irq < 0) {
		dev_err(&zfs->pdev->dev, "[ZET] gpio_to_irq failed.\n");
		error = zfs->irq;
		return error;
	}
#endif

	error = request_irq(zfs->irq, zfs_interrupt, IRQF_TRIGGER_RISING, "zfs1096", zfs);
	// error = request_irq(zfs->irq, zfs_interrupt, IRQF_TRIGGER_RISING, "fpc1020", zfs);
	
	zfs->interrupt_done = false;
	
	INIT_WORK(&zfs->zfs_int_work, zfs_interrupt_work);	
	
	zfs->zfs_int_workqueue = create_singlethread_workqueue("zfs_int_workqueue");

	if (error) {
		dev_err(&zfs->pdev->dev,
			"[ZET] request_irq %i failed.\n",
			zfs->irq);

		zfs->irq = -EINVAL;

		return error;
	}

	return error;
}

#ifdef ZET_REE_SPI_SUPPORT

/* -------------------------------------------------------------------- */
static int zfs_spi_setup(zfs_data_t *zfs,
					struct zfs_platform_data *pdata)
{
	int error = 0;
	
#ifdef FPC_MTK	
	struct mt_chip_conf *spi_par;
#endif

	pr_debug("%s\n", __func__);

#ifdef FPC_MTK
	zfs->pdev->controller_data = (void *)&spi_conf;
	spi_par = &spi_conf;

	spi_par->setuptime = 20;//20;
	spi_par->holdtime = 20;//20;
	spi_par->high_time = 8;//50--1m; //10--6m   15--4m   20--3m  30--2m  [ 60--1m 120--0.5m  300--0.2m]
	spi_par->low_time = 8;//50;
	spi_par->cs_idletime = 2;//5
	spi_par->rx_mlsb = 1;
	spi_par->tx_mlsb = 1;
	spi_par->tx_endian = 0;
	spi_par->rx_endian = 0;
	spi_par->cpol = 0;
	spi_par->cpha = 0;
	spi_par->com_mod = DMA_TRANSFER; //FIFO_TRANSFER;
	spi_par->pause = 1;
	spi_par->finish_intr = 1;
	spi_par->deassert = 0;

	zfs->pdev->mode = SPI_MODE_0;
	zfs->pdev->bits_per_word = 8;
	zfs->pdev->chip_select = 0;

#else
	//albert added for hikey
    zfs->pdev->mode = SPI_MODE_0; //SPI_MODE_3;
    zfs->pdev->bits_per_word = 8;
    zfs->pdev->max_speed_hz = 8000000; //jmt setting
#endif

	error = spi_setup(zfs->pdev);

	if (error) {
		dev_err(&zfs->pdev->dev, "[ZET] spi_setup failed\n");
		goto out_err;
	}
	
	///< ALBERT_MANUAL_CS 
	//error = fpc1020_gpio_set(CS_HIGH);

#if 0 //albert=def MANUAL_CS
	if (gpio_is_valid(pdata->cs_gpio)) {

		dev_info(&zfs->pdev->dev,
			"Assign SPI.CS -> GPIO%d\n",
			pdata->cs_gpio);

		error = gpio_request(pdata->cs_gpio, "fpc1020_cs");
		if (error) {
			dev_err(&zfs->pdev->dev,
				"gpio_request (cs) failed.\n");

			goto out_err;
		}

		zfs->cs_gpio = pdata->cs_gpio;

		error = gpio_direction_output(zfs->cs_gpio, 1);
		if (error) {
			dev_err(&zfs->pdev->dev,
				"gpio_direction_output(cs) failed.\n");
			goto out_err;
		}
	} else {
		error = -EINVAL;
	}
#endif

out_err:
	return error;
}

#endif /* ZET_REE_SPI_SUPPORT */

/* -------------------------------------------------------------------- */

static int zfs_gpio_init(void)
{
#ifdef FPC_MTK
	struct device_node *node;
	struct platform_device *dev;
	int ret;

	// node = of_find_compatible_node(NULL, NULL, "mediatek,zfs1096");
	node = of_find_compatible_node(NULL, NULL, "mediatek,fpc1145");
	dev = of_find_device_by_node(node);

	if (node) {
		zfs_pinctrl = devm_pinctrl_get(&dev->dev);
		if (IS_ERR(zfs_pinctrl)) {
			ret = PTR_ERR(zfs_pinctrl);
			dev_err(&dev->dev, "[ZET] zfs cannot find pinctrl\n");
			//return 1; //fail
		}
		
		/*zfs_pin_default = pinctrl_lookup_state(zfs_pinctrl, "default");
		if (IS_ERR(zfs_pin_default)) {
			ret = PTR_ERR(zfs_pin_default);
			dev_err(&dev->dev, "zfs cannot find pinctrl default\n");
			//return 1; //fail
		}		*/

		zfs_pin_eint = pinctrl_lookup_state(zfs_pinctrl, "pin_int");
		if (IS_ERR(zfs_pin_eint)) {
			ret = PTR_ERR(zfs_pin_eint);
			dev_err(&dev->dev, "[ZET] zfs cannot find pinctrl pin_int\n");
			zfs_pin_eint_success = false;
			//return 1; //fail
		}else
			zfs_pin_eint_success = true;
		
		zfs_pin_rst_high = pinctrl_lookup_state(zfs_pinctrl, "pin_rst_high");
		if (IS_ERR(zfs_pin_rst_high)) {
			ret = PTR_ERR(zfs_pin_rst_high);
			dev_err(&dev->dev, "[ZET] zfs cannot find pinctrl reset\n");
			zfs_pin_rst_high_success = false;
			//return 1; //fail
		}else
			zfs_pin_rst_high_success = true;
		
		zfs_pin_rst_low = pinctrl_lookup_state(zfs_pinctrl, "pin_rst_low");
		if (IS_ERR(zfs_pin_rst_low)) {
			ret = PTR_ERR(zfs_pin_rst_low);
			dev_err(&dev->dev, "[ZET] zfs cannot find pinctrl reset\n");
			zfs_pin_rst_low_success = false;
			//return 1; //fail
		}else
			zfs_pin_rst_low_success = true;
#ifdef CS_MANUAL
		zfs_pin_cs_high = pinctrl_lookup_state(zfs_pinctrl, "pin_cs_high");
		if (IS_ERR(zfs_pin_cs_high)) {
			ret = PTR_ERR(zfs_pin_cs_high);
			dev_err(&dev->dev, "[ZET] zfs cannot find pinctrl cs\n");
			zfs_pin_cs_high_success = false;
			//return 1; //fail
		}else
			zfs_pin_cs_high_success = true;
		
		zfs_pin_cs_low = pinctrl_lookup_state(zfs_pinctrl, "pin_cs_low");
		if (IS_ERR(zfs_pin_cs_low)) {
			ret = PTR_ERR(zfs_pin_cs_low);
			dev_err(&dev->dev, "[ZET] zfs cannot find pinctrl cs\n");
			zfs_pin_cs_low_success = false;
			//return 1; //fail
		}else
			zfs_pin_cs_low_success = true;
#endif
				
		return 0;//success
	}
	
	pr_err("[ZET] %s can't find compatible node\n", __func__);
	
	return 1;//fail

#else  //for hikey

	struct device_node *node;
	int ret;

	node = of_find_compatible_node(NULL, NULL, "zet,zfs1096");

	zet_gpio_reset = of_get_named_gpio(node, "zetfp,reset-gpio", 0);
	if(!gpio_is_valid(zet_gpio_reset)) {
		pr_err("[ZET] %s: failed to get reset gpio\n", __func__);
		return 1;
	}
	
	ret = gpio_request(zet_gpio_reset, "reset-gpio");
	if (ret) {
		pr_err("[ZET] %s: gpio_request failed.[reset-gpio], ret=%d\n", __func__, ret);
		return 1;
	}

	zet_gpio_irq = of_get_named_gpio(node, "zetfp,irq-gpio", 0);
	if(!gpio_is_valid(zet_gpio_irq)) {
		pr_err("[ZET] %s: failed to get irq gpio\n", __func__);
		return 1;
	}
	
	/*ret = gpio_request(zet_gpio_irq, "irq-gpio");
	if (ret) {
		pr_err("%s: gpio_request failed, ret = %d\n", __func__, ret);
		return 1;
	}*/
	
	/*
	
	zet_gpio_cs = of_get_named_gpio(node, "zetfp,cs-gpio", 0);
	if(!gpio_is_valid(zet_gpio_reset)) {
		pr_err("[ZET] %s: failed to get reset gpio\n", __func__);
		return 1;
	}
	
	ret = gpio_request(zet_gpio_cs, "cs-gpio");
	if (ret) {
		pr_err("[ZET] %s: gpio_request failed.[cs-gpio], ret=%d\n", __func__, ret);
		return 1;
	}	
	
	*/
	
	return 0;

#endif
}

/* -------------------------------------------------------------------- */
static int zfs_suspend(struct device *dev)
{
	
#if 0
	fpc1020_data_t *fpc1020 = dev_get_drvdata(dev);

	dev_dbg(&fpc1020->pdev->dev, "%s\n", __func__);

	fpc1020_worker_goto_idle(fpc1020);

	return fpc1020_sleep(fpc1020, true);
#endif

	zfs_data_t *zfs = dev_get_drvdata(dev);
	zfs_send_uevent( &zfs->pdev->dev, ZET_UEVENT_SUSPEND );

	printk("[ZET] %s++++++\n", __func__);
	return 0;
}

/* -------------------------------------------------------------------- */
static int zfs_resume(struct device *dev)
{
	
#if 0
	fpc1020_data_t *fpc1020 = dev_get_drvdata(dev);

	dev_dbg(&fpc1020->pdev->dev, "%s\n", __func__);

	if (fpc1020_wake_up(fpc1020)) {
		if (fpc1020->input.enabled)
			fpc1020_start_input(fpc1020);
	}
#endif

	zfs_data_t *zfs = dev_get_drvdata(dev);
	zfs_send_uevent( &zfs->pdev->dev, ZET_UEVENT_RESUME );
	
	printk("[ZET] %s++++++\n", __func__);
	return 0;
}

/* -------------------------------------------------------------------- */

static int zfs_spi_remove(struct spi_device *spi)
{
	zfs_data_t *zfs = spi_get_drvdata(spi);	

	pr_debug("%s\n", __func__);

	zfs_manage_sysfs(zfs, spi, false);

	cdev_del(&zfs->cdev);

	unregister_chrdev_region(zfs->devno, 1);

	return 0;
}

/* -------------------------------------------------------------------- */
static int zfs_spi_probe(struct spi_device *spi)
{
	struct zfs_platform_data *zfs_spi_pdata; // *fpc1020_pdata;
	int error = 0;
	zfs_data_t *zfs_dev = NULL; // *fpc1020
	size_t buffer_size; 	
	int i;
	
#ifdef FPC_MTK
	struct device *dev = &spi->dev;
	struct regulator *vmch = regulator_get(dev,"vmch");
#endif

	pr_err("[ZET] zfs_spi_probe enter++++++\n");

#ifdef ZET_REE_SPI_SUPPORT	
	zfs1096_spi_support |= (1<<0);
#endif

#ifdef ZET_CAPTURE_SUPPORT
	zfs1096_spi_support |= (1<<1);
#endif
	
	pr_err("[ZET] DRIVER : ");
		 
	for(i=0; i<sizeof(zfs1096_driver_info); i++)
	{
		pr_err("0x%02x ", zfs1096_driver_info[i]);
	}
	
	pr_err(" SPI_SUPPORT : 0x%02x\n", zfs1096_spi_support);	
	
	error = zfs_gpio_init();	
	if (error) {
		dev_err(&spi->dev, "[ZET] Could not do gpio init.\n");
		//return -EIO;
	}

#ifdef FPC_MTK
////< power on 3.3V
	regulator_set_voltage(vmch,3300000,3300000);
	error = regulator_enable(vmch);
////<
#endif

	zfs_dev = kzalloc(sizeof(*zfs_dev), GFP_KERNEL);
	if (!zfs_dev) {
		
		dev_err(&spi->dev,
		"[ZET] failed to allocate memory for struct zfs_platform_data\n");
		
		return -ENOMEM;
	}

	pr_alert("[ZET] %s\n", __func__);
	
	spi_set_drvdata(spi, zfs_dev);
	zfs_dev->pdev = spi;
	zfs_dev->spi_freq_khz = 8000u;

#ifdef FPC_MTK
	zfs_dev->reset_gpio = -EINVAL;
	zfs_dev->irq_gpio   = -EINVAL;
	zfs_dev->cs_gpio    = -EINVAL;
	zfs_dev->irq        = -EINVAL;
#else
	//albert add for hikey
	zfs_dev->reset_gpio = zet_gpio_reset;
	//zfs_dev->cs_gpio    = zet_gpio_cs;
	zfs_dev->irq_gpio   = zet_gpio_irq;
	//zfs_dev->irq        = -EINVAL;
#endif
	
	zfs_dev->scan_mode = ZFS1096_STATUS_REG_BIT_FINGER | ZFS1096_STATUS_REG_BIT_KEY;

	init_waitqueue_head(&zfs_dev->wq_irq_return);

	error = zfs_init_capture(zfs_dev);
	if (error)
		goto err;

	zfs_spi_pdata = spi->dev.platform_data;

	error = zfs_param_init(zfs_dev, zfs_spi_pdata);
	if (error)
		goto err;
	
	//zfs_dev->zfs1096_fw_info = kzalloc(17, GFP_KERNEL);
	
#ifdef ZET_REE_SPI_SUPPORT
	
#ifdef ZET_BOOTLOADER

	if(zfs_dev->flash_buffer == NULL)
	{
  		zfs_dev->flash_buffer = kmalloc(MAX_FLASH_BUF_SIZE, GFP_KERNEL);
		memset(zfs_dev->flash_buffer,0xFF,MAX_FLASH_BUF_SIZE);
	}	
	
	memcpy(zfs_dev->flash_buffer, zeitec_zfs1096_firmware, sizeof(zeitec_zfs1096_firmware));
	
	zfs1096_download_task(zfs_dev);	

#endif	

#endif
	
	error = zfs_gpio_set(RST_HIGh);
	if (error) {
		dev_err(&spi->dev, "[ZET] Could not set gpio.\n");
		/*return -EIO;*/
	}	
	udelay(ZFS1096_RESET_HIGH1_US);
	
	error = zfs_gpio_set(RST_LOW);
	if (error) {
		dev_err(&spi->dev, "[ZET] Could not set gpio.\n");
		/*return -EIO;*/
	}
	udelay(ZFS1096_RESET_LOW_US);
	
	error = zfs_gpio_set(RST_HIGh);
	if (error) {
		dev_err(&spi->dev, "[ZET] Could not set gpio.\n");
		/*return -EIO;*/
	}	
	mdelay(ZFS1096_RESET_HIGH2_MS);
	///< reset operation end
	
	error = zfs_irq_init(zfs_dev, zfs_spi_pdata);
	if (error)
		goto err;

#ifdef ZET_REE_SPI_SUPPORT	

	error = zfs_spi_setup(zfs_dev, zfs_spi_pdata);
	if (error)
		goto err;
	
#endif /* ZET_REE_SPI_SUPPORT */

#ifdef ZET_CAPTURE_SUPPORT	

	error = zfs_check_hw_id(zfs_dev);
	if (error)
		goto err;
	
#endif /* ZET_CAPTURE_SUPPORT */
	
#ifdef FPC_MTK
	zfs_dev->spi_freq_khz = zfs_dev->chip.spi_max_khz;
#endif

	pr_err("[ZET] Req. SPI frequency : %d kHz.\n",zfs_dev->spi_freq_khz);

	buffer_size = zfs_calc_huge_buffer_minsize(zfs_dev);
	
	error = zfs_manage_huge_buffer(zfs_dev, buffer_size);
	
	printk("[ZET] buffer_size#2 = %d\n",(int)zfs_dev->huge_buffer_size);	
	if (error)
		goto err;
	
	/*error = zfs_setup_defaults(zfs_dev);
	if (error)
		goto err;*/

	error = zfs_create_class(zfs_dev);
	/*if (error)
		goto err;*/

	error = zfs_create_device(zfs_dev);
	/*if (error)
		goto err;*/

	sema_init(&zfs_dev->mutex, 0);
	sema_init(&zfs_dev->sem_frame, 0);

	error = zfs_manage_sysfs(zfs_dev, spi, true);
	/*if (error)
		goto err;*/

	cdev_init(&zfs_dev->cdev, &zfs_fops);
	zfs_dev->cdev.owner = THIS_MODULE;

	error = cdev_add(&zfs_dev->cdev, zfs_dev->devno, 1);
	if (error) {
		dev_err(&zfs_dev->pdev->dev, "cdev_add failed.\n");
		goto err_chrdev;
	}
	
	error = zfs_input_init(zfs_dev);
	if (error)
		goto err_cdev;	
	
#ifdef ZET_CAPTURE_SUPPORT	

	error = zfs_worker_init(zfs_dev);
	if (error)
		goto err_cdev;

	error = zfs1096_read_hw_info(zfs_dev);
	if(error == 0)
	{
		printk("0xB2 0xF9 read successfully or not support!");
	}
	
	zfs_input_enable(zfs_dev, true);
	
	error = zfs_start_input(zfs_dev);
	/*if (error)
		goto err_cdev;*/
	
#endif /* ZET_CAPTURE_SUPPORT */

	up(&zfs_dev->mutex);
	up(&zfs_dev->sem_frame);		

	pr_err("[ZET] zfs_spi_probe probe done+++++++\n");
	
	return 0;

err_cdev:
	cdev_del(&zfs_dev->cdev);

err_chrdev:
	unregister_chrdev_region(zfs_dev->devno, 1);

	zfs_manage_sysfs(zfs_dev, spi, false);

err:
	//fpc1020_cleanup(fpc1020, spi);

	return error;
}

/* -------------------------------------------------------------------- */
/* External interface							*/
/* -------------------------------------------------------------------- */

#if 1
static SIMPLE_DEV_PM_OPS(zfs_pm, zfs_suspend, zfs_resume);
#else
static const struct dev_pm_ops zfs_pm = {
	.suspend = zfs_suspend,
	.resume = zfs_resume
};
#endif

#ifdef CONFIG_OF
static struct of_device_id zfs_of_match[] = {
	//{ .compatible = "zet,zfs1096", },
	{ .compatible = "fpc,fpc1020", },
	{}
};

MODULE_DEVICE_TABLE(of, zfs_of_match);
#endif

static struct spi_driver zfs_spi_driver = {
	.driver = {
		.name	= ZFS1096_DEV_NAME,
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
		.pm     = &zfs_pm,
#ifdef CONFIG_OF
		.of_match_table = zfs_of_match,
#endif
	},
	.probe	= zfs_spi_probe,
	.remove	= zfs_spi_remove,
};

static int __init zfs_spi_init(void)
{
	pr_err("[ZET] zfs_init+++++++++++++++++++\n");
	if (spi_register_driver(&zfs_spi_driver))
		return -EINVAL;
	pr_err("[ZET] zfs_init end++++++++++++++++++++++++\n");
	return 0;
}

/* -------------------------------------------------------------------- */
static void __exit zfs_spi_exit(void)
{
	pr_debug("[ZET] %s\n", __func__);

	spi_unregister_driver(&zfs_spi_driver);
}

module_init(zfs_spi_init);
module_exit(zfs_spi_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Albert <albert.lin@zeitecsemi.com>");
MODULE_DESCRIPTION("ZFS1096 touch sensor driver.");

