/**
 * @file drivers/input/touchscreen/zet6221.c
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * ZEITEC Semiconductor Co., Ltd
  * @author JLJuang <JL.Juang@zeitecsemi.com>
 * @note Copyright (c) 2010, Zeitec Semiconductor Ltd., all rights reserved.
 * @version $Revision: 7 $
 * @note
*/

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>

#include "tpd_zet_common.h"
#include "zet_core.h"
#include "tpd.h"
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/input/mt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/input/mt.h>

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
#include <mach/md32_ipi.h>
#include <mach/md32_helper.h>
#endif

#include "zet6221_fw.h"
#include "zet6231_fw.h"
#include "zet6223_fw.h"
#include "zet6270_fw.h"
#include "zet7100_fw.h"
#include "zet6251_fw.h"

////////@zet.Begin
/*
#include <linux/file.h> 
#include <cust_eint.h>
#include <linux/input/mt.h>

#ifndef TPD_NO_GPIO 
#include "cust_gpio_usage.h"
#endif ///< for TPD_NO_GPIO

#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include <mach/upmu_common.h>
#include "cust_gpio_usage.h"

//#define TPD_PROXIMITY
#ifdef TPD_PROXIMITY
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#endif

#include <linux/virkey.h>
#include <linux/suspend.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/hw_module_info.h>
#include <asm/dma-mapping.h>
#include "upmu_common.h"
#include <mt-plat/mt_gpio.h>
#include "tpd.h"
*/
////////@zet.End

#include "upmu_common.h"
#include "mt_gpio.h"
//#include "tpd.h"

//#define ZET_DEBUG

//#define FEATURE_WDT_API
///=============================================================================================///
/// 
///=============================================================================================///
///---------------------------------------------------------------------------------///
///  1. FW Upgrade
///---------------------------------------------------------------------------------///
//#define FEATURE_FW_CHECK_SUM       ///< 
#define FEATURE_FW_UPGRADE			///< 
#ifdef FEATURE_FW_UPGRADE
	//#define FEATURE_FW_SIGNATURE		///<
	//#define FEATURE_FW_COMPARE		///<
	#define FEATURE_FW_UPGRADE_RESUME	///< 
		#define FEATURE_FW_CHECK_SUM       ///< 
	//#define FEATURE_FW_SKIP_FF		///< 
#endif ///< for FEATURE_FW_UPGRADE

///---------------------------------------------------------------------------------///
///  2. MTK 8 Byte mode
///---------------------------------------------------------------------------------///
//#define FEATURE_MTK_I2C_8BYTES_MODE


///---------------------------------------------------------------------------------///
///  3. Hardware check only and do no FW upgrade
///---------------------------------------------------------------------------------///
//#define FEATURE_HW_CHECK_ONLY			///< 

///---------------------------------------------------------------------------------///
///  4. Read TP information (B2 Command)
///---------------------------------------------------------------------------------///
//#define FEATURE_TPINFO				///< 
///---------------------------------------------------------------------------------///
///  5. Touch Panel Virtual Key
///---------------------------------------------------------------------------------///
//#define FEATURE_VIRTUAL_KEY
//#define FEATURE_MTK_VIRTUAL_KEY

///---------------------------------------------------------------------------------///
///  6. Multi-touch type B
///---------------------------------------------------------------------------------///
#define FEATURE_MT_TYPE_B				///< Android multitouch type B protocol
//#define FEATURE_BTN_TOUCH			///< 
#ifdef FEATURE_MT_TYPE_B
	//#define FEATURE_LIGHT_LOAD_REPORT_MODE   ///<  
#endif ///< for FEATURE_MT_TYPE_B
#define PRESSURE_CONST	(1)

///---------------------------------------------------------------------------------///
///  7. Coordinate translation
///---------------------------------------------------------------------------------///
//#define FEATURE_TRANSLATE_ENABLE		///< 

///---------------------------------------------------------------------------------///
///  8. Auto Zoom translation
///---------------------------------------------------------------------------------///
//#define FEATURE_AUTO_ZOOM_ENABLE			///< FW to driver XY auto zoom in

///=============================================================================================///
/// Macro Definition
///=============================================================================================///

///---------------------------------------------------------------------------------///
///  Virtual Key
///---------------------------------------------------------------------------------///
#ifdef FEATURE_VIRTUAL_KEY
  #define TP_AA_X_MAX		480	///< X resolution of TP VA(View Area)
  #define TP_AA_Y_MAX		854   ///< Y resolution of TP VA(View Area)
#endif ///< for FEATURE_VIRTUAL_KEY

///---------------------------------------------------------------------------------///
///  9. Hihg impedance mode (ZET6221)
///---------------------------------------------------------------------------------///
//#define FEATURE_HIGH_IMPEDENCE_MODE  		///< ZET6221 high impedance 
#define IMPENDENCE_BYTE 	(0xf1)	///< ZET6221 only High Impendence Mode : (8M) 0xf1 (16M) 0xf2 

///---------------------------------------------------------------------------------///
///  10. Finger number 
///---------------------------------------------------------------------------------///
#define FINGER_NUMBER 				(5)		///< 
#define MT_FINGER_NUMBER 			(5)	
///---------------------------------------------------------------------------------///
///  11. key number 
///---------------------------------------------------------------------------------///
#define KEY_NUMBER 		(0)		///< 

///---------------------------------------------------------------------------------///
///  12. Finger up debounce count  
///---------------------------------------------------------------------------------///
#define DEBOUNCE_NUMBER		(1)		///< 

///=========================================================================================///
///  13. Device Name 
///=========================================================================================///
#define ZET_TS_ID_NAME		"zet6221-ts"

///---------------------------------------------------------------------------------///
///  12.1 key lock not report finger
///---------------------------------------------------------------------------------///
//#define FEATURE_KEY_LOCK_FINGER  ///< ?
///=========================================================================================///
///  16. Charge mode  
///=========================================================================================///
//#define FEATURE_CHARGER_MODE		///< 
#ifdef TP_CHARGE_MODE
#define FEATURE_CHARGER_MODE
#include <linux/workqueue.h>

#endif

#ifdef FEATURE_CHARGER_MODE
extern int charger_on;			///< 
#else ///< for FEATURE_CHARGER_MODE
int charger_on  		= 0;	///< 
#endif ///< for FEATURE_CHARGER_MODE

///---------------------------------------------------------------------------------///
///  18. IOCTRL Debug
///---------------------------------------------------------------------------------///
#define FEATURE_IDEV_OUT_ENABLE
#define FEATURE_MBASE_OUT_ENABLE
#define FEATURE_MDEV_OUT_ENABLE
#define FEATURE_INFO_OUT_EANBLE
#define FEATURE_IBASE_OUT_ENABLE
#define FEATURE_FPC_OPEN_ENABLE
#define FEATURE_FPC_SHORT_ENABLE

///---------------------------------------------------------------------------------///
///  19. TRACE SETTING GPIO  
///---------------------------------------------------------------------------------///
#define FEATURE_TRACE_SETTING_GPIO
#ifdef FEATURE_TRACE_SETTING_GPIO
        #define FEATRUE_TRACE_GPIO_OUTPUT
        #define FEATRUE_TRACE_GPIO_INPUT
        #ifdef FEATRUE_TRACE_GPIO_INPUT
                //#define FEATRUE_TRACE_SENSOR_ID
        #endif ///< for FEATRUE_TRACE_GPIO_INPUT
#endif ///< for FEATURE_TRACE_SETTING_GPIO

#ifdef FEATRUE_TRACE_SENSOR_ID
	#include "zet6223_fw_01.h"
	#include "zet6223_fw_02.h"
	#include "zet6223_fw_03.h"
	#include "zet6231_fw_01.h"
	#include "zet6231_fw_02.h"
	#include "zet6231_fw_03.h"
	#include "zet6251_fw_01.h"
	#include "zet6251_fw_02.h"
	#include "zet6251_fw_03.h"
#endif ///< for FEATURE_TRACE_SETTING_GPIO

///---------------------------------------------------------------------------------///
///  20. suspend/resume clean finger
///---------------------------------------------------------------------------------///
#define FEATURE_SUSPEND_CLEAN_FINGER

///---------------------------------------------------------------------------------///
///  21. int pin free 
///---------------------------------------------------------------------------------///
//#define FEATURE_INT_FREE

///---------------------------------------------------------------------------------///
///  22. Fram rate
///---------------------------------------------------------------------------------///
#define FEATURE_FRAM_RATE

///---------------------------------------------------------------------------------///
///  24. ESD CHECKSUM
///---------------------------------------------------------------------------------///
//#define ESD_CHECKSUM
#ifdef ESD_CHECKSUM
	int fw_checksum_value = 0;
	int zet_get_fw_checksum(u8 * pFW);
	int zet_get_flash_checksum(void);
#endif

///---------------------------------------------------------------------------------///
///  22. key event to Finger event
///---------------------------------------------------------------------------------///
//#define FEATURE_KEY_TO_FINGER


///---------------------------------------------------------------------------------///
///  14. I2C setting
///---------------------------------------------------------------------------------///
#define MAX_TRANSACTION_LENGTH 	(8)
#define I2C_DEVICE_ADDRESS_LEN 	(2)
//#define MAX_I2C_TRANSFER_SIZE (MAX_TRANSACTION_LENGTH - I2C_DEVICE_ADDRESS_LEN)
#define MAX_I2C_TRANSFER_SIZE 	(MAX_TRANSACTION_LENGTH)
#define ZET6221_I2C_FREQ	(400)


///=============================================================================================///
/// 14. Flash control 
///=============================================================================================///
#define CMD_WRITE_PASSWORD			(0x20)
	#define CMD_PASSWORD_HIBYTE			(0xC5)
	#define CMD_PASSWORD_LOBYTE			(0x9D)

	#define CMD_PASSWORD_1K_HIBYTE			(0xB9)
	#define CMD_PASSWORD_1K_LOBYTE			(0xA3)
	
	#define CMD_WRITE_PASSWORD_LEN			(3)
#define CMD_WRITE_CODE_OPTION			(0x21)
#define CMD_WRITE_PROGRAM			(0x22)
#define CMD_PAGE_ERASE				(0x23)
	#define CMD_PAGE_ERASE_LEN         		(2)
#define CMD_MASS_ERASE				(0x24)
#define CMD_PAGE_READ_PROGRAM			(0x25)
	#define CMD_PAGE_READ_PROGRAM_LEN		(2)
#define CMD_MASS_READ_PROGRAM			(0x26)
#define CMD_READ_CODE_OPTION			(0x27)
#define CMD_ERASE_CODE_OPTION			(0x28)
#define CMD_RESET_MCU				(0x29)
#define CMD_OUTPUT_CLOCK			(0x2A)
#define CMD_WRITE_SFR				(0x2B)
#define CMD_READ_SFR				(0x2C)
	#define SFR_UNLOCK_FLASH			(0x3D)
	#define SFR_LOCK_FLASH				(0x7D)	
#define CMD_ERASE_SPP				(0x2D)
#define CMD_WRITE_SPP				(0x2E)
#define CMD_READ_SPP				(0x2F)
#define CMD_PROG_INF				(0x30)
#define CMD_PROG_MAIN				(0x31)
#define CMD_PROG_CHECK_SUM			(0x36)
#define CMD_PROG_GET_CHECK_SUM			(0x37)
#define CMD_OUTPUT_CLOCK1			(0x3B)
#define CMD_FILL_FIFO				(0x60)
#define CMD_READ_FIFO				(0x61)

#define FLASH_PAGE_LEN				(128)

#define  FLASH_SIZE_ZET6221			(0x4000)
#define  FLASH_SIZE_ZET6223			(0x10000)
#define  FLASH_SIZE_ZET6231			(0x8000)
#define  FLASH_SIZE_ZET6270			(0x0A000)
#define  FLASH_SIZE_ZET7100			(0x0F000)

#define CMD_71xx_WRITER					(0x58)
#define CMD_71xx_READER					(0x8F)		
#define FLASH_PAGE_LEN_71xx				(256)	
#define CMD_SECTOR_ERASE				(0x5C)

///=============================================================================================///
/// GPIO Input/Output command
///=============================================================================================///
#define GPIO_TRACE_OUTPUT_SET   		(0xD0)
#define GPIO_TRACE_OUTPUT_GET   		(0xD1)
#define GPIO_TRACE_OUTPUT_CNT  	        	(0xD2)
	#define TRACE_GPIO1_INDEX               	(0x01)
	#define TRACE_GPIO2_INDEX               	(0x02)
	#define TRACE_GPIO3_INDEX               	(0x04)
	#define TRACE_GPIO4_INDEX               	(0x08)
	#define TRACE_GPIO5_INDEX               	(0x10)
	#define TRACE_GPIO6_INDEX               	(0x20)	
	#define TRACE_GPIO7_INDEX               	(0x40)
	#define TRACE_GPIO8_INDEX               	(0x80)
#define GPIO_TRACE_INPUT_GET    		(0xD3)
#define GPIO_TRACE_INPUT_CNT    		(0xD4)
	#define SENID_00		        	(0x00)
	#define SENID_01				(0x01)
	#define SENID_02				(0x02)
	#define SENID_03				(0x03)	
///=============================================================================================///
/// Macro Definition
///=============================================================================================///
#define MAX_FLASH_BUF_SIZE			(0x10000)
#define MAX_DATA_FLASH_BUF_SIZE	                (0x400)
#define DATA_FLASH_START_ADDR                   (0x7C00)
#define SENSOR_ID_INDEX_ADDR                    (0x7C8E)
#define DATA_FLASH_START_ID		        (248)
#define SENID_MAX_CNT                           (4)
#define SENID_MAX_INDEX                         (15)
#define PROJECT_CODE_MAX_CNT                    (8)
#define FINGER_REPROT_DATA_HEADER		(0x3C)
#define INT_FREE_DATA_HEADER			(0x3B)
#define FINGER_PACK_SIZE			(4)
#define FINGER_HEADER_SHIFT			(3)
/// for debug INT
#define GPIO_BASE                		(0x01c20800)
#define GPIO_RANGE               		(0x400)
#define PH2_CTRL_OFFSET          		(0x104)
#define PH_DATA_OFFSET          		(0x10c)
///=========================================================================================///
///  13. TP related define : configured for all tp
///=========================================================================================///
/// Boolean definition
#define TRUE 			(1)
#define FALSE 			(0)

/// Origin definition
#define ORIGIN_TOP_RIGHT	(0)
#define ORIGIN_TOP_LEFT  	(1)
#define ORIGIN_BOTTOM_RIGHT	(2)
#define ORIGIN_BOTTOM_LEFT	(3)
#define ORIGIN					(ORIGIN_BOTTOM_RIGHT)
/// Max key number
#define MAX_KEY_NUMBER    	(8)

/// Max finger number
#define MAX_FINGER_NUMBER	(16)
/// X, Y Resolution
//#define FW_X_RESOLUTION				(720)		///< the FW setting X resolution
//#define FW_Y_RESOLUTION				(1280)		///< the FW setting Y resolution



//#define LCD_WIDTH  simple_strtoul(LCM_WIDTH, NULL, 0)
//#define LCD_HEIGHT  simple_strtoul(LCM_HEIGHT, NULL, 0)
#define FW_X_RESOLUTION		720		///< the FW setting X resolution
#define FW_Y_RESOLUTION		1280		///< the FW setting Y resolution
#define X_MAX	 		720		///< X resolution of TP AA(Action Area)
#define Y_MAX 			1280		///< Y resolution of TP AA(Action Area)
#define LCM_X_RES 	720		// X resolution of LCM
#define LCM_Y_RES 	1280		// Y resolution of LCM

#define TPD_POINT_INFO_LEN  	(4) 
#define TPD_MAX_POINTS      	(FINGER_NUMBER)
#define S_POLLING_TIME  	(100)

///---------------------------------------------------------------------------------///
///  Model Type
///---------------------------------------------------------------------------------///
#define MODEL_ZET6221				(0)
#define MODEL_ZET6223				(1)
#define MODEL_ZET6231				(2)
#define MODEL_ZET6241				(3)
#define MODEL_ZET6251				(4)
#define MODEL_ZET6270				(5)
#define MODEL_ZET7130				(6)
#define MODEL_ZET7150				(7)


///=========================================================================================///
///  Rom Type   jack.chen add by  20140519
///=========================================================================================///
#define ROM_TYPE_UNKNOWN			(0x00)
#define ROM_TYPE_SRAM				(0x02)
#define ROM_TYPE_OTP				(0x06)
#define ROM_TYPE_FLASH				(0x0F)


///=============================================================================================///
/// Device numbers
///=============================================================================================///
//#define REGISTER_CHRDEV_REGION
#define ZET_RW_IIC_DRV  						"zet_i2c_dev"
#define ZET_RW_IIC_DRV_MAJOR 					210    

#ifdef REGISTER_CHRDEV_REGION
static int zet_rw_iic_drv_major = ZET_RW_IIC_DRV_MAJOR;

struct zet_rw_i2c_dev 
{
	struct cdev cdev;
	struct mutex rw_i2c_mutex;
	struct i2c_client *client;
};
struct zet_rw_i2c_dev *zet_rw_i2c_dev_tt;
#endif

#define I2C_MINORS 				(256)		///< 
#define I2C_MAJOR 				ZET_RW_IIC_DRV_MAJOR //(126)		///< updated by albert @ 20160613

#define BUF_LEN 				(80)       ///< Max length of the message from the device 


///=========================================================================================///
///  17. Signature
///=========================================================================================///
#ifdef FEATURE_FW_SIGNATURE
#define SIG_PAGE_ID             		(255)   		///< 
#define SIG_DATA_ADDR           		(128  - SIG_DATA_LEN)   ///< 
#define SIG_DATA_LEN            		(4)    		 	///< 
static const u8 sig_data[SIG_DATA_LEN] = {'Z', 'e', 'i', 'T'};

#endif ///< for FEATURE_FW_SIGNATURE

///=============================================================================================///
/// IOCTL control Definition
///=============================================================================================///
#define ZET_IOCTL_CMD_FLASH_READ		(20)
#define ZET_IOCTL_CMD_FLASH_WRITE		(21)
#define ZET_IOCTL_CMD_RST      			(22)
#define ZET_IOCTL_CMD_RST_HIGH 		   	(23)
#define ZET_IOCTL_CMD_RST_LOW    		(24)

#define ZET_IOCTL_CMD_DYNAMIC			(25)

#define ZET_IOCTL_CMD_FW_FILE_PATH_GET		(26)
#define ZET_IOCTL_CMD_FW_FILE_PATH_SET   	(27)

#define ZET_IOCTL_CMD_MDEV   			(28)
#define ZET_IOCTL_CMD_MDEV_GET   		(29)

#define ZET_IOCTL_CMD_TRAN_TYPE_PATH_GET	(30)
#define ZET_IOCTL_CMD_TRAN_TYPE_PATH_SET	(31)

#define ZET_IOCTL_CMD_IDEV   			(32)
#define ZET_IOCTL_CMD_IDEV_GET   		(33)

#define ZET_IOCTL_CMD_MBASE   			(34)
#define ZET_IOCTL_CMD_MBASE_GET  		(35)

#define ZET_IOCTL_CMD_INFO_SET			(36)
#define ZET_IOCTL_CMD_INFO_GET			(37)

#define ZET_IOCTL_CMD_TRACE_X_SET		(38)
#define ZET_IOCTL_CMD_TRACE_X_GET		(39)
	
#define ZET_IOCTL_CMD_TRACE_Y_SET		(40)
#define ZET_IOCTL_CMD_TRACE_Y_GET		(41)

#define ZET_IOCTL_CMD_IBASE   			(42)
#define ZET_IOCTL_CMD_IBASE_GET   		(43)

#define ZET_IOCTL_CMD_DRIVER_VER_GET		(44)
#define ZET_IOCTL_CMD_MBASE_EXTERN_GET  	(45)

#define ZET_IOCTL_CMD_GPIO_HIGH			(46)
#define ZET_IOCTL_CMD_GPIO_LOW			(47)

#define ZET_IOCTL_CMD_SENID_GET		        (48)
#define ZET_IOCTL_CMD_PCODE_GET		        (49)

#define ZET_IOCTL_CMD_TRACE_X_NAME_SET	        (50)
#define ZET_IOCTL_CMD_TRACE_X_NAME_GET	        (51)

#define ZET_IOCTL_CMD_TRACE_Y_NAME_SET	        (52)
#define ZET_IOCTL_CMD_TRACE_Y_NAME_GET	        (53)

#define ZET_IOCTL_CMD_WRITE_CMD		        (54)
#define ZET_IOCTL_CMD_UI_FINGER		        (55)

#define ZET_IOCTL_CMD_FRAM_RATE		        (56)

#define ZET_IOCTL_CMD_FPC_OPEN_SET		(57)
#define ZET_IOCTL_CMD_FPC_OPEN_GET		(58)

#define ZET_IOCTL_CMD_FPC_SHORT_SET		(59)
#define ZET_IOCTL_CMD_FPC_SHORT_GET		(60)
///   jack.chen add by  20140519
#define ZET_IOCTL_CMD_GET_REPORT_DATA_1	(61)
#define ZET_IOCTL_CMD_GET_REPORT_DATA_2	(62)

#define ZET_IOCTL_CMD_REPORT_SIZE_GET	(63)
#define ZET_IOCTL_CMD_REPORT_SIZE_SET	(64)  
#define ZET_IOCTL_CMD_TRAN_TYPE_SET		(65)
#define ZET_IOCTL_CMD_FINGER_XMAX_YMAX_GET (66)

#define IOCTL_MAX_BUF_SIZE          		(1024)

///----------------------------------------------------///
/// IOCTL ACTION
///----------------------------------------------------///
#define IOCTL_ACTION_NONE			(0)
#define IOCTL_ACTION_FLASH_DUMP			(1<<0)

static int ioctl_action = IOCTL_ACTION_NONE;

///=============================================================================================///
///  Transfer type
///=============================================================================================///
#define TRAN_TYPE_DYNAMIC		        (0x00)
#define TRAN_TYPE_MUTUAL_SCAN_BASE         	(0x01)
#define TRAN_TYPE_MUTUAL_SCAN_DEV           	(0x02)
#define TRAN_TYPE_INIT_SCAN_BASE 		(0x03)
#define TRAN_TYPE_INIT_SCAN_DEV		      	(0x04)
#define TRAN_TYPE_KEY_MUTUAL_SCAN_BASE		(0x05)
#define TRAN_TYPE_KEY_MUTUAL_SCAN_DEV 		(0x06)
#define TRAN_TYPE_KEY_DATA  			(0x07)
#define TRAN_TYPE_MTK_TYPE  			(0x0A)
#define TRAN_TYPE_FOCAL_TYPE  			(0x0B)
#define TRAN_TYPE_INFORMATION_TYPE		(0x0C)
#define TRAN_TYPE_TRACE_X_TYPE		        (0x0D)
#define TRAN_TYPE_TRACE_Y_TYPE		        (0x0E)
#define TRAN_TYPE_FPC_OPEN			(0x0F)
#define TRAN_TYPE_FPC_SHORT			(0x10)

#define TRAN_TYPE_AP_USED_DATA		(0x11)
#define TRAN_TYPE_MIX_FINGER_AP_USED_DATA   	(0x12)


///=============================================================================================///
///  TP Trace
///=============================================================================================///
#define TP_DEFAULT_ROW 				(10)
#define TP_DEFAULT_COL 				(24)

///=============================================================================================///
///  Fram rate definition
///=============================================================================================///
#define FRAM_RATE_TIMER				(1000)
#define INT_FREE_TIMER				(5)

///=========================================================================================///
/// TP related parameters 
///=========================================================================================///
/// resolutions setting

/// resolutions setting
static u16 resolution_x;
static u16 resolution_y;

/// Finger and key
static int finger_packet_size 	= 0;
static u16 finger_num 		= 5;
static u16 key_num 		= 0;

static u8 xy_exchange 		= 0;
static int finger_up_cnt 	= 0;

static u8 pcode[8];  			///< project code[] from b2
static u8 sfr_data[16]		= {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
				   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static u8  ic_model 		= MODEL_ZET6221;
#define DRIVER_VERSION "$Revision: 29 $"

int info_recv = 1; //inital as 1
int i_copy_to_report_data = 0;
///=========================================================================================///
///  the light load report mode
///=========================================================================================///
#ifdef FEATURE_LIGHT_LOAD_REPORT_MODE
#define PRE_PRESSED_DEFAULT_VALUE            (-1)
struct light_load_report_mode
{
	u32 pre_x;
	u32 pre_y;
	u32 pre_z;
	int pressed;
};
static struct light_load_report_mode pre_event[MAX_FINGER_NUMBER];
#endif ///< for  FEATURE_LIGHT_LOAD_REPORT_MODE
///=============================================================================================///
/// Macro Definition
///=============================================================================================///
struct finger_coordinate_struct
{
	u32 report_x;
	u32 report_y;
	u32 report_z;
	u32 last_report_x;
	u32 last_report_y;
	u32 last_report_z;
	u32 coordinate_x;
	u32 coordinate_y;
	u32 last_coordinate_x;
	u32 last_coordinate_y;
	u32 predicted_coordinate_x;
	u32 predicted_coordinate_y;
	u32 last_distance;
	u8 valid;
};
static struct finger_coordinate_struct finger_report[MAX_FINGER_NUMBER];
static u8 finger_report_key;

#ifdef FEATURE_FRAM_RATE
static u32 fram_rate =0;
static u32 last_fram_rate = 0;
#endif ///< for FEATURE_FRAM_RATE

struct i2c_dev
{
	struct list_head list;	
	struct i2c_adapter *adap;
	struct device *dev;
};

static struct class *i2c_dev_class;
static LIST_HEAD (i2c_dev_list);
static DEFINE_SPINLOCK(i2c_dev_list_lock);

//static union
//{
//	unsigned short		dirty_addr_buf[2];
//	const unsigned short	normal_i2c[2];
//}u_i2c_addr = {{0x00},};

///----------------------------------------------------///
/// FW variables
///----------------------------------------------------///
static u16 pcode_addr[8]	= {0x3DF1,0x3DF4,0x3DF7,0x3DFA,0x3EF6,0x3EF9,0x3EFC,0x3EFF}; ///< default pcode addr: zet6221
static u16 pcode_addr_6221[8]	= {0x3DF1,0x3DF4,0x3DF7,0x3DFA,0x3EF6,0x3EF9,0x3EFC,0x3EFF}; ///< zet6221 pcode_addr[8]
static u16 pcode_addr_6223[8]	= {0x7BFC,0x7BFD,0x7BFE,0x7BFF,0x7C04,0x7C05,0x7C06,0x7C07}; ///< zet6223 pcode_addr[8]
static u16 pcode_addr_6270[8] = {0x9BFC,0x9BFD,0x9BFE,0x9BFF,0x9C04,0x9C05,0x9C06,0x9C07}; ///< zet6275 pcode_addr[8]   add by jack.chen 201141103
static u16 pcode_addr_7130[8] = {0xDFFC,0xDFFD,0xDFFE,0xDFFF,0xE004,0xE005,0xE006,0xE007}; ///< zet7130 pcode_addr[8]
//static int dummy_report_cnt	= 0;
static int charger_status	= 0;	///< 0 : discharge,  1 : charge
#ifdef FEATURE_CHARGER_MODE
static u16 polling_time = S_POLLING_TIME;
#endif ///< for FEATURE_CHARGER_MODE
static u8 hover_status		= 0;
static u8 download_ok 		= FALSE;

#ifdef TPD_PROXIMITY
static u8 tpd_proximity_flag = 0;
static u8 tpd_proximity_detect = 1;//0-->close ; 1--> far away
static u8 tpd_in_suspend=0;
#endif


///-------------------------------------///
/// key variables
///-------------------------------------///
static u8 key_menu_pressed	= 0x00;	///< key#0
static u8 key_back_pressed	= 0x00;	///< key#1
static u8 key_home_pressed	= 0x00;	///< key#2
//static u8 key_search_pressed	= 0x00;	///< key#3

static u8 zet_tx_data[257] __initdata;
static u8 zet_rx_data[257] __initdata;
struct zet622x_tsdrv *zet62xx_ts;
static unsigned char rom_type		= ROM_TYPE_FLASH; ///< Flash:0xf SRAM:0x2 OTP:0x6
///---------------------------------------------------------------------------------///
/// trace setting GPIO
///---------------------------------------------------------------------------------///
#ifdef FEATRUE_TRACE_GPIO_OUTPUT
static u8 trace_output_status = 0;
#endif ///< for FEATRUE_TRACE_GPIO_OUTPUT
#ifdef FEATRUE_TRACE_GPIO_INPUT
static u8 trace_input_status = 0;
#endif ///< for FEATRUE_TRACE_GPIO_INPUT
#ifdef FEATRUE_TRACE_SENSOR_ID
static u8 sensor_id_status = 0xFF;
static u8 sensor_id = 0x0;
#endif ///< for FEATRUE_TRACE_SENSOR_ID


///=========================================================================================///
/// 19. suspend no read any finger packet
///=========================================================================================///
static u8 suspend_mode 		= FALSE;
//static u8 reseting 		= FALSE;
///=========================================================================================///
/// 20. resume wait download finish then send charger mode
///=========================================================================================///
static u8 resume_download	= FALSE;
static int zet622x_resume_downloader(struct i2c_client *client, u8 upgrade, u8 *romtype, u8 icmodel);

///=========================================================================================///
/// key variables
///=========================================================================================///
#ifdef TPD_HAVE_BUTTON//FEATURE_MTK_VIRTUAL_KEY 
#define TPD_KEY_GAP 		(40)
#define TPD_KEY_Y_CENTER 	(1500)
#define TPD_KEY_Y_START 	(TPD_KEY_Y_CENTER-TPD_KEY_GAP/2)

static int tpd_keys_local[] 	= { KEY_MENU, KEY_HOMEPAGE, KEY_BACK};

#define TPD_KEY_COUNT sizeof(tpd_keys_local)/sizeof(tpd_keys_local[0])

static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = 
{
///	{X_LEFT_BOUNDARY,X_RIGHT_BOUNDARY,Y_TOP_BOUNDARY,Y_BOTTOM_BOUNDARY}
//	{33, 122, 897, 1019},
	{120,  TPD_KEY_Y_CENTER, TPD_KEY_GAP, TPD_KEY_GAP},
	{240, TPD_KEY_Y_CENTER, TPD_KEY_GAP, TPD_KEY_GAP},
	{360, TPD_KEY_Y_CENTER, TPD_KEY_GAP, TPD_KEY_GAP},	
};
#endif ///< for FEATURE_MTK_VIRTUAL_KEY

#ifdef FEATURE_KEY_TO_FINGER
static unsigned int key_1[]={120, TPD_KEY_Y_CENTER};          
static unsigned int key_2[]={240, TPD_KEY_Y_CENTER};          
static unsigned int key_3[]={360, TPD_KEY_Y_CENTER};     
static unsigned int key_4[]={0, TPD_KEY_Y_CENTER}; 
#endif ///< for FEATURE_KEY_TO_FINGER 

///=========================================================================================///
///  Hardware check only and do no FW upgrade
///=========================================================================================///
static u8 firmware_upgrade	= TRUE; 
#ifdef FEATURE_HW_CHECK_ONLY
  #ifndef FEATURE_FW_UPGRADE
#define FEATURE_FW_UPGRADE
  #endif ///< for FEATURE_FW_UPGRADE
firmware_upgrade 		= FALSE;
#endif ///< for HW_CHECK_ONLY

///-------------------------------------///
///  firmware save / load
///-------------------------------------///
u32 data_offset 			= 0;
u8 *flash_buffer 			= NULL;
u8 *flash_buffer_01 			= NULL;
u8 *flash_buffer_02 			= NULL;
u8 *flash_buffer_03 			= NULL;
struct inode *inode 			= NULL;
mm_segment_t old_fs;

char driver_version[128];
char pcode_version[128];

#define FW_FILE_NAME 			"/mnt/sdcard/zet62xx.bin"
u8 chk_have_bin_file  = FALSE;
char fw_file_name[128];
///-------------------------------------///
///  Transmit Type Mode Path parameters 
///-------------------------------------///
///  External SD-Card could be
///      "/mnt/sdcard/"
///      "/mnt/extsd/"
///-------------------------------------///
#define TRAN_MODE_FILE_PATH		"/data/app-lib/"
char tran_type_mode_file_name[128];
u8 *tran_data = NULL;
///jack.chen add by  20140519
u8 *tran_tmp_data[2] = {NULL, NULL};
u8 * tran_report_point = NULL;
int tran_index = 0;
int tran_data_size = 0;
int tran_type_setting = TRAN_TYPE_DYNAMIC;
int tran_data_file_id = 0;
#define TRAN_DATA_TYPE_FILE_NAME	"zettran"
#define TRAN_PACK_DATA_SIZE		(64)
#define TRAN_MAX_FILE_ID	(10)


///-------------------------------------///
///  Mutual Dev Mode  parameters 
///-------------------------------------///
///  External SD-Card could be
///      "/mnt/sdcard/zetmdev"
///      "/mnt/extsd/zetmdev"
///-------------------------------------///
#ifdef FEATURE_MDEV_OUT_ENABLE
	#define MDEV_FILE_NAME		"zetmdev"
	#define MDEV_MAX_FILE_ID	(10)
	#define MDEV_MAX_DATA_SIZE	(2048)
///-------------------------------------///
///  mutual dev variables
///-------------------------------------///
	u8 *mdev_data = NULL;
	int mdev_file_id = 0;
#endif ///< FEATURE_MDEV_OUT_ENABLE

///-------------------------------------///
///  Initial Base Mode  parameters 
///-------------------------------------///
///  External SD-Card could be
///      "/mnt/sdcard/zetibase"
///      "/mnt/extsd/zetibase"
///-------------------------------------///
#ifdef FEATURE_IBASE_OUT_ENABLE
	#define IBASE_FILE_NAME		"zetibase"
	#define IBASE_MAX_FILE_ID	(10)
	#define IBASE_MAX_DATA_SIZE	(512)

///-------------------------------------///
///  initial base variables
///-------------------------------------///
	u8 *ibase_data = NULL;
	int ibase_file_id = 0;
#endif ///< FEATURE_IBASE_OUT_ENABLE

#ifdef FEATURE_FPC_OPEN_ENABLE
	#define FPC_OPEN_FILE_NAME		"zetfpcopen"
	#define FPC_OPEN_MAX_FILE_ID		(10)
	#define FPC_OPEN_MAX_DATA_SIZE	(512)
	#define FPC_OPEN_CMD_LEN		(1)
	#define FPC_OPEN_CMD			(0xA1)
///-------------------------------------///
///  fpc open variables
///-------------------------------------///
	u8 *fpcopen_data = NULL;
	int fpcopen_file_id = 0;
#endif ///< FEATURE_FPC_OPEN_ENABLE

#ifdef FEATURE_FPC_SHORT_ENABLE
	#define FPC_SHORT_FILE_NAME		"zetfpcshort"
	#define FPC_SHORT_MAX_FILE_ID		(10)
	#define FPC_SHORT_MAX_DATA_SIZE	(512)
	#define FPC_SHORT_CMD_LEN		(1)
	#define FPC_SHORT_CMD			(0xA0)
	
///-------------------------------------///
///  fpc short variables
///-------------------------------------///
	u8 *fpcshort_data = NULL;
	int fpcshort_file_id = 0;
#endif ///< FEATURE_FPC_SHORT_ENABLE

///-------------------------------------///
///  Initial Dev Mode  parameters 
///-------------------------------------///
///  External SD-Card could be
///      "/mnt/sdcard/zetidev"
///      "/mnt/extsd/zetidev"
///-------------------------------------///
#ifdef FEATURE_IDEV_OUT_ENABLE
	#define IDEV_FILE_NAME		"zetidev"
	#define IDEV_MAX_FILE_ID	(10)	
	#define IDEV_MAX_DATA_SIZE	(512)

///-------------------------------------///
///  initial dev variables
///-------------------------------------///
	u8 *idev_data = NULL;
	int idev_file_id = 0;
#endif ///< FEATURE_IDEV_OUT_ENABLE

///-------------------------------------///
///  Mutual Base Mode  parameters 
///-------------------------------------///
///  External SD-Card could be
///      "/mnt/sdcard/zetmbase"
///      "/mnt/extsd/zetmbase"
///-------------------------------------///
#ifdef FEATURE_MBASE_OUT_ENABLE
	#define MBASE_FILE_NAME		"zetmbase"
	#define MBASE_MAX_FILE_ID	(10)
	#define MBASE_MAX_DATA_SIZE	(2048)

///-------------------------------------///
///  mutual base variables
///-------------------------------------///
	u8 *mbase_data = NULL;
	int mbase_file_id = 0;
#endif ///< FEATURE_MBASE_OUT_ENABLE

///-------------------------------------///
///  infomation variables
///-------------------------------------///
#ifdef FEATURE_INFO_OUT_EANBLE
	#define INFO_MAX_DATA_SIZE	(64)
	#define INFO_DATA_SIZE		(17)
	#define ZET6221_INFO		(0x00)
	#define ZET6231_INFO		(0x0B)
	#define ZET6223_INFO		(0x04) ///jack.chen add by  20140519
	#define ZET6223_INFO_PLUS	(0x0D) ///jack.chen add by  20140519
	#define ZET6251_INFO		(0x0C)	
	#define ZET6270_INFO		(0x0F)
	#define ZET7130_INFO		(0x11)
	#define UNKNOW_INFO		(0xFF)
	#define INFO_FILE_NAME		"zetinfo"
	u8 *info_data = NULL;
	u8 *trace_x_data = NULL;
	u8 *trace_y_data = NULL;
#endif ///< FEATURE_INFO_OUT_EANBLE
///-------------------------------------///
///  Default transfer type
///-------------------------------------///
u8 transfer_type = TRAN_TYPE_DYNAMIC;

///-------------------------------------///
///  Default TP TRACE
///-------------------------------------///
int row = TP_DEFAULT_ROW;
int col = TP_DEFAULT_COL;

///=========================================================================================///
///  TP related parameters/structures : configured for all tp
///=========================================================================================///
//static struct task_struct *resume_download_task = NULL;
static struct i2c_client *this_client 		= NULL;
struct i2c_client *zet_i2c_client 			= NULL;
struct input_dev *zet_input_dev				= NULL;

struct zet6221_tsdrv
{	struct work_struct work1; ///<  write_cmd
	struct workqueue_struct *ts_workqueue1; ///< write_cmd
	struct timer_list zet622x_ts_timer_task;
#ifdef FEATURE_INT_FREE
	struct work_struct work2; ///<  polling
	struct workqueue_struct *ts_workqueue2; ///< polling
	struct timer_list zet622x_ts_timer_task1;
#endif ///< for FEATURE_INT_FREE
#ifdef FEATURE_FRAM_RATE
	struct timer_list zet622x_ts_timer_task2;
#endif ///< for FEATURE_FRAM_RATE
};

///-------------------------------------///
/// key lock not report finger setting
///-------------------------------------///
#ifdef FEATURE_KEY_LOCK_FINGER
  #define KEY_LOCK_MAX_FRAME (1)  ///< LOCK N times frame
  int key_lock_finger_cnt = 0; 
#endif ///< for FEATURE_KEY_LOCK_FINGER

static int major;             ///< major number assigned to our device driver 
//static int device_open 		= 0;   ///< Is device open?  Used to prevent multiple  
//static char msg[BUF_LEN];     ///< The msg the device will give when asked    
//static char *msg_ptr;
extern struct tpd_device *tpd;

static int zet622x_int_flag 			= 0;
static struct task_struct *thread 		= NULL;

static DECLARE_WAIT_QUEUE_HEAD(waiter);

static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id);

static int zet622x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int zet622x_ts_detect(struct i2c_client *client, struct i2c_board_info *info);
static int zet622x_ts_remove(struct i2c_client *client);
static int zet622x_ts_work(void *unused);

static unsigned int touch_irq = 0;
#define TPD_OK 0

///************************************************************************
///	i2c_driver
///************************************************************************

//static struct i2c_board_info __initdata zet6221_i2c_tpd = { I2C_BOARD_INFO(ZET_TS_ID_NAME, (0xec>>1))};

static const struct i2c_device_id zet622x_i2c_id[] 	= {{ZET_TS_ID_NAME,0},{}};
static const struct of_device_id zet_dt_match[] = {
	{.compatible = "zet6221-ts"},
	{},
};
MODULE_DEVICE_TABLE(of, zet_dt_match);

static struct i2c_driver tpd_i2c_driver =
{                       
	.driver = 
	{
		.of_match_table = of_match_ptr(zet_dt_match),
		.name = ZET_TS_ID_NAME,
	},
    .probe = zet622x_ts_probe,                                   
    .remove = zet622x_ts_remove,	
    .id_table = zet622x_i2c_id,              
    .detect = zet622x_ts_detect, 
}; 

#ifndef FEATURE_MTK_I2C_8BYTES_MODE

#define IIC_RETRY_NUM (5)
static u8 *zet62xx_i2c_dma_va = NULL; ///< dma virtural address
static dma_addr_t *zet62xx_i2c_dma_pa = NULL; ///< dma physical address

//static struct mutex i2c_data_mutex;
static DEFINE_MUTEX(i2c_data_mutex);

	static void msg_dma_alloct(void)
	{
	    if (NULL == zet62xx_i2c_dma_va)
    	{
       		tpd->dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
       		zet62xx_i2c_dma_va = (u8 *)dma_alloc_coherent(&tpd->dev->dev, 4096, (dma_addr_t*)&zet62xx_i2c_dma_pa, GFP_KERNEL);//DMA size 4096 for customer
    	}		
		
	    if(!zet62xx_i2c_dma_va)
		{
	        	TPD_DMESG("[DMA][Error][ZET] Allocate DMA I2C Buffer failed!\n");
	    }
	}
	
	static void msg_dma_release(void){
		if(zet62xx_i2c_dma_va)
		{
	    	dma_free_coherent(NULL, 4096, zet62xx_i2c_dma_va, (dma_addr_t)zet62xx_i2c_dma_pa);
	       	zet62xx_i2c_dma_va = NULL;
	       	zet62xx_i2c_dma_pa = 0;
			TPD_DMESG("[DMA][release][ZET] Allocate DMA I2C Buffer release!\n");
	    }
	}

#endif ///< for not FEATURE_MTK_I2C_8BYTES_MODE

#if 0
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);

extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
#endif

static int  zet_fw_size(void);
static void zet_fw_save(char *file_name);
static void zet_fw_load(char *file_name);
static void zet_fw_init(void);
#ifdef FEATURE_MDEV_OUT_ENABLE
static void zet_mdev_save(char *file_name);
#endif ///< for FEATURE_MDEV_OUT_ENABLE
#ifdef FEATURE_IDEV_OUT_ENABLE
static void zet_idev_save(char *file_name);
#endif ///< for FEATURE_IDEV_OUT_ENABLE
#ifdef FEATURE_IBASE_OUT_ENABLE
static void zet_ibase_save(char *file_name);
#endif ///< for FEATURE_IBASE_OUT_ENABLE

#ifdef FEATURE_FPC_OPEN_ENABLE
static void zet_fpcopen_save(char *file_name);
#endif ///< FEATURE_FPC_OPEN_ENABLE
#ifdef FEATURE_FPC_SHORT_ENABLE
static void zet_fpcshort_save(char *file_name);
#endif ///< FEATURE_FPC_SHORT_ENABLE

#ifdef FEATURE_MBASE_OUT_ENABLE
static void zet_mbase_save(char *file_name);
#endif ///< for FEATURE_MBASE_OUT_ENABLE
#ifdef FEATURE_INFO_OUT_EANBLE
static void zet_information_save(char *file_name);
static void zet_trace_x_save(char *file_name);
static void zet_trace_y_save(char *file_name);
#endif ///< for FEATURE_INFO_OUT_EANBLE

u8 zet622x_ts_check_version(void);
int __init zet622x_downloader( struct i2c_client *client, u8 upgrade, u8 *pRomType, u8 icmodel);
int zet622x_ts_data_flash_download(struct i2c_client *client);

#ifdef FEATRUE_TRACE_SENSOR_ID
u8 zet622x_ts_check_sensor_id_index(void);
#endif ///< FEATRUE_TRACE_SENSOR_ID

static void zet_tran_data_save(char *file_name);

///=============================================================================================///
///  Guesture
///=============================================================================================///
//#define GESTURE_WAKEUP

#ifdef GESTURE_WAKEUP
typedef enum
{
    DOZE_DISABLED = 0,
    DOZE_ENABLED = 1,
    DOZE_WAKEUP = 2,
}DOZE_T;
static DOZE_T doze_status = DOZE_DISABLED;
static s8 zet_enter_doze();
#endif



///***********************************************************************
///   [function]:  ctp_set_reset_low
///   [parameters]:   
///   [return]: void
///************************************************************************
void ctp_set_reset_low(void)
{
	tpd_gpio_output(GTP_RST_PORT, 0);	
}

///***********************************************************************
///   [function]:  ctp_set_reset_high
///   [parameters]:   
///   [return]: void
///************************************************************************
void ctp_set_reset_high(void)
{
	tpd_gpio_output(GTP_RST_PORT, 1);	
}

///***********************************************************************
///   [function]:  ctp_set_power_down
///   [parameters]:   
///   [return]: void
///************************************************************************
void ctp_set_power_down(void)
{
	#if defined(CONFIG_MTK_LEGACY)
	hwPowerDown(TPD_POWER_SOURCE_CUSTOM, "TP");
	#endif
}

///***********************************************************************
///   [function]:  ctp_set_power_on
///   [parameters]:   
///   [return]: void
///************************************************************************
void ctp_set_power_on(void)
{
	////< MT75xx
	#if 1//def TPD_POWER_SOURCE_CUSTOM
	int ret;
	ret = regulator_set_voltage(tpd->reg, 2800000, 2800000);	/* set 2.8v */
	if (ret)
		printk("regulator_set_voltage() failed!\n");
	ret = regulator_enable(tpd->reg);	/* enable regulator */
	if (ret)
		printk("regulator_enable() failed!\n");
	msleep(5);
        #else
	hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
	#endif
	msleep(5);
}

#ifdef FEATURE_KEY_TO_FINGER
///**********************************************************************
///   [function]:  tp_up
///   [parameters]: x, y 
///   [return]: void
///**********************************************************************
static void ctp_tpd_down(int x, int y) 
{
       ////printk("\r\n [ZET] : key ctp_tpd_down x=%d, y=%d\n", x, y);
#ifdef FEATURE_MT_TYPE_B
	input_mt_slot(tpd->dev, finger_num);
	input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER,true);
#endif ///< for FEATURE_MT_TYPE_B						

	input_report_key(tpd->dev, BTN_TOUCH, 1);
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 8);
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
#ifndef FEATURE_MT_TYPE_B
	input_mt_sync(tpd->dev);	
#endif ///< for FEATURE_MT_TYPE_B
	if(FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
	{   
		tpd_button(x, y, 1);  
	}
	if(y > TPD_RES_Y) //virtual key debounce to avoid android ANR issue
	{
		msleep(50);
	 	////printk("D virtual key \n");
	}
}
 
///**********************************************************************
///   [function]:  tp_up
///   [parameters]: x, y
///   [return]: void
///**********************************************************************
static void ctp_tpd_up(int x, int y) 
{
	input_mt_slot(tpd->dev, finger_num);
	input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER,false);
	input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, -1);
        if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
	{   
		tpd_button(x, y, 0); 
	}   		 
}
#endif

///**********************************************************************
///   [function]:  zet622x_i2c_get_free_dev
///   [parameters]: adap
///   [return]: void
///**********************************************************************
static struct i2c_dev *zet622x_i2c_get_free_dev(struct i2c_adapter *adap) 
{
	struct i2c_dev *i2c_dev;

	if (adap->nr >= I2C_MINORS)
	{
		////printk("[ZET] : i2c-dev:out of device minors (%d) \n",adap->nr);
		return ERR_PTR (-ENODEV);
	}

	i2c_dev = kzalloc(sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
	{
		return ERR_PTR(-ENOMEM);
	}
	i2c_dev->adap = adap;

	spin_lock(&i2c_dev_list_lock);
	list_add_tail(&i2c_dev->list, &i2c_dev_list);
	spin_unlock(&i2c_dev_list_lock);
	
	return i2c_dev;
}


///**********************************************************************
///   [function]:  zet622x_i2c_dev_get_by_minor
///   [parameters]: index
///   [return]: i2c_dev
///**********************************************************************
static struct i2c_dev *zet622x_i2c_dev_get_by_minor(unsigned index)
{
	struct i2c_dev *i2c_dev;
	spin_lock(&i2c_dev_list_lock);
	
	list_for_each_entry(i2c_dev, &i2c_dev_list, list)
	{
		////printk(" [ZET] : line = %d ,i2c_dev->adapt->nr = %d,index = %d.\n",__LINE__,i2c_dev->adap->nr,index);
		if(i2c_dev->adap->nr == index)
		{
		     goto LABEL_FOUND;
		}
	}
	i2c_dev = NULL;
	
LABEL_FOUND: 
	spin_unlock(&i2c_dev_list_lock);
	
	return i2c_dev ;
}

#ifdef FEATURE_MTK_I2C_8BYTES_MODE
///************************************************************************
///   [function]:  callback: read data by i2c interface;
///   [parameters]: 
///			    client[in]:  struct i2c_client represent an I2C slave device;
///			    data [out]:  data buffer to read;
///			    length[in]:  data length to read;
///   [return]: Returns negative errno, else the number of messages executed;
///************************************************************************
s32 zet622x_i2c_read_tsdata(struct i2c_client *client, u8 *rxbuf, u8 length)
{
	u8 buffer[I2C_DEVICE_ADDRESS_LEN];
	u8 retry;
    	u16 left 	= length;
    	u16 offset 	= 0;
    	u16 addr;

	struct i2c_msg msg[2] =
	{
	{
	    .addr = client->addr,
	    .flags = 0,
	    .buf = buffer,
	    .len = I2C_DEVICE_ADDRESS_LEN,
	    .timing = ZET6221_I2C_FREQ
	},
	{
	    .addr = client->addr,
	    .flags = I2C_M_RD,
	    .timing = ZET6221_I2C_FREQ
	},
	};

	if ( rxbuf == NULL )
	{
		return FALSE;
	}
	
	addr = client->addr;
	////printk("[ZET] : i2c_read_bytes to device 0x%02X address 0x%04X len %d\n", client->addr, addr, len );

	while ( left > 0 )
	{
		buffer[0] = ( ( addr+offset ) >> 8 ) & 0xFF;
		buffer[1] = ( addr+offset ) & 0xFF;

		msg[1].buf = &rxbuf[offset];

		if ( left > MAX_TRANSACTION_LENGTH )
		{
		    msg[1].len = MAX_TRANSACTION_LENGTH;
		    left -= MAX_TRANSACTION_LENGTH;
		    offset += MAX_TRANSACTION_LENGTH;
		}
		else
		{
		    msg[1].len = left;
		    left = 0;
		}

		retry = 0;

		while ( i2c_transfer( client->adapter, &msg[1], 1 ) != 1 )
		{
		    retry++;

		    if ( retry == 20 )
		    {
		        ////printk("[ZET] : I2C read 0x%X length=%d failed\n", addr + offset, len);
			return FALSE;
		    }
		}
	}

	return TRUE;
}

///************************************************************************
///   [function]:  callback: write data by i2c interface;
///   [parameters]: client[in]:  struct i2c_client represent an I2C slave device;
///			    data [out]:  data buffer to write;
///			    length[in]:  data length to write;
///   [return]: Returns negative errno, else the number of messages executed;
///************************************************************************
s32 zet622x_i2c_write_tsdata(struct i2c_client *client, u8 *txbuf, u8 length)
{

	u8 buffer[MAX_TRANSACTION_LENGTH];
	u16 left 	= length;
	u16 offset 	= 0;
	u8 retry 	= 0;

	struct i2c_msg msg = 
	{
	.addr 	= client->addr,
	.flags 	= 0,
	.buf 	= buffer
	};


	if ( txbuf == NULL )
	{
		return FALSE;
	}

	////printk("[ZET] : i2c_write_bytes to device %02X address %04X len %d\n", client->addr, addr, len );

	while ( left > 0 )
	{
		retry = 0;

		if ( left > MAX_I2C_TRANSFER_SIZE )
		{
		    //memcpy( &buffer[I2C_DEVICE_ADDRESS_LEN], &txbuf[offset], MAX_I2C_TRANSFER_SIZE );
		    memcpy( &buffer, &txbuf[offset], MAX_I2C_TRANSFER_SIZE );
		    msg.len = MAX_TRANSACTION_LENGTH;
		    left -= MAX_I2C_TRANSFER_SIZE;
		    offset += MAX_I2C_TRANSFER_SIZE;
		}
		else
		{
		    //memcpy( &buffer[I2C_DEVICE_ADDRESS_LEN], &txbuf[offset], left );
		    memcpy( &buffer, &txbuf[offset], left );
		    msg.len = left;
		    left = 0;
		}

		////printk("[ZET] : byte left %d offset %d\n", left, offset );

		while ( i2c_transfer( client->adapter, &msg, 1 ) != 1 )
		{
		    retry++;

		    if ( retry == 20 )
		    {
		        ////printk("[ZET] : I2C write 0x%X%X length=%d failed\n", buffer[0], buffer[1], len);
		        return FALSE;
		    }
		}
	}

	return TRUE;
}
#else ///< for FEATURE_MTK_I2C_8BYTES_MODE

///************************************************************************
///   [function]:  callback: read data by i2c interface;
///   [parameters]: 
///			    client[in]:  struct i2c_client represent an I2C slave device;
///			    data [out]:  data buffer to read;
///			    length[in]:  data length to read;
///   [return]: Returns negative errno, else the number of messages executed;
///************************************************************************
s32 zet622x_i2c_read_tsdata(struct i2c_client *client, u8 *data, u8 length)
{
#if 1
	int ret;
	// for DMA I2c transfer
	
	#ifdef ZET_DEBUG
	printk("[ZET] zet622x_i2c_read_tsdata len=%d\n",length);
	#endif
	
    mutex_lock(&i2c_data_mutex);
	
	this_client->addr = (this_client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;
	ret = i2c_master_recv(this_client, (u8 *)zet62xx_i2c_dma_pa, length);
	memcpy(data, zet62xx_i2c_dma_va, length);
	this_client->addr = (this_client->addr & I2C_MASK_FLAG) &(~ I2C_DMA_FLAG);	
	
    mutex_unlock(&i2c_data_mutex);
	
	return ret;
#else

    int ret;
    int retry = 0;
 
    struct i2c_msg msg=
    {
        .addr = (client->addr & I2C_MASK_FLAG),
        .ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
        .flags = I2C_M_RD,
        .buf = zet62xx_i2c_dma_pa,
        .len = length,
        .timing = 400
    };
	
	mutex_lock(&i2c_data_mutex);    
    //GTP_DEBUG("dma i2c read: 0x%04X, %d bytes(s)", addr, len);
    for (retry = 0; retry < 5; ++retry)
    {
        ret = i2c_transfer(client->adapter, &msg, 1);
        if (ret < 0)
        {
            continue;
        }
		memcpy(data, zet62xx_i2c_dma_va, length);
		mutex_unlock(&i2c_data_mutex); 
        return 0;
    }
    printk("[ZET] Dma I2C Read Error: %d byte(s), err-code: %d", length, ret);
	mutex_unlock(&i2c_data_mutex); 
    return ret;	

#endif

}

///************************************************************************
///   [function]:  callback: write data by i2c interface;
///   [parameters]: client[in]:  struct i2c_client represent an I2C slave device;
///			    data [out]:  data buffer to write;
///			    length[in]:  data length to write;
///   [return]: Returns negative errno, else the number of messages executed;
///************************************************************************
s32 zet622x_i2c_write_tsdata(struct i2c_client *client, u8 *data, u8 length)
{
	int ret;

	#ifdef ZET_DEBUG
	printk("[ZET] zet622x_i2c_write_tsdata len=%d\n",length);
	#endif	
		
    mutex_lock(&i2c_data_mutex);
  
	memcpy(zet62xx_i2c_dma_va, data, length);
	
	this_client->addr = (this_client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;
	if((ret=i2c_master_send(this_client, (u8 *)zet62xx_i2c_dma_pa, length))!=length)
		printk("[ZET] i2c write failed\n");
    this_client->addr = (this_client->addr & I2C_MASK_FLAG) &(~ I2C_DMA_FLAG);

	mutex_unlock(&i2c_data_mutex);
        
	return ret;
}
#endif ///< for FEATURE_MTK_I2C_8BYTES_MODE

#ifdef GESTURE_WAKEUP
static s8 zet_enter_doze()
{
	u8 data = 0xb7;
	int ret;
	ret = zet622x_i2c_write_tsdata(this_client, &data, 1);
	doze_status = DOZE_ENABLED;
	printk("-lantao- DOZE_ENABLED. ***************************\n ");
    return ret;
}

#endif

///**********************************************************************
///   [function]:  zet622x_cmd_sndpwd
///   [parameters]: client
///   [return]: u8
///**********************************************************************
u8 zet622x_cmd_sndpwd(struct i2c_client *client)
{
	u8 ts_cmd[3] = {CMD_WRITE_PASSWORD, CMD_PASSWORD_HIBYTE, CMD_PASSWORD_LOBYTE};	
	int ret;	
	ret = zet622x_i2c_write_tsdata(client, ts_cmd, 3);
	printk("[ZET]: zet622x_cmd_sndpwd , ret=%d\n",ret);
	return ret;
}
///**********************************************************************
///   [function]:  zet622x_cmd_sndpwd_1k (ZET6223 only)
///   [parameters]: client
///   [return]: u8
///**********************************************************************
u8 zet622x_cmd_sndpwd_1k(struct i2c_client *client)
{
	u8 ts_cmd[3] = {CMD_WRITE_PASSWORD, CMD_PASSWORD_1K_HIBYTE, CMD_PASSWORD_1K_LOBYTE};	
	int ret;	
	ret = zet622x_i2c_write_tsdata(client, ts_cmd, 3);
	return ret;
}

///***********************************************************************
///   [function]:  zet622x_cmd_masserase
///   [parameters]: client  
///   [return]: u8
///************************************************************************
u8 zet622x_cmd_masserase(struct i2c_client *client)
{
	u8 ts_cmd[1] = {CMD_MASS_ERASE};
	int ret;
	ret = zet622x_i2c_write_tsdata(client, ts_cmd, 1);
	return ret;
}
///***********************************************************************
///   [function]:  zet622x_cmd_pageerase
///   [parameters]: client, npage
///   [return]: u8
///************************************************************************
u8 zet622x_cmd_pageerase(struct i2c_client *client,int npage)
{
	u8  ts_cmd[4] = {CMD_PAGE_ERASE, 0x00, 0x00, 0x00};
	u8 len		= 0;
	int ret = TRUE;
	//int sector_id = 0;

	switch(ic_model)
	{
		case MODEL_ZET6221: ///< 6221
			ts_cmd[1] = npage;
			len = 2;
			ret = zet622x_i2c_write_tsdata(client, ts_cmd, len);
			printk( " [ZET] : page erase\n");
			msleep(30);				
			break;
		case MODEL_ZET7130: ///< 7130
			ts_cmd[0] = CMD_SECTOR_ERASE;
			ts_cmd[1] = (u8)(npage & 0xF0); // sector id = npage / 16
			ts_cmd[2] = (u8)((npage >> 8) & 0xFF); // sector id = npage / 16
			ts_cmd[3] = (u8)((npage >> 16) & 0xFF); // sector id = npage / 16
			len = 4;
			ret = zet622x_i2c_write_tsdata(client, ts_cmd, len);
			printk( " [ZET] : %06x sector erase\n", npage & 0xF0);
			msleep(300);			
			break;
		case MODEL_ZET6223: ///< 6223
		case MODEL_ZET6231: ///< 6231
			ts_cmd[1] = npage & 0xff;
			ts_cmd[2] = npage >> 8;
			len = 3;
			ret = zet622x_i2c_write_tsdata(client, ts_cmd, len);	
			printk( " [ZET] : page erase\n");			
			break;
		case MODEL_ZET7150: ///< 7150	
		case MODEL_ZET6251: ///< 6251		
		case MODEL_ZET6270: ///< for 6270
		default: 
			break;
	}

	return ret;
}

#ifdef FEATURE_MTK_I2C_8BYTES_MODE

///************************************************************************
///   [function]:  zet622x_cmd_codeoption
///   [parameters]: client, romtype
///   [return]: u8
///**********************************************************************
u8 zet622x_cmd_codeoption(struct i2c_client *client, u8 *romtype)
{
       int ret;
       u8 fifo_cmd[1] 	= {CMD_READ_FIFO};
	u8 ts_cmd[2]   	  = {CMD_READ_CODE_OPTION, 0x00};
	u8 ts_in_data[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	u8 addr 	  = 0;
	u16 model;
	int i;

        ///----------------------------------------------------///
        /// 1. Read code option[0~7]
        ///----------------------------------------------------///        
        ts_cmd[1] = addr;
        ret = zet622x_i2c_write_tsdata(client, ts_cmd, 2);
        msleep(1);
        ret = zet622x_i2c_read_tsdata(client, &ts_in_data[addr], 8);
        msleep(1);
        addr += 8;       
        
        ///----------------------------------------------------///
        /// 2. Read code option[8~15]
        ///----------------------------------------------------///        
        ts_cmd[1] = addr;
        ret = zet622x_i2c_write_tsdata(client, ts_cmd, 2);
        msleep(1);
        ret = zet622x_i2c_read_tsdata(client, &ts_in_data[addr], 8);
        msleep(1);
        addr+= 8;
       
        ///----------------------------------------------------///
        /// 3, print code option
        ///----------------------------------------------------///        
	for(i = 0; i < 16; i++)
	{
	        if(i == 0)
	        {
                  //      //printk("[ZET] : Codeoption[] = %02x ",ts_in_data[i]); 
                }
	        else
                {
                   //     //printk("%02x ",ts_in_data[i]); 
                }
	}
    //    //printk("\n"); 

        model = 0x0;
        model = ts_in_data[7];
        model = (model << 8) | ts_in_data[6];
	
			//	//printk("[ZET] : 0x%04X model \n", model); 
        switch(model) { 
        case 0xFFFF: 
                ret = 1;
                ic_model = MODEL_ZET6221;
                for(i = 0; i < 8; i++)
                {
                        pcode_addr[i] = pcode_addr_6221[i];
                }
                break; 
        case 0x6231:
                ret = 1;
                ic_model = MODEL_ZET6231;
                for(i = 0; i < 8; i++)
                {
                        pcode_addr[i] = pcode_addr_6223[i];
                }
		break;
        case 0x6223:
                ret = 1;
                ic_model = MODEL_ZET6223;
                for(i = 0; i < 8; i++)
                {
                        pcode_addr[i] = pcode_addr_6223[i];
                }
		break; 
	case 0x6251:
		ic_model = MODEL_ZET6251;
		for(i=0;i<8;i++)
		{
			pcode_addr[i] = pcode_addr_6223[i];
		}
		break;  
	case 0x6270:
		ic_model = MODEL_ZET6270;
		for(i = 0 ; i < 8 ; i++)
		{
			pcode_addr[i] = pcode_addr_6270[i];
		}
		break;
        default: 
                ret = 1;
                ic_model = MODEL_ZET6223;
                for(i = 0; i < 8; i++)
                {
                        pcode_addr[i] = pcode_addr_6223[i];
                }
                break;         
        } 

        ///----------------------------------------------------///
        /// 1. Send FIFO Command
        ///----------------------------------------------------///        
        ret = zet622x_i2c_write_tsdata(client, fifo_cmd, 1);
        msleep(1);
        
        return ret;
}

///**********************************************************************
///   [function]:  zet622x_cmd_sfr_read
///   [parameters]: client
///   [return]: u8
///**********************************************************************
u8 zet622x_cmd_sfr_read(struct i2c_client *client)
{
	int ret;
	
        u8 fifo_cmd[1] 	= {CMD_READ_FIFO};
	u8 ts_cmd[2]   	= {CMD_READ_SFR, 0x00};
	int i;
	u8 addr 	= 0;

        ///----------------------------------------------------///
        /// 1. Send FIFO Command
        ///----------------------------------------------------///        
        ret = zet622x_i2c_write_tsdata(client, fifo_cmd, 1);
        msleep(1);

        ///----------------------------------------------------///
        /// 2. Read SFR[0~7]
        ///----------------------------------------------------///        
        ts_cmd[1] = addr;
        ret = zet622x_i2c_write_tsdata(client, ts_cmd, 2);
        msleep(1);
        ret = zet622x_i2c_read_tsdata(client, &sfr_data[addr], 8);
        msleep(1);
        addr += 8;       
        
        ///----------------------------------------------------///
        /// 3. Read SFRn[8~15]
        ///----------------------------------------------------///        
        ts_cmd[1] = addr;
        ret = zet622x_i2c_write_tsdata(client, ts_cmd, 2);
        msleep(1);
        ret = zet622x_i2c_read_tsdata(client, &sfr_data[addr], 8);
        msleep(1);
        addr+= 8;	

        ///----------------------------------------------------///
        /// 4. print SFR[]
        ///----------------------------------------------------/// 
	for(i = 0; i < 16; i++)
	{
	        if(i == 0)
                {
        		////printk("[ZET] : SFR[] = %02x ", sfr_data[i]); 
                }
	        else
                {
               		////printk("%02x ", sfr_data[i]); 
                }
	}
	////printk("\n"); 

        ///----------------------------------------------------///
        /// 5. check if it is a ZET IC
        ///----------------------------------------------------///        
	if((sfr_data[14] != 0x3D) && 
	   (sfr_data[14] != 0x7D))
	{
		////printk("[ZET] : The SFR[14] shall be 0x3D or 0x7D\n"); 
		return FALSE;
	}
	
	return TRUE;
}

///**********************************************************************
///   [function]:  zet622x_cmd_sfr_unlock
///   [parameters]: client
///   [return]: u8
///**********************************************************************
u8 zet622x_cmd_sfr_unlock(struct i2c_client *client)
{
        int ret 	= TRUE;
        u8 fifo_cmd[8] 	= {CMD_FILL_FIFO, 0x00, 0x00 , 0x00, 0x00, 0x00, 0x00, 0x00};        
	u8 ts_cmd[1]   	= {CMD_WRITE_SFR};
	int i 		= 0;
	int idx 	= 0;

        ///----------------------------------------------------///
        /// 1. Check if unlock
        ///----------------------------------------------------///    
	if(sfr_data[14] == SFR_UNLOCK_FLASH)
	{
	        return ret;
        }
	sfr_data[14] = SFR_UNLOCK_FLASH;	
        ///----------------------------------------------------///
        /// 2. Fill SFR[0~6] to FIFO 
        ///----------------------------------------------------///    
	for(i = 0; i < 7; i++)
	{
                fifo_cmd[i+1] = sfr_data[idx];
                idx++;
	}	
        ret = zet622x_i2c_write_tsdata(client, fifo_cmd, 8);

        ///----------------------------------------------------///
        /// 3. Fill SFR[7~13] to FIFO
        ///----------------------------------------------------///    
        for(i = 0; i < 7; i++)
	{
                fifo_cmd[i+1] = sfr_data[idx];
                idx++;
	}	
        ret = zet622x_i2c_write_tsdata(client, fifo_cmd, 8);

        ///----------------------------------------------------///
        /// 4. Fill SFR[14~15] to FIFO
        ///----------------------------------------------------///    
        for(i = 0; i < 2; i++)
	{
                fifo_cmd[i+1] = sfr_data[idx];
                idx++;
	}	
        ret = zet622x_i2c_write_tsdata(client, fifo_cmd, 3);


        ///----------------------------------------------------///
        /// 5. Send SFR Write Command
        ///----------------------------------------------------///    	
        ret = zet622x_i2c_write_tsdata(client, ts_cmd, 1);

	
	return TRUE;
}

///***********************************************************************
///   [function]:  zet622x_cmd_writepage
///   [parameters]: client, page_id, buf
///   [return]: int
///************************************************************************
int zet622x_cmd_writepage(struct i2c_client *client, int page_id, u8 * buf)
{
	int ret;
	int cmd_len 	= 131;
	int cmd_idx 	= 3;

	int reset_len 	= FLASH_PAGE_LEN;
	int idx 	= 0;
	
	u8 tx_buf[256];
	u8 fifo_cmd[8] 	= {CMD_FILL_FIFO, 0x00, 0x00 , 0x00, 0x00, 0x00, 0x00, 0x00}; 
	int i;

	switch(ic_model)
	{
		case MODEL_ZET6221: ///< for 6221
			cmd_len = 130;
			
			tx_buf[0] = CMD_WRITE_PROGRAM;
			tx_buf[1] = page_id;			
			cmd_idx = 2;
			for(i = 0 ; i < FLASH_PAGE_LEN; i++)
                	{
                		tx_buf[i + cmd_idx] = buf[i];
                	}
                	ret = zet622x_i2c_write_tsdata(client, tx_buf, cmd_len);
                	if(ret <= 0)
                	{
                		printk("[ZET] : write page %d failed!!", page_id);
                	}
			return TRUE;
		case MODEL_ZET6223: ///< for 6223
                        cmd_len = 131;			
			tx_buf[0] = CMD_WRITE_PROGRAM;
			tx_buf[1] = page_id & 0xff;
			tx_buf[2] = page_id >> 8;
			
			cmd_idx = 3;
			for(i = 0; i < FLASH_PAGE_LEN; i++)
                	{
                		tx_buf[i + cmd_idx] = buf[i];
                	}
			
                	ret = zet622x_i2c_write_tsdata(client, tx_buf, cmd_len);
                	if(ret <= 0)
                	{
                		printk("[ZET] : write page %d failed!!", page_id);
                	}
                	
			return TRUE;
		case MODEL_ZET6231: ///< for 6223		
		case MODEL_ZET6251: ///< for 6223
		case MODEL_ZET6270;
		default: 
			break;
	}

        	
        ///----------------------------------------------------///
        /// 1. Fill FIFO
        ///----------------------------------------------------///    
        while(reset_len > 0)
        {
                cmd_len = 8;
                if(reset_len > (cmd_len - 1))
                {
                	for(i = 0; i < 7; i++)
                	{
                                fifo_cmd[i+1] = buf[idx];
                                idx++;
                	}	                	
                	ret = zet622x_i2c_write_tsdata(client, fifo_cmd, cmd_len);
                	reset_len -= (cmd_len - 1);
                }
                else
                {
                        for(i = 0; i < reset_len; i++)
                	{
                                fifo_cmd[i+1] = buf[idx];
                                idx++;
                	}	
                        ret = zet622x_i2c_write_tsdata(client, fifo_cmd, (reset_len + 1));
                        reset_len = 0;
                }
        }

        ///----------------------------------------------------///
        /// 2. Send Command
        ///----------------------------------------------------///    
	cmd_len = 3;	
	tx_buf[0] = CMD_WRITE_PROGRAM;
	tx_buf[1] = page_id & 0xff;
	tx_buf[2] = page_id >> 8;
	ret = zet622x_i2c_write_tsdata(client, tx_buf, cmd_len);
	
	return ret;
}

///***********************************************************************
///   [function]:  zet622x_cmd_readpage
///   [parameters]: client, page_id, buf
///   [return]: int
///************************************************************************
int zet622x_cmd_readpage(struct i2c_client *client, int page_id, u8 * buf)
{
	int ret;
	u8 ts_cmd[4];
	int cmd_len;
	int i;

	////printk("[ZET]: MTK read page");

  	for(i = 0 ; i < (FLASH_PAGE_LEN/8); i++)
  	{
    		///--------------------------------------------------------///
    		/// Write FIFO Read Command
    		///--------------------------------------------------------///
   		cmd_len = 4;
  		ts_cmd[0] = CMD_PAGE_READ_PROGRAM;
  		ts_cmd[1] = (u8)(page_id) & 0xff; 
  		ts_cmd[2] = (u8)(page_id) >> 8;   		
  		ts_cmd[3] = i*8;   	        	
  		ret = zet622x_i2c_write_tsdata(client, ts_cmd, cmd_len);
  		///--------------------------------------------------------///
    		/// Read FIFO Read Command
    		///--------------------------------------------------------///
    		ret = zet622x_i2c_read_tsdata(client, &buf[i*8], 8);
  	}
	return TRUE;
}

#else ///< for FEATURE_MTK_I2C_8BYTES_MODE

///**********************************************************************
///   [function]:  zet622x_cmd_codeoption
///   [parameters]: client, romtype
///   [return]: u8
///**********************************************************************
u8 zet622x_cmd_codeoption(struct i2c_client *client, u8 *romtype)
{
	u8 ts_cmd[1] 			= {CMD_READ_CODE_OPTION};
	u8 code_option[32] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
#ifdef FEATURE_HIGH_IMPEDENCE_MODE
	u8 ts_code_option_erase[1] = {CMD_ERASE_CODE_OPTION};
	u8 tx_buf[18] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
#endif ///< for FEATURE_HIGH_IMPEDENCE_MODE
	int ret;
	u16 model;
	int i;
	printk("[ZET] : option write : %02x\n",ts_cmd[0]);
	ret = zet622x_i2c_write_tsdata(client, ts_cmd, 1);
	if (ret <= 0)
	{
		printk("[ZET]: option write fail, ret=%d\n",ret);
		return ret;
	}
	msleep(1);
	printk("[ZET] : read code_option: ");
	ret = zet622x_i2c_read_tsdata(client, code_option, 16);
	if (ret <= 0)
	{
		printk("[ZET]: read fail, ret=%d\n",ret);
		return ret;
	}
	else
	{
	for(i = 0; i < 16; i++)
	{
			printk("%02x ",code_option[i]);
		}
	}
	msleep(1);
	printk("\n");
	model = 0x0;
	model = code_option[7];
	model = (model << 8) | code_option[6];
	/// Set the rom type
	*romtype = (code_option[2] & 0xf0)>>4;
	switch(model)
	{
        case 0xFFFF: 
        	ret = 1;
            	ic_model = MODEL_ZET6221;
			*romtype = ROM_TYPE_FLASH;
			for(i = 0; i < 8; i++)
			{
				pcode_addr[i]=pcode_addr_6221[i];
			}		
#ifdef FEATURE_HIGH_IMPEDENCE_MODE
			if(code_option[2] != IMPEDENCE_BYTE)
			{
				///------------------------------------------///
				/// unlock the flash
				///------------------------------------------///
				if(zet622x_cmd_sfr_read(client) == 0)
				{
					return 0;
				}
				if(zet622x_cmd_sfr_unlock(client) == 0)
				{
					return 0;
				}
				///------------------------------------------///
				/// Erase Code Option
				///------------------------------------------///
				ret = zet622x_i2c_write_tsdata(client, ts_code_option_erase, 1);
				msleep(50);	
				///------------------------------------------///
				/// Write Code Option
				///------------------------------------------///
				tx_buf[0] = CMD_WRITE_CODE_OPTION;
				tx_buf[1] = 0xc5;
				for(i = 2; i < 18; i++)
				{
					tx_buf[i]=code_option[i-2];
				}
				tx_buf[4] = IMPEDENCE_BYTE;
				ret = zet622x_i2c_write_tsdata(client, tx_buf, 18);
				msleep(50);
				///------------------------------------------///
				/// Read Code Option back check
				///------------------------------------------///
				ret = zet622x_i2c_write_tsdata(client, ts_cmd, 1);
				msleep(5);
				printk("%02x ",ts_cmd[0]);
				printk("[ZET] : (2)read : ");
				ret = zet622x_i2c_read_tsdata(client, code_option, 16);
				msleep(1);
				for(i = 0; i < 16; i++)
				{
					printk("%02x ",code_option[i]);
				}
				printk("\n");
			}									
#endif  ///< for FEATURE_HIGH_IMPEDENCE_MODE			
            break; 
	case 0x6231:  
		ret = 1;
		ic_model = MODEL_ZET6231;
			*romtype = ROM_TYPE_FLASH;
		for(i = 0; i < 8; i++)
		{
			pcode_addr[i] = pcode_addr_6223[i];
		}
	    break; 
	case 0x6223:
		ret = 1;
		ic_model = MODEL_ZET6223;
			*romtype = ROM_TYPE_FLASH;
		for(i = 0; i < 8; i++)
		{
			pcode_addr[i] = pcode_addr_6223[i];
		}
	    break; 
	case 0x6251:
		ic_model = MODEL_ZET6251;
		*romtype = ROM_TYPE_SRAM;
		for(i = 0; i < 8; i++)
		{
			pcode_addr[i] = pcode_addr_6223[i];
		}
	    break; 
	case 0x6270:
		ic_model = MODEL_ZET6270;
			*romtype = ROM_TYPE_SRAM;
			for(i = 0 ; i < 8 ; i++)
			{
				pcode_addr[i] = pcode_addr_6270[i];
			}
			break;
		case 0x7130:
			ic_model = MODEL_ZET7130;
			*romtype = ROM_TYPE_FLASH;
			for(i = 0 ; i < 8 ; i++)
			{
				pcode_addr[i] = pcode_addr_6270[i];
			}
			break;		
		case 0x7150:
			ic_model = MODEL_ZET7150;
			*romtype = ROM_TYPE_SRAM;
		for(i = 0 ; i < 8 ; i++)
		{
			pcode_addr[i] = pcode_addr_6270[i];
		}
	break;
	default: 
		ret = 1;
		ic_model = MODEL_ZET6223;
			*romtype = ROM_TYPE_FLASH;
		for(i = 0; i < 8; i++)
		{
			pcode_addr[i] = pcode_addr_6223[i];
		}
	    break;         
	}
	    
	return ret;
	} 

///**********************************************************************
///   [function]:  zet71xx_cmd_enable_64k (ZET71xx only)
///   [parameters]: client
///   [return]: u8
///**********************************************************************
u8 zet71xx_cmd_enable_64k(struct i2c_client *client)
{
	u8 ts_cmd[3] = {CMD_WRITE_PASSWORD, CMD_PASSWORD_1K_HIBYTE, CMD_PASSWORD_1K_LOBYTE};	
	int ret;
	ret = zet622x_i2c_write_tsdata(client, ts_cmd, 3);
	return ret;
}

///**********************************************************************
///   [function]:  zet71xx_cmd_writer (ZET71xx only)
///   [parameters]: client
///   [return]: u8
///**********************************************************************
u8 zet71xx_cmd_writer(struct i2c_client *client)
{
	u8 ts_cmd[1] = {CMD_71xx_WRITER};	
	int ret;
	ret = zet622x_i2c_write_tsdata(client, ts_cmd, 1);
	printk("[ZET]: zet71xx_cmd_writer, ret=%d\n",ret);
	return ret;
}

///**********************************************************************
///   [function]:  zet71xx_cmd_reader (ZET71xx only)
///   [parameters]: client
///   [return]: u8
///**********************************************************************
u8 zet71xx_cmd_reader(struct i2c_client *client)
{
	u8 ts_cmd[1] = {CMD_71xx_READER};	
	int ret;
	ret = zet622x_i2c_write_tsdata(client, ts_cmd, 1);
	printk("[ZET]: zet71xx_cmd_reader, ret=%d\n",ret);
	return ret;
}
///**********************************************************************
///   [function]:  zet622x_cmd_sfr_read
///   [parameters]: client  
///   [return]: u8
///**********************************************************************
u8 zet622x_cmd_sfr_read(struct i2c_client *client)
{
	u8 ts_cmd[1] 	  = {CMD_READ_SFR};
	int ret;
	int i;
	printk("[ZET] : write : %02x\n",ts_cmd[0]);
	ret = zet622x_i2c_write_tsdata(client, ts_cmd, 1);
	msleep(5);
	printk("[ZET] : sfr_read : ");
	ret = zet622x_i2c_read_tsdata(client, sfr_data, 16);
	msleep(1);
	for(i = 0; i < 16; i++)
	{
		printk("%02x ",sfr_data[i]);
	}
	printk("\n");
	if((sfr_data[14] != SFR_UNLOCK_FLASH) &&
	   (sfr_data[14] != SFR_LOCK_FLASH))
	{
		printk("[ZET] : The SFR[14] shall be 0x3D or 0x7D\n");
		////return FALSE;
	}
	////return TRUE;
	return ret;
	}
///**********************************************************************
///   [function]:  zet622x_cmd_sfr_unlock
///   [parameters]: client  
///   [return]: u8
///**********************************************************************
u8 zet622x_cmd_sfr_unlock(struct i2c_client *client)
{
	u8 tx_buf[17] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	int ret = 1;
	int i;
	printk("[ZET] sfr_update : ");
	for(i=0;i<16;i++)
	{
		tx_buf[i+1] = sfr_data[i];
		printk("%02x ",sfr_data[i]);
	}
	printk("\n");
	if(sfr_data[14]!=SFR_UNLOCK_FLASH)
	{
		tx_buf[0]  = CMD_WRITE_SFR;
		tx_buf[15] = SFR_UNLOCK_FLASH;
		ret = zet622x_i2c_write_tsdata(client, tx_buf, 17);
	}
	return ret;
}
///***********************************************************************
///   [function]:  zet622x_cmd_writepage
///   [parameters]: client, page_id, buf
///   [return]: int
///************************************************************************
int zet622x_cmd_writepage(struct i2c_client *client, int page_id, u8 * buf)
{
	int ret;
	int cmd_len = 131;
	int cmd_idx = 3;
	int page_len = FLASH_PAGE_LEN;
	u8 tx_buf[257];
	int i;
	int rest_len;
	int page_count;
	
	if(ic_model == MODEL_ZET7130 || ic_model == MODEL_ZET7150)
	{
		/*cmd_len = 257;
		tx_buf[0] = CMD_FILL_FIFO;
		cmd_idx = 1;
		page_len = FLASH_PAGE_LEN_71xx;
		
		printk("[ZET] : %02x ", tx_buf[0] );
		for(i = 0 ; i < page_len ; i++)
		{
			tx_buf[i + cmd_idx] = buf[i];
			printk("%02x ", buf[i] );
		}
		printk("\n");
		ret = zet622x_i2c_write_tsdata(client, tx_buf, cmd_len);
		if(ret <= 0)
		{
			printk("[ZET] : FIFO page %d failed!!\n", page_id);
		}*/
		cmd_len = 129;
		tx_buf[0] = CMD_FILL_FIFO;
		cmd_idx = 1;
		page_len = FLASH_PAGE_LEN;
		rest_len = FLASH_PAGE_LEN_71xx;
		page_count = 0;
		
		while(rest_len > 0)
		{			
			for(i = 0; i < page_len; i++)
			{
				tx_buf[i + cmd_idx] = buf[page_count];
				page_count++;
			}
			ret = zet622x_i2c_write_tsdata(client, tx_buf, cmd_idx+page_len);
			if(ret <= 0)
			{
				printk("[ZET] : FIFO page %d failed!!\n", page_id);
				return ret;
			}			
			rest_len -= page_len;
			if(rest_len < FLASH_PAGE_LEN)
				page_len = rest_len;
		}
	}

	switch(ic_model)
	{
		case MODEL_ZET6221: ///< for 6221
			cmd_len = 130;			
			tx_buf[0] = CMD_WRITE_PROGRAM;
			tx_buf[1] = page_id;
			cmd_idx = 2;
			page_len = FLASH_PAGE_LEN;
			break;
		case MODEL_ZET7130: ///< for 7130
		case MODEL_ZET7150: ///< for 7150
			cmd_len = 4;
			tx_buf[0] = CMD_WRITE_PROGRAM;
			tx_buf[1] = (page_id & 0xff);
			tx_buf[2] = (u8)( (page_id >> 8) & 0xff);
			tx_buf[3] = (u8)( (page_id >> 16) & 0xff);
			cmd_idx = 0;
			page_len = 0;
			break;
		case MODEL_ZET6223: ///< for 6223
		case MODEL_ZET6231: ///< for 6231
		case MODEL_ZET6251: ///< for 6251
		case MODEL_ZET6270:
		default: 
			cmd_len = 131;			
			tx_buf[0] = CMD_WRITE_PROGRAM;
			tx_buf[1] = (page_id & 0xff);
			tx_buf[2] = (u8)(page_id >> 8);
			cmd_idx = 3;
			page_len = FLASH_PAGE_LEN;
			break;
	}
	for(i = 0 ; i < page_len ; i++)
	{
		tx_buf[i + cmd_idx] = buf[i];
	}
	printk("[ZET] :zet622x_cmd_writepage: cmd_len=%d tx_buf = %02x %02x %02x %02x\n", cmd_len,tx_buf[0],tx_buf[1],tx_buf[2],tx_buf[3]);
	ret = zet622x_i2c_write_tsdata(client, tx_buf, cmd_len);	
	if(ret <= 0)
	{
		printk("[ZET] : write page %d failed!!\n", page_id);
	}
	return ret;
}
///***********************************************************************
///   [function]:  zet622x_cmd_readpage
///   [parameters]: client, page_id, buf
///   [return]: int
///************************************************************************
int zet622x_cmd_readpage(struct i2c_client *client, int page_id, u8 * buf)
{
	int ret;
	int cmd_len = 3;
	int page_len = FLASH_PAGE_LEN;

	switch(ic_model)
	{
		case MODEL_ZET6221:
			buf[0] = CMD_PAGE_READ_PROGRAM;
			buf[1] = (u8)(page_id); ///< (pcode_addr[0]/128);
			cmd_len = 2;
			page_len = FLASH_PAGE_LEN;
			break;
		case MODEL_ZET6223: 
		case MODEL_ZET6231: 
		case MODEL_ZET6251: 
		case MODEL_ZET6270:
			buf[0] = CMD_PAGE_READ_PROGRAM;
			buf[1] = (u8)(page_id) & 0xff; ///< (pcode_addr[0]/128);
			buf[2] = (u8)(page_id >> 8);   ///< (pcode_addr[0]/128);
			cmd_len = 3;
			page_len = FLASH_PAGE_LEN;
			break;
		case MODEL_ZET7130:
		case MODEL_ZET7150:
			buf[0] = CMD_PAGE_READ_PROGRAM;
			buf[1] = (u8)(page_id) & 0xff; 
			buf[2] = (u8)((page_id & 0xff00) >> 8);   
			buf[3] = (u8)(page_id >> 16);   	
			cmd_len = 4;
			page_len = FLASH_PAGE_LEN_71xx;
			break;
		default: 
			buf[0] = CMD_PAGE_READ_PROGRAM;
			buf[1] = (u8)(page_id) & 0xff; ///< (pcode_addr[0]/128);
			buf[2] = (u8)(page_id >> 8);   ///< (pcode_addr[0]/128);
			cmd_len = 3;
			page_len = FLASH_PAGE_LEN;
			break;
	}
	ret = zet622x_i2c_write_tsdata(client, buf, cmd_len);
	if(ret <= 0)
	{
		printk("[ZET]: Read page command fail\n");
		return ret;
	}
	ret = zet622x_i2c_read_tsdata(client, buf, FLASH_PAGE_LEN);
	if(ret <= 0)		
	{
		printk("[ZET]: Read page data fail\n");
		return ret;
	}
	return 1;
}
#endif ///< for FEATURE_MTK_I2C_8BYTES_MODE

///***********************************************************************
///   [function]:  zet622x_cmd_resetmcu
///   [parameters]: client  
///   [return]: u8
///************************************************************************
u8 zet622x_cmd_resetmcu(struct i2c_client *client)
{
	u8 ts_cmd[1] = {CMD_RESET_MCU};
	int ret;
	ret = zet622x_i2c_write_tsdata(client, ts_cmd, 1);
	return ret;
}
///***********************************************************************
///   [function]:  zet622x_cmd_read_check_sum (SRAM : ZET6251 only)
///   [parameters]: client, page_id, buf
///   [return]: int
///************************************************************************
int zet622x_cmd_read_check_sum(struct i2c_client *client, int page_id, u8 * buf)
{
	int ret;
	int cmd_len = 3;

	if(ic_model == MODEL_ZET7130 || ic_model == MODEL_ZET7150)
	{
		cmd_len = 4;
	}
	else
	{
		cmd_len = 3;
	}	
	
	buf[0] = CMD_PROG_CHECK_SUM;
	buf[1] = (u8)(page_id) & 0xff; 
	buf[2] = (u8)(page_id >> 8);   		
	buf[3] = (u8)(page_id >> 16);   	
	ret = zet622x_i2c_write_tsdata(client, buf, cmd_len);
	if(ret <= 0)
	{
		printk("[ZET]: Read check sum fail\n");
		return ret;
	}

	msleep(1);
	
	buf[0] = CMD_PROG_GET_CHECK_SUM;
	cmd_len = 1;
	ret = zet622x_i2c_write_tsdata(client, buf, cmd_len);
	if(ret <= 0)
	{
		printk("[ZET]: Read check sum fail\n");
		return ret;
	}	
	cmd_len = 1;
	ret = zet622x_i2c_read_tsdata(client, buf, cmd_len);
	if(ret <= 0)		
	{
		printk("[ZET]: Read check sum fail\n");
		return ret;
	}
	return 1;
}
///**********************************************************************
///   [function]:  zet622x_ts_gpio_output_on
///   [parameters]: client
///   [return]: void
///**********************************************************************
#ifdef FEATRUE_TRACE_GPIO_INPUT
void zet622x_ts_gpio_input_get(void)
{
	u8 ts_buf[1] = {GPIO_TRACE_INPUT_GET};
	u8 cmd_len = 1;
	int ret;
        /// write the gpio status
        ret = zet622x_i2c_write_tsdata(this_client, ts_buf, 1);
        if(ret <= 0)
        {
        	printk("[ZET] : Set the GPIO_TRACE_OUTPUT_SET command fail\n");
        }
	/// get the gpio status
	ret = zet622x_i2c_read_tsdata(this_client, ts_buf, cmd_len);
	if(ret <= 0)		
	{
		printk("[ZET]: Read check sum fail\n");
		return ;
	}
        trace_input_status = ts_buf[0];		
	printk("[ZET] : trace input status :%d\n", trace_input_status);
}
#endif ///< for FEATRUE_TRACE_GPIO_INPUT
///**********************************************************************
///   [function]:  zet622x_ts_gpio_output_on
///   [parameters]: client
///   [return]: void
///**********************************************************************
#ifdef FEATRUE_TRACE_SENSOR_ID
u8 zet622x_ts_sensor_id_bin_set(u8 status)
{
	int i;
	u8 write_data_flash = FALSE;
	int flash_total_len 	= 0;
        sensor_id_status = status;
        /// if used the bin file not to check bin
        if(chk_have_bin_file == TRUE)
        {
        	sensor_id_status = 0xF0 | sensor_id_status;
        	printk("[ZET] : have bin then bypass Sensor ID status:%d\n", sensor_id_status);
        	return FALSE;
        }
	/// first check the version is match
	if(zet622x_ts_check_sensor_id_index() == FALSE)
	{
		printk("[ZET] : sensor id is same %d status : %d\n", sensor_id, sensor_id_status);
		return FALSE;
	}
	else
	{
		printk("[ZET]: Version different sensor id : %d status : %d\n", sensor_id, sensor_id_status);
		write_data_flash = TRUE;
	}
	flash_total_len = zet_fw_size();
	switch(sensor_id)
	{
	case SENID_01:
		printk("[ZET] : reload senid 01 FW\n");
		for(i = 0 ; i < flash_total_len ; i++)
		{	
			flash_buffer[i] = flash_buffer_01[i];
		}
	break;
	case SENID_02:
		printk("[ZET] : reload senid 02 FW\n");
		for(i = 0 ; i < flash_total_len ; i++)
		{	
			flash_buffer[i] = flash_buffer_02[i];
		}
	break;
	case SENID_03:
		printk("[ZET] : reload senid 03 FW\n");
		for(i = 0 ; i < flash_total_len ; i++)
		{	
			flash_buffer[i] = flash_buffer_03[i];
		}
	break;
	default:
	case SENID_00:
		printk("[ZET] : default fail \n");
		return FALSE;
	break;
	}
	printk("[ZET] : SendID pcode_new : ");
	for(i = 0 ; i < PROJECT_CODE_MAX_CNT ; i++)
	{
		printk("%02x ",flash_buffer[pcode_addr[i]]);
	}
	printk("\n");
        return TRUE;     
}
#endif ///< for FEATRUE_TRACE_SENSOR_ID
///**********************************************************************
///   [function]:  zet622x_ts_gpio_output_on
///   [parameters]: client
///   [return]: void
///**********************************************************************
#ifdef FEATRUE_TRACE_GPIO_OUTPUT
void zet622x_ts_gpio_output_on(u8 index)
{
        u8 ts_write_cmd[2] = {GPIO_TRACE_OUTPUT_SET, 0x00};
        u8 status = trace_output_status;
        int ret = 0;
        /// setting the bit on status
        status |= index;
        /// write the gpio status
        ts_write_cmd[1] = status;
        ret = zet622x_i2c_write_tsdata(this_client, ts_write_cmd, 2);
        if(ret <= 0)
        {
        	printk("[ZET] : Set the GPIO_TRACE_OUTPUT_SET command fail\n");
        }
        trace_output_status = status;
}
#endif ///< for FEATRUE_TRACE_GPIO_OUTPUT
///**********************************************************************
///   [function]:  zet622x_ts_gpio_output_off
///   [parameters]: client
///   [return]: void
///**********************************************************************
#ifdef FEATRUE_TRACE_GPIO_OUTPUT
void zet622x_ts_gpio_output_off(u8 index)
{
        u8 ts_write_cmd[2] = {GPIO_TRACE_OUTPUT_SET, 0x00};
        u8 status = trace_output_status;
        int ret = 0;
        /// setting the bit off status
        status &= ~(index);
        /// write the gpio status
        ts_write_cmd[1] = status;
        ret = zet622x_i2c_write_tsdata(this_client, ts_write_cmd, 2);
        if(ret <= 0)
        {
        	printk("[ZET] : Set the GPIO_TRACE_OUTPUT_SET command fail\n");
        }
        trace_output_status = status;
}
#endif ///< for FEATRUE_TRACE_GPIO_OUTPUT
///**********************************************************************
///   [function]:  zet622x_ts_gpio1_output
///   [parameters]: client
///   [return]: void
///**********************************************************************
#ifdef FEATRUE_TRACE_GPIO_OUTPUT
void zet622x_ts_gpio1_output(int status)
{
	if(status > 0)
  	{
                zet622x_ts_gpio_output_on(TRACE_GPIO1_INDEX);
  	}
        else
        {
                zet622x_ts_gpio_output_off(TRACE_GPIO1_INDEX);
        }
}
EXPORT_SYMBOL_GPL(zet622x_ts_gpio1_output);
#endif ///< for FEATRUE_TRACE_GPIO_OUTPUT
///**********************************************************************
///   [function]:  zet622x_ts_gpio_output
///   [parameters]: client
///   [return]: void
///**********************************************************************
#ifdef FEATRUE_TRACE_GPIO_OUTPUT
void zet622x_ts_gpio_output(int index, int status)
{
  u8 gpio_index = TRACE_GPIO1_INDEX;
        switch(index)
        {
        case 1:
                gpio_index = TRACE_GPIO1_INDEX;
        break;
        case 2:
                gpio_index = TRACE_GPIO2_INDEX;
        break;
        case 3:
                gpio_index = TRACE_GPIO3_INDEX;
        break;
       case 4:
                gpio_index = TRACE_GPIO4_INDEX;
        break;
        case 5:
                gpio_index = TRACE_GPIO5_INDEX;
        break;
        case 6:
                gpio_index = TRACE_GPIO6_INDEX;
        break;
        case 7:
                gpio_index = TRACE_GPIO7_INDEX;
        break;
        case 8:
                gpio_index = TRACE_GPIO8_INDEX;
        break;
        }
  	if(status > 0)
  	{
                zet622x_ts_gpio_output_on(gpio_index);
  	}
        else
        {
                zet622x_ts_gpio_output_off(gpio_index);
        }
}
EXPORT_SYMBOL_GPL(zet622x_ts_gpio_output);
#endif ///< for FEATRUE_TRACE_GPIO_OUTPUT

///************************************************************************
///   [function]:  zet622x_ts_version
///   [parameters]: client  
///   [return]: int
///************************************************************************
u8 zet622x_ts_version( struct i2c_client *client)
{	
	int i;
	int ret;
	
	///=========================================================================///
	/// 1. read the first 4 bytes project code data
	///=========================================================================///		
	memset(zet_rx_data, 0x00, 131);
	ret = zet622x_cmd_readpage(client, (pcode_addr[0]>>7), &zet_rx_data[0]);
	if(ret <= 0)
	{
		return ret;
	}		
	////printk("[ZET] page=%d ",(pcode_addr[0] >> 7));
	for(i = 0; i < 4; i++)
	{
		pcode[i]=zet_rx_data[(pcode_addr[i] & 0x7f)];
		////printk("offset[%d]=%d ",i,(pcode_addr[i] & 0x7f));
	}
	////printk("\n");
	
	
	///=========================================================================///
	/// 2. read the last 4 bytes project code data
	///=========================================================================///
	memset(zet_rx_data, 0x00, 131);
	ret = zet622x_cmd_readpage(client, (pcode_addr[4]>>7), &zet_rx_data[0]);
	if(ret <= 0)
	{
		return ret;
	}	

	////printk("[ZET] page=%d ",(pcode_addr[4] >> 7)); 
	for(i = 4; i < 8; i++)
	{
		pcode[i]=zet_rx_data[(pcode_addr[i] & 0x7f)]; 
		////printk("offset[%d]=%d ",i,(pcode_addr[i] & 0x7f));  
	}
	////printk("\n");
	
		
	///=========================================================================///
	/// 3. compare the version
	///=========================================================================///
	////printk("[ZET] read pcode: ");
	for(i = 0; i < 8; i++)
	{
		////printk("%02x ",pcode[i]);
	}
	////printk("\n");
	
	////printk("[ZET] FW src: ");
	for(i = 0; i < 8; i++)
	{
		////printk("%02x ",flash_buffer[pcode_addr[i]]);
	}
	////printk("\n");
	
	for(i = 0; i < 8; i++)
	{
		if(pcode[i] != flash_buffer[pcode_addr[i]])
		{
			////printk("[ZET]: Version different\n");
			return 0;
		}
	}	
	////printk("[ZET]: Version the same\n");			
	return 1;
}

///***********************************************************************
///   [function]:  zet622x_cmd_ioctl_write_data
///   [parameters]: client, len, buf
///   [return]: int
///************************************************************************
int zet622x_cmd_ioctl_write_data(struct i2c_client *client, u8 len, u8 * buf)
{
	u8 tx_buf[256];
	int i;
	int ret;	
	for(i = 0 ; i < len ; i++)
	{
		tx_buf[i] = buf[i];
	}	
	ret = zet622x_i2c_write_tsdata(client, tx_buf, len);
	if(ret <= 0)
	{
		printk("[ZET] : write cmd failed!!:");
	}
	else
	{
		printk("[ZET] : write cmd :");
	}	
	for(i = 0 ; i < len ; i++)
	{
	        printk("%02x ", tx_buf[i]);
	        if((i%0x10) == 0x0F)
	        {
	                printk("\n");
	        }
	        else if((i%0x08) == 0x07)
	        {
	                printk(" - ");
	        }
	}
	printk("\n");
	return ret;
}

///***********************************************************************
///   [function]:  zet622x_ts_check_version
///   [parameters]: void
///   [return]: void
///************************************************************************
u8 zet622x_ts_check_version(void)
{	
	int i;	
	for(i = 0 ; i < PROJECT_CODE_MAX_CNT ; i++)
	{
		if(pcode[i] != flash_buffer[pcode_addr[i]])
		{
			printk("[ZET]: Version different\n");
			/// if reload the bin file mode 
			return FALSE;
		}
	}
	printk("[ZET]: Version the same\n");
	return TRUE;
}
///***********************************************************************
///   [function]:  zet622x_ts_check_sensor_id_index
///   [parameters]: void
///   [return]: void
///************************************************************************
#ifdef FEATRUE_TRACE_SENSOR_ID
u8 zet622x_ts_check_sensor_id_index(void)
{
        u8 ret = FALSE;
	 sensor_id = flash_buffer[SENSOR_ID_INDEX_ADDR];
        switch(sensor_id_status)
        {
        case SENID_01:
      	 if(sensor_id != flash_buffer_01[SENSOR_ID_INDEX_ADDR])
        {
        	sensor_id = SENID_01;
        	ret = TRUE;
        }
        break;
        case SENID_02:
      	 if(sensor_id != flash_buffer_02[SENSOR_ID_INDEX_ADDR])
        {
        	sensor_id = SENID_02;
        	ret = TRUE;
        }
        break;
        case SENID_03:
      	 if(sensor_id != flash_buffer_03[SENSOR_ID_INDEX_ADDR])
        {
        	sensor_id = SENID_03;
        	ret = TRUE;
        }
        break;
        default:
        case SENID_00:
        	sensor_id = SENID_00;
              ret = FALSE;
         break;
        }
        return ret;
}
#endif ///< FEATRUE_TRACE_SENSOR_ID
///***********************************************************************
///   [function]:  zet622x_ts_check_skip_page
///   [parameters]: u8 point
///   [return]: skip download is TRUE/FALSE
///************************************************************************
#ifdef FEATURE_FW_SKIP_FF
u8 zet622x_ts_check_skip_page(u8 *data)
{	
	int j;
	int len;
	
	if(ic_model == MODEL_ZET7130 || ic_model == MODEL_ZET7150)
	{
		len = FLASH_PAGE_LEN_71xx;
	}
	else
	{
		len = FLASH_PAGE_LEN;
	}
	
	for(j = 0 ; j < len ; j++)
	{
		if(data[j] != 0xFF)
		{
			return FALSE;
		}
	}		
	return TRUE;
}
#endif ///< for FEATURE_FW_SKIP_FF
#ifdef FEATURE_FW_CHECK_SUM
///***********************************************************************
///   [function]:  zet622x_ts_sram_check_sum
///   [parameters]: u8 point
///   [return]: check sum is TRUE/FALSE
///************************************************************************
u8 zet622x_ts_sram_check_sum(struct i2c_client *client, int page_id, u8 *data)
{		
	u8 get_check_sum	= 0;
	u8 check_sum 		= 0;
	int i;
	int ret;
	u8 tmp_data[16];
	int page_len		= FLASH_PAGE_LEN;

	if(ic_model == MODEL_ZET7130 || ic_model == MODEL_ZET7150)
	{
		page_len = FLASH_PAGE_LEN_71xx;
	}
	else
	{
		page_len = FLASH_PAGE_LEN;
	}

	///---------------------------------///
	///  Get check sum
	///---------------------------------///
	for(i = 0 ; i < page_len ; i++)
	{
		if(i == 0)
		{
			check_sum = data[i];
		}
		else
		{
			check_sum = check_sum ^ data[i];
		}
	}	
	///---------------------------------///
	/// Read check sum
	///---------------------------------///
	memset(tmp_data, 0, 16);
	ret = zet622x_cmd_read_check_sum(client, page_id, &tmp_data[0]);	
	if(ret <= 0)
	{
		return FALSE;
	}
	get_check_sum = tmp_data[0];
	if(check_sum == get_check_sum)
	{
		return TRUE;
	}
	else
	{
		printk("[ZET]: page=%3d  ,Check sum : %x ,get check sum : %x\n", page_id, check_sum, get_check_sum);
		return FALSE;
	}
}
#endif ///< for FEATURE_FW_CHECK_SUM
///**********************************************************************
///   [function]:  zet622x_ts_hover_status_get
///   [parameters]: void
///   [return]: u8
///**********************************************************************
u8 zet622x_ts_hover_status_get(void)
{
	return hover_status;
}
EXPORT_SYMBOL_GPL(zet622x_ts_hover_status_get);

#ifdef TPD_PROXIMITY
static s32 tpd_get_ps_value(void)
{
	if(zet622x_ts_hover_status_get()==1)
	{
		tpd_proximity_detect = 0;
	}else
	{
		tpd_proximity_detect = 1;
	}
	//tpd_proximity_detect = !intZetGetHoverStatus();
    return tpd_proximity_detect;
}

static s32 tpd_enable_ps(s32 enable)
{
    u8  state;
    s32 ret = -1;

    if (enable)
    {
        state = 1;
        tpd_proximity_flag = 1;
     //   //printk("TPD proximity function to be on\n");
	}
    else
    {
        state = 0;
        tpd_proximity_flag = 0;
      //  //printk("TPD proximity function to be off\n");
    }

 //   //printk("TPD proximity function %s success.", state ? "enable" : "disable");
    return 0;
}

s32 psensor_ps_operate(void *self, u32 command, void *buff_in, s32 size_in,
                   void *buff_out, s32 size_out, s32 *actualout)
{
    s32 err = 0;
    s32 value;
    hwm_sensor_data *sensor_data;

    switch (command)
    {
        case SENSOR_DELAY:
            if ((buff_in == NULL) || (size_in < sizeof(int)))
            {
                ////printk("Set delay parameter error\n");
                err = -EINVAL;
            }

            // Do nothing
            break;

        case SENSOR_ENABLE:
            if ((buff_in == NULL) || (size_in < sizeof(int)))
            {
                ////printk("Enable sensor parameter error\n");
                err = -EINVAL;
            }
            else
            {
                value = *(int *)buff_in;
                err = tpd_enable_ps(value);
            }

            break;

        case SENSOR_GET_DATA:
            if ((buff_out == NULL) || (size_out < sizeof(hwm_sensor_data)))
            {
                ////printk("Get sensor data parameter error\n");
                err = -EINVAL;
            }
            else
            {
                sensor_data = (hwm_sensor_data *)buff_out;
                sensor_data->values[0] = tpd_get_ps_value();
                sensor_data->value_divide = 1;
                sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
            }

            break;

        default:
            ////printk("proxmy sensor operate function no this parameter %d!\n", command);
            err = -1;
            break;
    }

    return err;
}
EXPORT_SYMBOL_GPL(psensor_ps_operate);
#endif



///**********************************************************************
///   [function]:  zet622x_ts_set_transfer_type
///   [parameters]: void
///   [return]: void
///**********************************************************************
int zet622x_ts_set_transfer_type(u8 bTransType)
{
	u8 ts_cmd[10] = {0xC1, 0x02, TRAN_TYPE_DYNAMIC, 0x55, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00}; 
	int ret = 0;
	ts_cmd[2] = bTransType;
	ret = zet622x_i2c_write_tsdata(this_client, ts_cmd, 10);
        return ret;
}
///**********************************************************************
///   [function]:  zet622x_ts_parse_mutual_dev
///   [parameters]: client
///   [return]: u8
///**********************************************************************
#ifdef FEATURE_MDEV_OUT_ENABLE
u8 zet622x_ts_parse_mutual_dev(struct i2c_client *client)
{
	int mdev_packet_size = (row+2) * (col + 2);
	int ret = 0;
	int idx = 0;
	int len =  mdev_packet_size;
	char mdev_file_name_out[128];
	
	int step_size = col + 2;
	
#ifdef FEATURE_INT_FREE
	ret = zet622x_i2c_read_tsdata(client, &tran_data[idx],  1);
	if(tran_data[idx] != INT_FREE_DATA_HEADER)
	{
		return FALSE;
	}	
#endif ///< for FEATURE_INT_FREE
	while(len > 0)
	{
		if(len < step_size)
		{
			step_size = len;
		}

		ret = zet622x_i2c_read_tsdata(client, &tran_data[idx], step_size);
		len -= step_size;
		idx += step_size;
	}
	sprintf(mdev_file_name_out, "%s%s%02d.bin", tran_type_mode_file_name, MDEV_FILE_NAME, mdev_file_id);	
	zet_mdev_save(mdev_file_name_out);
	mdev_file_id  =  (mdev_file_id +1)% (MDEV_MAX_FILE_ID);
	return ret;
}
#endif ///< FEATURE_MDEV_OUT_ENABLE
///**********************************************************************
///   [function]:  zet622x_ts_parse_initial_base
///   [parameters]: client
///   [return]: u8
///**********************************************************************
#ifdef FEATURE_IBASE_OUT_ENABLE
u8 zet622x_ts_parse_initial_base(struct i2c_client *client)
{
	int ibase_packet_size = (row + col) * 2;
	int ret = 0;
	int idx = 0;
	int len =  ibase_packet_size;
	char ibase_file_name_out[128];
	
	int step_size = ibase_packet_size;
	
#ifdef FEATURE_INT_FREE
	ret = zet622x_i2c_read_tsdata(client, &tran_data[idx], 1);
	if(tran_data[idx] != INT_FREE_DATA_HEADER)
	{
		return FALSE;
	}	
#endif ///< for FEATURE_INT_FREE

	while(len > 0)
	{
		if(len < step_size)
		{
			step_size = len;
		}
		ret = zet622x_i2c_read_tsdata(client, &tran_data[idx], step_size);
		len -= step_size;
		idx += step_size;
	}
	sprintf(ibase_file_name_out, "%s%s%02d.bin", tran_type_mode_file_name, IBASE_FILE_NAME, ibase_file_id);	
	zet_ibase_save(ibase_file_name_out);
	ibase_file_id  =  (ibase_file_id +1)% (IBASE_MAX_FILE_ID);
	return ret;
}
#endif ///< FEATURE_IBASE_OUT_ENABLE

///**********************************************************************
///   [function]:  zet622x_ts_parse_fpc_open
///   [parameters]: client
///   [return]: u8
///**********************************************************************
#ifdef FEATURE_FPC_OPEN_ENABLE
u8 zet622x_ts_parse_fpc_open(struct i2c_client *client)
{
	int fpcopen_packet_size = (row + col) ;
	int ret = 0;
	int idx = 0;
	int len =  fpcopen_packet_size;
	char fpcopen_file_name_out[128];
	
	int step_size = fpcopen_packet_size;
	
#ifdef FEATURE_INT_FREE
	ret = zet622x_i2c_read_tsdata(client, &tran_data[idx],  1);
	if(tran_data[idx] != INT_FREE_DATA_HEADER)
	{
		return FALSE;
	}	
#endif ///< for FEATURE_INT_FREE

	while(len > 0)
	{
		if(len < step_size)
		{
			step_size = len;
		}
		ret = zet622x_i2c_read_tsdata(client, &tran_data[idx], step_size);
		len -= step_size;
		idx += step_size;
	}
	sprintf(fpcopen_file_name_out, "%s%s%02d.bin", tran_type_mode_file_name, FPC_OPEN_FILE_NAME, fpcopen_file_id);	
	zet_fpcopen_save(fpcopen_file_name_out);
	fpcopen_file_id  =  (fpcopen_file_id +1)% (FPC_OPEN_MAX_FILE_ID);
	return ret;
}
#endif ///< FEATURE_FPC_OPEN_ENABLE

///**********************************************************************
///   [function]:  zet622x_ts_parse_fpc_short
///   [parameters]: client
///   [return]: u8
///**********************************************************************
#ifdef FEATURE_FPC_SHORT_ENABLE
u8 zet622x_ts_parse_fpc_short(struct i2c_client *client)
{
	int fpcshort_packet_size = (row + col) * 2;
	int ret = 0;
	int idx = 0;
	int len =  fpcshort_packet_size;
	char fpcshort_file_name_out[128];
	
	int step_size = col*2;
	
#ifdef FEATURE_INT_FREE
	ret = zet622x_i2c_read_tsdata(client, &tran_data[idx],  1);
	if(tran_data[idx] != INT_FREE_DATA_HEADER)
	{
		return FALSE;
	}	
#endif ///< for FEATURE_INT_FREE

	while(len > 0)
	{
		if(len < step_size)
		{
			step_size = len;
		}
		ret = zet622x_i2c_read_tsdata(client, &tran_data[idx], step_size);
		len -= step_size;
		idx += step_size;
	}
	sprintf(fpcshort_file_name_out, "%s%s%02d.bin", tran_type_mode_file_name, FPC_SHORT_FILE_NAME, fpcshort_file_id);	
	zet_fpcshort_save(fpcshort_file_name_out);
	fpcshort_file_id  =  (fpcshort_file_id +1)% (FPC_OPEN_MAX_FILE_ID);
	return ret;
}
#endif ///< FEATURE_FPC_SHORT_ENABLE

///**********************************************************************
///   [function]:  zet622x_ts_parse_initial_dev
///   [parameters]: client
///   [return]: u8
///**********************************************************************
#ifdef FEATURE_IDEV_OUT_ENABLE
u8 zet622x_ts_parse_initial_dev(struct i2c_client *client)
{
	int idev_packet_size = (row + col);
	int ret = 0;
	int idx = 0;
	int len =  idev_packet_size;
	char idev_file_name_out[128];
	
	int step_size = idev_packet_size;
	
#ifdef FEATURE_INT_FREE
	ret = zet622x_i2c_read_tsdata(client, &tran_data[idx], 1);
	if(tran_data[idx] != INT_FREE_DATA_HEADER)
	{
		return FALSE;
	}	
#endif ///< for FEATURE_INT_FREE

	while(len > 0)
	{
		if(len < step_size)
		{
			step_size = len;
		}
		ret = zet622x_i2c_read_tsdata(client, &tran_data[idx], step_size);
		len -= step_size;
		idx += step_size;
	}
	sprintf(idev_file_name_out, "%s%s%02d.bin", tran_type_mode_file_name, IDEV_FILE_NAME, idev_file_id);	
	zet_idev_save(idev_file_name_out);
	idev_file_id  =  (idev_file_id +1)% (IDEV_MAX_FILE_ID);
	return ret;
}
#endif ///< FEATURE_IDEV_OUT_ENABLE

///**********************************************************************
///   [function]:  zet622x_ts_parse_mutual_base
///   [parameters]: client
///   [return]: u8
///**********************************************************************
#ifdef FEATURE_MBASE_OUT_ENABLE
u8 zet622x_ts_parse_mutual_base(struct i2c_client *client)
{
	int mbase_packet_size = (row * col * 2);
	int ret = 0;
	int idx = 0;
	int len =  mbase_packet_size;
	char mbase_file_name_out[128];
	
	int step_size = col*2;
	
#ifdef FEATURE_INT_FREE
	ret = zet622x_i2c_read_tsdata(client, &tran_data[idx], 1);
	if(tran_data[idx] != INT_FREE_DATA_HEADER)
	{
		return FALSE;
	}	
#endif ///< for FEATURE_INT_FREE

	while(len > 0)
	{
		if(len < step_size)
		{
			step_size = len;
		}

		ret = zet622x_i2c_read_tsdata(client, &tran_data[idx], step_size);
		len -= step_size;
		idx += step_size;
	}
	sprintf(mbase_file_name_out, "%s%s%02d.bin",tran_type_mode_file_name, MBASE_FILE_NAME, mbase_file_id);	
	zet_mbase_save(mbase_file_name_out);
	mbase_file_id  =  (mbase_file_id +1)% (MBASE_MAX_FILE_ID);
	return ret;
}
#endif ///< FEATURE_MBASE_OUT_ENABLE

///**********************************************************************
///   [function]:  zet622x_ts_set_transfer_type
///   [parameters]: void
///   [return]: int
///**********************************************************************
#ifdef FEATURE_INFO_OUT_EANBLE
int zet622x_ts_set_info_type(void)
{
	int ret = 1;
	u8 ts_cmd[1] = {0xB2};
	transfer_type = TRAN_TYPE_INFORMATION_TYPE;
	ret = zet622x_i2c_write_tsdata(this_client, ts_cmd, 1);
	if(ret <= 0)
	{
		transfer_type = TRAN_TYPE_DYNAMIC;
	}
	info_recv--;
	return ret;
}
#endif ///< FEATURE_INFO_OUT_EANBLE

///   [function]:  zet622x_ts_reveice_transfer_data
///   [parameters]: client, pack_size
///   [return]: u8
///**********************************************************************
u8 zet622x_ts_reveice_transfer_data(struct i2c_client *client, int pack_size)
{
	int ret = 0;
	int idx = 0;
	int len =  pack_size;
	int step_size = pack_size;
	
	if(step_size > TRAN_PACK_DATA_SIZE)
	{
		step_size = TRAN_PACK_DATA_SIZE;
	}
	
	printk("[ZET] : zet622x_ts_reveice_transfer_data : %d\n", pack_size);
	
#ifdef FEATURE_INT_FREE
	ret = zet622x_i2c_read_tsdata(client, &tran_data[idx], 1);
	if(tran_data[idx] != INT_FREE_DATA_HEADER)
	{
		return FALSE;
	}	
#endif ///< for FEATURE_INT_FREE

	while(len > 0)
	{
		if(len < step_size)
		{
			step_size = len;
		}

		ret = zet622x_i2c_read_tsdata(client, &tran_data[idx], step_size);
		len -= step_size;
		idx += step_size;
	}
	return TRUE;
}

u8 zet622x_ts_tran_user_setting_data(struct i2c_client *client)
{
	int pack_size = tran_data_size;
	int ret = 0;
	char tran_file_name_out[128];
	
	ret = zet622x_ts_reveice_transfer_data(client, pack_size);
	if(ret == FALSE)
	{
		printk("[ZET] : zet622x_ts_tran_data_user_setting fail \n");
		return ret;
	}
	
	sprintf(tran_file_name_out, "%s%s%02d.bin",tran_type_mode_file_name, TRAN_DATA_TYPE_FILE_NAME, tran_data_file_id);	
	zet_tran_data_save(tran_file_name_out);
	tran_data_file_id  =  (tran_data_file_id +1)% (TRAN_MAX_FILE_ID);
	return ret;
}


///**********************************************************************
///   [function]:  zet622x_ts_parse_info
///   [parameters]: client
///   [return]: u8
///**********************************************************************
#ifdef FEATURE_INFO_OUT_EANBLE
int zet622x_ts_parse_info(struct i2c_client *client)
{
	int packet_size = INFO_DATA_SIZE;
	int ret = 0;
	int i;
	int len = packet_size;
	char info_file_name_out[128];
	u8 key_enable = FALSE;
	if(info_recv > 0)
		return -1;
	info_recv++;
	ret = zet622x_i2c_read_tsdata(client, &tran_data[0], len);
	if(tran_data[0] == FINGER_REPROT_DATA_HEADER)
	{
		return FALSE;
	}	

	transfer_type = TRAN_TYPE_DYNAMIC;

	/// check the ic type is right
	if(tran_data[0] != ZET6231_INFO && 
	   tran_data[0] != ZET6251_INFO &&
	   tran_data[0] != ZET6223_INFO &&
	   tran_data[0] != ZET6223_INFO_PLUS &&
	   tran_data[0] != ZET6270_INFO &&
	   tran_data[0] != ZET7130_INFO)
	{
		printk("[ZET] :  zet622x_ts_parse_info IC model fail 0x%X,  0x%X\n", tran_data[0] ,  tran_data[1] );
		return -1;
	}

	if(tran_data[0] == ZET6231_INFO)
	{
		ic_model = MODEL_ZET6231;
		rom_type = ROM_TYPE_FLASH;
	}
	else if(tran_data[0] == ZET6251_INFO)
	{
		ic_model = MODEL_ZET6251;
		rom_type = ROM_TYPE_SRAM;
	}
	else if( tran_data[0] == ZET6223_INFO)
	{
		ic_model = MODEL_ZET6223;
		rom_type = ROM_TYPE_FLASH;
	}
	else if(tran_data[0] == ZET6223_INFO_PLUS)
	{
		ic_model = MODEL_ZET6223;
		rom_type = ROM_TYPE_FLASH;
	}
	else if(tran_data[0] == ZET6270_INFO)
	{
		ic_model = MODEL_ZET6270;
		rom_type = ROM_TYPE_SRAM;
	}else if(tran_data[0] == ZET7130_INFO)
	{
		ic_model = MODEL_ZET7130;
		rom_type = ROM_TYPE_FLASH;
	}

	for(i = 0 ; i < 8 ; i++)
	{
		pcode[i] = tran_data[i] & 0xff;
	}
        
#ifndef FEATURE_FW_UPGRADE
        if(tran_data[0] == ZET6231_INFO)
        {
                ic_model = MODEL_ZET6231;
        }
        else if(tran_data[0] == ZET6251_INFO)
        {
                ic_model = MODEL_ZET6251;
        }
        else if( tran_data[0] == ZET6223_INFO)
        {
                ic_model = MODEL_ZET6223;
        }
		else if( tran_data[0] == ZET6270_INFO)
        {
                ic_model = MODEL_ZET6270;
        }else if( tran_data[0] == ZET7130_INFO)
        {
                ic_model = MODEL_ZET7130;
        }
#endif ///< for FEATURE_FW_UPGRADE	

	xy_exchange = (tran_data[16] & 0x8) >> 3;
	if(xy_exchange == 1)
	{
		resolution_y = tran_data[9] & 0xff;
		resolution_y = (resolution_y << 8)|(tran_data[8] & 0xff);
		resolution_x = tran_data[11] & 0xff;
		resolution_x = (resolution_x << 8) | (tran_data[10] & 0xff);
	}
	else
	{
		resolution_x = tran_data[9] & 0xff;
		resolution_x = (resolution_x << 8)|(tran_data[8] & 0xff);
		resolution_y = tran_data[11] & 0xff;
		resolution_y = (resolution_y << 8) | (tran_data[10] & 0xff);
	}
	finger_num = (tran_data[15] & 0x7f);
	key_enable = (tran_data[15] & 0x80);
	if(key_enable == 0)
	{
		finger_packet_size  = 3 + 4*finger_num;
	}
	else
	{
		finger_packet_size  = 3 + 4*finger_num + 1;
	}
	col = tran_data[13];  ///< trace x
	row = tran_data[14];  ///< trace y
	sprintf(info_file_name_out, "%s%s.bin",tran_type_mode_file_name, INFO_FILE_NAME);	
	zet_information_save(info_file_name_out);
	return ret;
}
#endif ///< FEATURE_INFO_OUT_EANBLE
///**********************************************************************
///   [function]:  zet622x_ts_set_trace_x_type
///   [parameters]: void
///   [return]: int
///**********************************************************************
#ifdef FEATURE_INFO_OUT_EANBLE
int zet622x_ts_set_trace_x_type(void)
{
	int ret = 0;
	u8 ts_cmd[10] = {0xC1, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; 
	transfer_type = TRAN_TYPE_TRACE_X_TYPE;
	ret = zet622x_i2c_write_tsdata(this_client, ts_cmd, 10);
	if(ret <= 0)
	{
		transfer_type = TRAN_TYPE_DYNAMIC;
	}
	return ret;        
}
#endif ///< FEATURE_INFO_OUT_EANBLE
///**********************************************************************
///   [function]:  zet622x_ts_parse_trace_x
///   [parameters]: client
///   [return]: u8
///**********************************************************************
#ifdef FEATURE_INFO_OUT_EANBLE
u8 zet622x_ts_parse_trace_x(struct i2c_client *client)
{
	int packet_size = col;
	int ret = 0;
	int len = packet_size;
	char info_file_name_out[128];
	ret = zet622x_i2c_read_tsdata(client, &tran_data[0], len);
	transfer_type = TRAN_TYPE_DYNAMIC;
	if(tran_data[0] == FINGER_REPROT_DATA_HEADER)
	{
		return FALSE;
	}
	sprintf(info_file_name_out, "%stracex.bin",tran_type_mode_file_name);	
	zet_trace_x_save(info_file_name_out);
	return ret;
}
#endif ///< FEATURE_INFO_OUT_EANBLE
///**********************************************************************
///   [function]:  zet622x_ts_set_trace_y_type
///   [parameters]: void
///   [return]: int
///**********************************************************************
#ifdef FEATURE_INFO_OUT_EANBLE
int zet622x_ts_set_trace_y_type(void)
{
	int ret = 0;
	u8 ts_cmd[10] = {0xC1, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; 
	transfer_type = TRAN_TYPE_TRACE_Y_TYPE;
	ret = zet622x_i2c_write_tsdata(this_client, ts_cmd, 10);
	if(ret <= 0)
	{
		transfer_type = TRAN_TYPE_DYNAMIC;
	}
	return ret;        
}
#endif ///< FEATURE_INFO_OUT_EANBLE
///**********************************************************************
///   [function]:  zet622x_ts_parse_trace_x
///   [parameters]: client
///   [return]: u8
///**********************************************************************
#ifdef FEATURE_INFO_OUT_EANBLE
u8 zet622x_ts_parse_trace_y(struct i2c_client *client)
{
	int packet_size = row;
	int ret = 0;
	int len = packet_size;
	char info_file_name_out[128];
	ret = zet622x_i2c_read_tsdata(client, &tran_data[0], len);
	transfer_type = TRAN_TYPE_DYNAMIC;
	if(tran_data[0] == FINGER_REPROT_DATA_HEADER)
	{
		return FALSE;
	}
	sprintf(info_file_name_out, "%stracey.bin",tran_type_mode_file_name);	
	zet_trace_y_save(info_file_name_out);
	return ret;
}
#endif ///< FEATURE_INFO_OUT_EANBLE
///**********************************************************************
///   [function]:  zet622x_ts_get_information
///   [parameters]: client
///   [return]: u8
///**********************************************************************
u8 zet622x_ts_get_information(struct i2c_client *client)
{
	u8 ts_report_cmd[1] = {0xB2};
	u8 ts_in_data[17] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	int ret;
	int i;
	u8 key_enable = FALSE;
	ret=zet622x_i2c_write_tsdata(client, ts_report_cmd, 1);
	if (ret > 0)
	{
			msleep(10);
			printk ("[ZET] : B2 read\n");
			ret=zet622x_i2c_read_tsdata(client, ts_in_data, 17);		
			if(ret > 0)
			{	
				for(i = 0; i < 8; i++)
				{
					pcode[i]=ts_in_data[i] & 0xff;
				}
				xy_exchange = (ts_in_data[16] & 0x8) >> 3;
				if(xy_exchange == 1)
				{
					resolution_y= ts_in_data[9] & 0xff;
					resolution_y= (resolution_y << 8)|(ts_in_data[8] & 0xff);
					resolution_x= ts_in_data[11] & 0xff;
					resolution_x= (resolution_x << 8) | (ts_in_data[10] & 0xff);
				}
				else
				{
					resolution_x = ts_in_data[9] & 0xff;
					resolution_x = (resolution_x << 8)|(ts_in_data[8] & 0xff);
					resolution_y = ts_in_data[11] & 0xff;
					resolution_y = (resolution_y << 8) | (ts_in_data[10] & 0xff);
				}			
				finger_num = (ts_in_data[15] & 0x7f);
			key_enable = (ts_in_data[15] & 0x80);
			if(key_enable == 0)
			{
				finger_packet_size  = 3+4*finger_num;
			}
			else
			{
				finger_packet_size  = 3+4*finger_num+1;
			}
			}
			else
			{
			printk ("[ZET] : B2 fail\n");
				return ret;
			}						
	}
	else
	{
		return ret;
	}
	return 1;
}

#if 0
///**********************************************************************
///   [function]:  zet622x_ts_interrupt
///   [parameters]: 
///   [return]:
///************************************************************************
#ifndef FEATURE_INT_FREE
static void zet622x_ts_interrupt(void)
{  		
    	
	///-------------------------------------------------------------///
	///   1. check the INT PIN is low
	///-------------------------------------------------------------///
    if(mt_get_gpio_in(GPIO_CTP_EINT_PIN) == FALSE)
	{		
		zet622x_int_flag = 1;
	}
    wake_up_interruptible(&waiter);
} 
#endif ///< for FEATURE_INT_FREE
#endif

///************************************************************************
///   [function]:  zet62xx_ts_init
///   [parameters]: 
///   [return]: void
///************************************************************************
#ifdef FEATURE_LIGHT_LOAD_REPORT_MODE
static void zet62xx_ts_init(void)
{
	u8 i;
	/// initital the pre-finger status 
	for(i = 0 ; i < MAX_FINGER_NUMBER ; i++)
	{
		pre_event[i].pressed = PRE_PRESSED_DEFAULT_VALUE;
	}
}
#endif ///< for FEATURE_LIGHT_LOAD_REPORT_MODE

///**********************************************************************
///   [function]:  zet622x_ts_clean_finger
///   [parameters]: work
///   [return]: void
///**********************************************************************
#ifdef FEATURE_SUSPEND_CLEAN_FINGER
static void zet622x_ts_clean_finger(void)
{
	int i;
	////printk("[ZET] : clean point:\n");

		
#ifdef FEATURE_BTN_TOUCH
	input_report_key(tpd->dev, BTN_TOUCH, 0);
#endif ///< for FEATURE_BTN_TOUCH

#ifdef FEATURE_MT_TYPE_B
	for(i = 0; i < finger_num; i++)
	{
		input_mt_slot(tpd->dev, i);
		input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, false);
		input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, -1);
	}
	input_mt_report_pointer_emulation(tpd->dev, true);
	#else ///< for FEATURE_MT_TYPE_B				
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
	input_mt_sync(tpd->dev);
#endif ///< for FEATURE_MT_TYPE_B
	input_sync(tpd->dev);		
}
#endif ///< for FEATURE_SUSPEND_CLEAN_FINGER

///**********************************************************************
///   [function]:  zet62xx_ts_auto_zoom
///   [parameters]: finger_report_data
///   [return]: void
///**********************************************************************
void zet62xx_ts_auto_zoom(struct finger_coordinate_struct* finger_report_data)
{
	int i;
	u32 value_x;
	u32 value_y;
	for(i = 0 ; i < MAX_FINGER_NUMBER ; i++)
	{
		if(finger_report_data[i].valid != TRUE)
		{
			continue;
		}
		value_x = (u32)(((finger_report_data[i].report_x*X_MAX*10)/FW_X_RESOLUTION + 5)/10);
		value_y = (u32)(((finger_report_data[i].report_y*Y_MAX*10)/FW_Y_RESOLUTION + 5)/10);
		finger_report_data[i].report_x = value_x;
		finger_report_data[i].report_y = value_y;	
	}
}
///**********************************************************************
///   [function]:  zet622x_ts_coordinate_translating
///   [parameters]: px, py, p
///   [return]: void
///**********************************************************************
void zet622x_ts_coordinate_translating(struct finger_coordinate_struct* finger_report_data)
{
	int i;
#if ORIGIN == ORIGIN_TOP_RIGHT
	for(i = 0 ; i < MAX_FINGER_NUMBER ; i++)
	{
		if(finger_report_data[i].valid == TRUE)
		{
			finger_report_data[i].report_x  = X_MAX - finger_report_data[i].report_x ;
		}
	}
#elif ORIGIN == ORIGIN_BOTTOM_RIGHT
	for(i = 0 ; i < MAX_FINGER_NUMBER ; i++)
	{
		if(finger_report_data[i].valid  == TRUE)
		{
			finger_report_data[i].report_x  = X_MAX - finger_report_data[i].report_x ;
			finger_report_data[i].report_y = Y_MAX - finger_report_data[i].report_y;
		}
	}
#elif ORIGIN == ORIGIN_BOTTOM_LEFT
	for(i = 0 ; i < MAX_FINGER_NUMBER ; i++)
	{
		if(finger_report_data[i].valid == TRUE)
		{
			finger_report_data[i].report_y = Y_MAX - finger_report_data[i].report_y;
		}
	}
#endif ///< for ORIGIN
}
///**********************************************************************
///   [function]:  zet622x_ts_copy_to_report_data
///   [parameters]: i2c_client
///   [return]: void
///**********************************************************************
static void zet622x_ts_copy_to_report_data(u8 *pdata, int size)
{
	/// copy to tran tmp data
	tran_data_size = size;
	memcpy(tran_tmp_data[tran_index], pdata, size);
	tran_report_point = tran_tmp_data[tran_index];
	tran_index = (tran_index + 1)%2;
	#ifdef ZET_DEBUG
	printk("[ZET] zet622x_ts_copy_to_report_data size=%d\n",size);
	#endif
	return;
}
///**********************************************************************
///   [function]:  zet622x_ts_data_to_dynamic_finger
///   [parameters]: i2c_client
///   [return]: void
///**********************************************************************
static void zet622x_ts_data_to_dynamic_finger(u8 *pData)
{	
	int i;
	u16 valid;
	u8 pressed;
	valid = pData[1];
	valid = (valid << 8) | pData[2];
	/// parse the finger data	
	/// parse the valid data to finger report data
	for(i = 0; i < finger_num; i++)
	{
		pressed = (valid >> (MAX_FINGER_NUMBER-i-1)) & 0x1;
		/// keep the last point data
		finger_report[i].last_report_x = finger_report[i].report_x;
		finger_report[i].last_report_y = finger_report[i].report_y;
		finger_report[i].last_report_z = finger_report[i].report_z;
		/// get the finger data
		finger_report[i].report_x = (u8)((pData[FINGER_HEADER_SHIFT+FINGER_PACK_SIZE*i])>>4)*256 + (u8)pData[(FINGER_HEADER_SHIFT+FINGER_PACK_SIZE*i)+1];
		finger_report[i].report_y = (u8)((pData[FINGER_HEADER_SHIFT+FINGER_PACK_SIZE*i]) & 0x0f)*256 + (u8)pData[(FINGER_HEADER_SHIFT+FINGER_PACK_SIZE*i)+2];
		finger_report[i].report_z = (u8)((pData[(FINGER_HEADER_SHIFT+FINGER_PACK_SIZE*i)+3]) & 0xff);
		finger_report[i].valid = pressed;
		
		#ifdef ZET_DEBUG
			printk("[ZET] zet622x_ts_data_to_dynamic_finger: i=%d p=%d x=%d y=%d \n",i,pressed,finger_report[i].report_x,finger_report[i].report_y);
		#endif
	}
	//if key enable
	if(key_num > 0)
	{
		finger_report_key = pData[FINGER_HEADER_SHIFT+FINGER_PACK_SIZE*finger_num];
	}
	//Hover Flag
	hover_status = (pData[6] >> 7);
#ifdef FEATURE_AUTO_ZOOM_ENABLE
	zet62xx_ts_auto_zoom(finger_report);
#endif ///< for FEATURE_AUTO_ZOOM_ENABLE
#ifdef FEATURE_TRANSLATE_ENABLE
	zet622x_ts_coordinate_translating(finger_report);
#endif ///< for FEATURE_TRANSLATE_ENABLE
#ifdef FEATURE_FRAM_RATE
	fram_rate = fram_rate + 1;
#endif ///< FEATURE_FRAM_RATE
}
///**********************************************************************
///   [function]:  zet622x_ts_parse_dynamic_finger
///   [parameters]: i2c_client
///   [return]: void
///**********************************************************************
static u8 zet622x_ts_parse_dynamic_finger(struct i2c_client *client)
{
	u8  ts_data[70];
	int ret;
	memset(ts_data,0,70);
	ret = zet622x_i2c_read_tsdata(client, &ts_data[0], finger_packet_size);
	if(ts_data[0] != FINGER_REPROT_DATA_HEADER)
	{
#ifdef ZET_DEBUG
		printk("[ZET] ts_data[0] != 0x3C\n");
#endif	
		return FALSE;
	}

	//ret = zet622x_i2c_read_tsdata(client, &ts_data[1], finger_packet_size-1);
	zet622x_ts_copy_to_report_data(ts_data, finger_packet_size);
	zet622x_ts_data_to_dynamic_finger(ts_data);	
	return TRUE;
}
///**********************************************************************
///   [function]:  zet622x_ts_finger_up_report
///   [parameters]: ts,  index
///   [return]: void
///**********************************************************************
static void zet622x_ts_finger_up_report(int index)
{
#ifdef FEATURE_LIGHT_LOAD_REPORT_MODE
	if(pre_event[index].pressed == FALSE)  ///< check the pre-finger status is up
	{
		return;
	}
	pre_event[index].pressed = FALSE;
#endif  ///< for FEATURE_LIGHT_LOAD_REPORT_MODE
#ifdef FEATURE_MT_TYPE_B
	input_mt_slot(tpd->dev, index);
	input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, false);
	input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, -1);
#endif
}

///**********************************************************************
///   [function]:  zet62xx_ts_finger_down_report
///   [parameters]: ts, index
///   [return]: void
///**********************************************************************
static void zet62xx_ts_finger_down_report(int index, struct finger_coordinate_struct* report_data)
{

#ifdef FEATURE_BTN_TOUCH
	input_report_key(tpd->dev, BTN_TOUCH, 1);
#endif ///< for FEATURE_BTN_TOUCH 
#ifdef FEATURE_LIGHT_LOAD_REPORT_MODE
	/// check the pre-finger status is pressed and X,Y is same, than skip report to the host
	if((pre_event[index].pressed == TRUE) &&
	(pre_event[index].pre_x == report_data[index].report_x) &&
	(pre_event[index].pre_y == report_data[index].report_y))
	{
		return;
	}
	/// Send finger down status to host
	pre_event[index].pressed = TRUE;
	pre_event[index].pre_x =report_data[index].report_x;
	pre_event[index].pre_y = report_data[index].report_y;
	pre_event[index].pre_z =  report_data[index].report_z;
#endif ///< for FEATURE_LIGHT_LOAD_REPORT_MODE 
#ifdef FEATURE_VIRTUAL_KEY
	if( report_data[index].report_y > TP_AA_Y_MAX)
	{ 
		report_data[index].report_y = TP_AA_Y_MAX;
	}
#endif ///< for FEATURE_VIRTUAL_KEY
#ifdef FEATURE_MT_TYPE_B
	input_mt_slot(tpd->dev, index);
	input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, true);
	input_report_abs(tpd->dev,ABS_MT_PRESSURE,  PRESSURE_CONST);
#endif ///< for FEATURE_MT_TYPE_B

	input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, index);
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, PRESSURE_CONST);
	input_report_abs(tpd->dev, ABS_MT_POSITION_X,  report_data[index].report_x); //evan modify
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y,  report_data[index].report_y);	//evan modify	
	#ifdef ZET_DEBUG
	printk( "[ZET] zet62xx_ts_finger_down_report ID=%d x=%d y=%d \n",index,report_data[index].report_x,report_data[index].report_y);
	#endif
#ifndef FEATURE_MT_TYPE_B	    		
	input_mt_sync(tpd->dev);
#endif ///< for FEATURE_MT_TYPE_B
}

///**********************************************************************
///   [function]:  zet622x_ts_key_report
///   [parameters]: ts, key index
///   [return]: void
///**********************************************************************
static void zet622x_ts_key_report(u8 ky)
{
	int i;
	u8 pressed;
  
	//printk("[ZET] : zet622x_ts_key_report : 0x%X", ky);
#ifdef FEATURE_VIRTUAL_KEY
LABEL_KEY_FINGER:
#endif ///< for FEATURE_VIRTUAL_KEY
	if(key_num <= 0)
	{
		return;
	}
	//printk("-lantao- key ky = %x.\n ",ky);
	for(i = 0 ; i < KEY_NUMBER ; i++)
	{			
		pressed = ky & ( 0x01 << i );
		switch(i)
		{
			case 0:
				if(pressed == 0x01)
				{
					if(!key_back_pressed)
					{
                    #ifdef FEATURE_KEY_TO_FINGER
                        ctp_tpd_down(key_1[0], key_1[1]);
                    #else ///< for FEATURE_KEY_TO_FINGER
						input_report_key(tpd->dev, KEY_BACK, 1);
					#endif ///< for FEATURE_KEY_TO_FINGER
                        key_back_pressed = 0x1;
					}
				}
				else
				{
					if(key_back_pressed)
					{
                    #ifdef FEATURE_KEY_TO_FINGER
                        ctp_tpd_up(key_1[0], key_1[1]);
                    #else ///< for FEATURE_KEY_TO_FINGER
						input_report_key(tpd->dev, KEY_BACK, 0);
					#endif ///< for FEATURE_KEY_TO_FINGER
						key_back_pressed = 0x0;
					}
				}
				
				break;
			case 1:
				if(pressed == 0x02)
				{
					if(!key_home_pressed)
					{
                    #ifdef FEATURE_KEY_TO_FINGER
                        ctp_tpd_down(key_2[0], key_2[1]);
                    #else ///< for FEATURE_KEY_TO_FINGER
						input_report_key(tpd->dev, KEY_HOMEPAGE, 1);
					#endif ///< for FEATURE_KEY_TO_FINGER
						key_home_pressed = 0x1;
					}
				}
				else
				{
					if(key_home_pressed)
					{
                    #ifdef FEATURE_KEY_TO_FINGER
                        ctp_tpd_up(key_2[0], key_2[1]);
                    #else ///< for FEATURE_KEY_TO_FINGER
						input_report_key(tpd->dev, KEY_HOMEPAGE, 0);
					#endif ///< for FEATURE_KEY_TO_FINGER
						key_home_pressed = 0x0;
					}
				}
				
				break;
			case 2:
				if(pressed == 0x04)
				{
					if(!key_menu_pressed)
					{
                    #ifdef FEATURE_KEY_TO_FINGER
                        ctp_tpd_down(key_3[0], key_3[1]);
                    #else ///< for FEATURE_KEY_TO_FINGER
						input_report_key(tpd->dev, KEY_MENU, 1);
					#endif ///< for FEATURE_KEY_TO_FINGER
						key_menu_pressed = 0x1;
					}
				}
				else
				{
					if(key_menu_pressed)
					{
                    #ifdef FEATURE_KEY_TO_FINGER
                        ctp_tpd_up(key_3[0], key_3[1]);
                    #else ///< for FEATURE_KEY_TO_FINGER
						input_report_key(tpd->dev, KEY_MENU, 0);
					#endif ///< for FEATURE_KEY_TO_FINGER
						key_menu_pressed = 0x0;
					}
				}	
				break;
			case 3:
				break;
			case 4:
				break;
			case 5:
				break;
			case 6:
				break;
			case 7:
				break;
		}

	}
}


///**********************************************************************
///   [function]:  zet622x_ts_finger_report
///   [parameters]: work
///   [return]: void
///**********************************************************************
static void zet622x_ts_finger_report(void)
{
	int i;
	u8 finger_cnt = 0;
	u8 chk_finger = FALSE;
	u8 ky = finger_report_key;
	///-------------------------------------------///
	//// check have finger data
	///-------------------------------------------///
	for(i = 0; i < finger_num; i++)
	{
		if(finger_report[i].valid == TRUE)
		{
			chk_finger = TRUE;
			finger_cnt = finger_cnt + 1; 
		}
	}

#ifdef FEATURE_VIRTUAL_KEY
	key_num = TPD_KEY_COUNT;
	 /// only finger 1 enable and the report location on the virtual key area
	if((finger_cnt == 1) && (finger_report[0].pressed == TRUE) && (finger_report[0].y > TP_AA_Y_MAX))
	{
		if((finger_report[0].x >= tpd_keys_dim[0][0]) && 
		   (finger_report[0].x <= tpd_keys_dim[0][1]) && 
		   (finger_report[0].y >= tpd_keys_dim[0][2]) && 
		   (finger_report[0].y <= tpd_keys_dim[0][3]))
		{
			ky=0x1;
		}
		else if((finger_report[0].x >= tpd_keys_dim[1][0]) && 
			(finger_report[0].x <= tpd_keys_dim[1][1]) && 
			(finger_report[0].y >= tpd_keys_dim[1][2]) && 
			(finger_report[0].y <= tpd_keys_dim[1][3]) ) 
		{
			ky=0x2;
		}
		else if((finger_report[0].x >= tpd_keys_dim[2][0]) && 
			(finger_report[0].x <= tpd_keys_dim[2][1]) && 
			(finger_report[0].y >= tpd_keys_dim[2][2]) && 
			(finger_report[0].y <= tpd_keys_dim[2][3])) 
		{
			ky=0x4;
		}
		else if((finger_report[0].x >= tpd_keys_dim[3][0]) && 
			(finger_report[0].x <= tpd_keys_dim[3][1]) && 
			(finger_report[0].y >= tpd_keys_dim[3][2]) && 
			(finger_report[0].y <= tpd_keys_dim[3][3])) 
		{
			ky=0x8;
		}
		goto LABEL_KEY_REPORT;
	}
#endif ///< for FEATURE_VIRTUAL_KEY
	///-------------------------------------------///
	/// all finger up report 
	///-------------------------------------------///
	if(chk_finger == FALSE)
	{
		/// finger up debounce check
		finger_up_cnt++;
		if(finger_up_cnt >= DEBOUNCE_NUMBER)
		{
			finger_up_cnt = 0;
#ifdef FEATURE_BTN_TOUCH
			input_report_key(tpd->dev, BTN_TOUCH, 0);
#endif ///< for FEATURE_BTN_TOUCH
			for(i = 0; i < finger_num; i++)
			{
				/// finger up setting
				zet622x_ts_finger_up_report(i);
			}
#ifdef FEATURE_MT_TYPE_B
			input_mt_report_pointer_emulation(tpd->dev, true);
#else
			input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
			input_mt_sync(tpd->dev);
#endif ///< for FEATURE_MT_TYPE_B
		}
	}
	else
	{
		///-------------------------------------------///
		/// parse finger report 
		///-------------------------------------------///
		finger_up_cnt = 0;
		for(i = 0 ; i < finger_num ; i++)
		{
			if(finger_report[i].valid == TRUE)
			{
				/// finger down setting
				zet62xx_ts_finger_down_report(i, finger_report);
			}
			else
			{
				/// finger up setting
				zet622x_ts_finger_up_report(i);				
			}
		}
#ifdef FEATURE_MT_TYPE_B
		input_mt_report_pointer_emulation(tpd->dev, true);
#endif ///< for FEATURE_MT_TYPE_B
		
	}
#ifdef FEATURE_VIRTUAL_KEY
LABEL_KEY_REPORT:
#endif ///< FEATURE_VIRTUAL_KEY

	zet622x_ts_key_report(ky);
	input_sync(tpd->dev);	
}

///**********************************************************************
///   [function]:  zet622x_ts_work_content
///   [parameters]: void
///   [return]:
///************************************************************************
static void zet622x_ts_work_content(void)
{
	//	struct i2c_client *tsclient1 = tpd->i2c_dev;
                        
        ///-------------------------------------------///
        /// Read no fingers in suspend mode
        ///-------------------------------------------///
		#ifdef ZET_DEBUG
		printk("[ZET] zet622x_ts_work_content.1\n ");
		#endif
		
		#ifdef GESTURE_WAKEUP
		int keycode = 0;
		if( DOZE_ENABLED == doze_status )
		{
			printk("-lantao- DOZE_WAKEUP. ***************************\\n ");
			doze_status = DOZE_WAKEUP;
			keycode = KEY_RIGHT;
			input_report_key(tpd->dev, keycode, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, keycode, 0);
			input_sync(tpd->dev);
			goto finger_report;
		}
		#endif
		
        if(suspend_mode == TRUE)
        {
            return;
        }

        if(resume_download == TRUE)
        {
                return;
        }
        #ifdef ZET_DEBUG
		printk("[ZET] zet622x_ts_work_content.2\n ");
		#endif
		
#ifdef FEATURE_INFO_OUT_EANBLE
        ///-------------------------------------------///
        /// Transfer Type :get IC information
        ///-------------------------------------------///
        if(transfer_type == TRAN_TYPE_INFORMATION_TYPE)
        {
                zet622x_ts_parse_info(this_client);
                return;
        }

        ///-------------------------------------------///
        /// Transfer Type :get trace Y name 
        ///-------------------------------------------///
        if(transfer_type == TRAN_TYPE_TRACE_Y_TYPE)
        {
                zet622x_ts_parse_trace_y(this_client);
                return;
        }

        ///-------------------------------------------///
        /// Transfer Type :get trace X name 
        ///-------------------------------------------///
        if(transfer_type == TRAN_TYPE_TRACE_X_TYPE)
        {
                zet622x_ts_parse_trace_x(this_client);
                return;
        }
                
#endif ///< for FEATURE_INFO_OUT_EANBLE
        
        ///-------------------------------------------///
        /// Transfer Type : Mutual Dev Mode 
        ///-------------------------------------------///
#ifdef FEATURE_MDEV_OUT_ENABLE
        if(transfer_type == TRAN_TYPE_MUTUAL_SCAN_DEV)
        {
                zet622x_ts_parse_mutual_dev(this_client);
                return;
        }
#endif ///< FEATURE_MDEV_OUT_ENABLE
        
        ///-------------------------------------------///
        /// Transfer Type : Initial Base Mode 
        ///-------------------------------------------///
#ifdef FEATURE_IBASE_OUT_ENABLE
        if(transfer_type == TRAN_TYPE_INIT_SCAN_BASE)
        {
                zet622x_ts_parse_initial_base(this_client);
                return;
        }       
#endif ///< FEATURE_IBASE_OUT_ENABLE

        ///-------------------------------------------///
        /// Transfer Type :  fpc open Mode 
        ///-------------------------------------------///
#ifdef FEATURE_FPC_OPEN_ENABLE
        if(transfer_type == TRAN_TYPE_FPC_OPEN)
        {
                zet622x_ts_parse_fpc_open(this_client);
                return;
        }       
#endif ///< FEATURE_FPC_OPEN_ENABLE

        ///-------------------------------------------///
        /// Transfer Type : fpc short  Mode 
        ///-------------------------------------------///
#ifdef FEATURE_FPC_SHORT_ENABLE
        if(transfer_type == TRAN_TYPE_FPC_SHORT)
        {
                zet622x_ts_parse_fpc_short(this_client);
                return;
        }       
#endif ///< FEATURE_FPC_SHORT_ENABLE

        ///-------------------------------------------///
        /// Transfer Type : Initial Dev Mode 
        ///-------------------------------------------///
#ifdef FEATURE_IDEV_OUT_ENABLE
        if(transfer_type == TRAN_TYPE_INIT_SCAN_DEV)
        {
                zet622x_ts_parse_initial_dev(this_client);
                return;
        }
#endif ///< TRAN_TYPE_INIT_SCAN_DEV

        ///-------------------------------------------///
        /// Transfer Type : Mutual Base Mode 
        ///-------------------------------------------///
#ifdef FEATURE_MBASE_OUT_ENABLE
        if(transfer_type == TRAN_TYPE_MUTUAL_SCAN_BASE)
        {
                zet622x_ts_parse_mutual_base(this_client);
                return;
        }
#endif ///< FEATURE_MBASE_OUT_ENABLE

		if(transfer_type == TRAN_TYPE_AP_USED_DATA)
		{
			zet622x_ts_tran_user_setting_data(this_client);
			return; 
		}
		
		if(transfer_type == TRAN_TYPE_MIX_FINGER_AP_USED_DATA)
		{
			zet622x_ts_tran_user_setting_data(this_client);
			zet622x_ts_data_to_dynamic_finger(tran_data);
			zet622x_ts_finger_report();
			return; 
		}

#ifdef GESTURE_WAKEUP
finger_report:
#endif
        ///-------------------------------------------///
        /// Transfer Type : Dynamic Mode 
        ///-------------------------------------------///
        if(zet622x_ts_parse_dynamic_finger(this_client) != TRUE)
        {
                return;
        }
        ///-------------------------------------------///
        /// report the finger data
        ///-------------------------------------------///
        zet622x_ts_finger_report();
		
		#ifdef ZET_DEBUG
		printk("[ZET] zet622x_ts_work_content.3\n ");
		#endif

}

///**********************************************************************
///   [function]:  zet622x_ts_work
///   [parameters]: unused
///   [return]:
///************************************************************************
static int zet622x_ts_work(void *unused)
{
	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD }; 
	sched_setscheduler(current, SCHED_RR, &param); 
	
#ifdef TPD_PROXIMITY
    s32 err = 0;
	int ret=0;
    hwm_sensor_data sensor_data;
//    u8 proximity_status;
#endif	

	do
	{
		//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
		#ifdef ZET_DEBUG
		printk("[ZET] zet622x_ts_work \n");
		#endif
		
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, zet622x_int_flag != 0);
		zet622x_int_flag = 0;
		//TPD_DEBUG_SET_TIME;
		set_current_state(TASK_RUNNING);
		zet622x_ts_work_content();
		
		#ifdef ZET_DEBUG
		printk("[ZET] zet622x_ts_work.2 \n");
		#endif
			
#if 0					
		if (tpd_proximity_flag == 1)
		{
			//get raw data
			//map and store data to hwm_sensor_data
			sensor_data.values[0] = tpd_get_ps_value();
			sensor_data.value_divide = 1;
			sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
			//report to the up-layer
			ret = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data);

			if (ret)
			{
				////printk("Call hwmsen_get_interrupt_data fail = %d\n", err);
			}
		}
#endif						
			
	} while ( !kthread_should_stop() ); 

    return 0;
}

extern int mt_get_gpio_inversion(unsigned long pin);
static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id)
{
	//TPD_DEBUG("TPD interrupt has been triggered\n");
	//tpd_flag = 1;
	
	//printk("+++++++++22simon,tpd_eint_interrupt_handler\n");
	if(mt_get_gpio_in((10 | 0x80000000)) == FALSE)
	{		
	zet622x_int_flag = 1;
	}
	wake_up_interruptible(&waiter);
	return IRQ_HANDLED;
}

static int tpd_irq_registration(void)
{
	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = {0,0};

	node = of_find_matching_node(node, touch_of_match);
	if (node) {
		/*touch_irq = gpio_to_irq(tpd_int_gpio_number);*/
		of_property_read_u32_array(node,"debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);

		touch_irq = irq_of_parse_and_map(node, 0);
		ret = request_irq(touch_irq, tpd_eint_interrupt_handler,
					IRQF_TRIGGER_FALLING, "TOUCH_PANEL-eint", NULL);
		if (ret > 0)
			TPD_DMESG("tpd request_irq IRQ LINE NOT AVAILABLE!.");
		printk("zet touch_irq = %d\n",touch_irq);
	} else {
		TPD_DMESG("[%s] tpd request_irq can not find touch eint device node!.", __func__);
	}
	return 0;
}

///**********************************************************************
///   [function]:  zet622x_ts_fram_rate
///   [parameters]: NULL
///   [return]: void
///***********************************************************************
#ifdef FEATURE_FRAM_RATE
static void zet622x_ts_fram_rate(void)
{
	last_fram_rate = fram_rate; 	
	fram_rate = 0;
	////printk("[ZET] : fram rate : %d\n", last_fram_rate);
}
#endif ///< FEATURE_FRAM_RATE

///***********************************************************************
///   [function]:  zet622x_ts_timer_task
///   [parameters]:  arg
///   [return]:  NULL
///************************************************************************
#ifdef FEATURE_CHARGER_MODE
static void zet622x_ts_timer_task(unsigned long arg)
{

	struct zet6221_tsdrv *ts_drv = (struct zet6221_tsdrv *)arg;
	queue_work(ts_drv->ts_workqueue1, &ts_drv->work1);
	mod_timer(&ts_drv->zet622x_ts_timer_task,jiffies + msecs_to_jiffies(polling_time));	
}
#endif ///< for FEATURE_CHARGER_MODE

///**********************************************************************
///   [function]:  zet622x_ts_polling_task
///   [parameters]: arg
///   [return]: void
///***********************************************************************
#ifdef FEATURE_INT_FREE
static void zet622x_ts_polling_task(unsigned long arg)
{
	struct zet6221_tsdrv *ts_drv = (struct zet6221_tsdrv *)arg;
	queue_work(ts_drv->ts_workqueue, &ts_drv->work1);
	mod_timer(&ts_drv->zet622x_ts_timer_task1, jiffies + msecs_to_jiffies(INT_FREE_TIMER));	
}
#endif ///< for FEATURE_INT_FREE

///**********************************************************************
///   [function]:  zet622x_ts_fram_rate_task
///   [parameters]: arg
///   [return]: void
///***********************************************************************
#ifdef FEATURE_FRAM_RATE
static void zet622x_ts_fram_rate_task(unsigned long arg)
{
	struct zet6221_tsdrv *ts_drv = (struct zet6221_tsdrv *)arg;
	zet622x_ts_fram_rate();
	mod_timer(&ts_drv->zet622x_ts_timer_task2, jiffies + msecs_to_jiffies(FRAM_RATE_TIMER));	
}
#endif ///< FEATURE_FRAM_RATE

///**********************************************************************
///   [function]:  zet622x_ts_charge_mode_enable
///   [parameters]: void
///   [return]: void
///**********************************************************************
void zet622x_ts_charge_mode_enable(void)
{
	u8 ts_write_charge_cmd[1] = {0xb5}; 
	int ret = 0;
#ifdef FEATURE_FW_UPGRADE_RESUME
	if(resume_download == TRUE)
	{
		return;
	}
#endif ///< for FEATURE_FW_UPGRADE_RESUME
	if(suspend_mode == TRUE)
	{
		return;
	}
	printk("[ZET] : enable charger mode\n");
	ret = zet622x_i2c_write_tsdata(this_client, ts_write_charge_cmd, 1);
}
EXPORT_SYMBOL_GPL(zet622x_ts_charge_mode_enable);
///**********************************************************************
///   [function]:  zet622x_ts_charge_mode_disable
///   [parameters]: void
///   [return]: void
///**********************************************************************
void zet622x_ts_charge_mode_disable(void)
{
	u8 ts_write_cmd[1] = {0xb6}; 
	int ret = 0;
#ifdef FEATURE_FW_UPGRADE_RESUME
	if(resume_download == TRUE)
	{
		return;
	}
#endif ///< for FEATURE_FW_UPGRADE_RESUME
	if(suspend_mode == TRUE)
	{
		return;
	}
	printk("[ZET] : disable charger mode\n");
	ret = zet622x_i2c_write_tsdata(this_client, ts_write_cmd, 1);
}
EXPORT_SYMBOL_GPL(zet622x_ts_charge_mode_disable);
///**********************************************************************
///   [function]:  zet622x_ts_polling_task
///   [parameters]: arg
///   [return]: void
///***********************************************************************
#ifdef FEATURE_INT_FREE
static void zet622x_ts_polling_task(unsigned long arg)
{
	struct zet622x_tsdrv *ts_drv = (struct zet622x_tsdrv *)arg;
	queue_work(ts_drv->ts_workqueue, &ts_drv->work1);
	mod_timer(&ts_drv->zet622x_ts_timer_task1, jiffies + msecs_to_jiffies(INT_FREE_TIMER));	
}
#endif ///< for FEATURE_INT_FREE

///***********************************************************************
///   [function]:  zet622x_charger_cmd_work
///   [parameters]:  _work
///   [return]:  NULL
///************************************************************************
void zet622x_ts_polling_work(struct work_struct *_work)
{
        zet622x_ts_work_content();
}

///***********************************************************************
///   [function]:  zet622x_charger_cmd_work
///   [parameters]:  _work
///   [return]:  NULL
///************************************************************************
void zet622x_charger_cmd_work(struct work_struct *_work)
{
	if(suspend_mode == TRUE)
	{
		return;
	}

	if(resume_download == TRUE)
	{
		return;
	}

	if(charger_on != charger_status)
	{	
		if(charger_on == TRUE)
		{
			zet622x_ts_charge_mode_enable();
			////printk("[ZET]:Charger Mode On\n");		
		}
		else if(charger_on == FALSE)
		{
			zet622x_ts_charge_mode_disable();
			////printk("[ZET]:Charger Mode Off\n");		
		}
		charger_status = charger_on;
	}

	///-------------------------------------------------------------------///
	/// IOCTL Action
	///-------------------------------------------------------------------///
	if(ioctl_action  & IOCTL_ACTION_FLASH_DUMP)
	{
		////printk("[ZET]: IOCTL_ACTION: Dump flash\n");
		zet_fw_save(fw_file_name);
		ioctl_action &= ~IOCTL_ACTION_FLASH_DUMP;
	}
}

///************************************************************************
///   [function]:  zet622x_ts_sig_check
///   [parameters]: client
///   [return]: void
///************************************************************************
#ifdef FEATURE_FW_SIGNATURE
int zet622x_ts_sig_check(struct i2c_client *client)
{
	int i;
	int ret = TRUE;
	///---------------------------------///
        /// if zet6221, then leaves
        ///---------------------------------///
	if(ic_model == MODEL_ZET6221 || ic_model == MODEL_ZET7130 || ic_model == MODEL_ZET7150)
	{
		printk("[ZET]: signature check ignored\n");
		return	TRUE;
	}
	///---------------------------------///
        /// Read sig page
        ///---------------------------------///
	ret = zet622x_cmd_readpage(client, SIG_PAGE_ID, &zet_rx_data[0]);
        if(ret <= 0)
        {
		printk("[ZET]: signature check fail\n");
        	return FALSE;
        }	
	///---------------------------------///
        /// Clear the signature position
        ///---------------------------------///
        for(i = 0 ; i < SIG_DATA_LEN ; i++)
	{
		/// erase the sig page last 4 bytes data
		flash_buffer[SIG_PAGE_ID * FLASH_PAGE_LEN + SIG_DATA_ADDR + i] = 0xFF;
	}
	///---------------------------------///
        /// check signature
        ///---------------------------------///
        printk("[ZET]: sig_curr[] =  ");
        for(i = 0 ; i < SIG_DATA_LEN ; i++)
	{
		printk("%02X ", zet_rx_data[i + SIG_DATA_ADDR]);
        }
	printk("\n");
	printk("[ZET]: sig_data[] =  ");
        for(i = 0 ; i < SIG_DATA_LEN ; i++)
	{
		printk("%02X ", sig_data[i]);
        }
	printk("\n");
    	printk("[ZET]: sig_data[] =  ");
	for(i = 0 ; i < SIG_DATA_LEN ; i++)
	{
		if(zet_rx_data[i + SIG_DATA_ADDR] != sig_data[i])
		{
			printk("[ZET]: signature check : not signatured!!\n");
			return FALSE;
		}
	}
	printk("[ZET]: signature check : signatured\n");
	return  TRUE;
}
#endif ///< for FEATURE_FW_SIGNATURE

///************************************************************************
///   [function]:  zet622x_ts_sig_write
///   [parameters]: client
///   [return]: void
///************************************************************************
#ifdef FEATURE_FW_SIGNATURE
int zet622x_ts_sig_write(struct i2c_client *client)
{
	int i;
	int ret;
	u8 tx_buf[256];
	
	///---------------------------------///
        /// if zet6221, ZET6251, then leaves
        ///---------------------------------///
	if(ic_model == MODEL_ZET6221 || ic_model == MODEL_ZET7130 || ic_model == MODEL_ZET7150)
	{
		////printk("[ZET]: signature write ignore\n");
		return	TRUE;
	}
	memset(tx_buf, 0x00, 256);
	
	///---------------------------------///
        /// get the flash_buffer page data
        ///---------------------------------///
	for(i = 0; i < FLASH_PAGE_LEN; i++)
	{
		tx_buf[i] = flash_buffer[SIG_PAGE_ID * FLASH_PAGE_LEN + i];
	}

	////printk("[ZET] : old data\n");
        for(i = 0; i < FLASH_PAGE_LEN; i++)
        {
                ////printk("%02x ", tx_buf[i]);
                if((i%0x10) == 0x0F)
                {
                        ////printk("\n");
                }
                else if((i%0x08) == 0x07)
                {
                        ////printk(" - ");
                }
        }


	///---------------------------------///
        /// set signature
        ///---------------------------------///
        for(i = 0 ; i < SIG_DATA_LEN ; i++)
        {
                tx_buf[ i + SIG_DATA_ADDR] = sig_data[i];
        }

	////printk("[ZET] : new data\n");
        for(i = 0; i < FLASH_PAGE_LEN; i++)
	{
		////printk("%02x ", tx_buf[i]);
		if((i%0x10) == 0x0F)
		{
			////printk("\n");
		}
		else if((i%0x08) == 0x07)
		{
			////printk(" - ");
		}		
	}

	///---------------------------------///
        /// write sig page
        ///---------------------------------///
	ret = zet622x_cmd_writepage(client, SIG_PAGE_ID, &tx_buf[0]);
        if(ret <= 0)
        {
		////printk("[ZET]: signature write fail\n");
        	return FALSE;
	}
	msleep(2);
	ret = zet622x_ts_sig_check(client);
	if(ret <= 0)
        {
		////printk("[ZET]: signature write fail\n");
        	return FALSE;
	}
	////printk("[ZET]: signature write ok\n");	
	return TRUE;
}
#endif ///< for FEATURE_FW_SIGNATURE

///************************************************************************
///   [function]:  zet622x_ts_project_code_get
///   [parameters]: client
///   [return]: int
///************************************************************************
int zet622x_ts_project_code_get(struct i2c_client *client)
{
	int i;
	int ret;
	///----------------------------------------///
	/// Read Data page for flash version check#1
	///----------------------------------------///
	ret = zet622x_cmd_readpage(client, (pcode_addr[0]>>7), &zet_rx_data[0]);		
	if(ret <= 0)
	{
		return ret;
	}
	printk("[ZET]: page=%3d ",(pcode_addr[0] >> 7)); ///< (pcode_addr[0]/128));
	for(i = 0 ; i < 4 ; i++)
	{
		pcode[i] = zet_rx_data[(pcode_addr[i] & 0x7f)]; ///< [(pcode_addr[i]%128)];
		printk("offset[%04x] = %02x ",i,(pcode_addr[i] & 0x7f));    ///< (pcode_addr[i]%128));
	}
	printk("\n");
	///----------------------------------------///
	/// Read Data page for flash version check#2
  	///----------------------------------------///
	ret = zet622x_cmd_readpage(client, (pcode_addr[4]>>7), &zet_rx_data[0]);
	if(ret <= 0)
	{
		return ret;
	}
	printk("[ZET]: page=%3d ",(pcode_addr[4] >> 7)); //(pcode_addr[4]/128));
	for(i = 4 ; i < PROJECT_CODE_MAX_CNT ; i++)
	{
		pcode[i] = zet_rx_data[(pcode_addr[i] & 0x7f)]; //[(pcode_addr[i]%128)];
		printk("offset[%04x] = %02x ",i,(pcode_addr[i] & 0x7f));  //(pcode_addr[i]%128));
	}
	printk("\n");
	printk("[ZET]: pcode_now : ");
	for(i = 0 ; i < PROJECT_CODE_MAX_CNT ; i++)
	{
		printk("%02x ",pcode[i]);
	}
	printk("\n");
	printk("[ZET]: pcode_new : ");
	for(i = 0 ; i < PROJECT_CODE_MAX_CNT ; i++)
	{
		printk("%02x ", flash_buffer[pcode_addr[i]]);
	}
	printk("\n");
        return ret;
	}

///************************************************************************
///   [function]:  zet71xx_ts_project_code_get
///   [parameters]: client
///   [return]: int
///************************************************************************
int zet71xx_ts_project_code_get(struct i2c_client *client)
{
	int i;
	int ret;

	///----------------------------------------///
	/// Read Data page for flash version check#1
	///----------------------------------------///
 ret = zet622x_cmd_readpage(client, (pcode_addr_7130[0]>>8), &zet_rx_data[0]);
	if(ret <= 0)
	{
		return ret;
	}
 printk("[ZET]: page=%3d ",(pcode_addr_7130[0] >> 8)); ///< (pcode_addr_7130[0]/256));
	for(i = 0 ; i < 4 ; i++)
	{
  pcode[i] = zet_rx_data[(pcode_addr_7130[i] & 0xff)]; ///< [(pcode_addr_7130[i]%256)];
  printk("offset[%04x] = %02x ",i,(pcode_addr_7130[i] & 0xff));    ///< (pcode_addr_7130[i]%256));
	}
	printk("\n");

	///----------------------------------------///
	/// Read Data page for flash version check#2
  	///----------------------------------------///
 ret = zet622x_cmd_readpage(client, (pcode_addr_7130[4]>>8), &zet_rx_data[0]);
	if(ret <= 0)
	{
		return ret;
	}	

 printk("[ZET]: page=%3d ",(pcode_addr_7130[4] >> 8)); //(pcode_addr_7130[4]/256));
	for(i = 4 ; i < PROJECT_CODE_MAX_CNT ; i++)
	{
  pcode[i] = zet_rx_data[(pcode_addr_7130[i] & 0xff)]; //[(pcode_addr_7130[i]%256)];
  printk("offset[%04x] = %02x ",i,(pcode_addr_7130[i] & 0xff));  //(pcode_addr_7130[i]%256));
	}
	printk("\n");
        
	printk("[ZET]: pcode_now : ");
	for(i = 0 ; i < PROJECT_CODE_MAX_CNT ; i++)
	{
		printk("%02x ",pcode[i]);
	}
	printk("\n");
	
	printk("[ZET]: pcode_new : ");
	for(i = 0 ; i < PROJECT_CODE_MAX_CNT ; i++)
	{
  printk("%02x ", flash_buffer[pcode_addr_7130[i]]);
	}
	printk("\n");
        
        return ret;
}

///**********************************************************************
///   [function]:  zet622x_download_recover
///   [parameters]: client, 
///   [return]: void
///**********************************************************************
#define RECOVER_COUNT 20
u8 zet622x_download_recover(struct i2c_client *client,u8 romtype)
{
	int ret = 0;
	int count = 0;
	
RECOVER_FROM_RESET:
	ctp_set_reset_high();
	msleep(1);
	ctp_set_reset_low();
	msleep(1);
//RECOVER_FROM_PWD:
	ret = zet622x_cmd_sndpwd(client);
	if(ret <= 0)
	{
		if(count<=RECOVER_COUNT)
		{
			count++;
			goto RECOVER_FROM_RESET;
		}
		return ret;
	}
	msleep(10);
	
	if(romtype == ROM_TYPE_UNKNOWN)
		goto EXIT;
	
//RECOVER_FROM_SFR:

	///--------------------------------------------------------------zet71xx_cmd_writer------------///
	/// 71xx need to enable reader and writer
	///--------------------------------------------------------------------------///
	if(ic_model == MODEL_ZET7130 || ic_model == MODEL_ZET7150)
	{
		ret = zet71xx_cmd_writer(client);	
		if(ret <= 0)
		{
			if(count<=RECOVER_COUNT)
			{
				count++;
				goto RECOVER_FROM_RESET;
			}
			return ret;
		}			
		
		ret = zet71xx_cmd_reader(client);	
		if(ret <= 0)
		{
			if(count<=RECOVER_COUNT)
			{
				count++;
				goto RECOVER_FROM_RESET;
			}
			return ret;
		}	
		
		goto EXIT;
		
	}

	if(romtype == ROM_TYPE_FLASH)
	{
		///----------------------------------------///
		/// Read SFR
		///----------------------------------------///
		ret = zet622x_cmd_sfr_read(client);
		if(ret <= 0)
		{
			if(count<=RECOVER_COUNT)
			{
				count++;
				goto RECOVER_FROM_RESET;
			}
			return ret;
		}
		///----------------------------------------///
		/// Update the SFR[14] = 0x3D
		///----------------------------------------///
		ret = zet622x_cmd_sfr_unlock(client);
		if(ret <= 0)
		{
			if(count<=RECOVER_COUNT)
			{
				count++;
				goto RECOVER_FROM_RESET;
			}
			return ret;
		}
		msleep(20);
	}
EXIT:
	return ret;
}
///**********************************************************************
///   [function]:  zet622x_ts_data_flash_download
///   [parameters]: client
///   [return]: void
///**********************************************************************
#ifdef FEATRUE_TRACE_SENSOR_ID
int zet622x_ts_data_flash_download(struct i2c_client *client)
{
	int ret = 0;
	int i;
	int flash_total_len 	= 0;
	int flash_rest_len 	= 0;	
	int flash_page_id	= 0;
	int now_flash_rest_len	= 0;
	int now_flash_page_id	= 0;
	int retry_count		= 0;
	download_ok = TRUE;
	printk("[ZET] : zet622x_ts_data_flash_download\n");
	///----------------------------------------///
	/// 1. set_reset pin low
	///----------------------------------------///
	ctp_set_reset_low();
	msleep(1);
	///----------------------------------------///
	/// 2. send password
	///----------------------------------------///
	ret = zet622x_cmd_sndpwd(client);	
	if(ret <= 0)
	{
		ret = zet622x_download_recover(client,ROM_TYPE_UNKNOWN);
		if(ret <= 0)
		{
			download_ok = FALSE;
			retry_count = 0;
			goto LABEL_DATAFLASH_DOWNLOAD_EXIT;
		}
	}
	msleep(10);
	/// unlock the write protect of 0xFC00~0xFFFF
	if((ic_model != MODEL_ZET6251) && (ic_model != MODEL_ZET6270))
	{
	    if(ic_model == MODEL_ZET6223)
		{
	        ret = zet622x_cmd_sndpwd_1k(client);	
	        if(ret <= 0)
	        {
				download_ok = FALSE;
				retry_count = 0;
				goto LABEL_DATAFLASH_DOWNLOAD_EXIT;
	        }	
        }	
		///----------------------------------------///
		/// Read SFR
		///----------------------------------------///
		ret = zet622x_cmd_sfr_read(client);	
		if(ret <= 0)
		{
			ret = zet622x_download_recover(client,ROM_TYPE_FLASH);
			if(ret <= 0)
			{
				download_ok = FALSE;
				retry_count = 0;
				goto LABEL_DATAFLASH_DOWNLOAD_EXIT;
			}
		}
		///----------------------------------------///
		/// Update the SFR[14] = 0x3D
		///----------------------------------------///  
		ret = zet622x_cmd_sfr_unlock(client);
		if(ret <= 0)
		{
			ret = zet622x_download_recover(client,ROM_TYPE_FLASH);
			if(ret <= 0)
		{
				download_ok = FALSE;
				retry_count = 0;
				goto LABEL_DATAFLASH_DOWNLOAD_EXIT;
			}
		}
		msleep(20);
	}
	/// first erase the Sig. page
#ifdef FEATURE_FW_SIGNATURE
	if((ic_model != MODEL_ZET6251) && (ic_model != MODEL_ZET6270))
	{
		///------------------------------///
		/// Do page erase
		///------------------------------///    
	    ret = zet622x_cmd_pageerase(client, SIG_PAGE_ID);
	    if(ret <= 0)
		{
			download_ok = FALSE;
			retry_count = 0;
			goto LABEL_DATAFLASH_DOWNLOAD_EXIT;
		}
	        msleep(30);
	}
#endif ///< for FEATURE_FW_SIGNATURE
	flash_total_len = MAX_DATA_FLASH_BUF_SIZE;
	flash_page_id = DATA_FLASH_START_ID;
	flash_rest_len = flash_total_len;
	while(flash_rest_len >0)
	{
		memset(zet_tx_data, 0x00, 131);
//#ifdef FEATURE_FW_COMPARE
LABEL_DATA_FLASH_PAGE:		
//#endif ///< for FEATURE_FW_COMPARE
		/// Do page erase 
		if((ic_model != MODEL_ZET6251) && (ic_model != MODEL_ZET6270))
  		{
 			///------------------------------///
    			/// Do page erase
    			///------------------------------///    
			ret = zet622x_cmd_pageerase(client, flash_page_id);
	
			if(ret <= 0)
			{
				ret = zet622x_download_recover(client,ROM_TYPE_FLASH);
				if(ret <= 0)
				{
					download_ok = FALSE;
					retry_count = 0;
					ctp_set_reset_high();
					msleep(1);
					ctp_set_reset_low();
					msleep(1);
					goto LABEL_DATAFLASH_DOWNLOAD_EXIT;
				}
				else
				{
					if(retry_count<5)
					{
						retry_count++;
						goto LABEL_DATA_FLASH_PAGE;
					}
					else
					{
						download_ok = FALSE;
						retry_count = 0;
						ctp_set_reset_high();
						msleep(1);
						ctp_set_reset_low();
						msleep(1);
						goto LABEL_DATAFLASH_DOWNLOAD_EXIT;
					}
				}						
 		}

			msleep(30);
 		}
		//printk( " [ZET] : write page%d\n", flash_page_id);
		now_flash_rest_len	= flash_rest_len;
		now_flash_page_id	= flash_page_id;		
		///---------------------------------///
		/// Write page
		///---------------------------------///		
		ret = zet622x_cmd_writepage(client, flash_page_id, &flash_buffer[flash_page_id * FLASH_PAGE_LEN]);
		
		if(ret <= 0)
		{
			ret = zet622x_download_recover(client,rom_type);
			if(ret <= 0)
			{
				download_ok = FALSE;
				retry_count = 0;
				ctp_set_reset_high();
				msleep(1);
				ctp_set_reset_low();
				msleep(1);
				goto LABEL_DATAFLASH_DOWNLOAD_EXIT;
			}
			else
			{
				if(retry_count<5)
				{
					retry_count++;
					goto LABEL_DATA_FLASH_PAGE;
				}
				else
				{
					download_ok = FALSE;
					retry_count = 0;
					ctp_set_reset_high();
					msleep(1);
					ctp_set_reset_low();
					msleep(1);
					goto LABEL_DATAFLASH_DOWNLOAD_EXIT;
				}
			}	
		}
		
		flash_rest_len -= FLASH_PAGE_LEN;
		if((ic_model != MODEL_ZET6251) && (ic_model != MODEL_ZET6270))
		{
			msleep(5);
		}		
#ifdef FEATURE_FW_COMPARE
		///---------------------------------///
		/// Read page
		///---------------------------------///
		ret = zet622x_cmd_readpage(client, flash_page_id, &zet_rx_data[0]);		
		if(ret <= 0)
		{
			ret = zet622x_download_recover(client,rom_type);
			if(ret <= 0)
			{
				download_ok = FALSE;
				retry_count = 0;
				ctp_set_reset_high();
				msleep(1);
				ctp_set_reset_low();
				msleep(1);
				goto LABEL_DATAFLASH_DOWNLOAD_EXIT;
			}else
			{
				retry_count++;
				goto LABEL_DATA_FLASH_PAGE;
			}
		}	
		for(i = 0 ; i < FLASH_PAGE_LEN ; i++)
		{
			if(i < now_flash_rest_len)
			{
				if(flash_buffer[flash_page_id * FLASH_PAGE_LEN + i] != zet_rx_data[i])
				{
					flash_rest_len = now_flash_rest_len;
					flash_page_id = now_flash_page_id;				
					if(retry_count < 5)
					{
						retry_count++;
						goto LABEL_DATA_FLASH_PAGE;
					}
					else
					{
						download_ok = FALSE;
						retry_count = 0;						
						ctp_set_reset_high();
						msleep(1);
						ctp_set_reset_low();
						msleep(1);
						goto LABEL_DATAFLASH_DOWNLOAD_EXIT;
					}
				}
			}
		}		
#endif ///< for FEATURE_FW_COMPARE
		retry_count=0;
		flash_page_id+=1;
	}


LABEL_DATAFLASH_DOWNLOAD_EXIT:	
	
	if(download_ok == TRUE && ic_model == MODEL_ZET6223)
	{
#ifdef FEATURE_FW_SIGNATURE
                if(zet622x_ts_sig_write(client) == FALSE)
                {
                	download_ok = FALSE;
                }
#endif ///< for FEATURE_FW_SIGNATURE
        }

	//zet622x_cmd_resetmcu(client);
	//msleep(10);

	ctp_set_reset_high();
	msleep(2);

	if(download_ok == FALSE)
	{
		printk("[ZET] : download data flash failed!\n");
	}
        else
        {    
                ///---------------------------------///
        	/// update the project code
        	///---------------------------------///
		printk("[ZET] : download data flash pass!\n");
                for(i = 0 ; i < PROJECT_CODE_MAX_CNT ; i++)
                {      
                        pcode[i] = flash_buffer[pcode_addr[i]];
                }
        }
                   
	ctp_set_reset_low();
        msleep(5);
	ctp_set_reset_high();

        msleep(20);

	return ret;
}
#endif ///< for  FEATRUE_TRACE_SENSOR_ID

///************************************************************************
///   [function]:  zet622x_downloader
///   [parameters]: client, upgrade, 
///   [return]: int
///************************************************************************
#ifdef FEATURE_FW_UPGRADE
int __init zet622x_downloader( struct i2c_client *client, u8 upgrade, u8 *pRomType, u8 icmodel)
{
 	int ret;
//#ifdef FEATURE_FW_COMPARE
	int i;
//#endif ///< for FEATURE_FW_COMPARE
	int flash_total_len 	= 0;
	int flash_rest_len 	= 0;	
	int flash_page_id	= 0;
	int now_flash_rest_len	= 0;
	int now_flash_page_id	= 0;
	int retry_count		= 0;
	u8 uRomType=*pRomType;
#ifdef FEATURE_FW_SKIP_FF
	u8 bSkipWrite = TRUE;
#endif ///< for FEATURE_FW_SKIP_FF
#ifdef FEATURE_FW_DOWNLOAD_CHECK_SUM
	u8 check_sum		= 0;
#endif ///< for FEATURE_FW_CHECK_SUM
	int sector_sub_id = 0;
	int flash_page_len = 0;

	download_ok = TRUE;
	
	///----------------------------------------///
	/// 1. set_reset pin low
	///----------------------------------------///
	ctp_set_reset_low();  
	msleep(1);
	///----------------------------------------///
	/// 2. send password
	///----------------------------------------///
	ret=zet622x_cmd_sndpwd(client);
	if(ret<=0)
	{
		ret = zet622x_download_recover(client,ROM_TYPE_UNKNOWN);
		if(ret <= 0)
		{
			download_ok = FALSE;
			retry_count = 0;
			goto LABEL_EXIT_DOWNLOAD;
		}
	}	
	msleep(10);
	///----------------------------------------///
	/// Read Code Option
	///----------------------------------------///
LABEL_READ_CODEOPTION:
	ret = zet622x_cmd_codeoption(client, &uRomType);
	if(ret <= 0)
	{
		ret = zet622x_download_recover(client,ROM_TYPE_UNKNOWN);
	if(ret<=0)
	{
			download_ok = FALSE;
			retry_count = 0;
			goto LABEL_EXIT_DOWNLOAD;
		}else
		{
			if(retry_count < 5)
			{
				retry_count++;
				goto LABEL_READ_CODEOPTION;
			}else
			{
				download_ok = FALSE;
				retry_count = 0;
				goto LABEL_EXIT_DOWNLOAD;
			}
		}
	}
	retry_count = 0;
	
 	*pRomType = uRomType;
	msleep(10);
	if(upgrade == 0)
	{
		printk("[ZET]: HW_CHECK_ONLY enable! It is zeitec product and not going to upgrade FW. \n");
		return 1;
	}	
	///--------------------------------------------------------------------------///
	/// 4.1 the ZET6223 need unlock the write protect of 0xFC00~0xFFFF
	///--------------------------------------------------------------------------///
	if(ic_model == MODEL_ZET6223)
	{
		ret = zet622x_cmd_sndpwd_1k(client);	
		if(ret <= 0)
		{
			download_ok = FALSE;
			retry_count = 0;
			goto LABEL_EXIT_DOWNLOAD;
		}	
	}
	
	///--------------------------------------------------------------------------///
	/// 71xx need to enable reader and writer
	///--------------------------------------------------------------------------///
	if(ic_model == MODEL_ZET7130 || ic_model == MODEL_ZET7150)
	{
		ret = zet71xx_cmd_writer(client);	
		if(ret <= 0)
		{
			download_ok = FALSE;
			retry_count = 0;
			goto LABEL_EXIT_DOWNLOAD;
		}			
		
		ret = zet71xx_cmd_reader(client);	
		if(ret <= 0)
		{
			download_ok = FALSE;
			retry_count = 0;
			goto LABEL_EXIT_DOWNLOAD;
		}
	
		goto LABEL_EXIT_DOWNLOAD; //albert+ 20160222 exit download if ic=7135 for 3D touch.	
		
	}	
	
	///------------------------------------------------///
	/// init the file
	///------------------------------------------------///
	zet_fw_init();
	///------------------------------------------------///
	/// the SRAM need download code
	///------------------------------------------------///
	if((ic_model == MODEL_ZET6251) || (ic_model == MODEL_ZET6270))
	{
		goto LABEL_START_DOWNLOAD;
	}else if(ic_model == MODEL_ZET7150)
	{
		goto LABEL_START_DOWNLOAD_71xx;
	}
	
	///================================///
	///        Check version
	///================================///
	if (uRomType == ROM_TYPE_FLASH)
	{
		
	///----------------------------------------///
	/// Clear Read-in buffer
	///----------------------------------------///
		memset(zet_rx_data, 0x00, 257);
        
		if(ic_model == MODEL_ZET7130)
		{
			zet71xx_ts_project_code_get(client);
		}else
		{
	zet622x_ts_project_code_get(client);
		}

#ifdef FEATURE_FW_SIGNATURE
	///----------------------------------------///
        /// Check the data flash version
        ///----------------------------------------///
        if(zet622x_ts_sig_check(client)==TRUE)
        {
		///----------------------------------------///
		/// Check the data flash version
		///----------------------------------------///
        	if(zet622x_ts_check_version() == TRUE)
		{
			goto LABEL_EXIT_DOWNLOAD;
		}
        }
#else ///< for FEATURE_FW_SIGNATURE
	///----------------------------------------///
	/// Check the data flash version
	///----------------------------------------///
	if(zet622x_ts_check_version() == TRUE)
	{
		goto LABEL_EXIT_DOWNLOAD;
	}
#endif  ///< for FEATURE_FW_SIGNATURE
	
		if(ic_model == MODEL_ZET7130)	
		{
			goto LABEL_START_DOWNLOAD_71xx;
	        }
	}
	
	///================================///
	///        Start to download
	///================================///
LABEL_START_DOWNLOAD:	

	if(uRomType == ROM_TYPE_FLASH)
	{
	///----------------------------------------///
	/// Read SFR
	///----------------------------------------///
	ret=zet622x_cmd_sfr_read(client);
	if(ret<=0)
	{
			ret = zet622x_download_recover(client,ROM_TYPE_FLASH);
			if(ret <= 0)
			{
				download_ok = FALSE;
				retry_count = 0;
				goto LABEL_EXIT_DOWNLOAD;
			}
	}
	///----------------------------------------///
	/// Update the SFR[14] = 0x3D
	///----------------------------------------///
		ret = zet622x_cmd_sfr_unlock(client);
		if(ret <= 0)
		{
			ret = zet622x_download_recover(client,ROM_TYPE_FLASH);
			if(ret <= 0)
	{
				download_ok = FALSE;
				retry_count = 0;
				goto LABEL_EXIT_DOWNLOAD;				
			}
	}
	msleep(20);
	///------------------------------///
	/// mass erase
	///------------------------------///
		ret = zet622x_cmd_masserase(client);
		if(ret <= 0)
		{
			if(retry_count < 5)
			{
				retry_count++;
				goto LABEL_START_DOWNLOAD;
			}
			else
	{
				download_ok = FALSE;
				retry_count = 0;
				goto LABEL_EXIT_DOWNLOAD;
			}			
		}
		msleep(40);
		retry_count = 0;
	}
	
LABEL_START_DOWNLOAD_71xx:
	
	if(ic_model == MODEL_ZET7130 || ic_model == MODEL_ZET7150)
	{
		flash_page_len = FLASH_PAGE_LEN_71xx;
	}else
	{
		flash_page_len = FLASH_PAGE_LEN;
	}

	flash_total_len = zet_fw_size();
	flash_rest_len = flash_total_len;
	while(flash_rest_len >0)
	{	
		memset(zet_tx_data, 0x00, 257);
#ifdef FEATURE_FW_DOWNLOAD_CHECK_SUM
LABEL_CHECKSUM_RESUME_DOWNLOAD_PAGE:
#endif ///< for FEATURE_FW_DOWNLOAD_CHECK_SUM		
#ifdef FEATURE_FW_COMPARE
LABEL_DOWNLOAD_PAGE:
#endif ///< for FEATURE_FW_COMPARE
LABEL_DOWNLOAD_RECOVER:
		/// Do page erase
		if(ic_model == MODEL_ZET7130)
		{
			sector_sub_id = flash_page_id & 0x0F;
			if( 0 == sector_sub_id && 0 == retry_count )
			{
				retry_count = 1; // fake retry
			}else if( retry_count > 0 )
			{
				flash_rest_len += (FLASH_PAGE_LEN_71xx * sector_sub_id);
				flash_page_id -= sector_sub_id;
			}
		}
		if(retry_count > 0)
  		{
 			///------------------------------///
    		/// Do sector/page erase
    			///------------------------------///   
    			if(uRomType == ROM_TYPE_FLASH)
    			{
      			ret = zet622x_cmd_pageerase(client, flash_page_id);
				if(ret <= 0)
				{
					ret = zet622x_download_recover(client,ROM_TYPE_FLASH);
					if(ret <= 0)
					{
						download_ok = FALSE;
						retry_count = 0;
						ctp_set_reset_high();
						msleep(1);
						ctp_set_reset_low();
						msleep(1);
						goto LABEL_EXIT_DOWNLOAD;
					}
					else
					{
						if(retry_count<5)
    			{
							retry_count++;
							goto LABEL_DOWNLOAD_RECOVER;
						}
						else
						{
							download_ok = FALSE;
							retry_count = 0;
							ctp_set_reset_high();
							msleep(1);
							ctp_set_reset_low();
							msleep(1);
							goto LABEL_EXIT_DOWNLOAD;
						}
					}						
				}
    			}
 		}
		//printk( " [ZET] : write page%d\n", flash_page_id);
		now_flash_rest_len	= flash_rest_len;
		now_flash_page_id	= flash_page_id;
#ifdef FEATURE_FW_SKIP_FF
		if(retry_count==0)
		{
			bSkipWrite = zet622x_ts_check_skip_page(&flash_buffer[flash_page_id * flash_page_len]);

			if(bSkipWrite == TRUE)
		{
				//printk( " [ZET] : skip write page%d\n", flash_page_id);
			retry_count = 0;
	                flash_page_id += 1;	
				flash_rest_len-=flash_page_len;				
			continue;
		}	
			bSkipWrite = FALSE;
		}
#endif ///< for FEATURE_SKIP_FF	
		///---------------------------------///
		/// Write page
		///---------------------------------///		
		ret = zet622x_cmd_writepage(client, flash_page_id, &flash_buffer[flash_page_id * flash_page_len]);
		
		if(ret <= 0)
		{
			ret = zet622x_download_recover(client,uRomType);
			if(ret <= 0)
			{
				download_ok = FALSE;
				retry_count = 0;
				ctp_set_reset_high();
				msleep(1);
				ctp_set_reset_low();
				msleep(1);
				goto LABEL_EXIT_DOWNLOAD;
			}
			else
			{
				if(retry_count<5)
				{
					retry_count++;
					goto LABEL_DOWNLOAD_RECOVER;
				}
				else
				{
					download_ok = FALSE;
					retry_count = 0;
					ctp_set_reset_high();
					msleep(1);
					ctp_set_reset_low();
					msleep(1);
					goto LABEL_EXIT_DOWNLOAD;
				}
			}	
		}
		
		flash_rest_len -= flash_page_len;
		
		if(uRomType == ROM_TYPE_FLASH)
		{
			if(ic_model == MODEL_ZET7130)
			{
				msleep(30);
			}else
		        {
			        msleep(5);
		        }
		}

#ifdef FEATURE_FW_DOWNLOAD_CHECK_SUM
	  	if((ic_model == MODEL_ZET6251) || (ic_model == MODEL_ZET6270) || ic_model == MODEL_ZET7130 || ic_model == MODEL_ZET7150)
	  	{
			check_sum = zet622x_ts_sram_check_sum(client, flash_page_id, &flash_buffer[flash_page_id * flash_page_len]);
		
			if(check_sum == FALSE)
			{
				flash_rest_len = now_flash_rest_len;
				flash_page_id = now_flash_page_id;
				if(retry_count < 5)
		{
						retry_count++;
						goto LABEL_CHECKSUM_RESUME_DOWNLOAD_PAGE;
				}
				else
				{
					download_ok = FALSE;
					retry_count = 0;
					ctp_set_reset_high();
					msleep(1);
					ctp_set_reset_low();
					msleep(1);
					printk("[ZET] zet622x downloader checksum fail\n");
					goto LABEL_EXIT_DOWNLOAD;
				}
			}
		}		
#endif  ///< for FEATURE_FW_DOWNLOAD_CHECK_SUM
		
#ifdef FEATURE_FW_COMPARE
		///---------------------------------///
		/// Read page
		///---------------------------------///
		ret = zet622x_cmd_readpage(client, flash_page_id, &zet_rx_data[0]);		
		if(ret<=0)
		{
			ret = zet622x_download_recover(client,uRomType);
			if(ret <= 0)
			{
				download_ok = FALSE;
				retry_count = 0;
				ctp_set_reset_high();
				msleep(1);
				ctp_set_reset_low();
				msleep(1);
				goto LABEL_EXIT_DOWNLOAD;
			}else
			{
				retry_count++;
				goto LABEL_DOWNLOAD_PAGE;
			}
		}		
		///--------------------------------------------------------------------------///
		/// 10. compare data
		///--------------------------------------------------------------------------///
		for(i = 0 ; i < flash_page_len ; i++)
		{
			if(i < now_flash_rest_len)
			{
				if(flash_buffer[flash_page_id * flash_page_len + i] != zet_rx_data[i])
				{
					flash_rest_len = now_flash_rest_len;
					flash_page_id = now_flash_page_id;			
					if(retry_count < 5)
					{
						retry_count++;
						goto LABEL_DOWNLOAD_PAGE;
					}
					else
					{
						download_ok = FALSE;
						retry_count = 0;						
						ctp_set_reset_high();
						msleep(1);
						ctp_set_reset_low();
						msleep(1);
						goto LABEL_EXIT_DOWNLOAD;
					}
				}
			}
		}		
#endif ///< for FEATURE_FW_COMPARE
		retry_count=0;
		flash_page_id+=1;
	}

    	///---------------------------------///  
    	/// WDT reset
 	///---------------------------------///
#ifdef FEATURE_WDT_API
	mtk_wdt_restart(WD_TYPE_NORMAL); ///< kick external WDT   
	mtk_wdt_restart(WD_TYPE_NOLOCK);
#endif ///< for FEATURE_WDT_API

	///---------------------------------///
        /// write signature
        ///---------------------------------///
#ifdef FEATURE_FW_SIGNATURE
        if(download_ok == TRUE && uRomType == ROM_TYPE_FLASH)
		{
			if(zet622x_ts_sig_write(client) == FALSE)
			{
				download_ok = FALSE;
			}
		}
#endif ///< for FEATURE_FW_SIGNATURE
LABEL_EXIT_DOWNLOAD:
	if(download_ok == FALSE)
	{
		printk("[ZET] : download failed!\n");
	}

	//zet622x_cmd_resetmcu(client);
	//msleep(10);

	ctp_set_reset_high();

	ctp_set_reset_low();
        msleep(5);
	ctp_set_reset_high();

	if(ic_model == MODEL_ZET6221 || download_ok == FALSE)
	{
	        return 1;
        }
        /// download pass then copy the pcode
	for(i = 0 ; i < PROJECT_CODE_MAX_CNT ; i++)
	{      
	        pcode[i] = flash_buffer[pcode_addr[i]];
	}
#ifdef FEATRUE_TRACE_GPIO_INPUT
        /// get the gpio input setting
	zet622x_ts_gpio_input_get();
        #ifdef FEATRUE_TRACE_SENSOR_ID
        /// get sensor id setting on the data flash 
        if(zet622x_ts_sensor_id_bin_set(trace_input_status) == TRUE)
        {
                zet622x_ts_data_flash_download(client);
        }
        #endif ///< for FEATRUE_TRACE_SENSOR_ID

#endif ///< for FEATRUE_TRACE_GPIO_INPUT
	return 1;
}
#endif ///< for FEATURE_FW_UPGRADE

///************************************************************************
///   [function]:  zet622x_resume_downloader
///   [parameters]: client, upgrade, romtype, icmodel
///   [return]: int
///************************************************************************
static int zet622x_resume_downloader(struct i2c_client *client, u8 upgrade, u8 *romtype, u8 icmodel)
{
	int ret = 0;
#ifdef FEATURE_FW_SKIP_FF
	u8 bSkipWrite;
#endif ///< for FEATURE_FW_SKIP_FF
	int retry_count		= 0;
#ifdef FEATURE_FW_CHECK_SUM
	u8 check_sum		= 0;
#endif ///< for FEATURE_FW_CHECK_SUM
	int flash_total_len 	= FLASH_SIZE_ZET6231;
	int flash_rest_len 	= 0;
	int flash_page_id 	= 0;
	int flash_page_len  = 0;
	int sector_sub_id = 0;
	int now_flash_rest_len	= 0;
	int now_flash_page_id	= 0;	
	u8 uRomType=*romtype;
	///-------------------------------------------------------------///
	///   1. Set RST=LOW
	///-------------------------------------------------------------///
	ctp_set_reset_low();
	printk("[ZET] : RST = LOW\n");
	///-------------------------------------------------------------///
	/// 2.Send password
	///-------------------------------------------------------------///
	ret = zet622x_cmd_sndpwd(client);
	if(ret <= 0)
	{
		ret = zet622x_download_recover(client,ROM_TYPE_UNKNOWN);
		if(ret <= 0)
		{
			retry_count = 0;
			goto LABEL_RESUME_DOWNLOAD_FINISH;
		}
	}

/////////////////////////////////////////////////////////work aroud for info
	ret = zet622x_cmd_codeoption(client, &uRomType);
	if(ret <= 0)
	{
		ret = zet622x_download_recover(client,ROM_TYPE_UNKNOWN);
		if(ret<=0)
		{
			printk( " [ZET] : zet622x_resume_downloader -> zet622x_cmd_codeoption failed!! \n");
			goto LABEL_RESUME_DOWNLOAD_FINISH;
		}
	}
	
/////////////////////////////////////////////////////////	
	
	
	switch(ic_model)
	{
		case MODEL_ZET6221:
			flash_total_len = FLASH_SIZE_ZET6221;
			printk( " [ZET] : ic model 6221\n");
			break;
		case MODEL_ZET6223: 
			flash_total_len =  FLASH_SIZE_ZET6223;
			printk( " [ZET] : ic model 6223\n");
			break;
		case MODEL_ZET6270:
			flash_total_len = FLASH_SIZE_ZET6270;
			printk( " [ZET] : ic model 6270\n");
			break;
		case MODEL_ZET7130: ///< for 7130
			flash_total_len = FLASH_SIZE_ZET7100;			
			printk( " [ZET] : ic model 7130\n");
			break;
		case MODEL_ZET7150: ///< for 7150
			flash_total_len = FLASH_SIZE_ZET7100;
			printk( " [ZET] : ic model 7150\n");
			break;
		case MODEL_ZET6231: 			
			flash_total_len =  FLASH_SIZE_ZET6231;
			printk( " [ZET] : ic model 6231\n");
			break;
		case MODEL_ZET6251: 
		default: 
			flash_total_len =  FLASH_SIZE_ZET6231;
			printk( " [ZET] : ic model 6251\n");
			break;
	}
  	/// unlock the write protect of 0xFC00~0xFFFF
  	if(ic_model == MODEL_ZET6223)
  	{
  		ret = zet622x_cmd_sndpwd_1k(client);	
  		if(ret <= 0)
  		{
			retry_count = 0;
			goto LABEL_RESUME_DOWNLOAD_FINISH;
		}
	}
	
	///--------------------------------------------------------------------------///
	/// 71xx need to enable reader and writer
	///--------------------------------------------------------------------------///
	if(ic_model == MODEL_ZET7130 || ic_model == MODEL_ZET7150)
	{
		ret = zet71xx_cmd_writer(client);	
		if(ret <= 0)
		{
			retry_count = 0;
			goto LABEL_RESUME_DOWNLOAD_FINISH;
  		}
		
		ret = zet71xx_cmd_reader(client);	
		if(ret <= 0)
		{
			retry_count = 0;
			goto LABEL_RESUME_DOWNLOAD_FINISH;
  	} 	
		goto LABEL_START_DOWNLOAD_71xx;
	}		
	
LABEL_RESUME_DOWNLOAD_SFR:	

	if(rom_type == ROM_TYPE_FLASH)
	{
	  	///----------------------------------------///
	  	/// Read SFR
	  	///----------------------------------------///
	  	ret = zet622x_cmd_sfr_read(client);	
	  	if(ret <= 0)
	  	{
			ret = zet622x_download_recover(client,ROM_TYPE_FLASH);
			if(ret <= 0)
			{
				retry_count = 0;
				goto LABEL_RESUME_DOWNLOAD_FINISH;
			}
	  	}
	  	///----------------------------------------///
	  	/// Update the SFR[14] = 0x3D
	  	///----------------------------------------///  
		ret = zet622x_cmd_sfr_unlock(client);
		if(ret <= 0)
		{
			ret = zet622x_download_recover(client,ROM_TYPE_FLASH);
			if(ret <= 0)
	  	{
				retry_count = 0;
				goto LABEL_RESUME_DOWNLOAD_FINISH;
			}
	  	}
	  	msleep(20);			
	  	///------------------------------///
	  	/// mass erase
	  	///------------------------------///		
		zet622x_cmd_masserase(client);
		if(ret <= 0)
		{
			if(retry_count < 5)
			{
				retry_count++;
				goto LABEL_RESUME_DOWNLOAD_SFR;
			}
			else
			{
				retry_count = 0;
				goto LABEL_RESUME_DOWNLOAD_FINISH;
			}			
		}		
		msleep(30);
		retry_count = 0;
	}

LABEL_START_DOWNLOAD_71xx:
	
	if(ic_model == MODEL_ZET7130 || ic_model == MODEL_ZET7150)
	{
		flash_page_len = FLASH_PAGE_LEN_71xx;
	}else
	{
		flash_page_len = FLASH_PAGE_LEN;
	}

	flash_rest_len = zet_fw_size();//flash_total_len;
	printk( " [ZET] : flash_rest_len=%d\n", flash_rest_len);
	///-------------------------------------------------------------///
	/// Read Firmware from BIN if any
	///-------------------------------------------------------------///
	zet_fw_load(fw_file_name);
#if 0 //def FEATURE_FW_CHECK_SUM
  	if((ic_model == MODEL_ZET6251) || (ic_model == MODEL_ZET6270) || (ic_model == MODEL_ZET7130) || (ic_model == MODEL_ZET7150))
  	{
		printk( " [ZET] : FEATURE_FW_CHECK_SUM \n");
	///-------------------------------------------------------------///
	/// add the sram check sum to compare the data
	///-------------------------------------------------------------///
		while(flash_rest_len>0)
		{
	#ifdef FEATURE_FW_SKIP_FF
			bSkipWrite = zet622x_ts_check_skip_page(&flash_buffer[flash_page_id * flash_page_len]);
        		if(bSkipWrite == TRUE)
			{
        		        printk( " [ZET] : FEATURE_FW_CHECK_SUM skip write page%d\n", flash_page_id);
					flash_rest_len-=flash_page_len;
			        flash_page_id += 1;
			        continue;
			}
	#endif ///< for FEATURE_SKIP_FF 
            check_sum = zet622x_ts_sram_check_sum(client, flash_page_id, &flash_buffer[flash_page_id * flash_page_len]);
			if(check_sum == FALSE)
			{
				printk("[ZET] :  check the check sum have differ\n");
				goto LABEL_START_RESUME_DOWNLOAD;
			}
			printk( " [ZET] : FEATURE_FW_CHECK_SUM correct page%d\n", flash_page_id);
			flash_rest_len -= flash_page_len;
			flash_page_id++;
		}
		goto LABEL_RESUME_DOWNLOAD_FINISH;
	}
LABEL_START_RESUME_DOWNLOAD:
	printk("[ZET] :  LABEL_START_RESUME_DOWNLOAD\n");
	flash_rest_len = zet_fw_size();//flash_total_len;
	flash_page_id = 0;
#endif  ///< for FEATURE_FW_CHECK_SUM
	while(flash_rest_len>0)
	{
#ifdef FEATURE_FW_SKIP_FF
		bSkipWrite = zet622x_ts_check_skip_page(&flash_buffer[flash_page_id * flash_page_len]);
		if(bSkipWrite == TRUE)
		{
		        printk( " [ZET] : skip write page%d\n", flash_page_id);
		        flash_rest_len-=flash_page_len;
		        flash_page_id += 1;
		        continue;
		}
		bSkipWrite = FALSE;
#endif ///< for FEATURE_SKIP_FF 
		//---------------------------------///
		/// 5. Write page
		///--------------------------------///		
//#ifdef FEATURE_FW_CHECK_SUM
LABEL_RETRY_DOWNLOAD_PAGE:
//#endif  ///< for FEATURE_FW_CHECK_SUM
		/// Do page erase
		if(ic_model == MODEL_ZET7130)
		{
			sector_sub_id = flash_page_id & 0x0F;
			if( 0 == sector_sub_id && 0 == retry_count )
			{
				retry_count = 1; // fake retry
			}else if( retry_count > 0 )
			{
				flash_rest_len += (FLASH_PAGE_LEN_71xx * sector_sub_id);
				flash_page_id -= sector_sub_id;
			}
		}	
		if(retry_count > 0)
		{
		if(rom_type == ROM_TYPE_FLASH)
	{
		///------------------------------///
		/// Do page erase
		///------------------------------///    
			ret = zet622x_cmd_pageerase(client, flash_page_id);
			if(ret<=0)
			{
				ret = zet622x_download_recover(client,ROM_TYPE_FLASH);
				if(ret <= 0)
				{
					retry_count = 0;
					ctp_set_reset_high();
					msleep(1);
					ctp_set_reset_low();
					msleep(1);
					goto LABEL_RESUME_DOWNLOAD_FINISH;
				}
				else
				{
					if(retry_count<5)
					{
						retry_count++;
						goto LABEL_RETRY_DOWNLOAD_PAGE;
					}
					else
					{
						retry_count = 0;
						ctp_set_reset_high();
						msleep(1);
						ctp_set_reset_low();
						msleep(1);
						goto LABEL_RESUME_DOWNLOAD_FINISH;
					}
				}					
			}
 		}
 		}
		
		now_flash_rest_len = flash_rest_len;
		now_flash_page_id  = flash_page_id;
		
		ret = zet622x_cmd_writepage(client, flash_page_id, &flash_buffer[flash_page_id * flash_page_len]);
		if(ret <= 0)
		{
			ret = zet622x_download_recover(client,rom_type);
			if(ret <= 0)
			{
				retry_count = 0;
				ctp_set_reset_high();
				msleep(1);
				ctp_set_reset_low();
				msleep(1);
				goto LABEL_RESUME_DOWNLOAD_FINISH;
			}
			else
			{
				if(retry_count<5)
				{
					retry_count++;
					goto LABEL_RETRY_DOWNLOAD_PAGE;
				}
				else
				{
					retry_count = 0;
					ctp_set_reset_high();
					msleep(1);
					ctp_set_reset_low();
					msleep(1);
					goto LABEL_RESUME_DOWNLOAD_FINISH;
				}
			}	
		}	
		
		flash_rest_len -= flash_page_len;

		if(uRomType == ROM_TYPE_FLASH)
		{
			if(ic_model == MODEL_ZET7130)
			{
		msleep(30);
			}else
		{
			msleep(5);
	}
	}
		
#ifdef FEATURE_FW_CHECK_SUM
	  	if((ic_model == MODEL_ZET6251) || (ic_model == MODEL_ZET6270) || ic_model == MODEL_ZET7130 || ic_model == MODEL_ZET7150)
		{
			check_sum = zet622x_ts_sram_check_sum(client, flash_page_id, &flash_buffer[flash_page_id * flash_page_len]);
			if(check_sum == FALSE)
			{		
				flash_rest_len = now_flash_rest_len;
				flash_page_id = now_flash_page_id;			
				if(retry_count < 5)
				{
					retry_count++;
					//flash_rest_len += flash_page_len;
					/// zet6251 add reset function
					//ctp_set_reset_high();
					//msleep(1);		
					//ctp_set_reset_low();
					//msleep(1);
					//zet622x_cmd_sndpwd(client);		
					goto LABEL_RETRY_DOWNLOAD_PAGE;
				}
				else
				{
					retry_count = 0;						
					ctp_set_reset_high();
					msleep(1);
					ctp_set_reset_low();
					msleep(1);
					printk("[ZET] zet622x_resume_downloader fail\n");
					goto LABEL_RESUME_DOWNLOAD_FINISH;
				}
		}
		retry_count  = 0;	
	  	}
#endif  ///< for FEATURE_FW_CHECK_SUM
		retry_count  = 0;
		flash_page_id++;			 	
	}
//#ifdef FEATURE_FW_CHECK_SUM
LABEL_RESUME_DOWNLOAD_FINISH:
//#endif ///< for FEATURE_FW_CHECK_SUM
	printk("[ZET] RST = HIGH\n");
	///-------------------------------------------------------------///
	/// reset_mcu command
	///-------------------------------------------------------------///
    //printk("[ZET] zet622x_cmd_resetmcu\n");
	//zet622x_cmd_resetmcu(client);
	//msleep(10);
	///-------------------------------------------------------------///
	///   SET RST=HIGH
	///-------------------------------------------------------------///
	ctp_set_reset_high();
	msleep(20);
	///-------------------------------------------------------------///
	/// RST toggle 	
	///-------------------------------------------------------------///
	ctp_set_reset_low();
	msleep(2);
	ctp_set_reset_high();
	msleep(2);
	printk("[ZET]: zet622x_resume_downloader finish\n");
	return ret;
}
#if 0 //def FEATURE_FW_UPGRADE_RESUME
///************************************************************************
///   [function]:  zet622x_resume_download_thread
///   [parameters]: arg
///   [return]: int
///************************************************************************
static int zet622x_resume_download_thread(void *arg)
{
	int ret = 0;
	printk("[ZET] : Thread Enter\n");
	resume_download = TRUE;
	if((rom_type == ROM_TYPE_SRAM) ||
	   (rom_type == ROM_TYPE_OTP)) //SRAM,OTP
  	{
	if((ic_model == MODEL_ZET6251) || (ic_model == MODEL_ZET6270))
	{
			zet622x_resume_downloader(this_client, firmware_upgrade, &rom_type, ic_model);
			printk("[ZET] zet622x download OK\n");
  		}
	}
	printk("[ZET] : Thread Leave\n");
	resume_download = FALSE;
	return ret;
}
#endif ///< for FEATURE_FW_UPGRADE_RESUME

///************************************************************************
///   [function]:  zet622x_ts_detect
///   [parameters]: 
///   [return]:
///************************************************************************
static int zet622x_ts_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    strcpy(info->type, TPD_DEVICE);
    return 0;
}

///***********************************************************************
///   [function]:  zet_file_save
///   [parameters]: char *
///   [return]: void
///************************************************************************
static void zet_file_save(char *file_name, int data_total_len)
{
	struct file *fp;

	///-------------------------------------------------------///        
	/// create the file that stores the data
	///-------------------------------------------------------///        
	fp = filp_open(file_name, O_RDWR | O_CREAT, 0644);
	if(IS_ERR(fp))
	{
		printk("[ZET] : Failed to open %s\n", file_name);
		return;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_write(fp, tran_data, data_total_len, &(fp->f_pos));
	set_fs(old_fs);
	filp_close(fp, 0);

	return;
}

///***********************************************************************
///   [function]:  zet_tran_data_save
///   [parameters]: char *
///   [return]: void
///************************************************************************
static void zet_tran_data_save(char *file_name)
{
	int data_total_len  = tran_data_size;
	
	if(i_copy_to_report_data != 0)
	{
		zet622x_ts_copy_to_report_data(tran_data, data_total_len);
	}
	zet_file_save(file_name, data_total_len);
	return;
}


///***********************************************************************
///   [function]:  zet_mdev_save
///   [parameters]: char *
///   [return]: void
///************************************************************************
static void zet_mdev_save(char *file_name)
{
	struct file *fp;
	int data_total_len  = (row+2) * (col + 2)*2;

	memcpy(mdev_data, tran_data, data_total_len);

        ///-------------------------------------------------------///        
        /// create the file that stores the mutual dev data
        ///-------------------------------------------------------///        
        fp = filp_open(file_name, O_RDWR | O_CREAT, 0644);
        if(IS_ERR(fp))
        {
                ////printk("[ZET] : Failed to open %s\n", file_name);
                return;
        }
        old_fs = get_fs();
        set_fs(KERNEL_DS);

        vfs_write(fp, tran_data, data_total_len, &(fp->f_pos));
        set_fs(old_fs);
        filp_close(fp, 0);

        return;
}

///***********************************************************************
///   [function]:  zet_idev_save
///   [parameters]: char *
///   [return]: void
///************************************************************************
#ifdef FEATURE_IDEV_OUT_ENABLE
static void zet_idev_save(char *file_name)
{
	struct file *fp;
	int data_total_len  = (row + col);

	memcpy(idev_data, tran_data, data_total_len);

	///-------------------------------------------------------///        
	/// create the file that stores the initial dev data
	///-------------------------------------------------------///        
	fp = filp_open(file_name, O_RDWR | O_CREAT, 0644);
	if(IS_ERR(fp))
	{
		////printk("[ZET] : Failed to open %s\n", file_name);
		return;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);

        vfs_write(fp, tran_data, data_total_len, &(fp->f_pos));
        set_fs(old_fs);
        filp_close(fp, 0);

        return;
}
#endif ///< FEATURE_IDEV_OUT_ENABLE

///***********************************************************************
///   [function]:  zet_ibase_save
///   [parameters]: char *
///   [return]: void
///************************************************************************
#ifdef FEATURE_IBASE_OUT_ENABLE
static void zet_ibase_save(char *file_name)
{
	struct file *fp;
	int data_total_len  = (row + col) * 2;
	
	memcpy(ibase_data, tran_data, data_total_len);
	///-------------------------------------------------------///        
	/// create the file that stores the initial base data
	///-------------------------------------------------------///        
	fp = filp_open(file_name, O_RDWR | O_CREAT, 0644);
	if(IS_ERR(fp))
	{
		////printk("[ZET] : Failed to open %s\n", file_name);
		return;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_write(fp, tran_data, data_total_len, &(fp->f_pos));
	set_fs(old_fs);
	filp_close(fp, 0);

	return;
}
#endif ///< FEATURE_IBASE_OUT_ENABLE

///***********************************************************************
///   [function]:  zet_fpcopen_save
///   [parameters]: char *
///   [return]: void
///************************************************************************
#ifdef FEATURE_FPC_OPEN_ENABLE
static void zet_fpcopen_save(char *file_name)
{
	struct file *fp;
	int data_total_len  = (row + col) ;
	
	memcpy(fpcopen_data, tran_data, data_total_len);
	///-------------------------------------------------------///        
	/// create the file that stores the initial base data
	///-------------------------------------------------------///        
	fp = filp_open(file_name, O_RDWR | O_CREAT, 0644);
	if(IS_ERR(fp))
	{
		////printk("[ZET] : Failed to open %s\n", file_name);
		return;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_write(fp, tran_data, data_total_len, &(fp->f_pos));
	set_fs(old_fs);
	filp_close(fp, 0);

	return;
}
#endif ///< FEATURE_FPC_OPEN_ENABLE

///***********************************************************************
///   [function]:  zet_fpcshort_save
///   [parameters]: char *
///   [return]: void
///************************************************************************
#ifdef FEATURE_FPC_SHORT_ENABLE
static void zet_fpcshort_save(char *file_name)
{
	struct file *fp;
	int data_total_len  = (row + col)*2 ;
	
	memcpy(fpcshort_data, tran_data, data_total_len);
	///-------------------------------------------------------///        
	/// create the file that stores the initial base data
	///-------------------------------------------------------///        
	fp = filp_open(file_name, O_RDWR | O_CREAT, 0644);
	if(IS_ERR(fp))
	{
		////printk("[ZET] : Failed to open %s\n", file_name);
		return;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_write(fp, tran_data, data_total_len, &(fp->f_pos));
	set_fs(old_fs);
	filp_close(fp, 0);

	return;
}
#endif ///< FEATURE_FPC_SHORT_ENABLE

///***********************************************************************
///   [function]:  zet_mbase_save
///   [parameters]: char *
///   [return]: void
///************************************************************************
#ifdef FEATURE_MBASE_OUT_ENABLE
static void zet_mbase_save(char *file_name)
{
	struct file *fp;
	int data_total_len  = (row * col * 2);
				

	///-------------------------------------------------------///        
	/// create the file that stores the mutual base data
	///-------------------------------------------------------/// 
	memcpy(mbase_data, tran_data, data_total_len);       
	fp = filp_open(file_name, O_RDWR | O_CREAT, 0644);
	if(IS_ERR(fp))
	{
		////printk("[ZET] : Failed to open %s\n", file_name);
		return;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_write(fp, tran_data, data_total_len, &(fp->f_pos));
	set_fs(old_fs);
	filp_close(fp, 0);

	return;
}
#endif ///< FEATURE_MBASE_OUT_ENABLE

///***********************************************************************
///   [function]:  zet_information_save
///   [parameters]: char *
///   [return]: void
///************************************************************************
#ifdef FEATURE_INFO_OUT_EANBLE
static void zet_information_save(char *file_name)
{
	struct file *fp;
	int data_total_len  = INFO_DATA_SIZE;
				

	///-------------------------------------------------------///        
	/// create the file that stores the mutual base data
	///-------------------------------------------------------/// 
	memcpy(info_data, tran_data, data_total_len);       
	fp = filp_open(file_name, O_RDWR | O_CREAT, 0644);
	if(IS_ERR(fp))
	{
		////printk("[ZET] : Failed to open %s\n", file_name);
		filp_close(fp, 0);
		return;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_write(fp, info_data, data_total_len, &(fp->f_pos));
	set_fs(old_fs);
	filp_close(fp, 0);

	return;
}
#endif ///< FEATURE_INFO_OUT_EANBLE

///***********************************************************************
///   [function]:  zet_trace_x_save
///   [parameters]: char *
///   [return]: void
///************************************************************************
#ifdef FEATURE_INFO_OUT_EANBLE
static void zet_trace_x_save(char *file_name)
{
	struct file *fp;
	int data_total_len  = col;

	///-------------------------------------------------------///        
	/// create the file that stores the trace X data
	///-------------------------------------------------------///  
	memcpy(trace_x_data, tran_data, data_total_len);      
	fp = filp_open(file_name, O_RDWR | O_CREAT, 0644);
	if(IS_ERR(fp))
	{
		////printk("[ZET] : Failed to open %s\n", file_name);
		return;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_write(fp, tran_data, data_total_len, &(fp->f_pos));
	set_fs(old_fs);
	filp_close(fp, 0);

	return;
}
#endif ///< FEATURE_INFO_OUT_EANBLE

///***********************************************************************
///   [function]:  zet_trace_y_save
///   [parameters]: char *
///   [return]: void
///************************************************************************
#ifdef FEATURE_INFO_OUT_EANBLE
static void zet_trace_y_save(char *file_name)
{
	struct file *fp;
	int data_total_len  = row;

	///-------------------------------------------------------///        
	/// create the file that stores the trace Y data
	///-------------------------------------------------------///     
	memcpy(trace_y_data, tran_data, data_total_len);   
	fp = filp_open(file_name, O_RDWR | O_CREAT, 0644);
	if(IS_ERR(fp))
	{
		////printk("[ZET] : Failed to open %s\n", file_name);
		return;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_write(fp, tran_data, data_total_len, &(fp->f_pos));
	set_fs(old_fs);
	filp_close(fp, 0);

	return;
}
#endif ///< FEATURE_INFO_OUT_EANBLE

///************************************************************************
///   [function]:  zet_dv_set_file_name
///   [parameters]: void
///   [return]: void
///************************************************************************
static void zet_dv_set_file_name(char *file_name)
{
	strcpy(driver_version, file_name);
}

///************************************************************************
///   [function]:  zet_dv_set_file_name
///   [parameters]: void
///   [return]: void
///************************************************************************
static void zet_fw_set_pcode_name(char *file_name)
{
	strcpy(pcode_version, file_name);
}

///************************************************************************
///   [function]:  zet_fw_set_file_name
///   [parameters]: void
///   [return]: void
///************************************************************************
static void zet_fw_set_file_name(char *file_name)
{
	strcpy(fw_file_name, file_name);
}

///************************************************************************
///   [function]:  zet_mdev_set_file_name
///   [parameters]: void
///   [return]: void
///************************************************************************
static void zet_tran_type_set_file_name(char *file_name)
{
	strcpy(tran_type_mode_file_name, file_name);
}

///***********************************************************************
///   [function]:  zet_fw_size
///   [parameters]:  void
///   [return]: void
///************************************************************************
int zet_fw_size(void)
{
	int flash_total_len 	= FLASH_SIZE_ZET6231;
	
	switch(ic_model)
	{
		case MODEL_ZET6221:
			flash_total_len = FLASH_SIZE_ZET6221;
			break;
		case MODEL_ZET6223: 
			flash_total_len = FLASH_SIZE_ZET6223;
			break;
		case MODEL_ZET6270:
			flash_total_len = FLASH_SIZE_ZET6270;
			break;
		case MODEL_ZET7130:
		case MODEL_ZET7150:
			flash_total_len = 0xF000; //60k
		case MODEL_ZET6231: 			
		case MODEL_ZET6251: ///< ZET6231 & ZET6251 is size same
		default: 
			flash_total_len = FLASH_SIZE_ZET6231;
			break;
	}
	
	return flash_total_len;
}

///***********************************************************************
///   [function]:  zet_fw_save
///   [parameters]: file name
///   [return]: void
///************************************************************************
static void zet_fw_save(char *file_name)
{
	struct file *fp;
	int flash_total_len 	= 0;
	
	fp = filp_open(file_name, O_RDWR | O_CREAT, 0644);
	if(IS_ERR(fp))
	{
		////printk("[ZET] : Failed to open %s\n", file_name);
		return;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	flash_total_len = zet_fw_size();
	////printk("[ZET] : flash_total_len = 0x%04x\n",flash_total_len );

	vfs_write(fp, flash_buffer, flash_total_len, &(fp->f_pos));
	
	set_fs(old_fs);

	filp_close(fp, 0);	

	
	return;
}

///***********************************************************************
///   [function]:  zet_fw_load
///   [parameters]: file name
///   [return]: void
///************************************************************************
static void zet_fw_load(char *file_name)
{	
	int file_length = 0;
	struct file *fp;
	loff_t *pos;
	
	//printk("[ZET]: find %s\n", file_name);
	fp = filp_open(file_name, O_RDONLY, 0644);
	if(IS_ERR(fp))
	{			
		//printk("[ZET]: No firmware file detected\n");
		return;
	}

	///----------------------------///
	/// Load from file
	///----------------------------///		
	//printk("[ZET]: Load from %s\n", file_name);	

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	/// Get file size
	inode = fp->f_dentry->d_inode;
	file_length = (int)inode->i_size;

	pos = &(fp->f_pos); 

	vfs_read(fp, &flash_buffer[0], file_length, pos);

	//file_length
	set_fs(old_fs);
	filp_close(fp, 0);
	chk_have_bin_file = TRUE;

}

///************************************************************************
///   [function]:  zet_mem_init
///   [parameters]: void
///   [return]: void
///************************************************************************
static void zet_mem_init(void)
{
	
	if(flash_buffer == NULL)
	{
  		flash_buffer = kmalloc(MAX_FLASH_BUF_SIZE, GFP_KERNEL);	
	}

        ///---------------------------------------------///
        /// Init the mutual dev buffer
        ///---------------------------------------------///
	if(mdev_data== NULL)
	{
		mdev_data   = kmalloc(MDEV_MAX_DATA_SIZE, GFP_KERNEL);
	}
	if(idev_data== NULL)
	{
		idev_data   = kmalloc(IDEV_MAX_DATA_SIZE, GFP_KERNEL);
	}

	if(mbase_data== NULL)
	{
		mbase_data  = kmalloc(MBASE_MAX_DATA_SIZE, GFP_KERNEL);
	}
	if(ibase_data== NULL)
	{
		ibase_data  = kmalloc(IBASE_MAX_DATA_SIZE, GFP_KERNEL);
	}	
	
#ifdef FEATURE_FPC_OPEN_ENABLE
	if(fpcopen_data == NULL)
	{
		fpcopen_data  = kmalloc(FPC_OPEN_MAX_DATA_SIZE, GFP_KERNEL);
	}
#endif ///< for FEATURE_FPC_OPEN_ENABLE

#ifdef FEATURE_FPC_SHORT_ENABLE
	if(fpcshort_data == NULL)
	{
		fpcshort_data  = kmalloc(FPC_SHORT_MAX_DATA_SIZE, GFP_KERNEL);
	}
#endif ///< for FEATURE_FPC_SHORT_ENABLE
	
	if(tran_data == NULL)
	{
	        tran_data  = kmalloc(MBASE_MAX_DATA_SIZE, GFP_KERNEL);
	}

	if(tran_tmp_data[0] == NULL)
	{
		tran_tmp_data[0]  = kmalloc(MBASE_MAX_DATA_SIZE, GFP_KERNEL);
	}

	if(tran_tmp_data[1] == NULL)
	{
		tran_tmp_data[1]  = kmalloc(MBASE_MAX_DATA_SIZE, GFP_KERNEL);
	}

	if(info_data == NULL)
	{
	        info_data  = kmalloc(INFO_MAX_DATA_SIZE, GFP_KERNEL);
	}

	if(trace_x_data == NULL)
	{
	        trace_x_data  = kmalloc(INFO_MAX_DATA_SIZE, GFP_KERNEL);
	}
	if(trace_y_data == NULL)
	{
	        trace_y_data  = kmalloc(INFO_MAX_DATA_SIZE, GFP_KERNEL);
	}
	
}
///************************************************************************
///   [function]:  zet_get_fw_checksum
///   [parameters]: u8 *
///   [return]: u8
///************************************************************************
#ifdef ESD_CHECKSUM
int zet_get_fw_checksum(u8 * pFW)
{
	int i;
	int fw_len= 0;
	u8  * pCheckSum;
	int FWCheckSum = 0;
	fw_len = sizeof(zeitec_zet6270_firmware);
	printk("[ZET]: FW len = %3d ",fw_len);
	pCheckSum = pFW;
	fw_checksum_value = 0;
	for(i=0; i < fw_len ;i++)
	{
		FWCheckSum ^= *pCheckSum;
		pCheckSum++;
	}
	printk("[ZET]: FW checksum = %3d \n",FWCheckSum);
	fw_checksum_value ^= (FWCheckSum & 0xff000000)>>24;
	fw_checksum_value ^= (FWCheckSum & 0x00ff0000)>>16;
	fw_checksum_value ^= (FWCheckSum & 0x0000ff00)>>8;
	fw_checksum_value ^= (FWCheckSum & 0x000000ff)>>0;
	printk("[ZET]: FW checksum = %3d \n",fw_checksum_value);
	return fw_checksum_value;
}
///************************************************************************
///   [function]:  zet_get_flash_checksum
///   [parameters]: u8 *
///   [return]: u8
///************************************************************************
int zet_get_flash_checksum(void)
{
	int ret;
	u8 flash_checksum_value = 0;
	u8 ts_cmd[1] = {0xB8};
	u8 ts_cmd_c1_b[10] = {0xC1,0x02,0x0F,0x55,0xAA,0x00,0x00,0x00,0x00,0x00};
	u8 ts_cmd_c1_e[10] = {0xC1,0x02,0x00,0x55,0xAA,0x00,0x00,0x00,0x00,0x00};
	//reset ic
	ctp_wakeup();
	msleep(20);
	//send b8 cmd
	ret = zet622x_i2c_write_tsdata(this_client,ts_cmd,1);
	if(ret <= 0)
	{
		printk("[ZET]: flash checksum write B8 cmd error !\n");
		return ret;
	}
	//send c1 cmd
	msleep(1);
	ret = zet622x_i2c_write_tsdata(this_client,ts_cmd_c1_b,10);
	msleep(10);
	//read data from flash
	ret = zet622x_i2c_read_tsdata(this_client,&flash_checksum_value,1);
	if(ret <= 0)
	{
		printk("[ZET]: flash checksum read data error !\n");
		return ret;
	}
	//send c1 cmd
	msleep(1);
	ret = zet622x_i2c_write_tsdata(this_client,ts_cmd_c1_e,10);
	msleep(10);
	printk("[ZET]: Flash checksum = %3d \n",flash_checksum_value);
	return flash_checksum_value;
}
#endif //ESD_CHECKSUM
///************************************************************************
///   [function]:  zet_fw_init
///   [parameters]: void
///   [return]: void
///************************************************************************
static void zet_fw_init(void)
{
	int i;
	//printk("[ZET]: Load from header\n");

	if(ic_model == MODEL_ZET6221)
	{
		for(i = 0 ; i < sizeof(zeitec_zet6221_firmware) ; i++)
		{
			flash_buffer[i] = zeitec_zet6221_firmware[i];
		}
                
	}
	else if(ic_model == MODEL_ZET6223)
	{
		for(i = 0 ; i < sizeof(zeitec_zet6223_firmware) ; i++)
		{
			flash_buffer[i] = zeitec_zet6223_firmware[i];
		}
#ifdef FEATRUE_TRACE_SENSOR_ID
               flash_buffer_01 = &zeitec_zet6223_01_firmware[0];
               flash_buffer_02 = &zeitec_zet6223_02_firmware[0];
               flash_buffer_03 = &zeitec_zet6223_03_firmware[0];
#endif ///< FEATRUE_TRACE_SENSOR_ID
	}
	else if(ic_model == MODEL_ZET6231)
	{
		for(i = 0 ; i < sizeof(zeitec_zet6231_firmware) ; i++)
		{
			flash_buffer[i] = zeitec_zet6231_firmware[i];
		}
#ifdef FEATRUE_TRACE_SENSOR_ID
               flash_buffer_01 = &zeitec_zet6231_01_firmware[0];
               flash_buffer_02 = &zeitec_zet6231_02_firmware[0];
               flash_buffer_03 = &zeitec_zet6231_03_firmware[0];
#endif ///< FEATRUE_TRACE_SENSOR_ID
	}
	else if(ic_model == MODEL_ZET6251)
	{
		for(i = 0 ; i < sizeof(zeitec_zet6251_firmware) ; i++)
		{
			flash_buffer[i] = zeitec_zet6251_firmware[i];
		}
#ifdef FEATRUE_TRACE_SENSOR_ID
		flash_buffer_01 = &zeitec_zet6251_01_firmware[0];
      flash_buffer_02 = &zeitec_zet6251_02_firmware[0];
      flash_buffer_03 = &zeitec_zet6251_03_firmware[0];
#endif ///< FEATRUE_TRACE_SENSOR_ID
	}
	else if(ic_model == MODEL_ZET6270)
	{
		for(i=0;i<sizeof(zeitec_zet6270_firmware);i++)
		{
			flash_buffer[i] = zeitec_zet6270_firmware[i];
		}
	}
	else if(ic_model == MODEL_ZET7130)
	{
		for(i = 0 ; i < sizeof(zeitec_zet7100_firmware) ; i++)
		{
			flash_buffer[i] = zeitec_zet7100_firmware[i];
		}
                
	}
	else if(ic_model == MODEL_ZET7150)
	{
		for(i = 0 ; i < sizeof(zeitec_zet7100_firmware) ; i++)
		{
			flash_buffer[i] = zeitec_zet7100_firmware[i];
		}
	}
	/// Load firmware from bin file
	zet_fw_load(fw_file_name);
}

///************************************************************************
///   [function]:  zet_fw_exit
///   [parameters]: void
///   [return]: void
///************************************************************************
static void zet_fw_exit(void)
{
        ///---------------------------------------------///
	/// free mdev_data
        ///---------------------------------------------///
	if(mdev_data!=NULL)
	{
		kfree(mdev_data);
		mdev_data = NULL;
	}

	if(idev_data!=NULL)
	{			
		kfree(idev_data);
		idev_data = NULL;
	}

	if(mbase_data!=NULL)
	{	
		kfree(mbase_data);
		mbase_data = NULL;
	}

	if(ibase_data!=NULL)
	{	
		kfree(ibase_data);
		ibase_data = NULL;
	}
	
#ifdef FEATURE_FPC_OPEN_ENABLE
	if(fpcopen_data!=NULL)
	{	
		kfree(fpcopen_data);
		fpcopen_data = NULL;
	}
#endif ///< for FEATURE_FPC_OPEN_ENABLE
	
#ifdef FEATURE_FPC_OPEN_ENABLE
	if(fpcshort_data!=NULL)
	{	
		kfree(fpcshort_data);
		fpcshort_data = NULL;
	}
#endif ///< for FEATURE_FPC_OPEN_ENABLE
	
	if(tran_data != NULL)	
	{
		kfree(tran_data);
		tran_data = NULL;
	}

	if(tran_tmp_data[0] != NULL)
	{
		kfree(tran_tmp_data[0]);
		tran_tmp_data[0] = NULL;
	}

	if(tran_tmp_data[1] != NULL)
	{
		kfree(tran_tmp_data[1]);
		tran_tmp_data[1] = NULL;
	}

	if(info_data != NULL)	
	{
		kfree(info_data);
		info_data = NULL;
	}

	if(trace_x_data != NULL)	
	{
		kfree(trace_x_data);
		trace_x_data = NULL;
	}

	if(trace_y_data != NULL)	
	{
		kfree(trace_y_data);
		trace_y_data = NULL;
	}

	///---------------------------------------------///
	/// free flash buffer
        ///---------------------------------------------///
	if(flash_buffer!=NULL)
	{
		kfree(flash_buffer);
		flash_buffer = NULL;
	}
}
#ifdef REGISTER_CHRDEV_REGION

/************************************************************************
* Name: zet_rw_iic_drv_open
* Brief: char device open function interface
* Input: node, file point
* Output: no
* Return: 0
***********************************************************************/
static int zet_rw_iic_drv_open(struct inode *inode, struct file *filp)
{
	filp->private_data = zet_rw_i2c_dev_tt;
	return 0;
}

/************************************************************************
* Name: zet_rw_iic_drv_release
* Brief: char device close function interface
* Input: node, file point
* Output: no
* Return: 0
***********************************************************************/
static int zet_rw_iic_drv_release(struct inode *inode, struct file *filp)
{

	return 0;
}

#else  ////< end of REGISTER_CHRDEV_REGION
	
///************************************************************************
///   [function]:  zet_fops_open
///   [parameters]: inode,   file,
///   [return]: u8
///************************************************************************
static int zet_fops_open(struct inode *inode, struct file *file)
{
	int subminor;
	int ret = 0;	
	struct i2c_client *client;
	struct i2c_adapter *adapter;	
	struct i2c_dev *i2c_dev;	
	
	subminor = iminor(inode);
	//printk("[ZET] : ZET_FOPS_OPEN ,  subminor=%d\n",subminor);
	
	i2c_dev = zet622x_i2c_dev_get_by_minor(subminor);	
	if (!i2c_dev)
	{	
		//printk("error i2c_dev\n");		
		return -ENODEV;	
	}
	
	adapter = i2c_get_adapter(i2c_dev->adap->nr);	
	if(!adapter)
	{		
		return -ENODEV;	
	}	
	
	client = kzalloc(sizeof(*client), GFP_KERNEL);	
	
	if(!client)
	{		
		i2c_put_adapter(adapter);		
		ret = -ENOMEM;	
	}	
	snprintf(client->name, I2C_NAME_SIZE, "pctp_i2c_ts%d", adapter->nr);
	//client->driver = &tpd_i2c_driver;
	client->adapter = adapter;		
	file->private_data = client;
		
	return 0;
}
	

///************************************************************************
///   [function]:  zet_fops_release
///   [parameters]: inode,   file,
///   [return]: u8
///************************************************************************
static int zet_fops_release(struct inode *inode, struct file *file)
{
	struct i2c_client *client = file->private_data;

	//printk("[ZET] : zet_fops_release -> line : %d\n",__LINE__ );
	
	i2c_put_adapter(client->adapter);
	kfree(client);
	file->private_data = NULL;
	return 0;	
}

///************************************************************************
///   [function]:  zet_fops_read
///   [parameters]: file, buf, count, ppos
///   [return]: size_t
///************************************************************************
static ssize_t zet_fops_read(struct file *file, char __user *buf, size_t count,
			loff_t *ppos)
{
	int i;
	int iCnt = 0;
	char str[256];
	int len = 0;

	//printk("[ZET] : zet_fops_read -> line : %d\n",__LINE__ );
	
	///-------------------------------///
	/// Print message
	///-------------------------------///	
	sprintf(str, "Please check \"%s\"\n", fw_file_name);
	len = strlen(str);

	///-------------------------------///
	/// if read out
	///-------------------------------///		
	if(data_offset >= len)
	{
		return 0;
        }		
	
	for(i = 0 ; i < count-1 ; i++)
	{
		buf[i] = str[data_offset];
		buf[i+1] = 0;
		iCnt++;
		data_offset++;
		if(data_offset >= len)
		{
			break;
		}
	}	
	
	///-------------------------------///
	/// Save file
	///-------------------------------///	
	if(data_offset == len)
	{
		zet_fw_save(fw_file_name);
	}	
	return iCnt;
}

///************************************************************************
///   [function]:  zet_fops_write
///   [parameters]: file, buf, count, ppos
///   [return]: size_t
///************************************************************************
static ssize_t zet_fops_write(struct file *file, const char __user *buf,
                                                size_t count, loff_t *ppos)
{	
	printk("[ZET]: zet_fops_write ->  %s\n", buf);
	data_offset = 0;
	return count;
}

#endif ////< end of not REGISTER_CHRDEV_REGION

///************************************************************************
///   [function]:  ioctl
///   [parameters]: file , cmd , arg
///   [return]: long
///************************************************************************
static long zet_fops_ioctl(struct file *file, unsigned int cmd, unsigned long arg )
{
    u8 __user * user_buf = (u8 __user *) arg;

	u8 buf[IOCTL_MAX_BUF_SIZE];
	int input_data;
	int data_size;
	char name[64];

#ifdef REGISTER_CHRDEV_REGION
	mutex_lock(&zet_rw_i2c_dev_tt->rw_i2c_mutex);
#endif

	if(copy_from_user(buf, user_buf, IOCTL_MAX_BUF_SIZE))
	{
		printk("[ZET]: zet_ioctl: copy_from_user fail\n");
		return 0;
	}
	printk("[ZET]: zet_ioctl ->  cmd = %d, %02x, %02x\n",  cmd, buf[0], buf[1]);
	if(cmd == ZET_IOCTL_CMD_FLASH_READ)
	{
		printk("[ZET]: zet_ioctl -> ZET_IOCTL_CMD_FLASH_DUMP  cmd = %d, file=%s\n",  cmd, (char *)buf);
		ioctl_action |= IOCTL_ACTION_FLASH_DUMP;
	}
	else if(cmd == ZET_IOCTL_CMD_FLASH_WRITE)
	{
		printk("[ZET]: zet_ioctl -> ZET_IOCTL_CMD_FLASH_WRITE  cmd = %d\n",  cmd);
		resume_download = TRUE;
	        //zet_fw_load(fw_file_name);
			zet622x_resume_downloader(this_client, firmware_upgrade, &rom_type, ic_model);
			
			//zet622x_downloader(this_client, firmware_upgrade,&rom_type,ic_model);
		resume_download = FALSE;
	}
	else if(cmd == ZET_IOCTL_CMD_RST)
	{
		printk("[ZET]: zet_ioctl -> ZET_IOCTL_CMD_RST  cmd = %d\n",  cmd);
		//ctp_reset();
		ctp_set_reset_high();
		ctp_set_reset_low();	
		msleep(20);
		ctp_set_reset_high();
		transfer_type = TRAN_TYPE_DYNAMIC;			
	}
	else if(cmd == ZET_IOCTL_CMD_RST_HIGH)
	{
		ctp_set_reset_high();
	}
	else if(cmd == ZET_IOCTL_CMD_RST_LOW)
	{
		ctp_set_reset_low();	
	}
	else if(cmd == ZET_IOCTL_CMD_GPIO_HIGH)
	{
		input_data = (int)buf[0];
#ifdef FEATRUE_TRACE_GPIO_OUTPUT
		zet622x_ts_gpio_output(input_data, TRUE);
#endif ///< for FEATRUE_TRACE_GPIO_OUTPUT
	}
	else if(cmd == ZET_IOCTL_CMD_GPIO_LOW)
	{
		input_data = (int)buf[0];
#ifdef FEATRUE_TRACE_GPIO_OUTPUT
		zet622x_ts_gpio_output(input_data, FALSE);
#endif ///< for FEATRUE_TRACE_GPIO_OUTPUT
	}
	else if(cmd == ZET_IOCTL_CMD_MDEV)
	{
		///---------------------------------------------------///
		/// set mutual dev mode
		///---------------------------------------------------///
		zet622x_ts_set_transfer_type(TRAN_TYPE_MUTUAL_SCAN_DEV);
		transfer_type = TRAN_TYPE_MUTUAL_SCAN_DEV;				
	}
	else if(cmd == ZET_IOCTL_CMD_IBASE)
	{
		///---------------------------------------------------///
		/// set initial base mode
		///---------------------------------------------------///
		zet622x_ts_set_transfer_type(TRAN_TYPE_INIT_SCAN_BASE);
		transfer_type = TRAN_TYPE_INIT_SCAN_BASE;	
	}	
#ifdef FEATURE_IDEV_OUT_ENABLE 
	else if(cmd == ZET_IOCTL_CMD_IDEV)
	{
		///---------------------------------------------------///
		/// set initial dev mode
		///---------------------------------------------------///
		zet622x_ts_set_transfer_type(TRAN_TYPE_INIT_SCAN_DEV);
		transfer_type = TRAN_TYPE_INIT_SCAN_DEV;	
	}
#endif ///< 	FEATURE_IDEV_OUT_ENABLE
#ifdef FEATURE_MBASE_OUT_ENABLE
	else if(cmd == ZET_IOCTL_CMD_MBASE)
	{
		///---------------------------------------------------///
		/// set Mutual Base mode
		///---------------------------------------------------///
		zet622x_ts_set_transfer_type(TRAN_TYPE_MUTUAL_SCAN_BASE);
		transfer_type = TRAN_TYPE_MUTUAL_SCAN_BASE;	
	}
#endif ///< FEATURE_MBASE_OUT_ENABLE
 	else if(cmd == ZET_IOCTL_CMD_DYNAMIC)
	{
		zet622x_ts_set_transfer_type(TRAN_TYPE_DYNAMIC);
		transfer_type = TRAN_TYPE_DYNAMIC;
	}
	else if(cmd == ZET_IOCTL_CMD_FW_FILE_PATH_GET)
	{
		memset(buf, 0x00, 64);
		strcpy(buf, fw_file_name);		
		printk("[ZET]: zet_ioctl: Get FW_FILE_NAME = %s\n", buf);
	}
	else if(cmd == ZET_IOCTL_CMD_FW_FILE_PATH_SET)
	{
		strcpy(fw_file_name, buf);		
		printk("[ZET]: zet_ioctl: set FW_FILE_NAME = %s\n", buf);
	}
	else if(cmd == ZET_IOCTL_CMD_MDEV_GET)
	{
		data_size = (row+2)*(col+2);
		memcpy(buf, mdev_data, data_size);
              	printk("[ZET]: zet_ioctl: Get MDEV data size=%d\n", data_size);
       	 }
	else if(cmd == ZET_IOCTL_CMD_TRAN_TYPE_PATH_SET)
	{
		strcpy(tran_type_mode_file_name, buf);		
		printk("[ZET]: zet_ioctl: Set ZET_IOCTL_CMD_TRAN_TYPE_PATH_ = %s\n", buf);
	}
	else if(cmd == ZET_IOCTL_CMD_TRAN_TYPE_PATH_GET)
	{
		memset(buf, 0x00, 64);
		strcpy(buf, tran_type_mode_file_name);	
		printk("[ZET]: zet_ioctl: Get ZET_IOCTL_CMD_TRAN_TYPE_PATH = %s\n", buf);
	}
	else if(cmd == ZET_IOCTL_CMD_IDEV_GET)
	{
		data_size = (row + col);
		memcpy(buf, idev_data, data_size);
		printk("[ZET]: zet_ioctl: Get IDEV data size=%d\n", data_size);
	}
	else if(cmd == ZET_IOCTL_CMD_IBASE_GET)
	{
		data_size = (row + col)*2;
		memcpy(buf, ibase_data, data_size);
		printk("[ZET]: zet_ioctl: Get IBASE data size=%d\n", data_size);
	}	
	else if(cmd == ZET_IOCTL_CMD_MBASE_GET)
	{
		data_size = (row*col*2);
		if(data_size > IOCTL_MAX_BUF_SIZE)
		{
			data_size = IOCTL_MAX_BUF_SIZE;
		}
		memcpy(buf, mbase_data, data_size);
		printk("[ZET]: zet_ioctl: Get MBASE data size=%d\n", data_size);
	}
	else if(cmd == ZET_IOCTL_CMD_INFO_SET)
	{
		i_copy_to_report_data = 1;
		printk("[ZET]: zet_ioctl: ZET_IOCTL_CMD_INFO_SET\n");
		zet622x_ts_set_info_type();
	}
	else if(cmd == ZET_IOCTL_CMD_INFO_GET)
	{
		data_size = INFO_DATA_SIZE;
 #ifdef FEATURE_INFO_OUT_EANBLE
		memcpy(buf, info_data, data_size);
		printk("[ZET]: zet_ioctl: Get INFO data size=%d,IC: %x,X:%d,Y:%d,DEV_TYPE:%d\n", data_size, info_data[0], info_data[13], info_data[14], info_data[3]);
#endif ///< for FEATURE_INFO_OUT_EANBLE		
  	}
	else if(cmd == ZET_IOCTL_CMD_TRACE_X_NAME_SET)
	{
 #ifdef FEATURE_INFO_OUT_EANBLE
 		zet622x_ts_set_trace_x_type();
  #endif ///< for FEATURE_INFO_OUT_EANBLE		
	}
	else if(cmd == ZET_IOCTL_CMD_TRACE_X_NAME_GET)
	{
		data_size = col;
 #ifdef FEATURE_INFO_OUT_EANBLE
		memcpy(buf, trace_x_data, data_size);
  #endif ///< for FEATURE_INFO_OUT_EANBLE	
	}
	else if(cmd == ZET_IOCTL_CMD_TRACE_Y_NAME_SET)
	{
 #ifdef FEATURE_INFO_OUT_EANBLE
 		zet622x_ts_set_trace_y_type();
  #endif ///< for FEATURE_INFO_OUT_EANBLE		
	}
	else if(cmd == ZET_IOCTL_CMD_TRACE_Y_NAME_GET)
	{
		data_size = row;
 #ifdef FEATURE_INFO_OUT_EANBLE
		memcpy(buf, trace_y_data, data_size);
  #endif ///< for FEATURE_INFO_OUT_EANBLE	
	}
	else if(cmd == ZET_IOCTL_CMD_TRACE_X_SET)
	{
		printk("[ZET]: zet_ioctl: ZET_IOCTL_CMD_TRACE_X_SET\n");
		row = (int)(*buf);
	}
	//else if(cmd == ZET_IOCTL_CMD_WRITE_CMD)
	//{
	//	zet622x_cmd_ioctl_write_data(this_client, buf[0], &buf[1]);
	//}
	else if(cmd == ZET_IOCTL_CMD_TRACE_X_GET)
	{
		printk("[ZET]: zet_ioctl: Get TRACEX data\n");
		memset(buf, 0x00, 64);
		data_size = sizeof(int);
		memcpy(buf, &row, data_size);
  	
	}
	else if(cmd == ZET_IOCTL_CMD_TRACE_Y_SET)
	{
		printk("[ZET]: zet_ioctl: ZET_IOCTL_CMD_TRACE_Y_SET\n");
		col = (int)(*buf);
	}
	else if(cmd == ZET_IOCTL_CMD_TRACE_Y_GET)
	{
		printk("[ZET]: zet_ioctl: Get TRACEY data \n");
		memset(buf, 0x00, 64);
		data_size = sizeof(int);
		memcpy(buf, &col, data_size);
	}
	else if(cmd == ZET_IOCTL_CMD_DRIVER_VER_GET)
	{
		memset(buf, 0x00, 64);
		strcpy(buf, driver_version);		
		printk("[ZET]: zet_ioctl: Get DRIVER_VERSION = %s\n", buf);
		printk("[ZET]: zet_ioctl: Get SVN = %s\n", DRIVER_VERSION);
	}
	else if(cmd == ZET_IOCTL_CMD_SENID_GET)
	{
		memset(buf, 0x00, 64);
		
        #ifdef FEATRUE_TRACE_SENSOR_ID
		buf[0] = sensor_id_status;
		buf[1] = sensor_id;
		printk("[ZET]: zet_ioctl: Get ZET_IOCTL_CMD_SENID_GET = %d/ %d\n", sensor_id_status, sensor_id);
        #else ///< for FEATRUE_TRACE_SENSOR_ID
		memset(buf, 0xFF, 64);
        #endif ///< for FEATRUE_TRACE_SENSOR_ID
	}
	else if(cmd == ZET_IOCTL_CMD_FINGER_XMAX_YMAX_GET)
	{
		memset(buf, 0x00, 64);
		buf[0] = FINGER_NUMBER;
		buf[1] = (u8)X_MAX;
		buf[2] = (u8)X_MAX >> 8;
		buf[3] = (u8)Y_MAX;	
		buf[4] = (u8)Y_MAX >> 8;
		printk("[ZET]: zet_ioctl: Get ZET_IOCTL_CMD_FINGER_XMAX_YMAX_GET \n  = %d %d %d %d %d \n", buf[0],buf[1],buf[2],buf[3],buf[4]);	
	}
	else if(cmd == ZET_IOCTL_CMD_PCODE_GET)
	{
		memset(buf, 0x00, 64);
		sprintf(name, "%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X", 
                       pcode[0], pcode[1], pcode[2], pcode[3], 
                       pcode[4], pcode[5], pcode[6], pcode[7] );
		zet_fw_set_pcode_name(name);
		strcpy(buf, pcode_version);	
		printk("[ZET]: zet_ioctl: Get ZET_IOCTL_CMD_PCODE_GET = %s\n", buf);
	}
	else if(cmd == ZET_IOCTL_CMD_MBASE_EXTERN_GET)
	{
		data_size = (row*col*2) - IOCTL_MAX_BUF_SIZE;
		if(data_size < 1)
		{
			data_size = 1;
		}
		memcpy(buf, (mbase_data+IOCTL_MAX_BUF_SIZE), data_size);
		printk("[ZET]: zet_ioctl: Get MBASE extern data size=%d\n", data_size);
	}
	else if(cmd == ZET_IOCTL_CMD_FRAM_RATE)
	{
#ifdef FEATURE_FRAM_RATE
		memset(buf, 0x00, 64);
		data_size = sizeof(int);
		memcpy(buf, &last_fram_rate, data_size);
#else
		memset(buf,0x00,64);
#endif ///< for FEATURE_FRAM_RATE
	}
	else if(cmd == ZET_IOCTL_CMD_FPC_OPEN_GET)
	{
#ifdef FEATURE_FPC_OPEN_ENABLE	
		data_size = (row + col);
		memcpy(buf, fpcopen_data, data_size);
		printk("[ZET]: zet_ioctl: Get IDEV data size=%d\n", data_size);
#endif ///< for FEATURE_FPC_OPEN_ENABLE
	}
	else if(cmd == ZET_IOCTL_CMD_FPC_SHORT_GET)
	{
#ifdef FEATURE_FPC_SHORT_ENABLE
		data_size = (row + col)*2;
		memcpy(buf, fpcshort_data, data_size);
		printk("[ZET]: zet_ioctl: Get IBASE data size=%d\n", data_size);
#endif ///< for FEATURE_FPC_SHORT_ENABLE
	}
	else if(cmd == ZET_IOCTL_CMD_FPC_SHORT_SET)
	{
#ifdef FEATURE_FPC_SHORT_ENABLE
		buf[0] = FPC_SHORT_CMD_LEN;
		buf[1] = FPC_SHORT_CMD;
		zet622x_cmd_ioctl_write_data(this_client, buf[0], &buf[1]);
		transfer_type = TRAN_TYPE_FPC_SHORT;
#endif ///< for FEATURE_FPC_SHORT_ENABLE
	}
	else if(cmd == ZET_IOCTL_CMD_FPC_OPEN_SET)
	{
#ifdef FEATURE_FPC_OPEN_ENABLE	
		buf[0] = FPC_OPEN_CMD_LEN;
		buf[1] = FPC_OPEN_CMD;
		zet622x_cmd_ioctl_write_data(this_client, buf[0], &buf[1]);
		transfer_type = TRAN_TYPE_FPC_OPEN;
#endif ///< for FEATURE_FPC_OPEN_ENABLE
	}
	else if(cmd == ZET_IOCTL_CMD_WRITE_CMD)
	{
		transfer_type = tran_type_setting;
		zet622x_cmd_ioctl_write_data(this_client, buf[0], &buf[1]);
	}
	else if(cmd == ZET_IOCTL_CMD_GET_REPORT_DATA_1)
	{
		data_size = tran_data_size;
		if(data_size > IOCTL_MAX_BUF_SIZE)
		{
			data_size = IOCTL_MAX_BUF_SIZE;
		}	
		if(tran_report_point != NULL)
		{
			memcpy(buf, tran_report_point, data_size);
		}
		else
		{
			memset(buf, 0 , IOCTL_MAX_BUF_SIZE);
		}
	}
	else if(cmd == ZET_IOCTL_CMD_GET_REPORT_DATA_2)
	{	
		data_size = tran_data_size - IOCTL_MAX_BUF_SIZE;
		if(data_size < 1)
		{
			data_size = 1;
		}	
		if(tran_report_point != NULL)
		{
			memcpy(buf, (tran_report_point+IOCTL_MAX_BUF_SIZE), data_size);
		}
		else
		{
			memset(buf, 0 , 255);
		}
	}
	else if(cmd == ZET_IOCTL_CMD_REPORT_SIZE_GET)
	{
		memset(buf, 0x00, 64);
		data_size = sizeof(int);
		memcpy(buf, &tran_data_size, data_size);
	}
	else if(cmd == ZET_IOCTL_CMD_REPORT_SIZE_SET)
	{
		data_size = sizeof(int);
		memcpy(&tran_data_size, buf, data_size);
		printk("[ZET]: zet_ioctl: Set report data size=%d\n", tran_data_size);
	}
	else if(cmd == ZET_IOCTL_CMD_TRAN_TYPE_SET)
	{
		data_size = sizeof(int);
		memcpy(&tran_type_setting, buf, data_size);
		printk("[ZET]: zet_ioctl: Set tran type data size=%d\n", tran_type_setting);
	}
	if(copy_to_user(user_buf, buf, IOCTL_MAX_BUF_SIZE))
	{
		printk("[ZET]: zet_ioctl: copy_to_user fail\n");
	}
	
#ifdef REGISTER_CHRDEV_REGION
	mutex_unlock(&zet_rw_i2c_dev_tt->rw_i2c_mutex);
#endif	
	
	return 0;
}

///************************************************************************
///	character device
///************************************************************************
#ifdef REGISTER_CHRDEV_REGION

static const struct file_operations zet_rw_iic_drv_fops = {
	.owner			= THIS_MODULE,
	.open			= zet_rw_iic_drv_open,
	.release			= zet_rw_iic_drv_release,
	.unlocked_ioctl	= zet_fops_ioctl,
	//.compat_ioctl	= zet_fops_ioctl,
};

#else

static struct file_operations zet622x_ts_fops = {
	.owner		= THIS_MODULE, 	
	.open 		= zet_fops_open,
	.read 		= zet_fops_read, 	
	.write		= zet_fops_write, 
	.unlocked_ioctl = zet_fops_ioctl,
	.compat_ioctl	= zet_fops_ioctl,
	.release 	= zet_fops_release
};

#endif

////< CTL_IIC & APK_DEBUG begin
#ifdef REGISTER_CHRDEV_REGION

/************************************************************************
* Name: zet_rw_iic_drv_setup_cdev
* Brief: setup char device 
* Input: device point, index number
* Output: no
* Return: no
***********************************************************************/
static void zet_rw_iic_drv_setup_cdev(struct zet_rw_i2c_dev *dev, int index)
{
	int err, devno = MKDEV(zet_rw_iic_drv_major, index);

	cdev_init(&dev->cdev, &zet_rw_iic_drv_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &zet_rw_iic_drv_fops;
	err = cdev_add(&dev->cdev, devno, 1);
	if (err)
		printk(KERN_NOTICE "Error %d adding LED%d", err, index);
}

/************************************************************************
* Name: zet_rw_iic_drv_myinitdev
* Brief: initial char device 
* Input: i2c info
* Output: no
* Return: fail <0
***********************************************************************/
static int zet_rw_iic_drv_myinitdev(struct i2c_client *client)
{
	int err = 0;
	dev_t devno = MKDEV(zet_rw_iic_drv_major, 0);

	if (zet_rw_iic_drv_major)
		err = register_chrdev_region(devno, 1, ZET_RW_IIC_DRV);
	else 
	{
		err = alloc_chrdev_region(&devno, 0, 1, ZET_RW_IIC_DRV);
		zet_rw_iic_drv_major = MAJOR(devno);
	}
	if (err < 0) 
	{
		dev_err(&client->dev, "%s:zet_rw_iic_drv failed  error code=%d---\n",
				__func__, err);
		return err;
	}

	zet_rw_i2c_dev_tt = kmalloc(sizeof(struct zet_rw_i2c_dev), GFP_KERNEL);
	if (!zet_rw_i2c_dev_tt)
	{
		err = -ENOMEM;
		unregister_chrdev_region(devno, 1);
		dev_err(&client->dev, "%s:zet_rw_iic_drv failed\n",
				__func__);
		return err;
	}
	zet_rw_i2c_dev_tt->client = client;
	mutex_init(&zet_rw_i2c_dev_tt->rw_i2c_mutex);
	zet_rw_iic_drv_setup_cdev(zet_rw_i2c_dev_tt, 0); 

	i2c_dev_class = class_create(THIS_MODULE, "zet_class");
	if (IS_ERR(i2c_dev_class)) 
	{
		dev_err(&client->dev, "%s:failed in creating class.\n",
				__func__);
		return -1; 
	} 
	/*create device node*/
	device_create(i2c_dev_class, NULL, MKDEV(zet_rw_iic_drv_major, 0), 
			NULL, ZET_RW_IIC_DRV);

	return 0;
}

/************************************************************************
* Name: zet_rw_iic_drv_init
* Brief: call initial char device 
* Input: i2c info
* Output: no
* Return: fail <0
***********************************************************************/
int zet_rw_iic_drv_init(struct i2c_client *client)
{
    dev_dbg(&client->dev, "[ZET]----zet_rw_iic_drv init ---\n");
	return zet_rw_iic_drv_myinitdev(client);

#endif
////< CTL_IIC & APK_DEBUG end

///************************************************************************
///   [function]:  zet622x_ts_remove
///   [parameters]: client
///   [return]:
///************************************************************************
static int zet622x_ts_remove(struct i2c_client *dev)
{

////////@zet.Begin		
	//struct zet622x_tsdrv *zet6221_ts = i2c_get_clientdata(dev);
	//kfree(zet6221_ts);   
	//i2c_set_clientdata(dev, NULL);
////////@zet.End	

	TPD_DEBUG("TPD removed\n");
	
//#ifdef CONFIG_MTK_I2C_EXTENSION
	msg_dma_release();
//#endif

	/// release the buffer
	zet_fw_exit();

	return 0;

}

///************************************************************************
///   [function]:  zet622x_ts_probe
///   [parameters]: client , id
///   [return]:
///************************************************************************
static int  __init zet622x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{             
	int err = 0;
	struct i2c_dev *i2c_dev;
	struct device *dev;
	struct zet6221_tsdrv *zet6221_ts;
	char name[64];

	printk("[ZET]: Probe Zet62xx\n");

#ifdef TPD_PROXIMITY
    struct hwmsen_object obj_ps;
#endif	

	///------------------------------------------------///
	/// setting the charger mode event handler
	///------------------------------------------------///
        
	this_client = client;
	zet_i2c_client = client;
	zet_input_dev=tpd->dev;	

	if(this_client->addr != 0x76)
	{
		this_client->addr = 0x76;
		printk("To modify ZET i2c this_client->addr=%d\n",this_client->addr);
	}	
	
	TPD_DMESG("mtk_tpd: tpd_probe zet62xx\n");
	
	err = regulator_enable(tpd->reg);
	if (err != 0)
		TPD_DMESG("Failed to enable reg-vgp6: %d\n", err);

	/* Reset CTP */
	tpd_gpio_output(GTP_RST_PORT, 1);
	msleep(1);
	tpd_gpio_output(GTP_RST_PORT, 0);
	msleep(1);
	tpd_gpio_output(GTP_RST_PORT, 1);
	msleep(50);
	TPD_DMESG("mtk_tpd: tpd_probe zet62xx HW reset\n");	

	/* set INT mode */
    tpd_gpio_as_int(GTP_INT_PORT);
	//gpio_direction_input(tpd_int_gpio_number);

	tpd_irq_registration();
	msleep(100);

#ifndef FEATURE_MTK_I2C_8BYTES_MODE
	this_client->timing = 400;
	msg_dma_alloct();
#endif ///< for not FEATURE_MTK_I2C_8BYTES_MODE
        
    zet6221_ts = kzalloc(sizeof(struct zet6221_tsdrv), GFP_KERNEL);	

	///------------------------------------------------///
	/// init the finger pressed data
	///------------------------------------------------///
#ifdef FEATURE_LIGHT_LOAD_REPORT_MODE
	zet62xx_ts_init();
#endif  ///< for FEATURE_LIGHT_LOAD_REPORT_MODE

	///-----------------------------------------------///
	/// Set the default firmware bin file name & mutual dev file name
	///-----------------------------------------------///
	zet_dv_set_file_name(DRIVER_VERSION);
	zet_fw_set_file_name(FW_FILE_NAME);
	zet_tran_type_set_file_name(TRAN_MODE_FILE_PATH);

	///------------------------------------------------///
	/// init the memory
	///------------------------------------------------///
	zet_mem_init();
#if 0 //FEATURE_FW_UPGRADE
	///------------------------------------------------///
	/// check the FW version and download
	///------------------------------------------------///
	if(zet622x_downloader(client, firmware_upgrade,&rom_type,ic_model) <= 0)
	{
		//printk("[ZET] FW upgrade failed!\n");
	}
	else
	{
		//printk("[ZET] FW upgrade successed!\n");
	}
#endif ///< for FEATURE_FW_UPGRADE

	///------------------------------------------------///
	/// RESET the IC
	///------------------------------------------------///
	ctp_set_reset_high();  
	msleep(1);  
	ctp_set_reset_low();
	msleep(1);  
	ctp_set_reset_high();
	msleep(50);
	
#if defined(FEATURE_TPINFO)
	///------------------------------------------------///
	/// Send B2 command then to get TP information
	///------------------------------------------------///

	msleep(3);  	
	
	if(zet622x_ts_get_information(client)<=0)
	{
		return err;
	}
		
	if(key_num>0)
	{
		input_set_abs_params(tpd->dev, ABS_MT_POSITION_X, 0, resolution_x, 0, 0);
		input_set_abs_params(tpd->dev, ABS_MT_POSITION_Y, 0, resolution_y, 0, 0);
	}
	else
	{
#ifndef FEATURE_VIRTUAL_KEY   
		input_set_abs_params(tpd->dev, ABS_MT_POSITION_X, 0, resolution_x, 0, 0);
		input_set_abs_params(tpd->dev, ABS_MT_POSITION_Y, 0, resolution_y, 0, 0);	
#endif ///< for FEATURE_VIRTUAL_KEY
	}

	
#else ///< for FEATURE_TPINFO
	///------------------------------------------------///
	/// not send B2 command and Set TP information
	///------------------------------------------------///

	resolution_x = X_MAX;
	resolution_y = Y_MAX;
	finger_num = FINGER_NUMBER;
	key_num = KEY_NUMBER;   
	if(key_num == 0)
	{
		finger_packet_size  = 3+TPD_POINT_INFO_LEN*TPD_MAX_POINTS;
	}
	else
	{
		finger_packet_size  = 3+TPD_POINT_INFO_LEN*TPD_MAX_POINTS+1;
	}
	
#endif ///< for FEATURE_TPINFO
	sprintf(name, "%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X", 
                       pcode[0], pcode[1], pcode[2], pcode[3], 
                       pcode[4], pcode[5], pcode[6], pcode[7] );
	zet_fw_set_pcode_name(name);

	printk( "[ZET6221] TP resolution_x=%d resolution_y=%d finger_num=%d key_num=%d\n",resolution_x,resolution_y,finger_num,key_num);

#ifdef FEATURE_MT_TYPE_B
        #ifdef FEATURE_KEY_TO_FINGER
	        input_mt_init_slots(tpd->dev, finger_num + 1, 0);	
        #else ///< for FEATURE_KEY_TO_FINGER
	        input_mt_init_slots(tpd->dev, finger_num, 0);	
        #endif ///<  for FEATURE_KEY_TO_FINGER
#endif ///< for FEATURE_MT_TYPE_B 
	
#ifdef GESTURE_WAKEUP
    input_set_capability(tpd->dev, EV_KEY, KEY_RIGHT);
    input_set_capability(tpd->dev, EV_KEY, KEY_UP);
    input_set_capability(tpd->dev, EV_KEY, KEY_DOWN);	
    input_set_capability(tpd->dev, EV_KEY, KEY_LEFT);  
    input_set_capability(tpd->dev, EV_KEY, KEY_C);
    input_set_capability(tpd->dev, EV_KEY, KEY_W);
    input_set_capability(tpd->dev, EV_KEY, KEY_M);
    input_set_capability(tpd->dev, EV_KEY, KEY_S);
    input_set_capability(tpd->dev, EV_KEY, KEY_E);
    input_set_capability(tpd->dev, EV_KEY, KEY_O);
    input_set_capability(tpd->dev, EV_KEY, KEY_V);
    input_set_capability(tpd->dev, EV_KEY, 0X5E);
    input_set_capability(tpd->dev, EV_KEY, KEY_Z);	
//    input_set_capability(tpd->dev, EV_KEY, KEY_POWER);
#endif	

	tpd_load_status = 1;
	///------------------------------------------------///
	/// setting the INT pin event handler
	///------------------------------------------------///

#if 0
	thread_tpd = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread_tpd)) {
		retval = PTR_ERR(thread_tpd);
		TPD_DMESG(TPD_DEVICE " failed to create kernel thread_tpd: %d\n", retval);
	}

	TPD_DMESG("[ZET] Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");	
#endif
	
#ifndef FEATURE_INT_FREE
	thread = kthread_run(zet622x_ts_work, 0, TPD_DEVICE);
	if (IS_ERR(thread))
	{ 
		err = PTR_ERR(thread);
		TPD_DMESG(TPD_DEVICE " failed to create kernel thread_tpd: %d\n", err);
	}
	
	TPD_DMESG("[ZET] Touch Panel Device Probe %s\n", (err < TPD_OK) ? "FAIL" : "PASS");
#else  ///< for not FEATURE_INT_FREE

	INIT_WORK(&zet6221_ts->work2, zet622x_ts_polling_work);
	zet6221_ts->ts_workqueue2 = create_singlethread_workqueue(dev_name(&client->dev)); //  workqueue
	if (!zet6221_ts->ts_workqueue2)
	{
		//printk("[ZET] : ts_workqueue1 ts_probe error ==========\n");
		return 0;
	}

	///------------------------------------------///
	/// Set polling timer
	///------------------------------------------///
	setup_timer(&zet6221_ts->zet622x_ts_timer_task1, zet622x_ts_polling_task, (unsigned long)zet6221_ts);
	mod_timer(&zet6221_ts->zet622x_ts_timer_task1, jiffies + msecs_to_jiffies(800));
#endif ///< for FEATURE_INT_FREE

#ifdef FEATURE_CHARGER_MODE
	/*   charger detect : write_cmd */
	INIT_WORK(&zet6221_ts->work1, zet622x_charger_cmd_work);
	zet6221_ts->ts_workqueue1 = create_singlethread_workqueue(dev_name(&client->dev)); //  workqueue
	if (!zet6221_ts->ts_workqueue1)
	{
		//printk("[ZET] : ts_workqueue1 ts_probe error ==========\n");
		return 0;
	}
	/*   charger detect : write_cmd */

	setup_timer(&zet6221_ts->zet622x_ts_timer_task, zet622x_ts_timer_task, (unsigned long)zet6221_ts);
	mod_timer(&zet6221_ts->zet622x_ts_timer_task,jiffies + msecs_to_jiffies(800));

#endif ///< for FEATURE_CHARGER_MODE

#ifdef TPD_PROXIMITY
    //obj_ps.self = cm3623_obj;
    obj_ps.polling = 1;         //0--interrupt mode;1--polling mode;
    obj_ps.sensor_operate = psensor_ps_operate;

    if ((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
    {
        //printk("hwmsen attach fail, return:%d.\n", err);
    }

#endif

	
#ifdef FEATURE_FRAM_RATE
	///------------------------------------------///
	/// Set fram rate timer
	///------------------------------------------///
	setup_timer(&zet6221_ts->zet622x_ts_timer_task2, zet622x_ts_fram_rate_task, (unsigned long)zet6221_ts);
	mod_timer(&zet6221_ts->zet622x_ts_timer_task2, jiffies + msecs_to_jiffies(FRAM_RATE_TIMER));
#endif ///< for FEATURE_FRAM_RATE

	///--------------------------------------------///
	/// Get a free i2c dev
	///--------------------------------------------///
	
#ifdef REGISTER_CHRDEV_REGION

	if (zet_rw_iic_drv_init(this_client) < 0)
		dev_err(&client->dev, "%s:[ZET] create zet control iic driver failed\n", __func__);

	//zet_create_apk_debug_channel(this_client);

#else
	
	i2c_dev = zet622x_i2c_get_free_dev(client->adapter);	
	if(IS_ERR(i2c_dev))
	{	
		err = PTR_ERR(i2c_dev);		
		return err;	
	}
	dev = device_create(i2c_dev_class, &client->adapter->dev, 
				MKDEV(I2C_MAJOR,client->adapter->nr), NULL, "zet62xx_ts%d", client->adapter->nr);	
	if(IS_ERR(dev))
	{		
		err = PTR_ERR(dev);		
		return err;	
	}
	
#endif
	
	printk("[ZET] : zet6221 zet622x_ts_probe Finish\n");

	return 0;
}

static struct device_attribute *zet62xx_attrs[] = {
//#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
//	&dev_attr_tpd_scp_ctrl,
//#endif
};

///************************************************************************
///   [function]:  zet622x_ts_local_init
///   [parameters]: void
///   [return]: int
///************************************************************************
static int zet622x_ts_local_init(void) 
{
	int retval;

	TPD_DMESG("ZET I2C Touchscreen Driver...\n");
	tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
	retval = regulator_set_voltage(tpd->reg, 2800000, 2800000);
	if (retval != 0) {
		TPD_DMESG("Failed to set reg-vgp6 voltage: %d\n", retval);
      	return -1;
    }
	if (i2c_add_driver(&tpd_i2c_driver) != 0) {
		TPD_DMESG("unable to add i2c driver.\n");
    	return -1;
    }

#ifdef TPD_HAVE_BUTTON
	tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);
#else
	if (tpd_dts_data.use_tpd_button) {
		tpd_button_setting(tpd_dts_data.tpd_key_num, tpd_dts_data.tpd_key_local,
		tpd_dts_data.tpd_key_dim_local);
	}
#endif
	 
	#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))    
    		TPD_DO_WARP = 1;
    		memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
    		memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
	#endif 

#if (defined(CONFIG_TPD_HAVE_CALIBRATION) && !defined(CONFIG_TPD_CUSTOM_CALIBRATION))
	memcpy(tpd_calmat, tpd_def_calmat_local_factory, 8 * 4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local_factory, 8 * 4);

	memcpy(tpd_calmat, tpd_def_calmat_local_normal, 8 * 4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local_normal, 8 * 4);
	#endif  
	
	TPD_DMESG("end %s, %d\n", __func__, __LINE__);
	tpd_type_cap = 1;

    return 0; 	
}

///************************************************************************
///   [function]:  zet622x_ts_late_resume
///   [parameters]: inode,   file,
///   [return]: u8
///************************************************************************
static void zet622x_ts_late_resume(  struct device *h  )
{

	int retval = TPD_OK;
	charger_status = FALSE;
	
	i_copy_to_report_data = 0;
	
	TPD_DEBUG("[ZET] TPD wake up\n");

	retval = regulator_enable(tpd->reg);
	if (retval != 0)
		TPD_DMESG("Failed to enable reg-vgp6: %d\n", retval);	

#ifdef TPD_PROXIMITY
    if(0==tpd_in_suspend)
    	return;
#endif	

	ctp_set_reset_high();
	msleep(1);  
	ctp_set_reset_low();
	msleep(1);  
	ctp_set_reset_high();
	msleep(20);  

#if 0 //FEATURE_FW_UPGRADE
	#ifdef FEATURE_FW_UPGRADE_RESUME
	///---------------------------------------------///
	/// resume download 
	///--------------------------------------------///
	resume_download_task = kthread_create(zet622x_resume_download_thread, NULL, "resume_download");
	if(IS_ERR(resume_download_task))
	{
		////printk(KERN_ERR "[ZET] : %s: cread thread failed\n",__FILE__);	
	}
  	wake_up_process(resume_download_task); 
	#endif ///< for FEATURE_FW_UPGRADE_RESUME
#endif ///< for TURE_FW_UPGRADE

	///------------------------------------------------///
	/// init the finger pressed data
	///------------------------------------------------///
#ifdef FEATURE_LIGHT_LOAD_REPORT_MODE
	zet62xx_ts_init();
#endif ///< for FEATURE_LIGHT_LOAD_REPORT_MODE

#ifdef GESTURE_WAKEUP
	doze_status = DOZE_DISABLED;
#else
	//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
#endif
	
	suspend_mode = FALSE;

	enable_irq(touch_irq);
	
#ifdef TPD_PROXIMITY
	  tpd_in_suspend=0;
#endif		

	
  ///--------------------------------------///
  /// Set transfer type to dynamic mode
  ///--------------------------------------///
	transfer_type = TRAN_TYPE_DYNAMIC;
	
}

///************************************************************************
///   [function]:  zet622x_ts_early_suspend
///   [parameters]: early_suspend
///   [return]: int
///************************************************************************
static void zet622x_ts_early_suspend(  struct device *h  )
{
	u8 data = 0xb1;	
	int retval = TPD_OK;
	
	i_copy_to_report_data = 0;
	
#ifdef TPD_PROXIMITY
    if (tpd_proximity_flag == 1)
    {
        return ;
    }
#endif	

#ifdef GESTURE_WAKEUP
	suspend_mode = TRUE;
	zet_enter_doze();
	zet622x_ts_clean_finger();
#else

	suspend_mode = TRUE;
	zet622x_i2c_write_tsdata(this_client, &data, 1);
	zet622x_ts_clean_finger();

	disable_irq(touch_irq);	
	retval = regulator_disable(tpd->reg);
	if (retval != 0)
		TPD_DMESG("[ZET] Failed to disable reg-vgp6: %d\n", retval);	

	//mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
#endif

#ifdef TPD_PROXIMITY
	tpd_in_suspend = 1;
#endif
	
}

///************************************************************************
///	generic driver 
///************************************************************************
static struct tpd_driver_t tpd_device_driver =
{
    .tpd_device_name = ZET_TS_ID_NAME,
    .tpd_local_init = zet622x_ts_local_init,
    .suspend = zet622x_ts_early_suspend,
    .resume = zet622x_ts_late_resume,
	.attrs = {
		.attr = zet62xx_attrs,
		.num  = ARRAY_SIZE(zet62xx_attrs),
	},
};

///************************************************************************
///   [function]:  zet622x_module_init
///   [parameters]: 
///   [return]:
///************************************************************************
static int __init zet622x_module_init(void)
{
	int ret;
	
	TPD_DMESG("[ZET] MediaTek zet touch panel driver init\n");
	tpd_get_dts_info();
#ifdef TPD_HAVE_BUTTON
	tpd_dts_data.use_tpd_button = 1;
#else
	tpd_dts_data.use_tpd_button = 0;
#endif
	if (tpd_driver_add(&tpd_device_driver) < 0)
		TPD_DMESG("add ZET driver failed\n");

////////@zet.Begin
	
#ifndef REGISTER_CHRDEV_REGION	
	major = register_chrdev(I2C_MAJOR, ZET_TS_ID_NAME, &zet622x_ts_fops);	
	if (major < 0) 
	{
		printk("[ZET] : Registering the character device failed with %d\n", major);
	}	
	///---------------------------------///
	/// Create device class
	///---------------------------------///
	i2c_dev_class = class_create(THIS_MODULE,"zet_i2c_dev");
	if(IS_ERR(i2c_dev_class))
	{		
		ret = PTR_ERR(i2c_dev_class);		
		class_destroy(i2c_dev_class);	
	}
#endif
////////@zet.End
	
	return 0;	
}

///************************************************************************
///   [function]:  zet622x_module_exit
///   [parameters]: 
///   [return]:
///************************************************************************
static void __exit zet622x_module_exit(void)
{
    printk("[ZET] MediaTek zet touch panel driver exit\n");
	tpd_driver_remove(&tpd_device_driver);

////////@zet.Begin	
#ifndef REGISTER_CHRDEV_REGION	
	unregister_chrdev(major, ZET_TS_ID_NAME);
#endif	
////////@zet.End
}

module_init(zet622x_module_init);
module_exit(zet622x_module_exit);
//MODULE_DESCRIPTION("ZET622X I2C Touch Screen driver");
//MODULE_LICENSE("GPL v2");

