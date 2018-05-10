/* Copyright (c) 2009-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 * this needs to be before <linux/kernel.h> is loaded,
 * and <linux/sched.h> loads <linux/kernel.h>
 */
#define DEBUG 1

#include <linux/slab.h>
#include <linux/earlysuspend.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <asm/atomic.h>

#include <mach/msm_rpcrouter.h>
#include <mach/msm_battery.h>
#ifdef CONFIG_MSM_SM_EVENT
#include <linux/sm_event_log.h>
#include <linux/sm_event.h>
#endif

#if defined(CONFIG_MSM8X25_B962_EMMC) || defined(CONFIG_MSM8X25_T049) || defined(CONFIG_MSM8X25_B708)
#ifndef CHARGER_ONLY_CASE
#define CHARGER_ONLY_CASE
#endif
#endif

#if defined(CONFIG_MSM8X25_B962) || defined(CONFIG_MSM8X25_B902) ||defined(CONFIG_MSM8X25_T049) || defined(CONFIG_MSM8X25_B708)
#ifndef AP_CAL_BATT_CAP
#define AP_CAL_BATT_CAP
#endif
#else
#ifdef AP_CAL_BATT_CAP
#undef AP_CAL_BATT_CAP
#endif
#endif

#if defined(CHARGER_ONLY_CASE)
#include <mach/socinfo.h>
extern void mdp_clk_check(int on);
extern uint8_t current_boot_mode;
//static struct wake_lock charger_only_wk;
#endif

#ifdef CONFIG_TOUCHSCREEN_ZET622X_I2C

extern int probe_finished;

#define ZET_CHARGER_ON              (1)
#define ZET_CHARGER_OFF             (0)

int charger_on = ZET_CHARGER_OFF;
EXPORT_SYMBOL_GPL(charger_on);


void zet_charger_status_set  (int charger_status )
{
        if(charger_status == ZET_CHARGER_ON)
        {
                pr_debug("[ZET]: SET CHARGER_ON\n");
        }
        else
        {
                pr_debug("[ZET]: SET CHARGER_OFF\n");        
        }
        charger_on = charger_status;
}
#endif

#ifdef AP_CAL_BATT_CAP
#include <mach/irqs.h>
#include <linux/gpio.h>
#include <linux/time.h>
#include <linux/ktime.h>

#define HL_ADJ_CHG 1

extern void msm_pm_set_max_sleep_time(int64_t max_sleep_time_ns);
extern int get_max_sleep_time(void);
extern long get_sleep_sec(void);
extern void clear_pm_seq(void);
extern void kernel_power_off(void);
#endif

#define	BATTERY_RPC_PROG		0x30000089
#define	BATTERY_RPC_VER_5_1		0x00050001

#define	BATTERY_REGISTER_PROC		2
#define	BATTERY_DEREGISTER_CLIENT_PROC	5

#define	BATTERY_RPC_CB_PROG		(BATTERY_RPC_PROG | 0x01000000)

#define	BATTERY_CB_TYPE_PROC		1
#define	BATTERY_CB_ID_ALL		1
#define	BATTERY_CB_ID_LOW_V		2
#define	BATTERY_CB_ID_CHG_EVT		3

#define	CHG_RPC_PROG			0x3000001a
#define	CHG_RPC_VER_1_1			0x00010001

#define	CHG_GET_GENERAL_STATUS_PROC	9

#define	RPC_TIMEOUT			5000	/* 5 sec */
#define	INVALID_HANDLER			-1

#ifdef AP_CAL_BATT_CAP
#define MSM_BATT_POLLING_TIME           (30 * HZ) /*must < 60*/
#define MSM_BATT_SUS_POLLING_TIME       (60 * HZ)
#define ADD_FOR_SLEEP  (MSM_BATT_SUS_POLLING_TIME/MSM_BATT_POLLING_TIME)
#else
#define	MSM_BATT_POLLING_TIME		(15 * HZ)
#endif
#define	BATTERY_HIGH			4200
#define	BATTERY_LOW			3500
#define BATT_VOT_MIN			3000
#define	TEMPERATURE_HOT			350
#define	TEMPERATURE_COLD		50



#ifdef HL_ADJ_CHG
static u32 batt_cap;
static u32 chg_count = 0;
static u32 chg_tmp_count = 0;
static int lcd_off = 0;
static int chg_on = 0;
static int chg_changed = 0;
#endif

#ifdef AP_CAL_BATT_CAP
#define GET_VOL_M  60/(MSM_BATT_POLLING_TIME/HZ)
static struct wake_lock charger_lock;
#define VOL_THRES 150
#define CAP_THRES 1
#define FIRST_COUNT  GET_VOL_M
#define BATT_COUNT (3*GET_VOL_M)
#define SLEEP_ADD 5
#define CHG_SLEEP_ADD 3
#define NORMAL_ADD 20
#define CHG_NORMAL_ADD 10
#define CHG_MAX (5*60*GET_VOL_M)
#define DISCHG_MAX (10*60*GET_VOL_M)
#define ACT_LOW_BATT 3570
#define SLEEP_LOW_BATT 3590
#if defined(CONFIG_MSM8X25_B962) || defined(CONFIG_MSM8X25_B902)
#define DIFF_FROM_CHG 20
#define GPIO_CHG_STAT 18
static int using_irq = 0;
#elif defined(CONFIG_MSM8X25_T049) || defined(CONFIG_MSM8X25_B708)
#define DIFF_FROM_CHG 10
#else
#define DIFF_FROM_CHG 10
#endif
#if defined(CONFIG_MSM8X25_B708)
#define VOLT_AS_FULL 97
#else
#define VOLT_AS_FULL 99
#endif

static ktime_t  now_tm;
static struct timespec delta_tm;
static struct timespec delta_tm2;
static long int del_sec = 0;
static int cal_seq = 0;
static int chg_finished = 0;
static int chg_time = 0;
static int dec_when_chg = 0;
static int inc_when_dischg = 0;
static unsigned int dischg_time = 0;
static int just_after_resume = 0;
static int just_after_sleep = 0;
static u32 first_count = 0;
static u32 last_batt_vol = 0;
static int vol_need_try = 0;
static u32 last_cap = 0;
static u32 vol_count = 0;
static u32 cap_sum = 0;
static int low_batt_count = 0;
static int best_power_off = 0;
static u32 del_for_sleep[8][2] = {//{minutes/per_batt_cap,last_cap}
	{180,90},
	{160,80},
	{160,70},
	{140,60},
	{140,50},
	{120,40},
	{120,30},
	{100,0},
};
static u32 (*vol2cap)[2];
#define CAP_ARRAY_NUM  22
#if defined(CONFIG_MSM8X25_B962) || defined(CONFIG_MSM8X25_B962) || defined(CONFIG_MSM8X25_B902)
static u32 sku5_voltage_to_capacity_table[CAP_ARRAY_NUM][2] = {
	   {4190,  99},
	   {4090,  95},
	   {4050,  90},
	   {4000,  85},
	   {3960,  80},
	   {3930,  75},
	   {3900,  70},
	   {3860,  65},
	   {3830,  60},
	   {3800,  55},
	   {3776,  50},
	   {3750,  45},
	   {3740,  40},
	   {3730,  35},
	   {3720,  30},
	   {3710,  25},
	   {3690,  20},
	   {3660,  15},
	   {3640,  10},
	   {3610,  5},
	   {3560,  0},
	   {   0,  0},
   };
static u32 sku5_chg_voltage_to_capacity_table[CAP_ARRAY_NUM][2] = {
	   {4190,  99},
	   {4120,  95},
	   {4080,  90},
	   {4040,  85},
	   {3990,  80},
	   {3950,  75},
	   {3920,  70},
	   {3890,  65},
	   {3850,  60},
	   {3820,  55},
	   {3796,  50},
	   {3766,  45},
	   {3746,  40},
	   {3738,  35},
	   {3728,  30},
	   {3718,  25},
	   {3708,  20},
	   {3698,  15},
	   {3658,  10},
	   {3630,  5},
	   {3570,  0},
	   {   0,  0},
   };
#elif defined(CONFIG_MSM8X25_T049)
static u32 sku5_voltage_to_capacity_table[CAP_ARRAY_NUM][2] = {
	   {4188,  99},
	   {4110,  95},
	   {4070,  90},
	   {4020,  85},
	   {3990,  80},
	   {3940,  75},
	   {3910,  70},
	   {3880,  65},
	   {3840,  60},
	   {3800,  55},
	   {3780,  50},
	   {3760,  45},
	   {3740,  40},
	   {3730,  35},
	   {3720,  30},
	   {3710,  25},
	   {3690,  20},
	   {3660,  15},
	   {3640,  10},
	   {3610,  5},
	   {3570,  0},
	   {   0,  0},
   };
static u32 sku5_chg_voltage_to_capacity_table[CAP_ARRAY_NUM][2] = {
	   {4188,  99},
	   {4150,  95},
	   {4090,  90},
	   {4050,  85},
	   {4010,  80},
	   {3970,  75},
	   {3930,  70},
	   {3910,  65},
	   {3870,  60},
	   {3840,  55},
	   {3800,  50},
	   {3786,  45},
	   {3766,  40},
	   {3748,  35},
	   {3738,  30},
	   {3728,  25},
	   {3718,  20},
	   {3700,  15},
	   {3680,  10},
	   {3630,  5},
	   {3570,  0},
	   {   0,  0},
   };
#else //B708
static u32 sku5_voltage_to_capacity_table[CAP_ARRAY_NUM][2] = {
	   {4186,  99},
	   {4100,  95},
	   {4060,  90},
	   {4020,  85},
	   {3980,  80},
	   {3940,  75},
	   {3910,  70},
	   {3870,  65},
	   {3830,  60},
	   {3800,  55},
	   {3770,  50},
	   {3750,  45},
	   {3735,  40},
	   {3725,  35},
	   {3705,  30},
	   {3695,  25},
	   {3675,  20},
	   {3660,  15},
	   {3630,  10},
	   {3615,  5},
	   {3570,  0},
	   {   0,  0},
   };
static u32 sku5_chg_voltage_to_capacity_table[CAP_ARRAY_NUM][2] = {
	   {4186,  99},
	   {4150,  95},
	   {4120,  90},
	   {4080,  85},
	   {4040,  80},
	   {3900,  75},
	   {3960,  70},
	   {3930,  65},
	   {3890,  60},
	   {3850,  55},
	   {3820,  50},
	   {3790,  45},
	   {3760,  40},
	   {3748,  35},
	   {3728,  30},
	   {3718,  25},
	   {3705,  20},
	   {3690,  15},
	   {3670,  10},
	   {3635,  5},
	   {3590,  0},
	   {   0,  0},
   };
#endif
#endif//#ifdef AP_CAL_BATT_CAP
struct msm_battery_info {
	struct msm_rpc_endpoint *charger_endpoint;
	struct msm_rpc_client *battery_client;
	u32 charger_api_version;
	u32 battery_api_version;

	u32 voltage_max_design;
	u32 voltage_min_design;
	u32 battery_technology;
	u32 available_charger_src;
	u32 (*calculate_capacity)(u32 voltage);

	struct power_supply *msm_psy_ac;
	struct power_supply *msm_psy_usb;
	struct power_supply *msm_psy_battery;
	struct power_supply *msm_psy_unknown;

#if defined(CONFIG_BATTERY_EARLYSUSPEND) || defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
	bool is_suspended;
	struct mutex suspend_lock;
#endif /* CONFIG_BATTERY_EARLYSUSPEND */

	struct workqueue_struct	*battery_queue;
	struct delayed_work battery_work;
	struct mutex update_mutex;
	struct wake_lock charger_cb_wake_lock;

	s32 charger_handler;
	s32 battery_handler;

	int fuel_gauge;
	int (*get_battery_mvolts) (void);
	int (*get_battery_temperature) (void);
	int (*is_battery_present) (void);
	int (*is_battery_temp_within_range) (void);
	int (*is_battery_id_valid) (void);
	int (*get_battery_status)(void);
	int (*get_batt_remaining_capacity) (void);

	u32 charger_status;
	u32 charger_hardware;
	u32 hide;
	u32 battery_status;
	u32 battery_voltage;
	u32 battery_capacity;
	s32 battery_temp;
	u32 is_charging;
	u32 is_charging_complete;
	u32 is_charging_failed;

	struct power_supply *current_psy;
	u32 current_charger_src;

	u32 psy_status;
	u32 psy_health;
};

static struct msm_battery_info msm_battery_info = {
	.battery_handler = INVALID_HANDLER,
	.charger_handler = INVALID_HANDLER,
	.charger_status = CHARGER_STATUS_NULL,
	.charger_hardware = CHARGER_TYPE_USB_PC,
	.hide = 0,
	.battery_status = BATTERY_STATUS_GOOD,
	.battery_voltage = 3700,
	.battery_capacity = 50,
	.battery_temp = 200,
	.is_charging = false,
	.is_charging_complete = false,
	.is_charging_failed = false,
	.psy_status = POWER_SUPPLY_STATUS_DISCHARGING,
	.psy_health = POWER_SUPPLY_HEALTH_GOOD,
};

static enum power_supply_property msm_charger_psy_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *msm_charger_supplied_to[] = {
	"battery",
};

static int msm_charger_psy_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	//printk("+++ get_property: psp=%d,type=%d,charger_src=%d\n",psp,psy->type, msm_battery_info.current_charger_src);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		switch(psy->type) {
		case POWER_SUPPLY_TYPE_MAINS:
			val->intval = msm_battery_info.current_charger_src & AC_CHG
				? 1 : 0;
			break;
		case POWER_SUPPLY_TYPE_USB:
			val->intval = msm_battery_info.current_charger_src & USB_CHG
				? 1 : 0;
			break;
		case POWER_SUPPLY_TYPE_UNKNOWN:
			val->intval = msm_battery_info.current_charger_src & UNKNOWN_CHG
				? 1 : 0;
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static struct power_supply msm_psy_ac = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.supplied_to = msm_charger_supplied_to,
	.num_supplicants = ARRAY_SIZE(msm_charger_supplied_to),
	.properties = msm_charger_psy_properties,
	.num_properties = ARRAY_SIZE(msm_charger_psy_properties),
	.get_property = msm_charger_psy_get_property,
};

static struct power_supply msm_psy_unknown = {
	.name = "unknown",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.supplied_to = msm_charger_supplied_to,
	.num_supplicants = ARRAY_SIZE(msm_charger_supplied_to),
	.properties = msm_charger_psy_properties,
	.num_properties = ARRAY_SIZE(msm_charger_psy_properties),
	.get_property = msm_charger_psy_get_property,
};

static struct power_supply msm_psy_usb = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.supplied_to = msm_charger_supplied_to,
	.num_supplicants = ARRAY_SIZE(msm_charger_supplied_to),
	.properties = msm_charger_psy_properties,
	.num_properties = ARRAY_SIZE(msm_charger_psy_properties),
	.get_property = msm_charger_psy_get_property,
};

static enum power_supply_property msm_battery_psy_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP
};

static int msm_battery_psy_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = msm_battery_info.psy_status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = msm_battery_info.psy_health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = (msm_battery_info.battery_status !=
			       BATTERY_STATUS_NULL);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = msm_battery_info.battery_technology;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = msm_battery_info.voltage_max_design;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = msm_battery_info.voltage_min_design;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = msm_battery_info.battery_voltage;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = msm_battery_info.battery_capacity;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = msm_battery_info.battery_temp;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static struct power_supply msm_psy_battery = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = msm_battery_psy_properties,
	.num_properties = ARRAY_SIZE(msm_battery_psy_properties),
	.get_property = msm_battery_psy_get_property,
};

#ifndef CONFIG_BATTERY_MSM_FAKE
struct rpc_reply_charger {
	struct	rpc_reply_hdr hdr;
	u32 more_data;

	u32 charger_status;
	u32 charger_hardware;
	u32 hide;
	u32 battery_status;
	u32 battery_voltage;
	u32 battery_capacity;
	s32 battery_temp;
	u32 is_charging;
	u32 is_charging_complete;
	u32 is_charging_failed;
};

static struct rpc_reply_charger reply_charger;

#define	be32_to_cpu_self(v)	(v = be32_to_cpu(v))

static int msm_battery_get_charger_status(void)
{
	int rc;
	struct rpc_request_charger {
		struct rpc_request_hdr hdr;
		u32 more_data;
	} request_charger;

	request_charger.more_data = cpu_to_be32(1);
	memset(&reply_charger, 0, sizeof(reply_charger));

	rc = msm_rpc_call_reply(msm_battery_info.charger_endpoint,
				CHG_GET_GENERAL_STATUS_PROC,
				&request_charger, sizeof(request_charger),
				&reply_charger, sizeof(reply_charger),
				msecs_to_jiffies(RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("BATT: ERROR: %s, charger rpc call %d, rc=%d\n",
		       __func__, CHG_GET_GENERAL_STATUS_PROC, rc);
		return rc;
	} else if (be32_to_cpu(reply_charger.more_data)) {
		be32_to_cpu_self(reply_charger.charger_status);
		be32_to_cpu_self(reply_charger.charger_hardware);
		be32_to_cpu_self(reply_charger.hide);
		be32_to_cpu_self(reply_charger.battery_status);
		be32_to_cpu_self(reply_charger.battery_voltage);
		be32_to_cpu_self(reply_charger.battery_capacity);
		be32_to_cpu_self(reply_charger.battery_temp);
		be32_to_cpu_self(reply_charger.is_charging);
		be32_to_cpu_self(reply_charger.is_charging_complete);
		be32_to_cpu_self(reply_charger.is_charging_failed);
	} else {
		pr_err("BATT: ERROR: %s, No data in charger rpc reply\n",
		       __func__);
		return -EIO;
	}
	#ifdef HL_ADJ_CHG
	if (0 == batt_cap)
			batt_cap = reply_charger.battery_capacity & 0x7F;
	#endif
	return 0;
}

static void update_charger_type(u32 charger_hardware)
{
	switch(charger_hardware) {
	case CHARGER_TYPE_USB_PC:
		pr_debug("BATT: usb pc charger inserted\n");

		msm_battery_info.current_psy = &msm_psy_usb;
		msm_battery_info.current_charger_src = USB_CHG;
		break;
	case CHARGER_TYPE_USB_WALL:
		pr_debug("BATT: usb wall changer inserted\n");

		msm_battery_info.current_psy = &msm_psy_ac;
		msm_battery_info.current_charger_src = AC_CHG;
		break;
	case CHARGER_TYPE_USB_UNKNOWN:
		pr_debug("BATT: unknown changer inserted\n");

		msm_battery_info.current_psy = &msm_psy_unknown;
		msm_battery_info.current_charger_src = UNKNOWN_CHG;
		break;
	default:
		pr_debug("BATT: CAUTION: charger hardware\n");

		msm_battery_info.current_psy = &msm_psy_unknown;
		msm_battery_info.current_charger_src = UNKNOWN_CHG;
		break;
	}
}

#ifdef AP_CAL_BATT_CAP
static u32 cal_batt_cap(void)
{
	u32 cap_real = 50;
	u32 base, top, percent_diff, cap_tmp;
	u32 v_tmp;
	u32 v_read;
	u32 v_oem;
	int i;
	u32 cap_thres;
	#ifdef GPIO_CHG_STAT
        if (1 < using_irq) {
		if ((4186 < msm_battery_info.battery_voltage) && (99 < msm_battery_info.battery_capacity)) {
			printk("+++BATT charging finished\n");
			chg_finished = 1;
			last_cap = 100;
			return 100;
		} else
			printk("+++ BATT charging finished signal received\n");
	}
	#endif
	v_read = reply_charger.battery_voltage >> 16;
	if (BATT_VOT_MIN > v_read)
		v_read = reply_charger.battery_voltage & 0xFFFF;
        v_oem = (reply_charger.battery_capacity >> 7) & 0x1FFF;
	v_tmp = v_read;
	if (cal_seq) {
		now_tm = ktime_get_boottime();
		delta_tm = ktime_to_timespec(now_tm);
		cal_seq = 0;	
	} else {
		now_tm = ktime_get_boottime();
		delta_tm2 = ktime_to_timespec(now_tm);
		cal_seq = 1;
	}
	if (lcd_off) {
		if (chg_on)
			v_tmp += CHG_SLEEP_ADD;
		else
			v_tmp += SLEEP_ADD;
	} else {
		if (chg_on)
			v_tmp += CHG_NORMAL_ADD;
		else
			v_tmp += NORMAL_ADD;
	}
	if ((1 == lcd_off) && (1 == chg_on))
		chg_time++;

	if (FIRST_COUNT > first_count) {
		if (BATT_VOT_MIN < v_oem)
			v_tmp = v_oem;
		if(BATT_VOT_MIN > v_tmp)
			v_tmp = sku5_chg_voltage_to_capacity_table[CAP_ARRAY_NUM-1][0]+10;
		if((chg_on) && (sku5_chg_voltage_to_capacity_table[13][0] < v_tmp))
			v_tmp -= DIFF_FROM_CHG;
		if((chg_on) && (sku5_chg_voltage_to_capacity_table[16][0] < v_tmp))
			v_tmp -= DIFF_FROM_CHG;
		last_batt_vol = v_tmp;
	} else {
		if ((200 > dischg_time) || (0 == just_after_resume)) {
		if (VOL_THRES < (((v_tmp-last_batt_vol)>0)?(v_tmp-last_batt_vol):(last_batt_vol-v_tmp))){
			if(1 < vol_need_try) {
				last_batt_vol = v_tmp;
				vol_need_try = 0;
			} else {
				v_tmp = last_batt_vol;
				vol_need_try++;
			}
		} else {
			last_batt_vol = v_tmp;
			vol_need_try = 0;
		}
		}
	}	

	if (v_tmp >= vol2cap[0][0]) {
		if(99 < last_cap)
			cap_tmp = 100;
		else
			cap_tmp = 99;
	} else {
		for (i = 1; i < CAP_ARRAY_NUM; i++) {
			if (v_tmp > vol2cap[i][0]) {
				//printk("+++ batt_vol=%d > tab[%d][0]=%d\n",v_tmp,i,vol2cap[i][0]);
				break;
			}
		}
		base = vol2cap[i][0];
		top = vol2cap[i-1][0];
		percent_diff = vol2cap[i-1][1] - vol2cap[i][1];
		cap_tmp = vol2cap[i][1] + (v_tmp - base) * percent_diff / (top - base);
		//printk("+++  base=%d top=%d diff=%d cap_tmp=%d\n",base,top,percent_diff,cap_tmp);
		if ((99 < last_cap) && (97 < cap_tmp))
			cap_tmp = 100;
	}

	vol_count ++;
	if ((FIRST_COUNT > first_count) && (1 < dischg_time))
		first_count = FIRST_COUNT;//special case

	if (FIRST_COUNT > first_count) {
		last_cap = cap_tmp;
		cap_real = cap_tmp;
		//printk("BATT:+++ fist time: batt_vol=%d, batt_cap=%d\n",v_read,cap_real);
		vol_count = 0;
		first_count ++;
	} else {
		first_count = FIRST_COUNT;
        	printk("+++ BATT: chg_time =%d dis_chg_time = %d\n",chg_time,dischg_time);
		if (BATT_COUNT > vol_count) {	
			if (delta_tm2.tv_sec > delta_tm.tv_sec)
				del_sec = delta_tm2.tv_sec - delta_tm.tv_sec;
			else
				del_sec = delta_tm.tv_sec - delta_tm2.tv_sec;
			if (25 > del_sec) {
				vol_count --;
				return last_cap;
			}
			cap_sum += cap_tmp;
			printk("BATT:+++ count=%d: cap_sum=%d, cap_tmp=%d\n",vol_count,cap_sum,cap_tmp);
			cap_real = last_cap;
		} else {
			cap_real = cap_sum/(vol_count - 1);
			//printk("BATT:=== count=%d: cap_sum=%d, cap_real=%d\n",vol_count,cap_sum,cap_real);
			if (last_cap > cap_real)
				cap_tmp = last_cap - cap_real;
			else
				cap_tmp = cap_real - last_cap;
	
			printk("BATT:=== last_cap=%d, avr_cap=%d cap_sum=%d, cap_diff=%d\n",last_cap,cap_real,cap_sum,cap_tmp);
			if (lcd_off)
				cap_thres = ADD_FOR_SLEEP * CAP_THRES;
			else
				cap_thres = CAP_THRES;

			if (40 > cap_real)
				cap_thres ++;
			if (20 < cap_tmp)
				cap_thres ++;
			if (10 < cap_tmp)
				cap_thres ++;
			if (5 < cap_tmp)
				cap_thres ++;

			if(cap_thres < cap_tmp) {
				if (last_cap > cap_real) {
					inc_when_dischg = 0;
					if (chg_on) {
						dec_when_chg++;
						if ((1 < dec_when_chg) ||(msm_battery_info.current_charger_src & USB_CHG)){
							if (lcd_off)
								cap_real = last_cap - 1;
							else
								cap_real = last_cap - cap_thres;
							dec_when_chg = 0;
						} else
							cap_real = last_cap;
					} else
						cap_real = last_cap - cap_thres;
				} else {
					if (1 == chg_on) {
						dec_when_chg = 0;
						cap_real = last_cap + cap_thres;
						if (40 < chg_time && v_read < 4100) {
							cap_tmp -= cap_thres;
							cap_real += (cap_tmp > cap_thres)?cap_thres:1;
							chg_time = 0;
						}
					} else {
						if (last_cap < cap_real)
							inc_when_dischg ++;
						else
							inc_when_dischg = 0;

						if (1 < inc_when_dischg) {
							if (50 > last_cap)
								cap_real = last_cap + 1;
							else
								cap_real = last_cap;
							inc_when_dischg = 0;
						}else
							cap_real = last_cap;
					}
				}
			} else {
				if ((0 == chg_on) && ((last_cap < cap_real)))
					inc_when_dischg ++;
				else
					inc_when_dischg = 0;

				if (1 < inc_when_dischg) {
					if (50 > last_cap)
						cap_real = last_cap + 1;
					else
						cap_real = last_cap;
					inc_when_dischg = 0;
				} else {
					if ((last_cap < cap_real) && (0 == chg_on))
						cap_real = last_cap;
				}
			}
			last_cap = cap_real;
			if((VOLT_AS_FULL <= last_cap) && chg_on) {
				chg_finished++;
				if (lcd_off)
					chg_finished++;
				if (12 < chg_finished) {
					last_cap = cap_real = 100;
					chg_finished = 1;
				}
			} else
				chg_finished = 1;
			cap_sum = 0;
			vol_count = 0;
		}
	}
	return cap_real;
}
#endif

void msm_battery_update_psy_status(void)
{
	u32 charger_status;
	u32 charger_hardware;
	u32 hide;
	u32 battery_status;
	u32 battery_voltage;
	u32 battery_capacity;
	s32 battery_temp;
	u32 is_charging;
	u32 is_charging_complete;
	u32 is_charging_failed;
	bool is_awake = true;
	int is_true = 0;
#ifdef CONFIG_MSM_SM_EVENT
	sm_msm_battery_data_t battery_data;
#endif
#ifdef CONFIG_BATTERY_EARLYSUSPEND
	is_awake = !msm_battery_info.is_suspended;
#elif defined(HL_ADJ_CHG)
	is_awake = !(lcd_off);
#endif
	//pr_debug("BATT: msm_battery_update_psy_status");

	mutex_lock(&msm_battery_info.update_mutex);

	if (msm_battery_get_charger_status()) {
		goto done;
	}

	if (msm_battery_info.fuel_gauge)
	{
		charger_status		= reply_charger.charger_status;
		charger_hardware	= reply_charger.charger_hardware;
		hide			= reply_charger.hide;
		battery_status		= msm_battery_info.get_battery_status();
		battery_voltage		= msm_battery_info.get_battery_mvolts();
		battery_capacity	= msm_battery_info.get_batt_remaining_capacity();
		battery_temp		= msm_battery_info.get_battery_temperature();
		is_charging		= reply_charger.is_charging;
		is_charging_complete	= reply_charger.is_charging_complete;
		is_charging_failed	= reply_charger.is_charging_failed;
	}
	else
	{
		charger_status		= reply_charger.charger_status;
		charger_hardware	= reply_charger.charger_hardware;
		hide			= reply_charger.hide;
		battery_status		= reply_charger.battery_status;
		battery_voltage		= reply_charger.battery_voltage & 0xFFFF;
		battery_capacity	= reply_charger.battery_capacity & 0x7F;
		battery_temp		= reply_charger.battery_temp * 10;
		is_charging		= reply_charger.is_charging;
		is_charging_complete	= reply_charger.is_charging_complete;
		is_charging_failed	= reply_charger.is_charging_failed;
	}
#ifdef CONFIG_MSM_SM_EVENT
		battery_data.charger_status = charger_status;
		battery_data.battery_voltage = battery_voltage;
		battery_data.battery_temp = battery_temp;
		sm_add_event (SM_POWER_EVENT|SM_POWER_EVENT_BATTERY_UPDATE, 0, 0, (void *)&battery_data, sizeof(battery_data));
#endif

	pr_debug("BATT: received, %d, %d, 0x%x; %d, %d, %d, %d; %d, %d, %d; %d, %d, %d\n",
		  charger_status, charger_hardware, hide,
		  battery_status, battery_voltage, battery_capacity, battery_temp,
		  is_charging, is_charging_complete, is_charging_failed,
		  reply_charger.battery_voltage >> 16,
		  (reply_charger.battery_capacity >> 7) & 0x1FFF,
		  reply_charger.battery_capacity >> 20);
	is_true = 0;
#ifdef AP_CAL_BATT_CAP
	if (chg_on) {
		vol2cap = sku5_chg_voltage_to_capacity_table;
	} else {
		vol2cap = sku5_voltage_to_capacity_table;
	}
	if ((vol2cap[CAP_ARRAY_NUM-2][0] > battery_voltage) && (BATT_VOT_MIN < battery_voltage))
		is_true ++;
	if ((vol2cap[CAP_ARRAY_NUM-2][0] > (reply_charger.battery_voltage >> 16)) && (BATT_VOT_MIN < (reply_charger.battery_voltage >> 16)))
		is_true ++;
	if (0 != is_true) {
		if(1 == best_power_off) {
                        //printk("BATT:+++ try to power off +++++\n");
			#if defined(CHARGER_ONLY_CASE)
			if ((MSM_BOOT_CHARGER != current_boot_mode) || ((MSM_BOOT_CHARGER == current_boot_mode) && (chg_changed)))
			#endif
			{
			printk("BATT:+++ need to power off, batt_cap=%d\n", battery_capacity);
                        kernel_power_off();
			}
		}
		#if defined(CHARGER_ONLY_CASE)
		if ((MSM_BOOT_CHARGER != current_boot_mode) && (6 > msm_battery_info.battery_capacity)) {
		#else
		if (6 > msm_battery_info.battery_capacity) {
		#endif
			battery_capacity = 0;
			msm_battery_info.battery_capacity = 0;
		} else {
			if (11 > msm_battery_info.battery_capacity) {
				battery_capacity = last_cap = 5;
				msm_battery_info.battery_capacity = 5;
			} else {
				battery_capacity = last_cap = 10;
				msm_battery_info.battery_capacity = 10;
			}
		} 
                power_supply_changed(msm_battery_info.current_psy);
		is_true = 0;
		if ((ACT_LOW_BATT > battery_voltage) && (BATT_VOT_MIN < battery_voltage))
			is_true ++;
		if ((ACT_LOW_BATT > (reply_charger.battery_voltage >> 16)) && (BATT_VOT_MIN < (reply_charger.battery_voltage >> 16)))
			is_true ++;
		if ((0 == last_cap)||(0 != is_true )) {
			low_batt_count ++;
			if (2 < low_batt_count) {
			mutex_unlock(&msm_battery_info.update_mutex);
			#if defined(CHARGER_ONLY_CASE)
			if ((MSM_BOOT_CHARGER != current_boot_mode) || ((MSM_BOOT_CHARGER == current_boot_mode) && (chg_changed)))
			#endif
			{
			printk("BATT: prepare power down for low batt!! %d \n",low_batt_count);
			kernel_power_off();
			}
			return;
			}
		}
		goto done;
	}
	battery_capacity = cal_batt_cap();
	#if defined(CHARGER_ONLY_CASE)
	if ((MSM_BOOT_CHARGER == current_boot_mode))
		if(1 > battery_capacity)
			battery_capacity = 1;
	#endif
	if (1 > battery_capacity) {//special case
		mutex_unlock(&msm_battery_info.update_mutex);
		#if defined(CHARGER_ONLY_CASE)
		if ((MSM_BOOT_CHARGER != current_boot_mode) || ((MSM_BOOT_CHARGER == current_boot_mode) && (chg_changed)))
		#endif
		{
		printk("BATT: power down for low batt!!  \n");
		kernel_power_off();
		}
		return;
	}
	printk("BATT:+++ batt_vol=%d, batt_cap=%d\n",last_batt_vol, battery_capacity);
	low_batt_count = 0;
	//printk("BATT:+++ best_power_off=%d\n", best_power_off);
	if(1 == best_power_off) {
		printk("BATT:+++ need to power off, batt_cap=%d\n", battery_capacity);
		if (11 > battery_capacity) {
			#if defined(CHARGER_ONLY_CASE)
			if ((MSM_BOOT_CHARGER != current_boot_mode) || ((MSM_BOOT_CHARGER == current_boot_mode) && (chg_changed)))
			#endif
			{
			printk("BATT:+++ try to power off +++++\n");
			kernel_power_off();
			}
		}
		wake_unlock(&charger_lock);
		best_power_off = 0;
	}
#endif//#ifdef AP_CAL_BATT_CAP
	if (charger_status	== msm_battery_info.charger_status &&
	    charger_hardware	== msm_battery_info.charger_hardware &&
	    hide		== msm_battery_info.hide &&
	    battery_status	== msm_battery_info.battery_status &&
	    battery_voltage	== msm_battery_info.battery_voltage &&
	    battery_capacity	== msm_battery_info.battery_capacity &&
	    battery_temp	== msm_battery_info.battery_temp &&
	    is_charging		== msm_battery_info.is_charging &&
	    is_charging_complete== msm_battery_info.is_charging_complete &&
	    is_charging_failed	== msm_battery_info.is_charging_failed) {
 
		goto done;
	}

	if (msm_battery_info.charger_status != charger_status) {
		if (msm_battery_info.charger_status == CHARGER_STATUS_NULL) {
			pr_debug("BATT: start charging\n");
			#ifdef HL_ADJ_CHG
			chg_on = 1;
			chg_tmp_count= 0;
			chg_count = 0;
			batt_cap = 0;
			wake_lock(&charger_lock);
			#endif
			#ifdef GPIO_CHG_STAT
			using_irq = 1;
			#endif
			#ifdef CONFIG_TOUCHSCREEN_ZET622X_I2C
			if(probe_finished)
                        {
			        zet_charger_status_set(ZET_CHARGER_ON);
			}
			#endif
			update_charger_type(charger_hardware);
		} else if (charger_status == CHARGER_STATUS_NULL) {
			pr_debug("BATT: end charging\n");
			#ifdef HL_ADJ_CHG
			chg_on = 0;
			chg_changed = 1;
			chg_tmp_count= 0;
			chg_count = 0;
			batt_cap = 0;//msm_battery_info.battery_capacity;
			wake_unlock(&charger_lock);
			#endif
			#ifdef GPIO_CHG_STAT
			using_irq = 1;
			#endif
			#ifdef CONFIG_TOUCHSCREEN_ZET622X_I2C
			if(probe_finished)
			{			
                                zet_charger_status_set(ZET_CHARGER_OFF);
			}
			#endif
			if (msm_battery_info.current_charger_src & USB_CHG) {
				pr_debug("BATT: usb pc charger removed\n");
			} else if (msm_battery_info.current_charger_src & AC_CHG) {
				pr_debug("BATT: usb wall charger removed\n");
			} else if (msm_battery_info.current_charger_src & UNKNOWN_CHG) {
				pr_debug("BATT: unknown wall charger removed\n");
			} else {
				pr_debug("BATT: CAUTION: charger invalid: %d\n",
					  msm_battery_info.current_charger_src);
			}

			msm_battery_info.current_psy = &msm_psy_battery;
			msm_battery_info.current_charger_src = 0;
		} else {
			pr_err("BATT: CAUTION: charger status change\n");
		}
	} else if(charger_hardware != msm_battery_info.charger_hardware) {
		// Due to the limit of the method we used to detect non-standard wall-charger,
		// the charger type may change when the phone is charging
		pr_debug("BATT: charger type changed while charging\n");
		update_charger_type(charger_hardware);
	}

	if (charger_status == CHARGER_STATUS_NULL) {
		msm_battery_info.psy_status = POWER_SUPPLY_STATUS_DISCHARGING;
	} else if (battery_capacity == 100) {
		msm_battery_info.psy_status = POWER_SUPPLY_STATUS_FULL;
	} else if (msm_battery_info.charger_status == CHARGER_STATUS_NULL) {
		msm_battery_info.current_psy = &msm_psy_battery;
		msm_battery_info.current_charger_src = 0;
		msm_battery_info.psy_status = POWER_SUPPLY_STATUS_DISCHARGING;
	} else {
		msm_battery_info.psy_status = POWER_SUPPLY_STATUS_CHARGING;
	}

	if (msm_battery_info.battery_status != battery_status) {
		if (battery_status == BATTERY_STATUS_NULL) {
			pr_debug("BATT: battery removed\n");

			msm_battery_info.psy_health = POWER_SUPPLY_HEALTH_DEAD;
		} else if (battery_status == BATTERY_STATUS_OVER_TEMPERATURE) {
			if (battery_temp >= TEMPERATURE_HOT) {
				pr_debug("BATT: battery overheat\n");

				msm_battery_info.psy_health = POWER_SUPPLY_HEALTH_OVERHEAT;
			} else if (battery_temp <= TEMPERATURE_COLD) {
				pr_debug("BATT: battery cold\n");

				msm_battery_info.psy_health = POWER_SUPPLY_HEALTH_COLD;
			}
		} else if (battery_status == BATTERY_STATUS_GOOD) {
			pr_debug("BATT: battery good\n");

			msm_battery_info.psy_health = POWER_SUPPLY_HEALTH_GOOD;
		} else {
			pr_err("BATT: CAUTION: battery status invalid: %d\n",
				  battery_status);
		}
	}

	msm_battery_info.charger_status		= charger_status;
	msm_battery_info.charger_hardware	= charger_hardware;
	msm_battery_info.hide			= hide;
	msm_battery_info.battery_status		= battery_status;
	msm_battery_info.battery_voltage	= battery_voltage;
	msm_battery_info.battery_capacity	= battery_capacity;
	msm_battery_info.battery_temp		= battery_temp;
	msm_battery_info.is_charging		= is_charging;
	msm_battery_info.is_charging_complete	= is_charging_complete;
	msm_battery_info.is_charging_failed	= is_charging_failed;

	if (msm_battery_info.current_psy) {
		power_supply_changed(msm_battery_info.current_psy);
	}

done:

	mutex_unlock(&msm_battery_info.update_mutex);

	if(is_awake)
		queue_delayed_work(msm_battery_info.battery_queue,
				   &msm_battery_info.battery_work,
				   MSM_BATT_POLLING_TIME);
#ifdef AP_CAL_BATT_CAP
	else
		queue_delayed_work(msm_battery_info.battery_queue,
				   &msm_battery_info.battery_work,
				MSM_BATT_SUS_POLLING_TIME);
#endif
	return;
}
EXPORT_SYMBOL(msm_battery_update_psy_status);

#ifdef CONFIG_MSM_SM_EVENT
uint32_t msm_batt_get_batt_voltage (void)
{
	return msm_battery_info.battery_voltage;
}
EXPORT_SYMBOL(msm_batt_get_batt_voltage);
#endif
int msm_batt_cap(void)
{
	return msm_battery_info.battery_capacity;
}
EXPORT_SYMBOL(msm_batt_cap);
#ifdef AP_CAL_BATT_CAP
static inline void handle_dischg_time(void)
{
	int del;

	for (del = 0; del < 8; del ++) {
		if (del_for_sleep[del][0] > dischg_time)
			continue;
		if (del_for_sleep[del][1] < last_cap) {
			if ((dischg_time/del_for_sleep[del][0]) > (last_cap - del_for_sleep[del][1])) {
				last_cap = del_for_sleep[del][1];
				dischg_time -= ((last_cap - del_for_sleep[del][1])*del_for_sleep[del][0]);
			} else {
				last_cap -= (dischg_time/del_for_sleep[del][0]);
				dischg_time -= ((dischg_time/del_for_sleep[del][0])*del_for_sleep[del][0]);
			}
		}
	}
	printk("+++BATT: after handle_dischg_time, last_cap=%d, dischg_time=%d \n",last_cap,dischg_time);
}

#endif
#if  defined(CONFIG_BATTERY_EARLYSUSPEND) || defined(CONFIG_HAS_EARLYSUSPEND)
void msm_battery_early_suspend(struct early_suspend *h)
{
	printk("BATT: %s\n", __func__);
#ifdef HL_ADJ_CHG
	lcd_off = 1;
#endif
#if  defined(CONFIG_BATTERY_EARLYSUSPEND)
	mutex_lock(&msm_battery_info.suspend_lock);
	if(msm_battery_info.is_suspended) {
		mutex_unlock(&msm_battery_info.suspend_lock);
		pr_err("BATT: CAUTION: %s, is already suspended\n", __func__);
		return;
	}
	msm_battery_info.is_suspended = true;
	mutex_unlock(&msm_battery_info.suspend_lock);
	flush_workqueue(msm_battery_info.battery_queue);
#endif
	

}

void msm_battery_late_resume(struct early_suspend *h)
{
#ifdef AP_CAL_BATT_CAP
	long int del = 0;
#endif
	printk("BATT: %s\n", __func__);

#if  defined(CONFIG_BATTERY_EARLYSUSPEND)
	mutex_lock(&msm_battery_info.suspend_lock);
	if(!msm_battery_info.is_suspended) {
		mutex_unlock(&msm_battery_info.suspend_lock);
		pr_err("BATT: CAUTION: %s, is already resumed\n", __func__);
		return;
	}
	msm_battery_info.is_suspended = false;
	mutex_unlock(&msm_battery_info.suspend_lock);
	queue_delayed_work(msm_battery_info.battery_queue,
			   &msm_battery_info.battery_work,
			   0);
#endif
#ifdef AP_CAL_BATT_CAP
	if (just_after_sleep) {
		del = get_sleep_sec();
		clear_pm_seq();
		just_after_sleep = 0;
		if (0 <= del)
			dischg_time += (del/60);
		printk("+++ BATT: late_resume, get_delta_tm = %ld, dischg_time=%d,last_cap=%d\n",del,dischg_time,last_cap);

		if (del_for_sleep[7][0] < dischg_time) {
			handle_dischg_time();
		}	
		printk("+++ BATT: late_resume, dischg_time=%d, last_cap=%d\n",dischg_time,last_cap);
		msm_battery_info.battery_capacity = last_cap;
		if (msm_battery_info.current_psy)
			power_supply_changed(msm_battery_info.current_psy);
	}
	just_after_resume = 1;
	queue_delayed_work(msm_battery_info.battery_queue,
			   &msm_battery_info.battery_work,
			   0);
#endif
#ifdef HL_ADJ_CHG
	lcd_off = 0;
#endif
}
#endif
#if defined(CONFIG_PM) && defined(AP_CAL_BATT_CAP) 
static int batt_suspend(struct device *dev)
{
	if (get_max_sleep_time())
		msm_pm_set_max_sleep_time((int64_t)((int64_t)600 * NSEC_PER_SEC));
	printk("BATT: %s\n", __func__);
	#if defined(CHARGER_ONLY_CASE)
	mdp_clk_check(0);
	#endif
	return 0;
}

static int batt_resume(struct device *dev)
{
	long int del = 0;
	int is_true = 0;
	printk("BATT: %s\n", __func__);

	del = get_sleep_sec();
	if (0 > del)
		dischg_time += 1;
	else
		dischg_time += (del/60);

	printk("+++ BATT: resume, get_delta_tm = %ld, dischg_time=%d\n",del,dischg_time);
	clear_pm_seq();
	if (0 == msm_battery_get_charger_status()) {
		is_true = 0;
		if ((SLEEP_LOW_BATT >= (reply_charger.battery_voltage & 0xFFFF)) && (BATT_VOT_MIN < (reply_charger.battery_voltage & 0xFFFF)))
			is_true ++;
		if ((SLEEP_LOW_BATT >= (reply_charger.battery_voltage >> 16)) && (BATT_VOT_MIN < (reply_charger.battery_voltage >> 16)))
			is_true ++;

		if (0 != is_true){
			#if defined(CHARGER_ONLY_CASE)
			if ((MSM_BOOT_CHARGER != current_boot_mode) || ((MSM_BOOT_CHARGER == current_boot_mode) && (chg_changed)))
			#endif
			{
			//kernel_power_off();/*for protect battery*/
			printk("+++++ BATT:  need to power off !!!!!\n");
			if(0 == best_power_off) {
				best_power_off = 1;
				wake_lock(&charger_lock);
			}
			}
		}
		printk("+++ BATT: resume, batt_vol=%d, reply_batt_vol=%d\n",(reply_charger.battery_voltage & 0xFFFF),(reply_charger.battery_voltage >> 16));
	} else
		printk("+++ BATT: resume, get batt status failed\n");

	if (del_for_sleep[7][0] < dischg_time) {
		handle_dischg_time();
		if(5 > last_cap)/*for protect battery*/{
			#if defined(CHARGER_ONLY_CASE)
			if ((MSM_BOOT_CHARGER != current_boot_mode) || ((MSM_BOOT_CHARGER == current_boot_mode) && (chg_changed)))
			#endif
			{
			//kernel_power_off();
			printk("+++++ BATT:  need to power off 1 !!!!!\n");
			if (0 == best_power_off) {
				wake_lock(&charger_lock);
				best_power_off = 1;
			}
			}
		}
	}
	if((!(chg_on)) && (1 > last_cap) && (1 > msm_battery_info.battery_capacity)) {/*for protect battery*/
		#if defined(CHARGER_ONLY_CASE)
		if ((MSM_BOOT_CHARGER != current_boot_mode) || ((MSM_BOOT_CHARGER == current_boot_mode) && (chg_changed)))
		#endif
		{
		//kernel_power_off();
			printk("+++++ BATT:  need to power off 2 !!!!!\n");
			if (0 == best_power_off) {
				wake_lock(&charger_lock);
				best_power_off = 1;
			}
		}
	}
	just_after_sleep = 1;
	return 0;
}

static struct dev_pm_ops batt_dev_pm_ops = {
	.suspend = batt_suspend,
	.resume = batt_resume,
};

#endif

struct battery_client_registration_request {
	/* Voltage at which cb func should be called */
	u32 desired_battery_voltage;
	/* Direction when the cb func should be called */
	u32 voltage_direction;
	/* Registered callback to be called */
	u32 battery_cb_id;
	/* Call back data */
	u32 cb_data;
	u32 more_data;
	u32 battery_error;
};

struct battery_client_registration_reply {
	u32 handler;
};

static int msm_battery_register_arg_func(struct msm_rpc_client *battery_client,
					 void *buffer, void *data)
{
	struct battery_client_registration_request *battery_register_request =
		(struct battery_client_registration_request *)data;

	u32 *request = (u32 *)buffer;
	int size = 0;

	*request = cpu_to_be32(battery_register_request->desired_battery_voltage);
	size += sizeof(u32);
	request++;

	*request = cpu_to_be32(battery_register_request->voltage_direction);
	size += sizeof(u32);
	request++;

	*request = cpu_to_be32(battery_register_request->battery_cb_id);
	size += sizeof(u32);
	request++;

	*request = cpu_to_be32(battery_register_request->cb_data);
	size += sizeof(u32);
	request++;

	*request = cpu_to_be32(battery_register_request->more_data);
	size += sizeof(u32);
	request++;

	*request = cpu_to_be32(battery_register_request->battery_error);
	size += sizeof(u32);

	return size;
}

static int msm_battery_register_ret_func(struct msm_rpc_client *battery_client,
					 void *buffer, void *data)
{
	struct battery_client_registration_reply *data_ptr, *buffer_ptr;

	data_ptr = (struct battery_client_registration_reply *)data;
	buffer_ptr = (struct battery_client_registration_reply *)buffer;

	data_ptr->handler = be32_to_cpu(buffer_ptr->handler);
	return 0;
}

static int msm_battery_register(u32 desired_battery_voltage, u32 voltage_direction,
				u32 battery_cb_id, u32 cb_data, s32 *handle)
{
	struct battery_client_registration_request battery_register_request;
	struct battery_client_registration_reply battery_register_reply;
	void *request;
	void *reply;
	int rc;

	battery_register_request.desired_battery_voltage = desired_battery_voltage;
	battery_register_request.voltage_direction = voltage_direction;
	battery_register_request.battery_cb_id = battery_cb_id;
	battery_register_request.cb_data = cb_data;
	battery_register_request.more_data = 1;
	battery_register_request.battery_error = 0;
	request = &battery_register_request;

	reply = &battery_register_reply;

	rc = msm_rpc_client_req(msm_battery_info.battery_client,
				BATTERY_REGISTER_PROC,
				msm_battery_register_arg_func, request,
				msm_battery_register_ret_func, reply,
				msecs_to_jiffies(RPC_TIMEOUT));
	if (rc < 0) {
		pr_err("BATT: ERROR: %s, vbatt register, rc=%d\n", __func__, rc);
		return rc;
	}

	*handle = battery_register_reply.handler;

	return 0;
}

struct battery_client_deregister_request {
	u32 handler;
};

struct battery_client_deregister_reply {
	u32 battery_error;
};

static int msm_battery_deregister_arg_func(struct msm_rpc_client *battery_client,
					   void *buffer, void *data)
{
	struct battery_client_deregister_request *deregister_request =
		(struct  battery_client_deregister_request *)data;
	u32 *request = (u32 *)buffer;
	int size = 0;

	*request = cpu_to_be32(deregister_request->handler);
	size += sizeof(u32);

	return size;
}

static int msm_battery_deregister_ret_func(struct msm_rpc_client *battery_client,
					   void *buffer, void *data)
{
	struct battery_client_deregister_reply *data_ptr, *buffer_ptr;

	data_ptr = (struct battery_client_deregister_reply *)data;
	buffer_ptr = (struct battery_client_deregister_reply *)buffer;

	data_ptr->battery_error = be32_to_cpu(buffer_ptr->battery_error);

	return 0;
}

static int msm_battery_deregister(u32 handler)
{
	int rc;
	struct battery_client_deregister_request request;
	struct battery_client_deregister_reply reply;

	request.handler = handler;

	rc = msm_rpc_client_req(msm_battery_info.battery_client,
				BATTERY_DEREGISTER_CLIENT_PROC,
				msm_battery_deregister_arg_func, &request,
				msm_battery_deregister_ret_func, &reply,
				msecs_to_jiffies(RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("BATT: ERROR: %s, vbatt deregister, rc=%d\n", __func__, rc);
		return rc;
	}

	if (reply.battery_error != BATTERY_DEREGISTRATION_SUCCESSFUL) {
		pr_err("BATT: ERROR: %s, vbatt %d deregistration, code=%d\n",
		       __func__, handler, reply.battery_error);
		return -EIO;
	}

	return 0;
}
#endif /* CONFIG_BATTERY_MSM_FAKE */

static int msm_battery_cleanup(void)
{
	int rc = 0;

	pr_debug("BATT: %s\n", __func__);

#ifndef CONFIG_BATTERY_MSM_FAKE
	if (msm_battery_info.charger_handler != INVALID_HANDLER) {
		rc = msm_battery_deregister(msm_battery_info.charger_handler);
		if (rc < 0)
			pr_err("BATT: ERROR %s, charger deregister, rc=%d\n",
			       __func__, rc);
	}
	msm_battery_info.charger_handler = INVALID_HANDLER;

	if (msm_battery_info.battery_handler != INVALID_HANDLER) {
		rc = msm_battery_deregister(msm_battery_info.battery_handler);
		if (rc < 0)
			pr_err("BATT: ERROR: %s, battery deregister, rc=%d\n",
			       __func__, rc);
	}
	msm_battery_info.battery_handler = INVALID_HANDLER;

	if(msm_battery_info.battery_queue)
		destroy_workqueue(msm_battery_info.battery_queue);

#if defined(CONFIG_BATTERY_EARLYSUSPEND) || defined(CONFIG_HAS_EARLYSUSPEND)
	if (msm_battery_info.early_suspend.suspend == msm_battery_early_suspend)
		unregister_early_suspend(&msm_battery_info.early_suspend);
#endif
#endif

	if (msm_battery_info.msm_psy_ac)
		power_supply_unregister(msm_battery_info.msm_psy_ac);
	if (msm_battery_info.msm_psy_usb)
		power_supply_unregister(msm_battery_info.msm_psy_usb);
	if (msm_battery_info.msm_psy_unknown)
		power_supply_unregister(msm_battery_info.msm_psy_unknown);
	if (msm_battery_info.msm_psy_battery)
		power_supply_unregister(msm_battery_info.msm_psy_battery);

#ifndef CONFIG_BATTERY_MSM_FAKE
	if (msm_battery_info.charger_endpoint) {
		rc = msm_rpc_close(msm_battery_info.charger_endpoint);
		if (rc < 0) {
			pr_err("BATT: ERROR: %s, charger rpc close, rc=%d\n",
			       __func__, rc);
		}
	}

	if (msm_battery_info.battery_client)
		msm_rpc_unregister_client(msm_battery_info.battery_client);
#endif

	return rc;
}

static u32 msm_battery_capacity(u32 current_voltage)
{
	u32 low_voltage = msm_battery_info.voltage_min_design;
	u32 high_voltage = msm_battery_info.voltage_max_design;

	if (current_voltage <= low_voltage)
		return 0;
	else if (current_voltage >= high_voltage)
		return 100;
	else
		return (current_voltage - low_voltage) * 100
			/ (high_voltage - low_voltage);
}

#ifndef CONFIG_BATTERY_MSM_FAKE
static int msm_battery_cb_func(struct msm_rpc_client *client,
			       void *buffer, int in_size)
{
	int rc = 0;
	struct rpc_request_hdr *request;
	u32 procedure;
	u32 accept_status;

	pr_debug("BATT: %s\n", __func__);

	request = (struct rpc_request_hdr *)buffer;
	procedure = be32_to_cpu(request->procedure);

	switch (procedure) {
	case BATTERY_CB_TYPE_PROC:
		accept_status = RPC_ACCEPTSTAT_SUCCESS;
		break;

	default:
		accept_status = RPC_ACCEPTSTAT_PROC_UNAVAIL;
		pr_err("BATT: ERROR: %s, procedure (%d) not supported\n",
		       __func__, procedure);
		break;
	}

	msm_rpc_start_accepted_reply(msm_battery_info.battery_client,
				     be32_to_cpu(request->xid), accept_status);

	rc = msm_rpc_send_accepted_reply(msm_battery_info.battery_client, 0);
	if (rc)
		pr_err("BATT: ERROR: %s, sending reply, rc=%d\n", __func__, rc);

	if (accept_status == RPC_ACCEPTSTAT_SUCCESS)
	{
		wake_lock_timeout(&msm_battery_info.charger_cb_wake_lock,
				  10 * HZ);
		msm_battery_update_psy_status();
	}

	return rc;
}
#endif  /* CONFIG_BATTERY_MSM_FAKE */

int msm_battery_fuel_register(struct msm_batt_gauge *batt)
{
	int rc = 0;
	if(!batt)
	{
		pr_err("BATT: ERROR: %s, null gauge pointer\n" ,__func__);
		return EIO;
	}

	msm_battery_info.fuel_gauge = 1;
	msm_battery_info.get_battery_mvolts = batt->get_battery_mvolts;
	msm_battery_info.get_battery_temperature = batt->get_battery_temperature;
	msm_battery_info.is_battery_present = batt->is_battery_present;
	msm_battery_info.is_battery_temp_within_range = batt->is_battery_temp_within_range;
	msm_battery_info.is_battery_id_valid = batt->is_battery_id_valid;
	msm_battery_info.get_battery_status = batt->get_battery_status;
	msm_battery_info.get_batt_remaining_capacity = batt->get_batt_remaining_capacity;

	pr_debug("BATT: %s\n", __func__);

	return rc;

}
EXPORT_SYMBOL(msm_battery_fuel_register);

void msm_battery_fuel_unregister(struct msm_batt_gauge* batt)
{
	if(!batt)
	{
		pr_err("BATT: ERROR: %s, null gauge pointer\n", __func__);
		return;
	}

	msm_battery_info.fuel_gauge = 0;
	msm_battery_info.get_battery_mvolts = NULL;
	msm_battery_info.get_battery_temperature = NULL;
	msm_battery_info.is_battery_present = NULL;
	msm_battery_info.is_battery_temp_within_range = NULL;
	msm_battery_info.is_battery_id_valid = NULL;
	msm_battery_info.get_battery_status = NULL;
	msm_battery_info.get_batt_remaining_capacity = NULL;

	pr_debug("BATT: %s\n", __func__);

	return;
}
EXPORT_SYMBOL(msm_battery_fuel_unregister);

#ifndef CONFIG_BATTERY_MSM_FAKE
static void msm_battery_worker(struct work_struct *work)
{
	msm_battery_update_psy_status();
}
#endif  /* CONFIG_BATTERY_MSM_FAKE */

#ifdef GPIO_CHG_STAT
static irqreturn_t chg_comp_int(int irq, void *dev_id)
{
	disable_irq_nosync(irq);
	using_irq ++;
	enable_irq(irq);
	return IRQ_HANDLED;
}
#endif
static int __devinit msm_battery_probe(struct platform_device *pdev)
{
	int rc;
	struct msm_psy_batt_pdata *pdata = pdev->dev.platform_data;

	if (pdev->id != -1) {
		dev_err(&pdev->dev,
			"BATT: ERROR: %s, msm chipsets only support one battery",
			__func__);
		return -EINVAL;
	}

	pr_debug("BATT: %s, enter\n", __func__);
#if defined(AP_CAL_BATT_CAP)
	wake_lock_init(&charger_lock, WAKE_LOCK_SUSPEND,"charger_lock");
#endif

	msm_battery_info.voltage_max_design = pdata->voltage_max_design;
	if (!msm_battery_info.voltage_max_design) {
		msm_battery_info.voltage_max_design = BATTERY_HIGH;
	}

	msm_battery_info.voltage_min_design = pdata->voltage_min_design;
	if (!msm_battery_info.voltage_min_design) {
		msm_battery_info.voltage_min_design = BATTERY_LOW;
	}

	msm_battery_info.battery_technology = pdata->batt_technology;
	if (!msm_battery_info.battery_technology) {
		msm_battery_info.battery_technology = POWER_SUPPLY_TECHNOLOGY_LION;
	}
#ifndef CONFIG_BATTERY_MSM_FAKE
	if (pdata->avail_chg_sources & AC_CHG) {
#else
	{
#endif
		rc = power_supply_register(&pdev->dev, &msm_psy_ac);
		if (rc < 0) {
			dev_err(&pdev->dev,
				"BATT: ERROR: %s, register msm_psy_ac, "
				"rc = %d\n", __func__, rc);
			msm_battery_cleanup();
			return rc;
		}
		msm_battery_info.available_charger_src |= AC_CHG;
		msm_battery_info.msm_psy_ac = &msm_psy_ac;
	}

	if (pdata->avail_chg_sources & UNKNOWN_CHG) {
		rc = power_supply_register(&pdev->dev, &msm_psy_unknown);
		if (rc < 0) {
			dev_err(&pdev->dev,
				"BATT: ERROR: %s, register msm_psy_unknown, "
				"rc = %d\n", __func__, rc);
			msm_battery_cleanup();
			return rc;
		}
		msm_battery_info.available_charger_src |= UNKNOWN_CHG;
		msm_battery_info.msm_psy_unknown = &msm_psy_unknown;
	}
	if (pdata->avail_chg_sources & USB_CHG) {
		rc = power_supply_register(&pdev->dev, &msm_psy_usb);
		if (rc < 0) {
			dev_err(&pdev->dev,
				"BATT: ERROR %s, register msm_psy_usb, "
				"rc = %d\n", __func__, rc);
			msm_battery_cleanup();
			return rc;
		}
		msm_battery_info.available_charger_src |= USB_CHG;
		msm_battery_info.msm_psy_usb = &msm_psy_usb;
	}

	if (!msm_battery_info.available_charger_src) {
		dev_err(&pdev->dev,
			"BATT: ERROR: %s, no power supply(ac or usb) "
			"is avilable\n", __func__);
		msm_battery_cleanup();
		return -ENODEV;
	}

	msm_battery_info.calculate_capacity = pdata->calculate_capacity;
	if (!msm_battery_info.calculate_capacity) {
		msm_battery_info.calculate_capacity = msm_battery_capacity;
	}

	rc = power_supply_register(&pdev->dev, &msm_psy_battery);
	if (rc < 0) {
		dev_err(&pdev->dev, "BATT: ERROR: %s, register battery "
			"rc=%d\n", __func__, rc);
		msm_battery_cleanup();
		return rc;
	}
	msm_battery_info.msm_psy_battery = &msm_psy_battery;

	msm_battery_info.current_psy = &msm_psy_battery;
	msm_battery_info.current_charger_src = 0;
	power_supply_changed(msm_battery_info.current_psy);

#ifndef CONFIG_BATTERY_MSM_FAKE
#if defined(CONFIG_BATTERY_EARLYSUSPEND) || defined(CONFIG_HAS_EARLYSUSPEND)
	msm_battery_info.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	msm_battery_info.early_suspend.suspend = msm_battery_early_suspend;
	msm_battery_info.early_suspend.resume = msm_battery_late_resume;
	msm_battery_info.is_suspended = false;
	mutex_init(&msm_battery_info.suspend_lock);
	register_early_suspend(&msm_battery_info.early_suspend);
#endif
	msm_battery_info.battery_queue = create_singlethread_workqueue(
			"battery_queue");
	if (!msm_battery_info.battery_queue) {
		dev_err(&pdev->dev, "BATT: ERROR: %s, create battey work queue\n",
			__func__);
		msm_battery_cleanup();
		return -ENOMEM;
	}
	INIT_DELAYED_WORK(&msm_battery_info.battery_work, msm_battery_worker);
	mutex_init(&msm_battery_info.update_mutex);
	rc = msm_battery_register(msm_battery_info.voltage_min_design,
				  BATTERY_VOLTAGE_BELOW_THIS_LEVEL,
				  BATTERY_CB_ID_LOW_V,
				  BATTERY_VOLTAGE_BELOW_THIS_LEVEL,
				  &msm_battery_info.battery_handler);
	if (rc < 0) {
		dev_err(&pdev->dev,
			"BATT: ERROR: %s, battery register, rc = %d\n",
			__func__, rc);
		msm_battery_cleanup();
		return rc;
	}

	rc = msm_battery_register(msm_battery_info.voltage_min_design,
				  VBATT_CHG_EVENTS,
				  BATTERY_CB_ID_CHG_EVT,
				  VBATT_CHG_EVENTS,
				  &msm_battery_info.charger_handler);
	if (rc < 0) {
		dev_err(&pdev->dev,
			"BATT: ERROR: %s, charger register, rc = %d\n",
			__func__, rc);
		msm_battery_cleanup();
		return rc;
	}
	wake_lock_init(&msm_battery_info.charger_cb_wake_lock, WAKE_LOCK_SUSPEND,
		       "msm_charger_cb");

	queue_delayed_work(msm_battery_info.battery_queue,
			   &msm_battery_info.battery_work,
			   0);

#if defined(CHARGER_ONLY_CASE)
	if (MSM_BOOT_CHARGER == current_boot_mode)
		printk("+++ BATT:boot mode is charger\n");
	else
		printk("+++ BATT:boot mode is normal\n");
	//wake_lock_init(&charger_only_wk, WAKE_LOCK_SUSPEND,"charger_only_lock");
	if ((CHARGER_TYPE_USB_WALL == msm_battery_info.charger_hardware) ) {
		if (MSM_BOOT_CHARGER == current_boot_mode) {
			//wake_lock(&charger_only_wk);
			//printk("++++ wake lock for charger only\n");
		}
	}
#endif
#else
	msm_battery_info.current_psy = &msm_psy_ac;
	msm_battery_info.current_charger_src = AC_CHG;
	power_supply_changed(msm_battery_info.current_psy);
#endif /* CONFIG_BATTERY_MSM_FAKE */
#if defined(AP_CAL_BATT_CAP)
#ifdef GPIO_CHG_STAT
	rc = gpio_request(GPIO_CHG_STAT, "chg_stat_irq");
	if (0 > rc)
		using_irq = 0;
	else
		using_irq = 1;

	if(1 == using_irq) {
		gpio_tlmm_config(GPIO_CFG(GPIO_CHG_STAT, 0, GPIO_CFG_INPUT,
	                GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		rc = request_irq(MSM_GPIO_TO_INT(GPIO_CHG_STAT), chg_comp_int, IRQF_TRIGGER_FALLING, "chg_comp_irq", NULL);
		if (rc < 0) {
			printk("BATT: request chg_comp_irq failed\n");
			using_irq = 0;
		}
	}
#endif
#endif
	pr_debug("BATT: %s, exit\n", __func__);

	return 0;
}

static int __devexit msm_battery_remove(struct platform_device *pdev)
{
	int rc = 0;

	wake_lock_destroy(&msm_battery_info.charger_cb_wake_lock);
	rc = msm_battery_cleanup();
	if (rc < 0) {
		dev_err(&pdev->dev,
			"BATT: ERROR: %s, battery cleanup, rc=%d\n",
			__func__, rc);
		return rc;
	}

	return 0;
}

static struct platform_driver msm_batt_driver = {
	.probe  = msm_battery_probe,
	.remove = __devexit_p(msm_battery_remove),
	.driver = {
		   .name = "msm-battery",
		   .owner = THIS_MODULE,
#if defined(CONFIG_PM) && defined(AP_CAL_BATT_CAP) 
		   .pm = &batt_dev_pm_ops,
#endif
		   },
};

static int __devinit msm_battery_init_rpc(void)
{
	int rc = 0;

#ifdef CONFIG_BATTERY_MSM_FAKE
	pr_debug("BATT: CAUTION: fake msm battery\n");
#else
	msm_battery_info.charger_endpoint =
		msm_rpc_connect_compatible(CHG_RPC_PROG, CHG_RPC_VER_1_1, 0);
	if (msm_battery_info.charger_endpoint == NULL) {
		pr_err("BATT: ERROR: %s, rpc charger no server\n", __func__);
		return -ENODEV;
	}
	if (IS_ERR(msm_battery_info.charger_endpoint)) {
		rc = PTR_ERR(msm_battery_info.charger_endpoint);
		pr_err("BATT: ERROR: %s, rpc charger not connect, rc=%d\n",
		       __func__, rc);
		msm_battery_info.charger_endpoint = NULL;
		return -ENODEV;
	}
	msm_battery_info.charger_api_version = CHG_RPC_VER_1_1;

	msm_battery_info.battery_client = msm_rpc_register_client(
			"battery", BATTERY_RPC_PROG, BATTERY_RPC_VER_5_1, 1,
			msm_battery_cb_func);
	if (msm_battery_info.battery_client == NULL) {
		pr_err("BATT: ERROR: %s, rpc battery no client\n", __func__);
		return -ENODEV;
	}
	if (IS_ERR(msm_battery_info.battery_client)) {
		rc = PTR_ERR(msm_battery_info.battery_client);
		pr_err("BATT: ERROR: %s, rpc battery not register, rc=%d\n",
		       __func__, rc);
		msm_battery_info.battery_client = NULL;
		return -ENODEV;
	}
	msm_battery_info.battery_api_version = BATTERY_RPC_VER_5_1;
#endif /* CONFIG_BATTERY_MSM_FAKE */

	pr_debug("BATT: %s, rpc version charger=0x%08x, battery=0x%08x\n",
		__func__, msm_battery_info.charger_api_version,
		msm_battery_info.battery_api_version);

	return 0;
}

static int __init msm_battery_init(void)
{
	int rc = 0;

	pr_debug("BATT: %s, enter\n", __func__);

	rc = msm_battery_init_rpc();
	if (rc < 0) {
		msm_battery_cleanup();
		return rc;
	}

	rc = platform_driver_register(&msm_batt_driver);
	if (rc < 0) {
		pr_err("BATT: ERROR: %s, platform_driver_register, rc=%d\n",
		       __func__, rc);
	}

	pr_debug("BATT: %s, exit\n", __func__);
	return 0;
}

static void __exit msm_battery_exit(void)
{
	platform_driver_unregister(&msm_batt_driver);
}

module_init(msm_battery_init);
module_exit(msm_battery_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Kiran Kandi, Qualcomm Innovation Center, Inc.");
MODULE_DESCRIPTION("Battery driver for Qualcomm MSM chipsets.");
MODULE_VERSION("2.0");
MODULE_ALIAS("platform:msm_battery");
