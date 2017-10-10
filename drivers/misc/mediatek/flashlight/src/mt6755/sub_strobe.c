
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#ifdef CONFIG_COMPAT
#include <linux/fs.h>
#include <linux/compat.h>
#endif
#include "kd_flashlight.h"
/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */
#define TAG_NAME "[sub_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_WARN(fmt, arg...)        pr_warn(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_NOTICE(fmt, arg...)      pr_notice(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_INFO(fmt, arg...)        pr_info(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_TRC_FUNC(f)              pr_debug(TAG_NAME "<%s>\n", __func__)
#define PK_TRC_VERBOSE(fmt, arg...) pr_debug(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       pr_err(TAG_NAME "%s: " fmt, __func__ , ##arg)

#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#define PK_VER PK_TRC_VERBOSE
#define PK_ERR PK_ERROR
#else
#define PK_DBG(a, ...)
#define PK_VER(a, ...)
#define PK_ERR(a, ...)
#endif

// <<< 2015/10/21-youchihwang. Porting flash light driver LM3644TTYFFR. front flash

//
// 0 : sub strobe isn't used.
// 1 : sub strobe is used.
//
static int strobe_Res = 0;
static int gDuty = 0;
static int g_timeOutTimeMs = 0;
static struct hrtimer g_timeOutTimer;
static DEFINE_SPINLOCK (g_strobeSMPLock); /* cotta-- SMP proection */
static struct work_struct workTimeOut;

extern struct i2c_client *RT4505_i2c_client;
extern int RT4505_write_reg(struct i2c_client *client, U8 reg, U8 val);
extern int RT4505_read_reg(struct i2c_client *client, U8 reg,U8 *val);

static int FL_Sub_Disable(void);

// <<< 2015/12/09-youchihwang. front flashlight flash & torch mode current setting
static int gIsTorch[18] = {   1,    0,    0,    0, 0, 0, 0, 0, 0, 0, 0, 0,  0,  0,  0,  0,  0,  0};
static int gLedDuty[18] = {0x23, 0x10, 0x18, 0x21, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
// >>> 2015/12/09-youchihwang. front flashlight flash & torch mode current setting


static void work_timeOutFunc (struct work_struct *data)
{
        FL_Sub_Disable ();
        PK_DBG("ledTimeOut_callback\n");
}


enum hrtimer_restart Sub_ledTimeOutCallback (struct hrtimer *timer)
{
        schedule_work (&workTimeOut);

        return HRTIMER_NORESTART;
}


void Sub_timerInit (void)
{
        INIT_WORK (&workTimeOut, work_timeOutFunc);
        g_timeOutTimeMs = 1000; //1s
        hrtimer_init (&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        g_timeOutTimer.function = Sub_ledTimeOutCallback;
}


static int FL_Sub_Enable (void)
{
        U8 chip_register_address = 0x00;
        U8 chip_register_value   = 0x00;
		
		PK_DBG ("FL_Sub_Enable () + \n");

        if (gIsTorch[gDuty] == 1){
                chip_register_address = 0x01;
                chip_register_value   = 0x09;
        }
        else {
                chip_register_address = 0x01;
                chip_register_value   = 0x0D;
        }
        RT4505_write_reg (RT4505_i2c_client, chip_register_address, chip_register_value);
        PK_DBG ("FL_Sub_Enable () - \n");

        return 0;
}


static int FL_Sub_Disable(void)
{
        U8 chip_register_address = 0x00;
        U8 chip_register_value   = 0x00;
		
		PK_DBG ("FL_Sub_Disable () + \n");

        chip_register_address = 0x01;
        chip_register_value   = 0x00;
        RT4505_write_reg (RT4505_i2c_client, chip_register_address, chip_register_value);
        PK_DBG ("FL_Sub_Disable () - \n");
        
        return 0;
}


static int FL_Sub_dim_duty (unsigned long duty)
{
        U8 chip_register_address = 0x00;
        U8 chip_register_value   = 0x00;

        PK_DBG ("FL_Sub_dim_duty () + \n");

        if (duty > 17)
                duty = 17;
        if (duty < 0)
                duty = 0;

        gDuty = duty;

        if (gIsTorch[gDuty] == 1) {
                chip_register_address = 0x05;
                chip_register_value   = gLedDuty[duty];
        }
        else {
                chip_register_address = 0x03;
                chip_register_value   = gLedDuty[duty];
        }
        RT4505_write_reg (RT4505_i2c_client, chip_register_address, chip_register_value);

        PK_DBG ("FL_Sub_dim_duty () + \n");

        return 0;
}


static int FL_Sub_Init (void)
{
        PK_DBG ("FL_Sub_Init () +\n");
        FL_Sub_Disable ();
        PK_DBG ("FL_Sub_Init () -\n");
        
        return 0;
}


static int FL_Sub_Uninit (void)
{
        PK_DBG ("FL_Sub_Uninit () +\n");
        FL_Sub_Disable ();
        PK_DBG ("FL_Sub_Uninit () -\n");

        return 0;
}


static int sub_strobe_ioctl(unsigned int cmd, unsigned long arg)
{
        // <<< 2016/03/24-youchihwang. flashlight. modify for response value real time not previous state
        int return_value = 0;
		// >>> 2016/03/24-youchihwang. flashlight. modify for response value real time not previous state

        PK_DBG ("sub_strobe_ioctl () + \n");

        switch (cmd)
        {
                case FLASH_IOC_SET_TIME_OUT_TIME_MS :
                        PK_DBG ("sub flash FLASH_IOC_SET_TIME_OUT_TIME_MS command argument : %d\n", (unsigned int)arg);
                        g_timeOutTimeMs = arg;
                        break;

                case FLASH_IOC_SET_DUTY :
                        PK_DBG ("sub flash FLASH_IOC_SET_DUTY command argument : %d\n", (unsigned int)arg);
                        FL_Sub_dim_duty (arg);
                        break;

                case FLASH_IOC_SET_ONOFF :
                        PK_DBG ("FLASH_IOC_SET_ONOFF command argument : %d\n", (unsigned int)arg);
                        if (arg == 1) {
                                if (g_timeOutTimeMs != 0) {
                                        ktime_t ktime;
                                        ktime = ktime_set (0, g_timeOutTimeMs * 1000000);
                                        hrtimer_start (&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
                                }
                                FL_Sub_Enable ();
                        }
                        else {
                                FL_Sub_Disable ();
                                hrtimer_cancel (&g_timeOutTimer);
                        }
                        break;
                        
                default :
                        PK_DBG (" sub flash doesn't have such command\n");
                        return_value = -EPERM;
                        break;
        }

        PK_DBG ("sub_strobe_ioctl () - \n");

        return return_value;
}


static int sub_strobe_open(void *pArg)
{
        // <<< 2016/03/24-youchihwang. flashlight. modify for response value real time not previous state
        int return_value = 0;
		// >>> 2016/03/24-youchihwang. flashlight. modify for response value real time not previous state

        PK_DBG ("sub_strobe_open () + \n");
        
        if (strobe_Res == 0) {
                FL_Sub_Init ();
                Sub_timerInit ();
        }
        
        spin_lock_irq (&g_strobeSMPLock);
        
        if (strobe_Res)
                return_value = -EBUSY;
        else
                strobe_Res++;
        
        spin_unlock_irq (&g_strobeSMPLock);
        
        PK_DBG ("sub_strobe_open () - \n");

        return return_value;
}


static int sub_strobe_release(void *pArg)
{
        PK_DBG ("sub_strobe_release () + \n");

        if (strobe_Res) {
                spin_lock_irq (&g_strobeSMPLock);
                strobe_Res--;
                spin_unlock_irq (&g_strobeSMPLock);
                FL_Sub_Uninit ();
        }
        
        PK_DBG ("sub_strobe_release () - \n");
        
        return 0;
}
// >>> 2015/10/21-youchihwang. Porting flash light driver LM3644TTYFFR. front flash


FLASHLIGHT_FUNCTION_STRUCT subStrobeFunc = {
	sub_strobe_open,
	sub_strobe_release,
	sub_strobe_ioctl
};


MUINT32 subStrobeInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &subStrobeFunc;
	return 0;
}
