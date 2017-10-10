#ifdef CONFIG_COMPAT

#include <linux/fs.h>
#include <linux/compat.h>

#endif
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
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>
// <<< 2015/11/23-youchihwang. Porting flash light driver LM3644TTYFFR.
#include <mt-plat/mt_gpio.h>
#include <mach/gpio_const.h>
// >>> 2015/11/23-youchihwang. Porting flash light driver LM3644TTYFFR.


/*
// flash current vs index
    0       1      2       3    4       5      6       7    8       9     10
93.74  140.63  187.5  281.25  375  468.75  562.5  656.25  750  843.75  937.5
     11    12       13      14       15    16
1031.25  1125  1218.75  1312.5  1406.25  1500mA
*/
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

#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)


#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#else
#define PK_DBG(a, ...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */


static u32 strobe_Res;
static u32 strobe_Timeus;
static BOOL g_strobe_On;

static int gDuty = -1;
static int g_timeOutTimeMs;

static DEFINE_MUTEX(g_strobeSem);





static struct work_struct workTimeOut;

/* #define FLASH_GPIO_ENF GPIO12 */
/* #define FLASH_GPIO_ENT GPIO13 */


// <<< 2015/12/07-youchihwang. setting rear flashlight flash current setting
static int gIsTorch[18] = {   1,    0,    0,    0, 0, 0, 0, 0, 0, 0, 0, 0,  0,  0,  0,  0,  0,  0};
static int gLedDuty[18] = {0x23, 0x1c, 0x32, 0x54, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
// >>> 2015/12/07-youchihwang. setting rear flashlight flash current setting

/* current(mA) 50,94,141,188,281,375,469,563,656,750,844,938,1031,1125,1220,1313,1406,1500 */



/*****************************************************************************
Functions
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data);

//<2015/09/11-youchihwang. Porting flash light driver LM3644TTYFFR. Extern RT4505_i2c_client to kd_flashlightlist.c
struct i2c_client *RT4505_i2c_client;
//>2015/09/11-youchihwang. Porting flash light driver LM3644TTYFFR. Extern RT4505_i2c_client to kd_flashlightlist.c



struct RT4505_platform_data {
	u8 torch_pin_enable;	/* 1:  TX1/TORCH pin isa hardware TORCH enable */
	u8 pam_sync_pin_enable;	/* 1:  TX2 Mode The ENVM/TX2 is a PAM Sync. on input */
	u8 thermal_comp_mode_enable;	/* 1: LEDI/NTC pin in Thermal Comparator Mode */
	u8 strobe_pin_disable;	/* 1 : STROBE Input disabled */
	u8 vout_mode_enable;	/* 1 : Voltage Out Mode enable */
};

struct RT4505_chip_data {
	struct i2c_client *client;

	/* struct led_classdev cdev_flash; */
	/* struct led_classdev cdev_torch; */
	/* struct led_classdev cdev_indicator; */

	struct RT4505_platform_data *pdata;
	struct mutex lock;

	u8 last_flag;
	u8 no_pdata;
};

//<2015/09/13-youchihwang. Porting flash light driver LM3644TTYFFR. Extern RT4505_write_reg to kd_flashlightlist.c
int RT4505_write_reg(struct i2c_client *client, u8 reg, u8 val)
//>2015/09/13-youchihwang. Porting flash light driver LM3644TTYFFR. Extern RT4505_write_reg to kd_flashlightlist.c
{
	int ret = 0;
	struct RT4505_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		PK_DBG("failed writing at 0x%02x\n", reg);
	return ret;
}

static int RT4505_read_reg(struct i2c_client *client, u8 reg)
{
	int val = 0;
	struct RT4505_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);


	return val;
}



/* ========================= */




static int RT4505_chip_init(struct RT4505_chip_data *chip)
{
        //<2015/09/15-youchihwang. Porting flash light driver LM3644TTYFFR. flashlight driver init
        u8 chip_register_address = 0x00;
        u8 chip_register_value   = 0x00;		
		
        PK_DBG ("LM3644 chip init +\n");
        
        //
        // set gpio pin of torch
        //
        mt_set_gpio_mode (GPIO82, GPIO_MODE_GPIO);
        mt_set_gpio_dir  (GPIO82, GPIO_DIR_OUT);
        mt_set_gpio_out  (GPIO82, GPIO_OUT_ZERO);
        
        
        //
        // set gpio pin of strobe
        //
        mt_set_gpio_mode (GPIO80, GPIO_MODE_GPIO);
        mt_set_gpio_dir  (GPIO80, GPIO_DIR_OUT);
        mt_set_gpio_out  (GPIO80, GPIO_OUT_ZERO);
        
        
        //
        // set gpio pin of tx
        //
        mt_set_gpio_mode (GPIO81, GPIO_MODE_GPIO);
        mt_set_gpio_dir  (GPIO81, GPIO_DIR_OUT);
        mt_set_gpio_out  (GPIO81, GPIO_OUT_ZERO);

                
        //
        // set gpio pin of hwen, hardware pin
        //
        mt_set_gpio_mode (GPIO86, GPIO_MODE_GPIO);
        mt_set_gpio_dir  (GPIO86, GPIO_DIR_OUT);
        mt_set_gpio_out  (GPIO86, GPIO_OUT_ONE);
        
        
        //
        // Timing Configuration Register (0x08)
        // bit07         : RFU
        // bit06 ~ bit04 : Torch Current Ramp Time
        // bit03 ~ bit00 : Flash Time-Out Duration
        //
        
        chip_register_address = 0x08;
        // <<< 2015/10/21-youchihwang. Camera Hardware Requirement, Setting front flash time-out duration to 600ms
        chip_register_value   = 0x1A;    // Torch Current Ramp Time : 1 ms // Flash Time-Out Duration : 600 ms
        // >>> 2015/10/21-youchihwang. Camera Hardware Requirement, Setting front flash time-out duration to 600ms
        RT4505_write_reg (chip->client, chip_register_address, chip_register_value);


        //
        // front LED flash brightness set
        // LED1 Flash Brightness Register (0x03)
        //
        // Bit 07      : LED2 Flash Current Override
        // Bit 06 ~ 00 : LED1 Flash Brightness Level
        //
        chip_register_address = 0x03;
        // <<< 2015/10/21-youchihwang. Camera Hardware Requirement, Setting front flash mode current to 397.825 mA
        chip_register_value   = 0x21;  // flash current = 397.825 mA = ((Brightness Code * 11.725 mA) + 10.9 mA)
        // >>> 2015/10/21-youchihwang. Camera Hardware Requirement, Setting front flash mode current to 397.825 mA
        RT4505_write_reg (chip->client, chip_register_address, chip_register_value);


        //
        // front LED torch brightness set
        // LED1 Torch Brightness Register (0x05)
        //
        // Bit 07      : LED2 Torch Current Override
        // Bit 06 ~ 00 : LED1 Torch Brightn e s s Le vels
        //
        chip_register_address = 0x05;
        // <<< 2015/10/21-youchihwang. Camera Hardware Requirement, Setting front torch mode current to 99.954 mA
        chip_register_value   = 0x23;  // torch current = 99.954 mA = (Brightness Code x 2.8 mA) + 1.954 mA
        // >>> 2015/10/21-youchihwang. Camera Hardware Requirement, Setting front torch mode current to 99.954 mA
        RT4505_write_reg (chip->client, chip_register_address, chip_register_value);


        //
        // rear LED flash brightness set
        // LED2 Flash Brightness Register (0x04)
        //
        // Bit 07      : RFU
        // Bit 06 ~ 00 : LED2 Flash Brightness Levels
        //
        chip_register_address = 0x04;
        // <<< 2015/10/21-youchihwang. Camera Hardware Requirement, Setting rear flash mode current to 995.8 mA
        chip_register_value   = 0x54;  // flash current = 902 mA = (Brightness Code x 11.725 mA) + 10.9 mA
        // >>> 2015/10/21-youchihwang. Camera Hardware Requirement, Setting rear flash mode current to 995.8 mA
        RT4505_write_reg (chip->client, chip_register_address, chip_register_value);


        //
        // rear LED torch brightness set
        // LED2 Torch Brightness Register (0x06)
        //
        // Bit 07      : RFU
        // Bit 06 ~ 00 : LED2 Torch Brightness Levels
        //
        chip_register_address = 0x06;
        // <<< 2015/10/21-youchihwang. Camera Hardware Requirement, Setting rear torch mode current to 99.954 mA
        chip_register_value   = 0x23;  // torch current = 99.954 mA = (Brightness Code x 2.8 mA) + 1.954 mA
        // >>> 2015/10/21-youchihwang. Camera Hardware Requirement, Setting rear torch mode current to 99.954 mA
        RT4505_write_reg (chip->client, chip_register_address, chip_register_value);

        
        PK_DBG ("LM3644 chip init -\n");
        //>2015/09/15-youchihwang. Porting flash light driver LM3644TTYFFR. flashlight driver init

	return 0;
}

static int RT4505_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct RT4505_chip_data *chip;
	struct RT4505_platform_data *pdata = client->dev.platform_data;

	int err = -1;

	PK_DBG("RT4505_probe start--->.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		PK_DBG("RT4505 i2c functionality check fail.\n");
		return err;
	}

	chip = kzalloc(sizeof(struct RT4505_chip_data), GFP_KERNEL);
	chip->client = client;

	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	if (pdata == NULL) {	/* values are set to Zero. */
		PK_DBG("RT4505 Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct RT4505_platform_data), GFP_KERNEL);
		chip->pdata = pdata;
		chip->no_pdata = 1;
	}

	chip->pdata = pdata;
	if (RT4505_chip_init(chip) < 0)
		goto err_chip_init;

	RT4505_i2c_client = client;
	PK_DBG("RT4505 Initializing is done\n");

	return 0;

err_chip_init:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	PK_DBG("RT4505 probe is failed\n");
	return -ENODEV;
}

static int RT4505_remove(struct i2c_client *client)
{
	struct RT4505_chip_data *chip = i2c_get_clientdata(client);

	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);
	return 0;
}


#define RT4505_NAME "leds-RT4505"
static const struct i2c_device_id RT4505_id[] = {
	{RT4505_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id RT4505_of_match[] = {
	{.compatible = "mediatek,strobe_main"},
	{},
};
#endif

static struct i2c_driver RT4505_i2c_driver = {
	.driver = {
		   .name = RT4505_NAME,
#ifdef CONFIG_OF
		   .of_match_table = RT4505_of_match,
#endif
		   },
	.probe = RT4505_probe,
	.remove = RT4505_remove,
	.id_table = RT4505_id,
};


//<2015/09/13-youchihwang. Porting flash light driver LM3644TTYFFR. create flashlight driver platform data
struct RT4505_platform_data RT4505_pdata = {0, 0, 0, 0, 0};
static struct i2c_board_info __initdata i2c_RT4505 = {
        .type          = RT4505_NAME,
        .addr          = 0x63,
        .platform_data = &RT4505_pdata,
};
//>2015/09/13-youchihwang. Porting flash light driver LM3644TTYFFR. create flashlight driver platform data

static int __init RT4505_init(void)
{
        int Status = 0;

        PK_DBG("RT4505_init +\n");
        
        //<2015/09/13-youchihwang. Porting flash light driver LM3644TTYFFR. register lm3644 to i2c
        i2c_register_board_info (1, &i2c_RT4505, 1);

        Status = i2c_add_driver(&RT4505_i2c_driver);
        if (Status == 0) {
                printk ("[flashlight] lm3644 add i2c driver success");
                PK_DBG("RT4505_init -\n");
                return 0;
        } else {
                printk ("[flashlight] lm3644 add i2c driver fail");
                PK_DBG("RT4505_init -\n");
                return Status;
        }
        //>2015/09/13-youchihwang. Porting flash light driver LM3644TTYFFR. register lm3644 to i2c
}

static void __exit RT4505_exit(void)
{
	i2c_del_driver(&RT4505_i2c_driver);
}


module_init(RT4505_init);
module_exit(RT4505_exit);

MODULE_DESCRIPTION("Flash driver for RT4505");
MODULE_AUTHOR("pw <pengwei@mediatek.com>");
MODULE_LICENSE("GPL v2");

int readReg(int reg)
{

	int val;

	val = RT4505_read_reg(RT4505_i2c_client, reg);
	return (int)val;
}

int FL_Enable(void)
{
        // <<< 2015/10/20-youchihwang. Porting flash light driver LM3644TTYFFR. front flashlight enable function
        u8 chip_register_address = 0x00;
        u8 chip_register_value   = 0x00;

        if (gIsTorch[gDuty] == 1) {
                chip_register_address = 0x01;
                chip_register_value   = 0x0A;
        }
        else {
                chip_register_address = 0x01;
                chip_register_value   = 0x0E;
        }
        RT4505_write_reg(RT4505_i2c_client, chip_register_address, chip_register_value);
        // >>> 2015/10/20-youchihwang. Porting flash light driver LM3644TTYFFR. front flashlight enable function
        PK_DBG(" FL_Enable line=%d\n", __LINE__);
        return 0;
}



int FL_Disable(void)
{
        // <<< 2015/10/20-youchihwang. Porting flash light driver LM3644TTYFFR. front flashlight disable function.
        u8 chip_register_address = 0x00;
        u8 chip_register_value   = 0x00;
        
        chip_register_address = 0x01;
        chip_register_value   = 0x00;
        RT4505_write_reg(RT4505_i2c_client, chip_register_address, chip_register_value);
        // >>> 2015/10/20-youchihwang. Porting flash light driver LM3644TTYFFR. front flashlight disable function.
        PK_DBG(" FL_Disable line=%d\n", __LINE__);
        return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
        // <<< 2015/10/20-youchihwang. Porting flash light driver LM3644TTYFFR. front flashlight disable function.
        u8 chip_register_address = 0x00;
        u8 chip_register_value   = 0x00;

        if (duty > 17)
                duty = 17;
        if (duty < 0)
                duty = 0;
        
        gDuty = duty;

        if (gIsTorch[gDuty] == 1) {
                chip_register_address = 0x06;
                chip_register_value   = gLedDuty[gDuty];
        }
        else {
                chip_register_address = 0x04;
                chip_register_value   = gLedDuty[gDuty];
        }
        
        // >>> 2015/10/20-youchihwang. Porting flash light driver LM3644TTYFFR. front flashlight disable function.
        RT4505_write_reg(RT4505_i2c_client, chip_register_address, chip_register_value);
        PK_DBG(" FL_dim_duty line=%d\n", __LINE__);
        return 0;
}




int FL_Init(void)
{
        // <<< 2015/10/20-youchihwang. Porting flash light driver LM3644TTYFFR. FL_Init ()
        FL_Disable ();
        // <<< 2015/10/20-youchihwang. Porting flash light driver LM3644TTYFFR. FL_Init ()
        PK_DBG(" FL_Init line=%d\n", __LINE__);
        return 0;
}


int FL_Uninit(void)
{
	FL_Disable();
	return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable();
	PK_DBG("ledTimeOut_callback\n");
}



enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}

static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
	static int init_flag;

	if (init_flag == 0) {
		init_flag = 1;
		INIT_WORK(&workTimeOut, work_timeOutFunc);
		g_timeOutTimeMs = 1000;
		hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		g_timeOutTimer.function = ledTimeOutCallback;
	}
}



static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;

	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));
/*	PK_DBG
	    ("RT4505 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",
	     __LINE__, ior_shift, iow_shift, iowr_shift, (int)arg);
*/
	switch (cmd) {

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", (int)arg);
		g_timeOutTimeMs = arg;
		break;


	case FLASH_IOC_SET_DUTY:
		PK_DBG("FLASHLIGHT_DUTY: %d\n", (int)arg);
		FL_dim_duty(arg);
		break;


	case FLASH_IOC_SET_STEP:
		PK_DBG("FLASH_IOC_SET_STEP: %d\n", (int)arg);

		break;

	case FLASH_IOC_SET_ONOFF:
		PK_DBG("FLASHLIGHT_ONOFF: %d\n", (int)arg);
		if (arg == 1) {
			if (g_timeOutTimeMs != 0) {
				ktime_t ktime;

				ktime = ktime_set(0, g_timeOutTimeMs * 1000000);
				hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
			}
			FL_Enable();
		} else {
			FL_Disable();
			hrtimer_cancel(&g_timeOutTimer);
		}
		break;
	default:
		PK_DBG(" No such command\n");
		i4RetValue = -EPERM;
		break;
	}
	return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
	int i4RetValue = 0;

	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res) {
		FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


	if (strobe_Res) {
		PK_DBG(" busy!\n");
		i4RetValue = -EBUSY;
	} else {
		strobe_Res += 1;
	}


	spin_unlock_irq(&g_strobeSMPLock);
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
	PK_DBG(" constant_flashlight_release\n");

	if (strobe_Res) {
		spin_lock_irq(&g_strobeSMPLock);

		strobe_Res = 0;
		strobe_Timeus = 0;

		/* LED On Status */
		g_strobe_On = FALSE;

		spin_unlock_irq(&g_strobeSMPLock);

		FL_Uninit();
	}

	PK_DBG(" Done\n");

	return 0;

}


FLASHLIGHT_FUNCTION_STRUCT constantFlashlightFunc = {
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &constantFlashlightFunc;
	return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

	return 0;
}
EXPORT_SYMBOL(strobe_VDIrq);
