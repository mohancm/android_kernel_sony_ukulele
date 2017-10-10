/* touchscreen/ektf2k_kthread_mtk.c - ELAN EKTF2K touchscreen driver
 * for MTK65xx serial platform.
 *
 * Copyright (C) 2012 Elan Microelectronics Corporation.
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
 * 2011/12/06: The first release, version 0x0001
 * 2012/2/15:  The second release, version 0x0002 for new bootcode
 * 2012/5/8:   Release version 0x0003 for china market
 *             Integrated 2 and 5 fingers driver code together and
 *             auto-mapping resolution.
 * 2012/8/24:  MTK version
 * 2013/2/1:   Release for MTK6589/6577/6575/6573/6513 Platform
 *             For MTK6575/6573/6513, please disable both of ELAN_MTK6577 and MTK6589DMA.
 *                          It will use 8+8+2 received packet protocol
 *             For MTK6577, please enable ELAN_MTK6577 and disable MTK6589DMA.
 *                          It will use Elan standard protocol (18bytes of protocol).
 *             For MTK6589, please enable both of ELAN_MTK6577 and MTK6589DMA.
 * 2013/5/15   Fixed MTK6589_DMA issue.
 */
//#define SOFTKEY_AXIS_VER
//#define ELAN_TEN_FINGERS
//#define   MTK6589_DMA
//#define   ELAN_MTK6577
#define   MTK6755_DMA
#define   ELAN_MTK6755
#define ELAN_BUTTON //[SM20]zihweishen modify protocol_A for report touch error 2015/12/25
//#define TPD_HAVE_BUTTON
#define ELAN_TEN_FINGERS	// Thunder Open flag for Ten fingers flag 20141229 

#if defined( MTK6589_DMA ) || defined( MTK6755_DMA )
#define   ELAN_DMA_MODE
#endif

#ifdef ELAN_TEN_FINGERS
#define   PACKET_SIZE       44    /* support 10 fingers packet */
#else
//<<Mel - 4/10, Modify firmware support 2 fingers.
//#define   PACKET_SIZE       8     /* support 2 fingers packet  */
#define PACKET_SIZE       18    /* support 5 fingers packet  */
//>>Mel - 4/10, Modify firmware support 2 fingers.
#endif

//[SM20]zihweishen modify protocol_B for report error coordinate begin 2015/11/23
#if 1
#include <linux/module.h>
#define PROTOCOL_A //[SM20]zihweishen modify protocol_A for report touch error 2015/12/25

#ifdef PROTOCOL_B
#include <linux/input/mt.h>
#endif
#ifdef PROTOCOL_A
#include <linux/input.h>
#endif
//[SM20]zihweishen modify protocol_B for report error coordinate end 2015/11/23

#include <linux/interrupt.h>
//#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/hrtimer.h>
#endif

#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/suspend.h>
#include <linux/delay.h>
#include <linux/rtpm_prio.h>
#include <linux/kthread.h>
#include <linux/gpio.h>

/* DMA */
#include <linux/dma-mapping.h> 

#ifdef CONFIG_MTK_BOOT
#include "mt_boot_common.h"
#endif

#include "tpd.h"
#include "ektf2k_mtk.h"
#include <linux/miscdevice.h>
#include <linux/hrtimer.h>
#include <linux/cdev.h>
#include <linux/slab.h>
//#include <linux/uaccess.h>
#include <asm/uaccess.h>
#include <asm/ioctl.h>

//#include <mach/mt_pm_ldo.h>
//#include <mach/mt_typedefs.h>
//#include <mach/mt_boot.h>


//[SM20]zihweishen modify protocol_B for report error coordinate begin 2015/11/23
#if 1
/* For linux 2.6.36.3 */
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/ioctl.h>
#endif

#define   ELAN_DEBUG
#define   MAX_FINGER_SIZE		255
//[SM20]zihweishen modify protocol_B for report error coordinate end 2015/11/23
#define   I2C_ELAN_DEV_ADDR     (0x10)  //(0x15)  //(0x5D)
#define   I2C_ELAN_SLAVE_ADDR   (I2C_ELAN_DEV_ADDR<<1)  /* 0x2A(0x15<<1), 0x40(0x20<<1), 0xBA(0x5D<<1) */
#define   I2C_ELAN_BUS          0 //I2C0

#define   PWR_STATE_DEEP_SLEEP  0
#define   PWR_STATE_NORMAL      1
#define   PWR_STATE_MASK        BIT(3)

#define   CMD_S_PKT             (0x52)
#define   CMD_R_PKT             (0x53)
#define   CMD_W_PKT             (0x54)

#define   HELLO_PKT             (0x55)
#define   FIVE_FINGERS_PKT      (0x5D)

#define   TWO_FINGERS_PKT       (0x5A)
#define   TEN_FINGERS_PKT       (0x62)
#define   MTK_FINGERS_PKT       (0x6D)  /** 2 Fingers: 5A 5 Fingers 5D, 10 Fingers: 62 **/
//[SM20]zihweishen modify protocol_B for report error coordinate begin 2015/11/23
#define   IamLive_PKT		         0x78

#define   RESET_PKT             (0x77)
#define   CALIB_PKT             (0xA8)

#define ELAN_X_MAX 	 1664  //recover mode use
#define ELAN_Y_MAX	 2304  //recover mode use
#define LCM_X_MAX	   1080
#define LCM_Y_MAX	   1920
//[SM20]zihweishen modify protocol_B for report error coordinate end 2015/11/23

#define   TPD_OK                0
//#define HAVE_TOUCH_KEY

#define   LCT_VIRTUAL_KEY

#if defined( ELAN_DMA_MODE )
static uint8_t  * gpDMABuf_va = NULL;
//static uint32_t   gpDMABuf_pa = NULL;
//static uint32_t   gpDMABuf_pa = 0;
static dma_addr_t gpDMABuf_pa=0;
#endif

/** 0604 add -start **/
//#define ESD_CHECK
#if defined( ESD_CHECK )
  static int    have_interrupts = 0;
  static struct workqueue_struct *esd_wq = NULL;
  static struct delayed_work      esd_work;
  static unsigned long  delay = 2*HZ;

//declare function
  static void elan_touch_esd_func(struct work_struct *work);
#endif
/** 0604 add -end **/

unsigned int touch_irq = 0;

#if defined( TPD_HAVE_BUTTON )
#define   TPD_KEY_COUNT         3
#define   TPD_KEYS              { KEY_MENU, KEY_HOMEPAGE, KEY_BACK }
#define   TPD_KEYS_DIM          {{ 107, 1370, 109, TPD_BUTTON_HEIGH }, \
                                 { 365, 1370, 109, TPD_BUTTON_HEIGH }, \
                                 { 617, 1370, 102, TPD_BUTTON_HEIGH }}

static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif /* End.. (TPD_HAVE_BUTTON) */

//static int gPrint_point = 0;////[SM20]zihweishen modify protocol_B for report error coordinate 2015/11/23
//<<Mel - 4/10, Add pin define.
#define   CUST_EINT_POLARITY_LOW            0
#define   CUST_EINT_TOUCH_PANEL_SENSITIVE   1
//Mel - 4/10, Add pin define>>.

// modify
#define   SYSTEM_RESET_PIN_SR   135

//Add these Define

//<2014/13/44-Yuting Shih. Masked for compile error if non-firmware.
#define   IAP_PORTION           1   //1	//If as ture, Open Auto update when Handset power on 20141229
//>2014/13/44-Yuting Shih.
#define   PAGERETRY             30
#define   IAPRESTART            5
#define   CMD_54001234          0 

//[SM20]zihweishen modify protocol_B for report error coordinate begin 2015/11/23
#define NO_DEBUG       0
#define DEBUG_ERROR  1
#define DEBUG_INFO     2
#define DEBUG_MESSAGES 5
#define DEBUG_TRACE   10

static int debug = DEBUG_INFO;
#define touch_debug(level, ...) \
	do { \
		if (debug >= (level)) \
		printk("[ektf]:" __VA_ARGS__); \
	} while (0)
//[SM20]zihweishen modify protocol_B for report error coordinate end 2015/11/23


/*0625 start*/
//<2014/13/44-Yuting Shih. Add #if for compile error if non-firmware.
#if IAP_PORTION
// Thunder removed, 20141229  
//static uint8_t file_fw_data_old[] = {
//#include "eKTF2150_I5-3_Truly_V5505.i" /* modify */
//};

///static uint8_t file_fw_data_new[] = {
//#include "eKTF2150_I5-3_Truly_V5505.i" /* modify */
//};

//static uint8_t *file_fw_data[] = file_fw_data_old;
#endif
//>2014/13/44-Yuting Shih.
/*0625 end*/

// For Firmware Update
#define   ELAN_IOCTLID              0xD0
#define   IOCTL_I2C_SLAVE           _IOW(ELAN_IOCTLID,  1, int)
#define   IOCTL_MAJOR_FW_VER        _IOR(ELAN_IOCTLID,  2, int)
#define   IOCTL_MINOR_FW_VER        _IOR(ELAN_IOCTLID,  3, int)
#define   IOCTL_RESET               _IOR(ELAN_IOCTLID,  4, int)
#define   IOCTL_IAP_MODE_LOCK       _IOR(ELAN_IOCTLID,  5, int)
#define   IOCTL_CHECK_RECOVERY_MODE _IOR(ELAN_IOCTLID,  6, int)
#define   IOCTL_FW_VER              _IOR(ELAN_IOCTLID,  7, int)
#define   IOCTL_X_RESOLUTION        _IOR(ELAN_IOCTLID,  8, int)
#define   IOCTL_Y_RESOLUTION        _IOR(ELAN_IOCTLID,  9, int)
#define   IOCTL_FW_ID               _IOR(ELAN_IOCTLID, 10, int)
#define   IOCTL_ROUGH_CALIBRATE     _IOR(ELAN_IOCTLID, 11, int)
#define   IOCTL_IAP_MODE_UNLOCK     _IOR(ELAN_IOCTLID, 12, int)
#define   IOCTL_I2C_INT             _IOR(ELAN_IOCTLID, 13, int)
#define   IOCTL_RESUME              _IOR(ELAN_IOCTLID, 14, int)
#define   IOCTL_POWER_LOCK          _IOR(ELAN_IOCTLID, 15, int)
#define   IOCTL_POWER_UNLOCK        _IOR(ELAN_IOCTLID, 16, int)
#define   IOCTL_FW_UPDATE           _IOR(ELAN_IOCTLID, 17, int)
#define   IOCTL_BC_VER              _IOR(ELAN_IOCTLID, 18, int)
#define   IOCTL_2WIREICE            _IOR(ELAN_IOCTLID, 19, int)


#define   CUSTOMER_IOCTLID          0xA0
#define   IOCTL_CIRCUIT_CHECK       _IOR(CUSTOMER_IOCTLID,  1, int)
#define   IOCTL_GET_UPDATE_PROGREE  _IOR(CUSTOMER_IOCTLID,  2, int)
//[SM20][zihweishen] Do not need to distinguish file_fw_data_liano and file_fw_data_truly begin 2015/10/14
#if 0 
#define Touch_Detect_Pin (GPIO97|0x80000000) //>Zihwei 2015 03/20
#endif
//[SM20][zihweishen] Do not need to distinguish file_fw_data_liano and file_fw_data_truly end 2015/10/14
/*[Lavender][bozhi_lin] store touch vendor and firmware version to tpd_show_vendor_firmware 20150213 begin*/
#if defined(TPD_REPORT_VENDOR_FW)
extern char *tpd_show_vendor_firmware;
//<2015/10/15-stevenchen, store lcm vendor information in /sys/module/tpd_misc/parameters/lcm_vendor
extern char *lcm_vendor;
//>2015/10/15-stevenchen
#endif
/*[Lavender][bozhi_lin] 20150213 end*/

extern struct tpd_device *tpd;

/*[Lavender][bozhi_lin] elan touch detect glove or finger in glove mode 20150720 begin*/
// [All][Main][Glove mode][Feature][number][potingchang] Touch Glove mode   20141219 BEGIN
#define GLOVEMODE
static bool glovemode=0;
static bool glovemode_is_finger=0;
// [All][Main][Glove mode][Feature][number][potingchang] 20141219 END
/*[Lavender][bozhi_lin] 20150720 end*/
uint8_t RECOVERY    = 0x00;
int   FW_VERSION    = 0x00;
int   X_RESOLUTION  = 0x00;  /* Please fill the right resolution if resolution mapping error. */
int   Y_RESOLUTION  = 0x00;  /* Please fill the right resolution if resolution mapping error. */
int   FW_ID         = 0x00;
int   BC_VERSION    = 0x00;
int   work_lock     = 0x00;
int   power_lock    = 0x00;
int   circuit_ver   = 0x01;
int   button_state  = 0;
/*++++i2c transfer start+++++++*/
//<<Mel - 4/10, Modify I2C device address to 0x15(7-bit address).
//int   file_fops_addr=0x10;
//int   file_fops_addr = 0x15;
int   file_fops_addr = I2C_ELAN_DEV_ADDR;
//Mel - 4/10, Modify I2C slave address to 0x15>>.
/*++++i2c transfer end+++++++*/
int   tpd_down_flag = 0;

struct i2c_client   * i2c_client = NULL;
struct task_struct  * thread = NULL;

static DECLARE_WAIT_QUEUE_HEAD(waiter);
static inline int elan_ktf2k_ts_parse_xy(uint8_t *data, uint16_t *x, uint16_t *y);
#if 0
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);
#endif

static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect(struct i2c_client *client, struct i2c_board_info *info);
static int tpd_remove(struct i2c_client *client);
//[SM20]zihweishen modify protocol_B for report error coordinate begin 2015/11/23
#if 0
static int touch_event_handler(void *unused);
#endif

static int tpd_flag = 0;
//[SM20]zihweishen modify protocol_B for report error coordinate end 2015/11/23

#if 0
static int key_pressed = -1;

struct osd_offset{
  int left_x;
  int right_x;
  unsigned int key_event;
};

static struct osd_offset OSD_mapping[] = { /* Range need define by Case!  */
  {  35, 290, KEY_MENU },   /* menu_left_x, menu_right_x, KEY_MENU */
  { 303, 467, KEY_HOME },   /* home_left_x, home_right_x, KEY_HOME */
  { 473, 637, KEY_BACK },   /* back_left_x, back_right_x, KEY_BACK */
  { 641, 905, KEY_SEARCH }, /* search_left_x, search_right_x, KEY_SEARCH */
};
#endif

#if( IAP_PORTION )
uint8_t ic_status   = 0x00; /* 0:OK 1:master fail 2:slave fail  */
//<<Mel - 4/10, Modify I2C slave address to 0x15.
//uint8_t I2C_DATA[3] = {/*0x10*/0x15, 0x20, 0x21 };  /*I2C devices address*/
uint8_t I2C_DATA[3] = { I2C_ELAN_DEV_ADDR, 0x20, 0x21 };  /*I2C devices address*/
//Mel - 4/10, Modify I2C slave address to 0x15>>.
int   update_progree  = 0;
int   is_OldBootCode  = 0;  /* 0:new, 1:old */



/*The newest firmware, if update must be changed here */
// Zihwei modify for update touch Firmware 55A1->55A2 2015 12/25
// Zihwei modify for update touch Firmware 55A0->55A1 2015 12/18
// Zihwei modify for update touch Firmware 2015 04/15
static uint8_t file_fw_data_truly[] = {
  #include "SM20_V55A2.i" /* modify */
};
//static uint8_t file_fw_data_liano[] = {
//  #include "eKTF2150_i5-3_GIS_V5501.i" /* modify */ //>Zihwei 2015 03/20
//};
static uint8_t *file_fw_data=file_fw_data_truly; //>Zihwei 2015 03/20

int PageNum     = 0;
enum
{
  PageSize    = 132,
  //PageNum     = 249,
  ACK_Fail    = 0x00,
  ACK_OK      = 0xAA,
  ACK_REWRITE = 0x55,
};

enum
{
  E_FD        = -1,
};
#endif

// Add 0821 start
#define ELAN_DRIVER_NAME  "ektf2k_mtk"
static const struct i2c_device_id tpd_id[] =
{
  { ELAN_DRIVER_NAME, 0 },
  { }
};

//Steven start
//#if defined( ELAN_MTK6577 ) || defined( ELAN_MTK6755 )
//  static struct i2c_board_info __initdata i2c_tpd = { I2C_BOARD_INFO("ektf2k_mtk", ( I2C_ELAN_SLAVE_ADDR >> 1))};
//#else
  unsigned short force[] = { I2C_ELAN_BUS, I2C_ELAN_SLAVE_ADDR, I2C_CLIENT_END, I2C_CLIENT_END };
  static const unsigned short *const forces[] = { force, NULL };
//static struct i2c_client_address_data addr_data = { .forces = forces, };
//#endif
//Steven end

static const struct of_device_id tpd_of_match[] = {
	{.compatible = "mediatek,cap_touch"},
	{},
};

static struct i2c_driver tpd_i2c_driver =
{
    .driver = {
      .name   = ELAN_DRIVER_NAME,
      .of_match_table = tpd_of_match,
    },
    .probe    = tpd_probe,
    .remove   = tpd_remove,
    .id_table = tpd_id,
    .detect   = tpd_detect,
    .driver.name = ELAN_DRIVER_NAME,
    .address_list = (const unsigned short *)forces,
};
//Add 0821 end



struct elan_ktf2k_ts_data {
  struct i2c_client       * client;
  struct input_dev        * input_dev;
  struct workqueue_struct * elan_wq;
  struct work_struct      work;
/*  struct early_suspend    early_suspend; */
  int   intr_gpio;
/* Firmware Information */
  int   fw_ver;
  int   fw_id;
  int   bc_ver;
  int   x_resolution;
  int   y_resolution;
/* For Firmare Update */
  struct miscdevice   firmware;
  struct hrtimer      timer;
};

static struct elan_ktf2k_ts_data  * private_ts;
static int __hello_packet_handler(struct i2c_client *client);
static int  __fw_packet_handler(struct i2c_client *client);
static int  elan_ktf2k_ts_rough_calibrate(struct i2c_client *client);
static void  tpd_resume(struct device *h);

#if( IAP_PORTION )
//int   Update_FW_One(/*struct file *filp,*/ struct i2c_client *client, int recovery);
int Update_FW_One(void *unused);	// Thunder modify for update touch Firmware 20141229
int IAPReset(void);
#endif

#define   ELAN_ENABLE         1
#define   ELAN_DISABLE        0
static void elan_power_enable(int enable)
{
#if !defined(BUILD_LK)
    int ret;
#endif

    if( ELAN_ENABLE == enable )
    {
    #if defined( BUILD_LK )
        mt6351_set_register_value( flagname, 1 );
    #else
        printk("Device Tree get regulator! \n");
//[SM20] zihweishen Modify VTOUCH-supply vio28 to vtouch for not power on 2016/01/15
        tpd->reg = regulator_get(tpd->tpd_dev, "vtouch"); //[SM20] zihweishen Modify VTOUCH-supply vldo28 to vio28 for camera AF 2015/12/30
//[SM20] zihweishen end 2016/01/15
        ret = regulator_set_voltage(tpd->reg, 2800000, 2800000);	//set 2.8v
	if (ret) {
            printk("[elan]regulator_set_voltage(%d) failed!\n", ret);
            return;
        }

        ret = regulator_enable(tpd->reg);	/*enable regulator*/
        if (ret)
        printk("[elan]regulator_enable() failed!\n");
    #endif
    }
    else
    {
    #if defined( BUILD_LK )
        mt6351_set_register_value( flagname, 0 );
    #else
        ret = regulator_disable(tpd->reg);	/*disable regulator*/
        if (ret)
            printk("[elna]regulator_disable() failed!\n");
    #endif
    }
}

//Steven start
/*1 enable,0 disable,touch_panel_eint default status, need to confirm after register eint*/
//[SM20]zihweishen modify protocol_B for report error coordinate begin 2015/11/23
int irq_flag = 1;
static spinlock_t irq_flag_lock;
//[SM20]zihweishen modify protocol_B for report error coordinate end 2015/11/23

void elan_irq_enable(void)
{
	unsigned long flags;

	spin_lock_irqsave(&irq_flag_lock, flags);

	if (irq_flag == 0) {
		irq_flag = 1;
		spin_unlock_irqrestore(&irq_flag_lock, flags);
		enable_irq(touch_irq);
	} else if (irq_flag == 1) {
		spin_unlock_irqrestore(&irq_flag_lock, flags);
		printk("Touch Eint already enabled!");
	} else {
		spin_unlock_irqrestore(&irq_flag_lock, flags);
		printk("Invalid irq_flag %d!", irq_flag);
	}
	/*GTP_INFO("Enable irq_flag=%d",irq_flag);*/
}

void elan_irq_disable(void)
{
	unsigned long flags;

	spin_lock_irqsave(&irq_flag_lock, flags);

	if (irq_flag == 1) {
		irq_flag = 0;
		spin_unlock_irqrestore(&irq_flag_lock, flags);
		disable_irq(touch_irq);
	} else if (irq_flag == 0) {
		spin_unlock_irqrestore(&irq_flag_lock, flags);
		printk("Touch Eint already disabled!");
	} else {
		spin_unlock_irqrestore(&irq_flag_lock, flags);
		printk("Invalid irq_flag %d!", irq_flag);
	}
	/*GTP_INFO("Disable irq_flag=%d",irq_flag);*/
}
//Steven end

#if defined( ELAN_DMA_MODE )

static int elan_i2c_dma_recv_data(struct i2c_client *client, uint8_t *buf,uint8_t len)
{
int       rc;
uint8_t * pReadData  = 0;
unsigned  short addr = 0;

  pReadData = gpDMABuf_va;
  addr = client->addr ;

  client->addr |= I2C_DMA_FLAG;
  if( !pReadData )
  {
    printk("[elan] dma_alloc_coherent failed!\n");
    return -1;
  }
  rc = i2c_master_recv( client, (u8*)gpDMABuf_pa, len );
  printk("[elan] elan_i2c_dma_recv_data rc=%d!\n", rc);
//copy_to_user( buf, pReadData, len );
  client->addr = addr;

  return rc;
}

static int elan_i2c_dma_send_data(struct i2c_client *client, uint8_t *buf,uint8_t len)
{
int rc;
unsigned short addr = 0;
uint8_t *pWriteData = gpDMABuf_va;

  addr = client->addr ;

  client->addr |= I2C_DMA_FLAG;
  if( !pWriteData )
  {
    printk("[elan] dma_alloc_coherent failed!\n");
    return -1;
  }
//copy_from_user(pWriteData, ((void*)buf), len);

  rc = i2c_master_send( client, (u8*)gpDMABuf_pa, len);
  printk("[elan] elan_i2c_dma_send_data rc=%d!\n", rc);
  client->addr = addr;

  return rc;
}
#endif

/* For Firmware Update */
int elan_iap_open(struct inode *inode, struct file *filp)
{
  printk("[ELAN]into elan_iap_open\n");
  if( private_ts == NULL )
    printk("private_ts is NULL~~~");

  return 0;
}

int elan_iap_release(struct inode *inode, struct file *filp)
{
  return 0;
}

static ssize_t elan_iap_write(struct file *filp, const char *buff, size_t count, loff_t *offp)
{
int   ret;
char *tmp;

  printk("[ELAN]into elan_iap_write\n");

#if defined( ESD_CHECK )  //0604
  have_interrupts = 1;
#endif
  if( count > 8192  )
    count = 8192;

  tmp = kmalloc( count, GFP_KERNEL );
  if( tmp == NULL )
    return -ENOMEM;

#if defined( ELAN_DMA_MODE )
  if( copy_from_user( gpDMABuf_va, buff, count ))
  {
    return -EFAULT;
  }
  ret = elan_i2c_dma_send_data( private_ts->client, tmp, count );
#else
  if( copy_from_user( tmp, buff, count ))
  {
    return -EFAULT;
  }
  ret = i2c_master_send( private_ts->client, tmp, count );
#endif /* End.. (ELAN_DMA_MODE) */

  kfree( tmp );
  return (( ret == 1 ) ? count : ret );
}

ssize_t elan_iap_read(struct file *filp, char *buff, size_t count, loff_t *offp)
{
long  rc;
int   ret;
char *tmp;

  printk("[ELAN]into elan_iap_read\n");

#if defined( ESD_CHECK ) //0604
  have_interrupts = 1;
#endif
  if( count > 8192 )
    count = 8192;

  tmp = kmalloc( count, GFP_KERNEL );
  if( tmp == NULL)
    return -ENOMEM;

#if defined( ELAN_DMA_MODE )
  ret = elan_i2c_dma_recv_data( private_ts->client, tmp, count );
  if( ret >= 0 )
    rc = copy_to_user( buff, gpDMABuf_va, count );
#else
  ret = i2c_master_recv( private_ts->client, tmp, count );
  if( ret >= 0 )
    rc = copy_to_user( buff, tmp, count );
#endif /* End.. (ELAN_DMA_MODE) */

  kfree( tmp );
//return ret;
  return (( ret == 1 ) ? count : ret );
}

//Steven start
struct device dummy;
//Steven end
static long elan_iap_ioctl(/*struct inode *inode,*/ struct file *filp,    unsigned int cmd, unsigned long arg)
{
//Steven start, marked temporarily
//int __user *ip = (int __user *)arg;
//Steven end

  printk("[ELAN]into elan_iap_ioctl\n");
  printk("cmd value %x\n",cmd);

  switch ( cmd )
  {
    case IOCTL_I2C_SLAVE:
    {
      private_ts->client->addr = (int __user)arg;
      private_ts->client->addr &= I2C_MASK_FLAG;
      private_ts->client->addr |= I2C_ENEXT_FLAG;
    //file_fops_addr = 0x15;
    } break;
    case IOCTL_MAJOR_FW_VER:
      break;
    case IOCTL_MINOR_FW_VER:
      break;
    case IOCTL_RESET:
    {
        GTP_GPIO_OUTPUT(GTP_RST_PORT, 1);
        mdelay(10);

        GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);
        mdelay(10);

        GTP_GPIO_OUTPUT(GTP_RST_PORT, 1);
    } break;
    case IOCTL_IAP_MODE_LOCK:
    {
      //if( 0 == work_lock )	// Thunder remove for factory AATS issue on Android L 20141229
      //{
        //printk("[elan]%s %X = IOCTL_IAP_MODE_LOCK\n", __func__, IOCTL_IAP_MODE_LOCK );
        //printk("[elan]%X = IOCTL_IAP_MODE_LOCK\n", IOCTL_IAP_MODE_LOCK );
        work_lock = 1;
        //mt_eint_mask( CUST_EINT_TOUCH_PANEL_NUM );  //disable_irq( CUST_EINT_TOUCH_PANEL_NUM );
        elan_irq_disable();
      //cancel_work_sync( &private_ts->work );
      #if defined( ESD_CHECK )  //0604
        cancel_delayed_work_sync( &esd_work );
      #endif
      //}
    } break;
    case IOCTL_IAP_MODE_UNLOCK:
    {
      //if( 1 == work_lock )	// Thunder remove for factory AATS issue on Android L 20141229
      //{
        work_lock = 0;
        //mt_eint_unmask( CUST_EINT_TOUCH_PANEL_NUM ); //enable_irq(CUST_EINT_TOUCH_PANEL_NUM);
        elan_irq_enable();

      #if defined( ESD_CHECK )  //0604
        queue_delayed_work( esd_wq, &esd_work, delay );
      #endif
      //}
    } break;
    case IOCTL_CHECK_RECOVERY_MODE:
    {
      return RECOVERY;
    } //break;
    case IOCTL_FW_VER:
    {
      __fw_packet_handler( private_ts->client );
      return FW_VERSION;
    } //break;
    case IOCTL_X_RESOLUTION:
    {
      __fw_packet_handler( private_ts->client );
      return X_RESOLUTION;
    } //break;
    case IOCTL_Y_RESOLUTION:
    {
      __fw_packet_handler( private_ts->client );
      return Y_RESOLUTION;
    } //break;
    case IOCTL_FW_ID:
    {
      __fw_packet_handler( private_ts->client );
      return FW_ID;
    } //break;
    case IOCTL_ROUGH_CALIBRATE:
    {
      return elan_ktf2k_ts_rough_calibrate( private_ts->client );
    } //break;
    case IOCTL_I2C_INT:
    {
//Steven start, marked temporarily
/*
      put_user( mt_get_gpio_in( GPIO_CTP_EINT_PIN ), ip );
      printk("[elan]GPIO_CTP_EINT_PIN = %d\n", mt_get_gpio_in( GPIO_CTP_EINT_PIN ));
*/
//Steven end
    } break;
    case IOCTL_RESUME:
    {
      tpd_resume(&dummy);
    } break;
    case IOCTL_CIRCUIT_CHECK:
    {
      return circuit_ver;
    } //break;
    case IOCTL_POWER_LOCK:
    {
      power_lock = 1;
    } break;
    case IOCTL_POWER_UNLOCK:
    {
      power_lock = 0;
    } break;
  #if( IAP_PORTION )
    case IOCTL_GET_UPDATE_PROGREE:
    {
      update_progree = (int __user)arg;
    } break;
    case IOCTL_FW_UPDATE:
    {
        struct task_struct *fw_update_thread;
        s32 err = 0;

        RECOVERY = 0; //= IAPReset( private_ts->client );
     
        fw_update_thread = kthread_run(Update_FW_One, 0, "elan_update");
        if (IS_ERR(fw_update_thread)) {
            err = PTR_ERR(fw_update_thread);
            printk(TPD_DEVICE " failed to create auto-update thread: %d\n", err);
        }
    } break;
  #endif
    case IOCTL_BC_VER:
    {
      __fw_packet_handler( private_ts->client );
      return BC_VERSION;
    } //break;
    default:
    {
    } break;
  }

  return 0;
}

struct file_operations elan_touch_fops =
{
    .open     = elan_iap_open,
    .write    = elan_iap_write,
    .read     = elan_iap_read,
    .release  = elan_iap_release,
  .unlocked_ioctl = elan_iap_ioctl,
  .compat_ioctl		= elan_iap_ioctl, 

};

#if( IAP_PORTION )
int EnterISPMode(struct i2c_client *client, uint8_t  *isp_cmd)
{
int   len = 0;
char  buff[4] = { 0x00 };

  //len = i2c_master_send( private_ts->client, isp_cmd,  sizeof( isp_cmd ));
  len = i2c_master_send( private_ts->client, isp_cmd, 4);	// Thunder modify for update touch Firmware 20141229
  if( len != sizeof( buff ))
  {
    printk("[ELAN] ERROR: EnterISPMode fail! len=%d\r\n", len);
    return -1;
  }
  else
    printk("[ELAN] IAPMode write data successfully! cmd = [%02X, %02X, %02X, %02X]\n", isp_cmd[0], isp_cmd[1], isp_cmd[2], isp_cmd[3]);

  return 0;
}

int ExtractPage(struct file *filp, uint8_t * szPage, int byte)
{
int len = 0;

  len = filp->f_op->read( filp, szPage,byte, &filp->f_pos);
  if( len != byte )
  {
    printk("[ELAN] ExtractPage ERROR: read page error, read error. len=%d\r\n", len);
    return -1;
  }

  return 0;
}

int WritePage(uint8_t * szPage, int byte)
{
int len = 0;

  len = i2c_master_send( private_ts->client, szPage, byte );
  if( len != byte )
  {
    printk("[ELAN] ERROR: write page error, write error. len=%d\r\n", len );
    return -1;
  }

  return 0;
}

int GetAckData(struct i2c_client *client)
{
int   len = 0;
char  buff[2] = { 0x00 };

  len = i2c_master_recv( private_ts->client, buff, sizeof( buff ));
  if( len != sizeof( buff ))
  {
    printk("[ELAN] ERROR: read data error, write 50 times error. len=%d\r\n", len );
    return -1;
  }

  printk("[ELAN] GetAckData:%02X,%02X\n", buff[0], buff[1] );
  if( buff[0] == 0xAA /* && buff[1] == 0xAA */)
    return ACK_OK;
  else if(( buff[0] == 0x55 ) && ( buff[1] == 0x55 ))
    return ACK_REWRITE;
  else
    return ACK_Fail;

  return 0;
}

void print_progress(int page, int ic_num, int j)
{
int   i, percent, page_tatol, percent_tatol;
char  str[256] = { '\0' };

  str[0] = '\0';
  for( i = 0; i < (( page ) / 10 ); i++ )
  {
    str[i]    = '#';
    str[i+1]  = '\0';
  }

  page_tatol    = page + PageNum * ( ic_num - j );
  percent       = (( 100 * page ) / ( PageNum ));
  percent_tatol = (( 100 * page_tatol ) / ( PageNum * ic_num ));

  if( PageNum ==  page )
    percent = 100;

  if(( PageNum * ic_num ) == page_tatol )
    percent_tatol = 100;

  printk("\rprogress %s| %d%%", str, percent );

  if( PageNum == page )
    printk("\n");
}

/*
* Restet and (Send normal_command ?)
* Get Hello Packet
*/
int IAPReset(void)
{
    int   res = 1;

    GTP_GPIO_OUTPUT(GTP_RST_PORT, 1);
    mdelay( 10 );

    GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);
    mdelay( 10 );

    GTP_GPIO_OUTPUT(GTP_RST_PORT, 1);

#if 0
  printk("[ELAN] read Hello packet data!\n");
  res = __hello_packet_handler( client );
#endif
  return  res;
}

/* Check Master & Slave is "55 aa 33 cc" */
int CheckIapMode(void)
{
char buff[4] = { 0 }, len = 0;

//WaitIAPVerify(1000000);
//len = read( fd, buff, sizeof( buff ));
  len = i2c_master_recv( private_ts->client, buff, sizeof( buff ));
  if( sizeof( buff ) != len )
  {
    printk("[ELAN] CheckIapMode ERROR: read data error,len = %d\r\n", len );
    return -1;
  }
  else
  {
    if(( buff[0] == 0x55 ) && ( buff[1] == 0xAA ) && ( buff[2] == 0x33 ) && ( buff[3] == 0xCC ))
    {
    //printk("[ELAN] CheckIapMode is 55 AA 33 CC\n");
      return 0;
    }
    else// if( j == 9 )
    {
      printk("[ELAN] Mode = 0x%02X 0x%02X 0x%02X 0x%02X\r\n", buff[0], buff[1], buff[2], buff[3] );
      printk("[ELAN] ERROR: CheckIapMode error\n");
      return -1;
    }
  }
  printk("\n");
}

//int Update_FW_One(struct i2c_client *client, int recovery)
//static int Update_FW_One()
int Update_FW_One(void *unused)	// Thunder modify for update touch Firmware 20141229
{
int res = 0, ic_num = 1;
int iPage = 0, rewriteCnt = 0; /* rewriteCnt for PAGE_REWRITE */
int i = 0;
int restartCnt = 0, checkCnt = 0; /* For IAP_RESTART */
int curIndex = 0;
int byte_count;
uint8_t *szBuff = NULL;
#if( CMD_54001234 )
uint8_t isp_cmd[] = { 0x54, 0x00, 0x12, 0x34 };  //54 00 12 34
#else
uint8_t isp_cmd[] = { 0x45, 0x49, 0x41, 0x50 };  //45 49 41 50
#endif
uint8_t data;

IAP_RESTART:

  data = I2C_DATA[0]; /* Master */
  //dev_dbg( &client->dev, "[ELAN] %s: address data=0x%x \r\n", __func__, data );

//if( recovery != 0x80 )
//{
    printk("[ELAN] Firmware upgrade normal mode !\n");
    
    curIndex = 0; //0126
    IAPReset();
    mdelay( 20 );

    res = EnterISPMode( private_ts->client, isp_cmd );  /* enter ISP mode */
    ///* 1111 /* modify */

#if 1
   // res = i2c_master_recv( private_ts->client, recovery_buffer, 4 );  /* 55 AA 33 CC */
  //  printk("[ELAN] recovery byte data: %02X, %02X, %02X, %02X \n",
  //      recovery_buffer[0], recovery_buffer[1], recovery_buffer[2], recovery_buffer[3]);
    mdelay( 20 ); //0126
#endif

    //*/ /* modify */
#if 1  //1111
  /* Check IC's status is IAP mode(55 AA 33 CC) or not */
    res = CheckIapMode();  /* Step 1 enter ISP mode */
    if( -1 == res ) /* CheckIapMode fail */
    {
      checkCnt ++;
      if (checkCnt >= 5)
      {
        printk("[ELAN] ERROR: CheckIapMode %d times fails!\n", IAPRESTART);
        //return Upd_Fail;
        goto Upd_Fail;	// Thunder modify for update touch Firmware 20141229
      }
      else
      {
        printk("[ELAN] CheckIapMode retry %dth times! And restart IAP~~~\n\n", checkCnt);
        goto IAP_RESTART;
      }
    }
    else
      printk("[ELAN]  CheckIapMode ok!\n");
#endif
//} else
//    printk("[ELAN] Firmware upgrade recovery mode !\n");

/* Send Dummy Byte */
  printk("[ELAN] send one byte data: %X, %X", private_ts->client->addr, data );
  res = i2c_master_send( private_ts->client, &data,  sizeof( data ));
  if( sizeof( data ) != res )
  {
    printk("[ELAN] dummy error code = %d\n", res );
  }
  mdelay( 50 );

/* Start IAP */
  //PageNum = (sizeof(file_fw_data_truly)/sizeof(uint8_t)/PageSize); //1111
  PageNum = (sizeof(file_fw_data_truly)/sizeof(uint8_t)/PageSize);	// Zihwei modify for update touch Firmware 2015/03/23
  for( iPage = 1; iPage <= PageNum; iPage++ )
  {
PAGE_REWRITE:
#if 1
  /* 8byte mode */
  //szBuff = fw_data + (( iPage - 1) * PageSize );
    for( byte_count = 1; byte_count <= 17; byte_count++ )
    {
    //printk("[ELAN] byte %d, curIndex = %d\n", byte_count, curIndex );
      szBuff    = file_fw_data + curIndex;
      if( 17 != byte_count )
      {
        curIndex  = curIndex + 8;
      //ioctl( fd, IOCTL_IAP_MODE_LOCK, data );
        res = WritePage( szBuff, 8 );
      }
      else
      {
        curIndex  =  curIndex + 4;
      //ioctl( fd, IOCTL_IAP_MODE_LOCK, data );
        res = WritePage( szBuff, 4 );
      }
    } /* End.. for(byte_count=1;...) */
#endif
#if 0 /* 132byte mode */
    szBuff    = file_fw_data + curIndex;
    curIndex  = curIndex + PageSize;
    res = WritePage( szBuff, PageSize );
#endif
#if 1
    if(( PageNum == iPage ) || ( 1 == iPage )) //0327, Joanna
    {
      mdelay( 70 );
    }
    else
    {
      mdelay( 70 );
    }
#endif
    res = GetAckData( private_ts->client );
    if( ACK_OK != res )
    {
      mdelay( 70 );
      printk("[ELAN] ERROR: GetAckData fail! res = %d\r\n", res );
      if( ACK_REWRITE == res )
      {
        rewriteCnt = rewriteCnt + 1;
        if( PAGERETRY == rewriteCnt )
        {
          printk("[ELAN] ID 0x%02x %dth page ReWrite %d times fails!\n", data, iPage, PAGERETRY );
          //return Upd_Fail; //1111
          goto Upd_Fail;	// Thunder modify for update touch Firmware 20141229
        }
        else
        {
          printk("[ELAN] ---%d--- page ReWrite %d times!\n",  iPage, rewriteCnt );
          curIndex = curIndex - PageSize;
          goto PAGE_REWRITE;
        }
      }
      else
      {
        restartCnt = restartCnt + 1;
        if( restartCnt >= 5 )
        {
          printk("[ELAN] ID 0x%02x ReStart %d times fails!\n", data, IAPRESTART );
          //return Upd_Fail; 
          goto Upd_Fail; // Thunder modify for update touch Firmware 20141229
        }
        else
        {
          printk("[ELAN] ===%d=== page ReStart %d times!\n",  iPage, restartCnt );
          goto IAP_RESTART;
        }
      }
    }
    else
    {
      printk("  data : 0x%02X ", data );
      rewriteCnt = 0;
      print_progress( iPage,ic_num, i );
    }

    mdelay( 10 );
  } /* End.. for(iPage=1;...) */

  //1111 - start
  
  
  IAPReset();/* Zihwei -> Modify for update touch Firmware. 2015/03/31*/
	mdelay(100); 
	//res= __hello_packet_handler(client); 
	//mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);/* Zihwei -> Modify for update touch Firmware. 2015/03/31*/
        elan_irq_disable();
	
	res= __hello_packet_handler(private_ts->client);	// Thunder modify for update touch Firmware 20141229
	if (res > 0) 
    printk("[ELAN] Update ALL Firmware successfully!\n"); 
  __fw_packet_handler(private_ts->client); 
  
  printk("1111 show_fw_update mt_eint_unmask\n");
	//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
        elan_irq_enable();
	
  #ifdef ESD_CHECK
	   queue_delayed_work(esd_wq, &esd_work, delay);	
  #endif
  
  //1111 - end
  return 0;
  
  //1111 - start  
Upd_Fail:

  printk("FAIL 1111 show_fw_update mt_eint_unmask\n");
	//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
        elan_irq_enable();
	
#ifdef ESD_CHECK  //1111
	queue_delayed_work(esd_wq, &esd_work, delay);	
#endif

	return -1;
 //1111 - end   
  
}

#endif
/* End Firmware Update */


#if 0
static void elan_ktf2k_ts_early_suspend(struct early_suspend *h);
static void elan_ktf2k_ts_late_resume(struct early_suspend *h);
#endif

#if defined( GLOVEMODE )
// [All][Main][Glove mode][Feature][number][potingchang] Touch Glove mode   20141219 BEGIN
static ssize_t elan_ktf2k_glove_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
	ssize_t   ret = 0;
	sprintf( buf, "glovemode=%d\n", glovemode );
	ret = strlen( buf) + 1;
	return ret;
}
//the fuction is used to swich glove mode or normal mode
static void elan_glove_enable(bool glove)
{
	uint8_t buff[4]={0};
	uint8_t ret;
	if(glove){
		glovemode=0x01;
		buff[0]=0x54;
		buff[1]=0x57;
		buff[2]=0x02;
		buff[3]=0x01;
		printk("elan_glove_enable=%d\n",glovemode);
		ret=i2c_master_send(private_ts->client, buff, 4);
		if(ret != 4){
			printk("glove error send,ret=%d\n",ret);
		}
		printk("glovemode=%d\n",glovemode);
		printk(" enter glovemode\n");
	}
	else{
		glovemode=0x00;
		buff[0]=0x54;
		buff[1]=0x57;
		buff[2]=0x01;
		buff[3]=0x01;
		printk("elan_glove_enable=%d\n",glovemode);
		ret=i2c_master_send(private_ts->client, buff, 4);
		if(ret != 4){
			printk("glove error send,  ret=%d\n",ret);
		}
		printk("glovemode=%c\n",glovemode);
		printk("not enter glovemode\n");
	}
}

static ssize_t  elan_ktf2k_glove_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	bool GloveSitch;
	//int b = a-'0';
	GloveSitch=buf[0]-'0';
	//GloveSitch=simple_strtol(buf[0],0,10);
	if( GloveSitch == 1)
		elan_glove_enable( true );
	else if(GloveSitch == 0)
		elan_glove_enable( false );
	else
		printk("you send neither 1 nor 0 buf[0]=%d\n",buf[0]);
	printk("elan_ktf2k_vendor_store enter......\n");
	printk("buf[store]=%d %d %d %d\n",buf[0],buf[1],buf[2],buf[3]);
	//	ret = strlen(buf) + 1;
	return count;

}
static DEVICE_ATTR( glove, S_IWUSR|S_IRUSR|S_IRGRP, elan_ktf2k_glove_show, elan_ktf2k_glove_store );
// [All][Main][Glove mode][Feature][number][potingchang] 20141219 END

/*[Lavender][bozhi_lin] elan touch detect glove or finger in glove mode 20150720 begin*/
static ssize_t elan_ktf2k_glovemode_is_finger_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d", glovemode_is_finger);
}

static DEVICE_ATTR( glovemode_is_finger, S_IWUSR|S_IRUSR|S_IRGRP|S_IROTH, elan_ktf2k_glovemode_is_finger_show, NULL );
/*[Lavender][bozhi_lin] 20150720 end*/

#endif

//Steven start, marked temporarily, unused?
#if 0
static ssize_t elan_ktf2k_gpio_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
struct elan_ktf2k_ts_data *ts = private_ts;
int   ret = 0;

  ret = mt_get_gpio_in( GPIO_CTP_EINT_PIN );  // = gpio_get_value(ts->intr_gpio);
  printk( KERN_DEBUG "GPIO_TP_INT_N = %d\n", ts->intr_gpio );
  sprintf( buf, "GPIO_TP_INT_N=%d\n", ret );
  ret = strlen(buf) + 1;
  return ret;
}

//static DEVICE_ATTR( gpio, S_IRUGO, elan_ktf2k_gpio_show, NULL );
#endif
//Steven end

// [All][Main][Glove mode][Feature][number][potingchang] Touch Glove mode   20141219 BEGIN
#if defined( GLOVEMODE )
static struct kobject * android_touch_kobj = NULL;

static int elan_ktf2k_touch_sysfs_init(void)
{
	int ret = 0;

	android_touch_kobj = kobject_create_and_add( "android_touch", NULL ) ;
	if( android_touch_kobj == NULL )
	{
		printk( KERN_ERR "[elan]%s: subsystem_register failed\n", __func__ );
		ret = -ENOMEM;
		return ret;
	}

	ret = sysfs_create_file( android_touch_kobj, &dev_attr_glove.attr );
	if( ret )
	{
	printk( KERN_ERR "[elan]%s: sysfs_create_group (dev_attr_glove) failed\n", __func__ );
	return ret;
	}

	/*[Lavender][bozhi_lin] elan touch detect glove or finger in glove mode 20150720 begin*/
	ret = sysfs_create_file( android_touch_kobj, &dev_attr_glovemode_is_finger.attr );
	if( ret )
	{
		printk( KERN_ERR "[elan]%s: sysfs_create_group (dev_attr_glovemode_is_finger) failed\n", __func__ );
		return ret;
	}
	/*[Lavender][bozhi_lin] 20150720 end*/

	return 0 ;
}
#endif
// [All][Main][Glove mode][Feature][number][potingchang] 20141219 END
#if defined( GLOVEMODE )
 // [All][Main][Glove mode][Feature][number][potingchang] Touch Glove mode   20141219 BEGIN
/*
static void elan_touch_sysfs_deinit(void)
{
  sysfs_remove_file( android_touch_kobj, &dev_attr_glove.attr );
  kobject_del( android_touch_kobj );
}
*/
// [All][Main][Glove mode][Feature][number][potingchang] 20141219 END
#endif


static int __elan_ktf2k_ts_poll(struct i2c_client *client)
{
//Steven start, marked temporarily
#if 0
int   status = 0, retry = 10;

  do
  {
    status = mt_get_gpio_in( GPIO_CTP_EINT_PIN ); // = gpio_get_value(ts->intr_gpio);
    //printk("[elan]: %s: status = %d\n", __func__, status );
    printk("[elan]:: status = %d\n", status );
    retry--;
  //mdelay( 200 ); //0403 modify
    mdelay( 10 );
  } while( status == 1 && retry > 0 );

//  printk("[elan]%s: poll interrupt status %s\n",
//      __func__, ( status == 1 ? "high" : "low" ));

  printk("[elan]: poll interrupt status %s\n",
      ( status == 1 ? "high" : "low" ));

  return ( status == 0 ? 0 : -ETIMEDOUT );
#else
  return 1;
#endif
//Steven end
}

static int elan_ktf2k_ts_poll(struct i2c_client *client)
{
  return __elan_ktf2k_ts_poll( client );
}

static int elan_ktf2k_ts_get_data(struct i2c_client *client, uint8_t *cmd,
      uint8_t *buf, size_t size)
{
int rc;

  //dev_dbg( &client->dev, "[elan]%s: enter\n", __func__ );
  if( buf == NULL )
    return -EINVAL;

  if (( i2c_master_send( client, cmd, 4 )) != 4 )
  {
    //dev_err( &client->dev, "[elan]%s: i2c_master_send failed\n", __func__ );
    return -EINVAL;
  }

  rc = elan_ktf2k_ts_poll( client );
  if( rc < 0 )
    return -EINVAL;
  else
  {
    if(( i2c_master_recv( client, buf, size ) != size ) ||
       ( buf[0] != CMD_S_PKT ))
      return -EINVAL;
  }

  return 0;
}

static int __hello_packet_handler(struct i2c_client *client)
{
int rc;
uint8_t buf_recv[8] = { 0 };
//uint8_t buf_recv1[4] = { 0 };

//mdelay(1500);

  rc = elan_ktf2k_ts_poll( client );
  if( rc < 0 )
  {
    //printk( "[elan] %s: Int poll failed!\n", __func__ );
    printk( "[elan] Int poll failed!\n");
    RECOVERY = 0x80;
    return RECOVERY;
  }

  rc = i2c_master_recv( client, buf_recv, 8 );
 // printk("[elan] %s: Hello Packet %02x:%02X:%02x:%02x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);
  //printk("[elan] : Hello Packet %02x:%02X:%02x:%02x\n",  buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);
// Received 8 bytes data will let TP die on old firmware on ektf21xx carbon player and MK5
/* Zihwei -> Modify Touch read value. 2015/03/31*/
  rc = i2c_master_recv(client, buf_recv, 8);
  printk("[elan] %s: hello packet %2x:%2X:%2x:%2x:%2x:%2X:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3] , buf_recv[4], buf_recv[5], buf_recv[6], buf_recv[7]);

  if(( buf_recv[0] == 0x55 ) && ( buf_recv[1] == 0x55 ) && ( buf_recv[2] == 0x80)  && ( buf_recv[3] == 0x80 ))
  {
    RECOVERY = 0x80;

/*    rc = i2c_master_recv( client, buf_recv, 4 );
    //printk("[elan] %s: Bootcode Verson %02x:%02X:%02x:%02x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3] );
    printk("[elan]  Bootcode Verson %02x:%02X:%02x:%02x\n", buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3] );*/
    return RECOVERY;
  }
/* Zihwei -> Modify Touch read value. 2015/03/31*/

/* For ektf3xxx serial, waiting re-calibration and received the packet 0x66 0x66 0x66 0x66 */
//mdelay( 300 );
  mdelay( 150 ); //0403 modify
  rc = i2c_master_recv( client, buf_recv, 4 );
  //printk("[elan] %s: Calibration Packet %02x:%02X:%02x:%02x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3] );
  printk("[elan] Calibration Packet %02x:%02X:%02x:%02x\n", buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3] );

  return 0;
}

static int __fw_packet_handler(struct i2c_client *client)
{
int rc;
int major, minor;
uint8_t cmd[]     = { CMD_R_PKT, 0x00, 0x00, 0x01 };/* Get Firmware Version*/
uint8_t cmd_x[]   = { 0x53, 0x60, 0x00, 0x00 };     /*Get x resolution*/
uint8_t cmd_y[]   = { 0x53, 0x63, 0x00, 0x00 };     /*Get y resolution*/
uint8_t cmd_id[]  = { 0x53, 0xF0, 0x00, 0x01 };     /*Get firmware ID*/
uint8_t cmd_bc[]  = { CMD_R_PKT, 0x10, 0x00, 0x01 };/* Get BootCode Version*/ //0403 modify
uint8_t buf_recv[4] = { 0x00 };

  //printk( "[elan] %s: n", __func__);
  printk( "[elan] : fw_packet_handler");
/* Firmware version */
  rc = elan_ktf2k_ts_get_data( client, cmd, buf_recv, 4 );
  if( rc < 0 )
    return rc;
  major = (( buf_recv[1] & 0x0F) << 4 ) | (( buf_recv[2] & 0xF0) >> 4 );
  minor = (( buf_recv[2] & 0x0F) << 4 ) | (( buf_recv[3] & 0xF0) >> 4 );
  private_ts->fw_ver = major << 8 | minor;
  FW_VERSION = ( major << 8 ) | minor;

/* Firmware ID */
  rc = elan_ktf2k_ts_get_data( client, cmd_id, buf_recv, 4 );
  if( rc < 0 )
    return rc;
  major = (( buf_recv[1] & 0x0F) << 4 ) | (( buf_recv[2] & 0xF0 ) >> 4 );
  minor = (( buf_recv[2] & 0x0F) << 4 ) | (( buf_recv[3] & 0xF0 ) >> 4 );
  private_ts->fw_id = major << 8 | minor;
  FW_ID = ( major << 8 ) | minor;

/* X Resolution */
  rc = elan_ktf2k_ts_get_data( client, cmd_x, buf_recv, 4 );
  if( rc < 0 )
    return rc;
  minor = (( buf_recv[2] )) | (( buf_recv[3] & 0xF0 ) << 4 );
  private_ts->x_resolution =minor;
  X_RESOLUTION = minor;

/* Y Resolution */
  rc = elan_ktf2k_ts_get_data( client, cmd_y, buf_recv, 4 );
  if( rc < 0 )
    return rc;
  minor = (( buf_recv[2] )) | (( buf_recv[3] & 0xF0 ) << 4 );
  private_ts->y_resolution =minor;
  Y_RESOLUTION = minor;

/* Bootcode version */
  rc = elan_ktf2k_ts_get_data( client, cmd_bc, buf_recv, 4 );
  if( rc < 0 )
    return rc;
  major = (( buf_recv[1] & 0x0F) << 4 ) | (( buf_recv[2] & 0xF0 ) >> 4 );
  minor = (( buf_recv[2] & 0x0F) << 4 ) | (( buf_recv[3] & 0xF0 ) >> 4 );
  private_ts->bc_ver = major << 8 | minor;
  BC_VERSION = major << 8 | minor;

//  printk( "[elan] %s: firmware version: 0x%4.4X\n",
//      __func__, FW_VERSION);
//  printk( "[elan] %s: firmware ID: 0x%4.4X\n",
//      __func__, FW_ID);
//  printk( "[elan] %s: x resolution: %d, y resolution: %d, LCM x resolution: %d, LCM y resolution: %d\n",
//      __func__, X_RESOLUTION, Y_RESOLUTION, LCM_X_MAX, LCM_Y_MAX );
//  printk( "[elan] %s: bootcode version: 0x%4.4X\n",
//      __func__, BC_VERSION );

  printk( "[elan] : firmware version: 0x%4.4X\n",
       FW_VERSION);
  printk( "[elan] : firmware ID: 0x%4.4X\n",
       FW_ID);
  printk( "[elan] : x resolution: %d, y resolution: %d, LCM x resolution: %d, LCM y resolution: %d\n",
       X_RESOLUTION, Y_RESOLUTION, LCM_X_MAX, LCM_Y_MAX );
  printk( "[elan] : bootcode version: 0x%4.4X\n",
       BC_VERSION );

/*[Lavender][bozhi_lin] store touch vendor and firmware version to tpd_show_vendor_firmware 20150216 begin*/
#if defined(TPD_REPORT_VENDOR_FW)
	{
		char buf[80]={0};
		sprintf(buf, "Elan_0x%4.4X", FW_VERSION);	
		if (tpd_show_vendor_firmware == NULL) {
			tpd_show_vendor_firmware = kmalloc(strlen(buf) + 1, GFP_ATOMIC);
		}
		if (tpd_show_vendor_firmware != NULL) {
			strcpy(tpd_show_vendor_firmware, buf);
		}
	}
//<2015/10/15-stevenchen, store lcm vendor information in /sys/module/tpd_misc/parameters/lcm_vendor
	{
		char buf1[10]={0};

		sprintf(buf1, "Truly");	
		if (lcm_vendor == NULL) {
			lcm_vendor = kmalloc(strlen(buf1) + 1, GFP_ATOMIC);
		}
		if (lcm_vendor != NULL) {
			strcpy(lcm_vendor, buf1);
		}
	}
//>2015/10/15-stevenchen
#endif
/*[Lavender][bozhi_lin] 20150216 end*/

  return 0;
}

static inline int elan_ktf2k_ts_parse_xy(uint8_t *data,
      uint16_t *x, uint16_t *y)
{
  *x = *y = 0;

  *x = ( data[0] & 0xF0 );
  *x <<= 4;
  *x |= data[1];

  *y = ( data[0] & 0x0F );
  *y <<= 8;
  *y |= data[2];

  return 0;
}

static int elan_ktf2k_ts_setup(struct i2c_client *client)
{
int rc;

  rc = __hello_packet_handler( client );
  printk("[elan] hellopacket's rc = %d\n", rc );
  mdelay( 10 );

  if( 0x80 != rc )
  {
    rc = __fw_packet_handler( client );
    if( rc < 0 )
      //printk("[elan] %s, fw_packet_handler fail, rc = %d", __func__, rc );
      printk("[elan]  fw_packet_handler fail, rc = %d",  rc );
    else
      //printk("[elan] %s: firmware checking done.\n", __func__ );
      printk("[elan] firmware checking done.\n" );
    /* Check for FW_VERSION, if 0x0000 means FW update fail! */
    if( 0x00 == FW_VERSION )
    {
      rc = 0x80;
      printk("[elan] FW_VERSION = %d, last FW update fail\n", FW_VERSION );
    }
  }
  return rc; /* Firmware need to be update if rc equal to 0x80(Recovery mode)   */
}

static int elan_ktf2k_ts_rough_calibrate(struct i2c_client *client)
{
uint8_t cmd[] = { CMD_W_PKT, 0x29, 0x00, 0x01 };

//dev_info(&client->dev, "[elan] %s: enter\n", __func__);
  //printk("[elan] %s: enter\n", __func__ );
  printk("[elan]  enter\n" );
  dev_info( &client->dev, "[elan] dump cmd: %02X, %02X, %02X, %02X\n",
      cmd[0], cmd[1], cmd[2], cmd[3]);

  if(( i2c_master_send(client, cmd, sizeof( cmd ))) != sizeof( cmd ))
  {
    //dev_err( &client->dev, "[elan] %s: i2c_master_send failed\n", __func__ );
    return -EINVAL;
  }

  return 0;
}


static int elan_ktf2k_ts_recv_data(struct i2c_client *client, uint8_t *buf)
{
int   rc, bytes_to_recv = PACKET_SIZE;
uint8_t *pReadData = 0;
unsigned short addr = 0;
long rc1;

  if( buf == NULL )
    return -EINVAL;

  memset( buf, 0, bytes_to_recv );

//#if defined( ELAN_MTK6577 ) || defined( ELAN_MTK6755 )
#if defined( ELAN_DMA_MODE )
  addr  = client->addr ;
  client->addr |= I2C_DMA_FLAG;
  pReadData     = gpDMABuf_va;
  if( !pReadData )
  {
    printk("[elan] dma_alloc_coherent failed!\n");
  }
  rc = i2c_master_recv( client, (u8*)gpDMABuf_pa, bytes_to_recv );
  rc1 = copy_to_user( buf, pReadData, bytes_to_recv );
  client->addr = addr;
  #if defined( ELAN_DEBUG )
  printk("[elan_dma] %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\n",
      buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],buf[8], buf[9],
      buf[10], buf[11], buf[12], buf[13], buf[14], buf[15],buf[16], buf[17] );
  #endif
#else
  rc = i2c_master_recv( client, buf, 8 );
  if( rc != 8 )
    printk("[elan_debug] The first package error.\n");
  printk("[elan_recv] %x %x %x %x %x %x %x %x\n",
      buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
  mdelay( 1 );

  if( buf[0] == FIVE_FINGERS_PKT ) /* For five finger */
  {
    rc = i2c_master_recv( client, buf + 8, 8 );
    if( rc != 8 )
      printk("[elan_debug] The second package error.\n");
    printk("[elan_recv] %x %x %x %x %x %x %x %x\n",
        buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);
    rc = i2c_master_recv( client, buf + 16, 2 );
    if( rc != 2 )
      printk("[elan_debug] The third package error.\n");
    mdelay( 1 );
    printk("[elan_recv] %x %x \n", buf[16], buf[17]);
  }
#endif

  return rc;
}
//[SM20]zihweishen modify protocol_B for report error coordinate begin 2015/11/23
#define FINGER_NUM 10 //zihwei modify

#ifdef PROTOCOL_B
/* Protocol B  */
//#define FINGER_NUM 10 //zihwei modify

static int mTouchStatus[FINGER_NUM] = {0};  /* finger_num=10 */
void force_release_pos(struct i2c_client *client)
{
	//struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int i;
	for (i=0; i < FINGER_NUM; i++) {
		if (mTouchStatus[i] == 0) continue;
		input_mt_slot(tpd->dev, i);
		input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, 0);
		mTouchStatus[i] = 0;
	}

	input_sync(tpd->dev);
}

static void elan_ktf2k_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
	
	struct input_dev *idev = tpd->dev;
	uint16_t x =0, y =0,touch_size, pressure_size;
	uint16_t fbits=0;
	uint8_t i, num;
	uint16_t active = 0; 
	uint8_t idx, btn_idx;
	int finger_num;
	//int finger_id;
	static uint8_t size_index[10] = {35, 35, 36, 36, 37, 37, 38, 38, 39, 39};
	//int pen_hover = 0;  //zihwei modify
	//int pen_down = 0; //zihwei modify
	//uint16_t p = 0; //zihwei modify

	/* for 10 fingers */
	if (buf[0] == TEN_FINGERS_PKT){
		finger_num = 10;
		num = buf[2] & 0x0f;
		fbits = buf[2] & 0x30;
		fbits = (fbits << 4) | buf[1];
		idx=3;
		btn_idx=33;
	}
	/* for 5 fingers  */
	else if ((buf[0] == MTK_FINGERS_PKT) || (buf[0] == FIVE_FINGERS_PKT)){
		finger_num = 5;
		num = buf[1] & 0x07;
		fbits = buf[1] >>3;
		idx=2;
		btn_idx=17;
	}else{
		/* for 2 fingers */
		finger_num = 2;
		num = buf[7] & 0x03;    // for elan old 5A protocol the finger ID is 0x06
		fbits = buf[7] & 0x03;
		//        fbits = (buf[7] & 0x03) >> 1; // for elan old 5A protocol the finger ID is 0x06
		idx=1;
		btn_idx=7;
	}

	switch (buf[0]) {
	case MTK_FINGERS_PKT:
	case TWO_FINGERS_PKT:
	case FIVE_FINGERS_PKT:
	case TEN_FINGERS_PKT:

		for(i = 0; i < finger_num; i++){
			active = fbits & 0x1;
			if(active || mTouchStatus[i]){
				input_mt_slot(tpd->dev, i);
				input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, active);
				if(active){
					elan_ktf2k_ts_parse_xy(&buf[idx], &x, &y);
					
					#if 1
            if(( X_RESOLUTION > 0 )&& ( Y_RESOLUTION > 0 ))
            {
              x = ( x * LCM_X_MAX ) / X_RESOLUTION;
              y = ( y * LCM_Y_MAX ) / Y_RESOLUTION;
            }
            else
            {
              x = ( x * LCM_X_MAX ) / ELAN_X_MAX;
              y = ( y * LCM_Y_MAX )  /ELAN_Y_MAX;
            }
          #endif
					
					input_report_key(idev, BTN_TOUCH, 1); //for all finger down
					touch_size = ((i & 0x01) ? buf[size_index[i]] : (buf[size_index[i]] >> 4)) & 0x0F;
					pressure_size = touch_size << 4; // shift left touch size value to 4 bits for max pressure value 255   	      	
					input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 100);
					input_report_key(idev, BTN_TOOL_FINGER, 1); //[SM20]zihweishen add for test 2015/12/25
					input_report_abs(idev, ABS_MT_PRESSURE, 100);
					input_report_abs(idev, ABS_MT_POSITION_X, x);
					input_report_abs(idev, ABS_MT_POSITION_Y, y);
					
					//if(unlikely(gPrint_point)) 
					touch_debug(DEBUG_INFO, "[elan] finger id=%d x=%d y=%d size=%d press=%d \n", i, x, y, touch_size, pressure_size);
				}
			}
			mTouchStatus[i] = active;
			fbits = fbits >> 1;
			idx += 3;
		}
		if (num == 0){
			//printk("[ELAN] ALL Finger Up\n");
			input_report_key(idev, BTN_TOUCH, 0); //for all finger up
			input_report_key(idev, BTN_TOOL_FINGER, 0);//[SM20]zihweishen add for test 2015/12/25
			force_release_pos(client);
		}
		input_sync(idev);
		break;
	case IamLive_PKT://0512
		touch_debug(DEBUG_TRACE,"%x %x %x %x\n",buf[0],buf[1],buf[2],buf[3] );
		break;

	default:
		dev_err(&client->dev,
		"[elan] %s: unknown packet type: %0x\n", __func__, buf[0]);
		break;
	} // end switch

	return;
}

#endif


#ifdef PROTOCOL_A
//[SM20]zihweishen modify protocol_B for report error coordinate end 2015/11/23
#if defined( SOFTKEY_AXIS_VER ) /* SOFTKEY is reported via AXI */
static void elan_ktf2k_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
//struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
struct input_dev *idev = tpd->dev;
uint16_t  x, y;
uint16_t  fbits=0;
int   finger_num;
int   limitY = ELAN_Y_MAX - 100; /* limitY need define by Case! */
uint8_t   i, num, reported = 0;
uint8_t   idx, btn_idx;

/* for 10 fingers */
  if( buf[0] == TEN_FINGERS_PKT )
  {
    finger_num = 10;
    num   = buf[2] & 0x0F;
    fbits = buf[2] & 0x30;
    fbits = ( fbits << 4 ) | buf[1];
    idx   = 3;
    btn_idx = 33;
  }
/* for 5 fingers */
  else if(( buf[0] == MTK_FINGERS_PKT ) || ( buf[0] == FIVE_FINGERS_PKT ))
  {
    finger_num = 5;
    num   = buf[1] & 0x07;
    fbits = buf[1] >> 3;
    idx   = 2;
    btn_idx = 17;
  }
/* For 2 fingers */
  else
  {
    finger_num = 2;
    num   = buf[7] & 0x03;
    fbits = buf[7] & 0x03;
    idx   = 1;
    btn_idx = 7;
  }

  switch( buf[0] )
  {
//<<Mel - 4/10, Add 0x78 packet.
    case 0x78 :  /* chip may reset due to watch dog */
    {
      //printk(KERN_EMERG "!!!!!!!tp chip check event\n");
    } break;
//Mel - 4/10, Add 0x78 packet>>.

    case MTK_FINGERS_PKT:
    case TWO_FINGERS_PKT:
    case FIVE_FINGERS_PKT:
    case TEN_FINGERS_PKT:
    {
    //input_report_key(idev, BTN_TOUCH, 1);
      if( num == 0 )
      {
      //dev_dbg(&client->dev, "no press\n");
        if( key_pressed < 0 )
        {
          input_report_abs( idev, ABS_MT_TOUCH_MAJOR, 0 );
          input_report_abs( idev, ABS_MT_WIDTH_MAJOR, 0 );
          input_mt_sync( idev );
          if(( FACTORY_BOOT == get_boot_mode() ) || ( RECOVERY_BOOT == get_boot_mode()))
          {
            tpd_button( x, y, 0 );
          }
            TPD_EM_PRINT( x, y, x, y, 0, 0 );
        }
        else
        {
        //dev_err( &client->dev, "[elan] KEY_RELEASE: key_code:%d\n",OSD_mapping[key_pressed].key_event);
          input_report_key( idev, OSD_mapping[key_pressed].key_event, 0 );
          key_pressed = -1;
        }
      }
      else
      {
      //dev_dbg( &client->dev, "[elan] %d fingers\n", num );
      //input_report_key( idev, BTN_TOUCH, 1 );
        for( i = 0; i < finger_num; i++ )
        {
          if(( fbits & 0x01 ))
          {
          #if 1
            elan_ktf2k_ts_parse_xy( &buf[idx], &x, &y );
          #else
            elan_ktf2k_ts_parse_xy( &buf[idx], &y, &x );
            x = X_RESOLUTION - x;
            y = Y_RESOLUTION - y;
          #endif

          #if 1
            if(( X_RESOLUTION > 0 ) && ( Y_RESOLUTION > 0 ))
            {
              //x = ( x * LCM_X_MAX ) / X_RESOLUTION;
              //y = ( y * LCM_Y_MAX ) / Y_RESOLUTION;
              x = X_RESOLUTION;
              y = Y_RESOLUTION;
            }
            else
            {
              x = ( x * LCM_X_MAX ) / ELAN_X_MAX;
              y = ( y * LCM_Y_MAX ) / ELAN_Y_MAX;
            }
          #endif
            //intk("[elan_debug SOFTKEY_AXIS_VER] %s, x=%d, y=%d\n",__func__, x , y );
            printk("[elan_debug SOFTKEY_AXIS_VER]  x=%d, y=%d\n" , x , y );

            if( !(( x <= 0 ) || ( y <= 0 ) || ( x >= X_RESOLUTION ) || ( y >= Y_RESOLUTION )))
            {
              if( y < limitY )
              {
                input_report_abs( idev, ABS_MT_TRACKING_ID, i );
                input_report_abs( idev, ABS_MT_TOUCH_MAJOR, 8 );
                input_report_abs( idev, ABS_MT_POSITION_X, x );
                input_report_abs( idev, ABS_MT_POSITION_Y, y );
                input_mt_sync( idev );
                if(( FACTORY_BOOT == get_boot_mode()) || ( RECOVERY_BOOT == get_boot_mode()))
                {
                  tpd_button( x, y, 1 );
                }
                TPD_EM_PRINT( x, y, x, y, i - 1, 1 );
              }
              else
              {
              int j = 0;

                for( j = 0; j < 4; j++)
                {
                  if(( x > OSD_mapping[j].left_x ) && ( x < OSD_mapping[j].right_x ))
                  {
                  //dev_err(&client->dev, "[elan] KEY_PRESS: key_code:%d\n",OSD_mapping[j].key_event );
                  //printk("[elan] %d KEY_PRESS: key_code:%d\n", j, OSD_mapping[j].key_event );
                    input_report_key( idev, OSD_mapping[j].key_event, 1 );
                    key_pressed = j;
                  }
                }
              }
              reported++;
            } /* End if(!((x<=0)||..) border */
          } /* End.. if finger status */
            fbits = fbits >> 1;
            idx += 3;
        } /* End.. for(i=0;..) */
      } /* End.. if(num).. else */

      if( reported )
        input_sync( idev );
      else
      {
        input_mt_sync( idev );
        input_sync( idev );
      }

    } break;

    default:
    {
      //dev_err( &client->dev, "[elan] %s: unknown packet type: %02X\n", __func__, buf[0] );
    } break;
  } /* End.. switch */

  return;
}

#else /* SOFTKEY is reported via BTN bit */
static void elan_ktf2k_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
//struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
struct input_dev *idev = tpd->dev;
uint16_t  x=0, y=0;
uint16_t  fbits=0;
int       finger_num;
uint8_t   i, num, reported = 0;
uint8_t   idx, btn_idx;

/* For 10 fingers */
  if( buf[0] == TEN_FINGERS_PKT )
  {
    finger_num = 10;
    num   = buf[2] & 0x0F;
    fbits = buf[2] & 0x30;
    fbits = ( fbits << 4 ) | buf[1];
    idx   = 3;
    btn_idx = 33;
  }
/* For 5 fingers */
  else if(( buf[0] == MTK_FINGERS_PKT ) || ( buf[0] == FIVE_FINGERS_PKT ))
  {
    finger_num = 5;
    num   = buf[1] & 0x07;
    fbits = buf[1] >>3;
    idx   = 2;
    btn_idx = 17;
  }
/* For 2 fingers */
  else
  {
    finger_num = 2;
    num   = buf[7] & 0x03;
    fbits = buf[7] & 0x03;
    idx   = 1;
    btn_idx = 7;
  }

  switch( buf[0] )
  {
  //<<Mel - 4/10, Add 0x78 packet.
    case 0x78 :  /* chip may reset due to watch dog */
    {
    //printk(KERN_EMERG "!!!!!!!tp chip check event\n");
    } break;
  //Mel - 4/10, Add 0x78 packet>>.
    case MTK_FINGERS_PKT:
    case TWO_FINGERS_PKT:
    case FIVE_FINGERS_PKT:
    case TEN_FINGERS_PKT:
    {
    
      if( num == 0 )
      {
        dev_dbg(&client->dev, "no press\n");
      #if defined( ELAN_DEBUG )
        printk("tp button_state0 = %X\n", button_state );
        printk("tp buf[btn_idx] = %X, KEY_MENU = %X, KEY_HOME = %X, KEY_BACK = %X, KEY_SEARCH = %X\n",
            buf[btn_idx], KEY_MENU, KEY_HOME, KEY_BACK, KEY_SEARCH );
      #endif
      #if defined( ELAN_BUTTON )
        switch( buf[btn_idx] )
        {
          case ELAN_KEY_BACK:
          {
            printk("KEY back 1\n");
          #if !defined( LCT_VIRTUAL_KEY )
            input_report_key( idev, KEY_BACK, 1 );
          #else
            input_report_key( idev, BTN_TOUCH, 1 );
            input_report_abs( idev, ABS_MT_TOUCH_MAJOR, 8 );
            input_report_abs( idev, ABS_MT_POSITION_X, 617 );
            input_report_abs( idev, ABS_MT_POSITION_Y, 1360 );
          #endif
            button_state = KEY_BACK;
          } break;

          case ELAN_KEY_HOME:
          {
            printk("KEY home 1\n");
          #if !defined( LCT_VIRTUAL_KEY )
            input_report_key( idev, KEY_HOMEPAGE, 1 );
          #else
            input_report_key( idev, BTN_TOUCH, 1 );
            input_report_abs( idev, ABS_MT_TOUCH_MAJOR, 8 );
            input_report_abs( idev, ABS_MT_POSITION_X, 365 );
            input_report_abs( idev, ABS_MT_POSITION_Y, 1360 );
          #endif
            button_state = KEY_HOMEPAGE;
          } break;

          case ELAN_KEY_MENU:
          {
            printk("KEY menu 1\n");
          #ifndef LCT_VIRTUAL_KEY
            input_report_key( idev, KEY_MENU, 1 );
          #else
            input_report_key( idev, BTN_TOUCH, 1 );
            input_report_abs( idev, ABS_MT_TOUCH_MAJOR, 8 );
            input_report_abs( idev, ABS_MT_POSITION_X, 107 );
            input_report_abs( idev, ABS_MT_POSITION_Y, 1360 );
          #endif
            button_state = KEY_MENU;
          } break;

        /* TOUCH release*/
          default:
          {
            printk("[ELAN ] test tpd up\n");
            input_report_key( idev, BTN_TOUCH, 0 );
            input_report_abs( idev, ABS_MT_TOUCH_MAJOR, 0 );
            input_report_abs( idev, ABS_MT_WIDTH_MAJOR, 0 );
            input_mt_sync( idev );
            tpd_down_flag = 0;
          } break;
        } /* End.. switch(buf[btn_idx]) */
      //input_sync(idev);
      #endif /* End.. (ELAN_BUTTON) */
        if(( FACTORY_BOOT == get_boot_mode()) || ( RECOVERY_BOOT == get_boot_mode()))
        {
          tpd_button( x, y, 0);
        }
        TPD_EM_PRINT( x, y, x, y, 0, 0 );
      }
      else
      {
		//dev_dbg(&client->dev, "[elan] %d fingers\n", num);
        //printk( "[elan] %d fingers\n", num);
        input_report_key( idev, BTN_TOUCH, 1 );
        printk("[elan] input_report_key, After BTN_TOUCH\n");		
        for( i = 0; i < finger_num; i++ )
        {
          if(( fbits & 0x01 ))
          {
            elan_ktf2k_ts_parse_xy( &buf[idx], &x, &y );
          //elan_ktf2k_ts_parse_xy(&buf[idx], &y, &x );
          #if 1
            if(( X_RESOLUTION > 0 )&& ( Y_RESOLUTION > 0 ))
            {
              x = ( x * LCM_X_MAX ) / X_RESOLUTION;
              y = ( y * LCM_Y_MAX ) / Y_RESOLUTION;
            }
            else
            {
              x = ( x * LCM_X_MAX ) / ELAN_X_MAX;
              y = ( y * LCM_Y_MAX )  /ELAN_Y_MAX;
            }
          #endif
          //x = ( x * LCM_X_MAX ) / ELAN_X_MAX;
          //y = ( y * LCM_Y_MAX ) / ELAN_Y_MAX;


          #if defined( ELAN_DEBUG )
            //printk("[elan_debug  BTN bit] %s, x=%d, y=%d\n",__func__, x , y );
            printk("[elan_debug  BTN bit] , x=%d, y=%d\n", x , y );
          #endif
            //x = LCM_X_MAX - x;
            //y = Y_RESOLUTION - y;
            if( !(( x <= 0 ) || ( y <= 0) || ( x >= LCM_X_MAX ) || ( y >= LCM_Y_MAX )))
            {
          //[SM20]zihweishen modify protocol_A for report touch error 2015/12/25 begin
              input_report_key( idev, BTN_TOUCH, 1 );
              input_report_abs( idev, ABS_MT_TOUCH_MAJOR, 8 );
              input_report_abs( idev, ABS_MT_POSITION_X, x );
              input_report_abs( idev, ABS_MT_POSITION_Y, y );
              input_report_abs( idev, ABS_MT_PRESSURE, 100 );
	//[SM20]zihweishen modify protocol_A for report touch error 2015/12/25 end
              input_mt_sync( idev );
              reported++;
              tpd_down_flag = 1;
              if(( FACTORY_BOOT == get_boot_mode()) || ( RECOVERY_BOOT == get_boot_mode()))
              {
                tpd_button( x, y, 1 );
              }
              TPD_EM_PRINT(x, y, x, y, i - 1, 1 );
            } /* End.. if border */
          } /* end.. if finger status */
          fbits = fbits >> 1;
          idx += 3;
        } /* End.. for */
      } /* End.. if(num==0).. else */

      if( reported )
        input_sync( idev );
      else
      {
        input_mt_sync( idev );
        input_sync( idev );
      }
    } break;

    default:
    {
      //printk("[elan] %s: unknown packet type: %0X\n", __func__, buf[0]);
      printk("[elan] : unknown packet type: %0X\n", buf[0]);
    } break;
  } /* End.. switch(buf[0]) */

  return;
}
#endif /* End.. (SOFTKEY_AXIS_VER) */
//[SM20]zihweishen modify protocol_B for report error coordinate begin 2015/11/23
#endif  //#ifdef PROTOCOL_A

#if 0
static int touch_event_handler(void *unused)
{
struct sched_param  param = { .sched_priority = RTPM_PRIO_TPD };
int   rc;
uint8_t buf[PACKET_SIZE] = { 0 };

  sched_setscheduler( current, SCHED_RR, &param );

  do
  {
    //mt_eint_unmask( CUST_EINT_TOUCH_PANEL_NUM );  //enable_irq(CUST_EINT_TOUCH_PANEL_NUM);
    elan_irq_enable();

    set_current_state( TASK_INTERRUPTIBLE );
    wait_event_interruptible( waiter, tpd_flag != 0 );
    tpd_flag = 0;
    set_current_state( TASK_RUNNING );
    //mt_eint_mask( CUST_EINT_TOUCH_PANEL_NUM );  //disable_irq(CUST_EINT_TOUCH_PANEL_NUM);
    elan_irq_disable();

    rc = elan_ktf2k_ts_recv_data( private_ts->client, buf );
    if( rc < 0 )
    {
      printk("[elan] rc<0\n");
      continue;
    }

    elan_ktf2k_ts_report_data(/*ts*/private_ts->client, buf );

  } while( !kthread_should_stop());

  return 0;
}
#endif

static int tpd_detect(struct i2c_client *client, struct i2c_board_info *info)
{
  strcpy( info->type, TPD_DEVICE );
  return 0;
}

static irqreturn_t tpd_eint_interrupt_handler(unsigned irq, struct irq_desc *desc)
{
  
  int rc;
  uint8_t buf[PACKET_SIZE] = { 0 };
#if 0
  unsigned long flags;
  spin_lock_irqsave(&irq_flag_lock, flags);

  if (irq_flag == 0) 
  {
      spin_unlock_irqrestore(&irq_flag_lock, flags);
      return IRQ_HANDLED;
  }
  /* enter EINT handler disable INT, make sure INT is disable when handle touch event including top/bottom half */
  /* use _nosync to avoid deadlock */
  irq_flag = 0;
  spin_unlock_irqrestore(&irq_flag_lock, flags);
  disable_irq_nosync(touch_irq);
  
  wake_up_interruptible( &waiter );
#endif

  
	if (gpio_get_value(private_ts->intr_gpio))
	{
		printk("[elan] Detected the jitter on INT pin");
		return IRQ_HANDLED;
	}
#if defined( ESD_CHECK )  //0604
  have_interrupts = 1;
#endif
  tpd_flag = 1;
  
  rc = elan_ktf2k_ts_recv_data(private_ts->client, buf);
	if (rc < 0)
	{
		printk("[elan] Received the packet Error.\n");
		return IRQ_HANDLED;
	}

	touch_debug(DEBUG_TRACE,"%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x ....., %2x\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[17]);

  elan_ktf2k_ts_report_data(private_ts->client, buf);
//[SM20]zihweishen modify protocol_B for report error coordinate end 2015/11/23
  

  return IRQ_HANDLED;
}

//Steven start
static int tpd_irq_registration(void)
{
    struct device_node *node = NULL;
    //struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);//[SM20]zihweishen modify protocol_B for report error coordinate 2015/11/23
    int ret = 0;
    u32 ints[2] = { 0, 0 };

    printk("Device Tree Tpd_irq_registration! \n");

    node = of_find_matching_node(node, touch_of_match);

    if (node) 
    {
        of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
        gpio_set_debounce(ints[0], ints[1]);

        touch_irq = irq_of_parse_and_map(node, 0);

//[SM20]zihweishen modify protocol_B for report error coordinate begin 2015/11/23
        //ret = request_irq(touch_irq, (irq_handler_t) tpd_eint_interrupt_handler, IRQF_TRIGGER_FALLING,"TOUCH_PANEL-eint", NULL);
        ret = request_threaded_irq(touch_irq, NULL, (irq_handler_t) tpd_eint_interrupt_handler, 
                                   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,"TOUCH_PANEL-eint", private_ts);     
//[SM20]zihweishen modify protocol_B for report error coordinate end 2015/11/23
        if (ret > 0) 
        {
            ret = -1;
            printk("tpd request_irq IRQ LINE NOT AVAILABLE!.");
        }
		
    } 
    else 
    {
        printk("tpd request_irq can not find touch eint device node!.");
        ret = -1;
    }
    
    printk("[%s]irq:%d, debounce:%d-%d: \n", __func__, touch_irq, ints[0], ints[1]);
	
    return ret;
}
//Steven end

static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
static struct elan_ktf2k_ts_data ts;
//int   err = 0;
int   fw_err = 0;
int   New_FW_ID;
int   New_FW_VER;
//int   i;
int   retval = TPD_OK;
struct task_struct *fw_update_thread;	// Thunder modify for update touch Firmware 20141229
//[SM20][zihweishen] Modify for detect elan and Synaptics touch driver begin 2015/10/14
/*[Lavender][bozhi_lin] Dynamic detect elan and Synaptics touch driver 20150119 begin*/
//#if defined(LAVENDER)
uint8_t buf_recv[8] = { 0 };
//#endif
/*[Lavender][bozhi_lin] 20150119 end*/
//[SM20][zihweishen] Modify for detect elan and Synaptics touch driver end 2015/10/14

//Steven start
  printk("[elan]tpd_probe \n");
//Steven end

  client->addr |= I2C_ENEXT_FLAG;

 // printk("[elan] %s: client addr is %x, TPD_DEVICE = %s\n", __func__, client->addr, TPD_DEVICE );
 // printk("[elan] %s: I2C_WR_FLAG = %x, I2C_MASK_FLAG= %x, I2C_ENEXT_FLAG = %x\n", __func__, I2C_WR_FLAG, I2C_MASK_FLAG, I2C_ENEXT_FLAG );

  //printk("[elan] %x = IOCTL_I2C_INT\n", IOCTL_I2C_INT );
  //printk("[elan] %x = IOCTL_IAP_MODE_LOCK\n", IOCTL_IAP_MODE_LOCK );
  //printk("[elan] %x = IOCTL_IAP_MODE_UNLOCK\n", IOCTL_IAP_MODE_UNLOCK );

  printk("[elan] : client addr is %x, TPD_DEVICE = %s\n",  client->addr, TPD_DEVICE );
  printk("[elan] : I2C_WR_FLAG = %x, I2C_MASK_FLAG= %x, I2C_ENEXT_FLAG = %x\n",  I2C_WR_FLAG, I2C_MASK_FLAG, I2C_ENEXT_FLAG );

//  printk("[elan] %x = IOCTL_I2C_INT\n", IOCTL_I2C_INT );
//  printk("[elan] %x = IOCTL_IAP_MODE_LOCK\n", IOCTL_IAP_MODE_LOCK );
//  printk("[elan] %x = IOCTL_IAP_MODE_UNLOCK\n", IOCTL_IAP_MODE_UNLOCK );

//client->addr   &= I2C_MASK_FLAG;
    client->timing  = 400; //I2C rate=400k Hz
  //client->timing  = 100; //I2C rate=100k Hz
#if 1
  i2c_client = client;
  private_ts = &ts;
  private_ts->client = client;
//private_ts->addr = I2C_ELAN_SLAVE_ADDR;
#endif

/* Power configuration. */
  elan_power_enable( ELAN_ENABLE );
  msleep( 10 );

#if 0
  /* LDO enable */
  mt_set_gpio_mode( GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO );
  mt_set_gpio_dir( GPIO_CTP_EN_PIN, GPIO_DIR_OUT );
  mt_set_gpio_out( GPIO_CTP_EN_PIN, GPIO_OUT_ZERO );
  msleep( 50 );
  mt_set_gpio_out( GPIO_CTP_EN_PIN, GPIO_OUT_ONE );
#endif
  printk("[elan] ELAN enter tpd_probe_ ,the I2C addr = 0x%02X\n", client->addr);
//printk("GPIO43 =%d,GPIO_CTP_EINT_PIN =%d,GPIO_DIR_IN=%d,CUST_EINT_TOUCH_PANEL_NUM=%d\n",GPIO43,GPIO_CTP_EINT_PIN,GPIO_DIR_IN,CUST_EINT_TOUCH_PANEL_NUM);

/* Reset Touch Pannel */
  GTP_GPIO_OUTPUT(GTP_RST_PORT, 1);
  mdelay(10);

  GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);
  mdelay(10);

  GTP_GPIO_OUTPUT(GTP_RST_PORT, 1);
  mdelay(50);

//[SM20][zihweishen] Modify for detect elan and Synaptics touch driver begin 2015/10/14
/*[Lavender][bozhi_lin] Dynamic detect elan and Synaptics touch driver 20150119 begin*/
//#if defined(LAVENDER)
  fw_err = i2c_master_recv( client, buf_recv, 4 );
  if (fw_err < 0)
  {
//[SM20][zihweishen] Modify for disable touch LDO_VGP1 in suspend state begin 2015/10/14
/*[Lavender][bozhi_lin] disable touch LDO_VGP1 in suspend state 20150327 begin*/
  elan_power_enable( ELAN_DISABLE );
/*[Lavender][bozhi_lin] 20150327 end*/
//[SM20][zihweishen] Modify for disable touch LDO_VGP1 in suspend state end 2015/10/14
    return -EIO;
  }
//#endif
/*[Lavender][bozhi_lin] 20150119 end*/
//[SM20][zihweishen] Modify for detect elan and Synaptics touch driver end 2015/10/14

#if defined( HAVE_TOUCH_KEY )
int retry;
  for( retry = 0; retry < 3; retry++)
  {
    input_set_capability( tpd->dev, EV_KEY, tpd_keys_local[retry] );
  }
#endif

//Steven start
    GTP_GPIO_AS_INT(GTP_INT_PORT);

    mdelay(50);

    tpd_irq_registration();

    elan_irq_enable();
//Steven end

  tpd_load_status = 1;

#if defined( ELAN_DMA_MODE )
  //gpDMABuf_va = (u8 *)dma_alloc_coherent( NULL, 4096, &gpDMABuf_pa, GFP_KERNEL ); //1112
  
  gpDMABuf_va = (u8 *)dma_alloc_coherent(&tpd->dev->dev, 4096, &gpDMABuf_pa, GFP_KERNEL); //1112
  if( !gpDMABuf_va )
  {
    printk(KERN_INFO "[elan] Allocate DMA I2C Buffer failed\n");
  }
#endif

  //mt_eint_mask( CUST_EINT_TOUCH_PANEL_NUM );
  elan_irq_disable();

  fw_err = elan_ktf2k_ts_setup( client );
  if( fw_err < 0 )
  {
    printk( KERN_INFO "[elan] No Elan chip inside\n");
  }
  //mt_eint_unmask( CUST_EINT_TOUCH_PANEL_NUM );
  elan_irq_enable();

//[SM20]zihweishen modify protocol_B for report error coordinate begin 2015/11/23
#ifdef PROTOCOL_A
	input_set_abs_params(tpd->dev, ABS_X, 0,  X_RESOLUTION, 0, 0);
	input_set_abs_params(tpd->dev, ABS_Y, 0,  Y_RESOLUTION, 0, 0);
	input_set_abs_params(tpd->dev, ABS_TOOL_WIDTH, 0, MAX_FINGER_SIZE, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_TRACKING_ID, 0, FINGER_NUM, 0, 0);	
#endif
//<2015/11/17-Zihwei Shen modify
	input_set_abs_params(tpd->dev, ABS_MT_POSITION_Y, 0, Y_RESOLUTION, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_POSITION_X, 0, X_RESOLUTION, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_TOUCH_MAJOR, 0, MAX_FINGER_SIZE, 0, 0);
	input_set_abs_params(tpd->dev, ABS_PRESSURE, 0, MAX_FINGER_SIZE, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_PRESSURE, 0, MAX_FINGER_SIZE, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_DISTANCE, 0, MAX_FINGER_SIZE, 0, 0);
//<2015/11/17-Zihwei Shen modify

#ifdef PROTOCOL_B
	input_mt_init_slots(tpd->dev, FINGER_NUM, 1);
#endif

#if !defined( LCT_VIRTUAL_KEY )
  set_bit( KEY_BACK,  tpd->dev->keybit );
  set_bit( KEY_HOMEPAGE,  tpd->dev->keybit );
  set_bit( KEY_MENU,  tpd->dev->keybit );
#endif

#if 0
  thread = kthread_run( touch_event_handler, 0, TPD_DEVICE );
  if( IS_ERR( thread ))
  {
    retval = PTR_ERR( thread );
    printk(TPD_DEVICE "[elan]  failed to create kernel thread: %d\n", retval);
  }
#endif
//[SM20]zihweishen modify protocol_B for report error coordinate end 2015/11/23
  
  printk("[elan]  ELAN Touch Panel Device Probe %s\n", ( retval < TPD_OK ) ? "FAIL" : "PASS");
// [All][Main][Glove mode][Feature][number][potingchang] Touch Glove mode   20141219 BEGIN
//glove mode register
#if defined( GLOVEMODE )
	elan_ktf2k_touch_sysfs_init();
#endif
// [All][Main][Glove mode][Feature][number][potingchang] 20141219 END

/* Firmware Update */
  /* MISC */
  ts.firmware.minor = MISC_DYNAMIC_MINOR;
  ts.firmware.name = "elan-iap";
  ts.firmware.fops = &elan_touch_fops;
  ts.firmware.mode = S_IRWXUGO;

  if( misc_register( &ts.firmware ) < 0 )
    printk("[elan] misc_register failed!!");
  else
    printk("[elan] misc_register finished!!");
/* End Firmware Update */

#if defined( ESD_CHECK ) //0604
  INIT_DELAYED_WORK( &esd_work, elan_touch_esd_func );
  esd_wq = create_singlethread_workqueue( "esd_wq" );
  if( !esd_wq )
  {
    retval = -ENOMEM;
  }

  queue_delayed_work( esd_wq, &esd_work, delay );
#endif

#if( IAP_PORTION )
  if( IAP_PORTION )// Thunder modify for update touch Firmware 20141229
  {
    work_lock = 1;
    //mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);  //disable_irq(CUST_EINT_TOUCH_PANEL_NUM);/* Zihwei <- Modify for update touch Firmware. 2015/03/31*/
    power_lock = 1;
    printk("[elan] start fw update");

/* FW ID & FW VER*/
//[SM20][zihweishen] Do not need to distinguish file_fw_data_liano and file_fw_data_truly begin 2015/10/14
#if 0  /* For ektf21xx and ektf20xx  */
    /*
    printk("[ELAN]  [7bd0]=0x%02x,  [7bd1]=0x%02x, [7bd2]=0x%02x, [7bd3]=0x%02x\n",  file_fw_data[31696],file_fw_data[31697],file_fw_data[31698],file_fw_data[31699]);
    New_FW_ID = file_fw_data[31699]<<8  | file_fw_data[31698] ;
    New_FW_VER = file_fw_data[31697]<<8  | file_fw_data[31696] ;
    */
//>Zihwei 2015/03/23
static int  lcm_id_gpio;
    mt_set_gpio_mode( Touch_Detect_Pin, GPIO_MODE_GPIO );
    mt_set_gpio_dir( Touch_Detect_Pin, GPIO_DIR_IN );
    lcm_id_gpio = mt_get_gpio_in( Touch_Detect_Pin );
    //lcm_id_gpio = 1;
	if (!lcm_id_gpio)
	 file_fw_data=file_fw_data_liano;
//<Zihwei 2015/03/23
   	 New_FW_ID   = ( file_fw_data[0x7D67] << 8 ) | file_fw_data[0x7D66] ;
    	 New_FW_VER  = ( file_fw_data[0x7D65] << 8 ) | file_fw_data[0x7D64] ;

#endif
//[SM20][zihweishen] Do not need to distinguish file_fw_data_liano and file_fw_data_truly end 2015/10/14

   	 New_FW_ID   = ( file_fw_data[0x7D67] << 8 ) | file_fw_data[0x7D66] ;
    	 New_FW_VER  = ( file_fw_data[0x7D65] << 8 ) | file_fw_data[0x7D64] ;
#if 0
  /* for ektf31xx 2 wire ice ex: 2wireice -b xx.bin */
    printk(" [7c16]=0x%02x,  [7c17]=0x%02x, [7c18]=0x%02x, [7c19]=0x%02x\n",  file_fw_data[31766],file_fw_data[31767],file_fw_data[31768],file_fw_data[31769]);
    New_FW_ID   = ( file_fw_data[31769] << 8 )  | file_fw_data[31768] ;
    New_FW_VER  = ( file_fw_data[31767] << 8 )  | file_fw_data[31766] ;
#endif
#if 0
  /* for ektf31xx iap ekt file   */
    printk(" [7bd8]=0x%02x,  [7bd9]=0x%02x, [7bda]=0x%02x, [7bdb]=0x%02x\n",  file_fw_data[31704],file_fw_data[31705],file_fw_data[31706],file_fw_data[31707]);
    New_FW_ID   = ( file_fw_data[31707] << 8 ) | file_fw_data[31706] ;
    New_FW_VER  = ( file_fw_data[31705] << 8 ) | file_fw_data[31704] ;
#endif

    printk("[Elan] FW_ID=0x%x,   New_FW_ID=0x%x \n",  FW_ID, New_FW_ID);
    printk("[Elan] FW_VERSION=0x%x,   New_FW_VER=0x%x \n",  FW_VERSION  , New_FW_VER);

/* for firmware auto-upgrade  */
//<Zihwei 2015/08/05  no two project
//<Zihwei 2015/03/20
/*[C][Zihwei] enable touch FW of truly and liano for firmware auto-upgrade 201503/31 begin*/
    //if( New_FW_ID == FW_ID )
/*[C][Zihwei] enable touch FW of truly and liano for firmware auto-upgrade 201503/31 end*/
	if( RECOVERY==0x80 )
	{
            s32 err = 0;

	    file_fw_data = file_fw_data_truly;
            printk("[B]%s(%d): \n", __func__, __LINE__);

            fw_update_thread = kthread_run(Update_FW_One, 0, "elan_update");
            if (IS_ERR(fw_update_thread)) {
		err = PTR_ERR(fw_update_thread);
		printk(TPD_DEVICE " failed to create auto-update thread: %d\n", err);
            }
	}
	else
	{
		if( (New_FW_VER >  FW_VERSION ) || (New_FW_ID != FW_ID) )
		{
                        s32 err = 0;
			file_fw_data = file_fw_data_truly;
			printk("[B]%s(%d): \n", __func__, __LINE__);

			fw_update_thread = kthread_run(Update_FW_One, 0, "elan_update");
                        if (IS_ERR(fw_update_thread)) {
                            err = PTR_ERR(fw_update_thread);
                            printk(TPD_DEVICE " failed to create auto-update thread: %d\n", err);
                        }
		}
	}
  //20140521 tracy modify for update touch Fireware
   //Update_FW_One( client, RECOVERY );	// Thunder removed for update touch Firmware 20141229
//>Zihwei 2015/08/05  no two project
    work_lock = 0;
    //mt_eint_unmask( CUST_EINT_TOUCH_PANEL_NUM );  //enable_irq(CUST_EINT_TOUCH_PANEL_NUM);/* Zihwei <- Modify for update touch Firmware. 2015/03/31*/
  }
#endif /* End.. (IAP_PORTION) */
/*[C][Zihwei] Fix Touch ATS.apk read error issue 2015/03/25 begin*/ 
#if defined(TPD_REPORT_VENDOR_FW)
	{
		
		char buf[80]={0};
		sprintf(buf, "Elan_0x%4.4X", FW_VERSION);
		if (tpd_show_vendor_firmware == NULL) {
			tpd_show_vendor_firmware = kmalloc(strlen(buf) + 1, GFP_ATOMIC);
		}
		if (tpd_show_vendor_firmware != NULL) {
			strcpy(tpd_show_vendor_firmware, buf);
		}
	}
#endif
/*[C][Zihwei] 2015/03/25 end*/
return 0;

}

#if defined( ESD_CHECK ) //0604
static void elan_touch_esd_func(struct work_struct *work)
{
int   res;

  printk("[elan esd] %s: enter.......\n", __FUNCTION__);  /* elan_dlx */

  if( have_interrupts == 1 )
  {
    //printk("[elan esd] %s: had interrup not need check\n", __func__);
    printk("[elan esd] : had interrup not need check\n");
  }
  else
  {
    GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);
    msleep( 10 );

    /* for enable/reset pin */
    GTP_GPIO_OUTPUT(GTP_RST_PORT, 1);
    msleep( 100 );
  }

  have_interrupts = 0;
  queue_delayed_work( esd_wq, &esd_work, delay );
  printk("[elan esd] %s: exit.......\n", __FUNCTION__ );  /* elan_dlx */
}
#endif


static int tpd_remove(struct i2c_client *client)

{
  printk("[elan] TPD removed\n");

#if defined( ELAN_DMA_MODE )
  if( gpDMABuf_va )
  {
    dma_free_coherent( NULL, 4096, gpDMABuf_va, gpDMABuf_pa );
    gpDMABuf_va = NULL;
    gpDMABuf_pa = 0;
  }
#endif

  return 0;
}


static void tpd_suspend(struct device *h)
{
uint8_t cmd[] = { CMD_W_PKT, 0x50, 0x00, 0x01 };

  printk("[elan] TP enter into sleep mode\n");
  if(( i2c_master_send(private_ts->client, cmd, sizeof( cmd ))) != sizeof( cmd ))
  {
    //printk("[elan] %s: i2c_master_send failed\n", __func__);
    printk("[elan] : i2c_master_send failed\n");
    return;
  }
  //mt_eint_mask( CUST_EINT_TOUCH_PANEL_NUM );
  elan_irq_disable();

#if 0
  elan_power_enable( ELAN_DISABLE );
#endif
}


static void tpd_resume(struct device *h)
{
    printk("[elan]  wake up\n");

#if 0
  elan_power_enable( ELAN_ENABLE );
#endif
//<zihweishen 2015/08/05  no two project
//zihweishen modify touch resume after, touch not functional for awhile 2015/06/11 begin
/* Reset Touch Pannel */
    GTP_GPIO_OUTPUT(GTP_RST_PORT, 1);
    mdelay(20);

    GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);
    mdelay(20);
  
    GTP_GPIO_OUTPUT(GTP_RST_PORT, 1);
    mdelay(120);

//zihweishen modify touch resume after, touch not functional for awhile 2015/06/11 end
//>zihweishen 2015/08/05  no two project

  //mt_eint_unmask( CUST_EINT_TOUCH_PANEL_NUM );
  elan_irq_enable();

// [All][Main][Glove mode][Feature][number][potingchang] Touch Glove mode   20141219 BEGIN
//delay for reset
#if defined( GLOVEMODE )
	if (glovemode == true)// 49)
	{
		msleep(800);//it has to be delay,or glove aommand no work			
		elan_glove_enable( true );
		printk("glovemode tpd_resume  enter\n");

	}
// [All][Main][Glove mode][Feature][number][potingchang] 20141219 END
#endif
}

static int tpd_local_init(void)
{
  spin_lock_init(&irq_flag_lock);

  printk("[elan]: I2C Touchscreen Driver init\n");
  if(i2c_add_driver( &tpd_i2c_driver ) != 0)
  {
    printk("[elan]: unable to add i2c driver.\n");
    return -1;
  }

  if( tpd_load_status == 0 )
  {
    printk("ektf2152 add error touch panel driver.\n");
    i2c_del_driver( &tpd_i2c_driver );
    return -1;
  }

#if defined( TPD_HAVE_BUTTON )
#if defined( LCT_VIRTUAL_KEY )
  tpd_button_setting( TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local );  /* initialize tpd button data */
#endif
#endif
#if ( defined( TPD_WARP_START ) && defined( TPD_WARP_END ))
  TPD_DO_WARP = 1;
  memcpy( tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT * 4 );
  memcpy( tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT * 4 );
#endif
#if ( defined( TPD_HAVE_CALIBRATION ) && !defined( TPD_CUSTOM_CALIBRATION ))
  memcpy( tpd_calmat, tpd_def_calmat_local, 8 * 4 );
  memcpy( tpd_def_calmat, tpd_def_calmat_local, 8 * 4 );
#endif
  printk("[elan] end %s, %d\n", __FUNCTION__, __LINE__ );
  tpd_type_cap = 1;

  return 0;
}


static struct tpd_driver_t tpd_device_driver =
{
  .tpd_device_name = "ektf2k_mtk",
  .tpd_local_init = tpd_local_init,
  .suspend = tpd_suspend,
  .resume = tpd_resume,
#if defined( TPD_HAVE_BUTTON )
  .tpd_have_button = 1,
#else
  .tpd_have_button = 0,
#endif
};

static int __init tpd_driver_init(void)
{
    printk("[elan]: Driver Verison MTK0005 for MTK67XX serial\n");

    tpd_get_dts_info();

    printk("[elan] Enable ELAN_MTK6755\n");
    //i2c_register_board_info( I2C_ELAN_BUS, &i2c_tpd, 1 );

    if( tpd_driver_add(&tpd_device_driver) < 0 )
        printk("[elan]: driver failed \n");

    return 0;
}

static void __exit tpd_driver_exit(void)
{
  printk("[elan]: %s elan touch panel driver exit\n", __func__ );
  tpd_driver_remove( &tpd_device_driver );
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);




