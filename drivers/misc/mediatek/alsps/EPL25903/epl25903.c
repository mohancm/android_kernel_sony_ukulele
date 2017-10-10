/* drivers/hwmon/mt6516/amit/epl259x.c - EPL259x ALS/PS driver
 *
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
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


////..#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
////..#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
////..#include <linux/hwmsensor.h>
////..#include <linux/hwmsen_dev.h>
////..#include <linux/sensors_io.h>
#include <asm/io.h>
////..#include <cust_eint.h>
////..#include <cust_alsps.h>
#include "cust_alsps.h"
////..#include <linux/hwmsen_helper.h>
#include "epl259x.h"
#include <linux/input/mt.h>
////..#include <mach/devs.h>
////..#include <mach/mt_typedefs.h>
////..#include <mach/mt_gpio.h>
////..#include <mach/mt_pm_ldo.h>
//add for fix resume issue
////..#include <linux/earlysuspend.h>
#include <linux/wakelock.h>
//add for fix resume issue end
#include <alsps.h>
#include <linux/sched.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

/******************************************************************************
 * driver info
*******************************************************************************/
#define EPL_DEV_NAME   		    "EPL259x"
#define DRIVER_VERSION          "3.0.0_arima"

/******************************************************************************
 * ALS / PS sensor structure
*******************************************************************************/
// MTK_LTE is chose structure hwmsen_object or control_path, data_path
// MTK_LTE = 1, is control_path, data_path
// MTK_LTE = 0 hwmsen_object
////..#ifdef CONFIG_MTK_AUTO_DETECT_ALSPS
#define MTK_LTE         1
////..#else
////..#define MTK_LTE         0
////..#endif

#if MTK_LTE
#include <alsps.h>
#endif

#define ARIMA_PATCH 1
#define ARIMA_DYNAMIC_CAL 1//0 <2015/05/03-ShermanWei, Enable Dynamic Cal
#define PS_BOOT_K 1
/******************************************************************************
 *  PS_BOOT_K
 ******************************************************************************/
#if PS_BOOT_K
#define PS_BOOT_MAX_CT  250
#endif /*PS_BOOT_K*/


#if ARIMA_PATCH
#define CALI_APP_JNI 0
//<2015/04/16-ShermanWei, for Dynamic Cal
#define PS_CAL_DELTA_NORMAL 0
#define PS_CAL_DELTA_ABNORMAL 1
#define PS_CAL_FILE_NOEXIST 2
//>2015/04/16-ShermanWei

char high[8];
int high_length =0;
char ctalk[8];
int ctalk_length =0;

#endif
/******************************************************************************
 *  ALS / PS define
 ******************************************************************************/
#define LUX_PER_COUNT	400  //ALS lux per count
/*ALS interrupt table*/
unsigned long als_lux_intr_level[] = {15, 39, 63, 316, 639, 4008, 5748, 10772, 14517, 65535};
unsigned long als_adc_intr_level[10] = {0};

int als_frame_time = 0;
int ps_frame_time = 0;

/******************************************************************************
 *  factory setting
 ******************************************************************************/
static const char ps_cal_file[]="/data/data/com.eminent.ps.calibration/ps.dat";  //ps calibration file path
static const char als_cal_file[]="/data/data/com.eminent.ps.calibration/als.dat";  //als calibration file path

static int PS_h_offset = 3000;  //factory high threshold offset
static int PS_l_offset = 2000;  //factory low threshold offset
static int PS_MAX_XTALK = 30000;  //factory max crosstalk, if real crosstalk > max crosstalk, return fail

/******************************************************************************
 *I2C function define
*******************************************************************************/
#define TXBYTES 				2
#define PACKAGE_SIZE 			48
#define I2C_RETRY_COUNT 		2
int i2c_max_count=8;

/******************************************************************************
 * extern functions
*******************************************************************************/
#if defined(MT6575) || defined(MT6571) || defined(MT6589)
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_polarity(kal_uint8 eintno, kal_bool ACT_Polarity);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);
////...#else
////...extern void mt_eint_unmask(unsigned int line);
////...extern void mt_eint_mask(unsigned int line);
////...extern void mt_eint_set_polarity(kal_uint8 eintno, kal_bool ACT_Polarity);
////...extern void mt_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
////...extern kal_uint32 mt_eint_set_sens(kal_uint8 eintno, kal_bool sens);
////...extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
#endif

/******************************************************************************
 *  configuration
 ******************************************************************************/
#ifndef CUST_EINT_ALS_TYPE
#define CUST_EINT_ALS_TYPE  8
#endif

////..#define POWER_NONE_MACRO MT65XX_POWER_NONE

struct hwmsen_object *ps_hw, * als_hw;

struct alsps_hw alsps_cust;
static struct alsps_hw *hw = &alsps_cust;

static struct epl_sensor_priv *epl_sensor_obj = NULL;
#if !MTK_LTE
static struct platform_driver epl_sensor_alsps_driver;
#endif
static struct wake_lock ps_lock;
static struct mutex sensor_mutex;

static epl_optical_sensor epl_sensor;
static struct i2c_client *epl_sensor_i2c_client = NULL;
static const struct i2c_device_id epl_sensor_i2c_id[] = {{EPL_DEV_NAME,0},{}};
////..static struct i2c_board_info __initdata i2c_epl_sensor= { I2C_BOARD_INFO(EPL_DEV_NAME, (0x92>>1))};

#if ARIMA_PATCH
typedef struct _epl_ps_als_factory
{
    bool cal_file_exist;
    bool cal_finished;
    u16 ps_cal_h;
    u16 ps_cal_l;
//    char s1[16];
//    char s2[16];
}epl_ps_als_factory;
#endif

typedef struct _epl_raw_data
{
    u8 raw_bytes[PACKAGE_SIZE];
#if ARIMA_PATCH
///sherman modify_5
    u16 ps_state;
    u16 ps_raw;
    s16 cal_ps_delta;
    u32 mmisw;
	u32 ps_calfile_status;
	u16 ps_condition;
	u16 ps_sta;
	struct _epl_ps_als_factory ps_als_factory;
#endif
} epl_raw_data;
static epl_raw_data	gRawData;

#define APS_TAG                 	  	"[ALS/PS] "
#define APS_FUN(f)              	  	printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    	    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    	    printk(KERN_INFO APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    	    printk(KERN_INFO fmt, ##args)

typedef unsigned char       U8;

static int epl_sensor_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int epl_sensor_i2c_remove(struct i2c_client *client);
static int epl_sensor_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int epl_sensor_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int epl_sensor_i2c_resume(struct i2c_client *client);
////...static void epl_sensor_eint_func(void);
static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd);
static int set_lsensor_intr_threshold(uint16_t low_thd, uint16_t high_thd);
void epl_sensor_update_mode(struct i2c_client *client);
void epl_sensor_fast_update(struct i2c_client *client);
int epl_sensor_read_ps(struct i2c_client *client);
static int ps_sensing_time(int intt, int adc, int cycle);
static int als_sensing_time(int intt, int adc, int cycle);
static int als_intr_update_table(struct epl_sensor_priv *epld);
int epl_sensor_ps_data(void);
int epl_sensor_als_data(void);
#if PS_BOOT_K
void epl_sensor_ps_boot_k(struct i2c_client *client);
#endif
#if MTK_LTE
static int ps_open_report_data(int open);
static int ps_enable_nodata(int en);
static int ps_set_delay(u64 ns);
static int ps_get_data(int* value, int* status);
static int als_open_report_data(int open);
static int als_enable_nodata(int en);
static int als_set_delay(u64 ns);
static int als_get_data(int* value, int* status);
#else
int epl_sensor_ps_operate(void* self, uint32_t command, void* buff_in, int size_in, void* buff_out, int size_out, int* actualout);
int epl_sensor_als_operate(void* self, uint32_t command, void* buff_in, int size_in, void* buff_out, int size_out, int* actualout);
#endif

typedef enum
{
    CMC_BIT_ALS   	= 1,
    CMC_BIT_PS     	= 2,

} CMC_BIT;

typedef enum
{
    CMC_BIT_RAW   			= 0x0,
    CMC_BIT_PRE_COUNT     	= 0x1,
    CMC_BIT_DYN_INT			= 0x2,
    CMC_BIT_TABLE			= 0x3,
    CMC_BIT_INTR_LEVEL		= 0x4,
} CMC_ALS_REPORT_TYPE;

struct epl_sensor_i2c_addr      /*define a series of i2c slave address*/
{
    u8  write_addr;
};

struct epl_sensor_priv
{
    struct alsps_hw  *hw;
    struct i2c_client *client;
////...    struct delayed_work  eint_work;
    struct input_dev *gs_input_dev;
    /*i2c address group*/
    struct epl_sensor_i2c_addr  addr;
    /*misc*/
    atomic_t   	als_suspend;
    atomic_t    ps_suspend;
///sherman modify_5
    atomic_t	ps_thd_val_high;
    atomic_t	ps_thd_val_low;
    /*data*/

    u16		    lux_per_count;
    ulong       enable;         	/*record HAL enalbe status*/
    /*data*/
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];
    /*als interrupt*/
    int als_intr_level;
    int als_intr_lux;
///sherman modify_5
    int 	cover_state;
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif
};

#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{ .compatible = "mediatek,alsps", },
	{},
};
#endif

static struct i2c_driver epl_sensor_i2c_driver =
{
    .probe     	= epl_sensor_i2c_probe,
    .remove     = epl_sensor_i2c_remove,
    .detect     = epl_sensor_i2c_detect,
    .suspend    = epl_sensor_i2c_suspend,
    .resume     = epl_sensor_i2c_resume,
    .id_table   = epl_sensor_i2c_id,
    //.address_data = &epl_sensor_addr_data,
    .driver = {
        //.owner  = THIS_MODULE,
        .name   = EPL_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = alsps_of_match,
#endif
    },
};

#if MTK_LTE
static int alsps_init_flag =-1; // 0<==>OK -1 <==> fail
static int alsps_local_init(void);
static int alsps_remove(void);
static struct alsps_init_info epl_sensor_init_info = {
		.name = EPL_DEV_NAME,
		.init = alsps_local_init,
		.uninit = alsps_remove,
};
#endif
extern struct alsps_context *alsps_context_obj;

/*
static int epl_sensor_I2C_Write_Block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{   ///because address also occupies one byte, the maximum length for write is 7 bytes
    int err, idx, num;
    char buf[C_I2C_FIFO_SIZE];
    err =0;

    if (!client)
    {
        return -EINVAL;
    }
    else if (len >= C_I2C_FIFO_SIZE)
    {
        APS_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
        return -EINVAL;
    }

    num = 0;
    buf[num++] = addr;
    for (idx = 0; idx < len; idx++)
    {
        buf[num++] = data[idx];
    }

    err = i2c_master_send(client, buf, num);
    if (err < 0)
    {
        APS_ERR("send command error!!\n");

        return -EFAULT;
    }

    return err;
}
*/

static int epl_sensor_I2C_Write_Cmd(struct i2c_client *client, uint8_t regaddr, uint8_t data, uint8_t txbyte)
{
    uint8_t buffer[2];
    int ret = 0;
    int retry;

    buffer[0] = regaddr;
    buffer[1] = data;

    for(retry = 0; retry < I2C_RETRY_COUNT; retry++)
    {
        ret = i2c_master_send(client, buffer, txbyte);

        if (ret == txbyte)
        {
            break;
        }

        APS_ERR("i2c write error,TXBYTES %d\n",ret);
    }


    if(retry>=I2C_RETRY_COUNT)
    {
        APS_ERR("i2c write retry over %d\n", I2C_RETRY_COUNT);
        return -EINVAL;
    }

    return ret;
}

static int epl_sensor_I2C_Write(struct i2c_client *client, uint8_t regaddr, uint8_t data)
{
    int ret = 0;
    ret = epl_sensor_I2C_Write_Cmd(client, regaddr, data, 0x02);
    return ret;
}

static int epl_sensor_I2C_Read(struct i2c_client *client, uint8_t regaddr, uint8_t bytecount)
{
    int ret = 0;
    int retry;
    int read_count=0, rx_count=0;
    while(bytecount>0)
    {
        epl_sensor_I2C_Write_Cmd(client, regaddr+read_count, 0x00, 0x01);

        for(retry = 0; retry < I2C_RETRY_COUNT; retry++)
        {
            rx_count = bytecount>i2c_max_count?i2c_max_count:bytecount;
            ret = i2c_master_recv(client, &gRawData.raw_bytes[read_count], rx_count);

            if (ret == rx_count)
                break;

            APS_ERR("i2c read error %d\r\n",ret);
        }

        if(retry>=I2C_RETRY_COUNT)
        {
            APS_ERR("i2c read retry over %d\n", I2C_RETRY_COUNT);
            return -EINVAL;
        }
        bytecount-=rx_count;
        read_count+=rx_count;
    }

    return ret;
}



static void write_global_variable(struct i2c_client *client)
{
    u8 buf;

    epl_sensor_I2C_Write(client, 0x11, EPL_POWER_OFF | EPL_RESETN_RESET);
    //wake up chip
    //buf = epl_sensor.reset | epl_sensor.power;
    //epl_sensor_I2C_Write(client, 0x11, buf);

    /* read revno*/
    epl_sensor_I2C_Read(client, 0x20, 2);
    epl_sensor.revno = gRawData.raw_bytes[0] | gRawData.raw_bytes[1] << 8;

    /*chip refrash*/
    epl_sensor_I2C_Write(client, 0xfd, 0x8e);
    epl_sensor_I2C_Write(client, 0xfe, 0x22);
    epl_sensor_I2C_Write(client, 0xfe, 0x02);
    epl_sensor_I2C_Write(client, 0xfd, 0x00);

    epl_sensor_I2C_Write(client, 0xfc, EPL_A_D | EPL_NORMAL| EPL_GFIN_ENABLE | EPL_VOS_ENABLE | EPL_DOC_ON);

    {
        /*ps setting*/
        buf = epl_sensor.ps.integration_time | epl_sensor.ps.gain;
        epl_sensor_I2C_Write(client, 0x03, buf);

        buf = epl_sensor.ps.adc | epl_sensor.ps.cycle;
        epl_sensor_I2C_Write(client, 0x04, buf);

        buf = epl_sensor.ps.ir_on_control | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_drive;
        epl_sensor_I2C_Write(client, 0x05, buf);

        buf = epl_sensor.interrupt_control | epl_sensor.ps.persist |epl_sensor.ps.interrupt_type;
        epl_sensor_I2C_Write(client, 0x06, buf);

        buf = epl_sensor.ps.compare_reset | epl_sensor.ps.lock;
        epl_sensor_I2C_Write(client, 0x1b, buf);

        epl_sensor_I2C_Write(client, 0x22, (u8)(epl_sensor.ps.cancelation& 0xff));
        epl_sensor_I2C_Write(client, 0x23, (u8)((epl_sensor.ps.cancelation & 0xff00) >> 8));
        ///sherman modify_4
        ////set_psensor_intr_threshold(epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);

        /*als setting*/
        buf = epl_sensor.als.integration_time | epl_sensor.als.gain;
        epl_sensor_I2C_Write(client, 0x01, buf);

        buf = epl_sensor.als.adc | epl_sensor.als.cycle;
        epl_sensor_I2C_Write(client, 0x02, buf);

        buf = epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type;
        epl_sensor_I2C_Write(client, 0x07, buf);

        buf = epl_sensor.als.compare_reset | epl_sensor.als.lock;
        epl_sensor_I2C_Write(client, 0x12, buf);

        set_lsensor_intr_threshold(epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);
	}

    //set mode and wait
    buf = epl_sensor.wait | epl_sensor.mode;
    epl_sensor_I2C_Write(client, 0x00, buf);

}

static int write_factory_calibration(struct epl_sensor_priv *epl_data, char* ps_data, int ps_cal_len)
{
    struct file *fp_cal;

	mm_segment_t fs;
	loff_t pos;

	APS_FUN();
    pos = 0;

	fp_cal = filp_open(ps_cal_file, O_CREAT|O_RDWR|O_TRUNC, 0755/*S_IRWXU*/);
	if (IS_ERR(fp_cal))
	{
		APS_ERR("[ELAN]create file_h error\n");
		return -1;
	}

    fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_write(fp_cal, ps_data, ps_cal_len, &pos);

    filp_close(fp_cal, NULL);

	set_fs(fs);

	return 0;
}
#if !ARIMA_PATCH
static bool read_factory_calibration(void)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    struct file *fp;
    mm_segment_t fs;
    loff_t pos;
    char buffer[100]= {0};
    if(epl_sensor.ps.factory.calibration_enable && !epl_sensor.ps.factory.calibrated)
    {
        int ps_cancelation = 0, ps_hthr = 0, ps_lthr = 0;
		fp = filp_open(ps_cal_file, O_RDWR, S_IRUSR);
        if (IS_ERR(fp))
        {
            APS_ERR("NO calibration file\n");
            epl_sensor.ps.factory.calibration_enable =  false;
        }
        else
        {
            pos = 0;
            fs = get_fs();
            set_fs(KERNEL_DS);
            vfs_read(fp, buffer, sizeof(buffer), &pos);
            filp_close(fp, NULL);

            sscanf(buffer, "%d,%d,%d", &ps_cancelation, &ps_hthr, &ps_lthr);
			epl_sensor.ps.factory.cancelation = ps_cancelation;
			epl_sensor.ps.factory.high_threshold = ps_hthr;
			epl_sensor.ps.factory.low_threshold = ps_lthr;
            set_fs(fs);

            epl_sensor.ps.high_threshold = epl_sensor.ps.factory.high_threshold;
            epl_sensor.ps.low_threshold = epl_sensor.ps.factory.low_threshold;
            epl_sensor.ps.cancelation = epl_sensor.ps.factory.cancelation;
        }

        epl_sensor_I2C_Write(obj->client,0x22, (u8)(epl_sensor.ps.cancelation& 0xff));
        epl_sensor_I2C_Write(obj->client,0x23, (u8)((epl_sensor.ps.cancelation & 0xff00) >> 8));
        set_psensor_intr_threshold(epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);

        epl_sensor.ps.factory.calibrated = true;
    }

    if(epl_sensor.als.factory.calibration_enable && !epl_sensor.als.factory.calibrated)
    {
        fp = filp_open(ps_cal_file, O_RDWR, S_IRUSR);
        if (IS_ERR(fp))
        {
            APS_ERR("NO calibration file\n");
            epl_sensor.als.factory.calibration_enable =  false;
        }
        else
        {
            int als_lux_per_count = 0;
            pos = 0;
            fs = get_fs();
            set_fs(KERNEL_DS);
            vfs_read(fp, buffer, sizeof(buffer), &pos);
            filp_close(fp, NULL);
            sscanf(buffer, "%d", &als_lux_per_count);
			epl_sensor.als.factory.lux_per_count = als_lux_per_count;
            set_fs(fs);
        }
        epl_sensor.als.factory.calibrated = true;
    }
    return true;
}
#endif
#if ARIMA_PATCH

static int arima_ps_calibration_read(struct epl_sensor_priv *epl_data)
{
	struct file *fp_h;
	struct file *fp_ct;
    //struct i2c_client *client = epl_data->client;
	mm_segment_t fs;
	loff_t pos;
    int read_ret = 0;
    //char data[4] = {0};
    U8 data[4];
	u32 cal_value_h = 0;
	u32 cal_value_l = 0;
//<2015/04/16-ShermanWei, for Dynamic Cal
	gRawData.ps_calfile_status = PS_CAL_DELTA_NORMAL;
	gRawData.mmisw = 1;
//>2015/04/16-ShermanWei
	APS_LOG("[ELAN] %s \n", __func__);

	fp_h = filp_open("/data/prod/h-threshold.dat", O_RDONLY, 0755);
	if (IS_ERR(fp_h))
	{
		APS_ERR("[ELAN] /data/prod/h-threshold.dat not exist\n");
		fp_h = filp_open("/protect_f/h-threshold.dat", O_RDONLY, 0755);
		if (IS_ERR(fp_h))
		{
			APS_ERR("[ELAN] /protect_f/h-threshold.dat not exist\n");
			///// Set default Delta
			gRawData.ps_als_factory.ps_cal_h = 0;
			gRawData.ps_als_factory.ps_cal_l = 0;
//<2015/04/16-ShermanWei, for Dynamic Cal
			gRawData.cal_ps_delta = 5000;//3800;///500;//100;
			gRawData.ps_calfile_status = PS_CAL_FILE_NOEXIST;
///sherman modify_5
#if !ARIMA_DYNAMIC_CAL
			set_psensor_intr_threshold(epl_data->hw ->ps_threshold_low,epl_data->hw ->ps_threshold_high);
#endif			
//>2015/04/16-ShermanWei
			return 0;
		}
	}

	fp_ct = filp_open("/data/prod/cross-talk.dat", O_RDONLY, 0755);
	if (IS_ERR(fp_ct))
	{
		APS_ERR("[ELAN] /data/prod/cross-talk.dat not exist\n");
		fp_ct = filp_open("/protect_f/cross-talk.dat", O_RDONLY, 0755);
		if (IS_ERR(fp_ct))
		{
			APS_ERR("[ELAN] /protect_f/cross-talk.dat not exist\n");
			///// Set default Delta
			gRawData.ps_als_factory.ps_cal_h = 0;
			gRawData.ps_als_factory.ps_cal_l = 0;
//<2015/04/16-ShermanWei, for Dynamic Cal
			gRawData.cal_ps_delta = 5000;//3800;///500;//100;
			gRawData.ps_calfile_status = PS_CAL_FILE_NOEXIST;
///sherman modify_5
#if !ARIMA_DYNAMIC_CAL
			set_psensor_intr_threshold(epl_data->hw ->ps_threshold_low,epl_data->hw ->ps_threshold_high);
#endif			
//>2015/04/16-ShermanWei
			return 0;
		}
	}

	fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
	read_ret = vfs_read(fp_h, data, sizeof(data), &pos);
	cal_value_h = (data[0] << 24) + (data[1] << 16) + (data[2] << 8) + data[3];

	pos = 0;
    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;

	read_ret = vfs_read(fp_ct, data, sizeof(data), &pos);
	cal_value_l = (data[0] << 24) + (data[1] << 16) + (data[2] << 8) + data[3];

	filp_close(fp_h, NULL);
	filp_close(fp_ct, NULL);
	set_fs(fs);

	APS_LOG("[ELAN] read cal_value_h: %x, cal_value_l : %x\n", cal_value_h, cal_value_l);
	gRawData.ps_als_factory.ps_cal_h = cal_value_h;
	gRawData.ps_als_factory.ps_cal_l = cal_value_l;

	gRawData.cal_ps_delta = gRawData.ps_als_factory.ps_cal_h - gRawData.ps_als_factory.ps_cal_l;
	APS_LOG("[ELAN]+ read cal_h: %d , cal_ct : %d\n", gRawData.ps_als_factory.ps_cal_h,gRawData.ps_als_factory.ps_cal_l);
//<2015/04/16-ShermanWei, for Dynamic Cal
	/// check calibration data if reasonable
#if ARIMA_DYNAMIC_CAL
	if ( (gRawData.cal_ps_delta <= 1300/*100*/) || (gRawData.cal_ps_delta >= 5000) ) {
	APS_LOG("[ELAN] Error:Abnormal Delta: %d\n", gRawData.cal_ps_delta);
	gRawData.ps_calfile_status = PS_CAL_DELTA_ABNORMAL;
	gRawData.cal_ps_delta = 5000;//3800;///1000;///500;//100;
	}
#endif
	///// CT+0.5Delta, CT+Delta
#if !ARIMA_DYNAMIC_CAL
    set_psensor_intr_threshold(gRawData.ps_als_factory.ps_cal_l + gRawData.cal_ps_delta/2, gRawData.ps_als_factory.ps_cal_l + gRawData.cal_ps_delta);
#endif
//>2015/04/16-ShermanWei

	return 1;
}

static int arima_als_calibration_read(struct epl_sensor_priv *epl_data)
{
	struct file *fp_h;
	mm_segment_t fs;
	loff_t pos;
    //char data[4];
    U8 data[4];
    int read_ret = 0;
	u32 cal_value = 0;
	APS_LOG("[ELAN] %s \n", __func__);

	fp_h = filp_open("/data/prod/als-h-threshold.dat", O_RDONLY, 0755);
	if (IS_ERR(fp_h))
	{
		APS_ERR("[ELAN] /data/prod/als-h-threshold.dat not exist\n");
		fp_h = filp_open("/protect_f/als-h-threshold.dat", O_RDONLY, 0755);
		if (IS_ERR(fp_h))
		{
			APS_ERR("[ELAN] /protect_f/als-h-threshold.dat not exist\n");
			///// Set default Delta
			epl_data->lux_per_count = LUX_PER_COUNT;
			return 0;
		}
	}

	fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
	read_ret = vfs_read(fp_h, data, sizeof(data), &pos);
	cal_value = (data[0] << 24) + (data[1] << 16) + (data[2] << 8) + data[3];

	filp_close(fp_h, NULL);
	set_fs(fs);
	APS_LOG("[ELAN]+ read cal_value: %x\n", cal_value);
	if(cal_value > 0)
	{
	    epl_data->lux_per_count = 500000 / cal_value;
	    ///<2016/06/17 shermanwei shift data for DMS09289269 support 5000~9000Lux
	    epl_sensor.als.factory.lux_per_count = (epl_data->lux_per_count) << 2;
		///>2016/06/17
    }
	else
	{
		APS_ERR("[ELAN]cal_value is zero. error!\n");
		return 0;
	}

	APS_LOG("[ELAN] lux_per_count : %d\n", epl_data->lux_per_count);

	return 1;
}
//>2014/11/25-Quakentsai, for calibration APK interface

static int epl_sensor_calibaration_write(struct epl_sensor_priv *epl_data, char* h_data, int h_length) /* ,  char* l_data, int l_length )*/
{
    struct file *fp_h;
    //struct file *fp_l;
    //struct i2c_client *client = epl_data->client;

    mm_segment_t fs;
    loff_t pos;
    //int read_ret = 0;

    APS_LOG(" %s\n", __func__);

    fp_h = filp_open("/protect_f/h-threshold.dat", O_CREAT|O_RDWR|O_TRUNC, 0755/*S_IRWXU*/);
    if (IS_ERR(fp_h))
	{
		APS_ERR("[ELAN]create file_h error\n");
		return -1;
	}

    fs = get_fs();
    set_fs(KERNEL_DS);
    pos = 0;
    APS_ERR("[ELAN]h_length=%d \n",h_length);
    vfs_write(fp_h, h_data, h_length, &pos);

    filp_close(fp_h, NULL);
    set_fs(fs);

    return 0;
}


static int epl_sensor_crosstalk_write(struct epl_sensor_priv *epl_data, char* ct_data, int ct_length)
{
    struct file *fp_ct;
    //struct i2c_client *client = epl_data->client;
    mm_segment_t fs;
    loff_t pos;
    //int read_ret = 0;

    APS_LOG(" %s\n", __func__);

    fp_ct = filp_open("/protect_f/cross-talk.dat", O_CREAT|O_RDWR|O_TRUNC, 0755/*S_IRWXU*/);
    if (IS_ERR(fp_ct))
    {
	APS_ERR("[ELAN]create file_h error\n");
	return -1;
    }

    fs = get_fs();
    set_fs(KERNEL_DS);
    pos = 0;
    APS_ERR("[ELAN]ct_length=%d \n",ct_length);
    vfs_write(fp_ct, ct_data, ct_length, &pos);
    filp_close(fp_ct, NULL);
    set_fs(fs);

    ////gRawData.ps_als_factory.ps_cal_file_exist = 1;

	return 0;
}
#endif

static int epl_run_ps_calibration(struct epl_sensor_priv *epl_data)
{
    struct epl_sensor_priv *epld = epl_data;
    u16 ch1=0;
    u32 ch1_all=0;
    int count =1, i;
    int ps_hthr=0, ps_lthr=0, ps_cancelation=0, ps_cal_len = 0;
    char ps_calibration[20];


    if(PS_MAX_XTALK < 0)
    {
        APS_ERR("[%s]:Failed: PS_MAX_XTALK < 0 \r\n", __func__);
        return -EINVAL;
    }

    for(i=0; i<count; i++)
    {
        msleep(50);
    	switch(epl_sensor.mode)
    	{
    		case EPL_MODE_PS:
    		case EPL_MODE_ALS_PS:
    			ch1 = epl_sensor_ps_data();
		    break;
    	}

    	ch1_all = ch1_all + ch1;
    }


    ch1 = (u16)(ch1_all/count);

    if(ch1 > PS_MAX_XTALK)
    {
        APS_ERR("[%s]:Failed: ch1 > max_xtalk(%d) \r\n", __func__, ch1);
        return -EINVAL;
    }
    else if(ch1 < 0)
    {
        APS_ERR("[%s]:Failed: ch1 < 0\r\n", __func__);
        return -EINVAL;
    }

    ps_hthr = ch1 + PS_h_offset;
    ps_lthr = ch1 + PS_l_offset;

    ps_cal_len = sprintf(ps_calibration, "%d,%d,%d", ps_cancelation, ps_hthr, ps_lthr);

    if(write_factory_calibration(epld, ps_calibration, ps_cal_len) < 0)
    {
        APS_ERR("[%s] create file error \n", __func__);
        return -EINVAL;
    }

    epl_sensor.ps.low_threshold = ps_lthr;
	epl_sensor.ps.high_threshold = ps_hthr;
	set_psensor_intr_threshold(epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);

	APS_LOG("[%s]: ch1 = %d\n", __func__, ch1);

	return ch1;
}


static void set_als_ps_intr_type(struct i2c_client *client, bool ps_polling, bool als_polling)
{
    //set als / ps interrupt control mode and interrupt type
	switch((ps_polling << 1) | als_polling)
	{
		case 0: // ps and als interrupt
			epl_sensor.interrupt_control = 	EPL_INT_CTRL_ALS_OR_PS;
			epl_sensor.als.interrupt_type = EPL_INTTY_ACTIVE;
			epl_sensor.ps.interrupt_type = EPL_INTTY_ACTIVE;
		break;

		case 1: //ps interrupt and als polling
			epl_sensor.interrupt_control = 	EPL_INT_CTRL_PS;
			epl_sensor.als.interrupt_type = EPL_INTTY_DISABLE;
			epl_sensor.ps.interrupt_type = EPL_INTTY_ACTIVE;
		break;

		case 2: // ps polling and als interrupt
			epl_sensor.interrupt_control = 	EPL_INT_CTRL_ALS;
			epl_sensor.als.interrupt_type = EPL_INTTY_ACTIVE;
			epl_sensor.ps.interrupt_type = EPL_INTTY_DISABLE;
		break;

		case 3: //ps and als polling
			epl_sensor.interrupt_control = 	EPL_INT_CTRL_ALS_OR_PS;
			epl_sensor.als.interrupt_type = EPL_INTTY_DISABLE;
			epl_sensor.ps.interrupt_type = EPL_INTTY_DISABLE;
		break;
	}
}

static void initial_global_variable(struct i2c_client *client, struct epl_sensor_priv *obj)
{
    //general setting
    epl_sensor.power = EPL_POWER_ON;
    epl_sensor.reset = EPL_RESETN_RUN;
    epl_sensor.mode = EPL_MODE_IDLE;
    epl_sensor.wait = EPL_WAIT_0_MS;
    epl_sensor.osc_sel = EPL_OSC_SEL_1MHZ;

    //als setting
    epl_sensor.als.polling_mode = obj->hw->polling_mode_als;
    ///<2016/06/17 shermanwei shift data for DMS09289269 support 5000~9000Lux
    epl_sensor.als.integration_time = EPL_ALS_INTT_256;///EPL_ALS_INTT_1024; //EPL_ALS_INTT_64;
    epl_sensor.als.gain = EPL_GAIN_LOW;
    epl_sensor.als.adc = EPL_PSALS_ADC_11;
    epl_sensor.als.cycle = EPL_CYCLE_8;
    epl_sensor.als.interrupt_channel_select = EPL_ALS_INT_CHSEL_1;
    epl_sensor.als.persist = EPL_PERIST_1;
    epl_sensor.als.compare_reset = EPL_CMP_RUN;
    epl_sensor.als.lock = EPL_UN_LOCK;
    epl_sensor.als.report_type = CMC_BIT_PRE_COUNT; //CMC_BIT_RAW;
    epl_sensor.als.high_threshold = obj->hw->als_threshold_high;
    epl_sensor.als.low_threshold = obj->hw->als_threshold_low;
    //als factory
    epl_sensor.als.factory.calibration_enable =  false;
    epl_sensor.als.factory.calibrated = false;
    epl_sensor.als.factory.lux_per_count = LUX_PER_COUNT;
    //update als intr table
    if(epl_sensor.als.polling_mode == 0)
        als_intr_update_table(obj);

    //ps setting
    epl_sensor.ps.polling_mode = obj->hw->polling_mode_ps;
    epl_sensor.ps.integration_time = EPL_PS_INTT_272;
    epl_sensor.ps.gain = EPL_GAIN_LOW;
    epl_sensor.ps.adc = EPL_PSALS_ADC_11;
    epl_sensor.ps.cycle = EPL_CYCLE_8;
    epl_sensor.ps.persist = EPL_PERIST_1;
    epl_sensor.ps.ir_on_control = EPL_IR_ON_CTRL_ON;
    epl_sensor.ps.ir_mode = EPL_IR_MODE_CURRENT;
    epl_sensor.ps.ir_drive = EPL_IR_DRIVE_100;
    epl_sensor.ps.compare_reset = EPL_CMP_RUN;
    epl_sensor.ps.lock = EPL_UN_LOCK;
    epl_sensor.ps.high_threshold = obj->hw->ps_threshold_high;
    epl_sensor.ps.low_threshold = obj->hw->ps_threshold_low;
    //ps factory
    epl_sensor.ps.factory.calibration_enable =  true; //false;
    epl_sensor.ps.factory.calibrated = false;
    epl_sensor.ps.factory.cancelation= 0;
    //set als / ps interrupt control mode and trigger type
    set_als_ps_intr_type(client, epl_sensor.ps.polling_mode, epl_sensor.als.polling_mode);
    //write setting to sensor
    write_global_variable(client);
}

static int epl_sensor_get_als_value(struct epl_sensor_priv *obj, u16 als)
{
    int idx;
    int invalid = 0;
    int level=0;
	long lux;

    switch(epl_sensor.als.report_type)
    {
        case CMC_BIT_RAW:
            return als;
        break;

        case CMC_BIT_PRE_COUNT:
            return (als * epl_sensor.als.factory.lux_per_count)/1000;
        break;

        case CMC_BIT_TABLE:
            for(idx = 0; idx < obj->als_level_num; idx++)
            {
                if(als < obj->hw->als_level[idx])
                {
                    break;
                }
            }

            if(idx >= obj->als_value_num)
            {
                APS_ERR("exceed range\n");
                idx = obj->als_value_num - 1;
            }

            if(!invalid)
            {
                APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);
                return obj->hw->als_value[idx];
            }
            else
            {
                APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);
                return als;
            }
        break;
        case CMC_BIT_INTR_LEVEL:
            lux = (als * epl_sensor.als.factory.lux_per_count)/1000;
            if (lux > 0xFFFF)
				lux = 0xFFFF;

		    for (idx = 0; idx < 10; idx++)
		    {
    			if (lux <= (*(als_lux_intr_level + idx))) {
    				level = idx;
    				if (*(als_lux_intr_level + idx))
    					break;
    			}
    			if (idx == 9) {
    				level = idx;
    				break;
    			}
    		}
            obj->als_intr_level = level;
            obj->als_intr_lux = lux;
            return level;

        break;
    }

    return 0;
}


static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
////..    struct i2c_client *client = epld->client;
    uint8_t high_msb ,high_lsb, low_msb, low_lsb;
    u8 buf[4];

    buf[3] = high_msb = (uint8_t) (high_thd >> 8);
    buf[2] = high_lsb = (uint8_t) (high_thd & 0x00ff);
    buf[1] = low_msb  = (uint8_t) (low_thd >> 8);
    buf[0] = low_lsb  = (uint8_t) (low_thd & 0x00ff);
////..    epl_sensor_I2C_Write_Block(client, 0x0c, buf, 4);

///sherman modify_5
    atomic_set(&epld->ps_thd_val_high, high_thd);
    atomic_set(&epld->ps_thd_val_low, low_thd);

    APS_LOG("%s:++++ low_thd = %d, high_thd = %d \n", __FUNCTION__, low_thd, high_thd);
    return 0;
}

static int set_lsensor_intr_threshold(uint16_t low_thd, uint16_t high_thd)
{
////..    struct epl_sensor_priv *epld = epl_sensor_obj;
////..    struct i2c_client *client = epld->client;
    uint8_t high_msb ,high_lsb, low_msb, low_lsb;
    u8 buf[4];

    buf[3] = high_msb = (uint8_t) (high_thd >> 8);
    buf[2] = high_lsb = (uint8_t) (high_thd & 0x00ff);
    buf[1] = low_msb  = (uint8_t) (low_thd >> 8);
    buf[0] = low_lsb  = (uint8_t) (low_thd & 0x00ff);
////..    epl_sensor_I2C_Write_Block(client, 0x08, buf, 4);

    APS_LOG("%s: low_thd = %d, high_thd = %d \n", __FUNCTION__, low_thd, high_thd);
    return 0;
}


/*----------------------------------------------------------------------------*/
static void epl_sensor_dumpReg(struct i2c_client *client)
{
    APS_LOG("chip id REG 0x00 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x00));
    APS_LOG("chip id REG 0x01 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x01));
    APS_LOG("chip id REG 0x02 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x02));
    APS_LOG("chip id REG 0x03 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x03));
    APS_LOG("chip id REG 0x04 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x04));
    APS_LOG("chip id REG 0x05 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x05));
    APS_LOG("chip id REG 0x06 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x06));
    APS_LOG("chip id REG 0x07 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x07));
    APS_LOG("chip id REG 0x11 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x11));
    APS_LOG("chip id REG 0x12 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x12));
    APS_LOG("chip id REG 0x1B value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x1B));
    APS_LOG("chip id REG 0x20 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x20));
    APS_LOG("chip id REG 0x21 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x21));
    APS_LOG("chip id REG 0x24 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x24));
    APS_LOG("chip id REG 0x25 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x25));
    APS_LOG("chip id REG 0xfb value = 0x%x\n", i2c_smbus_read_byte_data(client, 0xfb));
    APS_LOG("chip id REG 0xfc value = 0x%x\n", i2c_smbus_read_byte_data(client, 0xfc));
}

int hw8k_init_device(struct i2c_client *client)
{
    APS_LOG("hw8k_init_device.........\r\n");
    epl_sensor_i2c_client = client;
    APS_LOG(" I2C Addr==[0x%x],line=%d\n", epl_sensor_i2c_client->addr,__LINE__);
    return 0;
}

int epl_sensor_get_addr(struct alsps_hw *hw, struct epl_sensor_i2c_addr *addr)
{
    if(!hw || !addr)
    {
        return -EFAULT;
    }
    addr->write_addr= hw->i2c_addr[0];
    return 0;
}

static void epl_sensor_power(struct alsps_hw *hw, unsigned int on)
{

////..    static unsigned int power_on = 0;

    //APS_LOG("power %s\n", on ? "on" : "off");
APS_LOG("[%s]: power_on=%d \r\n", __func__, on);
////..    if(hw->power_id != POWER_NONE_MACRO)
////..    {
/*
        if(power_on == on)
        {
            APS_LOG("ignore power control: %d\n", on);
        }
        else if(on)
        {
            if(!hwPowerOn(hw->power_id, hw->power_vol, EPL_DEV_NAME))
            {
                APS_ERR("power on fails!!\n");
            }
        }
        else
        {
            if(!hwPowerDown(hw->power_id, EPL_DEV_NAME))
            {
                APS_ERR("power off fail!!\n");
            }
        }
////..    }
    power_on = on;
*/
}

/*
static void epl_sensor_report_ps_status(void)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    int err;
#if !MTK_LTE
    ////..hwm_sensor_data sensor_data;
    struct hwm_sensor_data sensor_data;
#endif
    APS_FUN();
    APS_LOG("[%s]: epl_sensor.ps.data.data=%d, ps status=%d \r\n", __func__, epl_sensor.ps.data.data, (epl_sensor.ps.compare_low >> 3));

#if MTK_LTE
    err = ps_report_interrupt_data((epl_sensor.ps.compare_low >> 3));
    if(err != 0)  //if report status is fail, write unlock again.
    {
        APS_ERR("epl_sensor_eint_work err: %d\n", err);
        epl_sensor.ps.compare_reset = EPL_CMP_RESET;
    	epl_sensor.ps.lock = EPL_UN_LOCK;
    	epl_sensor_I2C_Write(epld->client,0x1b, epl_sensor.ps.compare_reset | epl_sensor.ps.lock);
    }
#else
    sensor_data.values[0] = epl_sensor.ps.compare_low >> 3;
	sensor_data.values[1] = epl_sensor.ps.data.data;
	sensor_data.value_divide = 1;
	sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
	if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data))) //if report status is fail, write unlock again.
	{
	    APS_ERR("get interrupt data failed\n");
	    APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
	    epl_sensor.ps.compare_reset = EPL_CMP_RESET;
    	epl_sensor.ps.lock = EPL_UN_LOCK;
    	epl_sensor_I2C_Write(epld->client,0x1b, epl_sensor.ps.compare_reset | epl_sensor.ps.lock);
	}
#endif
}
*/

/*
static void epl_sensor_intr_als_report_lux(void)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    u16 als;
    int err = 0;
#if !MTK_LTE
    ////..hwm_sensor_data sensor_data;
    struct hwm_sensor_data sensor_data;
#endif

    epl_sensor.als.compare_reset = EPL_CMP_RESET;
	epl_sensor.als.lock = EPL_UN_LOCK;
	epl_sensor_I2C_Write(obj->client,0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);

    APS_LOG("[%s]: IDEL MODE \r\n", __func__);
    //epl_sensor_I2C_Write(obj->client, 0x00, epl_sensor.wait | EPL_MODE_IDLE);
	epl_sensor_I2C_Write(obj->client, 0x11, EPL_POWER_OFF | EPL_RESETN_RESET);  //After runing CMP_RESET, dont clean interrupt_flag

    als = epl_sensor_get_als_value(obj, epl_sensor.als.data.channels[1]);

    {
        APS_LOG("[%s]: report als = %d \r\n", __func__, als);
#if MTK_LTE
        if((err = als_data_report(alsps_context_obj->idev, als, SENSOR_STATUS_ACCURACY_MEDIUM)))
        {
            APS_ERR("epl_sensor call als_report_interrupt_data fail = %d\n", err);
        }
#else
        sensor_data.values[0] = als;
    	sensor_data.values[1] = epl_sensor.als.data.channels[1];
    	sensor_data.value_divide = 1;
    	sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
    	if((err = hwmsen_get_interrupt_data(ID_LIGHT, &sensor_data)))
    	{
    	    APS_ERR("get interrupt data failed\n");
    	    APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
    	}
#endif
    }

    if(epl_sensor.als.report_type == CMC_BIT_INTR_LEVEL)
    {
        {
            epl_sensor.als.low_threshold = *(als_adc_intr_level + (als-1)) + 1;
            epl_sensor.als.high_threshold = *(als_adc_intr_level + als);
            if( (als == 0) || (epl_sensor.als.data.channels[1] == 0) )
            {
                epl_sensor.als.low_threshold = 0;
            }
        }
    }
    else
    {
        //set dynamic threshold
    	if(epl_sensor.als.compare_high >> 4)
    	{
    		epl_sensor.als.high_threshold = epl_sensor.als.high_threshold + 250;
    		epl_sensor.als.low_threshold = epl_sensor.als.low_threshold + 250;
    		if (epl_sensor.als.high_threshold > 60000)
    		{
    			epl_sensor.als.high_threshold = epl_sensor.als.high_threshold - 250;
    			epl_sensor.als.low_threshold = epl_sensor.als.low_threshold - 250;
    		}
    	}
    	if(epl_sensor.als.compare_low>> 3)
    	{
    		epl_sensor.als.high_threshold = epl_sensor.als.high_threshold - 250;
    		epl_sensor.als.low_threshold = epl_sensor.als.low_threshold - 250;
    		if (epl_sensor.als.high_threshold < 250)
    		{
    			epl_sensor.als.high_threshold = epl_sensor.als.high_threshold + 250;
    			epl_sensor.als.low_threshold = epl_sensor.als.low_threshold + 250;
    		}
    	}

        if(epl_sensor.als.high_threshold < epl_sensor.als.low_threshold)
    	{
    	    APS_LOG("[%s]:recover default setting \r\n", __FUNCTION__);
    	    epl_sensor.als.high_threshold = obj->hw->als_threshold_high;
    	    epl_sensor.als.low_threshold = obj->hw->als_threshold_low;
    	}
    }

	//write new threshold
	set_lsensor_intr_threshold(epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);

    epl_sensor_I2C_Write(obj->client, 0x11, EPL_POWER_ON | EPL_RESETN_RUN);
    //epl_sensor_I2C_Write(obj->client, 0x00, epl_sensor.wait | epl_sensor.mode);
    APS_LOG("[%s]: MODE=0x%x \r\n", __func__, epl_sensor.mode);
}
*/
/*----------------------------------------------------------------------------*/

int epl_sensor_read_als(struct i2c_client *client)
{
    struct epl_sensor_priv *obj = i2c_get_clientdata(client);
    u8 buf[5];
    if(client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }

    mutex_lock(&sensor_mutex);
    epl_sensor_I2C_Read(obj->client, 0x12, 5);
    buf[0] = gRawData.raw_bytes[0];
    buf[1] = gRawData.raw_bytes[1];
    buf[2] = gRawData.raw_bytes[2];
    buf[3] = gRawData.raw_bytes[3];
    buf[4] = gRawData.raw_bytes[4];
    mutex_unlock(&sensor_mutex);

    epl_sensor.als.saturation = (buf[0] & 0x20);
    epl_sensor.als.compare_high = (buf[0] & 0x10);
    epl_sensor.als.compare_low = (buf[0] & 0x08);
    epl_sensor.als.interrupt_flag = (buf[0] & 0x04);
    epl_sensor.als.compare_reset = (buf[0] & 0x02);
    epl_sensor.als.lock = (buf[0] & 0x01);
    epl_sensor.als.data.channels[0] = (buf[2]<<8) | buf[1];
    epl_sensor.als.data.channels[1] = (buf[4]<<8) | buf[3];

	APS_LOG("als: ~~~~ ALS ~~~~~ \n");
	APS_LOG("als: buf = 0x%x\n", buf[0]);
	APS_LOG("als: sat = 0x%x\n", epl_sensor.als.saturation);
	APS_LOG("als: cmp h = 0x%x, l = %d\n", epl_sensor.als.compare_high, epl_sensor.als.compare_low);
	APS_LOG("als: int_flag = 0x%x\n",epl_sensor.als.interrupt_flag);
	APS_LOG("als: cmp_rstn = 0x%x, lock = 0x%0x\n", epl_sensor.als.compare_reset, epl_sensor.als.lock);
    APS_LOG("[%s]: read als channel 0 = %d\n", __func__, epl_sensor.als.data.channels[0]);
    APS_LOG("[%s]: read als channel 1 = %d\n", __func__, epl_sensor.als.data.channels[1]);
    return 0;
}

/*----------------------------------------------------------------------------*/
int epl_sensor_read_ps(struct i2c_client *client)
{
    u8 buf[5];
///sherman modify_5
struct epl_sensor_priv *obj = i2c_get_clientdata(client);
    if(client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }

    mutex_lock(&sensor_mutex);
    epl_sensor_I2C_Read(client,0x1b, 5);
    buf[0] = gRawData.raw_bytes[0];
    buf[1] = gRawData.raw_bytes[1];
    buf[2] = gRawData.raw_bytes[2];
    buf[3] = gRawData.raw_bytes[3];
    buf[4] = gRawData.raw_bytes[4];
    mutex_unlock(&sensor_mutex);

    epl_sensor.ps.saturation = (buf[0] & 0x20);
    epl_sensor.ps.compare_high = (buf[0] & 0x10);
    epl_sensor.ps.compare_low = (buf[0] & 0x08);
    epl_sensor.ps.interrupt_flag = (buf[0] & 0x04);
    epl_sensor.ps.compare_reset = (buf[0] & 0x02);
    epl_sensor.ps.lock= (buf[0] & 0x01);
    epl_sensor.ps.data.ir_data = (buf[2]<<8) | buf[1];
    epl_sensor.ps.data.data = (buf[4]<<8) | buf[3];
///sherman modify_5
if ( epl_sensor.ps.data.data > atomic_read(&obj->ps_thd_val_high) )
obj->cover_state = true;
if( (epl_sensor.ps.data.data < atomic_read(&obj->ps_thd_val_low)) && (obj->cover_state == true))
obj->cover_state = false;

gRawData.ps_state = (obj->cover_state == true) ? 0 : 1;
	
	APS_LOG("ps: ~~~~ PS ~~~~~ gRawData.ps_state=%d\n", gRawData.ps_state);
	APS_LOG("ps: buf = 0x%x\n", buf[0]);
	APS_LOG("ps: sat = 0x%x\n", epl_sensor.ps.saturation);
	APS_LOG("ps: cmp h = 0x%x, l = 0x%x\n", epl_sensor.ps.compare_high, epl_sensor.ps.compare_low);
	APS_LOG("ps: int_flag = 0x%x\n",epl_sensor.ps.interrupt_flag);
	APS_LOG("ps: cmp_rstn = 0x%x, lock = %x\n", epl_sensor.ps.compare_reset, epl_sensor.ps.lock);
	APS_LOG("[%s]: data = %d\n", __func__, epl_sensor.ps.data.data);
	APS_LOG("[%s]: ir data = %d\n", __func__, epl_sensor.ps.data.ir_data);
    return 0;
}

int epl_sensor_ps_data(void)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    bool enable_ps = test_bit(CMC_BIT_PS, &epld->enable) && atomic_read(&epld->ps_suspend)==0;
///sherman modify_1
	bool enable_als = test_bit(CMC_BIT_ALS, &epld->enable) && atomic_read(&epld->als_suspend)==0;
APS_LOG("[%s]: enable_ps=%d ;enable_als=%d\r\n", __func__, enable_ps, enable_als);
    if(enable_ps == 0)
    {
        set_bit(CMC_BIT_PS, &epld->enable);
        epl_sensor_fast_update(epld->client);
        epl_sensor_update_mode(epld->client);
		///sherman modify_2
        msleep(ps_frame_time);
        enable_ps = true;
    }

    ///if(enable_ps == true && epl_sensor.ps.polling_mode == 0)
    ///sherman modify_1
    if(enable_ps == true)
        epl_sensor_read_ps(epld->client);

    return epl_sensor.ps.data.data;
}

int epl_sensor_als_data(void)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    bool enable_als = test_bit(CMC_BIT_ALS, &epld->enable) && atomic_read(&epld->als_suspend)==0;
///sherman modify_2
bool enable_ps = test_bit(CMC_BIT_PS, &epld->enable) && atomic_read(&epld->ps_suspend)==0;
APS_LOG("[%s]: enable_ps=%d ;enable_als=%d \r\n", __func__, enable_ps, enable_als);
    if(enable_als == 0)
    {
        set_bit(CMC_BIT_ALS, &epld->enable);
        epl_sensor_fast_update(epld->client);
        epl_sensor_update_mode(epld->client);
        ///sherman modify_2
        msleep(als_frame_time);
        enable_als = true;
    }

    ///if(enable_als == true && epl_sensor.ps.polling_mode == 0)
    ///sherman modify_1
	if(enable_als == true)
        epl_sensor_read_als(epld->client);

    return epl_sensor.als.data.channels[1];
}

//for 3637
static int als_sensing_time(int intt, int adc, int cycle)
{
    long sensing_us_time;
    int sensing_ms_time;
    int als_intt, als_adc, als_cycle;

    als_intt = als_intt_value[intt>>2];
    als_adc = adc_value[adc>>3];
    als_cycle = cycle_value[cycle];
    APS_LOG("ALS: INTT=%d, ADC=%d, Cycle=%d \r\n", als_intt, als_adc, als_cycle);

    sensing_us_time = (als_intt + als_adc*2*2) * 2 * als_cycle;
    sensing_ms_time = sensing_us_time / 1000;
    APS_LOG("[%s]: sensing=%d ms \r\n", __func__, sensing_ms_time);
    return (sensing_ms_time + 5);
}

static int ps_sensing_time(int intt, int adc, int cycle)
{
    long sensing_us_time;
    int sensing_ms_time;
    int ps_intt, ps_adc, ps_cycle;

    ps_intt = ps_intt_value[intt>>2];
    ps_adc = adc_value[adc>>3];
    ps_cycle = cycle_value[cycle];
    APS_LOG("PS: INTT=%d, ADC=%d, Cycle=%d \r\n", ps_intt, ps_adc, ps_cycle);

    sensing_us_time = (ps_intt*3 + ps_adc*2*3) * ps_cycle;
    sensing_ms_time = sensing_us_time / 1000;
    APS_LOG("[%s]: sensing=%d ms\r\n", __func__, sensing_ms_time);
    return (sensing_ms_time + 5);
}

void epl_sensor_fast_update(struct i2c_client *client)
{
    int als_fast_time = 0;

    APS_FUN();
    mutex_lock(&sensor_mutex);
    set_als_ps_intr_type(client, 1, 1);
    als_fast_time = als_sensing_time(epl_sensor.als.integration_time, epl_sensor.als.adc, EPL_CYCLE_1);

    epl_sensor_I2C_Write(client, 0x11, EPL_POWER_OFF | EPL_RESETN_RESET);

    epl_sensor_I2C_Write(client, 0x06, epl_sensor.interrupt_control | epl_sensor.ps.persist |epl_sensor.ps.interrupt_type);
    epl_sensor_I2C_Write(client, 0x07, epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);

    epl_sensor_I2C_Write(client, 0x02, epl_sensor.als.adc | EPL_CYCLE_1);
    epl_sensor_I2C_Write(client, 0x01, epl_sensor.als.integration_time | epl_sensor.als.gain);

    epl_sensor_I2C_Write(client, 0x00, epl_sensor.wait | EPL_MODE_ALS);
    epl_sensor_I2C_Write(client, 0x11, EPL_POWER_ON | EPL_RESETN_RUN);
    mutex_unlock(&sensor_mutex);

    msleep(als_fast_time);
    APS_LOG("[%s]: msleep(%d)\r\n", __func__, als_fast_time);

    mutex_lock(&sensor_mutex);
    if(epl_sensor.als.polling_mode == 0)
    {
        //fast_mode is already ran one frame, so must to reset CMP bit for als intr mode
        //ALS reset and run
        epl_sensor_I2C_Write(client, 0x12, EPL_CMP_RESET | EPL_UN_LOCK);
    	epl_sensor_I2C_Write(client, 0x12, EPL_CMP_RUN | EPL_UN_LOCK);
    }

    epl_sensor_I2C_Write(client, 0x11, EPL_POWER_OFF | EPL_RESETN_RESET);

    epl_sensor_I2C_Write(client, 0x02, epl_sensor.als.adc | epl_sensor.als.cycle);

    set_als_ps_intr_type(client, epl_sensor.ps.polling_mode, epl_sensor.als.polling_mode);
    epl_sensor_I2C_Write(client, 0x06, epl_sensor.interrupt_control | epl_sensor.ps.persist |epl_sensor.ps.interrupt_type);
    epl_sensor_I2C_Write(client, 0x07, epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);
    //epl_sensor_I2C_Write(client, 0x11, EPL_POWER_ON | EPL_RESETN_RUN);
    mutex_unlock(&sensor_mutex);
}

void epl_sensor_update_mode(struct i2c_client *client)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    int als_time = 0, ps_time = 0;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;

    als_frame_time = als_time = als_sensing_time(epl_sensor.als.integration_time, epl_sensor.als.adc, epl_sensor.als.cycle);
    ps_frame_time = ps_time = ps_sensing_time(epl_sensor.ps.integration_time, epl_sensor.ps.adc, epl_sensor.ps.cycle);

////    mutex_lock(&sensor_mutex);
	APS_LOG("%s: mode selection =0x%x,ps_frame_time=%d,als_frame_time=%d\n", __func__, enable_ps | (enable_als << 1),ps_frame_time,als_frame_time);
    //**** mode selection ****
    switch((enable_als << 1) | enable_ps)
    {
        case 0: //disable all
            epl_sensor.mode = EPL_MODE_IDLE;
            break;
        case 1: //als = 0, ps = 1
            epl_sensor.mode = EPL_MODE_PS;
            break;
        case 2: //als = 1, ps = 0
            epl_sensor.mode = EPL_MODE_ALS;
            break;
        case 3: //als = 1, ps = 1
            epl_sensor.mode = EPL_MODE_ALS_PS;
            break;
    }

    //**** write setting ****
    // step 1. set sensor at idle mode
    // step 2. uplock and reset als / ps status
    // step 3. set sensor at operation mode
    // step 4. delay sensing time
    // step 5. unlock and run als / ps status
////    epl_sensor_I2C_Write(client, 0x11, EPL_POWER_OFF | EPL_RESETN_RESET);

    // initial factory calibration variable
#if !ARIMA_PATCH
    read_factory_calibration();   //arima dont use
#endif
	mutex_lock(&sensor_mutex);
    epl_sensor_I2C_Write(client, 0x11, EPL_POWER_OFF | EPL_RESETN_RESET);
    epl_sensor_I2C_Write(obj->client, 0x00, epl_sensor.wait | epl_sensor.mode);

    if(epl_sensor.mode != EPL_MODE_IDLE)    // if mode isnt IDLE, PWR_ON and RUN    //check this
        epl_sensor_I2C_Write(client, 0x11, EPL_POWER_ON | EPL_RESETN_RUN);
	
	mutex_unlock(&sensor_mutex);
    //**** check setting ****
    if(enable_ps == 1)
    {
///        APS_LOG("[%s] PS:low_thd = %d, high_thd = %d \n",__func__, epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);
		APS_LOG("[%s] PS:low_thd = %d, high_thd = %d \n",__func__, atomic_read(&obj->ps_thd_val_low), atomic_read(&obj->ps_thd_val_high));
    }
    if(enable_als == 1 && epl_sensor.als.polling_mode == 0)
    {
        APS_LOG("[%s] ALS:low_thd = %d, high_thd = %d \n",__func__, epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);
    }

	APS_LOG("[%s] reg0x00= 0x%x\n", __func__,  epl_sensor.wait | epl_sensor.mode);
	APS_LOG("[%s] reg0x07= 0x%x\n", __func__, epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);
	APS_LOG("[%s] reg0x06= 0x%x\n", __func__, epl_sensor.interrupt_control | epl_sensor.ps.persist |epl_sensor.ps.interrupt_type);
	APS_LOG("[%s] reg0x11= 0x%x\n", __func__, epl_sensor.power | epl_sensor.reset);
	APS_LOG("[%s] reg0x12= 0x%x\n", __func__, epl_sensor.als.compare_reset | epl_sensor.als.lock);
	APS_LOG("[%s] reg0x1b= 0x%x\n", __func__, epl_sensor.ps.compare_reset | epl_sensor.ps.lock);

////    mutex_unlock(&sensor_mutex);
////sherman modify_3
#if ARIMA_DYNAMIC_CAL///ARIMA_PATCH
    if(enable_ps == 1)
    {
        msleep(ps_frame_time);
        APS_LOG("[%s]: PS msleep(%d)\r\n", __func__, ps_frame_time);
        epl_sensor_read_ps(client);

//<2015/04/16-ShermanWei, for Dynamic Cal
/////
////Dynamic Calibration
	    gRawData.ps_sta = epl_sensor.ps.saturation;
	    gRawData.ps_raw = epl_sensor.ps.data.data;
	    gRawData.ps_condition = epl_sensor.ps.data.ir_data;

	    if( (gRawData.mmisw) && (gRawData.ps_raw < 16000/*2000*/) && (gRawData.ps_raw > 0) && (gRawData.ps_condition < 45000) && (gRawData.ps_sta != 1) )
	    {
		    APS_LOG("[ELAN]+ DynamicK CT:%d;Delta:%d\n", gRawData.ps_raw, gRawData.cal_ps_delta);
			set_psensor_intr_threshold(gRawData.ps_raw + (gRawData.cal_ps_delta/10)*7, gRawData.ps_raw + gRawData.cal_ps_delta);
/*
    		if (gRawData.ps_raw < 2000)
    		    set_psensor_intr_threshold(gRawData.ps_raw + (gRawData.cal_ps_delta/4)*3, gRawData.ps_raw + gRawData.cal_ps_delta);
    		else if (gRawData.ps_raw < 6000)
    		    set_psensor_intr_threshold(gRawData.ps_raw + 1000 + 200, gRawData.ps_raw + 1000 + 500);
    		else
    		    set_psensor_intr_threshold(6000 + 1000, 6000 + 1000 + 500);
*/
    		gRawData.mmisw = 0;
	    }
    	else if (gRawData.mmisw)
    	{
    		gRawData.mmisw = 0;
    		switch (gRawData.ps_calfile_status)
    		{
        		case PS_CAL_DELTA_NORMAL:///Delta is invidual factory cal
    			    APS_LOG("[ELAN]+ FactoryK CT:%d;Delta:%d\n", gRawData.ps_als_factory.ps_cal_l, gRawData.cal_ps_delta);
    			    set_psensor_intr_threshold(gRawData.ps_als_factory.ps_cal_l + (gRawData.cal_ps_delta/10)*7,
    										gRawData.ps_als_factory.ps_cal_l + gRawData.cal_ps_delta);
///    			    set_psensor_intr_threshold(5000 + 500, 5000 + 1000);
            	break;

        		case PS_CAL_DELTA_ABNORMAL:///Delta is factory average
    			    APS_LOG("[ELAN]+ AverageK CT:%d;Delta:%d\n", gRawData.ps_als_factory.ps_cal_l, gRawData.cal_ps_delta);
    			    set_psensor_intr_threshold(gRawData.ps_als_factory.ps_cal_l + (gRawData.cal_ps_delta/10)*7,
    										gRawData.ps_als_factory.ps_cal_l + gRawData.cal_ps_delta);
///    			    set_psensor_intr_threshold(5000 + 500, 5000 + 1000);
           	 	break;

        		case PS_CAL_FILE_NOEXIST:///threshold is default value
        		default:

    			    APS_LOG("[ELAN]+ DefaultK Threshold");
    			    set_psensor_intr_threshold(obj->hw->ps_threshold_low,obj->hw->ps_threshold_high);
///    			    set_psensor_intr_threshold(5000 + 500, 5000 + 1000);
            	break;
    		}
    	}
//>2015/04/16-ShermanWei
    }
#endif
}

#if PS_BOOT_K
void epl_sensor_ps_boot_k(struct i2c_client *client)
{
    int ps_time=0;

    ps_time = ps_sensing_time(EPL_PS_INTT_80, epl_sensor.ps.adc, epl_sensor.ps.cycle);

    epl_sensor_I2C_Write(client, 0x11, EPL_POWER_OFF | EPL_RESETN_RESET);
    //epl_sensor_I2C_Write(client, 0x04, epl_sensor.ps.adc | EPL_CYCLE_8);    // using 8 cycle
    epl_sensor_I2C_Write(client, 0x03, EPL_PS_INTT_80 | epl_sensor.ps.gain); //using INTEG=80us
    epl_sensor_I2C_Write(client, 0x05, EPL_IR_ON_CTRL_OFF | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_drive); //disable LED
    epl_sensor_I2C_Write(client, 0x06, epl_sensor.interrupt_control | epl_sensor.ps.persist | EPL_INTTY_DISABLE); //disable interrupt
    epl_sensor_I2C_Write(client, 0x00, epl_sensor.wait | EPL_MODE_PS);
    epl_sensor_I2C_Write(client, 0x11, EPL_POWER_ON | EPL_RESETN_RUN);
    msleep(ps_time);
    APS_LOG("[%s]: msleep(%dms) \r\n", __func__, ps_time);

    epl_sensor_read_ps(client);

    epl_sensor_I2C_Write(client, 0x11, EPL_POWER_OFF | EPL_RESETN_RESET);
    epl_sensor_I2C_Write(client, 0x00, epl_sensor.wait | EPL_MODE_IDLE);
    if(epl_sensor.ps.data.data < PS_BOOT_MAX_CT)
    {
        epl_sensor.ps.cancelation = epl_sensor.ps.data.data;
        epl_sensor_I2C_Write(client, 0x22, (u8)(epl_sensor.ps.cancelation& 0xff));
        epl_sensor_I2C_Write(client, 0x23, (u8)((epl_sensor.ps.cancelation & 0xff00) >> 8));
        APS_LOG("[%s]: setting epl_sensor.ps.cancelation = %d \r\n", __func__, epl_sensor.ps.cancelation);
    }
    /*recover setting*/
    //epl_sensor_I2C_Write(client, 0x04, epl_sensor.ps.adc | epl_sensor.ps.cycle);
    epl_sensor_I2C_Write(client, 0x03, epl_sensor.ps.integration_time | epl_sensor.ps.gain);
    epl_sensor_I2C_Write(client, 0x05, epl_sensor.ps.ir_on_control | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_drive);
    epl_sensor_I2C_Write(client, 0x06, epl_sensor.interrupt_control | epl_sensor.ps.persist | epl_sensor.ps.interrupt_type);
}
#endif /*PS_BOOT_K*/

/*----------------------------------------------------------------------------*/
/*
void epl_sensor_eint_func(void)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    // APS_LOG(" interrupt fuc\n");
    if(!obj)
    {
        return;
    }
#if defined(MT6575) || defined(MT6571) || defined(MT6589)
    mt65xx_eint_mask(CUST_EINT_ALS_NUM);
#else
    mt_eint_mask(CUST_EINT_ALS_NUM);
#endif
    schedule_delayed_work(&obj->eint_work, 0);
}
*/
/*----------------------------------------------------------------------------*/
/*
static void epl_sensor_eint_work(struct work_struct *work)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;

    APS_LOG("xxxxxxxxxxx\n\n");

    epl_sensor_read_ps(obj->client);
    epl_sensor_read_als(obj->client);
    if(epl_sensor.ps.interrupt_flag == EPL_INT_TRIGGER)
    {
        if(enable_ps)
        {
            wake_lock_timeout(&ps_lock, 2*HZ);
            epl_sensor_report_ps_status();
        }
        //PS unlock interrupt pin and restart chip
		epl_sensor.ps.compare_reset = EPL_CMP_RUN;
		epl_sensor.ps.lock = EPL_UN_LOCK;
		epl_sensor_I2C_Write(obj->client,0x1b, epl_sensor.ps.compare_reset | epl_sensor.ps.lock);
    }

    if(epl_sensor.als.interrupt_flag == EPL_INT_TRIGGER)
    {
        epl_sensor_intr_als_report_lux();
        //ALS unlock interrupt pin and restart chip
    	epl_sensor.als.compare_reset = EPL_CMP_RUN;
    	epl_sensor.als.lock = EPL_UN_LOCK;
    	epl_sensor_I2C_Write(obj->client,0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);
    }

#if defined(MT6575) || defined(MT6571) || defined(MT6589)
    mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#else
    mt_eint_unmask(CUST_EINT_ALS_NUM);
#endif
}
*/

/*
int epl_sensor_setup_eint(struct i2c_client *client)
{
    APS_LOG("epl_sensor_setup_eint\n");

    ///configure to GPIO function, external interrupt
    mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

#if defined(MT6575) || defined(MT6571) || defined(MT6589)
    mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_LEVEL_SENSITIVE );//CUST_EINT_EDGE_SENSITIVE);
    mt65xx_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY);
    mt65xx_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
    mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_POLARITY_LOW, epl_sensor_eint_func, 0);
    mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#else
    mt_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_TYPE, epl_sensor_eint_func, 0);

	mt_eint_unmask(CUST_EINT_ALS_NUM);
#endif
    return 0;
}

*/
static int epl_sensor_init_client(struct i2c_client *client)
{
    ////..struct epl_sensor_priv *obj = i2c_get_clientdata(client);
    int err=0;
    /*  interrupt mode */
    APS_FUN();
    APS_LOG("I2C Addr==[0x%x],line=%d\n", epl_sensor_i2c_client->addr, __LINE__);

/*
    if(obj->hw->polling_mode_ps == 0)
    {

#if defined(MT6575) || defined(MT6571) || defined(MT6589)
        mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#else
        mt_eint_unmask(CUST_EINT_ALS_NUM);
#endif

        if((err = epl_sensor_setup_eint(client)))
        {
            APS_ERR("setup eint: %d\n", err);
            return err;
        }
        APS_LOG("epl_sensor interrupt setup\n");
    }
*/
///...    if((err = hw8k_init_device(client)) != 0)
///...    {
///...        APS_ERR("init dev: %d\n", err);
///...        return err;
///...    }

    /*  interrupt mode */
    //if(obj->hw->polling_mode_ps == 0)
    //     mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
    return err;
}


/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_show_reg(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    struct i2c_client *client = epl_sensor_obj->client;

    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x00 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x00));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x01 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x01));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x02 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x02));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x03 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x03));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x04 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x04));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x05 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x05));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x06 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x06));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x07 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x07));
    if(epl_sensor.als.polling_mode == 0)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x08 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x08));
        len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x09 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x09));
        len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0A value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0A));
        len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0B value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0B));
    }
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0C value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0C));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0D value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0D));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0E value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0E));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0F value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0F));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x11 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x11));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x12 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x12));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x13 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x13));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x14 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x14));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x15 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x15));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x16 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x16));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x1B value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x1B));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x1C value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x1C));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x1D value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x1D));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x1E value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x1E));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x1F value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x1F));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x22 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x22));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x23 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x23));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x24 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x24));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x25 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x25));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0xFC value = 0x%x\n", i2c_smbus_read_byte_data(client, 0xFC));
    return len;
}

static ssize_t epl_sensor_show_status(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    struct epl_sensor_priv *epld = epl_sensor_obj;
    bool enable_ps = test_bit(CMC_BIT_PS, &epld->enable) && atomic_read(&epld->ps_suspend)==0;
    bool enable_als = test_bit(CMC_BIT_ALS, &epld->enable) && atomic_read(&epld->als_suspend)==0;
    if(!epl_sensor_obj)
    {
        APS_ERR("epl_sensor_obj is null!!\n");
        return 0;
    }

    len += snprintf(buf+len, PAGE_SIZE-len, "chip is %s, ver is %s \n", EPL_DEV_NAME, DRIVER_VERSION);
    len += snprintf(buf+len, PAGE_SIZE-len, "als/ps polling is %d-%d\n", epl_sensor.als.polling_mode, epl_sensor.ps.polling_mode);
    len += snprintf(buf+len, PAGE_SIZE-len, "wait = %d, mode = %d\n",epl_sensor.wait >> 4, epl_sensor.mode);
    len += snprintf(buf+len, PAGE_SIZE-len, "interrupt control = %d\n", epl_sensor.interrupt_control >> 4);
    len += snprintf(buf+len, PAGE_SIZE-len, "frame time ps=%dms, als=%dms\n", ps_frame_time, als_frame_time);
    if(enable_ps)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "PS: \n");
        len += snprintf(buf+len, PAGE_SIZE-len, "INTEG = %d, gain = %d\n", epl_sensor.ps.integration_time >> 2, epl_sensor.ps.gain);
        len += snprintf(buf+len, PAGE_SIZE-len, "ADC = %d, cycle = %d, ir drive = %d\n", epl_sensor.ps.adc >> 3, epl_sensor.ps.cycle, epl_sensor.ps.ir_drive);
        len += snprintf(buf+len, PAGE_SIZE-len, "saturation = %d, int flag = %d\n", epl_sensor.ps.saturation >> 5, epl_sensor.ps.interrupt_flag >> 2);
        len += snprintf(buf+len, PAGE_SIZE-len, "Thr(L/H) = (%d/%d)\n", epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);
        len += snprintf(buf+len, PAGE_SIZE-len, "pals data = %d, data = %d\n", epl_sensor.ps.data.ir_data, epl_sensor.ps.data.data);
    }
    if(enable_als)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "ALS: \n");
        len += snprintf(buf+len, PAGE_SIZE-len, "INTEG = %d, gain = %d\n", epl_sensor.als.integration_time >> 2, epl_sensor.als.gain);
        len += snprintf(buf+len, PAGE_SIZE-len, "ADC = %d, cycle = %d\n", epl_sensor.als.adc >> 3, epl_sensor.als.cycle);
        if(epl_sensor.als.polling_mode == 0)
            len += snprintf(buf+len, PAGE_SIZE-len, "Thr(L/H) = (%d/%d)\n", epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);
        len += snprintf(buf+len, PAGE_SIZE-len, "ch0 = %d, ch1 = %d\n", epl_sensor.als.data.channels[0], epl_sensor.als.data.channels[1]);
    }
    return len;
}

static ssize_t epl_sensor_store_als_enable(struct device_driver *ddri, const char *buf, size_t count)
{
    uint16_t mode=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;
    APS_FUN();

    sscanf(buf, "%hu", &mode);
    if(enable_als != mode)
    {
        if(mode)
        {
            set_bit(CMC_BIT_ALS, &obj->enable);
            epl_sensor_fast_update(obj->client);
        }
        else
        {
            clear_bit(CMC_BIT_ALS, &obj->enable);
        }
        epl_sensor_update_mode(obj->client);
    }
    return count;
}

static ssize_t epl_sensor_store_ps_enable(struct device_driver *ddri, const char *buf, size_t count)
{
    uint16_t mode=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    APS_FUN();

    sscanf(buf, "%hu", &mode);
    if(enable_ps != mode)
    {
        if(mode)
        {
            //wake_lock(&ps_lock);
            set_bit(CMC_BIT_PS, &obj->enable);
        }
        else
        {
            clear_bit(CMC_BIT_PS, &obj->enable);
            //wake_unlock(&ps_lock);
        }
        epl_sensor_fast_update(obj->client); //if fast to on/off ALS, it read als for DOC_OFF setting for als_operate
        epl_sensor_update_mode(obj->client);
    }
    return count;
}

static ssize_t epl_sensor_show_cal_raw(struct device_driver *ddri, char *buf)
{
    u16 ch1=0;
    u32 ch1_all=0;
    int count=1, i=0;
    ssize_t len = 0;
    if(!epl_sensor_obj)
    {
        APS_ERR("epl_sensor_obj is null!!\n");
        return 0;
    }

    for(i=0; i<count; i++)
    {
        msleep(50);
        switch(epl_sensor.mode)
        {

            case EPL_MODE_PS:
                ch1 = epl_sensor_ps_data();
                break;

            case EPL_MODE_ALS:
                ch1 = epl_sensor_als_data();
                break;
        }

        ch1_all = ch1_all + ch1;
    }

    ch1 = (u16)ch1_all/count;
    APS_LOG("cal_raw = %d \r\n" , ch1);
    len += snprintf(buf + len, PAGE_SIZE - len, "%d \r\n", ch1);
    return  len;
}


/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_threshold(struct device_driver *ddri,const char *buf, size_t count)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    int low, high;
    APS_FUN();
    if(!obj)
    {
        APS_ERR("epl_sensor_obj is null!!\n");
        return 0;
    }
    sscanf(buf, "%d,%d", &low, &high);

    switch(epl_sensor.mode)
    {
        case EPL_MODE_PS:
            obj->hw->ps_threshold_low = low;
            obj->hw->ps_threshold_high = high;
            epl_sensor.ps.low_threshold = low;
            epl_sensor.ps.high_threshold = high;
            set_psensor_intr_threshold(epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);
            break;

        case EPL_MODE_ALS:
            obj->hw->als_threshold_low = low;
            obj->hw->als_threshold_high = high;
            epl_sensor.als.low_threshold = low;
            epl_sensor.als.high_threshold = high;
            set_lsensor_intr_threshold(epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);
            break;

    }
    return  count;
}

static ssize_t epl_sensor_show_threshold(struct device_driver *ddri, char *buf)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    ssize_t len = 0;
    if(!obj)
    {
        APS_ERR("epl_sensor_obj is null!!\n");
        return 0;
    }

    switch(epl_sensor.mode)
    {
        case EPL_MODE_PS:
            len += snprintf(buf + len, PAGE_SIZE - len, "obj->hw->ps_threshold_low=%d \r\n", obj->hw->ps_threshold_low);
            len += snprintf(buf + len, PAGE_SIZE - len, "obj->hw->ps_threshold_high=%d \r\n", obj->hw->ps_threshold_high);
            break;

        case EPL_MODE_ALS:
            len += snprintf(buf + len, PAGE_SIZE - len, "obj->hw->als_threshold_low=%d \r\n", obj->hw->als_threshold_low);
            len += snprintf(buf + len, PAGE_SIZE - len, "obj->hw->als_threshold_high=%d \r\n", obj->hw->als_threshold_high);
            break;

    }
    return  len;

}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_wait_time(struct device_driver *ddri, const char *buf, size_t count)
{
    int val;

    sscanf(buf, "%d",&val);
    epl_sensor.wait = (val & 0xf) << 4;

    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_gain(struct device_driver *ddri, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	int value = 0;
    APS_FUN();

    sscanf(buf, "%d", &value);
    value = value & 0x03;

	switch (epl_sensor.mode)
	{
        case EPL_MODE_PS:
            epl_sensor.ps.gain = value;
	        epl_sensor_I2C_Write(epld->client, 0x03, epl_sensor.ps.integration_time | epl_sensor.ps.gain);
		break;

        case EPL_MODE_ALS: //als
            epl_sensor.als.gain = value;
	        epl_sensor_I2C_Write(epld->client, 0x01, epl_sensor.als.integration_time | epl_sensor.als.gain);
		break;

    }
	epl_sensor_update_mode(epld->client);
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_mode(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    int value=0;
    APS_FUN();

    clear_bit(CMC_BIT_PS, &obj->enable);
    clear_bit(CMC_BIT_ALS, &obj->enable);

    sscanf(buf, "%d",&value);
    switch (value)
    {
        case 0:
            epl_sensor.mode = EPL_MODE_IDLE;
            break;

        case 1:
            //set_bit(CMC_BIT_ALS, &obj->enable);
            epl_sensor.mode = EPL_MODE_ALS;
            break;

        case 2:
            //set_bit(CMC_BIT_PS, &obj->enable);
            epl_sensor.mode = EPL_MODE_PS;
            break;

        case 3:
            //set_bit(CMC_BIT_ALS, &obj->enable);
            //set_bit(CMC_BIT_PS, &obj->enable);
            epl_sensor.mode = EPL_MODE_ALS_PS;
            break;
    }

    //epl_sensor_update_mode(obj->client);
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_ir_mode(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();

    sscanf(buf, "%d", &value);

    switch (epl_sensor.mode)
    {
        case EPL_MODE_PS:
            switch(value)
            {
                case 0:
                    epl_sensor.ps.ir_mode = EPL_IR_MODE_CURRENT;
                    break;

                case 1:
                    epl_sensor.ps.ir_mode = EPL_IR_MODE_VOLTAGE;
                    break;
            }

            epl_sensor_I2C_Write(obj->client, 0x05, epl_sensor.ps.ir_on_control | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_drive);
         break;
    }

    epl_sensor_I2C_Write(obj->client, 0x00, epl_sensor.wait | epl_sensor.mode);

    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_ir_contrl(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    uint8_t  data;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();

    sscanf(buf, "%d",&value);

    switch (epl_sensor.mode)
    {
        case EPL_MODE_PS:
            switch(value)
            {
                case 0:
                    epl_sensor.ps.ir_on_control = EPL_IR_ON_CTRL_OFF;
                    break;

                case 1:
                    epl_sensor.ps.ir_on_control = EPL_IR_ON_CTRL_ON;
                    break;
            }

            data = epl_sensor.ps.ir_on_control | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_drive;
            APS_LOG("%s: 0x05 = 0x%x\n", __FUNCTION__, data);

            epl_sensor_I2C_Write(obj->client, 0x05, epl_sensor.ps.ir_on_control | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_drive);
         break;
    }

    epl_sensor_I2C_Write(obj->client, 0x00, epl_sensor.wait | epl_sensor.mode);

    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_ir_drive(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();

    sscanf(buf, "%d", &value);

    switch(epl_sensor.mode)
    {
        case EPL_MODE_PS:
            epl_sensor.ps.ir_drive = (value & 0x03);
            epl_sensor_I2C_Write(obj->client, 0x05, epl_sensor.ps.ir_on_control | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_drive);
        break;
    }

    epl_sensor_I2C_Write(obj->client, 0x00, epl_sensor.wait | epl_sensor.mode);

    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_interrupt_type(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();

    sscanf(buf, "%d",&value);

    switch (epl_sensor.mode)
    {
        case EPL_MODE_PS:
            if(!obj->hw->polling_mode_ps)
            {
                epl_sensor.ps.interrupt_type = value & 0x03;
                epl_sensor_I2C_Write(obj->client, 0x06, epl_sensor.interrupt_control | epl_sensor.ps.persist |epl_sensor.ps.interrupt_type);
                APS_LOG("%s: 0x06 = 0x%x\n", __FUNCTION__, epl_sensor.interrupt_control | epl_sensor.ps.persist |epl_sensor.ps.interrupt_type);
            }
            break;

        case EPL_MODE_ALS: //als
            if(!obj->hw->polling_mode_als)
            {
                epl_sensor.als.interrupt_type = value & 0x03;
                epl_sensor_I2C_Write(obj->client, 0x07, epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);
                APS_LOG("%s: 0x07 = 0x%x\n", __FUNCTION__, epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);
            }
            break;
    }

    epl_sensor_update_mode(obj->client);
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_integration(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();

    sscanf(buf, "%d",&value);

    switch (epl_sensor.mode)
    {
        case EPL_MODE_PS:
            epl_sensor.ps.integration_time = (value & 0xf) << 2;
            epl_sensor_I2C_Write(obj->client, 0x03, epl_sensor.ps.integration_time | epl_sensor.ps.gain);
            epl_sensor_I2C_Read(obj->client, 0x03, 1);
            APS_LOG("%s: 0x03 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.ps.integration_time | epl_sensor.ps.gain, gRawData.raw_bytes[0]);
            break;

        case EPL_MODE_ALS: //als
            epl_sensor.als.integration_time = (value & 0xf) << 2;
            epl_sensor_I2C_Write(obj->client, 0x01, epl_sensor.als.integration_time | epl_sensor.als.gain);
            epl_sensor_I2C_Read(obj->client, 0x01, 1);
            APS_LOG("%s: 0x01 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.als.integration_time | epl_sensor.als.gain, gRawData.raw_bytes[0]);
            break;

    }

    epl_sensor_update_mode(obj->client);
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_adc(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();

    sscanf(buf, "%d",&value);

    switch (epl_sensor.mode)
    {
        case EPL_MODE_PS:
            epl_sensor.ps.adc = (value & 0x3) << 3;
            epl_sensor_I2C_Write(obj->client, 0x04, epl_sensor.ps.adc | epl_sensor.ps.cycle);
            epl_sensor_I2C_Read(obj->client, 0x04, 1);
            APS_LOG("%s: 0x04 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.ps.adc | epl_sensor.ps.cycle, gRawData.raw_bytes[0]);
            break;

        case EPL_MODE_ALS: //als
            epl_sensor.als.adc = (value & 0x3) << 3;
            epl_sensor_I2C_Write(obj->client, 0x02, epl_sensor.als.adc | epl_sensor.als.cycle);
            epl_sensor_I2C_Read(obj->client, 0x02, 1);
            APS_LOG("%s: 0x02 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.als.adc | epl_sensor.als.cycle, gRawData.raw_bytes[0]);
            break;
    }

    epl_sensor_update_mode(obj->client);
    return count;
}

static ssize_t epl_sensor_store_cycle(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();

    sscanf(buf, "%d",&value);

    switch (epl_sensor.mode)
    {
        case EPL_MODE_PS:
            epl_sensor.ps.cycle = (value & 0x7);
            epl_sensor_I2C_Write(obj->client, 0x04, epl_sensor.ps.adc | epl_sensor.ps.cycle);
            epl_sensor_I2C_Read(obj->client, 0x04, 1);
            APS_LOG("%s: 0x04 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.ps.adc | epl_sensor.ps.cycle, gRawData.raw_bytes[0]);
            break;

        case EPL_MODE_ALS: //als
            epl_sensor.als.cycle = (value & 0x7);
            epl_sensor_I2C_Write(obj->client, 0x02, epl_sensor.als.adc | epl_sensor.als.cycle);
            epl_sensor_I2C_Read(obj->client, 0x02, 1);
            APS_LOG("%s: 0x02 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.als.adc | epl_sensor.als.cycle, gRawData.raw_bytes[0]);
            break;
    }

    epl_sensor_update_mode(obj->client);
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_als_report_type(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    struct epl_sensor_priv *epld = epl_sensor_obj;
    APS_FUN();

    sscanf(buf, "%d", &value);
    epl_sensor.als.report_type = value & 0xf;

    if(epl_sensor.als.polling_mode == 0)
        als_intr_update_table(epld);
    return count;
}


static ssize_t epl_sensor_store_ps_polling_mode(struct device_driver *ddri, const char *buf, size_t count)
{
       struct epl_sensor_priv *epld = epl_sensor_obj;
#if !MTK_LTE
    struct hwmsen_object obj_ps;
#endif

    sscanf(buf, "%d",&epld->hw->polling_mode_ps);



    APS_LOG("epld->hw->polling_mode_ps=%d \r\n", epld->hw->polling_mode_ps);

    epl_sensor.ps.polling_mode = epld->hw->polling_mode_ps;

#if !MTK_LTE
    hwmsen_detach(ID_PROXIMITY);
    obj_ps.self = epl_sensor_obj;
    obj_ps.polling = epld->hw->polling_mode_ps;
    obj_ps.sensor_operate = epl_sensor_ps_operate;

    if(hwmsen_attach(ID_PROXIMITY, &obj_ps))
    {
        APS_ERR("[%s]: attach fail !\n", __FUNCTION__);
    }
#else
    alsps_context_obj->ps_ctl.is_report_input_direct = epl_sensor.ps.polling_mode==0? true:false;
#endif  /*!MTK_LTE*/

    epl_sensor_fast_update(epld->client);
    epl_sensor_update_mode(epld->client);

    return count;
}

static ssize_t epl_sensor_store_als_polling_mode(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
#if !MTK_LTE
    struct hwmsen_object obj_als;
#endif
    sscanf(buf, "%d",&epld->hw->polling_mode_als);



    APS_LOG("epld->hw->polling_mode_als=%d \r\n", epld->hw->polling_mode_als);
    epl_sensor.als.polling_mode = epld->hw->polling_mode_als;

#if !MTK_LTE
    hwmsen_detach(ID_LIGHT);
    obj_als.self = epl_sensor_obj;
    obj_als.polling = epld->hw->polling_mode_als;
    obj_als.sensor_operate = epl_sensor_als_operate;

    if(hwmsen_attach(ID_LIGHT, &obj_als))
    {
        APS_ERR("[%s]: attach fail !\n", __FUNCTION__);
    }
#else
    alsps_context_obj->als_ctl.is_report_input_direct = epl_sensor.als.polling_mode==0? true:false;
#endif /*!MTK_LTE*/

    //update als intr table
    if(epl_sensor.als.polling_mode == 0)
        als_intr_update_table(epld);

    epl_sensor_fast_update(epld->client);
    epl_sensor_update_mode(epld->client);

    return count;
}
static ssize_t epl_sensor_store_ps_w_calfile(struct device_driver *ddri, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
    int ps_hthr=0, ps_lthr=0, ps_cancelation=0;
    int ps_cal_len = 0;
    char ps_calibration[20];
	APS_FUN();

	if(!epl_sensor_obj)
    {
        APS_ERR("epl_obj is null!!\n");
        return 0;
    }
    sscanf(buf, "%d,%d,%d",&ps_cancelation, &ps_hthr, &ps_lthr);

    ps_cal_len = sprintf(ps_calibration, "%d,%d,%d",  ps_cancelation, ps_hthr, ps_lthr);

    write_factory_calibration(epld, ps_calibration, ps_cal_len);
	return count;
}
/*----------------------------------------------------------------------------*/

static ssize_t epl_sensor_store_unlock(struct device_driver *ddri, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
    int mode;
    APS_FUN();

    sscanf(buf, "%d",&mode);

    APS_LOG("mode = %d \r\n", mode);
	switch (mode)
	{
		case 0: //PS unlock and run
        	epl_sensor.ps.compare_reset = EPL_CMP_RUN;
        	epl_sensor.ps.lock = EPL_UN_LOCK;
        	epl_sensor_I2C_Write(epld->client,0x1b, epl_sensor.ps.compare_reset |epl_sensor.ps.lock);
		break;

		case 1: //PS unlock and reset
        	epl_sensor.ps.compare_reset = EPL_CMP_RESET;
        	epl_sensor.ps.lock = EPL_UN_LOCK;
        	epl_sensor_I2C_Write(epld->client,0x1b, epl_sensor.ps.compare_reset |epl_sensor.ps.lock);
		break;

		case 2: //ALS unlock and run
        	epl_sensor.als.compare_reset = EPL_CMP_RUN;
        	epl_sensor.als.lock = EPL_UN_LOCK;
        	epl_sensor_I2C_Write(epld->client,0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);
		break;

        case 3: //ALS unlock and reset
    		epl_sensor.als.compare_reset = EPL_CMP_RESET;
        	epl_sensor.als.lock = EPL_UN_LOCK;
        	epl_sensor_I2C_Write(epld->client,0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);
        break;

		case 4: //ps+als
		    //PS unlock and run
        	epl_sensor.ps.compare_reset = EPL_CMP_RUN;
        	epl_sensor.ps.lock = EPL_UN_LOCK;
        	epl_sensor_I2C_Write(epld->client,0x1b, epl_sensor.ps.compare_reset |epl_sensor.ps.lock);

			//ALS unlock and run
        	epl_sensor.als.compare_reset = EPL_CMP_RUN;
        	epl_sensor.als.lock = EPL_UN_LOCK;
        	epl_sensor_I2C_Write(epld->client,0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);
		break;
	}
    /*double check PS or ALS lock*/


	return count;
}

static ssize_t epl_sensor_store_als_ch_sel(struct device_driver *ddri, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
    int ch_sel;
    APS_FUN();

    sscanf(buf, "%d",&ch_sel);

    APS_LOG("channel selection = %d \r\n", ch_sel);
	switch (ch_sel)
	{
		case 0: //ch0
		    epl_sensor.als.interrupt_channel_select = EPL_ALS_INT_CHSEL_0;
		break;

		case 1: //ch1
        	epl_sensor.als.interrupt_channel_select = EPL_ALS_INT_CHSEL_1;
		break;
	}
    epl_sensor_I2C_Write(epld->client,0x07, epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);

    epl_sensor_update_mode(epld->client);

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_als_lux_per_count(struct device_driver *ddri, const char *buf, size_t count)
{
    int lux_per_count = 0;
    sscanf(buf, "%d",&lux_per_count);
    epl_sensor.als.factory.lux_per_count = lux_per_count;
    return count;
}

static ssize_t epl_sensor_store_ps_cancelation(struct device_driver *ddri, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
    int cancelation;
    APS_FUN();

    sscanf(buf, "%d",&cancelation);

    epl_sensor.ps.cancelation = cancelation;

    APS_LOG("epl_sensor.ps.cancelation = %d \r\n", epl_sensor.ps.cancelation);

    epl_sensor_I2C_Write(epld->client,0x22, (u8)(epl_sensor.ps.cancelation& 0xff));
    epl_sensor_I2C_Write(epld->client,0x23, (u8)((epl_sensor.ps.cancelation & 0xff00) >> 8));

	return count;
}

static ssize_t epl_sensor_show_ps_polling(struct device_driver *ddri, char *buf)
{
    u16 *tmp = (u16*)buf;
    tmp[0]= epl_sensor.ps.polling_mode;
    return 2;
}

static ssize_t epl_sensor_show_als_polling(struct device_driver *ddri, char *buf)
{
    u16 *tmp = (u16*)buf;
    tmp[0]= epl_sensor.als.polling_mode;
    return 2;
}

static ssize_t epl_sensor_show_ps_run_cali(struct device_driver *ddri, char *buf)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	ssize_t len = 0;
    int ret;

    APS_FUN();

    ret = epl_run_ps_calibration(epld);

    len += snprintf(buf+len, PAGE_SIZE-len, "ret = %d\r\n", ret);

	return len;
}

static ssize_t epl_sensor_show_pdata(struct device_driver *ddri, char *buf)
{
      ssize_t len = 0;
      int ps_raw;
      APS_FUN();

      ps_raw = epl_sensor_ps_data();

      APS_LOG("[%s]: ps_raw=%d \r\n", __func__, ps_raw);
      len += snprintf(buf + len, PAGE_SIZE - len, "%d", ps_raw);
      return len;

}

static ssize_t epl_sensor_show_als_data(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    u16 als_raw = 0;
    APS_FUN();

    als_raw = epl_sensor_als_data();

    APS_LOG("[%s]: als_raw=%d \r\n", __func__, als_raw);
    len += snprintf(buf + len, PAGE_SIZE - len, "%d", als_raw);
    return len;

}

static ssize_t epl_sensor_store_reg_write(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    int reg;
    int data;
    APS_FUN();

    sscanf(buf, "%x,%x",&reg, &data);

    APS_LOG("[%s]: reg=0x%x, data=0x%x", __func__, reg, data);

    if(reg == 0x00 && ((data & 0x0f) == EPL_MODE_PS || (data & 0x0f) == EPL_MODE_ALS_PS))
    {
        set_bit(CMC_BIT_PS, &epld->enable);
    }
    else
    {
        clear_bit(CMC_BIT_PS, &epld->enable);
    }

    epl_sensor_I2C_Write(epld->client, reg, data);

    return count;
}


static ssize_t epl_sensor_show_renvo(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;

    APS_FUN();
    APS_LOG("gRawData.renvo=0x%x \r\n", epl_sensor.revno);

    len += snprintf(buf+len, PAGE_SIZE-len, "%x", epl_sensor.revno);

    return len;
}

static ssize_t epl_sensor_show_als_intr_table(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    int idx = 0;

    len += snprintf(buf+len, PAGE_SIZE-len, "ALS Lux Table\n");
    for (idx = 0; idx < 10; idx++)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "[%d]: %ld", idx, (*(als_lux_intr_level + idx)));
	}

    len += snprintf(buf+len, PAGE_SIZE-len, "\n ALS ADC Table\n");
    for (idx = 0; idx < 10; idx++)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "[%d]: %ld", idx, (*(als_adc_intr_level + idx)));
	}

    return len;
}

#if ARIMA_PATCH
static ssize_t epl_sensor_show_ps_cal_raw(struct device_driver *ddri, char *buf)
{
#if CALI_APP_JNI
    long *tmp = (long*)buf;
#endif
    struct epl_sensor_priv *epld = epl_sensor_obj;
    u16 ch1=0;
    u32 ch1_all=0;
    int count=3, i, ps_raw=0;
    ssize_t len = 0;

    APS_FUN();
    if(!epl_sensor_obj)
    {
        APS_ERR("epl_sensor_obj is null!!\n");
        return 0;
    }
    for(i=0; i<count; i++)
    {
        ps_raw = epl_sensor_ps_data();
	    ch1_all = ch1_all+ ps_raw;
    }
//sherman modify_2
	///ch1 = (u16)ch1_all/count;
	ch1 = (u16)(ch1_all/count);
	APS_LOG("[%s]: ch1=%d\n", __FUNCTION__, ch1);
	clear_bit(CMC_BIT_PS, &epld->enable);
	epl_sensor_update_mode(epld->client);

	memset(high, '\0', sizeof(high));
	high_length = 0;
	high_length = sprintf(high, "%d",  ch1);
#if CALI_APP_JNI
    tmp[0] = ch1;
    return 2;
#else
	len += snprintf(buf+len, PAGE_SIZE-len, "%d \r\n", ch1);
	return len;
#endif
}

static ssize_t epl_sensor_show_ps_crosstalk_raw(struct device_driver *ddri, char *buf)
{

#if CALI_APP_JNI
    long *tmp = (long*)buf;
#endif
    struct epl_sensor_priv *epld = epl_sensor_obj;
    u16 ch1=0;
    u32 ch1_all=0, ps_raw;
    int count =3, i;
    ssize_t len = 0;

    APS_FUN();
    if(!epl_sensor_obj)
    {
        APS_ERR("epl_sensor_obj is null!!\n");
        return 0;
    }
    for(i=0; i<count; i++)
    {
        ps_raw = epl_sensor_ps_data();
	    ch1_all = ch1_all+ ps_raw;
    }
//sherman modify_2
//	ch1 = (u16)ch1_all/count;
	ch1 = (u16)(ch1_all/count);
	APS_LOG("[%s]: ch1=%d\n", __FUNCTION__, ch1);
	clear_bit(CMC_BIT_PS, &epld->enable);
    epl_sensor_update_mode(epld->client);

	memset(ctalk, '\0', sizeof(ctalk));
	ctalk_length = 0;
	ctalk_length = sprintf(ctalk, "%d",  ch1);
#if CALI_APP_JNI
    tmp[0] = ch1;
    return 2;
#else
	len += snprintf(buf+len, PAGE_SIZE-len, "%d \r\n", ch1);
	return len;
#endif
}


/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_show_ps_rawdata(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    int ps_raw;
////sherman modify_2
    struct epl_sensor_priv *epld = epl_sensor_obj;
#if CALI_APP_JNI
    long *tmp = (long*)buf;
#endif
    APS_FUN();

    ps_raw = epl_sensor_ps_data();

    APS_LOG("[%s]: ps_raw=%d \r\n", __func__, ps_raw);
////sherman modify_2
	clear_bit(CMC_BIT_PS, &epld->enable);
    epl_sensor_update_mode(epld->client);
////  return len;
#if CALI_APP_JNI
    tmp[0] = ps_raw;
    return 2;
#else
	len += snprintf(buf+len, PAGE_SIZE-len, "%d \r\n", ps_raw);
	return len;
#endif
}

static ssize_t epl_sensor_show_ps_read_cal(struct device_driver *ddri, char *buf)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
#if CALI_APP_JNI
    long *tmp = (long*)buf;
#endif
    ssize_t len = 0;
    APS_LOG("[%s]: \n", __FUNCTION__);

    arima_ps_calibration_read(epld);
#if CALI_APP_JNI
    tmp[0] = gRawData.ps_als_factory.ps_cal_h;
    return 2;
#else
    len += snprintf(buf+len, PAGE_SIZE-len, "%d \r\n", gRawData.ps_als_factory.ps_cal_h);
	return len;
#endif
}

static ssize_t epl_sensor_show_ps_read_ct(struct device_driver *ddri, char *buf)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
#if CALI_APP_JNI
    long *tmp = (long*)buf;
#endif
    ssize_t len = 0;
    APS_LOG("[%s]: \n", __FUNCTION__);

    arima_ps_calibration_read(epld);
#if CALI_APP_JNI
    tmp[0] = gRawData.ps_als_factory.ps_cal_l;
    return 2;
#else
    len += snprintf(buf+len, PAGE_SIZE-len, "%d \r\n", gRawData.ps_als_factory.ps_cal_l);
	return len;
#endif
}

static ssize_t epl_sensor_show_ps_savedata(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    struct epl_sensor_priv *epld = epl_sensor_obj;

    APS_LOG("[%s]: \n", __FUNCTION__);

    epl_sensor_calibaration_write(epld, high, high_length);
    epl_sensor_crosstalk_write(epld, ctalk, ctalk_length);

    return len;
}

static ssize_t epl_sensor_show_als_rawdata(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    u16 als_raw = 0;
////sherman modify_2
    struct epl_sensor_priv *epld = epl_sensor_obj;
#if CALI_APP_JNI
    long *tmp = (long*)buf;
#endif
    APS_FUN();

    als_raw = epl_sensor_als_data();

    APS_LOG("[%s]: als_raw=%d \r\n", __func__, als_raw);
////sherman modify_2
	clear_bit(CMC_BIT_ALS, &epld->enable);
    epl_sensor_update_mode(epld->client);
#if CALI_APP_JNI
    tmp[0] = als_raw;
    return 2;
#else
	len += snprintf(buf+len, PAGE_SIZE-len, "%d \r\n", als_raw);
	return len;
#endif
}

#endif

/*CTS --> S_IWUSR | S_IRUGO*/
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(elan_status,	S_IWUSR  | S_IRUGO, epl_sensor_show_status,	NULL);
static DRIVER_ATTR(elan_reg,    				S_IWUSR  | S_IRUGO, epl_sensor_show_reg,   				NULL										);
static DRIVER_ATTR(als_enable,					S_IWUSR  | S_IRUGO, NULL,   							epl_sensor_store_als_enable					);
static DRIVER_ATTR(als_report_type,				S_IWUSR  | S_IRUGO, NULL,								epl_sensor_store_als_report_type			);
static DRIVER_ATTR(als_polling_mode,			S_IWUSR  | S_IRUGO, epl_sensor_show_als_polling,   		epl_sensor_store_als_polling_mode			);
static DRIVER_ATTR(als_lux_per_count,			S_IWUSR  | S_IRUGO, NULL,   					 		epl_sensor_store_als_lux_per_count			);
static DRIVER_ATTR(ps_enable,					S_IWUSR  | S_IRUGO, NULL,   							epl_sensor_store_ps_enable					);
static DRIVER_ATTR(ps_polling_mode,			    S_IWUSR  | S_IRUGO, epl_sensor_show_ps_polling,   		epl_sensor_store_ps_polling_mode			);
static DRIVER_ATTR(ir_mode,					    S_IWUSR  | S_IRUGO, NULL,   							epl_sensor_store_ir_mode					);
static DRIVER_ATTR(ir_drive,					S_IWUSR  | S_IRUGO, NULL,   							epl_sensor_store_ir_drive					);
static DRIVER_ATTR(ir_on,						S_IWUSR  | S_IRUGO, NULL,								epl_sensor_store_ir_contrl					);
static DRIVER_ATTR(interrupt_type,				S_IWUSR  | S_IRUGO, NULL,								epl_sensor_store_interrupt_type				);
static DRIVER_ATTR(integration,					S_IWUSR  | S_IRUGO, NULL,								epl_sensor_store_integration				);
static DRIVER_ATTR(gain,					    S_IWUSR  | S_IRUGO, NULL,								epl_sensor_store_gain					    );
static DRIVER_ATTR(adc,					        S_IWUSR  | S_IRUGO, NULL,								epl_sensor_store_adc						);
static DRIVER_ATTR(cycle,						S_IWUSR  | S_IRUGO, NULL,								epl_sensor_store_cycle						);
static DRIVER_ATTR(mode,						S_IWUSR  | S_IRUGO, NULL,   							epl_sensor_store_mode						);
static DRIVER_ATTR(wait_time,					S_IWUSR  | S_IRUGO, NULL,   					 		epl_sensor_store_wait_time					);
static DRIVER_ATTR(set_threshold,     			S_IWUSR  | S_IRUGO, epl_sensor_show_threshold,                epl_sensor_store_threshold			);
static DRIVER_ATTR(cal_raw, 					S_IWUSR  | S_IRUGO, epl_sensor_show_cal_raw, 	  		NULL										);
static DRIVER_ATTR(unlock,				        S_IWUSR  | S_IRUGO, NULL,			                    epl_sensor_store_unlock						);
static DRIVER_ATTR(als_ch,				        S_IWUSR  | S_IRUGO, NULL,			                    epl_sensor_store_als_ch_sel					);
static DRIVER_ATTR(ps_cancel,				    S_IWUSR  | S_IRUGO, NULL,			                    epl_sensor_store_ps_cancelation				);
static DRIVER_ATTR(run_ps_cali, 				S_IWUSR  | S_IRUGO, epl_sensor_show_ps_run_cali, 	  	NULL								    	);
static DRIVER_ATTR(pdata,                       S_IWUSR  | S_IRUGO, epl_sensor_show_pdata,              NULL                                        );
static DRIVER_ATTR(als_data,                    S_IWUSR  | S_IRUGO, epl_sensor_show_als_data,           NULL                                        );
static DRIVER_ATTR(ps_w_calfile,				S_IWUSR  | S_IRUGO, NULL,			                    epl_sensor_store_ps_w_calfile				);
static DRIVER_ATTR(i2c_w,                       S_IWUSR  | S_IRUGO, NULL,                               epl_sensor_store_reg_write                  );
static DRIVER_ATTR(elan_renvo,                  S_IWUSR  | S_IRUGO, epl_sensor_show_renvo,              NULL                                        );
static DRIVER_ATTR(als_intr_table,					    S_IWUSR  | S_IRUGO, epl_sensor_show_als_intr_table,            NULL                         );
#if ARIMA_PATCH
static DRIVER_ATTR(ps_cal_raw, 		    S_IWUSR  | S_IRUGO, epl_sensor_show_ps_cal_raw, 	  	NULL);
static DRIVER_ATTR(ps_crosstalk_raw, 	S_IWUSR  | S_IRUGO, epl_sensor_show_ps_crosstalk_raw, 	NULL);
static DRIVER_ATTR(ps_rawdata, 		    S_IWUSR  | S_IRUGO, epl_sensor_show_ps_rawdata, 	  	NULL);
static DRIVER_ATTR(ps_savedata, 	    S_IWUSR  | S_IRUGO, epl_sensor_show_ps_savedata, 	  	NULL);
static DRIVER_ATTR(ps_read_cal, 	    S_IWUSR  | S_IRUGO, epl_sensor_show_ps_read_cal, 	  	NULL);
static DRIVER_ATTR(ps_read_ct,  	    S_IWUSR  | S_IRUGO, epl_sensor_show_ps_read_ct, 	  	NULL);
static DRIVER_ATTR(als_rawdata, 	    S_IWUSR  | S_IRUGO, epl_sensor_show_als_rawdata, 	  	NULL);
#endif

/*----------------------------------------------------------------------------*/
static struct driver_attribute * epl_sensor_attr_list[] =
{
    &driver_attr_elan_status,
    &driver_attr_elan_reg,
    &driver_attr_als_enable,
    &driver_attr_als_report_type,
    &driver_attr_als_polling_mode,
    &driver_attr_als_lux_per_count,
    &driver_attr_ps_enable,
    &driver_attr_ps_polling_mode,
    &driver_attr_elan_renvo,
    &driver_attr_mode,
    &driver_attr_ir_mode,
    &driver_attr_ir_drive,
    &driver_attr_ir_on,
    &driver_attr_interrupt_type,
    &driver_attr_cal_raw,
    &driver_attr_set_threshold,
    &driver_attr_wait_time,
    &driver_attr_integration,
    &driver_attr_gain,
    &driver_attr_adc,
    &driver_attr_cycle,
    &driver_attr_unlock,
    &driver_attr_ps_cancel,
    &driver_attr_als_ch,
    &driver_attr_run_ps_cali,
    &driver_attr_pdata,
    &driver_attr_als_data,
    &driver_attr_ps_w_calfile,
    &driver_attr_i2c_w,
    &driver_attr_als_intr_table,
#if ARIMA_PATCH
    &driver_attr_ps_cal_raw,
    &driver_attr_ps_crosstalk_raw,
    &driver_attr_ps_rawdata,
    &driver_attr_ps_savedata,
    &driver_attr_ps_read_cal,
    &driver_attr_ps_read_ct,
    &driver_attr_als_rawdata,
#endif
};

/*----------------------------------------------------------------------------*/
static int epl_sensor_create_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(epl_sensor_attr_list)/sizeof(epl_sensor_attr_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        if((err = driver_create_file(driver, epl_sensor_attr_list[idx])))
        {
            APS_ERR("driver_create_file (%s) = %d\n", epl_sensor_attr_list[idx]->attr.name, err);
            break;
        }
    }
    return err;
}



/*----------------------------------------------------------------------------*/
static int epl_sensor_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(epl_sensor_attr_list)/sizeof(epl_sensor_attr_list[0]));

    if (!driver)
        return -EINVAL;

    for (idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, epl_sensor_attr_list[idx]);
    }

    return err;
}



/******************************************************************************
 * Function Configuration
******************************************************************************/
static int epl_sensor_open(struct inode *inode, struct file *file)
{
    file->private_data = epl_sensor_i2c_client;

    APS_FUN();
#if PS_BOOT_K
    epl_sensor_ps_boot_k(epl_sensor_i2c_client);
#endif /*PS_BOOT_K*/
    if (!file->private_data)
    {
        APS_ERR("null pointer!!\n");
        return -EINVAL;
    }

    return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int epl_sensor_release(struct inode *inode, struct file *file)
{
    APS_FUN();
    file->private_data = NULL;
    return 0;
}

/*----------------------------------------------------------------------------*/
static long epl_sensor_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct i2c_client *client = (struct i2c_client*)file->private_data;
    struct epl_sensor_priv *obj = i2c_get_clientdata(client);
    int err=0, ps_result=0, threshold[2], als_raw=0;
    void __user *ptr = (void __user*) arg;
    int dat=0;
    uint32_t enable;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;
#if 0
    int ps_cali;
#endif
    APS_LOG("%s cmd = 0x%04x", __FUNCTION__, cmd);
    switch (cmd)
    {
        case ALSPS_SET_PS_MODE:
            if(copy_from_user(&enable, ptr, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }

            if(enable_ps != enable)
            {
                if(enable)
                {
                    //wake_lock(&ps_lock);
                    set_bit(CMC_BIT_PS, &obj->enable);
                }
                else
                {
                    clear_bit(CMC_BIT_PS, &obj->enable);
                    //wake_unlock(&ps_lock);
                }
                epl_sensor_fast_update(obj->client); //if fast to on/off ALS, it read als for DOC_OFF setting for als_operate
                epl_sensor_update_mode(obj->client);
            }
            break;

        case ALSPS_GET_PS_MODE:
            enable=test_bit(CMC_BIT_PS, &obj->enable);
            if(copy_to_user(ptr, &enable, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_PS_DATA:

            epl_sensor_ps_data();
            dat = epl_sensor.ps.compare_low >> 3;

            APS_LOG("ioctl ps state value = %d \n", dat);

            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_PS_RAW_DATA:

            dat = epl_sensor_ps_data();

            APS_LOG("ioctl ps raw value = %d \n", dat);

            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_SET_ALS_MODE:
            if(copy_from_user(&enable, ptr, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            if(enable_als != enable)
            {
                if(enable)
                {
                    set_bit(CMC_BIT_ALS, &obj->enable);
                    epl_sensor_fast_update(obj->client);
                }
                else
                {
                    clear_bit(CMC_BIT_ALS, &obj->enable);
                }
                epl_sensor_update_mode(obj->client);
            }
            break;

        case ALSPS_GET_ALS_MODE:
            enable=test_bit(CMC_BIT_ALS, &obj->enable);
            if(copy_to_user(ptr, &enable, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_ALS_DATA:

            als_raw = epl_sensor_als_data();
            dat = epl_sensor_get_als_value(obj, als_raw);

            APS_LOG("ioctl get als data = %d\n", dat);

            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_ALS_RAW_DATA:

            dat = epl_sensor_als_data();
            APS_LOG("ioctl get als raw data = %d\n", dat);

            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

/*----------------------------------for factory mode test---------------------------------------*/
		case ALSPS_GET_PS_TEST_RESULT:
            if(enable_ps == 0)
            {
                set_bit(CMC_BIT_PS, &obj->enable);
                epl_sensor_update_mode(obj->client);
            }
            epl_sensor_read_ps(obj->client);
            if(epl_sensor.ps.data.data > obj->hw->ps_threshold_high)
			{
			    ps_result = 0;
			}
			else
			    ps_result = 1;

			APS_LOG("[%s] ps_result = %d \r\n", __func__, ps_result);

			if(copy_to_user(ptr, &ps_result, sizeof(ps_result)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
#if 0 //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

		case ALSPS_IOCTL_CLR_CALI:
#if 0
			if(copy_from_user(&dat, ptr, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(dat == 0)
				obj->ps_cali = 0;
#else

            APS_LOG("[%s]: ALSPS_IOCTL_CLR_CALI \r\n", __func__);
#endif
			break;

		case ALSPS_IOCTL_GET_CALI:
#if 0
			ps_cali = obj->ps_cali ;
			APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
			if(copy_to_user(ptr, &ps_cali, sizeof(ps_cali)))
			{
				err = -EFAULT;
				goto err_out;
			}
#else
            APS_LOG("[%s]: ALSPS_IOCTL_GET_CALI \r\n", __func__);
#endif
			break;

		case ALSPS_IOCTL_SET_CALI:
#if 0
			if(copy_from_user(&ps_cali, ptr, sizeof(ps_cali)))
			{
				err = -EFAULT;
				goto err_out;
			}

			obj->ps_cali = ps_cali;
			APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
#else
            APS_LOG("[%s]: ALSPS_IOCTL_SET_CALI \r\n", __func__);
#endif
			break;

		case ALSPS_SET_PS_THRESHOLD:
			if(copy_from_user(threshold, ptr, sizeof(threshold)))
			{
				err = -EFAULT;
				goto err_out;
			}
#if 0
			APS_ERR("%s set threshold high: 0x%x, low: 0x%x\n", __func__, threshold[0],threshold[1]);
			atomic_set(&obj->ps_thd_val_high,  (threshold[0]+obj->ps_cali));
			atomic_set(&obj->ps_thd_val_low,  (threshold[1]+obj->ps_cali));//need to confirm
			set_psensor_threshold(obj->client);
#else
            APS_LOG("[%s] set threshold high: %d, low: %d\n", __func__, threshold[0],threshold[1]);
            obj->hw->ps_threshold_high = threshold[0];
            obj->hw->ps_threshold_low = threshold[1];
            set_psensor_intr_threshold(obj->hw->ps_threshold_low, obj->hw->ps_threshold_high);
#endif
			break;
#endif //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		case ALSPS_GET_PS_THRESHOLD_HIGH:
#if 0
			APS_ERR("%s get threshold high before cali: 0x%x\n", __func__, atomic_read(&obj->ps_thd_val_high));
			threshold[0] = atomic_read(&obj->ps_thd_val_high) - obj->ps_cali;
			APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
			APS_ERR("%s get threshold high: 0x%x\n", __func__, threshold[0]);
#else
            threshold[0] = obj->hw->ps_threshold_high;
            APS_LOG("[%s] get threshold high: %d\n", __func__, threshold[0]);
#endif
			if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_THRESHOLD_LOW:
#if 0
			APS_ERR("%s get threshold low before cali: 0x%x\n", __func__, atomic_read(&obj->ps_thd_val_low));
			threshold[0] = atomic_read(&obj->ps_thd_val_low) - obj->ps_cali;
			APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
			APS_ERR("%s get threshold low: 0x%x\n", __func__, threshold[0]);
#else
            threshold[0] = obj->hw->ps_threshold_low;
            APS_LOG("[%s] get threshold low: %d\n", __func__, threshold[0]);
#endif
			if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
		/*------------------------------------------------------------------------------------------*/
        default:
            APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
            err = -ENOIOCTLCMD;
            break;
    }

err_out:
    return err;
}


/*----------------------------------------------------------------------------*/
static struct file_operations epl_sensor_fops =
{
    .owner = THIS_MODULE,
    .open = epl_sensor_open,
    .release = epl_sensor_release,
    .unlocked_ioctl = epl_sensor_unlocked_ioctl,
};


/*----------------------------------------------------------------------------*/
static struct miscdevice epl_sensor_device =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "als_ps",
    .fops = &epl_sensor_fops,
};


/*----------------------------------------------------------------------------*/
static int epl_sensor_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
    //struct epl_sensor_priv *obj = i2c_get_clientdata(client);
    //bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    //bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;
    APS_FUN();
#if 0
    if(msg.event == PM_EVENT_SUSPEND)
    {
        if(!obj)
        {
            APS_ERR("null pointer!!\n");
            return -EINVAL;
        }
        atomic_set(&obj->als_suspend, 1);
        if(enable_ps == 1){
            atomic_set(&obj->ps_suspend, 0);
            APS_LOG("[%s]: ps enable \r\n", __func__);
        }
        else{
            atomic_set(&obj->ps_suspend, 1);
            APS_LOG("[%s]: ps disable \r\n", __func__);
            epl_sensor_update_mode(obj->client);
        }

        epl_sensor_power(obj->hw, 0);
    }
#endif
    return 0;

}



/*----------------------------------------------------------------------------*/
static int epl_sensor_i2c_resume(struct i2c_client *client)
{
    //struct epl_sensor_priv *obj = i2c_get_clientdata(client);
    //bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    //bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;
    APS_FUN();
#if 0
    if(!obj)
    {
        APS_ERR("null pointer!!\n");
        return -EINVAL;
    }
    atomic_set(&obj->ps_suspend, 0);
    atomic_set(&obj->als_suspend, 0);
    if(enable_ps == 1)
    {
        APS_LOG("[%s]: ps enable \r\n", __func__);
    }
    else
    {
        APS_LOG("[%s]: ps disable \r\n", __func__);
        epl_sensor_update_mode(obj->client);
    }

    epl_sensor_power(obj->hw, 1);
#endif
    return 0;
}



/*----------------------------------------------------------------------------*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
static void epl_sensor_early_suspend(struct early_suspend *h)
{
    ///early_suspend is only applied for ALS
    struct epl_sensor_priv *obj = container_of(h, struct epl_sensor_priv, early_drv);
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;

    APS_FUN();

    if(!obj)
    {
        APS_ERR("null pointer!!\n");
        return;
    }

    atomic_set(&obj->als_suspend, 1);
    if(enable_ps == 1){
        //atomic_set(&obj->ps_suspend, 0);
        APS_LOG("[%s]: ps enable \r\n", __func__);
    }
    else{
        //atomic_set(&obj->ps_suspend, 1);
        APS_LOG("[%s]: ps disable \r\n", __func__);
        epl_sensor_update_mode(obj->client);
    }
}



/*----------------------------------------------------------------------------*/
static void epl_sensor_late_resume(struct early_suspend *h)
{
    /*late_resume is only applied for ALS*/
    struct epl_sensor_priv *obj = container_of(h, struct epl_sensor_priv, early_drv);
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable);

    APS_FUN();

    if(!obj)
    {
        APS_ERR("null pointer!!\n");
        return;
    }

    atomic_set(&obj->als_suspend, 0);
    atomic_set(&obj->ps_suspend, 0);

    if(enable_ps == 1)
    {
        //atomic_set(&obj->ps_suspend, 0);
        APS_LOG("[%s]: ps enable \r\n", __func__);
    }
    else
    {
        APS_LOG("[%s]: ps disable \r\n", __func__);
        epl_sensor_fast_update(obj->client);
        epl_sensor_update_mode(obj->client);
    }
}

#endif

#if MTK_LTE /*MTK_LTE start .................*/
/*--------------------------------------------------------------------------------*/
static int als_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int als_enable_nodata(int en)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;
	if(!obj)
	{
		APS_ERR("obj is null!!\n");
		return -1;
	}
	APS_LOG("[%s] als enable en = %d\n", __func__, en);

    if(enable_als != en)
    {
        if(en)
        {
            set_bit(CMC_BIT_ALS, &obj->enable);
#if ARIMA_PATCH
            arima_als_calibration_read(obj);
#endif
            epl_sensor_fast_update(obj->client);
        }
        else
        {
            clear_bit(CMC_BIT_ALS, &obj->enable);
        }
        epl_sensor_update_mode(obj->client);

    }

	return 0;
}
/*--------------------------------------------------------------------------------*/
static int als_set_delay(u64 ns)
{
	return 0;
}
/*--------------------------------------------------------------------------------*/
static int als_get_data(int* value, int* status)
{
	int err = 0;
	u16 report_lux = 0;
	struct epl_sensor_priv *obj = epl_sensor_obj;
	if(!obj)
	{
		APS_ERR("obj is null!!\n");
		return -1;
	}

    epl_sensor_read_als(obj->client);
    report_lux = epl_sensor_get_als_value(obj, epl_sensor.als.data.channels[1]);

    *value = report_lux;
    *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    APS_LOG("[%s]:*value = %d\n", __func__, *value);

	return err;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int ps_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}
/*--------------------------------------------------------------------------------*/

//<2015/12/02 ShermanWei,wakelock while psensor on
void epl_psensor_lock(int enable_pflag){

	APS_LOG("[%s], enable_pflag=%d\n",__func__,enable_pflag);

	if(enable_pflag){
		wake_lock(&ps_lock);
	}
	else{
		wake_unlock(&ps_lock);
	}

}
//>2015/12/02 ShermanWei,

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int ps_enable_nodata(int en)
{
	struct epl_sensor_priv *obj = epl_sensor_obj;
    struct i2c_client *client = obj->client;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    APS_LOG("ps enable = %d\n", en);

    if(enable_ps != en)
    {
        if(en)
        {
            //wake_lock(&ps_lock);
            epl_psensor_lock(1);
            set_bit(CMC_BIT_PS, &obj->enable);
#if ARIMA_PATCH
            arima_ps_calibration_read(obj);
#endif
        }
        else
        {
            clear_bit(CMC_BIT_PS, &obj->enable);
            //wake_unlock(&ps_lock);
            epl_psensor_lock(0);
        }
        epl_sensor_fast_update(client); //if fast to on/off ALS, it read als for DOC_OFF setting for als_operate
        epl_sensor_update_mode(client);
    }

	return 0;

}
/*--------------------------------------------------------------------------------*/
static int ps_set_delay(u64 ns)
{
	return 0;
}
/*--------------------------------------------------------------------------------*/
static int ps_get_data(int* value, int* status)
{

    int err = 0;
    struct epl_sensor_priv *obj = epl_sensor_obj;
    struct i2c_client *client = obj->client;

    APS_LOG("---SENSOR_GET_DATA---\n\n");

    epl_sensor_read_ps(client);

//<2015/12/02 ShermanWei,correct report status
    ////*value = epl_sensor.ps.compare_low >> 3;
    *value = gRawData.ps_state;
//>2015/12/02 ShermanWei
    *status = SENSOR_STATUS_ACCURACY_MEDIUM;

    APS_LOG("[%s]:*value = %d\n", __func__, *value);

	return err;
}
/*----------------------------------------------------------------------------*/

#else   /*MTK_LTE*/

/*----------------------------------------------------------------------------*/
int epl_sensor_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
                       void* buff_out, int size_out, int* actualout)
{
    int err = 0;
    int value;
    struct hwm_sensor_data* sensor_data;
    struct epl_sensor_priv *obj = (struct epl_sensor_priv *)self;
    struct i2c_client *client = obj->client;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    APS_LOG("epl_sensor_ps_operate command = %x\n",command);
    switch (command)
    {
        case SENSOR_DELAY:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_ERR("Set delay parameter error!\n");
                err = -EINVAL;
            }
            break;

        case SENSOR_ENABLE:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_ERR("Enable sensor parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                value = *(int *)buff_in;
                APS_LOG("ps enable = %d\n", value);
                if(enable_ps != value)
                {
                    if(value)
                    {
						///sherman modify_5
						gRawData.ps_state=1;
                        //wake_lock(&ps_lock);
                        set_bit(CMC_BIT_PS, &obj->enable);
#if ARIMA_PATCH
                        arima_ps_calibration_read(obj);
#endif
                    }
                    else
                    {
                        clear_bit(CMC_BIT_PS, &obj->enable);
                        //wake_unlock(&ps_lock);
                    }
                    epl_sensor_fast_update(client); //if fast to on/off ALS, it read als for DOC_OFF setting for als_operate
                    epl_sensor_update_mode(client);
                }
            }

            break;

        case SENSOR_GET_DATA:
            APS_LOG(" get ps data !!!!!!\n");
            if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
            {
                APS_ERR("get sensor data parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                APS_LOG("---SENSOR_GET_DATA---\n\n");

                epl_sensor_read_ps(client);

                sensor_data = (struct hwm_sensor_data *)buff_out;
                ///sensor_data->values[0] = epl_sensor.ps.compare_low >> 3;
                ///sherman modify_5
                sensor_data->values[0] = gRawData.ps_state;
                sensor_data->values[1] = epl_sensor.ps.data.data;
                sensor_data->value_divide = 1;
                sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;

            }
            break;

        default:
            APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
            err = -1;
            break;
    }

    return err;

}



int epl_sensor_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
                        void* buff_out, int size_out, int* actualout)
{
    int err = 0;
    int value = 0;
    struct hwm_sensor_data* sensor_data;
    struct epl_sensor_priv *obj = (struct epl_sensor_priv *)self;
    struct i2c_client *client = obj->client;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;
    u16 report_lux = 0;
    APS_FUN();
    APS_LOG("epl_sensor_als_operate command = %x\n",command);

    switch (command)
    {
        case SENSOR_DELAY:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_ERR("Set delay parameter error!\n");
                err = -EINVAL;
            }
            break;


        case SENSOR_ENABLE:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_ERR("Enable sensor parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                APS_LOG("als enable = %d\n", value);

                value = *(int *)buff_in;
                if(enable_als != value)
                {
                    if(value)
                    {
                        set_bit(CMC_BIT_ALS, &obj->enable);
#if ARIMA_PATCH
                        arima_als_calibration_read(obj);
#endif
                        epl_sensor_fast_update(client);
                    }
                    else
                    {
                        clear_bit(CMC_BIT_ALS, &obj->enable);
                    }
                    epl_sensor_update_mode(client);
                }
            }
            break;


        case SENSOR_GET_DATA:
            APS_LOG("get als data !!!!!!\n");

            if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
            {
                APS_ERR("get sensor data parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                epl_sensor_read_als(client);
                report_lux = epl_sensor_get_als_value(obj, epl_sensor.als.data.channels[1]);

                sensor_data = (struct hwm_sensor_data *)buff_out;
                sensor_data->values[0] = report_lux;
                sensor_data->values[1] = epl_sensor.als.data.channels[1];
                sensor_data->value_divide = 1;
                sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
                APS_LOG("get als data->values[0] = %d\n", sensor_data->values[0]);
            }
            break;

        default:
            APS_ERR("light sensor operate function no this parameter %d!\n", command);
            err = -1;
            break;

    }

    return err;

}
#endif /*MTK_LTE end .................*/


static int als_intr_update_table(struct epl_sensor_priv *epld)
{
	int i;
	for (i = 0; i < 10; i++) {
		if ( als_lux_intr_level[i] < 0xFFFF )
		{
		    {
                als_adc_intr_level[i] = als_lux_intr_level[i] * 1000 / epl_sensor.als.factory.lux_per_count;
		    }


            if(i != 0 && als_adc_intr_level[i] <= als_adc_intr_level[i-1])
    		{
                als_adc_intr_level[i] = 0xffff;
    		}
		}
		else {
			als_adc_intr_level[i] = als_lux_intr_level[i];
		}
		APS_LOG(" [%s]:als_adc_intr_level: [%d], %ld \r\n", __func__, i, als_adc_intr_level[i]);
	}

	return 0;
}

/*----------------------------------------------------------------------------*/

static int epl_sensor_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    strcpy(info->type, EPL_DEV_NAME);
    return 0;
}


/*----------------------------------------------------------------------------*/
static int epl_sensor_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct epl_sensor_priv *obj;
#if MTK_LTE
    struct als_control_path als_ctl={0};
	struct als_data_path als_data={0};
	struct ps_control_path ps_ctl={0};
	struct ps_data_path ps_data={0};
#else
    struct hwmsen_object obj_ps, obj_als;
#endif
    int err = 0;
    APS_FUN();
printk("sherman +++++ epl_sensor_i2c_probe.1 +++++ \n");
    epl_sensor_dumpReg(client);

    if((i2c_smbus_read_byte_data(client, 0x21)) != EPL_REVNO){ //check chip
        APS_LOG("elan ALS/PS sensor is failed. \n");
        err = -1;
        goto exit;
    }

////..    client->timing = 400;
printk("sherman +++++ epl_sensor_i2c_probe.2 +++++ \n");

    if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
    {
        err = -ENOMEM;
        goto exit;
    }

    memset(obj, 0, sizeof(*obj));

    epl_sensor_obj = obj;
////...    obj->hw = get_cust_alsps_hw();
	obj->hw = hw;
////...    epl_sensor_get_addr(obj->hw, &obj->addr);

    obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
    obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);
    BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
    memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
    BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
    memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));

////...    INIT_DELAYED_WORK(&obj->eint_work, epl_sensor_eint_work);
    obj->client = client;

    mutex_init(&sensor_mutex);
    wake_lock_init(&ps_lock, WAKE_LOCK_SUSPEND, "ps wakelock");

    i2c_set_clientdata(client, obj);

    atomic_set(&obj->als_suspend, 0);
    atomic_set(&obj->ps_suspend, 0);

    obj->enable = 0;

    epl_sensor_i2c_client = client;

    //initial global variable and write to senosr
    initial_global_variable(client, obj);


    if((err = epl_sensor_init_client(client)))
    {
        goto exit_init_failed;
    }

    if((err = misc_register(&epl_sensor_device)))
    {
        APS_ERR("epl_sensor_device register failed\n");
        goto exit_misc_device_register_failed;
    }
#if MTK_LTE /*MTK_LTE start .................*/
    if((err = epl_sensor_create_attr(&epl_sensor_init_info.platform_diver_addr->driver)))
    {
        APS_ERR("create attribute err = %d\n", err);
        goto exit_create_attr_failed;
    }

    als_ctl.open_report_data= als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay  = als_set_delay;
	als_ctl.is_report_input_direct = epl_sensor.als.polling_mode==0? true:false; //false;
#ifdef CUSTOM_KERNEL_SENSORHUB
	als_ctl.is_support_batch = true;
#else
    als_ctl.is_support_batch = false;
#endif

	err = als_register_control_path(&als_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_create_attr_failed;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	err = als_register_data_path(&als_data);
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_create_attr_failed;
	}


	ps_ctl.open_report_data= ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay  = ps_set_delay;
	ps_ctl.is_report_input_direct = epl_sensor.ps.polling_mode==0? true:false; //false;
//<2015/12/02 ShermanWei,fail to be polled periodically
	ps_ctl.is_polling_mode = epl_sensor.ps.polling_mode==0? false:true;
//>2015/12/02 ShermanWei,
#ifdef CUSTOM_KERNEL_SENSORHUB
	ps_ctl.is_support_batch = true;
#else
    ps_ctl.is_support_batch = false;
#endif

	err = ps_register_control_path(&ps_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_create_attr_failed;
	}

	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_create_attr_failed;
	}

#else /*MTK_LTE */
printk("sherman +++++ epl_sensor_i2c_probe.3 +++++ \n");
    if((err = epl_sensor_create_attr(&epl_sensor_alsps_driver.driver)))
    {
        APS_ERR("create attribute err = %d\n", err);
        goto exit_create_attr_failed;
    }

    obj_ps.self = epl_sensor_obj;

    ps_hw = &obj_ps;

    if( obj->hw->polling_mode_ps)
    {
        obj_ps.polling = 1;
        APS_LOG("ps_interrupt == false\n");
    }
    else
    {
        obj_ps.polling = 0;//interrupt mode
        APS_LOG("ps_interrupt == true\n");
    }

    obj_ps.sensor_operate = epl_sensor_ps_operate;

    if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
    {
        APS_ERR("attach fail = %d\n", err);
        goto exit_create_attr_failed;
    }


    obj_als.self = epl_sensor_obj;

    als_hw = &obj_als;

    if( obj->hw->polling_mode_als)
    {
        obj_als.polling = 1;
        APS_LOG("ps_interrupt == false\n");
    }
    else
    {
        obj_als.polling = 0;//interrupt mode
        APS_LOG("ps_interrupt == true\n");
    }

    obj_als.sensor_operate = epl_sensor_als_operate;
    APS_LOG("als polling mode\n");


    if((err = hwmsen_attach(ID_LIGHT, &obj_als)))
    {
        APS_ERR("attach fail = %d\n", err);
        goto exit_create_attr_failed;
    }
#endif  /*MTK_LTE end .................*/

#if defined(CONFIG_HAS_EARLYSUSPEND)
    obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
    obj->early_drv.suspend  = epl_sensor_early_suspend,
    obj->early_drv.resume   = epl_sensor_late_resume,
    register_early_suspend(&obj->early_drv);
#endif

/*
    if(obj->hw->polling_mode_ps == 0 || obj->hw->polling_mode_als == 0)
        epl_sensor_setup_eint(client);
*/
#if MTK_LTE
    alsps_init_flag = 0;
#endif
printk("sherman +++++ epl_sensor_i2c_probe.OK +++++ \n");
    APS_LOG("%s: OK\n", __FUNCTION__);
    return 0;

exit_create_attr_failed:
    misc_deregister(&epl_sensor_device);
exit_misc_device_register_failed:
exit_init_failed:
    //i2c_detach_client(client);
//	exit_kfree:
    kfree(obj);
exit:
    epl_sensor_i2c_client = NULL;
#if MTK_LTE
    alsps_init_flag = -1;
#endif
    APS_ERR("%s: err = %d\n", __FUNCTION__, err);
    return err;



}



/*----------------------------------------------------------------------------*/
static int epl_sensor_i2c_remove(struct i2c_client *client)
{
    int err;
#if MTK_LTE
    if((err = epl_sensor_delete_attr(&epl_sensor_init_info.platform_diver_addr->driver)))
    {
        APS_ERR("epl_sensor_delete_attr fail: %d\n", err);
    }
#else
    if((err = epl_sensor_delete_attr(&epl_sensor_i2c_driver.driver)))
    {
        APS_ERR("epl_sensor_delete_attr fail: %d\n", err);
    }
#endif
    if((err = misc_deregister(&epl_sensor_device)))
    {
        APS_ERR("misc_deregister fail: %d\n", err);
    }

    epl_sensor_i2c_client = NULL;
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));

    return 0;
}


#if !MTK_LTE
/*----------------------------------------------------------------------------*/
static int epl_sensor_probe(struct platform_device *pdev)
{
////..    struct alsps_hw *hw = get_cust_alsps_hw();

APS_LOG("[%s]: \r\n", __func__);
printk("sherman +++++ epl_sensor_probe +++++\n");
    epl_sensor_power(hw, 1);

    if(i2c_add_driver(&epl_sensor_i2c_driver))
    {
        APS_ERR("add driver error\n");
		printk("sherman +++++ add driver error +++++\n");
        return -1;
    }
printk("sherman +++++ epl_sensor_probe +++++OK!\n");
    return 0;
}



/*----------------------------------------------------------------------------*/
static int epl_sensor_remove(struct platform_device *pdev)
{
////..    struct alsps_hw *hw = get_cust_alsps_hw();
    APS_FUN();
    epl_sensor_power(hw, 0);

    APS_ERR("EPL259x remove \n");
    i2c_del_driver(&epl_sensor_i2c_driver);
    return 0;
}

////..
/*
#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{ .compatible = "mediatek,als_ps", },
	{},
};
#endif
*/

/*----------------------------------------------------------------------------*/
static struct platform_driver epl_sensor_alsps_driver =
{
    .probe      = epl_sensor_probe,
    .remove     = epl_sensor_remove,
    .driver     = {
        ///.name  = "als_ps",
	.name  = "als_ps",
#ifdef CONFIG_OF
		.of_match_table = alsps_of_match,
#endif
        //.owner = THIS_MODULE,
    }
};
#endif

#if MTK_LTE


/*----------------------------------------------------------------------------*/
static int alsps_local_init(void)
{
////..    struct alsps_hw *hw = get_cust_alsps_hw();
	//printk("fwq loccal init+++\n");
	printk("sherman +++++ alsps_local_init +++++ \n");
	epl_sensor_power(hw, 1);

	if(i2c_add_driver(&epl_sensor_i2c_driver))
	{
		APS_ERR("add driver error\n");
		printk("sherman +++++ add driver error +++++ \n");
		return -1;
	}

	if(-1 == alsps_init_flag)
	{
	   return -1;
	}
	//printk("fwq loccal init---\n");
	printk("sherman +++++ alsps_local_init +++++ OK!\n");

	return 0;
}
/*----------------------------------------------------------------------------*/
static int alsps_remove(void)
{
////..    struct alsps_hw *hw = get_cust_alsps_hw();
    APS_FUN();
    epl_sensor_power(hw, 0);

    APS_ERR("epl_sensor remove \n");

    i2c_del_driver(&epl_sensor_i2c_driver);
    return 0;
}
#endif

/*----------------------------------------------------------------------------*/
static int __init epl_sensor_init(void)
{
////..    struct alsps_hw *hw = get_cust_alsps_hw();
	const char *name = "mediatek,epl25903";
    APS_FUN();
    ///APS_ERR("+++++++++\n");
    printk("sherman +++++ epl_sensor_init +++++ \n");

	hw = get_alsps_dts_func(name, hw); 
	if (!hw) {
		printk("sherman +++++ get_alsps_dts_func fail\n");
		return 0;
	}
    ///i2c_register_board_info(hw->i2c_num, &i2c_epl_sensor, 1);
#if MTK_LTE
    alsps_driver_add(&epl_sensor_init_info);
#else
    if(platform_driver_register(&epl_sensor_alsps_driver))
    {
        APS_ERR("failed to register driver");
        return -ENODEV;
    }
#endif
    printk("sherman +++++ epl_sensor_init +++++ OK!\n");
    return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit epl_sensor_exit(void)
{
    APS_FUN();
#if !MTK_LTE
    platform_driver_unregister(&epl_sensor_alsps_driver);
#endif
}
/*----------------------------------------------------------------------------*/
module_init(epl_sensor_init);
module_exit(epl_sensor_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("renato.pan@eminent-tek.com");
MODULE_DESCRIPTION("EPL259x ALPsr driver");
MODULE_LICENSE("GPL");

