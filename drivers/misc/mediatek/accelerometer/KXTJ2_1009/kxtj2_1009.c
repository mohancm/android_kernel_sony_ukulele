/* KXTJ2_1009 motion sensor driver
 *
 *
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

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
//#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

//#include <mach/mt_typedefs.h>
//#include <mach/mt_gpio.h>
//#include <mach/mt_pm_ldo.h>
//#include <accel.h>
#include "accel.h"

//#define POWER_NONE_MACRO MT65XX_POWER_NONE

//Doze
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/pm_wakeup.h>
#include <linux/input.h>
//Doze

#include <cust_acc.h>
//#include <linux/hwmsensor.h>
//#include <linux/hwmsen_dev.h>
//#include <linux/sensors_io.h>
#include <hwmsensor.h>
#include <hwmsen_dev.h>
#include <sensors_io.h>
#include "kxtj2_1009.h"
#include <hwmsen_helper.h>
/*----------------------------------------------------------------------------*/
#define I2C_DRIVERID_KXTJ2_1009 150
/*----------------------------------------------------------------------------*/
#define DEBUG 1
/*----------------------------------------------------------------------------*/
//#define CONFIG_KXTJ2_1009_LOWPASS   /*apply low pass filter on output*/       
#define SW_CALIBRATION
//#define USE_EARLY_SUSPEND
/*----------------------------------------------------------------------------*/
#define KXTJ2_1009_AXIS_X          0
#define KXTJ2_1009_AXIS_Y          1
#define KXTJ2_1009_AXIS_Z          2
#define KXTJ2_1009_AXES_NUM        3
#define KXTJ2_1009_DATA_LEN        6
#define KXTJ2_1009_DEV_NAME        "KXTJ2_1009"
/*----------------------------------------------------------------------------*/

#ifdef CONFIG_OF
static const struct of_device_id accel_of_match[] = {
	{.compatible = "mediatek,gsensor"},
	{},
};
#endif
/*----------------------------------------------------------------------------*/
static const char *shake_idev_name = "acc_shake_event";//Doze
static const struct i2c_device_id kxtj2_1009_i2c_id[] = {{KXTJ2_1009_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_kxtj2_1009={ I2C_BOARD_INFO(KXTJ2_1009_DEV_NAME, (KXTJ2_1009_I2C_SLAVE_ADDR>>1))};
//static struct i2c_board_info __initdata i2c_kxtj2_1009={ I2C_BOARD_INFO(KXTJ2_1009_DEV_NAME, 0x18)}; //RickLiu test
/*the adapter id will be available in customization*/
//static unsigned short kxtj2_1009_force[] = {0x00, KXTJ2_1009_I2C_SLAVE_ADDR, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const kxtj2_1009_forces[] = { kxtj2_1009_force, NULL };
//static struct i2c_client_address_data kxtj2_1009_addr_data = { .forces = kxtj2_1009_forces,};

/*----------------------------------------------------------------------------*/
static int kxtj2_1009_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int kxtj2_1009_i2c_remove(struct i2c_client *client);
static int kxtj2_1009_i2c_detect(struct i2c_client *client/*, int kind*/, struct i2c_board_info *info);
static int kxtj2_1009_suspend(struct i2c_client *client, pm_message_t msg);
static int kxtj2_1009_resume(struct i2c_client *client);

//< RickLiu
static int kxtj2_1009_local_init(void);
static int kxtj2_1009_remove(void);
//static int kxtj2_1009_remove(struct platform_device *pdev);
//> RickLiu

//
struct acc_hw accel_cust;
static struct acc_hw *hw = &accel_cust;
//
#define IRQ_GPIO_NUM 116

/*----------------------------------------------------------------------------*/
typedef enum {
    ADX_TRC_FILTER  = 0x01,
    ADX_TRC_RAWDATA = 0x02,
    ADX_TRC_IOCTL   = 0x04,
    ADX_TRC_CALI	= 0X08,
    ADX_TRC_INFO	= 0X10,
	ADX_TRC_LP		= 0x80,
} ADX_TRC;
/*----------------------------------------------------------------------------*/
struct scale_factor{
    u8  whole;
    u8  fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
    struct scale_factor scalefactor;
    int                 sensitivity;
};
/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/
struct data_filter {
    s16 raw[C_MAX_FIR_LENGTH][KXTJ2_1009_AXES_NUM];
    int sum[KXTJ2_1009_AXES_NUM];
    int num;
    int idx;
};
/*----------------------------------------------------------------------------*/
struct kxtj2_1009_i2c_data {
    struct i2c_client *client;
    struct acc_hw *hw;
    struct hwmsen_convert   cvt;
    struct input_dev *shake_idev;//Doze
    /*misc*/
    struct data_resolution *reso;
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
	atomic_t				filter;
    s16                     cali_sw[KXTJ2_1009_AXES_NUM+1];

    /*data*/
    s8                      offset[KXTJ2_1009_AXES_NUM+1];  /*+1: for 4-byte alignment*/
    s16                     data[KXTJ2_1009_AXES_NUM+1];

#if defined(CONFIG_KXTJ2_1009_LOWPASS)
    atomic_t                firlen;
    atomic_t                fir_en;
    struct data_filter      fir;
#endif 
    /*early suspend*/
//#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(USE_EARLY_SUSPEND)
    //struct early_suspend    early_drv;
//#endif
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver kxtj2_1009_i2c_driver = {
    .driver = {
//        .owner          = THIS_MODULE,
        .name           = KXTJ2_1009_DEV_NAME,
		#ifdef CONFIG_OF
		.of_match_table = accel_of_match,
		#endif
    },
	.probe      		= kxtj2_1009_i2c_probe,
	.remove    			= kxtj2_1009_i2c_remove,
	.detect				= kxtj2_1009_i2c_detect,
//#if !defined(CONFIG_HAS_EARLYSUSPEND) || !defined(USE_EARLY_SUSPEND)
    .suspend            = kxtj2_1009_suspend,
    .resume             = kxtj2_1009_resume,
//#endif
	.id_table = kxtj2_1009_i2c_id,
//	.address_data = &kxtj2_1009_addr_data,
};

/*----------------------------------------------------------------------------*/
static struct i2c_client *kxtj2_1009_i2c_client = NULL;
static int kxtj2_1009_init_flag = -1; //RickLiu
static struct platform_driver kxtj2_1009_gsensor_driver;

static struct kxtj2_1009_i2c_data *obj_i2c_data = NULL;
static bool sensor_power = true;
static struct GSENSOR_VECTOR3D gsensor_gain;
static char selftestRes[8]= {0}; 
static DEFINE_MUTEX(kxtj2_1009_mutex);
static bool enable_status = false;

//< RickLiu
static struct acc_init_info kxtj2_1009_init_info = {
	.name   = KXTJ2_1009_DEV_NAME,
	.init   = kxtj2_1009_local_init,
	.uninit = kxtj2_1009_remove,
};
//> RickLiu

/*----------------------------------------------------------------------------*/
#define GSE_TAG                  "[Gsensor] "
#define GSE_FUN(f)               pr_debug(GSE_TAG"%s\n", __FUNCTION__)
#define GSE_ERR(fmt, args...)    pr_err(GSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    pr_debug(GSE_TAG fmt, ##args)
/*----------------------------------------------------------------------------*/
static struct data_resolution kxtj2_1009_data_resolution[1] = {
 /* combination by {FULL_RES,RANGE}*/
    {{ 0, 9}, 1024}, // dataformat +/-2g  in 12-bit resolution;  { 3, 9} = 3.9 = (2*2*1000)/(2^12);  256 = (2^12)/(2*2)          
};
/*----------------------------------------------------------------------------*/
static struct data_resolution kxtj2_1009_offset_resolution = {{15, 6}, 64};
/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_SetPowerMode(struct i2c_client *client, bool enable);
/*--------------------KXTJ2_1009 power control function----------------------------------*/
//static void KXTJ2_1009_power(struct acc_hw *hw, unsigned int on) //Doze
static int KXTJ2_1009_power(struct acc_hw *hw, unsigned int on) //Doze
{
	static unsigned int power_on = 0;
	power_on = on;
	return -ENOENT;
	//Doze
}
/*----------------------------------------------------------------------------*/

//
/*----------------------------------------------------------------------------*/
//static int kxtj2_1009_remove(struct platform_device *pdev)
static int kxtj2_1009_remove(void)
{
    //struct acc_hw *hw = get_cust_acc_hw();
	hw = get_accel_dts_func("mediatek,kxtj2_1009", hw);

    GSE_FUN();
	printk("[Rick] kxtj2_1009_remove, power off \n");
    KXTJ2_1009_power(hw, 0);    
    i2c_del_driver(&kxtj2_1009_i2c_driver);
	kxtj2_1009_init_flag = -1; //RickLiu
    return 0;
}
//

/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_SetDataResolution(struct kxtj2_1009_i2c_data *obj)
{
	int err;
	u8  databuf[2];
	bool cur_sensor_power = sensor_power;
	printk("[Rick] KXTJ2_1009_SetDataResolution, power off \n");
	KXTJ2_1009_SetPowerMode(obj->client, false);

	if(hwmsen_read_block(obj->client, KXTJ2_1009_REG_DATA_RESOLUTION, databuf, 0x01))
	{
		GSE_ERR("kxtj2_1009 read Dataformat failt \n");
		return KXTJ2_1009_ERR_I2C;
	}

	databuf[0] &= ~KXTJ2_1009_RANGE_DATA_RESOLUTION_MASK;
	databuf[0] |= KXTJ2_1009_RANGE_DATA_RESOLUTION_MASK;//12bit
	databuf[1] = databuf[0];
	databuf[0] = KXTJ2_1009_REG_DATA_RESOLUTION;


	err = i2c_master_send(obj->client, databuf, 0x2);

	if(err <= 0)
	{
		return KXTJ2_1009_ERR_I2C;
	}

	KXTJ2_1009_SetPowerMode(obj->client, cur_sensor_power/*true*/);

	//kxtj2_1009_data_resolution[0] has been set when initialize: +/-2g  in 8-bit resolution:  15.6 mg/LSB*/   
	obj->reso = &kxtj2_1009_data_resolution[0];

	return 0;
}
/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_ReadData(struct i2c_client *client, s16 data[KXTJ2_1009_AXES_NUM])
{
	struct kxtj2_1009_i2c_data *priv = i2c_get_clientdata(client);        
	u8 addr = KXTJ2_1009_REG_DATAX0;
	u8 buf[KXTJ2_1009_DATA_LEN] = {0};
	//u8 tmpbuf=0;
	int err = 0;
	int i;

	if(NULL == client)
	{
		err = -EINVAL;
	}
	else if((err = hwmsen_read_block(client, addr, buf, 0x06)) != 0)
	{
		GSE_ERR("error: %d\n", err);
	}
	else
	{
		data[KXTJ2_1009_AXIS_X] = (s16)((buf[KXTJ2_1009_AXIS_X*2] >> 4) |
		         (buf[KXTJ2_1009_AXIS_X*2+1] << 4));
		data[KXTJ2_1009_AXIS_Y] = (s16)((buf[KXTJ2_1009_AXIS_Y*2] >> 4) |
		         (buf[KXTJ2_1009_AXIS_Y*2+1] << 4));
		data[KXTJ2_1009_AXIS_Z] = (s16)((buf[KXTJ2_1009_AXIS_Z*2] >> 4) |
		         (buf[KXTJ2_1009_AXIS_Z*2+1] << 4));

		for(i=0;i<3;i++)				
		{								//because the data is store in binary complement number formation in computer system
			if ( data[i] == 0x0800 )	//so we want to calculate actual number here
				data[i]= -2048;			//10bit resolution, 512= 2^(12-1)
			else if ( data[i] & 0x0800 )//transfor format
			{							//GSE_LOG("data 0 step %x \n",data[i]);
				data[i] -= 0x1; 		//GSE_LOG("data 1 step %x \n",data[i]);
				data[i] = ~data[i]; 	//GSE_LOG("data 2 step %x \n",data[i]);
				data[i] &= 0x07ff;		//GSE_LOG("data 3 step %x \n\n",data[i]);
				data[i] = -data[i]; 	
			}
		}	


		if(atomic_read(&priv->trace) & ADX_TRC_RAWDATA)
		{
			GSE_LOG("[%08X %08X %08X] => [%5d %5d %5d]\n", data[KXTJ2_1009_AXIS_X], data[KXTJ2_1009_AXIS_Y], data[KXTJ2_1009_AXIS_Z],
		                               data[KXTJ2_1009_AXIS_X], data[KXTJ2_1009_AXIS_Y], data[KXTJ2_1009_AXIS_Z]);
		}
#ifdef CONFIG_KXTJ2_1009_LOWPASS
		if(atomic_read(&priv->filter))
		{
			if(atomic_read(&priv->fir_en) && !atomic_read(&priv->suspend))
			{
				int idx, firlen = atomic_read(&priv->firlen);   
				if(priv->fir.num < firlen)
				{                
					priv->fir.raw[priv->fir.num][KXTJ2_1009_AXIS_X] = data[KXTJ2_1009_AXIS_X];
					priv->fir.raw[priv->fir.num][KXTJ2_1009_AXIS_Y] = data[KXTJ2_1009_AXIS_Y];
					priv->fir.raw[priv->fir.num][KXTJ2_1009_AXIS_Z] = data[KXTJ2_1009_AXIS_Z];
					priv->fir.sum[KXTJ2_1009_AXIS_X] += data[KXTJ2_1009_AXIS_X];
					priv->fir.sum[KXTJ2_1009_AXIS_Y] += data[KXTJ2_1009IK_AXIS_Y];
					priv->fir.sum[KXTJ2_1009_AXIS_Z] += data[KXTJ2_1009_AXIS_Z];
					if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", priv->fir.num,
							priv->fir.raw[priv->fir.num][KXTJ2_1009_AXIS_X], priv->fir.raw[priv->fir.num][KXTJ2_1009_AXIS_Y], priv->fir.raw[priv->fir.num][KXTJ2_1009_AXIS_Z],
							priv->fir.sum[KXTJ2_1009_AXIS_X], priv->fir.sum[KXTJ2_1009_AXIS_Y], priv->fir.sum[KXTJ2_1009_AXIS_Z]);
					}
					priv->fir.num++;
					priv->fir.idx++;
				}
				else
				{
					idx = priv->fir.idx % firlen;
					priv->fir.sum[KXTJ2_1009_AXIS_X] -= priv->fir.raw[idx][KXTJ2_1009_AXIS_X];
					priv->fir.sum[KXTJ2_1009_AXIS_Y] -= priv->fir.raw[idx][KXTJ2_1009_AXIS_Y];
					priv->fir.sum[KXTJ2_1009_AXIS_Z] -= priv->fir.raw[idx][KXTJ2_1009_AXIS_Z];
					priv->fir.raw[idx][KXTJ2_1009_AXIS_X] = data[KXTJ2_1009_AXIS_X];
					priv->fir.raw[idx][KXTJ2_1009_AXIS_Y] = data[KXTJ2_1009_AXIS_Y];
					priv->fir.raw[idx][KXTJ2_1009_AXIS_Z] = data[KXTJ2_1009_AXIS_Z];
					priv->fir.sum[KXTJ2_1009_AXIS_X] += data[KXTJ2_1009_AXIS_X];
					priv->fir.sum[KXTJ2_1009_AXIS_Y] += data[KXTJ2_1009_AXIS_Y];
					priv->fir.sum[KXTJ2_1009_AXIS_Z] += data[KXTJ2_1009_AXIS_Z];
					priv->fir.idx++;
					data[KXTJ2_1009_AXIS_X] = priv->fir.sum[KXTJ2_1009_AXIS_X]/firlen;
					data[KXTJ2_1009_AXIS_Y] = priv->fir.sum[KXTJ2_1009_AXIS_Y]/firlen;
					data[KXTJ2_1009_AXIS_Z] = priv->fir.sum[KXTJ2_1009_AXIS_Z]/firlen;
					if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n", idx,
						priv->fir.raw[idx][KXTJ2_1009_AXIS_X], priv->fir.raw[idx][KXTJ2_1009_AXIS_Y], priv->fir.raw[idx][KXTJ2_1009_AXIS_Z],
						priv->fir.sum[KXTJ2_1009_AXIS_X], priv->fir.sum[KXTJ2_1009_AXIS_Y], priv->fir.sum[KXTJ2_1009_AXIS_Z],
						data[KXTJ2_1009_AXIS_X], data[KXTJ2_1009_AXIS_Y], data[KXTJ2_1009_AXIS_Z]);
					}
				}
			}
		}	
#endif         
	}
	//test
	/*
	hwmsen_read_block(client, KXTJ2_1009_REG_INTSUR, &tmpbuf, 0x01);
	printk("[Rick] KXTJ2_1009_ReadData, KXTJ2_1009_REG_INTSUR = %x \n", tmpbuf);
	if(tmpbuf == 0x02)
	{
		printk("[Rick] G sensor SHAKE!! \n");
	}
	*/
	//test
	return err;
}
/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_ReadOffset(struct i2c_client *client, s8 ofs[KXTJ2_1009_AXES_NUM])
{    
	int err = 0;

	ofs[1]=ofs[2]=ofs[0]=0x00;

	GSE_LOG("offesx=%x, y=%x, z=%x",ofs[0],ofs[1],ofs[2]);
	
	return err;
}
/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_ResetCalibration(struct i2c_client *client)
{
	struct kxtj2_1009_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;

	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	memset(obj->offset, 0x00, sizeof(obj->offset));
	return err;
}
/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_ReadCalibration(struct i2c_client *client, int dat[KXTJ2_1009_AXES_NUM])
{
    struct kxtj2_1009_i2c_data *obj = i2c_get_clientdata(client);
    int mul;

	#ifdef SW_CALIBRATION
		mul = 0;//only SW Calibration, disable HW Calibration
	#else
	    if ((err = KXTJ2_1009_ReadOffset(client, obj->offset))) {
        GSE_ERR("read offset fail, %d\n", err);
        return err;
    	}    
    	mul = obj->reso->sensitivity/kxtj2_1009_offset_resolution.sensitivity;
	#endif

    dat[obj->cvt.map[KXTJ2_1009_AXIS_X]] = obj->cvt.sign[KXTJ2_1009_AXIS_X]*(obj->offset[KXTJ2_1009_AXIS_X]*mul + obj->cali_sw[KXTJ2_1009_AXIS_X]);
    dat[obj->cvt.map[KXTJ2_1009_AXIS_Y]] = obj->cvt.sign[KXTJ2_1009_AXIS_Y]*(obj->offset[KXTJ2_1009_AXIS_Y]*mul + obj->cali_sw[KXTJ2_1009_AXIS_Y]);
    dat[obj->cvt.map[KXTJ2_1009_AXIS_Z]] = obj->cvt.sign[KXTJ2_1009_AXIS_Z]*(obj->offset[KXTJ2_1009_AXIS_Z]*mul + obj->cali_sw[KXTJ2_1009_AXIS_Z]);                        
                                       
    return 0;
}
/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_ReadCalibrationEx(struct i2c_client *client, int act[KXTJ2_1009_AXES_NUM], int raw[KXTJ2_1009_AXES_NUM])
{  
	/*raw: the raw calibration data; act: the actual calibration data*/
	struct kxtj2_1009_i2c_data *obj = i2c_get_clientdata(client);
    #ifdef SW_CALIBRATION
    #else
	int err;
    #endif
	int mul;

 

	#ifdef SW_CALIBRATION
		mul = 0;//only SW Calibration, disable HW Calibration
	#else
		if(err = KXTJ2_1009_ReadOffset(client, obj->offset))
		{
			GSE_ERR("read offset fail, %d\n", err);
			return err;
		}   
		mul = obj->reso->sensitivity/kxtj2_1009_offset_resolution.sensitivity;
	#endif
	
	raw[KXTJ2_1009_AXIS_X] = obj->offset[KXTJ2_1009_AXIS_X]*mul + obj->cali_sw[KXTJ2_1009_AXIS_X];
	raw[KXTJ2_1009_AXIS_Y] = obj->offset[KXTJ2_1009_AXIS_Y]*mul + obj->cali_sw[KXTJ2_1009_AXIS_Y];
	raw[KXTJ2_1009_AXIS_Z] = obj->offset[KXTJ2_1009_AXIS_Z]*mul + obj->cali_sw[KXTJ2_1009_AXIS_Z];

	act[obj->cvt.map[KXTJ2_1009_AXIS_X]] = obj->cvt.sign[KXTJ2_1009_AXIS_X]*raw[KXTJ2_1009_AXIS_X];
	act[obj->cvt.map[KXTJ2_1009_AXIS_Y]] = obj->cvt.sign[KXTJ2_1009_AXIS_Y]*raw[KXTJ2_1009_AXIS_Y];
	act[obj->cvt.map[KXTJ2_1009_AXIS_Z]] = obj->cvt.sign[KXTJ2_1009_AXIS_Z]*raw[KXTJ2_1009_AXIS_Z];                        
	                       
	return 0;
}
/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_WriteCalibration(struct i2c_client *client, int dat[KXTJ2_1009_AXES_NUM])
{
	struct kxtj2_1009_i2c_data *obj = i2c_get_clientdata(client);
	int err;
	int cali[KXTJ2_1009_AXES_NUM], raw[KXTJ2_1009_AXES_NUM];
#ifdef SW_CALIBRATION
#else
    int lsb = kxtj2_1009_offset_resolution.sensitivity;
	int divisor = obj->reso->sensitivity/lsb;
#endif

	if(0 != (err = KXTJ2_1009_ReadCalibrationEx(client, cali, raw)))	/*offset will be updated in obj->offset*/
	{ 
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}

	GSE_LOG("OLDOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n", 
		raw[KXTJ2_1009_AXIS_X], raw[KXTJ2_1009_AXIS_Y], raw[KXTJ2_1009_AXIS_Z],
		obj->offset[KXTJ2_1009_AXIS_X], obj->offset[KXTJ2_1009_AXIS_Y], obj->offset[KXTJ2_1009_AXIS_Z],
		obj->cali_sw[KXTJ2_1009_AXIS_X], obj->cali_sw[KXTJ2_1009_AXIS_Y], obj->cali_sw[KXTJ2_1009_AXIS_Z]);

	/*calculate the real offset expected by caller*/
	cali[KXTJ2_1009_AXIS_X] += dat[KXTJ2_1009_AXIS_X];
	cali[KXTJ2_1009_AXIS_Y] += dat[KXTJ2_1009_AXIS_Y];
	cali[KXTJ2_1009_AXIS_Z] += dat[KXTJ2_1009_AXIS_Z];

	GSE_LOG("UPDATE: (%+3d %+3d %+3d)\n", 
		dat[KXTJ2_1009_AXIS_X], dat[KXTJ2_1009_AXIS_Y], dat[KXTJ2_1009_AXIS_Z]);

#ifdef SW_CALIBRATION
	obj->cali_sw[KXTJ2_1009_AXIS_X] = obj->cvt.sign[KXTJ2_1009_AXIS_X]*(cali[obj->cvt.map[KXTJ2_1009_AXIS_X]]);
	obj->cali_sw[KXTJ2_1009_AXIS_Y] = obj->cvt.sign[KXTJ2_1009_AXIS_Y]*(cali[obj->cvt.map[KXTJ2_1009_AXIS_Y]]);
	obj->cali_sw[KXTJ2_1009_AXIS_Z] = obj->cvt.sign[KXTJ2_1009_AXIS_Z]*(cali[obj->cvt.map[KXTJ2_1009_AXIS_Z]]);	
#else
	obj->offset[KXTJ2_1009_AXIS_X] = (s8)(obj->cvt.sign[KXTJ2_1009_AXIS_X]*(cali[obj->cvt.map[KXTJ2_1009_AXIS_X]])/(divisor));
	obj->offset[KXTJ2_1009_AXIS_Y] = (s8)(obj->cvt.sign[KXTJ2_1009_AXIS_Y]*(cali[obj->cvt.map[KXTJ2_1009_AXIS_Y]])/(divisor));
	obj->offset[KXTJ2_1009_AXIS_Z] = (s8)(obj->cvt.sign[KXTJ2_1009_AXIS_Z]*(cali[obj->cvt.map[KXTJ2_1009_AXIS_Z]])/(divisor));

	/*convert software calibration using standard calibration*/
	obj->cali_sw[KXTJ2_1009_AXIS_X] = obj->cvt.sign[KXTJ2_1009_AXIS_X]*(cali[obj->cvt.map[KXTJ2_1009_AXIS_X]])%(divisor);
	obj->cali_sw[KXTJ2_1009_AXIS_Y] = obj->cvt.sign[KXTJ2_1009_AXIS_Y]*(cali[obj->cvt.map[KXTJ2_1009_AXIS_Y]])%(divisor);
	obj->cali_sw[KXTJ2_1009_AXIS_Z] = obj->cvt.sign[KXTJ2_1009_AXIS_Z]*(cali[obj->cvt.map[KXTJ2_1009_AXIS_Z]])%(divisor);

	GSE_LOG("NEWOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n", 
		obj->offset[KXTJ2_1009_AXIS_X]*divisor + obj->cali_sw[KXTJ2_1009_AXIS_X], 
		obj->offset[KXTJ2_1009_AXIS_Y]*divisor + obj->cali_sw[KXTJ2_1009_AXIS_Y], 
		obj->offset[KXTJ2_1009_AXIS_Z]*divisor + obj->cali_sw[KXTJ2_1009_AXIS_Z], 
		obj->offset[KXTJ2_1009_AXIS_X], obj->offset[KXTJ2_1009_AXIS_Y], obj->offset[KXTJ2_1009_AXIS_Z],
		obj->cali_sw[KXTJ2_1009_AXIS_X], obj->cali_sw[KXTJ2_1009_AXIS_Y], obj->cali_sw[KXTJ2_1009_AXIS_Z]);

	if(err = hwmsen_write_block(obj->client, KXTJ2_1009_REG_OFSX, obj->offset, KXTJ2_1009_AXES_NUM))
	{
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}
#endif

	return err;
}
/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_CheckDeviceID(struct i2c_client *client)
{
	u8 databuf[10];    
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);    
	databuf[0] = KXTJ2_1009_REG_DEVID;   

	res = i2c_master_send(client, databuf, 0x1);
	if(res <= 0)
	{
		goto exit_KXTJ2_1009_CheckDeviceID;
	}
	
	udelay(500);

	databuf[0] = 0x0;        
	res = i2c_master_recv(client, databuf, 0x01);
	if(res <= 0)
	{
		goto exit_KXTJ2_1009_CheckDeviceID;
	}
	

	if(false)
	{
		GSE_ERR("KXTJ2_1009_CheckDeviceID 0x%x failt!\n ", databuf[0]);
		return KXTJ2_1009_ERR_IDENTIFICATION;
	}
	else
	{
		GSE_LOG("KXTJ2_1009_CheckDeviceID 0x%x pass!\n ", databuf[0]);
	}
	
	exit_KXTJ2_1009_CheckDeviceID:
	if (res <= 0)
	{
		return KXTJ2_1009_ERR_I2C;
	}
	
	return KXTJ2_1009_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static void KXTJ2_1009_GetPowerMode(struct i2c_client *client)
{
	u8 databuf[2];
	//int res = 0;
	u8 addr = KXTJ2_1009_REG_POWER_CTL;

	if (hwmsen_read_block(client, addr, databuf, 0x01))	{
		GSE_ERR("read power ctl register err!\n");
	} else{
		GSE_LOG("reg[0x%x] = 0x%x, bit[7] should be 0. (%c)\n",
			KXTJ2_1009_REG_POWER_CTL, databuf[0], (databuf[0]&0x80) == 0x00?'O':'X');
	}
}
/*----------------------------------------------------------------------------*/
//static int KXTJ2_1009_SetPowerMode(struct i2c_client *client, bool enable)
/*static int KXTJ2_1009_SetPowerMode_locked(struct i2c_client *client, bool enable)//Doze
{
	u8 databuf[2];    
	int res = 0;
	u8 addr = KXTJ2_1009_REG_POWER_CTL;
	
	if(enable == sensor_power)
	{
		GSE_LOG("Sensor power status is newest!\n");
		return KXTJ2_1009_SUCCESS;
	}

	if(hwmsen_read_block(client, addr, databuf, 0x01))
	{
		GSE_ERR("read power ctl register err!\n");
		return KXTJ2_1009_ERR_I2C;
	}

	
	if(enable == true)
	{
		databuf[0] |= KXTJ2_1009_MEASURE_MODE;
	}
	else
	{
		databuf[0] &= ~KXTJ2_1009_MEASURE_MODE;
	}
	databuf[1] = databuf[0];
	databuf[0] = KXTJ2_1009_REG_POWER_CTL;
	

	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return KXTJ2_1009_ERR_I2C;
	}


	GSE_LOG("KXTJ2_1009_SetPowerMode %d!\n ",enable);


	sensor_power = enable;

	mdelay(5);
	
	return KXTJ2_1009_SUCCESS;
}*/

//Doze
#define KXTJ2_1009_REG_TILT   0x06
#define KXTJ2_1009_REG_INTSU  0x09
#define KXTJ2_1009_REG_SR     0x0b
#define KXTJ2_1009_REG_RANGE  0x16
#define KXTJ2_1009_REG_INTMAP 0x18

#define ALL_AXIS_SHAKE (6 << 5)
#define INT_PUSH_PULL  (1 << 6)
#define MODE_ACTIVE    (1 << 0)
#define S_RATE_100HZ    (2 << 0)//Doze
#define SHAKE_THRESH   (0 << 0)
#define SHAKE_THRESH_M (3 << 0)
#define SHAKE_IRQ_INT2 (6 << 5)

struct tilt_config {
	u8 intsu;
//	u8 mode;//Doze
	u8 sr;
	u8 range;
	u8 intmap;
	u8 tilt;
};

//Doze
#define RETRY 10

static int read_retry(struct i2c_client *client, u8 reg, u8 *data)
{
	int rc, i;

	for (i = 0; i < RETRY; i++) {
		rc = hwmsen_read_byte(client, reg, data);
		if (!rc)
			break;
	}
	if (rc)
		dev_err(&client->dev, "%s: reg %02x, rc = %d", __func__,
				reg, rc);
	return rc;
}

static int write_retry(struct i2c_client *client, u8 reg, u8 data)
{
	int rc, i;

	for (i = 0; i < RETRY; i++) {
		rc = hwmsen_write_byte(client, reg, data);
		if (!rc)
			break;
	}
	if (rc)
		dev_err(&client->dev, "%s: reg %02x, val %02x, rc = %d",
				__func__, reg, data, rc);
	return rc;
}
//Doze

static int read_tilt_config(struct kxtj2_1009_i2c_data *obj, struct tilt_config *cfg)
{
	int rc;

	memset(cfg, 0, sizeof(*cfg));

	rc = read_retry(obj->client, KXTJ2_1009_REG_INTSU, &cfg->intsu);
	if (rc)
		goto err;
	
	rc = read_retry(obj->client, KXTJ2_1009_REG_SR, &cfg->sr);
	if (rc)
		goto err;
	
	rc = read_retry(obj->client, KXTJ2_1009_REG_RANGE, &cfg->range);
	if (rc)
		goto err;
	
	rc = read_retry(obj->client, KXTJ2_1009_REG_INTMAP, &cfg->intmap);
	if (rc)
		goto err;
	
	rc = read_retry(obj->client, KXTJ2_1009_REG_TILT, &cfg->tilt);
err:
	if (rc)
		dev_err(&obj->client->dev, "%s = %d\n", __func__, rc);
	return rc;
}

static int kxtj2_1009_setup_shake_detection(struct kxtj2_1009_i2c_data *obj, bool enable)
{
	int rc;
	u8 x;

	struct tilt_config cfg;
	printk("[Rick] Doze kxtj2_1009_setup_shake_detection start! \n");
	dev_dbg(&obj->client->dev, "%s, enable %d\n", __func__, enable);
	rc = read_tilt_config(obj, &cfg);
	if (rc)
	{
		printk("[Rick] Doze kxtj2_1009_setup_shake_detection, read_tilt_config FAIL \n");
		goto err;
	}

	//doze
	if (enable) {
		printk("[Rick] Doze kxtj2_1009_setup_shake_detection, enable True \n");
		cfg.intmap |= SHAKE_IRQ_INT2;
		cfg.intsu |= ALL_AXIS_SHAKE;
		x = cfg.sr & 7;
		if (x != S_RATE_100HZ)
			cfg.sr = (cfg.sr & ~7) | S_RATE_100HZ;
		cfg.range = (cfg.range & ~SHAKE_THRESH_M) | SHAKE_THRESH;
	} else {
		printk("[Rick] Doze kxtj2_1009_setup_shake_detection, enable False \n");
		cfg.intmap &= ~SHAKE_IRQ_INT2;
		cfg.intsu &= ~ALL_AXIS_SHAKE;
	}
	//Doze
err:
	if (rc)
		dev_err(&obj->client->dev, "%s = %d\n", __func__, rc);
	return rc;
}

enum acc_client {
	REQ_ACC_DATA = 1 << 0,
	REQ_SHAKE_SENSOR = 1 << 1,
};

struct kxtj2_1009_op_mode {
	int req;
	int saved_req;
	struct mutex lock;
};

static struct kxtj2_1009_op_mode kxtj2_1009_op_mode;

static int vote_op_mode_locked(struct i2c_client *client, bool enable, enum acc_client request)
{
	int req;
	struct kxtj2_1009_i2c_data *obj = i2c_get_clientdata(client);
	int rc = 0;//doze
	req = kxtj2_1009_op_mode.req;
	printk("[Rick] Doze vote_op_mode_locked start! \n");
	if (enable)
		kxtj2_1009_op_mode.req |= request;
	else
		kxtj2_1009_op_mode.req &= ~request;

	//dev_info(&client->dev, "%s: request 0x%02x, was 0x%02x\n", __func__,kxtj2_1009_op_mode.req, req);

	printk("[Rick] Doze vote_op_mode_locked, kxtj2_1009_op_mode.req = 0x%02x \n ", kxtj2_1009_op_mode.req);
	printk("[Rick] Doze vote_op_mode_locked, req = 0x%02x \n ", req);
	
	if (req != kxtj2_1009_op_mode.req) 
	{
		printk("[Rick] Doze vote_op_mode_locked, req != kxtj2_1009_op_mode.req \n");
		disable_irq(gpio_to_irq(IRQ_GPIO_NUM));
		enable = !!kxtj2_1009_op_mode.req;
	
		//Doze
		if (!(req & REQ_SHAKE_SENSOR) && (kxtj2_1009_op_mode.req & REQ_SHAKE_SENSOR)) 
		{
			printk("[Rick] Doze vote_op_mode_locked , setup_shake_detection, true \n");
			kxtj2_1009_setup_shake_detection(obj, true);
		}
		else if ((req & REQ_SHAKE_SENSOR) && !(kxtj2_1009_op_mode.req & REQ_SHAKE_SENSOR)) 
		{
			printk("[Rick] Doze vote_op_mode_locked , setup_shake_detection, false \n");
			kxtj2_1009_setup_shake_detection(obj, false);
		}
		//Doze
		/*if (enable) {
			u8 mode = MODE_ACTIVE | ((req & REQ_SHAKE_SENSOR)? INT_PUSH_PULL : 0);
			rc = write_retry(obj->client, KXTJ2_1009_REG_MODE, mode);
			if (rc) {
				dev_err(&client->dev,"%s: unable set mode 0x%02x\n",__func__, mode);
				goto err;
			}
			dev_dbg(&obj->client->dev, "%s mode 0x%02x\n",__func__, mode);
			//rc = STK831X_SetVD(obj->client);//need to finish this function
		}
		else
		{
			printk("[Rick] Doze vote_op_mode_locked, not enable \n ");
		}*/
		sensor_power = enable;
		enable_irq(gpio_to_irq(IRQ_GPIO_NUM));
		printk("[Rick] Doze vote_op_mode_locked, After enable_irq \n ");
	}
	else
	{
		printk("[Rick] Doze vote_op_mode_locked, req == kxtj2_1009_op_mode.req \n");
	}
//err:
	return rc;
}

static int vote_op_mode(struct i2c_client *client, bool enable,	enum acc_client request)
{
	int rc;
	//struct kxtj2_1009_i2c_data *obj = obj_i2c_data;
	//u8 data8;
	
	mutex_lock(&kxtj2_1009_op_mode.lock);
	printk("[Rick] Doze vote_op_mode, enable = %d \n", enable);
	rc = vote_op_mode_locked(client, enable, request);
	//test
	/*rc = write_retry(obj->client, 0x1B, 0x00);
	rc = write_retry(obj->client, 0x1B, 0x42);
	rc = write_retry(obj->client, 0x1D, 0x06);
	rc = write_retry(obj->client, 0x1F, 0x7F);
	rc = write_retry(obj->client, 0x29, 0x05);
	rc = write_retry(obj->client, 0x6A, 0x02);
	rc = write_retry(obj->client, 0x1E, 0x30);
	rc = write_retry(obj->client, 0x1B, 0xC2);
	rc = read_retry(obj->client, 0x1A, &data8);//Release (INT_REL) register
	printk("[Rick] wufe_fun, done ! \n");*/
	//test
	mutex_unlock(&kxtj2_1009_op_mode.lock);
	return rc;
}

static int op_mode_suspend(struct i2c_client *client, bool suspend)
{
	int rc;
	//struct kxtj2_1009_i2c_data *obj = obj_i2c_data;
	//u8 data8;
	
	mutex_lock(&kxtj2_1009_op_mode.lock);
	printk("[Rick] Doze op_mode_suspend Start \n");
	printk("[Rick] Doze op_mode_suspend suspend = %d \n", suspend);
	if (suspend) {
		printk("[Rick] Doze op_mode_suspend, suspend = true \n");
		kxtj2_1009_op_mode.saved_req = kxtj2_1009_op_mode.req;
		rc = vote_op_mode_locked(client, false, REQ_ACC_DATA);
	} else {
		printk("[Rick] Doze op_mode_suspend, suspend = false \n");
		rc = vote_op_mode_locked(client, true, kxtj2_1009_op_mode.saved_req);
	}
	printk("[Rick] Doze op_mode_suspend End \n");
	
	//test
	/*
	rc = write_retry(obj->client, 0x1B, 0x01);
	rc = write_retry(obj->client, 0x1B, 0x42);
	rc = write_retry(obj->client, 0x1D, 0x06);
	rc = write_retry(obj->client, 0x1F, 0x3F);
	rc = write_retry(obj->client, 0x29, 0x05);
	rc = write_retry(obj->client, 0x6A, 0x02);
	rc = write_retry(obj->client, 0x1E, 0x30);
	rc = write_retry(obj->client, 0x1B, 0xC2);
	rc = read_retry(obj->client, 0x1A, &data8);
	printk("[Rick] wufe_fun, done ! \n");
	*/
	//test
	
	mutex_unlock(&kxtj2_1009_op_mode.lock);
	return rc;
}

static int KXTJ2_1009_SetPowerMode(struct i2c_client *client, bool enable)
{
	return vote_op_mode(client, enable, REQ_ACC_DATA);
}
//Doze

/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
	struct kxtj2_1009_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[10];    
	int res = 0;
	bool cur_sensor_power = sensor_power;

	memset(databuf, 0, sizeof(u8)*10);  
	printk("[Rick] KXTJ2_1009_SetDataFormat, power off \n");
	KXTJ2_1009_SetPowerMode(client, false);

	if(hwmsen_read_block(client, KXTJ2_1009_REG_DATA_FORMAT, databuf, 0x01))
	{
		GSE_ERR("kxtj2_1009 read Dataformat failt \n");
		return KXTJ2_1009_ERR_I2C;
	}

	databuf[0] &= ~KXTJ2_1009_RANGE_MASK;
	databuf[0] |= dataformat;
	databuf[1] = databuf[0];
	databuf[0] = KXTJ2_1009_REG_DATA_FORMAT;


	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return KXTJ2_1009_ERR_I2C;
	}

	KXTJ2_1009_SetPowerMode(client, cur_sensor_power/*true*/);
	
	GSE_LOG("KXTJ2_1009_SetDataFormat OK! \n");
	

	return KXTJ2_1009_SetDataResolution(obj);    
}
/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_SetBWRate(struct i2c_client *client, u8 bwrate)
{
	u8 databuf[10];    
	int res = 0;
	bool cur_sensor_power = sensor_power;

	memset(databuf, 0, sizeof(u8)*10);    

	printk("[Rick] KXTJ2_1009_SetBWRate, power off \n");
	KXTJ2_1009_SetPowerMode(client, false);

	if(hwmsen_read_block(client, KXTJ2_1009_REG_BW_RATE, databuf, 0x01))
	{
		GSE_ERR("kxtj2_1009 read rate failt \n");
		return KXTJ2_1009_ERR_I2C;
	}

	databuf[0] &= 0xf0;
	databuf[0] |= bwrate;
	databuf[1] = databuf[0];
	databuf[0] = KXTJ2_1009_REG_BW_RATE;


	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return KXTJ2_1009_ERR_I2C;
	}

	
	KXTJ2_1009_SetPowerMode(client, cur_sensor_power/*true*/);
	GSE_LOG("KXTJ2_1009_SetBWRate OK! \n");
	
	return KXTJ2_1009_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_SetIntEnable(struct i2c_client *client, u8 intenable)
{
	u8 databuf[10];    
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);    
	databuf[0] = KXTJ2_1009_REG_INT_ENABLE;    
	databuf[1] = 0x00;

	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return KXTJ2_1009_ERR_I2C;
	}
	
	return KXTJ2_1009_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
static int kxtj2_1009_init_client(struct i2c_client *client, int reset_cali)
{
	struct kxtj2_1009_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;

	res = KXTJ2_1009_CheckDeviceID(client); 
	if(res != KXTJ2_1009_SUCCESS)
	{
		return res;
	}	

	res = KXTJ2_1009_SetPowerMode(client, enable_status/*false*/);
	if(res != KXTJ2_1009_SUCCESS)
	{
		return res;
	}
	

	res = KXTJ2_1009_SetBWRate(client, KXTJ2_1009_BW_100HZ);
	if(res != KXTJ2_1009_SUCCESS ) //0x2C->BW=100Hz
	{
		return res;
	}

	res = KXTJ2_1009_SetDataFormat(client, KXTJ2_1009_RANGE_2G);
	if(res != KXTJ2_1009_SUCCESS) //0x2C->BW=100Hz
	{
		return res;
	}

	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;


	res = KXTJ2_1009_SetIntEnable(client, 0x00);        
	if(res != KXTJ2_1009_SUCCESS)//0x2E->0x80
	{
		return res;
	}

	if(0 != reset_cali)
	{ 
		/*reset calibration only in power on*/
		res = KXTJ2_1009_ResetCalibration(client);
		if(res != KXTJ2_1009_SUCCESS)
		{
			return res;
		}
	}
	GSE_LOG("kxtj2_1009_init_client OK!\n");
#ifdef CONFIG_KXTJ2_1009_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));  
#endif

	return KXTJ2_1009_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	u8 databuf[10];    

	memset(databuf, 0, sizeof(u8)*10);

	if((NULL == buf)||(bufsize<=30))
	{
		return -1;
	}
	
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	sprintf(buf, "KXTJ2_1009 Chip");
	return 0;
}
/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_ReadSensorData(struct i2c_client *client, char *buf, int bufsize)
{
	struct kxtj2_1009_i2c_data *obj = (struct kxtj2_1009_i2c_data*)i2c_get_clientdata(client);
	u8 databuf[20];
	int acc[KXTJ2_1009_AXES_NUM];
	int res = 0;
	memset(databuf, 0, sizeof(u8)*10);

	if(NULL == buf)
	{
		return -1;
	}
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	if (atomic_read(&obj->suspend))
	{
		return 0;
	}
	mutex_lock(&kxtj2_1009_op_mode.lock);//Doze
	if(sensor_power == false)
	{
		//res = KXTJ2_1009_SetPowerMode(client, true);//Doze
		res = vote_op_mode_locked(client, true, REQ_ACC_DATA);//Doze
		if(res)
		{
			GSE_ERR("Power on kxtj2_1009 error %d!\n", res);
		}
	}
	//Doze
	//if(0 != (res = KXTJ2_1009_ReadData(client, obj->data)))
	res = KXTJ2_1009_ReadData(client, obj->data);
	mutex_unlock(&kxtj2_1009_op_mode.lock);
	if(res)//Doze
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	}
	else
	{
		//GSE_LOG("raw data x=%d, y=%d, z=%d \n",obj->data[KXTJ2_1009_AXIS_X],obj->data[KXTJ2_1009_AXIS_Y],obj->data[KXTJ2_1009_AXIS_Z]);
		obj->data[KXTJ2_1009_AXIS_X] += obj->cali_sw[KXTJ2_1009_AXIS_X];
		obj->data[KXTJ2_1009_AXIS_Y] += obj->cali_sw[KXTJ2_1009_AXIS_Y];
		obj->data[KXTJ2_1009_AXIS_Z] += obj->cali_sw[KXTJ2_1009_AXIS_Z];
		
		//GSE_LOG("cali_sw x=%d, y=%d, z=%d \n",obj->cali_sw[KXTJ2_1009_AXIS_X],obj->cali_sw[KXTJ2_1009_AXIS_Y],obj->cali_sw[KXTJ2_1009_AXIS_Z]);
		
		/*remap coordinate*/
		acc[obj->cvt.map[KXTJ2_1009_AXIS_X]] = obj->cvt.sign[KXTJ2_1009_AXIS_X]*obj->data[KXTJ2_1009_AXIS_X];
		acc[obj->cvt.map[KXTJ2_1009_AXIS_Y]] = obj->cvt.sign[KXTJ2_1009_AXIS_Y]*obj->data[KXTJ2_1009_AXIS_Y];
		acc[obj->cvt.map[KXTJ2_1009_AXIS_Z]] = obj->cvt.sign[KXTJ2_1009_AXIS_Z]*obj->data[KXTJ2_1009_AXIS_Z];
		//GSE_LOG("cvt x=%d, y=%d, z=%d \n",obj->cvt.sign[KXTJ2_1009_AXIS_X],obj->cvt.sign[KXTJ2_1009_AXIS_Y],obj->cvt.sign[KXTJ2_1009_AXIS_Z]);


		//GSE_LOG("Mapped gsensor data: %d, %d, %d!\n", acc[KXTJ2_1009_AXIS_X], acc[KXTJ2_1009_AXIS_Y], acc[KXTJ2_1009_AXIS_Z]);

		//Out put the mg
		//GSE_LOG("mg acc=%d, GRAVITY=%d, sensityvity=%d \n",acc[KXTJ2_1009_AXIS_X],GRAVITY_EARTH_1000,obj->reso->sensitivity);
		acc[KXTJ2_1009_AXIS_X] = acc[KXTJ2_1009_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[KXTJ2_1009_AXIS_Y] = acc[KXTJ2_1009_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[KXTJ2_1009_AXIS_Z] = acc[KXTJ2_1009_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;		
		
	

		sprintf(buf, "%04x %04x %04x", acc[KXTJ2_1009_AXIS_X], acc[KXTJ2_1009_AXIS_Y], acc[KXTJ2_1009_AXIS_Z]);
		printk("kxtj2 acc x, y, z =%6d %6d %6d", acc[KXTJ2_1009_AXIS_X], acc[KXTJ2_1009_AXIS_Y], acc[KXTJ2_1009_AXIS_Z]);
		printk("kxtj2 buf x, y, z =%6d %6d %6d", buf[KXTJ2_1009_AXIS_X], buf[KXTJ2_1009_AXIS_Y], buf[KXTJ2_1009_AXIS_Z]);
		if(atomic_read(&obj->trace) & ADX_TRC_IOCTL)
		{
			GSE_LOG("gsensor data: %s!\n", buf);
		}
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_ReadRawData(struct i2c_client *client, char *buf)
{
	struct kxtj2_1009_i2c_data *obj = (struct kxtj2_1009_i2c_data*)i2c_get_clientdata(client);
	int res = 0;

	mutex_lock(&kxtj2_1009_op_mode.lock);//Doze
	//if (!buf || !client)//Doze
	if(sensor_power == false)//Doze
 	{
		res = vote_op_mode_locked(client, true, REQ_ACC_DATA);//Doze
		return EINVAL;
	}
	//Doze
	//if(0 != (res = KXTJ2_1009_ReadData(client, obj->data)))
	res = KXTJ2_1009_ReadData(client, obj->data);
	mutex_unlock(&kxtj2_1009_op_mode.lock);
	if(res)
	//Doze
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return EIO;
	}
	else
	{
		sprintf(buf, "KXTJ2_1009_ReadRawData %04x %04x %04x", obj->data[KXTJ2_1009_AXIS_X], 
			obj->data[KXTJ2_1009_AXIS_Y], obj->data[KXTJ2_1009_AXIS_Z]);
	
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int KXTJ2_1009_InitSelfTest(struct i2c_client *client)
{
	int res = 0;
	u8  data,result;
	
	res = hwmsen_read_byte(client, KXTJ2_1009_REG_CTL_REG3, &data);
	if(res != KXTJ2_1009_SUCCESS)
	{
		return res;
	}
//enable selftest bit
	res = hwmsen_write_byte(client, KXTJ2_1009_REG_CTL_REG3,  KXTJ2_1009_SELF_TEST|data);
	if(res != KXTJ2_1009_SUCCESS) //0x2C->BW=100Hz
	{
		return res;
	}
//step 1
	res = hwmsen_read_byte(client, KXTJ2_1009_DCST_RESP, &result);
	if(res != KXTJ2_1009_SUCCESS)
	{
		return res;
	}
	GSE_LOG("step1: result = %x",result);
	if(result != 0xaa)
		return -EINVAL;

//step 2
	res = hwmsen_write_byte(client, KXTJ2_1009_REG_CTL_REG3,  KXTJ2_1009_SELF_TEST|data);
	if(res != KXTJ2_1009_SUCCESS) //0x2C->BW=100Hz
	{
		return res;
	}
//step 3
	res = hwmsen_read_byte(client, KXTJ2_1009_DCST_RESP, &result);
	if(res != KXTJ2_1009_SUCCESS)
	{
		return res;
	}
	GSE_LOG("step3: result = %x",result);
	if(result != 0xAA)
		return -EINVAL;
		
//step 4
	res = hwmsen_read_byte(client, KXTJ2_1009_DCST_RESP, &result);
	if(res != KXTJ2_1009_SUCCESS)
	{
		return res;
	}
	GSE_LOG("step4: result = %x",result);
	if(result != 0x55)
		return -EINVAL;
	else
		return KXTJ2_1009_SUCCESS;
}
/*----------------------------------------------------------------------------*/
#if 0
static int KXTJ2_1009_JudgeTestResult(struct i2c_client *client, s32 prv[KXTJ2_1009_AXES_NUM], s32 nxt[KXTJ2_1009_AXES_NUM])
{

    int res=0;
	u8 test_result=0;
    if(0 != (res = hwmsen_read_byte(client, 0x0c, &test_result)))
        return res;

	GSE_LOG("test_result = %x \n",test_result);
    if ( test_result != 0xaa ) 
	{
        GSE_ERR("KXTJ2_1009_JudgeTestResult failt\n");
        res = -EINVAL;
    }
    return res;
}
#endif
/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = kxtj2_1009_i2c_client;
	char strbuf[KXTJ2_1009_BUFSIZE];
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	
	KXTJ2_1009_ReadChipInfo(client, strbuf, KXTJ2_1009_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);        
}
#if 0
static ssize_t gsensor_init(struct device_driver *ddri, char *buf, size_t count)
	{
		struct i2c_client *client = kxtj2_1009_i2c_client;
		char strbuf[KXTJ2_1009_BUFSIZE];
		
		if(NULL == client)
		{
			GSE_ERR("i2c client is null!!\n");
			return 0;
		}
		kxtj2_1009_init_client(client, 1);
		return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);			
	}
#endif
/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = kxtj2_1009_i2c_client;
	char strbuf[KXTJ2_1009_BUFSIZE];
	
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	KXTJ2_1009_ReadSensorData(client, strbuf, KXTJ2_1009_BUFSIZE);
	//KXTJ2_1009_ReadRawData(client, strbuf);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);            
}
#if 0
static ssize_t show_sensorrawdata_value(struct device_driver *ddri, char *buf, size_t count)
	{
		struct i2c_client *client = kxtj2_1009_i2c_client;
		char strbuf[KXTJ2_1009_BUFSIZE];
		
		if(NULL == client)
		{
			GSE_ERR("i2c client is null!!\n");
			return 0;
		}
		//KXTJ2_1009_ReadSensorData(client, strbuf, KXTJ2_1009_BUFSIZE);
		KXTJ2_1009_ReadRawData(client, strbuf);
		return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);			
	}
#endif
/*----------------------------------------------------------------------------*/
static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = kxtj2_1009_i2c_client;
	struct kxtj2_1009_i2c_data *obj;
	int err, len = 0, mul;
	int tmp[KXTJ2_1009_AXES_NUM];

	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);



	if(0 != (err = KXTJ2_1009_ReadOffset(client, obj->offset)))
	{
		return -EINVAL;
	}
	else if(0 != (err = KXTJ2_1009_ReadCalibration(client, tmp)))
	{
		return -EINVAL;
	}
	else
	{    
		mul = obj->reso->sensitivity/kxtj2_1009_offset_resolution.sensitivity;
		len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,                        
			obj->offset[KXTJ2_1009_AXIS_X], obj->offset[KXTJ2_1009_AXIS_Y], obj->offset[KXTJ2_1009_AXIS_Z],
			obj->offset[KXTJ2_1009_AXIS_X], obj->offset[KXTJ2_1009_AXIS_Y], obj->offset[KXTJ2_1009_AXIS_Z]);
		len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1, 
			obj->cali_sw[KXTJ2_1009_AXIS_X], obj->cali_sw[KXTJ2_1009_AXIS_Y], obj->cali_sw[KXTJ2_1009_AXIS_Z]);

		len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n", 
			obj->offset[KXTJ2_1009_AXIS_X]*mul + obj->cali_sw[KXTJ2_1009_AXIS_X],
			obj->offset[KXTJ2_1009_AXIS_Y]*mul + obj->cali_sw[KXTJ2_1009_AXIS_Y],
			obj->offset[KXTJ2_1009_AXIS_Z]*mul + obj->cali_sw[KXTJ2_1009_AXIS_Z],
			tmp[KXTJ2_1009_AXIS_X], tmp[KXTJ2_1009_AXIS_Y], tmp[KXTJ2_1009_AXIS_Z]);
		
		return len;
    }
}
/*----------------------------------------------------------------------------*/
static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = kxtj2_1009_i2c_client;  
	int err, x, y, z;
	int dat[KXTJ2_1009_AXES_NUM];

	if(!strncmp(buf, "rst", 3))
	{
		if(0 != (err = KXTJ2_1009_ResetCalibration(client)))
		{
			GSE_ERR("reset offset err = %d\n", err);
		}	
	}
	else if(3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z))
	{
		dat[KXTJ2_1009_AXIS_X] = x;
		dat[KXTJ2_1009_AXIS_Y] = y;
		dat[KXTJ2_1009_AXIS_Z] = z;
		if(0 != (err = KXTJ2_1009_WriteCalibration(client, dat)))
		{
			GSE_ERR("write calibration err = %d\n", err);
		}		
	}
	else
	{
		GSE_ERR("invalid format\n");
	}
	
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_self_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = kxtj2_1009_i2c_client;

	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	
    return snprintf(buf, 8, "%s\n", selftestRes);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_self_value(struct device_driver *ddri, const char *buf, size_t count)
{   /*write anything to this register will trigger the process*/
	struct item{
	s16 raw[KXTJ2_1009_AXES_NUM];
	};
	
	struct i2c_client *client = kxtj2_1009_i2c_client;  
	int res, num;
	struct item *prv = NULL, *nxt = NULL;
	u8 data;

	if(1 != sscanf(buf, "%d", &num))
	{
		GSE_ERR("parse number fail\n");
		return count;
	}
	else if(num == 0)
	{
		GSE_ERR("invalid data count\n");
		return count;
	}

	prv = kzalloc(sizeof(*prv) * num, GFP_KERNEL);
	nxt = kzalloc(sizeof(*nxt) * num, GFP_KERNEL);
	if (!prv || !nxt)
	{
		goto exit;
	}


	GSE_LOG("NORMAL:\n");
	KXTJ2_1009_SetPowerMode(client,true); 

	/*initial setting for self test*/
	if(!KXTJ2_1009_InitSelfTest(client))
	{
		GSE_LOG("SELFTEST : PASS\n");
		strcpy(selftestRes,"y");
	}	
	else
	{
		GSE_LOG("SELFTEST : FAIL\n");		
		strcpy(selftestRes,"n");
	}

	res = hwmsen_read_byte(client, KXTJ2_1009_REG_CTL_REG3, &data);
	if(res != KXTJ2_1009_SUCCESS)
	{
		return res;
	}

	res = hwmsen_write_byte(client, KXTJ2_1009_REG_CTL_REG3,  ~KXTJ2_1009_SELF_TEST&data);
	if(res != KXTJ2_1009_SUCCESS) //0x2C->BW=100Hz
	{
		return res;
	}
	
	exit:
	/*restore the setting*/    
	kxtj2_1009_init_client(client, 0);
	kfree(prv);
	kfree(nxt);
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_selftest_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = kxtj2_1009_i2c_client;
	struct kxtj2_1009_i2c_data *obj;

	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->selftest));
}
/*----------------------------------------------------------------------------*/
static ssize_t store_selftest_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct kxtj2_1009_i2c_data *obj = obj_i2c_data;
	int tmp;

	if(NULL == obj)
	{
		GSE_ERR("i2c data obj is null!!\n");
		return 0;
	}
	
	
	if(1 == sscanf(buf, "%d", &tmp))
	{        
		if(atomic_read(&obj->selftest) && !tmp)
		{
			/*enable -> disable*/
			kxtj2_1009_init_client(obj->client, 0);
		}
		else if(!atomic_read(&obj->selftest) && tmp)
		{
			/*disable -> enable*/
			KXTJ2_1009_InitSelfTest(obj->client);            
		}
		
		GSE_LOG("selftest: %d => %d\n", atomic_read(&obj->selftest), tmp);
		atomic_set(&obj->selftest, tmp); 
	}
	else
	{ 
		//GSE_ERR("invalid content: '%s', length = %d\n", buf, count);   
	}
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_KXTJ2_1009_LOWPASS
	struct i2c_client *client = kxtj2_1009_i2c_client;
	struct kxtj2_1009_i2c_data *obj = i2c_get_clientdata(client);
	if(atomic_read(&obj->firlen))
	{
		int idx, len = atomic_read(&obj->firlen);
		GSE_LOG("len = %2d, idx = %2d\n", obj->fir.num, obj->fir.idx);

		for(idx = 0; idx < len; idx++)
		{
			GSE_LOG("[%5d %5d %5d]\n", obj->fir.raw[idx][KXTJ2_1009_AXIS_X], obj->fir.raw[idx][KXTJ2_1009_AXIS_Y], obj->fir.raw[idx][KXTJ2_1009_AXIS_Z]);
		}
		
		GSE_LOG("sum = [%5d %5d %5d]\n", obj->fir.sum[KXTJ2_1009_AXIS_X], obj->fir.sum[KXTJ2_1009_AXIS_Y], obj->fir.sum[KXTJ2_1009_AXIS_Z]);
		GSE_LOG("avg = [%5d %5d %5d]\n", obj->fir.sum[KXTJ2_1009_AXIS_X]/len, obj->fir.sum[KXTJ2_1009_AXIS_Y]/len, obj->fir.sum[KXTJ2_1009_AXIS_Z]/len);
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->firlen));
#else
	return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}
/*----------------------------------------------------------------------------*/
static ssize_t store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
#ifdef CONFIG_KXTJ2_1009_LOWPASS
	struct i2c_client *client = kxtj2_1009_i2c_client;  
	struct kxtj2_1009_i2c_data *obj = i2c_get_clientdata(client);
	int firlen;

	if(1 != sscanf(buf, "%d", &firlen))
	{
		GSE_ERR("invallid format\n");
	}
	else if(firlen > C_MAX_FIR_LENGTH)
	{
		GSE_ERR("exceeds maximum filter length\n");
	}
	else
	{ 
		atomic_set(&obj->firlen, firlen);
		if(NULL == firlen)
		{
			atomic_set(&obj->fir_en, 0);
		}
		else
		{
			memset(&obj->fir, 0x00, sizeof(obj->fir));
			atomic_set(&obj->fir_en, 1);
		}
	}
#endif    
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct kxtj2_1009_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct kxtj2_1009_i2c_data *obj = obj_i2c_data;
	int trace;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&obj->trace, trace);
	}	
	else
	{
		//GSE_ERR("invalid content: '%s', length = %d\n", buf, count);
	}
	
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;    
	struct kxtj2_1009_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}	
	
	if(obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n", 
	            obj->hw->i2c_num, obj->hw->direction, obj->hw->power_id, obj->hw->power_vol);   
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t show_power_status_value(struct device_driver *ddri, char *buf)
{
	u8 databuf[2];    
	u8 addr = KXTJ2_1009_REG_POWER_CTL;
	if(hwmsen_read_block(kxtj2_1009_i2c_client, addr, databuf, 0x01))
	{
		GSE_ERR("read power ctl register err!\n");
		return KXTJ2_1009_ERR_I2C;
	}
    
	if(sensor_power)
		GSE_LOG("G sensor is in work mode, sensor_power = %d\n", sensor_power);
	else
		GSE_LOG("G sensor is in standby mode, sensor_power = %d\n", sensor_power);

	return snprintf(buf, PAGE_SIZE, "%x\n", databuf[0]);
}

//<RickLiu 20151111 show_resetcali
static ssize_t show_resetcali(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = kxtj2_1009_i2c_client;
	int err =0;
	err = KXTJ2_1009_ResetCalibration(client);

	return err; //if return 0 mean is sucessful.
}
//>RickLiu 20151111 show_resetcali

//<RickLiu 20151110 show_do_calibration
#define G_Sensor_Data_Count 20

static ssize_t show_do_calibration(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = kxtj2_1009_i2c_client;
	s32 sum[KXTJ2_1009_AXES_NUM];
	char strbuf[KXTJ2_1009_BUFSIZE];
	int i = 0;
	int count = 0;
	int retcode = 0;
	int data[KXTJ2_1009_AXES_NUM];
	ssize_t len = 0;
	int err=0;
	//int status; 
	struct kxtj2_1009_i2c_data *obj = (struct kxtj2_1009_i2c_data*)i2c_get_clientdata(client);
	GSE_FUN();
		
	KXTJ2_1009_ResetCalibration(client);
	
	if(err!=0) 
	{
		printk("show_do_calibration KXTJ2_1009_ResetCalibration fail:%d\n", err);
		return 0;
	}
	sum[KXTJ2_1009_AXIS_X] = 0;
	sum[KXTJ2_1009_AXIS_Y] = 0;
	sum[KXTJ2_1009_AXIS_Z] = 0;
	
	for(count=0; count<G_Sensor_Data_Count; count++)
	{
		data[KXTJ2_1009_AXIS_X] = 0;
		data[KXTJ2_1009_AXIS_Y] = 0;
		data[KXTJ2_1009_AXIS_Z] = 0;
				
		retcode = KXTJ2_1009_ReadSensorData(client, strbuf, KXTJ2_1009_BUFSIZE);
		
		if(retcode !=0) //tammy
		{
			printk("show_do_calibration retcode:%d\n", retcode);
			return 0;		
		}
		
		sscanf(strbuf, "%x %x %x", &data[KXTJ2_1009_AXIS_X], &data[KXTJ2_1009_AXIS_Y], &data[KXTJ2_1009_AXIS_Z]);				
		printk("1 show_do_calibration %d %d %d\n", data[KXTJ2_1009_AXIS_X], data[KXTJ2_1009_AXIS_Y], data[KXTJ2_1009_AXIS_Z]);
		
		sum[KXTJ2_1009_AXIS_X] += data[KXTJ2_1009_AXIS_X];
		sum[KXTJ2_1009_AXIS_Y] += data[KXTJ2_1009_AXIS_Y];
		sum[KXTJ2_1009_AXIS_Z] += data[KXTJ2_1009_AXIS_Z];

		mdelay(50);
	}
	
	//len += snprintf(buf+len, PAGE_SIZE-len, "A %d %d %d  \n", sum[KXTJ2_1009_AXIS_X], sum[KXTJ2_1009_AXIS_Y], sum[KXTJ2_1009_AXIS_Z]);
	
	sum[KXTJ2_1009_AXIS_X] = sum[KXTJ2_1009_AXIS_X] / G_Sensor_Data_Count;
	sum[KXTJ2_1009_AXIS_Y] = sum[KXTJ2_1009_AXIS_Y] / G_Sensor_Data_Count;
	sum[KXTJ2_1009_AXIS_Z] = sum[KXTJ2_1009_AXIS_Z] / G_Sensor_Data_Count;
	
	sum[KXTJ2_1009_AXIS_Z] = sum[KXTJ2_1009_AXIS_Z] - GRAVITY_EARTH_1000;
		
	for(i=0; i<KXTJ2_1009_AXES_NUM; i++)//remap coorindate
	{
		sum[i] = -(sum[i]);
	}

	//len += snprintf(buf+len, PAGE_SIZE-len, "B %d %d %d  \n", sum[KXTJ2_1009_AXIS_X], sum[KXTJ2_1009_AXIS_Y], sum[KXTJ2_1009_AXIS_Z]);
	
	sum[KXTJ2_1009_AXIS_X] = sum[KXTJ2_1009_AXIS_X] * obj->reso->sensitivity / GRAVITY_EARTH_1000;
	sum[KXTJ2_1009_AXIS_Y] = sum[KXTJ2_1009_AXIS_Y] * obj->reso->sensitivity / GRAVITY_EARTH_1000;
	sum[KXTJ2_1009_AXIS_Z] = sum[KXTJ2_1009_AXIS_Z] * obj->reso->sensitivity / GRAVITY_EARTH_1000;
	
	printk("2 show_do_calibration %d %d %d , sens=%d  \n", 
	sum[KXTJ2_1009_AXIS_X], sum[KXTJ2_1009_AXIS_Y], sum[KXTJ2_1009_AXIS_Z], obj->reso->sensitivity);
	
	KXTJ2_1009_WriteCalibration(client, sum);

	//len += snprintf(buf+len, PAGE_SIZE-len, "C %d %d %d  \n", sum[KXTJ2_1009_AXIS_X], sum[KXTJ2_1009_AXIS_Y], sum[KXTJ2_1009_AXIS_Z]);

	//<20151130 Misc TA 60100 for g-sensor calibration data
	len += snprintf(buf+len, PAGE_SIZE-len, "%d %d %d", sum[KXTJ2_1009_AXIS_X], sum[KXTJ2_1009_AXIS_Y], sum[KXTJ2_1009_AXIS_Z]);
	//>20151130 Misc TA 60100 for g-sensor calibration data
	
	return len;
}
//>RickLiu 20151110 show_do_calibration

//Doze
static int print_tilt_config(struct tilt_config *cfg, char *buf, size_t size)
{
	return scnprintf(buf, size,"INTSU %02x SR %02x RANGE %02x INTMAP %02x TILT %02x\n",cfg->intsu, cfg->sr, cfg->range, cfg->intmap, cfg->tilt);
}

static ssize_t show_tilt_config(struct device_driver *ddri, char *buf)
{
	int rc;
	struct tilt_config cfg;
	struct kxtj2_1009_i2c_data *obj = obj_i2c_data;

	rc = read_tilt_config(obj, &cfg);
	if (rc)
		goto err;
	rc = print_tilt_config(&cfg, buf, PAGE_SIZE);
err:
	return rc;
}

static ssize_t store_tilt_config(struct device_driver *ddri, const char *buf, size_t count)
{
	struct kxtj2_1009_i2c_data *obj = obj_i2c_data;
	int rc;
	
	dev_dbg(&obj->client->dev, "%s: enable %c\n", __func__, *buf);//Doze
	rc = vote_op_mode(obj->client, *buf == '1', REQ_SHAKE_SENSOR);
	//dev_info(&obj->client->dev, "%s: enable %c\n", __func__, *buf);//Doze
	return rc ? rc : count;
}


static ssize_t show_pm_relax(struct device_driver *ddri, char *buf)
{
	struct kxtj2_1009_i2c_data *obj = obj_i2c_data;
	u8 x;
	int rc = hwmsen_read_byte(obj->client, KXTJ2_1009_REG_TILT, &x);

	pm_relax(&obj->client->dev);
	if (rc)
		goto err;
	dev_info(&obj->client->dev, "%s 0x%02x\n", __func__, x);
	rc = scnprintf(buf, PAGE_SIZE, "%x", x);
err:
	return rc;
}


static ssize_t store_reg(struct device_driver *ddri, const char *buf, size_t count)
{
	struct kxtj2_1009_i2c_data *obj = obj_i2c_data;
	unsigned int reg, val;
	u8 x;
	int rc = sscanf(buf, "%x,%x", &val, &reg);

	if (rc == 2) {
		dev_info(&obj->client->dev, "writing %02x to %02x\n", val, reg);
		rc = hwmsen_write_byte(obj->client, reg, val);
		if (rc)
			goto err;
		rc = hwmsen_read_byte(obj->client, reg, &x);
		dev_info(&obj->client->dev, "read back %02x from %02x\n",
				x, reg);
	} else {
		dev_err(&obj->client->dev, "%s: EINVAL", __func__);
		rc = -EINVAL;
	}
err:
	return rc ? rc : count;
}

static u8 i2c_dev_reg =0 ;

static ssize_t wufe_fun(struct device_driver *ddri, char *buf)
{
	u8 data8;
	struct kxtj2_1009_i2c_data *obj = obj_i2c_data;
	i2c_dev_reg = simple_strtoul(buf, NULL, 16);
	
	write_retry(obj->client, 0x1B, 0x01);
	write_retry(obj->client, 0x1B, 0x42);
	write_retry(obj->client, 0x1D, 0x06);
	write_retry(obj->client, 0x1F, 0x3F);
	write_retry(obj->client, 0x29, 0x05);
	write_retry(obj->client, 0x6A, 0x02);
	write_retry(obj->client, 0x1E, 0x30);
	write_retry(obj->client, 0x1B, 0xC2);
	read_retry(obj->client, 0x1A, &data8);
	printk("[Rick] wufe_fun, done ! \n");
	return 0;
}

//Doze

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo,   S_IWUSR | S_IRUGO, show_chipinfo_value,      NULL);
static DRIVER_ATTR(sensordata, S_IWUSR | S_IRUGO, show_sensordata_value,    NULL);
static DRIVER_ATTR(cali,       S_IWUSR | S_IRUGO, show_cali_value,          store_cali_value);
static DRIVER_ATTR(selftest, S_IWUSR | S_IRUGO, show_self_value,  store_self_value);
static DRIVER_ATTR(self,   S_IWUSR | S_IRUGO, show_selftest_value,      store_selftest_value);
static DRIVER_ATTR(firlen,     S_IWUSR | S_IRUGO, show_firlen_value,        store_firlen_value);
static DRIVER_ATTR(trace,      S_IWUSR | S_IRUGO, show_trace_value,         store_trace_value);
static DRIVER_ATTR(status,               S_IRUGO, show_status_value,        NULL);
static DRIVER_ATTR(powerstatus,               S_IRUGO, show_power_status_value,        NULL);
//Doze
static DRIVER_ATTR(tilt, S_IWUSR | S_IRUGO,		show_tilt_config, store_tilt_config);
static DRIVER_ATTR(pmrelax, S_IRUGO, show_pm_relax, NULL);
static DRIVER_ATTR(reg, S_IWUSR, NULL, store_reg);
//Doze
static DRIVER_ATTR(wufe, S_IWUSR | S_IRUGO, wufe_fun, NULL);

/*----------------------------------------------------------------------------*/


static ssize_t show_register(struct device_driver *pdri, char *buf)
{
	GSE_LOG("i2c_dev_reg is 0x%2x \n", i2c_dev_reg);

	return 0;
}

static ssize_t store_register(struct device_driver *ddri, const char *buf, size_t count)
{
	i2c_dev_reg = simple_strtoul(buf, NULL, 16);
	GSE_LOG("set i2c_dev_reg = 0x%2x \n", i2c_dev_reg);

	return 0;
}
static ssize_t store_register_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct kxtj2_1009_i2c_data *obj = obj_i2c_data;
	u8 databuf[2];  
	unsigned long input_value;
	int res;
	
	memset(databuf, 0, sizeof(u8)*2);    

	input_value = simple_strtoul(buf, NULL, 16);
	GSE_LOG("input_value = 0x%2x \n", (unsigned int)input_value);

	if(NULL == obj)
	{
		GSE_ERR("i2c data obj is null!!\n");
		return 0;
	}

	databuf[0] = i2c_dev_reg;
	databuf[1] = input_value;
	GSE_LOG("databuf[0]=0x%2x  databuf[1]=0x%2x \n", databuf[0],databuf[1]);

	res = i2c_master_send(obj->client, databuf, 0x2);

	if(res <= 0)
	{
		return KXTJ2_1009_ERR_I2C;
	}
	return 0;
	
}

static ssize_t show_register_value(struct device_driver *ddri, char *buf)
{
		struct kxtj2_1009_i2c_data *obj = obj_i2c_data;
		u8 databuf[1];	
		
		memset(databuf, 0, sizeof(u8)*1);	 
	
		if(NULL == obj)
		{
			GSE_ERR("i2c data obj is null!!\n");
			return 0;
		}
		
		if(hwmsen_read_block(obj->client, i2c_dev_reg, databuf, 0x01))
		{
			GSE_ERR("read power ctl register err!\n");
			return KXTJ2_1009_ERR_I2C;
		}

		GSE_LOG("i2c_dev_reg=0x%2x  data=0x%2x \n", i2c_dev_reg,databuf[0]);
	
		return 0;
		
}


static DRIVER_ATTR(i2c,      S_IWUSR | S_IRUGO, show_register_value,         store_register_value);
static DRIVER_ATTR(register,      S_IWUSR | S_IRUGO, show_register,         store_register);
//<RickLiu 20151110 show_do_calibration
static DRIVER_ATTR(docali,     S_IWUSR | S_IRUGO, show_do_calibration,        NULL);
//>RickLiu 20151110 show_do_calibration

//<RickLiu 20151111 show_resetcali
static DRIVER_ATTR(resetcali,     S_IWUSR | S_IRUGO, show_resetcali,NULL);
//>RickLiu 20151111 show_resetcali

/*----------------------------------------------------------------------------*/
static struct driver_attribute *kxtj2_1009_attr_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_cali,         /*show calibration data*/
	&driver_attr_self,         /*self test demo*/
	&driver_attr_selftest,     /*self control: 0: disable, 1: enable*/
	&driver_attr_firlen,       /*filter length: 0: disable, others: enable*/
	&driver_attr_trace,        /*trace log*/
	&driver_attr_status,
	&driver_attr_powerstatus,
	&driver_attr_register,
	&driver_attr_i2c,
	&driver_attr_resetcali,//<RickLiu 20151111 resetcali
	&driver_attr_docali,   //<RickLiu 20151110 show_do_calibration
	&driver_attr_tilt,//Doze
	&driver_attr_pmrelax,//Doze
	&driver_attr_reg,//Doze
	&driver_attr_wufe,//WUFE
};
/*----------------------------------------------------------------------------*/
static int kxtj2_1009_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(kxtj2_1009_attr_list)/sizeof(kxtj2_1009_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if(0 != (err = driver_create_file(driver, kxtj2_1009_attr_list[idx])))
		{            
			GSE_ERR("driver_create_file (%s) = %d\n", kxtj2_1009_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int kxtj2_1009_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(kxtj2_1009_attr_list)/sizeof(kxtj2_1009_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}
	

	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, kxtj2_1009_attr_list[idx]);
	}
	

	return err;
}

/*----------------------------------------------------------------------------*/
int gsensor_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value, sample_delay;	
	struct kxtj2_1009_i2c_data *priv = (struct kxtj2_1009_i2c_data*)self;
	struct hwm_sensor_data* gsensor_data;
	char buff[KXTJ2_1009_BUFSIZE];

	struct kxtj2_1009_i2c_data *obj = obj_i2c_data;
	u8 data8;
	//GSE_FUN(f);
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value <= 5)
				{
					sample_delay = KXTJ2_1009_BW_200HZ;
				}
				else if(value <= 10)
				{
					sample_delay = KXTJ2_1009_BW_100HZ;
				}
				else
				{
					sample_delay = KXTJ2_1009_BW_50HZ;
				}

				mutex_lock(&kxtj2_1009_mutex);
				err = KXTJ2_1009_SetBWRate(priv->client, sample_delay);
				mutex_unlock(&kxtj2_1009_mutex);
				if(err != KXTJ2_1009_SUCCESS ) //0x2C->BW=100Hz
				{
					GSE_ERR("Set delay parameter error!\n");
				}

				if(value >= 50)
				{
					atomic_set(&priv->filter, 0);
				}
				else
				{	
				#if defined(CONFIG_KXTJ2_1009_LOWPASS)
					priv->fir.num = 0;
					priv->fir.idx = 0;
					priv->fir.sum[KXTJ2_1009_AXIS_X] = 0;
					priv->fir.sum[KXTJ2_1009_AXIS_Y] = 0;
					priv->fir.sum[KXTJ2_1009_AXIS_Z] = 0;
					atomic_set(&priv->filter, 1);
				#endif
				}
			}
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				mutex_lock(&kxtj2_1009_mutex);
				GSE_LOG("[Rick] Gsensor KXTJ2_1009_SetPowerMode, SENSOR_ENABLE Start !\n");
				if(((value == 0) && (sensor_power == false)) ||((value == 1) && (sensor_power == true)))
				{
					enable_status = sensor_power;
					GSE_LOG("[Rick] Gsensor device have updated!\n");
				}
				else
				{
					enable_status = !sensor_power;
					GSE_LOG("[Rick] Gsensor KXTJ2_1009_SetPowerMode, enable_status = %d\n",enable_status);
					if (atomic_read(&priv->suspend) == 0)
					{
						if(enable_status!=0)
						{
							err = KXTJ2_1009_SetPowerMode( priv->client, enable_status);
							GSE_LOG("[Rick] Gsensor not in suspend KXTJ2_1009_SetPowerMode!, enable_status = %d\n",enable_status);
						}
						else{
							GSE_LOG("[Rick] Gsensor not in suspend KXTJ2_1009_SetPowerMode, enable_status = %d\n",enable_status);
							//test
							
							//0x00, the purpose to initialize accelerometer in stand-by mode
							write_retry(obj->client, 0x1B, 0x00);
							//0x42, keep the accelerometer in stand-by mode
							//to set the performance mode of the KXTJ2 to high current 12 bit resolution
							write_retry(obj->client, 0x1B, 0x42);
							//0x06, set the Output Data Rate of the Wake Up function (motion detection) (OWUF) to 50 Hz
							write_retry(obj->client, 0x1D, 0x06);
							//0x7F, define the direction of detected motion for all positive and negative directions
							write_retry(obj->client, 0x1F, 0x7F);
							//0x05, Interrupt Wake-Up Timer (WAKEUP_TIMER) to set the time motion must be present
							//before a wake-up interrupt is set to 0.1 second
							write_retry(obj->client, 0x29, 0x05);
							write_retry(obj->client, 0x6A, 0x02);
							//0x30, output the physical interrupt of the previously defined Wake-Up detect function.
							//This value will create an active high and latched interrupt
							write_retry(obj->client, 0x1E, 0x30);
							//0xC2, set the accelerometer in operating mode with the previously defined settings
							write_retry(obj->client, 0x1B, 0xC2);
							read_retry(obj->client, 0x1A, &data8);
							
							//test
						}
					}
					else
					{
						GSE_LOG("[Rick] Gsensor in suspend and can not enable or disable!enable_status = %d\n",enable_status);
					}
				}
				GSE_LOG("[Rick] Gsensor KXTJ2_1009_SetPowerMode, SENSOR_ENABLE End !\n");
				mutex_unlock(&kxtj2_1009_mutex);
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
			{
				GSE_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				gsensor_data = (struct hwm_sensor_data *)buff_out;
				mutex_lock(&kxtj2_1009_mutex);
				KXTJ2_1009_ReadSensorData(priv->client, buff, KXTJ2_1009_BUFSIZE);
				mutex_unlock(&kxtj2_1009_mutex);
				sscanf(buff, "%x %x %x", &gsensor_data->values[0], 
					&gsensor_data->values[1], &gsensor_data->values[2]);				
				gsensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;				
				gsensor_data->value_divide = 1000;
			}
			break;
		default:
			GSE_ERR("gsensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int kxtj2_1009_open(struct inode *inode, struct file *file)
{
	file->private_data = kxtj2_1009_i2c_client;

	if(file->private_data == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int kxtj2_1009_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
//static int kxtj2_1009_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
//       unsigned long arg)
static long kxtj2_1009_unlocked_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct kxtj2_1009_i2c_data *obj = (struct kxtj2_1009_i2c_data*)i2c_get_clientdata(client);	
	char strbuf[KXTJ2_1009_BUFSIZE];
	void __user *data;
	struct SENSOR_DATA sensor_data;
	long err = 0;
	int cali[3];

	//GSE_FUN(f);
	if(_IOC_DIR(cmd) & _IOC_READ)
	{
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
	{
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if(err)
	{
		GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case GSENSOR_IOCTL_INIT:
			kxtj2_1009_init_client(client, 0);			
			break;

		case GSENSOR_IOCTL_READ_CHIPINFO:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			KXTJ2_1009_ReadChipInfo(client, strbuf, KXTJ2_1009_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;
			}				 
			break;	  

		case GSENSOR_IOCTL_READ_SENSORDATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			KXTJ2_1009_SetPowerMode(obj->client, true);
			KXTJ2_1009_ReadSensorData(client, strbuf, KXTJ2_1009_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}				 
			break;

		case GSENSOR_IOCTL_READ_GAIN:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &gsensor_gain, sizeof(struct GSENSOR_VECTOR3D)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

		case GSENSOR_IOCTL_READ_RAW_DATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			KXTJ2_1009_ReadRawData(client, strbuf);
			if(copy_to_user(data, &strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}
			break;	  

		case GSENSOR_IOCTL_SET_CALI:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if(copy_from_user(&sensor_data, data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;	  
			}
			if(atomic_read(&obj->suspend))
			{
				GSE_ERR("Perform calibration in suspend state!!\n");
				err = -EINVAL;
			}
			else
			{
				cali[KXTJ2_1009_AXIS_X] = sensor_data.x * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[KXTJ2_1009_AXIS_Y] = sensor_data.y * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[KXTJ2_1009_AXIS_Z] = sensor_data.z * obj->reso->sensitivity / GRAVITY_EARTH_1000;			  
				err = KXTJ2_1009_WriteCalibration(client, cali);			 
			}
			break;

		case GSENSOR_IOCTL_CLR_CALI:
			err = KXTJ2_1009_ResetCalibration(client);
			break;

		case GSENSOR_IOCTL_GET_CALI:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if(0 != (err = KXTJ2_1009_ReadCalibration(client, cali)))
			{
				break;
			}
			
			sensor_data.x = cali[KXTJ2_1009_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.y = cali[KXTJ2_1009_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.z = cali[KXTJ2_1009_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			if(copy_to_user(data, &sensor_data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;
			}		
			break;
		

		default:
			GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
			
	}

	return err;
}
 

/*----------------------------------------------------------------------------*/
static struct file_operations kxtj2_1009_fops = {
	.owner = THIS_MODULE,
	.open = kxtj2_1009_open,
	.release = kxtj2_1009_release,
	.unlocked_ioctl = kxtj2_1009_unlocked_ioctl,
	//.ioctl = kxtj2_1009_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice kxtj2_1009_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &kxtj2_1009_fops,
};
/*----------------------------------------------------------------------------*/
//#if !defined(CONFIG_HAS_EARLYSUSPEND) || !defined(USE_EARLY_SUSPEND)
/*----------------------------------------------------------------------------*/
static int kxtj2_1009_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct kxtj2_1009_i2c_data *obj = i2c_get_clientdata(client);    
	int err = 0;
	GSE_FUN();    
	printk("[Rick] Doze kxtj2_1009_suspend Start \n");
	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(obj == NULL)
		{
			GSE_ERR("null pointer!!\n");
			return -EINVAL;
		}
        mutex_lock(&kxtj2_1009_mutex);
		err = op_mode_suspend(client, true);//Doze
		atomic_set(&obj->suspend, 1);
		if(0 != (err = KXTJ2_1009_SetPowerMode(obj->client, false)))
		{
			GSE_ERR("write power control fail!!\n");
			printk("[Rick] kxtj2_1009_suspend, write power control fail!! \n");
			mutex_unlock(&kxtj2_1009_mutex);
			return -1;
		}
        mutex_unlock(&kxtj2_1009_mutex);
		printk("[Rick] kxtj2_1009_suspend, power off \n");
		KXTJ2_1009_power(obj->hw, 0);
	}
	if (atomic_read(&obj->trace) & ADX_TRC_LP) {
		KXTJ2_1009_GetPowerMode(obj->client);
	}
	printk("[Rick] Doze kxtj2_1009_suspend End \n");
	return err;
}
/*----------------------------------------------------------------------------*/
static int kxtj2_1009_resume(struct i2c_client *client)
{
	struct kxtj2_1009_i2c_data *obj = i2c_get_clientdata(client);        
	int err;
	GSE_FUN();
	printk("[Rick] Doze kxtj2_1009_resume + \n");
	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}

	if (atomic_read(&obj->trace) & ADX_TRC_LP) {
		KXTJ2_1009_GetPowerMode(client);
	}
    mutex_lock(&kxtj2_1009_mutex);
	//Doze
	if (!KXTJ2_1009_power(obj->hw, 1)) {
		if( (err = kxtj2_1009_init_client(client, 0)) )
		{
			GSE_ERR("initialize client fail!!\n");
			return err;
		}
	}
	//Doze
	op_mode_suspend(client, false);
	atomic_set(&obj->suspend, 0);
    mutex_unlock(&kxtj2_1009_mutex);
	printk("[Rick] Doze kxtj2_1009_resume - \n");
	return 0;
}
/*----------------------------------------------------------------------------*/
//#else //!defined(CONFIG_HAS_EARLYSUSPEND) || !defined(USE_EARLY_SUSPEND)
/*----------------------------------------------------------------------------*/
/*static void kxtj2_1009_early_suspend(struct early_suspend *h) 
{
	struct kxtj2_1009_i2c_data *obj = container_of(h, struct kxtj2_1009_i2c_data, early_drv);   
	int err;
	GSE_FUN();    

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}
	mutex_lock(&kxtj2_1009_mutex);
	atomic_set(&obj->suspend, 1); 
	if(err = KXTJ2_1009_SetPowerMode(obj->client, false))
	{
		GSE_ERR("write power control fail!!\n");
		mutex_unlock(&kxtj2_1009_mutex);
		return;
	}
	mutex_unlock(&kxtj2_1009_mutex);

	//sensor_power = false;
	
	KXTJ2_1009_power(obj->hw, 0);
}*/
/*----------------------------------------------------------------------------*/
//#endif //!defined(CONFIG_HAS_EARLYSUSPEND) || !defined(USE_EARLY_SUSPEND)
/*----------------------------------------------------------------------------*/
static int kxtj2_1009_i2c_detect(struct i2c_client *client/*, int kind*/, struct i2c_board_info *info) 
{    
	strcpy(info->type, KXTJ2_1009_DEV_NAME);
	return 0;
}

//Doze
static irqreturn_t kxtj2_1009_irq_handler(int irq, void *handle)
{
	u8 data8;
	struct kxtj2_1009_i2c_data *obj = obj_i2c_data;
	//struct kxtj2_1009_i2c_data *obj = (struct kxtj2_1009_i2c_data *)handle;//will crash system
	
	printk("[Rick] Doze kxtj2_1009_irq_handler Start \n");

	//pm_stay_awake(&obj->client->dev);
	//pm_wakeup_event(&obj->client->dev,1000);
	input_report_rel(obj->shake_idev, REL_MISC, 1);
	input_sync(obj->shake_idev);
	msleep(100);
	/*
	input_event(obj->shake_idev, EV_KEY, KEY_WAKEUP, 1);//press
	input_sync(obj->shake_idev);
	msleep(100);
	input_event(obj->shake_idev, EV_KEY, KEY_WAKEUP, 0);//release
	input_sync(obj->shake_idev);
	msleep(100);
	*/
	
	read_retry(obj->client, 0x1A, &data8);//Release (INT_REL) register
	printk("[Rick] G sensor SHAKE!! \n");
	printk("[Rick] Doze kxtj2_1009_irq_handler End\n");
	return IRQ_HANDLED;
}


static int kxtj2_1009_setup_irq(struct kxtj2_1009_i2c_data *obj)
{
	int rc;
	int irq;
	struct device *dev = &obj->client->dev;
	printk("[Rick] Doze kxtj2_1009_setup_irq \n");
	gpio_direction_input(IRQ_GPIO_NUM);
	irq = gpio_to_irq(IRQ_GPIO_NUM);
	rc = request_threaded_irq(irq, NULL, kxtj2_1009_irq_handler,IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING|IRQF_ONESHOT,dev_name(dev),NULL);
	
	if (rc) {
		printk("[Rick] Doze could not request irq %d\n", irq);
		dev_err(dev, "could not request irq %d\n", irq);
	} else {
		enable_irq_wake(irq);
		device_init_wakeup(dev, 1);
		printk("[Rick] Doze requested irq %d\n", irq);
		dev_dbg(dev, "requested irq %d\n", irq);
	}
	printk("[Rick] Doze kxtj2_1009_setup_irq End ! \n");
	return rc;
}
//Doze

/*----------------------------------------------------------------------------*/
static int kxtj2_1009_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct kxtj2_1009_i2c_data *obj;
	struct hwmsen_object sobj;
	int err = 0;
	//u8 data8;
	GSE_FUN();

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	printk("[Rick] kxtj2_1009_i2c_probe !");
	memset(obj, 0, sizeof(struct kxtj2_1009_i2c_data));

	//obj->hw = get_cust_acc_hw();
	obj->hw = get_accel_dts_func("mediatek,kxtj2_1009", hw);
	
	if(0 != (err = hwmsen_get_convert(obj->hw->direction, &obj->cvt)))
	{
		GSE_ERR("invalid direction: %d\n", obj->hw->direction);
		goto exit;
	}

	obj_i2c_data = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client,obj);
	
	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	
#ifdef CONFIG_KXTJ2_1009_LOWPASS
	if(obj->hw->firlen > C_MAX_FIR_LENGTH)
	{
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	}	
	else
	{
		atomic_set(&obj->firlen, obj->hw->firlen);
	}
	
	if(atomic_read(&obj->firlen) > 0)
	{
		atomic_set(&obj->fir_en, 1);
	}
	
#endif

	kxtj2_1009_i2c_client = new_client;	

	//Doze
	obj->shake_idev = devm_input_allocate_device(&client->dev);
	if (!obj->shake_idev) {
		dev_err(&client->dev, "unable to allocate input device\n");
		err = -ENODEV;
		printk("[Rick] Doze init fail !");
		goto exit_init_failed;
	}
	obj->shake_idev->name = shake_idev_name;
	input_set_capability(obj->shake_idev, EV_REL, REL_MISC);
	input_set_capability(obj->shake_idev, EV_KEY, KEY_WAKEUP);
	input_set_drvdata(obj->shake_idev, obj);
	err = input_register_device(obj->shake_idev);
	if (err < 0) {
		dev_err(&client->dev, "unable to register input device: %d\n", err);
		printk("[Rick] Doze init fail !");
		goto exit_init_failed;
	}

	mutex_init(&kxtj2_1009_op_mode.lock);
	kxtj2_1009_op_mode.req = 0;
	kxtj2_1009_setup_irq(obj);
	printk("[Rick] Doze kxtj2_1009_setup_irq done !");
	//Doze

	if(0 != (err = kxtj2_1009_init_client(new_client, 1)))
	{
		goto exit_init_failed;
	}
	

	if(0 != (err = misc_register(&kxtj2_1009_device)))
	{
		GSE_ERR("kxtj2_1009_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	if(0 != (err = kxtj2_1009_create_attr(&kxtj2_1009_gsensor_driver.driver)))
	{
		GSE_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

	sobj.self = obj;
    sobj.polling = 1;
    sobj.sensor_operate = gsensor_operate;
	if(0 != (err = hwmsen_attach(ID_ACCELEROMETER, &sobj)))
	{
		GSE_ERR("attach fail = %d\n", err);
		goto exit_kfree;
	}

//#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(USE_EARLY_SUSPEND)
	//obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	//obj->early_drv.suspend  = kxtj2_1009_early_suspend,
	//obj->early_drv.resume   = kxtj2_1009_late_resume,    
	//register_early_suspend(&obj->early_drv);
//#endif 
	kxtj2_1009_init_flag = 0; //RickLiu
	printk("[Rick] kxtj2_1009_i2c_probe OK!");
	GSE_LOG("%s: OK\n", __func__);    
	return 0;

	exit_create_attr_failed:
	misc_deregister(&kxtj2_1009_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	//i2c_detach_client(new_client);
	exit_kfree:
	kfree(obj);
	exit:
	kxtj2_1009_init_flag = -1; //RickLiu
	GSE_ERR("%s: err = %d\n", __func__, err);        
	return err;
}

/*----------------------------------------------------------------------------*/
static int kxtj2_1009_i2c_remove(struct i2c_client *client)
{
	int err = 0;	
	
	if(0 != (err = kxtj2_1009_delete_attr(&kxtj2_1009_gsensor_driver.driver)))
	{
		GSE_ERR("kxtj2_1009_delete_attr fail: %d\n", err);
	}
	
	if(0 != (err = misc_deregister(&kxtj2_1009_device)))
	{
		GSE_ERR("misc_deregister fail: %d\n", err);
	}

	if(0 != (err = hwmsen_detach(ID_ACCELEROMETER)))
    {
		GSE_ERR("hwmsen_detach fail: %d\n", err);
	}    

	kxtj2_1009_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}
/*----------------------------------------------------------------------------*/
static int kxtj2_1009_probe(struct platform_device *pdev) 
{
	//struct acc_hw *hw = get_cust_acc_hw();
	hw = get_accel_dts_func("mediatek,kxtj2_1009", hw);
	
	GSE_FUN();

	KXTJ2_1009_power(hw, 1);
	//kxtj2_1009_force[0] = hw->i2c_num;
	if(i2c_add_driver(&kxtj2_1009_i2c_driver))
	{
		GSE_ERR("add driver error\n");
		return -1;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static struct platform_driver kxtj2_1009_gsensor_driver = {
	.probe      = kxtj2_1009_probe,
	//.remove     = kxtj2_1009_remove,    
	.driver     = {
		.name  = "gsensor",
		.owner = THIS_MODULE,
	}
};

//< RickLiu
/*****************************************
 *** gma30x_local_init
 *****************************************/
static int  kxtj2_1009_local_init(void)
{
    //struct acc_hw *hw = get_cust_acc_hw();
	hw = get_accel_dts_func("mediatek,kxtj2_1009", hw);
    
	GSE_FUN();    
	KXTJ2_1009_power(hw, 1);    
  
	if (i2c_add_driver(&kxtj2_1009_i2c_driver)) {
		GSE_ERR("add driver error\n");
		return -1;
	}

	if (-1 == kxtj2_1009_init_flag) {
		GSE_ERR("init driver error\n");
		return -1;
	}
	return 0;
}
//> RickLiu

/*----------------------------------------------------------------------------*/
static int __init kxtj2_1009_init(void)
{
	//struct acc_hw *hw = get_cust_acc_hw();
	hw = get_accel_dts_func("mediatek,kxtj2_1009", hw);
	
    GSE_FUN();
	GSE_LOG("%s: i2c_number=%d\n", __func__,hw->i2c_num);
	if (hw->i2c_addr[0])
	{
		GSE_LOG("%s: i2c_slave_addr=%d\n", __func__,hw->i2c_addr[0]);
		i2c_kxtj2_1009.addr = hw->i2c_addr[0]>>1;
	}
	i2c_register_board_info(hw->i2c_num, &i2c_kxtj2_1009, 1);
	if(platform_driver_register(&kxtj2_1009_gsensor_driver))
	{
		GSE_ERR("failed to register driver");
		return -ENODEV;
	}
	acc_driver_add(&kxtj2_1009_init_info); //< RickLiu
	return 0;    
}
/*----------------------------------------------------------------------------*/
static void __exit kxtj2_1009_exit(void)
{
	GSE_FUN();
	platform_driver_unregister(&kxtj2_1009_gsensor_driver);
}
/*----------------------------------------------------------------------------*/
module_init(kxtj2_1009_init);
module_exit(kxtj2_1009_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("KXTJ2_1009 I2C driver");
MODULE_AUTHOR("Dexiang.Liu@mediatek.com");
