/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 IMX234mipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//<2015/11/15-kylechang, for Android M
//#include <linux/xlog.h>
//#include "kd_camera_hw.h"
#include <linux/types.h>
#include "kd_camera_typedef.h"
//>2015/10/15-kylechang
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "imx234mipi_Sensor.h"

#define PFX "imx234_camera_sensor"
//#define LOG_WRN(format, args...) xlog_printk(ANDROID_LOG_WARN ,PFX, "[%S] " format, __FUNCTION__, ##args)
//#defineLOG_INF(format, args...) xlog_printk(ANDROID_LOG_INFO ,PFX, "[%s] " format, __FUNCTION__, ##args)
//#define LOG_DBG(format, args...) xlog_printk(ANDROID_LOG_DEBUG ,PFX, "[%S] " format, __FUNCTION__, ##args)
//<2015/11/16-kylechang, LOG_INF is replaced by printk
//#define LOG_INF(format, args...)	pr_debug(PFX, "[%s] " format, __FUNCTION__, ##args)
//>2015/11/16-kylechang
static DEFINE_SPINLOCK(imgsensor_drv_lock);

//<2015/11/02-kylechang, Add long exposure feature
kal_bool Is_longExp=0;
//>2015/11/02-kylechang

static imgsensor_info_struct imgsensor_info = { 
	//<2015/10/15-kylechang, Porting Sub Camera Driver IMX234
	//Check Register: 0x0A36=0x51 (0x0A37=0x60, 0x0A38=0x23, 0x0A39=0x4e, 0x0A3A~0x0A3D)
	.sensor_id =IMX234_SENSOR_ID,		//record sensor id defined in Kd_imgsensor.h
	//>2015/10/15-kylechang
	.checksum_value = 0x22866f5f,		//checksum value for Camera Auto Test
	
	.pre = {
		.pclk = 288000000,				//record different mode's pclk
		.linelength = 6144,				//record different mode's linelength
		.framelength = 1560,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 2656,   //2672		//record different mode's width of grabwindow
		.grabwindow_height = 1494,	 //1500 	//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
	.cap = {
		.pclk = 573000000,
		.linelength = 6144,
		.framelength = 3106,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 5312,  //5344
		.grabwindow_height = 2988,  //3000
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
	}, 
	.cap1 = {							//capture for PIP 24fps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
		.pclk = 462000000,
		.linelength = 6144,
		.framelength = 3106,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 5312,
		.grabwindow_height = 2988,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 240,	//less than 13M(include 13M),cap1 max framerate is 24fps,16M max framerate is 20fps, 20M max framerate is 15fps  
	},
	.normal_video = {
		.pclk = 573000000,
		.linelength = 6144,
		.framelength = 3106,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 5312,
		.grabwindow_height = 2988,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 429000000,
		.linelength = 6144,
		.framelength = 580,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 640,
		.grabwindow_height = 480,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 1200,
	},
	.slim_video = {
		.pclk = 288000000,
		.linelength = 6144,
		.framelength = 1560,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2656,
		.grabwindow_height = 1494,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
	},
	//<2015/12/31-SimonLin, for saving electricity
	.halfsize_video = {
		
		
		#if SUB_FULLSIZE
		.pclk = 573000000,
		.linelength = 6144,
		.framelength = 3106,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 5312,
		.grabwindow_height = 2988,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
		#elif SUB_HALFSIZE
		.pclk = 288000000,				//record different mode's pclk
		.linelength = 6144,				//record different mode's linelength
		.framelength = 1560,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 2656,   //2672		//record different mode's width of grabwindow
		.grabwindow_height = 1494,	 //1500 	//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
		
		#elif SUB_BINNING
		.pclk = 355000000,
		.linelength = 6144,
		.framelength = 1540,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2656,
		.grabwindow_height = 1496,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
		#endif
		
	},
	//>2015/12/31-SimonLin,
	.margin = 4,			//sensor framelength & shutter margin
	.min_shutter = 4,		//min shutter
	.max_frame_length = 0x7fff,//max framelength by sensor register's limitation
	.ae_shut_delay_frame = 0,	//shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
	.ae_sensor_gain_delay_frame = 0,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	.ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
	.ihdr_support = 0,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 5,	  //support sensor mode num
	
	.cap_delay_frame = 2,		//enter capture delay frame num // 20160122 RickLiu 1 > 2 fix EV value down when take picture
	.pre_delay_frame = 2, 		//enter preview delay frame num // 20160122 RickLiu 1 > 2 fix EV value down when take picture
	.video_delay_frame = 1,		//enter video delay frame num
	.hs_video_delay_frame = 1,	//enter high speed video  delay frame num
	.slim_video_delay_frame = 1,//enter slim video delay frame num
	
	.isp_driving_current = ISP_DRIVING_8MA, //mclk driving current
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,//sensor_interface_type
    .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
    .mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,//sensor output first pixel color
	.mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
	.mipi_lane_num = SENSOR_MIPI_4_LANE,//mipi lane num
	.i2c_addr_table = {0x20, 0x34, 0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x4C00,					//current shutter
	.gain = 0x0200,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 0,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = 0, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x34,//record current sensor's i2c write id
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =	 
{{ 5344, 3000,	  0,	0, 5344, 3000, 2672, 1500, 0000, 0000, 2672,  1500,	  0,	0, 2672, 1500}, // Preview 
 { 5344, 3000,	  0,	0, 5344, 3000, 5344, 3000, 0000, 0000, 5344,  3000,	  0,	0, 5344, 3000}, // capture 
 #if SUB_HALFSIZE
 	//<2016/01/14-SimonLin, modify video in half size
 {5344, 3000,	  0,	0, 5344, 3000, 2672, 1500, 0000, 0000, 2672,  1500,	  0,	0, 2672, 1500}, // half size video
#elif SUB_FULLSIZE
{ 5344, 3000,	  0,    0, 5344, 3000, 5344, 3000, 0000, 0000, 5344,  3000,	  0,	0, 5344, 3000}, // video 

 #elif SUB_BINNING
 { 5344, 3000,	  0,    4, 5344, 2992, 2672, 1496, 8, 0000, 2656,  1494,	  0,	0, 2656, 1494},
 #endif
 	//<2016/01/14-SimonLin, modify video in half size
 { 5344, 3000,	  0,  536, 5344, 2456,  640,  480, 0000, 0000,  640,   480,	  0,	0, 	640,  480}, //hight speed video 
 { 5344, 3000,	  0,	0, 5344, 3000, 2672, 1500, 0000, 0000, 2672,  1500,	  0,	0, 2672, 1500}};// slim video 


#define IMX234MIPI_MaxGainIndex (119)
kal_uint16 IMX234MIPI_sensorGainMapping[IMX234MIPI_MaxGainIndex][2] ={	
	{64 ,0	},
	{65 ,8	},
	{66 ,16 },
	{67 ,25 },
	{68 ,30 },
	{69 ,37 },
	{70 ,45 },
	{71 ,51 },
	{72 ,57 },
	{73 ,63 },
	{74 ,67 },
	{75 ,75 },
	{76 ,81 },
	{77 ,85 },
	{78 ,92 },
	{79 ,96 },
	{80 ,103},
	{81 ,107},
	{82 ,112},
	{83 ,118},
	{84 ,122},
	{86 ,133},
	{88 ,140},
	{89 ,144},
	{90 ,148},
	{93 ,159},
	{96 ,171},
	{97 ,174},
	{99 ,182},
	{101,188},
	{102,192},
	{104,197},
	{106,202},
	{107,206},
	{109,211},
	{112,220},
	{113,222},
	{115,228},
	{118,235},
	{120,239},
	{125,250},
	{126,252},
	{128,256},
	{129,258},
	{130,260},
	{132,263},
	{133,266},
	{135,269},
	{136,271},
	{138,274},
	{139,276},
	{141,279},
	{142,282},
	{144,284},
	{145,286},
	{147,289},
	{149,292},
	{150,294},
	{152,296},
	{154,299},
	{155,301},
	{157,303},
	{158,305},
	{161,309},
	{163,311},
	{166,315},
	{170,319},
	{172,322},
	{174,324},
	{176,326},
	{179,329},
	{182,332},
	{185,335},
	{188,338},
	{192,341},
	{195,344},
	{196,345},
	{199,347},
	{200,348},
	{202,350},
	{205,352},
	{207,354},
	{210,356},
	{211,357},
	{214,359},
	{217,361},
	{218,362},
	{221,364},
	{224,366},
	{231,370},
	{237,374},
	{246,379},
	{250,381},
	{252,382},
	{256,384},
	{260,386},
	{262,387},
	{273,392},
	{275,393},
	{280,395},
	{290,399},
	{306,405},
	{312,407},
	{321,410},
	{331,413},
	{345,417},
	{352,419},
	{360,421},
	{364,422},
	{372,424},
	{386,427},
	{400,430},
	{410,432},
	{420,434},
	{431,436},
	{437,437},
	{449,439},
	{468,442},
	{512,448},
};

//<2016/01/15 ShermanWei,ISO6400
#define IMX234MIPI_MaxGainIndex2 (152)
kal_uint16 IMX234MIPI_sensorGainMapping2[IMX234MIPI_MaxGainIndex2][2] ={
	{72,0x0120},
	{80,0x0140},
	{88,0x0160},
	{96,0x0180},
	{104,0x01A0},
	{112,0x01C0},
	{120,0x01E0},
	{128,0x0200},
	{136,0x0220},
	{144,0x0240},
	{152,0x0260},
	{160,0x0280},
	{168,0x02A0},
	{176,0x02C0},
	{184,0x02E0},
	{192,0x0300},
	{200,0x0320},
	{208,0x0340},
	{216,0x0360},
	{224,0x0380},
	{232,0x03A0},
	{240,0x03C0},
	{248,0x03E0},
	{256,0x0400},
	{264,0x0420},
	{272,0x0440},
	{280,0x0460},
	{288,0x0480},
	{296,0x04A0},
	{304,0x04C0},
	{312,0x04E0},
	{320,0x0500},
	{328,0x0520},
	{336,0x0540},
	{344,0x0560},
	{352,0x0580},
	{360,0x05A0},
	{368,0x05C0},
	{376,0x05E0},
	{384,0x0600},
	{392,0x0620},
	{400,0x0640},
	{408,0x0660},
	{416,0x0680},
	{424,0x06A0},
	{432,0x06C0},
	{440,0x06E0},
	{448,0x0700},
	{456,0x0720},
	{464,0x0740},
	{472,0x0760},
	{480,0x0780},
	{488,0x07A0},
	{496,0x07C0},
	{504,0x07E0},
	{512,0x0800},
	{520,0x0820},
	{528,0x0840},
	{536,0x0860},
	{544,0x0880},
	{552,0x08A0},
	{560,0x08C0},
	{568,0x08E0},
	{576,0x0900},
	{584,0x0920},
	{592,0x0940},
	{600,0x0960},
	{608,0x0980},
	{616,0x09A0},
	{624,0x09C0},
	{632,0x09E0},
	{640,0x0A00},
	{648,0x0A20},
	{656,0x0A40},
	{664,0x0A60},
	{672,0x0A80},
	{680,0x0AA0},
	{688,0x0AC0},
	{696,0x0AE0},
	{704,0x0B00},
	{712,0x0B20},
	{720,0x0B40},
	{728,0x0B60},
	{736,0x0B80},
	{744,0x0BA0},
	{752,0x0BC0},
	{760,0x0BE0},
	{768,0x0C00},
	{776,0x0C20},
	{784,0x0C40},
	{792,0x0C60},
	{800,0x0C80},
	{808,0x0CA0},
	{816,0x0CC0},
	{824,0x0CE0},
	{832,0x0D00},
	{840,0x0D20},
	{848,0x0D40},
	{856,0x0D60},
	{864,0x0D80},
	{872,0x0DA0},
	{880,0x0DC0},
	{888,0x0DE0},
	{896,0x0E00},
	{904,0x0E20},
	{912,0x0E40},
	{920,0x0E60},
	{928,0x0E80},
	{936,0x0EA0},
	{944,0x0EC0},
	{952,0x0EE0},
	{960,0x0F00},
	{968,0x0F20},
	{976,0x0F40},
	{984,0x0F60},
	{992,0x0F80},
	{1000,0x0FA0},
	{1008,0x0FC0},
	{1016,0x0FE0},
	{1024,0x0FEF},
	{1032,0x0FFF},
///	{1024,0x1000},
///	{1032,0x1020},
	{1040,0x1040},
	{1048,0x1060},
	{1056,0x1080},
	{1064,0x10A0},
	{1072,0x10C0},
	{1080,0x10E0},
	{1088,0x1100},
	{1096,0x1120},
	{1104,0x1140},
	{1112,0x1160},
	{1120,0x1180},
	{1128,0x11A0},
	{1136,0x11C0},
	{1144,0x11E0},
	{1152,0x1200},
	{1160,0x1220},
	{1168,0x1240},
	{1176,0x1260},
	{1184,0x1280},
	{1192,0x12A0},
	{1200,0x12C0},
	{1208,0x12E0},
	{1216,0x1300},
	{1224,0x1320},
	{1232,0x1340},
	{1240,0x1360},
	{1248,0x1380},
	{1256,0x13A0},
	{1264,0x13C0},
	{1272,0x13E0},
	{1280,0x1400},
};
//>2016/01/15 ShermanWei,

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;

	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}


static void set_dummy(void)
{
	printk("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	/* you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel, or you can set dummy by imgsensor.frame_length and imgsensor.line_length */
	//write_cmos_sensor(0x0104, 0x01); 
	   
	write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
	write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);	  
	write_cmos_sensor(0x0342, imgsensor.line_length >> 8);
	write_cmos_sensor(0x0343, imgsensor.line_length & 0xFF);

	//write_cmos_sensor(0x0104, 0x00);
  
}	/*	set_dummy  */

//<2015/10/15-kylechang, Porting Sub Camera Driver IMX234
//Check Register: 0x0A36=0x51 (0x0A37:0x60, 0x0A38:0x23, 0x0A39:0x4e, 0x0A3A~0x0A3D)
static kal_uint32 return_sensor_id(void)
{
	kal_uint16 id_byte=0;
	kal_uint16 id_byte_1=0;
	kal_uint16 id_byte_2=0;
	kal_uint16 id_byte_3=0;
	kal_uint8 retry = 3;
	//Init Setting
	write_cmos_sensor(0x0100, 0x00);
	write_cmos_sensor(0x0A02, 0x1F);
	write_cmos_sensor(0x0A00, 0x01);
	//Read Register 0xA01, if value=0x01=>Ready
	do {
		id_byte=read_cmos_sensor(0xA01);
		printk("Check IMX234 Ready? 0x%x \n", id_byte);

		if(id_byte==0x01)
			break;
		retry--;

	} while(retry > 0);
	//
	id_byte=read_cmos_sensor(0x0A36);
	id_byte_1=read_cmos_sensor(0x0A37);
	id_byte_2=read_cmos_sensor(0x0A38);
	id_byte_3=read_cmos_sensor(0x0A39);

	printk("IMX234 Sensor ID= A36:0x%x, A37:0x%x, A38:0x%x, A39:0x%x , result = 0x%x\n", id_byte, id_byte_1, id_byte_2, id_byte_3,((read_cmos_sensor(0x0A38) << 4) | (read_cmos_sensor(0x0A39)>>4)));

   // return id_byte;
    return ((read_cmos_sensor(0x0A38) << 4) | (read_cmos_sensor(0x0A39)>>4));
}
//>2015/10/15-kylechang

static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

	printk("framerate = %d, min framelength should enable = %d\n", framerate,min_framelength_en);
   
	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length; 
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	//dummy_line = frame_length - imgsensor.min_frame_length;
	//if (dummy_line < 0)
		//imgsensor.dummy_line = 0;
	//else
		//imgsensor.dummy_line = dummy_line;
	//imgsensor.frame_length = frame_length + imgsensor.dummy_line;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */

//<2015/11/02-kylechang, Add long exposure feature
static void write_shutter(kal_uint32 shutter)
//>2015/11/02-kylechang
{
    unsigned long flags;
	kal_uint16 realtime_fps = 0;

	//<2015/11/02-kylechang, Add long exposure feature
	//kal_uint32 long_time=0; 
    //kal_uint32 long_shutter=0;
    kal_uint16 Base_FrameLength1=0xb627;//0.5s
    kal_uint32 Base_FrameLength2=0x16c4e;//1s
    kal_uint16 Multi=0; 
	//>2015/11/02-kylechang
	
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */
	
	// OV Recommend Solution
	// if shutter bigger than frame_length, should extend frame length first
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)		
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);

	//<2015/11/02-kylechang, Add long exposure feature
	if(shutter>0xFFFF)
	   Is_longExp=1;

	if(Is_longExp)
	{  
	
		 Multi=shutter/Base_FrameLength2;
	   
	printk("IMX234.Multi=%d \n ", Multi);
	 
	 switch(Multi){
		case 1: //0.5*2
		  write_cmos_sensor(0x0350, 1); //auto frame length enable
		  write_cmos_sensor(0x3028, 1);
		  write_cmos_sensor(0x0202, (Base_FrameLength1 >>8) & 0xFF);
		  write_cmos_sensor(0x0203, Base_FrameLength1& 0xFF);
		  break;
		case 2://0.5*4
		  write_cmos_sensor(0x0350, 1); //auto frame length enable
		  write_cmos_sensor(0x3028, 2);
		  write_cmos_sensor(0x0202, (Base_FrameLength1 >>8) & 0xFF);
		  write_cmos_sensor(0x0203, Base_FrameLength1& 0xFF);
		  break;
		case 4://0.5*8
		  write_cmos_sensor(0x0350, 1); //auto frame length enable
		  write_cmos_sensor(0x3028, 3);
					 write_cmos_sensor(0x0202, (Base_FrameLength1 >>8) & 0xFF);
		  write_cmos_sensor(0x0203, Base_FrameLength1& 0xFF);
		  break;
		case 6:  //0.375s*16
			Base_FrameLength1=0x6F62;
		  write_cmos_sensor(0x0350, 1); //auto frame length enable
		  write_cmos_sensor(0x3028, 4);
		  write_cmos_sensor(0x0202, (Base_FrameLength1 >>8) & 0xFF);
		  write_cmos_sensor(0x0203, Base_FrameLength1& 0xFF);
		  break;
		case 8: //0.5*16
		  write_cmos_sensor(0x0350, 1); //auto frame length enable
		  write_cmos_sensor(0x3028, 4);
		  write_cmos_sensor(0x0202, (Base_FrameLength1 >>8) & 0xFF);
		  write_cmos_sensor(0x0203, Base_FrameLength1& 0xFF);
		  break;
		case 10: //0.625s*16
		  Base_FrameLength1=0xB9A4;
		  write_cmos_sensor(0x0350, 1); //auto frame length enable
			 write_cmos_sensor(0x3028, 4);
		  write_cmos_sensor(0x0202, (Base_FrameLength1 >>8) & 0xFF);
		  write_cmos_sensor(0x0203, Base_FrameLength1& 0xFF);
	
		  break;
		case 12: //0.375*32
		   Base_FrameLength1=0x6F62;
		  write_cmos_sensor(0x0350, 1); //auto frame length enable
		  write_cmos_sensor(0x3028, 5);
		  write_cmos_sensor(0x0202, (Base_FrameLength1 >>8) & 0xFF);
		  write_cmos_sensor(0x0203, Base_FrameLength1& 0xFF);
		  break;
		case 14: //0.4375*32
		   Base_FrameLength1=0x81F3;
		  write_cmos_sensor(0x0350, 1); //auto frame length enable
		  write_cmos_sensor(0x3028, 5);
		  write_cmos_sensor(0x0202, (Base_FrameLength1 >>8) & 0xFF);
		  write_cmos_sensor(0x0203, Base_FrameLength1& 0xFF);
		  break;
		case 16: //0.5*32
		  write_cmos_sensor(0x0350, 1); //auto frame length enable
		  write_cmos_sensor(0x3028, 5);
		  write_cmos_sensor(0x0202, (Base_FrameLength1 >>8) & 0xFF);
		  write_cmos_sensor(0x0203, Base_FrameLength1& 0xFF);
		  break;
		case 18: //0.5625*32
		  Base_FrameLength1=0xA714;
		  write_cmos_sensor(0x0350, 1); //auto frame length enable
		  write_cmos_sensor(0x3028, 5);
		  write_cmos_sensor(0x0202, (Base_FrameLength1 >>8) & 0xFF);
		  write_cmos_sensor(0x0203, Base_FrameLength1& 0xFF);
		  break;
		case 20: //0.625s*32
			  Base_FrameLength1=0xB9A4;
		  write_cmos_sensor(0x0350, 1); //auto frame length enable
		  write_cmos_sensor(0x3028, 5);
		  write_cmos_sensor(0x0202, (Base_FrameLength1 >>8) & 0xFF);
		  write_cmos_sensor(0x0203, Base_FrameLength1& 0xFF);
		  break;
		case 22: //0.344s*64
			  Base_FrameLength1=0x662D;
		  write_cmos_sensor(0x0350, 1); //auto frame length enable
		  write_cmos_sensor(0x3028, 6);
		  write_cmos_sensor(0x0202, (Base_FrameLength1 >>8) & 0xFF);
		  write_cmos_sensor(0x0203, Base_FrameLength1& 0xFF);
		  break;
	   case 24: //0.375*64
			  Base_FrameLength1=0x6F62;
		  write_cmos_sensor(0x0350, 1); //auto frame length enable
		  write_cmos_sensor(0x3028, 6); 
		  write_cmos_sensor(0x0202, (Base_FrameLength1 >>8) & 0xFF);
		  write_cmos_sensor(0x0203, Base_FrameLength1& 0xFF);
		  break;
	   case 26: //0.406*64
		  Base_FrameLength1=0x7897;
		  write_cmos_sensor(0x0350, 1); //auto frame length enable
		  write_cmos_sensor(0x3028, 6);
		  write_cmos_sensor(0x0202, (Base_FrameLength1 >>8) & 0xFF);
		  write_cmos_sensor(0x0203, Base_FrameLength1& 0xFF);
		  break;
	   case 28: //0.4375*64
		   Base_FrameLength1=0x81F3;
		   write_cmos_sensor(0x0350, 1); //auto frame length enable
		  write_cmos_sensor(0x3028, 6);
		  write_cmos_sensor(0x0202, (Base_FrameLength1 >>8) & 0xFF);
		  write_cmos_sensor(0x0203, Base_FrameLength1& 0xFF);
		  break;
	   case 30: //0.469*64
		   Base_FrameLength1=0x8B4E;
		  write_cmos_sensor(0x0350, 1); //auto frame length enable
		  write_cmos_sensor(0x3028, 6);
		  write_cmos_sensor(0x0202, (Base_FrameLength1 >>8) & 0xFF);
		  write_cmos_sensor(0x0203, Base_FrameLength1& 0xFF);
	
		  break;
	   case 32://0.5*64   
		  write_cmos_sensor(0x0350, 1); //auto frame length enable
		  write_cmos_sensor(0x3028, 6);
		  write_cmos_sensor(0x0202, (Base_FrameLength1 >>8) & 0xFF);
		  write_cmos_sensor(0x0203, Base_FrameLength1& 0xFF);
	
		  break;
		default://1s
		  write_cmos_sensor(0x0350, 1); //auto frame length enable
		  write_cmos_sensor(0x3028, 1);
		  write_cmos_sensor(0x0202, (Base_FrameLength1 >>8) & 0xFF);
		  write_cmos_sensor(0x0203, Base_FrameLength1& 0xFF);
				 printk("ERR::long exp is not even number!");
		  break;
	  }
	Is_longExp=0;
	printk("shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);
	}
	else
    { 
		write_cmos_sensor(0x0350, 0x01);
		write_cmos_sensor(0x3028, 0x00);
	//>2015/11/02-kylechang
	
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
	
	if (imgsensor.autoflicker_en) { 
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);	
        else {
        // Extend frame length
 		//write_cmos_sensor(0x0104, 0x01); 
		write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
		//write_cmos_sensor(0x0104, 0x00);
        }
	} else {
		// Extend frame length
		//write_cmos_sensor(0x0104, 0x01); 
		write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
		//write_cmos_sensor(0x0104, 0x00);
	}

	// Update Shutter
	//write_cmos_sensor(0x0104, 0x01);      	
    write_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
    write_cmos_sensor(0x0203, shutter  & 0xFF);	
    //write_cmos_sensor(0x0104, 0x00);    
	printk("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);
	//<2015/11/02-kylechang, Add long exposure feature
	}
	//>2015/11/02-kylechang

	//LOG_INF("frame_length = %d ", frame_length);
	
}	/*	write_shutter  */



/*************************************************************************
* FUNCTION
*	set_shutter
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
//<2015/11/02-kylechang, Add long exposure feature
static void set_shutter(kal_uint32 shutter)
//>2015/11/02-kylechang
{
	unsigned long flags;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	
	write_shutter(shutter);
}	/*	set_shutter */



static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint8 iI;
    printk("[IMX234MIPI]enter IMX234MIPIGain2Reg function\n");
    for (iI = 0; iI < IMX234MIPI_MaxGainIndex; iI++) 
	{
		if(gain < IMX234MIPI_sensorGainMapping[iI][0])
		{                
			return IMX234MIPI_sensorGainMapping[iI][1];       
		}
			

    }
    if(gain != IMX234MIPI_sensorGainMapping[iI][0])
    {
         printk("Gain mapping don't correctly:%d %d \n", gain, IMX234MIPI_sensorGainMapping[iI][0]);
    }
	printk("exit IMX234MIPIGain2Reg function\n");
    return IMX234MIPI_sensorGainMapping[iI-1][1];
}


//<2016/01/15 ShermanWei,ISO6400
static kal_uint16 gain2reg2(const kal_uint16 gain)
{
    kal_uint8 i;

    for (i = 0; i < IMX234MIPI_MaxGainIndex2; i++) { 
    	if(gain <= IMX234MIPI_sensorGainMapping2[i][0])
		{                
			return IMX234MIPI_sensorGainMapping2[i][1];       
		}
    	
	}
    printk("Gain mapping don't correctly:%d index=%d \n", gain, i);
    return IMX234MIPI_sensorGainMapping2[i-1][1];
}

//>2016/01/15 ShermanWei,
/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
#if 0
	kal_uint16 reg_gain;
	
	if (gain < BASEGAIN || gain > 8 * BASEGAIN) {
		printk("Error gain setting");

		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 8 * BASEGAIN)
			gain = 8 * BASEGAIN;		 
		}

 
	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain; 
	spin_unlock(&imgsensor_drv_lock);
	printk("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	//write_cmos_sensor(0x0104, 0x01);
    write_cmos_sensor(0x0204, (reg_gain>>8)& 0xFF);
	write_cmos_sensor(0x0205, reg_gain & 0xFF);
    //write_cmos_sensor(0x0104, 0x00);
	
	return gain;
#else/////<2016/01/15 ShermanWei,ISO6400
    kal_uint16 reg_gain;
	kal_uint16 gain2;
	kal_uint16 reg_gain2;

    /* 0x350A[0:1], 0x350B[0:7] AGC real gain */
    /* [0:3] = N meams N /16 X    */
    /* [4:9] = M meams M X         */
    /* Total gain = M + N /16 X   */

    //
	gain2 = gain / 8;
	if (gain < BASEGAIN || gain > 80 * BASEGAIN) {
        printk("Error gain setting");

        if (gain < BASEGAIN)
            gain = BASEGAIN;
		else if (gain > 80 * BASEGAIN)
			gain = 80 * BASEGAIN;		 
    }

	if(gain <= 8 * BASEGAIN)
	{
	    reg_gain = gain2reg(gain);
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.gain = reg_gain;
	    spin_unlock(&imgsensor_drv_lock);
	    printk("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

		//write_cmos_sensor(0x0104, 0x01);
	    write_cmos_sensor(0x0204, (reg_gain>>8)& 0xFF);
		write_cmos_sensor(0x0205, reg_gain & 0xFF);
	    //write_cmos_sensor(0x0104, 0x00);
		// <20160128 Rick Liu, FIX - EV down issue when take picture
		write_cmos_sensor(0x020E,0x01); 
		write_cmos_sensor(0x020F,0x00); 
		write_cmos_sensor(0x0210,0x01); 
		write_cmos_sensor(0x0211,0x00); 
		write_cmos_sensor(0x0212,0x01); 
		write_cmos_sensor(0x0213,0x00); 
		write_cmos_sensor(0x0214,0x01); 
		write_cmos_sensor(0x0215,0x00);
		//> 20160128 Rick Liu, FIX - EV down issue when take picture
	}
	else if(gain <= 160 * BASEGAIN)
	{
		
		reg_gain2 = gain2reg2(gain2);
		printk("gain = %d , digital_reg_gain = 0x%x \n ", gain ,reg_gain2);

		//write_cmos_sensor(0x0104, 0x01);
		
		write_cmos_sensor(0x0204, 0x01);
		write_cmos_sensor(0x0205, 0xC0);

		write_cmos_sensor(0x020E, (reg_gain2>>8)&0xFF);
		write_cmos_sensor(0x020F, reg_gain2 & 0xFF);
		write_cmos_sensor(0x0210, (reg_gain2>>8)&0xFF);
		write_cmos_sensor(0x0211, reg_gain2 & 0xFF);
		write_cmos_sensor(0x0212, (reg_gain2>>8)&0xFF);
		write_cmos_sensor(0x0213, reg_gain2 & 0xFF);
		write_cmos_sensor(0x0214, (reg_gain2>>8)&0xFF);
		write_cmos_sensor(0x0215, reg_gain2 & 0xFF);
		
		//write_cmos_sensor(0x0104, 0x00);

	}
    return gain;

#endif
//>2016/01/15 ShermanWei,
}	/*	set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	printk("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);

	write_cmos_sensor(0x3820, 0x81);   //enable ihdr
 	
	if (imgsensor.ihdr_en) {
		
		spin_lock(&imgsensor_drv_lock);
			if (le > imgsensor.min_frame_length - imgsensor_info.margin)		
				imgsensor.frame_length = le + imgsensor_info.margin;
			else
				imgsensor.frame_length = imgsensor.min_frame_length;
			if (imgsensor.frame_length > imgsensor_info.max_frame_length)
				imgsensor.frame_length = imgsensor_info.max_frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
			if (se < imgsensor_info.min_shutter) se = imgsensor_info.min_shutter;
			
			
				// Extend frame length first
				write_cmos_sensor(0x380e, (imgsensor.frame_length >> 8)& 0xFF);
				write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);

		write_cmos_sensor(0x3502, (le << 4) & 0xFF);
		write_cmos_sensor(0x3501, (le >> 4) & 0xFF);	 
		write_cmos_sensor(0x3500, (le >> 12) & 0x0F);
		
		write_cmos_sensor(0x3508, (se << 4) & 0xFF); 
		write_cmos_sensor(0x3507, (se >> 4) & 0xFF);
		write_cmos_sensor(0x3506, (se >> 12) & 0x0F); 

		set_gain(gain);
	}

}


#if 0
static void set_mirror_flip(kal_uint8 image_mirror)
{
	printk("image_mirror = %d\n", image_mirror);

	/********************************************************
	   *
	   *   0x3820[2] ISP Vertical flip
	   *   0x3820[1] Sensor Vertical flip
	   *
	   *   0x3821[2] ISP Horizontal mirror
	   *   0x3821[1] Sensor Horizontal mirror
	   *
	   *   ISP and Sensor flip or mirror register bit should be the same!!
	   *
	   ********************************************************/
	
	switch (image_mirror) {
		case IMAGE_NORMAL:
			write_cmos_sensor(0x0101, 0x00);
			break;
		case IMAGE_H_MIRROR:
			write_cmos_sensor(0x0101, 0x01);
			break;
		case IMAGE_V_MIRROR:
			write_cmos_sensor(0x0101, 0x10);	
			break;
		case IMAGE_HV_MIRROR:
			write_cmos_sensor(0x0101, 0x11);
			break;
		default:
			printk("Error image_mirror setting\n");
	}

}
#endif
/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/ 
}	/*	night_mode	*/

static void sensor_init(void)
{
	printk("IMX234_Sensor_Init_2lane E\n");
	
	write_cmos_sensor(0x0101,0x00);
	write_cmos_sensor(0x0136,0x18);
	write_cmos_sensor(0x0137,0x00);
//<2016/01/15-SimonLin, fix take a pink photo when using front camera
write_cmos_sensor(0x301A,	0x00);
//>2016/01/15-SimonLin, fix take a pink photo when using front camera
	write_cmos_sensor(0x3021,0x00);
	write_cmos_sensor(0x3024,0x00);
	write_cmos_sensor(0x3025,0x00);
	write_cmos_sensor(0x4128,0x00);
	write_cmos_sensor(0x4129,0x0F);
	write_cmos_sensor(0x412E,0x00);
	write_cmos_sensor(0x412F,0x0E);
	write_cmos_sensor(0x4159,0x5D);
	write_cmos_sensor(0x4550,0x02);
	write_cmos_sensor(0x45D0,0x01);
	write_cmos_sensor(0x4612,0x30);
	write_cmos_sensor(0x4619,0x38);
	write_cmos_sensor(0x461A,0x7C);
	write_cmos_sensor(0x461B,0x2A);
	write_cmos_sensor(0x463F,0x18);
	write_cmos_sensor(0x4653,0x29);
	write_cmos_sensor(0x4657,0x29);
	write_cmos_sensor(0x465B,0x28);
	write_cmos_sensor(0x465F,0x23);
	write_cmos_sensor(0x4667,0x22);
	write_cmos_sensor(0x466B,0x22);
	write_cmos_sensor(0x4673,0x1A);
	write_cmos_sensor(0x4904,0x00);
	write_cmos_sensor(0x4905,0xA6);
	write_cmos_sensor(0x4906,0x00);
	write_cmos_sensor(0x4907,0x8E);
	write_cmos_sensor(0x4908,0x01);
	write_cmos_sensor(0x4909,0x18);
	write_cmos_sensor(0x490A,0x00);
	write_cmos_sensor(0x490B,0xD7);
	write_cmos_sensor(0x4A5C,0x01);
	write_cmos_sensor(0x4A5D,0x0A);
	write_cmos_sensor(0x4A62,0x01);
	write_cmos_sensor(0x4A63,0x0A);
	write_cmos_sensor(0x4A72,0x00);
	write_cmos_sensor(0x4A73,0x8C);
	write_cmos_sensor(0x4A76,0x01);
	write_cmos_sensor(0x4A77,0x07);
	write_cmos_sensor(0x4A7A,0x01);
	write_cmos_sensor(0x4A7B,0x07);
	write_cmos_sensor(0x4A84,0x01);
	write_cmos_sensor(0x4A85,0x0A);
	write_cmos_sensor(0x6227,0x11);
//<2016/01/15-SimonLin, fix take a pink photo when using front camera
write_cmos_sensor(0x6283,	0x04);
//>2016/01/15-SimonLin, fix take a pink photo when using front camera
	write_cmos_sensor(0xA900,0x7F);
	write_cmos_sensor(0xA908,0x00);
	write_cmos_sensor(0xA909,0x3D);
	write_cmos_sensor(0xAE1C,0x05);
	write_cmos_sensor(0xAE1D,0x30);
	write_cmos_sensor(0xAE1E,0x04);
	write_cmos_sensor(0xAE1F,0xA4);
	write_cmos_sensor(0xAE20,0x04);
	write_cmos_sensor(0xAE21,0xA4);
//<2016/01/15-SimonLin, fix take a pink photo when using front camera
write_cmos_sensor(0xAE32,	0x00);
//>2016/01/15-SimonLin, fix take a pink photo when using front camera
	write_cmos_sensor(0xAE3A,0x05);
	write_cmos_sensor(0xAE3B,0x30);
	


}	/*	sensor_init  */


static void preview_setting(void)
{
	printk(" IMX234PreviewSetting_4lane enter\n");

	write_cmos_sensor(0x0100,0x00);
	
	write_cmos_sensor(0x0114,0x03);
	write_cmos_sensor(0x0340,0x06);
	write_cmos_sensor(0x0341,0x18);
	write_cmos_sensor(0x0342,0x18);
	write_cmos_sensor(0x0343,0x00);
	write_cmos_sensor(0x0344,0x00);
	write_cmos_sensor(0x0345,0x00);
	write_cmos_sensor(0x0346,0x00);
	write_cmos_sensor(0x0347,0x00);
	write_cmos_sensor(0x0348,0x14);
	write_cmos_sensor(0x0349,0xDF);
	write_cmos_sensor(0x034A,0x0B);
	write_cmos_sensor(0x034B,0xB7);
	write_cmos_sensor(0x0381,0x01);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0385,0x01);
	write_cmos_sensor(0x0387,0x01);
	write_cmos_sensor(0x0900,0x01);
	write_cmos_sensor(0x0901,0x22);
	write_cmos_sensor(0x0902,0x00);
	write_cmos_sensor(0x3029,0x00);
	write_cmos_sensor(0x305C,0x11);

	write_cmos_sensor(0x0112,0x0A);
	write_cmos_sensor(0x0113,0x0A);
	write_cmos_sensor(0x034C,0x0A);
	write_cmos_sensor(0x034D,0x70);
	write_cmos_sensor(0x034E,0x05);
	write_cmos_sensor(0x034F,0xDC);
	write_cmos_sensor(0x0401,0x00);
	write_cmos_sensor(0x0404,0x00);
	write_cmos_sensor(0x0405,0x10);
	write_cmos_sensor(0x0408,0x00);
	write_cmos_sensor(0x0409,0x00);
	write_cmos_sensor(0x040A,0x00);
	write_cmos_sensor(0x040B,0x00);
	write_cmos_sensor(0x040C,0x0A);
	write_cmos_sensor(0x040D,0x70);
	write_cmos_sensor(0x040E,0x05);
	write_cmos_sensor(0x040F,0xDC);

	write_cmos_sensor(0x0301,0x04);
	write_cmos_sensor(0x0303,0x02);
	write_cmos_sensor(0x0305,0x04);
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0x60);
	write_cmos_sensor(0x0309,0x0A);
	write_cmos_sensor(0x030B,0x01);
	write_cmos_sensor(0x030D,0x0F);
	write_cmos_sensor(0x030E,0x00);
	write_cmos_sensor(0x030F,0xFA);
	write_cmos_sensor(0x0310,0x01);

	write_cmos_sensor(0x0820,0x06);
	write_cmos_sensor(0x0821,0x40);
	write_cmos_sensor(0x0822,0x00);
	write_cmos_sensor(0x0823,0x00);

	write_cmos_sensor(0x0202,0x06);
	write_cmos_sensor(0x0203,0x0E);

	write_cmos_sensor(0x0204,0x00);
	write_cmos_sensor(0x0205,0x00);
	write_cmos_sensor(0x020E,0x01);
	write_cmos_sensor(0x020F,0x00);
	write_cmos_sensor(0x0210,0x01);
	write_cmos_sensor(0x0211,0x00);
	write_cmos_sensor(0x0212,0x01);
	write_cmos_sensor(0x0213,0x00);
	write_cmos_sensor(0x0214,0x01);
	write_cmos_sensor(0x0215,0x00);										  
	
	write_cmos_sensor(0x0100,0x01);
	
}	/*	preview_setting  */

#if SUB_BINNING
static void binning_video_setting(kal_uint16 currefps)
{
	printk("binning_video_setting Enter! currefps\n");

	write_cmos_sensor(0x0100,0x00);
	
	write_cmos_sensor(0x0114,0x03);
	write_cmos_sensor(0x0340,0x06);
	write_cmos_sensor(0x0341,0x18);
	write_cmos_sensor(0x0342,0x18);
	write_cmos_sensor(0x0343,0x00);
	write_cmos_sensor(0x0344,0x00);
	write_cmos_sensor(0x0345,0x00);
	write_cmos_sensor(0x0346,0x00);
	write_cmos_sensor(0x0347,0x00);
	write_cmos_sensor(0x0348,0x14);
	write_cmos_sensor(0x0349,0xDF);
	write_cmos_sensor(0x034A,0x0B);
	write_cmos_sensor(0x034B,0xB7);
	write_cmos_sensor(0x0381,0x01);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0385,0x01);
	write_cmos_sensor(0x0387,0x01);
	write_cmos_sensor(0x0900,0x01);
	write_cmos_sensor(0x0901,0x22);
	write_cmos_sensor(0x0902,0x00);
	write_cmos_sensor(0x3029,0x00);
	write_cmos_sensor(0x305C,0x11);

	write_cmos_sensor(0x0112,0x0A);
	write_cmos_sensor(0x0113,0x0A);
	write_cmos_sensor(0x034C,0x0A);
	write_cmos_sensor(0x034D,0x70);
	write_cmos_sensor(0x034E,0x05);
	write_cmos_sensor(0x034F,0xDC);
	write_cmos_sensor(0x0401,0x00);
	write_cmos_sensor(0x0404,0x00);
	write_cmos_sensor(0x0405,0x10);
	write_cmos_sensor(0x0408,0x00);
	write_cmos_sensor(0x0409,0x00);
	write_cmos_sensor(0x040A,0x00);
	write_cmos_sensor(0x040B,0x00);
	write_cmos_sensor(0x040C,0x0A);
	write_cmos_sensor(0x040D,0x70);
	write_cmos_sensor(0x040E,0x05);
	write_cmos_sensor(0x040F,0xDC);

	write_cmos_sensor(0x0301,0x04);
	write_cmos_sensor(0x0303,0x02);
	write_cmos_sensor(0x0305,0x04);
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0x60);
	write_cmos_sensor(0x0309,0x0A);
	write_cmos_sensor(0x030B,0x01);
	write_cmos_sensor(0x030D,0x0F);
	write_cmos_sensor(0x030E,0x00);
	write_cmos_sensor(0x030F,0xE1);
	write_cmos_sensor(0x0310,0x01);

	write_cmos_sensor(0x0820,0x05);
	write_cmos_sensor(0x0821,0xA0);
	write_cmos_sensor(0x0822,0x00);
	write_cmos_sensor(0x0823,0x00);

	write_cmos_sensor(0x0202,0x06);
	write_cmos_sensor(0x0203,0x0E);

	write_cmos_sensor(0x0204,0x00);
	write_cmos_sensor(0x0205,0x00);
	write_cmos_sensor(0x020E,0x01);
	write_cmos_sensor(0x020F,0x00);
	write_cmos_sensor(0x0210,0x01);
	write_cmos_sensor(0x0211,0x00);
	write_cmos_sensor(0x0212,0x01);
	write_cmos_sensor(0x0213,0x00);
	write_cmos_sensor(0x0214,0x01);
	write_cmos_sensor(0x0215,0x00);										  
	
	write_cmos_sensor(0x0100,0x01);

}
#endif
static void capture_setting(kal_uint16 currefps)
{
	printk("IMX234CaptureSetting_4lane enter! currefps:%d\n",currefps);
	if (currefps == 240) { //24fps for PIP
		write_cmos_sensor(0x0100,0x00);
		
		write_cmos_sensor(0x0114,0x03);
		write_cmos_sensor(0x0340,0x0C);
//<2016/01/15-SimonLin, update pip setting
write_cmos_sensor(0x0341,	0x3C);
//>2016/01/15-SimonLin, update pip setting
		write_cmos_sensor(0x0342,0x18);
		write_cmos_sensor(0x0343,0x00);
		write_cmos_sensor(0x0344,0x00);
		write_cmos_sensor(0x0345,0x00);
		write_cmos_sensor(0x0346,0x00);
		write_cmos_sensor(0x0347,0x00);
		write_cmos_sensor(0x0348,0x14);
		write_cmos_sensor(0x0349,0xDF);
		write_cmos_sensor(0x034A,0x0B);
//<2016/01/15-SimonLin, update pip setting
write_cmos_sensor(0x034B,	0xAF);
//>2016/01/15-SimonLin, update pip setting
		write_cmos_sensor(0x0381,0x01);
		write_cmos_sensor(0x0383,0x01);
		write_cmos_sensor(0x0385,0x01);
		write_cmos_sensor(0x0387,0x01);
		write_cmos_sensor(0x0900,0x00);
		write_cmos_sensor(0x0901,0x11);
		write_cmos_sensor(0x0902,0x00);
		write_cmos_sensor(0x3029,0x00);
		write_cmos_sensor(0x305C,0x11);

		write_cmos_sensor(0x0112,0x0A);
		write_cmos_sensor(0x0113,0x0A);
		write_cmos_sensor(0x034C,0x14);
//<2016/01/15-SimonLin, update pip setting
write_cmos_sensor(0x034D,	0xC0);
//>2016/01/15-SimonLin, update pip setting
		write_cmos_sensor(0x034E,0x0B);
//<2016/01/15-SimonLin, update pip setting
write_cmos_sensor(0x034F,	0xAC);
//>2016/01/15-SimonLin, update pip setting
		write_cmos_sensor(0x0401,0x00);
		write_cmos_sensor(0x0404,0x00);
		write_cmos_sensor(0x0405,0x10);
		write_cmos_sensor(0x0408,0x00);
//<2016/01/15-SimonLin, update pip setting
write_cmos_sensor(0x0409,	0x10);
//>2016/01/15-SimonLin, update pip setting
		write_cmos_sensor(0x040A,0x00);
		write_cmos_sensor(0x040B,0x00);
		write_cmos_sensor(0x040C,0x14);
//<2016/01/15-SimonLin, update pip setting
write_cmos_sensor(0x040D,	0xC0);
//>2016/01/15-SimonLin, update pip setting
		write_cmos_sensor(0x040E,0x0B);
//<2016/01/15-SimonLin, update pip setting
write_cmos_sensor(0x040F,	0xAC);
//>2016/01/15-SimonLin, update pip setting

		write_cmos_sensor(0x0301,0x04);
		write_cmos_sensor(0x0303,0x02);
		write_cmos_sensor(0x0305,0x04);
		write_cmos_sensor(0x0306,0x00);
		write_cmos_sensor(0x0307,0x9A);
		write_cmos_sensor(0x0309,0x0A);
		write_cmos_sensor(0x030B,0x01);
		write_cmos_sensor(0x030D,0x0F);
		write_cmos_sensor(0x030E,0x02);
		write_cmos_sensor(0x030F,0xA3);
		write_cmos_sensor(0x0310,0x01);

		write_cmos_sensor(0x0820,0x10);
		write_cmos_sensor(0x0821,0xE0);
		write_cmos_sensor(0x0822,0x00);
		write_cmos_sensor(0x0823,0x00);

		write_cmos_sensor(0x0202,0x0C);
//<2016/01/15-SimonLin, update pip setting
write_cmos_sensor(0x0203,	0x32);
//>2016/01/15-SimonLin, update pip setting

		write_cmos_sensor(0x0204,0x00);
		write_cmos_sensor(0x0205,0x00);
		write_cmos_sensor(0x020E,0x01);
		write_cmos_sensor(0x020F,0x00);
		write_cmos_sensor(0x0210,0x01);
		write_cmos_sensor(0x0211,0x00);
		write_cmos_sensor(0x0212,0x01);
		write_cmos_sensor(0x0213,0x00);
		write_cmos_sensor(0x0214,0x01);
		write_cmos_sensor(0x0215,0x00);

		write_cmos_sensor(0x0100,0x01);
		
	} else{ // for 30fps need ti update
		write_cmos_sensor(0x0100,0x00);
		
		write_cmos_sensor(0x0114,0x03);
		write_cmos_sensor(0x0340,0x0C);
//<2016/01/15-SimonLin, fix take a pink photo when using front camera
write_cmos_sensor(0x0341,	0x24 );
//>2016/01/15-SimonLin, fix take a pink photo when using front camera
		write_cmos_sensor(0x0342,0x18);
		write_cmos_sensor(0x0343,0x00);
		write_cmos_sensor(0x0344,0x00);
		write_cmos_sensor(0x0345,0x00);
		write_cmos_sensor(0x0346,0x00);
//<2016/01/15-SimonLin, fix take a pink photo when using front camera
write_cmos_sensor(0x0347,	0x04 );
//>2016/01/15-SimonLin, fix take a pink photo when using front camera
		write_cmos_sensor(0x0348,0x14);
		write_cmos_sensor(0x0349,0xDF);
		write_cmos_sensor(0x034A,0x0B);
//<2016/01/15-SimonLin, fix take a pink photo when using front camera
write_cmos_sensor(0x034B,	0xAF );
//>2016/01/15-SimonLin, fix take a pink photo when using front camera
		write_cmos_sensor(0x0381,0x01);
		write_cmos_sensor(0x0383,0x01);
		write_cmos_sensor(0x0385,0x01);
		write_cmos_sensor(0x0387,0x01);
		write_cmos_sensor(0x0900,0x00);
		write_cmos_sensor(0x0901,0x11);
		write_cmos_sensor(0x0902,0x00);
		write_cmos_sensor(0x3029,0x00);
		write_cmos_sensor(0x305C,0x11);
		
		write_cmos_sensor(0x0112,0x0A);
		write_cmos_sensor(0x0113,0x0A);
		write_cmos_sensor(0x034C,0x14);
//<2016/01/15-SimonLin, fix take a pink photo when using front camera
write_cmos_sensor(0x034D,	0xC0 );
//>2016/01/15-SimonLin, fix take a pink photo when using front camera
		write_cmos_sensor(0x034E,0x0B);
//<2016/01/15-SimonLin, fix take a pink photo when using front camera
write_cmos_sensor(0x034F,	0xAC );
//>2016/01/15-SimonLin, fix take a pink photo when using front camera
		write_cmos_sensor(0x0401,0x00);
		write_cmos_sensor(0x0404,0x00);
		write_cmos_sensor(0x0405,0x10);
		write_cmos_sensor(0x0408,0x00);
//<2016/01/15-SimonLin, fix take a pink photo when using front camera
write_cmos_sensor(0x0409,	0x10 );
//>2016/01/15-SimonLin, fix take a pink photo when using front camera
		write_cmos_sensor(0x040A,0x00);
		write_cmos_sensor(0x040B,0x00);
		write_cmos_sensor(0x040C,0x14);
//<2016/01/15-SimonLin, fix take a pink photo when using front camera
write_cmos_sensor(0x040D,	0xC0 );
//>2016/01/15-SimonLin, fix take a pink photo when using front camera
		write_cmos_sensor(0x040E,0x0B);
//<2016/01/15-SimonLin, fix take a pink photo when using front camera
write_cmos_sensor(0x040F,	0xAC );
//>2016/01/15-SimonLin, fix take a pink photo when using front camera
		
		write_cmos_sensor(0x0301,0x04);
		write_cmos_sensor(0x0303,0x02);
		write_cmos_sensor(0x0305,0x04);
		write_cmos_sensor(0x0306,0x00);
		write_cmos_sensor(0x0307,0xBF);
		write_cmos_sensor(0x0309,0x0A);
		write_cmos_sensor(0x030B,0x01);
		write_cmos_sensor(0x030D,0x0F);
		write_cmos_sensor(0x030E,0x03);
		write_cmos_sensor(0x030F,0x39);
		write_cmos_sensor(0x0310,0x01);
		
		write_cmos_sensor(0x0820,0x14);
		write_cmos_sensor(0x0821,0xA0);
		write_cmos_sensor(0x0822,0x00);
		write_cmos_sensor(0x0823,0x00);
		
		write_cmos_sensor(0x0202,0x0C);
//<2016/01/15-SimonLin, fix take a pink photo when using front camera
write_cmos_sensor(0x0203,	0x1A );
//>2016/01/15-SimonLin, fix take a pink photo when using front camera
		write_cmos_sensor(0x0204,0x00);
		write_cmos_sensor(0x0205,0x00);
		write_cmos_sensor(0x020E,0x01);
		write_cmos_sensor(0x020F,0x00);
		write_cmos_sensor(0x0210,0x01);
		write_cmos_sensor(0x0211,0x00);
		write_cmos_sensor(0x0212,0x01);
		write_cmos_sensor(0x0213,0x00);
		write_cmos_sensor(0x0214,0x01);
		write_cmos_sensor(0x0215,0x00);

		write_cmos_sensor(0x0100,0x01);
	} 
	
		
}
#if SUB_HALFSIZE
static void halfsize_video_setting(kal_uint16 currefps)
{
	printk("test_video_setting Enter! currefps:\n");
	
	write_cmos_sensor(0x0100,0x00);
	
	write_cmos_sensor(0x0114,0x03);
	write_cmos_sensor(0x0340,0x06);
	write_cmos_sensor(0x0341,0x18);
	write_cmos_sensor(0x0342,0x18);
	write_cmos_sensor(0x0343,0x00);
	write_cmos_sensor(0x0344,0x00);
	write_cmos_sensor(0x0345,0x00);
	write_cmos_sensor(0x0346,0x00);
	write_cmos_sensor(0x0347,0x00);
	write_cmos_sensor(0x0348,0x14);
	write_cmos_sensor(0x0349,0xDF);
	write_cmos_sensor(0x034A,0x0B);
	write_cmos_sensor(0x034B,0xB7);
	write_cmos_sensor(0x0381,0x01);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0385,0x01);
	write_cmos_sensor(0x0387,0x01);
	write_cmos_sensor(0x0900,0x01);
	write_cmos_sensor(0x0901,0x22);
	write_cmos_sensor(0x0902,0x00);
	write_cmos_sensor(0x3029,0x00);
	write_cmos_sensor(0x305C,0x11);

	write_cmos_sensor(0x0112,0x0A);
	write_cmos_sensor(0x0113,0x0A);
	write_cmos_sensor(0x034C,0x0A);
	write_cmos_sensor(0x034D,0x70);
	write_cmos_sensor(0x034E,0x05);
	write_cmos_sensor(0x034F,0xDC);
	write_cmos_sensor(0x0401,0x00);
	write_cmos_sensor(0x0404,0x00);
	write_cmos_sensor(0x0405,0x10);
	write_cmos_sensor(0x0408,0x00);
	write_cmos_sensor(0x0409,0x00);
	write_cmos_sensor(0x040A,0x00);
	write_cmos_sensor(0x040B,0x00);
	write_cmos_sensor(0x040C,0x0A);
	write_cmos_sensor(0x040D,0x70);
	write_cmos_sensor(0x040E,0x05);
	write_cmos_sensor(0x040F,0xDC);

	write_cmos_sensor(0x0301,0x04);
	write_cmos_sensor(0x0303,0x02);
	write_cmos_sensor(0x0305,0x04);
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0x60);
	write_cmos_sensor(0x0309,0x0A);
	write_cmos_sensor(0x030B,0x01);
	write_cmos_sensor(0x030D,0x0F);
	write_cmos_sensor(0x030E,0x00);
	write_cmos_sensor(0x030F,0xFA);
	write_cmos_sensor(0x0310,0x01);

	write_cmos_sensor(0x0820,0x06);
	write_cmos_sensor(0x0821,0x40);
	write_cmos_sensor(0x0822,0x00);
	write_cmos_sensor(0x0823,0x00);

	write_cmos_sensor(0x0202,0x06);
	write_cmos_sensor(0x0203,0x0E);

	write_cmos_sensor(0x0204,0x00);
	write_cmos_sensor(0x0205,0x00);
	write_cmos_sensor(0x020E,0x01);
	write_cmos_sensor(0x020F,0x00);
	write_cmos_sensor(0x0210,0x01);
	write_cmos_sensor(0x0211,0x00);
	write_cmos_sensor(0x0212,0x01);
	write_cmos_sensor(0x0213,0x00);
	write_cmos_sensor(0x0214,0x01);
	write_cmos_sensor(0x0215,0x00);										  
	
	write_cmos_sensor(0x0100,0x01);
	
}
#endif
#if SUB_FULLSIZE
static void normal_video_setting(kal_uint16 currefps)
{
	printk("normal_video_setting Enter! currefps\n");
	#if 0
	//<2015/12/31-SimonLin, for saving electricity
	//preview_setting();
	//>2015/12/31-SimonLin,
	#else
	write_cmos_sensor(0x0100,0x00);
		
	write_cmos_sensor(0x0114,0x03);
	write_cmos_sensor(0x0340,0x0C);
	write_cmos_sensor(0x0341,0x22);
	write_cmos_sensor(0x0342,0x18);
	write_cmos_sensor(0x0343,0x00);
	write_cmos_sensor(0x0344,0x00);
	write_cmos_sensor(0x0345,0x00);
	write_cmos_sensor(0x0346,0x00);
	write_cmos_sensor(0x0347,0x00);
	write_cmos_sensor(0x0348,0x14);
	write_cmos_sensor(0x0349,0xDF);
	write_cmos_sensor(0x034A,0x0B);
	write_cmos_sensor(0x034B,0xB7);
	write_cmos_sensor(0x0381,0x01);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0385,0x01);
	write_cmos_sensor(0x0387,0x01);
	write_cmos_sensor(0x0900,0x00);
	write_cmos_sensor(0x0901,0x11);
	write_cmos_sensor(0x0902,0x00);
	write_cmos_sensor(0x3029,0x00);
	write_cmos_sensor(0x305C,0x11);

	write_cmos_sensor(0x0112,0x0A);
	write_cmos_sensor(0x0113,0x0A);
	write_cmos_sensor(0x034C,0x14);
	write_cmos_sensor(0x034D,0xE0);
	write_cmos_sensor(0x034E,0x0B);
	write_cmos_sensor(0x034F,0xB8);
	write_cmos_sensor(0x0401,0x00);
	write_cmos_sensor(0x0404,0x00);
	write_cmos_sensor(0x0405,0x10);
	write_cmos_sensor(0x0408,0x00);
	write_cmos_sensor(0x0409,0x00);
	write_cmos_sensor(0x040A,0x00);
	write_cmos_sensor(0x040B,0x00);
	write_cmos_sensor(0x040C,0x14);
	write_cmos_sensor(0x040D,0xE0);
	write_cmos_sensor(0x040E,0x0B);
	write_cmos_sensor(0x040F,0xB8);

	write_cmos_sensor(0x0301,0x04);
	write_cmos_sensor(0x0303,0x02);
	write_cmos_sensor(0x0305,0x04);
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0xBF);
	write_cmos_sensor(0x0309,0x0A);
	write_cmos_sensor(0x030B,0x01);
	write_cmos_sensor(0x030D,0x0F);
	write_cmos_sensor(0x030E,0x03);
	write_cmos_sensor(0x030F,0x39);
	write_cmos_sensor(0x0310,0x01);

	write_cmos_sensor(0x0820,0x14);
	write_cmos_sensor(0x0821,0xA0);
	write_cmos_sensor(0x0822,0x00);
	write_cmos_sensor(0x0823,0x00);

	write_cmos_sensor(0x0202,0x0C);
	write_cmos_sensor(0x0203,0x18);

	write_cmos_sensor(0x0204,0x00);
	write_cmos_sensor(0x0205,0x00);
	write_cmos_sensor(0x020E,0x01);
	write_cmos_sensor(0x020F,0x00);
	write_cmos_sensor(0x0210,0x01);
	write_cmos_sensor(0x0211,0x00);
	write_cmos_sensor(0x0212,0x01);
	write_cmos_sensor(0x0213,0x00);
	write_cmos_sensor(0x0214,0x01);
	write_cmos_sensor(0x0215,0x00);

	write_cmos_sensor(0x0100,0x01);
	#endif
}
#endif

static void hs_video_setting(void)
{ 
	printk("hs_video_setting enter!\n");
	write_cmos_sensor(0x0100,0x00);

//VGA 120fps
	write_cmos_sensor(0x0114,0x03);
	write_cmos_sensor(0x0340,0x02);
	write_cmos_sensor(0x0341,0x44);
	write_cmos_sensor(0x0342,0x18);
	write_cmos_sensor(0x0343,0x00);
	write_cmos_sensor(0x0344,0x00);
	write_cmos_sensor(0x0345,0x00);
	write_cmos_sensor(0x0346,0x02);
	write_cmos_sensor(0x0347,0x18);
	write_cmos_sensor(0x0348,0x14);
	write_cmos_sensor(0x0349,0xDF);
	write_cmos_sensor(0x034A,0x09);
	write_cmos_sensor(0x034B,0x97);
	write_cmos_sensor(0x0381,0x01);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0385,0x01);
	write_cmos_sensor(0x0387,0x01);
	write_cmos_sensor(0x0900,0x01);
	write_cmos_sensor(0x0901,0x44);
	write_cmos_sensor(0x0902,0x00);
	write_cmos_sensor(0x3029,0x00);
	write_cmos_sensor(0x305C,0x11);

	write_cmos_sensor(0x0112,0x0A);
	write_cmos_sensor(0x0113,0x0A);
	write_cmos_sensor(0x034C,0x02);
	write_cmos_sensor(0x034D,0x80);
	write_cmos_sensor(0x034E,0x01);
	write_cmos_sensor(0x034F,0xE0);
	write_cmos_sensor(0x0401,0x00);
	write_cmos_sensor(0x0404,0x00);
	write_cmos_sensor(0x0405,0x10);
	write_cmos_sensor(0x0408,0x01);
	write_cmos_sensor(0x0409,0x5C);
	write_cmos_sensor(0x040A,0x00);
	write_cmos_sensor(0x040B,0x00);
	write_cmos_sensor(0x040C,0x02);
	write_cmos_sensor(0x040D,0x80);
	write_cmos_sensor(0x040E,0x01);
	write_cmos_sensor(0x040F,0xE0);

	write_cmos_sensor(0x0301,0x04);
	write_cmos_sensor(0x0303,0x02);
	write_cmos_sensor(0x0305,0x04);
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0x8F);
	write_cmos_sensor(0x0309,0x0A);
	write_cmos_sensor(0x030B,0x01);
	write_cmos_sensor(0x030D,0x0F);
	write_cmos_sensor(0x030E,0x00);
	write_cmos_sensor(0x030F,0xFA);
	write_cmos_sensor(0x0310,0x01);

	write_cmos_sensor(0x0820,0x06);
	write_cmos_sensor(0x0821,0x40);
	write_cmos_sensor(0x0822,0x00);
	write_cmos_sensor(0x0823,0x00);

	write_cmos_sensor(0x0202,0x02);
	write_cmos_sensor(0x0203,0x3A);

	write_cmos_sensor(0x0204,0x00);
	write_cmos_sensor(0x0205,0x00);
	write_cmos_sensor(0x020E,0x01);
	write_cmos_sensor(0x020F,0x00);
	write_cmos_sensor(0x0210,0x01);
	write_cmos_sensor(0x0211,0x00);
	write_cmos_sensor(0x0212,0x01);
	write_cmos_sensor(0x0213,0x00);
	write_cmos_sensor(0x0214,0x01);
	write_cmos_sensor(0x0215,0x00);

	write_cmos_sensor(0x0100,0x01);
}


static void slim_video_setting(void)
{
	printk("slim_video_setting enter!\n");
	write_cmos_sensor(0x0100,0x00);
	
	write_cmos_sensor(0x0114,0x03);
	write_cmos_sensor(0x0340,0x06);
	write_cmos_sensor(0x0341,0x18);
	write_cmos_sensor(0x0342,0x18);
	write_cmos_sensor(0x0343,0x00);
	write_cmos_sensor(0x0344,0x00);
	write_cmos_sensor(0x0345,0x00);
	write_cmos_sensor(0x0346,0x00);
	write_cmos_sensor(0x0347,0x00);
	write_cmos_sensor(0x0348,0x14);
	write_cmos_sensor(0x0349,0xDF);
	write_cmos_sensor(0x034A,0x0B);
	write_cmos_sensor(0x034B,0xB7);
	write_cmos_sensor(0x0381,0x01);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0385,0x01);
	write_cmos_sensor(0x0387,0x01);
	write_cmos_sensor(0x0900,0x01);
	write_cmos_sensor(0x0901,0x22);
	write_cmos_sensor(0x0902,0x00);
	write_cmos_sensor(0x3029,0x00);
	write_cmos_sensor(0x305C,0x11);

	write_cmos_sensor(0x0112,0x0A);
	write_cmos_sensor(0x0113,0x0A);
	write_cmos_sensor(0x034C,0x0A);
	write_cmos_sensor(0x034D,0x70);
	write_cmos_sensor(0x034E,0x05);
	write_cmos_sensor(0x034F,0xDC);
	write_cmos_sensor(0x0401,0x00);
	write_cmos_sensor(0x0404,0x00);
	write_cmos_sensor(0x0405,0x10);
	write_cmos_sensor(0x0408,0x00);
	write_cmos_sensor(0x0409,0x00);
	write_cmos_sensor(0x040A,0x00);
	write_cmos_sensor(0x040B,0x00);
	write_cmos_sensor(0x040C,0x0A);
	write_cmos_sensor(0x040D,0x70);
	write_cmos_sensor(0x040E,0x05);
	write_cmos_sensor(0x040F,0xDC);

	write_cmos_sensor(0x0301,0x04);
	write_cmos_sensor(0x0303,0x02);
	write_cmos_sensor(0x0305,0x04);
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0x60);
	write_cmos_sensor(0x0309,0x0A);
	write_cmos_sensor(0x030B,0x01);
	write_cmos_sensor(0x030D,0x0F);
	write_cmos_sensor(0x030E,0x00);
	write_cmos_sensor(0x030F,0xFA);
	write_cmos_sensor(0x0310,0x01);

	write_cmos_sensor(0x0820,0x06);
	write_cmos_sensor(0x0821,0x40);
	write_cmos_sensor(0x0822,0x00);
	write_cmos_sensor(0x0823,0x00);

	write_cmos_sensor(0x0202,0x06);
	write_cmos_sensor(0x0203,0x0E);

	write_cmos_sensor(0x0204,0x00);
	write_cmos_sensor(0x0205,0x00);
	write_cmos_sensor(0x020E,0x01);
	write_cmos_sensor(0x020F,0x00);
	write_cmos_sensor(0x0210,0x01);
	write_cmos_sensor(0x0211,0x00);
	write_cmos_sensor(0x0212,0x01);
	write_cmos_sensor(0x0213,0x00);
	write_cmos_sensor(0x0214,0x01);
	write_cmos_sensor(0x0215,0x00);										  
	
	write_cmos_sensor(0x0100,0x01);

}



/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID 
*
* PARAMETERS
*	*sensorID : return the sensor ID 
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id) 
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			//<2015/10/15-kylechang, Porting Sub Camera Driver IMX234
			//*sensor_id =  ((read_cmos_sensor(0x0016)&0x0F)<<8) + read_cmos_sensor(0x0017);
			*sensor_id = return_sensor_id();
			//>2015/10/15-kylechang
			printk(" **imgsensor_info.sensor_id = 0x%x  *sensor_id = 0x%x\n",imgsensor_info.sensor_id,*sensor_id);
			if ((*sensor_id) == (imgsensor_info.sensor_id)) {				
				printk("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);	  
				return ERROR_NONE;
			}else{	
				printk("Read sensor id fail,write id: 0x%x,sensor_id 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
			}
			retry--;
		} while(retry > 0);

	//<2015/10/15-kylechang, Porting Sub Camera Driver IMX234
	//for debug test
	#if 0
	if(imgsensor.ihdr_en==1)
	{
 		do {
 			//*sensor_id =  ((read_cmos_sensor(0x0016)&0x0F)<<8) + read_cmos_sensor(0x0017);
 			//*sensor_id = return_sensor_id();
 			//if (*sensor_id == imgsensor_info.sensor_id)
			//{				
 			//	LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);	  
 			//	return ERROR_NONE;
 			//}	
 			//LOG_INF("++LoopIMX234+++Read SensorId fail,write id: 0x%x,sensor_id 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
			LOG_INF("++LoopIMX234+++Write write id: 0x%x, Write 0x0100\n", imgsensor.i2c_write_id);
			write_cmos_sensor(0x0100, 0x00);
 		} while(1);
	}
	imgsensor.ihdr_en=imgsensor.ihdr_en+1;
	printk("+++KYLE+++ [IMX234]  ihdr_en=%d, i=%d\n", (int)imgsensor.ihdr_en, (int)i);
	#endif
	//>2015/10/15-kylechang
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		// if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF 
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
	//const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2};
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0; 

	printk("preview 1280*960@30fps,864Mbps/lane; video 1280*960@30fps,864Mbps/lane; capture 5M@30fps,864Mbps/lane\n");
	
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			//<2015/10/15-kylechang, Porting Sub Camera Driver IMX234
			//sensor_id =  ((read_cmos_sensor(0x0016)&0x0F)<<8) + read_cmos_sensor(0x0017);
			sensor_id = return_sensor_id();
			//>2015/10/15-kylechang
			if (sensor_id == imgsensor_info.sensor_id) {				
				printk("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);	  
				break;
			}	
			printk("Read sensor id fail, write id:0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}		 
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;
	
	/* initail sequence write in  */
	sensor_init();

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en= KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x4C00;
	imgsensor.gain = 0x0200;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}	/*	open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*	
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	printk("imx234mipi_sensor close\n");

	/*No Need to implement this function*/ 
	
	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk("imx234mipi_sensor preview\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength; 
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk("imx234mipi_sensor capture\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
            printk("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap.max_framerate/10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}

	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor.current_fps); 
	
	
	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk("imx234mipi_sensor normal_video\n");

	#if  1//half size
	//<2015/12/31-SimonLin, for saving electricity
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.halfsize_video.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.halfsize_video.linelength;
	imgsensor.frame_length = imgsensor_info.halfsize_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.halfsize_video.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	//normal_video_setting(0);

	#if SUB_HALFSIZE
	halfsize_video_setting(imgsensor.current_fps);
	#elif SUB_FULLSIZE
	normal_video_setting(0);
	#elif SUB_BINNING
	binning_video_setting(imgsensor.current_fps);
	#endif
	#else//org size
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;  
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	#endif
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk("imx234mipi_sensor hs_video\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	
	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk("imx234mipi_sensor slim_video\n");
	printk("imgsensor_info.slim_video.linelength = %d\n",imgsensor_info.slim_video.linelength);
	printk("imgsensor_info.slim_video.framelength = %d\n",imgsensor_info.slim_video.framelength);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	
	return ERROR_NONE;
}	/*	slim_video	 */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{

	printk("imx234mipi_sensor get_resolution\n");
	printk("imgsensor_info.slim_video.grabwindow_width = %d\n",imgsensor_info.slim_video.grabwindow_width);
	printk("imgsensor_info.slim_video.grabwindow_height = %d\n",imgsensor_info.slim_video.grabwindow_height);
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;
	
	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.halfsize_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.halfsize_video.grabwindow_height;		

	
	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.hs_video.grabwindow_height;
	
	sensor_resolution->SensorSlimVideoWidth	 = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;
	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk("scenario_id = %d\n", scenario_id);

	
	//sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
	//sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
    //imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

    sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
    sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
    sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorInterruptDelayLines = 4; /* not use */
    sensor_info->SensorResetActiveHigh = FALSE; /* not use */
    sensor_info->SensorResetDelayCount = 5; /* not use */

    sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
    sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
    sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;

	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;
	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame; 
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame; 
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
	
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame; 		 /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;	
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num; 
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */
	
	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;	// 0 is default 1x 
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;		
			
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			sensor_info->SensorGrabStartX = imgsensor_info.cap.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc; 

			break;	 
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			
			sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
	   
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc; 

			break;	  
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:			
			sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc; 

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc; 

			break;
		default:			
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;		
			
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}
	
	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			preview(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			capture(image_window, sensor_config_data);
			break;	
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video(image_window, sensor_config_data);
			break;	  
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			hs_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			slim_video(image_window, sensor_config_data);
			break;	  
		default:
			printk("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	printk("framerate = %d\n ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps,1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	printk("enable = %d, framerate = %d \n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) //enable auto flicker	  
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate) 
{
	kal_uint32 frame_length;
  
	printk("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(framerate == 0)
				return ERROR_NONE;
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;			
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:		
        	  if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
                frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            } else {
        		    if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
                    printk("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate/10);
                frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
            }
            //set_dummy();
			break;	
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;	
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
			break;		
		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
			printk("error scenario_id = %d, we use preview scenario \n", scenario_id);
			break;
	}	
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate) 
{
	printk("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*framerate = imgsensor_info.pre.max_framerate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*framerate = imgsensor_info.normal_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*framerate = imgsensor_info.cap.max_framerate;
			break;		
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*framerate = imgsensor_info.hs_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO: 
			*framerate = imgsensor_info.slim_video.max_framerate;
			break;
		default:
			break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	printk("enable: %d\n", enable);

	if (enable) {
		write_cmos_sensor(0x0601,0x02);
	} else {
		write_cmos_sensor(0x0601,0x00);
	}	 
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
							 UINT8 *feature_para,UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16=(UINT16 *) feature_para;
	UINT16 *feature_data_16=(UINT16 *) feature_para;
	UINT32 *feature_return_para_32=(UINT32 *) feature_para;
	UINT32 *feature_data_32=(UINT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;
    //unsigned long long *feature_return_para=(unsigned long long *) feature_para;
	
	SENSOR_WINSIZE_INFO_STRUCT *wininfo;	
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
 
	printk("feature_id = %d\n", feature_id);
	switch (feature_id) {
		case SENSOR_FEATURE_GET_PERIOD:
			*feature_return_para_16++ = imgsensor.line_length;
			*feature_return_para_16 = imgsensor.frame_length;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:	 
			*feature_return_para_32 = imgsensor.pclk;
			*feature_para_len=4;
			break;		   
		case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			night_mode((BOOL) *feature_data);
			break;
		case SENSOR_FEATURE_SET_GAIN:		
			set_gain((UINT16) *feature_data_16);
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
			break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
			break; 
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			get_imgsensor_id(feature_return_para_32); 
			break; 
		case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing			 
			*feature_return_para_32 = imgsensor_info.checksum_value;
			*feature_para_len=4;							 
			break;				
		case SENSOR_FEATURE_SET_FRAMERATE:
            printk("current fps :%d\n", (UINT32)*feature_data);
			spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
			spin_unlock(&imgsensor_drv_lock);		
			break;
		case SENSOR_FEATURE_SET_HDR:
			//LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data_16);
			printk("Warning! Not Support IHDR Feature");
			spin_lock(&imgsensor_drv_lock);
			//imgsensor.ihdr_en = (BOOL)*feature_data_16;
            imgsensor.ihdr_en = KAL_FALSE;
			spin_unlock(&imgsensor_drv_lock);		
			break;
		case SENSOR_FEATURE_GET_CROP_INFO:
            printk("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);
            wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
		
			switch (*feature_data_32) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;	  
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
			}
            break;
		case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            printk("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
			break;
		default:
			break;
	}
  
	return ERROR_NONE;
}	/*	feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 IMX234_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	printk("simon IMX234_MIPI_RAW_SensorInit\n");
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	IMX234_MIPI_RAW_SensorInit	*/
