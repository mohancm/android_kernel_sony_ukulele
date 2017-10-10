/*****************************************************************************
 *
 * Filename:
 * ---------
 *     IMX230mipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
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
//#include <asm/system.h>
//<2015/11/15-kylechang, for Android M
//#include <linux/xlog.h>
//#include "kd_camera_hw.h"
#include <linux/types.h>
#include "kd_camera_typedef.h"
//>2015/10/15-kylechang
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "imx230mipi_Sensor.h"

/****************************Modify Following Strings for Debug****************************/
#define PFX "IMX230_camera_sensor"
#define LOG_1 printk("IMX230,MIPI 4LANE\n")
#define LOG_2 printk("preview 2672*2008@30fps; video 5344*4016@30fps; capture 21M@24fps\n")
/****************************   Modify end    *******************************************/
//<2015/11/16-kylechang, printk is replaced by printk
//#define printk(format, args...)    pr_debug(PFX, "[%s] " format, __FUNCTION__, ##args)
//>2015/11/16-kylechang
static DEFINE_SPINLOCK(imgsensor_drv_lock);

//<2015/11/02-kylechang, Add long exposure feature
kal_bool Is_longExp_1=0;
//>2015/11/02-kylechang

//
#define BYTE unsigned char
static BOOL read_spc_flag = FALSE;
static BYTE imx230_SPC_data[352]={0};
static BYTE eeprom_version[2]={0};





static imgsensor_info_struct imgsensor_info = {
    .sensor_id = IMX230_SENSOR_ID,        //record sensor id defined in Kd_imgsensor.h

    .checksum_value = 0xafd83a68,        //checksum value for Camera Auto Test

    .pre = {
        .pclk = 381000000,                //record different mode's pclk
        .linelength = 6024,                //record different mode's linelength
        .framelength = 2108,            //record different mode's framelength
        .startx = 0,                    //record different mode's startx of grabwindow
        .starty = 0,                    //record different mode's starty of grabwindow
        .grabwindow_width = 2672,        //record different mode's width of grabwindow
        .grabwindow_height = 2008,        //record different mode's height of grabwindow
        /*     following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario    */
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        /*     following for GetDefaultFramerateByScenario()    */
        .max_framerate = 300,
    },
    .cap = {   
        .pclk = 597000000,
        .linelength = 6024,
        .framelength = 4126,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 5344,
        .grabwindow_height = 4016,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 240,
    },
    .cap1 = {                            //capture for PIP 24fps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
        .pclk = 375000000,
        .linelength = 6024,
        .framelength = 4126,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 5344,
        .grabwindow_height = 4016,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 150,    //less than 13M(include 13M),cap1 max framerate is 24fps,16M max framerate is 20fps, 20M max framerate is 15fps
    },
    .normal_video = {
		 #if MAIN_HALFSIZE
        .pclk = 381000000,                //record different mode's pclk
        .linelength = 6024,                //record different mode's linelength
        .framelength = 2108,            //record different mode's framelength
        .startx = 0,                    //record different mode's startx of grabwindow
        .starty = 0,                    //record different mode's starty of grabwindow
        .grabwindow_width = 2672,        //record different mode's width of grabwindow
        .grabwindow_height = 2008,        //record different mode's height of grabwindow
        /*     following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario    */
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        /*     following for GetDefaultFramerateByScenario()    */
        .max_framerate = 300,
		#elif MAIN_FULLSIZE
        .pclk = 564000000,
        .linelength = 6024,
        .framelength = 3120,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 5344,
        .grabwindow_height = 3006,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 300,
		#endif
    },
    .hs_video = {
        .pclk = 597000000,
        .linelength = 6024,
        .framelength = 824,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1280,
        .grabwindow_height = 720, 
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 1200,
    },
    .slim_video = {
        .pclk = 201000000,
        .linelength = 6024,
        .framelength = 1112,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1280,
        .grabwindow_height = 720,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 300,
    },
    .margin = 4,            //sensor framelength & shutter margin
    .min_shutter = 1,        //min shutter
    .max_frame_length = 0x7fff,//max framelength by sensor register's limitation
    .ae_shut_delay_frame = 0,    //shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
    .ae_sensor_gain_delay_frame = 0,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
    .ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
    .ihdr_support = 0,      //1, support; 0,not support
    .ihdr_le_firstline = 0,  //1,le first ; 0, se first
    .sensor_mode_num = 5,      //support sensor mode num

    .cap_delay_frame = 2,        //enter capture delay frame num
    .pre_delay_frame = 2,         //enter preview delay frame num
    .video_delay_frame = 1,        //enter video delay frame num
    .hs_video_delay_frame = 3,    //enter high speed video  delay frame num
    .slim_video_delay_frame = 3,//enter slim video delay frame num

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
    .mirror = IMAGE_NORMAL,                //mirrorflip information
    .sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
    .shutter = 0x3D0,                    //current shutter
    .gain = 0x100,                        //current gain
    .dummy_pixel = 0,                    //current dummypixel
    .dummy_line = 0,                    //current dummyline
    .current_fps = 300,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
    .autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
    .test_pattern = KAL_FALSE,        //test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
    .ihdr_mode = 0, //sensor need support LE, SE with HDR feature
    .i2c_write_id = 0x6c,//record current sensor's i2c write id
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =
{
 { 5344, 4016,    0,    0, 5344, 4016, 2572, 2008, 0000, 0000, 2572, 2008,      0,    0, 2572, 2008}, // Preview
	
 { 5344, 4016,    0,    0, 5344, 4016, 5344, 4016, 0000, 0000, 5344, 4016,      0,    0, 5344, 4016}, // capture
 
 #if MAIN_HALFSIZE
 { 5344, 4016,    0,    0, 5344, 4016, 2572, 2008, 0000, 0000, 2572, 2008,      0,    0, 2572, 2008}, // Preview
 #elif MAIN_FULLSIZE
 { 5344, 4016,    0,  504, 5344, 3006, 5344, 3006, 0000, 0000, 5344, 3006,      0,    0, 5344, 3006}, 
 #endif
  
 
 { 5344, 4016,    0,  568, 5344, 2880, 1280,  720, 0000, 0000, 1280,  720,      0,    0, 1280,  720}, //hight speed video
 { 5344, 4016,    0,  504, 5344, 3006, 1280,  720, 0000, 0000, 1280,  720,      0,    0, 1280,  720}};// slim video

//
/*VC1 for HDR(DT=0X35) , VC2 for PDAF(DT=0X36), unit : 8bit*/
 static SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[3]=
 {/* Preview mode setting */
 {0x03, 0x0a,   0x00,   0x08, 0x40, 0x00, 
  0x00, 0x2b, 0x0A70, 0x07D8, 0x00, 0x35, 0x0280, 0x0001,
  //0x00, 0x00, 0x00, 0x000, 0x0, 0x00, 0x0000, 0x0000},
  0x00, 0x36, 0x0C48, 0x0001, 0x03, 0x00, 0x0000, 0x0000},
  /* Capture mode setting */
  {0x03, 0x0a,	 0x00,	 0x08, 0x40, 0x00,
   0x00, 0x2b, 0x14E0, 0x0FB0, 0x00, 0x35, 0x0280, 0x0001,
   //0x00, 0x00, 0x00, 0x000, 0x0, 0x00, 0x0000, 0x0000},
  0x00, 0x36, 0x1a18, 0x0001, 0x03, 0x00, 0x0000, 0x0000},
   /* Video mode setting */
  {0x02, 0x0a,	 0x00,	 0x08, 0x40, 0x00,
   0x00, 0x2b, 0x14E0, 0x0FB0, 0x01, 0x00, 0x0000, 0x0000,
   //0x00, 0x00, 0x00, 0x000, 0x0, 0x00, 0x0000, 0x0000},
   0x00, 0x36, 0x1a18, 0x0001, 0x03, 0x00, 0x0000, 0x0000}
};

//typedef struct
//{
//    MUINT16 DarkLimit_H;
//    MUINT16 DarkLimit_L;
//    MUINT16 OverExp_Min_H;
//    MUINT16 OverExp_Min_L;
//    MUINT16 OverExp_Max_H;
//    MUINT16 OverExp_Max_L;
//}SENSOR_ATR_INFO, *pSENSOR_ATR_INFO;


//static SENSOR_ATR_INFO sensorATR_Info[4]=    
//{/* Strength Range Min */
  //  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    //* Strength Range Std */
  //  {0x00, 0x32, 0x00, 0x3c, 0x03, 0xff},
    //* Strength Range Max */
  //  {0x3f, 0xff, 0x3f, 0xff, 0x3f, 0xff},
    //* Strength Range Custom */
  //  {0x3F, 0xFF, 0x00, 0x0, 0x3F, 0xFF}};


#define IMX230MIPI_MaxGainIndex (115)
kal_uint16 IMX230MIPI_sensorGainMapping[IMX230MIPI_MaxGainIndex][2] ={	
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
	{97 ,175},
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
	{132,264},
	{133,266},
	{135,269},
	{136,271},
	{138,274},
	{139,276},
	{141,279},
	{142,282},
	{144,285},
	{145,286},
	{147,290},
	{149,292},
	{150,294},
	{155,300},
	{157,303},
	{158,305},
	{161,309},
	{163,311},
	{170,319},
	{172,322},
	{174,324},
	{176,326},
	{179,329},
	{181,331},
	{185,335},
	{189,339},
	{193,342},
	{195,344},
	{196,345},
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

//<2016/01/08 ShermanWei,ISO12800
#define IMX230MIPI_MaxGainIndex2 (152)
kal_uint16 IMX230MIPI_sensorGainMapping2[IMX230MIPI_MaxGainIndex2][2] ={
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
//>2016/01/08 ShermanWei,

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

/*static kal_uint32 imx230_ATR(UINT16 DarkLimit, UINT16 OverExp)
{
	
    write_cmos_sensor(0x6e50,sensorATR_Info[DarkLimit].DarkLimit_H);
    write_cmos_sensor(0x6e51,sensorATR_Info[DarkLimit].DarkLimit_L);
    write_cmos_sensor(0x9340,sensorATR_Info[OverExp].OverExp_Min_H);
    write_cmos_sensor(0x9341,sensorATR_Info[OverExp].OverExp_Min_L);
    write_cmos_sensor(0x9342,sensorATR_Info[OverExp].OverExp_Max_H);
    write_cmos_sensor(0x9343,sensorATR_Info[OverExp].OverExp_Max_L);
    write_cmos_sensor(0x9706,0x10);
    write_cmos_sensor(0x9707,0x03);
    write_cmos_sensor(0x9708,0x03);
    write_cmos_sensor(0x9e24,0x00);
    write_cmos_sensor(0x9e25,0x8c);
    write_cmos_sensor(0x9e26,0x00);
    write_cmos_sensor(0x9e27,0x94);
    write_cmos_sensor(0x9e28,0x00);
    write_cmos_sensor(0x9e29,0x96);
    printk("DarkLimit 0x6e50(0x%x), 0x6e51(0x%x)\n",sensorATR_Info[DarkLimit].DarkLimit_H,
                                                     sensorATR_Info[DarkLimit].DarkLimit_L);
    printk("OverExpMin 0x9340(0x%x), 0x9341(0x%x)\n",sensorATR_Info[OverExp].OverExp_Min_H,
                                                     sensorATR_Info[OverExp].OverExp_Min_L);
    printk("OverExpMin 0x9342(0x%x), 0x9343(0x%x)\n",sensorATR_Info[OverExp].OverExp_Max_H,
                                                     sensorATR_Info[OverExp].OverExp_Max_L);
                                                     
    return ERROR_NONE; 
}*/

//Get LiteOn module version
static kal_uint16 get_sensor_version(void)
{
//	int version = 0;
	kal_uint16 version = 0;

	read_eeprom_version(eeprom_version);
	//printk("[Rick] After, get_sensor_version 0x%x \n", eeprom_version[0]);
	/* NEW version //
	New LiteOn: [0]:0xc, [1]:0x1
	Truly: [0]:0x1, [1]:0x2
	*/
	if(eeprom_version[0] == 0xc && eeprom_version[1] == 0x1){ // new LiteOn
		version = 1;
	}
	else if(eeprom_version[0] == 0x1 && eeprom_version[1] == 0x2){ //Turly
		version = 1;
	}
	else{ // old LiteOn
		version = 0;
	}
	return version;
}
//
static void imx230_apply_SPC(void)
{
	unsigned int start_reg = 0x7c00;
	int i;
	//printk("[Rick] imx230_apply_SPC A \n");
	if(read_spc_flag == FALSE)
	{
		read_imx230_SPC(imx230_SPC_data);
		read_spc_flag = TRUE;
		//printk("[Rick] imx230_apply_SPC B \n");
		return;
	}

	for(i=0;i<352;i++)
	{
		write_cmos_sensor(start_reg, imx230_SPC_data[i]);
		//printk("[Rick] imx230_apply_SPC C \n");	
		start_reg++;
	}		
}
//

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
}    /*    set_dummy  */

static kal_uint32 return_sensor_id(void)
{
	write_cmos_sensor(0x0A02, 0x1F);
	write_cmos_sensor(0x0A00, 0x01);
	write_cmos_sensor(0x0A01, 0x01);
    return ((read_cmos_sensor(0x0A38) << 4) | (read_cmos_sensor(0x0A39)>>4));
}
static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
    kal_uint32 frame_length = imgsensor.frame_length;
    //unsigned long flags;

    printk("framerate = %d, min framelength should enable=%d\n", framerate,min_framelength_en);

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
}    /*    set_max_framerate  */



/*************************************************************************
* FUNCTION
*    set_shutter
*
* DESCRIPTION
*    This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*    iShutter : exposured lines
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
//<2015/11/02-kylechang, Add long exposure feature
static void set_shutter(kal_uint32 shutter)
//>2015/11/02-kylechang
{
    unsigned long flags;
    kal_uint16 realtime_fps = 0;

	//<2015/11/02-kylechang, Add long exposure feature
	//kal_uint32 long_time=0; 
    //kal_uint32 long_shutter=0;
    kal_uint16 Base_FrameLength1=0xc18f;//0.5s
    kal_uint32 Base_FrameLength2=0x1831e;//1s
    kal_uint16 Multi=0; 
	//>2015/11/02-kylechang
	
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    //write_shutter(shutter);
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
	   Is_longExp_1=1;

	if(Is_longExp_1)
	{  
	
		 Multi=shutter/Base_FrameLength2;
	   
	printk("IMX230.Multi=%d \n ", Multi);
	 
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
	Is_longExp_1=0;
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


}    /*    set_shutter */



static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint8 iI;
    printk("[IMX230MIPI]enter IMX230MIPIGain2Reg function\n");
    for (iI = 0; iI < IMX230MIPI_MaxGainIndex; iI++) 
	{
		if(gain < IMX230MIPI_sensorGainMapping[iI][0])
		{                
			return IMX230MIPI_sensorGainMapping[iI][1];       
		}
			

    }
	if(iI != IMX230MIPI_MaxGainIndex)
	{
    	if(gain != IMX230MIPI_sensorGainMapping[iI][0])
    	{
        	 printk("Gain mapping don't correctly:%d %d \n", gain, IMX230MIPI_sensorGainMapping[iI][0]);
    	}
	}
	printk("exit IMX230MIPIGain2Reg function\n");
    return IMX230MIPI_sensorGainMapping[iI-1][1];
}

//<2016/01/08 ShermanWei,ISO12800

static kal_uint16 gain2reg2(const kal_uint16 gain)
{
    kal_uint8 i;

    for (i = 0; i < IMX230MIPI_MaxGainIndex2; i++) { 
    	if(gain <= IMX230MIPI_sensorGainMapping2[i][0])
		{                
			return IMX230MIPI_sensorGainMapping2[i][1];       
		}
    	
	}
    printk("Gain mapping don't correctly:%d index=%d \n", gain, i);
    return IMX230MIPI_sensorGainMapping2[i-1][1];
}

//>2016/01/08 ShermanWei,
/*************************************************************************
* FUNCTION
*    set_gain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    iGain : sensor global gain(base: 0x40)
*
* RETURNS
*    the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
#if 0
    kal_uint16 reg_gain;

    /* 0x350A[0:1], 0x350B[0:7] AGC real gain */
    /* [0:3] = N meams N /16 X    */
    /* [4:9] = M meams M X         */
    /* Total gain = M + N /16 X   */

    //
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
#else/////<2016/01/08 ShermanWei,ISO12800
    kal_uint16 reg_gain;
	kal_uint16 gain2;
	kal_uint16 reg_gain2;

    /* 0x350A[0:1], 0x350B[0:7] AGC real gain */
    /* [0:3] = N meams N /16 X    */
    /* [4:9] = M meams M X         */
    /* Total gain = M + N /16 X   */

    //
	gain2 = gain / 8;
	if (gain < BASEGAIN || gain > 160 * BASEGAIN) {
        printk("Error gain setting");

        if (gain < BASEGAIN)
            gain = BASEGAIN;
		else if (gain > 160 * BASEGAIN)
			gain = 160 * BASEGAIN;		 
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
		gain2 = gain / 8;
		//#if 1
		reg_gain2 = gain2reg2(gain2);
		//#else
		//reg_gain2 = (((kal_uint8)(gain / BASEGAIN)) << 8) || ((kal_uint8)(0x20 * (gain - gain / BASEGAIN) / (BASEGAIN / 8) ));
		//#endif
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
//>2016/01/08 ShermanWei,

}
/*    set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{

    kal_uint16 realtime_fps = 0;
    kal_uint16 reg_gain;
    printk("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
    spin_lock(&imgsensor_drv_lock);
    if (le > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = le + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);
    if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
    if (imgsensor.autoflicker_en) { 
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
        if(realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296,0);
        else if(realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146,0);
        else {
        //write_cmos_sensor(0x0104, 0x01); 
        write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
        write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
        //write_cmos_sensor(0x0104, 0x00);
        }
    } else {
        //write_cmos_sensor(0x0104, 0x01); 
        write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
        write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
        //write_cmos_sensor(0x0104, 0x00);
    }
    //write_cmos_sensor(0x0104, 0x01);
    /* Long exposure */
    write_cmos_sensor(0x0202, (le >> 8) & 0xFF);
    write_cmos_sensor(0x0203, le  & 0xFF);
    /* Short exposure */
    write_cmos_sensor(0x0224, (se >> 8) & 0xFF);
    write_cmos_sensor(0x0225, se  & 0xFF); 
    reg_gain = gain2reg(gain);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_gain; 
    spin_unlock(&imgsensor_drv_lock);
    /* Global analog Gain for Long expo*/
    write_cmos_sensor(0x0204, (reg_gain>>8)& 0xFF);
    write_cmos_sensor(0x0205, reg_gain & 0xFF);
    /* Global analog Gain for Short expo*/
    write_cmos_sensor(0x0216, (reg_gain>>8)& 0xFF);
    write_cmos_sensor(0x0217, reg_gain & 0xFF);
    //write_cmos_sensor(0x0104, 0x00);

}



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
			write_cmos_sensor(0x3A27, 0x00);
			write_cmos_sensor(0x3A28, 0x00);
			write_cmos_sensor(0x3A29, 0x01);
			write_cmos_sensor(0x3A2A, 0x00);
			write_cmos_sensor(0x3A2B, 0x00);
			write_cmos_sensor(0x3A2C, 0x00);
			write_cmos_sensor(0x3A2D, 0x01);
			write_cmos_sensor(0x3A2E, 0x01);
			break;
        case IMAGE_H_MIRROR:
			write_cmos_sensor(0x0101, 0x01);
			write_cmos_sensor(0x3A27, 0x01);
			write_cmos_sensor(0x3A28, 0x01);
			write_cmos_sensor(0x3A29, 0x00);
			write_cmos_sensor(0x3A2A, 0x00);
			write_cmos_sensor(0x3A2B, 0x01);
			write_cmos_sensor(0x3A2C, 0x00);
			write_cmos_sensor(0x3A2D, 0x00);
			write_cmos_sensor(0x3A2E, 0x01);
            break;
        case IMAGE_V_MIRROR:
			write_cmos_sensor(0x0101, 0x02);
			write_cmos_sensor(0x3A27, 0x10);
			write_cmos_sensor(0x3A28, 0x10);
			write_cmos_sensor(0x3A29, 0x01);
			write_cmos_sensor(0x3A2A, 0x01);
			write_cmos_sensor(0x3A2B, 0x00);
			write_cmos_sensor(0x3A2C, 0x01);
			write_cmos_sensor(0x3A2D, 0x01);
			write_cmos_sensor(0x3A2E, 0x00);
            break;
        case IMAGE_HV_MIRROR:
			write_cmos_sensor(0x0101, 0x03);
			write_cmos_sensor(0x3A27, 0x11);
			write_cmos_sensor(0x3A28, 0x11);
			write_cmos_sensor(0x3A29, 0x00);
			write_cmos_sensor(0x3A2A, 0x01);
			write_cmos_sensor(0x3A2B, 0x01);
			write_cmos_sensor(0x3A2C, 0x01);
			write_cmos_sensor(0x3A2D, 0x00);
			write_cmos_sensor(0x3A2E, 0x00);
            break;
        default:
            printk("Error image_mirror setting\n");
    }
}


/*************************************************************************
* FUNCTION
*    night_mode
*
* DESCRIPTION
*    This function night mode of sensor.
*
* PARAMETERS
*    bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}    /*    night_mode    */

static void sensor_init(void)
{
	 printk("IMX230 sensor_init start\n"); 
	//DPC2D Setting
write_cmos_sensor(0x68A9,0x00);
write_cmos_sensor(0x68C5,0x00);
write_cmos_sensor(0x68DF,0x00);
write_cmos_sensor(0x6953,0x01);
write_cmos_sensor(0x6962,0x3A);
write_cmos_sensor(0x69CD,0x3A);
write_cmos_sensor(0x9258,0x00);
write_cmos_sensor(0x933A,0x02);
write_cmos_sensor(0x933B,0x02);
write_cmos_sensor(0x934B,0x1B);
write_cmos_sensor(0x934C,0x0A);
write_cmos_sensor(0x9356,0x8C);
write_cmos_sensor(0x9357,0x50);
write_cmos_sensor(0x9358,0x1B);
write_cmos_sensor(0x9359,0x8C);
write_cmos_sensor(0x935A,0x1B);
write_cmos_sensor(0x935B,0x0A);
write_cmos_sensor(0x940D,0x07);
write_cmos_sensor(0x940E,0x07);
write_cmos_sensor(0x9414,0x06);
write_cmos_sensor(0x945B,0x07);
write_cmos_sensor(0x945D,0x07);
write_cmos_sensor(0x9901,0x35);
write_cmos_sensor(0x9903,0x23);
write_cmos_sensor(0x9905,0x23);
write_cmos_sensor(0x9906,0x00);
write_cmos_sensor(0x9907,0x31);
write_cmos_sensor(0x9908,0x00);
write_cmos_sensor(0x9909,0x1B);
write_cmos_sensor(0x990A,0x00);
write_cmos_sensor(0x990B,0x15);
write_cmos_sensor(0x990D,0x3F);
write_cmos_sensor(0x990F,0x3F);
write_cmos_sensor(0x9911,0x3F);
write_cmos_sensor(0x9913,0x64);
write_cmos_sensor(0x9915,0x64);
write_cmos_sensor(0x9917,0x64);
write_cmos_sensor(0x9919,0x50);
write_cmos_sensor(0x991B,0x60);
write_cmos_sensor(0x991D,0x65);
write_cmos_sensor(0x991F,0x01);
write_cmos_sensor(0x9921,0x01);
write_cmos_sensor(0x9923,0x01);
write_cmos_sensor(0x9925,0x23);
write_cmos_sensor(0x9927,0x23);
write_cmos_sensor(0x9929,0x23);
write_cmos_sensor(0x992B,0x2F);
write_cmos_sensor(0x992D,0x1A);
write_cmos_sensor(0x992F,0x14);
write_cmos_sensor(0x9931,0x3F);
write_cmos_sensor(0x9933,0x3F);
write_cmos_sensor(0x9935,0x3F);
write_cmos_sensor(0x9937,0x6B);
write_cmos_sensor(0x9939,0x7C);
write_cmos_sensor(0x993B,0x81);
write_cmos_sensor(0x9943,0x0F);
write_cmos_sensor(0x9945,0x0F);
write_cmos_sensor(0x9947,0x0F);
write_cmos_sensor(0x9949,0x0F);
write_cmos_sensor(0x994B,0x0F);
write_cmos_sensor(0x994D,0x0F);
write_cmos_sensor(0x994F,0x42);
write_cmos_sensor(0x9951,0x0F);
write_cmos_sensor(0x9953,0x0B);
write_cmos_sensor(0x9955,0x5A);
write_cmos_sensor(0x9957,0x13);
write_cmos_sensor(0x9959,0x0C);
write_cmos_sensor(0x995A,0x00);
write_cmos_sensor(0x995B,0x00);
write_cmos_sensor(0x995C,0x00);
write_cmos_sensor(0x996B,0x00);
write_cmos_sensor(0x996D,0x10);
write_cmos_sensor(0x996F,0x10);
write_cmos_sensor(0x9971,0xC8);
write_cmos_sensor(0x9973,0x32);
write_cmos_sensor(0x9975,0x04);
write_cmos_sensor(0x9976,0x0A);
write_cmos_sensor(0x99B0,0x20);
write_cmos_sensor(0x99B1,0x20);
write_cmos_sensor(0x99B2,0x20);
write_cmos_sensor(0x99C6,0x6E);
write_cmos_sensor(0x99C7,0x6E);
write_cmos_sensor(0x99C8,0x6E);
write_cmos_sensor(0x9A1F,0x0A);
write_cmos_sensor(0x9AB0,0x20);
write_cmos_sensor(0x9AB1,0x20);
write_cmos_sensor(0x9AB2,0x20);
write_cmos_sensor(0x9AC6,0x6E);
write_cmos_sensor(0x9AC7,0x6E);
write_cmos_sensor(0x9AC8,0x6E);
write_cmos_sensor(0x9B01,0x35);
write_cmos_sensor(0x9B03,0x14);
write_cmos_sensor(0x9B05,0x14);
write_cmos_sensor(0x9B07,0x31);
write_cmos_sensor(0x9B08,0x01);
write_cmos_sensor(0x9B09,0x1B);
write_cmos_sensor(0x9B0A,0x01);
write_cmos_sensor(0x9B0B,0x15);
write_cmos_sensor(0x9B0D,0x1E);
write_cmos_sensor(0x9B0F,0x1E);
write_cmos_sensor(0x9B11,0x1E);
write_cmos_sensor(0x9B13,0x64);
write_cmos_sensor(0x9B15,0x64);
write_cmos_sensor(0x9B17,0x64);
write_cmos_sensor(0x9B19,0x50);
write_cmos_sensor(0x9B1B,0x60);
write_cmos_sensor(0x9B1D,0x65);
write_cmos_sensor(0x9B1F,0x01);
write_cmos_sensor(0x9B21,0x01);
write_cmos_sensor(0x9B23,0x01);
write_cmos_sensor(0x9B25,0x14);
write_cmos_sensor(0x9B27,0x14);
write_cmos_sensor(0x9B29,0x14);
write_cmos_sensor(0x9B2B,0x2F);
write_cmos_sensor(0x9B2D,0x1A);
write_cmos_sensor(0x9B2F,0x14);
write_cmos_sensor(0x9B31,0x1E);
write_cmos_sensor(0x9B33,0x1E);
write_cmos_sensor(0x9B35,0x1E);
write_cmos_sensor(0x9B37,0x6B);
write_cmos_sensor(0x9B39,0x7C);
write_cmos_sensor(0x9B3B,0x81);
write_cmos_sensor(0x9B43,0x0F);
write_cmos_sensor(0x9B45,0x0F);
write_cmos_sensor(0x9B47,0x0F);
write_cmos_sensor(0x9B49,0x0F);
write_cmos_sensor(0x9B4B,0x0F);
write_cmos_sensor(0x9B4D,0x0F);
write_cmos_sensor(0x9B4F,0x2D);
write_cmos_sensor(0x9B51,0x0B);
write_cmos_sensor(0x9B53,0x08);
write_cmos_sensor(0x9B55,0x40);
write_cmos_sensor(0x9B57,0x0D);
write_cmos_sensor(0x9B59,0x08);
write_cmos_sensor(0x9B5A,0x00);
write_cmos_sensor(0x9B5B,0x00);
write_cmos_sensor(0x9B5C,0x00);
write_cmos_sensor(0x9B5D,0x08);
write_cmos_sensor(0x9B5E,0x0E);
write_cmos_sensor(0x9B60,0x08);
write_cmos_sensor(0x9B61,0x0E);
write_cmos_sensor(0x9B6B,0x00);
write_cmos_sensor(0x9B6D,0x10);
write_cmos_sensor(0x9B6F,0x10);
write_cmos_sensor(0x9B71,0xC8);
write_cmos_sensor(0x9B73,0x32);
write_cmos_sensor(0x9B75,0x04);
write_cmos_sensor(0x9B76,0x0A);
write_cmos_sensor(0x9BB0,0x20);
write_cmos_sensor(0x9BB1,0x20);
write_cmos_sensor(0x9BB2,0x20);
write_cmos_sensor(0x9BC6,0x6E);
write_cmos_sensor(0x9BC7,0x6E);
write_cmos_sensor(0x9BC8,0x6E);
write_cmos_sensor(0x9BCC,0x20);
write_cmos_sensor(0x9BCD,0x20);
write_cmos_sensor(0x9BCE,0x20);
write_cmos_sensor(0x9C01,0x10);
write_cmos_sensor(0x9C03,0x1D);
write_cmos_sensor(0x9C05,0x20);
write_cmos_sensor(0x9C13,0x10);
write_cmos_sensor(0x9C15,0x10);
write_cmos_sensor(0x9C17,0x10);
write_cmos_sensor(0x9C19,0x04);
write_cmos_sensor(0x9C1B,0x67);
write_cmos_sensor(0x9C1D,0x80);
write_cmos_sensor(0x9C1F,0x0A);
write_cmos_sensor(0x9C21,0x29);
write_cmos_sensor(0x9C23,0x32);
write_cmos_sensor(0x9C27,0x56);
write_cmos_sensor(0x9C29,0x60);
write_cmos_sensor(0x9C39,0x67);
write_cmos_sensor(0x9C3B,0x80);
write_cmos_sensor(0x9C3D,0x80);
write_cmos_sensor(0x9C3F,0x80);
write_cmos_sensor(0x9C41,0x80);
write_cmos_sensor(0x9C55,0xC8);
write_cmos_sensor(0x9C57,0xC8);
write_cmos_sensor(0x9C59,0xC8);
write_cmos_sensor(0x9C87,0x48);
write_cmos_sensor(0x9C89,0x48);
write_cmos_sensor(0x9C8B,0x48);
write_cmos_sensor(0x9CB0,0x20);
write_cmos_sensor(0x9CB1,0x20);
write_cmos_sensor(0x9CB2,0x20);
write_cmos_sensor(0x9CC6,0x6E);
write_cmos_sensor(0x9CC7,0x6E);
write_cmos_sensor(0x9CC8,0x6E);
write_cmos_sensor(0x9D13,0x10);
write_cmos_sensor(0x9D15,0x10);
write_cmos_sensor(0x9D17,0x10);
write_cmos_sensor(0x9D19,0x04);
write_cmos_sensor(0x9D1B,0x67);
write_cmos_sensor(0x9D1F,0x0A);
write_cmos_sensor(0x9D21,0x29);
write_cmos_sensor(0x9D23,0x32);
write_cmos_sensor(0x9D55,0xC8);
write_cmos_sensor(0x9D57,0xC8);
write_cmos_sensor(0x9D59,0xC8);
write_cmos_sensor(0x9D91,0x20);
write_cmos_sensor(0x9D93,0x20);
write_cmos_sensor(0x9D95,0x20);
write_cmos_sensor(0x9E01,0x10);
write_cmos_sensor(0x9E03,0x1D);
write_cmos_sensor(0x9E13,0x10);
write_cmos_sensor(0x9E15,0x10);
write_cmos_sensor(0x9E17,0x10);
write_cmos_sensor(0x9E19,0x04);
write_cmos_sensor(0x9E1B,0x67);
write_cmos_sensor(0x9E1D,0x80);
write_cmos_sensor(0x9E1F,0x0A);
write_cmos_sensor(0x9E21,0x29);
write_cmos_sensor(0x9E23,0x32);
write_cmos_sensor(0x9E25,0x30);
write_cmos_sensor(0x9E27,0x56);
write_cmos_sensor(0x9E29,0x60);
write_cmos_sensor(0x9E39,0x67);
write_cmos_sensor(0x9E3B,0x80);
write_cmos_sensor(0x9E3D,0x80);
write_cmos_sensor(0x9E3F,0x80);
write_cmos_sensor(0x9E41,0x80);
write_cmos_sensor(0x9E55,0xC8);
write_cmos_sensor(0x9E57,0xC8);
write_cmos_sensor(0x9E59,0xC8);
write_cmos_sensor(0x9E91,0x20);
write_cmos_sensor(0x9E93,0x20);
write_cmos_sensor(0x9E95,0x20);
write_cmos_sensor(0x9F8F,0xA0);
write_cmos_sensor(0xA027,0x67);
write_cmos_sensor(0xA029,0x80);
write_cmos_sensor(0xA02D,0x67);
write_cmos_sensor(0xA02F,0x80);
write_cmos_sensor(0xA031,0x80);
write_cmos_sensor(0xA033,0x80);
write_cmos_sensor(0xA035,0x80);
write_cmos_sensor(0xA037,0x80);
write_cmos_sensor(0xA039,0x80);
write_cmos_sensor(0xA03B,0x80);
write_cmos_sensor(0xA067,0x20);
write_cmos_sensor(0xA068,0x20);
write_cmos_sensor(0xA069,0x20);
write_cmos_sensor(0xA071,0x48);
write_cmos_sensor(0xA073,0x48);
write_cmos_sensor(0xA075,0x48);
write_cmos_sensor(0xA08F,0xA0);
write_cmos_sensor(0xA091,0x3A);
write_cmos_sensor(0xA093,0x3A);
write_cmos_sensor(0xA095,0x0A);
write_cmos_sensor(0xA097,0x0A);
write_cmos_sensor(0xA099,0x0A);

write_cmos_sensor(0x9012,0x00);
write_cmos_sensor(0x9098,0x1A);
write_cmos_sensor(0x9099,0x04);
write_cmos_sensor(0x909A,0x20);
write_cmos_sensor(0x909B,0x20);
write_cmos_sensor(0x909C,0x13);
write_cmos_sensor(0x909D,0x13);
write_cmos_sensor(0xA716,0x13);
write_cmos_sensor(0xA801,0x08);
write_cmos_sensor(0xA803,0x0C);
write_cmos_sensor(0xA805,0x10);
write_cmos_sensor(0xA806,0x00);
write_cmos_sensor(0xA807,0x18);
write_cmos_sensor(0xA808,0x00);
write_cmos_sensor(0xA809,0x20);
write_cmos_sensor(0xA80A,0x00);
write_cmos_sensor(0xA80B,0x30);
write_cmos_sensor(0xA80C,0x00);
write_cmos_sensor(0xA80D,0x40);
write_cmos_sensor(0xA80E,0x00);
write_cmos_sensor(0xA80F,0x60);
write_cmos_sensor(0xA810,0x00);
write_cmos_sensor(0xA811,0x80);
write_cmos_sensor(0xA812,0x00);
write_cmos_sensor(0xA813,0xC0);
write_cmos_sensor(0xA814,0x01);
write_cmos_sensor(0xA815,0x00);
write_cmos_sensor(0xA816,0x01);
write_cmos_sensor(0xA817,0x80);
write_cmos_sensor(0xA818,0x02);
write_cmos_sensor(0xA819,0x00);
write_cmos_sensor(0xA81A,0x03);
write_cmos_sensor(0xA81B,0x00);
write_cmos_sensor(0xA81C,0x03);
write_cmos_sensor(0xA81D,0xAC);
write_cmos_sensor(0xA838,0x03);
write_cmos_sensor(0xA83C,0x28);
write_cmos_sensor(0xA83D,0x5F);
write_cmos_sensor(0xA881,0x08);
write_cmos_sensor(0xA883,0x0C);
write_cmos_sensor(0xA885,0x10);
write_cmos_sensor(0xA886,0x00);
write_cmos_sensor(0xA887,0x18);
write_cmos_sensor(0xA888,0x00);
write_cmos_sensor(0xA889,0x20);
write_cmos_sensor(0xA88A,0x00);
write_cmos_sensor(0xA88B,0x30);
write_cmos_sensor(0xA88C,0x00);
write_cmos_sensor(0xA88D,0x40);
write_cmos_sensor(0xA88E,0x00);
write_cmos_sensor(0xA88F,0x60);
write_cmos_sensor(0xA890,0x00);
write_cmos_sensor(0xA891,0x80);
write_cmos_sensor(0xA892,0x00);
write_cmos_sensor(0xA893,0xC0);
write_cmos_sensor(0xA894,0x01);
write_cmos_sensor(0xA895,0x00);
write_cmos_sensor(0xA896,0x01);
write_cmos_sensor(0xA897,0x80);
write_cmos_sensor(0xA898,0x02);
write_cmos_sensor(0xA899,0x00);
write_cmos_sensor(0xA89A,0x03);
write_cmos_sensor(0xA89B,0x00);
write_cmos_sensor(0xA89C,0x03);
write_cmos_sensor(0xA89D,0xAC);
write_cmos_sensor(0xA8B8,0x03);
write_cmos_sensor(0xA8BB,0x13);
write_cmos_sensor(0xA8BC,0x28);
write_cmos_sensor(0xA8BD,0x25);
write_cmos_sensor(0xA8BE,0x1D);
write_cmos_sensor(0xA8C0,0x3A);
write_cmos_sensor(0xA8C1,0xE0);
write_cmos_sensor(0xB24F,0x80);

write_cmos_sensor(0x3198,0x0F);
//< AF hunting issue, RickLiu 20160303
write_cmos_sensor(0x31A0,0x02);
write_cmos_sensor(0x31A1,0x02);
write_cmos_sensor(0x31A2,0x02);
write_cmos_sensor(0x31A3,0x03);
//< AF hunting issue, RickLiu 20160303
write_cmos_sensor(0x31A8,0x18);
write_cmos_sensor(0x822C,0x01);
write_cmos_sensor(0x9503,0x07);
write_cmos_sensor(0x9504,0x07);
write_cmos_sensor(0x9505,0x07);
write_cmos_sensor(0x9506,0x00);
write_cmos_sensor(0x9507,0x00);
write_cmos_sensor(0x9508,0x00);

write_cmos_sensor(0x8858,0x00);

write_cmos_sensor(0x6B42,0x40);
write_cmos_sensor(0x6B46,0x00);
write_cmos_sensor(0x6B47,0x4B);
write_cmos_sensor(0x6B4A,0x00);
write_cmos_sensor(0x6B4B,0x4B);
write_cmos_sensor(0x6B4E,0x00);
write_cmos_sensor(0x6B4F,0x4B);
write_cmos_sensor(0x6B44,0x00);
write_cmos_sensor(0x6B45,0x8C);
write_cmos_sensor(0x6B48,0x00);
write_cmos_sensor(0x6B49,0x8C);
write_cmos_sensor(0x6B4C,0x00);
write_cmos_sensor(0x6B4D,0x8C);

}



static void preview_setting(void)
{
	write_cmos_sensor(0x0100,0x00);

write_cmos_sensor(0x0114,0x03);
write_cmos_sensor(0x0220,0x00);
write_cmos_sensor(0x0221,0x11);
write_cmos_sensor(0x0222,0x01);
write_cmos_sensor(0x0340,0x08);
write_cmos_sensor(0x0341,0x3C);
write_cmos_sensor(0x0342,0x17);
write_cmos_sensor(0x0343,0x88);
write_cmos_sensor(0x0344,0x00);
write_cmos_sensor(0x0345,0x00);
write_cmos_sensor(0x0346,0x00);
write_cmos_sensor(0x0347,0x00);
write_cmos_sensor(0x0348,0x14);
write_cmos_sensor(0x0349,0xDF);
write_cmos_sensor(0x034A,0x0F);
write_cmos_sensor(0x034B,0xAF);
write_cmos_sensor(0x0381,0x01);
write_cmos_sensor(0x0383,0x01);
write_cmos_sensor(0x0385,0x01);
write_cmos_sensor(0x0387,0x01);
write_cmos_sensor(0x0900,0x01);
write_cmos_sensor(0x0901,0x22);
write_cmos_sensor(0x0902,0x00);
write_cmos_sensor(0x3000,0x74);
write_cmos_sensor(0x3001,0x00);
write_cmos_sensor(0x305C,0x11);

write_cmos_sensor(0x0112,0x0A);
write_cmos_sensor(0x0113,0x0A);
write_cmos_sensor(0x034C,0x0A);
write_cmos_sensor(0x034D,0x70);
write_cmos_sensor(0x034E,0x07);
write_cmos_sensor(0x034F,0xD8);
write_cmos_sensor(0x0401,0x00);
write_cmos_sensor(0x0404,0x00);
write_cmos_sensor(0x0405,0x10);
write_cmos_sensor(0x0408,0x00);
write_cmos_sensor(0x0409,0x00);
write_cmos_sensor(0x040A,0x00);
write_cmos_sensor(0x040B,0x00);
write_cmos_sensor(0x040C,0x0A);
write_cmos_sensor(0x040D,0x70);
write_cmos_sensor(0x040E,0x07);
write_cmos_sensor(0x040F,0xD8);

write_cmos_sensor(0x0301,0x04);
write_cmos_sensor(0x0303,0x02);
write_cmos_sensor(0x0305,0x04);
write_cmos_sensor(0x0306,0x00);
write_cmos_sensor(0x0307,0x7F);
write_cmos_sensor(0x0309,0x0A);
write_cmos_sensor(0x030B,0x02);
write_cmos_sensor(0x030D,0x0C);
write_cmos_sensor(0x030E,0x01);
write_cmos_sensor(0x030F,0xD6);
write_cmos_sensor(0x0310,0x01);

write_cmos_sensor(0x0820,0x07);
write_cmos_sensor(0x0821,0x58);
write_cmos_sensor(0x0822,0x00);
write_cmos_sensor(0x0823,0x00);

write_cmos_sensor(0x0202,0x08);
write_cmos_sensor(0x0203,0x32);
write_cmos_sensor(0x0224,0x01);
write_cmos_sensor(0x0225,0xF4);

write_cmos_sensor(0x0204,0x00);
write_cmos_sensor(0x0205,0x00);
write_cmos_sensor(0x0216,0x00);
write_cmos_sensor(0x0217,0x00);
write_cmos_sensor(0x020E,0x01);
write_cmos_sensor(0x020F,0x00);
write_cmos_sensor(0x0210,0x01);
write_cmos_sensor(0x0211,0x00);
write_cmos_sensor(0x0212,0x01);
write_cmos_sensor(0x0213,0x00);
write_cmos_sensor(0x0214,0x01);
write_cmos_sensor(0x0215,0x00);

write_cmos_sensor(0x3006,0x01);
write_cmos_sensor(0x3007,0x02);
write_cmos_sensor(0x31E0,0x03);
write_cmos_sensor(0x31E1,0xFF);
write_cmos_sensor(0x31E4,0x02);

write_cmos_sensor(0x3A22,0x20);
write_cmos_sensor(0x3A23,0x14);
write_cmos_sensor(0x3A24,0xE0);
write_cmos_sensor(0x3A25,0x07);
write_cmos_sensor(0x3A26,0xD8);
write_cmos_sensor(0x3A2F,0x00);
write_cmos_sensor(0x3A30,0x00);
write_cmos_sensor(0x3A31,0x00);
write_cmos_sensor(0x3A32,0x00);
write_cmos_sensor(0x3A33,0x14);
write_cmos_sensor(0x3A34,0xDF);
write_cmos_sensor(0x3A35,0x0F);
write_cmos_sensor(0x3A36,0xAF);
write_cmos_sensor(0x3A37,0x00);
write_cmos_sensor(0x3A38,0x01);
write_cmos_sensor(0x3A39,0x00);

write_cmos_sensor(0x3A21,0x00);

write_cmos_sensor(0x3011,0x00);
write_cmos_sensor(0x3013,0x01);
//
if(imgsensor.pdaf_mode == 1)
	{
		/*PDAF*/
		/*PD_CAL_ENALBE*/
		write_cmos_sensor(0x3121,0x01);
		/*AREA MODE*/
		write_cmos_sensor(0x31B0,0x01);// 8x6 output
		/*PD_OUT_EN=1*/
		write_cmos_sensor(0x3123,0x01);
		
		/*Fixed area mode*/
		write_cmos_sensor(0x3150,0x00);
		write_cmos_sensor(0x3151,0x38);// X offset	112/2 = 56
		write_cmos_sensor(0x3152,0x00);
		write_cmos_sensor(0x3153,0x2c);// Y offset 88/2 = 44 
		write_cmos_sensor(0x3154,0x01);// X size 640 / 2 = 320
		write_cmos_sensor(0x3155,0x40);
		write_cmos_sensor(0x3156,0x01);// Y size 640 /2 =320
		write_cmos_sensor(0x3157,0x40);
		
	}
		write_cmos_sensor(0x0100,0x01);

}    /*    preview_setting  */

static void preview_setting_HDR_ES2(void)
{
	write_cmos_sensor(0x0100,0x00);
	//printk("[Rick] preview_setting_HDR_ES2 1 \n");
	write_cmos_sensor(0x0114,0x03);
	write_cmos_sensor(0x0220,0x03);
	write_cmos_sensor(0x0221,0x22);
	//<20160309 RickLiu Reduce ghost section when switch to HDR video
	write_cmos_sensor(0x0222,0x02);
	write_cmos_sensor(0x6958,3);
	//>20160309 RickLiu Reduce ghost section when switch to HDR video
	write_cmos_sensor(0x0340,0x08);
	write_cmos_sensor(0x0341,0x3C);
	write_cmos_sensor(0x0342,0x17);
	write_cmos_sensor(0x0343,0x88);
	write_cmos_sensor(0x0344,0x00);
	write_cmos_sensor(0x0345,0x00);
	write_cmos_sensor(0x0346,0x00);
	write_cmos_sensor(0x0347,0x00);
	write_cmos_sensor(0x0348,0x14);
	write_cmos_sensor(0x0349,0xDF);
	write_cmos_sensor(0x034A,0x0F);
	write_cmos_sensor(0x034B,0xAF);
	write_cmos_sensor(0x0381,0x01);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0385,0x01);
	write_cmos_sensor(0x0387,0x01);
	write_cmos_sensor(0x0900,0x01);
	write_cmos_sensor(0x0901,0x22);
	write_cmos_sensor(0x0902,0x00);
	write_cmos_sensor(0x3000,0x75);
	write_cmos_sensor(0x3001,0x01);/*bit[0]HDR enable */
	write_cmos_sensor(0x305C,0x11);
	//printk("[Rick] preview_setting_HDR_ES2 2 \n");
	write_cmos_sensor(0x0112,0x0A);
	write_cmos_sensor(0x0113,0x0A);
	write_cmos_sensor(0x034C,0x0A);
	write_cmos_sensor(0x034D,0x70);
	write_cmos_sensor(0x034E,0x07);
	write_cmos_sensor(0x034F,0xD8);
	write_cmos_sensor(0x0401,0x00);
	write_cmos_sensor(0x0404,0x00);
	write_cmos_sensor(0x0405,0x10);
	write_cmos_sensor(0x0408,0x00);
	write_cmos_sensor(0x0409,0x00);
	write_cmos_sensor(0x040A,0x00);
	write_cmos_sensor(0x040B,0x00);
	write_cmos_sensor(0x040C,0x0A);
	write_cmos_sensor(0x040D,0x70);
	write_cmos_sensor(0x040E,0x07);
	write_cmos_sensor(0x040F,0xD8);

	write_cmos_sensor(0x0301,0x04);
	write_cmos_sensor(0x0303,0x02);
	write_cmos_sensor(0x0305,0x04);
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0x7F);
	write_cmos_sensor(0x0309,0x0A);
	write_cmos_sensor(0x030B,0x01);
	write_cmos_sensor(0x030D,0x0C);
	write_cmos_sensor(0x030E,0x01);
	write_cmos_sensor(0x030F,0xD6);
	write_cmos_sensor(0x0310,0x01);

	write_cmos_sensor(0x0820,0x0e);
	write_cmos_sensor(0x0821,0xb0);
	write_cmos_sensor(0x0822,0x00);
	write_cmos_sensor(0x0823,0x00);

	write_cmos_sensor(0x0202,0x08);
	write_cmos_sensor(0x0203,0x32);
	write_cmos_sensor(0x0224,0x00);
	write_cmos_sensor(0x0225,0xb3);

	write_cmos_sensor(0x0204,0x00);
	write_cmos_sensor(0x0205,0x00);
	write_cmos_sensor(0x0216,0x00);
	write_cmos_sensor(0x0217,0x00);
	write_cmos_sensor(0x020E,0x01);
	write_cmos_sensor(0x020F,0x00);
	write_cmos_sensor(0x0210,0x01);
	write_cmos_sensor(0x0211,0x00);
	write_cmos_sensor(0x0212,0x01);
	write_cmos_sensor(0x0213,0x00);
	write_cmos_sensor(0x0214,0x01);
	write_cmos_sensor(0x0215,0x00);

	write_cmos_sensor(0x3006,0x01);
	write_cmos_sensor(0x3007,0x01);
	write_cmos_sensor(0x31E0,0x3f);
	write_cmos_sensor(0x31E1,0xFF);
	write_cmos_sensor(0x31E4,0x00);

	write_cmos_sensor(0x3A22,0x00);
	write_cmos_sensor(0x3A23,0x14);
	write_cmos_sensor(0x3A24,0xE0);
	write_cmos_sensor(0x3A25,0x0f);
	write_cmos_sensor(0x3A26,0xb0);
	write_cmos_sensor(0x3A2F,0x00);
	write_cmos_sensor(0x3A30,0x00);
	write_cmos_sensor(0x3A31,0x00);
	write_cmos_sensor(0x3A32,0x00);
	write_cmos_sensor(0x3A33,0x14);
	write_cmos_sensor(0x3A34,0xDF);
	write_cmos_sensor(0x3A35,0x0F);
	write_cmos_sensor(0x3A36,0xAF);
	write_cmos_sensor(0x3A37,0x00);
	write_cmos_sensor(0x3A38,0x00);
	write_cmos_sensor(0x3A39,0x10);

	write_cmos_sensor(0x3A21,0x02);

	write_cmos_sensor(0x3011,0x00);
	write_cmos_sensor(0x3013,0x01);

	if(imgsensor.pdaf_mode == 1)
	{	
		write_cmos_sensor(0x3001,0x01);/*bit[0]PDAF enable during HDR on*/
		/*PDAF*/
		/*PD_CAL_ENALBE*/
		write_cmos_sensor(0x3121,0x01);
		/*AREA MODE*/
		write_cmos_sensor(0x31B0,0x01);// 8x6 output
		/*PD_OUT_EN=1*/
		write_cmos_sensor(0x3123,0x01);
		write_cmos_sensor(0x0100,0x01);
		/*Fixed area mode*/
		write_cmos_sensor(0x3150,0x00);
		write_cmos_sensor(0x3151,0x38);// X offset	112/2 = 56
		write_cmos_sensor(0x3152,0x00);
		write_cmos_sensor(0x3153,0x2c);// Y offset 88/2 = 44 
		write_cmos_sensor(0x3154,0x01);// X size 640 / 2 = 320
		write_cmos_sensor(0x3155,0x40);
		write_cmos_sensor(0x3156,0x01);// Y size 640 /2 =320
		write_cmos_sensor(0x3157,0x40);
	}
//
	write_cmos_sensor(0x0100,0x01);
}    /*    preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
    printk("E! currefps:%d\n",currefps);
    if (currefps == 150) { //15fps for PIP
	write_cmos_sensor(0x0100,0x00);

write_cmos_sensor(0x0114,0x03);
write_cmos_sensor(0x0220,0x00);
write_cmos_sensor(0x0221,0x11);
write_cmos_sensor(0x0222,0x01);
write_cmos_sensor(0x0340,0x10);
write_cmos_sensor(0x0341,0x1E);
write_cmos_sensor(0x0342,0x17);
write_cmos_sensor(0x0343,0x88);
write_cmos_sensor(0x0344,0x00);
write_cmos_sensor(0x0345,0x00);
write_cmos_sensor(0x0346,0x00);
write_cmos_sensor(0x0347,0x00);
write_cmos_sensor(0x0348,0x14);
write_cmos_sensor(0x0349,0xDF);
write_cmos_sensor(0x034A,0x0F);
write_cmos_sensor(0x034B,0xAF);
write_cmos_sensor(0x0381,0x01);
write_cmos_sensor(0x0383,0x01);
write_cmos_sensor(0x0385,0x01);
write_cmos_sensor(0x0387,0x01);
write_cmos_sensor(0x0900,0x00);
write_cmos_sensor(0x0901,0x11);
write_cmos_sensor(0x0902,0x00);
write_cmos_sensor(0x3000,0x74);
write_cmos_sensor(0x3001,0x00);
write_cmos_sensor(0x305C,0x11);

write_cmos_sensor(0x0112,0x0A);
write_cmos_sensor(0x0113,0x0A);
write_cmos_sensor(0x034C,0x14);
write_cmos_sensor(0x034D,0xE0);
write_cmos_sensor(0x034E,0x0F);
write_cmos_sensor(0x034F,0xB0);
write_cmos_sensor(0x0401,0x00);
write_cmos_sensor(0x0404,0x00);
write_cmos_sensor(0x0405,0x10);
write_cmos_sensor(0x0408,0x00);
write_cmos_sensor(0x0409,0x00);
write_cmos_sensor(0x040A,0x00);
write_cmos_sensor(0x040B,0x00);
write_cmos_sensor(0x040C,0x14);
write_cmos_sensor(0x040D,0xE0);
write_cmos_sensor(0x040E,0x0F);
write_cmos_sensor(0x040F,0xB0);

write_cmos_sensor(0x0301,0x04);
write_cmos_sensor(0x0303,0x02);
write_cmos_sensor(0x0305,0x04);
write_cmos_sensor(0x0306,0x00);
write_cmos_sensor(0x0307,0x7D);
write_cmos_sensor(0x0309,0x0A);
write_cmos_sensor(0x030B,0x01);
write_cmos_sensor(0x030D,0x0C);
write_cmos_sensor(0x030E,0x01);
write_cmos_sensor(0x030F,0xB8);
write_cmos_sensor(0x0310,0x01);

write_cmos_sensor(0x0820,0x0D);
write_cmos_sensor(0x0821,0xC0);
write_cmos_sensor(0x0822,0x00);
write_cmos_sensor(0x0823,0x00);

write_cmos_sensor(0x0202,0x10);
write_cmos_sensor(0x0203,0x14);
write_cmos_sensor(0x0224,0x01);
write_cmos_sensor(0x0225,0xF4);

write_cmos_sensor(0x0204,0x00);
write_cmos_sensor(0x0205,0x00);
write_cmos_sensor(0x0216,0x00);
write_cmos_sensor(0x0217,0x00);
write_cmos_sensor(0x020E,0x01);
write_cmos_sensor(0x020F,0x00);
write_cmos_sensor(0x0210,0x01);
write_cmos_sensor(0x0211,0x00);
write_cmos_sensor(0x0212,0x01);
write_cmos_sensor(0x0213,0x00);
write_cmos_sensor(0x0214,0x01);
write_cmos_sensor(0x0215,0x00);

write_cmos_sensor(0x3006,0x01);
write_cmos_sensor(0x3007,0x02);
write_cmos_sensor(0x31E0,0x03);
write_cmos_sensor(0x31E1,0xFF);
write_cmos_sensor(0x31E4,0x02);

write_cmos_sensor(0x3A22,0x20);
write_cmos_sensor(0x3A23,0x14);
write_cmos_sensor(0x3A24,0xE0);
write_cmos_sensor(0x3A25,0x0F);
write_cmos_sensor(0x3A26,0xB0);
write_cmos_sensor(0x3A2F,0x00);
write_cmos_sensor(0x3A30,0x00);
write_cmos_sensor(0x3A31,0x00);
write_cmos_sensor(0x3A32,0x00);
write_cmos_sensor(0x3A33,0x14);
write_cmos_sensor(0x3A34,0xDF);
write_cmos_sensor(0x3A35,0x0F);
write_cmos_sensor(0x3A36,0xAF);
write_cmos_sensor(0x3A37,0x00);
write_cmos_sensor(0x3A38,0x00);
write_cmos_sensor(0x3A39,0x00);

write_cmos_sensor(0x3A21,0x00);

write_cmos_sensor(0x3011,0x00);
write_cmos_sensor(0x3013,0x01);


	write_cmos_sensor(0x0100,0x01);		
    } else {   //24fps            //24fps for Normal capture & ZSD
	write_cmos_sensor(0x0100,0x00);

	write_cmos_sensor(0x0114,0x03);
	write_cmos_sensor(0x0220,0x00);
	write_cmos_sensor(0x0221,0x11);
	write_cmos_sensor(0x0222,0x01);
	write_cmos_sensor(0x0340,0x10);
	write_cmos_sensor(0x0341,0x1E);
	write_cmos_sensor(0x0342,0x17);
	write_cmos_sensor(0x0343,0x88);
	write_cmos_sensor(0x0344,0x00);
	write_cmos_sensor(0x0345,0x00);
	write_cmos_sensor(0x0346,0x00);
	write_cmos_sensor(0x0347,0x00);
	write_cmos_sensor(0x0348,0x14);
	write_cmos_sensor(0x0349,0xDF);
	write_cmos_sensor(0x034A,0x0F);
	write_cmos_sensor(0x034B,0xAF);
	write_cmos_sensor(0x0381,0x01);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0385,0x01);
	write_cmos_sensor(0x0387,0x01);
	write_cmos_sensor(0x0900,0x00);
	write_cmos_sensor(0x0901,0x11);
	write_cmos_sensor(0x0902,0x00);
	write_cmos_sensor(0x3000,0x74);
	write_cmos_sensor(0x3001,0x00);
	write_cmos_sensor(0x305C,0x11);

	write_cmos_sensor(0x0112,0x0A);
	write_cmos_sensor(0x0113,0x0A);
	write_cmos_sensor(0x034C,0x14);
	write_cmos_sensor(0x034D,0xE0);
	write_cmos_sensor(0x034E,0x0F);
	write_cmos_sensor(0x034F,0xB0);
	write_cmos_sensor(0x0401,0x00);
	write_cmos_sensor(0x0404,0x00);
	write_cmos_sensor(0x0405,0x10);
	write_cmos_sensor(0x0408,0x00);
	write_cmos_sensor(0x0409,0x00);
	write_cmos_sensor(0x040A,0x00);
	write_cmos_sensor(0x040B,0x00);
	write_cmos_sensor(0x040C,0x14);
	write_cmos_sensor(0x040D,0xE0);
	write_cmos_sensor(0x040E,0x0F);
	write_cmos_sensor(0x040F,0xB0);

	write_cmos_sensor(0x0301,0x04);
	write_cmos_sensor(0x0303,0x02);
	write_cmos_sensor(0x0305,0x04);
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0xC7);
	write_cmos_sensor(0x0309,0x0A);
	write_cmos_sensor(0x030B,0x01);
	write_cmos_sensor(0x030D,0x0C);
	write_cmos_sensor(0x030E,0x02);
	write_cmos_sensor(0x030F,0xC1);
	write_cmos_sensor(0x0310,0x01);

	write_cmos_sensor(0x0820,0x16);
	write_cmos_sensor(0x0821,0x08);
	write_cmos_sensor(0x0822,0x00);
	write_cmos_sensor(0x0823,0x00);

	write_cmos_sensor(0x0202,0x10);
	write_cmos_sensor(0x0203,0x14);
	write_cmos_sensor(0x0224,0x01);
	write_cmos_sensor(0x0225,0xF4);

	write_cmos_sensor(0x0204,0x00);
	write_cmos_sensor(0x0205,0x00);
	write_cmos_sensor(0x0216,0x00);
	write_cmos_sensor(0x0217,0x00);
	write_cmos_sensor(0x020E,0x01);
	write_cmos_sensor(0x020F,0x00);
	write_cmos_sensor(0x0210,0x01);
	write_cmos_sensor(0x0211,0x00);
	write_cmos_sensor(0x0212,0x01);
	write_cmos_sensor(0x0213,0x00);
	write_cmos_sensor(0x0214,0x01);
	write_cmos_sensor(0x0215,0x00);

	write_cmos_sensor(0x3006,0x01);
	write_cmos_sensor(0x3007,0x02);
	write_cmos_sensor(0x31E0,0x03);
	write_cmos_sensor(0x31E1,0xFF);
	write_cmos_sensor(0x31E4,0x02);

	write_cmos_sensor(0x3A22,0x20);
	write_cmos_sensor(0x3A23,0x14);
	write_cmos_sensor(0x3A24,0xE0);
	write_cmos_sensor(0x3A25,0x0F);
	write_cmos_sensor(0x3A26,0xB0);
	write_cmos_sensor(0x3A2F,0x00);
	write_cmos_sensor(0x3A30,0x00);
	write_cmos_sensor(0x3A31,0x00);
	write_cmos_sensor(0x3A32,0x00);
	write_cmos_sensor(0x3A33,0x14);
	write_cmos_sensor(0x3A34,0xDF);
	write_cmos_sensor(0x3A35,0x0F);
	write_cmos_sensor(0x3A36,0xAF);
	write_cmos_sensor(0x3A37,0x00);
	write_cmos_sensor(0x3A38,0x00);
	write_cmos_sensor(0x3A39,0x00);

	write_cmos_sensor(0x3A21,0x00);

	write_cmos_sensor(0x3011,0x00);
	write_cmos_sensor(0x3013,0x01);
	write_cmos_sensor(0x0100,0x01); 

	}
}

static void capture_setting_HDR_ES2(void)
{

  /*24fps for Normal capture & ZSD*/
	write_cmos_sensor(0x0100,0x00);

	write_cmos_sensor(0x0114,0x03);
	write_cmos_sensor(0x0220,0x03);
	write_cmos_sensor(0x0221,0x11);
	write_cmos_sensor(0x0222,0x08);
	//<20160309 RickLiu Reduce ghost section when switch to HDR video
	write_cmos_sensor(0x0222,0x02);
	write_cmos_sensor(0x6958,3);
	//>20160309 RickLiu Reduce ghost section when switch to HDR video
	write_cmos_sensor(0x0340,0x10);
	write_cmos_sensor(0x0341,0x1E);
	write_cmos_sensor(0x0342,0x17);
	write_cmos_sensor(0x0343,0x88);
	write_cmos_sensor(0x0344,0x00);
	write_cmos_sensor(0x0345,0x00);
	write_cmos_sensor(0x0346,0x00);
	write_cmos_sensor(0x0347,0x00);
	write_cmos_sensor(0x0348,0x14);
	write_cmos_sensor(0x0349,0xDF);
	write_cmos_sensor(0x034A,0x0F);
	write_cmos_sensor(0x034B,0xAF);
	write_cmos_sensor(0x0381,0x01);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0385,0x01);
	write_cmos_sensor(0x0387,0x01);
	write_cmos_sensor(0x0900,0x00);
	write_cmos_sensor(0x0901,0x11);
	write_cmos_sensor(0x0902,0x00);
	write_cmos_sensor(0x3000,0x75);
	write_cmos_sensor(0x3001,0x01);/*bit[0]HDR enable */
	write_cmos_sensor(0x305C,0x11);

	write_cmos_sensor(0x0112,0x0A);
	write_cmos_sensor(0x0113,0x0A);
	write_cmos_sensor(0x034C,0x14);
	write_cmos_sensor(0x034D,0xE0);
	write_cmos_sensor(0x034E,0x0F);
	write_cmos_sensor(0x034F,0xB0);
	write_cmos_sensor(0x0401,0x00);
	write_cmos_sensor(0x0404,0x00);
	write_cmos_sensor(0x0405,0x10);
	write_cmos_sensor(0x0408,0x00);
	write_cmos_sensor(0x0409,0x00);
	write_cmos_sensor(0x040A,0x00);
	write_cmos_sensor(0x040B,0x00);
	write_cmos_sensor(0x040C,0x14);
	write_cmos_sensor(0x040D,0xE0);
	write_cmos_sensor(0x040E,0x0F);
	write_cmos_sensor(0x040F,0xB0);

	write_cmos_sensor(0x0301,0x04);
	write_cmos_sensor(0x0303,0x02);
	write_cmos_sensor(0x0305,0x04);
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0xC7);
	write_cmos_sensor(0x0309,0x0A);
	write_cmos_sensor(0x030B,0x01);
	write_cmos_sensor(0x030D,0x0C);
	write_cmos_sensor(0x030E,0x02);
	write_cmos_sensor(0x030F,0xC1);
	write_cmos_sensor(0x0310,0x01);

	write_cmos_sensor(0x0820,0x16);
	write_cmos_sensor(0x0821,0x08);
	write_cmos_sensor(0x0822,0x00);
	write_cmos_sensor(0x0823,0x00);

	write_cmos_sensor(0x0202,0x10);
	write_cmos_sensor(0x0203,0x14);
	write_cmos_sensor(0x0224,0x02);
	write_cmos_sensor(0x0225,0x02);

	write_cmos_sensor(0x0204,0x00);
	write_cmos_sensor(0x0205,0x00);
	write_cmos_sensor(0x0216,0x00);
	write_cmos_sensor(0x0217,0x00);
	write_cmos_sensor(0x020E,0x01);
	write_cmos_sensor(0x020F,0x00);
	write_cmos_sensor(0x0210,0x01);
	write_cmos_sensor(0x0211,0x00);
	write_cmos_sensor(0x0212,0x01);
	write_cmos_sensor(0x0213,0x00);
	write_cmos_sensor(0x0214,0x01);
	write_cmos_sensor(0x0215,0x00);

	write_cmos_sensor(0x3006,0x01);
	write_cmos_sensor(0x3007,0x01);
	write_cmos_sensor(0x31E0,0x3F);
	write_cmos_sensor(0x31E1,0xFF);
	write_cmos_sensor(0x31E4,0x00);

	write_cmos_sensor(0x3A22,0x00);
	write_cmos_sensor(0x3A23,0x14);
	write_cmos_sensor(0x3A24,0xE0);
	write_cmos_sensor(0x3A25,0x0F);
	write_cmos_sensor(0x3A26,0xB0);
	write_cmos_sensor(0x3A2F,0x00);
	write_cmos_sensor(0x3A30,0x00);
	write_cmos_sensor(0x3A31,0x00);
	write_cmos_sensor(0x3A32,0x00);
	write_cmos_sensor(0x3A33,0x14);
	write_cmos_sensor(0x3A34,0xDF);
	write_cmos_sensor(0x3A35,0x0F);
	write_cmos_sensor(0x3A36,0xAF);
	write_cmos_sensor(0x3A37,0x00);
	write_cmos_sensor(0x3A38,0x00);
	write_cmos_sensor(0x3A39,0x00);

	write_cmos_sensor(0x3A21,0x02);

	write_cmos_sensor(0x3011,0x00);
	write_cmos_sensor(0x3013,0x01);


	/*PDAF*/
	/*PD_CAL_ENALBE*/
	write_cmos_sensor(0x3121,0x01);
	/*AREA MODE*/
	write_cmos_sensor(0x31B0,0x01);
	/*PD_OUT_EN=1*/
	write_cmos_sensor(0x3123,0x01);
	
	write_cmos_sensor(0x0100,0x01);
    /*Fixed area mode*/
	write_cmos_sensor(0x3150,0x00);
	write_cmos_sensor(0x3151,0x00);
	write_cmos_sensor(0x3152,0x00);
	write_cmos_sensor(0x3153,0x00);
	write_cmos_sensor(0x3154,0x14);
	write_cmos_sensor(0x3155,0xE0);
	write_cmos_sensor(0x3156,0x0F);
	write_cmos_sensor(0x3157,0xB0);
}

static void capture_setting_pdaf(kal_uint16 currefps)
{
    //printk("E! currefps:%d\n",currefps);
    if (currefps == 150) { //15fps for PIP
	write_cmos_sensor(0x0100,0x00);

	write_cmos_sensor(0x0114,0x03);
	write_cmos_sensor(0x0220,0x00);
	write_cmos_sensor(0x0221,0x11);
	write_cmos_sensor(0x0222,0x01);
	write_cmos_sensor(0x0340,0x10);
	write_cmos_sensor(0x0341,0x1E);
	write_cmos_sensor(0x0342,0x17);
	write_cmos_sensor(0x0343,0x88);
	write_cmos_sensor(0x0344,0x00);
	write_cmos_sensor(0x0345,0x00);
	write_cmos_sensor(0x0346,0x00);
	write_cmos_sensor(0x0347,0x00);
	write_cmos_sensor(0x0348,0x14);
	write_cmos_sensor(0x0349,0xDF);
	write_cmos_sensor(0x034A,0x0F);
	write_cmos_sensor(0x034B,0xAF);
	write_cmos_sensor(0x0381,0x01);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0385,0x01);
	write_cmos_sensor(0x0387,0x01);
	write_cmos_sensor(0x0900,0x00);
	write_cmos_sensor(0x0901,0x11);
	write_cmos_sensor(0x0902,0x00);
	write_cmos_sensor(0x3000,0x74);
	write_cmos_sensor(0x3001,0x00);
	write_cmos_sensor(0x305C,0x11);

	write_cmos_sensor(0x0112,0x0A);
	write_cmos_sensor(0x0113,0x0A);
	write_cmos_sensor(0x034C,0x14);
	write_cmos_sensor(0x034D,0xE0);
	write_cmos_sensor(0x034E,0x0F);
	write_cmos_sensor(0x034F,0xB0);
	write_cmos_sensor(0x0401,0x00);
	write_cmos_sensor(0x0404,0x00);
	write_cmos_sensor(0x0405,0x10);
	write_cmos_sensor(0x0408,0x00);
	write_cmos_sensor(0x0409,0x00);
	write_cmos_sensor(0x040A,0x00);
	write_cmos_sensor(0x040B,0x00);
	write_cmos_sensor(0x040C,0x14);
	write_cmos_sensor(0x040D,0xE0);
	write_cmos_sensor(0x040E,0x0F);
	write_cmos_sensor(0x040F,0xB0);

	write_cmos_sensor(0x0301,0x04);
	write_cmos_sensor(0x0303,0x02);
	write_cmos_sensor(0x0305,0x04);
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0x7D);
	write_cmos_sensor(0x0309,0x0A);
	write_cmos_sensor(0x030B,0x01);
	write_cmos_sensor(0x030D,0x0C);
	write_cmos_sensor(0x030E,0x01);
	write_cmos_sensor(0x030F,0xB8);
	write_cmos_sensor(0x0310,0x01);

	write_cmos_sensor(0x0820,0x0D);
	write_cmos_sensor(0x0821,0xC0);
	write_cmos_sensor(0x0822,0x00);
	write_cmos_sensor(0x0823,0x00);

	write_cmos_sensor(0x0202,0x10);
	write_cmos_sensor(0x0203,0x14);
	write_cmos_sensor(0x0224,0x01);
	write_cmos_sensor(0x0225,0xF4);

	write_cmos_sensor(0x0204,0x00);
	write_cmos_sensor(0x0205,0x00);
	write_cmos_sensor(0x0216,0x00);
	write_cmos_sensor(0x0217,0x00);
	write_cmos_sensor(0x020E,0x01);
	write_cmos_sensor(0x020F,0x00);
	write_cmos_sensor(0x0210,0x01);
	write_cmos_sensor(0x0211,0x00);
	write_cmos_sensor(0x0212,0x01);
	write_cmos_sensor(0x0213,0x00);
	write_cmos_sensor(0x0214,0x01);
	write_cmos_sensor(0x0215,0x00);

	write_cmos_sensor(0x3006,0x01);
	write_cmos_sensor(0x3007,0x02);
	write_cmos_sensor(0x31E0,0x03);
	write_cmos_sensor(0x31E1,0xFF);
	write_cmos_sensor(0x31E4,0x02);

	write_cmos_sensor(0x3A22,0x20);
	write_cmos_sensor(0x3A23,0x14);
	write_cmos_sensor(0x3A24,0xE0);
	write_cmos_sensor(0x3A25,0x0F);
	write_cmos_sensor(0x3A26,0xB0);
	write_cmos_sensor(0x3A2F,0x00);
	write_cmos_sensor(0x3A30,0x00);
	write_cmos_sensor(0x3A31,0x00);
	write_cmos_sensor(0x3A32,0x00);
	write_cmos_sensor(0x3A33,0x14);
	write_cmos_sensor(0x3A34,0xDF);
	write_cmos_sensor(0x3A35,0x0F);
	write_cmos_sensor(0x3A36,0xAF);
	write_cmos_sensor(0x3A37,0x00);
	write_cmos_sensor(0x3A38,0x00);
	write_cmos_sensor(0x3A39,0x00);

	write_cmos_sensor(0x3A21,0x00);

	write_cmos_sensor(0x3011,0x00);
	write_cmos_sensor(0x3013,0x01);

    } else {   //24fps            //24fps for Normal capture & ZSD
		write_cmos_sensor(0x0100,0x00);
		
		write_cmos_sensor(0x0114,0x03);
		write_cmos_sensor(0x0220,0x00);
		write_cmos_sensor(0x0221,0x11);
		write_cmos_sensor(0x0222,0x01);
		write_cmos_sensor(0x0340,0x10);
		write_cmos_sensor(0x0341,0x1E);
		write_cmos_sensor(0x0342,0x17);
		write_cmos_sensor(0x0343,0x88);
		write_cmos_sensor(0x0344,0x00);
		write_cmos_sensor(0x0345,0x00);
		write_cmos_sensor(0x0346,0x00);
		write_cmos_sensor(0x0347,0x00);
		write_cmos_sensor(0x0348,0x14);
		write_cmos_sensor(0x0349,0xDF);
		write_cmos_sensor(0x034A,0x0F);
		write_cmos_sensor(0x034B,0xAF);
		write_cmos_sensor(0x0381,0x01);
		write_cmos_sensor(0x0383,0x01);
		write_cmos_sensor(0x0385,0x01);
		write_cmos_sensor(0x0387,0x01);
		write_cmos_sensor(0x0900,0x00);
		write_cmos_sensor(0x0901,0x11);
		write_cmos_sensor(0x0902,0x00);
		write_cmos_sensor(0x3000,0x74);
		write_cmos_sensor(0x3001,0x00);
		write_cmos_sensor(0x305C,0x11);
		
		write_cmos_sensor(0x0112,0x0A);
		write_cmos_sensor(0x0113,0x0A);
		write_cmos_sensor(0x034C,0x14);
		write_cmos_sensor(0x034D,0xE0);
		write_cmos_sensor(0x034E,0x0F);
		write_cmos_sensor(0x034F,0xB0);
		write_cmos_sensor(0x0401,0x00);
		write_cmos_sensor(0x0404,0x00);
		write_cmos_sensor(0x0405,0x10);
		write_cmos_sensor(0x0408,0x00);
		write_cmos_sensor(0x0409,0x00);
		write_cmos_sensor(0x040A,0x00);
		write_cmos_sensor(0x040B,0x00);
		write_cmos_sensor(0x040C,0x14);
		write_cmos_sensor(0x040D,0xE0);
		write_cmos_sensor(0x040E,0x0F);
		write_cmos_sensor(0x040F,0xB0);
		
		write_cmos_sensor(0x0301,0x04);
		write_cmos_sensor(0x0303,0x02);
		write_cmos_sensor(0x0305,0x04);
		write_cmos_sensor(0x0306,0x00);
		write_cmos_sensor(0x0307,0xC7);
		write_cmos_sensor(0x0309,0x0A);
		write_cmos_sensor(0x030B,0x01);
		write_cmos_sensor(0x030D,0x0C);
		write_cmos_sensor(0x030E,0x02);
		write_cmos_sensor(0x030F,0xC1);
		write_cmos_sensor(0x0310,0x01);
		
		write_cmos_sensor(0x0820,0x16);
		write_cmos_sensor(0x0821,0x08);
		write_cmos_sensor(0x0822,0x00);
		write_cmos_sensor(0x0823,0x00);
		
		write_cmos_sensor(0x0202,0x10);
		write_cmos_sensor(0x0203,0x14);
		write_cmos_sensor(0x0224,0x01);
		write_cmos_sensor(0x0225,0xF4);
		
		write_cmos_sensor(0x0204,0x00);
		write_cmos_sensor(0x0205,0x00);
		write_cmos_sensor(0x0216,0x00);
		write_cmos_sensor(0x0217,0x00);
		write_cmos_sensor(0x020E,0x01);
		write_cmos_sensor(0x020F,0x00);
		write_cmos_sensor(0x0210,0x01);
		write_cmos_sensor(0x0211,0x00);
		write_cmos_sensor(0x0212,0x01);
		write_cmos_sensor(0x0213,0x00);
		write_cmos_sensor(0x0214,0x01);
		write_cmos_sensor(0x0215,0x00);
		
		write_cmos_sensor(0x3006,0x01);
		write_cmos_sensor(0x3007,0x02);
		write_cmos_sensor(0x31E0,0x03);
		write_cmos_sensor(0x31E1,0xFF);
		write_cmos_sensor(0x31E4,0x02);
		
		write_cmos_sensor(0x3A22,0x20);
		write_cmos_sensor(0x3A23,0x14);
		write_cmos_sensor(0x3A24,0xE0);
		write_cmos_sensor(0x3A25,0x0F);
		write_cmos_sensor(0x3A26,0xB0);
		write_cmos_sensor(0x3A2F,0x00);
		write_cmos_sensor(0x3A30,0x00);
		write_cmos_sensor(0x3A31,0x00);
		write_cmos_sensor(0x3A32,0x00);
		write_cmos_sensor(0x3A33,0x14);
		write_cmos_sensor(0x3A34,0xDF);
		write_cmos_sensor(0x3A35,0x0F);
		write_cmos_sensor(0x3A36,0xAF);
		write_cmos_sensor(0x3A37,0x00);
		write_cmos_sensor(0x3A38,0x00);
		write_cmos_sensor(0x3A39,0x00);
		
		write_cmos_sensor(0x3A21,0x00);
		
		write_cmos_sensor(0x3011,0x00);
		write_cmos_sensor(0x3013,0x01);
    	}
	/*PDAF*/
	/*PD_CAL_ENALBE*/
	write_cmos_sensor(0x3121,0x01);
	/*AREA MODE*/
	write_cmos_sensor(0x31B0,0x01);
	/*PD_OUT_EN=1*/
	write_cmos_sensor(0x3123,0x01);
	
    /*Fixed area mode*/
	/*Fixed area mode*/
	write_cmos_sensor(0x3150,0x00);
	write_cmos_sensor(0x3151,0x70);// X offset	112
	write_cmos_sensor(0x3152,0x00);
	write_cmos_sensor(0x3153,0x58);// Y offset 88 
	write_cmos_sensor(0x3154,0x02);// X size 640
	write_cmos_sensor(0x3155,0x80);
	write_cmos_sensor(0x3156,0x02);// Y size 640
	write_cmos_sensor(0x3157,0x80);

	write_cmos_sensor(0x0100,0x01);
}
#if MAIN_FULLSIZE
static void normal_video_setting_pdaf(kal_uint16 currefps)
{
	printk("E! currefps:%d\n",currefps);	
	write_cmos_sensor(0x0100,0x00);
			
	write_cmos_sensor(0x0114,0x03);
	write_cmos_sensor(0x0220,0x00);
	write_cmos_sensor(0x0221,0x11);
	write_cmos_sensor(0x0222,0x01);
	write_cmos_sensor(0x0340,0x0C);
	write_cmos_sensor(0x0341,0x30);
	write_cmos_sensor(0x0342,0x17);
	write_cmos_sensor(0x0343,0x88);
	write_cmos_sensor(0x0344,0x00);
	write_cmos_sensor(0x0345,0x00);
	write_cmos_sensor(0x0346,0x01);
	write_cmos_sensor(0x0347,0xF8);
	write_cmos_sensor(0x0348,0x14);
	write_cmos_sensor(0x0349,0xDF);
	write_cmos_sensor(0x034A,0x0D);
	write_cmos_sensor(0x034B,0xB7);
	write_cmos_sensor(0x0381,0x01);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0385,0x01);
	write_cmos_sensor(0x0387,0x01);
	write_cmos_sensor(0x0900,0x00);
	write_cmos_sensor(0x0901,0x11);
	write_cmos_sensor(0x0902,0x00);
	write_cmos_sensor(0x3000,0x74);
	write_cmos_sensor(0x3001,0x00);
	write_cmos_sensor(0x305C,0x11);
	
	write_cmos_sensor(0x0112,0x0A);
	write_cmos_sensor(0x0113,0x0A);
	write_cmos_sensor(0x034C,0x14);
	write_cmos_sensor(0x034D,0xE0);
	write_cmos_sensor(0x034E,0x0B);
	write_cmos_sensor(0x034F,0xBE);
	write_cmos_sensor(0x0401,0x00);
	write_cmos_sensor(0x0404,0x00);
	write_cmos_sensor(0x0405,0x10);
	write_cmos_sensor(0x0408,0x00);
	write_cmos_sensor(0x0409,0x00);
	write_cmos_sensor(0x040A,0x00);
	write_cmos_sensor(0x040B,0x02);
	write_cmos_sensor(0x040C,0x14);
	write_cmos_sensor(0x040D,0xE0);
	write_cmos_sensor(0x040E,0x0B);
	write_cmos_sensor(0x040F,0xBE);
	
	write_cmos_sensor(0x0301,0x04);
	write_cmos_sensor(0x0303,0x02);
	write_cmos_sensor(0x0305,0x04);
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0xBC);
	write_cmos_sensor(0x0309,0x0A);
	write_cmos_sensor(0x030B,0x01);
	write_cmos_sensor(0x030D,0x0C);
	write_cmos_sensor(0x030E,0x02);
	write_cmos_sensor(0x030F,0x99);
	write_cmos_sensor(0x0310,0x01);
	
	write_cmos_sensor(0x0820,0x14);
	write_cmos_sensor(0x0821,0xC8);
	write_cmos_sensor(0x0822,0x00);
	write_cmos_sensor(0x0823,0x00);
	
	write_cmos_sensor(0x0202,0x0C);
	write_cmos_sensor(0x0203,0x26);
	write_cmos_sensor(0x0224,0x01);
	write_cmos_sensor(0x0225,0xF4);

	write_cmos_sensor(0x0204,0x00);
	write_cmos_sensor(0x0205,0x00);
	write_cmos_sensor(0x0216,0x00);
	write_cmos_sensor(0x0217,0x00);
	write_cmos_sensor(0x020E,0x01);
	write_cmos_sensor(0x020F,0x00);
	write_cmos_sensor(0x0210,0x01);
	write_cmos_sensor(0x0211,0x00);
	write_cmos_sensor(0x0212,0x01);
	write_cmos_sensor(0x0213,0x00);
	write_cmos_sensor(0x0214,0x01);
	write_cmos_sensor(0x0215,0x00);
	
	write_cmos_sensor(0x3006,0x01);
	write_cmos_sensor(0x3007,0x02);
	write_cmos_sensor(0x31E0,0x03);
	write_cmos_sensor(0x31E1,0xFF);
	write_cmos_sensor(0x31E4,0x02);

	write_cmos_sensor(0x3A22,0x20);
	write_cmos_sensor(0x3A23,0x14);
	write_cmos_sensor(0x3A24,0xE0);
	write_cmos_sensor(0x3A25,0x0B);
	write_cmos_sensor(0x3A26,0xC0);
	write_cmos_sensor(0x3A2F,0x00);
	write_cmos_sensor(0x3A30,0x00);
	write_cmos_sensor(0x3A31,0x01);
	write_cmos_sensor(0x3A32,0xF8);
	write_cmos_sensor(0x3A33,0x14);
	write_cmos_sensor(0x3A34,0xDF);
	write_cmos_sensor(0x3A35,0x0D);
	write_cmos_sensor(0x3A36,0xB7);
	write_cmos_sensor(0x3A37,0x00);
	write_cmos_sensor(0x3A38,0x00);
	write_cmos_sensor(0x3A39,0x00);

	write_cmos_sensor(0x3A21,0x00);

	write_cmos_sensor(0x3011,0x00);
	write_cmos_sensor(0x3013,0x01);
	//need Sony support
	write_cmos_sensor(0x3001,0x01);/*bit[0]PDAF enable during HDR on*/
	/*PDAF*/
	/*PD_CAL_ENALBE*/
	write_cmos_sensor(0x3121,0x01);
	/*AREA MODE*/
	write_cmos_sensor(0x31B0,0x01);// 8x6 output
	/*PD_OUT_EN=1*/
	write_cmos_sensor(0x3123,0x01);
	write_cmos_sensor(0x0100,0x01);
	/*Fixed area mode*/
    write_cmos_sensor(0x3150,0x00);
    write_cmos_sensor(0x3151,0x70);// X offset 112
    write_cmos_sensor(0x3152,0x00);
    write_cmos_sensor(0x3153,0x02);// Y offset 2 
    write_cmos_sensor(0x3154,0x02);// X size 640
    write_cmos_sensor(0x3155,0x80);
    write_cmos_sensor(0x3156,0x01);// Y size 500
    write_cmos_sensor(0x3157,0xF4);
	
	write_cmos_sensor(0x0100,0x01);
}

static void normal_video_setting(kal_uint16 currefps)
{
    printk("E! currefps:%d\n",currefps);    
	write_cmos_sensor(0x0100,0x00);
		
	write_cmos_sensor(0x0114,0x03);
	write_cmos_sensor(0x0220,0x00);
	write_cmos_sensor(0x0221,0x11);
	write_cmos_sensor(0x0222,0x01);
	write_cmos_sensor(0x0340,0x0C);
	write_cmos_sensor(0x0341,0x30);
	write_cmos_sensor(0x0342,0x17);
	write_cmos_sensor(0x0343,0x88);
	write_cmos_sensor(0x0344,0x00);
	write_cmos_sensor(0x0345,0x00);
	write_cmos_sensor(0x0346,0x01);
	write_cmos_sensor(0x0347,0xF8);
	write_cmos_sensor(0x0348,0x14);
	write_cmos_sensor(0x0349,0xDF);
	write_cmos_sensor(0x034A,0x0D);
	write_cmos_sensor(0x034B,0xB7);
	write_cmos_sensor(0x0381,0x01);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0385,0x01);
	write_cmos_sensor(0x0387,0x01);
	write_cmos_sensor(0x0900,0x00);
	write_cmos_sensor(0x0901,0x11);
	write_cmos_sensor(0x0902,0x00);
	write_cmos_sensor(0x3000,0x74);
	write_cmos_sensor(0x3001,0x00);
	write_cmos_sensor(0x305C,0x11);

	write_cmos_sensor(0x0112,0x0A);
	write_cmos_sensor(0x0113,0x0A);
	write_cmos_sensor(0x034C,0x14);
	write_cmos_sensor(0x034D,0xE0);
	write_cmos_sensor(0x034E,0x0B);
	write_cmos_sensor(0x034F,0xBE);
	write_cmos_sensor(0x0401,0x00);
	write_cmos_sensor(0x0404,0x00);
	write_cmos_sensor(0x0405,0x10);
	write_cmos_sensor(0x0408,0x00);
	write_cmos_sensor(0x0409,0x00);
	write_cmos_sensor(0x040A,0x00);
	write_cmos_sensor(0x040B,0x02);
	write_cmos_sensor(0x040C,0x14);
	write_cmos_sensor(0x040D,0xE0);
	write_cmos_sensor(0x040E,0x0B);
	write_cmos_sensor(0x040F,0xBE);

	write_cmos_sensor(0x0301,0x04);
	write_cmos_sensor(0x0303,0x02);
	write_cmos_sensor(0x0305,0x04);
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0xBC);
	write_cmos_sensor(0x0309,0x0A);
	write_cmos_sensor(0x030B,0x01);
	write_cmos_sensor(0x030D,0x0C);
	write_cmos_sensor(0x030E,0x02);
	write_cmos_sensor(0x030F,0x99);
	write_cmos_sensor(0x0310,0x01);

	write_cmos_sensor(0x0820,0x14);
	write_cmos_sensor(0x0821,0xC8);
	write_cmos_sensor(0x0822,0x00);
	write_cmos_sensor(0x0823,0x00);

	write_cmos_sensor(0x0202,0x0C);
	write_cmos_sensor(0x0203,0x26);
	write_cmos_sensor(0x0224,0x01);
	write_cmos_sensor(0x0225,0xF4);

	write_cmos_sensor(0x0204,0x00);
	write_cmos_sensor(0x0205,0x00);
	write_cmos_sensor(0x0216,0x00);
	write_cmos_sensor(0x0217,0x00);
	write_cmos_sensor(0x020E,0x01);
	write_cmos_sensor(0x020F,0x00);
	write_cmos_sensor(0x0210,0x01);
	write_cmos_sensor(0x0211,0x00);
	write_cmos_sensor(0x0212,0x01);
	write_cmos_sensor(0x0213,0x00);
	write_cmos_sensor(0x0214,0x01);
	write_cmos_sensor(0x0215,0x00);

	write_cmos_sensor(0x3006,0x01);
	write_cmos_sensor(0x3007,0x02);
	write_cmos_sensor(0x31E0,0x03);
	write_cmos_sensor(0x31E1,0xFF);
	write_cmos_sensor(0x31E4,0x02);

	write_cmos_sensor(0x3A22,0x20);
	write_cmos_sensor(0x3A23,0x14);
	write_cmos_sensor(0x3A24,0xE0);
	write_cmos_sensor(0x3A25,0x0B);
	write_cmos_sensor(0x3A26,0xC0);
	write_cmos_sensor(0x3A2F,0x00);
	write_cmos_sensor(0x3A30,0x00);
	write_cmos_sensor(0x3A31,0x01);
	write_cmos_sensor(0x3A32,0xF8);
	write_cmos_sensor(0x3A33,0x14);
	write_cmos_sensor(0x3A34,0xDF);
	write_cmos_sensor(0x3A35,0x0D);
	write_cmos_sensor(0x3A36,0xB7);
	write_cmos_sensor(0x3A37,0x00);
	write_cmos_sensor(0x3A38,0x00);
	write_cmos_sensor(0x3A39,0x00);

	write_cmos_sensor(0x3A21,0x00);

	write_cmos_sensor(0x3011,0x00);
	write_cmos_sensor(0x3013,0x01);

	
	write_cmos_sensor(0x0100,0x01);

}
#endif

static void hs_video_setting(void)
{
    printk("hs_video_setting E\n");
	write_cmos_sensor(0x0100,0x00);

write_cmos_sensor(0x0114,0x03);
write_cmos_sensor(0x0220,0x00);
write_cmos_sensor(0x0221,0x11);
write_cmos_sensor(0x0222,0x01);
write_cmos_sensor(0x0340,0x03);
write_cmos_sensor(0x0341,0x38);
write_cmos_sensor(0x0342,0x17);
write_cmos_sensor(0x0343,0x88);
write_cmos_sensor(0x0344,0x00);
write_cmos_sensor(0x0345,0x00);
write_cmos_sensor(0x0346,0x02);
write_cmos_sensor(0x0347,0x38);
write_cmos_sensor(0x0348,0x14);
write_cmos_sensor(0x0349,0xDF);
write_cmos_sensor(0x034A,0x0D);
write_cmos_sensor(0x034B,0x77);
write_cmos_sensor(0x0381,0x01);
write_cmos_sensor(0x0383,0x01);
write_cmos_sensor(0x0385,0x01);
write_cmos_sensor(0x0387,0x01);
write_cmos_sensor(0x0900,0x01);
write_cmos_sensor(0x0901,0x44);
write_cmos_sensor(0x0902,0x00);
write_cmos_sensor(0x3000,0x74);
write_cmos_sensor(0x3001,0x00);
write_cmos_sensor(0x305C,0x11);

write_cmos_sensor(0x0112,0x0A);
write_cmos_sensor(0x0113,0x0A);
write_cmos_sensor(0x034C,0x05);
write_cmos_sensor(0x034D,0x00);
write_cmos_sensor(0x034E,0x02);
write_cmos_sensor(0x034F,0xD0);
write_cmos_sensor(0x0401,0x00);
write_cmos_sensor(0x0404,0x00);
write_cmos_sensor(0x0405,0x10);
write_cmos_sensor(0x0408,0x00);
write_cmos_sensor(0x0409,0x1C);
write_cmos_sensor(0x040A,0x00);
write_cmos_sensor(0x040B,0x00);
write_cmos_sensor(0x040C,0x05);
write_cmos_sensor(0x040D,0x00);
write_cmos_sensor(0x040E,0x02);
write_cmos_sensor(0x040F,0xD0);

write_cmos_sensor(0x0301,0x04);
write_cmos_sensor(0x0303,0x02);
write_cmos_sensor(0x0305,0x04);
write_cmos_sensor(0x0306,0x00);
write_cmos_sensor(0x0307,0xC7);
write_cmos_sensor(0x0309,0x0A);
write_cmos_sensor(0x030B,0x01);
write_cmos_sensor(0x030D,0x0C);
write_cmos_sensor(0x030E,0x00);
write_cmos_sensor(0x030F,0xC8);
write_cmos_sensor(0x0310,0x01);

write_cmos_sensor(0x0820,0x06);
write_cmos_sensor(0x0821,0x40);
write_cmos_sensor(0x0822,0x00);
write_cmos_sensor(0x0823,0x00);

write_cmos_sensor(0x0202,0x03);
write_cmos_sensor(0x0203,0x2E);
write_cmos_sensor(0x0224,0x01);
write_cmos_sensor(0x0225,0xF4);

write_cmos_sensor(0x0204,0x00);
write_cmos_sensor(0x0205,0x00);
write_cmos_sensor(0x0216,0x00);
write_cmos_sensor(0x0217,0x00);
write_cmos_sensor(0x020E,0x01);
write_cmos_sensor(0x020F,0x00);
write_cmos_sensor(0x0210,0x01);
write_cmos_sensor(0x0211,0x00);
write_cmos_sensor(0x0212,0x01);
write_cmos_sensor(0x0213,0x00);
write_cmos_sensor(0x0214,0x01);
write_cmos_sensor(0x0215,0x00);

write_cmos_sensor(0x3006,0x01);
write_cmos_sensor(0x3007,0x02);
write_cmos_sensor(0x31E0,0x03);
write_cmos_sensor(0x31E1,0xFF);
write_cmos_sensor(0x31E4,0x02);

write_cmos_sensor(0x3A22,0x20);
write_cmos_sensor(0x3A23,0x14);
write_cmos_sensor(0x3A24,0xE0);
write_cmos_sensor(0x3A25,0x02);
write_cmos_sensor(0x3A26,0xD0);
write_cmos_sensor(0x3A2F,0x00);
write_cmos_sensor(0x3A30,0x00);
write_cmos_sensor(0x3A31,0x02);
write_cmos_sensor(0x3A32,0x38);
write_cmos_sensor(0x3A33,0x14);
write_cmos_sensor(0x3A34,0xDF);
write_cmos_sensor(0x3A35,0x0D);
write_cmos_sensor(0x3A36,0x77);
write_cmos_sensor(0x3A37,0x00);
write_cmos_sensor(0x3A38,0x02);
write_cmos_sensor(0x3A39,0x00);

write_cmos_sensor(0x3A21,0x00);

write_cmos_sensor(0x3011,0x00);
write_cmos_sensor(0x3013,0x01);

	write_cmos_sensor(0x0100,0x01);

}

static void slim_video_setting(void)
{
    printk("slim_video_setting E\n");
    //@@video_720p_30fps_800Mbps

	write_cmos_sensor(0x0100,0x00);
	
write_cmos_sensor(0x0114,0x03);
write_cmos_sensor(0x0220,0x00);
write_cmos_sensor(0x0221,0x11);
write_cmos_sensor(0x0222,0x01);
write_cmos_sensor(0x0340,0x04);
write_cmos_sensor(0x0341,0x58);
write_cmos_sensor(0x0342,0x17);
write_cmos_sensor(0x0343,0x88);
write_cmos_sensor(0x0344,0x00);
write_cmos_sensor(0x0345,0x00);
write_cmos_sensor(0x0346,0x01);
write_cmos_sensor(0x0347,0xF8);
write_cmos_sensor(0x0348,0x14);
write_cmos_sensor(0x0349,0xDF);
write_cmos_sensor(0x034A,0x0D);
write_cmos_sensor(0x034B,0xB7);
write_cmos_sensor(0x0381,0x01);
write_cmos_sensor(0x0383,0x01);
write_cmos_sensor(0x0385,0x01);
write_cmos_sensor(0x0387,0x01);
write_cmos_sensor(0x0900,0x01);
write_cmos_sensor(0x0901,0x44);
write_cmos_sensor(0x0902,0x00);
write_cmos_sensor(0x3000,0x74);
write_cmos_sensor(0x3001,0x00);
write_cmos_sensor(0x305C,0x11);

write_cmos_sensor(0x0112,0x0A);
write_cmos_sensor(0x0113,0x0A);
write_cmos_sensor(0x034C,0x05);
write_cmos_sensor(0x034D,0x00);
write_cmos_sensor(0x034E,0x02);
write_cmos_sensor(0x034F,0xD0);
write_cmos_sensor(0x0401,0x00);
write_cmos_sensor(0x0404,0x00);
write_cmos_sensor(0x0405,0x10);
write_cmos_sensor(0x0408,0x00);
write_cmos_sensor(0x0409,0x1C);
write_cmos_sensor(0x040A,0x00);
write_cmos_sensor(0x040B,0x10);
write_cmos_sensor(0x040C,0x05);
write_cmos_sensor(0x040D,0x00);
write_cmos_sensor(0x040E,0x02);
write_cmos_sensor(0x040F,0xD0);

write_cmos_sensor(0x0301,0x04);
write_cmos_sensor(0x0303,0x02);
write_cmos_sensor(0x0305,0x04);
write_cmos_sensor(0x0306,0x00);
write_cmos_sensor(0x0307,0x43);
write_cmos_sensor(0x0309,0x0A);
write_cmos_sensor(0x030B,0x02);
write_cmos_sensor(0x030D,0x0C);
write_cmos_sensor(0x030E,0x01);
write_cmos_sensor(0x030F,0x2C);
write_cmos_sensor(0x0310,0x01);

write_cmos_sensor(0x0820,0x04);
write_cmos_sensor(0x0821,0xB0);
write_cmos_sensor(0x0822,0x00);
write_cmos_sensor(0x0823,0x00);

write_cmos_sensor(0x0202,0x04);
write_cmos_sensor(0x0203,0x4E);
write_cmos_sensor(0x0224,0x01);
write_cmos_sensor(0x0225,0xF4);

write_cmos_sensor(0x0204,0x00);
write_cmos_sensor(0x0205,0x00);
write_cmos_sensor(0x0216,0x00);
write_cmos_sensor(0x0217,0x00);
write_cmos_sensor(0x020E,0x01);
write_cmos_sensor(0x020F,0x00);
write_cmos_sensor(0x0210,0x01);
write_cmos_sensor(0x0211,0x00);
write_cmos_sensor(0x0212,0x01);
write_cmos_sensor(0x0213,0x00);
write_cmos_sensor(0x0214,0x01);
write_cmos_sensor(0x0215,0x00);

write_cmos_sensor(0x3006,0x01);
write_cmos_sensor(0x3007,0x02);
write_cmos_sensor(0x31E0,0x03);
write_cmos_sensor(0x31E1,0xFF);
write_cmos_sensor(0x31E4,0x02);

write_cmos_sensor(0x3A22,0x20);
write_cmos_sensor(0x3A23,0x14);
write_cmos_sensor(0x3A24,0xE0);
write_cmos_sensor(0x3A25,0x02);
write_cmos_sensor(0x3A26,0xF0);
write_cmos_sensor(0x3A2F,0x00);
write_cmos_sensor(0x3A30,0x00);
write_cmos_sensor(0x3A31,0x01);
write_cmos_sensor(0x3A32,0xF8);
write_cmos_sensor(0x3A33,0x14);
write_cmos_sensor(0x3A34,0xDF);
write_cmos_sensor(0x3A35,0x0D);
write_cmos_sensor(0x3A36,0xB7);
write_cmos_sensor(0x3A37,0x00);
write_cmos_sensor(0x3A38,0x02);
write_cmos_sensor(0x3A39,0x00);

write_cmos_sensor(0x3A21,0x00);

write_cmos_sensor(0x3011,0x00);
write_cmos_sensor(0x3013,0x01);

	write_cmos_sensor(0x0100,0x01);

}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
    printk("enable: %d\n", enable);

    if (enable) {
        write_cmos_sensor(0x0601, 0x02);
    } else {
        write_cmos_sensor(0x0601, 0x00);
    }
    spin_lock(&imgsensor_drv_lock);
    imgsensor.test_pattern = enable;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*    get_imgsensor_id
*
* DESCRIPTION
*    This function get the sensor ID
*
* PARAMETERS
*    *sensorID : return the sensor ID
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
	printk("[Rick]get_imgsensor_id imx230, Enter \n");
    //sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            *sensor_id = return_sensor_id();
			printk("[Rick][imx230][get_imgsensor_id] return sensor id: 0x%x\n", *sensor_id);
			//new version for LiteOn
			if((*sensor_id == 0x0230) && (get_sensor_version() == 1))
			{
				printk("[Rick][imx230] get_imgsensor_id, new LiteOn sensor version \n");
				*sensor_id = 0x0231;
			}
			//new version for LiteOn
            if (*sensor_id == imgsensor_info.sensor_id) {
				printk("[Rick][imx230][get_imgsensor_id] i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
                return ERROR_NONE;
            }
            printk("[Rick][imx230][get_imgsensor_id] Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        retry = 2;
    }
    if (*sensor_id != imgsensor_info.sensor_id) {
        // if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF
        *sensor_id = 0xFFFFFFFF;
		printk("[Rick][imx230][get_imgsensor_id] ERROR_SENSOR_CONNECT_FAIL, sensor_id: 0x%x\n", *sensor_id);
        return ERROR_SENSOR_CONNECT_FAIL;
    }
	printk("[Rick][get_imgsensor_id] sensor id: 0x%x   end \n", *sensor_id);
    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*    open
*
* DESCRIPTION
*    This function initialize the registers of CMOS sensor
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    kal_uint32 sensor_id = 0;
    LOG_1;
    LOG_2;
    //sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            sensor_id = return_sensor_id();
			//new version for LiteOn
			if(get_sensor_version() == 1)
			{
				printk("[Rick][open]imx230, new LiteOn sensor version \n");
				//sensor_id = 0x0231;
				return ERROR_SENSOR_CONNECT_FAIL;
			}
			//new version for LiteOn
            if (sensor_id == imgsensor_info.sensor_id) {
                printk("[Rick][open]imx230, i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
                break;
            }
            printk("open, Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        if (sensor_id == imgsensor_info.sensor_id){
			printk("[Rick][open]imx230, sensor_id == imgsensor_info.sensor_id \n");
            break;
		}
        retry = 2;
    }
    if (imgsensor_info.sensor_id != sensor_id){
		printk("[Rick][open]imx230, imgsensor_info.sensor_id != sensor_id \n");
        return ERROR_SENSOR_CONNECT_FAIL;
	}
    /* initail sequence write in  */
    sensor_init();
	imx230_apply_SPC();//20160428, RickLiu
	printk("[Rick] imx230 Init Done! \n");
    spin_lock(&imgsensor_drv_lock);

    imgsensor.autoflicker_en= KAL_FALSE;
    imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.dummy_pixel = 0;
    imgsensor.dummy_line = 0;
    imgsensor.ihdr_mode = 0;
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    spin_unlock(&imgsensor_drv_lock);

    return ERROR_NONE;
}    /*    open  */



/*************************************************************************
* FUNCTION
*    close
*
* DESCRIPTION
*
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
    printk("close E\n");

    /*No Need to implement this function*/

    return ERROR_NONE;
}    /*    close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*    This function start the sensor preview.
*
* PARAMETERS
*    *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    printk("imx230mipi_sensor preview E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
	if(imgsensor.ihdr_mode == 2) {
		//printk("[Rick] preview HDR + PDAF + \n");
        preview_setting_HDR_ES2(); /*HDR + PDAF*/
		//printk("[Rick] preview HDR + PDAF - \n");
    }else{
		//printk("[Rick] preview PDAF + \n");
		preview_setting(); /*PDAF only*/
		//printk("[Rick] preview PDAF - \n");
	}
	set_mirror_flip(sensor_config_data->SensorImageMirror);
    return ERROR_NONE;
}    /*    preview   */

/*************************************************************************
* FUNCTION
*    capture
*
* DESCRIPTION
*    This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    printk("capture E\n");
	
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
    if(imgsensor.ihdr_mode == 2) 
		capture_setting_HDR_ES2();/*HDR+PDAF*/
	else if(imgsensor.pdaf_mode == 1)
		capture_setting_pdaf(imgsensor.current_fps);/*PDAF only*/
    else
		capture_setting(imgsensor.current_fps);/*Full mode*/

    return ERROR_NONE;
}    /* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    printk("normal_video E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
    imgsensor.pclk = imgsensor_info.normal_video.pclk;
    imgsensor.line_length = imgsensor_info.normal_video.linelength;
    imgsensor.frame_length = imgsensor_info.normal_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
    //imgsensor.current_fps = 300;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
	#if MAIN_FULLSIZE
	if(imgsensor.pdaf_mode == 1)
		normal_video_setting_pdaf(imgsensor.current_fps);
	else
    	normal_video_setting(imgsensor.current_fps);	
	#elif MAIN_HALFSIZE
	preview_setting();
	#endif
	set_mirror_flip(sensor_config_data->SensorImageMirror);
    return ERROR_NONE;
}    /*    normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    printk("hs_video E\n");

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
	set_mirror_flip(sensor_config_data->SensorImageMirror);
    return ERROR_NONE;
}    /*    hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    printk("E\n");

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
	set_mirror_flip(sensor_config_data->SensorImageMirror);

    return ERROR_NONE;
}    /*    slim_video     */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
    printk("E\n");
    sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
    sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

    sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
    sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

    sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
    sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


    sensor_resolution->SensorHighSpeedVideoWidth     = imgsensor_info.hs_video.grabwindow_width;
    sensor_resolution->SensorHighSpeedVideoHeight     = imgsensor_info.hs_video.grabwindow_height;

    sensor_resolution->SensorSlimVideoWidth     = imgsensor_info.slim_video.grabwindow_width;
    sensor_resolution->SensorSlimVideoHeight     = imgsensor_info.slim_video.grabwindow_height;
    return ERROR_NONE;
}    /*    get_resolution    */

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

    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;          /* The frame of setting shutter default 0 for TG int */
    sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;    /* The frame of setting sensor gain */
    sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
    sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
    sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
    sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	sensor_info->PDAF_Support = 2; /*0: NO PDAF, 1: PDAF Raw Data mode, 2:PDAF VC mode*/

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
    sensor_info->SensorHightSampling = 0;    // 0 is default 1x
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
}    /*    get_info  */


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
}    /* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{//This Function not used after ROME
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


static kal_uint32 imx230_awb_gain(SET_SENSOR_AWB_GAIN *pSetSensorAWB)
{ 
    UINT32 rgain_32, grgain_32, gbgain_32, bgain_32;
	//LOG_INF("imx230_awb_gain\n");
    grgain_32 = (pSetSensorAWB->ABS_GAIN_GR << 8) >> 9;
    rgain_32 = (pSetSensorAWB->ABS_GAIN_R << 8) >> 9;
    bgain_32 = (pSetSensorAWB->ABS_GAIN_B << 8) >> 9;
    gbgain_32 = (pSetSensorAWB->ABS_GAIN_GB << 8) >> 9;

    printk("[imx230_awb_gain] ABS_GAIN_GR:%d, grgain_32:%d\n", pSetSensorAWB->ABS_GAIN_GR, grgain_32);
    printk("[imx230_awb_gain] ABS_GAIN_R:%d, rgain_32:%d\n", pSetSensorAWB->ABS_GAIN_R, rgain_32);
    printk("[imx230_awb_gain] ABS_GAIN_B:%d, bgain_32:%d\n", pSetSensorAWB->ABS_GAIN_B, bgain_32);
    printk("[imx230_awb_gain] ABS_GAIN_GB:%d, gbgain_32:%d\n", pSetSensorAWB->ABS_GAIN_GB, gbgain_32);

    write_cmos_sensor(0x0b8e, (grgain_32 >> 8) & 0xFF);
    write_cmos_sensor(0x0b8f, grgain_32 & 0xFF);
    write_cmos_sensor(0x0b90, (rgain_32 >> 8) & 0xFF);
    write_cmos_sensor(0x0b91, rgain_32 & 0xFF);
    write_cmos_sensor(0x0b92, (bgain_32 >> 8) & 0xFF);
    write_cmos_sensor(0x0b93, bgain_32 & 0xFF);
    write_cmos_sensor(0x0b94, (gbgain_32 >> 8) & 0xFF);
    write_cmos_sensor(0x0b95, gbgain_32 & 0xFF);
	
    return ERROR_NONE;
}


static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                             UINT8 *feature_para,UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16=(UINT16 *) feature_para;
    UINT16 *feature_data_16=(UINT16 *) feature_para;
    UINT32 *feature_return_para_32=(UINT32 *) feature_para;
    UINT32 *feature_data_32=(UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long*)feature_para;
	//unsigned long long *feature_return_data = (unsigned long long*)feature_para;

    SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    SENSOR_VC_INFO_STRUCT *pvcinfo;
    SET_SENSOR_AWB_GAIN *pSetSensorAWB=(SET_SENSOR_AWB_GAIN *)feature_para;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

    printk("feature_id = %d\n", feature_id);
    switch (feature_id) {
        case SENSOR_FEATURE_GET_PERIOD:
            *feature_return_para_16++ = imgsensor.line_length;
            *feature_return_para_16 = imgsensor.frame_length;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            printk("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
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
            set_gain((UINT16) *feature_data);
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
            set_video_mode(*feature_data_16);
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
            get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, (MUINT32 *)(uintptr_t)(*(feature_data+1)));
			break;
		case SENSOR_FEATURE_GET_PDAF_DATA:	
			//printk("SENSOR_FEATURE_GET_PDAF_DATA\n"); 
			read_imx230_DCC((kal_uint16 )(*feature_data),(char*)(uintptr_t)(*(feature_data+1)),(kal_uint32)(*(feature_data+2)));
            break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
            break;
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
            *feature_return_para_32 = imgsensor_info.checksum_value;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_FRAMERATE:
            //printk("current fps :%d\n", (UINT32)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_GET_CROP_INFO:
            //printk("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);
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
		/*HDR CMD*/
		case SENSOR_FEATURE_SET_HDR:
            //printk("[Rick] SENSOR_FEATURE_SET_HDR \n");
            spin_lock(&imgsensor_drv_lock);
			imgsensor.ihdr_mode = *feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            //printk("[Rick] SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN \n");
            ihdr_write_shutter_gain(*feature_data,*(feature_data+1),*(feature_data+2));
            break;
		case SENSOR_FEATURE_GET_VC_INFO:
            //printk("[Rick] SENSOR_FEATURE_GET_VC_INFO \n");
            pvcinfo = (SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
            switch (*feature_data_32) {
            case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[1],sizeof(SENSOR_VC_INFO_STRUCT));
                break;
            case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[2],sizeof(SENSOR_VC_INFO_STRUCT));
                break;
            case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            default:
                memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[0],sizeof(SENSOR_VC_INFO_STRUCT));
                break;
            }
            break;
		case SENSOR_FEATURE_SET_AWB_GAIN:
            imx230_awb_gain(pSetSensorAWB);
            break;
		/*END OF HDR CMD*/
		/*PDAF CMD*/
		case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
			//printk("[Rick] SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY \n");
			//PDAF capacity enable or not, 2p8 only full size support PDAF
			switch (*feature_data) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1; 
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
					break;
				default:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
			}
			break;
		case SENSOR_FEATURE_SET_PDAF:
			//printk("[Rick] SENSOR_FEATURE_SET_PDAF \n");
			imgsensor.pdaf_mode= *feature_data_16;
			break;
		/*End of PDAF*/
        default:
            break;
    }

    return ERROR_NONE;
}    /*    feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
    open,
    get_info,
    get_resolution,
    feature_control,
    control,
    close
};

UINT32 IMX230_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&sensor_func;
    return ERROR_NONE;
}    /*    IMX230_MIPI_RAW_SensorInit    */
