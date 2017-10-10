#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>
#include "kd_camera_typedef.h"


#define PFX "IMX230_pdafotp"
//#define LOG_INF(format, args...)	xlog_printk(ANDROID_LOG_INFO   , PFX, "[%s] " format, __FUNCTION__, ##args)
#define LOG_INF(format, args...)	pr_debug(PFX, "[%s] " format, __FUNCTION__, ##args)
#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern void kdSetI2CSpeed(u16 i2cSpeed);
//extern int iBurstWriteReg_multi(u8 *pData, u32 bytes, u16 i2cId, u16 transfer_length);
extern int iMultiReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId, u8 number);


#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)

#define IMX230_EEPROM_READ_ID  0xA0
#define IMX230_EEPROM_WRITE_ID   0xA1
#define IMX230_I2C_SPEED        400  
#define IMX230_MAX_OFFSET		0xFFFF

#define DATA_SIZE 2048

BYTE IMX230_DCC_data[96]= {0};
BYTE IMX230_SPC_data[352]= {0};
BYTE EEPROM_version[2]={0};


static bool get_done = false;
static int last_size = 0;
static int last_offset = 0;


static bool selective_read_eeprom(u16 addr, BYTE* data)
{
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	printk("[Rick] imx230,Start selective_read_eeprom \n");
    if(addr > IMX230_MAX_OFFSET){
		printk("[Rick] selective_read_eeprom addr > IMX230_MAX_OFFSET FAIL!! \n");
        return false;
	}
	kdSetI2CSpeed(IMX230_I2C_SPEED);

	if(iReadRegI2C(pu_send_cmd, 2, (u8*)data, 1, IMX230_EEPROM_READ_ID)<0){
		printk("[Rick] imx230, selective_read_eeprom iReadRegI2C FAIL!! \n");
		return false;
	}
	printk("[Rick] imx230,selective_read_eeprom iReadRegI2C OK!! \n");
    return true;
}

static bool _read_imx230_eeprom(u16 addr, BYTE* data, u8 size ){
	int i = 0;
	u16 offset = addr;
	//LOG_INF("enter _read_eeprom size = %d\n",size);
	//printk("[Rick] Start read imx230 DCC\n");
	
	for(i = 0; i < 96; i++) { // Rick Liu, fix cannot read DCC data issue
		//printk("[Rick] Start read imx230 DCC for loop \n");
		if(!selective_read_eeprom(offset, &data[i])){
			return false;
		}
		printk("[Rick] imx230,read_eeprom 0x%0x %d\n",offset, data[i]);
		offset++;
	}
	get_done = true;
	last_size = size;
	last_offset = addr;
    return true;
}

static bool _read_imx230_eeprom_SPC(u16 addr, BYTE* data, u8 size_spc ){
	int i = 0;
	u16 offset = addr;
	//printk("[Rick] enter _read_eeprom spc size_spc = %d\n", size_spc);
	//for(i = 0; i < size_spc; i++) {
	for(i = 0; i < 352; i++) { //Rick test
		printk("[Rick] imx230,read_eeprom i= %d \n", i);
		if(!selective_read_eeprom(offset, &data[i])){
			//printk("[Rick] read_eeprom, selective_read_eeprom fail \n");
			return false;
		}
		printk("[Rick] imx230,read_eeprom 0x%0x %d\n",offset, data[i]);
		offset++;
	}
	get_done = true;
	last_size = size_spc;
	last_offset = addr;
    return true;
}

//< Read eeprom version
void read_eeprom_version(BYTE* data){
	//int i;
	int addr = 0x3; //Layout version, 0x0C:NEW
	int addr2 = 0x4; //Factory version, 0x1:LiteOn, 0x2:Truly
	int size_spc = 1;
	
	selective_read_eeprom(addr, &data[0]);
	selective_read_eeprom(addr2, &data[1]);
	printk("[Rick] imx230,read_eeprom Layout version addr:0x%0x, return:0x%x \n",addr, data[0]);
	printk("[Rick] imx230,read_eeprom Factory version addr2:0x%0x, return:0x%x \n",addr2, data[1]);

	//memcpy(data, EEPROM_version , size_spc);
	get_done = true;
	last_size = size_spc;
	last_offset = addr;
	return;
}
//> Read eeprom version

void read_imx230_SPC(BYTE* data){
	//int i;
	int addr = 0x2E8;
	int size_spc = 352;
	
	printk("[Rick] read imx230 SPC, size_spc = %d , last_size = %d \n", size_spc, last_size);
	
	if(!get_done || last_size != size_spc || last_offset != addr) {
		if(!_read_imx230_eeprom_SPC(addr, IMX230_SPC_data, size_spc)){
			get_done = 0;
            last_size = 0;
            last_offset = 0;
			//return -1;
			return;
		}
	}

	memcpy(data, IMX230_SPC_data , size_spc);
    //return 0;
	return;
}


void read_imx230_DCC( u16 addr,BYTE* data, u32 size){
	//int i;
	u16 addr_ = addr;
	u32 size_ = size;
	addr_ = 0x448;
	size_ = 96;
	
	printk("[Rick] read imx230 DCC, size = %d\n", size);
	
	if(!get_done || last_size != size_ || last_offset != addr_) {
		printk("[Rick] Before read imx230 DCC\n");
		if(!_read_imx230_eeprom(addr_, IMX230_DCC_data, size_)){
			get_done = 0;
            last_size = 0;
            last_offset = 0;
			//return -1;
			return;
		}
	}
	//printk("data addr 0x%x\n",data);
	memcpy(data, IMX230_DCC_data , size);
    //return 0;
	return;
}


