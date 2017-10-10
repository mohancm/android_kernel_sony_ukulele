/*
 * ALSA SoC Texas Instruments TAS2555 High Performance 4W Smart Amplifier
 *
 * Copyright (C) 2015 Texas Instruments, Inc.
 *
 * Author: Peter Ujfalusi <peter.ujfalusi@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#define DEBUG
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
//#include <dt-bindings/sound/tas2555.h>
#include <linux/dma-mapping.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>

#include "tas2555.h"

#define MTK_PLATFORM_DRIVER
#define PPC_WITH_DRIVER_VERSION		0x010bc000
#define TAS2555_CAL_NAME    "/data/prod/tas2555_cal.bin"

#ifdef MTK_PLATFORM_DRIVER
#include "mt_soc_codec_63xx.h"
#include "AudDrv_Type_Def.h"
#include "AudDrv_Def.h"
#include "mt_soc_analog_type.h"
#include "AudDrv_Afe.h"


extern uint32 mBlockSampleRate[AUDIO_ANALOG_DEVICE_INOUT_MAX];
extern int mt63xx_codec_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *Daiport);

#define MT_SOC_CODEC_TAS2555 "tas2555.4-004c"        

//set default PLL CLKIN to GPI2 (MCLK) = 0x0D
#define TAS2555_DEFAULT_PLL_CLKIN 0x0D

#define P_GPIO_TAS2555_RST        21
#define GPIO_TAS2555_RST_PIN         (P_GPIO_TAS2555_RST | 0x80000000)

#define GPIO_OUT_ONE  1
#define GPIO_OUT_ZERO 0
#define GPIO_MODE_06 6
#define GPIO_MODE_03 3
#define GPIO_MODE_00 0
#define GPIO_DIR_OUT 1
extern int mt_set_gpio_mode(unsigned long pin, unsigned long mode);
extern int mt_set_gpio_dir(unsigned long pin, unsigned long dir);
extern int mt_set_gpio_out(unsigned long pin, unsigned long output);
extern int mt_set_gpio_pull_enable(unsigned long pin,  unsigned long enable);
extern int mt_set_gpio_pull_select(unsigned long pin, unsigned long select);

//extern int AudDrv_GPIO_SMP_RST_PIN(int bEnable);                  //wilbur use pinctl GPIO 

#endif

#define KCONTROL_CODEC
#undef ENABLE_GPIO_RESET
#define ENABLE_TILOAD			//only enable this for in-system tuning or debug, not for production systems
#ifdef ENABLE_TILOAD
#include "tiload.h"
#endif

#define TAS2555_FW_NAME     "tas2555_uCDSP.bin"
#define TAS2555_FW_FORMAT   0x01
const char *tas2555_fw_header = "TAS2555-uCDSP";
static struct tas2555_register register_addr = { 0 };

#define MAX_TUNINGS 16

#define TAS2555_UDELAY 0xFFFFFFFE

#define FW_ERR_HEADER -1
#define FW_ERR_SIZE -2

#define TAS2555_BLOCK_BASE_MAIN		0x01
#define TAS2555_BLOCK_CONF_COEFF	0x03
#define TAS2555_BLOCK_CONF_PRE		0x04
#define TAS2555_BLOCK_CONF_POST		0x05
#define TAS2555_BLOCK_CONF_POST_POWER	0x06
#define CRC8_TABLE_SIZE 256 
#define PPC_DRIVER_VERSION			0x00000200 

#define TAS2555_REG_IS_VALID(book, page, reg) \
        ((book >= 0) && (book <= 255) &&\
        (page >= 0) && (page <= 255) &&\
        (reg >= 0) && (reg <= 127))

static struct i2c_client *g_client = NULL;
static int g_logEnable = 0;

static int fw_parse(struct tas2555_priv *pTAS2555,TFirmware * pFirmware, unsigned char *pData,
	unsigned int nSize);
static int tas2555_load_block(struct tas2555_priv *pTAS2555, TBlock * pBlock);
static int tas2555_load_data(struct tas2555_priv *pTAS2555, TData * pData,
	unsigned int nType);
static int tas2555_load_configuration(struct tas2555_priv *pTAS2555,
	unsigned int nConfiguration, bool bLoadSame);
static int tas2555_enable(struct tas2555_priv *pTAS2555, bool bEnable);

static DEFINE_MUTEX(smartpa_lock); 

static u8 *gpDMABuf_va = NULL;
static dma_addr_t gpDMABuf_pa;
#define GTP_DMA_MAX_TRANSACTION_LENGTH  128   // for DMA mode
#define I2C_MASTER_CLOCK                400
#define SND_SOC_ADV_MT_FMTS (\
			       SNDRV_PCM_FMTBIT_S16_LE |\
			       SNDRV_PCM_FMTBIT_S16_BE |\
			       SNDRV_PCM_FMTBIT_U16_LE |\
			       SNDRV_PCM_FMTBIT_U16_BE |\
			       SNDRV_PCM_FMTBIT_S24_LE |\
			       SNDRV_PCM_FMTBIT_S24_BE |\
			       SNDRV_PCM_FMTBIT_U24_LE |\
			       SNDRV_PCM_FMTBIT_U24_BE |\
			       SNDRV_PCM_FMTBIT_S32_LE |\
			       SNDRV_PCM_FMTBIT_S32_BE |\
				  SNDRV_PCM_FMTBIT_U32_LE |\
				  SNDRV_PCM_FMTBIT_U32_BE)

static int tas2555_i2c_read_byte(struct i2c_client *client, unsigned char addr, unsigned int *returnData)
{
    char     cmd_buf[1] = {0x00};
    char     readData = 0;
    int     ret = 0;
    cmd_buf[0] = addr;

    mutex_lock(&smartpa_lock);

    ret = i2c_master_send(client, &cmd_buf[0], 1);
    if (ret < 0)
    {
        mutex_unlock(&smartpa_lock);
		printk("%s, i2c send fail %d \n", __FUNCTION__, ret);
        return -1;
    }
    ret = i2c_master_recv(client, &readData, 1);
    mutex_unlock(&smartpa_lock);
    if (ret < 0)
    {
		printk("%s, i2c recv fail %d \n", __FUNCTION__, ret);
        return -1;
    }
	if(g_logEnable) printk("%s, R[0x%x]=0x%x\n", __FUNCTION__, addr, readData);
    *returnData = readData;
    //printk("addr 0x%x data 0x%x \n", addr, readData);
    return 1;
}

//write register
static int tas2555_i2c_write_byte(struct i2c_client *client, unsigned char addr, unsigned char writeData)
{
    char    write_data[2] = {0};
    int    ret = 0;
    write_data[0] = addr;         // ex. 0x01
    write_data[1] = writeData;

    mutex_lock(&smartpa_lock);
    ret = i2c_master_send(client, write_data, 2);
	if(g_logEnable) printk("%s, R[0x%x]=0x%x\n", __FUNCTION__, addr, writeData);
    mutex_unlock(&smartpa_lock);
    if (ret < 0)
    {
		printk("%s, send fail %d \n", __FUNCTION__, ret);
        return -1;
    }
    //printk("addr 0x%x data 0x%x \n", addr, writeData);
    return 1;
}

static int tas2555_i2c_dma_read(struct i2c_client *client, unsigned char addr, unsigned char *rxbuf, int len)
{
    int ret;
    int retry = 0;
    unsigned char buffer[2];

    struct i2c_msg msg[2] =
    {
        {
            .addr = (client->addr & I2C_MASK_FLAG),
            .flags = 0,
            .buf = buffer,
            .len = 1,
            .timing = I2C_MASTER_CLOCK
        },
        {
            .addr = (client->addr & I2C_MASK_FLAG),
            .ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
            .flags = I2C_M_RD,
            .buf = (unsigned char *)gpDMABuf_pa,     
            .len = len,
            .timing = I2C_MASTER_CLOCK
        },
    };
    
    buffer[0] = addr;

	if(g_logEnable) printk("%s, R[0x%x] len=%d\n", __FUNCTION__, addr, len);
    for (retry = 0; retry < 5; ++retry)
    {
        ret = i2c_transfer(client->adapter, &msg[0], 2);
        if (ret < 0)
        {
            continue;
        }
        memcpy(rxbuf, gpDMABuf_va, len);
        return 0;
    }

    return ret;
}


static int tas2555_i2c_dma_write(struct i2c_client *client, unsigned char addr, unsigned char *txbuf, unsigned int len)
{
    int ret;
    int retry = 0;
    unsigned char *wr_buf = gpDMABuf_va;
    
    struct i2c_msg msg =
    {
        .addr = (client->addr & I2C_MASK_FLAG),
        .ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
        .flags = 0,
        .buf = (unsigned char *)gpDMABuf_pa,
        .len = 1 + len,
        .timing = I2C_MASTER_CLOCK
    };
    
    wr_buf[0] = addr;


    memcpy(&wr_buf[1], txbuf, len);

	if(g_logEnable) printk("%s, R[0x%x] len=%d\n", __FUNCTION__, addr, len);
	
    for (retry = 0; retry < 5; ++retry)
    {
        ret = i2c_transfer(client->adapter, &msg, 1);
        if (ret < 0)
        {
            continue;
        }
        return 0;
    }

    return ret;
}

static int tas2555_change_book_and_page(struct tas2555_priv *pTAS2555, int book, int page)
{
	int ret = 0;
	
    if ((pTAS2555->mnCurrentBook == book) &&(pTAS2555->mnCurrentPage == page))
        return 1;

//  printk("change book: from %d to %d\n\r", tas2555->mnCurrentBook, book);
	
	if(pTAS2555->mnCurrentBook == book){	
		ret = tas2555_i2c_write_byte(pTAS2555->client, TAS2555_BOOKCTL_PAGE, page);
		if (ret < 0) {			
			dev_err(pTAS2555->dev, "%s, I2C error\n", __func__);			
			pTAS2555->mnErrorCode |= TAS2555_ERROR_I2CIO;	
			goto end;
		} else {
			pTAS2555->mnCurrentPage = page;
		}
	}else{
		ret = tas2555_i2c_write_byte(pTAS2555->client, TAS2555_BOOKCTL_PAGE, 0);
		if (ret < 0) {			
			dev_err(pTAS2555->dev, "%s, I2C error\n", __func__);			
			pTAS2555->mnErrorCode |= TAS2555_ERROR_I2CIO;	
			goto end;
		} else {
			pTAS2555->mnCurrentPage = 0;
			ret = tas2555_i2c_write_byte(pTAS2555->client, TAS2555_BOOKCTL_REG, book);
			if (ret < 0) {			
				dev_err(pTAS2555->dev, "%s, I2C error\n", __func__);			
				pTAS2555->mnErrorCode |= TAS2555_ERROR_I2CIO;	
				goto end;
			} else {
				pTAS2555->mnCurrentBook = book;			
				if(pTAS2555->mnCurrentPage != page){
					ret = tas2555_i2c_write_byte(pTAS2555->client, TAS2555_BOOKCTL_PAGE, page);
					if (ret < 0) {			
						dev_err(pTAS2555->dev, "%s, I2C error\n", __func__);			
						pTAS2555->mnErrorCode |= TAS2555_ERROR_I2CIO;	
						goto end;
					} else 	{
						pTAS2555->mnCurrentPage = page;
					}
				}
			}
		}
	}	

end:
	
	return ret;
}

static int tas2555_dev_read(struct tas2555_priv *pTAS2555, unsigned int reg, unsigned int *value)
{
    int ret = 0;

	mutex_lock(&pTAS2555->dev_lock);

    if (pTAS2555->mbTILoadActive)
    {
        if (!(reg & 0x80000000)) 
			goto end; // let only reads from TILoad pass.
        reg &= ~0x80000000;
    }

#if 0	
    dev_err(tas2555->dev, "%s: BOOK:PAGE:REG %u:%u:%u\n", __func__,
        TAS2555_BOOK_ID(reg), TAS2555_PAGE_ID(reg),
        TAS2555_PAGE_REG(reg));
#endif
		
    ret = tas2555_change_book_and_page(pTAS2555, TAS2555_BOOK_ID(reg), TAS2555_PAGE_ID(reg));
	if(ret < 0) 
		goto end;

#if 1
	ret = tas2555_i2c_read_byte(pTAS2555->client, TAS2555_PAGE_REG(reg), value);
	if (ret < 0) {			
		dev_err(pTAS2555->dev, "%s, I2C error\n", __func__);			
		pTAS2555->mnErrorCode |= TAS2555_ERROR_I2CIO;					
	}

#else
    ret = regmap_read(tas2555->mpRegmap, TAS2555_PAGE_REG(reg), value);
#endif

	if(g_logEnable != 0)
		printk("%s, rd B[%d]P[%d]R[%d]=0x%x\n", __FUNCTION__, TAS2555_BOOK_ID(reg), TAS2555_PAGE_ID(reg), TAS2555_PAGE_REG(reg), *value);

end:
	mutex_unlock(&pTAS2555->dev_lock);
    return ret;
}

static int tas2555_dev_bulk_read(struct tas2555_priv *pTAS2555, unsigned int reg, unsigned char *data, unsigned int len)
{
    int ret = 0, i;
	unsigned int temp;

	mutex_lock(&pTAS2555->dev_lock);

    if (pTAS2555->mbTILoadActive)
    {
        if (!(reg & 0x80000000)) 
			goto end; // let only reads from TILoad pass.
        reg &= ~0x80000000;
    }

#if 0	
    dev_err(tas2555->dev, "%s: BOOK:PAGE:REG %u:%u:%u\n", __func__,
        TAS2555_BOOK_ID(reg), TAS2555_PAGE_ID(reg),
        TAS2555_PAGE_REG(reg));
#endif
		
    ret = tas2555_change_book_and_page(pTAS2555, TAS2555_BOOK_ID(reg), TAS2555_PAGE_ID(reg));
	if(ret < 0) 
		goto end;

#if 1
	ret = tas2555_i2c_dma_read(pTAS2555->client, TAS2555_PAGE_REG(reg), data, len);
	if (ret < 0) {			
		dev_err(pTAS2555->dev, "%s, I2C error\n", __func__);			
		pTAS2555->mnErrorCode |= TAS2555_ERROR_I2CIO;					
	}

#else
    ret = regmap_bulk_read(tas2555->mpRegmap, TAS2555_PAGE_REG(reg), data, len);
#endif

	if(g_logEnable != 0){
		for(i=0; i < len; i++){
			temp = reg+i;
			printk("%s, bulk rd B[%d]P[%d]R[%d]=0x%x\n", __FUNCTION__, TAS2555_BOOK_ID(temp ), TAS2555_PAGE_ID(temp), TAS2555_PAGE_REG(temp), data[i]);
		}
	}
end:
	mutex_unlock(&pTAS2555->dev_lock);
    return ret;
}

static unsigned int tas2555_read(struct snd_soc_codec *codec, unsigned int reg)
{
    struct tas2555_priv *tas2555 = snd_soc_codec_get_drvdata(codec);
    unsigned int value = 0;
    int ret = 0;

	ret = tas2555_dev_read(tas2555, reg, &value);
	
	if(ret > 0) return value;
	else return ret;
}

static int tas2555_dev_write(struct tas2555_priv *pTAS2555, unsigned int reg,  unsigned int value)
{
    int ret = 0;

	mutex_lock(&pTAS2555->dev_lock);

    if ((reg == 0xAFFEAFFE) && (value == 0xBABEBABE))
    {
        pTAS2555->mbTILoadActive = true;
        goto end;
    }

    if ((reg == 0xBABEBABE) && (value == 0xAFFEAFFE))
    {
        pTAS2555->mbTILoadActive = false;
         goto end;
    }
    
    if (pTAS2555->mbTILoadActive)
    {
        if (!(reg & 0x80000000))  
			goto end;; // let only writes from TILoad pass.
        reg &= ~0x80000000;
    }
 
    ret = tas2555_change_book_and_page(pTAS2555, TAS2555_BOOK_ID(reg), TAS2555_PAGE_ID(reg));
	if(ret < 0) 
		goto end;
	
	if(g_logEnable != 0)
		printk("%s, wr B[%d]P[%d]R[%d]=0x%x\n", __FUNCTION__, TAS2555_BOOK_ID(reg), TAS2555_PAGE_ID(reg), TAS2555_PAGE_REG(reg), value);
#if 0	
	dev_err(tas2555->dev, "%s: BOOK:PAGE:REG %u:%u:%u, VAL: 0x%02x\n",
      __func__, TAS2555_BOOK_ID(reg), TAS2555_PAGE_ID(reg),
      TAS2555_PAGE_REG(reg), value);
#endif
#if 1
	
	ret = tas2555_i2c_write_byte(pTAS2555->client, TAS2555_PAGE_REG(reg), value);
	if (ret < 0) {			
		dev_err(pTAS2555->dev, "%s, I2C error\n", __func__);			
		pTAS2555->mnErrorCode |= TAS2555_ERROR_I2CIO;					
	}

#else
    ret = regmap_write(tas2555->mpRegmap, TAS2555_PAGE_REG(reg), value);
#endif
end:
	mutex_unlock(&pTAS2555->dev_lock);

    return ret;
}

static int tas2555_dev_update_bits(struct tas2555_priv *pTAS2555, unsigned int reg, unsigned int mask ,unsigned int value)
{
    int ret = 0;
	unsigned int nValue;
	unsigned int nTemp;

	mutex_lock(&pTAS2555->dev_lock);

    if ((reg == 0xAFFEAFFE) && (value == 0xBABEBABE))
    {
        pTAS2555->mbTILoadActive = true;
        goto end;
    }

    if ((reg == 0xBABEBABE) && (value == 0xAFFEAFFE))
    {
        pTAS2555->mbTILoadActive = false;
        goto end;;
    }
    
    if (pTAS2555->mbTILoadActive)
    {
        if (!(reg & 0x80000000)) 
			goto end;; // let only writes from TILoad pass.
        reg &= ~0x80000000;
    }
 
    ret = tas2555_change_book_and_page(pTAS2555, TAS2555_BOOK_ID(reg), TAS2555_PAGE_ID(reg));
	if(ret < 0) 
		goto end;
	
	if(g_logEnable != 0)
		printk("%s, wr B[%d]P[%d]R[%d]=0x%x\n", __FUNCTION__, TAS2555_BOOK_ID(reg), TAS2555_PAGE_ID(reg), TAS2555_PAGE_REG(reg), value);
#if 0	
	dev_err(tas2555->dev, "%s: BOOK:PAGE:REG %u:%u:%u, VAL: 0x%02x\n",
      __func__, TAS2555_BOOK_ID(reg), TAS2555_PAGE_ID(reg),
      TAS2555_PAGE_REG(reg), value);
#endif
#if 1
	ret = tas2555_i2c_read_byte(pTAS2555->client, TAS2555_PAGE_REG(reg), &nValue);	
	if(ret >= 0) {
		value &= 0xff;
		mask &= 0xff;
		if ((nValue & mask) != (value & mask)){
			nTemp = value&mask;
			nValue &= ~mask;
			nValue |= nTemp;
			ret = tas2555_i2c_write_byte(pTAS2555->client, TAS2555_PAGE_REG(reg), nValue);			
			if (ret < 0) {			
				dev_err(pTAS2555->dev, "%s, I2C error\n", __func__);			
				pTAS2555->mnErrorCode |= TAS2555_ERROR_I2CIO;
			}			
		}
	} else {			
		dev_err(pTAS2555->dev, "%s, I2C error\n", __func__);			
		pTAS2555->mnErrorCode |= TAS2555_ERROR_I2CIO;					
	}

#else
    ret = regmap_write(tas2555->mpRegmap, TAS2555_PAGE_REG(reg), value);
#endif
end:
	mutex_unlock(&pTAS2555->dev_lock);

    return ret;
}

static int tas2555_write(struct snd_soc_codec *codec, unsigned int reg, unsigned int value)
{
    struct tas2555_priv *tas2555 = snd_soc_codec_get_drvdata(codec);
    int ret = 0;
	
	ret = tas2555_dev_write(tas2555, reg, value);
  
    return ret;
}

static int tas2555_dev_bulk_write(struct tas2555_priv *pTAS2555, unsigned int reg, unsigned char *data, unsigned int len)
{
    int ret = 0, i;
	
	mutex_lock(&pTAS2555->dev_lock);

    if (pTAS2555->mbTILoadActive)
    {
        if (!(reg & 0x80000000))  
			goto end;; // let only writes from TILoad pass.
        reg &= ~0x80000000;
    }
 
#if 0
    dev_err(tas2555->dev, "%s: BOOK:PAGE:REG %u:%u:%u, len: %d\n",
        __func__, TAS2555_BOOK_ID(reg), TAS2555_PAGE_ID(reg),
        TAS2555_PAGE_REG(reg), (int)len);
#endif		


    ret = tas2555_change_book_and_page(pTAS2555, TAS2555_BOOK_ID(reg), TAS2555_PAGE_ID(reg));
	if(ret < 0) 
		goto end;

#if 1
	ret = tas2555_i2c_dma_write(pTAS2555->client, TAS2555_PAGE_REG(reg),data, len);
	if (ret < 0) {			
		dev_err(pTAS2555->dev, "%s, I2C error\n", __func__);			
		pTAS2555->mnErrorCode |= TAS2555_ERROR_I2CIO;					
	}

		
#else
    ret = regmap_bulk_write(tas2555->mpRegmap, TAS2555_PAGE_REG(reg), data, len);
#endif	
	if(g_logEnable != 0){
		for(i=0; i < len; i++){
			printk("%s, bulk wd B[%d]P[%d]R[%d]=0x%x\n", __FUNCTION__, TAS2555_BOOK_ID(reg+i), TAS2555_PAGE_ID(reg+i), TAS2555_PAGE_REG(reg+i), data[i]);
		}
	}
end:
	mutex_unlock(&pTAS2555->dev_lock);

    return ret;
}

static bool tas2555_volatile(struct device *dev, unsigned int reg)
{
    return false;
}

static bool tas2555_writeable(struct device *dev, unsigned int reg)
{
    return true;
}

#if 0
static const struct regmap_range_cfg tas2555_ranges[] = {
    {
        .range_min = 0,
        .range_max = 255 * 128,
        .selector_reg = TAS2555_PAGECTL_REG,
        .selector_mask = 0xff,
        .selector_shift = 0,
        .window_start = 0,
        .window_len = 128,
    },
};
#endif

static const struct regmap_config tas2555_i2c_regmap = {
    .reg_bits = 8,
    .val_bits = 8,
    .writeable_reg = tas2555_writeable,
    .volatile_reg = tas2555_volatile,
    .cache_type = REGCACHE_NONE,
#if 0	
    .ranges = tas2555_ranges,
    .num_ranges = ARRAY_SIZE(tas2555_ranges),
    .max_register = 255 * 128,
#else
    .max_register = 128,	
#endif	
};

#define TAS2555_PCTRL1_MASK (TAS2555_MADC_POWER_UP | \
                 TAS2555_MDAC_POWER_UP | \
                 TAS2555_DSP_POWER_UP)
#define TAS2555_PCTRL2_MASK (TAS2555_VSENSE_ENABLE | \
                 TAS2555_ISENSE_ENABLE | \
                 TAS2555_BOOST_ENABLE)
#define TAS2555_MUTE_MASK   (TAS2555_ISENSE_MUTE | TAS2555_CLASSD_MUTE)
#define TAS2555_SOFT_MUTE_MASK  (TAS2555_PDM_SOFT_MUTE | \
                 TAS2555_VSENSE_SOFT_MUTE | \
                 TAS2555_ISENSE_SOFT_MUTE | \
                 TAS2555_CLASSD_SOFT_MUTE)

static unsigned int p_tas2555_default_data[] = {
	TAS2555_SAR_ADC2_REG, 0x05,	/* enable SAR ADC */
	TAS2555_CLK_ERR_CTRL2, 0x39,	//enable clock error detection on PLL
	TAS2555_CLK_ERR_CTRL3, 0x11,	//enable clock error detection on PLL
	TAS2555_SAFE_GUARD_REG, TAS2555_SAFE_GUARD_PATTERN,	//safe guard
	0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2555_irq_config[] = {
	TAS2555_CLK_HALT_REG, 0x71,
	TAS2555_INT_GEN1_REG, 0x11,	/* enable spk OC and OV */
	TAS2555_INT_GEN2_REG, 0x11,	/* enable clk err1 and die OT */
	TAS2555_INT_GEN3_REG, 0x11,	/* enable clk err2 and brownout */
	TAS2555_INT_GEN4_REG, 0x01,	/* disable SAR, enable clk halt */
	TAS2555_GPIO4_PIN_REG, 0x07,	/*set GPIO4 as int1, default */
	TAS2555_INT_MODE_REG, 0x80,	/* active high until INT_STICKY_1 and INT_STICKY_2 are read to be cleared. */
	0xFFFFFFFF, 0xFFFFFFFF
};

#define TAS2555_STARTUP_DATA_PLL_CLKIN_INDEX 3
//#define TAS2555_STARTUP_DATA_PLL_CLKIN_INDEX 1

static unsigned int p_tas2555_startup_data[] = {
	TAS2555_CLK_ERR_CTRL1, 0x00,	//disable clock error detection on PLL
	TAS2555_PLL_CLKIN_REG, TAS2555_DEFAULT_PLL_CLKIN,
	TAS2555_POWER_CTRL2_REG, 0xA0,	//Class-D, Boost power up
	TAS2555_POWER_CTRL2_REG, 0xA3,	//Class-D, Boost, IV sense power up
	TAS2555_POWER_CTRL1_REG, 0xF8,	//PLL, DSP, clock dividers power up
	TAS2555_UDELAY, 2000,		//delay 2ms
	TAS2555_POWER_CTRL1_REG, 0x78,	//toggle DSP 
	TAS2555_UDELAY, 2000,		//delay 2ms
	TAS2555_POWER_CTRL1_REG, 0xf8,
	TAS2555_UDELAY, 2000,		//delay 2ms
	//TAS2555_CLK_ERR_CTRL1, 0x0B,	//disable clock error detection on PLL peter
	0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2555_unmute_data[] = {
	TAS2555_MUTE_REG, 0x00,		//unmute
	TAS2555_SOFT_MUTE_REG, 0x00,	//soft unmute
	0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2555_shutdown_data[] = {
//	TAS2555_CLK_ERR_CTRL1, 0x00,	//disable clock error detection on PLL peter
	TAS2555_SOFT_MUTE_REG, 0x01,	//soft mute
	TAS2555_UDELAY, 10000,		//delay 10ms
	TAS2555_MUTE_REG, 0x03,		//mute
	TAS2555_PLL_CLKIN_REG, 0x0F,	//PLL clock input = osc
	TAS2555_POWER_CTRL1_REG, 0x60,	//DSP power down
	TAS2555_UDELAY, 2000,		//delay 2ms
	TAS2555_POWER_CTRL2_REG, 0x00,	//Class-D, Boost power down
	TAS2555_POWER_CTRL1_REG, 0x00,	//all power down
	0xFFFFFFFF, 0xFFFFFFFF
};

#if 0
static unsigned int p_tas2555_shutdown_clk_err[] = {
	TAS2555_CLK_ERR_CTRL, 0x09,	//enable clock error detection on PLL
	0xFFFFFFFF, 0xFFFFFFFF
};
#endif

static unsigned int p_tas2555_mute_DSP_down_data[] = {
	TAS2555_MUTE_REG, 0x03,		//mute
	TAS2555_PLL_CLKIN_REG, 0x0F,	//PLL clock input = osc
	TAS2555_POWER_CTRL1_REG, 0x60,	//DSP power down
	TAS2555_UDELAY, 0xFF,		//delay
	0xFFFFFFFF, 0xFFFFFFFF
};

static int tas2555_dev_load_data(struct tas2555_priv *pTAS2555,
	unsigned int *pData)
{
	int ret = 0;
	unsigned int n = 0;
	unsigned int nRegister;
	unsigned int nData;

	do {
		nRegister = pData[n * 2];
		nData = pData[n * 2 + 1];
		if (nRegister == TAS2555_UDELAY)
			udelay(nData);
		else if (nRegister != 0xFFFFFFFF) {
			ret = pTAS2555->write(pTAS2555, nRegister, nData);
			if(ret < 0) {
				dev_err(pTAS2555->dev, "Reg Write err %d\n", ret);
				break;
			}
		}
		n++;
	} while (nRegister != 0xFFFFFFFF);

	return ret;
}

void tas2555_clear_firmware(TFirmware *pFirmware)	
{
	unsigned int n, nn;

	if (!pFirmware) 
		return;
	if (pFirmware->mpDescription)
		kfree(pFirmware->mpDescription);

	if (pFirmware->mpPLLs) {
		for (n = 0; n < pFirmware->mnPLLs; n++) {
			kfree(pFirmware->mpPLLs[n].mpDescription);
			kfree(pFirmware->mpPLLs[n].mBlock.mpData);
		}
		kfree(pFirmware->mpPLLs);
	}

	if (pFirmware->mpPrograms) {
		for (n = 0; n < pFirmware->mnPrograms; n++) {
			kfree(pFirmware->mpPrograms[n].mpDescription);
			kfree(pFirmware->mpPrograms[n].mData.mpDescription);
			for (nn = 0; nn < pFirmware->mpPrograms[n].mData.mnBlocks; nn++)
				kfree(pFirmware->mpPrograms[n].mData.mpBlocks[nn].mpData);
			kfree(pFirmware->mpPrograms[n].mData.mpBlocks);
		}
		kfree(pFirmware->mpPrograms);
	}

	if (pFirmware->mpConfigurations) {
		for (n = 0; n < pFirmware->mnConfigurations; n++) {
			kfree(pFirmware->mpConfigurations[n].mpDescription);	
			kfree(pFirmware->mpConfigurations[n].mData.mpDescription);
			for (nn = 0; nn < pFirmware->mpConfigurations[n].mData.mnBlocks; nn++)
				kfree(pFirmware->mpConfigurations[n].mData.mpBlocks[nn].mpData);
			kfree(pFirmware->mpConfigurations[n].mData.mpBlocks);
		}
		kfree(pFirmware->mpConfigurations);
	}

	if (pFirmware->mpCalibrations) {
		for (n = 0; n < pFirmware->mnCalibrations; n++) {
			kfree(pFirmware->mpCalibrations[n].mpDescription);
			kfree(pFirmware->mpCalibrations[n].mBlock.mpData);
		}
		kfree(pFirmware->mpCalibrations);
	}

	memset(pFirmware, 0x00, sizeof(TFirmware));	
}

static void failsafe(struct tas2555_priv *pTAS2555)
{
	dev_err(pTAS2555->dev, "%s\n", __func__);
	pTAS2555->mnErrorCode |= TAS2555_ERROR_FAILSAFE;
	dev_dbg(pTAS2555->dev, "Enable: load shutdown sequence\n");
	tas2555_dev_load_data(pTAS2555, p_tas2555_shutdown_data);
	pTAS2555->mbPowerUp = false;
	pTAS2555->hw_reset(pTAS2555);
	pTAS2555->write(pTAS2555, TAS2555_SW_RESET_REG, 0x01);
	udelay(1000);
	pTAS2555->write(pTAS2555, TAS2555_SPK_CTRL_REG, 0x04);
	if (pTAS2555->mpFirmware != NULL)
		tas2555_clear_firmware(pTAS2555->mpFirmware);
}

static int tas2555_load_calibration(struct tas2555_priv *pTAS2555,
	char *pFileName)
{
	int nResult;

	int nFile;

	mm_segment_t fs;

	unsigned char pBuffer[512];
	
	int nSize = 0;
	
	
	dev_dbg(pTAS2555->dev, "%s:\n", __func__);


	fs = get_fs();

	set_fs(KERNEL_DS);
	
	nFile = sys_open(pFileName, O_RDONLY,0755);


	dev_info(pTAS2555->dev, "TAS2555 calibration file = %s, handle = %d\n",
	
		pFileName, nFile);
	

	if (nFile >= 0) {

		nSize = sys_read(nFile, pBuffer, 512);

		sys_close(nFile);

	} else {
	
		dev_err(pTAS2555->dev, "TAS2555 cannot open calibration file: %s\n",
	
			pFileName);	
	}
	
	set_fs(fs);

	if (!nSize)	
		return -1;
	
	tas2555_clear_firmware(pTAS2555->mpCalFirmware);
			
	dev_info(pTAS2555->dev, "TAS2555 calibration file size = %d\n", nSize);
	
	nResult = fw_parse(pTAS2555,pTAS2555->mpCalFirmware, pBuffer, nSize);
	
	if (nResult) {
	
		dev_err(pTAS2555->dev, "TAS2555 calibration file is corrupt\n");
	
		return -1;

	}
	
	dev_info(pTAS2555->dev, "TAS2555 calibration: %d calibrations\n",
	
		pTAS2555->mpCalFirmware->mnCalibrations);

	return 0;
	
}

/*
static int tas2555_dapm_pre_post_event(struct snd_soc_dapm_widget *pWidget,
	struct snd_kcontrol *pKcontrol, int nEvent)
{
	struct snd_soc_codec *pCodec = pWidget->codec;

	printk(KERN_ERR "TAS2555: tas2555_dapm_pre_post_event: %d\n\r", nEvent);

	switch (nEvent) {
	case SND_SOC_DAPM_POST_PMU:
		tas2555_enable(pCodec, true);
		break;

	case SND_SOC_DAPM_PRE_PMD:
		tas2555_enable(pCodec, false);
		break;
	}

	return 0;
}
*/

static const struct snd_soc_dapm_widget tas2555_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("ASI1", "ASI1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("ASI2", "ASI2 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("ASIM", "ASIM Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC", NULL, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_OUT_DRV("ClassD", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_SUPPLY("PLL", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("NDivider", SND_SOC_NOPM, 0, 0, NULL, 0),

//	SND_SOC_DAPM_POST("Post Event", tas2555_dapm_pre_post_event),
//	SND_SOC_DAPM_PRE("Pre Event", tas2555_dapm_pre_post_event),

	SND_SOC_DAPM_OUTPUT("OUT")
};

static const struct snd_soc_dapm_route tas2555_audio_map[] = {
	{"DAC", NULL, "ASI1"},
	{"DAC", NULL, "ASI2"},
	{"DAC", NULL, "ASIM"},
	{"ClassD", NULL, "DAC"},
	{"OUT", NULL, "ClassD"},
	{"DAC", NULL, "PLL"},
	{"DAC", NULL, "NDivider"},
};

static int tas2555_setup_clocks(struct snd_soc_codec *pCodec,
	struct snd_pcm_hw_params *pParams)
{
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(pCodec);
	int nSamplingRate = params_rate(pParams);
	TConfiguration *pConfiguration;
	unsigned int nConfiguration;
	int nResult = 0;

	printk(KERN_ERR "tas2555_setup_clocks: nSamplingRate = %d [Hz]\n",
		nSamplingRate);

	if ((!pTAS2555->mpFirmware->mpPrograms) || (!pTAS2555->mpFirmware->mpConfigurations)) 
	{
		printk(KERN_ERR "TAS2555: Firmware not loaded\n\r");
		return -EINVAL;
	}

	pConfiguration =
		&(pTAS2555->mpFirmware->mpConfigurations[pTAS2555->mnCurrentConfiguration]);
	if (pConfiguration->mnSamplingRate == nSamplingRate) 
	{
		printk(KERN_ERR
			"TAS2555: Sampling rate for current configuration matches: %d\n\r",
			nSamplingRate);
		return 0;
	}

	for (nConfiguration = 0;
		nConfiguration < pTAS2555->mpFirmware->mnConfigurations;
		nConfiguration++) {
		pConfiguration =
			&(pTAS2555->mpFirmware->mpConfigurations[nConfiguration]);
		if (pConfiguration->mnSamplingRate == nSamplingRate) {
			printk(KERN_ERR
				"TAS2555: Found configuration: %s, with compatible sampling rate %d\n\r",
				pConfiguration->mpName, nSamplingRate);
			nResult = tas2555_load_configuration(pTAS2555, nConfiguration, false);
			//if (nResult < 0) failsafe();
			return 0;
		}
	}

	printk(KERN_ERR
		"TAS2555: Cannot find a configuration that supports sampling rate: %d\n\r",
		nSamplingRate);

	return -EINVAL;
}

int tas2555_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
//  struct snd_soc_codec *codec = dai->codec;
//  struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);

	printk(KERN_ERR "TAS2555: %s\n", __func__);
	return 0;
}

void tas2555_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
//  struct snd_soc_codec *codec = dai->codec;
//  struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);

//	printk(KERN_ERR "TAS2555: %s\n", __func__);
}
#if 1
static int tas2555_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	
	dev_dbg(pTAS2555->dev, "%s\n", __func__);
	//tas2555_enable(pTAS2555, !mute);
	
	return 0;
}
#endif
static int tas2555_set_dai_sysclk(struct snd_soc_dai *pDAI,
	int nClkID, unsigned int nFreqency, int nDir)
{
//    struct snd_soc_codec *pCodec = pDAI->codec;
//    struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(pCodec);

//	printk(KERN_ERR "TAS2555: tas2555_set_dai_sysclk: freq = %u\n", nFreqency);

	return 0;
}

static int tas2555_hw_params(struct snd_pcm_substream *pSubstream,
	struct snd_pcm_hw_params *pParams, struct snd_soc_dai *pDAI)
{
	struct snd_soc_codec *pCodec = pDAI->codec;
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(pCodec);

	dev_err(pTAS2555->dev, "%s\n", __func__);

	pTAS2555->mclk_clkin = 1536000;
	tas2555_setup_clocks(pCodec, pParams);

	return 0;
}

static int tas2555_set_dai_fmt(struct snd_soc_dai *pDAI, unsigned int nFormat)
{
	return 0;
}

static int tas2555_prepare(struct snd_pcm_substream *pSubstream,
	struct snd_soc_dai *pDAI)
{
#ifdef MTK_PLATFORM_DRIVER
	//---------------------------------------//add start MTK
	if (pSubstream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		pr_warn
			("mt63xx_codec_prepare set up SNDRV_PCM_STREAM_CAPTURE rate = %d\n",
			pSubstream->runtime->rate);
		mBlockSampleRate[AUDIO_ANALOG_DEVICE_IN_ADC] =
			pSubstream->runtime->rate;

	} else if (pSubstream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		pr_warn
			("mt63xx_codec_prepare set up SNDRV_PCM_STREAM_PLAYBACK rate = %d\n",
			pSubstream->runtime->rate);
		mBlockSampleRate[AUDIO_ANALOG_DEVICE_OUT_DAC] =
			pSubstream->runtime->rate;
	}
	//----------------------------------//add end MTK
#endif
	return 0;
}

static int tas2555_set_bias_level(struct snd_soc_codec *pCodec,
	enum snd_soc_bias_level eLevel)
{
	printk(KERN_ERR "TAS2555: tas2555_set_bias_level: %d, do nothing\n\r", eLevel);

	switch (eLevel) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
//		if (pCodec->dapm.bias_level == SND_SOC_BIAS_OFF)
//			tas2555_enable(pCodec, true);
		break;
	case SND_SOC_BIAS_OFF:
//		tas2555_enable(pCodec, false);
		break;
	}
	pCodec->dapm.bias_level = eLevel;

	return 0;
}

static void fw_print_header(struct tas2555_priv *pTAS2555, TFirmware * pFirmware)
{
	dev_info(pTAS2555->dev, "FW Size        = %d", pFirmware->mnFWSize);
	dev_info(pTAS2555->dev, "Checksum       = 0x%04X", pFirmware->mnChecksum);
	dev_info(pTAS2555->dev, "PPC Version    = 0x%04X", pFirmware->mnPPCVersion);
	dev_info(pTAS2555->dev, "FW  Version    = 0x%04X", pFirmware->mnFWVersion);
	if(pFirmware->mnPPCVersion >= PPC_WITH_DRIVER_VERSION)
	dev_info(pTAS2555->dev, "Driver Version = 0x%04X", pFirmware->mnDriverVersion);
	dev_info(pTAS2555->dev, "Timestamp      = %d", pFirmware->mnTimeStamp);
	dev_info(pTAS2555->dev, "DDC Name       = %s", pFirmware->mpDDCName);
	dev_info(pTAS2555->dev, "Description    = %s", pFirmware->mpDescription);
}

inline unsigned int fw_convert_number(unsigned char *pData)
{
	return pData[3] + (pData[2] << 8) + (pData[1] << 16) + (pData[0] << 24);
}

static int fw_parse_header(struct tas2555_priv *pTAS2555, 
	TFirmware * pFirmware, unsigned char *pData,
	unsigned int nSize)
{
	unsigned char *pDataStart = pData;
	unsigned int n;
	unsigned char pMagicNumber[] = { 0x35, 0x35, 0x35, 0x32 };
	if (nSize < 102) {
		dev_err(pTAS2555->dev, "Firmware: Header too short");
		return -1;
	}

	if (memcmp(pData, pMagicNumber, 4)) {
		dev_err(pTAS2555->dev, "Firmware: Magic number doesn't match");
		return -1;
	}

	pData += 4;

	pFirmware->mnFWSize = fw_convert_number(pData);
	pData += 4;

	pFirmware->mnChecksum = fw_convert_number(pData);
	pData += 4;

	pFirmware->mnPPCVersion = fw_convert_number(pData);
	pData += 4;

	pFirmware->mnFWVersion = fw_convert_number(pData);
	pData += 4;

	if(pFirmware->mnPPCVersion >= PPC_WITH_DRIVER_VERSION){
		pFirmware->mnDriverVersion = fw_convert_number(pData);
		pData += 4;		
	}
			
	pFirmware->mnTimeStamp = fw_convert_number(pData);
	pData += 4;

	memcpy(pFirmware->mpDDCName, pData, 64);
	pData += 64;

	n = strlen(pData);
	pFirmware->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
	pData += n + 1;

	if ((pData - pDataStart) >= nSize) {
		dev_err(pTAS2555->dev, "Firmware: Header too short after DDC description");
		return -1;
	}

	pFirmware->mnDeviceFamily = fw_convert_number(pData);
	pData += 4;

	pFirmware->mnDevice = fw_convert_number(pData);
	pData += 4;

	fw_print_header(pTAS2555, pFirmware);

	return pData - pDataStart;
}

static int fw_parse_block_data(struct tas2555_priv *pTAS2555, TFirmware *pFirmware,
	TBlock * pBlock, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;

	pBlock->mnType = fw_convert_number(pData);
	pData += 4;
	
	if(pFirmware->mnDriverVersion >= PPC_DRIVER_VERSION){
		pBlock->mbPChkSumPresent = pData[0];
		pData++;
		
		pBlock->mnPChkSum = pData[0];
		pData++;
		
		pBlock->mbYChkSumPresent = pData[0];
		pData++;
		
		pBlock->mnYChkSum = pData[0];
		pData++;
	}else{
		pBlock->mbPChkSumPresent = 0;
		pBlock->mbYChkSumPresent = 0;
	}

	pBlock->mnCommands = fw_convert_number(pData);
	pData += 4;

	n = pBlock->mnCommands * 4;
	pBlock->mpData = kmemdup(pData, n, GFP_KERNEL);
	pData += n;

	return pData - pDataStart;
}

static int fw_parse_data(struct tas2555_priv *pTAS2555, TFirmware *pFirmware,
	TData * pImageData, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int nBlock;
	unsigned int n;

	memcpy(pImageData->mpName, pData, 64);
	pData += 64;

	n = strlen(pData);
	pImageData->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
	pData += n + 1;

	pImageData->mnBlocks = (pData[0] << 8) + pData[1];
	pData += 2;

	pImageData->mpBlocks =
		kmalloc(sizeof(TBlock) * pImageData->mnBlocks, GFP_KERNEL);

	for (nBlock = 0; nBlock < pImageData->mnBlocks; nBlock++) {
		n = fw_parse_block_data(pTAS2555, pFirmware, &(pImageData->mpBlocks[nBlock]), pData);
		pData += n;
	}

	return pData - pDataStart;
}

static int fw_parse_pll_data(struct tas2555_priv *pTAS2555,
	TFirmware *pFirmware, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;
	unsigned int nPLL;
	TPLL *pPLL;

	pFirmware->mnPLLs = (pData[0] << 8) + pData[1];
	pData += 2;

	if (pFirmware->mnPLLs == 0)
		goto end;

	pFirmware->mpPLLs = kmalloc(sizeof(TPLL) * pFirmware->mnPLLs, GFP_KERNEL);
	for (nPLL = 0; nPLL < pFirmware->mnPLLs; nPLL++) {
		pPLL = &(pFirmware->mpPLLs[nPLL]);

		memcpy(pPLL->mpName, pData, 64);
		pData += 64;

		n = strlen(pData);
		pPLL->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
		pData += n + 1;

		n = fw_parse_block_data(pTAS2555, pFirmware, &(pPLL->mBlock), pData);
		pData += n;
	}

end:

	return pData - pDataStart;
}

static int fw_parse_program_data(struct tas2555_priv *pTAS2555,
	TFirmware * pFirmware, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;
	unsigned int nProgram;
	TProgram *pProgram;

	pFirmware->mnPrograms = (pData[0] << 8) + pData[1];
	pData += 2;

	if (pFirmware->mnPrograms == 0)
		goto end;

	pFirmware->mpPrograms =
		kmalloc(sizeof(TProgram) * pFirmware->mnPrograms, GFP_KERNEL);
	for (nProgram = 0; nProgram < pFirmware->mnPrograms; nProgram++) {
		pProgram = &(pFirmware->mpPrograms[nProgram]);
		memcpy(pProgram->mpName, pData, 64);
		pData += 64;

		n = strlen(pData);
		pProgram->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
		pData += n + 1;

		n = fw_parse_data(pTAS2555, pFirmware, &(pProgram->mData), pData);
		pData += n;
	}

end:

	return pData - pDataStart;
}

static int fw_parse_configuration_data(struct tas2555_priv *pTAS2555,
	TFirmware * pFirmware,	unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;
	unsigned int nConfiguration;
	TConfiguration *pConfiguration;

	pFirmware->mnConfigurations = (pData[0] << 8) + pData[1];
	pData += 2;

	if (pFirmware->mnConfigurations == 0)
		goto end;

	pFirmware->mpConfigurations =
		kmalloc(sizeof(TConfiguration) * pFirmware->mnConfigurations,
		GFP_KERNEL);
	for (nConfiguration = 0; nConfiguration < pFirmware->mnConfigurations;
		nConfiguration++) {
		pConfiguration = &(pFirmware->mpConfigurations[nConfiguration]);
		memcpy(pConfiguration->mpName, pData, 64);
		pData += 64;

		n = strlen(pData);
		pConfiguration->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
		pData += n + 1;

		pConfiguration->mnProgram = pData[0];
		pData++;

		pConfiguration->mnPLL = pData[0];
		pData++;

		pConfiguration->mnSamplingRate = fw_convert_number(pData);
		pData += 4;

		n = fw_parse_data(pTAS2555, pFirmware, &(pConfiguration->mData), pData);
		pData += n;
	}

end:

	return pData - pDataStart;
}

int fw_parse_calibration_data(struct tas2555_priv *pTAS2555,
	TFirmware * pFirmware, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;
	unsigned int nCalibration;
	TCalibration *pCalibration;

	pFirmware->mnCalibrations = (pData[0] << 8) + pData[1];
	pData += 2;

	if (pFirmware->mnCalibrations == 0)
		goto end;

	pFirmware->mpCalibrations =
		kmalloc(sizeof(TCalibration) * pFirmware->mnCalibrations, GFP_KERNEL);
	for (nCalibration = 0;
		nCalibration < pFirmware->mnCalibrations;
		nCalibration++) {
		pCalibration = &(pFirmware->mpCalibrations[nCalibration]);
		memcpy(pCalibration->mpName, pData, 64);
		pData += 64;

		n = strlen(pData);
		pCalibration->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
		pData += n + 1;

		pCalibration->mnProgram = pData[0];
		pData++;

		pCalibration->mnConfiguration = pData[0];
		pData++;

		n = fw_parse_block_data(pTAS2555, pFirmware, &(pCalibration->mBlock), pData);
		pData += n;
	}

end:

	return pData - pDataStart;
}

static int fw_parse(struct tas2555_priv *pTAS2555,
	TFirmware * pFirmware,
	unsigned char *pData,
	unsigned int nSize)
{
	int nPosition = 0;

	nPosition = fw_parse_header(pTAS2555, pFirmware, pData, nSize);
	if (nPosition < 0) {
		dev_err(pTAS2555->dev, "Firmware: Wrong Header");
		return FW_ERR_HEADER;
	}

	if (nPosition >= nSize) {
		dev_err(pTAS2555->dev, "Firmware: Too short");
		return FW_ERR_SIZE;
	}

	pData += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	nPosition = fw_parse_pll_data(pTAS2555, pFirmware, pData);

	pData += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	nPosition = fw_parse_program_data(pTAS2555, pFirmware, pData);

	pData += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	nPosition = fw_parse_configuration_data(pTAS2555, pFirmware, pData);

	pData += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	if (nSize > 64)
		nPosition = fw_parse_calibration_data(pTAS2555, pFirmware, pData);

	return 0;
}

static const unsigned char crc8_lookup_table[CRC8_TABLE_SIZE] = {
0x00, 0x4D, 0x9A, 0xD7, 0x79, 0x34, 0xE3, 0xAE, 0xF2, 0xBF, 0x68, 0x25, 0x8B, 0xC6, 0x11, 0x5C, 
0xA9, 0xE4, 0x33, 0x7E, 0xD0, 0x9D, 0x4A, 0x07, 0x5B, 0x16, 0xC1, 0x8C, 0x22, 0x6F, 0xB8, 0xF5, 
0x1F, 0x52, 0x85, 0xC8, 0x66, 0x2B, 0xFC, 0xB1, 0xED, 0xA0, 0x77, 0x3A, 0x94, 0xD9, 0x0E, 0x43, 
0xB6, 0xFB, 0x2C, 0x61, 0xCF, 0x82, 0x55, 0x18, 0x44, 0x09, 0xDE, 0x93, 0x3D, 0x70, 0xA7, 0xEA, 
0x3E, 0x73, 0xA4, 0xE9, 0x47, 0x0A, 0xDD, 0x90, 0xCC, 0x81, 0x56, 0x1B, 0xB5, 0xF8, 0x2F, 0x62, 
0x97, 0xDA, 0x0D, 0x40, 0xEE, 0xA3, 0x74, 0x39, 0x65, 0x28, 0xFF, 0xB2, 0x1C, 0x51, 0x86, 0xCB, 
0x21, 0x6C, 0xBB, 0xF6, 0x58, 0x15, 0xC2, 0x8F, 0xD3, 0x9E, 0x49, 0x04, 0xAA, 0xE7, 0x30, 0x7D, 
0x88, 0xC5, 0x12, 0x5F, 0xF1, 0xBC, 0x6B, 0x26, 0x7A, 0x37, 0xE0, 0xAD, 0x03, 0x4E, 0x99, 0xD4, 
0x7C, 0x31, 0xE6, 0xAB, 0x05, 0x48, 0x9F, 0xD2, 0x8E, 0xC3, 0x14, 0x59, 0xF7, 0xBA, 0x6D, 0x20, 
0xD5, 0x98, 0x4F, 0x02, 0xAC, 0xE1, 0x36, 0x7B, 0x27, 0x6A, 0xBD, 0xF0, 0x5E, 0x13, 0xC4, 0x89, 
0x63, 0x2E, 0xF9, 0xB4, 0x1A, 0x57, 0x80, 0xCD, 0x91, 0xDC, 0x0B, 0x46, 0xE8, 0xA5, 0x72, 0x3F, 
0xCA, 0x87, 0x50, 0x1D, 0xB3, 0xFE, 0x29, 0x64, 0x38, 0x75, 0xA2, 0xEF, 0x41, 0x0C, 0xDB, 0x96, 
0x42, 0x0F, 0xD8, 0x95, 0x3B, 0x76, 0xA1, 0xEC, 0xB0, 0xFD, 0x2A, 0x67, 0xC9, 0x84, 0x53, 0x1E, 
0xEB, 0xA6, 0x71, 0x3C, 0x92, 0xDF, 0x08, 0x45, 0x19, 0x54, 0x83, 0xCE, 0x60, 0x2D, 0xFA, 0xB7, 
0x5D, 0x10, 0xC7, 0x8A, 0x24, 0x69, 0xBE, 0xF3, 0xAF, 0xE2, 0x35, 0x78, 0xD6, 0x9B, 0x4C, 0x01, 
0xF4, 0xB9, 0x6E, 0x23, 0x8D, 0xC0, 0x17, 0x5A, 0x06, 0x4B, 0x9C, 0xD1, 0x7F, 0x32, 0xE5, 0xA8 
};

static int isYRAM(struct tas2555_priv *pTAS2555, TYCRC *pCRCData, 
	unsigned char nBook, unsigned char nPage, unsigned char nReg, unsigned char len)
{
	int result = -1;

	if (nBook == TAS2555_YRAM_BOOK) {
		if (nPage == TAS2555_YRAM1_PAGE) {
			if ((nReg >= TAS2555_YRAM1_START_REG)
				&&(nReg <= TAS2555_YRAM1_END_REG)) {
				if ((nReg + len -1) <= TAS2555_YRAM1_END_REG) {
					result = 1;
					if (pCRCData != NULL) {
						pCRCData->mnOffset = nReg;
						pCRCData->mnLen = len;
					}
				} else {
					pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;
					dev_err(pTAS2555->dev, "nReg 0x%x error, len %d\n", nReg, len);
				}
			} else if (nReg > TAS2555_YRAM1_END_REG) {
				pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;
				dev_err(pTAS2555->dev, "nReg 0x%x error\n", nReg);
			} else if (len > 1) {
				if ((nReg + (len-1))> TAS2555_YRAM1_END_REG) {
					dev_err(pTAS2555->dev, "nReg 0x%x, len %d error\n", nReg, len);
					pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;
				} else if ((nReg + (len-1)) >= TAS2555_YRAM1_START_REG) {
					result = 1;
					if (pCRCData != NULL) {
						pCRCData->mnOffset = TAS2555_YRAM1_START_REG;
						pCRCData->mnLen = nReg + len - TAS2555_YRAM1_START_REG;
					}
				} else 
					result = 0;
			} else 
				result = 0;
		} else if ((nPage >= TAS2555_YRAM2_START_PAGE)
			&& (nPage <= TAS2555_YRAM2_END_PAGE)) {
			if (nReg > TAS2555_YRAM2_END_REG) {
				dev_err(pTAS2555->dev, "nReg 0x%x error\n", nReg);
				pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;
			} else if ((nReg >= TAS2555_YRAM2_START_REG)
				&& (nReg <= TAS2555_YRAM2_END_REG)) {
				if ((nReg + len -1) <= TAS2555_YRAM2_END_REG) {
					result = 1;
					if (pCRCData != NULL) {
						pCRCData->mnOffset = nReg;
						pCRCData->mnLen = len;
					}
				} else {
					dev_err(pTAS2555->dev, "nReg 0x%x error, len %d\n", nReg, len);
					pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;
				}
			} else if (len > 1) {
				if ((nReg + (len-1)) > TAS2555_YRAM2_END_REG) {
					dev_err(pTAS2555->dev, "nReg 0x%x, len %d error\n", nReg, len);
					pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;
				} else if ((nReg + (len-1)) >= TAS2555_YRAM2_START_REG) {
					result = 1;
					if (pCRCData != NULL) {
						pCRCData->mnOffset = TAS2555_YRAM2_START_REG;
						pCRCData->mnLen = nReg + len - TAS2555_YRAM1_START_REG;
					}
				} else
					result = 0;
			}else
				result = 0;
		} else if(nPage == TAS2555_YRAM3_PAGE) {
			if (nReg > TAS2555_YRAM2_END_REG) {
				dev_err(pTAS2555->dev, "nReg 0x%x, len %d error\n", nReg, len);
				pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;
			} else if (nReg > TAS2555_YRAM3_END_REG)
				result = 0;
			else if (nReg >= TAS2555_YRAM3_START_REG) {
				if ((nReg + len -1) <= TAS2555_YRAM3_END_REG) {
					if (pCRCData != NULL) {
						pCRCData->mnOffset = nReg;
						pCRCData->mnLen = len;
					}
					result = 1;
				} else if((nReg + len -1) <= TAS2555_YRAM2_END_REG) {
					if (pCRCData != NULL) {
						pCRCData->mnOffset = nReg;
						pCRCData->mnLen = TAS2555_YRAM3_END_REG - nReg + 1;
					}
					result = 1;
				} else {
					dev_err(pTAS2555->dev, "nReg 0x%x, len %d error\n", nReg, len);
					pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;
				}
			} else if (len > 1) {
				if ((nReg + (len-1)) > TAS2555_YRAM2_END_REG) {
					dev_err(pTAS2555->dev, "nReg 0x%x, len %d error\n", nReg, len);
					pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;
				} else if ((nReg + (len-1)) >= TAS2555_YRAM3_START_REG) {
					if ((nReg + len -1) <= TAS2555_YRAM3_END_REG) {
						if (pCRCData != NULL) {
							pCRCData->mnOffset = TAS2555_YRAM3_START_REG;
							pCRCData->mnLen = nReg + len - TAS2555_YRAM3_START_REG;
						}
						result = 1;
					} else if((nReg + len -1) <= TAS2555_YRAM2_END_REG) {
						if (pCRCData != NULL) {
							pCRCData->mnOffset = TAS2555_YRAM3_START_REG;
							pCRCData->mnLen = TAS2555_YRAM3_END_REG - TAS2555_YRAM3_START_REG + 1;
						}
						result = 1;
					} else {
						pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;
						dev_err(pTAS2555->dev, "nReg 0x%x, len %d error\n", nReg, len);
					}
				} else 
					result = 0;
			} else 
				result = 0;
		} else 
			result = 0;
	} else
		result = 0;

	return result;
}

/*
 * crc8 - calculate a crc8 over the given input data.
 *
 * table: crc table used for calculation.
 * pdata: pointer to data buffer.
 * nbytes: number of bytes in data buffer.
 * crc:	previous returned crc8 value.
 */
static u8 ti_crc8(const u8 table[CRC8_TABLE_SIZE], u8 *pdata, size_t nbytes, u8 crc)
{
	/* loop over the buffer data */
	while (nbytes-- > 0)
		crc = table[(crc ^ *pdata++) & 0xff];

	return crc;
}

static int doSingleRegCheckSum(struct tas2555_priv *pTAS2555,
	unsigned char nBook, unsigned char nPage, unsigned char nReg, unsigned char nValue)
{
	int nResult = -1;
	unsigned int nData = 0;
	unsigned char nRegVal = 0;

	nResult = isYRAM(pTAS2555, NULL, nBook, nPage, nReg, 1);
	if (nResult < 0) {
		dev_err(pTAS2555->dev, "firmware error\n");
		goto end;
	} else if (nResult == 1) {
		nResult = pTAS2555->read(pTAS2555, TAS2555_REG(nBook, nPage, nReg), &nData);
		if (nResult < 0) {
			goto end;
		}
		nRegVal = (unsigned char)nData;
		if(nValue != nRegVal){
			dev_err(pTAS2555->dev, 
				"error (line %d),B[0x%x]P[0x%x]R[0x%x] W[0x%x], R[0x%x]\n", 
				__LINE__, nBook, nPage, nReg, nValue, nRegVal);
			nResult = -EAGAIN;
			pTAS2555->mnErrorCode |= TAS2555_ERROR_VALUENOTMATCH;
			goto end;
		}

		nResult = ti_crc8(crc8_lookup_table, &nRegVal, 1, 0);
	}

end:

	return nResult;
}

static int doMultiRegCheckSum(struct tas2555_priv *pTAS2555,
	unsigned char nBook, unsigned char nPage, unsigned char nReg, unsigned int len)
{
	int nResult = -1, i;
	unsigned char nCRCChkSum = 0;
	unsigned char nBuf[127];
	TYCRC TCRCData;

	if ((nReg + len-1) > 127) {
		dev_err(pTAS2555->dev, "firmware error\n");
		pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;
		goto err;
	}

	nResult = isYRAM(pTAS2555, &TCRCData, nBook, nPage, nReg, len);
	if (nResult < 0) {
		dev_err(pTAS2555->dev, "firmware error\n");
		goto err;
	} else if (nResult == 1) {
		if (len == 1) {
			/* here shouldn't happen */
			dev_err(pTAS2555->dev, "firmware error\n");
			goto err;
		} else {
			nResult = pTAS2555->bulk_read(pTAS2555, 
				TAS2555_REG(nBook, nPage, TCRCData.mnOffset), nBuf, TCRCData.mnLen);  
			if(nResult < 0) {
				goto err;
			}

			for (i=0; i < TCRCData.mnLen; i++) {
				nCRCChkSum += ti_crc8(crc8_lookup_table, &nBuf[i], 1, 0);
			}
			nResult = nCRCChkSum;
		}
	}

err:

	return nResult;
}


static int tas2555_load_block(struct tas2555_priv *pTAS2555, TBlock * pBlock)
{
	int nResult = 0;
	int nRetry = 6;
	unsigned int nCommand = 0;
	unsigned char nBook;
	unsigned char nPage;
	unsigned char nOffset;
	unsigned char nData;
	unsigned int nLength;
	unsigned int nValue = 0;
	unsigned char nCRCChkSum = 0;
	unsigned char *pData = pBlock->mpData;

	dev_dbg(pTAS2555->dev, "TAS2555 load block: Type = %d, commands = %d\n",
		pBlock->mnType, pBlock->mnCommands);

start:
	if (pBlock->mbPChkSumPresent) {
		nResult = pTAS2555->write(pTAS2555, TAS2555_CRC_RESET_REG, 1);
		if (nResult < 0) {
			dev_err(pTAS2555->dev, "I2C err\n");
			goto end;
		}
		pTAS2555->mnErrorCode &= ~TAS2555_ERROR_PCHKSUM;
	}

	if (pBlock->mbYChkSumPresent) {
		nCRCChkSum = 0;
		pTAS2555->mnErrorCode &= ~TAS2555_ERROR_YCHKSUM;
	}

	nCommand = 0;

	while (nCommand < pBlock->mnCommands) {
		pData = pBlock->mpData + nCommand * 4;
		nBook = pData[0];
		nPage = pData[1];
		nOffset = pData[2];
		nData = pData[3];

		nCommand++;

		if (nOffset <= 0x7F){
			nResult = pTAS2555->write(pTAS2555, TAS2555_REG(nBook, nPage, nOffset), nData);
			if (nResult < 0) {
				goto end;
			}
			if (pBlock->mbYChkSumPresent) {
				nResult = doSingleRegCheckSum(pTAS2555, nBook, nPage, nOffset, nData);
				if (nResult < 0)
					goto check;
				nCRCChkSum += (unsigned char)nResult;
			}
		} else if (nOffset == 0x81) {
			unsigned int nSleep = (nBook << 8) + nPage;
			msleep(nSleep);
		} else if (nOffset == 0x85) {
			pData += 4;
			nLength = (nBook << 8) + nPage;
			nBook = pData[0];
			nPage = pData[1];
			nOffset = pData[2];
			if (nLength > 1) {
				nResult = pTAS2555->bulk_write(pTAS2555, TAS2555_REG(nBook, nPage, nOffset), pData + 3, nLength);
				if (nResult < 0) {
					goto end;
				}
				if (pBlock->mbYChkSumPresent) {
					nResult = doMultiRegCheckSum(pTAS2555, nBook, nPage, nOffset, nLength);
					if (nResult < 0)
						goto check;

					nCRCChkSum += (unsigned char)nResult;
				}
			} else {
				nResult = pTAS2555->write(pTAS2555, TAS2555_REG(nBook, nPage, nOffset), pData[3]);
				if (nResult < 0) {
					goto end;
				}
				if (pBlock->mbYChkSumPresent) {
					nResult = doSingleRegCheckSum(pTAS2555, nBook, nPage, nOffset, pData[3]);
					if (nResult < 0)
						goto check;
					nCRCChkSum += (unsigned char)nResult;
				}
			}
			nCommand++;
			if (nLength >= 2)
				nCommand += ((nLength - 2) / 4) + 1;
		}
	}

	if (pBlock->mbPChkSumPresent) {
		pTAS2555->read(pTAS2555, TAS2555_CRC_CHECKSUM_REG, &nValue);
		if ((nValue&0xff) != pBlock->mnPChkSum) {
			dev_err(pTAS2555->dev, "Block PChkSum Error: FW = 0x%x, Reg = 0x%x\n",
				pBlock->mnPChkSum, (nValue&0xff));
			nResult = -EAGAIN;
			pTAS2555->mnErrorCode |= TAS2555_ERROR_PCHKSUM;
			goto check;
		} else {
			nResult = 0;
			dev_dbg(pTAS2555->dev, "Block[0x%x] PChkSum match\n", pBlock->mnType);
		}
	}
	
	if (pBlock->mbYChkSumPresent) {
		if (nCRCChkSum != pBlock->mnYChkSum) {
			dev_err(pTAS2555->dev, "Block YChkSum Error: FW = 0x%x, YCRC = 0x%x\n",
			pBlock->mnYChkSum, nCRCChkSum);
			pTAS2555->mnErrorCode |= TAS2555_ERROR_YCHKSUM;
			nResult = -EAGAIN;
			goto check;
		} else {
			nResult = 0;
			dev_dbg(pTAS2555->dev, "Block[0x%x] YChkSum match\n", pBlock->mnType);
		}
	}

check:
	if (nResult == -EAGAIN) {
		nRetry--;
		if (nRetry > 0)
			goto start;
	}

end:
	if (nResult < 0) {
		dev_err(pTAS2555->dev, "Block (%d) load error\n",
				pBlock->mnType);
	}
	return nResult;
}

static int tas2555_load_data(struct tas2555_priv *pTAS2555, TData * pData,
	unsigned int nType)
{
	unsigned int nBlock;
	int nResult = 0;
	TBlock *pBlock;

	dev_dbg(pTAS2555->dev,
		"TAS2555 load data: %s, Blocks = %d, Block Type = %d\n", pData->mpName,
		pData->mnBlocks, nType);

	for (nBlock = 0; nBlock < pData->mnBlocks; nBlock++) {
		pBlock = &(pData->mpBlocks[nBlock]);
		if (pBlock->mnType == nType) {
			nResult = tas2555_load_block(pTAS2555, pBlock);
			if (nResult < 0)
				break;
		}
	}

	return nResult;
}

static int tas2555_get_ReCoefficient(struct tas2555_priv *pTAS2555, unsigned int *pRe)
{
	int ret = 0;
	unsigned char Buf[4];

	/* wait DSP to update the XRAM */
	udelay(2000);

	ret = pTAS2555->bulk_read(pTAS2555, TAS2555_COEFFICENT_RE_REG, Buf, 4);
	if (ret < 0) {
		goto err;
	}

	*pRe = ((unsigned int)Buf[0] << 24) | ((unsigned int)Buf[1] << 16) 
			| ((unsigned int)Buf[2] << 8) | Buf[3];

err:

	return ret;
}

int tas2555_enable(struct tas2555_priv *pTAS2555, bool bEnable)
{
	int nResult = 0;
	#if 0
	unsigned int nValue1, nValue2;
	#else
	unsigned int nValue2;
	#endif
	TConfiguration *pConfiguration;

	dev_dbg(pTAS2555->dev, "Enable: %d, test mute\n", bEnable);
	if ((!pTAS2555->mpFirmware->mpPrograms) ||
		(!pTAS2555->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2555->dev, "Firmware not loaded\n");
		pTAS2555->mnErrorCode |= TAS2555_ERROR_FWNOTLOAD;
		goto end;
	}
	
	if (bEnable) {
		if (!pTAS2555->mbPowerUp) {
		#if 0 
			nResult = pTAS2555->read(pTAS2555, TAS2555_DSP_MODE_SELECT_REG, &nValue1);
			if (nResult < 0)
				goto end;

			if (pTAS2555->mnCurrentProgram == 0) {
				/* smart-amp mode */
				if ((nValue1 & 0x03) != 0) {
					/* unexpected error happens */
					nResult = -1;
					goto end;
				}
			}
		#endif
			nResult = pTAS2555->read(pTAS2555, TAS2555_SAFE_GUARD_REG, &nValue2);
			if ((nValue2 & 0xff) != TAS2555_SAFE_GUARD_PATTERN) {
					/* failed to pass safe guard check */
					nResult = -1;
					goto end;
			}

			if (!pTAS2555->mbCalibrationLoaded) {
				tas2555_load_calibration(pTAS2555, TAS2555_CAL_NAME);
				pTAS2555->mbCalibrationLoaded = true;
			}
			dev_dbg(pTAS2555->dev, "Enable: load startup sequence\n");
//			nResult = pTAS2555->read(pTAS2555, TAS2555_FLAGS_1, &nValue); 
//			nResult = pTAS2555->read(pTAS2555, TAS2555_FLAGS_2, &nValue); 
			nResult = tas2555_dev_load_data(pTAS2555, p_tas2555_startup_data);
			if (nResult < 0)
				goto end;
			if (pTAS2555->mpFirmware->mpConfigurations) {
				pConfiguration = &(pTAS2555->mpFirmware->mpConfigurations[pTAS2555->mnCurrentConfiguration]);
				nResult = tas2555_load_data(pTAS2555, &(pConfiguration->mData), TAS2555_BLOCK_CONF_POST_POWER);
				if (nResult < 0)
					goto end;
				if (pTAS2555->mbLoadConfigurationPostPowerUp) {
					dev_dbg(pTAS2555->dev,	"Enable: load configuration: %s, %s\n", pConfiguration->mpName, pConfiguration->mpDescription);
					nResult = tas2555_load_data(pTAS2555, &(pConfiguration->mData), TAS2555_BLOCK_CONF_COEFF);
					if (nResult < 0)
						goto end;
					pTAS2555->mbLoadConfigurationPostPowerUp = false;
					if (pTAS2555->mpCalFirmware->mnCalibrations) {
						dev_dbg(pTAS2555->dev, "Enable: load calibration\n");
						nResult = tas2555_load_block(pTAS2555,
							&(pTAS2555->mpCalFirmware->mpCalibrations[pTAS2555->mnCurrentCalibration].mBlock));
						pTAS2555->mbCalibrationReUpdated = true;
						if (nResult < 0)
							goto end;
						nResult = tas2555_get_ReCoefficient(pTAS2555, &pTAS2555->mnReOrignal);
						if (nResult < 0)
							goto end;
						pTAS2555->mbLoadCalibrationPostPowerUp = false;
					} else if (pTAS2555->mnReOrignal == 0) {
						nResult = tas2555_get_ReCoefficient(pTAS2555, &pTAS2555->mnReOrignal);
						if (nResult < 0)
							goto end;
					}
				} else {
					if (pTAS2555->mpCalFirmware->mnCalibrations) {
						if (pTAS2555->mbLoadCalibrationPostPowerUp) {
							dev_dbg(pTAS2555->dev, "Enable: load calibration\n");
							nResult = tas2555_load_block(pTAS2555,
								&(pTAS2555->mpCalFirmware->mpCalibrations[pTAS2555->mnCurrentCalibration].mBlock));
							pTAS2555->mbCalibrationReUpdated = true;							
							if (nResult < 0)
								goto end;
							nResult = tas2555_get_ReCoefficient(pTAS2555, &pTAS2555->mnReOrignal);
							if (nResult < 0)
								goto end;
							pTAS2555->mbLoadCalibrationPostPowerUp = false;
						}
					}
				}
			}

			dev_dbg(pTAS2555->dev, "Enable: load unmute sequence\n");
			nResult = tas2555_dev_load_data(pTAS2555, p_tas2555_unmute_data);
			if (nResult < 0)
				goto end;
			pTAS2555->mbPowerUp = true;
		}
	} else {
		if (pTAS2555->mbPowerUp) {
			dev_dbg(pTAS2555->dev, "Enable: load shutdown sequence\n");
			#if 0 
			nResult = pTAS2555->read(pTAS2555, TAS2555_FLAGS_1, &nValue1);
			nResult = pTAS2555->read(pTAS2555, TAS2555_FLAGS_2, &nValue1);
			#endif
			nResult = tas2555_dev_load_data(pTAS2555, p_tas2555_shutdown_data);
			//tas2555_dev_load_data(pTAS2555, p_tas2555_shutdown_clk_err);
			pTAS2555->mbPowerUp = false;
		}
	}

end:
	if (nResult < 0)
		failsafe(pTAS2555);

	return nResult;
}

int tas2555_set_sampling_rate(struct tas2555_priv *pTAS2555, unsigned int nSamplingRate)
{
	TConfiguration *pConfiguration;
	unsigned int nConfiguration;

	dev_dbg(pTAS2555->dev, "tas2555_setup_clocks: nSamplingRate = %d [Hz]\n",
		nSamplingRate);

	if ((!pTAS2555->mpFirmware->mpPrograms) ||
		(!pTAS2555->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2555->dev, "Firmware not loaded\n");
		return -EINVAL;
	}

	pConfiguration = &(pTAS2555->mpFirmware->mpConfigurations[pTAS2555->mnCurrentConfiguration]);
	if (pConfiguration->mnSamplingRate == nSamplingRate) {
		dev_info(pTAS2555->dev, "Sampling rate for current configuration matches: %d\n",
			nSamplingRate);
		return 0;
	}

	for (nConfiguration = 0;
		nConfiguration < pTAS2555->mpFirmware->mnConfigurations;
		nConfiguration++) {
		pConfiguration =
			&(pTAS2555->mpFirmware->mpConfigurations[nConfiguration]);
		if ((pConfiguration->mnSamplingRate == nSamplingRate)
			&&(pConfiguration->mnProgram == pTAS2555->mnCurrentProgram)){
			dev_info(pTAS2555->dev,
				"Found configuration: %s, with compatible sampling rate %d\n",
				pConfiguration->mpName, nSamplingRate);
			return tas2555_load_configuration(pTAS2555, nConfiguration, false);
		}
	}

	dev_err(pTAS2555->dev, "Cannot find a configuration that supports sampling rate: %d\n",
		nSamplingRate);

	return -EINVAL;
}

#define Q_FACTOR 0x08000000

static bool chkReDeltaBoundary(struct tas2555_priv *pTAS2555, 
	unsigned int ReOrginal, unsigned int ReDelta, unsigned int Re, unsigned char scale, unsigned int *pActRe)
{
	bool nResult = false;
	unsigned int ReHigh_calc, ReLow_calc, Re_calc = 0;
	static unsigned int counter = 3;
	
	if (ReOrginal < Q_FACTOR)
		goto end;

	switch (scale) {
	case 0:
	/* 8Ohm speaker */
		Re_calc = Re;
		*pActRe = Re;
		ReHigh_calc = ReOrginal + ReDelta;
		ReLow_calc = ReOrginal - ReDelta;
		break;

	case 1:
	/* 6Ohm speaker */
		ReDelta = ((ReDelta / 3) * 4);
		ReHigh_calc = ReOrginal + ReDelta;
		ReLow_calc = ReOrginal - ReDelta;
		Re_calc = Re;
		*pActRe = ((Re_calc / 4) * 3);
		break;

	case 2:
	/* 4Ohm speaker */
		ReDelta = ReDelta * 2;
		ReHigh_calc = ReOrginal + ReDelta;
		ReLow_calc = ReOrginal - ReDelta;
		Re_calc = Re;
		*pActRe = Re_calc / 2;
		break;
	}

	if (Re_calc > 0) {
		pTAS2555->mnReLastKnown = *pActRe;

		if (Re_calc > ReHigh_calc) {
			counter--;
			pTAS2555->mnErrorCode |= TAS2555_ERROR_RETOOHIGH;
		} else if (Re_calc < ReLow_calc) {
			counter--;
			pTAS2555->mnErrorCode |= TAS2555_ERROR_RETOOLOW;
		} else
			counter = 3;

		if (counter == 0) {
			dev_err(pTAS2555->dev, "ReDelta=0x%x, ReOriginal=0x%x, prepare failsafe,scale=%d\n", ReDelta, ReOrginal,scale);
			nResult = true;
		}
	}

end:
	return nResult;
}

static bool chkReBoundary(struct tas2555_priv *pTAS2555, 
	unsigned int ReHigh, unsigned int ReLow, unsigned int Re, unsigned char scale, unsigned int *pActRe)
{
	bool nResult = false;
	unsigned int ReHigh_calc, ReLow_calc, Re_calc = 0;
	static unsigned int counter = 3;
	dev_err(pTAS2555->dev, "chkReBoundary ReHigh=0x%x, ReLow=0x%x  scale=%d \n",ReHigh,ReLow,scale);
	if (ReHigh < Q_FACTOR)
		goto end;

	if (ReLow < Q_FACTOR)
		goto end;

	switch (scale) {
	case 0:
	/* 8Ohm speaker */
		Re_calc = Re;
		*pActRe = Re;
		ReHigh_calc = ReHigh;
		ReLow_calc = ReLow;
		break;

	case 1:

	/* 6Ohm speaker */
		ReHigh_calc = (ReHigh / 3) * 4;
		ReLow_calc = (ReLow / 3) * 4;
		Re_calc = Re;
		*pActRe = (Re_calc / 4) * 3;
		break;

	case 2:
	/* 4Ohm speaker */
		ReHigh_calc = ReHigh * 2;
		ReLow_calc = ReLow * 2;
		Re_calc = Re;
		*pActRe = Re_calc / 2;
		break;
	}

	if (Re_calc > 0) {
		dev_err(pTAS2555->dev, "Re_calc>0\n");
		pTAS2555->mnReLastKnown = *pActRe;

		if (Re_calc > ReHigh_calc) {
			counter--;
			pTAS2555->mnErrorCode |= TAS2555_ERROR_RETOOHIGH;
			dev_err(pTAS2555->dev, "tas2555_TAS2555_ERROR_RETOOHIGH counter=%d\n",counter);
		} else if (Re_calc < ReLow_calc) {
			counter--;
			pTAS2555->mnErrorCode |= TAS2555_ERROR_RETOOLOW;
			dev_err(pTAS2555->dev, "tas2555_TAS2555_ERROR_RETOOLOW counter=%d\n",counter);
		} else
			{
			counter = 3;
			dev_err(pTAS2555->dev, "tas2555 counter=3\n");
			}

		if (counter == 0) {
			dev_err(pTAS2555->dev,
				"ReHigh=0x%x, ReLow=0x%x, prepare failsafe,scale=%d\n",
				ReHigh, ReLow,scale);
			nResult = true;
		}
	}

end:
	return nResult;
}


int tas2555_get_Re(struct tas2555_priv *pTAS2555, unsigned int *pRe)
{
	int ret = 0;
	unsigned int nRe = 0, nValue;
	bool bFailSafe = false;

	if (pTAS2555->mbPowerUp) {
		ret = tas2555_get_ReCoefficient(pTAS2555, &nRe);
		if (ret < 0)
			goto err;

		ret = pTAS2555->read(pTAS2555, TAS2555_CHANNEL_CTRL_REG, &nValue);
		if (ret < 0) {
			dev_err(pTAS2555->dev, "I2C error\n");
			goto err;
		}

		nValue = (nValue & 0x06) >> 1;

		if(!pTAS2555->mbCalibrationReUpdated) {
			bFailSafe = chkReBoundary(pTAS2555,
							pTAS2555->mnReHigh,pTAS2555->mnReLow,
							nRe, nValue, pRe);
		} else {
			bFailSafe = chkReDeltaBoundary(pTAS2555, 
							pTAS2555->mnReOrignal, pTAS2555->mnReDelta, 
							nRe, nValue, pRe);
		}

		if (bFailSafe)
			failsafe(pTAS2555);
	} else {
		*pRe = pTAS2555->mnReLastKnown;
	}

err:

	return ret;
}

int tas2555_get_errcode(struct tas2555_priv *pTAS2555, unsigned int *pErrCode)
{
	unsigned int errcode = 0;
	unsigned int nValue1, nValue2;
	int nResult = 0;

	nResult = pTAS2555->read(pTAS2555, TAS2555_FLAGS_1, &nValue1);
	if (nResult >= 0) {
		nResult = pTAS2555->read(pTAS2555, TAS2555_FLAGS_2, &nValue2);

		if (nValue1 & 0x04) {
			errcode |= TAS2555_ERROR_CLKPRESENT;
			nResult = pTAS2555->write(pTAS2555, TAS2555_CLK_ERR_CTRL1, 0x00);
		}
		if (nValue1 & 0x08)
			errcode |= TAS2555_ERROR_BROWNOUT;
		if (nValue1 & 0x10)
			errcode |= TAS2555_ERROR_OVERTMP;
		if (nValue1 & 0x40)
			errcode |= TAS2555_ERROR_UNDERVOLTAGET;
		if (nValue1 & 0x80)
			errcode |= TAS2555_ERROR_OVERCURRENT;
		/* here is power down, do we need to fail safe? */
		if (nValue1 & 0xdc) 
			nResult = tas2555_enable(pTAS2555, false);
	} 

	*pErrCode = errcode | pTAS2555->mnErrorCode;
	pTAS2555->mnErrorCode = 0;
	return nResult;
}

/*
* die temperature calculation:
* 	deltaT = (nT1 - nT2 ) / 2^10
* 	DieTemp = (deltaT - 0.3459) / 0.001
*/
int tas2555_get_die_delta_temperature(struct tas2555_priv *pTAS2555, int *pDeltaT)
{
	unsigned char nBuf[4];
	int nResult = 0;
	unsigned int nT1 = 0, nT2 = 0;

	nResult = pTAS2555->bulk_read(pTAS2555, TAS2555_SAR_T1MSB_REG, nBuf, 4);
	if (nResult >= 0) {
		nT1 = ((unsigned int)nBuf[0] << 2) | (((unsigned int)nBuf[1] & 0xc0) >> 6);
		nT2 = ((unsigned int)nBuf[2] << 2) | (((unsigned int)nBuf[3] & 0xc0) >> 6);
		*pDeltaT = nT1 - nT2;
	}

	return nResult;
}

int tas2555_configIRQ(struct tas2555_priv *pTAS2555)
{
	return tas2555_dev_load_data(pTAS2555, p_tas2555_irq_config);
}

int tas2555_load_platdata(struct tas2555_priv *pTAS2555)
{
	int nResult= 0;
	if (gpio_is_valid(pTAS2555->mnGpioINT)) {
		nResult = tas2555_configIRQ(pTAS2555);
		if (nResult < 0)
			goto end;
		nResult = pTAS2555->enableIRQ(pTAS2555, false, true);
	}

end:
	return nResult;
}

int tas2555_load_default(struct tas2555_priv *pTAS2555)
{
	int nResult = 0;
	
	nResult = tas2555_dev_load_data(pTAS2555, p_tas2555_default_data);
	if (nResult >= 0)
		nResult = tas2555_load_platdata(pTAS2555);

	return nResult;
}

static int tas2555_load_configuration(struct tas2555_priv *pTAS2555,
	unsigned int nConfiguration, bool bLoadSame)
{
	int nResult = 0;
	TConfiguration *pCurrentConfiguration;
	TConfiguration *pNewConfiguration;
	TPLL *pNewPLL;
	unsigned int nValue;

	dev_dbg(pTAS2555->dev, "tas2555_load_configuration: %d\n", nConfiguration);

	if ((!pTAS2555->mpFirmware->mpPrograms) ||
		(!pTAS2555->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2555->dev, "Firmware not loaded\n");
		pTAS2555->mnErrorCode |= TAS2555_ERROR_FWNOTLOAD;
		return -1;
	}

	if (nConfiguration >= pTAS2555->mpFirmware->mnConfigurations) {
		dev_err(pTAS2555->dev, "Configuration %d doesn't exist\n",
			nConfiguration);
		pTAS2555->mnErrorCode |= TAS2555_ERROR_INVALIDPARAM;
		return -1;
	}

	if ((nConfiguration == pTAS2555->mnCurrentConfiguration) && (!bLoadSame)) {
		dev_info(pTAS2555->dev, "Configuration %d is already loaded\n",
			nConfiguration);
		return 0;
	}

	pCurrentConfiguration =
		&(pTAS2555->mpFirmware->mpConfigurations[pTAS2555->mnCurrentConfiguration]);
	pNewConfiguration =
		&(pTAS2555->mpFirmware->mpConfigurations[nConfiguration]);

	if (pNewConfiguration->mnProgram != pCurrentConfiguration->mnProgram) {
		dev_err(pTAS2555->dev,
			"Configuration %d, %s doesn't share the same program as current %d\n",
			nConfiguration, pNewConfiguration->mpName, pCurrentConfiguration->mnProgram);
		pTAS2555->mnErrorCode |= TAS2555_ERROR_INVALIDPARAM;
		return -1;
	}

	if (pNewConfiguration->mnPLL >= pTAS2555->mpFirmware->mnPLLs) {
		dev_err(pTAS2555->dev,
			"Configuration %d, %s doesn't have a valid PLL index %d\n",
			nConfiguration, pNewConfiguration->mpName, pNewConfiguration->mnPLL);
		pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;
		return -1;
	}

	pNewPLL = &(pTAS2555->mpFirmware->mpPLLs[pNewConfiguration->mnPLL]);

	if (pTAS2555->mbPowerUp) {
		if (pNewConfiguration->mnPLL != pCurrentConfiguration->mnPLL) {
			dev_dbg(pTAS2555->dev,
				"TAS2555 is powered up -> mute and power down DSP before loading new configuration\n");
			//tas2555_dev_load_data(pTAS2555, p_tas2555_mute_DSP_down_data);
			nResult = tas2555_dev_load_data(pTAS2555, p_tas2555_shutdown_data);
			if (nResult < 0)
				goto end;
			dev_dbg(pTAS2555->dev,
				"load post block from current configuration: %s, before loading new configuration: %s\n",
				pCurrentConfiguration->mpName, pNewConfiguration->mpName);
			nResult = tas2555_load_data(pTAS2555, &(pCurrentConfiguration->mData),
				TAS2555_BLOCK_CONF_POST);
			if (nResult < 0)
				goto end;
			dev_dbg(pTAS2555->dev, "TAS2555: load new PLL: %s, block data\n",
				pNewPLL->mpName);
			nResult = tas2555_load_block(pTAS2555, &(pNewPLL->mBlock));
			if (nResult < 0)
				goto end;
			pTAS2555->mnCurrentSampleRate = pNewConfiguration->mnSamplingRate;
			dev_dbg(pTAS2555->dev,
				"load new configuration: %s, pre block data\n",
				pNewConfiguration->mpName);
			nResult = tas2555_load_data(pTAS2555, &(pNewConfiguration->mData),
				TAS2555_BLOCK_CONF_PRE);
			if (nResult < 0)
				goto end;
			dev_dbg(pTAS2555->dev, "TAS2555: power up TAS2555\n");
			nResult = pTAS2555->read(pTAS2555, TAS2555_FLAGS_1, &nValue);
			nResult = pTAS2555->read(pTAS2555, TAS2555_FLAGS_2, &nValue);			
			nResult = tas2555_dev_load_data(pTAS2555, p_tas2555_startup_data);
			if (nResult < 0)
				goto end;
			dev_dbg(pTAS2555->dev,
				"TAS2555: load new configuration: %s, post power up block data\n",
				pNewConfiguration->mpName);
			nResult = tas2555_load_data(pTAS2555, &(pNewConfiguration->mData),
				TAS2555_BLOCK_CONF_POST_POWER);
			if (nResult < 0)
				goto end;
			dev_dbg(pTAS2555->dev,
				"TAS2555: load new configuration: %s, coeff block data\n",
				pNewConfiguration->mpName);
			nResult = tas2555_load_data(pTAS2555, &(pNewConfiguration->mData),
				TAS2555_BLOCK_CONF_COEFF);
			if (nResult < 0)
				goto end;
			dev_dbg(pTAS2555->dev, "TAS2555: unmute TAS2555\n");
			nResult = tas2555_dev_load_data(pTAS2555, p_tas2555_unmute_data);
			if (nResult < 0)
				goto end;
		} else {
			dev_dbg(pTAS2555->dev,
				"TAS2555 is powered up, no change in PLL: load new configuration: %s, coeff block data\n",
				pNewConfiguration->mpName);
			nResult = tas2555_load_data(pTAS2555, &(pNewConfiguration->mData),
				TAS2555_BLOCK_CONF_COEFF);
			if (nResult < 0)
				goto end;
		}

		if (pTAS2555->mpCalFirmware->mnCalibrations) {
				dev_dbg(pTAS2555->dev, "Enable: load calibration\n");
				nResult = tas2555_load_block(pTAS2555, 
					&(pTAS2555->mpCalFirmware->mpCalibrations[pTAS2555->mnCurrentCalibration].mBlock));
				pTAS2555->mbLoadCalibrationPostPowerUp = false;
				pTAS2555->mbCalibrationReUpdated = true;				
				if (nResult < 0)
					goto end;
		}

		pTAS2555->mbLoadConfigurationPostPowerUp = false;
	} else {
		dev_dbg(pTAS2555->dev,
			"TAS2555 was powered down -> set flag to load configuration data when OS powers up the TAS2555 the next time\n");
		if (pNewConfiguration->mnPLL != pCurrentConfiguration->mnPLL) {
			dev_dbg(pTAS2555->dev,
				"load post block from current configuration: %s, before loading new configuration: %s\n",
				pCurrentConfiguration->mpName, pNewConfiguration->mpName);
			nResult = tas2555_load_data(pTAS2555, &(pCurrentConfiguration->mData),
				TAS2555_BLOCK_CONF_POST);
			if (nResult < 0)
				goto end;
			dev_dbg(pTAS2555->dev, "TAS2555: load new PLL: %s, block data\n",
				pNewPLL->mpName);
			nResult = tas2555_load_block(pTAS2555, &(pNewPLL->mBlock));
			if (nResult < 0)
				goto end;
			pTAS2555->mnCurrentSampleRate = pNewConfiguration->mnSamplingRate;
			dev_dbg(pTAS2555->dev,
				"load new configuration: %s, pre block data\n",
				pNewConfiguration->mpName);
			nResult = tas2555_load_data(pTAS2555, &(pNewConfiguration->mData),
				TAS2555_BLOCK_CONF_PRE);
			if (nResult < 0)
				goto end;
		}

		pTAS2555->mbLoadConfigurationPostPowerUp = true;
	}

	pTAS2555->mnCurrentConfiguration = nConfiguration;

end:

	if (nResult < 0)
		failsafe(pTAS2555);

	return nResult;
}

int tas2555_set_config(struct tas2555_priv *pTAS2555, int config)
{
	TConfiguration *pConfiguration;
	TProgram *pProgram;
	unsigned int nProgram = pTAS2555->mnCurrentProgram;
	unsigned int nConfiguration = config;

	if ((!pTAS2555->mpFirmware->mpPrograms) ||
		(!pTAS2555->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2555->dev, "Firmware not loaded\n");
		pTAS2555->mnErrorCode |= TAS2555_ERROR_FWNOTLOAD;
		return -1;
	}

	if (nConfiguration >= pTAS2555->mpFirmware->mnConfigurations) {
		dev_err(pTAS2555->dev, "Configuration %d doesn't exist\n",
			nConfiguration);
		pTAS2555->mnErrorCode |= TAS2555_ERROR_INVALIDPARAM;
		return -1;
	}

	pConfiguration = &(pTAS2555->mpFirmware->mpConfigurations[nConfiguration]);
	pProgram = &(pTAS2555->mpFirmware->mpPrograms[nProgram]);

	if (nProgram != pConfiguration->mnProgram) {
		dev_err(pTAS2555->dev,
			"Configuration %d, %s with Program %d isn't compatible with existing Program %d, %s\n",
			nConfiguration, pConfiguration->mpName, pConfiguration->mnProgram,
			nProgram, pProgram->mpName);
		pTAS2555->mnErrorCode |= TAS2555_ERROR_INVALIDPARAM;
		return -1;
	}

	tas2555_load_configuration(pTAS2555, nConfiguration, false);

	return 0;
}

int tas2555_set_program(struct tas2555_priv *pTAS2555, unsigned int nProgram, int nConfig)
{
	TPLL *pPLL;
	TConfiguration *pConfiguration;
	unsigned int nConfiguration = 0;
	unsigned int nSampleRate = 0;
	unsigned int Value = 0;
	bool bFound = false;
	int nResult = -1;

	if ((!pTAS2555->mpFirmware->mpPrograms) ||
		(!pTAS2555->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2555->dev, "Firmware not loaded\n");
		pTAS2555->mnErrorCode |= TAS2555_ERROR_FWNOTLOAD;
		return -1;
	}
	
	if (nProgram >= pTAS2555->mpFirmware->mnPrograms) {
		dev_err(pTAS2555->dev, "TAS2555: Program %d doesn't exist\n",
			nConfiguration);
		pTAS2555->mnErrorCode |= TAS2555_ERROR_INVALIDPARAM;
		return -1;
	}

	if (nConfig < 0) {
		nConfiguration = 0;
		nSampleRate = pTAS2555->mnCurrentSampleRate;

		while (!bFound 
			&& (nConfiguration < pTAS2555->mpFirmware->mnConfigurations)) {
			if (pTAS2555->mpFirmware->mpConfigurations[nConfiguration].mnProgram 
				== nProgram){
				if(nSampleRate == 0){
					bFound = true;
					dev_info(pTAS2555->dev, "find default configuration %d\n", nConfiguration);
				}else if(nSampleRate 
					== pTAS2555->mpFirmware->mpConfigurations[nConfiguration].mnSamplingRate){
					bFound = true;
					dev_info(pTAS2555->dev, "find matching configuration %d\n", nConfiguration);
				}else{
					nConfiguration++;
				}
			}else{
				nConfiguration++;
			}
		}
		if (!bFound) {
			dev_err(pTAS2555->dev, 
				"Program %d, no valid configuration found for sample rate %d, ignore\n",
				nProgram, nSampleRate);
			pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;
			return -1;
		}
	} else
		nConfiguration = nConfig;

	pTAS2555->mnCurrentProgram = nProgram;

	if (pTAS2555->mbPowerUp) {
		nResult = pTAS2555->enableIRQ(pTAS2555, false, true);
		if (nResult < 0)
			goto end;
		nResult = tas2555_dev_load_data(pTAS2555, p_tas2555_mute_DSP_down_data);
		if (nResult < 0)
			goto end;
	}

	nResult = pTAS2555->write(pTAS2555, TAS2555_SW_RESET_REG, 0x01);
	if (nResult < 0) {
		goto end;
	}
	msleep(1);

	nResult = tas2555_load_default(pTAS2555);
	if (nResult < 0)
		goto end;

	msleep(1);
	pTAS2555->mnCurrentBook = 0;
	pTAS2555->mnCurrentPage = 0;
	
	dev_info(pTAS2555->dev, "load program %d\n", nProgram);
	nResult = tas2555_load_data(pTAS2555, &(pTAS2555->mpFirmware->mpPrograms[nProgram].mData), TAS2555_BLOCK_BASE_MAIN);
	if (nResult < 0)
		goto end;

	pTAS2555->mnCurrentConfiguration = nConfiguration;

	pConfiguration = &(pTAS2555->mpFirmware->mpConfigurations[nConfiguration]);
	pPLL = &(pTAS2555->mpFirmware->mpPLLs[pConfiguration->mnPLL]);
	dev_dbg(pTAS2555->dev, "TAS2555 load PLL: %s block for Configuration %s\n", pPLL->mpName, pConfiguration->mpName);
	
	nResult = tas2555_load_block(pTAS2555, &(pPLL->mBlock));
	if (nResult < 0)
		goto end;
	pTAS2555->mnCurrentSampleRate = pConfiguration->mnSamplingRate;
	dev_dbg(pTAS2555->dev,
		"load configuration %s conefficient pre block\n",
		pConfiguration->mpName);
	nResult = tas2555_load_data(pTAS2555, &(pConfiguration->mData), TAS2555_BLOCK_CONF_PRE);
	if(nResult < 0)
		goto end;
	nResult = pTAS2555->read(pTAS2555, TAS2555_PLL_CLKIN_REG, &Value);
	if(nResult < 0) {
		goto end;
	}
	dev_info(pTAS2555->dev, "TAS2555 PLL_CLKIN = 0x%02X\n", Value);
	p_tas2555_startup_data[TAS2555_STARTUP_DATA_PLL_CLKIN_INDEX] = Value;

	if (pTAS2555->mbPowerUp){
		dev_dbg(pTAS2555->dev, "device powered up, load startup\n");
			nResult = pTAS2555->read(pTAS2555, TAS2555_FLAGS_1, &Value);
			nResult = pTAS2555->read(pTAS2555, TAS2555_FLAGS_2, &Value);		
		nResult = tas2555_dev_load_data(pTAS2555, p_tas2555_startup_data);
		if(nResult < 0)
			goto end;
		dev_dbg(pTAS2555->dev, 
			"device powered up, load configuration %s post power block\n",
			pConfiguration->mpName);
		nResult = tas2555_load_data(pTAS2555, &(pConfiguration->mData),
			TAS2555_BLOCK_CONF_POST_POWER);
		if(nResult < 0)
			goto end;
	}
	
	nResult = tas2555_load_configuration(pTAS2555, nConfiguration, true);
	if (nResult < 0)
		goto end;

	if (pTAS2555->mbPowerUp){
		dev_dbg(pTAS2555->dev,
			"device powered up, load unmute\n");
		nResult = tas2555_dev_load_data(pTAS2555, p_tas2555_unmute_data);
		if (nResult < 0)
			goto end;
	}

end:

	if (nResult < 0)
		failsafe(pTAS2555);

	return nResult;
}

void tas2555_fw_ready(const struct firmware *pFW, void *pContext)
{	
	struct tas2555_priv *pTAS2555 = (struct tas2555_priv *) pContext;	
	int nResult;	
	unsigned int nProgram = 0;	
	unsigned int nSampleRate = 0;	
	int nConfiguration = 0;

	dev_info(pTAS2555->dev, "%s:\n", __func__);	
	if (unlikely(!pFW) || unlikely(!pFW->data)) {		
		dev_err(pTAS2555->dev, "%s firmware is not loaded\n", TAS2555_FW_NAME);
		pTAS2555->mnErrorCode |= TAS2555_ERROR_FWNOTLOAD;		
		return;	
	}	
	
	nProgram = pTAS2555->mnCurrentProgram;
	nSampleRate = pTAS2555->mnCurrentSampleRate;
	nConfiguration = pTAS2555->mnCurrentConfiguration;

	if (pTAS2555->mpFirmware->mpConfigurations) {
		dev_dbg(pTAS2555->dev, "clear current firmware\n");
		tas2555_clear_firmware(pTAS2555->mpFirmware);
	}

	nResult = fw_parse(pTAS2555, pTAS2555->mpFirmware,
					(unsigned char *) (pFW->data), pFW->size);	
	release_firmware(pFW);	

	if (nResult) {		
		pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;		
		dev_err(pTAS2555->dev, "firmware is corrupt\n");		
		return;	
	}	

	if (!pTAS2555->mpFirmware->mnPrograms) {		
		pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;		
		dev_err(pTAS2555->dev, "firmware contains no programs\n");		
		return;	
	}	

	if (!pTAS2555->mpFirmware->mnConfigurations) {		
		dev_err(pTAS2555->dev, 			
			"firmware contains no configurations\n");		
		pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;		
		return;	
	}	

	if(nProgram >= pTAS2555->mpFirmware->mnPrograms){		
		dev_info(pTAS2555->dev, 			
			"no previous program, set to default\n");		
		nProgram = 0;	
	}	

	if (nConfiguration >= pTAS2555->mpFirmware->mnConfigurations) {
		dev_info(pTAS2555->dev, 
			"no previous program, set to default\n");
		nConfiguration = -1;
	}

	if (nConfiguration >= 0) {
		if (pTAS2555->mpFirmware->mpConfigurations[nConfiguration].mnProgram != nProgram)
			nConfiguration = -1;
	}

	if (nConfiguration >= 0) {
		if (pTAS2555->mpFirmware->mpConfigurations[nConfiguration].mnSamplingRate != nSampleRate)
			nConfiguration = -1;
	}

	tas2555_set_program(pTAS2555, nProgram, nConfiguration);
}

int tas2555_set_calibration(struct tas2555_priv *pTAS2555,
	int nCalibration)
{
	int nResult = 0;
	if ((!pTAS2555->mpFirmware->mpPrograms) || (!pTAS2555->mpFirmware->mpConfigurations)) 
	{
		dev_err(pTAS2555->dev, "Firmware not loaded\n\r");
		return -1;
	}

	if (nCalibration == 0x00FF)
	{
		dev_info(pTAS2555->dev, "load new calibration file %s\n", TAS2555_CAL_NAME); 	
		tas2555_load_calibration(pTAS2555, TAS2555_CAL_NAME);
		nCalibration = 0;
	}

	if (nCalibration >= pTAS2555->mpFirmware->mnCalibrations) {
		dev_err(pTAS2555->dev,
			"Calibration %d doesn't exist\n", nCalibration);
		pTAS2555->mnErrorCode |= TAS2555_ERROR_INVALIDPARAM;
		return -1;
	}

	pTAS2555->mnCurrentCalibration = nCalibration;
	if(pTAS2555->mbPowerUp){
		nResult = tas2555_load_block(pTAS2555, 
			&(pTAS2555->mpCalFirmware->mpCalibrations[pTAS2555->mnCurrentCalibration].mBlock));
		if (nResult >= 0) {
			pTAS2555->mbCalibrationReUpdated = true;
			nResult = tas2555_get_ReCoefficient(pTAS2555,&pTAS2555->mnReOrignal);
			pTAS2555->mbLoadCalibrationPostPowerUp = false; 
		}
	}else{
		pTAS2555->mbLoadCalibrationPostPowerUp = true; 
	}

	return nResult;
}

static int tas2555_power_ctrl_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue);
static int tas2555_power_ctrl_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue);


static const char *const smp_function[] = { "Off", "On" };                                  //wilbur add smp ctl
static const struct soc_enum SMP_Enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(smp_function), smp_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(smp_function), smp_function),};
static const struct snd_kcontrol_new tas2555_smp_ctl_controls[] = {
	SOC_ENUM_EXT("tas2555_smp_switch", SMP_Enum[0], tas2555_power_ctrl_get,             
		    tas2555_power_ctrl_put),

};

static int tas2555_codec_probe(struct snd_soc_codec *pCodec)
{
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(pCodec);

	dev_err(pCodec->dev, "%s\n", __func__);
	pTAS2555->mpCodec = pCodec;
	pCodec->control_data = (void *) g_client;

    dev_set_drvdata(pCodec->dev, pTAS2555);                    //wilbur add 

    snd_soc_add_codec_controls(pCodec, tas2555_smp_ctl_controls, ARRAY_SIZE(tas2555_smp_ctl_controls));   //wilbur  tas2555_smp_ctl  instead of dapm 

   // tas2555_dev_load_data(pTAS2555, p_tas2555_shutdown_data);//start up powerdown oliverchen 20160217

	return 0;
}

static int tas2555_codec_remove(struct snd_soc_codec *pCodec)
{
	return 0;
}

static int tas2555_get_reg_addr(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pUcontrol)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);

	pUcontrol->value.integer.value[0] 
		= TAS2555_REG((unsigned int) register_addr.book,
			(unsigned int) register_addr.page,
			(unsigned int) register_addr.reg);

	dev_dbg(pTAS2555->dev, "%s: Get address [%d, %d, %d]\n", __func__,
		register_addr.book, register_addr.page, register_addr.reg);

	return 0;
}

static int tas2555_put_reg_addr(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pUcontrol)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	unsigned int nValue = pUcontrol->value.integer.value[0];
	register_addr.book = TAS2555_BOOK_ID(nValue);
	register_addr.page = TAS2555_PAGE_ID(nValue);
	register_addr.reg = TAS2555_PAGE_REG(nValue);

	dev_dbg(pTAS2555->dev, "%s: Set address [%d, %d, %d]\n", __func__,
		register_addr.book, register_addr.page, register_addr.reg);

	return 0;
}

static int tas2555_get_reg_value(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pUcontrol)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	unsigned int nValue;
	unsigned int reg;

	if (TAS2555_REG_IS_VALID(register_addr.book,
			register_addr.page, register_addr.reg)) {
		reg = TAS2555_REG((unsigned int) register_addr.book,
			(unsigned int) register_addr.page,
			(unsigned int) register_addr.reg);
		pTAS2555->read(pTAS2555, reg, &nValue);
		pUcontrol->value.integer.value[0] = nValue;
	} else {
		dev_err(pTAS2555->dev, "%s: Invalid register address!\n", __func__);
		pUcontrol->value.integer.value[0] = 0xFFFF;
	}

	dev_dbg(pTAS2555->dev, "%s: Read [%d, %d, %d] = %ld\n", __func__,
		register_addr.book, register_addr.page, register_addr.reg,
		pUcontrol->value.integer.value[0]);
		
	return 0;
}

static int tas2555_put_reg_value(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pUcontrol)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	unsigned int reg;


	if (TAS2555_REG_IS_VALID(register_addr.book,
			register_addr.page, register_addr.reg)) {
		reg = TAS2555_REG((unsigned int) register_addr.book,
			(unsigned int) register_addr.page,
			(unsigned int) register_addr.reg);
		pTAS2555->write(pTAS2555, reg, pUcontrol->value.integer.value[0]);
	} else {
		dev_err(pTAS2555->dev, "%s: Invalid register address!\n", __func__);
	}

	dev_dbg(pTAS2555->dev, "%s: Write [%d, %d, %d] = %ld\n", __func__,
		register_addr.book, register_addr.page, register_addr.reg,
		pUcontrol->value.integer.value[0]);

	return 0;
}

static int tas2555_power_ctrl_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);

	pValue->value.integer.value[0] = pTAS2555->mnPowerCtrl;
	dev_dbg(pTAS2555->dev, "tas2555_power_ctrl_get = %d\n",
		pTAS2555->mnPowerCtrl);
		
	return 0;
}

static int tas2555_power_ctrl_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);

	pTAS2555->mnPowerCtrl = pValue->value.integer.value[0];

	dev_dbg(pTAS2555->dev, "tas2555_power_ctrl_put = %d\n",
		pTAS2555->mnPowerCtrl);

	if (pTAS2555->mnPowerCtrl == 1)
		tas2555_enable(pTAS2555, true);
	if (pTAS2555->mnPowerCtrl == 0)
		{
		tas2555_enable(pTAS2555, false);
		 Afe_Set_Reg(AUDIO_CLK_AUDDIV_0, 0x28, 0x28); //close MCLK 
		 dev_dbg(pTAS2555->dev, "tas2555 disable mclk\n");
		}
		
	return 0;
}

static int tas2555_fs_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);

	int nFS = 48000;

	if (pTAS2555->mpFirmware->mnConfigurations)
		nFS = pTAS2555->mpFirmware->mpConfigurations[pTAS2555->mnCurrentConfiguration].mnSamplingRate;
	
	pValue->value.integer.value[0] = nFS;
	
	dev_dbg(pTAS2555->dev, "tas2555_fs_get = %d\n", nFS);
	return 0;
}

static int tas2555_fs_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;
	int nFS = pValue->value.integer.value[0];
	
	dev_info(pTAS2555->dev, "tas2555_fs_put = %d\n", nFS);
	
	ret = tas2555_set_sampling_rate(pTAS2555, nFS);
	
	return ret;
}

static int tas2555_nRe_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	unsigned int nRe = 0;
	int ret = 0;

	if ((pTAS2555->mpFirmware->mnConfigurations > 0) && pTAS2555->mbPowerUp) {
		ret = tas2555_get_Re(pTAS2555, &nRe);
		if (ret >= 0)
			pValue->value.integer.value[0] = nRe;
		else
			pValue->value.integer.value[0] = 0;
	}

	dev_dbg(pTAS2555->dev, "tas2555_nRe_get = %d\n", nRe);
	return 0;
}

static int tas2555_nRe_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(pTAS2555->dev, "ignore tas2555_nRe_put\n");
	return 0;
}

static int tas2555_errcode_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	unsigned int errCode = 0;
	int nResult;

	nResult = tas2555_get_errcode(pTAS2555, &errCode);
	if (nResult >= 0)
		pValue->value.integer.value[0] = errCode;

	dev_dbg(pTAS2555->dev, "tas2555_errcode_get = 0x%x\n", errCode);
	return 0;
}

static int tas2555_errcode_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(pTAS2555->dev, "ignore tas2555_errcode_put\n");
	return 0;
}

static int tas2555_program_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	pValue->value.integer.value[0] = pTAS2555->mnCurrentProgram;
	dev_dbg(pTAS2555->dev, "tas2555_program_get = %d\n",
		pTAS2555->mnCurrentProgram);
	return 0;
}

static int tas2555_program_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	unsigned int nProgram = pValue->value.integer.value[0];
	int ret = 0;
	ret = tas2555_set_program(pTAS2555, nProgram, -1);
	return ret;
}

static int tas2555_configuration_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);

	pValue->value.integer.value[0] = pTAS2555->mnCurrentConfiguration;
	dev_dbg(pTAS2555->dev, "tas2555_configuration_get = %d\n",
		pTAS2555->mnCurrentConfiguration);
	return 0;
}

static int tas2555_configuration_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	unsigned int nConfiguration = pValue->value.integer.value[0];
	int ret = 0;

	ret = tas2555_set_config(pTAS2555, nConfiguration);
	return ret;
}

static int tas2555_calibration_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);

	pValue->value.integer.value[0] = pTAS2555->mnCurrentCalibration;
	dev_info(pTAS2555->dev,
		"tas2555_calibration_get = %d\n",
		pTAS2555->mnCurrentCalibration);
	return 0;
}

static int tas2555_calibration_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	unsigned int nCalibration = pValue->value.integer.value[0];
	int ret = 0;

	ret = tas2555_set_calibration(pTAS2555, nCalibration);

	return ret;
}
static int tas2555_fail_safe_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);

	pValue->value.integer.value[0] = ((pTAS2555->mnErrorCode & TAS2555_ERROR_FAILSAFE) != 0);
	dev_dbg(pTAS2555->dev, "tas2555_fail_safe_get = 0x%x\n",
		pTAS2555->mnErrorCode);

	return 0;
}

static int tas2555_fail_safe_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	int nFailsafe = pValue->value.integer.value[0];

	dev_dbg(pTAS2555->dev, "tas2555_fail_safe_put = %d\n",
		nFailsafe);

	if (nFailsafe == 1)
		failsafe(pTAS2555);

	return 0;
}

static int tas2555_DieTemp_DeltaT_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	int nResult = 0, nDeltaT;

	nResult = tas2555_get_die_delta_temperature(pTAS2555, &nDeltaT);
	if (nResult >= 0)
		pValue->value.integer.value[0] = nDeltaT & 0x3ff;

	dev_dbg(pTAS2555->dev, "tas2555_DieTemp_DeltaT_get = 0x%x\n",
		nDeltaT);

	return 0;
}

static int tas2555_DieTemp_DeltaT_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(pTAS2555->dev, "tas2555_DieTemp_DeltaT_put = bypass\n");

	return 0;
}

static int tas2555_nReDelta_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);

	pValue->value.integer.value[0] = pTAS2555->mnReDelta;

	dev_dbg(pTAS2555->dev, "tas2555_nReDelta_get = %d\n", pTAS2555->mnReDelta);
	return 0;
}

static int tas2555_nReDelta_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	unsigned int ReDelata = pValue->value.integer.value[0];

	pTAS2555->mnReDelta = ReDelata;

	return 0;
}

static int tas2555_nReHigh_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);

	pValue->value.integer.value[0] = pTAS2555->mnReHigh;

	dev_dbg(pTAS2555->dev, "tas2555_nReHigh_get = %d\n", pTAS2555->mnReHigh);
	return 0;
}

static int tas2555_nReHigh_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	unsigned int ReHigh = pValue->value.integer.value[0];

	pTAS2555->mnReHigh = ReHigh;
	dev_dbg(pTAS2555->dev, "tas2555_nReHigh_put = %d\n", pTAS2555->mnReHigh);

	return 0;
}

static int tas2555_nReLow_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);

	pValue->value.integer.value[0] = pTAS2555->mnReLow;

	dev_dbg(pTAS2555->dev, "tas2555_nReLow_get = %d\n", pTAS2555->mnReLow);
	return 0;
}

static int tas2555_nReLow_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	unsigned int ReLow = pValue->value.integer.value[0];

	pTAS2555->mnReLow = ReLow;
	dev_dbg(pTAS2555->dev, "tas2555_nReLow_put = %d\n", pTAS2555->mnReLow);

	return 0;
}

static int tas2555_fw_load_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	int nResult = 0;

	if ((pTAS2555->mpFirmware->mnConfigurations > 0)
		&& (pTAS2555->mpFirmware->mnPrograms > 0))
		nResult = 1;

	pValue->value.integer.value[0] = nResult;

	dev_dbg(pTAS2555->dev, "%s = 0x%x\n", __func__, nResult);

	return 0;
}

static int tas2555_fw_load_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	int nReload = pValue->value.integer.value[0];

	if (nReload)
		request_firmware_nowait(THIS_MODULE, 1, TAS2555_FW_NAME,
			pTAS2555->dev, GFP_KERNEL, pTAS2555, tas2555_fw_ready);

	dev_dbg(pTAS2555->dev, "%s, %d\n", __func__, nReload);

	return 0;
}

/*
 * DAC digital volumes. From 0 to 15 dB in 1 dB steps
 */
static DECLARE_TLV_DB_SCALE(dac_tlv, 0, 100, 0);

static const struct snd_kcontrol_new tas2555_snd_controls[] = {
	SOC_SINGLE_TLV("DAC Playback Volume", TAS2555_SPK_CTRL_REG, 3, 0x0f, 0,
		dac_tlv),
	SOC_SINGLE_EXT("Reg Addr", SND_SOC_NOPM, 0, 0x7fffffff, 0,
		tas2555_get_reg_addr, tas2555_put_reg_addr),
	SOC_SINGLE_EXT("Reg Value", SND_SOC_NOPM, 0, 0xFFFF, 0,
		tas2555_get_reg_value, tas2555_put_reg_value),
	SOC_SINGLE_EXT("PowerCtrl", SND_SOC_NOPM, 0, 0x0001, 0,
		tas2555_power_ctrl_get, tas2555_power_ctrl_put),
	SOC_SINGLE_EXT("Program", SND_SOC_NOPM, 0, 0x00FF, 0, tas2555_program_get,
		tas2555_program_put),
	SOC_SINGLE_EXT("Configuration", SND_SOC_NOPM, 0, 0x00FF, 0,
		tas2555_configuration_get, tas2555_configuration_put),
	SOC_SINGLE_EXT("TAS_FWLoad", SND_SOC_NOPM, 0, 0x0001, 0,
		tas2555_fw_load_get, tas2555_fw_load_put),
	SOC_SINGLE_EXT("FS", SND_SOC_NOPM, 8000, 48000, 0,
		tas2555_fs_get, tas2555_fs_put),
	SOC_SINGLE_EXT("nRe_Delta", SND_SOC_NOPM, 0, 0x7fffffff, 0,
		tas2555_nReDelta_get, tas2555_nReDelta_put),
	SOC_SINGLE_EXT("nRe_High", SND_SOC_NOPM, 0, 0x7fffffff, 0,
		tas2555_nReHigh_get, tas2555_nReHigh_put),
	SOC_SINGLE_EXT("nRe_Low", SND_SOC_NOPM, 0, 0x7fffffff, 0,
		tas2555_nReLow_get, tas2555_nReLow_put),		
	SOC_SINGLE_EXT("FailSafe", SND_SOC_NOPM, 0, 0x0001, 0,
	    tas2555_fail_safe_get, tas2555_fail_safe_put),
	SOC_SINGLE_EXT("nRe", SND_SOC_NOPM, 0, 0x7fffffff, 0,
		tas2555_nRe_get, tas2555_nRe_put),
	SOC_SINGLE_EXT("TAS_Status", SND_SOC_NOPM, 0, 0x7fffffff, 0,
		tas2555_errcode_get, tas2555_errcode_put),
	SOC_SINGLE_EXT("Calibration", SND_SOC_NOPM, 0, 0x00FF, 0,
		tas2555_calibration_get, tas2555_calibration_put),
	SOC_SINGLE_EXT("DieTempDeltaT", SND_SOC_NOPM, 0, 0x03ff, 0,
		tas2555_DieTemp_DeltaT_get, tas2555_DieTemp_DeltaT_put),		

};

static struct snd_soc_codec_driver soc_codec_driver_tas2555 = {
	.probe = tas2555_codec_probe,
	.remove = tas2555_codec_remove,
	.read = tas2555_read,
	.write = tas2555_write,
	.set_bias_level = tas2555_set_bias_level,
	.idle_bias_off = true,
//	.ignore_pmdown_time = true,

	.controls = tas2555_snd_controls,
	.num_controls = ARRAY_SIZE(tas2555_snd_controls),
	.dapm_widgets = tas2555_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tas2555_dapm_widgets),
	.dapm_routes = tas2555_audio_map,
	.num_dapm_routes = ARRAY_SIZE(tas2555_audio_map),
};

static struct snd_soc_dai_ops tas2555_dai_ops = {
#ifdef MTK_PLATFORM_DRIVER
	.startup = mt63xx_codec_startup,	//add mtk
#else
	.startup = tas2555_startup,
#endif
	.digital_mute = tas2555_mute,
	.shutdown = tas2555_shutdown,
	.hw_params = tas2555_hw_params,
	.prepare = tas2555_prepare,
	.set_sysclk = tas2555_set_dai_sysclk,
	.set_fmt = tas2555_set_dai_fmt,
};

#define TAS2555_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
             SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)
static struct snd_soc_dai_driver tas2555_dai_driver[] = {
	{
			.name = "tas2555 ASI1",
			.id = 0,
			.playback = {
					.stream_name = "ASI1 Playback",
					.channels_min = 2,
					.channels_max = 2,
					.rates = SNDRV_PCM_RATE_8000_192000,
					.formats = TAS2555_FORMATS,
		},
		.capture = { 
                  .stream_name ="ASI1 capture", 
                  .channels_min = 1, 
                  .channels_max = 2, 
                  .rates = SNDRV_PCM_RATE_8000_48000, 
                  .formats = SND_SOC_ADV_MT_FMTS, 
				},
			.ops = &tas2555_dai_ops,
			.symmetric_rates = 1,
		},
	{
			.name = "tas2555 ASI2",
			.id = 1,
			.playback = {
					.stream_name = "ASI2 Playback",
					.channels_min = 2,
					.channels_max = 2,
					.rates = SNDRV_PCM_RATE_8000_192000,
					.formats = TAS2555_FORMATS,
				},
			.ops = &tas2555_dai_ops,
			.symmetric_rates = 1,
		},
	{
			.name = "tas2555 ASIM",
			.id = 2,
			.playback = {
					.stream_name = "ASIM Playback",
					.channels_min = 2,
					.channels_max = 2,
					.rates = SNDRV_PCM_RATE_8000_192000,
					.formats = TAS2555_FORMATS,
				},
			.ops = &tas2555_dai_ops,
			.symmetric_rates = 1,
		},
};


static void tas2555_hw_reset(struct tas2555_priv *pTAS2555)
{
	/* Hardware Reset the chip */    
     mt_set_gpio_mode(GPIO_TAS2555_RST_PIN,GPIO_MODE_00);      
     mt_set_gpio_dir(GPIO_TAS2555_RST_PIN,GPIO_DIR_OUT);    
     mt_set_gpio_out(GPIO_TAS2555_RST_PIN,GPIO_OUT_ONE);    
     msleep(1);	
     mt_set_gpio_out(GPIO_TAS2555_RST_PIN, GPIO_OUT_ZERO);	
     msleep(10);	
     mt_set_gpio_out(GPIO_TAS2555_RST_PIN,GPIO_OUT_ONE);
     msleep(1);	
}

int tas2555_enableIRQ(struct tas2555_priv *pTAS2555, bool enable, bool clear)
{	
	unsigned int nValue;	
	int nResult = 0;	
	if (enable) {		
		if (clear) {			
			nResult = pTAS2555->read(pTAS2555, TAS2555_FLAGS_1, &nValue);
			if (nResult < 0)				
				goto end;			
			nResult = pTAS2555->read(pTAS2555, TAS2555_FLAGS_2, &nValue);
		}		
		if (!pTAS2555->mbIRQEnable) {			
			if (pTAS2555->mnIRQ != 0)				
				enable_irq(pTAS2555->mnIRQ);			
			pTAS2555->mbIRQEnable = true;		
		}	
	} else {		
		if (pTAS2555->mbIRQEnable) {			
			if (pTAS2555->mnIRQ != 0)				
				disable_irq_nosync(pTAS2555->mnIRQ);			
			pTAS2555->mbIRQEnable = false;		
		}		
		if (clear) {			
			nResult = pTAS2555->read(pTAS2555, TAS2555_FLAGS_1, &nValue);
			if (nResult < 0)				
				goto end;			
			nResult = pTAS2555->read(pTAS2555, TAS2555_FLAGS_2, &nValue);
		}	
	}
end:	
	return nResult;
}

static void irq_work_routine(struct work_struct *work)
{	
	int nResult = 0;	
	unsigned int nDevInt1Status = 0, nDevInt2Status = 0;	
	struct tas2555_priv *pTAS2555 =	container_of(work, struct tas2555_priv, irq_work.work);	
	if (!pTAS2555->mbPowerUp)		
		return;	
	nResult = tas2555_dev_read(pTAS2555, TAS2555_FLAGS_1, &nDevInt1Status);
	if (nResult < 0)		
		dev_err(pTAS2555->dev, "I2C doesn't work\n");	
	else		
		nResult = tas2555_dev_read(pTAS2555, TAS2555_FLAGS_2, &nDevInt2Status);	
	if ((nDevInt1Status & 0xdc) != 0) {		
		/* in case of INT_OC, INT_UV, INT_OT, INT_BO, INT_CL, INT_CLK1, INT_CLK2 */		
		dev_err(pTAS2555->dev, "critical error INT Status: 0x%x\n", nDevInt1Status);
		if (nDevInt1Status & 0x04) {			
			pTAS2555->mnErrorCode |= TAS2555_ERROR_CLKPRESENT;			
			nResult = pTAS2555->write(pTAS2555, TAS2555_CLK_ERR_CTRL1, 0x00);
		}		
		if (nDevInt1Status & 0x08)			
			pTAS2555->mnErrorCode |= TAS2555_ERROR_BROWNOUT;		
		if (nDevInt1Status & 0x10)			
			pTAS2555->mnErrorCode |= TAS2555_ERROR_OVERTMP;		
		if (nDevInt1Status & 0x40)			
			pTAS2555->mnErrorCode |= TAS2555_ERROR_UNDERVOLTAGET;		
		if (nDevInt1Status & 0x80)			
			pTAS2555->mnErrorCode |= TAS2555_ERROR_OVERCURRENT;		
		goto program;	
	} else
		dev_dbg(pTAS2555->dev, "%s, INT Status: 0x%x\n", __func__, nDevInt1Status);	
	return;
program:	
	/* hardware reset and reload */	
	tas2555_hw_reset(pTAS2555);	
	tas2555_set_program(pTAS2555, pTAS2555->mnCurrentProgram, pTAS2555->mnCurrentConfiguration);
}

static irqreturn_t tas2555_irq_handler(int irq, void *dev_id)
{	
	struct tas2555_priv *pTAS2555 = (struct tas2555_priv *)dev_id;	
	tas2555_enableIRQ(pTAS2555, false, false);	
	/* get IRQ status after 100 ms */	
	schedule_delayed_work(&pTAS2555->irq_work, msecs_to_jiffies(100));	
	return IRQ_HANDLED;
}

int tas2555_parse_dt(struct device *dev, struct tas2555_priv *pTAS2555)
{
	struct device_node *np = dev->of_node;
	int ret = 0;

	pTAS2555->mnResetGPIO = of_get_named_gpio(np, "ti,cdc-reset-gpio", 0);
	if (pTAS2555->mnResetGPIO < 0) {
		dev_err(pTAS2555->dev, "Looking up %s property in node %s failed %d\n",
			"ti,cdc-reset-gpio", np->full_name, pTAS2555->mnResetGPIO);
		ret = -EINVAL;
		pTAS2555->mnErrorCode |= TAS2555_ERROR_INVALIDPARAM;
	} else
		dev_dbg(pTAS2555->dev, "ti,cdc-reset-gpio=%d\n", pTAS2555->mnResetGPIO);

	if (ret >= 0) {
		pTAS2555->mnGpioINT = of_get_named_gpio(np, "ti,irq-gpio", 0);
		if (pTAS2555->mnGpioINT < 0) {
			dev_err(pTAS2555->dev, "Looking up %s property in node %s failed %d\n",
				"ti,irq-gpio", np->full_name, pTAS2555->mnGpioINT);
			ret = -EINVAL;
			pTAS2555->mnErrorCode |= TAS2555_ERROR_INVALIDPARAM;
		} else
			dev_dbg(pTAS2555->dev, "ti,irq-gpio=%d\n", pTAS2555->mnGpioINT);
	}

	return ret;
}

static int tas2555_i2c_probe(struct i2c_client *pClient,
	const struct i2c_device_id *pID)
{
	struct tas2555_priv *pTAS2555;
	unsigned int n;
	int nResult;

	g_client = pClient;

	printk(KERN_ERR "tas2555_i2c_probe\n\r");

#if 1         //dma 
	pClient->dev.coherent_dma_mask = DMA_BIT_MASK(32);
    gpDMABuf_va = (u8 *)dma_alloc_coherent(&pClient->dev, GTP_DMA_MAX_TRANSACTION_LENGTH, &gpDMABuf_pa, GFP_KERNEL);

    if(!gpDMABuf_va){
		dev_err(&pClient->dev, "%s, line %d, no memory for tas2555\n", __FUNCTION__, __LINE__);
    }
    memset(gpDMABuf_va, 0, GTP_DMA_MAX_TRANSACTION_LENGTH);
#endif


//  regmap_config = &tas2555_i2c_regmap;

	printk(KERN_ERR "tas2555_i2c_probe: alloc pTAS2555\n\r");
	pTAS2555 = devm_kzalloc(&pClient->dev, sizeof(*pTAS2555), GFP_KERNEL);
	if (!pTAS2555)
		return -ENOMEM;

	printk(KERN_ERR "tas2555_i2c_probe: set pTAS2555 dev and client\n\r");
	pTAS2555->dev = &pClient->dev;
	pTAS2555->client = pClient;
	i2c_set_clientdata(pClient, pTAS2555);

	dev_set_drvdata(&pClient->dev, pTAS2555);

	pTAS2555->client = pClient;

	/* Reset the chip */
	if (pClient->dev.of_node)		
		tas2555_parse_dt(&pClient->dev, pTAS2555);	
	
	pTAS2555->read = tas2555_dev_read;
	pTAS2555->write = tas2555_dev_write;
	pTAS2555->bulk_read = tas2555_dev_bulk_read;
	pTAS2555->bulk_write = tas2555_dev_bulk_write;
	pTAS2555->update_bits = tas2555_dev_update_bits;
	pTAS2555->enableIRQ = tas2555_enableIRQ;	
	pTAS2555->hw_reset = tas2555_hw_reset;	
	pTAS2555->set_config = tas2555_set_config;
	pTAS2555->set_calibration = tas2555_set_calibration;
	mutex_init(&pTAS2555->dev_lock);

	tas2555_hw_reset(pTAS2555);
	
	tas2555_dev_write(pTAS2555, TAS2555_SW_RESET_REG, 0x01);
	udelay(1000);

	printk(KERN_ERR "tas2555_i2c_probe: alloc mpFirmware\n\r");
	pTAS2555->mpFirmware =
		devm_kzalloc(&pClient->dev, sizeof(*(pTAS2555->mpFirmware)),
		GFP_KERNEL);
	if (!pTAS2555->mpFirmware)
		return -ENOMEM;
	pTAS2555->mpCalFirmware =
		devm_kzalloc(&pClient->dev, sizeof(*(pTAS2555->mpCalFirmware)),
		GFP_KERNEL);
	if (!pTAS2555->mpCalFirmware)
		return -ENOMEM;

	printk(KERN_ERR "tas2555_i2c_probe: read PG dev ID\n\r");
	nResult = tas2555_dev_read(pTAS2555, TAS2555_REV_PGID_REG, &n);
	dev_err(&pClient->dev, "TAS2555 PGID: 0x%02x\n", n);

	if (gpio_is_valid(pTAS2555->mnGpioINT)) {		
		nResult = gpio_request(pTAS2555->mnGpioINT, "TAS2555-IRQ");		
		if (nResult < 0) {			
			dev_err(pTAS2555->dev, "%s: GPIO %d request INT error\n",				
				__func__, pTAS2555->mnGpioINT);			
			goto fail;		
		}		
		gpio_direction_input(pTAS2555->mnGpioINT);		
		pTAS2555->mnIRQ = gpio_to_irq(pTAS2555->mnGpioINT);		
		dev_dbg(pTAS2555->dev, "irq = %d\n", pTAS2555->mnIRQ);		
		nResult = request_threaded_irq(pTAS2555->mnIRQ, tas2555_irq_handler,
			NULL, IRQF_TRIGGER_RISING | IRQF_ONESHOT,				
			pClient->name, pTAS2555);		
		if (nResult < 0) {			
			dev_err(pTAS2555->dev,				
				"request_irq failed, %d\n", nResult);			
			goto fail;		
		}		
		INIT_DELAYED_WORK(&pTAS2555->irq_work, irq_work_routine);	
	}

	nResult = request_firmware_nowait(THIS_MODULE, 1, TAS2555_FW_NAME,
		&pClient->dev, GFP_KERNEL, pTAS2555, tas2555_fw_ready);

	pTAS2555->mbTILoadActive = false;

	nResult = snd_soc_register_codec(&pClient->dev, &soc_codec_driver_tas2555,
		tas2555_dai_driver, ARRAY_SIZE(tas2555_dai_driver));

#ifdef ENABLE_TILOAD

	tiload_driver_init(pTAS2555);
#endif

fail:

	return nResult;
}

static int tas2555_i2c_remove(struct i2c_client *pClient)
{

    struct tas2555_priv *pTAS2555 = i2c_get_clientdata(pClient);

	snd_soc_unregister_codec(pTAS2555->dev);		

	return 0;

}

static const struct i2c_device_id tas2555_i2c_id[] = {
	{"tas2555", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, tas2555_i2c_id);

#if defined(CONFIG_OF)
static const struct of_device_id tas2555_of_match[] = {
#ifdef MTK_PLATFORM_DRIVER
	{.compatible = "mediatek,EXT_SPEAKER_AMP"},
#else
	{.compatible = "ti,tas2555"},
#endif
	{},
};

MODULE_DEVICE_TABLE(of, tas2555_of_match);
#endif

static struct i2c_driver tas2555_i2c_driver = {
	.driver = {
			.name = "tas2555",
			.owner = THIS_MODULE,
#if defined(CONFIG_OF)
			.of_match_table = of_match_ptr(tas2555_of_match),
#endif
		},
	.probe = tas2555_i2c_probe,
	.remove = tas2555_i2c_remove,
	.id_table = tas2555_i2c_id,
};

module_i2c_driver(tas2555_i2c_driver);

MODULE_AUTHOR("Peter Ujfalusi <peter.ujfalusi@ti.com>");
MODULE_DESCRIPTION("TAS2555 Smart Amplifier driver");
MODULE_LICENSE("GPLv2");
