#ifndef OTM1906A_FHD_DSI_VDO_6INCH_TRULY_C
#define OTM1906A_FHD_DSI_VDO_6INCH_TRULY_C

#if !defined( BUILD_LK )
    #include <linux/string.h>
    #include <linux/kernel.h>
#endif
    #include "lcm_drv.h"

#if defined( BUILD_LK )
    #include  <platform/upmu_common.h>
    #include  <platform/mt_gpio.h>
    #include  <platform/mt_pmic.h>
    #include  <string.h>
#elif defined( BUILD_UBOOT )
    #include  <asm/arch/mt_gpio.h>
#else
//    #include  <mach/mt_pm_ldo.h>
//    #include  <mach/mt_gpio.h>
#endif
//    #include <cust_gpio_usage.h>

#if !defined( TRUE )
    #define   TRUE      1
#endif

#if !defined( FALSE )
    #define   FALSE     0
#endif

#if defined(BUILD_LK)
#if !defined( GPIO_LCM_ID )
    #define   GPIO_LCM_ID               (GPIO55|0x80000000)
#endif

#if !defined( GPIO_LCM_ID_M_GPIO )
    #define   GPIO_LCM_ID_M_GPIO        GPIO_MODE_00
#endif
#endif

    #define   LCM_TRACE_ENABLE_N
    #define   LCM_DEBUG_ENABLE_N

#if defined( LCMDRV_TRACE_ENABLE )
  #if defined( BUILD_LK )
    #define   LCM_MSG(srt,arg...)       dprintf(CRITICAL,srt,##arg)
  #else
    #define   LCM_MSG(srt,arg...)       printk(KERN_INFO srt,##arg)
  #endif
#else
    #define   LCM_MSG(srt,arg...)       {/* Do Nothing */}
#endif

#if defined( LCMDRV_DEBUG_ENABLE )
  #if defined( BUILD_LK )
    #define   LCM_DBG(srt,arg...)       dprintf(CRITICAL,srt,##arg)
  #else
    #define   LCM_DBG(srt,arg...)       printk(KERN_DEBUG srt,##arg)
  #endif
#else
    #define   LCM_DBG(srt,arg...)       {/* Do Nothing */}
#endif

    #define   GP0550IA00260_AM31

/*****************************************************************************
** Local Functions
******************************************************************************/
    static const unsigned int BL_MIN_LEVEL = 20;
    static LCM_UTIL_FUNCS   lcm_util = { 0 };

    #define   SET_RESET_PIN(v)          (lcm_util.set_reset_pin((v)))
    #define   MDELAY(n)                 (lcm_util.mdelay(n))
    #define   UDELAY(n)                 (lcm_util.udelay(n))

    #define   dsi_set_cmdq(pdata, queue_size, force_update)     \
                                        lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
    #define   dsi_set_cmdq_V2(cmd, count, ppara, force_update)  \
                                        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
    #define   wrtie_cmd(cmd)            lcm_util.dsi_write_cmd(cmd)
    #define   write_regs(addr, pdata, byte_nums)    \
                                        lcm_util.dsi_write_regs(addr, pdata, byte_nums)
    #define   read_reg(cmd)             lcm_util.dsi_dcs_read_lcm_reg(cmd)
    #define   read_reg_v2(cmd, buffer, buffer_size) \
                                        lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

//<2015/11/18-stevenchen, Fix lcm gate driver of kernel to resolve system hang issue.
    #define   set_gpio_lcd_enp(cmd)     lcm_util.set_gpio_lcd_enp_bias(cmd)
    #define   set_gpio_lcd_enn(cmd)     lcm_util.set_gpio_lcd_enn_bias(cmd)
//>2015/11/18-stevenchen
/*****************************************************************************
** Local Constants
******************************************************************************/
    #define   LCM_ESD_RECOVERY          1   /* 1: Enable , 0: Disable */

    #define   ROLLBACK_ESD_PATCH        1

#if( ROLLBACK_ESD_PATCH )	
    #define   LCM_DSI_CMD_MODE          1
#else
    #define   LCM_DSI_CMD_MODE          0
#endif	

    #define   ENABLE_CABC_FUNCTION      1  // 1:Enable , 0:Disable

    #define   LCM_OTM1906A_ID           (0x40)

#if defined( FPGA_EARLY_PORTING )
    #define   FRAME_WIDTH               (480)
    #define   FRAME_HEIGHT              (800)
#else
    #define   FRAME_WIDTH               (1080)
    #define   FRAME_HEIGHT              (1920)
#endif

    #define   LCM_HSYNC_NUM             40 
    #define   LCM_HBP_NUM               60
    #define   LCM_HFP_NUM               80

    #define   LCM_VSYNC_NUM             2 
    #define   LCM_VBP_NUM               9 
    #define   LCM_VFP_NUM               3

    #define   REGFLAG_UDELAY            (0xFB)  /* RELOAD CMD1 */
    #define   REGFLAG_DELAY             (0xFC)
    #define   REGFLAG_END_OF_TABLE      (0xFD)  /* END OF REGISTERS MARKER */

  //#define   REGFLAG_RESET_LOW         (0xFE)  /* RD_CMDSTATUS: Read the Current Register Set */
  //#define   REGFLAG_RESET_HIGH        (0xFF)  /* CMD Page Select */

#if defined( BUILD_LK )
  #if 0
    extern void DSI_clk_HS_mode(unsigned char enter);
  #endif
#endif

/*****************************************************************************
** Local Varibble
******************************************************************************/
    struct LCM_setting_table
    {
        unsigned char   cmd;
        unsigned char   count;
        unsigned char   para_list[64];
    };

    LCM_DSI_MODE_SWITCH_CMD     lcm_switch_mode_cmd_truly;

//<2015/11/18-stevenchen, Fix lcm gate driver of kernel to resolve system hang issue.
/*****************************************************************************
** Gate driver function
******************************************************************************/
static void lcm_gate_enable(int enable)
{
    printk("[Steven] %s = %d \n", __func__, enable);
    if( TRUE == enable )
    {
        set_gpio_lcd_enp(1); //Bias AVDD Enable (GPIO12)
        MDELAY( 1 );
        set_gpio_lcd_enn(1); //Bias AVEE Enable (GPIO17)
    }
    else
    {
        set_gpio_lcd_enn(0);
        MDELAY( 1 );
        set_gpio_lcd_enp(0);
    }
}
//>2015/11/18-stevenchen

  /*==========================================================
  **
  **==========================================================*/
   //[Arima_Lavender][RaymondLin] Modify LCM suspend time 20150330 begin
  static struct LCM_setting_table lcm_suspend_setting[] =
  {
  /* Display OFF */
    { 0x28, 0, {}},

  /* Sleep In */
    { 0x10, 0, {}},
    { REGFLAG_DELAY, 120, {}},
  /* End */
    { REGFLAG_END_OF_TABLE, 0x00, {}}
  };
  //[Arima_Lavender][RaymondLin] Modify LCM suspend time 20150330 end

//<2015/12/16-stevenchen, Update parameters to reduce LCM noise
  /* update initial param for IC nt35520 0.01 */
  static struct LCM_setting_table lcm_initialization_setting[] =
  {
   
  { 0x00, 1, { 0x00}},  
  { 0xFF, 3, { 0x19,0x06,0x01}}, //Enable Access Command 2 & Software EXTC Enable
  { REGFLAG_DELAY, 1, {}}, 
  { 0x00, 1, { 0x80}},
  { 0xFF, 2, { 0x19,0x06}},  //Enable Access Orise Command 2
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0x80}}, 
  { 0xA4, 7, { 0x00,0x00,0x00,0x02,0x00,0x82,0x00}}, 
  { REGFLAG_DELAY, 1, {}}, 

  { 0x00, 1, { 0x80}}, 
  { 0xA5, 4, { 0x0f,0x00,0x01,0x08}}, 
  { REGFLAG_DELAY, 1, {}}, 

  { 0x00, 1, { 0x80}}, 
  { 0xC0, 14, { 0x00,0x80,0x00,0x04,0x06,0x00,0x80,0x04,0x06,0x00,0x80,0x00,0x04,0x06}}, 
  { REGFLAG_DELAY, 1, {}}, 

  { 0x00, 1, { 0xA0}}, 
  { 0xC0, 7, { 0x00,0x05,0x00,0x05,0x02,0x1b,0x04}}, 
  { REGFLAG_DELAY, 1, {}}, 
	
  { 0x00, 1, { 0xD0}}, 
  { 0xC0, 7, { 0x00,0x05,0x00,0x05,0x02,0x1b,0x04}}, 
  { REGFLAG_DELAY, 1, {}}, 

  { 0x00, 1, { 0x80}}, 
  { 0xC1, 2, { 0x11,0x11}}, 
  { REGFLAG_DELAY, 1, {}},
	
  { 0x00, 1, { 0x80}}, 
  { 0xC2, 12, { 0x84,0x01,0x40,0x40,0x82,0x01,0x17,0x18,0x00,0x00,0x00,0x00}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0x90}},
  { 0xC2, 12, { 0x00,0x00,0x00,0x00,0x02,0x01,0x08,0x0f,0x02,0x01,0x08,0x0f}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0xB0}},
  { 0xC2, 15, { 0x84,0x01,0x00,0x08,0x00,0x82,0x03,0x00,0x08,0x00,0x83,0x02,0x00,0x08,0x00}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0xC0}},
  { 0xC2, 15, { 0x81,0x04,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0xD0}},
  { 0xC2, 15, { 0x82,0x00,0x01,0x02,0x81,0x84,0x02,0x03,0x00,0x01,0x33,0x33,0x00,0x37,0x00}}, 
  { REGFLAG_DELAY, 1, {}},

//<2016/04/29-stevenchen, Update Truly LCM initial code
//Truly EMI test
  { 0x00, 1, { 0x80}},
  { 0xC3, 15, { 0x00,0x00,0x00,0x00,0x02,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}}, 
  { REGFLAG_DELAY, 1, {}},
//>2016/04/29-stevenchen

  { 0x00, 1, { 0xA0}},
  { 0xC3, 12, { 0x84,0x01,0x40,0x40,0x82,0x01,0x17,0x18,0x00,0x00,0x00,0x00}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0xB0}},
  { 0xC3, 14, { 0x00,0x00,0x00,0x00,0x84,0x01,0x00,0x08,0x00,0x82,0x03,0x00,0x08,0x00}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0xC0}},
  { 0xC3, 15, { 0x83,0x02,0x00,0x08,0x00,0x81,0x04,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0xE0}},
  { 0xC3, 4, { 0x33,0x33,0x00,0x37}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0x80}},
  { 0xCB, 11, { 0x00,0x00,0x00,0x00,0x30,0x00,0x00,0x00,0x00,0x00,0x00}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0x90}},
  { 0xCB, 15, { 0x00,0x00,0x00,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0xA0}},
  { 0xCB, 15, { 0x00,0x00,0x00,0xBF,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0xB0}},
  { 0xCB, 12, { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x77,0x77,0x00,0x00}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0xC0}},
  { 0xCB, 15, { 0x15,0x11,0x15,0x15,0x15,0x15,0x15,0x15,0x01,0x01,0x01,0x01,0x01,0x01,0x01}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0xD0}},
  { 0xCB, 15, { 0x01,0x3D,0x01,0xFF,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x11,0x01,0x11}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0xE0}},
  { 0xCB, 12, { 0x01,0x11,0x00,0x01,0x00,0x01,0x00,0x01,0x77,0x77,0x00,0x00}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0xF0}},
  { 0xCB, 11, { 0xF3,0xFF,0x55,0x55,0x7F,0xC0,0x03,0x33,0x03,0x00,0x02}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0x80}},
  { 0xCC, 12, { 0x08,0x09,0x18,0x19,0x0c,0x0d,0x0e,0x0f,0x07,0x07,0x07,0x07}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0x90}},
  { 0xCC, 12, { 0x18,0x09,0x08,0x19,0x0f,0x0e,0x0d,0x0c,0x07,0x07,0x07,0x07}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0xA0}},
  { 0xCC, 15, { 0x14,0x15,0x16,0x17,0x1C,0x1D,0x1E,0x1F,0x20,0x07,0x07,0x07,0x07,0x07,0x00}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0xB0}},
  { 0xCC, 8, { 0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x07}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0xC0}},
  { 0xCC, 15, { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x45,0x45,0x00}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0xD0}},
  { 0xCC, 15, { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x33,0x33,0x33,0x33}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0xE0}},
  { 0xCC, 12, { 0x33,0x33,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0x80}},
  { 0xCD, 15, { 0x01,0x11,0x12,0x07,0x08,0x04,0x18,0x1A,0x03,0x09,0x23,0x23,0x23,0x1D,0x1E}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0x90}},
  { 0xCD, 3, { 0x1F,0x23,0x23}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0xA0}},
  { 0xCD, 15, { 0x01,0x11,0x12,0x05,0x06,0x04,0x18,0x17,0x03,0x09,0x23,0x23,0x23,0x1D,0x1E}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0xB0}},
  { 0xCD, 3, { 0x1F,0x23,0x23}}, 
  { REGFLAG_DELAY, 1, {}},

  //HV Setting
  { 0x00, 1, { 0x91}},
  { 0xC5, 2, { 0x14,0x1e}}, 
  { REGFLAG_DELAY, 1, {}},
  { 0x00, 1, { 0xA1}},
  { 0xC5, 2, { 0x14,0x1e}}, 
  { REGFLAG_DELAY, 1, {}},

//<2016/06/17-stevenchen, Update initial code by vendor suggestion
  //VDD18
  { 0x00, 1, { 0xC1}},
  { 0xC5, 1, { 0xB5}}, 
  { REGFLAG_DELAY, 1, {}},
//>2016/06/17-stevenchen

#if (LCM_DSI_CMD_MODE)
  //VCOM Setting
  { 0x00, 1, { 0x00}},
  { 0xD9, 8, { 0x00,0xBD,0x00,0xBD,0x00,0x90,0x00,0x90}}, 
  { REGFLAG_DELAY, 1, {}},
#endif

//<2016/06/27-stevenchen, Low down GVDD to fix press trace will not disappear on white screen
  //GVDDP/N
  { 0x00, 1, { 0x00}},
  { 0xD8, 2, { 0x14,0x14}}, 
  { REGFLAG_DELAY, 1, {}},
//>2016/06/27-stevenchen

  //GAMMA2.2 20151211
  { 0x00, 1, { 0x00}},
  { 0xE1, 24, { 0x00,0x2c,0x34,0x3f,0x47,0x4f,0x5a,0x6a,0x74,0x83,0x8c,0x94,0x67,0x62,0x5e,0x52,0x43,0x32,0x26,0x20,0x17,0x0b,0x03,0x03}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0x00}},
  { 0xE2, 24, { 0x00,0x2c,0x34,0x3f,0x47,0x4f,0x5a,0x6a,0x74,0x83,0x8c,0x94,0x67,0x62,0x5e,0x52,0x43,0x32,0x26,0x20,0x17,0x0b,0x03,0x03}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0x00}},
  { 0xE3, 24, { 0x00,0x2c,0x34,0x3f,0x47,0x4f,0x5a,0x6a,0x74,0x83,0x8c,0x94,0x67,0x62,0x5e,0x52,0x43,0x32,0x26,0x20,0x17,0x0b,0x03,0x03}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0x00}},
  { 0xE4, 24, { 0x00,0x2c,0x34,0x3f,0x47,0x4f,0x5a,0x6a,0x74,0x83,0x8c,0x94,0x67,0x62,0x5e,0x52,0x43,0x32,0x26,0x20,0x17,0x0b,0x03,0x03}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0x00}},
  { 0xE5, 24, { 0x00,0x2c,0x34,0x3f,0x47,0x4f,0x5a,0x6a,0x74,0x83,0x8c,0x94,0x67,0x62,0x5e,0x52,0x43,0x32,0x26,0x20,0x17,0x0b,0x03,0x03}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0x00}},
  { 0xE6, 24, { 0x00,0x2c,0x34,0x3f,0x47,0x4f,0x5a,0x6a,0x74,0x83,0x8c,0x94,0x67,0x62,0x5e,0x52,0x43,0x32,0x26,0x20,0x17,0x0b,0x03,0x03}}, 
  { REGFLAG_DELAY, 1, {}},

  //inversion
  { 0x00, 1, { 0xB3}},
  { 0xC0, 1, { 0x88}}, 
  { REGFLAG_DELAY, 1, {}},
  { 0x00, 1, { 0xA0}},
  { 0xB3, 7, { 0x33,0x04,0x38,0x07,0x80,0x00,0x20}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0xE1}},
  { 0xF5, 1, { 0x16}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0x81}},
  { 0xF5, 1, { 0x16}}, 
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0x83}},
  { 0xF5, 1, { 0x16}}, 
  { REGFLAG_DELAY, 1, {}},

//<2016/06/07-stevenchen, Update Truly LCM initial code
//<2016/06/02-stevenchen, Fix flicker issue
//<2016/04/29-stevenchen, Update Truly LCM initial code
//Truly EMI test
  { 0x00, 1, { 0x80}},
  { 0xC4, 1, { 0x61}}, 	
  { REGFLAG_DELAY, 1, {}},

  { 0x00, 1, { 0x90}},
  { 0xE9, 1, { 0x11}}, 	
  { REGFLAG_DELAY, 1, {}},
//>2016/04/29-stevenchen
//>2016/06/02-stevenchen
//>2016/06/07-stevenchen

  { 0x00, 1, { 0x81}},
  { 0xA5, 1, { 0x01}}, 
  { REGFLAG_DELAY, 1, {}},

//<2016/06/17-stevenchen, Update initial code by vendor suggestion
  { 0x00, 1, { 0xA0}},
  { 0xC1, 3, { 0x00,0xcc,0x11}}, 
  { REGFLAG_DELAY, 1, {}},
//>2016/06/17-stevenchen

//<2016/04/29-stevenchen, Update Truly LCM initial code
//For Panel noise optimal  2016-04-13
  { 0x00, 1, { 0xB0}},
  { 0xC5, 4, { 0x00,0x00,0x01,0x00}}, 
  { REGFLAG_DELAY, 1, {}},
//>2016/04/29-stevenchen

//FOR video mode use
#if(LCM_DSI_CMD_MODE)
#else
//<2016/06/17-stevenchen, Update initial code by vendor suggestion
  { 0x00, 1, { 0x00}},		
  { 0x1C, 1, { 0x33}},	  
  { REGFLAG_DELAY, 1, {}},
//>2016/06/17-stevenchen
  
  //Video Mode Sync Signal Control
  { 0x00, 1, { 0xA0}},		
  { 0xC1, 3, { 0x00,0xcc,0x11}},	  
  { REGFLAG_DELAY, 1, {}},
  
  { 0x00, 1, { 0x82}},
  { 0xA4, 1, { 0x01}},	  
  { REGFLAG_DELAY, 1, {}},
#endif
  
  { 0x00, 1, { 0x00}},	//disable cmd2
  { 0xFF, 3, { 0x00,0x00,0x00}}, 
  { REGFLAG_DELAY, 1, {}},

#if( LCM_DSI_CMD_MODE )	|| (LCM_ESD_RECOVERY)
    {0x35,1,{0x00}},  	
	{ REGFLAG_DELAY, 10, {}},
#endif

//<2015/12/29-stevenchen, Enable Truly LCM CABC function
#if( ENABLE_CABC_FUNCTION )	
    {0x51,1,{0xFF}},  	
	{ REGFLAG_DELAY, 1, {}},

	{0x53,1,{0x24}},  	
	{ REGFLAG_DELAY, 1, {}},
	
	{0x55,1,{0x03}},  	
	{ REGFLAG_DELAY, 100, {}},
#endif	
//>2015/12/29-stevenchen

//<2016/04/29-stevenchen, Update Truly LCM initial code
//Fix abnormal display
  { 0x44, 2, { 0x03,0xC0}},	  
  { REGFLAG_DELAY, 1, {}},
//>2016/04/29-stevenchen

  /* Sleep Out */
  { 0x29, 0, {}},
  { REGFLAG_DELAY, 120, {}},

  /* Display ON */
  { 0x11, 0, {}},
  { REGFLAG_DELAY, 200, {}},	

    { REGFLAG_END_OF_TABLE, 0x00, {}}
  };
//>2015/12/16-stevenchen

#if 0
  static struct LCM_setting_table lcm_set_window[] =
  {
    { 0x2A, 4, { 0x00, 0x00, (FRAME_WIDTH  >> 8), (FRAME_WIDTH  & 0xFF)}},
    { 0x2B, 4, { 0x00, 0x00, (FRAME_HEIGHT >> 8), (FRAME_HEIGHT & 0xFF)}},
    { REGFLAG_END_OF_TABLE, 0x00, {}}
  };
#endif
#if 0
  static struct LCM_setting_table lcm_sleep_out_setting[] =
  {
  /* Sleep Out */
    { 0x11, 1, { 0x00 }},
    { REGFLAG_DELAY, 125, {}},
  /* Display ON */
    { 0x29, 1, { 0x00 }},
    { REGFLAG_DELAY, 25, {}},
  /* End */
    { REGFLAG_END_OF_TABLE, 0x00, {}}
  };

  static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] =
  {
  /* Display OFF */
    { 0x28, 1, { 0x00 }},
    { REGFLAG_DELAY, 25, {}},
  /* Sleep In */
    { 0x10, 1, { 0x00 }},
    { REGFLAG_DELAY, 125, {}},
  /* ENTER_DSTB_MODE */
    { 0x4F, 1, { 0x01 }}, /* Enter the Deep Standby Mode */
    { REGFLAG_DELAY, 125, {}}
  /* End */
    { REGFLAG_END_OF_TABLE, 0x00, {}}
  };
#endif

  static struct LCM_setting_table lcm_backlight_level_setting[] =
  {
    { 0x51, 1, { 0xFF }},
    { REGFLAG_END_OF_TABLE, 0x00, {}}
  };

/**********************************************************
**
***********************************************************/
static void push_table_truly(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
unsigned int i;

    for( i = 0; i < count; i++ )
    {
    unsigned cmd;

      cmd = table[i].cmd;
      switch( cmd )
      {
        case REGFLAG_DELAY :
        {
          if( table[i].count <= 10 )
            MDELAY( table[i].count );
          else
            MDELAY( table[i].count );
        } break;

        case REGFLAG_UDELAY :
        {
          UDELAY( table[i].count );
        } break;

        case REGFLAG_END_OF_TABLE :
        {
        } break;

        default:
        {
          dsi_set_cmdq_V2( cmd, table[i].count, table[i].para_list, force_update );
        } break;
      }
    }
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
static void lcm_set_util_funcs_truly(const LCM_UTIL_FUNCS *util)
{
    memcpy( &lcm_util, util, sizeof( LCM_UTIL_FUNCS ));
}

/**********************************************************
**
***********************************************************/
static void lcm_get_params_truly(LCM_PARAMS *params)
{
    memset( params, 0x00, sizeof( LCM_PARAMS ));

    params->type   = LCM_TYPE_DSI;

    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

#if( LCM_DSI_CMD_MODE )
    params->dsi.mode   = CMD_MODE;
    params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
#else
    params->dsi.mode   = SYNC_PULSE_VDO_MODE;
    params->dsi.switch_mode = CMD_MODE;
#endif
    params->dsi.switch_mode_enable = 0;

  /** DSI **/
  /* Command mode setting */
    params->dsi.LANE_NUM    = LCM_FOUR_LANE;
  /* The following defined the fomat for data coming from LCD engine. */
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

  /* Highly depends on LCD driver capability. */
    params->dsi.packet_size = 256;

  /* Video mode timing */
    params->dsi.PS  = LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.vertical_sync_active    = LCM_VSYNC_NUM;
    params->dsi.vertical_backporch      = LCM_VBP_NUM;
    params->dsi.vertical_frontporch     = LCM_VFP_NUM;
    params->dsi.vertical_active_line    = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active  = LCM_HSYNC_NUM;
    params->dsi.horizontal_backporch    = LCM_HBP_NUM;
    params->dsi.horizontal_frontporch   = LCM_HFP_NUM;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;
  //params->dsi.ssc_disable             = 1;
#if !defined( FPGA_EARLY_PORTING )
  #if( LCM_DSI_CMD_MODE )
	params->dsi.PLL_CLOCK = 450; //Vender suggestion 
  #else
    params->dsi.PLL_CLOCK = 450; //Vender suggestion 
  #endif
#else
    params->dsi.pll_div1  = 0x00;
    params->dsi.pll_div2  = 0x00;
    params->dsi.fbk_div   = 0x01;
#endif

    params->dsi.clk_lp_per_line_enable = 0;
#if LCM_ESD_RECOVERY
    params->dsi.esd_check_enable = 1; 
#else
    params->dsi.esd_check_enable = 0;
#endif
    params->dsi.customization_esd_check_enable      = 1;
    params->dsi.lcm_esd_check_table[0].cmd          = 0xDA;
    params->dsi.lcm_esd_check_table[0].count        = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x40;
	
#if( ROLLBACK_ESD_PATCH )
#else
	params->dsi.cont_clock=0;
#endif	
}

/**********************************************************
** Power manager
***********************************************************/
#define   LCM_ENABLE          1
#define   LCM_DISABLE         0
#define   LCM_POWER_VOL       VOL_DEFAULT //VOL_3000
static void lcm_power_enable_truly(int enable)
{
#if !defined( FPGA_EARLY_PORTING )
    if( LCM_ENABLE == enable )
    {
    #if 0
      #if defined( BUILD_LK )
        mt6325_upmu_set_rg_vgp1_en( 1 );
      #else
        hwPowerOn( MT6325_POWER_LDO_VGP1, LCM_POWER_VOL, "LCM_DRV" );
      #endif
    #endif
    }
    else
    {
    #if 0
      #if defined( BUILD_LK )
        mt6325_upmu_set_rg_vgp1_en( 0 );
      #else
        hwPowerDown( MT6325_POWER_LDO_VGP1, "LCM_DRV" );
      #endif
    #endif
    }
#endif
}

/**********************************************************
**
***********************************************************/
static void lcm_gate_power_truly(int enable)
{
    if( LCM_ENABLE == enable )
    {
        lcm_gate_enable( TRUE );
        UDELAY( 10 );
    }
    else
    {
        lcm_gate_enable( FALSE );
        UDELAY( 10 );
    }
}

/**********************************************************
**
***********************************************************/
static void lcm_init_power_truly(void)
{
    lcm_power_enable_truly( LCM_ENABLE );
}

static void lcm_suspend_power_truly(void)
{
    lcm_power_enable_truly( LCM_DISABLE );
}

/**********************************************************
**
***********************************************************/
static void lcm_resume_power_truly(void)
{
    lcm_power_enable_truly( LCM_ENABLE );
}

/**********************************************************
**
***********************************************************/
static void lcm_init_truly(void)
{
#if !defined( FPGA_EARLY_PORTING )
    lcm_gate_power_truly( LCM_ENABLE );
    MDELAY( 12 );
#endif /* End.. !(FPGA_EARLY_PORTING) */

    SET_RESET_PIN( 1 );
    MDELAY( 3 );
    SET_RESET_PIN( 0 );
    MDELAY( 10 );
    SET_RESET_PIN( 1 );
    MDELAY( 15 );

  /* when phone initial, config output high, enable backlight drv chip */
    push_table_truly( lcm_initialization_setting, sizeof( lcm_initialization_setting ) / sizeof( struct LCM_setting_table ), 1 );
}

/**********************************************************
**
***********************************************************/
static void lcm_suspend_truly(void)
{
    printk("[Steven][Kernel]%s: Otm1906a LCM suspend begin \n", __func__ );
    lcm_gate_enable( FALSE );
    push_table_truly( lcm_suspend_setting, sizeof( lcm_suspend_setting ) / sizeof( struct LCM_setting_table ), 1 );
  //SET_RESET_PIN( 0 );
    MDELAY( 10 );
    printk("[Steven][Kernel]%s: Otm1906a LCM suspend end \n", __func__ );	
}

/**********************************************************
**
***********************************************************/
static void lcm_resume_truly(void)
{
    printk("[Steven][Kernel]%s: Otm1906a LCM resume begin \n", __func__ );
    lcm_init_truly();
    printk("[Steven][Kernel]%s: Otm1906a LCM resume end \n", __func__ );
}

#if( LCM_DSI_CMD_MODE )
/**********************************************************
**
***********************************************************/
static void lcm_update_truly(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
unsigned int x0 = x;
unsigned int y0 = y;
unsigned int x1 = x0 + width - 1;
unsigned int y1 = y0 + height - 1;

unsigned char x0_MSB = (( x0 >> 8) & 0xFF );
unsigned char x0_LSB = ( x0 & 0xFF );
unsigned char x1_MSB = (( x1 >> 8) & 0xFF );
unsigned char x1_LSB = ( x1 & 0xFF );
unsigned char y0_MSB = (( y0 >> 8 ) & 0xFF );
unsigned char y0_LSB = ( y0 & 0xFF );
unsigned char y1_MSB = (( y1 >> 8 ) & 0xFF );
unsigned char y1_LSB = ( y1 & 0xFF );

unsigned int data_array[16];

    data_array[0] = 0x00053902;
    data_array[1] = ( x1_MSB << 24 ) | ( x0_LSB << 16 ) | ( x0_MSB << 8 ) | 0x2A;
    data_array[2] = ( x1_LSB );
    dsi_set_cmdq( data_array, 3, 1 );

    data_array[0] = 0x00053902;
    data_array[1] = ( y1_MSB << 24 ) | ( y0_LSB << 16 ) | ( y0_MSB << 8 ) | 0x2B;
    data_array[2] = ( y1_LSB );
    dsi_set_cmdq( data_array, 3, 1 );

    data_array[0] = 0x002C3909;
    dsi_set_cmdq( data_array, 1, 0 );
}
#endif

/**********************************************************
**
***********************************************************/
static unsigned int lcm_compare_id_truly(void)
{
    unsigned int  vRet = 0;

#if defined( BUILD_LK )
    unsigned int  id = 0;
    unsigned int  array[16];
    unsigned int  lcm_id_gpio;
    unsigned char buffer[2];

#if 0
    DSI_clk_HS_mode( 1 );
    MDELAY( 10 );
    DSI_clk_HS_mode( 0 );
#endif
    mt_set_gpio_mode( GPIO_LCM_ID, GPIO_LCM_ID_M_GPIO );
    mt_set_gpio_dir( GPIO_LCM_ID, GPIO_DIR_IN );
    MDELAY( 1 );

//[Arima_lavender][RaymondLin] Implement Dynamic LCM begin 
    lcm_gate_power_truly( LCM_ENABLE );
    MDELAY( 12 );
//[Arima_lavender][RaymondLin] Implement Dynamic LCM end

    SET_RESET_PIN( 1 );
    MDELAY( 3 );
    SET_RESET_PIN( 0 );
    MDELAY( 10 );
    SET_RESET_PIN( 1 );
    MDELAY( 15 );

    lcm_id_gpio = mt_get_gpio_in( GPIO_LCM_ID );

    array[0] = 0x00023700;  /* read id return two byte,version and id */
    dsi_set_cmdq( array, 1, 1 );

//[Arima_lavender][RaymondLin] Implement Dynamic LCM begin 
    read_reg_v2( 0xDA, buffer, 2 );
//[Arima_lavender][RaymondLin] Implement Dynamic LCM end
    id = buffer[0]; /* We only need ID */

#if defined( BUILD_LK )
    dprintf( 0, "[LK]%s: LCM chip ID = 0x%04X, ID pin = %d\n", __func__, id, lcm_id_gpio );
#else
    printk("[Kernel]%s: LCM chip ID = 0x%04X, ID pin = %d\n", __func__, id, lcm_id_gpio );
#endif

    vRet = 0;
//[Arima_lavender][RaymondLin] Implement Dynamic LCM begin
//[Arima_Lavender][RaymondLin] Modify LCM power on sequence begin
    if( id == LCM_OTM1906A_ID ) /* Innolux otm1906a 6" */
    {
        vRet=1;
        SET_RESET_PIN( 0 );
        MDELAY( 3 );
    }
//[Arima_Lavender][RaymondLin] Modify LCM power on sequence end
//[Arima_lavender][RaymondLin] Implement Dynamic LCM end
#endif

    return vRet;
}

/**********************************************************
** return TRUE : Need recovery
** return FALSE: No need recovery
***********************************************************/
static unsigned int lcm_esd_check_truly(void)
{
#if !defined( BUILD_LK )
int   array[4];
char  buffer[4];

    array[0] = 0x00013700;
    dsi_set_cmdq( array, 1, 1 );

    read_reg_v2( 0x53, buffer, 1 );

    if( buffer[0] != 0x24 )
    {
      printk("[LCM ERROR][0x53] = 0x%02x\n", buffer[0] );
      return TRUE;
    }
    else
    {
      printk("[LCM NORMAL][0x53] = 0x%02x\n", buffer[0] );
      return FALSE;
    }
#else
    return FALSE;
#endif

}

/**********************************************************
**
***********************************************************/
unsigned int lcm_ata_check_truly(unsigned char *buffer)
{
#if !defined( BUILD_LK )
unsigned int ret = 0;
unsigned int x0 = FRAME_WIDTH / 4;
unsigned int x1 = FRAME_WIDTH * 3 / 4;

unsigned char x0_MSB = (( x0 >> 8 ) & 0xFF );
unsigned char x0_LSB = ( x0 & 0xFF );
unsigned char x1_MSB = (( x1 >> 8 ) & 0xFF );
unsigned char x1_LSB = ( x1 & 0xFF );

unsigned int data_array[3];
unsigned char read_buf[4];

    printk("ATA check size = 0x%02X, 0x%02X, 0x%02X, 0x%02X\n", x0_MSB, x0_LSB, x1_MSB, x1_LSB );

    data_array[0] = 0x0005390A;  /* HS packet */
    data_array[1] = ( x1_MSB << 24 ) | ( x0_LSB << 16 ) | ( x0_MSB << 8 ) | 0x2A;
    data_array[2] = ( x1_LSB );
    dsi_set_cmdq( data_array, 3, 1 );

    data_array[0] = 0x00043700; /* read id return two byte,version and id */
    dsi_set_cmdq( data_array, 1, 1 );

    read_reg_v2( 0x2A, read_buf, 4 );

    if(( read_buf[0] == x0_MSB ) && ( read_buf[1] == x0_LSB )
    && ( read_buf[2] == x1_MSB ) && ( read_buf[3] == x1_LSB ))
      ret = 1;
    else
      ret = 0;

    x0 = 0;
    x1 = FRAME_WIDTH - 1;

    x0_MSB = (( x0 >> 8 ) & 0xFF );
    x0_LSB = ( x0 & 0xFF );
    x1_MSB = (( x1 >> 8 ) & 0xFF );
    x1_LSB = ( x1 & 0xFF );

    data_array[0] = 0x0005390A; /* HS packet  */
    data_array[1] = ( x1_MSB << 24 ) | ( x0_LSB << 16 ) | ( x0_MSB << 8 ) | 0x2A;
    data_array[2] = ( x1_LSB );
    dsi_set_cmdq( data_array, 3, 1 );

    return ret;
#else
    return 0;
#endif
}

/**********************************************************
**
***********************************************************/
static void lcm_setbacklight_truly(unsigned int level)
{
#if defined( BUILD_LK )
    dprintf( 0, "%s, LK nt35532 backlight: level = %d\n", __func__, level );
#else
    printk("%s, Kernel nt35532 backlight: level = %d\n", __func__, level );
#endif
  /* Refresh value of backlight level. */
    lcm_backlight_level_setting[0].para_list[0] = level;

    push_table_truly( lcm_backlight_level_setting, sizeof( lcm_backlight_level_setting ) / sizeof( struct LCM_setting_table ), 1 );
}

/**********************************************************
**
***********************************************************/
void* lcm_switch_mode_truly(int mode)
{
#if !defined( BUILD_LK )
/* customization: 1. V2C config 2 values, C2V config 1 value; 2. config mode control register */
    if( mode == 0 )
    { /* V2C */
      lcm_switch_mode_cmd_truly.mode    = CMD_MODE;
      lcm_switch_mode_cmd_truly.addr    = 0xBB; /* mode control addr */
      lcm_switch_mode_cmd_truly.val[0]  = 0x13; /* enabel GRAM firstly, ensure writing one frame to GRAM */
      lcm_switch_mode_cmd_truly.val[1]  = 0x10; /* disable video mode secondly */
    }
    else
    { /* V2C */
      lcm_switch_mode_cmd_truly.mode    = SYNC_PULSE_VDO_MODE;
      lcm_switch_mode_cmd_truly.addr    = 0xBB;
      lcm_switch_mode_cmd_truly.val[0]  = 0x03; /* disable GRAM and enable video mode */
    }

    return (void*)( &lcm_switch_mode_cmd_truly );
#else
    return NULL;
#endif
}

/**********************************************************
**
***********************************************************/
LCM_DRIVER otm1906a_fhd_dsi_vdo_6inch_truly_lcm_drv=
{
    .name             = "otm1906a_fhd_dsi_vdo_6inch_truly_drv",
    .set_util_funcs   = lcm_set_util_funcs_truly,
    .get_params       = lcm_get_params_truly,
    .init             = lcm_init_truly, /*tianma init fun.*/
    .suspend          = lcm_suspend_truly,
    .resume           = lcm_resume_truly,
    .compare_id       = lcm_compare_id_truly,
    .init_power       = lcm_init_power_truly,
    .resume_power     = lcm_resume_power_truly,
    .suspend_power    = lcm_suspend_power_truly,
    .esd_check        = lcm_esd_check_truly,
    .set_backlight    = lcm_setbacklight_truly,
    .ata_check        = lcm_ata_check_truly,
#if( LCM_DSI_CMD_MODE )
    .update           = lcm_update_truly,
#endif
    .switch_mode      = lcm_switch_mode_truly,
};

#undef OTM1906A_FHD_DSI_VDO_6INCH_TRULY_C
#endif
