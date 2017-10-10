#ifndef R61335_FHD_DSI_VDO_TRULY_C
#define R61335_FHD_DSI_VDO_TRULY_C

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
    #include  <cust_gpio_usage.h>
#elif defined( BUILD_UBOOT )
    #include  <asm/arch/mt_gpio.h>
#else
//    #include  <mach/mt_pm_ldo.h>
//    #include  <mach/mt_gpio.h>
#endif

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
    #define   LCM_PANEL_SOURCE          1   /* 1: Truly 6", 0: Innolux otm1906a 6" */
    #define   LCM_ID_R63422            (0x3422)
	#define   ENABLE_CABC_FUNCTION      1

    #define   LCM_DSI_CMD_MODE          1

#if defined( FPGA_EARLY_PORTING )
    #define   FRAME_WIDTH               (480)
    #define   FRAME_HEIGHT              (800)
#else
    #define   FRAME_WIDTH               (1080)
    #define   FRAME_HEIGHT              (1920)
#endif

    #define   LCM_HSYNC_NUM             (24)    /** Shall be larger than 3 **/
    #define   LCM_HBP_NUM               (80)   //20
    #define   LCM_HFP_NUM               (100)   //40

    #define   LCM_VSYNC_NUM             (2)     /** Shall be larger than 3 ? **/
    #define   LCM_VBP_NUM               (6)    //10
    #define   LCM_VFP_NUM               (8)    //10

    #define   REGFLAG_UDELAY            (0xFE)  //(0xFB)  /* RELOAD CMD1 */
    #define   REGFLAG_DELAY             (0xFC)
    #define   REGFLAG_END_OF_TABLE      (0xFD)  /* END OF REGISTERS MARKER */

  //#define   REGFLAG_RESET_LOW         (0xFE)  /* RD_CMDSTATUS: Read the Current Register Set */
  //#define   REGFLAG_RESET_HIGH        (0xFF)  /* CMD Page Select */

    #define   LCM_ESD_CHECK_REG         (0x53)
    #define   LCM_ESD_CHECK_VAL         (0x00)  //(0x24)

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

  //static unsigned int         lcm_esd_test_truly = FALSE; /* Only for ESD test */
  //static unsigned char        lcd_id_pins_value_truly = 0xFF;
    static const unsigned char  LCD_MODULE_ID_truly = 0x00;
    LCM_DSI_MODE_SWITCH_CMD     lcm_switch_mode_cmd_truly_r63422;

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
#if defined(BUILD_LK)
//[Arima_lavender][RaymondLin] Implement Truly LCM Power on sequence begin
static struct LCM_setting_table lcm_read_chip_id_setting[] =
  {
    {0xB0, 1, {0x04}},
	
	{ REGFLAG_DELAY, 100, {}}, 
  /* End */
    { REGFLAG_END_OF_TABLE, 0x00, {}}
  }; 
//[Arima_lavender][RaymondLin] Implement Truly LCM Power on sequence end 
#endif

  static struct LCM_setting_table lcm_suspend_setting[] =
  {
  /* Display OFF */
    { 0x28, 0, {}},
    { REGFLAG_DELAY, 25, {}},
//[Arima_Lavender][RaymondLin] Vender suggestion due to fix flicker issue begin
{ 0xF4, 9, {0xF3,0xFF,0x05,0x85,0x3D,0x01,0x19,0x19,0x00}},
   { REGFLAG_DELAY, 10, {}},
//[Arima_Lavender][RaymondLin] Vender suggestion due to fix flicker issue end
  /* Sleep In */
    { 0x10, 0, {}},
    { REGFLAG_DELAY, 125, {}},
//[Arima_Lavender][RaymondLin] add deep sleep mode to fix Truly LCM flicker issue begin
	   { 0xB0, 1, {0x00}},
   { REGFLAG_DELAY, 25, {}},
//[Arima_Lavender][RaymondLin] add deep sleep mode to fix Truly LCM flicker issue end
//[Arima_Lavender][RaymondLin] Fix Truly LCM blink after reboot issue begin
   { 0xB1, 1, {0x01}},
   { REGFLAG_DELAY, 25, {}},
//[Arima_Lavender][RaymondLin] Fix Truly LCM blink after reboot issue end
  /* End */
    { REGFLAG_END_OF_TABLE, 0x00, {}}
  };

  static struct LCM_setting_table lcm_initialization_setting[] =
  {

  //raymond test b
   {0xB0, 1, {0x04}},

//[Arima_lavender][RaymondLin] Truly fine tune gamma value 20150515 begin
   { 0x35, 1, { 0x00}},
//[Arima_lavender][RaymondLin] Truly fine tune gamma value 20150515 end
#if( LCM_DSI_CMD_MODE )	
   { 0xB3, 4, { 0x04,0x02,0x00,0x00}}, //Interface Setting
#else
   { 0xB3, 4, { 0x35,0x02,0x00,0x00}}, //Interface Setting
#endif
   { 0xB6, 3, { 0x3A,0xD3,0x00}}, //DSI Control (MIPI Speed)
//[Arima_Lavender][RaymondLin] Fix Truly LCM tearing effect 20150422 begin
//[Arima_Lavender][RaymondLin] Vender suggestion due to fix flicker issue begin
   { 0xC1, 35, { 0x80 ,0x60 ,0x11 ,0xF1 ,0xFF ,0xCF ,0xDA ,0xFF ,0xFF ,0x9F ,0x12 ,0xE4 ,0xA4 ,0x74 ,0x4A ,0x40 ,0xCC ,0xFF ,0xFF ,0x37 ,0xF6 ,0xFF ,0x8F ,0x10 ,0x10 ,0x10 ,0x10 ,0x00 ,0x02 ,0x01 ,0x02 ,0x22 ,0x11 ,0x00 ,0x01}},
//[Arima_Lavender][RaymondLin] Vender suggestion due to fix flicker issue end
//[Arima_Lavender][RaymondLin] Fix Truly LCM tearing effect 20150422 end
   { 0xC2, 8, { 0x31,0xF7,0x80,0x08,0x08,0x00,0x08,0x00 }}, //F7 Column inversion

   { 0xC3, 3, { 0x00,0x00,0x00 }}, 

   { 0xC4, 14, { 0x70,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0C,0x06,0x00}}, //Source Timing Setting

   { 0xC6, 21, { 0x78, 0x08,0x67,0x08,0x67,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x16,0x18,0x08,0xC8}}, //LTPS Timing Setting
//[Arima_Lavender][RaymondLin] implement Truly LCM gamma value 20150618 begin
   { 0xC7, 30, { 0x0F ,0x18 ,0x21 ,0x2C ,0x37 ,0x46 ,0x50 ,0x5D ,0x41 ,0x48 ,0x53 ,0x5F ,0x66 ,0x6B ,0x71 ,0x0F ,0x18 ,0x21 ,0x2C ,0x37 ,0x46 ,0x50 ,0x5D ,0x41 ,0x48 ,0x53 ,0x5F ,0x66 ,0x6B ,0x71}},
//[Arima_Lavender][RaymondLin] implement Truly LCM gamma value 20150618 end
   { 0xC8, 19, { 0x00,0x00,0x00,0x00,0x00,0xFC,0x00,0x00,0x00,0x00,0x00,0xFC,0x00,0x00,0x00,0x00,0x00,0xFC,0x00}}, 
//[Arima_Lavender][RaymondLin] Vender suggestion due to fix flicker issue begin
   { 0xCB, 14, { 0x31 ,0xFC ,0x3F ,0x8C ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0xE0 ,0x00}}, //Interface Setting
//[Arima_Lavender][RaymondLin] Vender suggestion due to fix flicker issue end
   { 0xCC, 1, { 0x0B }}, //Panel Interface Control

   { 0xD0, 4, { 0x11,0x99,0x19,0xFD }},
//[Arima_Lavender][RaymondLin] Fix Truly LCM blink after reboot issue begin
    { 0xD2, 16, { 0x8A,0x47,0x47,0x33,0x10,0x33,0x33,0x33,0x77,0x77,0x33,0x33,0x33,0x00,0x00,0x00 }},
//[Arima_Lavender][RaymondLin] Fix Truly LCM blink after reboot issue end
  { 0xD5, 7, { 0x06,0x00,0x00,0x01,0x2C,0x01,0x2C }}, //Vcom Setting

   { 0xE5, 4, { 0x00,0x3F,0xFF,0x10}}, 
   
   { 0xD6, 1, { 0x01}},
//[Arima_Lavender][RaymondLin] Fix Truly LCM tearing effect 20150422 begin   
   { 0x36, 1, { 0x00}}, //upside down
//[Arima_Lavender][RaymondLin] Fix Truly LCM tearing effect 20150422 end   
//[Arima_Lavender][RaymondLin] Fix Truly LCM tearing effect 20150729 begin
{ 0x44, 2, { 0x00,0xFF}},
//[Arima_Lavender][RaymondLin] Fix Truly LCM tearing effect 20150729 end
#if( ENABLE_CABC_FUNCTION )	
//[Arima_lavender][RaymondLin] Truly fine tune gamma value 20150515 begin
    //{0xB8,7,{0x57,0x3d,0x19,0x1e,0x0a,0x50,0x50}},  //GUI mode

//[Arima_Lavender][RaymondLin] Truly fix cross talk issue 20150527 begin 
	{0xB9,7,{0x6F ,0x48 ,0x10 ,0x67 ,0x0C ,0x70 ,0x70}},  	// Still mode
//[Arima_Lavender][RaymondLin] Truly fix cross talk issue 20150527 end
	
	//{0xBA,7,{0xb5,0x33,0x41,0x64,0x23,0xa0,0xa0}},  	// Movie mode
//[Arima_Lavender][RaymondLin] modify PWM frequence for fix Truly flicker issue by camera viewfinder begin
 	{0xCE,25,{0x55 ,0x40 ,0x49 ,0x53 ,0x59 ,0x5E ,0x63 ,0x68 ,0x6E ,0x74 ,0x7E ,0x8A ,0x98 ,0xA8 ,0xBB ,0xD0 ,0xFF ,0x0D ,0x00 ,0x04 ,0x04 ,0x42 ,0x04 ,0x69 ,0x5A}},	
//[Arima_Lavender][RaymondLin] modify PWM frequence for fix Truly flicker issue by camera viewfinder end
	{0x5E,1,{0x00}}, 
	
	 {0x51,1,{0xFF}},  	

    {0x53,1,{0x2C}}, 	

	{0x55,1,{0x02}},  	
//[Arima_lavender][RaymondLin] Truly fine tune gamma value 20150515 end
#endif	
   
   
  //raymond test e

  /* Display ON */
    { 0x29, 0, {}},
    { REGFLAG_DELAY, 30, {}},

  /* Sleep Out */
    { 0x11, 0, {}},
    { REGFLAG_DELAY, 180, {}},

  /* End */
    { REGFLAG_END_OF_TABLE, 0x00, {}}
  };

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
static void push_table_truly_r63422(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
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
static void lcm_set_util_funcs_truly_r63422(const LCM_UTIL_FUNCS *util)
{
    memcpy( &lcm_util, util, sizeof( LCM_UTIL_FUNCS ));
}

/**********************************************************
**
***********************************************************/
static void lcm_get_params_truly_r63422(LCM_PARAMS *params)
{
    memset( params, 0x00, sizeof( LCM_PARAMS ));

    params->type   = LCM_TYPE_DSI;

    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

#if( LCM_DSI_CMD_MODE )
    params->dsi.mode   = CMD_MODE;
    params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
#else
    params->dsi.mode   = BURST_VDO_MODE; //SYNC_PULSE_VDO_MODE;
    params->dsi.switch_mode = CMD_MODE;
#endif
    params->dsi.switch_mode_enable = 0;

  /** DSI **/
  /* Command mode setting */
    params->dsi.LANE_NUM    = LCM_FOUR_LANE;  /* 4 data lane */
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
    params->dsi.PLL_CLOCK = 500; //this value must be in MTK suggested table
  #else
    params->dsi.PLL_CLOCK = 480;  /* 920Mbps */ //LCM_DSI_6589_PLL_CLOCK_461_5; //LCM_DSI_6589_PLL_CLOCK_468;
  #endif
#else
    params->dsi.pll_div1  = 0x00;
    params->dsi.pll_div2  = 0x00;
    params->dsi.fbk_div   = 0x01;
#endif

    params->dsi.clk_lp_per_line_enable = 0;
    params->dsi.esd_check_enable = 0;//1
    params->dsi.customization_esd_check_enable      = 0;
    params->dsi.lcm_esd_check_table[0].cmd          = LCM_ESD_CHECK_REG;
    params->dsi.lcm_esd_check_table[0].count        = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = LCM_ESD_CHECK_VAL;

}

/**********************************************************
** Power manager
***********************************************************/
#define   LCM_ENABLE          1
#define   LCM_DISABLE         0
#define   LCM_POWER_VOL       VOL_DEFAULT //VOL_3000

static void lcm_gate_power_truly_r63422(int enable)
{
#if !defined( FPGA_EARLY_PORTING )

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
#endif /* End.. !(FPGA_EARLY_PORTING) */
}

/**********************************************************
**
***********************************************************/
static void lcm_init_power_truly_r63422(void)
{
    
}

static void lcm_suspend_power_truly_r63422(void)
{
    
}

/**********************************************************
**
***********************************************************/
static void lcm_resume_power_truly_r63422(void)
{
    
}

/**********************************************************
**
***********************************************************/
static void lcm_init_truly_r63422(void)
{
#if !defined( FPGA_EARLY_PORTING )
//[Arima_lavender][RaymondLin] Implement Truly LCM Power on sequence begin
    //lcm_gate_power_truly_r63422( LCM_ENABLE );
    //MDELAY( 12 );
//[Arima_lavender][RaymondLin] Implement Truly LCM Power on sequence end	
#endif /* End.. !(FPGA_EARLY_PORTING) */

//Steven start
    //SET_RESET_PIN( 1 );
    //MDELAY( 3 );
    //SET_RESET_PIN( 0 );
    //MDELAY( 20 );
    //SET_RESET_PIN( 1 );
    //MDELAY( 20 );

    SET_RESET_PIN( 0 );
    MDELAY( 3 );
    lcm_gate_power_truly_r63422( LCM_ENABLE );
    MDELAY( 20 );
    SET_RESET_PIN( 1 );
    MDELAY( 20 );
//Steven end

  /* when phone initial, config output high, enable backlight drv chip */
    push_table_truly_r63422( lcm_initialization_setting, sizeof( lcm_initialization_setting ) / sizeof( struct LCM_setting_table ), 1 );
}

/**********************************************************
**
***********************************************************/
static void lcm_suspend_truly_r63422(void)
{
    printk("[Steven] %s \n", __func__);
    lcm_gate_enable( FALSE );
    push_table_truly_r63422( lcm_suspend_setting, sizeof( lcm_suspend_setting ) / sizeof( struct LCM_setting_table ), 1 );
  //SET_RESET_PIN( 0 );
    MDELAY( 10 );
}

/**********************************************************
**
***********************************************************/
static void lcm_resume_truly_r63422(void)
{
    printk("[Steven] %s \n", __func__);
    lcm_init_truly_r63422();
}

#if( LCM_DSI_CMD_MODE )
/**********************************************************
**
***********************************************************/
static void lcm_update_truly_r63422(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
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
//<2015/09/03-stevenchen, Fix can not power on without LCM
static unsigned int lcm_compare_id_truly_r63422(void)
{
    unsigned int vRet=0;

#if defined( BUILD_LK )
    unsigned int id=0;
    unsigned char buffer[5];
    unsigned int array[16];  
    unsigned int  lcm_id_gpio;


    mt_set_gpio_mode( GPIO_LCM_ID, GPIO_LCM_ID_M_GPIO );
    mt_set_gpio_dir( GPIO_LCM_ID, GPIO_DIR_IN );
    MDELAY( 1 );

    SET_RESET_PIN( 0 );
    MDELAY( 3 );
    lcm_gate_power_truly_r63422( LCM_ENABLE );
    MDELAY( 12 );
    SET_RESET_PIN( 1 );
    MDELAY( 10 );

    lcm_id_gpio = mt_get_gpio_in( GPIO_LCM_ID );

    push_table_truly_r63422( lcm_read_chip_id_setting, sizeof( lcm_read_chip_id_setting ) / sizeof( struct LCM_setting_table ), 1 );

    array[0] = 0x00053700;// read id return two byte,version and id
    dsi_set_cmdq(array, 1, 1);
	  
    read_reg_v2(0xBF, buffer, 5);
    MDELAY(20);
    id = (buffer[2] << 8 )| buffer[3];

#if defined( BUILD_LK )
    dprintf( 0, "[LK]%s: LCM ID pin = %d, LCM chip ID = 0x%04X\n", __func__, lcm_id_gpio, id );
#else
    printk("[Kernel]%s: LCM chip ID = 0x%04X\n", __func__, id );
#endif

    vRet = 0;
    
    if( id == LCM_ID_R63422 ) /* Truly R63422 */
    {
    #if defined( BUILD_LK )
        dprintf( 0, "[LK][Steven]: LCM module is Truly r63422\n");
    #else
        printk("[Kernel][Steven]: LCM module is Truly r63422\n");
    #endif
        vRet=1;

        SET_RESET_PIN( 0 );
        MDELAY( 3 );
        return vRet;
    }

    lcm_gate_power_truly_r63422( LCM_DISABLE );
    SET_RESET_PIN( 0 );
    MDELAY( 3 );
#endif

    return vRet;
}
//>2015/09/03-stevenchen

/**********************************************************
** return TRUE : Need recovery
** return FALSE: No need recovery
***********************************************************/
static unsigned int lcm_esd_check_truly_r63422(void)
{
#if !defined( BUILD_LK )
int   array[4];
char  buffer[4];

    array[0] = 0x00013700;
    dsi_set_cmdq( array, 1, 1 );

    read_reg_v2( LCM_ESD_CHECK_REG, buffer, 1 );

    if( buffer[0] != LCM_ESD_CHECK_VAL )
    {
      printk("[LCM ERROR]%s: 0x%02X = 0x%02X\n", __func__, LCM_ESD_CHECK_REG, buffer[0] );
      return TRUE;
    }
    else
    {
      printk("[LCM NORMAL]%s: 0x%02X = 0x%02X\n", __func__, LCM_ESD_CHECK_REG, buffer[0] );
      return FALSE;
    }
#else
    return FALSE;
#endif

}

/**********************************************************
**
***********************************************************/
unsigned int lcm_ata_check_truly_r63422(unsigned char *buffer)
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
static void lcm_setbacklight_truly_r63422(unsigned int level)
{
#if defined( BUILD_LK )
    dprintf( 0, "%s, LK r63422 backlight: level = %d\n", __func__, level );
#else
    printk("%s, Kernel r63422 backlight: level = %d\n", __func__, level );
#endif
  /* Refresh value of backlight level. */
    lcm_backlight_level_setting[0].para_list[0] = level;

    push_table_truly_r63422( lcm_backlight_level_setting, sizeof( lcm_backlight_level_setting ) / sizeof( struct LCM_setting_table ), 1 );
}

/**********************************************************
**
***********************************************************/
void* lcm_switch_mode_truly_r63422(int mode)
{
#if !defined( BUILD_LK )
/* customization: 1. V2C config 2 values, C2V config 1 value; 2. config mode control register */
    if( mode == 0 )
    { /* V2C */
      lcm_switch_mode_cmd_truly_r63422.mode    = CMD_MODE;
      lcm_switch_mode_cmd_truly_r63422.addr    = 0xBB; /* mode control addr */
      lcm_switch_mode_cmd_truly_r63422.val[0]  = 0x13; /* enabel GRAM firstly, ensure writing one frame to GRAM */
      lcm_switch_mode_cmd_truly_r63422.val[1]  = 0x10; /* disable video mode secondly */
    }
    else
    { /* V2C */
      lcm_switch_mode_cmd_truly_r63422.mode    = SYNC_PULSE_VDO_MODE;
      lcm_switch_mode_cmd_truly_r63422.addr    = 0xBB;
      lcm_switch_mode_cmd_truly_r63422.val[0]  = 0x03; /* disable GRAM and enable video mode */
    }

    return (void*)( &lcm_switch_mode_cmd_truly_r63422 );
#else
    return NULL;
#endif
}

/**********************************************************
**
***********************************************************/
LCM_DRIVER r63422_fhd_dsi_cmd_truly_lcm_drv=
{
    .name             = "r63422_fhd_dsi_cmd_truly_drv",
    .set_util_funcs   = lcm_set_util_funcs_truly_r63422,
    .get_params       = lcm_get_params_truly_r63422,
    .init             = lcm_init_truly_r63422,   /*tianma init fun.*/
    .suspend          = lcm_suspend_truly_r63422,
    .resume           = lcm_resume_truly_r63422,
    .compare_id       = lcm_compare_id_truly_r63422,
    .init_power       = lcm_init_power_truly_r63422,
    .resume_power     = lcm_resume_power_truly_r63422,
    .suspend_power    = lcm_suspend_power_truly_r63422,
    .esd_check        = lcm_esd_check_truly_r63422,
    .set_backlight    = lcm_setbacklight_truly_r63422,
    .ata_check        = lcm_ata_check_truly_r63422,
#if( LCM_DSI_CMD_MODE )
    .update           = lcm_update_truly_r63422,
#endif
    .switch_mode      = lcm_switch_mode_truly_r63422,
};

#undef R61335_FHD_DSI_VDO_TRULY_C
#endif

