#ifndef LED_LM3533_C
#define LED_LM3533_C
/*******************************************************************************
**
********************************************************************************/
#include "leds_lm3533.h"
// <<< 2016/08/22-dangyiwang. change brightness mapping value
static int sony_backlight_map_ref[255]=
	{21	,21	,21	,21	,21	,21	,21	,21	,21	,21	,21	,21	,21	,21	,22	,22	,
	22	,22	,22	,22	,22	,22	,22	,22	,23	,23	,23	,23	,23	,23	,23	,23	,
	23	,23	,24	,24	,24	,24	,24	,24	,24	,24	,24	,24	,25	,25	,25	,25	,
	25	,25	,25	,25	,25	,25	,25	,26	,26	,26	,26	,26	,26	,27	,27	,27	,
	27	,27	,27	,28	,28	,28	,28	,28	,29	,29	,29	,29	,29	,29	,30	,30	,
	30	,30	,30	,30	,31	,31	,31	,31	,31	,32	,32	,33	,34	,34	,35	,36	,
	36	,37	,38	,38	,39	,40	,40	,41	,42	,42	,43	,44	,44	,45	,45	,46	,
	47	,47	,48	,48	,49	,50	,50	,51	,51	,52	,53	,53	,54	,54	,55	,56	,
	56	,57	,57	,58	,59	,59	,60	,61	,61	,62	,63	,64	,65	,66	,67	,68	,
	69	,70	,71	,72	,73	,74	,75	,76	,77	,78	,79	,80	,81	,82	,83	,84	,
	85	,86	,87	,88	,89	,90	,91	,92	,93	,94	,95	,97	,98	,100,102,103,
	105,107	,108,110,112,113,115,117,118,120,122,123,125,127,128,130,
	132,133	,135,137,138,140,142,143,145,147,148,150,152,153,155,157,
	158,160	,161,164,166,168,170,172,174,176,178,181,183,185,187,189,
	191,193	,195,197,200,202,204,206,208,210,212,214,216,219,221,223,
	225,227	,229,231,233,235,238,240,242,244,246,248,250,252,255};
// >>> 2016/08/22-dangyiwang. change brightness mapping value

static int lm3533_debug_enable = 1;
#define   LEDS_DEBUG(format, args...) do{ \
                if(lm3533_debug_enable) \
                {\
                    printk(KERN_DEBUG format,##args);\
                }\
              }while(0)

/*******************************************************************************
**
********************************************************************************/
//<2015/06/05-youchihwang, add i2c address probe
static int lm3533_i2c_auto_probe(struct i2c_client *client);
static unsigned short lm3533_i2c_auto_probe_address[] = {0x36, 0x38}; //I2C Address 0x36 : LM3533TME-40
#define LM3533_I2C_PROBE_ADDRESS_AMOUNT ARRAY_SIZE(lm3533_i2c_auto_probe_address)
//>2015/06/05-youchihwang,
/*******************************************************************************
**
********************************************************************************/
static const struct of_device_id lm3533_i2c_of_match[] = {
	{.compatible = "mediatek,lm3533"},
	{},
};

static struct i2c_driver lm3533_i2c_driver =
{
  .driver = {
    .name   = "lm3533-i2c",
    .owner  = THIS_MODULE,
    .of_match_table = lm3533_i2c_of_match,
  },
  .id_table = lm3533_i2c_ids,
  .probe    = lm3533_i2c_probe,
  .remove   = lm3533_i2c_remove,
};

// <<<<<----- 2015/04/28-youchihwang
// DMS06418317--[Ar]RT-GST-<5919_Notification>-<The phone Notification LED light is blue when receiving a missed call/Email/Message.>
// ALPS02045680
static int pattern_id = 0;
// <<< 2016/05/03-dangyiwang, remove debug log
//#define LED_TRACE_INFO
// >>> 2016/05/03-dangyiwang, remove debug log
// ----->>>>> 2015/04/28-youchihwang
// DMS06418317--[Ar]RT-GST-<5919_Notification>-<The phone Notification LED light is blue when receiving a missed call/Email/Message.>
// ALPS02045680

struct pinctrl *pinctrl = NULL;
struct pinctrl_state *lm3533_rst_output0 = NULL, *lm3533_rst_output1 = NULL;
/*******************************************************************************
**
********************************************************************************/
static int lm3533_write_reg(u8 reg, u8 writeData)
{
  u8    data_buf[2] = { 0x00 };
  int   ret = 0;

  data_buf[0] = reg;
  data_buf[1] = writeData;
  ret = i2c_master_send( lm3533_i2c_client, (const char*)data_buf, 2 );
  if( ret < 0 )
  {
    LEDS_DEBUG("Send command(0x%02X) error!!\n", data_buf[0]);
    return -EFAULT;
  }

  return 0;
} /* End.. lm3533_write_reg() */

/*******************************************************************************
**
********************************************************************************/
static int lm3533_read_reg(u8 reg, u8 *returnData)
{
  u8    data_buf[2] = { 0x00 };
  int   ret = 0;

  data_buf[0] = reg;
  ret = i2c_master_send( lm3533_i2c_client, (const char*)data_buf, 1 );
  if( ret < 0 )
  {
    LEDS_DEBUG("Send command(0x%02X) error!!\n", data_buf[0]);
    return -EFAULT;
  }

  ret = i2c_master_recv( lm3533_i2c_client, (char*)returnData, 1 );
  if( ret < 0 )
  {
    LEDS_DEBUG("Read reg(0x%02X) data error!!\n", data_buf[0]);
    return -EFAULT;
  }

  return 0;
} /* End.. lm3533_read_reg() */

/*******************************************************************************
**
********************************************************************************/
static void lm3533_device_enable( int enable )
{
	int ret;

#if defined( LED_TRACE_INFO )
  printk("[LED] %s: %d \n", __func__, enable );
#endif
		
#if 1		
	if (enable == false)
	{
	  ret = pinctrl_select_state(pinctrl, lm3533_rst_output0);
    printk("[LED] lm3533_device_enable ret=%d\n", ret);
	}  
	else
	{
	  ret = pinctrl_select_state(pinctrl, lm3533_rst_output1);
    printk("[LED] lm3533_device_enable ret=%d\n", ret);
	}  
#else
  mt_set_gpio_mode( GPIO_LCM_LED_EN, GPIO_LCM_LED_EN_M_GPIO );
  mt_set_gpio_dir( GPIO_LCM_LED_EN, GPIO_DIR_OUT );
  if( TRUE == enable )
    mt_set_gpio_out( GPIO_LCM_LED_EN, GPIO_OUT_ONE );
  else
    mt_set_gpio_out( GPIO_LCM_LED_EN, GPIO_OUT_ZERO );
#endif  
} /* End.. lm3533_device_enable() */

/*************************************************************************
** Show last android LED when turning off all TS LED
**************************************************************************/
void led_switch(void)
{
struct nled_setting   nled_tmp_setting = { 0, 0, 0 };
struct led_trigger  * p_led_trigger;
int   idx = MT65XX_LED_TYPE_RED;

#if defined( LED_TRACE_INFO )
  printk("[LED] lm3533 %s \n", __func__ );
#endif
        // <<< 2016/01/18-youchihwang. Feature FP019344 Illumination
        for (idx = MT65XX_LED_TYPE_RED; idx <= MT65XX_LED_TYPE_BLUE; idx++) {

                p_led_trigger = g_leds_data[idx]->cdev.trigger;

                if (p_led_trigger != NULL) {
                        if (!strcmp (p_led_trigger->name, "timer") && (g_leds_data[idx]->delay_on != 0) && (g_leds_data[idx]->delay_off != 0))
                                nled_tmp_setting.nled_mode = NLED_BLINK;
                        
                        nled_tmp_setting.blink_off_time = g_leds_data[idx]->delay_off;
                        nled_tmp_setting.blink_on_time  = g_leds_data[idx]->delay_on;
                        
                        if (idx == MT65XX_LED_TYPE_RED)
                                blink_set_lm3533 (MT65XX_LED_LM3533_L3, &nled_tmp_setting, g_leds_data[idx]->level);
                        else if (idx == MT65XX_LED_TYPE_GREEN)
                                blink_set_lm3533 (MT65XX_LED_LM3533_L2, &nled_tmp_setting, g_leds_data[idx]->level);
                        else if (idx == MT65XX_LED_TYPE_BLUE)
                                blink_set_lm3533 (MT65XX_LED_LM3533_L1, &nled_tmp_setting, g_leds_data[idx]->level);
                }
#if defined( LED_TRACE_INFO )
                printk("led_switch g_leds_data[%d] = %d\n", idx, g_leds_data[idx]->level );
#endif
                if (idx == MT65XX_LED_TYPE_RED)
                        brightness_set_lm3533 (MT65XX_LED_LM3533_L3, g_leds_data[idx]->level);
                else if (idx == MT65XX_LED_TYPE_GREEN)
                        brightness_set_lm3533 (MT65XX_LED_LM3533_L2, g_leds_data[idx]->level);
                else if (idx == MT65XX_LED_TYPE_BLUE)
                        brightness_set_lm3533 (MT65XX_LED_LM3533_L1, g_leds_data[idx]->level);
  }
        // >>> 2016/01/18-youchihwang. Feature FP019344 Illumination
} /* End.. led_switch() */

/*************************************************************************
** Add VALUE_BUTTON_3 and VALUE_PATTERN_2 for LED illumination
**************************************************************************/
void led_pulse_work_callback(struct work_struct *work)
{
u8  enablevalue = 0;

#if defined( LED_TRACE_INFO )
  printk("[LED] lm3533 %s \n", __func__ );
#endif
  if( is_keep_light )
  {
  /* Disable pattern */
    lm3533_write_reg( 0x28, 0x00 );
  }
  else
  {
    lm3533_read_reg( 0x27, &enablevalue );

  /* Disable R/G/B LED */
    enablevalue = enablevalue & 0x03;
    lm3533_write_reg( 0x27, enablevalue );

  /* Show last android LED when turning off all TS LED */
    if( pattern_enable == false )
      led_switch();

    pattern_enable = false;
  }
} /* End.. led_pulse_work_callback() */

static enum hrtimer_restart led_pulse_timer_func(struct hrtimer *timer)
{
  queue_work( led_pulse_workqueue, &led_pulse_work );

  return HRTIMER_NORESTART;
} /* End.. led_pulse_timer_func() */

#if 0
/*************************************************************************
** Add flash_mode property for LED VALUE_BUTTON_2 type
** Fix led illumination issues for SONY requirements
**      static int pattern_set_lm3533(struct cust_mt65xx_led *cust, int pattern_id)
** Connect native(lights.c) to driver(leds.c)
** Add falsh_mode for lm3533 light
**************************************************************************/
static int pattern_set_lm3533(struct cust_mt65xx_led *cust, int pattern_id, int flash_mode, u8 red_data, u8 green_data, u8 blue_data)
{
//u8  data_buf[2];
u8  enablevalue = 0, data_idx = 0, idx;
// <<<<<----- 2015/04/28-youchihwang
// DMS06418317--[Ar]RT-GST-<5919_Notification>-<The phone Notification LED light is blue when receiving a missed call/Email/Message.>
// ALPS02045680
return 0;
// ----->>>>> 2015/04/28-youchihwang
// DMS06418317--[Ar]RT-GST-<5919_Notification>-<The phone Notification LED light is blue when receiving a missed call/Email/Message.>
// ALPS02045680

#if defined( LED_TRACE_INFO )
  printk("[LED]LM3533 pattern_set_lm3533 pattern_id=%d, red=0x%02X, green=0x%02X, blue=0x%02X\n",
        pattern_id, red_data, green_data, blue_data );
#endif

  mutex_lock( &lm3533_i2c_access );

  /* Disable R/G/B */
  lm3533_read_reg( 0x27, &enablevalue );
#if defined( LED_TRACE_INFO )
  printk("[LED]LM3533 pattern_set_lm3533 enablevalue=0x%02X\n", enablevalue );
#endif
  enablevalue = enablevalue & 0x03;
  lm3533_write_reg( 0x27, enablevalue );

  is_keep_light     = false;
  keep_light_color  = 0x00;
  hrtimer_cancel( &led_pulse_timer );

  /* Show last android LED when turning off all TS LED */
  if(( 0 == blue_data ) && ( 0 == green_data ) && ( 0 == red_data ))
  {
    pattern_enable = false;
    lm3533_write_reg( 0x28, 0x00 );
    queue_work( led_pulse_workqueue, &led_pulse_work );
    mutex_unlock( &lm3533_i2c_access );
    return 0;
  }
  pattern_enable = true;

  if( blue_data )
  {
    enablevalue = enablevalue | 0x04;
    lm3533_write_reg( 0x42, blue_data );  /* Set current for blue LED */
  }

  if( green_data )
  {
    enablevalue = enablevalue | 0x08;
    lm3533_write_reg( 0x43, green_data ); /* Set current for green LED */
  }

  if( red_data )
  {
    enablevalue = enablevalue | 0x10;
    lm3533_write_reg( 0x44, red_data ); /* Set current for red LED */
  }

  /* Set pattern data */
  switch( pattern_id )
  {
    case 1:
    {
      if( flash_mode == 0 )
        data_idx = 2;
    } break;
    case 4:
    {
      data_idx = 3;
    } break;
    case 5:
    {
      data_idx = 1;
    } break;
    case 6:
    {
      data_idx = 4;
    } break;
  }

  for( idx = 0; idx < pattern_data[data_idx].count; idx++ )
  {
    lm3533_write_reg( pattern_data[data_idx].config_data[idx].cmd, pattern_data[data_idx].config_data[idx].data );
  }

  lm3533_write_reg( 0x28, 0x15 );
  lm3533_write_reg( 0x27, enablevalue );

  /* Enable timer to turn off R/G/B LED */
  switch( data_idx )
  {
    case 2:
    {
      is_keep_light = true;
      keep_light_color = enablevalue;
      hrtimer_start( &led_pulse_timer, ktime_set( 0, (500 * 1000000)), HRTIMER_MODE_REL );
    } break;
    case 3:
    {
      hrtimer_start( &led_pulse_timer, ktime_set( 1, (500 * 1000000)), HRTIMER_MODE_REL );
    } break;
    case 4:
    {
      hrtimer_start( &led_pulse_timer, ktime_set( 4, (500 * 1000000)), HRTIMER_MODE_REL );
    } break;
  }

  mutex_unlock( &lm3533_i2c_access );

  return 0;
} /* End.. pattern_set_lm3533() */
#endif

/*************************************************************************
** Modify R/G/B LED's level can be controlled by android
**************************************************************************/
//static int blink_set_lm3533(int led_num, struct nled_setting* led, int level)
int blink_set_lm3533(int led_num, struct nled_setting* led, int level)
{
        // <<< 2016/01/18-youchihwang. Feature FP019344 Illumination
        struct i2c_client *client = lm3533_i2c_client;
        u32 low_time = 0, high_time = 0;
        u8  data_buf[2], patternvalue = 0;

#if defined( LED_TRACE_INFO )
        printk("[LED] %s: led_num = %d, level = %d\n", __func__, led_num, level );
#endif

        mutex_lock( &lm3533_i2c_access );
        lm3533_device_enable( TRUE );

        if (1000 >= led->blink_on_time)
                high_time = led->blink_on_time / 16;
        else if (10000 >= led->blink_on_time)
                high_time = ((led->blink_on_time - 1000) / 131) + 0x3C;
        else
                high_time = ((led->blink_on_time - 10000) / 524) + 0x7F;

        if (0xFF < high_time)
                high_time = 0xFF;


        if (1000 >= led->blink_off_time)
                low_time = led->blink_off_time / 16;
        else if (10000 >= led->blink_off_time)
                low_time = ((led->blink_off_time - 1000) / 131) + 0x3C;
        else
                low_time = ((led->blink_off_time - 10000) / 524) + 0x7F;


        if (0xFF < low_time)
                low_time = 0xFF;

        if (level > 0xFF)
                level = 0xFF;

        data_buf[0] = 0x28;
        i2c_master_send (client, (const char*) data_buf, 1);
        i2c_master_recv (client, (char*) &patternvalue, 1);
#if defined( LED_TRACE_INFO )
        printk("[LED] patternvalue = 0x%X\n", patternvalue);
#endif

        lm3533_write_reg (0x28, 0);  // disable all led pattern to re-blink at the same time

        if (led_num == MT65XX_LED_LM3533_L3) {
                
                //
                // set red led related parameters
                //
                lm3533_write_reg (0x90, 0);    // Delay time
                lm3533_write_reg (0x91, (u8) low_time);    // Low time
                lm3533_write_reg (0x92, (u8) high_time);    // High time
                lm3533_write_reg (0x93, 0);    // Low level brightness
                lm3533_write_reg (0x94, 0);    // Rise time
                lm3533_write_reg (0x95, 0);    // Fall time
                //lm3533_write_reg (0x44, level);

                patternvalue |= 0x10;
        }
        else if (led_num == MT65XX_LED_LM3533_L2) {
                
                //
                // set green led related parameters
                //
                lm3533_write_reg (0x80, 0);    // Delay time
                lm3533_write_reg (0x81, (u8) low_time);    // Low time
                lm3533_write_reg (0x82, (u8) high_time);    // High time
                lm3533_write_reg (0x83, 0);    // Low level brightness
                lm3533_write_reg (0x84, 0);    // Rise time
                lm3533_write_reg (0x85, 0);    // Fall time
                //lm3533_write_reg (0x43, level);

                patternvalue |= 0x04;
        }
        else if (led_num == MT65XX_LED_LM3533_L1) {
                
                //
                // set blue led related parameters
                //
                lm3533_write_reg (0x70, 0);    // Delay time
                lm3533_write_reg (0x71, (u8) low_time);    // Low time
                lm3533_write_reg (0x72, (u8) high_time);   // High time
                lm3533_write_reg (0x73, 0);    // Low level brightness
                lm3533_write_reg (0x74, 0);    // Rise time
                lm3533_write_reg (0x75, 0);    // Fall time
                //lm3533_write_reg (0x42, level);

                patternvalue |= 0x01;
        }

#if defined( LED_TRACE_INFO )
        printk ("[LED] led_num = 0x%X, low_time = 0x%X, high_time = 0x%X\n", led_num, low_time, high_time);
        printk ("[LED] level = 0x%X, patternvalue = 0x%X\n", level, patternvalue);
#endif
        lm3533_write_reg (0x28, patternvalue); /* Pattern enable */
        mutex_unlock (&lm3533_i2c_access);

        return 0;
        // >>> 2016/01/18-youchihwang. Feature FP019344 Illumination
} /* End.. blink_set_lm3533() */

/*************************************************************************
** Using SONY brightness curve
**************************************************************************/
#if defined( USING_MAP_TABLE_LM3533 )
//<2015/05/12-stevenchen, Adjust LCM backlight curve to linear
static int brightness_map_sony_lm3533(int level)
{
    int   vRet = 0;

//<2016/01/15-stevenchen, Follow Lavender backlight curve to save power
#if 1
	// <<< 2016/08/16-dangyiwang. change brightness mapping value
	#if 0
	    if( level == 0 )
	        vRet = 0;
	    else if( level < 10 )
	    // <<< 2016/04/12-dangyiwang. DMS09210789 [Survey Comments]It's very dazzling in the night though adjusting the brightness level to min.
	    // <<< 2016/04/12-dangyiwang. change lowest brightness value 3 to 1;
	        vRet = 1;
		else if( level< 150)
		   vRet = (level-10)*(104-1)/(149-10)+1;
	    // >>> 2016/04/12-dangyiwang. change lowest brightness value 3 to 1;	   
	    // >>> 2016/04/12-dangyiwang. DMS09210789 [Survey Comments]It's very dazzling in the night though adjusting the brightness level to min.
	    else
	        vRet = (level-150)*(255-105)/(255-150)+105;
	#endif
	
	if (0 == level){
		vRet = 0; // 2016/11/14-dangyiwang. add level 0 backlight value.
	}
	else{
		vRet = sony_backlight_map_ref[level-1];
	}
	// >>> 2016/08/16-dangyiwang. change brightness mapping value
#else
    if( level == 0 )
        vRet = 0;
    else if( level < 10 )
        vRet = 3;
    else
        vRet = (level-10)*(255-3)/(255-10)+3;
#endif
//>2016/01/15-stevenchen
    
    //printk("[Steven] backlight level =%d, lm3533 level = %d \n", level, vRet);

    return vRet;
} /* End.. brightness_map_sony_lm3533() */
//>2015/05/12-stevenchen
#endif /* End.. (brightness_map_sony_lm3533) */

/*************************************************************************
** Fix backlight issue
**************************************************************************/
int brightness_set_lm3533(int led_num, int level)
{
		// <<< 2016/01/18-youchihwang. Feature FP019344 Illumination
        struct i2c_client *client = lm3533_i2c_client;
        u8    enablevalue = 0;
        u8    patternvalue = 0;

#if defined (LED_TRACE_INFO)
        printk("[LED] %s: led_num = %d, level = %d\n", __func__, led_num, level );
#endif

        LEDS_DEBUG ("[LED]LM3533#%d:%d\n", led_num, level);
        mutex_lock (&lm3533_i2c_access);
        lm3533_device_enable (TRUE);
        lm3533_read_reg (0x27, &enablevalue);

        //<<< 2016/02/01-dangyiwang. 
        //DMS06703563, [Ar]RT-GST-<5929_LED>When DUT rings (Missed Calls),LED won't flash.
        //DMS06705156, [Ar]ST1-GST-<5929_Messaging>-<The LED light display red blink instead of white blink when received a message in sleep mode.>
        //DMS06705395, [Ar]ST1-GST-<5929_LED>-DUT has unread Email ,LED has no response.
        //DMS06709867, [Ar]ST-GST-<5929_LED>-<LED will blink quickly and irregularly when receive message/miss a call/in low battery.>
  	//DMS06710847, [Ar]Feat.T-<5929_LED>-<Feature FP019344(Illumination requirement) fail. The LED will not flicker when in sleep mode and make a miss call.>
        //Feature FP019344 Illumination
        //Led does not blink when DUT has miss call, unread message or mail.
        //Modify the following code into comment, and move to next comment description about 2016/02/01-dangyiwang....
	//because of led was reflash when due change the backlight brightness
        
        //lm3533_write_reg (0x27, (enablevalue & 0x03)); // disable all led brightness to re-blink at the same time

	//Feature FP019344 Illumination
  	//Led does not blink when DUT has miss call, unread message or mail.
        //Modify the following code into comment, and move to next comment description about 2016/02/01-dangyiwang....
	//because of led was reflash when due change the backlight brightness
	//DMS06703563, [Ar]RT-GST-<5929_LED>When DUT rings (Missed Calls),LED won't flash.
        //DMS06705156, [Ar]ST1-GST-<5929_Messaging>-<The LED light display red blink instead of white blink when received a message in sleep mode.>
        //DMS06705395, [Ar]ST1-GST-<5929_LED>-DUT has unread Email ,LED has no response.
        //DMS06709867, [Ar]ST-GST-<5929_LED>-<LED will blink quickly and irregularly when receive message/miss a call/in low battery.>
  	//DMS06710847, [Ar]Feat.T-<5929_LED>-<Feature FP019344(Illumination requirement) fail. The LED will not flicker when in sleep mode and make a miss call.>
        //>>>2016/02/01-dangyiwang.  
        lm3533_read_reg (0x28, &patternvalue);
       
        if (client == NULL) {
                LEDS_DEBUG ("i2c client is null!!\n");
                mutex_unlock (&lm3533_i2c_access);
                return 0;
        }

        if ((led_num == MT65XX_LED_LM3533_L1) || (led_num == MT65XX_LED_LM3533_L2) || (led_num == MT65XX_LED_LM3533_L3)) {

                //<<< 2016/02/01-dangyiwang. 
                //DMS06703563, [Ar]RT-GST-<5929_LED>When DUT rings (Missed Calls),LED won't flash.
                //DMS06705156, [Ar]ST1-GST-<5929_Messaging>-<The LED light display red blink instead of white blink when received a message in sleep mode.>
                //DMS06705395, [Ar]ST1-GST-<5929_LED>-DUT has unread Email ,LED has no response.
                //DMS06709867, [Ar]ST-GST-<5929_LED>-<LED will blink quickly and irregularly when receive message/miss a call/in low battery.>
          	//DMS06710847, [Ar]Feat.T-<5929_LED>-<Feature FP019344(Illumination requirement) fail. The LED will not flicker when in sleep mode and make a miss call.>
          	//Feature FP019344 Illumination      
          	//Led does not blink when DUT has miss call, unread message or mail.
                //Modify the following code.  
                //because of led was reflash when due change the backlight brightness

                lm3533_write_reg (0x27, (enablevalue & 0x03));// disable all led brightness to re-blink at the same time

                //Feature FP019344 Illumination
          	//Led does not blink when DUT has miss call, unread message or mail.
            	//Modify the following code.  
                //because of led was reflash when due change the backlight brightness
            	//DMS06703563, [Ar]RT-GST-<5929_LED>When DUT rings (Missed Calls),LED won't flash.
                //DMS06705156, [Ar]ST1-GST-<5929_Messaging>-<The LED light display red blink instead of white blink when received a message in sleep mode.>
                //DMS06705395, [Ar]ST1-GST-<5929_LED>-DUT has unread Email ,LED has no response.
                //DMS06709867, [Ar]ST-GST-<5929_LED>-<LED will blink quickly and irregularly when receive message/miss a call/in low battery.>
          	//DMS06710847, [Ar]Feat.T-<5929_LED>-<Feature FP019344(Illumination requirement) fail. The LED will not flicker when in sleep mode and make a miss call.>
                //>>>2016/02/01-dangyiwang.  
                if (level == 0) {
                        enablevalue &= ~(1 << led_num);
                        patternvalue &= ~(0x3 << ((led_num - 2) << 1));
                        lm3533_write_reg (0x28, patternvalue);
                }
                else {
	                if (level > 0xFF)
  	                        level = 0xFF;
                        enablevalue |= (1 << led_num);
                }
                lm3533_write_reg ((0x40 + led_num), level); /* B/G/R led */
        }
        else if ((led_num == MT65XX_LED_LM3533_H1) || (led_num == MT65XX_LED_LM3533_H2)) {
                
#if defined (USING_MAP_TABLE_LM3533)
        level = brightness_map_sony_lm3533 (level);
#else
                if (level >= 0xFF)
                        level = 0xFF;
                else if (level == 0)
                        level = 0;
                else
                        level = 125 + (level >> 1);
#endif

//<2015/05/12-stevenchen, Adjust LCM backlight curve to linear
    //<2015/02/13-youchihwang, adjust LCM backlight lowest current
    //if (level == 20)
    //    level = 3;
    //>2015/02/13-youchihwang, adjust LCM backlight lowest current
//>2015/05/12-stevenchen
                if (level == 0)
                        enablevalue &= 0xFC;
                else
                        enablevalue |= 0x03;

                lm3533_write_reg (0x40, level);  /* Bank A */
                lm3533_write_reg (0x41, level);  /* Bank B */
        }

    
        lm3533_write_reg (0x27, enablevalue);

#if defined( LED_TRACE_INFO )
        printk("[LED]LM3533 2 enablevalue = 0x%02X\n", enablevalue );
#else
        LEDS_DEBUG("[LED]LM3533 2 enablevalue = 0x%x\n", enablevalue );
#endif

        mutex_unlock (&lm3533_i2c_access);

        return 0;
		// >>> 2016/01/18-youchihwang. Feature FP019344 Illumination
} /* End.. brightness_set_lm3533() */

/*************************************************************************
**
**************************************************************************/
static int lm3533_reg_init(struct lm3533 *lm3533)
{
#if defined( LED_TRACE_INFO )
  printk("[LED] %s start...\n", __func__ );
#endif

  /* LCM backlight */
  // <<< 2016/01/06-youchihwang. Setting Notification LED & Backlight LED Full-Scale current
  lm3533_write_reg( 0x1F, 0x12 );  // 0x13:20.2mA, 0x12:19.4mA   /* Set bank A full-scale current (5mA ~ 29.8mA) */
  // >>> 2016/01/06-youchihwang. Setting Notification LED & Backlight LED Full-Scale current

#if defined( USING_MAP_TABLE_LM3533 )
  lm3533_write_reg( 0x1A, 0x0A );  /* Bank A & Bank B => brightness mode, linear mode */
#endif
  lm3533_write_reg( 0x40, 0xA0 ); /* Brightness value for bank A */
  // <<< 2016/01/06-youchihwang. Setting Notification LED & Backlight LED Full-Scale current
  lm3533_write_reg( 0x20, 0x13 );  // 0x13:20.2mA, 0x12:19.4mA   /* Set bank B full-scale current (5mA ~ 29.8mA) */
  // >>> 2016/01/06-youchihwang. Setting Notification LED & Backlight LED Full-Scale current
  lm3533_write_reg( 0x41, 0xA0 ); /* Brightness value for bank B */
  lm3533_write_reg( 0x2C, 0x0D ); /* Boost OVP 0x0D:32V, 0x0B:24V */

  /* Blue LED */
  // <<< 2016/04/27-dangyiwang. Setting Notification LED & Backlight LED Full-Scale current
  lm3533_write_reg( 0x21, 0x12 );  // 0x13:20.2mA, 0x12:19.4mA   /* Set bank C full-scale current (5mA ~ 29.8mA) */
  // >>> 2016/04/27-dangyiwang. Setting Notification LED & Backlight LED Full-Scale current
  lm3533_write_reg( 0x1B, 0x04 ); /* Bank C => disable pattern generator, brightness mode, linear mode */
  lm3533_write_reg( 0x42, 0xFF ); /* Brightness value for bank C */

  /* Green LED */
  // <<< 2016/04/27-dangyiwang. Setting Notification LED & Backlight LED Full-Scale current
  lm3533_write_reg( 0x22, 0x12 );  // 0x13:20.2mA, 0x12:19.4mA   /* Set bank D full-scale current (5mA ~ 29.8mA) */
  // >>> 2016/04/27-dangyiwang. Setting Notification LED & Backlight LED Full-Scale current
  lm3533_write_reg( 0x1C, 0x04 ); /* Bank D => disable pattern generator, brightness mode, linear mode */
  lm3533_write_reg( 0x43, 0xFF ); /* Brightness value for bank D */

  /* Red LED */
  // <<< 2016/04/27-dangyiwang. Setting Notification LED & Backlight LED Full-Scale current
  lm3533_write_reg( 0x23, 0x12 );  // 0x13:20.2mA, 0x12:19.4mA   /* Set bank E full-scale current (5mA ~ 29.8mA) */
  // >>> 2016/04/27-dangyiwang. Setting Notification LED & Backlight LED Full-Scale current
  lm3533_write_reg( 0x1D, 0x04 ); /* bank E => disable pattern generator, brightness mode, linear mode */
  lm3533_write_reg( 0x44, 0xFF ); /* brightness value for bank E */

  // <<< 2015/12/23-youchihwang. Enabling CABC (PWM Input for Content Adjustable Brightness Control)
  lm3533_write_reg (0x14, 0x1);
  lm3533_write_reg (0x15, 0x1);
  // >>> 2015/12/23-youchihwang. Enabling CABC (PWM Input for Content Adjustable Brightness Control)

//#if defined( MTK_KERNEL_POWER_OFF_CHARGING )
//  if(( KERNEL_POWER_OFF_CHARGING_BOOT != get_boot_mode())
//  && ( LOW_POWER_OFF_CHARGING_BOOT != get_boot_mode()))
//#endif
//  {
    /* Blue LED */
//    lm3533_write_reg( 0x71, 0xFF ); /* low-time */
//    lm3533_write_reg( 0x74, 0x02 ); /* rise-time */
//    lm3533_write_reg( 0x75, 0x03 ); /* fall-time */

    /* Green LED */
//    lm3533_write_reg( 0x81, 0xFF ); /* low-time */
//    lm3533_write_reg( 0x84, 0x02 ); /* rise-time */
//    lm3533_write_reg( 0x85, 0x03 ); /* fall-time */

    /* Red LED */
//    lm3533_write_reg( 0x91, 0xFF ); /* low-time */
//    lm3533_write_reg( 0x94, 0x02 ); /* rise-time */
//    lm3533_write_reg( 0x95, 0x03 ); /* fall-time */

//    lm3533_write_reg( 0x28, 0x15 );

    //<2014/12/19-youchihwang, off led when off mode charging
//    lm3533_write_reg (0x27, 0x03 );
    //<2014/12/19-youchihwang, off led when off mode charging
//  }
  
  return 0;
} /* End.. lm3533_device_init() */

/*************************************************************************
**
**************************************************************************/
//<2015/06/05-youchihwang, add i2c address probe
/**
 * lm3533_i2c_auto_probe - auto probe the i2c addresses
 * @client: i2c device
 *
 * Auto probing the i2c address of the array lm3533_i2c_auto_probe_address.
 */
static int lm3533_i2c_auto_probe(struct i2c_client *client)
{
  int ret                  = 0; //return value
  u8 chip_register_address = 0;
  u8 temp                  = 0;

  for (temp = 0; temp < LM3533_I2C_PROBE_ADDRESS_AMOUNT; temp++) {
    client->addr = lm3533_i2c_auto_probe_address[temp];
#if defined( LED_TRACE_INFO )
    printk("[LED] auto probe address : 0x%x\n", client->addr);
#endif

    //test lm3533 brightness register A 0x40 whether can be accessed
    chip_register_address = 0x40;
    ret = i2c_master_send(client, (const char*)&chip_register_address, 1);
#if defined( LED_TRACE_INFO )
    printk("[LED] auto probe address 0x%x result : 0x%x\n", client->addr, ret);
#endif
    if (ret > 0) {
	  #if defined( LED_TRACE_INFO )
      printk("[LED] Valid lm3533 i2c address : 0x%x\n", client->addr);
      printk("[LED] 0x36 : LM3533TME-40  I2C Address \n");
      printk("[LED] 0x38 : LM3533TME-40A I2C Address \n");
	  #endif
      break;
    }
  }

  return ret;
}
//>2015/06/05-youchihwang
/*************************************************************************
**
**************************************************************************/
static int lm3533_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
  struct lm3533   *lm3533 = NULL;
  struct i2c_client *new_client;
  int   ret;
#if defined( LED_TRACE_INFO )
  printk("[LED] %s ... addr:0x%x\n", __func__, i2c->addr );
#endif
  lm3533 = kmalloc( sizeof(struct lm3533), GFP_KERNEL );
  if( !lm3533 )
          return -ENOMEM;

  #if 1
    lm3533_device_enable( TRUE );
  #else
    mt_set_gpio_mode( GPIO_LCM_LED_EN, GPIO_LCM_LED_EN_M_GPIO );
    mt_set_gpio_dir( GPIO_LCM_LED_EN, GPIO_DIR_OUT );
    mt_set_gpio_out( GPIO_LCM_LED_EN, GPIO_OUT_ZERO );
    mdelay( 1 );
    mt_set_gpio_out( GPIO_LCM_LED_EN, GPIO_OUT_ONE );
    mdelay(1);
  #endif          

  //<2015/06/05-youchihwang, add i2c address probe
  ret = lm3533_i2c_auto_probe(i2c);
  if (!(ret > 0)) {
    printk("[LED] There is no valid lm3533 \n");
  }  
  //>2015/06/05-youchihwang, add i2c address probe

  lm3533->dev = &i2c->dev;
  lm3533->i2c = i2c;  //obj->client = client;
  new_client = lm3533->i2c;
  
  i2c_set_clientdata( new_client, lm3533 );
  lm3533_i2c_client = new_client;

  /*
  if( ret = lm3533_create_attr( &mt65xx_leds_driver.driver ))
  {
    LEDS_DEBUG("create attribute ret = %d\n", ret);
  }
  */

  /* [5860] Add VALUE_BUTTON_3 and VALUE_PATTERN_2 for LED illumination */
  led_pulse_workqueue = create_singlethread_workqueue( "led_pulse" );
  INIT_WORK( &led_pulse_work, led_pulse_work_callback );
  hrtimer_init( &led_pulse_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
  led_pulse_timer.function = led_pulse_timer_func;

  ret = lm3533_reg_init( lm3533 );
  if( ret )
  {
    kfree( lm3533 );
    return ret;
  }

  return 0;
} /* End.. lm3533_i2c_probe() */

/*************************************************************************
**
**************************************************************************/
static int lm3533_i2c_remove(struct i2c_client *i2c)
{
  struct lm3533 *lm3533 = i2c_get_clientdata( i2c );

/* [5860] Show a white LED short pulse when user turns phone on */
  lm3533_write_reg( 0x28, 0x00 );
  lm3533_write_reg( 0x27, 0x00 );

  lm3533_i2c_client = NULL;
  i2c_unregister_device( i2c );
  kfree( lm3533 );

  lm3533_device_enable( FALSE );
  return 0;
} /* End.. lm3533_i2c_remove() */

/*************************************************************************
**
**************************************************************************/
static ssize_t store_lm3533(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
  lm3533_led_data * ledData = (lm3533_led_data *)buf;

  LEDS_DEBUG("[LED] size=%d, pattern_id=%d  , red=%x,  green=%x , blue=%x\n",
        (int)size,ledData->pattern_id, ledData->red, ledData->green,  ledData->blue);
    // <<<<<----- 2015/04/28-youchihwang
    // DMS06418317--[Ar]RT-GST-<5919_Notification>-<The phone Notification LED light is blue when receiving a missed call/Email/Message.>
    // ALPS02045680
    pattern_id = ledData->pattern_id;        
    //pattern_set_lm3533(NULL, ledData->pattern_id, ledData->flash_mode ,ledData->red, ledData->green, ledData->blue);
    // ----->>>>> 2015/04/28-youchihwang
    // DMS06418317--[Ar]RT-GST-<5919_Notification>-<The phone Notification LED light is blue when receiving a missed call/Email/Message.>
    // ALPS02045680
//pattern_set_lm3533(NULL, ledData->pattern_id, ledData->red, ledData->green, ledData->blue);

  return size;
} /* End.. store_lm3533() */
static DEVICE_ATTR( lm3533, 0664, NULL, store_lm3533 );

/*************************************************************************
** [xssm]Power saving in LED
**************************************************************************/
static ssize_t show_psnotification(struct device *dev,struct device_attribute *attr, char *buf)
{
  LEDS_DEBUG("show_psnotification=%d\n", notification_value_enable );
  return sprintf(buf, "%u", notification_value_enable);
}

/*************************************************************************
**
**************************************************************************/
static ssize_t show_psattention(struct device *dev,struct device_attribute *attr, char *buf)
{
  LEDS_DEBUG("show_psnotification=%d \n", attention_value_enable );
  return sprintf(buf, "%u", attention_value_enable);
}

/*************************************************************************
**
**************************************************************************/
static ssize_t store_psnotification(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
  char *pvalue = NULL;
  
  notification_value_enable = simple_strtoul( buf, &pvalue, 10 );
  LEDS_DEBUG("store_psnotification=%s", buf );
  return size;
}

/*************************************************************************
**
**************************************************************************/
static ssize_t store_psattention(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
  char *pvalue = NULL;

  attention_value_enable = simple_strtoul( buf, &pvalue, 10 );
  LEDS_DEBUG("store_psattention=%s", buf );
  return size;
}
static DEVICE_ATTR( psnotification, 0664, show_psnotification, store_psnotification );
static DEVICE_ATTR( psattention, 0664, show_psattention, store_psattention );

/*************************************************************************
**
**************************************************************************/
static ssize_t store_lm3533_reg_info(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
  char *pvalue = NULL;
  unsigned int reg_value = 0;
  unsigned int reg_address = 0;

  LEDS_DEBUG("store_lm3533_reg_info:size:%d,address:0x%s\n", (int)size,buf );

  if( buf != NULL && size != 0 )
  {
    reg_address = simple_strtoul( buf,&pvalue, 16 );

    LEDS_DEBUG("store_lm3533_reg_info:register:0x%x\n", reg_address );

    if(( reg_address >= 0x10 ) && ( reg_address <= 0xB2 ))
    {
      if( *pvalue && ( *pvalue == '#' ))
      {
        reg_value = simple_strtoul(( pvalue + 1), NULL, 16 );
        lm3533_write_reg( reg_address, reg_value );
        LEDS_DEBUG("set_lm3533_reg_info register:[0x%x]=0x%x\n", reg_address, reg_value );
      }
      else if( *pvalue && ( *pvalue == '@' ))
      {
        lm3533_read_reg( (u8)reg_address, (u8 *)&reg_value );
        LEDS_DEBUG("get_lm3533_reg_info register:[0x%x]=0x%x\n",reg_address,reg_value);
      }
    }
  }

  return size;
}

/*************************************************************************
**
**************************************************************************/
static ssize_t show_lm3533_reg_info(struct device *dev, struct device_attribute *attr, char *buf)
{
  u8 reg_value    = 0;
  u8 reg_address  = 0x27;

  LEDS_DEBUG("show_lm3533_reg_info\n");

  lm3533_read_reg( reg_address, &reg_value );
  LEDS_DEBUG("get_lm3533_reg_info register:[0x%02X]=0x%02X\n", reg_address, reg_value );

  return sprintf( buf, "reg(0x%x)=0x%x\n", reg_address, reg_value );
}
static DEVICE_ATTR( lm3533_reg_info, 0664, show_lm3533_reg_info, store_lm3533_reg_info );


/*************************************************************************
**
**************************************************************************/
int lm3533_create_file(int num)
{
  int vRet = 0;

#if defined( LED_TRACE_INFO )
  printk("[LED] %s ...\n", __func__ );
#endif

  vRet = device_create_file( g_leds_data[num]->cdev.dev, &dev_attr_lm3533 );
  if( vRet )
  {
    LEDS_DEBUG("[LED]device_create_file lm3533 fail!\n");
  }	

  /* Power saving in LED */
  vRet = device_create_file( g_leds_data[num]->cdev.dev, &dev_attr_psnotification );
  if( vRet )
  {
    LEDS_DEBUG("[LED]device_create_file psnotification fail!\n");
  }	

  vRet = device_create_file( g_leds_data[num]->cdev.dev, &dev_attr_psattention );
  if( vRet )
  {
    LEDS_DEBUG("[LED]device_create_file psnotification fail!\n");
  }	

  /* Debugging code by adb command */
  vRet = device_create_file( g_leds_data[num]->cdev.dev, &dev_attr_lm3533_reg_info );
  if( vRet )
  {
    LEDS_DEBUG("[LED]device_create_file lm3533_reg_info fail!\n");
  }	

  return vRet;
}

//int lm3533_i2c_init(int debug_log)
//{
//  printk("[LED] %s ......\n", __func__ );
//
//  lm3533_debug_enable = debug_log;
//
//  i2c_register_board_info( I2C_LM3533_BUS, &i2c_lm3533, 1 );
//  if( i2c_add_driver( &lm3533_i2c_driver ))
//  {
//    printk("[LED] LM3533 add I2C driver error.\n" );
//    return -1;
//  }
//  else
//  {
//    printk("[LED] LM3533 add I2C driver success.\n");
//  }
//  return 0;
//}

/*************************************************************************
**
**************************************************************************/
void lm3533_i2c_exit(void)
{
  #if defined( LED_TRACE_INFO )
  printk("[LED] %s ......\n", __func__ );
  #endif
  i2c_del_driver( &lm3533_i2c_driver );
}

static int leds_lm3533_probe(struct platform_device *pdev)
{
  #if defined( LED_TRACE_INFO )
  printk("[LED]leds_lm3533_probe start dev name:%s\n", dev_name(&pdev->dev));
  #endif
  pinctrl = devm_pinctrl_get(&pdev->dev);
  if (IS_ERR(pinctrl)) {
    printk("[LED]pinctrl error\n");	      
    return -1;
  }

  lm3533_rst_output0 = pinctrl_lookup_state(pinctrl, "lm3533_gpio_rst_low");
  if (IS_ERR(lm3533_rst_output0)) {
    printk("[LED]could not get rst 0 pinstate\n");
    return -1;
  }

  lm3533_rst_output1 = pinctrl_lookup_state(pinctrl, "lm3533_gpio_rst_high");     
  if (IS_ERR(lm3533_rst_output1)) {
    printk("[LED]could not get rst 1 pinstate\n");
    return -1;
  }

  i2c_register_board_info( I2C_LM3533_BUS, &i2c_lm3533, 1 );

  if( i2c_add_driver( &lm3533_i2c_driver )) {
    printk("[LED] lm3533 add I2C driver error\n");
    return -1;
  } else {
    #if defined( LED_TRACE_INFO )
    printk("[LED] lm3533 add I2C driver success\n");
	#endif
  }
  #if defined( LED_TRACE_INFO )
  printk("[LED]leds_lm3533_probe end dev name:%s\n", dev_name(&pdev->dev));
  #endif
	return 0;
}

static const struct of_device_id lm3533_device_of_match[] = {
	{.compatible = "mediatek,lm3533-device"},
	{},
};

static struct platform_driver leds_lm3533_driver = {
	.driver = {
		   .name = "lm3533-device",
		   .owner = THIS_MODULE,
       .of_match_table = lm3533_device_of_match,
		   },
	.probe = leds_lm3533_probe,
};

static int __init leds_lm3533_init(void)
{
	int ret;
  #if defined( LED_TRACE_INFO )
  printk("[LED]%s\n", __func__);
  #endif
	ret = platform_driver_register(&leds_lm3533_driver);

	if (ret) {
		printk("leds_lm3533_init:drv:E%d\n", ret);
		return ret;
	}
 
  return 0;
}

/*************************************************************************
**
**************************************************************************/
static void __exit leds_lm3533_exit(void)
{
  LEDS_DEBUG("[LED]%s\n", __func__ );
  i2c_del_driver( &lm3533_i2c_driver );
}

//module_param( lm3533_debug_enable, int, 0644 );
module_init( leds_lm3533_init );
module_exit( leds_lm3533_exit );
//module_i2c_driver(lm3533_i2c_driver);

MODULE_AUTHOR("ARIMA Inc.");
MODULE_DESCRIPTION("LED driver for TI LM3533 chip");
MODULE_LICENSE("GPL");
MODULE_ALIAS("leds-lm3533");

/*************************************************************************
**
**************************************************************************/
#undef LED_LM3533_C
#endif /* End.. !(LED_LM3533_C) */
