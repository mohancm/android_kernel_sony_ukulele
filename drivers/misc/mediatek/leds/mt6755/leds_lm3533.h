#ifndef LED_LM3533_H
#define LED_LM3533_H

    #include <linux/module.h>
    #include <linux/platform_device.h>
    #include <linux/delay.h>
    #include <linux/string.h>
    #include <linux/ctype.h>
    #include <linux/leds.h>
    #include <linux/of.h>
#if defined( LED_LM3533_C )
    #include  <linux/slab.h>
    #include  <linux/gpio.h>
    #include  <linux/i2c.h>

    #include  "leds_sw.h"
    #include  "lm3533.h"
#endif /* End.. (LED_LM3533_C) */

/*******************************************************************************
** Macro-define
********************************************************************************/
#if defined( GLOBAL )
    #undef  GLOBAL
#endif

#if defined( LED_LM3533_C )
    #define   I2C_LM3533_DEV_ADDR     (0x36)
    #define   I2C_LM3533_SLAVE_ADDR   (I2C_LM3533_DEV_ADDR<<1)  /* 0x6C */
    #define   I2C_LM3533_BUS          1 //I2C1
	
//Steven start
    #define   USING_MAP_TABLE_LM3533
//Steven end

  #if !defined( FALSE )
    #define   FALSE       0
  #endif
  #if !defined( TRUE )
    #define   TRUE        1
  #endif

    #define   GLOBAL
#else
    #define   GLOBAL      extern
#endif /* End.. (LED_LM3533_C) */

#if defined( LED_LM3533_EX_INIT )
    #undef    LED_LM3533_EX_INIT
#endif
    #define   LED_LM3533_EX_INIT

#if defined( LED_TRACE_INFO )
    #undef   LED_TRACE_INFO
#endif
  //#define  LED_TRACE_INFO

#if defined( LED_DEBUG_INFO )
    #undef   LED_DEBUG_INFO
#endif
  //#define  LED_DEBUG_INFO

/*******************************************************************************
**
********************************************************************************/
#if defined( LED_LM3533_C )
/* Connect native(lights.c) to driver(leds.c) */
    typedef struct
    {
        int   pattern_id;
        int   flash_mode;
        u8    red;
        u8    green;
        u8    blue;
    } lm3533_led_data;

/* Add LED fading function */
    struct Cmd_config
    {
        unsigned char   cmd;
        unsigned char   data;
    };

    struct LED_pattern_table
    {
        unsigned char       count;
        struct Cmd_config   config_data[30];
    };

#endif /* End.. (LED_LM3533_C) */


/*******************************************************************************
**
********************************************************************************/
#if defined( LED_LM3533_C )
    static DEFINE_MUTEX(lm3533_i2c_access);
    //static struct platform_driver   mt65xx_leds_driver;
    static struct i2c_client      * lm3533_i2c_client = NULL;
    static struct i2c_board_info __initdata   i2c_lm3533 = { I2C_BOARD_INFO("lm3533", (I2C_LM3533_SLAVE_ADDR>>1))};

    static const struct i2c_device_id lm3533_i2c_ids[] =
    {
      { "lm3533", 0 },
      { },
    };

    static struct hrtimer             led_pulse_timer;
    static struct work_struct         led_pulse_work;
    static struct workqueue_struct  * led_pulse_workqueue = NULL;
  //static struct mt65xx_led_data     last_leds_data[MT65XX_LED_TYPE_TOTAL];

    static bool   is_keep_light = false;
    static bool   pattern_enable = false;
    // <<< 2016/01/18-youchihwang. Feature FP019344 Illumination
    //static u8     keep_light_color = 0x00;
    // >>> 2016/01/18-youchihwang. Feature FP019344 Illumination

    unsigned int      notification_value_enable = 1;
    unsigned int      attention_value_enable = 1;

/* Add VALUE_BUTTON_3 and VALUE_PATTERN_2 for LED illumination */
    static const struct LED_pattern_table   pattern_data[] = 
    {
    /* Continous short pulse with fade. Fade in 524ms, Fade out 1049ms */
      { 12, {{0x71,0x00},{0x72,0x00},{0x74,0x02},{0x75,0x03},{0x81,0x00},{0x82,0x00},
             {0x84,0x02},{0x85,0x03},{0x91,0x00},{0x92,0x00},{0x94,0x02},{0x95,0x03}}},
    /* Continous short pulse with fade, Fade in 1049ms, Fade out 2097ms */
      { 12, {{0x71,0x00},{0x72,0x00},{0x74,0x03},{0x75,0x04},{0x81,0x00},{0x82,0x00},
             {0x84,0x03},{0x85,0x04},{0x91,0x00},{0x92,0x00},{0x94,0x03},{0x95,0x04}}},
    /* Continousy lit. Fades out with display. Fade in 524ms, lit until display is off, fade out 1049ms */
      { 12, {{0x71,0x00},{0x72,0x3C},{0x74,0x02},{0x75,0x03},{0x81,0x00},{0x82,0x3C},
             {0x84,0x02},{0x85,0x03},{0x91,0x00},{0x92,0x3C},{0x94,0x02},{0x95,0x03}}},
    /* One short pulse with fade. Fade in 524ms, Fade out 1049ms */
      { 12, {{0x71,0x3C},{0x72,0x00},{0x74,0x02},{0x75,0x03},{0x81,0x3C},{0x82,0x00},
             {0x84,0x02},{0x85,0x03},{0x91,0x3C},{0x92,0x00},{0x94,0x02},{0x95,0x03}}},
    /* One long pulse with fade. Fade in 2097ms, Fade out 4194ms */
      { 12, {{0x71,0x3C},{0x72,0x00},{0x74,0x04},{0x75,0x05},{0x81,0x3C},{0x82,0x00},
             {0x84,0x04},{0x85,0x05},{0x91,0x3C},{0x92,0x00},{0x94,0x04},{0x95,0x05}}}
    };


    extern struct mt65xx_led_data *g_leds_data[MT65XX_LED_TYPE_TOTAL];
    //extern boot_reason_t  g_boot_reason;
#endif /* End.. (LED_LM3533_C) */


/*******************************************************************************
**
********************************************************************************/
#if defined( LED_LM3533_C )
    static int  lm3533_write_reg(u8 reg, u8 writeData);
    static int  lm3533_read_reg(u8 reg, u8 *returnData);
    static void lm3533_device_enable(int enable);

  //static int  pattern_set_lm3533(struct cust_mt65xx_led *cust, int pattern_id, int flash_mode, u8 red_data, u8 green_data, u8 blue_data);
  //static int  blink_set_lm3533(int led_num, struct nled_setting* led, int level);
  //static int  brightness_set_lm3533(int led_num, int level);
  //static int  lm3533_create_file(int num);

    static int  lm3533_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id);
    static int  lm3533_i2c_remove(struct i2c_client *i2c);

  #if defined( USING_MAP_TABLE_LM3533 )
    static int  brightness_map_sony_lm3533(int level);
  #endif
    static int  lm3533_reg_init(struct lm3533 *lm3533);

    void  led_switch(void);
    void  led_pulse_work_callback(struct work_struct *work);
#else

#endif /* End.. (LED_LM3533_C) */

    GLOBAL int  blink_set_lm3533(int led_num, struct nled_setting* led, int level);
    GLOBAL int  brightness_set_lm3533(int led_num, int level);
    GLOBAL int  lm3533_create_file(int num);
  #if defined( LED_LM3533_EX_INIT )
    GLOBAL int  lm3533_i2c_init(int debug_log);
    GLOBAL void lm3533_i2c_exit(void);
  #endif

#if defined( GLOBAL )
    #undef  GLOBAL
#endif

#endif /* End.. !(LED_LM3533_H) */
