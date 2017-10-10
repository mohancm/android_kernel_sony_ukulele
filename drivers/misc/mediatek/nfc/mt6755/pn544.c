/*
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
//>2016/01/19-JackHu, Remove NFC kernel log for performace
//#define DEBUG
//#define NFC_DEBUG
//>2016/01/19-JackHu
/*****************************************************************************
** Include
******************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/device.h>
#include <linux/pn544.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
//>2016/02/04-JackHu--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
#include <linux/wakelock.h>
//>2016/02/04-JackHu--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode

/*****************************************************************************
** Macro-Define
******************************************************************************/
#define NFC_CLIENT_TIMING 400
/*========================================================
** For information print out
**========================================================*/
#define   I2C_DMA_USAGE
#if defined( I2C_DMA_USAGE )
    #include  <linux/dma-mapping.h>
#endif

#define   PN544_DEV_ADDR            (0x28)  //(0x2B)  /* PN547C2: 0x2B, PN547: 0x28*/
#define   PN544_SLAVE_ADDR          (PN544_DEV_ADDR<<1)
#define   NFC_DEV_NAME              "pn544"
#define   I2C_ID_NAME               "pn544"
#define   MAX_BUFFER_SIZE           512
#define   ENABLE_DELAY              50
#define   DISABLE_DELAY             70
#define   VEN_ENABALE               1
#define   VEN_DISABALE              0

//>2016/02/01-JackHu, Modify timing to improve meta test not stable issue
#define ENABLE_DELAY  50
#define DISABLE_DELAY  70
//>2016/02/01-JackHu

typedef struct st_pn544_dev
{
    wait_queue_head_t   read_wq;
    struct mutex        read_mutex;
    struct i2c_client * client;
    struct miscdevice   pn544_device;
    //>2016/02/04-JackHu--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
    struct wake_lock wake_lock;
    //>2016/02/04-JackHu--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
    unsigned int        ven_gpio;
    unsigned int        firm_gpio;
    unsigned int        irq_gpio;
    bool                irq_enabled;
    spinlock_t          irq_enabled_lock;
} PN544_DEV;

/* For DMA */
#if defined( I2C_DMA_USAGE )
static char   * I2CDMAWriteBuf = NULL;
static uintptr_t   I2CDMAWriteBuf_pa;  // = NULL;
static char   * I2CDMAReadBuf = NULL;
static uintptr_t  I2CDMAReadBuf_pa;   // = NULL;
#endif /* End.. (I2C_DMA_USAGE) */

struct pn544_i2c_platform_data  pn544_pdata;
static PN544_DEV  * g_pn544_dev = NULL;
static u32 nfc_irq;
struct platform_device *nfc_plt_dev = NULL;
struct pinctrl *gpctrl = NULL;
struct pinctrl_state *st_ven_h = NULL;
struct pinctrl_state *st_ven_l = NULL;
struct pinctrl_state *st_rst_h = NULL;
struct pinctrl_state *st_rst_l = NULL;
struct pinctrl_state *st_eint_h = NULL;
struct pinctrl_state *st_eint_l = NULL;
struct pinctrl_state *st_irq_init = NULL;

static int mt_nfc_pinctrl_select(struct pinctrl *p, struct pinctrl_state *s);
/*****************************************************************************
**
******************************************************************************/
enum {
	MTK_NFC_GPIO_DIR_IN = 0x0,
	MTK_NFC_GPIO_DIR_OUT,
	MTK_NFC_GPIO_DIR_INVALID,
};

static int mt_nfc_get_gpio_dir(int gpio_num)
{
	if (gpio_num == pn544_pdata.irq_gpio) {
		return MTK_NFC_GPIO_DIR_IN;	/* input */

	} else if ((gpio_num == pn544_pdata.ven_gpio) ||
		   (gpio_num == pn544_pdata.firm_gpio) ) {
		return MTK_NFC_GPIO_DIR_OUT;	/* output */

	} else {
		return MTK_NFC_GPIO_DIR_INVALID;

	}
}
static int mt_nfc_get_gpio_value(int gpio_num)
{
	int value = 0;
	if (mt_nfc_get_gpio_dir(gpio_num) != MTK_NFC_GPIO_DIR_INVALID) {
#if !defined(CONFIG_MTK_LEGACY)
		value = __gpio_get_value(gpio_num);
#else
		value = mt_get_gpio_in(gpio_num);
#endif
	}
	return value;
}

static void pn544_disable_irq(PN544_DEV *pn544_dev)
{
    unsigned long   flags;
    #if defined( DEBUG )
    printk("%s +++\n", __func__);
    #endif
    spin_lock_irqsave( &pn544_dev->irq_enabled_lock, flags );
    if( pn544_dev->irq_enabled )
    {
        #if defined( DEBUG )
        printk("%s: pn544_dev->irq_enabled\n", __func__);
        #endif
	disable_irq_nosync(nfc_irq);
        pn544_dev->irq_enabled = false;
    }
    spin_unlock_irqrestore( &pn544_dev->irq_enabled_lock, flags );
    #if defined( DEBUG )
    printk("%s ---\n", __func__);
    #endif
}

/*****************************************************************************
**
******************************************************************************/
static void pn544_enable_irq(PN544_DEV *pn544_dev)
{
    unsigned long   flags;
    #if defined( DEBUG )
    printk("%s +++\n", __func__);
    #endif
    spin_lock_irqsave( &pn544_dev->irq_enabled_lock, flags );
    if( !( pn544_dev->irq_enabled ))
    {
        #if defined( DEBUG )
        printk("%s: !( pn544_dev->irq_enabled )\n", __func__);
        #endif
	    enable_irq(nfc_irq);
        pn544_dev->irq_enabled = true;
    }
    spin_unlock_irqrestore( &pn544_dev->irq_enabled_lock, flags );
    #if defined( DEBUG )
    printk("%s ---\n", __func__);
    #endif
}

/*****************************************************************************
**
******************************************************************************/

static irqreturn_t pn544_dev_irq_handler(int irq, void *dev_id)
{
    PN544_DEV *pn544_dev = g_pn544_dev;
    if(mt_nfc_get_gpio_value(pn544_pdata.irq_gpio))
    {
      pn544_disable_irq( pn544_dev );
      wake_up_interruptible( &pn544_dev->read_wq );
    }
    else
    {
      printk( "%s, no irq \n", __func__);
    }
    return  IRQ_HANDLED;
}

/*****************************************************************************
**
******************************************************************************/
static ssize_t pn544_dev_read(struct file *filp, char __user *buf,
    size_t count, loff_t *offset)
{
    PN544_DEV *pn544_dev = filp->private_data;
    #if !defined( I2C_DMA_USAGE )
    char  tmp[MAX_BUFFER_SIZE];
    #endif
    int   ret = 0;
    #if defined( NFC_DEBUG )
    int   i;
    #endif /* NFC_DEBUG */
    unsigned short  addr;
    __u32           ext_flag;

    #if defined( DEBUG )
    printk("[NFC]%s: Enter for reading %zu bytes.\n", __func__, count );
    #endif
    if( count > MAX_BUFFER_SIZE )
      count = MAX_BUFFER_SIZE;
	  
    //>2016/02/04-JackHu--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
    current->flags |= PF_NOFREEZE;
    current->flags &= ~PF_FROZEN;
    //>2016/02/04-JackHu--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
	
    mutex_lock( &pn544_dev->read_mutex );
    #if defined( DEBUG )
    printk("%s: Mutex Lock\n", __func__);
    #endif
    if( !mt_nfc_get_gpio_value(pn544_pdata.irq_gpio) )
    {
        #if defined( DEBUG )
        printk("%s: no irq, irq_gpio: %d, irq_enabled: %d\n", __func__,
        mt_nfc_get_gpio_value(pn544_pdata.irq_gpio), pn544_dev->irq_enabled );
        #endif

        if( filp->f_flags & O_NONBLOCK )
        {
            ret = -EAGAIN;
            goto fail;
        }
        if( !( pn544_dev->irq_enabled ))
        {
            #if defined( DEBUG )
            printk("%s: enable_irq\n", __func__ );
            #endif
            pn544_enable_irq(pn544_dev);
        }
        #if defined( DEBUG )
        printk("%s: start wait!\n", __func__ );
        #endif
        //>2016/02/04-JackHu--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
        //ret = wait_event_interruptible( pn544_dev->read_wq, pn544_dev->irq_enabled == false);
        ret = wait_event_interruptible(pn544_dev->read_wq,(mt_nfc_get_gpio_value(pn544_pdata.irq_gpio)) );
        //>2016/02/04-JackHu--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode

        #if defined( DEBUG )
        printk("%s, ret: 0x%X\n", __func__, ret );
        #endif
        pn544_disable_irq( pn544_dev );
        if(!mt_nfc_get_gpio_value(pn544_pdata.irq_gpio))
        {
            ret = -EIO;
            goto fail;
        }
    }

    addr      = pn544_dev->client->addr;
    ext_flag  = pn544_dev->client->ext_flag;

#if defined( I2C_DMA_USAGE )
    pn544_dev->client->addr &= I2C_MASK_FLAG;
  //pn544_dev->client->addr     &= I2C_MASK_FLAG;
  //pn544_dev->client->addr     |= I2C_DMA_FLAG;
  //pn544_dev->client->addr     |= I2C_ENEXT_FLAG;
    pn544_dev->client->ext_flag |= I2C_DMA_FLAG;
  //pn544_dev->client->ext_flag |= I2C_DIRECTION_FLAG;
  //pn544_dev->client->ext_flag |= I2C_A_FILTER_MSG;
    pn544_dev->client->timing = NFC_CLIENT_TIMING;
    /* Read data */
    ret = i2c_master_recv( pn544_dev->client, (unsigned char *)I2CDMAReadBuf_pa, count );
#else
    /* Read data */
//    i = 0;
//    while (ret < 0 && i < 3) {
//        i++;
        pn544_dev->client->addr&=I2C_MASK_FLAG;
        ret = i2c_master_recv( pn544_dev->client, (unsigned char *)tmp, count );
//    }
#endif

    pn544_dev->client->addr     = addr;
    pn544_dev->client->ext_flag = ext_flag;
    mutex_unlock( &pn544_dev->read_mutex );
    #if defined( DEBUG )
    printk("%s: Mutex unLock\n", __func__);
    #endif

    if( ret < 0 )
    {
        #if defined( DEBUG )
        printk("%s: i2c_master_recv returned %d\n", __func__, ret );
        #endif
        return ret;
    }
    if( ret > count )
    {
        #if defined( DEBUG )
        printk("%s: received too many bytes from i2c (%d)\n", __func__, ret );
        #endif
        return -EIO;
    }

#if defined( I2C_DMA_USAGE )
    if( copy_to_user( buf, I2CDMAReadBuf, ret ))
#else
    if( copy_to_user( buf, tmp, ret ))
#endif
    {
        #if defined( DEBUG )
        pr_warning("%s : failed to copy to user space.\n", __func__);
        #endif
        return -EFAULT;
    }

#if defined( NFC_DEBUG )
    printk( "%s: bytes[%d] ", __func__, (int)count );
       #if defined( I2C_DMA_USAGE )
	if(I2CDMAReadBuf!=NULL)
        #else
	if(tmp!=NULL)
       #endif
	{
		for( i = 0; i < count; i++ )
		{
		#if defined( I2C_DMA_USAGE )
		  printk("%02X ", *(I2CDMAReadBuf + i));
		#else
		  printk("%02X ", tmp[i] );
		#endif
		}
	}
	else
		printk(" tmp==Null \n");
    printk(" \n");
#endif /* NFC_DEBUG */
    //>2016/02/04-JackHu--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
    wake_lock_timeout(&pn544_dev->wake_lock, msecs_to_jiffies(1500));
    //>2016/02/04-JackHu--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode

    #if defined( DEBUG )
    printk("%s complete, irq: %d\n", __func__, mt_nfc_get_gpio_value(pn544_pdata.irq_gpio));
    #endif
    return ret;

fail:
    mutex_unlock( &pn544_dev->read_mutex );
    #if defined( DEBUG )
    printk("%s error ---, ret: 0x%X\n", __func__, ret );
    #endif
    return ret;
}

/*****************************************************************************
**
******************************************************************************/
static ssize_t pn544_dev_write(struct file *filp, const char __user *buf,
    size_t count, loff_t *offset)
{
    #if 1
    PN544_DEV * pn544_dev;
    #if !defined( I2C_DMA_USAGE )
    char  tmp[MAX_BUFFER_SIZE];
    #endif
    #if defined( NFC_DEBUG )
    int   i;
    #endif /* NFC_DEBUG */
    int   ret;
    unsigned short  addr;
    __u32           ext_flag;
    #if defined( DEBUG )
    printk("[NFC]%s: Enter for writing %zu bytes.\n", __func__, count );
    #endif
    pn544_dev = filp->private_data;

    if( count > MAX_BUFFER_SIZE )
      count = MAX_BUFFER_SIZE;

#if defined( I2C_DMA_USAGE )
    if( copy_from_user( I2CDMAWriteBuf, buf, count ))
#else
    if( copy_from_user( tmp, buf, count ))
#endif
    {
      #if defined( DEBUG )
      printk( "%s : failed to copy from user space\n", __func__ );
      #endif
      return -EFAULT;
    }

#if defined( NFC_DEBUG )
    printk("%s: bytes[%d] ", __func__, (int)count );
    #if defined( I2C_DMA_USAGE )
	if(I2CDMAReadBuf!=NULL)
    #else
	if(tmp!=NULL)
    #endif
	{
		for( i = 0; i < count; i++ )
		{
		#if defined( I2C_DMA_USAGE )
		  printk("%02X ", *(I2CDMAWriteBuf + i));
		#else
		  printk("%02X ", tmp[i] );
		#endif
		}
	}else
		  printk(" temp==NULL\n");
    printk(" \n");
#endif /* NFC_DEBUG */

    addr      = pn544_dev->client->addr;
    ext_flag  = pn544_dev->client->ext_flag;
	#if defined( DEBUG )
	printk("%02X ",addr);
	#endif
#if defined( I2C_DMA_USAGE )
   // pn544_dev->client->addr     &= I2C_MASK_FLAG;
    pn544_dev->client->addr     |= I2C_DMA_FLAG;
  //pn544_dev->client->addr     |= I2C_ENEXT_FLAG;
    pn544_dev->client->ext_flag |= I2C_DMA_FLAG;
  //pn544_dev->client->ext_flag |= I2C_DIRECTION_FLAG;
  //pn544_dev->client->ext_flag |= I2C_A_FILTER_MSG;
#endif

    /* Write data */
#if defined( I2C_DMA_USAGE )
      ret = i2c_master_send( pn544_dev->client, (unsigned char *)I2CDMAWriteBuf_pa, count );
	//ret = i2c_master_send( pn544_dev->client, (unsigned char *)I2CDMAWriteBuf, count );
#else
    ret = i2c_master_send( pn544_dev->client, (unsigned char *)tmp, count );
#endif

    if( ret != count )
    {
      #if defined( DEBUG )
      printk("%s : i2c_master_send returned %d\n", __func__, ret );
      #endif
      ret = -EIO;
    }
    pn544_dev->client->addr     = addr;
    pn544_dev->client->ext_flag = ext_flag;
    #if defined( DEBUG )
    printk("%s : complete, result = %d\n", __func__, ret );
    #endif
    return ret;
#else	
	PN544_DEV * pn544_dev;
	int ret = 0, ret_tmp = 0, count_ori = 0,count_remain = 0, idx = 0;
	pn544_dev = filp->private_data;
    count_ori = count;
    count_remain = count_ori;
    #if defined( DEBUG )
	printk("[NFC]%s: Enter for writing %zu bytes.\n", __func__, count );
    #endif
	if (count > MAX_BUFFER_SIZE)
	{
		count = MAX_BUFFER_SIZE;
		count_remain -= count;
	}
    do
    {
        if (copy_from_user(I2CDMAWriteBuf, &buf[(idx*512)], count)) 
        {
            #if defined( DEBUG )
            printk(KERN_DEBUG "%s : failed to copy from user space\n", __func__);
            #endif
            return -EFAULT;
    	}
        #if defined( DEBUG )
    	printk(KERN_DEBUG "%s : writing %zu bytes, remain bytes %d.\n", __func__, count, count_remain);
        #endif
    	
    	/* Write data */
        pn544_dev->client->addr = (pn544_dev->client->addr & I2C_MASK_FLAG);// | I2C_DMA_FLAG;
        pn544_dev->client->ext_flag |= I2C_DMA_FLAG;
        pn544_dev->client->timing = NFC_CLIENT_TIMING;
        ret_tmp = i2c_master_send(pn544_dev->client, (unsigned char *)(uintptr_t)I2CDMAWriteBuf_pa, count);
        if (ret_tmp != count) 
        {
            #if defined( DEBUG )
            printk(KERN_DEBUG "%s : i2c_master_send returned %d\n", __func__, ret);
            #endif
            ret = -EIO;
            return ret;
        }
    	
        ret += ret_tmp;
        #if defined( DEBUG )
        printk(KERN_DEBUG "%s : %d,%d,%d\n", __func__, ret_tmp,ret,count_ori);
        #endif

        if( ret ==  count_ori)
    	{
            #if defined( DEBUG )
            printk(KERN_DEBUG "%s : ret== count_ori \n", __func__);
            #endif
            break;		
    	}
    	else
    	{
            if(count_remain > MAX_BUFFER_SIZE)
            {
                count = MAX_BUFFER_SIZE;
    		    count_remain -= MAX_BUFFER_SIZE;
            }
            else
            {
                count = count_remain;
                count_remain = 0;
            }
            idx++;		
            #if defined( DEBUG )
            printk(KERN_DEBUG "%s :remain_bytes, %d,%d,%d,%d,%d\n", __func__, ret_tmp,ret,(int)count,count_ori,idx);
            #endif
    	}
	}
	while(1);

	#if defined( DEBUG )
	printk(KERN_DEBUG "%s : writing %d bytes. Status %d \n", __func__, count_ori,ret);
	#endif
	return ret;
#endif
}

/*****************************************************************************
**
******************************************************************************/
static int pn544_dev_open(struct inode *inode, struct file *filp)
{
PN544_DEV *pn544_dev = container_of( filp->private_data,
                          PN544_DEV,
                          pn544_device );
    #if defined( DEBUG )
    printk("[NFC]%s: Enter...\n", __func__ );
    #endif
    filp->private_data = pn544_dev;
    #if defined( DEBUG )
    printk("%s : %d, %d\n", __func__, imajor( inode ), iminor( inode ));
    #endif
    return 0;
}

/*****************************************************************************
**
******************************************************************************/
static long pn544_dev_ioctl(struct file *filp,
          unsigned int cmd, unsigned long arg)
{
    #if defined( DEBUG )
    printk("[NFC]%s: Enter...\n", __func__ );
    #endif
    switch( cmd )
    {
      case PN544_SET_PWR:
      {
        if( arg == 2 )
        {
		    #if defined( DEBUG )
		    printk("[NFC]%s power on with firmware\n", __func__ );
            #endif
			//>2016/02/01-JackHu, Modify timing to improve meta test not stable issue
			mt_nfc_pinctrl_select(gpctrl,st_ven_l);
		    mt_nfc_pinctrl_select(gpctrl,st_eint_h);
		    usleep_range(ENABLE_DELAY*900, ENABLE_DELAY*1000);
		    mt_nfc_pinctrl_select(gpctrl,st_ven_l);
		    usleep_range(DISABLE_DELAY*900, DISABLE_DELAY*1000);
		    mt_nfc_pinctrl_select(gpctrl,st_ven_h);
		    usleep_range(ENABLE_DELAY*900, ENABLE_DELAY*1000);
			//>2016/02/01-JackHu
        }
        else if( arg == 1 )
        { /* power on */
            #if defined( DEBUG )
            printk("[NFC]%s power on\n", __func__ );
            #endif
			//>2016/02/04-JackHu--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
            pn544_enable_irq(filp->private_data);
            //>2016/02/04-JackHu--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
			//>2016/02/01-JackHu, Modify timing to improve meta test not stable issue
            mt_nfc_pinctrl_select(gpctrl,st_ven_h);
            mt_nfc_pinctrl_select(gpctrl,st_eint_l);
            usleep_range(ENABLE_DELAY*900, ENABLE_DELAY*1000);
			//>2016/02/01-JackHu
        }
        else if( arg == 0 )
        { /* power off */
            #if defined( DEBUG )
            printk("[NFC]%s power off\n", __func__ );
            #endif
            //>2016/02/04-JackHu--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
            pn544_enable_irq(filp->private_data);
            //>2016/02/04-JackHu--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
			//>2016/02/01-JackHu, Modify timing to improve meta test not stable issue
            mt_nfc_pinctrl_select(gpctrl,st_ven_l);
            mt_nfc_pinctrl_select(gpctrl,st_eint_l);
            usleep_range(DISABLE_DELAY*900, DISABLE_DELAY*1000);
			//>2016/02/01-JackHu
        }
        else
        {
            #if defined( DEBUG )
            printk("%s bad arg %lu\n", __func__, arg );
            #endif
            return -EINVAL;
        }
      } 
	  break;
      default:
      {
        #if defined( DEBUG )
        printk("%s bad ioctl %u\n", __func__, cmd );
        #endif
        return -EINVAL;
      }
    }
    return 0;
}

static const struct file_operations pn544_dev_fops = {
    .owner  = THIS_MODULE,
    .llseek = no_llseek,
    .read   = pn544_dev_read,
    .write  = pn544_dev_write,
    .open   = pn544_dev_open,
    .unlocked_ioctl = pn544_dev_ioctl,
};

static int mt_nfc_pinctrl_select(struct pinctrl *p, struct pinctrl_state *s)
{
	int ret = 0;
	if (p != NULL && s != NULL) {
		ret = pinctrl_select_state(p, s);
	} else {
	    #if defined( DEBUG )
		pr_debug("%s : pinctl_select err\n", __func__);
        #endif
		ret = -1;
	}
	return ret;
}
static int mt_nfc_pinctrl_init(struct platform_device *pdev,int probe_F)
{
	int ret = 0;
	gpctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(gpctrl)) {
		dev_err(&pdev->dev, "NFC Cannot find pinctrl!");
		ret = PTR_ERR(gpctrl);
		goto end;
	}
	
	st_ven_h = pinctrl_lookup_state(gpctrl, "ven_high");
	if (IS_ERR(st_ven_h)) {
		ret = PTR_ERR(st_ven_h);
		pr_debug("%s : pinctrl err, ven_high NFC\n", __func__);
	}
	st_ven_l = pinctrl_lookup_state(gpctrl, "ven_low");
	if (IS_ERR(st_ven_l)) {
		ret = PTR_ERR(st_ven_l);
		pr_debug("%s : pinctrl err, ven_low NFC\n", __func__);
	}
/*	st_rst_h = pinctrl_lookup_state(gpctrl, "rst_high");
	if (IS_ERR(st_rst_h)) {
		ret = PTR_ERR(st_rst_h);
		pr_debug("%s : pinctrl err, rst_high\n", __func__);
	}

	st_rst_l = pinctrl_lookup_state(gpctrl, "rst_low");
	if (IS_ERR(st_rst_l)) {
		ret = PTR_ERR(st_rst_l);
		pr_debug("%s : pinctrl err, rst_low\n", __func__);
	}
*/
	st_eint_h = pinctrl_lookup_state(gpctrl, "eint_high");
	if (IS_ERR(st_eint_h)) {
		ret = PTR_ERR(st_eint_h);
		pr_debug("%s : pinctrl err, eint_high NFC\n", __func__);
	}
	st_eint_l = pinctrl_lookup_state(gpctrl, "eint_low");
	if (IS_ERR(st_eint_l)) {
		ret = PTR_ERR(st_eint_l);
		pr_debug("%s : pinctrl err, eint_low NFC\n", __func__);
	}
	st_irq_init = pinctrl_lookup_state(gpctrl, "irq_init");
	if (IS_ERR(st_irq_init)) {
		ret = PTR_ERR(st_irq_init);
		pr_debug("%s : pinctrl err, irq_init NFC\n", __func__);
	}

	/* select state */
	if(probe_F==1)
	{
	    ret = mt_nfc_pinctrl_select(gpctrl, st_irq_init);
	    usleep_range(900, 1000);
	    ret = mt_nfc_pinctrl_select(gpctrl, st_ven_l);
	    usleep_range(900, 1000);
        //ret = mt_nfc_pinctrl_select(gpctrl, st_rst_h);
        //usleep_range(900, 1000);
	    ret = mt_nfc_pinctrl_select(gpctrl, st_eint_l);
	}
end:
	return ret;
}
static int mt_nfc_gpio_init(void)
{
	struct device_node *node;
	#if defined( DEBUG )
	printk("[NFC] %s : 1\n", __func__);
	#endif
	node = of_find_compatible_node(NULL, NULL, "mediatek,nfc-gpio-v2");
	#if defined( DEBUG )
	printk("[NFC] %s : 2\n", __func__);
	#endif
	if (node) {
	#if defined( DEBUG )
	printk("[NFC] %s : 3\n", __func__);
	#endif
		of_property_read_u32_array(node, "ven-gpio",
					   &(pn544_pdata.ven_gpio), 1);
	/*	of_property_read_u32_array(node, "gpio-rst",
					   &(mt6605_platform_data.sysrstb_gpio),
					   1);*/
		of_property_read_u32_array(node, "firm-gpio",
					   &(pn544_pdata.firm_gpio),
					   1);
		of_property_read_u32_array(node, "irq-gpio",
					   &(pn544_pdata.irq_gpio), 1);
	} else {
	    #if defined( DEBUG )
		pr_debug("%s : get gpio num err.\n", __func__);
        #endif
		return -1;
	}
	#if defined( DEBUG )
	printk("[NFC] %s : 4\n", __func__);	
	printk("%s, ven_gpio:%d, firm_gpio:%d, irq_gpio:%d\n", __func__,pn544_pdata.ven_gpio,pn544_pdata.firm_gpio,pn544_pdata.irq_gpio);
	#endif
	return 0;
}
static int nxp_nfc_probe(struct platform_device *pdev)
{
	int ret = 0;
	nfc_plt_dev = pdev;
	#if defined( DEBUG )
	printk("[NFC] %s : &nfc_plt_dev=%p\n", __func__, nfc_plt_dev);
	#endif
	/* pinctrl init */
	ret = mt_nfc_pinctrl_init(pdev,1);
	/* gpio init */
	#if defined( DEBUG )
	printk("[NFC] %s :1\n", __func__);
	#endif
	if (mt_nfc_gpio_init() != 0)
		pr_debug("%s : mt_nfc_gpio_init err.\n", __func__);
	#if defined( DEBUG )
	printk("[NFC] %s :2\n", __func__);
	#endif
	return 0;
}
static int nxp_nfc_remove(struct platform_device *pdev)
{
    #if defined( DEBUG )
	printk("[NFC] %s : &pdev=%p\n", __func__, pdev);
    #endif
	return 0;
}

static const struct of_device_id nfc_dev_of_match[] = {
	{.compatible = "mediatek,nfc-gpio-v2",},
	{},
};
static struct platform_driver nxp_nfc_platform_driver = {
	.probe = nxp_nfc_probe,
	.remove = nxp_nfc_remove,
	.driver = {
		   .name = "pn544",
		   .owner = THIS_MODULE,
		   .of_match_table = nfc_dev_of_match,
		   },
};
/*****************************************************************************
**
******************************************************************************/
static int pn544_probe(struct i2c_client *client,
    const struct i2c_device_id *id)
{
    struct pn544_i2c_platform_data *platform_data;
    PN544_DEV *pn544_dev;
    int ret;
    struct device_node *node;
    #if defined( DEBUG )
    printk("[NFC]%s: Enter...\n", __func__ );
    #endif
    client->timing    = NFC_CLIENT_TIMING; /* 400 KHz */
    platform_data=&pn544_pdata;
    #if defined( DEBUG )
    printk("[NFC]%s step 1...\n", __func__ );
    #endif
    if( !platform_data )
    {
        printk( "%s, Can not get platform_data\n", __func__ );
        return -EINVAL;
    }

    dev_dbg( &client->dev, "pn544 probe: %s, inside pn544 flags = %x\n",
          __func__, client->flags );

    if( platform_data == NULL )
    {
        printk("%s : nfc probe fail\n", __func__ );
        return  -ENODEV;
    }
    #if defined( DEBUG )
    printk("[NFC]%s step 2...\n", __func__ );
    #endif

    if( !i2c_check_functionality( client->adapter, I2C_FUNC_I2C ))
    {
        printk( "%s : need I2C_FUNC_I2C\n", __func__ );
        return  -ENODEV;
    }
    #if defined( DEBUG )
    printk("[NFC]%s step 3...\n", __func__ );
    #endif
	
    pn544_dev = kzalloc( sizeof( *pn544_dev ), GFP_KERNEL );
    if( pn544_dev == NULL )
    {
        dev_err( &client->dev, "failed to allocate memory for module data\n");
        ret = -ENOMEM;
        goto err_exit;
    }
    #if defined( DEBUG )
    printk("[NFC]%s step 4...\n", __func__ );
    #endif
	
    pn544_dev->irq_gpio   = platform_data->irq_gpio;
    pn544_dev->ven_gpio   = platform_data->ven_gpio;
    pn544_dev->firm_gpio  = platform_data->firm_gpio;
    pn544_dev->client     = client;

  /* init mutex and queues */
    init_waitqueue_head( &pn544_dev->read_wq );
    mutex_init( &pn544_dev->read_mutex );
    spin_lock_init( &pn544_dev->irq_enabled_lock );

    pn544_dev->pn544_device.minor = MISC_DYNAMIC_MINOR;
    pn544_dev->pn544_device.name  = NFC_DEV_NAME;
    pn544_dev->pn544_device.fops  = &pn544_dev_fops;
    ret = misc_register( &pn544_dev->pn544_device );
    if( ret )
    {
        printk("%s : misc_register failed\n", __FILE__);
        goto err_misc_register;
    }
    #if defined( DEBUG )
    printk("[NFC]%s step 5...\n", __func__ );
    #endif

#if defined( I2C_DMA_USAGE )
    if( I2CDMAWriteBuf == NULL )
    {
	    #ifdef CONFIG_64BIT    
	    I2CDMAWriteBuf = (char *)dma_alloc_coherent(&client->dev, MAX_BUFFER_SIZE, (dma_addr_t *)&I2CDMAWriteBuf_pa, GFP_KERNEL);
        #else
        I2CDMAWriteBuf = (char *)dma_alloc_coherent( NULL, MAX_BUFFER_SIZE, &I2CDMAWriteBuf_pa, GFP_KERNEL );
	    #endif
        if( I2CDMAWriteBuf == NULL )
        {
            printk("%s : failed to allocate dma write buffer\n", __func__ );
            goto err_request_irq_failed;
        }
    }
    #if defined( DEBUG )
    printk("[NFC]%s step 5-1...\n", __func__ );
    #endif

    if( I2CDMAReadBuf == NULL )
    {
	    #ifdef CONFIG_64BIT 	
	    I2CDMAReadBuf = (char *)dma_alloc_coherent(&client->dev, MAX_BUFFER_SIZE, (dma_addr_t *)&I2CDMAReadBuf_pa, GFP_KERNEL);
	    #else
        I2CDMAReadBuf = (char *)dma_alloc_coherent( NULL, MAX_BUFFER_SIZE, (dma_addr_t *)&I2CDMAReadBuf_pa, GFP_KERNEL );
	    #endif
        if( I2CDMAReadBuf == NULL )
        {
            printk("%s : failed to allocate dma read buffer\n", __func__ );
            goto err_request_irq_failed;
        }
    }
    #if defined( DEBUG )
    printk("[NFC]%s step 5-2...\n", __func__ );
    #endif
#endif /* End.. (I2C_DMA_USAGE) */

	node = of_find_compatible_node(NULL, NULL, "mediatek, IRQ_NFC-eint");
	if (node) {
		nfc_irq = irq_of_parse_and_map(node, 0);
		client->irq = nfc_irq;
		ret =request_irq(nfc_irq, pn544_dev_irq_handler,IRQF_TRIGGER_NONE, "irq_nfc-eint", NULL);
		if (ret) {
			printk("%s : EINT IRQ LINE NOT AVAILABLE\n", __func__);
			goto err_request_irq_failed;
		} else {
		    #if defined( DEBUG )
			printk("%s : set EINT finished, nfc_irq=%d\n", __func__,nfc_irq);
            #endif
                    pn544_dev->irq_enabled = true;
                    pn544_disable_irq( pn544_dev );
		}
	} else {
	    #if defined( DEBUG )
		printk("%s : can not find NFC eint compatible node\n",__func__);
        #endif
	}
	#if defined( DEBUG )
    printk("[NFC]%s step 6...\n", __func__ );
	#endif
    i2c_set_clientdata( client, pn544_dev );
	//>2016/02/04-JackHu--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
    wake_lock_init(&pn544_dev->wake_lock, WAKE_LOCK_SUSPEND, "pn544_nfc_read");
    //>2016/02/04-JackHu--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode

	#if defined( DEBUG )
    printk("[NFC]%s Success...\n", __func__ );
	#endif
	g_pn544_dev = pn544_dev;
	#if defined( DEBUG )
	if(g_pn544_dev->client==NULL)
	    printk("%s : g_pn544_dev->client==NULL  NFC\n", __func__);
	printk("[NFC]%s check\n", __func__ );
	#endif
	g_pn544_dev->irq_gpio=pn544_pdata.irq_gpio;
	g_pn544_dev->ven_gpio=pn544_pdata.ven_gpio;
	g_pn544_dev->firm_gpio=pn544_pdata.firm_gpio;
	#if defined( DEBUG )
	printk("%s : g_pn544_dev.ven:%d, firm:%d, irq:%d  NFC\n", __func__,g_pn544_dev->ven_gpio,g_pn544_dev->firm_gpio,g_pn544_dev->irq_gpio);
	printk("%s : g_pn544_dev.irq enabled:%d  NFC\n", __func__,g_pn544_dev->irq_enabled);
	#endif
    return 0;
err_request_irq_failed:
    misc_deregister( &pn544_dev->pn544_device );
err_misc_register:
    mutex_destroy( &pn544_dev->read_mutex );
//    mutex_destroy( &pn544_dev->rw_mutex );
err_exit:
    kfree( pn544_dev );
	mt_nfc_pinctrl_select(gpctrl,st_ven_l);
    mt_nfc_pinctrl_select(gpctrl,st_eint_l);
    //>2016/02/04-JackHu--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
    wake_lock_destroy(&pn544_dev->wake_lock);
    //>2016/02/04-JackHu--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode

    printk("[NFC]%s some error...\n", __func__ );
    return  ret;
}

/*****************************************************************************
**
******************************************************************************/
static int pn544_remove(struct i2c_client *client)
{
    PN544_DEV *pn544_dev;
    #if defined( DEBUG )
    printk("[NFC]%s: Enter...\n", __func__ );
    #endif
    pn544_dev = i2c_get_clientdata( client );

#if defined( I2C_DMA_USAGE )
    if( I2CDMAWriteBuf )
    {
	    #ifdef CONFIG_64BIT 	
	    dma_free_coherent(&client->dev, MAX_BUFFER_SIZE, I2CDMAWriteBuf, I2CDMAWriteBuf_pa);
	    #else
        dma_free_coherent( NULL, MAX_BUFFER_SIZE, I2CDMAWriteBuf, I2CDMAWriteBuf_pa );
	    #endif
        I2CDMAWriteBuf    = NULL;
        I2CDMAWriteBuf_pa = 0;
    }

    if( I2CDMAReadBuf )
    {
	    #ifdef CONFIG_64BIT
	    dma_free_coherent(&client->dev, MAX_BUFFER_SIZE, I2CDMAReadBuf, I2CDMAReadBuf_pa);
	    #else
        dma_free_coherent( NULL, MAX_BUFFER_SIZE, I2CDMAReadBuf, I2CDMAReadBuf_pa );
	    #endif
        I2CDMAReadBuf     = NULL;
        I2CDMAReadBuf_pa  = 0;
    }
#endif /* End.. (I2C_DMA_USAGE) */
    //>2016/02/04-JackHu--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
    wake_lock_destroy(&pn544_dev->wake_lock);
    //free_irq( client->irq, pn544_dev );
	//>2016/02/04-JackHu--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode

    misc_deregister( &pn544_dev->pn544_device );
    mutex_destroy( &pn544_dev->read_mutex );
    kfree( pn544_dev );
    return 0;
}

/*****************************************************************************
**
******************************************************************************/
/* i2c_register_board_info will be not to do over here */


static struct of_device_id pn544_match_table[] = {
    { .compatible = "mediatek,NFC" },
    {}
};

static const struct i2c_device_id pn544_id[] = {
    { "pn544", 0},
    {}
};

static struct i2c_driver pn544_driver = {
    .id_table = pn544_id,
    .probe    = pn544_probe,
    .remove   = pn544_remove,
    .driver   =
    {
      .owner  = THIS_MODULE,
      .name   = "pn544",
      .of_match_table = pn544_match_table,
    },
};

/*
 * module load/unload record keeping
 */

static int __init pn544_dev_init(void)
{
    int vRet = 0;
    #if defined( DEBUG )
    printk("[NFC] %s: Loading pn544 driver...Jacack\n", __func__ );
    #endif
#if 0
    i2c_register_board_info( 1, &pn544_i2c_device, ARRAY_SIZE(pn544_i2c_device));
#endif
    platform_driver_register(&nxp_nfc_platform_driver);
    if( i2c_add_driver( &pn544_driver ))
    {
      printk("[NFC] PN544 add I2C driver error\n");
      vRet = -1;
    }
    else
    {
      #if defined( DEBUG )
      printk("[NFC] PN544 add I2C driver success\n");
      #endif
    }
    return vRet;
}
module_init( pn544_dev_init );

static void __exit pn544_dev_exit(void)
{
    #if defined( DEBUG )
    printk("[NFC] Unloading pn544 driver\n");
    #endif
    i2c_del_driver( &pn544_driver );
}
module_exit( pn544_dev_exit );

MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_DESCRIPTION("NFC PN544 driver");
MODULE_LICENSE("GPL");
