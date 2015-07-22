/*
 * This file is part of the DI ap3xx6 sensor driver for MTK platform.
 * DI ap3xx6 is combined proximity, ambient light sensor and IRLED.
 *
 * Contact: YC Hou <yc.hou@liteonsemi.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 *
 * Filename: ap3xx6.c
 *
 * Summary:
 *	ap3xx6 sensor dirver.
 *
 * Modification History:
 * Date     By       Summary
 * -------- -------- -------------------------------------------------------
 * 05/11/12 YC		 Original Creation (Test version:1.0)
 * 05/30/12 YC		 Modify AP3xx6_check_and_clear_intr return value and exchange
 *                   AP3xx6_get_ps_value return value to meet our spec.
 * 05/30/12 YC		 Correct shift number in AP3xx6_read_ps.
 * 05/30/12 YC		 Correct ps data formula.
 * 05/31/12 YC		 1. Change the reg in clear int function from low byte to high byte 
 *                      and modify the return value.
 *                   2. Modify the eint_work function to filter als int.
 * 06/04/12 YC		 Add PS high/low threshold instead of using the same value.
 * 07/12/12 YC		 Add wakelock to prevent entering suspend when early suspending.
 *
 *
 *29/5/14   ansun modify code to mt6582 add ap3426
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
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
//#include <linux/wakelock.h>



#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>


//#include <mach/mt_devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <alsps.h>
#ifdef CUSTOM_KERNEL_SENSORHUB
#include <SCP_sensorHub.h>
#endif
#include "ap3xx6c.h"


/*-------------------------------flag---------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)                 
/******************************************************************************
 * extern functions
*******************************************************************************/
extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern void mt_eint_print_status(void);
/*---------------------------user define-------------------------------------------------*/
#define POWER_NONE_MACRO MT65XX_POWER_NONE

//#define USE_WAKELOCK        //20130718 add
//#define UNUSE_ENREAD
#define DELAYED_WORK 0
#define AP3426
#ifdef AP3426
	#define AP3xx6_DEV_NAME     "AP3426"
#else
	#define AP3xx6_DEV_NAME     "AP3216"
#endif
static struct i2c_client *AP3xx6_i2c_client = NULL;
//static struct AP3xx6_priv *g_AP3xx6_ptr = NULL;
static struct AP3xx6_priv *AP3xx6_obj = NULL;
static struct platform_driver AP3xx6_alsps_driver;

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id AP3xx6_i2c_id[] = {{AP3xx6_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_AP3xx6={ I2C_BOARD_INFO(AP3xx6_DEV_NAME, (0x3C >> 1))};
//static unsigned short AP3xx6_force[] = {0x00, 0x3C, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const AP3xx6_forces[] = { AP3xx6_force, NULL };
//static struct i2c_client_address_data AP3xx6_addr_data = { .forces = AP3xx6_forces,};

/******************************************************************************
 * configuration
*******************************************************************************/

static void AP3xx6_power(struct alsps_hw *hw, unsigned int on) ;



/*----------------------------------------------------------------------------*/
static int AP3xx6_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int AP3xx6_i2c_remove(struct i2c_client *client);
//static int AP3xx6_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
static int AP3xx6_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int AP3xx6_i2c_resume(struct i2c_client *client);

/*----------------------------------------------------------------------------*/


#if defined(USE_WAKELOCK)
struct wake_lock chrg_lock;
#endif

/*----------------------------------------------------------------------------*/
typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,

} CMC_BIT;
/*----------------------------------------------------------------------------*/
struct AP3xx6_i2c_addr {    /*define a series of i2c slave address*/
    u8  write_addr;  
    u8  ps_thd;     /*PS INT threshold*/
};
/*----------------------------------------------------------------------------*/
struct AP3xx6_priv {
    struct alsps_hw  *hw;
    struct i2c_client *client;
#if DELAYED_WORK
    struct delayed_work  eint_work;
#else
    struct work_struct  eint_work;
#endif
	  struct mutex lock;
    /*i2c address group*/
    struct AP3xx6_i2c_addr  addr;
    
    /*misc*/
    u16		    als_modulus;
    atomic_t    i2c_retry;
    atomic_t    als_suspend;
    atomic_t    als_debounce;   /*debounce time after enabling als*/
    atomic_t    als_deb_on;     /*indicates if the debounce is on*/
    atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
    atomic_t    ps_mask;        /*mask ps: always return far away*/
    atomic_t    ps_debounce;    /*debounce time after enabling ps*/
    atomic_t    ps_deb_on;      /*indicates if the debounce is on*/
    atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
    atomic_t    ps_suspend;


    /*data*/
    u16         als;
    u16          ps;
    u8          _align;
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];

    atomic_t    als_cmd_val;    /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_cmd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val_h;   /*the cmd value can't be read, stored in ram*/
	atomic_t    ps_thd_val_l;   /*the cmd value can't be read, stored in ram*/
#if defined(UNUSE_ENREAD)
    atomic_t    enable;
#else
    ulong       enable;         /*enable mask*/    
#endif
    ulong       pending_intr;   /*pending interrupt*/

    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver AP3xx6_i2c_driver = {	
	.probe      = AP3xx6_i2c_probe,
	.remove     = AP3xx6_i2c_remove,
//	.detect     = AP3xx6_i2c_detect,
	.suspend    = AP3xx6_i2c_suspend,
	.resume     = AP3xx6_i2c_resume,
	.id_table   = AP3xx6_i2c_id,
//	.address_data = &AP3xx6_addr_data,
	.driver = {
//		.owner          = THIS_MODULE,
		.name           = AP3xx6_DEV_NAME,
	},
};

static int AP3xx6_local_init(void);
static int AP3xx6_remove(void);

static struct alsps_init_info ap3xx6_init_info = {
		.name = "ap3xx6",
		.init = AP3xx6_local_init,
		.uninit = AP3xx6_remove,
	
};

/*----------------------------------------------------------------------------*/
static int AP3xx6_local_init(void) 
{
	struct alsps_hw *hw = get_cust_alsps_hw();

	AP3xx6_power(hw, 1);    
	//AP3xx6_force[0] = hw->i2c_num;
	//AP3xx6_force[1] = hw->i2c_addr[0];
	//APS_DBG("I2C = %d, addr =0x%x\n",AP3xx6_force[0],AP3xx6_force[1]);
	if(i2c_add_driver(&AP3xx6_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	}
	//if(-1 == ap3xx6_init_flag)
	//{
	//   return -1;
	//}
	return 0;
}

/*----------------------------------------------------------------------------*/
static int AP3xx6_remove(void)
{
	struct alsps_hw *hw = get_cust_alsps_hw();
	APS_FUN();    
	AP3xx6_power(hw, 0);    
	i2c_del_driver(&AP3xx6_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/

static int AP3xx6_read_reg(struct i2c_client *client,
		char reg, u8 mask, u8 shift)
{
	int ret = 0;
	char tmp[1];
	tmp[0]=reg;
	mutex_lock(&AP3xx6_obj->lock);

	ret = i2c_master_send(client, tmp, 0x01);
	if(ret <= 0)
	{
		printk("AP3xx6_read_reg 1 ret=%x\n",ret);
		goto EXIT_ERR;
	}
	ret = i2c_master_recv(client, tmp, 0x01);
	if(ret <= 0)
	{
		printk("AP3xx6_read_reg 2 ret=%d\n",ret);
		goto EXIT_ERR;
	}

	mutex_unlock(&AP3xx6_obj->lock);
	return ( tmp[0] & mask ) >> shift;

EXIT_ERR:
		APS_ERR("AP3xx6_read_reg fail\n");
        mutex_unlock(&AP3xx6_obj->lock);
		return ret;
}

static int AP3xx6_write_reg(struct i2c_client *client,
		char reg, u8 mask, u8 shift, u8 val)
{
	int ret = 0x00;
	char tmp[2];

    mutex_lock(&AP3xx6_obj->lock);

    tmp[0]=reg;
    tmp[1]=val;
	ret = i2c_master_send(client, tmp, 0x02);
	if(ret <= 0)
	{
		printk("AP3xx6_write_reg ret=%d\n",ret);
		goto EXIT_ERR;
	}

    mutex_unlock(&AP3xx6_obj->lock);
	return ret;

EXIT_ERR:
		APS_ERR("AP3xx6_write_reg fail\n");
    mutex_unlock(&AP3xx6_obj->lock);
		return ret;
}
/*----------------------------------------------------------------------------*/
int AP3xx6_get_addr(struct alsps_hw *hw, struct AP3xx6_i2c_addr *addr)
{
	if(!hw || !addr)
	{
		return -EFAULT;
	}
	addr->write_addr= hw->i2c_addr[0];
	return 0;
}
/*----------------------------------------------------------------------------*/
static void AP3xx6_power(struct alsps_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	if(hw->power_id != POWER_NONE_MACRO)
	{
		if(power_on == on)
		{
			APS_LOG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, AP3xx6_DEV_NAME)) 
			{
				APS_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, AP3xx6_DEV_NAME)) 
			{
				APS_ERR("power off fail!!\n");   
			}
		}
	}
	power_on = on;
}
/*----------------------------------------------------------------------------*/
static int AP3xx6_enable_als(struct i2c_client *client, int enable)
{
		struct AP3xx6_priv *obj = i2c_get_clientdata(client);
		u8 databuf[2];	  
		int res = 0;
		u8 buffer[1];
		int reg_value[1];
	
		if(client == NULL)
		{
			APS_DBG("CLIENT CANN'T EQUAL NULL\n");
			return -1;
		}
	
#if defined(UNUSE_ENREAD)
        reg_value[0]=atomic_read(&obj->enable);
#else
		buffer[0]=AP3xx6_ENABLE;
		reg_value[0] = AP3xx6_read_reg(client,buffer[0], 0xFF, 0x00);
		if(res < 0)
		{
			goto EXIT_ERR;
		}        
#endif
		if(enable)
		{
			databuf[0] = AP3xx6_ENABLE;	
			databuf[1] = reg_value[0] |0x01;
			res = AP3xx6_write_reg(client,databuf[0], 0xFF,0x00,databuf[1]);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			atomic_set(&obj->als_deb_on, 1);
			atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));
			APS_DBG("AP3xx6_ ALS enable\n");
		          #if DELAYED_WORK
		            schedule_delayed_work(&obj->eint_work,1100*HZ/1000); 
		          #endif
		}
		else
		{
			databuf[0] = AP3xx6_ENABLE;	
			databuf[1] = reg_value[0] &0xFE;
			res = AP3xx6_write_reg(client,databuf[0], 0xFF,0x00,databuf[1]);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			atomic_set(&obj->als_deb_on, 0);
			APS_DBG("AP3xx6_ ALS disable\n");
		}
		return 0;
		
	EXIT_ERR:
		APS_ERR("AP3xx6__enable_als fail\n");
		return res;
}

/*----------------------------------------------------------------------------*/
static int AP3xx6_enable_ps(struct i2c_client *client, int enable)
{
	struct AP3xx6_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;
	u8 buffer[1];
	u8 reg_value[1];

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUAL NULL\n");
		return -1;
	}

#if defined(UNUSE_ENREAD)
    reg_value[0]=atomic_read(&obj->enable);
#else
	buffer[0]=AP3xx6_ENABLE;
	reg_value[0] = AP3xx6_read_reg(client,buffer[0], 0xFF, 0x00);
	if(res < 0)
	{
		goto EXIT_ERR;
	}    
#endif
	if(enable)
	{
		//wake_lock(&ps_lock);
		databuf[0] = AP3xx6_ENABLE;    
		databuf[1] = reg_value[0] |0x02;
		res = AP3xx6_write_reg(client,databuf[0], 0xFF,0x00,databuf[1]);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		atomic_set(&obj->ps_deb_on, 1);
		atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));

		mt_eint_unmask(CUST_EINT_ALS_NUM);
		        #if DELAYED_WORK
				schedule_delayed_work(&obj->eint_work,110*HZ/1000);	// wait 110 ms
		        #endif
		APS_DBG("AP3xx6_ PS enable\n");
	}
	else
	{
		databuf[0] = AP3xx6_ENABLE;    
		databuf[1] = reg_value[0] &0xfd;
		res = AP3xx6_write_reg(client,databuf[0], 0xFF,0x00,databuf[1]);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		atomic_set(&obj->ps_deb_on, 0);
		APS_DBG("AP3xx6_ PS disable\n");

		if(0 == obj->hw->polling_mode_ps)
		{
		            #if (!DELAYED_WORK)
			cancel_work_sync(&obj->eint_work);
		            #endif
			mt_eint_mask(CUST_EINT_ALS_NUM);
		}
		//wake_unlock(&ps_lock);
	}
	return 0;
	
EXIT_ERR:
	APS_ERR("AP3xx6__enable_ps fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/
static int AP3xx6_check_and_clear_intr(struct i2c_client *client) 
{
	struct AP3xx6_priv *obj = i2c_get_clientdata(client);
	int res;
	u8 ints[1];

	/* Get Int status */
	ints[0] = AP3xx6_read_reg(client,AP3xx6_INT_STATUS, 0xFF, 0x00);
	if(ints[0] < 0)
	{
		goto EXIT_ERR;
	}

	/* Clear ALS int flag */
	res = AP3xx6_read_reg(client,AP3xx6_ADATA_H, 0xFF, 0x00);
	if(res < 0)
	{
		goto EXIT_ERR;
	}

	/* Clear PS int flag */
	res =AP3xx6_read_reg(client,AP3xx6_PDATA_H, 0xFF, 0x00);
	if(res < 0)
	{
		goto EXIT_ERR;
	}

	return ints[0];

EXIT_ERR:
	APS_ERR("AP3xx6_check_and_clear_intr fail\n");
	return -1;
}
//als
static int AP3xx6_set_ALSGain(struct i2c_client *client, int val)
{
    int re_val, err;
    
#ifdef AP3426
/*val=0x00~0x3F*/
		re_val=val << 4;
    err =AP3xx6_write_reg(client, 0x10,
            0xFF, 0x00, re_val);   
#else    
/*val=0x00~0xF*/
	re_val = AP3xx6_read_reg(client,0x20, 0xFF, 0x00);
	re_val=(re_val&0xF)|(val << 4);
#endif	

    err =AP3xx6_write_reg(client, 0x20,
            0xFF, 0x00, re_val);    
          
	return err;
}

//ps
static int AP3xx6_set_plthres(struct i2c_client *client, int val)
{
	int lsb, msb, err;
	
#ifdef AP3426
	msb = val >> 8;
	lsb = val & 0xFF;
#else
	msb = val >> 2;
	lsb = val & 0x03;
#endif

	err = AP3xx6_write_reg(client, 0x2A,
			0xFF, 0x00, lsb);
	if (err<=0)
		return err;

	err = AP3xx6_write_reg(client, 0x2B,
			0xFF, 0x00, msb);

	return err;
}

static int AP3xx6_set_phthres(struct i2c_client *client, int val)
{
	int lsb, msb, err;

#ifdef AP3426
	msb = val >> 8;
	lsb = val & 0xFF;
#else
	msb = val >> 2;
	lsb = val & 0x03;
#endif

	err = AP3xx6_write_reg(client, 0x2C,
			0xFF, 0x00, lsb);
	if (err<=0)
		return err;

	err = AP3xx6_write_reg(client, 0x2D,
			0xFF, 0x00, msb);

	return err;
}

static int AP3xx6_set_pcrosstalk(struct i2c_client *client, int val)
{
    int lsb, msb, err;
#ifdef AP3426
	msb = val >> 8;
	lsb = val & 0xFF;
#else    
	msb = val >> 1;
	lsb = val & 0x01;
#endif	
    err =AP3xx6_write_reg(client, 0x28,
            0xFF, 0x00, lsb);    
	if (err<=0)
		return err;            
    err =AP3xx6_write_reg(client, 0x29,
            0xFF, 0x00, msb);
          
	return err;
}

static int AP3xx6_set_PSTtime(struct i2c_client *client, int val)
{
    int re_val, err;
    
#ifdef AP3426
/*val=0x00~0x3F*/
		re_val=val&0x3F;
    err =AP3xx6_write_reg(client, 0x25,
            0xFF, 0x00, re_val);   
#else    
/*val=0x00~0xF*/
	re_val = AP3xx6_read_reg(client,0x20, 0xFF, 0x00);
	re_val=(re_val&0xF)|(val << 4);
#endif	

    err =AP3xx6_write_reg(client, 0x20,
            0xFF, 0x00, re_val);    
          
	return err;
}

/*val=0x00~0x03*/
static int AP3xx6_set_PSgain(struct i2c_client *client, int val)
{
    int re_val, err;
    
#ifdef AP3426
	re_val = val << 2;
#else    
	re_val = AP3xx6_read_reg(client,0x20, 0xFF, 0x00);
	re_val=(re_val&0xF3)|(val << 2);
#endif	

    err =AP3xx6_write_reg(client, 0x20,
            0xFF, 0x00, re_val);    
          
	return err;
}

static int AP3xx6_set_PSTPersistence(struct i2c_client *client, int val)
{
    int re_val, err;
    
#ifdef AP3426
/*val=0x00~0x3f*/
		re_val=val&0x3F;
    err =AP3xx6_write_reg(client, 0x26,
            0xFF, 0x00, re_val);   
#else    
/*val=0x00~0x03*/
	re_val = AP3xx6_read_reg(client,0x20, 0xFF, 0x00);
	re_val=(re_val&0xFC)|val;
#endif	

    err =AP3xx6_write_reg(client, 0x20,
            0xFF, 0x00, re_val);    
          
	return err;
}

/*val=0x00~0x03*/
static int AP3xx6_set_PSratio(struct i2c_client *client, int val)
{
    int re_val, err;
    
#ifdef AP3426
	re_val = val;
#else    
	re_val = AP3xx6_read_reg(client,0x21, 0xFF, 0x00);
	re_val=(re_val&0xFC)|val;
#endif	

    err =AP3xx6_write_reg(client, 0x21,
            0xFF, 0x00, re_val);    
          
	return err;
}


static int AP3xx6_set_PSpulse(struct i2c_client *client, int val)
{
    int re_val, err;
    
#ifdef AP3426

#else    
/*val=0x00~0x03*/
	re_val = AP3xx6_read_reg(client,0x21, 0xFF, 0x00);
	re_val=(re_val&0xCF)|(val<<4);
	

    err =AP3xx6_write_reg(client, 0x21,
            0xFF, 0x00, re_val);    
#endif
          
	return err;
}


static int AP3xx6_set_intform(struct i2c_client *client, int val)
{
    int re_val, err;
    
	re_val=val&0x1;
  err =AP3xx6_write_reg(client, 0x22,
            0xFF, 0x00, re_val);    
          
	return err;
}

static int AP3xx6_set_meantime(struct i2c_client *client, int val)
{
    int re_val, err;
/*val=0x00~0x03*/    
	re_val=val&0x3;
  err =AP3xx6_write_reg(client, 0x23,
            0xFF, 0x00, re_val);    
          
	return err;
}


static int AP3xx6_set_waittime(struct i2c_client *client, int val)
{
    int re_val, err;
    
#ifdef AP3426
/*val=0x00~0xff*/
	re_val=val&0xFF;
	err =AP3xx6_write_reg(client, 0x06,
            0xFF, 0x00, re_val);
#else    
/*val=0x00~0x3f*/
	re_val=val&0x3F;
  err =AP3xx6_write_reg(client, 0x24,
            0xFF, 0x00, re_val);    
#endif
          
	return err;
}
/*----------------------------------------------------------------------------*/
void AP3xx6_eint_func(void)
{
	printk("AP3xx6__eint_func\n");
	struct AP3xx6_priv *obj =AP3xx6_obj;// g_AP3xx6_ptr;
	if(!obj)
	{
		return;
	}
#if DELAYED_WORK
	schedule_delayed_work(&obj->eint_work,0);	
#else    
	schedule_work(&obj->eint_work);
#endif
}

/*----------------------------------------------------------------------------*/
// This function depends the real hw setting, customers should modify it. 2012/5/10 YC. 
int AP3xx6_setup_eint(struct i2c_client *client)
{
	struct AP3xx6_priv *obj = i2c_get_clientdata(client);        

//	g_AP3xx6_ptr = obj;
	
	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, TRUE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);
	mt_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_TYPE, AP3xx6_eint_func, 0);

	mt_eint_unmask(CUST_EINT_ALS_NUM);  
	return 0;
}

/*----------------------------------------------------------------------------*/
static int AP3xx6_init_client(struct i2c_client *client)
{
	struct AP3xx6_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;
   
	databuf[0] = AP3xx6_ENABLE;    
	databuf[1] = 0x00;
	res = AP3xx6_write_reg(client,databuf[0], 0xFF,0x00,databuf[1]);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

    AP3xx6_set_plthres(client,atomic_read(&obj->ps_thd_val_l));
    AP3xx6_set_phthres(client,atomic_read(&obj->ps_thd_val_h));
#if 0
/*******************************************/
	AP3xx6_set_PSTtime(client,0x00);    //modify integrated time

	AP3xx6_set_PSgain(client,0x01);     //modify ps gain 

	AP3xx6_set_PSpulse(client,0x02);    //modify the led plus 

	AP3xx6_set_meantime(client,0x02);   //modify the mean times.

	AP3xx6_set_pcrosstalk(client,50);  //set crosstalk 100
#endif
	if(res = AP3xx6_setup_eint(client))
	{
		APS_ERR("setup eint: %d\n", res);
		return res;
	}
	if((res = AP3xx6_check_and_clear_intr(client)) < 0)
	{
		APS_ERR("check/clear intr: %d\n", res);
	}
	//custom_resolution_alsps
	databuf[0] = AP3xx6_ALS_GAIN;    
	databuf[1] = 0x30;
	res = AP3xx6_write_reg(client,databuf[0], 0xFF,0x00,databuf[1]);

	databuf[0] = AP3xx6_PS_GAIN;    
	databuf[1] = 0xC;
	res = AP3xx6_write_reg(client,databuf[0], 0xFF,0x00,databuf[1]);
	
	
	return AP3xx6_SUCCESS;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
int AP3xx6_read_als(struct i2c_client *client, u16 *data)
{
	struct AP3xx6_priv *obj = i2c_get_clientdata(client);	 
	u8 als_value_low[1], als_value_high[1];
	
	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUAL NULL\n");
		return -1;
	}

	// get ALS adc count
	als_value_low[0] = AP3xx6_read_reg(client,AP3xx6_ADATA_L, 0xFF, 0x00);
	if(als_value_low[0] < 0)
	{
		goto EXIT_ERR;
	}

	als_value_high[0] = AP3xx6_read_reg(client,AP3xx6_ADATA_H, 0xFF, 0x00);
	if(als_value_high[0] < 0)
	{
		goto EXIT_ERR;
	}

	*data = als_value_low[0] | (als_value_high[0]<<8);

	if (*data < 0)
	{
		*data = 0;
		APS_DBG("als_value is invalid!!\n");
		goto EXIT_ERR;
	}

	return 0;

EXIT_ERR:
	APS_ERR("AP3xx6__read_als fail\n");
	return -1;
}
/*----------------------------------------------------------------------------*/
static int AP3xx6_get_als_value(struct AP3xx6_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als < obj->hw->als_level[idx])
		{
			break;
		}
	}
	
	if(idx >= obj->als_value_num)
	{
		APS_ERR("exceed range\n"); 
		idx = obj->als_value_num - 1;
	}
	
	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}
		
		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);	
		return obj->hw->als_value[idx];
	}
	else
	{
		APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);    
		return -1;
	}
}
/*----------------------------------------------------------------------------*/
int AP3xx6_read_ps(struct i2c_client *client, u16 *data)
{
	struct AP3xx6_priv *obj = i2c_get_clientdata(client);       
	u8 ps_value_low[1], ps_value_high[1];

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUAL NULL\n");
		return -1;
	}
	ps_value_low[0] = AP3xx6_read_reg(client,AP3xx6_PDATA_L, 0xFF, 0x00);
	if(ps_value_low[0] < 0)
	{
		goto EXIT_ERR;
	}

	ps_value_high[0] = AP3xx6_read_reg(client,AP3xx6_PDATA_H, 0xFF, 0x00);
	if(ps_value_high[0] < 0)
	{
		goto EXIT_ERR;
	}

#ifdef AP3426
	*data = (ps_value_low[0] & 0xFF) | ((ps_value_high[0] & 0x03) << 8);
#else
	*data = (ps_value_low[0] & 0x0f) | ((ps_value_high[0] & 0x3f) << 4);
#endif

	return 0;    

EXIT_ERR:
	printk("AP3xx6_read_ps fail\n");
	return -1;
}
/*----------------------------------------------------------------------------*/
/* 
   for AP3xx6_get_ps_value:
	return 1 = object close,
	return 0 = object far away. 2012/5/10 YC   // exchange 0 and 1 2012/5/30 YC 
*/
static int AP3xx6_get_ps_value(struct AP3xx6_priv *obj, u16 ps)
{
	int val;
	int invalid = 0;

	if(ps > atomic_read(&obj->ps_thd_val_h))
		val = 0;  /*close*/
	else if(ps < atomic_read(&obj->ps_thd_val_l))
		val = 1;  /*far away*/

	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	else if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (1 == atomic_read(&obj->ps_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		APS_DBG("PS:  %05d => %05d\n", ps, val);
		return val;
	}	
	else
	{
		return -1;
	}	
}

static int AP3xx6_get_OBJ(struct i2c_client *client)
{
     
	u8 ps_value_high[1];

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUAL NULL\n");
		return -1;
	}
#ifdef AP3426
	ps_value_high[0] = AP3xx6_read_reg(client,AP3xx6_INT_STATUS, 0xFF, 0x00);
	if(ps_value_high[0] < 0)
	{
		goto EXIT_ERR;
	}
	return !((ps_value_high[0]&0x10)>>4);	
#else
	ps_value_high[0] = AP3xx6_read_reg(client,AP3xx6_PDATA_H, 0xFF, 0x00);
	if(ps_value_high[0] < 0)
	{
		goto EXIT_ERR;
	}
	// printk("the ps_value_h>>7 is %d\n",ps_value_high[0]>>7);
	return !(ps_value_high[0]>>7);
#endif
  
EXIT_ERR:
	APS_ERR("AP3xx6_get_obj fail\n");
	return 0;
}


/*----------------------------------------------------------------------------*/
static void AP3xx6_eint_work(struct work_struct *work)
{
	struct AP3xx6_priv *obj = (struct AP3xx6_priv *)container_of(work, struct AP3xx6_priv, eint_work);
	int err;
	int data;
#ifdef CUSTOM_KERNEL_SENSORHUB
    int res = 0;

	data = AP3xx6_get_ps_value(obj, obj->ps);
    res = ps_report_interrupt_data(data);
    if(res != 0)
    {
        APS_ERR("%s err: %d\n", __func__, res);
    }
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	int res = 0;

	if((err = AP3xx6_check_and_clear_intr(obj->client)) < 0)
	{
		APS_ERR("AP3xx6_eint_work check intrs: %d\n", err);
	}
	else if (err & 0x01)
	{
		// ALS interrupt. User should add their code here if they want to handle ALS Int.
	}
	else if (err & 0x02)
	{
		AP3xx6_read_ps(obj->client, &obj->ps);
		data = AP3xx6_get_ps_value(obj, obj->ps);

		//let up layer to know
		APS_LOG("ap3216c interrupt value = %d\n", data);
		res = ps_report_interrupt_data(data);	
	}
	
	mt_eint_unmask(CUST_EINT_ALS_NUM);      
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
}

/*----------------------------------------------------------------------------*/

static ssize_t AP3xx6_em_read(struct device_driver *ddri, char *buf)
{

    int idx=0,count=0;
		int reg_value[1];
#ifdef AP3426		
	  #define AP3xx6_NUM_CACHABLE_REGS	29 
		u8 AP3xx6_reg[AP3xx6_NUM_CACHABLE_REGS] = 
		{0x00,0x01,0x02,0x06,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,
	   0x10,0x14,0x1a,0x1b,0x1c,0x1d,
	   0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x28,0x29,0x2a,0x2b,0x2c,0x2d};
#else
	  #define AP3xx6_NUM_CACHABLE_REGS	26 
		u8 AP3xx6_reg[AP3xx6_NUM_CACHABLE_REGS] = 
		{0x00,0x01,0x02,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,
	   0x10,0x19,0x1a,0x1b,0x1c,0x1d,
	   0x20,0x21,0x22,0x23,0x24,0x28,0x29,0x2a,0x2b,0x2c,0x2d};
#endif	 
    if(!AP3xx6_obj)
    {
        APS_ERR("AP3xx6_obj is null!!\n");
    		count+=sprintf(buf+count, "AP3xx6_obj is null!! \n");
    		return count;
    }
    for(idx=0;idx<AP3xx6_NUM_CACHABLE_REGS;idx++){

			reg_value[0] = AP3xx6_read_reg(AP3xx6_obj->client,AP3xx6_reg[idx], 0xFF, 0x00);
		if(reg_value[0] < 0)
		{
    		count+=sprintf(buf+count, "i2c read_reg err \n");
    		return count;
		}
		  count+=sprintf(buf+count, "[%x]=0x%x \n",AP3xx6_reg[idx],reg_value[0]);
    }
    AP3xx6_read_ps(AP3xx6_obj->client,&idx);
    count+=sprintf(buf+count, "[ps]=%d \n",idx);
    return count;
}

static ssize_t AP3xx6_em_write(struct device_driver *ddri,char *buf, size_t count)
{

	int addr,val;
	int ret = 0;

    if(!AP3xx6_obj)
    {
        APS_ERR("AP3xx6_obj is null!!\n");
        return 0;
    }

	sscanf(buf, "%x %x", &addr, &val);

	printk("Reg[%x].Write [%x]..\n",addr,val);

    ret = AP3xx6_write_reg(AP3xx6_obj->client,addr, 0xFF,0x00,val);

	return count;
}
static DRIVER_ATTR(em,      S_IWUGO | S_IRUGO, AP3xx6_em_read,AP3xx6_em_write);
static struct device_attribute *AP3xx6_attr_list[] = {
            &driver_attr_em
};

static int AP3xx6_create_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(AP3xx6_attr_list)/sizeof(AP3xx6_attr_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        if(err = driver_create_file(driver, AP3xx6_attr_list[idx]))
        {
            APS_ERR("driver_create_file (%s) = %d\n", AP3xx6_attr_list[idx]->attr.name, err);
            break;
        }
    }
    return err;
}

/*----------------------------------------------------------------------------*/
static int AP3xx6_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(AP3xx6_attr_list)/sizeof(AP3xx6_attr_list[0]));

    if (!driver)
        return -EINVAL;

    for (idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, AP3xx6_attr_list[idx]);
    }

    return err;
}

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int AP3xx6_open(struct inode *inode, struct file *file)
{
	file->private_data = AP3xx6_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int AP3xx6_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
//static int AP3xx6_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
//       unsigned long arg)
static int AP3xx6_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct AP3xx6_priv *obj = i2c_get_clientdata(client);  
	int err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;

	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if(err = AP3xx6_enable_ps(obj->client, 1))
				{
					APS_ERR("enable ps fail: %d\n", err); 
					goto err_out;
				}
				
#if defined(UNUSE_ENREAD)
                atomic_set(&obj->enable,atomic_read(&obj->enable)|0x02);
#else
				set_bit(CMC_BIT_PS, &obj->enable);                
#endif
			}
			else
			{
				if(err = AP3xx6_enable_ps(obj->client, 0))
				{
					APS_ERR("disable ps fail: %d\n", err); 
					goto err_out;
				}
				
#if defined(UNUSE_ENREAD)
                atomic_set(&obj->enable,atomic_read(&obj->enable)&0xFD);
#else
				clear_bit(CMC_BIT_PS, &obj->enable);                
#endif
			}
			break;

		case ALSPS_GET_PS_MODE:
#if defined(UNUSE_ENREAD)
            enable = (atomic_read(&obj->enable)&0x02) ? (1) : (0);
#else
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);            	
#endif
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:    
			if(err = AP3xx6_read_ps(obj->client, &obj->ps))
			{
				goto err_out;
			}
			
			dat = AP3xx6_get_ps_value(obj, obj->ps);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_RAW_DATA:    
			if(err = AP3xx6_read_ps(obj->client, &obj->ps))
			{
				goto err_out;
			}
			
			dat = obj->ps;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;              

		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if(err = AP3xx6_enable_als(obj->client, 1))
				{
					APS_ERR("enable als fail: %d\n", err); 
					goto err_out;
				}
#if defined(UNUSE_ENREAD)
                atomic_set(&obj->enable,atomic_read(&obj->enable)|0x01);
#else
				set_bit(CMC_BIT_ALS, &obj->enable);                
#endif
			}
			else
			{
				if(err = AP3xx6_enable_als(obj->client, 0))
				{
					APS_ERR("disable als fail: %d\n", err); 
					goto err_out;
				}
#if defined(UNUSE_ENREAD)
                atomic_set(&obj->enable,atomic_read(&obj->enable)&0xFE);
#else
				clear_bit(CMC_BIT_ALS, &obj->enable);                
#endif
			}
			break;

		case ALSPS_GET_ALS_MODE:
#if defined(UNUSE_ENREAD)
            enable=(atomic_read(&obj->enable)&0x01) ? (1) : (0);
#else
			enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);            	
#endif
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA: 
			if(err = AP3xx6_read_als(obj->client, &obj->als))
			{
				goto err_out;
			}

			dat = AP3xx6_get_als_value(obj, obj->als);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		case ALSPS_GET_ALS_RAW_DATA:    
			if(err = AP3xx6_read_als(obj->client, &obj->als))
			{
				goto err_out;
			}

			dat = obj->als;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		default:
			//APS_ERR("%s not supported = 0x%04x,ALSPS_GET_PS_RAW_DATA=%04lx", __FUNCTION__, cmd,ALSPS_GET_PS_RAW_DATA);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}
/*----------------------------------------------------------------------------*/
static struct file_operations AP3xx6_fops = {
//	.owner = THIS_MODULE,
	.open = AP3xx6_open,
	.release = AP3xx6_release,
//	.ioctl = AP3xx6_ioctl,
	.unlocked_ioctl = AP3xx6_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice AP3xx6_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &AP3xx6_fops,
};
/*----------------------------------------------------------------------------*/
static int AP3xx6_i2c_suspend(struct i2c_client *client, pm_message_t msg) 
{
	APS_FUN();    
	return 0;
}
/*----------------------------------------------------------------------------*/
static int AP3xx6_i2c_resume(struct i2c_client *client)
{
	APS_FUN();
	return 0;
}
/*----------------------------------------------------------------------------*/
static void AP3xx6_early_suspend(struct early_suspend *h) 
{   
	struct AP3xx6_priv *obj = container_of(h, struct AP3xx6_priv, early_drv);   
	int err;
	APS_FUN();    

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

	atomic_set(&obj->als_suspend, 1);
#if defined(UNUSE_ENREAD)
    if(atomic_read(&obj->enable)&0x01)
#else
	if(test_bit(CMC_BIT_ALS, &obj->enable))    	
#endif
	{
		if(err = AP3xx6_enable_als(obj->client, 0))
		{
			APS_ERR("disable als fail: %d\n", err); 
		}
	}
#if defined(USE_WAKELOCK)		
	#if defined(UNUSE_ENREAD)//20131209
		if (atomic_read(&obj->enable)&0x02)
	#else
		if(test_bit(CMC_BIT_PS, &obj->enable))    	
	#endif
	{
		APS_DBG("USE_WAKELOCK\n");
		wake_lock(&chrg_lock);
	}
#endif
}
/*----------------------------------------------------------------------------*/
static void AP3xx6_late_resume(struct early_suspend *h)
{   
	struct AP3xx6_priv *obj = container_of(h, struct AP3xx6_priv, early_drv);         
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

	atomic_set(&obj->als_suspend, 0);
#if defined(UNUSE_ENREAD)
    if(atomic_read(&obj->enable)&0x01)
#else
	if(test_bit(CMC_BIT_ALS, &obj->enable))    	
#endif
	{
		if(err = AP3xx6_enable_als(obj->client, 1))
		{
			APS_ERR("enable als fail: %d\n", err);        

		}
	}
#if defined(USE_WAKELOCK)
	#if defined(UNUSE_ENREAD)//20131209
		if (atomic_read(&obj->enable)&0x02)
	#else
		if(test_bit(CMC_BIT_PS, &obj->enable))    	
	#endif
		{
		wake_unlock(&chrg_lock);
	  }
#endif		
}

int AP3xx6_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct AP3xx6_priv *obj = (struct AP3xx6_priv *)self;
	
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{				
				value = *(int *)buff_in;
				if(value)
				{
					if(err = AP3xx6_enable_ps(obj->client, 1))
					{
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}
#if defined(UNUSE_ENREAD)
                atomic_set(&obj->enable,atomic_read(&obj->enable)|0x02);
#else
					set_bit(CMC_BIT_PS, &obj->enable);                
#endif
				}
				else
				{
					if(err = AP3xx6_enable_ps(obj->client, 0))
					{
						APS_ERR("disable ps fail: %d\n", err); 
						return -1;
					}
#if defined(UNUSE_ENREAD)
                atomic_set(&obj->enable,atomic_read(&obj->enable)&0xFD);
#else
					clear_bit(CMC_BIT_PS, &obj->enable);                
#endif
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;	
				AP3xx6_read_ps(obj->client, &obj->ps);
				
				sensor_data->values[0] = AP3xx6_get_ps_value(obj, obj->ps);
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;			
			}

			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}
static int temp_als = 0;
int AP3xx6_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct AP3xx6_priv *obj = (struct AP3xx6_priv *)self;

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;				
				if(value)
				{
					if(err = AP3xx6_enable_als(obj->client, 1))
					{
						APS_ERR("enable als fail: %d\n", err); 
						return -1;
					}
#if defined(UNUSE_ENREAD)
					atomic_set(&obj->enable,atomic_read(&obj->enable)|0x01);
#else
					set_bit(CMC_BIT_ALS, &obj->enable);					
#endif
				}
				else
				{
					if(err = AP3xx6_enable_als(obj->client, 0))
					{
						APS_ERR("disable als fail: %d\n", err); 
						return -1;
					}
#if defined(UNUSE_ENREAD)
					atomic_set(&obj->enable,atomic_read(&obj->enable)&0xFE);
#else
					clear_bit(CMC_BIT_ALS, &obj->enable);					
#endif
				}
				
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;
				AP3xx6_read_als(obj->client, &obj->als);
				APS_LOG("AP3xx6 ALS level=%d\n",obj->als);
				#if defined(MTK_AAL_SUPPORT)
				sensor_data->values[0] = obj->als;
				#else			
				if(obj->als == 0)
				{
					sensor_data->values[0] = temp_als;
				}else{
					u16 b[2];
					int i;
					for(i = 0;i < 2;i++){
						AP3xx6_read_als(obj->client, &obj->als);
						b[i] = obj->als;
					}
					(b[1] > b[0])?(obj->als = b[0]):(obj->als = b[1]);
					sensor_data->values[0] = AP3xx6_get_als_value(obj, obj->als);
					temp_als = sensor_data->values[0];
				}
				#endif
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;
		default:
			APS_ERR("light sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}


/*--------------------------------------------------------------------------------*/
static int als_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int als_enable_nodata(int en)
{
	int res = 0;
	if(!AP3xx6_obj)
	{
		APS_ERR("ap3xx6_obj is null!!\n");
		return -1;
	}
	APS_LOG("ap3xx6_obj als enable value = %d\n", en);

#ifdef CUSTOM_KERNEL_SENSORHUB
    if(en)
	{
		if((res = AP3xx6_enable_als(AP3xx6_obj->client, 1)))
		{
			APS_ERR("enable als fail: %d\n", res); 
			return -1;
		}
		set_bit(CMC_BIT_ALS, &AP3xx6_obj->enable);
	}
	else
	{
		if((res = AP3xx6_enable_als(AP3xx6_obj->client, 0)))
		{
			APS_ERR("disable als fail: %d\n", res); 
			return -1;
		}
		clear_bit(CMC_BIT_ALS, &AP3xx6_obj->enable);
	}
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
    if(en)
	{
		if((res = AP3xx6_enable_als(AP3xx6_obj->client, 1)))
		{
			APS_ERR("enable als fail: %d\n", res); 
			return -1;
		}
		set_bit(CMC_BIT_ALS, &AP3xx6_obj->enable);
	}
	else
	{
		if((res = AP3xx6_enable_als(AP3xx6_obj->client, 0)))
		{
			APS_ERR("disable als fail: %d\n", res); 
			return -1;
		}
		clear_bit(CMC_BIT_ALS, &AP3xx6_obj->enable);
	}
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
    
	if(res){
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}
	return 0;
}
/*--------------------------------------------------------------------------------*/
static int als_set_delay(u64 ns)
{
	return 0;
}
/*--------------------------------------------------------------------------------*/
static int als_get_data(int* value, int* status)
{
	int err = 0;

	if(!AP3xx6_obj)
	{
		APS_ERR("ap3xx6_obj is null!!\n");
		return -1;
	}

#ifdef CUSTOM_KERNEL_SENSORHUB
	if((err = AP3xx6_read_als(AP3xx6_obj->client, &AP3xx6_obj->als)))
	{
		err = -1;
	}
	else
	{
		*value = AP3xx6_get_als_value(AP3xx6_obj, AP3xx6_obj->als);
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
    if((err = AP3xx6_read_als(AP3xx6_obj->client, &AP3xx6_obj->als)))
	{
		err = -1;
	}
	else
	{
		*value = AP3xx6_get_als_value(AP3xx6_obj, AP3xx6_obj->als);
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

	return err;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int ps_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , alsps sensor only enabled but not report inputEvent to HAL
static int ps_enable_nodata(int en)
{
	int res = 0;
	if(!AP3xx6_obj)
	{
		APS_ERR("ap3xx6_obj is null!!\n");
		return -1;
	}
	APS_LOG("ap3xx6_obj als enable value = %d\n", en);

#ifdef CUSTOM_KERNEL_SENSORHUB
    if(en)
	{
		if((res = AP3xx6_enable_ps(AP3xx6_obj->client, 1)))
		{
			APS_ERR("enable ps fail: %d\n", res); 
			return -1;
		}
		set_bit(CMC_BIT_PS, &AP3xx6_obj->enable);
	}
	else
	{
		if((res = AP3xx6_enable_ps(AP3xx6_obj->client, 0)))
		{
			APS_ERR("disable ps fail: %d\n", res); 
			return -1;
		}
		clear_bit(CMC_BIT_PS, &AP3xx6_obj->enable);
	}
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	if(en)
	{
		if((res = AP3xx6_enable_ps(AP3xx6_obj->client, 1))!=0)
		{
			APS_ERR("enable ps fail: %d\n", res);
			return -1;
		}
		set_bit(CMC_BIT_PS, &AP3xx6_obj->enable);
	}
	else
	{
		if((res = AP3xx6_enable_ps(AP3xx6_obj->client, 0))!=0)
		{
			APS_ERR("disable ps fail: %d\n", res);
			return -1;
		}
		clear_bit(CMC_BIT_PS, &AP3xx6_obj->enable);
	}
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
    
	if(res){
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}
    
	return 0;

}
/*--------------------------------------------------------------------------------*/
static int ps_set_delay(u64 ns)
{
	return 0;
}
/*--------------------------------------------------------------------------------*/
static int ps_get_data(int* value, int* status)
{
    int err = 0;

    if(!AP3xx6_obj)
	{
		APS_ERR("ap3xx6_obj is null!!\n");
		return -1;
	}

#ifdef CUSTOM_KERNEL_SENSORHUB
	if((err = AP3xx6_read_ps(AP3xx6_obj->client, &AP3xx6_obj->ps)))
    {
        err = -1;;
    }
    else
    {
        *value = AP3xx6_get_ps_value(AP3xx6_obj, AP3xx6_obj->ps);
        *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    }
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	if((err = AP3xx6_read_ps(AP3xx6_obj->client, &AP3xx6_obj->ps)))
    {
        err = -1;;
    }
    else
    {
        *value = AP3xx6_get_ps_value(AP3xx6_obj, AP3xx6_obj->ps);
        *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    }
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
    
	return err;
}


/*----------------------------------------------------------------------------*/
static int AP3xx6_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) 
{    
	strcpy(info->type, AP3xx6_DEV_NAME);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int AP3xx6_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct AP3xx6_priv *obj;
    struct als_control_path als_ctl={0};
	struct als_data_path als_data={0};
	struct ps_control_path ps_ctl={0};
	struct ps_data_path ps_data={0};

	int err = 0;

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	AP3xx6_obj = obj;

	mutex_init(&obj->lock);
	obj->hw = get_cust_alsps_hw();
	AP3xx6_get_addr(obj->hw, &obj->addr);
#if DELAYED_WORK
	INIT_DELAYED_WORK(&obj->eint_work, AP3xx6_eint_work);
#else
	INIT_WORK(&obj->eint_work, AP3xx6_eint_work);
#endif
	obj->client = client;
	i2c_set_clientdata(client, obj);	
	atomic_set(&obj->als_debounce, 300);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 300);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->als_cmd_val, 0xDF);
	atomic_set(&obj->ps_cmd_val,  0xC1);
	atomic_set(&obj->ps_thd_val_h,  obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_l,  obj->hw->ps_threshold_low);
#if defined(UNUSE_ENREAD)
    atomic_set(&obj->enable,0);
#else
	obj->enable = 0;    
	set_bit(CMC_BIT_ALS, &obj->enable);
	set_bit(CMC_BIT_PS, &obj->enable);
#endif
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);  
	obj->als_modulus = (400*100*40)/(1*1500);

	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	
	AP3xx6_i2c_client = client;

	
	if(err = AP3xx6_init_client(client))
	{
		goto exit_init_failed;
	}
	APS_LOG("AP3xx6_init_client() OK!\n");
	//extern int hw_module_info_add_item(char *type, char *chip, char *vendor, int id, char *more);
	//hw_module_info_add_item("ALS/PS", AP3xx6_DEV_NAME, " ", 0x1E, " ");
	if(err = misc_register(&AP3xx6_device))
	{
		APS_ERR("AP3xx6_device register failed\n");
		goto exit_misc_device_register_failed;
	}
	/*------------------------ap3216c attribute file for debug--------------------------------------*/
	if((err = AP3xx6_create_attr(&ap3xx6_init_info.platform_diver_addr->driver)))
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	
	/*------------------------ap3216c attribute file for debug--------------------------------------*/
	als_ctl.open_report_data= als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay  = als_set_delay;
	als_ctl.is_report_input_direct = false;
#ifdef CUSTOM_KERNEL_SENSORHUB
	als_ctl.is_support_batch = true;
#else
    als_ctl.is_support_batch = false;
#endif
	
	err = als_register_control_path(&als_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_register_path_failed;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	err = als_register_data_path(&als_data);	
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_register_path_failed;
	}

	
	ps_ctl.open_report_data= ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay  = ps_set_delay;
	ps_ctl.is_report_input_direct = true;
#ifdef CUSTOM_KERNEL_SENSORHUB
	ps_ctl.is_support_batch = true;
#else
    ps_ctl.is_support_batch = false;
#endif
	
	err = ps_register_control_path(&ps_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_register_path_failed;
	}

	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);	
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_register_path_failed;
	}

APS_LOG("hwmsen_attach OK.%s: \n", __func__);

#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = AP3xx6_early_suspend,
	obj->early_drv.resume   = AP3xx6_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif

#if defined(USE_WAKELOCK)
	wake_lock_init(&chrg_lock, WAKE_LOCK_SUSPEND, "AP3xx6_wake_lock");
#endif	
	APS_LOG("%s: OK\n", __func__);
	return 0;

	exit_register_path_failed:
	AP3xx6_delete_attr(&ap3xx6_init_info.platform_diver_addr->driver);
	exit_create_attr_failed:
	misc_deregister(&AP3xx6_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	mutex_destroy(&AP3xx6_obj->lock);		
	exit_kfree:
	kfree(obj);
	exit:
	AP3xx6_i2c_client = NULL;           
	APS_ERR("%s: err = %d\n", __func__, err);
	return err;
}
/*----------------------------------------------------------------------------*/
static int AP3xx6_i2c_remove(struct i2c_client *client)
{
	int err;	

	if(err = misc_deregister(&AP3xx6_device))
	{
		APS_ERR("misc_deregister fail: %d\n", err);    
	}
	
	AP3xx6_i2c_client = NULL;
	i2c_unregister_device(client);
	mutex_destroy(&AP3xx6_obj->lock);
	kfree(i2c_get_clientdata(client));
#if defined(USE_WAKELOCK)	
	wake_lock_destroy(&chrg_lock);
#endif
	return 0;
}

static int __init AP3xx6_init(void)
{
	APS_FUN();
	struct alsps_hw *hw = get_cust_alsps_hw();
	APS_LOG("%s: i2c_number=%d\n", __func__,hw->i2c_num); 
	i2c_register_board_info(hw->i2c_num, &i2c_AP3xx6, 1);
	alsps_driver_add(&ap3xx6_init_info);

	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit AP3xx6_exit(void)
{
	APS_FUN();
	platform_driver_unregister(&AP3xx6_alsps_driver);
}


module_init(AP3xx6_init);
module_exit(AP3xx6_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("YC Hou");
MODULE_DESCRIPTION("AP3xx6 driver");
MODULE_LICENSE("GPL");
