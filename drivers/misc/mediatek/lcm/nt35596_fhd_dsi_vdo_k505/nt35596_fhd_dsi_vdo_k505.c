/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"
#include "tps65132_i2c.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#endif
#include <cust_i2c.h>
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define LCM_ID										0x96//0x9881

#define LCM_DSI_CMD_MODE								0

#define FRAME_WIDTH									(1080)
#define FRAME_HEIGHT									(1920)
#define REGFLAG_DELAY             							0xfc//0XAA
#define REGFLAG_END_OF_TABLE      						0xfd//0xAB   // END OF REGISTERS MARKER

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)									(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)				lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)					lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)												lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)								lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)												lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)							lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

//#ifndef BUILD_LK
#if 0
#include <linux/kernel.h>
#include <linux/module.h>  
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
//#include <linux/jiffies.h>
#include <linux/uaccess.h>
//#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
/***************************************************************************** 
 * Define
 *****************************************************************************/

#define TPS_I2C_BUSNUM  I2C_I2C_LCD_BIAS_CHANNEL//for I2C channel 0
#define I2C_ID_NAME "tps65132"
#define TPS_ADDR 0x3E

/***************************************************************************** 
 * GLobal Variable
 *****************************************************************************/
static struct i2c_board_info __initdata tps65132_board_info = {I2C_BOARD_INFO(I2C_ID_NAME, TPS_ADDR)};
static struct i2c_client *tps65132_i2c_client = NULL;


/***************************************************************************** 
 * Function Prototype
 *****************************************************************************/ 
static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tps65132_remove(struct i2c_client *client);
/***************************************************************************** 
 * Data Structure
 *****************************************************************************/

 struct tps65132_dev	{	
	struct i2c_client	*client;
	
};

static const struct i2c_device_id tps65132_id[] = {
	{ I2C_ID_NAME, 0 },
	{ }
};

//#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36))
//static struct i2c_client_address_data addr_data = { .forces = forces,};
//#endif
static struct i2c_driver tps65132_iic_driver = {
	.id_table	= tps65132_id,
	.probe		= tps65132_probe,
	.remove		= tps65132_remove,
	//.detect		= mt6605_detect,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "tps65132",
	},
 
};
/***************************************************************************** 
 * Extern Area
 *****************************************************************************/ 
 
 

/***************************************************************************** 
 * Function
 *****************************************************************************/ 
static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id)
{  
	printk( "tps65132_iic_probe\n");
	printk("TPS: info==>name=%s addr=0x%x\n",client->name,client->addr);
	tps65132_i2c_client  = client;		
	return 0;      
}


static int tps65132_remove(struct i2c_client *client)
{  	
  printk( "tps65132_remove\n");
  tps65132_i2c_client = NULL;
   i2c_unregister_device(client);
  return 0;
}


 static int tps65132_write_bytes(unsigned char addr, unsigned char value)
{	
	int ret = 0;
	struct i2c_client *client = tps65132_i2c_client;
	char write_data[2]={0};	
	write_data[0]= addr;
	write_data[1] = value;
    ret=i2c_master_send(client, write_data, 2);
	if(ret<0)
	printk("tps65132 write data fail !!\n");	
	return ret ;
}



/*
 * module load/unload record keeping
 */

static int __init tps65132_iic_init(void)
{

   printk( "tps65132_iic_init\n");
   i2c_register_board_info(TPS_I2C_BUSNUM, &tps65132_board_info, 1);
   printk( "tps65132_iic_init2\n");
   i2c_add_driver(&tps65132_iic_driver);
   printk( "tps65132_iic_init success\n");	
   return 0;
}

static void __exit tps65132_iic_exit(void)
{
  printk( "tps65132_iic_exit\n");
  i2c_del_driver(&tps65132_iic_driver);  
}


module_init(tps65132_iic_init);
module_exit(tps65132_iic_exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("MTK TPS65132 I2C Driver");
MODULE_LICENSE("GPL"); 

#endif

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};



static struct LCM_setting_table lcm_initialization_setting[] = {
	
	{0xFF,1,{0xEE}},
	{0x7C,1,{0x31}},
	{0xFB,1,{0x01}},
	{0xFF,1,{0x01}},
	{0x00,1,{0x01}},
	{0x01,1,{0x55}},
	{0x02,1,{0x40}},
	{0x05,1,{0x50}},
	{0x06,1,{0x4A}},
	{0x07,1,{0x29}},
	{0x08,1,{0x0C}},
	{0x0B,1,{0x87}},
	{0x0C,1,{0x87}},
	{0x0E,1,{0xB0}},
	{0x0F,1,{0xB3}},
	{0x11,1,{0x0c}},
	{0x12,1,{0x0c}},
	{0x13,1,{0x03}},
	{0x14,1,{0x4A}},
	{0x15,1,{0x12}},
	{0x16,1,{0x12}},
	{0x18,1,{0x00}},
	{0x19,1,{0x77}},
	{0x1A,1,{0x55}},
	{0x1B,1,{0x13}},
	{0x1C,1,{0x00}},
	{0x1D,1,{0x00}},
	{0x1E,1,{0x00}},
	{0x1F,1,{0x00}},
	{0x58,1,{0x82}},
	{0x59,1,{0x00}},
	{0x5A,1,{0x02}},
	{0x5B,1,{0x00}},
	{0x5C,1,{0x82}},
	{0x5D,1,{0x80}},
	{0x5E,1,{0x02}},
	{0x5F,1,{0x00}},
	{0x72,1,{0x31}},
	{0xFB,1,{0x01}},
	{0xFF,1,{0x01}},
	{0xFB,1,{0x01}},
	{0x75,1,{0x01}},
	{0x76,1,{0x46}},
	{0x77,1,{0x01}},
	{0x78,1,{0x4B}},
	{0x79,1,{0x01}},
	{0x7A,1,{0x57}},
	{0x7B,1,{0x01}},
	{0x7C,1,{0x61}},
	{0x7D,1,{0x01}},
	{0x7E,1,{0x68}},
	{0x7F,1,{0x01}},
	{0x80,1,{0x71}},
	{0x81,1,{0x01}},
	{0x82,1,{0x7B}},
	{0x83,1,{0x01}},
	{0x84,1,{0x81}},
	{0x85,1,{0x01}},
	{0x86,1,{0x87}},
	{0x87,1,{0x01}},
	{0x88,1,{0xA1}},
	{0x89,1,{0x01}},
	{0x8A,1,{0xB7}},
	{0x8B,1,{0x01}},
	{0x8C,1,{0xD8}},
	{0x8D,1,{0x01}},
	{0x8E,1,{0xFB}},
	{0x8F,1,{0x02}},
	{0x90,1,{0x2F}},
	{0x91,1,{0x02}},
	{0x92,1,{0x58}},
	{0x93,1,{0x02}},
	{0x94,1,{0x5A}},
	{0x95,1,{0x02}},
	{0x96,1,{0x83}},
	{0x97,1,{0x02}},
	{0x98,1,{0xB1}},
	{0x99,1,{0x02}},
	{0x9A,1,{0xD0}},
	{0x9B,1,{0x02}},
	{0x9C,1,{0xFC}},
	{0x9D,1,{0x03}},
	{0x9E,1,{0x19}},
	{0x9F,1,{0x03}},
	{0xA0,1,{0x4A}},
	{0xA2,1,{0x03}},
	{0xA3,1,{0x54}},
	{0xA4,1,{0x03}},
	{0xA5,1,{0x61}},
	{0xA6,1,{0x03}},
	{0xA7,1,{0x71}},
	{0xA9,1,{0x03}},
	{0xAA,1,{0x7F}},
	{0xAB,1,{0x03}},
	{0xAC,1,{0x8C}},
	{0xAD,1,{0x03}},
	{0xAE,1,{0xA4}},
	{0xAF,1,{0x03}},
	{0xB0,1,{0xFC}},
	{0xB1,1,{0x03}},
	{0xB2,1,{0xFF}},
	{0xB3,1,{0x01}},
	{0xB4,1,{0x46}},     
	{0xB5,1,{0x01}},
	{0xB6,1,{0x4B}},
  {0xB7,1,{0x01}},
  {0xB8,1,{0x57}},
  {0xB9,1,{0x01}},
  {0xBA,1,{0x61}},
  {0xBB,1,{0x01}},
  {0xBC,1,{0x68}},
  {0xBD,1,{0x01}},
  {0xBE,1,{0x71}},
  {0xBF,1,{0x01}},
  {0xC0,1,{0x7B}},
  {0xC1,1,{0x01}},
  {0xC2,1,{0x81}},
  {0xC3,1,{0x01}},
  {0xC4,1,{0x87}},
  {0xC5,1,{0x01}},
  {0xC6,1,{0xA1}},
  {0xC7,1,{0x01}},
  {0xC8,1,{0xB7}},
  {0xC9,1,{0x01}},
  {0xCA,1,{0xD8}},
  {0xCB,1,{0x01}},
  {0xCC,1,{0xFB}},
  {0xCD,1,{0x02}},
  {0xCE,1,{0x2F}},
	{0xCF,1,{0x02}},
	{0xD0,1,{0x58}},
	{0xD1,1,{0x02}},
	{0xD2,1,{0x5A}},
	{0xD3,1,{0x02}},
	{0xD4,1,{0x83}},
	{0xD5,1,{0x02}},
	{0xD6,1,{0xB1}},
	{0xD7,1,{0x02}},
	{0xD8,1,{0xD0}},  
	{0xD9,1,{0x02}},  
	{0xDA,1,{0xFC}},  
	{0xDB,1,{0x03}},  
	{0xDC,1,{0x19}},  
	{0xDD,1,{0x03}},  
	{0xDE,1,{0x4A}},  
	{0xDF,1,{0x03}},  
	{0xE0,1,{0x54}},  
	{0xE1,1,{0x03}},  
	{0xE2,1,{0x61}},  
	{0xE3,1,{0x03}},      
	{0xE4,1,{0x71}},  
	{0xE5,1,{0x03}},  
	{0xE6,1,{0x7F}},
	{0xE7,1,{0x03}},
	{0xE8,1,{0x8C}},
	{0xE9,1,{0x03}},
	{0xEA,1,{0xA4}},
	{0xEB,1,{0x03}},
	{0xEC,1,{0xFC}},
	{0xED,1,{0x03}},
	{0xEE,1,{0xFF}},
	{0xEF,1,{0x00}},
	{0xF0,1,{0x25}},
	{0xF1,1,{0x00}},
	{0xF2,1,{0x73}},
	{0xF3,1,{0x00}},
	{0xF4,1,{0xA1}},
	{0xF5,1,{0x00}},
	{0xF6,1,{0xBB}},
	{0xF7,1,{0x00}},
	{0xF8,1,{0xCF}},
	{0xF9,1,{0x00}},
	{0xFA,1,{0xE6}},
	{0xFF,1,{0x02}},
	{0xFB,1,{0x01}},
	{0x00,1,{0x00}},
	{0x01,1,{0xF6}},
	{0x02,1,{0x01}},
	{0x03,1,{0x04}},
	{0x04,1,{0x01}},
	{0x05,1,{0x0F}},
	{0x06,1,{0x01}},
	{0x07,1,{0x3C}},
	{0x08,1,{0x01}},
	{0x09,1,{0x5E}},
	{0x0A,1,{0x01}},
	{0x0B,1,{0x92}},
	{0x0C,1,{0x01}},
	{0x0D,1,{0xBF}},
	{0x0E,1,{0x02}},
	{0x0F,1,{0x02}},
	{0x10,1,{0x02}},
	{0x11,1,{0x36}},
	{0x12,1,{0x02}},
	{0x13,1,{0x39}},
	{0x14,1,{0x02}},
	{0x15,1,{0x69}},
	{0x16,1,{0x02}},
	{0x17,1,{0x9E}},
	{0x18,1,{0x02}},
	{0x19,1,{0xC1}},
	{0x1A,1,{0x02}},
	{0x1B,1,{0xF1}},
	{0x1C,1,{0x03}},
	{0x1D,1,{0x11}},
	{0x1E,1,{0x03}},
	{0x1F,1,{0x3B}},
	{0x20,1,{0x03}},
	{0x21,1,{0x48}},
	{0x22,1,{0x03}},
	{0x23,1,{0x56}},
	{0x24,1,{0x03}},
	{0x25,1,{0x66}},
	{0x26,1,{0x03}},
	{0x27,1,{0x7B}},
	{0x28,1,{0x03}},
	{0x29,1,{0x8C}},
	{0x2A,1,{0x03}},
	{0x2B,1,{0xA3}},
	{0x2D,1,{0x03}},
	{0x2F,1,{0xC4}},
	{0x30,1,{0x03}},
	{0x31,1,{0xFF}},
	{0x32,1,{0x00}},
	{0x33,1,{0x25}},
	{0x34,1,{0x00}},
	{0x35,1,{0x73}},
	{0x36,1,{0x00}},
	{0x37,1,{0xA1}},
	{0x38,1,{0x00}},
	{0x39,1,{0xBB}},
	{0x3A,1,{0x00}},
	{0x3B,1,{0xCF}},
	{0x3D,1,{0x00}},
	{0x3F,1,{0xE6}},
	{0x40,1,{0x00}},
	{0x41,1,{0xF6}},
	{0x42,1,{0x01}},
	{0x43,1,{0x04}},
	{0x44,1,{0x01}},
	{0x45,1,{0x0F}},
	{0x46,1,{0x01}},
	{0x47,1,{0x3C}},
	{0x48,1,{0x01}},
	{0x49,1,{0x5E}},
	{0x4A,1,{0x01}},
	{0x4B,1,{0x92}},
	{0x4C,1,{0x01}},
	{0x4D,1,{0xBF}},
	{0x4E,1,{0x02}},
	{0x4F,1,{0x02}},
	{0x50,1,{0x02}},
	{0x51,1,{0x36}},
	{0x52,1,{0x02}},
	{0x53,1,{0x39}},
	{0x54,1,{0x02}},
	{0x55,1,{0x69}},
	{0x56,1,{0x02}},
	{0x58,1,{0x9E}},
	{0x59,1,{0x02}},
	{0x5A,1,{0xC1}},
	{0x5B,1,{0x02}},
	{0x5C,1,{0xF1}},
	{0x5D,1,{0x03}},
	{0x5E,1,{0x11}},
	{0x5F,1,{0x03}},
	{0x60,1,{0x3B}},
	{0x61,1,{0x03}},
	{0x62,1,{0x48}},
	{0x63,1,{0x03}},
	{0x64,1,{0x56}},
	{0x65,1,{0x03}},
	{0x66,1,{0x66}},
	{0x67,1,{0x03}},
	{0x68,1,{0x7B}},
	{0x69,1,{0x03}},
	{0x6A,1,{0x8C}},
	{0x6B,1,{0x03}},
	{0x6C,1,{0xA3}},
	{0x6D,1,{0x03}},
	{0x6E,1,{0xC4}},
	{0x6F,1,{0x03}},
	{0x70,1,{0xFF}},
	{0x71,1,{0x01}},
	{0x72,1,{0x6A}},
	{0x73,1,{0x01}},
	{0x74,1,{0x6E}},
	{0x75,1,{0x01}},
	{0x76,1,{0x74}},
	{0x77,1,{0x01}},
	{0x78,1,{0x7A}},
	{0x79,1,{0x01}},
	{0x7A,1,{0x7F}},
	{0x7B,1,{0x01}},
	{0x7C,1,{0x85}},
	{0x7D,1,{0x01}},
	{0x7E,1,{0x8B}},
	{0x7F,1,{0x01}},
	{0x80,1,{0x90}},
	{0x81,1,{0x01}},
	{0x82,1,{0x94}},
	{0x83,1,{0x01}},
	{0x84,1,{0xA8}},
	{0x85,1,{0x01}},
	{0x86,1,{0xBA}},
	{0x87,1,{0x01}},
	{0x88,1,{0xD9}},
	{0x89,1,{0x01}},
	{0x8A,1,{0xF6}},
	{0x8B,1,{0x02}},
	{0x8C,1,{0x26}},
	{0x8D,1,{0x02}},
	{0x8E,1,{0x50}},
	{0x8F,1,{0x02}},
	{0x90,1,{0x52}},
	{0x91,1,{0x02}},
	{0x92,1,{0x7C}},
	{0x93,1,{0x02}},
	{0x94,1,{0xAE}},
	{0x95,1,{0x02}},
	{0x96,1,{0xD1}},
	{0x97,1,{0x03}},
	{0x98,1,{0x03}},
	{0x99,1,{0x03}},
	{0x9A,1,{0x28}},
	{0x9B,1,{0x03}},
	{0x9C,1,{0x76}},
	{0x9D,1,{0x03}},
	{0x9E,1,{0x87}},
	{0x9F,1,{0x03}},
	{0xA0,1,{0x89}},
	{0xA2,1,{0x03}},
	{0xA3,1,{0x8B}},
	{0xA4,1,{0x03}},
	{0xA5,1,{0x8D}},
	{0xA6,1,{0x03}},
	{0xA7,1,{0x8F}},
	{0xA9,1,{0x03}},
	{0xAA,1,{0x99}},
	{0xAB,1,{0x03}},
	{0xAC,1,{0xF6}},
	{0xAD,1,{0x03}},
	{0xAE,1,{0xFF}},
	{0xAF,1,{0x01}},
	{0xB0,1,{0x6A}},
	{0xB1,1,{0x01}},
	{0xB2,1,{0x6E}},
	{0xB3,1,{0x01}},
	{0xB4,1,{0x74}},
	{0xB5,1,{0x01}},
	{0xB6,1,{0x7A}},
	{0xB7,1,{0x01}},
	{0xB8,1,{0x7F}},
	{0xB9,1,{0x01}},
	{0xBA,1,{0x85}},
	{0xBB,1,{0x01}},
	{0xBC,1,{0x8B}},
	{0xBD,1,{0x01}},
	{0xBE,1,{0x90}},
	{0xBF,1,{0x01}},
	{0xC0,1,{0x94}},
	{0xC1,1,{0x01}},
	{0xC2,1,{0xA8}},
	{0xC3,1,{0x01}},
	{0xC4,1,{0xBA}},
	{0xC5,1,{0x01}},
	{0xC6,1,{0xD9}},
	{0xC7,1,{0x01}},
	{0xC8,1,{0xF6}},
	{0xC9,1,{0x02}},
	{0xCA,1,{0x26}},
	{0xCB,1,{0x02}},
	{0xCC,1,{0x50}},
	{0xCD,1,{0x02}},
	{0xCE,1,{0x52}},
	{0xCF,1,{0x02}},
	{0xD0,1,{0x7C}},
	{0xD1,1,{0x02}},
	{0xD2,1,{0xAE}},
	{0xD3,1,{0x02}},
	{0xD4,1,{0xD1}},
	{0xD5,1,{0x03}},
	{0xD6,1,{0x03}},
	{0xD7,1,{0x03}},
	{0xD8,1,{0x28}},
	{0xD9,1,{0x03}},
	{0xDA,1,{0x76}},
	{0xDB,1,{0x03}},
	{0xDC,1,{0x87}},
	{0xDD,1,{0x03}},
	{0xDE,1,{0x89}},
	{0xDF,1,{0x03}},
	{0xE0,1,{0x8B}},
	{0xE1,1,{0x03}},
	{0xE2,1,{0x8D}},
	{0xE3,1,{0x03}},
	{0xE4,1,{0x8F}},
	{0xE5,1,{0x03}},
	{0xE6,1,{0x99}},
	{0xE7,1,{0x03}},
	{0xE8,1,{0xF6}},
	{0xE9,1,{0x03}},
	{0xEA,1,{0xFF}},
	{0xFF,1,{0x05}},
	{0x00,1,{0x01}},
	{0x01,1,{0x0B}},
	{0x02,1,{0x0C}},
	{0x03,1,{0x09}},
	{0x04,1,{0x0A}},
	{0x05,1,{0x1A}},
	{0x06,1,{0x10}},
	{0x07,1,{0x00}},
	{0x08,1,{0x1A}},
	{0x09,1,{0x00}},
	{0x0A,1,{0x00}},
	{0x0B,1,{0x00}},
	{0x0C,1,{0x00}},
	{0x0D,1,{0x13}},
	{0x0E,1,{0x15}},
	{0x0F,1,{0x17}},
	{0x10,1,{0x01}},
	{0x11,1,{0x0B}},
	{0x12,1,{0x0C}},
	{0x13,1,{0x09}},
	{0x14,1,{0x0A}},
	{0x15,1,{0x1A}},
	{0x16,1,{0x10}},
	{0x17,1,{0x10}},
	{0x18,1,{0x1A}},
	{0x19,1,{0x00}},
	{0x1A,1,{0x00}},
	{0x1B,1,{0x00}},
	{0x1C,1,{0x00}},
	{0x1D,1,{0x13}},  
	{0x1E,1,{0x15}},  
	{0x1F,1,{0x17}},  
	{0x20,1,{0x00}},  
	{0x21,1,{0x01}},              
	{0x22,1,{0x00}},
	{0x23,1,{0x40}},  
	{0x24,1,{0x40}},  
	{0x25,1,{0x6D}},  
	{0x29,1,{0xD8}},  
	{0x2A,1,{0x2A}},  
	{0x2B,1,{0x00}},  
	{0xB6,1,{0x89}},  
	{0xB7,1,{0x14}},  
	{0xB8,1,{0x05}},  
	{0x4B,1,{0x04}},  
	{0x4C,1,{0x11}},  
	{0x4D,1,{0x10}},      
	{0x4E,1,{0x01}},  
	{0x4F,1,{0x01}}, 
	{0x50,1,{0x10}},  
	{0x51,1,{0x00}},  
	{0x52,1,{0x00}},  
	{0x53,1,{0x08}},  
	{0x54,1,{0x01}},
  {0x55,1,{0x6D}},
	{0x5B,1,{0x44}},
	{0x5C,1,{0x00}},
	{0x5F,1,{0x74}},
	{0x60,1,{0x75}},
	{0x63,1,{0xFF}},
	{0x64,1,{0x00}},
	{0x67,1,{0x04}},
	{0x68,1,{0x04}},
	{0x6C,1,{0x00}},
	{0x7A,1,{0x80}},
	{0x7B,1,{0x91}},
	{0x7C,1,{0xD8}},
	{0x7D,1,{0x60}},
	{0x7F,1,{0x15}},
	{0x80,1,{0x00}},
	{0x83,1,{0x00}},
	{0x93,1,{0x08}},
	{0x94,1,{0x0A}},
	{0x8A,1,{0x33}},
	{0xA4,1,{0x0F}},
	{0x9B,1,{0x0F}},
	{0xFB,1,{0x01}},
	{0xFF,1,{0x01}},
	{0x15,1,{0x0F}},
	{0x16,1,{0x0F}},
	{0x1B,1,{0x1B}},
	{0x1C,1,{0xF7}},
	{0x60,1,{0x0F}},
	{0x58,1,{0x82}},
	{0x59,1,{0x00}},
	{0x5A,1,{0x02}},
	{0x5B,1,{0x00}},
	{0x5C,1,{0x82}},
	{0x5D,1,{0x80}},
	{0x5E,1,{0x02}},
	{0x5F,1,{0x00}},
	{0x66,1,{0x01}},
	{0xFB,1,{0x01}},
	{0xFF,1,{0x05}},
	{0x85,1,{0x05}},
	{0xA6,1,{0x04}},
	{0xFB,1,{0x01}},
	{0xFF,1,{0xFF}},
	{0x4F,1,{0x03}},
	{0xFB,1,{0x01}},
	{0xFF,1,{0x00}},
	{0xD3,1,{0x08}},
	{0xD4,1,{0x08}},
	{0x35,1,{0x00}},
	
	{0x11,1,{0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0x29,1,{0x00}},	
	{REGFLAG_DELAY, 20, {}},
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	
};


static struct LCM_setting_table lcm_sleep_mode_in_setting[] = {
	{0xFF, 1, {0x01}},
	{0xFB, 1, {0x01}},	
	{REGFLAG_DELAY, 10, {}},	
	{0x11, 1, {0x78}},
    {0x12, 1, {0x78}},
    {0x13, 1, {0x00}},
	{REGFLAG_DELAY, 10, {}},	
	{0x11, 1, {0x78}},
    {0x12, 1, {0x78}},
    {0x13, 1, {0x00}},	
	

	{REGFLAG_DELAY, 200, {}},
	
	{0xFF, 1, {0x00}},
	{0xFB, 1, {0x01}},
	// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 200, {}},

	// Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table lcm_sleep_out_setting[] = {
	// Sleep Out
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 20, {}},

	// Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table lcm_compare_id_setting[] = {
	// Display off sequence
	{0xF0,	5,	{0x55, 0xaa, 0x52,0x08,0x00}},
	{REGFLAG_DELAY, 10, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table page1_select[] = {
	//CMD_Page 1
	{0xFF, 3,{0x98,0x81,0x01}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for(i = 0; i < count; i++) {
		
		unsigned cmd;
		cmd = table[i].cmd;
		
	switch (cmd) {
			
		case REGFLAG_DELAY :
 			MDELAY(table[i].count);
   			break;
				
		case REGFLAG_END_OF_TABLE :
   			break;
				
		default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
			MDELAY(2);
		}
	}
	
}


static void init_lcm_registers(void)
{
	unsigned int data_array[16];
}
// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
	params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;//SYNC_EVENT_VDO_MODE;
#else
    params->dsi.mode   = SYNC_PULSE_VDO_MODE;//SYNC_EVENT_VDO_MODE;
	params->dsi.switch_mode = CMD_MODE;
#endif
	params->dsi.switch_mode_enable = 0;

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order 	= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     	= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      		= LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	params->dsi.packet_size=256;
	//video mode timing

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active				= 2;
	params->dsi.vertical_backporch					= 6;//18;
	params->dsi.vertical_frontporch					= 8;//10;
	params->dsi.vertical_active_line					= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active				= 5;//10;
	params->dsi.horizontal_backporch				= 80;//100;
	params->dsi.horizontal_frontporch				= 80;//40;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
    	//params->dsi.ssc_disable							= 1;
#ifndef FPGA_EARLY_PORTING
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK = 200; //this value must be in MTK suggested table
#else
	params->dsi.PLL_CLOCK = 390; //this value must be in MTK suggested table
#endif
#else
	params->dsi.pll_div1 = 0;
	params->dsi.pll_div2 = 0;
	params->dsi.fbk_div = 0x1;
	/*
	params->dsi.pll_div1=1;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
	params->dsi.pll_div2=0; 		// div2=0,1,2,3;div1_real=1,2,4,4	
	params->dsi.fbk_div =15;   	 // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)
	*/
#endif

	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 0;// 1;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd          = 0;//0x53;
	params->dsi.lcm_esd_check_table[0].count        = 0;// 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x24;

}

static void lcm_init(void)
{
	unsigned int data_array[16];
	static int count = 0;
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
	int ret=0;
	cmd=0x00;
	data=0x0E;

	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);

	ret=tps65132_write_bytes(cmd,data);
	if(ret<0)
	printk("[KERNEL]nt35596----tps6132---cmd=%0x-- i2c write error-----\n",cmd);
	else
	printk("[KERNEL]nt35596----tps6132---cmd=%0x-- i2c write success-----\n",cmd);

	cmd=0x01;
	data=0x0E;
	ret=tps65132_write_bytes(cmd,data);
	if(ret<0)
	printk("[KERNEL]nt35596----tps6132---cmd=%0x-- i2c write error-----\n",cmd);
	else
	printk("[KERNEL]nt35596----tps6132---cmd=%0x-- i2c write success-----\n",cmd);

	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(20);
	
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
 
}



static void lcm_suspend(void)
{
	unsigned int data_array[16];
//	{0xFF, 1, {0x01}},
//	{0xFB, 1, {0x01}},	
//	{REGFLAG_DELAY, 10, {}},	
//	{0x11, 1, {0x78}},
//    {0x12, 1, {0x78}},
//    {0x13, 1, {0x00}},
//	{REGFLAG_DELAY, 10, {}},	
//	{0x11, 1, {0x78}},
 //   {0x12, 1, {0x78}},
 //   {0x13, 1, {0x00}},	
//	{REGFLAG_DELAY, 200, {}},


	data_array[0]=0x00023902; 
	data_array[1]=0x000001FF;
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0]=0x00023902; 
	data_array[1]=0x000001FB;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(10);
	
	data_array[0]=0x00023902; 
	data_array[1]=0x00007811;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902; 
	data_array[1]=0x00007812;
	dsi_set_cmdq(data_array, 2, 1);	
	data_array[0]=0x00023902; 
	data_array[1]=0x00000013;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(10);
	
	data_array[0]=0x00023902; 
	data_array[1]=0x00007811;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902; 
	data_array[1]=0x00007812;
	dsi_set_cmdq(data_array, 2, 1);	
	data_array[0]=0x00023902; 
	data_array[1]=0x00000013;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(200);

	data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);

	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);
		
	SET_RESET_PIN(0);
	MDELAY(120); // 1ms

}


static void lcm_resume(void)
{
	lcm_init();

	//push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);

}


static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}

static unsigned int lcm_compare_id(void)
{
unsigned int id = 0;
	unsigned char buffer[5];
	unsigned int array[16];
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
	int ret=0;
	cmd=0x00;
	data=0x0E;
	
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);

	ret=tps65132_write_bytes(cmd,data);
	if(ret<0)
	printk("[KERNEL]nt35596----tps6132---cmd=%0x-- i2c write error-----\n",cmd);
	else
	printk("[KERNEL]nt35596----tps6132---cmd=%0x-- i2c write success-----\n",cmd);

	cmd=0x01;
	data=0x0E;
	ret=tps65132_write_bytes(cmd,data);
	if(ret<0)
	printk("[KERNEL]nt35596----tps6132---cmd=%0x-- i2c write error-----\n",cmd);
	else
	printk("[KERNEL]nt35596----tps6132---cmd=%0x-- i2c write success-----\n",cmd); 

	SET_RESET_PIN(1);	//NOTE:should reset LCM firstly
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(50);
	SET_RESET_PIN(1);
	MDELAY(50);

	array[0] = 0x00023700;	// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0xf4, buffer, 2);
	id = buffer[0];
	
#ifdef BUILD_LK
	printf("%s,  nt35596 id = 0x%08x\n", __func__, id);
#else
	printk("%s,  nt35596 id = 0x%08x\n", __func__, id);
#endif
    return (LCM_ID == id)?1:0;
}

LCM_DRIVER nt35596_fhd_dsi_vdo_k505_drv = 
{
	.name				= "nt35596_fhd_dsi_vdo_k505",
	.set_util_funcs			= lcm_set_util_funcs,
	.compare_id			= lcm_compare_id,
	.get_params			= lcm_get_params,
	.init				= lcm_init,
	.suspend			= lcm_suspend,
	.resume         		= lcm_resume,
#if (LCM_DSI_CMD_MODE)
	.update				= lcm_update,
#endif
};
