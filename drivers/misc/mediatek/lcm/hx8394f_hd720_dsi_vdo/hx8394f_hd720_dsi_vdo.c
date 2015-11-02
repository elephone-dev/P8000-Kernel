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
#define LCM_ID										0x8394

#define LCM_DSI_CMD_MODE								0

#define FRAME_WIDTH									(720)
#define FRAME_HEIGHT									(1280)
#define REGFLAG_DELAY             							0XAA
#define REGFLAG_END_OF_TABLE      						0xAB   // END OF REGISTERS MARKER

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
	{0xb9, 3, {0xff,0x83,0x94}},
	
	{0xb1, 10,{0x50,0x15,0x75,0x09,0x32,0x44,0x71,0x31,0x55,0x2f}},
	
	{0xba, 6, {0x63,0x03,0x68,0x6b,0xb2,0xc0}},
	
	{0xd2, 1, {0x88}},
	
	{0xb2, 5, {0x00,0x80,0x64,0x10,0x07}},
	
	{0xb4, 21,{0x01,0x65,0x01,0x65,0x01,0x65,0x01,0x05,0x7e,0x35,
						 0x00,0x3f,0x01,0x65,0x01,0x65,0x01,0x65,0x01,0x05,0x7e}},
						 
	{0xd3, 33,{0x00,0x00,0x06,0x06,0x40,0x1a,0x08,0x00,0x32,0x10,
						 0x07,0x00,0x07,0x54,0x15,0x0f,0x05,0x04,0x02,0x12,
						 0x10,0x05,0x07,0x33,0x33,0x0b,0x0b,0x37,0x10,0x07,
						 0x07,0x10,0x40}},
						 
	{0xd5, 44,{0x19,0x19,0x18,0x18,0x1b,0x1b,0x1a,0x1a,0x04,0x05,
						 0x06,0x07,0x00,0x01,0x02,0x03,0x20,0x21,0x18,0x18,
						 0x22,0x23,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
						 0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
						 0x18,0x18,0x18,0x18}},
						 				 
	{0xd6, 44,{0x18,0x18,0x19,0x19,0x1b,0x1b,0x1a,0x1a,0x03,0x02,
						 0x01,0x00,0x07,0x06,0x05,0x04,0x23,0x22,0x18,0x18,
						 0x21,0x20,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
						 0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
						 0x18,0x18,0x18,0x18}},
						 
	{0xe0, 58,{0x00,0x01,0x07,0x0b,0x0d,0x11,0x13,0x11,0x25,0x35,
						 0x48,0x4a,0x57,0x6d,0x76,0x7c,0x8c,0x91,0x8e,0x9f,
						 0xb0,0x57,0x57,0x5a,0x60,0x64,0x6a,0x72,0x7f,0x00,
						 0x01,0x07,0x0b,0x0d,0x10,0x13,0x11,0x25,0x35,0x48,
						 0x4a,0x57,0x6d,0x76,0x7c,0x8c,0x91,0x8e,0x9f,0xb0,
						 0x57,0x57,0x5a,0x60,0x64,0x6a,0x72,0x7f}},
						 
	{0xcc, 1, {0x0b}},
	
	{0xc0, 2, {0x1f,0x73}},
	
	{0xb6, 2, {0x70,0x70}},
	
	{0xd4, 1, {0x02}},

	{0x11,  1 ,{0x00}},

	{REGFLAG_DELAY,200,{}},

	{0x29,  1 ,{0x00}},
	{REGFLAG_DELAY, 20, {}},
	
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


static struct LCM_setting_table lcm_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 200, {}},

	// Sleep Mode On
	{0x10, 1, {0x00}},
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

	params->dsi.vertical_sync_active				= 2;//5;//2;
	params->dsi.vertical_backporch					= 2;//14;//18;
	params->dsi.vertical_frontporch					= 2;//8;//10;
	params->dsi.vertical_active_line					= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active				= 6;//18;//10;
	params->dsi.horizontal_backporch				= 24;//95;//100;
	params->dsi.horizontal_frontporch				= 12;//95;//40;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
    	//params->dsi.ssc_disable							= 1;
#ifndef FPGA_EARLY_PORTING
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK = 200; //this value must be in MTK suggested table
#else
	params->dsi.PLL_CLOCK = 200; //this value must be in MTK suggested table
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

static void init_lcm_registers(void)
{
				unsigned int data_array[16];
				
			  data_array[0] = 0x00043902;
			  data_array[1] = 0x9483FFB9;
			  dsi_set_cmdq(&data_array, 2, 1);
			  MDELAY(1);
			  
			  data_array[0] = 0x000B3902;
			  data_array[1] = 0x751550B1;
			  data_array[2] = 0x71443209;
			  data_array[3] = 0x002f5531;
			  dsi_set_cmdq(&data_array, 4, 1);
			  MDELAY(1);
			  
			  data_array[0] = 0x00073902;
			  data_array[1] = 0x680363BA;
			  data_array[2] = 0x00c0b26b;
			  dsi_set_cmdq(&data_array, 3, 1);
			  MDELAY(1);
			  
			  
			  data_array[0] = 0x00023902;
			  data_array[1] = 0x000088D2;
			  dsi_set_cmdq(&data_array, 2, 1);
			  MDELAY(1);
			  
			  data_array[0] = 0x00063902;
			  data_array[1] = 0x648000B2;
			  data_array[2] = 0x00000710;
			  dsi_set_cmdq(&data_array, 3, 1);
			  MDELAY(1);
			  
			  data_array[0] = 0x00163902;
			  data_array[1] = 0x016501B4;
			  data_array[2] = 0x01650165;
			  data_array[3] = 0x00357e05;
			  data_array[4] = 0x0165013f;
			  data_array[5] = 0x01650165;
			  data_array[6] = 0x00007e05;
			  dsi_set_cmdq(&data_array, 7, 1);
			  MDELAY(1);
			  
			  data_array[0] = 0x00223902;
			  data_array[1] = 0x060000D3;
			  data_array[2] = 0x081a4006;
			  data_array[3] = 0x07103200;
			  data_array[4] = 0x15540700;
			  data_array[5] = 0x0204050f;
			  data_array[6] = 0x07051012;
			  data_array[7] = 0x0b0b3333;
			  data_array[8] = 0x07071037;
			  data_array[9] = 0x00004010;
			  dsi_set_cmdq(&data_array, 10, 1);
			  MDELAY(1);
			  
			    
			  data_array[0] = 0x002D3902;
			  data_array[1] = 0x181919D5;
			  data_array[2] = 0x1a1b1b18;
			  data_array[3] = 0x0605041a;
			  data_array[4] = 0x02010007;
			  data_array[5] = 0x18212003;
			  data_array[6] = 0x18232218;
			  data_array[7] = 0x18181818;
			  data_array[8] = 0x18181818;
			  data_array[9] = 0x18181818;
			  data_array[10] = 0x18181818;
			  data_array[11] = 0x18181818;
			  data_array[12] = 0x00000018;
			  dsi_set_cmdq(&data_array, 13, 1);
			  MDELAY(1);
			  
			 
			  
			  data_array[0] = 0x002D3902;
			  data_array[1] = 0x191818D6;
			  data_array[2] = 0x1a1b1b19;
			  data_array[3] = 0x0102031a;
			  data_array[4] = 0x05060700;
			  data_array[5] = 0x18222304;
			  data_array[6] = 0x18202118;
			  data_array[7] = 0x18181818;
			  data_array[8] = 0x18181818;
			  data_array[9] = 0x18181818;
			  data_array[10] = 0x18181818;
			  data_array[11] = 0x18181818;
			  data_array[12] = 0x00000018;
			  dsi_set_cmdq(&data_array, 13, 1);
			  MDELAY(1);
			  
			  data_array[0] = 0x003B3902;
			  data_array[1] = 0x070100E0;
			  data_array[2] = 0x13110d0b;
			  data_array[3] = 0x48352511;
			  data_array[4] = 0x766d574a;
			  data_array[5] = 0x8e918c7c;
			  data_array[6] = 0x5757b09f;
			  data_array[7] = 0x6a64605a;
			  data_array[8] = 0x01007f72;
			  data_array[9] = 0x100d0b07;
			  data_array[10] = 0x35251113;
			  data_array[11] = 0x6d574a48;
			  data_array[12] = 0x918c7c76;
			  data_array[13] = 0x57b09f8e;
			  data_array[14] = 0x64605a57;
			  data_array[15] = 0x007f726a;
			  dsi_set_cmdq(&data_array, 16, 1);
			  MDELAY(1);
			  
			  data_array[0] = 0x00023902;
			  data_array[1] = 0x00000BCC;
			  dsi_set_cmdq(&data_array, 2, 1);
			  MDELAY(1);
			  
			  data_array[0] = 0x00033902;
			  data_array[1] = 0x00731fC0;
			  dsi_set_cmdq(&data_array, 2, 1);
			  MDELAY(1);
			  
			  
			
			  
			  data_array[0] = 0x00033902;
			  data_array[1] = 0x007070B6;
			  dsi_set_cmdq(&data_array, 2, 1);
			  MDELAY(1);
			  
			  data_array[0] = 0x00023902;
			  data_array[1] = 0x000002D4;
			  dsi_set_cmdq(&data_array, 2, 1);
			  MDELAY(1);
			  
			  data_array[0] = 0x00023902;
			  data_array[1] = 0x000001BD;
			  dsi_set_cmdq(&data_array, 2, 1);
			  MDELAY(1);
			  
			  data_array[0] = 0x00023902;
			  data_array[1] = 0x000060B1;
			  dsi_set_cmdq(&data_array, 2, 1);
			  MDELAY(1);
			  
			  data_array[0] = 0x00023902;
			  data_array[1] = 0x000000BD;
			  dsi_set_cmdq(&data_array, 2, 1);
			  MDELAY(1);
			  
			  data_array[0] = 0x00083902;
			  data_array[1] = 0x508140BF;
			  data_array[2] = 0x01FC1A00;
			  dsi_set_cmdq(&data_array, 3, 1);
			  MDELAY(1);
			  
			  
			  data_array[0] = 0x00110500;
			  dsi_set_cmdq(&data_array, 1, 1);
			  MDELAY(200);
			  
			  data_array[0] = 0x00290500;
			  dsi_set_cmdq(&data_array, 1, 1);
			  MDELAY(10);
				
			  data_array[0] = 0x00033902;
			  data_array[1] = 0x000D01e4;
			  dsi_set_cmdq(&data_array, 2, 1);
			  MDELAY(1);	
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
	printk("[KERNEL]hx8394f----tps6132---cmd=%0x-- i2c write error-----\n",cmd);
	else
	printk("[KERNEL]hx8394f----tps6132---cmd=%0x-- i2c write success-----\n",cmd);

	cmd=0x01;
	data=0x0E;
	ret=tps65132_write_bytes(cmd,data);
	if(ret<0)
	printk("[KERNEL]hx8394f----tps6132---cmd=%0x-- i2c write error-----\n",cmd);
	else
	printk("[KERNEL]hx8394f----tps6132---cmd=%0x-- i2c write success-----\n",cmd);

	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(20);
	
	//push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
 init_lcm_registers();
}



static void lcm_suspend(void)
{
	unsigned int data_array[16];

	data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);

	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);
	
	SET_RESET_PIN(1);	
	SET_RESET_PIN(0);
	MDELAY(100); // 1ms
	
	SET_RESET_PIN(1);
	MDELAY(120);      

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

	SET_RESET_PIN(1);	//NOTE:should reset LCM firstly
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(50);
	SET_RESET_PIN(1);
	MDELAY(50);

	push_table(page1_select, sizeof(page1_select) / sizeof(struct LCM_setting_table), 1);
	array[0] = 0x00013700;	// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	
	read_reg_v2(0xDA, buffer, 1);//0X83
	
  read_reg_v2(0xDB, buffer+1, 1);//0X94
	
	id = buffer[0]<<8 |buffer[0];

#ifdef BUILD_LK
	printf("%s,  hx8394f id = 0x%08x\n", __func__, id);
#else
	printk("%s,  hx8394f id = 0x%08x\n", __func__, id);
#endif
    return (LCM_ID == id)?1:0;
}

LCM_DRIVER hx8394f_hd720_dsi_vdo_lcm_drv = 
{
	.name				= "hx8394f_hd720_dsi_vdo",
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
