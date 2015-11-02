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
#define R61318_LCM_ID										(0x1318)

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


#if 0
static struct LCM_setting_table lcm_initialization_setting[] = {
	//CMD_Page 7
	{0xB0,1,{0x00}},
	{0xFA,1,{0x00}},
	{0xB9,2,{0x00,0x00}},
	{0xB7,2,{0x60,0x02}},
	{0xDE,2,{0x03,0x00}},
	{0xD6,2,{0x05,0x00}},
	{0xB8,2,{0x00,0x00}},
	{0xB1,2,{0x08,0x02}},
	{0xB2,2,{0x40,0x14}},
	{0xB3,2,{0x10,0x16}},//
	{0xB4,2,{0xD0,0x02}},
	{0xB5,2,{0x00,0x05}},
	{0xB6,2,{0x07,0x00}},
	{0xBA,2,{0x06,0xC0}},
	{0xBB,2,{0x07,0x00}},
	{0xB9,2,{0x01,0x00}},
	{REGFLAG_DELAY, 100, {}},
	{0xBD,2,{0x00,0x00}},
	{0xB7,2,{0x20,0x02}},
	{0xBC,2,{0x02,0x00}},
	{0xBF,2,{0xB0,0x00}},
	{0xBC,2,{0x02,0x00}},
	{0xBF,2,{0xE3,0x01}},
	{0xBC,2,{0x06,0x00}},
	{0xBF,6,{0xB7,0x78,0x64,0x10,0x64,0xAB}},
	{0xBC,2,{0x18,0x00}},
	{0xBF,18,{0xB8,0xB4,0x40,0x43,0x49,
						0x55,0x62,0x71,0x82,0x94,
						0xA8,0xB9,0xCB,0xDB,0xE9,
						0xF5,0xFC,0xFF}},
	{0xBC,2,{0x03,0x00}},
	{0xBF,3,{0xB9,0xB4,0xA0}},
	{0xBC,2,{0x03,0x00}},
	{0xBF,3,{0xBA,0x01,0xBC}},
	{0xBC,2,{0x02,0x00}},
	{0xBF,2,{0xBB,0x22}},
	{0xBC,2,{0x07,0x00}},
	{0xBF,7,{0xBC,0x04,0x00,0x04,0x04,
					 0x00,0x00}},
	{0xBC,2,{0x11,0x00}},
	{0xBF,11,{0xBD,0x80,0x80,0x80,0x80,
		      	0x80,0x80,0x80,0x08,0x20,0x80}},
	{0xBC,2,{0x22,0x00}},
	{0xBF,22,{0xBE,0xFF,0x0A,0x4A,0x37,
						0xA0,0x55,0xF8,0x0C,0x0C,
						0x20,0x10,0x3F,0x3F,0x00,
						0x00,0x10,0x10,0x3F,0x3F,
						0x3F,0x3F}},
	{0xBC,2,{0x07,0x00}},
	{0xBF,7,{0xC0,0x23,0xB2,0x0F,0x0F,0x02,0x7F}},
	{0xBC,2,{0x08,0x00}},
	{0xBF,8,{0xC1,0xA8,0xA8,0x33,0x33,0x10,0x10,0x33}},
	{0xBC,2,{0x02,0x00}},
	{0xBF,2,{0xC3,0x20}},
	{0xBC,2,{0x04,0x00}},
	{0xBF,4,{0xC4,0xB4,0xD9,0x82}},
	{0xBC,2,{0x04,0x00}},
	{0xBF,4,{0xC5,0x04,0x05,0x13}},
	{0xBC,2,{0x02,0x00}},
	{0xBF,2,{0xC6,0x21}},
	{0xBC,2,{0x1D,0x00}},
	{0xBF,29,{0xC8,0x00,0x82,0x18,0xA4,
						0x98,0x83,0x92,0x5A,0xAC,
						0xB9,0x07,0xA3,0x9C,0xB4,
						0x56,0x3A,0x65,0x84,0xCF,
						0x35,0xB6,0x54,0x42,0xC7,
						0x14,0x32,0x44,0x00}},
	{0xBC,2,{0x1F,0x00}},
	{0xBF,31,{0xCA,0x01,0x07,0x0B,0x10,
						0x15,0x1A,0x1D,0x19,0x27,
						0x26,0x24,0x1B,0x16,0x11,
						0x04,0x01,0x07,0x0B,0x0F,
						0x15,0x1B,0x1E,0x1C,0x27,
						0x26,0x21,0x1B,0x16,0x11,
						0x04}},
	{0xBC,2,{0x20,0x00}},
	{0xBF,20,{0xCB,0x00,0x0B,0xF3,0xFA,
						0x06,0xFC,0x00,0x00,0xFE,
						0xFE,0x07,0xA2,0x00,0x24,
						0xE1,0xF6,0xF8,0x54,0x00}},
	{0xBC,2,{0x02,0x00}},
	{0xBF,2,{0xCD,0x00}},
	{0xBC,2,{0x04,0x00}},
	{0xBF,4,{0xD0,0x05,0x10,0x4B}},
	{0xBC,2,{0x02,0x00}},
	{0xBF,2,{0xD1,0x03}},
	{0xBC,2,{0x03,0x00}},
	{0xBF,3,{0xD2,0x81,0x1F}},
	{0xBC,2,{0x03,0x00}},
	{0xBF,3,{0xD3,0x33,0x33}},
	{0xBC,2,{0x02,0x00}},
	{0xBF,2,{0xE5,0x03}},
	{0xBC,2,{0x02,0x00}},
	{0xBF,2,{0xD4,0x69}},
	{0xBC,2,{0x03,0x00}},
	{0xBF,3,{0xD5,0x32,0x32}},
	{0xBC,2,{0x02,0x00}},
	{0xBF,2,{0xD6,0x01}},
	{0xC1,2,{0x10,0x00}},
	{REGFLAG_DELAY, 10, {}},
	
	{0xB7,2,{0xA0,0x03}},
	{0xBC,2,{0x01,0x00}},
	{0xBF,1,{0xBF}},
	{REGFLAG_DELAY, 20, {}},
	
	{0xFF,1,{0x00}},
	{0xFA,1,{0x00}},
	{0xFA,1,{0x00}},
	{0xFA,1,{0x00}},
	{0xFA,1,{0x00}},
	{0xFA,1,{0x00}},
	{0xB7,1,{0x60,0x02}},
	{REGFLAG_DELAY, 20, {}},
	
	{0xBC,3,{0x00,0x00,0x11}},
	
	{0x11,1,{0x00}},
	{REGFLAG_DELAY, 120, {}},
	
	{0xBC,2,{0x00,0x00}},
	{0x29,1,{0x00}},	
	{REGFLAG_DELAY, 20, {}},
	
	{0xB7,2,{0x6B,0x02}},
	{REGFLAG_DELAY, 20, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif

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
	params->dsi.vertical_backporch					= 13;//18;
	params->dsi.vertical_frontporch					= 17;//10;
	params->dsi.vertical_active_line					= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active				= 8;
	params->dsi.horizontal_backporch				= 80;//100;
	params->dsi.horizontal_frontporch				= 100;//40;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
    	//params->dsi.ssc_disable							= 1;
    	params->dsi.HS_TRAIL = 15;
#ifndef FPGA_EARLY_PORTING
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK = 208; //this value must be in MTK suggested table
#else
	params->dsi.PLL_CLOCK = 208; //this value must be in MTK suggested table
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
	printk("[KERNEL]r61318a1----tps6132---cmd=%0x-- i2c write error-----\n",cmd);
	else
	printk("[KERNEL]r61318a1----tps6132---cmd=%0x-- i2c write success-----\n",cmd);

	cmd=0x01;
	data=0x0E;
	ret=tps65132_write_bytes(cmd,data);
	if(ret<0)
	printk("[KERNEL]r61318a1----tps6132---cmd=%0x-- i2c write error-----\n",cmd);
	else
	printk("[KERNEL]r61318a1----tps6132---cmd=%0x-- i2c write success-----\n",cmd);

	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(20);
	
	data_array[0] = 0x00022902;
	data_array[1] = 0x000000b0;
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = 0x00022902;
	data_array[1] = 0x000001E3;	
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = 0x00072902;
	data_array[1] = 0x10B223C0; 
	data_array[2] = 0x007F0210;
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0] = 0x00082902;
	data_array[1] = 0x33A8A8C1; 
	data_array[2] = 0x33101033;
	dsi_set_cmdq(data_array, 3, 1); 

	data_array[0] = 0x00022902;
	data_array[1] = 0x000020C3;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00042902; 	 
	data_array[1] = 0x82D9B4C4; 
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = 0x00042902; 	 
	data_array[1] = 0x130502C5; 
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = 0x00022902; 	 
	data_array[1] = 0x000021C6; 
	dsi_set_cmdq(data_array, 2, 1);	
	
	data_array[0] = 0x001D2902;
	data_array[1] = 0x188200c8; 
	data_array[2] = 0x928398A4; 
	data_array[3] = 0x07B9AC5A; 
	data_array[4] = 0x56B49CA3; 
	data_array[5] = 0xCF84653A;
	data_array[6] = 0x4254B635; 
	data_array[7] = 0x443214C7; 
	data_array[8] = 0x00000000; 
	dsi_set_cmdq(data_array, 9, 1); 
	
	data_array[0] = 0x001F2902;
	data_array[1] = 0x110B03CA; 
	data_array[2] = 0x26251E16; 
	data_array[3] = 0x161E1F23; 
	data_array[4] = 0x01020305; 
	data_array[5] = 0x100B0600; 
	data_array[6] = 0x22222119; 
	data_array[7] = 0x05141C1E; 
	data_array[8] = 0x00040303; 
	dsi_set_cmdq(data_array, 9, 1); 

	data_array[0] = 0x00022902; 	 
	data_array[1] = 0x000000CD;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00042902; 	 
	data_array[1] = 0x4B1005D0;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00022902; 	 
	data_array[1] = 0x000003D1;
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = 0x00032902; 	 
	data_array[1] = 0x001F81D2;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00032902; 	 
	data_array[1] = 0x003333D3;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00022902;
	data_array[1] = 0x000003E5; 
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00022902;
	data_array[1] = 0x00006AD4; 
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00032902; 	 
	data_array[1] = 0x002A2AD5;
	dsi_set_cmdq(data_array, 2, 1);
	
	MDELAY(50);
	
	data_array[0]=0x00110500;
	dsi_set_cmdq(&data_array,1, 1);
	
	MDELAY(150);
	
	data_array[0]=0x00290500;
	dsi_set_cmdq(&data_array,1, 1);    

	MDELAY(60);
 
}



static void lcm_suspend(void)
{
	unsigned int data_array[16];

	data_array[0]=0x00280500; 
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(30);

	data_array[0] = 0x00100500; 
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);

	data_array[0] = 0x00022902;
	data_array[1] = 0x000000b0;
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = 0x00000500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00000500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00022902;
	data_array[1] = 0x000001b1;
	dsi_set_cmdq(data_array, 2, 1);	
	MDELAY(30);

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

	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00); //should power up ic in board firstly
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
	
	SET_RESET_PIN(1);	//NOTE:should reset LCM then
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(50);
	SET_RESET_PIN(1);
	MDELAY(50);

	array[0] = 0x00022902;                          
	array[1] = 0x000000b0; 
	dsi_set_cmdq(array, 2, 1);//01 22 13 18 XX


	array[0] = 0x00053700;// read id return two byte,version and id

	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xbf, buffer, 5);

	id = buffer[2]<<8 | buffer[3]; //we only need ID

#ifdef BUILD_LK
	printf("%s,  r61318a1 id = 0x%08x\n", __func__, id);
#else
	printk("%s,  r61318a1 id = 0x%08x\n", __func__, id);
 #endif
   return (R61318_LCM_ID == id)?1:0;
}

LCM_DRIVER r61318a1_hd720_dsi_vdo_k503_lcm_drv = 
{
	.name				= "r61318a1_hd720_dsi_vdo_k503",
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
