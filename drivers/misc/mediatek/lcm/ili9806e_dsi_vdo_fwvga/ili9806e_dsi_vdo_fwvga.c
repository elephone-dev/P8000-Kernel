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
#define LCM_ID										0x980604//0x40

#define LCM_DSI_CMD_MODE								0

#define FRAME_WIDTH									(480)
#define FRAME_HEIGHT									(854)
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

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};



#if 0//YT55F83E1
static struct LCM_setting_table lcm_initialization_setting[] = {
	//CMD_Page 7
	{0xFF, 3,{0x98,0x81,0x07}},
	{0x03,1,{0x20}},
	{0x04,1,{0x06}},
	{0x05,1,{0x00}},
	{0x06,1,{0x01}},
	{0x07,1,{0x00}},
	{0x08,1,{0x00}},
	{0x09,1,{0x00}},
	{0x0A,1,{0x01}},
	{0x0B,1,{0x64}},
	{0x0C,1,{0x00}},
	{0x0D,1,{0x00}},
	{0x0E,1,{0x69}},
	{0x0F,1,{0x00}},
	{0x10,1,{0x40}},
	{0x11,1,{0x02}},
	{0x12,1,{0x05}},
	{0x13,1,{0x00}},
	{0x14,1,{0x00}},
	{0x15,1,{0x00}},
	{0x16,1,{0x64}},
	{0x17,1,{0x64}},
	{0x18,1,{0x00}},
	{0x19,1,{0x00}},
	{0x1A,1,{0x00}},
	{0x1B,1,{0x50}},
	{0x1C,1,{0xBC}},
	{0x1D,1,{0x0C}},
	{0x1E,1,{0x00}},
	{0x1F,1,{0x00}},
	{0x20,1,{0x00}},
	{0x21,1,{0x00}},
	{0x22,1,{0x00}},
	{0x23,1,{0x80}},
	{0x24,1,{0x30}},
	{0x25,1,{0x00}},
	{0x26,1,{0x00}},
	{0x27,1,{0x03}},
	
	{0x30,1,{0x01}},
	{0x31,1,{0x23}},
	{0x32,1,{0x45}},
	{0x33,1,{0x67}},
	{0x34,1,{0x89}},
	{0x35,1,{0xab}},
	{0x36,1,{0x01}},
	{0x37,1,{0x23}},
	{0x38,1,{0x45}},
	{0x39,1,{0x67}},
	{0x3A,1,{0x89}},
	{0x3B,1,{0xab}},
	{0x3C,1,{0xcd}},
	{0x3D,1,{0xef}},
	
	{0x50,1,{0x11}},
	{0x51,1,{0x06}},
	{0x52,1,{0x0c}},
	{0x53,1,{0x0d}},
	{0x54,1,{0x0e}},
	{0x55,1,{0x0f}},
	{0x56,1,{0x02}},
	{0x57,1,{0x02}},
	{0x58,1,{0x02}},
	{0x59,1,{0x02}},
	{0x5A,1,{0x02}},
	{0x5B,1,{0x02}},
	{0x5C,1,{0x02}},
	{0x5D,1,{0x02}},
	{0x5E,1,{0x02}},
	{0x5F,1,{0x02}},
	{0x60,1,{0x05}},
	{0x61,1,{0x05}},
	{0x62,1,{0x05}},
	{0x63,1,{0x02}},
	{0x64,1,{0x01}},
	{0x65,1,{0x00}},
	{0x66,1,{0x08}},
	{0x67,1,{0x08}},
	{0x68,1,{0x0c}},
	{0x69,1,{0x0d}},
	{0x6A,1,{0x0e}},
	{0x6B,1,{0x0f}},
	{0x6C,1,{0x02}},
	{0x6D,1,{0x02}},
	{0x6E,1,{0x02}},
	{0x6F,1,{0x02}},
	{0x70,1,{0x02}},
	{0x71,1,{0x02}},
	{0x72,1,{0x02}},
	{0x73,1,{0x02}},
	{0x74,1,{0x02}},
	{0x75,1,{0x02}},
	{0x76,1,{0x05}},
	{0x77,1,{0x05}},
	{0x78,1,{0x05}},
	{0x79,1,{0x02}},
	{0x7A,1,{0x01}},
	{0x7B,1,{0x00}},
	{0x7C,1,{0x06}},
	//CMD_Page 8
	{0xFF, 3,{0x98,0x81,0x08}},
	{0x76,1,{0xB4}},		//VGH pumping ratio 3x
	{0x78,1,{0x02}},		//VGL pumping ratio 2x	     
	{0x74,1,{0x2B}},		//VGH clamp 15V
	{0x8E,1,{0x15}},		//VGL clamp -10V
	{0x40,1,{0x01}},		//POWER SAVING
#if 0
	{0x7B,1,{0x04}},
	{0x72,1,{0x25}},
	{0x87,1,{0x3A}},
	{0x7E,1,{0x4C}},
	{0x6C,1,{0x05}},
	{0x49,1,{0x10}},
	{0x2D,1,{0x80}},
	{0x50,1,{0x65}},
	{0x53,1,{0x29}},
	{0x54,1,{0x65}},
	{0x55,1,{0x38}},
	{0x57,1,{0x47}},
	{0x58,1,{0x47}},
#else
	{0x84,1,{0x81}},		//RF improve
	{0x72,1,{0x25}},
	{0xE3,1,{0x45}},		//Performance enhancement
	{0x7D,1,{0xCB}},
	{0x7E,1,{0x49}},
	{0x49,1,{0x10}},
#endif

	//CMD_Page 1
	{0xFF, 3,{0x98,0x81,0x01}},
	{0x22,1,{0x0A}},		//BGR, SS
	{0x53,1,{0x82}},		//VCOM1
	{0x55,1,{0x85}},		//VCOM2
	{0x50,1,{0xB9}},         	//VREG1OUT=4.7V
	{0x51,1,{0xBA}},        	//VREG2OUT=-4.7V
	{0x31,1,{0x00}},		//column inversion
	{0xA0,1,{0x00}},		//VP255	Gamma P
	{0xA1,1,{0x14}},               //VP251        
	{0xA2,1,{0x2B}},               //VP247        
	{0xA3,1,{0x16}},               //VP243        
	{0xA4,1,{0x1C}},               //VP239        
	{0xA5,1,{0x2E}},               //VP231        
	{0xA6,1,{0x18}},               //VP219        
	{0xA7,1,{0x20}},               //VP203        
	{0xA8,1,{0x78}},               //VP175        
	{0xA9,1,{0x20}},               //VP144        
	{0xAA,1,{0x2A}},               //VP111        
	{0xAB,1,{0x83}},               //VP80         
	{0xAC,1,{0x25}},               //VP52   //06      
	{0xAD,1,{0x1B}},               //VP36         
	{0xAE,1,{0x54}},               //VP24         
	{0xAF,1,{0x2B}},               //VP16         
	{0xB0,1,{0x2B}},               //VP12         
	{0xB1,1,{0x4F}},               //VP8          
	{0xB2,1,{0x67}},               //VP4          
	{0xB3,1,{0x39}},               //VP0                      
	{0xC0,1,{0x00}},		//VN255 GAMMA N
	{0xC1,1,{0x14}},               //VN251        
	{0xC2,1,{0x2B}},               //VN247        
	{0xC3,1,{0x16}},               //VN243        
	{0xC4,1,{0x1C}},               //VN239        
	{0xC5,1,{0x2E}},               //VN231        
	{0xC6,1,{0x18}},               //VN219        
	{0xC7,1,{0x20}},               //VN203        
	{0xC8,1,{0x7B}},               //VN175        
	{0xC9,1,{0x20}},               //VN144        
	{0xCA,1,{0x2A}},               //VN111        
	{0xCB,1,{0x83}},               //VN80         
	{0xCC,1,{0x25}},               //VN52 //06        
	{0xCD,1,{0x1B}},               //VN36         
	{0xCE,1,{0x54}},              //VN24         
	{0xCF,1,{0x2B}},               //VN16         
	{0xD0,1,{0x2B}},               //VN12         
	{0xD1,1,{0x4F}},               //VN8          
	{0xD2,1,{0x67}},               //VN4          
	{0xD3,1,{0x39}},               //VN0  

	//CMD_Page 0
	{0xFF, 3,{0x98,0x81,0x00}},
	//{0x35,1,{0x00}},			//TE on
	{0x11,1,{0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0x29,1,{0x00}},	
	{REGFLAG_DELAY, 20, {}},
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
#else//YT50F308E1
static struct LCM_setting_table lcm_initialization_setting[] = {
#if 0
 {0x00, 1,{0x00}},
 {0xff, 3,{0x80,0x09,0x01}},
 {0x00, 1,{0x80}},
 {0xff, 2,{0x80,0x09}},
 {0x00, 1,{0x03}},
 {0xff, 1,{0x01}},
 {0x00, 1,{0xb4}},
 {0xc0, 1,{0x10}},		// 0x50
 {0x00, 1,{0x92}},
 {0xc5, 1,{0x01}},
 {0x00, 1,{0x90}},
 {0xc5, 1,{0x96}},
 {0x00, 1,{0x95}},
 {0xc5, 1,{0x34}},
 {0x00, 1,{0x94}},
 {0xc5, 1,{0x33}},
 {0x00, 1,{0x82}},
 {0xc5, 1,{0x83}},
 {0x00, 1,{0xb1}},
 {0xc5, 1,{0xa9}},
 {0x00, 1,{0x91}},
 {0xc5, 1,{0x3c}},
 {0x00, 1,{0x00}},
 {0xd8, 2,{0x6f,0x6f}},
 {0x00, 1,{0x81}},
 {0xc1, 1,{0x66}},
 {0x00, 1,{0x00}},
 {0xd9, 1,{0x5c}},		//0x5b
 {0x00, 1,{0xa0}},
 {0xc1, 1,{0x02}},
 {0x00, 1,{0xb2}},
 {0xf5, 4,{0x15,0x00,0x15,0x00}},
 {0x00, 1,{0x93}},
 {0xc5, 1,{0x03}},
 {0x00, 1,{0x96}},
 {0xc5, 1,{0x23}},
 {0x00, 1,{0x81}},
 {0xc4, 1,{0x81}},
 {0x00, 1,{0x89}},
 {0xc4, 1,{0x08}},
 {0x00, 1,{0xa3}},
 {0xc0, 1,{0x00}},
 {0x00, 1,{0xa6}},
 {0xb3, 2,{0x20,0x01}},
 {0x00, 1,{0x90}},
 {0xc0, 6,{0x00,0x44,0x00,0x00,0x00,0x03}},
 {0x00, 1,{0xa6}},
 {0xc1, 3,{0x00,0x00,0x00}},
 {0x00, 1,{0x80}},
 {0xce,12,{0x86,0x01,0x00,0x85,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
 {0x00, 1,{0xa0}},
 {0xce,14,{0x18,0x05,0x03,0x39,0x00,0x00,0x00,0x18,0x04,0x03,0x3a,0x00,0x00,0x00}},
 {0x00, 1,{0xb0}},
 {0xce,14,{0x18,0x03,0x03,0x3b,0x00,0x00,0x00,0x18,0x02,0x03,0x3c,0x00,0x00,0x00}},
 {0x00, 1,{0xc0}},
 {0xcf,10,{0x01,0x01,0x20,0x20,0x00,0x00,0x01,0x00,0x00,0x00}},
 {0x00, 1,{0xc0}},
 {0xcb,15,{0x00,0x04,0x04,0x04,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
 {0x00, 1,{0xd0}},
 {0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x04,0x04,0x04,0x04,0x00,0x00,0x00,0x00}},
 {0x00, 1,{0xe0}},
 {0xcb,10,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
 {0x00, 1,{0x80}},
 {0xcc,10,{0x00,0x26,0x09,0x0b,0x01,0x25,0x00,0x00,0x00,0x00}},
 {0x00, 1,{0x90}},
 {0xcc,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x26,0x0a,0x0c,0x02}},
 {0x00, 1,{0xa0}},
 {0xcc,15,{0x25,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
 {0x00, 1,{0xb0}},
 {0xcc,10,{0x00,0x25,0x0c,0x0a,0x02,0x26,0x00,0x00,0x00,0x00}},
 {0x00, 1,{0xc0}},
 {0xcc,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x25,0x0b,0x09,0x01}},
 {0x00, 1,{0xd0}},
 {0xcc,15,{0x26,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
 {0x00, 1,{0x00}},
 {0xe1,16,{0x00,0x08,0x0d,0x0e,0x07,0x0d,0x0b,0x0a,0x04,0x07,0x10,0x08,0x0f,0x10,0x0a,0x01}},
 {0x00, 1,{0x00}},
 {0xe2,16,{0x00,0x08,0x0e,0x0e,0x07,0x0c,0x0b,0x0a,0x04,0x07,0x10,0x09,0x0f,0x10,0x0a,0x01}},
 {0x00, 1,{0x80}},
 {0xd6, 1,{0x18}},
 {0x00, 1,{0x00}},
 {0xff, 3,{0xff,0xff,0xff}},
 {0x00, 1,{0x00}},
 {0x3a, 1,{0x77}}, 
#else
 	/**************************************************/		
	//LCDD (Peripheral) Setting		
	/**************************************************/	
		
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x01}},     // Change to Page 1

	{0x08,1,{0x10}},                 
	{0x21,1,{0x01}},                 
	{0x30,1,{0x01}},                 // 480 X 854
	         
	{0x31,1,{0x00}},                 // column inversion
             
	{0x40,1,{0x11}},                 
	{0x41,1,{0x55}},               // DDVDH/DDVDL clamp
	{0x42,1,{0x03}},               // VGH/VGL 
	{0x43,1,{0x09}},                
	{0x44,1,{0x06}},               
	{0x45,1,{0x07}}, 
	         
	{0x50,1,{0x80}},                // VGMP
	{0x51,1,{0x80}},                 // VGMN
	{0x52,1,{0x00}},  
             
	{0x53,1,{0x25}},                 //Flicker
             
	{0x56,1,{0x00}},
	{0x57,1,{0x50}},  
             
	{0x60,1,{0x07}},                 
	{0x61,1,{0x00}},                
	{0x62,1,{0x07}},                 
	{0x63,1,{0x00}},                

	//Gamma Setting//

	{0xA0,1,{0x00}},  
	{0xA1,1,{0x0B}},  
	{0xA2,1,{0x19}}, 
	{0xA3,1,{0x11}},
	{0xA4,1,{0x09}},  
	{0xA5,1,{0x18}},
	{0xA6,1,{0x06}},
	{0xA7,1,{0x05}},  
	{0xA8,1,{0x09}},
	{0xA9,1,{0x09}}, 
	{0xAA,1,{0x06}},  
	{0xAB,1,{0x05}},  
	{0xAC,1,{0x0C}},  
	{0xAD,1,{0x29}},  
	{0xAE,1,{0x26}},  
	{0xAF,1,{0x00}},  
             
	{0xC0,1,{0x00}},   
	{0xC1,1,{0x0B}},  
	{0xC2,1,{0x17}}, 
	{0xC3,1,{0x12}}, 
	{0xC4,1,{0x0C}}, 
	{0xC5,1,{0x18}},
	{0xC6,1,{0x0B}}, 
	{0xC7,1,{0x0A}},  
	{0xC8,1,{0x00}},  
	{0xC9,1,{0x09}},
	{0xCA,1,{0x05}},
	{0xCB,1,{0x04}},  
	{0xCC,1,{0x0B}}, 
	{0xCD,1,{0x2B}}, 
	{0xCE,1,{0x26}},  
	{0xCF,1,{0x00}}, 
	
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x06}},     // Change to Page 6
	
	{0x00,1,{0x21}},
	{0x01,1,{0x06}},
	{0x02,1,{0x00}},    
	{0x03,1,{0x00}},
	{0x04,1,{0x16}},
	{0x05,1,{0x16}},
	{0x06,1,{0x80}},    
	{0x07,1,{0x02}},
	{0x08,1,{0x07}},
	{0x09,1,{0x00}},    
	{0x0A,1,{0x00}},    
	{0x0B,1,{0x00}},    
	{0x0C,1,{0x16}},
	{0x0D,1,{0x16}},
	{0x0E,1,{0x00}},
	{0x0F,1,{0x00}},
	{0x10,1,{0x77}},
	{0x11,1,{0xF0}},
	{0x12,1,{0x00}},
	{0x13,1,{0x00}},
	{0x14,1,{0x00}},
	{0x15,1,{0xC0}},
	{0x16,1,{0x08}},
	{0x17,1,{0x00}},
	{0x18,1,{0x00}},
	{0x19,1,{0x00}},
	{0x1A,1,{0x00}},
	{0x1B,1,{0x00}},
	{0x1C,1,{0x00}},
	{0x1D,1,{0x00}},
             
	{0x20,1,{0x01}},
	{0x21,1,{0x23}},
	{0x22,1,{0x45}},
	{0x23,1,{0x67}},
	{0x24,1,{0x01}},
	{0x25,1,{0x23}},
	{0x26,1,{0x45}},
	{0x27,1,{0x67}},

	{0x30,1,{0x11}},
	{0x31,1,{0x22}},
	{0x32,1,{0x11}},
	{0x33,1,{0x00}},
	{0x34,1,{0x66}},
	{0x35,1,{0x88}},
	{0x36,1,{0x22}},
	{0x37,1,{0x22}},
	{0x38,1,{0xAA}},
	{0x39,1,{0xCC}},
	{0x3A,1,{0xBB}},
	{0x3B,1,{0xDD}},
	{0x3C,1,{0x22}},
	{0x3D,1,{0x22}},
	{0x3E,1,{0x22}},
	{0x3F,1,{0x22}},
	{0x40,1,{0x22}},
	{0x52,1,{0x10}},
	{0x53,1,{0x12}},

	{0xFF,5,{0xFF,0x98,0x06,0x04,0x07}},      // Change to Page 7
	
	{0x17,1,{0x32}},                 
	{0x02,1,{0x77}},
	{0xE1,1,{0x79}},                 

	{0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},  
	
	{0x35,1,{0x00}},//TE ON
	
	{0x51,1,{0xFF}},
	{0x53,1,{0x2C}},
	{0x55,1,{0x00}},

	{0x11,1,{0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0x29,1,{0x00}},
	{REGFLAG_DELAY, 10, {}},
#endif    
    // Sleep Out
    {0x11, 0, {0x00}},
    {REGFLAG_DELAY, 150, {}},

    // Display ON
    {0x29, 0, {0x00}},	
    {REGFLAG_DELAY, 50, {}},
    
    {REGFLAG_END_OF_TABLE, 0x00, {}}
    
	
	// Setting ending by predefined flag
//	{REGFLAG_END_OF_TABLE, 0x00, {}}
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
	params->dsi.LANE_NUM				= LCM_TWO_LANE;
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
	params->dsi.vertical_backporch					= 18;
	params->dsi.vertical_frontporch					= 10;
	params->dsi.vertical_active_line					= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active				= 10;
	params->dsi.horizontal_backporch				= 100;
	params->dsi.horizontal_frontporch				= 40;
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

	data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);

	
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
   	int array[4];
        char buffer[5];
        char id_high=0;
        char id_midd=0;
        char id_low=0;
        int id=0;
 
        //Do reset here
        SET_RESET_PIN(1);
        SET_RESET_PIN(0);
        MDELAY(25);       
        SET_RESET_PIN(1);
        MDELAY(50);      
       
        array[0]=0x00063902;
        array[1]=0x0698ffff;
        array[2]=0x00000104;
        dsi_set_cmdq(array, 3, 1);
        MDELAY(10);
 
        array[0]=0x00033700;
        dsi_set_cmdq(array, 1, 1);
        //read_reg_v2(0x04, buffer, 3);//if read 0x04,should get 0x008000,that is both OK.
    
        read_reg_v2(0x00, buffer,1);
        id_high = buffer[0]; ///////////////////////0x98
 
        read_reg_v2(0x01, buffer,1);
        id_midd = buffer[0]; ///////////////////////0x06
 
        read_reg_v2(0x02, buffer,1);
        id_low = buffer[0]; ////////////////////////0x04
 
        id =(id_high << 16) | (id_midd << 8) | id_low;

#if defined(BUILD_LK)
			  printf("lk -- ili9806e 0x%x , 0x%x , 0x%x, 0x%x \n", id_high, id_midd, id_low, id);
#else
			  printk("kernel -- ili9806e 0x%x , 0x%x , 0x%x, 0x%x \n", id_high, id_midd, id_low, id);
#endif

		 return (id == LCM_ID)?1:0;
}

LCM_DRIVER ili9806e_dsi_vdo_fwvga_drv = 
{
	.name				= "ili9806e_dsi_vdo_fwvga",
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

