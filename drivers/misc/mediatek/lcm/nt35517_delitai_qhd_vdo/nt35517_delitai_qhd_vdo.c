#ifdef BUILD_LK
#define PRINT_LOG  printf
#else
#include <linux/string.h>
extern int printk(const char * fmt, ...);                       /* See kernel/printk.c */
#define PRINT_LOG  printk
#endif
#include "lcm_drv.h"

#define ADC_GUANCANGMAO_DATA 3415
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(540)
#define FRAME_HEIGHT 										(960)

#define REGFLAG_DELAY             							0XFFE
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

#ifndef TRUE
    #define   TRUE     1
#endif
 
#ifndef FALSE
    #define   FALSE    0
#endif

#define LCM_ID 0x5517
static unsigned int lcm_compare_id();

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};



static struct LCM_setting_table lcm_initialization_setting[] = {
	{0xF0,5,{0x55,0xAA,0x52,0x08,0x01}},
	{0xB0,3,{0x09,0x09,0x09}},
	{0xB1,3,{0x09,0x09,0x09}},
	{0xB2,3,{0x00,0x00,0x00}},
	{0xB3,3,{0x0A,0x0A,0x0A}},
	{0xB4,3,{0x08,0x08,0x08}},
	{0xB6,3,{0x43,0x43,0x43}},
	{0xB7,3,{0x33,0x33,0x33}},
	{0xB8,3,{0x33,0x33,0x33}},
	{0xB9,3,{0x23,0x23,0x23}},
	{0xBA,3,{0x23,0x23,0x23}},
	{0xBC,3,{0x00,0xA0,0x00}},
	{0xBD,3,{0x00,0xA0,0x00}},
	{0xBE,1,{0x57}},
	{0xC2,1,{0x00}},
	{0xD0,4,{0x0F,0x0F,0x10,0x10}},    
	{0xD1,16,{0X00,0X00,0X00,0X13,0X00,0X31,0X00,0X4C,0X00,0X63,0X00,0X7A,0X00,0X9F,0X00,0XD8}},
	{0xD2,16,{0X00,0XFE,0X01,0X3D,0X01,0X6C,0X01,0XB8,0X01,0XF4,0X01,0XF6,0X02,0X2F,0X02,0X6B}},
	{0xD3,16,{0X02,0X8E,0X02,0XBF,0X02,0XDE,0X03,0X08,0X03,0X1F,0X03,0X3B,0X03,0X47,0X03,0X53}},
	{0xD4,4 ,{0X03,0X59,0X03,0X5A}},                                              
	{0xD5,16,{0X00,0X00,0X00,0X13,0X00,0X31,0X00,0X4C,0X00,0X63,0X00,0X7A,0X00,0X9F,0X00,0XD8}},
	{0xD6,16,{0X00,0XFE,0X01,0X3D,0X01,0X6C,0X01,0XB8,0X01,0XF4,0X01,0XF6,0X02,0X2F,0X02,0X6B}},
	{0xD7,16,{0X02,0X8E,0X02,0XBF,0X02,0XDE,0X03,0X08,0X03,0X1F,0X03,0X3B,0X03,0X47,0X03,0X53}},
	{0xD8,4 ,{0X03,0X59,0X03,0X5A}},                                                            
	{0xD9,16,{0X00,0X00,0X00,0X13,0X00,0X31,0X00,0X4C,0X00,0X63,0X00,0X7A,0X00,0X9F,0X00,0XD8}},
	{0xDD,16,{0X00,0XFE,0X01,0X3D,0X01,0X6C,0X01,0XB8,0X01,0XF4,0X01,0XF6,0X02,0X2F,0X02,0X6B}},
	{0xDE,16,{0X02,0X8E,0X02,0XBF,0X02,0XDE,0X03,0X08,0X03,0X1F,0X03,0X3B,0X03,0X47,0X03,0X53}},
	{0xDF,4 ,{0X03,0X59,0X03,0X5A}},                                                            
	{0xE0,16,{0X00,0X00,0X00,0X13,0X00,0X31,0X00,0X4C,0X00,0X63,0X00,0X7A,0X00,0X9F,0X00,0XD8}},
	{0xE1,16,{0X00,0XFE,0X01,0X3D,0X01,0X6C,0X01,0XB8,0X01,0XF4,0X01,0XF6,0X02,0X2F,0X02,0X6B}},
	{0xE2,16,{0X02,0X8E,0X02,0XBF,0X02,0XDE,0X03,0X08,0X03,0X1F,0X03,0X3B,0X03,0X47,0X03,0X53}},
	{0xE3,4 ,{0X03,0X59,0X03,0X5A}},                                                            
	{0xE4,16,{0X00,0X00,0X00,0X13,0X00,0X31,0X00,0X4C,0X00,0X63,0X00,0X7A,0X00,0X9F,0X00,0XD8}},
	{0xE5,16,{0X00,0XFE,0X01,0X3D,0X01,0X6C,0X01,0XB8,0X01,0XF4,0X01,0XF6,0X02,0X2F,0X02,0X6B}},
	{0xE6,16,{0X02,0X8E,0X02,0XBF,0X02,0XDE,0X03,0X08,0X03,0X1F,0X03,0X3B,0X03,0X47,0X03,0X53}},
	{0xE7,4 ,{0X03,0X59,0X03,0X5A}},                                                            
	{0xE8,16,{0X00,0X00,0X00,0X13,0X00,0X31,0X00,0X4C,0X00,0X63,0X00,0X7A,0X00,0X9F,0X00,0XD8}},
	{0xE9,16,{0X00,0XFE,0X01,0X3D,0X01,0X6C,0X01,0XB8,0X01,0XF4,0X01,0XF6,0X02,0X2F,0X02,0X6B}},
	{0xEA,16,{0X02,0X8E,0X02,0XBF,0X02,0XDE,0X03,0X08,0X03,0X1F,0X03,0X3B,0X03,0X47,0X03,0X53}},
	{0xEB,4 ,{0X03,0X59,0X03,0X5A}},                                                            
	{0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},
	{0xB1,2,{0xFC,0x00}},
	{0xB3,1,{0x80}},
	{0xB6,1,{0x05}},
	{0xB7,2,{0x72,0x72}},
	{0xB8,4,{0x01,0x04,0x04,0x04}},
	{0xBC,3,{0x02,0x00,0x00}},
	{0xC7,11,{0x00,0xFF,0xFF,0x10,0x10,0x0A,0x0B,0xff,0xff,0x93,0x00}},
	{0x35,1,{0x00}},
	{0x36,1,{0x00}},
	{0x3A,1,{0x77}},

	{0x11, 1, {0x00}},       
	{REGFLAG_DELAY,120,{}},
	{0x29,	1,	{0x00}},
	{REGFLAG_DELAY,20,{}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}

};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
	// Sleep Out
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	// Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 20, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_In_setting[] = {
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 40, {}},
	// Sleep In
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},
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
            break;
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
    params->dsi.switch_mode = SYNC_EVENT_VDO_MODE;
#else
    params->dsi.mode   =   SYNC_EVENT_VDO_MODE; //SYNC_PULSE_VDO_MODE;
    params->dsi.switch_mode = CMD_MODE;
#endif
    params->dsi.switch_mode_enable = 0;

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_TWO_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    // Not support in MT6573
    params->dsi.packet_size=256;

    // Video mode setting		
    params->dsi.intermediat_buffer_num = 0;

    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.vertical_sync_active				= 2;//4;  //---3
    params->dsi.vertical_backporch				  = 60;//12; //---14
    params->dsi.vertical_frontporch 				= 60;//13;  //----8
    params->dsi.vertical_active_line				= FRAME_HEIGHT; 

    params->dsi.horizontal_sync_active			= 2;//6;//6;
    params->dsi.horizontal_backporch				= 60;//64;//28;
    params->dsi.horizontal_frontporch				= 60;//64;//28;
    params->dsi.horizontal_active_pixel 			= FRAME_WIDTH;

    params->dsi.PLL_CLOCK = 270; //this value must be in MTK suggested table

    params->dsi.ssc_disable = 0;
    params->dsi.clk_lp_per_line_enable = 0;
    params->dsi.esd_check_enable = 0;// 1;
    params->dsi.customization_esd_check_enable = 0;
    params->dsi.lcm_esd_check_table[0].cmd          = 0;//0x53;
    params->dsi.lcm_esd_check_table[0].count        = 0;// 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x24;
}


static void lcm_init(void)
{
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(30);
    SET_RESET_PIN(1);
    MDELAY(50);
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(30);
    SET_RESET_PIN(1);
    MDELAY(60);

    push_table(lcm_sleep_In_setting, sizeof(lcm_sleep_In_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_resume(void)
{
    lcm_init();
}

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);


static unsigned int lcm_compare_id()
{
	unsigned int id = 0;
	unsigned char buffer[2];

	unsigned int data_array[16];
	
	SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(50);	

	
		
//*************Enable CMD2 Page1  *******************//
	data_array[0]=0x00063902;
	data_array[1]=0x52AA55F0;
	data_array[2]=0x00000108;
	dsi_set_cmdq(data_array, 3, 1);
	MDELAY(10); 

	data_array[0] = 0x00023700;// read id return two byte,version and id
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10); 
	
	read_reg_v2(0xC5, buffer,2);//The buffer should be ={0x55, 0x17}
	//id = buffer[0]; //we only need ID 
	//id2= buffer[1]; //we test buffer 1
	id = buffer[0] << 8 | buffer[1];

#if defined(BUILD_LK)
    printf("%s,  id nt35517 id= 0x%x \n", __func__, id);
#else
    printk("%s,  id nt35517 id= 0x%x \n", __func__, id);
#endif

     return (LCM_ID == id)?1:0;


}



LCM_DRIVER nt35517_delitai_qhd_vdo_lcm_drv = 
{
    .name			= "nt35517_delitai_qhd_vdo_lcm_drv",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = lcm_compare_id,	
};

