#include <mach/charging.h>
#include <mach/upmu_common.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>
#include <linux/xlog.h>
#include <linux/delay.h>
#include <mach/mt_sleep.h>
#include <mach/mt_boot.h>
#include <mach/system.h>

#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#include "cust_battery_meter.h"
#include <cust_charging.h>
#include <cust_pmic.h>
#include <mach/mt6311.h>

     
 // ============================================================ //
 //define
 // ============================================================ //
#define STATUS_OK    0
#define STATUS_FAIL	1
#define STATUS_UNSUPPORTED    -1
#define GETARRAYNUM(array) (sizeof(array)/sizeof(array[0]))


 // ============================================================ //
 //global variable
 // ============================================================ //
kal_bool chargin_hw_init_done = KAL_TRUE; 
kal_bool charging_type_det_done = KAL_TRUE;

const kal_uint32 VBAT_CV_VTH[]=
{
	BATTERY_VOLT_03_775000_V, BATTERY_VOLT_03_800000_V, BATTERY_VOLT_03_850000_V, BATTERY_VOLT_03_900000_V,
	BATTERY_VOLT_04_000000_V, BATTERY_VOLT_04_050000_V, BATTERY_VOLT_04_100000_V, BATTERY_VOLT_04_125000_V,
	BATTERY_VOLT_04_137500_V, BATTERY_VOLT_04_150000_V, BATTERY_VOLT_04_162500_V, BATTERY_VOLT_04_175000_V,
	BATTERY_VOLT_04_187500_V, BATTERY_VOLT_04_200000_V, BATTERY_VOLT_04_212500_V, BATTERY_VOLT_04_225000_V,
    
	BATTERY_VOLT_04_237500_V, BATTERY_VOLT_04_250000_V, BATTERY_VOLT_04_262500_V, BATTERY_VOLT_04_275000_V,
	BATTERY_VOLT_04_300000_V, BATTERY_VOLT_04_325000_V, BATTERY_VOLT_04_350000_V, BATTERY_VOLT_04_375000_V,
	BATTERY_VOLT_04_440000_V, BATTERY_VOLT_04_425000_V, BATTERY_VOLT_02_200000_V, BATTERY_VOLT_02_200000_V,    
	BATTERY_VOLT_02_200000_V, BATTERY_VOLT_02_200000_V, BATTERY_VOLT_02_200000_V, BATTERY_VOLT_02_200000_V,
    
	BATTERY_VOLT_02_200000_V, BATTERY_VOLT_02_200000_V, BATTERY_VOLT_02_200000_V, BATTERY_VOLT_02_200000_V,
	BATTERY_VOLT_02_200000_V, BATTERY_VOLT_02_200000_V, BATTERY_VOLT_02_200000_V, BATTERY_VOLT_02_200000_V,
	BATTERY_VOLT_02_200000_V, BATTERY_VOLT_02_200000_V, BATTERY_VOLT_02_200000_V, BATTERY_VOLT_02_200000_V,
    BATTERY_VOLT_02_200000_V, BATTERY_VOLT_02_200000_V, BATTERY_VOLT_02_200000_V, BATTERY_VOLT_02_200000_V,
    
    BATTERY_VOLT_02_200000_V, BATTERY_VOLT_02_200000_V, BATTERY_VOLT_02_200000_V, BATTERY_VOLT_02_200000_V,
    BATTERY_VOLT_02_200000_V, BATTERY_VOLT_02_200000_V, BATTERY_VOLT_02_200000_V, BATTERY_VOLT_02_200000_V,
    BATTERY_VOLT_02_200000_V, BATTERY_VOLT_02_200000_V, BATTERY_VOLT_02_200000_V, BATTERY_VOLT_02_200000_V,
    BATTERY_VOLT_02_200000_V, BATTERY_VOLT_02_200000_V, BATTERY_VOLT_02_200000_V, BATTERY_VOLT_02_200000_V
};

const kal_uint32 CS_VTH[]=
{    
	CHARGE_CURRENT_2000_00_MA, CHARGE_CURRENT_1600_00_MA, CHARGE_CURRENT_1500_00_MA, CHARGE_CURRENT_1350_00_MA,
	CHARGE_CURRENT_1200_00_MA, CHARGE_CURRENT_1100_00_MA, CHARGE_CURRENT_1000_00_MA, CHARGE_CURRENT_900_00_MA,
	CHARGE_CURRENT_800_00_MA,  CHARGE_CURRENT_700_00_MA,  CHARGE_CURRENT_650_00_MA,  CHARGE_CURRENT_550_00_MA,
	CHARGE_CURRENT_450_00_MA,  CHARGE_CURRENT_300_00_MA,  CHARGE_CURRENT_200_00_MA,  CHARGE_CURRENT_70_00_MA
}; 


const kal_uint32 VCDT_HV_VTH[]=
{
	 BATTERY_VOLT_04_200000_V, BATTERY_VOLT_04_250000_V, BATTERY_VOLT_04_300000_V, BATTERY_VOLT_04_350000_V,
	 BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_450000_V, BATTERY_VOLT_04_500000_V, BATTERY_VOLT_04_550000_V,
	 
	 BATTERY_VOLT_06_000000_V, BATTERY_VOLT_06_000000_V, BATTERY_VOLT_06_500000_V, BATTERY_VOLT_07_000000_V,
	 BATTERY_VOLT_07_500000_V, BATTERY_VOLT_08_500000_V, BATTERY_VOLT_09_500000_V, BATTERY_VOLT_10_500000_V		 
};

// ============================================================ //
// function prototype
// ============================================================ //
 
 
// ============================================================ //
//extern variable
// ============================================================ //
 
// ============================================================ //
//extern function
// ============================================================ //
extern kal_uint32 upmu_get_reg_value(kal_uint32 reg);
extern void Charger_Detect_Init(void);
extern void Charger_Detect_Release(void);
extern int hw_charging_get_charger_type(void);
extern void mt_power_off(void);
extern kal_uint32 mt6311_get_chip_id(void);
extern int is_mt6311_exist(void);
extern int is_mt6311_sw_ready(void);

 
 // ============================================================ //
kal_uint32 charging_value_to_parameter(const kal_uint32 *parameter, const kal_uint32 array_size, const kal_uint32 val)
{
    if (val < array_size)
    {
        return parameter[val];
    }
    else
    {
        battery_xlog_printk(BAT_LOG_CRTI, "Can't find the parameter \r\n");    
        return parameter[0];
    }
}

 
kal_uint32 charging_parameter_to_value(const kal_uint32 *parameter, const kal_uint32 array_size, const kal_uint32 val)
{
    kal_uint32 i;

    for(i=0;i<array_size;i++)
    {
        if (val == *(parameter + i))
        {
                return i;
        }
    }

     battery_xlog_printk(BAT_LOG_CRTI, "NO register value match \r\n");
    //TODO: ASSERT(0);    // not find the value
    return 0;
}


static kal_uint32 bmt_find_closest_level(const kal_uint32 *pList,kal_uint32 number,kal_uint32 level)
{
     kal_uint32 i;
     kal_uint32 max_value_in_last_element;
 
     if(pList[0] < pList[1])
         max_value_in_last_element = KAL_TRUE;
     else
         max_value_in_last_element = KAL_FALSE;
 
     if(max_value_in_last_element == KAL_TRUE)
     {
         for(i = (number-1); i != 0; i--)     //max value in the last element
         {
             if(pList[i] <= level)
             {
                 return pList[i];
             }      
         }

         battery_xlog_printk(BAT_LOG_CRTI, "Can't find closest level \r\n");    
         return pList[0];
         //return CHARGE_CURRENT_0_00_MA;
     }
     else
     {
         for(i = 0; i< number; i++)  // max value in the first element
         {
             if(pList[i] <= level)
             {
                 return pList[i];
             }      
         }

          battery_xlog_printk(BAT_LOG_CRTI, "Can't find closest level \r\n");
         return pList[number -1];
         //return CHARGE_CURRENT_0_00_MA;
     }
}

static kal_uint32 charging_hw_init(void *data)
{
    kal_uint32 status = STATUS_OK;

/*
	pmic_set_register_value(PMIC_RG_CHRWDT_TD,0); 	// CHRWDT_TD, 4s
	pmic_set_register_value(PMIC_RG_CHRWDT_INT_EN,1); // CHRWDT_INT_EN
	pmic_set_register_value(PMIC_RG_CHRWDT_EN,1); // CHRWDT_EN
	pmic_set_register_value(PMIC_RG_CHRWDT_WR,1);// CHRWDT_WR*/

	pmic_set_register_value(PMIC_RG_VCDT_MODE,0);//VCDT_MODE
	pmic_set_register_value(PMIC_RG_VCDT_HV_EN,1);//VCDT_HV_EN 

	pmic_set_register_value(PMIC_RG_USBDL_SET,0);//force leave USBDL mode 
	pmic_set_register_value(PMIC_RG_USBDL_RST,1);//force leave USBDL mode

	pmic_set_register_value(PMIC_RG_BC11_BB_CTRL,1);//BC11_BB_CTRL 
	pmic_set_register_value(PMIC_RG_BC11_RST,1);//BC11_RST

	pmic_set_register_value(PMIC_RG_CSDAC_MODE,1);//CSDAC_MODE
	pmic_set_register_value(PMIC_RG_VBAT_OV_EN,1);//VBAT_OV_EN

#ifdef HIGH_BATTERY_VOLTAGE_SUPPORT
	pmic_set_register_value(PMIC_RG_VBAT_OV_VTH,3);//VBAT_OV_VTH, 4.4V,
#else
	pmic_set_register_value(PMIC_RG_VBAT_OV_VTH,2);//VBAT_OV_VTH, 4.3V,
#endif
	pmic_set_register_value(PMIC_RG_BATON_EN,1);//BATON_EN

	//Tim, for TBAT
	pmic_set_register_value(PMIC_RG_BATON_HT_EN,0);//BATON_HT_EN

	pmic_set_register_value(PMIC_RG_ULC_DET_EN,1);// RG_ULC_DET_EN=1
	pmic_set_register_value(PMIC_RG_LOW_ICH_DB,1);// RG_LOW_ICH_DB=000001'b


    #if defined(CONFIG_MTK_PUMP_EXPRESS_SUPPORT)
	pmic_set_register_value(PMIC_RG_CSDAC_DLY,0);// CSDAC_DLY	
	pmic_set_register_value(PMIC_RG_CSDAC_STP,1);// CSDAC_STP
	pmic_set_register_value(PMIC_RG_CSDAC_STP_INC,1);// CSDAC_STP_INC 
	pmic_set_register_value(PMIC_RG_CSDAC_STP_DEC,7);// CSDAC_STP_DEC
	pmic_set_register_value(PMIC_RG_CS_EN,1);// CS_EN 

	pmic_set_register_value(PMIC_RG_HWCV_EN,1);
	pmic_set_register_value(PMIC_RG_VBAT_CV_EN,1);// CV_EN 

	pmic_set_register_value(PMIC_RG_CHR_EN,1);// CHR_EN 
	pmic_set_register_value(PMIC_RG_CSDAC_EN,1);// CSDAC_EN 
    #endif

    return status;
}


static kal_uint32 charging_dump_register(void *data)
{
    kal_uint32 status = STATUS_OK;
/*
    kal_uint32 reg_val = 0;    
    kal_uint32 i = 0;

    for(i=MT6328_CHR_CON0 ; i<=MT6328_CHR_CON40 ; i+=2)
    {
        reg_val = upmu_get_reg_value(i);
        battery_xlog_printk(BAT_LOG_CRTI, "[0x%x]=0x%x,", i, reg_val);
    }

    battery_xlog_printk(BAT_LOG_CRTI, "\n");
*/
    return status;
}
     

static kal_uint32 charging_enable(void *data)
{
    kal_uint32 status = STATUS_OK;
    kal_uint32 enable = *(kal_uint32*)(data);
printk("\ncharging_enable= %d\n",enable);

    if(KAL_TRUE == enable)
    {
		mt_set_gpio_mode(GPIO_CHR_CE_PIN,GPIO_MODE_00);  
		mt_set_gpio_dir(GPIO_CHR_CE_PIN,GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CHR_CE_PIN,GPIO_OUT_ZERO);
    }
	else
	{
		mt_set_gpio_mode(GPIO_CHR_CE_PIN,GPIO_MODE_00);  
		mt_set_gpio_dir(GPIO_CHR_CE_PIN,GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CHR_CE_PIN,GPIO_OUT_ONE);
	}    
    return status;
}


static kal_uint32 charging_set_cv_voltage(void *data)
{
    kal_uint32 status = STATUS_OK;

    return status;
}     


static kal_uint32 charging_get_current(void *data)
{
    kal_uint32 status = STATUS_OK;
    kal_uint32 array_size;
    kal_uint32 reg_value;

    array_size = GETARRAYNUM(CS_VTH);
        reg_value=pmic_get_register_value(PMIC_RG_CS_VTH);//RG_CS_VTH
    *(kal_uint32 *)data = charging_value_to_parameter(CS_VTH,array_size,reg_value);
    
    return status;
}  
  

static kal_uint32 charging_set_current(void *data)
{
    kal_uint32 status = STATUS_OK;

    if(*(kal_uint32 *)data > CHARGE_CURRENT_500_00_MA)
    {
	     mt_set_gpio_mode(GPIO_CHR_PSEL_PIN,GPIO_MODE_00);  
	     mt_set_gpio_dir(GPIO_CHR_PSEL_PIN,GPIO_DIR_OUT);
	     mt_set_gpio_out(GPIO_CHR_PSEL_PIN,GPIO_OUT_ZERO);
    }
    else
    {
        mt_set_gpio_mode(GPIO_CHR_PSEL_PIN,GPIO_MODE_00);  
        mt_set_gpio_dir(GPIO_CHR_PSEL_PIN,GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_CHR_PSEL_PIN,GPIO_OUT_ONE);
    }

    return status;
}     


static kal_uint32 charging_set_input_current(void *data)
{
    kal_uint32 status = STATUS_OK;
    return status;
}     

static kal_uint32 charging_get_charging_status(void *data)
{
    kal_uint32 status = STATUS_OK;

/*** Do not determine bat full by the GPIO_EINT_CHG_STAT_PIN ***
    mt_set_gpio_mode(GPIO_EINT_CHG_STAT_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_EINT_CHG_STAT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_EINT_CHG_STAT_PIN, GPIO_PULL_DISABLE);
   	if(mt_get_gpio_in(GPIO_EINT_CHG_STAT_PIN))
		*(kal_uint32 *)data = KAL_TRUE;//charge full
	else
		*(kal_uint32 *)data = KAL_FALSE;
	***modified by liangbo 20150604***/
    return status;
}

 
static kal_uint32 charging_reset_watch_dog_timer(void *data)
{
	kal_uint32 status = STATUS_OK;
/*
	pmic_set_register_value(PMIC_RG_CHRWDT_TD,0);	// CHRWDT_TD, 4s
	pmic_set_register_value(PMIC_RG_CHRWDT_WR,1);	// CHRWDT_WR
	pmic_set_register_value(PMIC_RG_CHRWDT_INT_EN,1);	 // CHRWDT_INT_EN
	pmic_set_register_value(PMIC_RG_CHRWDT_EN,1);	// CHRWDT_EN
	pmic_set_register_value(PMIC_RG_CHRWDT_FLAG_WR,1);	// CHRWDT_WR
*/
	return status;
}


static kal_uint32 charging_set_hv_threshold(void *data)
{
	kal_uint32 status = STATUS_OK;

	kal_uint32 set_hv_voltage;
	kal_uint32 array_size;
	kal_uint16 register_value;
	kal_uint32 voltage = *(kal_uint32*)(data);

	array_size = GETARRAYNUM(VCDT_HV_VTH);
	set_hv_voltage = bmt_find_closest_level(VCDT_HV_VTH, array_size, voltage);
	register_value = charging_parameter_to_value(VCDT_HV_VTH, array_size ,set_hv_voltage);
	pmic_set_register_value(PMIC_RG_VCDT_HV_VTH,register_value);

	return status;
}


static kal_uint32 charging_get_hv_status(void *data)
{
    kal_uint32 status = STATUS_OK;

    *(kal_bool*)(data) = pmic_get_register_value(PMIC_RGS_VCDT_HV_DET);
       
    return status;
}

        
static kal_uint32 charging_get_battery_status(void *data)
{
    kal_uint32 status = STATUS_OK;
    kal_uint32 val = 0;
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
    *(kal_bool*)(data) = 0; // battery exist
    battery_xlog_printk(BAT_LOG_CRTI,"bat exist for evb\n");
#else
	val=pmic_get_register_value(PMIC_BATON_TDET_EN);
	battery_xlog_printk(BAT_LOG_FULL,"[charging_get_battery_status] BATON_TDET_EN = %d\n", val);
	if (val) {
	pmic_set_register_value(PMIC_BATON_TDET_EN,1);
	pmic_set_register_value(PMIC_RG_BATON_EN,1);
	*(kal_bool*)(data) = pmic_get_register_value(PMIC_RGS_BATON_UNDET);
	} else {
		*(kal_bool*)(data) =  KAL_FALSE;
	}
#endif
      
    return status;
}

    
static kal_uint32 charging_get_charger_det_status(void *data)
{
	kal_uint32 status = STATUS_OK;

#if defined(CONFIG_MTK_FPGA)
	*(kal_bool*)(data) = 1; 
	battery_xlog_printk(BAT_LOG_CRTI,"chr exist for fpga\n");
#else    
	*(kal_bool*)(data) = pmic_get_register_value(PMIC_RGS_CHRDET);
#endif     
	   
	 return status;
}


kal_bool charging_type_detection_done(void)
{
     return charging_type_det_done;
}


static kal_uint32 charging_get_charger_type(void *data)
{
    kal_uint32 status = STATUS_OK;
     
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
    *(CHARGER_TYPE*)(data) = STANDARD_HOST;
#else
    *(CHARGER_TYPE*)(data) = hw_charging_get_charger_type();
#endif

     return status;
}

static kal_uint32 charging_get_is_pcm_timer_trigger(void *data)
{
     kal_uint32 status = STATUS_OK;
     
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
    *(kal_bool*)(data) = KAL_FALSE;
#else 

   //  if(slp_get_wake_reason() == WR_PCM_TIMER)
   //      *(kal_bool*)(data) = KAL_TRUE;
  //   else
         *(kal_bool*)(data) = KAL_FALSE;
 
 //    battery_xlog_printk(BAT_LOG_CRTI, "slp_get_wake_reason=%d\n", slp_get_wake_reason());

#endif
        
    return status;
 }
 
 static kal_uint32 charging_set_platform_reset(void *data)
 {
     kal_uint32 status = STATUS_OK;
     
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)    
#else 
     battery_xlog_printk(BAT_LOG_CRTI, "charging_set_platform_reset\n");
  
     arch_reset(0,NULL);
#endif
         
     return status;
 }
 
 static kal_uint32 charging_get_platfrom_boot_mode(void *data)
 {
     kal_uint32 status = STATUS_OK;
     
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)   
#else   
     *(kal_uint32*)(data) = get_boot_mode();
 
     battery_xlog_printk(BAT_LOG_CRTI, "get_boot_mode=%d\n", get_boot_mode());
#endif
          
     return status;
}

static kal_uint32 charging_set_power_off(void *data)
{
    kal_uint32 status = STATUS_OK;
  
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
    battery_xlog_printk(BAT_LOG_CRTI, "charging_set_power_off\n");
    mt_power_off();
#endif
         
    return status;
}

static kal_uint32 charging_get_power_source(void *data)
{
    kal_uint32 status = STATUS_OK;	
    *(kal_bool *)data = KAL_FALSE;

    return status;
}

static kal_uint32 charging_get_csdac_full_flag(void *data)
{
    return STATUS_UNSUPPORTED;
}

static kal_uint32 charging_set_ta_current_pattern(void *data)
{
	return STATUS_UNSUPPORTED;	
}

static kal_uint32 charging_get_error_state(void *data)
{
	return STATUS_UNSUPPORTED;
}

static kal_uint32 charging_set_error_state(void *data)
{
	return STATUS_UNSUPPORTED;	
}

static kal_uint32 (* const charging_func[CHARGING_CMD_NUMBER])(void *data)=
{
	 charging_hw_init
	,charging_dump_register      
	,charging_enable
	,charging_set_cv_voltage
	,charging_get_current
	,charging_set_current
	,charging_set_input_current
	,charging_get_charging_status
	,charging_reset_watch_dog_timer
	,charging_set_hv_threshold
	,charging_get_hv_status
	,charging_get_battery_status
	,charging_get_charger_det_status
	,charging_get_charger_type
	,charging_get_is_pcm_timer_trigger
	,charging_set_platform_reset
	,charging_get_platfrom_boot_mode
	,charging_set_power_off
	,charging_get_power_source
	,charging_get_csdac_full_flag
	,charging_set_ta_current_pattern
	,charging_set_error_state
};
 
 
 /*
 * FUNCTION
 *        Internal_chr_control_handler
 *
 * DESCRIPTION                                                             
 *         This function is called to set the charger hw
 *
 * CALLS  
 *
 * PARAMETERS
 *        None
 *     
 * RETURNS
 *        
 *
 * GLOBALS AFFECTED
 *       None
 */
kal_int32 chr_control_interface(CHARGING_CTRL_CMD cmd, void *data)
{
     kal_int32 status;
     if(cmd < CHARGING_CMD_NUMBER)
         status = charging_func[cmd](data);
     else
         return STATUS_UNSUPPORTED;
 
     return status;
}


