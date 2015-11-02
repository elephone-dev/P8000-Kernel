////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2006-2014 MStar Semiconductor, Inc.
// All rights reserved.
//
// Unless otherwise stipulated in writing, any and all information contained
// herein regardless in any format shall remain the sole proprietary of
// MStar Semiconductor Inc. and be kept in strict confidence
// (??MStar Confidential Information??) by the recipient.
// Any unauthorized act including without limitation unauthorized disclosure,
// copying, use, reproduction, sale, distribution, modification, disassembling,
// reverse engineering and compiling of the contents of MStar Confidential
// Information is unlawful and strictly prohibited. MStar hereby reserves the
// rights to any and all damages, losses, costs and expenses resulting therefrom.
//
////////////////////////////////////////////////////////////////////////////////

/**
 *
 * @file    mstar_drv_platform_interface.c
 *
 * @brief   This file defines the interface of touch screen
 *
 *
 */

/*=============================================================*/
// INCLUDE FILE
/*=============================================================*/

#include "mstar_drv_platform_interface.h"
#include "mstar_drv_main.h"
#include "mstar_drv_ic_fw_porting_layer.h"
#include "mstar_drv_platform_porting_layer.h"
#include "mstar_drv_utility_adaption.h"

#ifdef CONFIG_ENABLE_HOTKNOT
#include "mstar_drv_hotknot.h"
#endif //CONFIG_ENABLE_HOTKNOT

/*=============================================================*/
// EXTERN VARIABLE DECLARATION
/*=============================================================*/

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
extern u32 g_GestureWakeupMode[2];
extern u8 g_GestureWakeupFlag;

#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
extern u8 g_GestureDebugFlag;
extern u8 g_GestureDebugMode;
#endif //CONFIG_ENABLE_GESTURE_DEBUG_MODE

#endif //CONFIG_ENABLE_GESTURE_WAKEUP

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
extern u8 g_EnableTpProximity;
#endif //CONFIG_ENABLE_PROXIMITY_DETECTION

/*=============================================================*/
// GLOBAL VARIABLE DEFINITION
/*=============================================================*/

extern struct input_dev *g_InputDevice;

#ifdef CONFIG_ENABLE_HOTKNOT
extern u8 g_HotKnotState;
#endif //CONFIG_ENABLE_HOTKNOT

/*=============================================================*/
// GLOBAL FUNCTION DEFINITION
/*=============================================================*/
//#define TPD_PROXIMITY
#ifdef TPD_PROXIMITY
#include <alsps.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>

int tpd_proximity_flag = 0;
int g_nPsSensorDate = 0;
static int mstar_tpd_enable_ps(int enable)
{
		u8 ps_store_data[4];
	u8 i;
	s16 rc = 0;
	
	//mutex_lock(&msg2133_sensor_mutex);
	if(enable == 1)
	{
		//if(tpd_proximity_flag == 0)
		{
			ps_store_data[0] = 0x52;
			ps_store_data[1] = 0x00;
			ps_store_data[2] = 0x4a;
			ps_store_data[3] = 0xa0;

			while(i < 5)
            {
				rc = IicWriteData(0x26, &ps_store_data[0], 4);
				if(rc >0)
				{
					tpd_proximity_flag = 1;
        				printk("MSG2633 open ps success!!!\n");
					break;
            }
				else
				{
					printk("MSG2633 open ps fail!!!\n");
					i++;
				}
			}

			if(i == 5 || rc < 0)
			{
				//tpd_proximity_flag = 0;
    				printk("MSG2633 open ps fail!!!\n");
			}						// suspend()µÄ²Ù×÷»áÊ¹TPÏÂµç
		}
	}
	else
	{	
		//if(tpd_proximity_flag == 1)
		{
			ps_store_data[0] = 0x52;
			ps_store_data[1] = 0x00;
			ps_store_data[2] = 0x4a;
			ps_store_data[3] = 0xa1;
			while(i < 5)
            {
				rc = IicWriteData(0x26, &ps_store_data[0], 4);
				if(rc >0)
				{
			tpd_proximity_flag = 0;			
        				printk("MSG2633 close ps success!!!\n");
					break;
		}
				else
				{
					i++;
				}
			}
			if(i == 5 || rc < 0)
			{
				//tpd_proximity_flag = 1;
    				printk("MSG2633 close ps fail!!!\n");
			}		
		}
	}
	//mutex_unlock(&msg2133_sensor_mutex);
	return 0;
}

 static int tpd_get_ps_value(void)
  {
  return 0;
  }
  
static int ps_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}
static int ps_set_delay(u64 ns)
{
	return 0;
}
static int ps_get_data(int* value, int* status)
{
	return 0;

}

int register_mstar_alsps(void)
{
                if(tpd_load_status == 0) 
                        return -1;
		int err;
		struct hwmsen_object obj_ps;	
		struct ps_control_path ps_ctl={0};
		struct ps_data_path ps_data={0};//0511 -2
		//set_bit(CMC_BIT_PS, &obj->enable);
		
		ps_ctl.open_report_data= ps_open_report_data;
		ps_ctl.enable_nodata = mstar_tpd_enable_ps;//ps_enable_nodata;
		ps_ctl.set_delay  = ps_set_delay;
		ps_ctl.is_report_input_direct = true;
#ifdef CUSTOM_KERNEL_SENSORHUB
		ps_ctl.is_support_batch = true;
#else
		ps_ctl.is_support_batch = false;
#endif
		err = ps_register_control_path_for_kpd(&ps_ctl);
		if(err)
		{
			printk("register fail = %d\n", err);

			return -1;
			//goto exit_register_path_failed;
		}
		ps_data.get_data = ps_get_data;
		ps_data.vender_div = 100;
		err = ps_register_data_path(&ps_data);	
		if(err)
		{
			printk("tregister fail = %d\n", err);
			return -1;
			//goto exit_register_path_failed;
		}

	err = batch_register_support_info(ID_PROXIMITY,ps_ctl.is_support_batch, 100, 0);
	if(err)
	{
		printk("register proximity batch support err = %d\n", err);
		return -1;
	}
	return 0;
}
#else
int register_mstar_alsps(void)
{
	return -1;
}
#endif


#ifdef CONFIG_ENABLE_NOTIFIER_FB
int MsDrvInterfaceTouchDeviceFbNotifierCallback(struct notifier_block *pSelf, unsigned long nEvent, void *pData)
{
    struct fb_event *pEventData = pData;
    int *pBlank;

    if (pEventData && pEventData->data && nEvent == FB_EVENT_BLANK)
    {
        pBlank = pEventData->data;

        if (*pBlank == FB_BLANK_UNBLANK)
        {
            DBG("*** %s() TP Resume ***\n", __func__);

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
            if (g_EnableTpProximity == 1)
            {
                DBG("g_EnableTpProximity = %d\n", g_EnableTpProximity);
                return 0;
            }
#endif //CONFIG_ENABLE_PROXIMITY_DETECTION
            
#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
            if (g_GestureDebugMode == 1)
            {
                DrvIcFwLyrCloseGestureDebugMode();
            }
#endif //CONFIG_ENABLE_GESTURE_DEBUG_MODE

            if (g_GestureWakeupFlag == 1)
            {
                DrvIcFwLyrCloseGestureWakeup();
            }
            else
            {
                DrvPlatformLyrEnableFingerTouchReport(); 
            }
#endif //CONFIG_ENABLE_GESTURE_WAKEUP
    
#ifdef CONFIG_ENABLE_HOTKNOT
            if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif //CONFIG_ENABLE_HOTKNOT        
            {
                DrvPlatformLyrTouchDevicePowerOn(); 
            }   
    
#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
            DrvIcFwLyrRestoreFirmwareModeToLogDataMode(); // Mark this function call for avoiding device driver may spend longer time to resume from suspend state.
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

#ifndef CONFIG_ENABLE_GESTURE_WAKEUP
            DrvPlatformLyrEnableFingerTouchReport(); 
#endif //CONFIG_ENABLE_GESTURE_WAKEUP
        }
        else if (*pBlank == FB_BLANK_POWERDOWN)
        {
            DBG("*** %s() TP Suspend ***\n", __func__);
            
#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
            if (g_EnableTpProximity == 1)
            {
                DBG("g_EnableTpProximity = %d\n", g_EnableTpProximity);
                return 0;
            }
#endif //CONFIG_ENABLE_PROXIMITY_DETECTION

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
            if (g_GestureWakeupMode[0] != 0x00000000 || g_GestureWakeupMode[1] != 0x00000000)
            {
                DrvIcFwLyrOpenGestureWakeup(&g_GestureWakeupMode[0]);
                return 0;
            }
#endif //CONFIG_ENABLE_GESTURE_WAKEUP


            DrvPlatformLyrFingerTouchReleased(0, 0); // Send touch end for clearing point touch
            input_sync(g_InputDevice);

            DrvPlatformLyrDisableFingerTouchReport();

#ifdef CONFIG_ENABLE_HOTKNOT
            if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif //CONFIG_ENABLE_HOTKNOT        
            {
                DrvPlatformLyrTouchDevicePowerOff(); 
            }    
        }
    }

    return 0;
}

#else

void MsDrvInterfaceTouchDeviceSuspend(struct early_suspend *pSuspend)
{
    DBG("*** %s() ***\n", __func__);

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
    if (g_EnableTpProximity == 1)
    {
        DBG("g_EnableTpProximity = %d\n", g_EnableTpProximity);
        return;
    }
#endif //CONFIG_ENABLE_PROXIMITY_DETECTION
#ifdef TPD_PROXIMITY

  if (tpd_proximity_flag == 1)
    {
    	//tpd_proximity_flag =0;
        DBG("tpd_proximity_flag = %d\n", tpd_proximity_flag);
        return;
    }
#endif

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
    if (g_GestureWakeupMode[0] != 0x00000000 || g_GestureWakeupMode[1] != 0x00000000)
    {
        DrvIcFwLyrOpenGestureWakeup(&g_GestureWakeupMode[0]);
        return;
    }
#endif //CONFIG_ENABLE_GESTURE_WAKEUP

    DrvPlatformLyrFingerTouchReleased(0, 0); // Send touch end for clearing point touch
    input_sync(g_InputDevice);

    DrvPlatformLyrDisableFingerTouchReport();

#ifdef CONFIG_ENABLE_HOTKNOT
    if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif //CONFIG_ENABLE_HOTKNOT        
    {
        DrvPlatformLyrTouchDevicePowerOff();  
    }    
}


void MsDrvInterfaceTouchDeviceResume(struct early_suspend *pSuspend)
{
    DBG("*** %s() ***\n", __func__);

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
    if (g_EnableTpProximity == 1)
    {
        DBG("g_EnableTpProximity = %d\n", g_EnableTpProximity);
        return;
    }
#endif //CONFIG_ENABLE_PROXIMITY_DETECTION
#ifdef TPD_PROXIMITY

  if (tpd_proximity_flag == 1)
    {
    	//tpd_proximity_flag =0;
        DBG("tpd_proximity_flag = %d\n", tpd_proximity_flag);
        return;
    }
#endif

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
    if (g_GestureDebugMode == 1)
    {
        DrvIcFwLyrCloseGestureDebugMode();
    }
#endif //CONFIG_ENABLE_GESTURE_DEBUG_MODE

    if (g_GestureWakeupFlag == 1)
    {
        DrvIcFwLyrCloseGestureWakeup();
    }
    else
    {
        DrvPlatformLyrEnableFingerTouchReport(); 
    }
#endif //CONFIG_ENABLE_GESTURE_WAKEUP
    
#ifdef CONFIG_ENABLE_HOTKNOT
    if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif //CONFIG_ENABLE_HOTKNOT        
    {
        DrvPlatformLyrTouchDevicePowerOn(); 
    }   
    
#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    DrvIcFwLyrRestoreFirmwareModeToLogDataMode(); // Mark this function call for avoiding device driver may spend longer time to resume from suspend state.
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

#ifndef CONFIG_ENABLE_GESTURE_WAKEUP
    DrvPlatformLyrEnableFingerTouchReport(); 
#endif //CONFIG_ENABLE_GESTURE_WAKEUP
}

#endif //CONFIG_ENABLE_NOTIFIER_FB

/* probe function is used for matching and initializing input device */
s32 /*__devinit*/ MsDrvInterfaceTouchDeviceProbe(struct i2c_client *pClient, const struct i2c_device_id *pDeviceId)
{
    s32 nRetVal = 0;

    DBG("*** %s() ***\n", __func__);
  
    DrvPlatformLyrInputDeviceInitialize(pClient);
  
    DrvPlatformLyrTouchDeviceRequestGPIO();

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
    DrvPlatformLyrTouchDeviceRegulatorPowerOn();
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON

    DrvPlatformLyrTouchDevicePowerOn();

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_ENABLE_DMA_IIC
    DmaAlloc();
#endif //CONFIG_ENABLE_DMA_IIC
#endif //CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM

    nRetVal = DrvMainTouchDeviceInitialize();
    if (nRetVal == -ENODEV)
    {
        DrvPlatformLyrTouchDeviceRemove(pClient);
        return nRetVal;
    }

    DrvPlatformLyrTouchDeviceRegisterFingerTouchInterruptHandler();

    DrvPlatformLyrTouchDeviceRegisterEarlySuspend();

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
    DrvIcFwLyrCheckFirmwareUpdateBySwId();
#endif //CONFIG_UPDATE_FIRMWARE_BY_SW_ID

    DBG("*** MStar touch driver registered ***\n");
    
    return nRetVal;
}

/* remove function is triggered when the input device is removed from input sub-system */
s32 /*__devexit*/ MsDrvInterfaceTouchDeviceRemove(struct i2c_client *pClient)
{
    DBG("*** %s() ***\n", __func__);

    return DrvPlatformLyrTouchDeviceRemove(pClient);
}

void MsDrvInterfaceTouchDeviceSetIicDataRate(struct i2c_client *pClient, u32 nIicDataRate)
{
    DBG("*** %s() ***\n", __func__);

    DrvPlatformLyrSetIicDataRate(pClient, nIicDataRate);
}    
