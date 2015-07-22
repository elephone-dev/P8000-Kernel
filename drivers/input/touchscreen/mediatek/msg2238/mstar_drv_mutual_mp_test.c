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
 * @file    mstar_drv_mutual_mp_test.c
 *
 * @brief   This file defines the interface of touch screen
 *
 *
 */

/*=============================================================*/
// INCLUDE FILE
/*=============================================================*/

#include "mstar_drv_mutual_mp_test.h"
#include "mstar_drv_utility_adaption.h"
#include "mstar_drv_mutual_fw_control.h"
#include "mstar_drv_platform_porting_layer.h"

#if defined(CONFIG_ENABLE_TOUCH_DRIVER_FOR_MUTUAL_IC)
#ifdef CONFIG_ENABLE_ITO_MP_TEST

#include "short_test_X.h"

/*=============================================================*/
// EXTERN VARIABLE DECLARATION
/*=============================================================*/

extern u32 SLAVE_I2C_ID_DBBUS;
extern u32 SLAVE_I2C_ID_DWI2C;

extern u8 g_ChipType;

/*=============================================================*/
// LOCAL VARIABLE DEFINITION
/*=============================================================*/

static u32 _gIsInMpTest = 0;
static u32 _gTestRetryCount = CTP_MP_TEST_RETRY_COUNT;
static ItoTestMode_e _gItoTestMode = 0;

static s32 _gCtpMpTestStatus = ITO_TEST_UNDER_TESTING;

static u16 _gSenseLineNum = 0;
static u16 _gDriveLineNum = 0;

static struct work_struct _gCtpItoTestWork;
static struct workqueue_struct *_gCtpMpTestWorkQueue = NULL;
 
static s32 _gDeltaC[MAX_MUTUAL_NUM] = {0};
static s32 _gResult[MAX_MUTUAL_NUM] = {0};
static u8 _gMode[MAX_MUTUAL_NUM] = {0};
static s32 _gSenseR[MAX_CHANNEL_NUM] = {0};
static s32 _gDriveR[MAX_CHANNEL_NUM] = {0};

static s32 _gTempDeltaC[MAX_MUTUAL_NUM] = {0};

static u8 _gTestFailChannel[MAX_MUTUAL_NUM] = {0};
static u32 _gTestFailChannelCount = 0;

static u8 _gShortTestChannel[MAX_CHANNEL_NUM] = {0};

TestScopeInfo_t g_TestScopeInfo = {0};

/*=============================================================*/
// LOCAL FUNCTION DEFINITION
/*=============================================================*/

static u16 _DrvMpTestItoOpenTestFirmwareGetState(void)
{
    u16 nCheckState = 0;

    DBG("*** %s() ***\n", __func__);

    nCheckState = RegGet16BitValue(0x3CDE); //bank:reg_PIU_MISC_0, addr:h006f

    return nCheckState;
}

static void _DrvMpTestItoTestMcuStop(void)
{
    DBG("*** %s() ***\n", __func__);

    RegSet16BitValue(0x0FE6, 0x0001); //bank:mheg5, addr:h0073
}

static void _DrvMpTestItoTestAnaSwitchToMutual(void)
{
    u16 nTemp = 0;

    DBG("*** %s() ***\n", __func__);

    nTemp = RegGet16BitValue(0x114A); //bank:ana, addr:h0025
    nTemp |= BIT0;
    RegSet16BitValue(0x114A, nTemp);
    nTemp = RegGet16BitValue(0x1116); //bank:ana, addr:h000b
    nTemp |= (BIT2 | BIT0);
    RegSet16BitValue(0x1116, nTemp);
}

static u16 _DrvMpTestItoTestAnaGetMutualChannelNum(void)
{
    u16 nSenseLineNum = 0;
    u16 nRegData = 0;

    DBG("*** %s() ***\n", __func__);

    nRegData = RegGet16BitValue(0x102E); //bank:ana3, addr:h0017
    nSenseLineNum = nRegData & 0x000F;

    DBG("nSenseLineNum = %d\n", nSenseLineNum);

    return nSenseLineNum;
}

static u16 _DrvMpTestItoTestAnaGetMutualSubFrameNum(void)
{
    u16 nDriveLineNum = 0;
    u16 nRegData = 0;

    DBG("*** %s() ***\n", __func__);

    nRegData = RegGet16BitValue(0x1216); //bank:ana2, addr:h000b
    nDriveLineNum = ((nRegData & 0xFF00) >> 8) + 1; //Since we only retrieve 8th~12th bit of reg_m_sf_num, 0xFF00 shall be changed to 0x1F00. 

    DBG("nDriveLineNum = %d\n", nDriveLineNum);

    return nDriveLineNum;
}

static void _DrvMpTestItoOpenTestAnaGetMutualCSub(u8 *pMode)
{
    u16 i, j;
    u16 nSenseLineNum;
    u16 nDriveLineNum;
    u16 nTotalNum;
    u8 szDataAna4[3];    
    u8 szDataAna3[3];    
    u8 szDataAna41[ANA4_MUTUAL_CSUB_NUMBER]; //200 = 392 - 192  
    u8 szDataAna31[ANA3_MUTUAL_CSUB_NUMBER]; //192 = 14 * 13 + 10   
    u8 szModeTemp[MAX_MUTUAL_NUM];

    DBG("*** %s() ***\n", __func__);

    nTotalNum = MAX_MUTUAL_NUM;
    nSenseLineNum = _DrvMpTestItoTestAnaGetMutualChannelNum();
    nDriveLineNum = _DrvMpTestItoTestAnaGetMutualSubFrameNum();

    if (ANA4_MUTUAL_CSUB_NUMBER > 0)
    {
        mdelay(100);
        for (i = 0; i < ANA4_MUTUAL_CSUB_NUMBER; i ++)
        {	
            szDataAna41[i] = 0;
        }

        szDataAna4[0] = 0x10;
        szDataAna4[1] = 0x15; //bank:ana4, addr:h0000
        szDataAna4[2] = 0x00;
        
        IicWriteData(SLAVE_I2C_ID_DBBUS, &szDataAna4[0], 3);
        IicReadData(SLAVE_I2C_ID_DBBUS, &szDataAna41[0], ANA4_MUTUAL_CSUB_NUMBER); //200

        nTotalNum -= (u16)ANA4_MUTUAL_CSUB_NUMBER;
    }

    for (i = 0; i < nTotalNum; i ++)
    {
        szDataAna31[i] = 0;
    }

    mdelay(100);

    szDataAna3[0] = 0x10;
    szDataAna3[1] = 0x10; //bank:ana3, addr:h0020
    szDataAna3[2] = 0x40;

    IicWriteData(SLAVE_I2C_ID_DBBUS, &szDataAna3[0], 3);
    IicReadData(SLAVE_I2C_ID_DBBUS, &szDataAna31[0], ANA3_MUTUAL_CSUB_NUMBER); //192

    for (i = 0; i < ANA3_MUTUAL_CSUB_NUMBER; i ++)
    {
        szModeTemp[i] = szDataAna31[i];
    }

    for (i = ANA3_MUTUAL_CSUB_NUMBER; i < (ANA3_MUTUAL_CSUB_NUMBER + ANA4_MUTUAL_CSUB_NUMBER); i ++)
    {
        szModeTemp[i] = szDataAna41[i - ANA3_MUTUAL_CSUB_NUMBER];
    }
    
    for (i = 0; i < nDriveLineNum; i ++)
    {
        for (j = 0; j < nSenseLineNum; j ++)
        {
            _gMode[j * nDriveLineNum + i] = szModeTemp[i * MAX_CHANNEL_SEN + j];

//            DBG("_gMode[%d] = %d\n", j * nDriveLineNum + i, _gMode[j * nDriveLineNum + i]);
        }
    }
}

static void _DrvMpTestItoTestDisableFilterNoiseDetect(void)
{
    u16 nTemp = 0;

    DBG("*** %s() ***\n", __func__);

    nTemp = RegGet16BitValue(0x1302); //bank:fir, addr:h0001
    nTemp &= (~(BIT2 | BIT1 | BIT0));
    RegSet16BitValue(0x1302, nTemp);
}

static void _DrvMpTestItoTestAnaSwReset(void)
{
    DBG("*** %s() ***\n", __func__);

    RegSet16BitValue(0x1100, 0xFFFF); //bank:ana, addr:h0000
    RegSet16BitValue(0x1100, 0x0000);
    mdelay(100);
}

static void _DrvMpTestItoTestEnableAdcOneShot(void)
{
    u16 nTemp = 0;

    DBG("*** %s() ***\n", __func__);

    RegSet16BitValue(0x130C, BIT15); //bank:fir, addr:h0006
    nTemp = RegGet16BitValue(0x1214); //bank:ana2, addr:h000a
    nTemp |= BIT0;
    RegSet16BitValue(0x1214, nTemp);
}

static void _DrvMpTestItoTestGetMutualOneShotRawIir(u16 wszResultData[][MAX_CHANNEL_DRV], u16 nDriveLineNum, u16 nSenseLineNum)
{
    u16 nRegData;
    u16 i, j;
    u16 nTemp;
    u16 nReadSize;
    u8 szDbBusTxData[3];
    u8 szShotData1[FILTER1_MUTUAL_DELTA_C_NUMBER]; //190 = (6 * 14 + 11) * 2
    u8 szShotData2[FILTER2_MUTUAL_DELTA_C_NUMBER]; //594 = (MAX_MUTUAL_NUM - (6 * 14 + 11)) * 2

    DBG("*** %s() ***\n", __func__);

    nTemp = RegGet16BitValue(0x3D08); //bank:intr_ctrl, addr:h0004
    nTemp &= (~(BIT8 | BIT4));
    RegSet16BitValue(0x3D08, nTemp);

    _DrvMpTestItoTestEnableAdcOneShot();
    
    nRegData = 0;
    while (0x0000 == (nRegData & BIT8))
    {
        nRegData = RegGet16BitValue(0x3D18); //bank:intr_ctrl, addr:h000c
    }

    for (i = 0; i < FILTER1_MUTUAL_DELTA_C_NUMBER; i ++)
    {
        szShotData1[i] = 0;
    }
    
    for (i = 0; i < FILTER2_MUTUAL_DELTA_C_NUMBER; i ++)
    {
        szShotData2[i] = 0;
    }

    mdelay(100);
    szDbBusTxData[0] = 0x10;
    szDbBusTxData[1] = 0x13; //bank:fir, addr:h0021
    szDbBusTxData[2] = 0x42;
    IicWriteData(SLAVE_I2C_ID_DBBUS, &szDbBusTxData[0], 3);
    IicReadData(SLAVE_I2C_ID_DBBUS, &szShotData1[0], FILTER1_MUTUAL_DELTA_C_NUMBER); //190

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
    mdelay(100);
    nReadSize = IicSegmentReadDataByDbBus(0x20, 0x00, &szShotData2[0], FILTER2_MUTUAL_DELTA_C_NUMBER, MAX_I2C_TRANSACTION_LENGTH_LIMIT); //594
    DBG("*** nReadSize = %d ***\n", nReadSize); // add for debug
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM)
    mdelay(100);
    szDbBusTxData[0] = 0x10;
    szDbBusTxData[1] = 0x20; //bank:fir2, addr:h0000
    szDbBusTxData[2] = 0x00;
    IicWriteData(SLAVE_I2C_ID_DBBUS, &szDbBusTxData[0], 3);
    IicReadData(SLAVE_I2C_ID_DBBUS, &szShotData2[0], FILTER2_MUTUAL_DELTA_C_NUMBER); //594
#endif

    for (j = 0; j < nDriveLineNum; j ++)
    {
        for (i = 0; i < nSenseLineNum; i ++)
        {
            // FILTER1 : SF0~SF5, AFE0~AFE13; SF6, AFE0~AFE10
            if ((j <= 5) || ((j == 6) && (i <= 10)))
            {
                nRegData = (u16)(szShotData1[(j * 14 + i) * 2] | szShotData1[(j * 14 + i) * 2 + 1] << 8);
                wszResultData[i][ j] = (short)nRegData;
            }
            else
            {
                // FILTER2 : SF6, AFE11~AFE13
                if ((j == 6) && (i > 10))
                {
                    nRegData = (u16)(szShotData2[((j - 6) * 14 + (i - 11)) * 2] | szShotData2[((j - 6) * 14 + (i - 11)) * 2 + 1] << 8);
                    wszResultData[i][j] = (short)nRegData;
                }
                else
                {
                    nRegData = (u16)(szShotData2[6 + ((j - 7) * 14 + i) * 2] | szShotData2[6 + ((j - 7) * 14 + i) * 2 + 1] << 8);
                    wszResultData[i][j] = (short)nRegData;
                }
            }
        }
    }

    nTemp = RegGet16BitValue(0x3D08); //bank:intr_ctrl, addr:h0004
    nTemp |= (BIT8 | BIT4);
    RegSet16BitValue(0x3D08, nTemp);
}

static void _DrvMpTestItoTestGetDeltaC(s32 *pTarget)
{
    s16 nTemp;
    u16 wszRawData[MAX_CHANNEL_SEN][MAX_CHANNEL_DRV];
    u16 i, j;
    u16 nDriveLineNum = 0, nSenseLineNum = 0, nShift = 0;

    DBG("*** %s() ***\n", __func__);

    nSenseLineNum = _DrvMpTestItoTestAnaGetMutualChannelNum();
    nDriveLineNum = _DrvMpTestItoTestAnaGetMutualSubFrameNum();

    _DrvMpTestItoTestGetMutualOneShotRawIir(wszRawData, nDriveLineNum, nSenseLineNum);

    for (i = 0; i < nSenseLineNum; i ++)
    {
        for (j = 0; j < nDriveLineNum; j ++)
        {
            nShift = (u16)(i * nDriveLineNum + j);
            nTemp = (s16)wszRawData[i][j];
            pTarget[nShift] = nTemp;

//            DBG("wszRawData[%d][%d] = %d\n", i, j, nTemp);
        }
    }
}

s32 _DrvMpTestItoOpenTest(void)
{
    s32 nRetVal = 0;
    s32 nPrev = 0, nDelta = 0;
    u16 i = 0, j = 0;
    u16 nCheckState = 0;

    DBG("*** %s() ***\n", __func__);

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_ENABLE_DMA_IIC
    DmaReset();
#endif //CONFIG_ENABLE_DMA_IIC
#endif //CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM

    DrvPlatformLyrDisableFingerTouchReport();

    DrvPlatformLyrTouchDeviceResetHw();

    DbBusEnterSerialDebugMode();
    DbBusStopMCU();
    DbBusIICUseBus();
    DbBusIICReshape();
    mdelay(100);
    
    /*
      0 : SYS_STATE_NULL
      1 : SYS_STATE_INIT
      4 : SYS_STATE_STRIKE
    */
    while ((nCheckState == 0 || nCheckState == 1 || nCheckState == 4) && (i < 10))
    {
        nCheckState = _DrvMpTestItoOpenTestFirmwareGetState();
        mdelay(100);
        i ++;
    }
    
    if (i >= 10)
    {
        DbBusIICNotUseBus();
        DbBusNotStopMCU();
        DbBusExitSerialDebugMode();

        DrvPlatformLyrTouchDeviceResetHw();
        mdelay(300);
        
        DrvPlatformLyrEnableFingerTouchReport();

        return -2;
    }
    _DrvMpTestItoTestMcuStop();
    mdelay(10);

    for (i = 0; i < MAX_MUTUAL_NUM; i ++)
    {
        _gTestFailChannel[i] = 0;
    }	

    _gTestFailChannelCount = 0; // Reset _gTestFailChannelCount to 0 before test start

    _gSenseLineNum = _DrvMpTestItoTestAnaGetMutualChannelNum();
    _gDriveLineNum = _DrvMpTestItoTestAnaGetMutualSubFrameNum();

    _DrvMpTestItoTestAnaSwitchToMutual();
    _DrvMpTestItoOpenTestAnaGetMutualCSub(_gMode);
    _DrvMpTestItoTestDisableFilterNoiseDetect();
    _DrvMpTestItoTestAnaSwReset();
    _DrvMpTestItoTestGetDeltaC(_gDeltaC);
    
    for (i = 0; i < _gSenseLineNum; i ++)
    {
        DBG("\nSense[%02d]\t", i);
        
        for (j = 0; j < _gDriveLineNum; j ++)
        {
            _gResult[i * _gDriveLineNum + j] = (4464*_gMode[i * _gDriveLineNum + j] - _gDeltaC[i * _gDriveLineNum + j]);
//            DBG("%d\t", _gResult[i * _gDriveLineNum + j]);
            DBG("%d  %d  %d\t", _gResult[i * _gDriveLineNum + j], 4464*_gMode[i * _gDriveLineNum + j], _gDeltaC[i * _gDriveLineNum + j]);
        }
    }
    
    DBG("\n\n\n");

//    for (j = 0; j < _gDriveLineNum; j ++)
    for (j = 0; j < (_gDriveLineNum-1); j ++)
    {
        for (i = 0; i < _gSenseLineNum; i ++)
        {
            if (_gResult[i * _gDriveLineNum + j] < FIR_THRESHOLD)
            {
                _gTestFailChannel[i * _gDriveLineNum + j] = 1;
                _gTestFailChannelCount ++; 
                nRetVal = -1;
                DBG("\nSense%d, Drive%d, MIN_Threshold = %d\t", i, j, _gResult[i * _gDriveLineNum + j]);
            }

            if (i > 0)
            {
                nDelta = _gResult[i * _gDriveLineNum + j] > nPrev ? (_gResult[i * _gDriveLineNum + j] - nPrev) : (nPrev - _gResult[i * _gDriveLineNum + j]);
                if (nDelta > nPrev*FIR_RATIO/100)
                {
                    if (0 == _gTestFailChannel[i * _gDriveLineNum + j]) // for avoid _gTestFailChannelCount to be added twice
                    {
                        _gTestFailChannel[i * _gDriveLineNum + j] = 1;
                        _gTestFailChannelCount ++; 
                    }
                    nRetVal = -1;
                    DBG("\nSense%d, Drive%d, MAX_Ratio = %d,%d\t", i, j, nDelta, nPrev);
                }
            }
            nPrev = _gResult[i * _gDriveLineNum + j];
        }
    }

    DbBusIICNotUseBus();
    DbBusNotStopMCU();
    DbBusExitSerialDebugMode();
    
    DrvPlatformLyrTouchDeviceResetHw();
    mdelay(300);

    DrvPlatformLyrEnableFingerTouchReport();

    return nRetVal;
}

static void _DrvMpTestItoTestSendDataIn(u16 nAddr, u16 nLength, u16 *data)
{
    u8 szDbBusTxData[256] = {0};
    int i = 0;

    szDbBusTxData[0] = 0x10;
    szDbBusTxData[1] = (nAddr >> 8) & 0xFF;
    szDbBusTxData[2] = (nAddr & 0xFF);

    for (i = 0; i <= nLength ; i ++)
    {
        szDbBusTxData[3+2*i] = (data[i] & 0xFF);
        szDbBusTxData[4+2*i] = (data[i] >> 8) & 0xFF;
    }

    IicWriteData(SLAVE_I2C_ID_DBBUS, &szDbBusTxData[0], 3+nLength*2);
}

static void _DrvMpTestItoShortTestSetPAD2GPO(u8 nItemID)
{
    u16 gpioSetting[MAX_CHANNEL_NUM] = {0};
    u16 gpioEnabling[MAX_CHANNEL_NUM] = {0};
    u16 gpioZero[MAX_CHANNEL_NUM] = {0};
    u8 	gpioNum = 0;
    u16 *gpioPIN = NULL;
    int i = 0;
    int j = 0;

    DBG("*** %s() ***\n", __func__);

    if (nItemID == 1)
    {
        gpioNum = SHORT_N1_GPO_NUMBER_X;
        gpioPIN = kzalloc(sizeof(u16) * gpioNum, GFP_KERNEL);

        for (i = 0; i <	gpioNum; i++)
        {
            gpioPIN[i] = SHORT_N1_GPO_PIN_X[i];
        }
    }
    else if (nItemID == 2)
    {
        gpioNum = SHORT_N2_GPO_NUMBER_X;
        gpioPIN = kzalloc(sizeof(u16) * gpioNum, GFP_KERNEL);
    
        for (i = 0; i <	gpioNum; i++)
        {
            gpioPIN[i] = SHORT_N2_GPO_PIN_X[i];
        }
    }
    else if (nItemID == 3)
    {
        gpioNum = SHORT_S1_GPO_NUMBER_X;
        gpioPIN = kzalloc(sizeof(u16) * gpioNum, GFP_KERNEL);
		
        for (i = 0; i <	gpioNum; i++)
        {
            gpioPIN[i] = SHORT_S1_GPO_PIN_X[i];
        }
    }
    else if (nItemID == 4)
    {
        gpioNum = SHORT_S2_GPO_NUMBER_X;
        gpioPIN = kzalloc(sizeof(u16) * gpioNum, GFP_KERNEL);
		
        for (i = 0; i <	gpioNum; i++)
        {
            gpioPIN[i] = SHORT_S2_GPO_PIN_X[i];
        }
    }
    DBG("ItemID %d, gpioNum %d",nItemID, gpioNum);

    for (i = 0; i < MAX_CHANNEL_NUM; i++)
    {
        gpioEnabling[i] = 0xFFFF;
    }

    for (i = 0; i < gpioNum; i++)
    {
        gpioSetting[gpioPIN[i] / 16] |= (u16)(1 << (gpioPIN[i] % 16));
        gpioEnabling[gpioPIN[i] / 16] &= (u16)(~(1 << (gpioPIN[i] % 16)));
    }

    ///cts sw overwrite
    {
        _DrvMpTestItoTestSendDataIn(0x1E66, gpioNum, &gpioSetting[0]);   ///who -> will be controlled
        _DrvMpTestItoTestSendDataIn(0x1E6C, gpioNum, &gpioEnabling[0]);   ///who -> enable sw overwrite
        _DrvMpTestItoTestSendDataIn(0x1E72, gpioNum, &gpioZero[0]);       ///who -> set2GPO
        _DrvMpTestItoTestSendDataIn(0x1E78, gpioNum, &gpioZero[0]);       ///who -> GPO2GND
    }

    for (j = 0; j < gpioNum; j++)
    {
        if (PIN_GUARD_RING == gpioPIN[j])
        {
            u16 u16RegData;
            u16RegData = RegGet16BitValue(0x1E12);
            u16RegData = ((u16RegData & 0xFFF9) | BIT0);
            RegSet16BitValue(0x1E12, u16RegData);
        }
    }

    kfree(gpioPIN);
}

static void _DrvMpTestItoShortTestChangeANASetting(u8 nItemID)
{
    u16 SHORT_MAP_ANA1[7] = {0};
    u16 SHORT_MAP_ANA2[1] = {0};
    u16 SHORT_MAP_ANA3[21] = {0};
    int i = 0;

    DBG("*** %s() ***\n", __func__);

    if (nItemID == 1)
    {
        for (i = 0; i <	7; i++)
        {
            SHORT_MAP_ANA1[i] = short_ANA1_N1_X[i];
        }
        
        for (i = 0; i < 21; i++)
        {
            SHORT_MAP_ANA3[i] = short_ANA3_N1_X[i];
        }

        SHORT_MAP_ANA2[0] = short_ANA2_N1_X[0];
    }
    else if (nItemID == 2)
    {
        for (i = 0; i <	7; i++)
        {
            SHORT_MAP_ANA1[i] = short_ANA1_N2_X[i];
        }
		
        for (i = 0; i < 21; i++)
        {
            SHORT_MAP_ANA3[i] = short_ANA3_N2_X[i];
        }

        SHORT_MAP_ANA2[0] = short_ANA2_N2_X[0];
    }
    else if (nItemID == 3)
    {
        for (i = 0; i <	7; i++)
        {
            SHORT_MAP_ANA1[i] = short_ANA1_S1_X[i];
        }
        
        for (i = 0; i < 21; i++)
        {
            SHORT_MAP_ANA3[i] = short_ANA3_S1_X[i];
        }

        SHORT_MAP_ANA2[0] = short_ANA2_S1_X[0];
    }
    else if (nItemID == 4)
    {
        for (i = 0; i <	7; i++)
        {
            SHORT_MAP_ANA1[i] = short_ANA1_S2_X[i];
        }
        
        for (i = 0; i < 21; i++)
        {
            SHORT_MAP_ANA3[i] = short_ANA3_S2_X[i];
        }

        SHORT_MAP_ANA2[0] = short_ANA2_S2_X[0];
    }

    ///change ANA setting
    {
        _DrvMpTestItoTestSendDataIn(0x1178, 7, &SHORT_MAP_ANA1[0]);		///ANA1_3C_42
        _DrvMpTestItoTestSendDataIn(0x1216, 1, &SHORT_MAP_ANA2[0]);   	///ANA2_0B
        _DrvMpTestItoTestSendDataIn(0x1006, 21, &SHORT_MAP_ANA3[0]);    ///ANA3_03_17
    }
}

static void _DrvMpTestItoShortTestAnaFixPrs(u16 nOption)
{
    u16 nTemp = 0;

    DBG("*** %s() ***\n", __func__);

    nTemp = RegGet16BitValue(0x1208); //bank:ana2, addr:h000a
    nTemp &= 0x00F1;
    nTemp |= (u16)((nOption << 1) & 0x000E);
    RegSet16BitValue(0x1208, nTemp);
}

static void _DrvMpTestItoShortTestSetNoiseSensorMode(u8 nEnable)
{
    DBG("*** %s() ***\n", __func__);

    if (nEnable)
    {
        RegSet16BitValueOn(0x110E, BIT11);
        RegSet16BitValueOff(0x1116, BIT2);
    }
    else
    {
        RegSet16BitValueOff(0x110E, BIT11);
    }
}

static void _DrvMpTestItoShortTestAndChangeCDtime(u16 nTime1, u16 nTime2)
{
    DBG("*** %s() ***\n", __func__);

    RegSet16BitValue(0x1224, nTime1);
    RegSet16BitValue(0x122A, nTime2);
}

static void _DrvMpTestItoShortTestMsg26xxm(u8 nItemID)
{
    int i;

    DBG("*** %s() ***\n", __func__);

    _DrvMpTestItoTestMcuStop();
    _DrvMpTestItoShortTestSetPAD2GPO(nItemID);
    _DrvMpTestItoShortTestChangeANASetting(nItemID);
    _DrvMpTestItoTestAnaSwitchToMutual();
    _DrvMpTestItoShortTestAnaFixPrs(7);
    _DrvMpTestItoTestDisableFilterNoiseDetect();
    _DrvMpTestItoShortTestSetNoiseSensorMode(1);
    _DrvMpTestItoShortTestAndChangeCDtime(SHORT_Charge_X, SHORT_Dump1_X);
    _DrvMpTestItoTestAnaSwReset();
    _DrvMpTestItoTestGetDeltaC(_gTempDeltaC);

    _DrvMpTestItoTestMcuStop();
    _DrvMpTestItoShortTestSetPAD2GPO(nItemID);
    _DrvMpTestItoShortTestChangeANASetting(nItemID);
    _DrvMpTestItoTestAnaSwitchToMutual();
    _DrvMpTestItoShortTestAnaFixPrs(7);
    _DrvMpTestItoTestDisableFilterNoiseDetect();
    _DrvMpTestItoShortTestSetNoiseSensorMode(1);
    _DrvMpTestItoShortTestAndChangeCDtime(SHORT_Charge_X, SHORT_Dump2_X);
    _DrvMpTestItoTestAnaSwReset();
    _DrvMpTestItoTestGetDeltaC(_gDeltaC);

    for (i = 0; i < MAX_MUTUAL_NUM ; i++)
    {
        if ((_gDeltaC[i] <= -(IIR_MAX)) || (_gTempDeltaC[i] <= -(IIR_MAX)) || (_gDeltaC[i] >= (IIR_MAX)) || (_gTempDeltaC[i] >= (IIR_MAX)))
        {
            _gDeltaC[i] = 0x7FFF;
        }
        else
        {
            _gDeltaC[i] = abs(_gDeltaC[i] - _gTempDeltaC[i]);
        }
        //DBG("ItemID%d, MUTUAL_NUM %d, _gDeltaC = %d\t", nItemID, i, _gDeltaC[i]);
    }
    DBG("\n");
}

static u8 _DrvMpTestItoShortTestCheckValueInRange(s32 nValue, s32 nMax, s32 nMin)
{
   	if (nValue <= nMax && nValue >= nMin)
   	{
   	   	return 1;
   	}
   	else
   	{
   	   	return 0;
   	}
}

static s32 _DrvMpTestItoShortTestCovertRValue(s32 nValue)
{
   	if (nValue == 0)
   	{
   	   	nValue = 1;
   	}   	   	   	

   	if (nValue >= IIR_MAX)
   	{
   	   	return 0;
   	}

   	return ((500*11398) / (nValue));
}

static ItoTestResult_e _DrvMpTestItoShortTestMsg26xxmJudge(u8 nItemID)
{
   	ItoTestResult_e nRetVal = ITO_TEST_OK;
   	u8 nTestPinLength = 0;
   	u16 i = 0;
   	u8 nGpioNum = 0;
   	u8* nTestGpio = NULL;

    DBG("*** %s() ***\n", __func__);

   	if (nItemID == 1)
   	{
   	   	nGpioNum = SHORT_N1_TEST_NUMBER_X;
   	   	nTestGpio = kzalloc(sizeof(u16) * nGpioNum, GFP_KERNEL);
   	   	
   	   	for (i = 0; i <	nGpioNum; i++)
   	   	{
   	   	   	nTestGpio[i] = SHORT_N1_TEST_PIN_X[i];
   	   	}
   	}
   	else if (nItemID == 2)
   	{
   	   	nGpioNum = SHORT_N2_TEST_NUMBER_X;
   	   	nTestGpio = kzalloc(sizeof(u16) * nGpioNum, GFP_KERNEL);
   	   	
   	   	for (i = 0; i <	nGpioNum; i++)
   	   	{
   	   	   	nTestGpio[i] = SHORT_N2_TEST_PIN_X[i];
   	   	}
   	}
   	else if (nItemID == 3)
   	{
   	   	nGpioNum = SHORT_S1_TEST_NUMBER_X;
   	   	nTestGpio = kzalloc(sizeof(u16) * nGpioNum, GFP_KERNEL);
   	   	
   	   	for (i = 0; i <	nGpioNum; i++)
   	   	{
   	   	   	nTestGpio[i] = SHORT_S1_TEST_PIN_X[i];
   	   	}
   	}
   	else if (nItemID == 4)
   	{
   	   	nGpioNum = SHORT_S2_TEST_NUMBER_X;
   	   	nTestGpio = kzalloc(sizeof(u16) * nGpioNum, GFP_KERNEL);
   	   	
   	   	for (i = 0; i <	nGpioNum; i++)
   	   	{
   	   	   	nTestGpio[i] = SHORT_S2_TEST_PIN_X[i];
   	   	}
   	}

   	nTestPinLength = nGpioNum;

   	for (i = 0;i < nTestPinLength; i++)
   	{
   	   	_gShortTestChannel[i] = nTestGpio[i];

   	   	if (0 == _DrvMpTestItoShortTestCheckValueInRange(_gDeltaC[i], SHORT_VALUE, -SHORT_VALUE))
   	   	{
   	   	   	nRetVal = ITO_TEST_FAIL;
   	   	   	_gTestFailChannelCount++;
   	   	   	DBG("_gShortTestChannel i = %d, _gDeltaC = %d\t", i, _gDeltaC[i]);
   	   	}
   	}
   	kfree(nTestGpio);

   	return nRetVal;
}

static ItoTestResult_e _DrvMpTestItoShortTest(void)
{
    ItoTestResult_e nRetVal1 = ITO_TEST_OK, nRetVal2 = ITO_TEST_OK, nRetVal3 = ITO_TEST_OK, nRetVal4 = ITO_TEST_OK, nRetVal5 = ITO_TEST_OK;
    u32 i = 0;
    u32 j = 0;
    u16 nTestPinCount = 0;
    s32 nShortThreshold = 0;

    DBG("*** %s() ***\n", __func__);

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_ENABLE_DMA_IIC
    DmaReset();
#endif //CONFIG_ENABLE_DMA_IIC
#endif //CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM

    DrvPlatformLyrDisableFingerTouchReport();
    DrvPlatformLyrTouchDeviceResetHw();

    DbBusEnterSerialDebugMode();
    DbBusStopMCU();
    DbBusIICUseBus();
    DbBusIICReshape();
    mdelay(100);

    for (i = 0; i < MAX_MUTUAL_NUM; i ++)
    {
        _gShortTestChannel[i] = 0;
        _gDeltaC[i] = 0;
    }

    _gSenseLineNum = _DrvMpTestItoTestAnaGetMutualChannelNum();
    _gDriveLineNum = _DrvMpTestItoTestAnaGetMutualSubFrameNum();

    for (i = 0; i < MAX_CHANNEL_NUM; i ++)
    {
        _gShortTestChannel[i] = 0xff;
    }

    _gTestFailChannelCount = 0;

    nTestPinCount = 0; // Reset nTestPinCount to 0 before test start

    //N1_ShortTest

    if (g_ChipType == CHIP_TYPE_MSG26XXM)
    {
        _DrvMpTestItoShortTestMsg26xxm(1); 
    }

    nRetVal2 = _DrvMpTestItoShortTestMsg26xxmJudge(1);

    for (i = 0; i < MAX_CHANNEL_NUM; i++)
    {
        if (_gShortTestChannel[i] != 0)
        {
            nTestPinCount++;
        }
    }

    for (i = 0; i < nTestPinCount; i++)
    {
        for (j = 0; j < NUM_OF_TOTAL_SENSORS_X; j++)
        {
            if (_gShortTestChannel[i] == SENSE_X[j])
            {
                _gSenseR[j] = _DrvMpTestItoShortTestCovertRValue(_gDeltaC[i]);

                DBG("_gSenseR[%d] = %d\t", j , _gSenseR[j]);
            }
        }
    }
    DBG("\n");

    //clear
    for (i = 0; i < MAX_CHANNEL_NUM; i ++)
    {
        _gShortTestChannel[i] = 0xff;
    }

    nTestPinCount = 0;

    //N2_ShortTest

    if (g_ChipType == CHIP_TYPE_MSG26XXM)
    {
        _DrvMpTestItoShortTestMsg26xxm(2);
    }

    nRetVal3 = _DrvMpTestItoShortTestMsg26xxmJudge(2);

    for (i = 0; i < MAX_CHANNEL_NUM; i++)
    {
        if (_gShortTestChannel[i] != 0)
        {
            nTestPinCount++;
        }            
    }

    for (i = 0; i < nTestPinCount; i++)
    {
        for (j = 0; j < NUM_OF_TOTAL_SENSORS_X; j++)
        {
            if (_gShortTestChannel[i] == SENSE_X[j])
            {
                _gSenseR[j] = _DrvMpTestItoShortTestCovertRValue(_gDeltaC[i]);

                DBG("_gSenseR[%d] = %d\t", j , _gSenseR[j]);
            }
        }
    }
    DBG("\n");

    for (i = 0; i < MAX_CHANNEL_NUM; i ++)
    {
        _gShortTestChannel[i] = 0xff;
    }
    nTestPinCount = 0;

    if (g_ChipType == CHIP_TYPE_MSG26XXM)
    {
        _DrvMpTestItoShortTestMsg26xxm(3);
    }
    
    nRetVal4 = _DrvMpTestItoShortTestMsg26xxmJudge(3);

    for (i = 0; i < MAX_CHANNEL_NUM; i++)
    {
        if (_gShortTestChannel[i] != 0)
        {
            nTestPinCount++;
        }
    }

    for (i = 0; i < nTestPinCount; i++)
    {
        for (j = 0; j < NUM_OF_TOTAL_DRIVERS_X - 1; j++)
        {
            if (_gShortTestChannel[i] == DRIVE_X[j])
            {
                _gDriveR[j] = _DrvMpTestItoShortTestCovertRValue(_gDeltaC[i]);
                DBG("_gDriveR[%d] = %d\t", j , _gDriveR[j]);
            }
        }
    }
    DBG("\n");
    
    for (i = 0; i < MAX_CHANNEL_NUM; i ++)
    {
        _gShortTestChannel[i] = 0xff;
    }
    nTestPinCount = 0;

    if (g_ChipType == CHIP_TYPE_MSG26XXM)
    {
        _DrvMpTestItoShortTestMsg26xxm(4);
    }
    
    nRetVal4 = _DrvMpTestItoShortTestMsg26xxmJudge(4);

    for (i = 0; i < MAX_CHANNEL_NUM; i++)
    {
        if (_gShortTestChannel[i] != 0)
        {
            nTestPinCount++;
        }
    }

    for (i = 0; i < nTestPinCount; i++)
    {
        for (j = 0; j < NUM_OF_TOTAL_DRIVERS_X - 1 ; j++)
        {
            if (_gShortTestChannel[i] == DRIVE_X[j])
            {
                _gDriveR[j] = _DrvMpTestItoShortTestCovertRValue(_gDeltaC[i]);
                DBG("_gDriveR[%d] = %d\t", j , _gDriveR[j]);
            }
        }
    }
    DBG("\n");
    
    for (i = 0; i < MAX_CHANNEL_NUM; i ++)
    {
        _gShortTestChannel[i] = 0xff;
    }
    nTestPinCount = 0;
    nShortThreshold = _DrvMpTestItoShortTestCovertRValue(SHORT_VALUE);

    for (i = 0; i < _gSenseLineNum; i++)
    {
        _gResult[i] = _gSenseR[i];
    }

    for (i = 0; i < _gDriveLineNum - 1; i++)
    {
        _gResult[i + _gSenseLineNum] = _gDriveR[i];
    }

    for (i = 0; i < (_gSenseLineNum + _gDriveLineNum - 1); i++)
    {
        if (_gResult[i] < nShortThreshold)
        {
            _gTestFailChannel[i] = 1;
        }
        else
        {
            _gTestFailChannel[i] = 0;
        }
    }

    DbBusIICNotUseBus();
    DbBusNotStopMCU();
    DbBusExitSerialDebugMode();

    DrvPlatformLyrTouchDeviceResetHw();
    mdelay(300);

    DrvPlatformLyrEnableFingerTouchReport();

    DBG("short test end\n");

    DBG("nRetVal1 = %d, nRetVal2 = %d, nRetVal3 = %d, nRetVal4 = %d, nRetVal5 = %d\n",nRetVal1,nRetVal2,nRetVal3,nRetVal4,nRetVal5);

    if ((nRetVal1 != ITO_TEST_OK) && (nRetVal2 == ITO_TEST_OK) && (nRetVal3 == ITO_TEST_OK) && (nRetVal4 == ITO_TEST_OK) && (nRetVal5 == ITO_TEST_OK))
    {
        return ITO_TEST_GET_TP_TYPE_ERROR;
    }
    else if ((nRetVal1 == ITO_TEST_OK) && ((nRetVal2 != ITO_TEST_OK) || (nRetVal3 != ITO_TEST_OK) || (nRetVal4 != ITO_TEST_OK) || (nRetVal5 != ITO_TEST_OK)))
    {
        return -1;
    }
    else
    {
        return ITO_TEST_OK;
    }
}

static void _DrvMpTestItoTestDoWork(struct work_struct *pWork)
{
    s32 nRetVal = 0;
    
    DBG("*** %s() _gIsInMpTest = %d, _gTestRetryCount = %d ***\n", __func__, _gIsInMpTest, _gTestRetryCount);

    if (_gItoTestMode == ITO_TEST_MODE_OPEN_TEST)
    {
        nRetVal = _DrvMpTestItoOpenTest();
    }
    else if (_gItoTestMode == ITO_TEST_MODE_SHORT_TEST)
    {
        nRetVal = _DrvMpTestItoShortTest();
    }
    else
    {
        DBG("*** Undefined Mp Test Mode = %d ***\n", _gItoTestMode);
        return;
    }

    DBG("*** ctp mp test result = %d ***\n", nRetVal);
    
    if (nRetVal == 0)
    {
        _gCtpMpTestStatus = ITO_TEST_OK; //PASS
        _gIsInMpTest = 0;
        DBG("mp test success\n");

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
//        DrvFwCtrlRestoreFirmwareModeToLogDataMode();    
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG
    }
    else
    {
        _gTestRetryCount --;
        if (_gTestRetryCount > 0)
        {
            DBG("_gTestRetryCount = %d\n", _gTestRetryCount);
            queue_work(_gCtpMpTestWorkQueue, &_gCtpItoTestWork);
        }
        else
        {
            if (nRetVal == -1)
            {
                _gCtpMpTestStatus = ITO_TEST_FAIL;
            }
            else
            {
                _gCtpMpTestStatus = ITO_TEST_UNDEFINED_ERROR;
            }

            _gIsInMpTest = 0;
            DBG("mp test failed\n");

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
//            DrvFwCtrlRestoreFirmwareModeToLogDataMode();    
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG
        }
    }
}

/*=============================================================*/
// GLOBAL FUNCTION DEFINITION
/*=============================================================*/

s32 DrvMpTestGetTestResult(void)
{
    DBG("*** %s() ***\n", __func__);
    DBG("_gCtpMpTestStatus = %d\n", _gCtpMpTestStatus);

    return _gCtpMpTestStatus;
}

void DrvMpTestGetTestFailChannel(ItoTestMode_e eItoTestMode, u8 *pFailChannel, u32 *pFailChannelCount)
{
    u32 i;
    
    DBG("*** %s() ***\n", __func__);
    DBG("_gTestFailChannelCount = %d\n", _gTestFailChannelCount);
    
    for (i = 0; i < MAX_MUTUAL_NUM; i ++)
    {
    	  pFailChannel[i] = _gTestFailChannel[i];
    }
    
    *pFailChannelCount = MAX_MUTUAL_NUM; // Return the test result of all channels, APK will filter out the fail channels.
}

void DrvMpTestGetTestDataLog(ItoTestMode_e eItoTestMode, u8 *pDataLog, u32 *pLength)
{
    u32 i, j, k;
    
    DBG("*** %s() ***\n", __func__);

    if (eItoTestMode == ITO_TEST_MODE_OPEN_TEST)
    {
        k = 0;
        
//        for (j = 0; j < _gDriveLineNum; j ++)
        for (j = 0; j < (_gDriveLineNum-1); j ++)
        {
            for (i = 0; i < _gSenseLineNum; i ++)
            {
//                DBG("\nDrive%d, Sense%d, Value = %d\t", j, i, _gResult[i * _gDriveLineNum + j]); // add for debug

                if (_gResult[i * _gDriveLineNum + j] >= 0)
                {
                    pDataLog[k*5] = 0; // + : a positive number
                }
                else
                {
                    pDataLog[k*5] = 1; // - : a negative number
                }

                pDataLog[k*5+1] = (_gResult[i * _gDriveLineNum + j] >> 24) & 0xFF;
                pDataLog[k*5+2] = (_gResult[i * _gDriveLineNum + j] >> 16) & 0xFF;
                pDataLog[k*5+3] = (_gResult[i * _gDriveLineNum + j] >> 8) & 0xFF;
                pDataLog[k*5+4] = (_gResult[i * _gDriveLineNum + j]) & 0xFF;
                
                k ++;
            }
        }

        DBG("\nk = %d\n", k);

        *pLength = k*5;
    }
    else if (eItoTestMode == ITO_TEST_MODE_SHORT_TEST)
    {
        k = 0;
        
        for (i = 0; i < (_gDriveLineNum-1 + _gSenseLineNum); i++)
        {
            if (_gResult[i] >= 0)
            {
                pDataLog[k*5] = 0; // + : a positive number
            }
            else
            {
                pDataLog[k*5] = 1; // - : a negative number
            }

            pDataLog[k*5+1] = (_gResult[i] >> 24) & 0xFF;
            pDataLog[k*5+2] = (_gResult[i] >> 16) & 0xFF;
            pDataLog[k*5+3] = (_gResult[i] >> 8) & 0xFF;
            pDataLog[k*5+4] = (_gResult[i]) & 0xFF;
            k ++;
        }

        DBG("\nk = %d\n", k);

        *pLength = k*5;
    }
    else 
    {
        DBG("*** Undefined MP Test Mode ***\n");
    }
}

void DrvMpTestGetTestScope(TestScopeInfo_t *pInfo)
{
    DBG("*** %s() ***\n", __func__);

    pInfo->nMy = _gDriveLineNum;
    pInfo->nMx = _gSenseLineNum;

    DBG("*** My = %d ***\n", pInfo->nMy);
    DBG("*** Mx = %d ***\n", pInfo->nMx);
}

void DrvMpTestScheduleMpTestWork(ItoTestMode_e eItoTestMode)
{
    DBG("*** %s() ***\n", __func__);

    if (_gIsInMpTest == 0)
    {
        DBG("ctp mp test start\n");
        
        _gItoTestMode = eItoTestMode;
        _gIsInMpTest = 1;
        _gTestRetryCount = CTP_MP_TEST_RETRY_COUNT;
        _gCtpMpTestStatus = ITO_TEST_UNDER_TESTING;
        
        queue_work(_gCtpMpTestWorkQueue, &_gCtpItoTestWork);
    }
}

void DrvMpTestCreateMpTestWorkQueue(void)
{
    DBG("*** %s() ***\n", __func__);

    _gCtpMpTestWorkQueue = create_singlethread_workqueue("ctp_mp_test");
    INIT_WORK(&_gCtpItoTestWork, _DrvMpTestItoTestDoWork);
}

#endif //CONFIG_ENABLE_ITO_MP_TEST
#endif //CONFIG_ENABLE_TOUCH_DRIVER_FOR_MUTUAL_IC