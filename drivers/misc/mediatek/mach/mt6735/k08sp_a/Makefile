
include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifneq ($(CONFIG_ARCH_MTK_PROJECT),)
ARCH_MTK_PROJECT := $(shell echo $(CONFIG_ARCH_MTK_PROJECT))
obj-y   += core/
obj-y   += dct/dct/
endif

ifeq ($(CONFIG_MTK_HDMI_SUPPORT),y)
obj-y	+=  hdmi/
endif

#obj-y   +=  lcm/
obj-y   +=  camera/camera/
obj-$(CONFIG_MTK_VIBRATOR)   +=  vibrator/
obj-$(CONFIG_MTK_LEDS)   +=  leds/
#obj-y   +=  keypad/
#obj-y   +=  sound/
obj-y   +=  accdet/
obj-y   +=  flashlight/

ifeq ($(CONFIG_MTK_SENSOR_SUPPORT),y)
ifeq ($(CONFIG_MTK_CM36652_NEW),y)
obj-y	+=  alsps/cm36652-new/
endif

ifeq ($(CONFIG_MTK_AP3XX6),y)
obj-y	+=  alsps/ap3xx6_mtk/
endif

ifeq ($(CONFIG_MTK_MC3410_NEW),y)
obj-y   +=  accelerometer/mc3410-new/
endif

ifeq ($(CONFIG_MTK_MC6470_ACC),y)
obj-y	+=  accelerometer/mc6470_acc/
endif

ifeq ($(CONFIG_MTK_BMA222E_NEW),y)
obj-y   +=  accelerometer/bma222E-new/
endif

ifeq ($(CONFIG_MTK_ITG1010_NEW),y)
obj-y   +=  gyroscope/ITG1010-new/
endif

ifeq ($(CONFIG_MTK_AKM09911_NEW),y)
obj-y	+=  magnetometer/akm09911-new/
endif

ifeq ($(CONFIG_MTK_MC64XX),y)
obj-y	+=  magnetometer/mc64xx/
endif

ifeq ($(CONFIG_MTK_MC6470),y)
obj-y	+=  magnetometer/mc6470/
endif
endif
