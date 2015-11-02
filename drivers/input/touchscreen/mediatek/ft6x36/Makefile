include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
#ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
#ccflags-y += -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/ft6x36

ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/ft6x36/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/ft6x36/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

obj-y	+=  focaltech_driver.o focaltech_ex_fun.o focaltech_ctl.o fts_alsps.o 
ifeq ($(ARCH),arm)
obj-y	+=  32bit/ft_gesture_lib.a
else
obj-y	+=  64bit/ft_gesture_lib.a
endif
