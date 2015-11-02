include $(srctree)/drivers/misc/mediatek/Makefile.custom


ccflags-y += -I$(MTK_PROJECT_PATH_ROOT)/touchpanel/synaptics_s2714/

# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/synaptics_s2714/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/synaptics_s2714/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/


ifeq ($(CONFIG_MTK_PLATFORM),"mt6582")
    ccflags-y += -DMT6582
endif
obj-y += synaptics_dsx_fw_update.o
obj-y += synaptics_dsx_i2c.o
obj-y += synaptics_dsx_rmi_dev.o


