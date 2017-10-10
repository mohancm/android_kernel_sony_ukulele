#include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/

obj-y	+=  synaptics_dsx_i2c.o
obj-y	+=  synaptics_dsx_rmi_dev.o
obj-y	+=  synaptics_dsx_fw_update.o
# [Lavender][bozhi_lin] add touch f54 test report for read capacity raw data 20150331 begin
obj-y	+=  synaptics_dsx_test_reporting.o
# [Lavender][bozhi_lin] 20150331 end
