
#include $(srctree)/drivers/misc/mediatek/Makefile.custom
ccflags-y += -I$(srctree)/drivers/misc/mediatek/magnetometer/inc
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/mt6755/include
# Module project related part

ifeq ($(CONFIG_MTK_AKM09916),y)
obj-y	+=  akm09916/
endif
