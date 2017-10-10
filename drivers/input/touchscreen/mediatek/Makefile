subdir-ccflags-y += -Werror
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek
# In case the platform does NOT support this type of sensors
obj-y   +=  mtk_tpd.o
obj-y   +=  tpd_button.o
obj-y   +=  tpd_calibrate.o
obj-y   +=  tpd_debug.o
obj-y   +=  tpd_default.o
obj-y   +=  tpd_init.o
obj-y   +=  tpd_misc.o
obj-y   +=  tpd_setting.o

obj-$(CONFIG_TOUCHSCREEN_MTK_GT1151)	+=  GT1151/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT910)	+=  GT910/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT9XXTB_HOTKNOT)	+=  GT9XXTB_hotknot/
obj-$(CONFIG_TOUCHSCREEN_MTK_SYNAPTICS_I2C_RMI4)	+=  synaptics_i2c_rmi4/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT928)	+=  GT928/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT9XX_HOTKNOT_SCP)	+=  GT9XX_hotknot_scp/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT911)	+=  GT911/
obj-$(CONFIG_TOUCHSCREEN_MTK_FT5X0X)	+=  ft5x0x/
obj-$(CONFIG_TOUCHSCREEN_MTK_SYNAPTICS_3320_50) +=  synaptics_3320_50/
#//[SM20][zihweishen] Synaptics S2726 touch driver porting 20151014 begin
ifeq ($(CONFIG_TOUCHSCREEN_SYNAPTICS_S2726), y)
# obj-y   +=  synaptics_2726/
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_S2726) +=  synaptics_2726/
endif
#//[SM20][zihweishen] 20151014 end
#//[SM20][zihweishen] Porting Touch 2nd source is Truly LCM+Focal driver IC 20160620 begin
#//Steven start
obj-$(CONFIG_TOUCHSCREEN_ELAN_EKTF2140) +=  ektf2140/
#//Steven end
obj-$(CONFIG_TOUCHSCREEN_FT3427) +=  ft3427/
#//[SM20][zihweishen] 20160620 end