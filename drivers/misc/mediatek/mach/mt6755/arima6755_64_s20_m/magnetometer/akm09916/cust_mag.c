#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_mag.h>

static struct mag_hw cust_mag_hw = {
    .i2c_num = 1,
    .direction = 7,
    .power_id = MT6351_POWER_LDO_VIO28,  /*!< LDO is not used */
    .power_vol= VOL_2800,        /*!< LDO is not used */
};
struct mag_hw* get_cust_mag_hw(void) 
{
    return &cust_mag_hw;
}
