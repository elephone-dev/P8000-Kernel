#include "tpd.h"
//#include "tpd_custom_fts.h"
#include "cust_gpio_usage.h"
#include <cust_eint.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/dma-mapping.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>

#include <alsps.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>

#include "mstar_drv_platform_interface.h"
#include "mstar_drv_common.h"


/*----------------------------------------------------------------------------*/
static int mstar_remove(void)
{
	
	return 0;
}
extern int register_mstar_alsps(void);
static int mstar_local_init(void) 
{
int ret;
	printk("11111111111111111 mstar_local_init\n");
	ret = register_mstar_alsps();
	if(ret != 0)
	{
		return ret;
	}
	return 0;
}
static struct alsps_init_info mstar_init_info = {
		.name = "mstar_alsps",
		.init = mstar_local_init,
		.uninit = mstar_remove,
	
};
/*----------------------------------------------------------------------------*/

static int __init mstar_init(void)
{
	alsps_driver_add(&mstar_init_info);
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit mstar_exit(void)
{
	
}


module_init(mstar_init);
module_exit(mstar_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("YC Hou");
MODULE_DESCRIPTION("fts driver");
MODULE_LICENSE("GPL");
