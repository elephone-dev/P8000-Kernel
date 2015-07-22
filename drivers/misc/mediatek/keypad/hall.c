#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include <linux/wakelock.h>



#ifdef MT6572
//#include <mach/mt_devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_pwm.h>
#include <mtk_kpd.h>	
#endif

#if 1//def MT6582
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/eint.h>
#include <mtk_kpd.h>	
#endif

#include <linux/input.h>

#ifdef GPIO_HALL_COVER_PIN

#ifdef MT6572
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt65xx_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, 
			unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
#endif

#ifdef MT6582
extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern void mt_eint_print_status(void);
#endif


#define HALL_NAME "cover"

/******************************************************************************
 * configuration
*******************************************************************************/

/*----------------------------------------------------------------------------*/
#define HALL_TAG                  "[HALL] "
#define HALL_FUN(f)               printk( HALL_TAG"%s\n", __FUNCTION__)
#define HALL_ERR(fmt, args...)    printk(KERN_ERR  HALL_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define HALL_LOG(fmt, args...)    printk( HALL_TAG fmt, ##args)
#define HALL_DBG(fmt, args...)    printk(HALL_TAG fmt, ##args)                 
/******************************************************************************
 * extern functions
*******************************************************************************/


/*----------------------------------------------------------------------------*/
struct hall_priv {
	struct input_dev	*idev;
	unsigned int		int_pin;
	unsigned int		sw_code;

	struct delayed_work  eint_work;
	u8 hall_status;
};

//static struct hall_priv *hall_obj = NULL;

static struct hall_priv *g_hall_ptr = NULL;

static struct workqueue_struct * hall_eint_workqueue = NULL;





/*-------------------------------------------------------------------------*/


static ssize_t hall_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int state;

	state = !!mt_get_gpio_in(GPIO_HALL_COVER_PIN);

	return sprintf(buf, "%d\n", state);
}


static DEVICE_ATTR(status, S_IRUGO,
		hall_status_show, NULL);

static struct attribute *hall_attributes[] = {
	&dev_attr_status.attr,
	NULL
};

static struct attribute_group hall_attribute_group = {
	.attrs = hall_attributes
};

int hall_read_status(u8 *data)
{
	int value;

	value = mt_get_gpio_in(GPIO_HALL_COVER_PIN);

	HALL_DBG("hall_read_status: %d \n", value);
 
 	*data = value;

	return 0;    
}



/*----------------------------------------------------------------------------*/
void hall_eint_func(void)
{
	struct hall_priv *priv = g_hall_ptr;
	HALL_LOG("hall_eint_func()!!!\n");
	if(!priv)
	{
		return;
	}
	
	//schedule_work(&obj->eint_work);
	schedule_delayed_work(&priv->eint_work,0);
}

/*----------------------------------------------------------------------------*/
static void hall_eint_work(struct work_struct *work)
{
	int err;
	
	HALL_LOG("hall_eint_work()\n");

		
	if((err = hall_read_status(&g_hall_ptr->hall_status)))
	{
		 HALL_ERR("hall_read_value: %d\n", err);
	}
	
	if (g_hall_ptr->hall_status == 1)
	{
		HALL_ERR("HALL OPEN!!! \n");
		mt_eint_set_polarity(CUST_EINT_HALL_COVER_NUM, MT_POLARITY_LOW);
	}
	else
	{
		HALL_ERR("HALL CLOSE!!! \n");
		mt_eint_set_polarity(CUST_EINT_HALL_COVER_NUM, MT_POLARITY_HIGH);
	}

	input_report_switch(g_hall_ptr->idev, g_hall_ptr->sw_code, g_hall_ptr->hall_status);
	input_sync(g_hall_ptr->idev);

	mt_eint_unmask(CUST_EINT_HALL_COVER_NUM);
}
/*----------------------------------------------------------------------------*/

int hall_setup_eint(void)
{
	/*configure to GPIO function, external interrupt*/
	mt_set_gpio_mode(GPIO_HALL_COVER_PIN, GPIO_HALL_COVER_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_HALL_COVER_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_HALL_COVER_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_HALL_COVER_PIN, GPIO_PULL_UP);
	
	HALL_LOG("hall_setup_eint 111\n");
	
	mt_eint_set_hw_debounce(CUST_EINT_HALL_COVER_NUM , CUST_EINT_HALL_COVER_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_HALL_COVER_NUM , CUST_EINT_HALL_COVER_TYPE, hall_eint_func, 0);
	mt_eint_unmask(CUST_EINT_HALL_COVER_NUM );  

  HALL_LOG("hall_setup_eint 222\n");

  return 0;
}

/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int hall_probe(struct platform_device *pdev) 
{
	int err = 0;
	struct hall_priv *priv;
	struct input_dev *input;
	char pyhs_str[50];
	
	HALL_FUN(); 
	priv = kzalloc(sizeof(struct hall_priv), GFP_KERNEL);
	input = input_allocate_device();

	if (!priv || !input) 
	{
		err = -ENOMEM;
		goto err_free_mem;
	}

	g_hall_ptr = priv;
	priv->idev = input;
	priv->int_pin = GPIO_HALL_COVER_PIN;
	priv->sw_code = 12;

	input->name = HALL_NAME;
	sprintf(pyhs_str, "%s/input0", input->name);
	input->phys = pyhs_str;
	input_set_capability(input, EV_SW, priv->sw_code);

	priv->hall_status = 0;


	hall_eint_workqueue = create_singlethread_workqueue("hall_eint");
	INIT_DELAYED_WORK(&priv->eint_work, hall_eint_work);
	
	err = input_register_device(priv->idev);
	if (err) {
		HALL_ERR("input_register_device: %d\n", err);
		goto err_free_mem;
	}
	input_set_drvdata(priv->idev, priv);
	platform_set_drvdata(pdev, priv);

	err = sysfs_create_group(&priv->idev->dev.kobj,
			&hall_attribute_group);
	if (err < 0)
	{
		HALL_ERR("sysfs_create_group: %d\n", err);
		goto error_sysfs;
	}

	if(err = hall_setup_eint())
	{
		HALL_ERR("setup eint: %d\n", err);
		goto error_sysfs;
	}
	
	HALL_LOG("%s: OK\n", __func__);    
	return 0;

error_sysfs:
	input_unregister_device(priv->idev);
err_free_mem:
	input_free_device(input);
	kfree(priv);
	HALL_LOG("%s: Fail\n", __func__);  
	return err;
}
/*----------------------------------------------------------------------------*/
static int hall_remove(struct platform_device *pdev)
{
	HALL_FUN(); 
	   
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct platform_device hall_device = {
    .name = "hall",
    .id   = 0,
};


static struct platform_driver hall_driver = {
	.probe      = hall_probe,
	.remove     = hall_remove,    
	.driver     = {
		.name  = "hall",
		.owner = THIS_MODULE,
	}
};

/*----------------------------------------------------------------------------*/
int hall_init(void)
{
	HALL_FUN();
	
	
	 if (platform_device_register(&hall_device))
   	 {	
        	return -ENODEV;
   	 }

	if(platform_driver_register(&hall_driver))
	{
		printk("failed to register driver");
		return -ENODEV;
	}
	return 0;
}

static __exit hall_exit(void)
{

	platform_driver_unregister(&hall_driver);
}

module_init(hall_init);
module_exit(hall_exit);

MODULE_DESCRIPTION("Prowave Hall driver");
MODULE_AUTHOR("Aka.Jiang <aka.jiang@hotmail.com>");
MODULE_LICENSE("GPL");

#endif /* GPIO_HALL_COVER_PIN */
