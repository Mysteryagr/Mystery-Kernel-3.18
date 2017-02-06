#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/proc_fs.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>

#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>
//#include <mach/upmu_common.h>
#include <mt-plat/mt_pwm.h>
#include <mt-plat/upmu_common.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
#include <linux/mutex.h>
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#endif



/******************************************************************************
 * Debug configuration
******************************************************************************/
// availible parameter
// ANDROID_LOG_ASSERT
// ANDROID_LOG_ERROR
// ANDROID_LOG_WARNING
// ANDROID_LOG_INFO
// ANDROID_LOG_DEBUG
// ANDROID_LOG_VERBOSE
#define TAG_NAME "leds_strobe.c"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_err(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_WARN(fmt, arg...)        pr_err(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_NOTICE(fmt, arg...)      pr_err(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)        pr_err(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_TRC_FUNC(f)              pr_err(TAG_NAME "<%s>\n", __FUNCTION__)
#define PK_TRC_VERBOSE(fmt, arg...) pr_err(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       pr_err(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)


#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
	#define PK_VER PK_TRC_VERBOSE
	#define PK_ERR PK_ERROR
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock); /* cotta-- SMP proection */

#pragma message("----------------------------------------------------------------------")

static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;
static u32 flash_init = 0;

static int g_duty=-1;
static int g_timeOutTimeMs=0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_strobeSem);
#else
static DECLARE_MUTEX(g_strobeSem);
#endif


#define STROBE_DEVICE_ID 0x60

static int flashCur[] = {0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
static struct work_struct workTimeOut;

/*****************************************************************************
Functions
*****************************************************************************/
//#define GPIO_ENF GPIO_CAMERA_FLASH_EN_PIN
//#define GPIO_ENT GPIO_CAMERA_FLASH_MODE_PIN

struct platform_device *flashPltFmDev = NULL;

struct pinctrl *flashctl = NULL;
struct pinctrl_state *flash_en_l = NULL;
struct pinctrl_state *flash_en_h = NULL;
struct pinctrl_state *flash_mode_en_l = NULL;
struct pinctrl_state *flash_mode_en_h = NULL;


int mtkflash_gpio_init(struct platform_device *pdev)
{
	int ret = 0;

	flashctl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(flashctl)) {
		PK_DBG("Cannot find flash pinctrl!");
		ret = PTR_ERR(flashctl);
	}
	/*Cam0 Power/Rst Ping initialization */
	flash_en_l = pinctrl_lookup_state(flashctl, "flash_en0");
	if (IS_ERR(flash_en_l)) {
		ret = PTR_ERR(flash_en_l);
		PK_DBG("%s : pinctrl err, flash_en0\n", __func__);
	}

	flash_en_h = pinctrl_lookup_state(flashctl, "flash_en1");
	if (IS_ERR(flash_en_h)) {
		ret = PTR_ERR(flash_en_h);
		PK_DBG("%s : pinctrl err, flash_en1\n", __func__);
	}


	flash_mode_en_l = pinctrl_lookup_state(flashctl, "flash_mode_en0");
	if (IS_ERR(flash_mode_en_l)) {
		ret = PTR_ERR(flash_mode_en_l);
		PK_DBG("%s : pinctrl err, flash_mode_en0\n", __func__);
	}

	flash_mode_en_h = pinctrl_lookup_state(flashctl, "flash_mode_en1");
	if (IS_ERR(flash_mode_en_h)) {
		ret = PTR_ERR(flash_mode_en_h);
		PK_DBG("%s : pinctrl err, flash_mode_en1\n", __func__);
	}

	return ret;
}

#if 0
static int flashlight_remove(struct platform_device *pdev)
{
	PK_DBG("flashlight_remove\n");
	flashPltFmDev = pdev;
	return 0;
}

static int flashlight_probe(struct platform_device *pdev)
{
	PK_DBG("flashlight_probe\n");
	flashPltFmDev = pdev;
	return 0;
}


#ifdef CONFIG_OF
static const struct of_device_id flashlight_of_match[] = {
	{.compatible = "mediatek,flashlight",},
	{},
};
#endif

static struct platform_driver flashlight_driver = {
	.probe	  = flashlight_probe,
	.remove	 = flashlight_remove,
	.driver = {

		.name  = "flashlight",
	#ifdef CONFIG_OF
		.of_match_table = flashlight_of_match,
		#endif
	}
};
#endif



    /*CAMERA-FLASH-EN */


extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
static void work_timeOutFunc(struct work_struct *data);











/* add by lisong start
--------------------------------------------
true flash(KTD265EJH-TR)
ENF:GPIO12
ENM(ENT):GPIO13
1)ENF-HIGH,ENM-LOW is flash-mode
2)ENF-LOW,ENM-HIGH is torch-mode
3)ENF-HIGH,ENM-HIGH is torch-mode
4)ENF-LOW,ENM-LOW is off

--------------------------------------------
fake flash
ENABLE_PIN:GPIO12
GPIO12-HIGH:on
GPIO12-LOW:off

add by lisong end */

#if 0
int FL_Enable(void)
{
	if(flashCur[g_duty]==0)
	{
		//mt_set_gpio_out(GPIO_ENT,GPIO_OUT_ONE);
		pinctrl_select_state(flashctl, flash_en_h);
//lisong 2014-3-31 [JWYY-22][to compatible true flash and fake flash]start        
//		mt_set_gpio_out(GPIO_ENF,GPIO_OUT_ZERO);
       // mt_set_gpio_out(GPIO_ENF,GPIO_OUT_ONE);
        pinctrl_select_state(flashctl, flash_mode_en_h);
//lisong 2014-3-31 [JWYY-22][to compatible true flash and fake flash]end
		PK_DBG("FL_Enable ,g_duty=%d,line=%d\n",g_duty,__LINE__);
	}
	else
	{
		pinctrl_select_state(flashctl, flash_en_h);
		pinctrl_select_state(flashctl, flash_mode_en_l);
		//mt_set_gpio_out(GPIO_ENT,GPIO_OUT_ZERO);
		//mt_set_gpio_out(GPIO_ENF,GPIO_OUT_ONE);
		
		PK_DBG("FL_Enable ,g_duty=%d,line=%d\n",g_duty,__LINE__);
	}

    return 0;
}

int FL_Disable(void)
{
	if(!flashctl){
	PK_DBG("flashctl is NULL!!!!!\n");
	return -1;	
	}
	if(!flash_en_l){
		PK_DBG("flash_en_l is NULL!!!!!\n");
		return -1;	
	}
	if(!flash_mode_en_l){
			PK_DBG("flash_mode_en_l is NULL!!!!!\n");
		return -1;	
	}
	//mt_set_gpio_out(GPIO_ENT,GPIO_OUT_ZERO);
	//mt_set_gpio_out(GPIO_ENF,GPIO_OUT_ZERO);
	pinctrl_select_state(flashctl, flash_en_l);
	pinctrl_select_state(flashctl, flash_mode_en_l);
	PK_DBG(" FL_Disable line=%d\n",__LINE__);
    return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
	g_duty=duty;
	PK_DBG(" FL_dim_duty line=%d\n",__LINE__);
    return 0;
}


int FL_Init(void)
{
	PK_DBG("FL_INit is in!!!!!!\n");
//		platform_driver_register(&flashlight_driver);
		flashPltFmDev = get_flashlight_platform_device();
		PK_DBG("find is ok!!!!!!!!!");
		if(!flashPltFmDev){
				PK_DBG("find flashlight node fail!!!!!!!");
				return -1;
		}
		PK_DBG("flashPltFmDev is not null!!!!!!!!!\n");
		if(mtkflash_gpio_init(flashPltFmDev)){
			PK_DBG("init flashlight GPIO fail!!!!!!!\n");
			return -1;
		}
		PK_DBG("before FL_disable!!!!!!");
		FL_Disable();
//	g_duty = 0;
//	FL_Enable();
		PK_DBG("after FL_disable!!!!!!");
	//if(mt_set_gpio_mode(GPIO_ENF,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
   // if(mt_set_gpio_dir(GPIO_ENF,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
    //if(mt_set_gpio_out(GPIO_ENF,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
    /*Init. to disable*/
    //if(mt_set_gpio_mode(GPIO_ENT,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
    //if(mt_set_gpio_dir(GPIO_ENT,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
    //if(mt_set_gpio_out(GPIO_ENT,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
		

    INIT_WORK(&workTimeOut, work_timeOutFunc);
//    driver_create_file(&flashlight_driver,)
		flash_init = 1;
    PK_DBG(" FL_Init line=%d\n",__LINE__);
    return 0;
}


int FL_Uninit(void)
{
	flash_init = 0;
	FL_Disable();
    return 0;
}

#endif

int FL_Enable(void)
{
#if 1
	if(g_duty==0)
	{
		pmic_set_register_value(PMIC_ISINK_CH0_EN,1);
		//pmic_set_register_value(PMIC_ISINK_CH1_EN,1);
		//pmic_set_register_value(PMIC_ISINK_CH2_EN,1);
		PK_DBG("Sub_FL_Enable ,g_duty=%d,line=%d\n",g_duty,__LINE__);
	}
	else
	{
		pmic_set_register_value(PMIC_ISINK_CH0_EN,1);
		pmic_set_register_value(PMIC_ISINK_CH1_EN,1);
		pmic_set_register_value(PMIC_ISINK_CH2_EN,1);
		pmic_set_register_value(PMIC_ISINK_CH3_EN,1);
		PK_DBG("Sub_FL_Enable ,g_duty=%d,line=%d\n",g_duty,__LINE__);
	}
#endif
#if 0
pmic_set_register_value(PMIC_ISINK_CH0_EN,1);
pmic_set_register_value(PMIC_ISINK_CH1_EN,1);
pmic_set_register_value(PMIC_ISINK_CH2_EN,0);
#endif
    return 0;
}

int FL_Disable(void)
{

pmic_set_register_value(PMIC_ISINK_CH0_EN,0);
pmic_set_register_value(PMIC_ISINK_CH1_EN,0);
pmic_set_register_value(PMIC_ISINK_CH2_EN,0);
pmic_set_register_value(PMIC_ISINK_CH3_EN,0);
	PK_DBG(" Sub_FL_Disable line=%d\n",__LINE__);
    return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
	g_duty=duty;
	PK_DBG(" Sub_FL_dim_duty line=%d\n",__LINE__);
    return 0;
}


int FL_Init(void)
{

	pmic_set_register_value(PMIC_RG_DRV_32K_CK_PDN,0x0); // Disable power down  
	pmic_set_register_value(PMIC_RG_ISINK0_CK_PDN,0);
	pmic_set_register_value(PMIC_RG_ISINK0_CK_SEL,0);
	pmic_set_register_value(PMIC_ISINK_CH0_MODE,0);
	pmic_set_register_value(PMIC_ISINK_CH0_STEP,5);//24mA 
	pmic_set_register_value(PMIC_RG_ISINK0_DOUBLE_EN,0x1);
	pmic_set_register_value(PMIC_ISINK_DIM0_DUTY,31);
	pmic_set_register_value(PMIC_ISINK_DIM0_FSEL,0);


	pmic_set_register_value(PMIC_RG_ISINK1_CK_PDN,0);
	pmic_set_register_value(PMIC_RG_ISINK1_CK_SEL,0);
	pmic_set_register_value(PMIC_ISINK_CH1_MODE,0);
	pmic_set_register_value(PMIC_ISINK_CH1_STEP,5);//24mA 
	pmic_set_register_value(PMIC_RG_ISINK1_DOUBLE_EN,0x1);
	pmic_set_register_value(PMIC_ISINK_DIM1_DUTY,31);
	pmic_set_register_value(PMIC_ISINK_DIM1_FSEL,0);

	pmic_set_register_value(PMIC_RG_ISINK2_CK_PDN,0);
	pmic_set_register_value(PMIC_RG_ISINK2_CK_SEL,0);
	pmic_set_register_value(PMIC_ISINK_CH2_MODE,0);
	pmic_set_register_value(PMIC_ISINK_CH2_STEP,5);//24mA 
	pmic_set_register_value(PMIC_RG_ISINK2_DOUBLE_EN,0x1);
	pmic_set_register_value(PMIC_ISINK_DIM2_DUTY,31);
	pmic_set_register_value(PMIC_ISINK_DIM2_FSEL,0);		
	
	pmic_set_register_value(PMIC_RG_ISINK3_CK_PDN,0);
	pmic_set_register_value(PMIC_RG_ISINK3_CK_SEL,0);
	pmic_set_register_value(PMIC_ISINK_CH3_MODE,0);
	pmic_set_register_value(PMIC_ISINK_CH3_STEP,5);//24mA 
	pmic_set_register_value(PMIC_RG_ISINK3_DOUBLE_EN,0x1);
	pmic_set_register_value(PMIC_ISINK_DIM3_DUTY,31);
	pmic_set_register_value(PMIC_ISINK_DIM3_FSEL,0);	
	

    INIT_WORK(&workTimeOut, work_timeOutFunc);
    PK_DBG(" Sub_FL_Init line=%d\n",__LINE__);
    return 0;
}


int FL_Uninit(void)
{
	FL_Disable();
    return 0;
}





/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
    FL_Disable();
    PK_DBG("ledTimeOut_callback\n");
    //printk(KERN_ALERT "work handler function./n");
}



enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut);
    return HRTIMER_NORESTART;
}
static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
	g_timeOutTimeMs=1000; //1s
	hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	g_timeOutTimer.function=ledTimeOutCallback;

}



static int constant_flashlight_ioctl(MUINT32 cmd, MUINT32 arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	PK_DBG("constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
    switch(cmd)
    {

		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",arg);
			g_timeOutTimeMs=arg;
		break;


    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("FLASHLIGHT_DUTY: %d\n",arg);
    		FL_dim_duty(arg);
    		break;


    	case FLASH_IOC_SET_STEP:
    		PK_DBG("FLASH_IOC_SET_STEP: %d\n",arg);

    		break;

    	case FLASH_IOC_SET_ONOFF :
    		PK_DBG("FLASHLIGHT_ONOFF: %d\n",arg);
    		if(arg==1)
    		{
				if(g_timeOutTimeMs!=0)
	            {
	            	ktime_t ktime;
					ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
					hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
	            }
    			FL_Enable();
    		}
    		else
    		{
    			FL_Disable();
				hrtimer_cancel( &g_timeOutTimer );
    		}
    		break;
		default :
    		PK_DBG(" No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
    int i4RetValue = 0;
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res)
	{
	    FL_Init();
		timerInit();
		
		
	
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


    if(strobe_Res)
    {
        PK_ERR(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }

		
    spin_unlock_irq(&g_strobeSMPLock);
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

    return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
    PK_DBG(" constant_flashlight_release\n");

    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock);

        strobe_Res = 0;
        strobe_Timeus = 0;

        /* LED On Status */
        g_strobe_On = FALSE;

        spin_unlock_irq(&g_strobeSMPLock);

    	FL_Uninit();
    }

    PK_DBG(" Done\n");

    return 0;

}


FLASHLIGHT_FUNCTION_STRUCT	constantFlashlightFunc=
{
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};




MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &constantFlashlightFunc;
    }
    return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

    return 0;
}

EXPORT_SYMBOL(strobe_VDIrq);



#if 1

static int flash_enable = 0;
static ssize_t flashlight_torch_enable_show(struct device_driver *ddp, char *buf)
{
	return snprintf(buf, 4, "%d\n", flash_enable);
}

static ssize_t flashlight_torch_enable_store(struct device_driver *ddp,
				      const char *buf, size_t count)
{
	PK_DBG("the buf is %s\n,count is %d\n",buf,count);
	if(*buf<'0'|| *buf > '9'){
		return count;
	}
	int value = (*buf) - '0';
	if (value == 1 )
		flash_enable = 1;
	else
		flash_enable = 0;
	g_duty = 0;
	if(flash_enable){
		if(!flash_init){
			PK_DBG("FL_INIT\n");
			FL_Init();	
		}
		FL_Enable();	
	}
	else{
		FL_Disable();
	}	
		
	return count;
}

static DRIVER_ATTR(flash_torch_en, 0644, flashlight_torch_enable_show, flashlight_torch_enable_store);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *flash_attr_list[] = {
	&driver_attr_flash_torch_en,
};
/*----------------------------------------------------------------------------*/
 int flash_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(flash_attr_list)/sizeof(flash_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, flash_attr_list[idx]);
		if (err) {
			PK_DBG("driver_create_file (%s) = %d\n", flash_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}
/*----------------------------------------------------------------------------*/
 int flash_delete_attr(struct device_driver *driver)
{
	int idx , err = 0;
	int num = (int)(sizeof(flash_attr_list)/sizeof(flash_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, flash_attr_list[idx]);

	return err;
}


#endif
