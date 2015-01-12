#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/jiffies.h>
#include <linux/sched.h>

#include <plat/adc.h>
#include <linux/proc_fs.h>

#include <linux/delay.h>
#include <linux/time.h>
#include <linux/timer.h>

#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#include <linux/earlysuspend.h>
#endif


#include <plat/gpio-cfg.h>
#include <linux/gpio.h>
#include <mach/gpio.h>

#include <plat/adc.h>


#define BATTERY_ADC_VREF	    	1800 /*3300 */   //mv
#define BATTERY_ADC_12BIT_MAX    	0xFFF
#define BATTERY_ADC_CHANNEL	    	0
#define BATTERY_ADC_SAMPLE_COUNT	16

#define BATTERY_TIMER_INTERVAL		(2000)//(1000)	//ms

#define BATTERY_AC_INTERVAL	        BATTERY_TIMER_INTERVAL
#define BATTERY_ADC_INTERVAL	    (BATTERY_TIMER_INTERVAL * 2)  

static struct platform_device * battery_dev;


static struct s3c_adc_client	*client = NULL;


struct delayed_work battery_pen_event_work;
struct workqueue_struct *battery_workqueue;

static int s_adc_raw        = 0;
static int s_adc_mv         = 0;
static int s_adc_percent    = 100;


#define PROC_NAME "battery"
struct proc_dir_entry *root_entry;
struct proc_dir_entry *entry;


#define SAMPLE_NUM		16
static int ucTemp[SAMPLE_NUM];
static int  BatteryLifePercent =0;
static int ucIndex=0xff;

extern struct proc_dir_entry proc_root;

#define ADC_ERR_DELAY	200
#define ADC_ERR_CNT 5


struct mutex		rp_battery_mutex;


int s3c_adc_get_adc_data(int channel){
	int adc_value = 0;
	int retry_cnt = 0;

	if (IS_ERR(client)) {
		return -1;  
	}
	
	do {
		adc_value = s3c_adc_read(client, channel);
		if (adc_value < 0) {
			pr_info("%s: adc read(%d), retry(%d)", __func__, adc_value, retry_cnt++);
			msleep(ADC_ERR_DELAY);
		}
	} while (((adc_value < 0) && (retry_cnt <= ADC_ERR_CNT)));

	if(retry_cnt > ADC_ERR_CNT ) 
		return -1;

	return adc_value;
}
EXPORT_SYMBOL(s3c_adc_get_adc_data);


static void battery_detect(unsigned long data);
static struct timer_list battery_timer =
		TIMER_INITIALIZER(battery_detect, 0, 0);


static enum power_supply_property properties_ac[] =
{
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property properties_usb[] =
{
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property properties_battery[] = 
{
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static int get_property_ac(struct power_supply *supply,
                           enum power_supply_property property,
                           union power_supply_propval * value)
{
    switch (property)
    {
    case POWER_SUPPLY_PROP_ONLINE:
        value->intval = 0;
        break;
    default:
        break;
    }
    return 0;
}


static int get_property_battery(struct power_supply * supply,
                                enum power_supply_property property,
                                union power_supply_propval * value)
{

    int ret;
    ret	= 0;

    switch (property)
    {
    case POWER_SUPPLY_PROP_STATUS:
            value->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
        break;
    case POWER_SUPPLY_PROP_PRESENT:
        value->intval = 1;
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        value->intval = POWER_SUPPLY_HEALTH_GOOD;
        break;
    case POWER_SUPPLY_PROP_MANUFACTURER:
        value->strval = "rongpin.inc";
        break;
    case POWER_SUPPLY_PROP_TECHNOLOGY:
        value->intval = POWER_SUPPLY_TECHNOLOGY_LION;
        break;
    case POWER_SUPPLY_PROP_CHARGE_COUNTER:
        value->intval = 1024;
        break;
    case POWER_SUPPLY_PROP_TEMP:
        value->intval = 200;   //internal temperature 20.0 degree
        break;
    case POWER_SUPPLY_PROP_CAPACITY:
        value->intval = s_adc_percent;
//        value->intval = 80;
        break;
    case POWER_SUPPLY_PROP_SERIAL_NUMBER:
        value->strval = "20130726";
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        value->intval = s_adc_mv * 1000;	//needs uv, not mv.
        break;
    default:
        ret = -EINVAL;
        break;
    }
    return ret;
}

static struct power_supply supply_ac =
{
    .name = "ac",
    .type = POWER_SUPPLY_TYPE_MAINS,
    .properties = properties_ac,
    .num_properties = ARRAY_SIZE(properties_ac),
    .get_property = get_property_ac,
};

static struct power_supply supply_battery =
{
    .name = "battery",
    .properties = properties_battery,
    .num_properties = ARRAY_SIZE(properties_battery),
    .get_property = get_property_battery,
    .use_for_apm = 1,
};

static inline unsigned int ms_to_jiffies(unsigned int ms)
{
        unsigned int j;
        j = (ms * HZ + 500) / 1000;
        return (j > 0) ? j : 1;
}

struct cap_table 
{
	int vol;
	int cap;
};

// for wky387885P
static struct cap_table s_capTable[] =
{
    {3500, 0},
    {3570, 5},
    {3610, 9},
    {3645, 19}, 
    {3670, 28},
    {3715, 42},
    {3740, 52},
    {3820, 66},
    {3890, 77},
    {4000, 90},
    {4050, 100},
};


int mv_to_percent(int voltage,struct cap_table *table,int table_lenth)
{
   	 int i, percent, d_vol, d_cap, len;
		len = table_lenth;

	if (voltage <= table[0].vol)
	        percent = 0;
	if (voltage >= table[len - 1].vol)
	        percent = 100;
	else
    	{
	        for (i = len - 2; i > 0; i--)
	        {
	           	 if (voltage >= table[i].vol)
	               	 break;
	        }
    
        d_vol = table[i + 1].vol - table[i].vol;
        d_cap = table[i + 1].cap - table[i].cap;
        percent = table[i].cap + (voltage - table[i].vol) * d_cap / d_vol;
    	}
	if(percent < 0)
		percent = 0;
	if(percent > 100)
	        percent = 100;
	
	return percent;
}


static void poll_adc_raw(int channel)
{
   	 int count, sum, i, avg;
   	 int buffer[BATTERY_ADC_SAMPLE_COUNT];
	
	int percent,tab_lenth;

	static int timeout = 0;

	 sum = 0;
   	 count = 0;

	
   	 s3c_adc_get_adc_data(channel);	
   	 for(i = 0; i < BATTERY_ADC_SAMPLE_COUNT; i++)
   	 {
	        buffer[i] = s3c_adc_get_adc_data(channel);
			
			if(sum += buffer[i] != 0)
			{
		    		sum += buffer[i];
				count++;
			}
   	 }
	

	if (count == 0)
	{
		return;  
	}

	avg = sum / count;

    sum = 0;
    count = 0;
    for(i = 0; i < BATTERY_ADC_SAMPLE_COUNT; i++)
    {
	        if(abs(buffer[i] - avg) < 200)
	        {
	            sum += buffer[i];
	            count++;
	        }
    }

	if (count == 0)  {
		return;
	}
		
	s_adc_raw = sum / count;
	
	s_adc_mv = (s_adc_raw * BATTERY_ADC_VREF / BATTERY_ADC_12BIT_MAX) * 3;



	tab_lenth = sizeof(s_capTable) / sizeof(s_capTable[0]);
	percent = mv_to_percent(s_adc_mv,s_capTable,tab_lenth);

	s_adc_percent = percent;
	
#if 0
	if(ucIndex==0xff)
	{	
		for(i=0;i<SAMPLE_NUM;i++)
			ucTemp[i]=s_adc_percent;

		ucIndex = 0;

	}
	else
	{
		if(ucIndex>=SAMPLE_NUM)
			ucIndex=0;
		
		 ucTemp[ ucIndex++ ]=s_adc_percent;
      		 BatteryLifePercent=0;
			  
		for(i=0;i<SAMPLE_NUM;i++)
		{
			BatteryLifePercent = BatteryLifePercent+ucTemp[i] ;
		}
		s_adc_percent = (BatteryLifePercent / SAMPLE_NUM);

	}
#endif
	if (s_adc_percent < 20)
		s_adc_percent = 20;
	    
//	printk("s_adc_raw=%d s_adc_mv=%d s_adc_percent=%d\n", s_adc_raw, s_adc_mv, s_adc_percent);
}


static void battert_work(struct work_struct *work)
{
  	 static int count = BATTERY_ADC_INTERVAL / BATTERY_TIMER_INTERVAL;
   	 int online; 
	 int charging;

	poll_adc_raw(BATTERY_ADC_CHANNEL);	
     	power_supply_changed(&supply_battery);


}


static void battery_detect(unsigned long data)
{
	if (!delayed_work_pending(&battery_pen_event_work)) 
	{	
		queue_delayed_work(battery_workqueue, &battery_pen_event_work, ms_to_jiffies(50));
	}
	mod_timer(&battery_timer, jiffies + ms_to_jiffies(BATTERY_TIMER_INTERVAL));

}

static int battery_proc_write(struct file *file, const char *buffer, 
                           unsigned long count, void *data) 
{      
	printk("\n battery_proc_write\n");

	return 1; 
} 

static int battery_proc_read(char *page, char **start, off_t off, 
			  int count, int *eof, void *data) 
{
	int value;
	int i;
	int len = 0;
#if 0	
//	len = sprintf(page, "\n%s = \n", __func__);
	for (i=0; i<5; i++)  {
		value = s3c_adc_get_adc_data(0);   // to-do
//		len += sprintf(page+len, " %d,", value);
		udelay(10);
	}
	udelay(100);
	len += sprintf(page+len, "\n");

	for (i=0; i<5; i++)  {
		value = s3c_adc_get_adc_data(1);// to-do
		len += sprintf(page+len, " %d,", value);
		udelay(10);
	}
	len += sprintf(page+len, "}\n");
#endif	

	mutex_lock(&rp_battery_mutex);
	for (i=0; i<1; i++)  {
		value = s3c_adc_get_adc_data(0);   // to-do
		len += sprintf(page+len, " %d,", value);
		mdelay(10);
	}

	len = sprintf(page, "%d", value);
	
	mutex_unlock(&rp_battery_mutex);
	return len;
	return 0;
}


#ifdef CONFIG_HAS_EARLYSUSPEND
void battery_early_suspend(struct early_suspend *h)
{
	mod_timer(&battery_timer, jiffies + ms_to_jiffies(5000)); 
	cancel_delayed_work_sync(&battery_pen_event_work);

}

void battery_late_resume(struct early_suspend *h)
{
	ucIndex = 0xff;
	queue_delayed_work(battery_workqueue, &battery_pen_event_work, ms_to_jiffies(80));

	ucIndex = 0xff;
	mod_timer(&battery_timer, jiffies + ms_to_jiffies(80)); 
	ucIndex = 0xff;

}

static struct early_suspend battery_android_suspend = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1,
	.suspend = battery_early_suspend,
	.resume  = battery_late_resume,
};
#endif /* CONFIG_HAS_EARLYSUSPEND */

static int __init battery_init(void)
{
    int ret = 0; 
    
    
    printk("++ RPDZKJ Battery driver init.\n");
    battery_dev = platform_device_register_simple("battery", 0, NULL, 0);
    printk("++ RPDZKJ Battery driver init----.\n");



	client = s3c_adc_register(battery_dev, NULL, NULL, 0);
	if (IS_ERR(client)) {
		dev_err(&battery_dev->dev, "cannot register adc\n");
		return PTR_ERR(client);
	}
	printk("++ RPDZKJ Battery driver init.---1\n");

	platform_set_drvdata(battery_dev, client);
	
	printk("++ RPDZKJ Battery driver init.---1\n");


#if 1	
    if(0 != power_supply_register(&battery_dev->dev, &supply_ac))
    {
        power_supply_unregister(&supply_ac);
        platform_device_unregister(battery_dev);
        ret = -1;
    }
	printk("++ RPDZKJ Battery driver init.---1q\n");

    
    supply_battery.name = battery_dev->name;
    if(0 != power_supply_register(&battery_dev->dev, &supply_battery))
    {
        power_supply_unregister(&supply_battery);
        platform_device_unregister(battery_dev);
        ret = -1;
    }
	printk("++ RPDZKJ Battery driver init.---1\n");
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&battery_android_suspend);
#endif /* CONFIG_HAS_EARLYSUSPEND */

    printk("++ RPDZKJ Battery driver init.---2\n");
	
	root_entry = proc_mkdir(PROC_NAME, &proc_root);
	if(root_entry)
	{
		//try = create_proc_entry(s_nods[i], 0666, root_entry);
		entry = create_proc_entry("battery_info" ,0666, root_entry);
		if(entry)
		{
			entry->write_proc = battery_proc_write;
			entry->read_proc =  battery_proc_read;
			//try->data = (void*)s_nods[i];	
			entry->data = (void*)0;	
		}
	}

	
	INIT_DELAYED_WORK(&battery_pen_event_work, battert_work);


	battery_workqueue = create_singlethread_workqueue("battery");

	battery_detect(0);
	
	mutex_init(&rp_battery_mutex);
	
    return 0;
}

static void __exit battery_exit(void)
{
    //cancel_work_sync(&battery_pen_event_work);
    cancel_delayed_work_sync(&battery_pen_event_work);

    power_supply_unregister(&supply_battery);
    power_supply_unregister(&supply_ac);
    platform_device_unregister(battery_dev);
}

late_initcall(battery_init);
module_exit(battery_exit);

MODULE_AUTHOR("rpdzkj inc.");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("rpdzkj battery");


