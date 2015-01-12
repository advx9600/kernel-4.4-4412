#include <linux/interrupt.h>
#include <linux/i2c.h>
//#include <linux/i2c-id.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/synclink.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include "../../../fs/proc/internal.h"

#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#include <linux/earlysuspend.h>
#include <linux/suspend.h>
#endif

//#define DEBUG 1
#define DEBUG_SENSOR
#ifdef DEBUG_SENSOR
#define dbg(format, arg...) printk(KERN_ALERT format, ## arg)
#else
#define dbg(format, arg...)
#endif

#define MMA7660_ADDR	0x4C

#define MMA7660_DATA_MASK		0x3F
//#define MMA7660_DATA_MASK		0x3E



//parameters for default
#define MMA7660_CHANNEL_X		1// 0
#define MMA7660_CHANNEL_Y		0// 1
#define MMA7660_CHANNEL_Z		2
#define MMA7660_DIR_X			(1)
#define MMA7660_DIR_Y			(-1)
#define MMA7660_DIR_Z			(-1)



#define GSENSOR_PROC_NAME "mma7660"
static struct proc_dir_entry * s_proc = NULL;
static struct i2c_client * s_i2c_client = NULL;

static struct early_suspend s_mma7660_early_suspend;
static int s_mma7660_suspend = 0;

//static void mma7660_poll_callback(struct work_struct *work);
//static DECLARE_WORK(mma7660_poll_work, mma7660_poll_callback);
static int s_kthread_run = 0;

#define MIN_POLL_INTERVAL	70	//milliseconds


static int __devinit mma7660_i2c_probe(struct i2c_client * client, const struct i2c_device_id * id);
static int mma7660_i2c_suspend(struct i2c_client * client, pm_message_t mesg);
static int mma7660_i2c_resume(struct i2c_client * client);


static void mma7660_read_thread(int run);

static inline unsigned int ms_to_jiffies(unsigned int ms)
{
        unsigned int j;
        j = (ms * HZ + 500) / 1000;
        return (j > 0) ? j : 1;
}

static volatile int s_x = 0;
static volatile int s_y = 0;
static volatile int s_z = 0;



int xyz_g_table[64] = {
0,40,80,120,160,200,240,280,  // 40
320,360,400,440,480,520,560,606, //40
670,734,798,862,926,990,1054,1118, //64
1198,1278,1358,1438,1518,1598,1678,1758, // 80
-1838,-1758,-1678,-1598,-1518,-1438,-1358,-1278,
-1198,-1118,-1054,-990,-926,-862,-798,-734,
-670,-600,-560,-520,-480,-440,-400,-360,
-320,-280,-240,-200,-160,-120,-80,-40
};


#if 1 //xiugai rongpin dzkj
int xyz_degree_table[64][2] = {
	[0] = {0, 9000},
	[1] = {269,8731},
	[2] = {538,8462},
	[3] = {808,8192},
	[4] = {1081,7919},
	[5] = {1355,7645},
	[6] = {1633,7367},
	[7] = {1916,7084},
	[8] = {2202,6798},
	[9] = {2495,6505},
	[10] = {2795,6205},
	[11] = {3104,5896},
	[12] = {3423,5577},
	[13] = {3754,5246},
	[14] = {4101,4899},
	[15] = {4468,4532},
	[16] = {4859,4141},
	[17] = {5283,3717},
	[18] = {5754,3246},
	[19] = {6295,2705},
	[20] = {6964,2036},
	[21] = {7986,1014},

	[22] = {9000,0},
	[23] = {9000,0},
	[24] = {9000,0},
	[25] = {9000,0},
	[26] = {9000,0},
	[27] = {9000,0},
	[28] = {9000,0},
	[29] = {9000,0},
	[30] = {9000,0},
	[31] = {9000,0},
	[32] = {9000,0},
	[33] = {-9000,0},
	[34] = {-9000,0},
	[35] = {-9000,0},
	[36] = {-9000,0},
	[37] = {-9000,0},
	[38] = {-9000,0},
	[39] = {-9000,0},
	[40] = {-9000,0},
	[41] = {-9000,0},
	[42] = {-9000,0},
	[43] = {-7986,-1014},
	[44] = {-6964,-2036},
	[45] = {-6295,-2705},
	[46] = {-5754,-3246},
	[47] = {-5283,-3717},
	[48] = {-4859,-4141},
	[49] = {-4468,-4532},
	[50] = {-4101,-4899},
	[51] = {-3754,-5246},
	[52] = {-3423,-5577},
	[53] = {-3104,-5896},
	[54] = {-2795,-6205},
	[55] = {-2495,-6505},
	[56] = {-2202,-6798},
	[57] = {-1916,-7084},
	[58] = {-1633,-7367},
	[59] = {-1355,-7645},
	[60] = {-1081,-7919},
	[61] = {-808,-8192},
	[62] = {-538,-8462},
	[63] = {-269,-8731},
};
#endif


unsigned char mma7660_i2c_read(unsigned char reg)
{
	if(!s_i2c_client)
	{
		printk("s_i2c_client is not ready\n");
		return -1;
	}
	return i2c_smbus_read_byte_data(s_i2c_client, reg);
}

int mma7660_i2c_write(unsigned char reg, unsigned char data)
{
	if(!s_i2c_client)
	{
		printk("s_i2c_client is not ready\n");
		return -1;
	}
	return i2c_smbus_write_byte_data(s_i2c_client, reg, data);
}

static int mma7660_i2c_write_bytes(struct i2c_client *client,uint8_t *data,int len)
{
        struct i2c_msg msg;
        int ret=-1;

        msg.flags=!I2C_M_RD;
        msg.addr=client->addr;
        msg.len=len;
        msg.buf=data;

        ret=i2c_transfer(client->adapter,&msg,1);
        return ret;
}

static void mma7660_chip_init(void)
{
	mma7660_i2c_write(0x07, 0);	//set it standby to write.
	mma7660_i2c_write(0x05, 0);
	mma7660_i2c_write(0x06, 0);
	mma7660_i2c_write(0x08, 0xFB);
//	mma7660_i2c_write(0x08, 0xF8);
//	mma7660_i2c_write(0x09, 0x1F);
	mma7660_i2c_write(0x0A, 0xFF);
 	mma7660_i2c_write(0x07, 0xF9);	//write finished, resume.

	printk("rongpin mma7660_chip_init\n");
}




static struct i2c_device_id mma7660_idtable[] = {
	{"mma7660", 0},
};

static struct i2c_driver mma7660_i2c_driver = 
{
	.driver = 
	{
		.name = "mma7660_i2c",
		.owner = THIS_MODULE,
	},
	.id_table = mma7660_idtable,
	.probe = mma7660_i2c_probe,
#if 0  //mask mma7660 suspend 
	.suspend = mma7660_i2c_suspend,
	.resume = mma7660_i2c_resume,
#endif
};

static struct platform_device mma7660_platform_i2c_device = 
{
    .name           = "mma7660_i2c",
    .id             = -1,
};


static int __devinit mma7660_i2c_probe(struct i2c_client * client, const struct i2c_device_id * id)
{
	s_i2c_client = client;

	printk("rongpin mma7660: s_i2c_client->addr = 0x%x\n",s_i2c_client->addr);
	
	mma7660_chip_init();
	mma7660_read_thread(1);	
	
	return 0;
}


static int mma7660_open(struct inode *inode, struct file *file)
{
	mma7660_chip_init();
	return nonseekable_open(inode, file);
}

static int mma7660_release(struct inode *inode, struct file *file)
{
	return 0;
}


static int mma7660_poll_thread(void * data)
{
	int x, y, z;
	int x_buf[15], y_buf[15], z_buf[15], i, tmp; 
	while(1)
	{
		x = mma7660_i2c_read(MMA7660_CHANNEL_X);
		y = mma7660_i2c_read(MMA7660_CHANNEL_Y);
		z = mma7660_i2c_read(MMA7660_CHANNEL_Z) / 2;
		if(x < 0 || y < 0 || z < 0)
		{
			//printk("mma7660 read error, delay\n");
			//msleep(2000);
		}
		s_x = x & MMA7660_DATA_MASK;
		s_y = y & MMA7660_DATA_MASK;
		s_z = z & MMA7660_DATA_MASK;

//		printk("(%d,%d,%d)",s_x,s_y,s_z);
		msleep(80);
	}
	return 0;
}

static int mma7660_poll_date(int * x, int * y, int * z)
{
	*x = s_x;
	*y = s_y;
	*z = s_z;
	//flush_work(&mma7660_poll_work);
	//schedule_work(&mma7660_poll_work);
	return 0;
}

ssize_t mma7660_read(struct file *filp, char __user *buf,
			     size_t count, loff_t *ppos)
{
	int readBuf[3];
	int outBuf[8];
	mma7660_poll_date(&readBuf[0], &readBuf[1], &readBuf[2]);
	outBuf[0] = xyz_g_table[readBuf[0]] * MMA7660_DIR_X;
	outBuf[1] = xyz_g_table[readBuf[1]] * MMA7660_DIR_Y;
	outBuf[2] = xyz_g_table[readBuf[2]] * MMA7660_DIR_Z;
	outBuf[3] = 65536 + 1;
	outBuf[4] = xyz_degree_table[readBuf[0]][0] * MMA7660_DIR_X;
	outBuf[5] = xyz_degree_table[readBuf[1]][0] * MMA7660_DIR_Y;
	outBuf[6] = xyz_degree_table[readBuf[2]][0] * MMA7660_DIR_Z;
	outBuf[7] = 65536 + 1;
	if(copy_to_user(buf, outBuf, sizeof(outBuf)))
		return -EFAULT;
	return sizeof(outBuf);
}

static void mma7660_read_thread(int run)
{
	if(run && (!s_kthread_run)) 
	{	
		kthread_run(mma7660_poll_thread, s_i2c_client, "mma7660_poll");
		s_kthread_run = 1;
	}
}

static int mma7660_writeproc(struct file *file,const char *buffer,
                           unsigned long count, void *data)
{
	return count;
}

static int mma7660_readproc(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int len = 0;
	int x, y, z;
	mma7660_poll_date(&x, &y, &z);
	len = sprintf(page, "x=%d\ny=%d\nz=%d\nx_degree=%d\ny_degree=%d\nz_degree=%d\n", 
				  xyz_g_table[x] * MMA7660_DIR_X, 
				  xyz_g_table[y] * MMA7660_DIR_Y, 
				  xyz_g_table[z] * MMA7660_DIR_Z,
				  xyz_degree_table[x][0] * MMA7660_DIR_X,
				  xyz_degree_table[y][0] * MMA7660_DIR_Y,
				  xyz_degree_table[z][0] * MMA7660_DIR_Z);
	return len;
}

static struct file_operations mma7660_fops = {
	.owner = THIS_MODULE,
	.open = mma7660_open,
	.release = mma7660_release,
	.read = mma7660_read,
};

static struct miscdevice mma7660_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mma7660",
	.fops = &mma7660_fops,
};


void ut_mma7660_early_suspend(struct early_suspend *h)
{
	mma7660_i2c_write(0x07, 0);
	s_mma7660_suspend = 1;
}

void ut_mma7660_late_resume(struct early_suspend *h)
{
	mma7660_chip_init();
	s_mma7660_suspend = 0;
}

static void init_mma7660_early_suspend(void)
{
	s_mma7660_early_suspend.suspend = ut_mma7660_early_suspend;
	s_mma7660_early_suspend.resume = ut_mma7660_late_resume;
	s_mma7660_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN/* + 1*/;
	register_early_suspend(&s_mma7660_early_suspend);
}

static int __init mma7660_init(void)
{
    int ret;
	printk(KERN_INFO "mma7660 G-Sensor driver: init\n");

	ret = platform_device_register(&mma7660_platform_i2c_device);
    if(ret)
        return ret;

	s_proc = create_proc_entry(GSENSOR_PROC_NAME, 0666, &proc_root);
	if (s_proc != NULL)
	{
		s_proc->write_proc = mma7660_writeproc;
		s_proc->read_proc = mma7660_readproc;
	}

	ret = misc_register(&mma7660_device);
	if (ret) {
		printk("mma7660_probe: mma7660_device register failed\n");
		return ret;
	}

	init_mma7660_early_suspend();

	return i2c_add_driver(&mma7660_i2c_driver); 
	 
}

static void __exit mma7660_exit(void)
{
	if (s_proc != NULL)
		remove_proc_entry(GSENSOR_PROC_NAME, &proc_root);
	misc_deregister(&mma7660_device);
	i2c_del_driver(&mma7660_i2c_driver);
    platform_device_unregister(&mma7660_platform_i2c_device);
}

//module_init(mma7660_init);
late_initcall(mma7660_init);
module_exit(mma7660_exit);
MODULE_AUTHOR("rongpin dzkj inc.");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("rongpin mma7660 G-sensor");

