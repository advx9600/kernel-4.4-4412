#include <linux/platform_device.h>
#include <linux/leds.h>
#include "s3cfb.h"


#define MAX_LCD_COUNT  64
static struct s3cfb_lcd * s_lcd[MAX_LCD_COUNT];
static int s_lcd_count = 0;
char s_selected_lcd_name[32] = {'\0'};
EXPORT_SYMBOL(s_selected_lcd_name);


int rpregister_lcd(struct s3cfb_lcd *lcd)
{
	if(!lcd)
		return -1;

	if(s_lcd_count >= MAX_LCD_COUNT - 1){
		printk("rpregister_lcd: can not add lcd: %s, reach max count.\n", lcd->name);
		return -1;
	}

	s_lcd[s_lcd_count] = lcd;
	s_lcd_count++;
	printk("rpregister_lcd: add lcd: %s\n", lcd->name);	
	return 0;
}


static int __init select_lcd_name(char *str)
{
	printk("select_lcd_name: str=%s\n", str);
	strcpy(s_selected_lcd_name, str);
	return 0;
}

/* name should be fixed as 's3cfb_set_lcd_info' */
void s3cfb_set_lcd_info(struct s3cfb_global *ctrl)
{
	int i;
	ctrl->lcd = s_lcd[0];
	return;

}


__setup("lcd=", select_lcd_name);


//******************** lcd parameter begin ************************
static void rp_lcd_init(void)
{
	static int s_init = 0;
	printk("LCD lcd selecter init.");
	if(!s_init)
	{
			s_init = 1;
	}
}

static struct s3cfb_lcd s_rplcd_param[] = 
{
#if 0	
	{
		.name = "rp7",
		.init_ldi = rp_lcd_init,
		.width = 800,
		.height = 480,
		.bpp = 24/*24*/,
		.freq	= 60,

		.timing = {
			.h_fp = 50,
			.h_bp = 30,
			.h_sw = 3,
			.v_fp = 20,
			.v_fpe = 1,
			.v_bp = 10,
			.v_bpe = 1,
			.v_sw = 1,
		},

		.polarity = {
			.rise_vclk = 0,
			.inv_hsync = 1,
			.inv_vsync = 1,
			.inv_vden = 0,
		},
	},
	
#else	

#if 0	
	{
		.name = "rp9",
		.init_ldi = rp_lcd_init,
		.width = 1024,//800,
		.height = 768,//480,
		.bpp = 24/*24*/,
		.freq	= 50,

		.timing = {
			.h_fp = 60,
			.h_bp = 80,		
			.h_sw = 36,
			.v_fp = 16,
			.v_fpe = 1,
			.v_bp = 6,
			.v_bpe = 1,
			.v_sw = 3,
		},

		.polarity = {
			.rise_vclk =0,
			.inv_hsync = 1,
			.inv_vsync = 1,
			.inv_vden = 0,
		},
	},
#else //vga
/*
	{
		.name = "rp9",
		.init_ldi = rp_lcd_init,
		.width = 1024,
		.height = 768,
		.bpp = 24,
		.freq	= 50,

		.timing = {
			.h_fp = 320,
			.h_bp = 434,
			.h_sw = 36,
			.v_fp = 16,
			.v_fpe = 1,
			.v_bp = 6,
			.v_bpe = 1,
			.v_sw = 3,
		},
		
		.polarity = {
			.rise_vclk = 1,// 1
			.inv_hsync = 1,//1,
			.inv_vsync = 1,//1,
			.inv_vden = 0,
		},
	},
*/
			
	{
		.name = "rp9",
		.init_ldi = rp_lcd_init,
		.width = 1024,
		.height = 768,
		.bpp = 24,
		.freq	= 50,

		.timing = {
			.h_fp = 300,
			.h_bp = 434,
			.h_sw = 36,
			.v_fp = 16,
			.v_fpe = 1,
			.v_bp = 6,
			.v_bpe = 1,
			.v_sw = 3,
		},
		
		.polarity = {
			.rise_vclk = 1,// 1
			.inv_hsync = 1,//1,
			.inv_vsync = 1,//1,
			.inv_vden = 0,
		},
	},

#endif
#endif	

};

static int __init regiser_lcd(void)
{
	int i;
	for(i = 0; i < sizeof(s_rplcd_param) / sizeof(s_rplcd_param[0]); i++)
		rpregister_lcd(&(s_rplcd_param[i]));
	return 0;
}

early_initcall(regiser_lcd);

