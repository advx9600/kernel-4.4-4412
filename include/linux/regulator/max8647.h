/*
 * National Semiconductors MAX8647 PMIC chip client interface
 */

#ifndef __LINUX_REGULATOR_MAX8647_H
#define __LINUX_REGULATOR_MAX8647_H
#include <linux/regulator/machine.h>


#define MAX8647_DCDC1 	0
#define MAX8647_DCDC2 	1
#define MAX8647_DCDC3 	2
#define MAX8647_DCDC4 	3

#define MAX8647_LDO5    4
#define MAX8647_LDO6    5
#define MAX8647_LDO7    6
#define MAX8647_LDO8    7
#define MAX8647_LDO9  	8
#define MAX8647_LDO10  	9
#define MAX8647_LDO11  	10
#define MAX8647_LDO12  	11
#define MAX8647_LDO13  	12
#define MAX8647_DCDC  13


struct MAX8647_regulator_subdev {
	int id;
	struct regulator_init_data *initdata;
};

struct  MAX8647_platform_data {
	int num_regulators;
	struct  MAX8647_regulator_subdev *regulators;
};

#endif
