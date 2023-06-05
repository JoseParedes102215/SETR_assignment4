/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

#include "i2c.h"


void config_i2c(){
		uint8_t ret;
		//printk("Hello World! %s\n", CONFIG_BOARD);
		if (!device_is_ready(dev_i2c.bus)) {
			printk("I2C bus %s is not ready!\n\r",dev_i2c.bus->name);
			return;
	}


		uint8_t config[2] = {READ_WRITE_CONFIGURATION_COMMAND,0*BIT(7)}; // Deveria selecionar o modo de operação
			ret = i2c_write_dt(&dev_i2c, config, sizeof(config));
			if(ret != 0){
				printk("Failed to write to I2C device address %x at Reg. %x \n", dev_i2c.addr,config[0]);
				return;
			}

		uint8_t data;
		uint8_t temp;

		uint8_t read_temp = READ_TEMP_COMMAND; // Deveria selecionar o modo de operação
			ret = i2c_write_dt(&dev_i2c, config, sizeof(config));
				if(ret != 0){
					printk("Failed to write to I2C device address %x at Reg. %x \n", dev_i2c.addr,config[0]);
					return;
				}
			ret = i2c_read_dt(&dev_i2c, &data, sizeof(data));
			if(ret != 0){
				printk("Failed to read from I2C device address %x at Reg. %x n", dev_i2c.addr,config[0]);
			}
			if(data == 64){
				ret = i2c_write_dt(&dev_i2c, read_temp, sizeof(read_temp));
				if(ret != 0){
					printk("Failed to write to I2C device address %x at Reg. %x \n", dev_i2c.addr,config[0]);
					return;
				}
			ret = i2c_read_dt(&dev_i2c, &temp, sizeof(temp));
			if(ret != 0){
				printk("Failed to read from I2C device address %x at Reg. %x n", dev_i2c.addr,config[0]);
			}
			
		}
		//printk("The data obtained is %u\r\n",data);
		printk("The temperature obtained is %u\r\n",temp);
		//k_msleep(SLEEP_TIME_MS);

	}
