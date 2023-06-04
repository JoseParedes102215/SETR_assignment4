#include <zephyr/kernel.h>          /* for k_msleep() */
#include <zephyr/device.h>          /* for device_is_ready() and device structure */
#include <zephyr/devicetree.h>      /* for DT_NODELABEL() */
#include <zephyr/drivers/gpio.h>    /* for GPIO api*/
#include <zephyr/sys/printk.h>      /* for printk()*/
#include <zephyr/drivers/i2c.h>

#ifndef I2C_H
#define I2C_H


#define I2C0_NODE DT_NODELABEL(temp_sensor)

static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C0_NODE);

#define READ_TEMP_COMMAND                        0x00
#define READ_WRITE_CONFIGURATION_COMMAND         0x01

#define NORMAL_MODE                              0x00

#define SLEEP_TIME_MS                            100



void config_i2c();



#endif