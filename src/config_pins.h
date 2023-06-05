#include <zephyr/kernel.h>          /* for k_msleep() */
#include <zephyr/device.h>          /* for device_is_ready() and device structure */
#include <zephyr/devicetree.h>      /* for DT_NODELABEL() */
#include <zephyr/drivers/gpio.h>    /* for GPIO api*/
#include <zephyr/sys/printk.h>      /* for printk()*/

#ifndef CONFIG_PINS_H
#define CONFIG_PINS_H

#include <stdint.h>
#define GPIO0_NODE DT_NODELABEL(gpio0)
static const struct device *gpio0_dev = DEVICE_DT_GET(GPIO0_NODE);


extern const uint8_t buttons_pins[];
extern const uint8_t led_pins[];

int config_pins();

#endif