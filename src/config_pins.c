#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#include "config_pins.h"

#define GPIO0_NODE DT_NODELABEL(gpio0)
static const struct device *gpio0_dev = DEVICE_DT_GET(GPIO0_NODE);

const uint8_t buttons_pins[] = {11, 12, 24, 25};
const uint8_t led_pins[] = {13, 14, 15, 16};

int config_pins() {
    int ret, i;

    printk("Digital IO accessing IO pins configuration startup\n\r");

    if (!device_is_ready(gpio0_dev)) {
        printk("Error: gpio0 device is not ready\n");
        return -1;
    } else {
        printk("Success: gpio0 device is ready\n");
    }

    for (i = 0; i < sizeof(led_pins); i++) {
        ret = gpio_pin_configure(gpio0_dev, led_pins[i], GPIO_OUTPUT_HIGH);
        if (ret < 0) {
            printk("Error: gpio_pin_configure failed for led %d/pin %d, error:%d\n\r", i + 1, led_pins[i], ret);
            return -1;
        } else {
            printk("Success: gpio_pin_configure for led %d/pin %d\n\r", i + 1, led_pins[i]);
        }
    }

    for (i = 0; i < sizeof(buttons_pins); i++) {
        ret = gpio_pin_configure(gpio0_dev, buttons_pins[i], GPIO_INPUT | GPIO_PULL_UP);
        if (ret < 0) {
            printk("Error: gpio_pin_configure failed for button %d/pin %d, error:%d\n\r", i + 1, buttons_pins[i], ret);
            return -1;
        } else {
            printk("Success: gpio_pin_configure for button %d/pin %d\n\r", i + 1, buttons_pins[i]);
        }
    }

    printk("All devices initialized successfully!\n\r");

    return 0;
}