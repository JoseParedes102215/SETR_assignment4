#include <zephyr/kernel.h>          /* for k_msleep() */
#include <zephyr/device.h>          /* for device_is_ready() and device structure */
#include <zephyr/devicetree.h>      /* for DT_NODELABEL() */
#include <zephyr/drivers/gpio.h>    /* for GPIO api*/
#include <zephyr/sys/printk.h>      /* for printk()*/

#include "config_pins.h"
#include "fifo.h"

#include <zephyr/drivers/i2c.h>

#define K_SLEEP_MS 500

/* #define K_HEAP_DEFINE(HEAP,300)
K_HEAP_DEFINE()
 */

void main(void)
{   

    //config_pins();
    queue q1;
    MyFifoInit(&q1,QUEUE_MAX_SIZE);

    char exemplo[32] = "ola";
    char exemplo2[32] = "xau";



    MyFifoInsert(&q1,exemplo,1);
    MyFifoInsert(&q1,exemplo2,1);

    printk("O valor retornado e %s",MyFifoPeep(&q1));
    
    k_msleep(K_SLEEP_MS);
    MyFifoRemove(&q1);

    printk("O valor retornado e %s",MyFifoPeep(&q1));




}

