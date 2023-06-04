#include <zephyr/kernel.h>          /* for k_msleep() */
#include <zephyr/device.h>          /* for device_is_ready() and device structure */
#include <zephyr/devicetree.h>      /* for DT_NODELABEL() */
#include <zephyr/drivers/gpio.h>    /* for GPIO api*/
#include <zephyr/sys/printk.h>      /* for printk()*/
#include <drivers/uart.h>

#include "config_pins.h"
#include "fifo.h"
#include "i2c.h"

#include <zephyr/drivers/i2c.h>

#define MAIN_SLEEP_TIME_MS 2000 /* Time between main() activations */ 

#define FATAL_ERR -1 /* Fatal error return code, app terminates */

#define UART_NODE DT_NODELABEL(uart0)    /* UART Node label, see dts */

#define RXBUF_SIZE 60                   /* RX buffer size */
#define TXBUF_SIZE 60                   /* TX buffer size */
#define RX_TIMEOUT 100                  /* Inactivity period after the instant when last char was received that triggers an rx event (in us) */


/* Struct for UART configuration (if using default values is not needed) */
const struct uart_config uart_cfg = {
		.baudrate = 115200,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE
};

/* UAR related variables */
const struct device *uart_dev;          /* Pointer to device struct */ 
static uint8_t rx_buf[RXBUF_SIZE];      /* RX buffer, to store received data */
static uint8_t rx_chars[RXBUF_SIZE];    /* chars actually received  */
volatile int uart_rxbuf_nchar=0;        /* Number of chars currnetly on the rx buffer */

/* UART callback function prototype */
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data);

void init_uart(){
    int err=0; /* Generic error variable */
        
        /* Bind to UART */
        uart_dev= device_get_binding(DT_LABEL(UART_NODE));
        if (uart_dev == NULL) {
            printk("device_get_binding() error for device %s!\n\r", DT_LABEL(UART_NODE));
            return;
        }
        else {
            printk("UART binding successful\n\r");
        }

        /* Configure UART */
        err = uart_configure(uart_dev, &uart_cfg);
        if (err == -ENOSYS) { /* If invalid configuration */
            printk("uart_configure() error. Invalid configuration\n\r");
            return; 
        }

        /* Register callback */
        err = uart_callback_set(uart_dev, uart_cb, NULL);
        if (err) {
            printk("uart_callback_set() error. Error code:%d\n\r",err);
            return;
        }
            
        /* Enable data reception */
        err =  uart_rx_enable(uart_dev ,rx_buf,sizeof(rx_buf),RX_TIMEOUT);
        if (err) {
            printk("uart_rx_enable() error. Error code:%d\n\r",err);
            return;
        }
}

void main(void)
{   

    //config_pins();
    init_uart();
    queue q1;
    MyFifoInit(&q1,QUEUE_MAX_SIZE);

    while(1){

        if(uart_rxbuf_nchar > 0) {
            rx_chars[uart_rxbuf_nchar] = 0; /* Terminate the string */
            uart_rxbuf_nchar = 0;           /* Reset counter */
            MyFifoInsert(&q1,rx_chars,1);
        }
        if(*MyFifoPeep(&q1) != 224){
            //printk(" valor do MyPeep %d\n",*MyFifoPeep(&q1));
            printk("O valor lido do teclado e: %s ", MyFifoPeep(&q1));
            MyFifoRemove(&q1);
           
        }
        config_i2c();
        k_msleep(MAIN_SLEEP_TIME_MS);
    }









/* 
    char exemplo[32] = "ola";
    char exemplo2[32] = "xau";

 */

   /*  MyFifoInsert(&q1,exemplo,1);
    MyFifoInsert(&q1,exemplo2,1);

    printk("O valor retornado e %s",MyFifoPeep(&q1));
    
    k_msleep(MAIN_SLEEP_TIME_MS);
    MyFifoRemove(&q1);

    printk("O valor retornado e %s",MyFifoPeep(&q1));
 */



}

/* UART callback implementation */
/* Note that callback functions are executed in the scope of interrupt handlers. */
/* They run asynchronously after hardware/software interrupts and have a higher priority than all threads */
/* Should be kept as short and simple as possible. Heavier processing should be deferred to a task with suitable priority*/
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    int err;

    switch (evt->type) {
	
        case UART_TX_DONE:
		    printk("UART_TX_DONE event \n\r");
            break;

    	case UART_TX_ABORTED:
	    	printk("UART_TX_ABORTED event \n\r");
		    break;
		
	    case UART_RX_RDY:
		    //printk("UART_RX_RDY event \n\r");
            /* Just copy data to a buffer. Usually it is preferable to use e.g. a FIFO to communicate with a task that shall process the messages*/
            //MyFifoInsert(&q1,rx_buf[evt->data.rx.offset],1);
            memcpy(&rx_chars[uart_rxbuf_nchar],&(rx_buf[evt->data.rx.offset]),evt->data.rx.len);
            uart_rxbuf_nchar++;           
		    break;

	    case UART_RX_BUF_REQUEST:
		    printk("UART_RX_BUF_REQUEST event \n\r");
		    break;

	    case UART_RX_BUF_RELEASED:
		    printk("UART_RX_BUF_RELEASED event \n\r");
		    break;
		
	    case UART_RX_DISABLED: 
            /* When the RX_BUFF becomes full RX is is disabled automaticaly.  */
            /* It must be re-enabled manually for continuous reception */
            printk("UART_RX_DISABLED event \n\r");
		    err =  uart_rx_enable(uart_dev ,rx_buf,sizeof(rx_buf),RX_TIMEOUT);
            if (err) {
                printk("uart_rx_enable() error. Error code:%d\n\r",err);
                exit(FATAL_ERR);                
            }
		    break;

	    case UART_RX_STOPPED:
		    printk("UART_RX_STOPPED event \n\r");
		    break;
		
	    default:
            printk("UART: unknown event \n\r");
		    break;
    }

}