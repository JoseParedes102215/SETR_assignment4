#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <drivers/uart.h>
#include <zephyr/timing/timing.h>

#include "config_pins.h"
#include "fifo.h"
#include "i2c.h"
#include "fifo_RTDB.h"

#include <zephyr/drivers/i2c.h>

#define MAIN_SLEEP_TIME_MS 2000
#define FATAL_ERR -1

#define UART_NODE DT_NODELABEL(uart0)

#define RXBUF_SIZE 60
#define TXBUF_SIZE 60
#define RX_TIMEOUT 100

const struct uart_config uart_cfg = {
    .baudrate = 115200,
    .parity = UART_CFG_PARITY_NONE,
    .stop_bits = UART_CFG_STOP_BITS_1,
    .data_bits = UART_CFG_DATA_BITS_8,
    .flow_ctrl = UART_CFG_FLOW_CTRL_NONE
};

const struct device *uart_dev;
static uint8_t rx_buf[RXBUF_SIZE];
static uint8_t rx_chars[RXBUF_SIZE];
volatile int uart_rxbuf_nchar = 0;

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data);

void init_uart() {
    int err = 0;

    uart_dev = device_get_binding(DT_LABEL(UART_NODE));
    if (uart_dev == NULL) {
        printk("device_get_binding() error for device %s!\n\r", DT_LABEL(UART_NODE));
        return;
    } else {
        printk("UART binding successful\n\r");
    }

    err = uart_configure(uart_dev, &uart_cfg);
    if (err == -ENOSYS) {
        printk("uart_configure() error. Invalid configuration\n\r");
        return;
    }

    err = uart_callback_set(uart_dev, uart_cb, NULL);
    if (err) {
        printk("uart_callback_set() error. Error code:%d\n\r", err);
        return;
    }

    err = uart_rx_enable(uart_dev, rx_buf, sizeof(rx_buf), RX_TIMEOUT);
    if (err) {
        printk("uart_rx_enable() error. Error code:%d\n\r", err);
        return;
    }
}

#define STACK_SIZE 1024

#define thread_A_prio 1
#define thread_B_prio 1
#define thread_C_prio 1
#define thread_D_prio 1
#define thread_E_prio 1

#define thread_A_period 1000
#define thread_B_period 2000
#define thread_C_period 3000
#define thread_D_period 3000
#define thread_E_period 500

K_THREAD_STACK_DEFINE(thread_A_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_B_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_C_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_D_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_E_stack, STACK_SIZE);

struct k_thread thread_A_data;
struct k_thread thread_B_data;
struct k_thread thread_C_data;
struct k_thread thread_D_data;
struct k_thread thread_E_data;

k_tid_t thread_A_tid;
k_tid_t thread_B_tid;
k_tid_t thread_C_tid;
k_tid_t thread_D_tid;
k_tid_t thread_E_tid;

queue q1;

RTDB RT= {
    //value ={0} ;/**< array de carateres armazenado no nodo */
    //state_led[4]={0};
    //tate_botao[4]={0};
    //temp_i2c = 0;
};

// Thread A, comandos leitura comandos da uart
void thread_A_code(void *arg1, void *arg2, void *arg3) {
    int64_t fin_time = 0, release_time = 0;
    timing_t start_time, end_time;

    printk("Thread A init (periodic)\n");

    release_time = k_uptime_get() + thread_A_period;

    while (1) {
        start_time = timing_counter_get();
        if (uart_rxbuf_nchar > 0) {
            rx_chars[uart_rxbuf_nchar] = 0;
            uart_rxbuf_nchar = 0;
            MyFifoInsert(&q1, rx_chars, 1);
        }
        if (*MyFifoPeep(&q1) != 224) {
            printk("O valor lido do teclado e: %s ", MyFifoPeep(&q1));
            // colocar os comandos na RTBD
            MyFifoRemove(&q1);
        }

        fin_time = k_uptime_get();
        if (fin_time < release_time) {
            k_msleep(release_time - fin_time);
            release_time += thread_A_period;
        }
    }

    timing_stop();
}

// thread que lê os valores do sensor de temperatura e atualiza a RTBD
void thread_B_code(void *arg1, void *arg2, void *arg3) {
    int64_t fin_time = 0, release_time = 0;
    timing_t start_time, end_time;

    printk("Thread B init (periodic)\n");

    release_time = k_uptime_get() + thread_B_period;

    while (1) {
        start_time = timing_counter_get();
        int t = get_temp();
        // dar lock aqui
        setRTDB_temp(&RT,t);
        // unlock aqui
        printk("temp=%d\n",t);
        fin_time = k_uptime_get();
        if (fin_time < release_time) {
            k_msleep(release_time - fin_time);
            release_time += thread_B_period;
        }
    }

    timing_stop();
    }

// thread que lê os valores dos botõers e atualiza a RTBD
void thread_C_code(void *arg1, void *arg2, void *arg3) {
    int button1,button2,button3,button4;
    int button_val[4];
     int64_t fin_time = 0, release_time = 0;
    timing_t start_time, end_time;

    printk("Thread C init (periodic)\n");

    release_time = k_uptime_get() + thread_C_period;

    while (1) {
        start_time = timing_counter_get();
        button1 = gpio_pin_get(gpio0_dev,11); 
        button2 = gpio_pin_get(gpio0_dev,12); 
        button3 = gpio_pin_get(gpio0_dev,24); 
        button4 = gpio_pin_get(gpio0_dev,25); 

        button_val[0] = button1;
        button_val[1] = button2;
        button_val[2] = button3;
        button_val[3] = button4;

        // dar lock aqui
        setRTDB_state_botao(&RT,button_val);
        // unlock aqui
        printk("button1:%d, button2:%d, button3:%d, button4:%d \r\n",button1,button2,button3,button4);

        fin_time = k_uptime_get();
        if (fin_time < release_time) {
            k_msleep(release_time - fin_time);
            release_time += thread_B_period;
        }
    }

    timing_stop();

}

// thread que lê o array da RTBD e atualiza o valor dos leds
void thread_D_code(void *arg1, void *arg2, void *arg3) {
     int64_t fin_time = 0, release_time = 0;
    timing_t start_time, end_time;

    printk("Thread C init (periodic)\n");

    release_time = k_uptime_get() + thread_C_period;

    while (1) {
        start_time = timing_counter_get();

        // lock 
        int led_vals[4];
         //led_vals =  *getRTDB_state_led(&RT);
         memcpy(&led_vals,getRTDB_state_led(&RT),4);
        // unlock aqui
        gpio_pin_set(gpio0_dev, 13,led_vals[0]);
        gpio_pin_set(gpio0_dev, 14,led_vals[1]);
        gpio_pin_set(gpio0_dev, 15,led_vals[2]);
        gpio_pin_set(gpio0_dev, 16,led_vals[3]);
        

        fin_time = k_uptime_get();
        if (fin_time < release_time) {
            k_msleep(release_time - fin_time);
            release_time += thread_B_period;
        }
    }

    timing_stop();

}


// esta thread concatena os carateres do fifo e coloca-os sem processamento no RTBD
void thread_E_code(void *arg1, void *arg2, void *arg3) {
    int64_t fin_time = 0, release_time = 0;
    timing_t start_time, end_time;

    printk("Thread A init (periodic)\n");

    /* release_time = k_uptime_get() + thread_E_period;
    node *tmp_curr = q1->head;
     */

    while (1) {
        start_time = timing_counter_get();
        
        /* for(int i = 0; i < MyFifoSize(&q1); i++){

            
        }
 */

        fin_time = k_uptime_get();
        if (fin_time < release_time) {
            k_msleep(release_time - fin_time);
            release_time += thread_E_period;
        }
    }

    timing_stop();
}

// outra thread processa os comandos dentro da RTBD
// adcionar os locks e os unlocks

void main(void) {
    config_pins();
    init_uart();
    MyFifoInit(&q1, QUEUE_MAX_SIZE);

    thread_A_tid = k_thread_create(&thread_A_data, thread_A_stack, STACK_SIZE,
                                   thread_A_code, NULL, NULL, NULL,
                                   thread_A_prio, 0, K_NO_WAIT);

    thread_B_tid = k_thread_create(&thread_B_data, thread_B_stack, STACK_SIZE,
                                   thread_B_code, NULL, NULL, NULL,
                                   thread_B_prio, 0, K_NO_WAIT);

    thread_C_tid = k_thread_create(&thread_C_data, thread_C_stack, STACK_SIZE,
                                   thread_C_code, NULL, NULL, NULL,
                                   thread_C_prio, 0, K_NO_WAIT);
    thread_D_tid = k_thread_create(&thread_D_data, thread_D_stack, STACK_SIZE,
                                   thread_D_code, NULL, NULL, NULL,
                                   thread_D_prio, 0, K_NO_WAIT);
    thread_E_tid = k_thread_create(&thread_E_data, thread_E_stack, STACK_SIZE,
                                   thread_E_code, NULL, NULL, NULL,
                                   thread_D_prio, 0, K_NO_WAIT);
}



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