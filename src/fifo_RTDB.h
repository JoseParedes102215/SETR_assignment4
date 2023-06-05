#ifndef FIFO_RTDB_H
#define FIFO_RTDB_H


#include <zephyr/kernel.h>          /* for k_msleep() */
#include <zephyr/device.h>          /* for device_is_ready() and device structure */
#include <zephyr/devicetree.h>      /* for DT_NODELABEL() */
#include <zephyr/drivers/gpio.h>    /* for GPIO api*/
#include <zephyr/sys/printk.h>      /* for printk()*/



#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>


typedef struct RTDB{
char value[64]; /**< array de carateres armazenado no nodo */
int state_led[4];
int state_botao[4];
int temp_i2c;
} RTDB;

char* getRTDB_chars(RTDB *R);

char* getRTDB_chars(RTDB *_RTDB);

int* getRTDB_state_led(RTDB *_RTDB);

int* getRTDB_state_botao(RTDB *_RTDB);

int getRTDB_temp(RTDB *_RTDB);

int setRTDB_chars(RTDB *_RTDB, char* new_chars);

int setRTDB_state_led(RTDB *_RTDB, int* new_states);

int setRTDB_state_botao(RTDB *_RTDB, int *new_state_botoes);

int setRTDB_temp(RTDB *_RTDB, int new_temp);



#endif