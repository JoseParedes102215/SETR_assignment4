#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include "fifo_RTDB.h"

/**

@file fifo.c
@brief Ficheiro fifo.c para as definições das funções


 @author José Paredes Manuel Miranda
 @date 12 March 2023

**/

char* getRTDB_chars(RTDB *rtdb){
    return rtdb->value;
}

int* getRTDB_state_led(RTDB *_RTDB){
    return _RTDB->state_led;
}

int* getRTDB_state_botao(RTDB *_RTDB){
    return _RTDB->state_botao;
}

int getRTDB_temp(RTDB *_RTDB){
    return _RTDB->temp_i2c;
}

int setRTDB_chars(RTDB *_RTDB, char* new_chars){
    memcpy(&_RTDB->value,&new_chars,sizeof(new_chars));
    return 0;
}




 int setRTDB_state_led(RTDB *_RTDB, int* new_states){
    memcpy(&_RTDB->state_led,&new_states,sizeof(new_states));
    return 0;
}

int setRTDB_state_botao(RTDB *_RTDB, int *new_state_botoes){
    memcpy(&_RTDB->state_botao,&new_state_botoes,sizeof(new_state_botoes));
        return 0;
}

int setRTDB_temp(RTDB *_RTDB, int new_temp){
    _RTDB->temp_i2c = new_temp;
        return 0;
} 