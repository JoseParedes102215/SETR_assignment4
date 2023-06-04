#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

/**

@file fifo.c
@brief Ficheiro fifo.c para as definições das funções


 @author José Paredes Manuel Miranda
 @date 12 March 2023


*/


#include "fifo.h"

void MyFifoInit(queue *q, int maxsize){
    q->head = NULL;
    q->tail = NULL;
    q->size = 0;
}

int MyFifoSize(queue *q){
    return q->size;
}

int MyFifoInsert(queue *q, char* value, int priority) {
    // Cria um novo nodo
    node *newnode = k_malloc(sizeof(node));
    if (newnode == NULL) {
        fprintf(stderr, "O malloc falhou\n");
        return MALLOC_FAILED; // malloc falhou
    }
    else {
        if (q->size == QUEUE_MAX_SIZE) {
            fprintf(stderr, "O Fifo está cheio\n");
            return QUEUE_FULL;
        }
        q->size += 1;
        strcpy(newnode->value, value);
        newnode->priority = priority;
        newnode->next = NULL;
        // Se já existir um tail, conectar essa tail à nova
        if (q->tail != NULL) {
            q->tail->next = newnode;
        }
        q->tail = newnode;
        // Assegurar que o head faz sentido
        if (q->head == NULL) {
            q->head = newnode;
        }
        return 0;
    }
}

int MyFifoCheckPriority(queue *q){
    int max_prior = -1;
    node *tmp_curr = q->head;
    int idx = -1;
    for (int i = 0; i < q->size; i++)
    {
        if(tmp_curr->priority > max_prior){
            max_prior = tmp_curr->priority;
            idx = i; 
        }
        tmp_curr = tmp_curr->next;
    }
    return idx;
}

int MyFifoRemove(queue *q){
    // Verificar se o fifo está vazio
    
    if(q->head == NULL) {
        return QUEUE_EMPTY; // fifo vazio*
    }
    node *tmp_curr = q->head;
    node *tmp = NULL;
    
    int idx = MyFifoCheckPriority(q);
    
    /* condição de o elemento de maior
    prioridade ser o primeiro */
    if (idx == 0) {
        q->head = q->head->next;
        if (q->head == NULL) {
            q->tail = NULL;
        }
        q->size = q->size - 1;
        k_free(tmp_curr);
    } else {
        for (int i = 0; i < idx; i++)
        {
            tmp = tmp_curr;
            tmp_curr = tmp_curr->next;
        }
        tmp->next = tmp_curr->next;
        q->size = q->size - 1;

        /* condição de o elemento de maior
        prioridade ser o ultimo */ 
        if (tmp_curr == q->tail) {
            q->tail = tmp;
        }
        k_free(tmp_curr);
    }
    return 0;
}


void printMyFifo(queue *q){
    node *tmp2 = q->head;
    if(tmp2 == NULL){
        printf("O fifo esta vazio \n");
    }
    else{

        for (int i = 0; i < q->size; i++)
        {
            //printf("%s -prioridade = %s\n",tmp2->value,tmp2->priority);
            tmp2 = tmp2->next;

        }
    }
}

char* MyFifoPeep(queue *q){
    if(MyFifoSize(q) == 0){
        //return QUEUE_EMPTY;
        return NULL;
    }
    else {
        return q->head->value; //retorna o elemento mais antigo
    }
}