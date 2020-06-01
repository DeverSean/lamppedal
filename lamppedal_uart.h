/*****************************************************************************
* lamppedal_uart.h
*
* Public interface for uart driver
*****************************************************************************/

#ifndef LAMPPEDAL_UART_H_INCLUDED
#define LAMPPEDAL_UART_H_INCLUDED

#include <stdint.h>
#include <queue.h>

/*****************************************************************************
* Public Prototypes
*****************************************************************************/
int8_t uart_task_init(TaskHandle_t *pxTask, QueueHandle_t hUartTxq);
uint8_t uart_puts(const char *pStr, QueueHandle_t hUartTxq);
#endif //LAMPPEDAL_UART_H_INCLUDED
