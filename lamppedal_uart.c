/*****************************************************************************
* uart.c
*
* Source code for interfacing with the STM32 USART
*****************************************************************************/
#include <stdint.h>
#include <assert.h>

#include <FreeRTOS.h>
#include <queue.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>

#include "lamppedal_uart.h"
/*****************************************************************************
* Private Data / Defined Constants
*****************************************************************************/
#define QUEUE_SIZE 256
#define DATA_BITS  8
#define STOP_BITS  1

/*****************************************************************************
* Private Prototypes
*****************************************************************************/
static void uart_setup(uint8_t bBusNum, uint32_t uBaud);
static void uart_task(void *arg __attribute((unused)));

/*****************************************************************************
* Code
*****************************************************************************/

/*----------------------------------------------------------------------------
@function: uart_task_init

@brief:    A function for creating a uart

@params:   - pxTask, a handle to the created task (upon exit)
           - hUartTxq, a handle to the uart message queue 

@returns:  - err, error indication for task creation
----------------------------------------------------------------------------*/
int8_t uart_task_init(TaskHandle_t *pxTask, QueueHandle_t hUartTxq)
{
  int8_t err = 0;
  //TODO fix unused arg warning
  uart_setup(1, 38400);
  
  hUartTxq = xQueueCreate(QUEUE_SIZE, sizeof(uint8_t));

  err = xTaskCreate(uart_task, "UART", 100, (void *)hUartTxq, configMAX_PRIORITIES-1, pxTask);

  return err;
}

/*----------------------------------------------------------------------------
@function: uart_setup

@brief:    A function for configuring the USART on the STM32

@params:   - bBusNum, the USART bus number to be used
           - uBaud, the desired baudrate 

@returns:  None
----------------------------------------------------------------------------*/
static void uart_setup(uint8_t bBusNum, uint32_t uBaud)
{
  if (bBusNum == 1)
  {
    rcc_periph_clock_enable(RCC_GPIOA);  //usart tx pin clock
    rcc_periph_clock_enable(RCC_USART1); //usart peripheral clock

    // Set up TX pin on PA9
    gpio_set_mode(GPIOA,
                  GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                  GPIO_USART1_TX);

    //TODO if this code is portable across buses move outside of conditional
    // all of type void
    usart_set_baudrate(USART1, uBaud);
    usart_set_databits(USART1, DATA_BITS);
    usart_set_stopbits(USART1, STOP_BITS);
    usart_set_mode(USART1, USART_MODE_TX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_enable(USART1);
  }
  else
  {
    assert(0);
  }

}

/*----------------------------------------------------------------------------
@function: uart_task

@brief:    A task for transmitting data via the USART

@params:   None

@returns:  None
----------------------------------------------------------------------------*/
static void uart_task(void *pArg)
{
  uint8_t uChar;
  
  while (1)
  {
    vTaskDelay(pdMS_TO_TICKS(100));
    
    if (xQueueReceive((QueueHandle_t)pArg, &uChar, 500) == pdPASS)
    {
      //Wait for usart to be ready
      while (usart_get_flag(USART1, USART_SR_TXE) == 0)
      {
        taskYIELD();
      }

      usart_send(USART1, uChar);
    }
  }
}

/*----------------------------------------------------------------------------
@function: uart_puts

@brief:    A helper function for submitting data to the tx queue
           Does not block if the queue is full

@params:   - pStr, a pointer to a constant ASCII string 

@returns:  None
----------------------------------------------------------------------------*/
uint8_t uart_puts(const char *pStr, QueueHandle_t hUartTxq)
{
  uint8_t err = 0;

  for (; *pStr; ++pStr)
  {
    err = xQueueSend(hUartTxq, pStr, 0);
  }

  return err;
}
