/*****************************************************************************
* main.c
*
* Entrance into the lamppedal application.
*****************************************************************************/

/*****************************************************************************
* Includes
*****************************************************************************/
#include <stdint.h>
#include <stdio.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/rcc.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "lamppedal_setup.h"
#include "lamppedal_adc.h"
#include "lamppedal_uart.h"

/*****************************************************************************
* Defines / Private Data
*****************************************************************************/
static QueueHandle_t m_hMessageQueue;

#define FALLING 0
#define RISING 1

/*****************************************************************************
* Function Prototypes
*****************************************************************************/
static void data_logger_task(void *pArg __attribute__((unused)));

/*****************************************************************************
* Data
*****************************************************************************/
int myTicks = 0;

/*****************************************************************************
* Code 
*****************************************************************************/

/*----------------------------------------------------------------------------
@function: gpio_setup

@brief:    A function for configuring GPIO

@params:   none

@returns:  none
----------------------------------------------------------------------------*/
static void gpio_setup(void)
{
  /* Enable GPIOC clock. */
  rcc_periph_clock_enable(RCC_GPIOB);

  /* Set GPIO12 (in GPIO port B) to 'output push-pull'. */
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL, GPIO5);
}

/*-----------------------------------------------------------------------------
@function: main

@brief:    The entry point into the lamppedal application

@params:   none

@returns:  int - the return value of the application (should never return)
-----------------------------------------------------------------------------*/
int main(void)
{
  clock_setup();
  gpio_setup();
  // tim_setup();
  // exti_setup();
  gpio_set(GPIOB,GPIO5);

  //adc_setup();
  uart_task_init(NULL, *m_hMessageQueue); //Don't save handle to task for now
  assert(xTaskCreate(data_logger_task, "data_logger", 100, NULL, configMAX_PRIORITIES-1, NULL) == 0);

  vTaskStartScheduler();

  while(1){}
  
  return 0;
}

/*-----------------------------------------------------------------------------
@function: main

@brief:    The entry point into the lamppedal application

@params:   none

@returns:  int - the return value of the application (should never return)
-----------------------------------------------------------------------------*/
static void data_logger_task(void *pArg __attribute__((unused)))
{
  uint8_t  err = 0;
  //uint16_t uwAdcVal;
  char    *aChar = "Test String\n";

  while(1)
  {
    vTaskDelay(pdMS_TO_TICKS(10));

    //uwAdcVal = read_adc(3);
    //sprintf(aChar, %0d, uwAdcVal);
    err = uart_puts(aChar);
  }
}