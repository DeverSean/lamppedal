/*****************************************************************************
* main.c
*
* Entrance into the lamppedal application.
*****************************************************************************/

/* Task based UART demo, using queued communication.
 *
 *	TX:	A9  ====> RX of TTL serial
 *	RX:	A10 <==== TX of TTL serial (not used)
 *	CTS:	A11 (not used)
 *	RTS:	A12 (not used)
 *	Config:	8N1
 *	Baud:	38400
 * Caution:
 *	Not all GPIO pins are 5V tolerant, so be careful to
 *	get the wiring correct.
 */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "uart.h"

static QueueHandle_t hUartTxq; // TX queue for UART

/*********************************************************************
 * Demo Task:
 *	Simply queues up two line messages to be TX, one second
 *	apart.
 *********************************************************************/
static void demo_task(void *args __attribute__((unused)))
{
  while(1)
  {
    uart_puts("Now this is a message..\n\r", hUartTxq);
    uart_puts("  sent via FreeRTOS queues.\n\n\r", hUartTxq);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/*********************************************************************
 * Main program & scheduler:
 *********************************************************************/
int main(void)
{

  rcc_clock_setup_in_hse_8mhz_out_72mhz(); // CPU clock is 72 MHz

  // GPIO PC13:
  rcc_periph_clock_enable(RCC_GPIOC);
  gpio_set_mode(
      GPIOC,
      GPIO_MODE_OUTPUT_2_MHZ,
      GPIO_CNF_OUTPUT_PUSHPULL,
      GPIO13);

  uart_setup(&hUartTxq);

  xTaskCreate(uart_task, "UART", 100, &hUartTxq, configMAX_PRIORITIES - 1, NULL);
  xTaskCreate(demo_task, "DEMO", 100, NULL, configMAX_PRIORITIES - 1, NULL);

  vTaskStartScheduler();
  while(1){}

  return 0;
}