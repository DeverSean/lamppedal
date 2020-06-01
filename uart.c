/*****************************************************************************
* uart.c
*
* UART driver implementation
*****************************************************************************/
#include <stdint.h>
#include <assert.h>


#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <libopencm3/stm32/usart.h>
/*********************************************************************
 * Configure and initialize USART1:
 *********************************************************************/
void uart_setup(QueueHandle_t *phTxQueue) 
{

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART1);

	// UART TX on PA9 (GPIO_USART1_TX)
	gpio_set_mode(GPIOA,
		GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
		GPIO_USART1_TX);

	usart_set_baudrate(USART1,38400);
	usart_set_databits(USART1,8);
	usart_set_stopbits(USART1,USART_STOPBITS_1);
	usart_set_mode(USART1,USART_MODE_TX);
	usart_set_parity(USART1,USART_PARITY_NONE);
	usart_set_flow_control(USART1,USART_FLOWCONTROL_NONE);
	usart_enable(USART1);

	// Create a queue for data to transmit from UART
	*phTxQueue = xQueueCreate(UART_QUEUE_SIZE, sizeof(char));
}

/*********************************************************************
 * USART Task: 
 * 
 * @param pArg - A pointer to the queue handle
 *********************************************************************/
void uart_task(void *pArg)
{
  char ch;
  
  assert(pArg);

  for (;;)
  {
    // Receive char to be TX
    if (xQueueReceive((QueueHandle_t)*pArg, &ch, 500) == pdPASS)
    {
      while (!usart_get_flag(USART1, USART_SR_TXE))
        taskYIELD(); // Yield until ready
      usart_send(USART1, ch);
    }
    // Toggle LED to show signs of life
    gpio_toggle(GPIOC, GPIO13);
  }
}

/*********************************************************************
 * Queue a string of characters to be TX
 * 
 * @param s - a pointer to the string to be enqueued
 * @param hTxQueue - the hande to the queue
 *********************************************************************/
void uart_puts(const char *s, QueueHandle_t hTxQueue)
{

  for (; *s; ++s)
  {
    // blocks when queue is full
    xQueueSend(hTxQueue, s, portMAX_DELAY);
  }
}