/*****************************************************************************
* lamppedal_uart.h
*
* Public interface for adc driver
*****************************************************************************/
#ifndef LAMPPEDAL_ADC_H_INCLUDED
#define LAMPPEDAL_ADC_H_INCLUDED

#include <stdint.h>
#include <queue.h>

/*****************************************************************************
* Public Prototypes
*****************************************************************************/
void adc_setup(void);
uint16_t read_adc(uint8_t channel);

#endif //LAMPPEDAL_ADC_H_INCLUDED
