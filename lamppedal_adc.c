/*****************************************************************************
* adc.c
*
* Source code for interfacing with the STM32 ADC
*****************************************************************************/
#include <stdint.h>
#include <assert.h>

#include <FreeRTOS.h>
#include "task.h"

#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
/*****************************************************************************
* Private Data / Defined Constants
*****************************************************************************/
#define ADC_CHANNEL 1

/*****************************************************************************
* Private Prototypes
*****************************************************************************/

/*****************************************************************************
* Code
*****************************************************************************/

/*----------------------------------------------------------------------------
@function: adc_setup

@brief:    A function for configuring the ADC on the STM32

@params:   - TODO

@returns:  None
----------------------------------------------------------------------------*/
void adc_setup()
{

  /* Set GPIOA/GPIO1 to ADC input */
  gpio_set_mode(GPIOA,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_ANALOG,
                GPIO1);

  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);

  adc_power_off(ADC1);
  rcc_peripheral_reset(&RCC_APB2RSTR, RCC_APB2RSTR_ADC1RST);
  rcc_peripheral_clear_reset(&RCC_APB2RSTR, RCC_APB2RSTR_ADC1RST);
  rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV6); // Set. 12MHz, Max. 14MHz
  adc_set_dual_mode(ADC_CR1_DUALMOD_IND);     // Independent mode
  adc_disable_scan_mode(ADC1);
  adc_set_right_aligned(ADC1);
  adc_set_single_conversion_mode(ADC1);
  adc_set_sample_time(ADC1, ADC_CHANNEL_TEMP, ADC_SMPR_SMP_239DOT5CYC);
  adc_set_sample_time(ADC1, ADC_CHANNEL_VREF, ADC_SMPR_SMP_239DOT5CYC);
  adc_enable_temperature_sensor();
  adc_power_on(ADC1);
  adc_reset_calibration(ADC1);
  adc_calibrate_async(ADC1);

  while (adc_is_calibrating(ADC1))
  {
    /* Wait for calibration to complete */
  }
}

/*-----------------------------------------------------------------------------
@function: read ADC channel

@brief:    read a sample value from the ADC peripheral

@params:   uint8_t channel - the adc channel to be read

@returns:  uint16_t - the value returned from adc_read_regular
-----------------------------------------------------------------------------*/
uint16_t read_adc(uint8_t channel)
{
	adc_set_sample_time(ADC1, channel, ADC_SMPR_SMP_239DOT5CYC);
	adc_set_regular_sequence(ADC1, 1, &channel);
	adc_start_conversion_direct(ADC1);
	while (!adc_eoc(ADC1))
	{
    taskYIELD();
  }
	return adc_read_regular(ADC1);
}
