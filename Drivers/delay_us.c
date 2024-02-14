/*
 * delay_us.c
 *
 *  Created on: Feb 10, 2024
 *      Author: seanp
 */


#include "delay_us.h"
#include "main.h"

void delay_us (uint16_t us, TIM_HandleTypeDef *htimer)
{
	// set the counter value a 0
	__HAL_TIM_SET_COUNTER(htimer,0);

	// wait for the counter to reach the us input in the parameter
	while (__HAL_TIM_GET_COUNTER(htimer) < us);
}
