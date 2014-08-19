/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <stdio.h>
#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/f4/nvic.h>
#include "ticks.h"

volatile uint32_t n_overflow = 0;
volatile uint32_t summed_periods = 0;
volatile uint32_t period_counter = 0;

static void clock_setup(void) {

  rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);

	/* Enable GPIOD clock for LED & USARTs. */
	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Enable clocks for USART2. */
	rcc_periph_clock_enable(RCC_USART2);


	/* Timer1: Input compare */

	/* Enable timer clock. */
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_TIM1EN);
	/* Reset timer. */
	timer_reset(TIM1);
	/* Configure prescaler. */
	timer_set_prescaler(TIM1, 0);
	/* Configure PE11 (AF1: TIM1_CH2) (SYNC_IN). */
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPEEN);
	gpio_mode_setup(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11);
	gpio_set_af(GPIOE, GPIO_AF1, GPIO11);
	/* Configure PE13: Toggle pin on falling edge via interrupt */
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPEEN);
	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13);
	/* Configure input capture. */
	timer_ic_disable(TIM1, TIM_IC2);
	timer_ic_set_input(TIM1, TIM_IC2, TIM_IC_IN_TI2);
	timer_ic_set_polarity(TIM1, TIM_IC2, TIM_IC_RISING);
	timer_ic_set_prescaler(TIM1, TIM_IC2, TIM_IC_PSC_OFF);
	timer_ic_set_filter(TIM1, TIM_IC2, TIM_IC_OFF);
	timer_ic_enable(TIM1, TIM_IC2);
	/* Enable counter. */
	TIM_CCER(TIM1) |= TIM_CCER_CC1E;
	timer_enable_counter(TIM1);
	/* Enable IRQs */
	nvic_enable_irq(NVIC_TIM1_UP_TIM10_IRQ);
	timer_enable_irq(TIM1, TIM_DIER_UIE);
	nvic_enable_irq(NVIC_TIM1_CC_IRQ);
	timer_enable_irq(TIM1, TIM_DIER_CC2IE);
}


void tim1_up_tim10_isr(void) {
	if(timer_get_flag(TIM1, TIM_SR_UIF)) {
		timer_clear_flag(TIM1, TIM_SR_UIF);
		n_overflow++;
	}
}

volatile double last_avg_period = 0.0;

// TODO: Overflow handling is totally b0rked.
void tim1_cc_isr(void) {
	if(timer_get_flag(TIM1, TIM_SR_CC2IF)) {
		timer_clear_flag(TIM1, TIM_SR_CC2IF);
		gpio_toggle(GPIOE, GPIO13); /* toggle pin for 'scope debugging */
		summed_periods += TIM_CCR2(TIM1);
		period_counter += 1;
		//n_overflow = 0;
		if (period_counter == 50) {
			last_avg_period = ((n_overflow*((2^16)-1)+summed_periods)/84000000.0);
			summed_periods=0;
			period_counter=0;
			n_overflow = 0;
		}
	}
}



static void usart_setup(void) {
	/* Setup USART2 parameters. */
	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART2);
}

static void gpio_setup(void) {
	/* Setup GPIO pin GPIO12 on GPIO port D for LED. */
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);

	/* Setup GPIO pins for USART2 transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);

	/* Setup USART2 TX pin as alternate function. */
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2);
}

int main(void) {

	clock_setup();
	systick_setup(1000);
	gpio_setup();
	usart_setup();

	printf("\nSTM32F4-Discovery skeleton code started.\n");
	uint32_t last_tick = mtime();
	while (1) {
		/* Blink the LED (PD12) on the board with every transmitted byte. */
		gpio_toggle(GPIOD, GPIO12);	/* LED on/off */
		printf("Average period: %f\n", last_avg_period);
		printf("Frequency: %f\n", 1/last_avg_period);
		uint32_t tick = mtime();
		printf("Tick: %d\n", tick - last_tick);
		last_tick = tick;
		fflush(stdout);
		msleep(1000); // sleep for one second

	}
	return 0;
}
