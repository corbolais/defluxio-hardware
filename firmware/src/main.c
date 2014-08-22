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
#include <inttypes.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/f4/nvic.h>
#include "ticks.h"


// maintain the state of a measurement
typedef enum {
  IDLE,
  PENDING,
  RUNNING
} mstate_t;
volatile mstate_t m_state = IDLE;
volatile uint32_t cycles_during_measurement = 0;
volatile uint32_t overcapture = 0;
volatile uint32_t overflow_counter = 0;
volatile uint32_t start_counter = 0;
volatile uint32_t edges = 0;

/* How many cycles do we use for a measurement? */
static const uint32_t NUM_EDGES = 50;
static const uint32_t TIMER1_HZ = 168000000;
static const uint32_t TIMER1_PERIOD = 0xFFFF;
static const uint32_t TIMER1_PRESCALER = 0xA8; // 1MHz sample rate

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
	/* Configure timer1. */
	timer_set_mode(TIM1,
			TIM_CR1_CKD_CK_INT,		// Internal clock
			TIM_CR1_CMS_EDGE,			// Edge synchronization
			TIM_CR1_DIR_UP);			// Count upward
	timer_set_prescaler(TIM1, TIMER1_PRESCALER);
	timer_set_period(TIM1, TIMER1_PERIOD); //Sets TIM1_ARR
	timer_continuous_mode(TIM1);
		/* Configure PE13: Toggle pin on falling edge via interrupt */
	//rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPEEN);
	//gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPIO13);
	/* Configure input capture. */
	timer_ic_disable(TIM1, TIM_IC2);
	timer_ic_set_polarity(TIM1, TIM_IC2, TIM_IC_RISING);
	timer_ic_set_prescaler(TIM1, TIM_IC2, TIM_IC_PSC_OFF);
	timer_ic_set_input(TIM1, TIM_IC2, TIM_IC_IN_TI2);
	// See RM, p. 561: digital filter
	//timer_ic_set_filter(TIM1, TIM_IC2, TIM_IC_DTF_DIV_32_N_8);
	timer_ic_set_filter(TIM1, TIM_IC2, TIM_IC_OFF);
	timer_ic_enable(TIM1, TIM_IC2);
	/* Enable counter. */
	timer_enable_counter(TIM1);
	timer_clear_flag (TIM1, TIM_SR_CC2IF);
	/* Enable IRQs */
	nvic_enable_irq(NVIC_TIM1_UP_TIM10_IRQ);
	timer_enable_irq(TIM1, TIM_DIER_UIE);
	nvic_enable_irq(NVIC_TIM1_CC_IRQ);
	timer_enable_irq(TIM1, TIM_DIER_CC2IE);
}


void tim1_up_tim10_isr(void) {
	if(timer_get_flag(TIM1, TIM_SR_UIF)) {
		timer_clear_flag(TIM1, TIM_SR_UIF);
		overflow_counter++;
	}
}


void tim1_cc_isr(void) {
	if(timer_get_flag(TIM1, TIM_SR_CC2IF)) {
		if (timer_get_flag(TIM1, TIM_SR_CC2OF)) 
			overcapture++;
		uint32_t current_counter = timer_get_counter(TIM1);
		uint32_t current_overflow = overflow_counter;
		timer_clear_flag(TIM1, TIM_SR_CC2IF);
		//gpio_toggle(GPIOE, GPIO13); /* toggle pin for 'scope debugging */
		if (m_state == PENDING) { // start measurement
			edges=0;
			start_counter = current_counter;
			overflow_counter = 0;
			m_state = RUNNING;
		}
		if (edges == NUM_EDGES) { // end measurement
			cycles_during_measurement = 
						(current_overflow*TIMER1_PERIOD) + current_counter - start_counter;
			m_state = IDLE;
		} else {
			edges += 1;
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
/* Configure PE11 (AF1: TIM1_CH2) (SYNC_IN). */
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPEEN);
	gpio_mode_setup(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11);
	gpio_set_af(GPIOE, GPIO_AF1, GPIO11);

}

int main(void) {

	clock_setup();
	gpio_setup();
	usart_setup();

	printf("\nDefluxio Frequency Measurement Hardware started.\n");
	while (1) {
		gpio_toggle(GPIOD, GPIO12);	/* LED as 'alive' signal*/
		m_state = PENDING; /* start new measurement */
		while (m_state != IDLE) ;; /* wait for measurement to finish */

		printf("Cycles during measurement: %" PRId32 ", no. of overflows: %" PRId32 ", overcapture: %" PRId32"\n", cycles_during_measurement, overflow_counter, overcapture);
		double avg_period = (double) cycles_during_measurement /
			(((double) TIMER1_HZ / (double) (TIMER1_PRESCALER+1)) * edges);
		printf("Period: %1.6f, Frequency: %f, delta %3.0f mHz\n", avg_period, 1/(avg_period), (1/(avg_period)-50)*1000);
		fflush(stdout);
	}
	return 0;
}
