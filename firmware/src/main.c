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
#include "freq_capture.h"
#include "log_formatter.h"


static void clock_setup(void) {
  rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);
}


static void usart_setup(void) {
	/* Enable clocks for USART2 & GPIOA (TX pin). */
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_GPIOA);
		/* Setup USART2 parameters. */
	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
	/* Finally enable the USART. */
	usart_enable(USART2);
/* Setup GPIO pins for USART2 transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
	/* Setup USART2 TX pin as alternate function. */
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2);

}

static void led_setup(void) {
	/* Setup GPIO pin GPIO0 on GPIO port E (PE0) for LED. */
	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO0);
	/* Enable GPIOE clock for LED. */
	rcc_periph_clock_enable(RCC_GPIOE);
}

int main(void) {
	clock_setup();
	usart_setup();
	led_setup();
	freq_capture_setup();
	log_info("Defluxio Frequency Measurement Hardware started.");
	char buf[100];
	while (1) {
		gpio_toggle(GPIOE, GPIO0);	/* LED as 'alive' signal*/
		freq_capture_start();
		while (freq_get_state() != IDLE) ;; /* wait for measurement to finish */
		double freq = freq_get_result();
		snprintf(buf, sizeof(buf), "Frequency: %f Hz, delta: %3.0f mHz", freq, (freq-50)*1000);
		log_info(buf);
		log_freq(freq);
	}
	return 0;
}
