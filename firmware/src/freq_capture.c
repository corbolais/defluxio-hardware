#include "freq_capture.h"
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/f4/nvic.h>


/* How many cycles do we use for a measurement? */
static const uint32_t NUM_EDGES = 50;
static const uint32_t TIMER1_HZ = 168000000;
static const uint32_t TIMER1_PERIOD = 0xFFFF;
static const uint32_t TIMER1_PRESCALER = 0xA8; // 1MHz sample rate

volatile mstate_t m_state = IDLE;
volatile uint32_t cycles_during_measurement = 0;
volatile uint32_t overcapture = 0;
volatile uint32_t overflow_counter = 0;
volatile uint32_t start_counter = 0;
volatile uint32_t edges = 0;


void freq_capture_setup(void) {
	/* Configure PE11 (AF1: TIM1_CH2) (SYNC_IN). */
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPEEN);
	gpio_mode_setup(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11);
	gpio_set_af(GPIOE, GPIO_AF1, GPIO11);

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

mstate_t freq_get_state(void) {
	return m_state;
}

void freq_capture_start(void) {
	m_state = PENDING; /* start new measurement */
}

double freq_get_result(void) {
	//printf("Cycles during measurement: %" PRId32 ", no. of overflows: %" PRId32 ", overcapture: %" PRId32"\n", cycles_during_measurement, overflow_counter, overcapture);
	double avg_period = (double) cycles_during_measurement /
		(((double) TIMER1_HZ / (double) (TIMER1_PRESCALER+1)) * edges);
//	printf("Period: %1.6f, Frequency: %f, delta %3.0f mHz\n", avg_period, 1/(avg_period), (1/(avg_period)-50)*1000);
	return 1/avg_period;
}
