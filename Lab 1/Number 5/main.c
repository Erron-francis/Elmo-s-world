/*
 * main.c
 * Author : Erron Francis
 * ID: 816023346
 */ 

#include <atmel_start.h>
#include <driver_init.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

volatile uint32_t millis_counter = 0;  // Counter

// Timer ISR to increment
void TC3_Handler(void) {
	if (TC3->COUNT16.INTFLAG.bit.OVF) {  // Check the overflow flag
		millis_counter++;  // Increment the counter
		TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_OVF;  // Clear the overflow interrupt flag
	}
}

// Function to get current time in milliseconds
uint32_t time(void) {
	return millis_counter;
}

// Sleep function using timer
void custom_sleep(uint32_t duration_ms) {
	uint32_t start_time = time();
	while ((time() - start_time) < duration_ms) {
	}
	return;
}

// Timer Initialization Function
void init_timer(void) {
	PM->APBCMASK.reg |= PM_APBCMASK_TC3;//Enable the peripheral clock for TC3

	// Configure generic clock for TC3
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(TC3_GCLK_ID) | GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0);
	while (GCLK->STATUS.bit.SYNCBUSY);

	// Set up TC3 for normal mode with prescaler
	TC3->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV64;
	TC3->COUNT16.CC[0].reg = (48000000 / 64) / 1000; // Assuming 48MHz clock, 1ms interrupt

	while (TC3->COUNT16.STATUS.bit.SYNCBUSY);

	// Enable overflow interrupt
	TC3->COUNT16.INTENSET.reg = TC_INTENSET_OVF;

	// Enable TC3 IRQ in NVIC
	NVIC_EnableIRQ(TC3_IRQn);

	// Enable TC3
	TC3->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
	while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
}

int main(void) {
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();

	// Initialize Timer
	init_timer();

	struct io_descriptor *io;
	usart_sync_get_io_descriptor(&TARGET_IO, &io);
	stdio_io_init(io);
	usart_sync_enable(&TARGET_IO);

	// Print a startup message
	io_write(io, (uint8_t*)"System Initialized.\r\n", 21);

	while (1) {
		// Capture the timestamp at which the task runs
		uint32_t timestamp = time();
		
		// Print a periodic message with timestamp
		char message[50];
		sprintf(message, "Task Running at %lu ms\r\n", timestamp);
		io_write(io, (uint8_t*)message, strlen(message));
		custom_sleep(1000);  // Sleep for 1000 milliseconds
	}
}
