/*
 * STDIOserial.c
 * Author : Erron Francis
 * ID: 816023346
 */ 

#include <atmel_start.h>
#include <driver_init.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

volatile uint32_t millis_counter = 0;// Counter for milliseconds

void TC3_Handler(void) {//Function Timer ISR to increment
	if (TC3->COUNT16.INTFLAG.bit.OVF) { //check overflow flag is set
		millis_counter++;  
		TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_OVF;//clear overflow
	}
}


uint32_t time(void) {//function to get time
	return millis_counter;
}


void custom_sleep(uint32_t duration_ms) {//function sleep
	uint32_t start_time = time();
	while ((time() - start_time) < duration_ms) {
	}
	return;
}


void init_timer(void) {// Timer initiation
	PM->APBCMASK.reg |= PM_APBCMASK_TC3;//enable TC3
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(TC3_GCLK_ID) | GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0);//configuration generic clock
	while (GCLK->STATUS.bit.SYNCBUSY);
	TC3->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV64;// tc3 is set for normal mode
	TC3->COUNT16.CC[0].reg = 750;// using 48 MHz clock
	while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
	TC3->COUNT16.INTENSET.reg = TC_INTENSET_OVF;//enable overflow interrupt
	NVIC_EnableIRQ(TC3_IRQn);//enable tc3 irq in (nvic)
	TC3->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;//enable tc3
	while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
}

int main(void) {
	atmel_start_init();//initiation driver
	init_timer();// initiation timer
	struct io_descriptor *io;
	usart_sync_get_io_descriptor(&TARGET_IO, &io);
	stdio_io_init(io);
	usart_sync_enable(&TARGET_IO);
	io_write(io, (uint8_t*)"System Initialized.\r\n", 21);//print message
	while (1) {
		io_write(io, (uint8_t*)"Task Running...\r\n", 17);
		custom_sleep(0.6); 
	}
}