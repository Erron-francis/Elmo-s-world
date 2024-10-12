// Name: Erron Francis
// ID: 816023346
// Implementation of a debounce FSM for Serial RX using if-else

#include "atmel_start.h"
#include <hal_usart_sync.h>
#include <hal_io.h>
#include <stdbool.h>
#include <stdio.h>


// Function to redirect printf output to the USART
int _write(int file, char *ptr, int len) {
	io_write(&TARGET_IO.io, (uint8_t *)ptr, len);  // Redirects to your USART target
	return len;
}
// Enumeration for FSM states
typedef enum {
	STATE_OFF,
	STATE_ON
} State_t;

// Function prototypes
bool serial_char_received(void);
char get_received_char(void);

// Main FSM implementation
int main(void) {
	// Initialize Atmel Start system
	atmel_start_init();

	State_t state = STATE_OFF;  // Initial state
	char last_char = '\0';
	char current_char = '\0';
	uint16_t debounce_counter = 0;
	bool char_received = false;

	while (1) {
		// Check if a character has been received
		if (serial_char_received()) {
			current_char = get_received_char();
			char_received = true;
			debounce_counter = 500;  // Start debounce countdown (500 ms)
		}

		// State machine transitions using if-else
		if (state == STATE_OFF) {
			if (char_received && current_char == last_char && debounce_counter == 0) {
				state = STATE_ON;
				printf("FSM Transitioned to ON\r\n");
				char_received = false;
				} else if (char_received && current_char != last_char) {
				last_char = current_char;
				char_received = false;
			}
			} else if (state == STATE_ON) {
			if (char_received && current_char != last_char && debounce_counter == 0) {
				state = STATE_OFF;
				printf("FSM Transitioned to OFF\r\n");
				last_char = current_char;
				char_received = false;
			}
		}

		// Decrement debounce counter if active
		if (debounce_counter > 0) {
			delay_ms(1);  // Delay for 1 ms
			debounce_counter--;
		}
	}
}

// Function for checking if a serial character has been received
bool serial_char_received(void) {
	// Use the correct Atmel Start function to check for received data
	return usart_sync_is_rx_not_empty(&TARGET_IO);
}

// Function for getting the received character
char get_received_char(void) {
	// Use the correct Atmel Start function to read received data
	uint8_t received_char;
	io_read(&TARGET_IO.io, &received_char, 1);
	return (char)received_char;
}