//Erron Francis_816023346
#include "sam.h"

int main(void) {
	int i;

	SystemInit();

	// Set pin 8 of PORT1 as output
	REG_PORT_DIR0 |= 1 << 20;

	while (1) {
		// Turn the LED on
		REG_PORT_OUT0 |= 1 << 20;
		for (i = 0; i < 100000; i++) {}  // Adjusted delay loop

		// Turn the LED off
		REG_PORT_OUT0 &= ~(1 << 20);
		for (i = 0; i < 100000; i++) {}  // Adjusted delay loop
	}
}