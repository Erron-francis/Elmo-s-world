/*
 * FreeRTOS Kernel V11.1.0
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

/*
 * This is a simple main that will start the FreeRTOS-Kernel and run a periodic task
 * that only delays if compiled with the template port, this project will do nothing.
 * For more information on getting started please look here:
 * https://freertos.org/FreeRTOS-quick-start-guide.html
 */

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "uart.h"   // Hypothetical UART driver for SAMD21G18A
#include "gpio.h"   // Hypothetical GPIO driver for SAMD21G18A

/* Define task priorities */
const int task1_priority = 2;
const int task2_priority = 2;
const int uart_task_priority = 1;

/* Create a mutex for GPIO access */
SemaphoreHandle_t xMutex;

/* Task 1: Turn GPIO on, wait 0.5 seconds, and yield */
void vTask1(void *pvParameters)
{
	for( ;; )
	{
		/* Take the mutex to safely access the GPIO pin */
		if( xSemaphoreTake( xMutex, portMAX_DELAY ) == pdTRUE )
		{
			gpio_set_pin_level(LED_PIN, true); // Turn on the LED
			vTaskDelay(pdMS_TO_TICKS(500));    // Actively wait for 0.5 seconds

			xSemaphoreGive( xMutex );          // Release the mutex
		}

		taskYIELD(); // Yield to allow the next task to run
	}
}

/* Task 2: Turn GPIO off, delay for 1 second */
void vTask2(void *pvParameters)
{
	for( ;; )
	{
		/* Take the mutex to safely access the GPIO pin */
		if( xSemaphoreTake( xMutex, portMAX_DELAY ) == pdTRUE )
		{
			gpio_set_pin_level(LED_PIN, false); // Turn off the LED
			vTaskDelay(pdMS_TO_TICKS(1000));    // Delay for 1 second

			xSemaphoreGive( xMutex );           // Release the mutex
		}
	}
}

/* Task 3: Print status via UART every 1 second */
void vUARTTask(void *pvParameters)
{
	for( ;; )
	{
		uart_write_string("System is running...\r\n"); // Send a status message over UART
		vTaskDelay(pdMS_TO_TICKS(1000));              // Delay for 1 second
	}
}

int main(void)
{
	/* Initialize hardware */
	gpio_init();
	uart_init();

	/* Create the mutex for GPIO access */
	xMutex = xSemaphoreCreateMutex();

	if( xMutex != NULL )
	{
		/* Create tasks */
		xTaskCreate(vTask1, "LED On Task", configMINIMAL_STACK_SIZE, NULL, task1_priority, NULL);
		xTaskCreate(vTask2, "LED Off Task", configMINIMAL_STACK_SIZE, NULL, task2_priority, NULL);
		xTaskCreate(vUARTTask, "UART Task", configMINIMAL_STACK_SIZE, NULL, uart_task_priority, NULL);

		/* Start the scheduler */
		vTaskStartScheduler();
	}

	/* If the code reaches here, there was not enough memory to start the scheduler */
	for( ;; );
}
