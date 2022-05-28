/**
 * @file    MKL25Z4_Project.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL25Z4.h"
#include "fsl_debug_console.h"

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include "led_column_driver.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Task priorities. */
#define led_column_driver_task_PRIORITY (configMAX_PRIORITIES - 1)
// TODO determine actual stack size with uxTaskGetStackHighWaterMark()
#define led_column_driver_task_STACK_SIZE (configMINIMAL_STACK_SIZE + 512)

#define renderer_task_PRIORITY 1
// TODO determine actual stack size with uxTaskGetStackHighWaterMark()
#define renderer_task_STACK_SIZE (configMINIMAL_STACK_SIZE + 80)

void rendererTask(void *pvParameters);

/*
 * @brief   Application entry point.
 */
int main(void) {

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    PRINTF("ASC 333 Controller - Enrico Gueli 2022\r\n");

    ledColumnDriverInit();

    xTaskCreate(
    		ledColumnDriverTask,
			"LED_column_driver_task",
			led_column_driver_task_STACK_SIZE,
			NULL,
			led_column_driver_task_PRIORITY,
			NULL
	);

    xTaskCreate(
    		rendererTask,
			"renderer_task",
			renderer_task_STACK_SIZE,
			NULL,
			renderer_task_PRIORITY,
			NULL
	);


    vTaskStartScheduler();
    for (;;)
        ;
}

void rendererTask(void *pvParameters) {
	uint32_t frame = 0;

	while (true) {
		// Turn on one pixel at a time
		const uint32_t xRed = (frame + 8) % (kLineSizeBytes * 8);
		const uint32_t xGreen = frame % (kLineSizeBytes * 8);

		color_image_t image;

		for (int row = 0; row < kNumRows; row++) {
			uint8_t * const redRow = image.red[row];
			uint8_t * const greenRow = image.green[row];

			// clear the row
			memset(redRow, 0, kLineSizeBytes);
			memset(greenRow, 0, kLineSizeBytes);

			// set just the bit corresponding to the pixel to turn on
			redRow[xRed >> 3] = 1 << (xRed % 8);
			greenRow[xGreen >> 3] = 1 << (xGreen % 8);
		}

		ledColumnDriverSendImage(&image);

		frame++;
		vTaskDelay(300 / portTICK_PERIOD_MS);
	}

	vTaskSuspend(NULL);
}
