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

    xTaskCreate(
    		led_column_driver_task,
			"LED_column_driver_task",
			led_column_driver_task_STACK_SIZE,
			NULL,
			led_column_driver_task_PRIORITY,
			NULL
	);

    vTaskStartScheduler();
    for (;;)
        ;
}


