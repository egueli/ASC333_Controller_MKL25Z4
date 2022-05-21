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

/* TODO: insert other definitions and declarations here. */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void master_task(void *pvParameters);

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Task priorities. */
#define master_task_PRIORITY (configMAX_PRIORITIES - 1)

/*
 * @brief   Application entry point.
 */
int main(void) {

	/*
	 * TODO:
	 * - Drive column lines via SPI0 (FreeRTOS)
	 * - Drive row lines via GPIO (busy loop)
	 * - Drive row lines via GPIO (FreeRTOS)
	 * - Display fixed image
	 * - Get image data via SPI
	 */

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    PRINTF("ASC 333 Controller - Enrico Gueli 2022\n");

    xTaskCreate(master_task, "Master_task", configMINIMAL_STACK_SIZE + 38, NULL, master_task_PRIORITY, NULL);

    vTaskStartScheduler();
    for (;;)
        ;
}

/*!
 * @brief Task responsible for master SPI communication.
 */
static void master_task(void *pvParameters)
{
    PRINTF("Hi, I'm the master task. I will do absolutely anything. And now I leave.\n");

	vTaskSuspend(NULL);
}
