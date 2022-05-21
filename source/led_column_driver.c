/*
 * led_column_driver.c
 *
 *  Created on: 21 May 2022
 *      Author: ris8a
 */

#include "led_column_driver.h"

#include "fsl_debug_console.h"

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"


/*!
 * @brief Task responsible for controlling the LED column drivers.
 */
void led_column_driver_task(void *pvParameters)
{
    PRINTF("Hi, I'm the task controlling the LED column drivers. I will do absolutely anything for now. And now I leave.\n");

	vTaskSuspend(NULL);
}
