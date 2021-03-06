/*
 * led_column_driver.h
 *
 *  Created on: 21 May 2022
 *      Author: Enrico Gueli
 */

#ifndef LED_COLUMN_DRIVER_H_
#define LED_COLUMN_DRIVER_H_

#include <stdint.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/**
 * The amount of bytes needed to represent an entire row of single-color pixels.
 * This many bytes will be sent to the LED column shift registers.
 */
#define kLineSizeBytes (11)

/**
 * How many rows the display have.
 */
#define kNumRows (7)

/**
 * An image of a single color that can be displayed. It is first split into rows,
 * then into columns, then into 8-bit pixels.
 */
typedef uint8_t image_t[kNumRows][kLineSizeBytes];

/**
 * A color image made of a red part and a green part.
 */
struct color_image {
	image_t red;
	image_t green;
};
typedef struct color_image color_image_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

void ledColumnDriverInit();

/**
 * The FreeRTOS task that displays images, to be used as argument of xTaskCreate.
 * It requires no parameters.
 */
void ledColumnDriverTask(void *pvParameters);

/**
 * Displays the specified image after the current one has been displayed.
 * If this function is called before the previous one had a chance to be displayed,
 * the previous one will be dropped.
 *
 * @param[in] image A pointer to the image. This pointer won't be used outside the
 * scope of this function.
 */
void ledColumnDriverSendImage(color_image_t const * image);


#endif /* LED_COLUMN_DRIVER_H_ */
