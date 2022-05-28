/*
 * led_column_driver.c
 *
 *  Created on: 21 May 2022
 *      Author: ris8a
 */

#include "led_column_driver.h"


/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_debug_console.h"
#include "fsl_spi.h"
#include "fsl_spi_freertos.h"
#include "fsl_gpio.h"

#include "pin_mux.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

// The column shift registers are connected to SPI0.
#define COLUMN_DRIVER_SPI_MASTER_BASE (SPI0_BASE)
#define COLUMN_DRIVER_SPI_MASTER_IRQN (SPI0_IRQn)
#define SPI_MASTER_CLK_SRC (SPI0_CLK_SRC)
#define SPI_MASTER_CLK_FREQ CLOCK_GetFreq((SPI0_CLK_SRC))

#define COLUMN_DRIVER_SPI_MASTER_BASEADDR ((SPI_Type *)COLUMN_DRIVER_SPI_MASTER_BASE)
#define SPI_NVIC_PRIO 2

enum color { GREEN, RED };
typedef enum color color_t;

/**
 * The amount of bytes needed to represent an entire row of single-color pixels.
 * This many bytes will be sent to the LED column shift registers.
 */
const size_t kLineSizeBytes = 11;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static bool initSpi();
static void sendColumnData();

static void beginStrobe();
static void endStrobe();

static void displayRow(const color_t color);
static void activateRow(const int row, const color_t color);
static void sendRowData(const bool q0, const bool q1, const bool q2, const bool q3Green, const bool q3Red);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static spi_rtos_handle_t master_rtos_handle;


/*!
 * @brief Task responsible for controlling the LED column drivers.
 */
void led_column_driver_task(void *pvParameters)
{
	if(!initSpi()) {
        vTaskSuspend(NULL);
        return;
	}

	while(true) {
		displayRow(RED);
		displayRow(GREEN);
	}
	vTaskSuspend(NULL);
}

static void displayRow(const color_t color) {
	for (int row = 0; row < 7; row++) {
		beginStrobe();
		sendColumnData();
		activateRow(row, color);
		endStrobe();

		// Keep the row lighted with that data for 1ms
	    vTaskDelay(portTICK_PERIOD_MS);
	}
}

static bool initSpi() {
    NVIC_SetPriority(COLUMN_DRIVER_SPI_MASTER_IRQN, SPI_NVIC_PRIO);

    /*
     * masterConfig.enableStopInWaitMode = false;
     * masterConfig.polarity = kSPI_ClockPolarityActiveHigh;
     * masterConfig.phase = kSPI_ClockPhaseFirstEdge;
     * masterConfig.direction = kSPI_MsbFirst;
     * masterConfig.dataMode = kSPI_8BitMode;
     * masterConfig.txWatermark = kSPI_TxFifoOneHalfEmpty;
     * masterConfig.rxWatermark = kSPI_RxFifoOneHalfFull;
     * masterConfig.pinMode = kSPI_PinModeNormal;
     * masterConfig.outputMode = kSPI_SlaveSelectAutomaticOutput;
     * masterConfig.baudRate_Bps = 500000U;
     */
    spi_master_config_t masterConfig;
    SPI_MasterGetDefaultConfig(&masterConfig);
    masterConfig.polarity = kSPI_ClockPolarityActiveLow;
    masterConfig.phase = kSPI_ClockPhaseSecondEdge;
    masterConfig.direction = kSPI_LsbFirst;
    masterConfig.baudRate_Bps = 400000;

    uint32_t sourceClock = SPI_MASTER_CLK_FREQ;
    status_t status;
    status = SPI_RTOS_Init(&master_rtos_handle, COLUMN_DRIVER_SPI_MASTER_BASEADDR, &masterConfig, sourceClock);

    if (status != kStatus_Success)
    {
        PRINTF("DSPI master: error during initialization. \r\n");
        return false;
    }

    return true;
}

static void sendColumnData() {
	uint8_t txBuff[kLineSizeBytes];
	uint8_t rxBuff[kLineSizeBytes]; // won't use this one

    spi_transfer_t masterXfer = {
    		.txData = txBuff,
			.rxData = rxBuff,
			.dataSize = kLineSizeBytes
    };

    TickType_t ticks = xTaskGetTickCount();
    uint32_t frame = ticks / 300;

    /* Init Buffer */
    memset(txBuff, 0, kLineSizeBytes);
    // Turn on one pixel at a time
    uint32_t x = frame % (kLineSizeBytes * 8);
	PRINTF("x: %d\r\n", x);
    txBuff[x >> 3] = 1 << (x % 8);

	status_t status = SPI_RTOS_Transfer(&master_rtos_handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("SPI transfer completed with error. \r\n");
    }
}

static void beginStrobe() {
    GPIO_WritePinOutput(BOARD_INITPINS_COLUMN_STROBE_GPIO, BOARD_INITPINS_COLUMN_STROBE_PIN, 0);
}

static void endStrobe() {
    GPIO_WritePinOutput(BOARD_INITPINS_COLUMN_STROBE_GPIO, BOARD_INITPINS_COLUMN_STROBE_PIN, 1);
}

/*!
 * @brief Activates the specified row and color of the display.
 */
static void activateRow(const int row, const color_t color) {
	const int rowBits = (color == RED) ? row : ((row + 1) % 7);
	sendRowData(
		(rowBits & (1 << 0)) != 0,
		(rowBits & (1 << 1)) != 0,
		(rowBits & (1 << 2)) != 0,
        !(color == GREEN),
		!(color == RED)
	);
}

static void sendRowData(const bool q0, const bool q1, const bool q2, const bool q3Green, const bool q3Red) {
	GPIO_WritePinOutput(BOARD_INITPINS_ROW_Q0_GPIO, BOARD_INITPINS_ROW_Q0_PIN, q0);
	GPIO_WritePinOutput(BOARD_INITPINS_ROW_Q1_GPIO, BOARD_INITPINS_ROW_Q1_PIN, q1);
	GPIO_WritePinOutput(BOARD_INITPINS_ROW_Q2_GPIO, BOARD_INITPINS_ROW_Q2_PIN, q2);
	GPIO_WritePinOutput(BOARD_INITPINS_ROW_Q3_GREEN_GPIO, BOARD_INITPINS_ROW_Q3_GREEN_PIN, q3Green);
	GPIO_WritePinOutput(BOARD_INITPINS_ROW_Q3_RED_GPIO, BOARD_INITPINS_ROW_Q3_RED_PIN, q3Red);
}
