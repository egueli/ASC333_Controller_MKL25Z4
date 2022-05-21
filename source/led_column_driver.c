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

#define COLUMN_STROBE_GPIO (GPIOD)
#define COLUMN_STROBE_GPIO_PIN (2u)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static bool initSpi();
static void sendData();

static void initStrobe();
static void beginStrobe();
static void endStrobe();

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

	initStrobe();

	while(true) {
		beginStrobe();
		sendData();
		endStrobe();

		// Keep the row lighted with that data for 1ms
	    vTaskDelay(portTICK_PERIOD_MS);
	}
	vTaskSuspend(NULL);
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

static void sendData() {
	const size_t kBufferSize = 11; // that's how many bytes can give room to 83 bits
	uint8_t txBuff[kBufferSize];
	uint8_t rxBuff[kBufferSize]; // won't use this one

    spi_transfer_t masterXfer = {
    		.txData = txBuff,
			.rxData = rxBuff,
			.dataSize = kBufferSize
    };

    /* Init Buffer */
    for (uint8_t i = 0; i < kBufferSize; i++)
    {
    	txBuff[i] = 1u;
    }

	status_t status = SPI_RTOS_Transfer(&master_rtos_handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("SPI transfer completed with error. \r\n");
    }
}

static void initStrobe() {
    gpio_pin_config_t column_strobe_config = {
        kGPIO_DigitalOutput, 1,
    };

	GPIO_PinInit(COLUMN_STROBE_GPIO, COLUMN_STROBE_GPIO_PIN, &column_strobe_config);
}

static void beginStrobe() {
    GPIO_WritePinOutput(COLUMN_STROBE_GPIO, COLUMN_STROBE_GPIO_PIN, 0);
}

static void endStrobe() {
    GPIO_WritePinOutput(COLUMN_STROBE_GPIO, COLUMN_STROBE_GPIO_PIN, 1);
}
