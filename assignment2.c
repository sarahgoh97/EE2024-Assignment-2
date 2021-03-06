/*****************************************************************************
 *
 *
 *   Copyright(C) 2011, EE2024
 *   All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>

#include "lpc17xx.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_uart.h"

#include "oled.h"
#include "acc.h"
#include "light.h"
#include "pca9532.h"
#include "temp.h"
#include "led7seg.h"
#include "uart2.h"
#include "rgb.h"

volatile uint32_t msTicks;
uint8_t sw3 = 0;
uint8_t light = 0;

int startRed = 0;
int startBlue = 0;
int firstAcc = 1;
int firstTemp = 1;
int firstLight = 1;
int currentBlue = 0;
int currentRed = 0;
int currentLed = 0;
uint32_t launchTicks = 0;
uint32_t lightTicks = 0;
int counter = 0;
uint32_t redTicks = 0;
uint32_t blueTicks = 0;
uint32_t bothTicks = 0;
uint32_t curr = 0;

int launch = 0;
int returning = 0;
//to keep track of when entering different modes
int enterStationary = 1;
int enterLaunch = 1;
int enterReturn = 1;

//For the message for uart
uint32_t light_value = 0;
char lightMessage[40] = { };
char tempValue[15] = { };
char display_acc[40] = { };
char message[70] = { };
char stationaryOLED[] = "STATIONARY";
char stationaryUART[] = "Entering STATIONARY mode\r\n";
char launchOLED[] = "LAUNCH    ";
char launchUART[] = "Entering LAUNCH mode\r\n";
char returningOLED[] = "RETURN    ";
char returningUART[] = "Entering RETURN mode\r\n";
char obstacle[] = "Obstacle near";
char obstacleUART[] = "Obstacle near.\r\n";
char avoided[] = "Obstacle avoided.\r\n";
char veer[] = "Veer off course";
char veerUART[] = "Veer off course.\r\n";
char temp[] = "Temp. too high";
char tempUART[] = "Temp. too high.\r\n";
char clearOLED[] = "               ";

int indexUART = 0;
uint8_t report[6];
int receive = 0;

void SysTick_Handler(void) {
	msTicks++;
}

uint32_t getTicks(void) {
	return msTicks;
}

void init_i2c(void) {
	PINSEL_CFG_Type PinCfg;

	/* Initialize I2C2 pin connect for accelerometer */
	PinCfg.Funcnum = 2;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 10;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize I2C peripheral
	I2C_Init(LPC_I2C2, 100000);

	/* Enable I2C1 operation */
	I2C_Cmd(LPC_I2C2, ENABLE);
}

void init_ssp(void) {
	SSP_CFG_Type SSP_ConfigStruct;
	PINSEL_CFG_Type PinCfg;

	/*
	 * Initialize SPI pin connect
	 * P0.7 - SCK;
	 * P0.8 - MISO
	 * P0.9 - MOSI
	 * P2.2 - SSEL - used as GPIO
	 */
	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 7;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 8;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 9;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);

	SSP_ConfigStructInit(&SSP_ConfigStruct);

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP1, ENABLE);

}

void init_GPIO(void) {
	// Initialize button SW3 & SW4
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 10;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Portnum = 1;
	PinCfg.Pinnum = 31;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, 1 << 10, 0);
	GPIO_SetDir(1, 1 << 31, 0);
	// Initialize for RGB
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 26;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 0;
	PINSEL_ConfigPin(&PinCfg);
}

void oled_acc() {
	int8_t x, y, z;
	acc_read(&x, &y, &z);

	if (x > 3.92 || y > 3.92) {
		startBlue = 1;
	}
	sprintf(display_acc, "X:%2.2f Y:%2.2f  ", x / 9.8, y / 9.8);
	oled_putString(0, 10, display_acc, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

}

void lsla(void) {
	light_value = light_read();
	if (light_value <= 250) {
		pca9532_setLeds(0x1, 0xFFFF);
	} else if (light_value <= 500) {
		pca9532_setLeds(0x3, 0xFFFF);
	} else if (light_value <= 750) {
		pca9532_setLeds(0x7, 0xFFFF);
	} else if (light_value <= 1000) {
		pca9532_setLeds(0xF, 0xFFFF);
	} else if (light_value <= 1250) {
		pca9532_setLeds(0x1F, 0xFFFF);
	} else if (light_value <= 1500) {
		pca9532_setLeds(0x3F, 0xFFFF);
	} else if (light_value <= 1750) {
		pca9532_setLeds(0x7F, 0xFFFF);
	} else if (light_value <= 2000) {
		pca9532_setLeds(0xFF, 0xFFFF);
	} else if (light_value <= 2250) {
		pca9532_setLeds(0x1FF, 0xFFFF);
	} else if (light_value <= 2500) {
		pca9532_setLeds(0x3FF, 0xFFFF);
	} else if (light_value <= 2750) {
		pca9532_setLeds(0x7FF, 0xFFFF);
	} else if (light_value <= 3000) {
		pca9532_setLeds(0xFFF, 0xFFFF);
	} else if (light_value <= 3250) {
		pca9532_setLeds(0x1FFF, 0xFFFF);
	} else if (light_value <= 3500) {
		pca9532_setLeds(0x3FFF, 0xFFFF);
	} else if (light_value <= 3750) {
		pca9532_setLeds(0x7FFF, 0xFFFF);
	} else if (light_value <= 4000) {
		pca9532_setLeds(0xFFFF, 0xFFFF);
	}
	if (light_value < 500 || light_value > 3000) {
		light = 1;
	} else {
		light = 0;
	}
}

void tempSens(void) {
	//Initialize temperature
	SysTick_Config(SystemCoreClock / 1000);
	temp_init(getTicks);
	int32_t my_temp_value;
	my_temp_value = temp_read();

	double floatingTemp = my_temp_value / 10.0;
	if (floatingTemp > 30) {
		startRed = 1;
	}
	sprintf(tempValue, "Temp: %2.2f", floatingTemp);
	oled_putString(0, 20, tempValue, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
}

// EINT3 Interrupt Handler
void EINT3_IRQHandler(void) {
	// Determine whether SW3 has occurred
	if ((LPC_GPIOINT ->IO2IntStatF >> 10) & 0x1) {
		sw3 = 1;
		LPC_GPIOINT ->IO2IntClr = 1 << 10;
	}
	/*else if ((LPC_GPIOINT ->IO2IntStatF >> 5) & 0x1) {
		light = 1;
		light_clearIrqStatus();
		LPC_GPIOINT ->IO2IntClr = 1 << 5;
	}*/

}

void countdown(void) {
	char count[] = "FEDCBA9876543210";
	int i = 0;
	while (startRed == 0 && i < 16) {
		led7seg_setChar(count[i], 0);
		tempSens();
		Timer0_Wait(600);
		i++;
	}
	if (i < 16) {
		tempRedLight();
		launch = 0;
		led7seg_setChar(count[0], 0);
	}
}

void pinsel_uart3(void) {
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 0;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 1;
	PINSEL_ConfigPin(&PinCfg);
}

void UART_IntReceive(void)
{
    /* Read the received data */
    if(UART_Receive(LPC_UART3, &report[indexUART], 1, NONE_BLOCKING) == 1) {
        if(report[indexUART] == '\r'){
            receive = 1;
        }
        indexUART++;
        if(indexUART == 5) {
        	indexUART = 0;
        }
    }
}

void init_uart(void) {
	UART_CFG_Type uartCfg;
	UART_FIFO_CFG_Type uartFIFO;
	uartCfg.Baud_rate = 115200;
	uartCfg.Databits = UART_DATABIT_8;
	uartCfg.Parity = UART_PARITY_NONE;
	uartCfg.Stopbits = UART_STOPBIT_1;
	//pin select for uart3;
	pinsel_uart3();
	//supply power & setup working parameters for uart3
	UART_Init(LPC_UART3, &uartCfg);
	//enable transmit for uart3
	UART_TxCmd(LPC_UART3, ENABLE);
	//enable interrupt and buffer
	UART_IntConfig(LPC_UART3, UART_INTCFG_RBR, ENABLE);
	UART_FIFOConfigStructInit(&uartFIFO);
	UART_FIFOConfig(LPC_UART3, &uartFIFO);
	UART_SetupCbs(LPC_UART3, 0, (void *)UART_IntReceive);
}

void UART3_IRQHandler(void) { //from UART_StdIntHandler
	UART_GenIntHandler(LPC_UART3);
}

void sendUART(void) {
	sprintf(message, "%s ACC %s\r\n", tempValue, display_acc);
	UART_Send(LPC_UART3, (uint8_t *) message, strlen(message), BLOCKING);
}

void sendLightUART(void) {
	sprintf(lightMessage, "Obstacle distance: %d\r\n", light_value);
	UART_Send(LPC_UART3, (uint8_t *) lightMessage, strlen(lightMessage), BLOCKING);
}

void clearWarning() {
	startRed = 0;
	startBlue = 0;
	currentBlue = 0;
	currentRed = 0;
	currentLed = 0;
	rgb_setLeds(0);
	oled_putString(0, 40, clearOLED, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	oled_putString(0, 50, clearOLED, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	firstAcc = 1;
	firstTemp = 1;
	firstLight = 1;
}

//MODE_TOGGLE
void toggleSta(void) {
	//when stationary and no temp warning
	if (startRed == 0) {
		launch = 1;
		countdown();
	}
	sw3 = 0;
}

void toggleLau(void) {
	//when launching and press twice
	if ((getTicks() - curr) <= 1000) {
		returning = 1;
		launch = 0;
	}
	curr = getTicks();
	sw3 = 0;
}

void toggleRet(void) {
	returning = 0;
	sw3 = 0;
}

//WARNING LEDS
void accBlueLight(void) {
		oled_putString(0, 50, (void*) veer, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		if (getTicks() - blueTicks >= 333) {
			if (currentBlue == 0) {
				currentBlue = 2;
				rgb_setLeds(currentBlue);
			} else if (currentBlue == 2) {
				currentBlue = 0;
				rgb_setLeds(currentBlue);
			}
			blueTicks = getTicks();
		}
		if (firstAcc == 1) {
			UART_Send(LPC_UART3, (uint8_t *) veerUART, strlen(veerUART),
					BLOCKING);
			firstAcc = 0;
		}
}

void tempRedLight(void) {
		oled_putString(0, 40, (void*) temp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		if (getTicks() - redTicks >= 333) {
			if (currentRed == 0) {
				currentRed = 1;
				rgb_setLeds(currentRed);
			} else if (currentRed == 1) {
				currentRed = 0;
				rgb_setLeds(currentRed);
			}
			redTicks = getTicks();
		}
		if (firstTemp == 1) { //sends only first time past threshold
			UART_Send(LPC_UART3, (uint8_t *) tempUART, strlen(tempUART),
					BLOCKING);
			firstTemp = 0;
		}
}

void bothLights() {
	oled_putString(0, 40, (void*) temp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	oled_putString(0, 50, (void*) veer, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	if (getTicks() - bothTicks >= 333) {
		if (currentLed == 1 || currentLed == 0) {
			currentLed = 2;
			rgb_setLeds(currentLed);
		} else if (currentLed == 2) {
			currentLed = 1;
			rgb_setLeds(currentLed);
		}
		bothTicks = getTicks();
	}
}

//CLEAR WARNING
void clear(void) {
	clearWarning();
	oled_putString(0, 30, clearOLED, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
}

//OBSTACLE AVOIDANCE
void obstacleNear(void) {
	oled_putString(0, 30, (void*) obstacle, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	if (firstLight == 1) { //sends only first time past threshold
		UART_Send(LPC_UART3, (uint8_t *) obstacleUART, strlen(obstacleUART), BLOCKING);
		firstLight = 0;
	}
}

void obstacleAvoid(void) {
	oled_putString(0, 30, (void*) clearOLED, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	if (firstLight == 0) {
		UART_Send(LPC_UART3, (uint8_t *) avoided, strlen(avoided), BLOCKING);
		firstLight = 1;
	}
}

void initAll(void) {
	init_i2c();
	init_ssp();
	init_GPIO();

	//rgb blinking
	rgb_init();

	//light sensor with LED array
	light_init();
	light_enable();
	light_setRange(LIGHT_RANGE_4000);
	pca9532_init();
	led7seg_init();

	//accelerometer with OLED display
	acc_init();
	oled_init();

	//uart
	//uart2_init(115200, CHANNEL_A);
	init_uart();

	oled_clearScreen(OLED_COLOR_BLACK);

	// Enable GPIO Interrupt P2.10
	LPC_GPIOINT ->IO2IntEnF |= 1 << 10;

	NVIC_SetPriorityGrouping(5);
	NVIC_SetPriority(SysTick_IRQn, 0x0000);
	NVIC_SetPriority(EINT3_IRQn, 0x0040);
	NVIC_SetPriority(UART3_IRQn, 0x0080);

	//UART interrupt

	NVIC_ClearPendingIRQ(UART3_IRQn);
	NVIC_EnableIRQ(UART3_IRQn);

	NVIC_ClearPendingIRQ(EINT3_IRQn);
	// Enable EINT3 interrupt
	NVIC_EnableIRQ(EINT3_IRQn);

	rgb_setLeds(0);

}

void stationaryMode(void) {
	if (enterStationary == 1) { //first time entering stationary mode
		oled_clearScreen(OLED_COLOR_BLACK);
		led7seg_setChar('F', 0);
		oled_putString(0, 0, stationaryOLED, OLED_COLOR_WHITE,
				OLED_COLOR_BLACK);
		UART_Send(LPC_UART3, (uint8_t *) stationaryUART, strlen(stationaryUART),
				BLOCKING);
		enterStationary = 0;
		enterLaunch = 1;
		enterReturn = 1;
	}
	tempSens();
	if (startRed == 1) {
		tempRedLight();
	}
	if (sw3 == 1) {
		toggleSta();
	}
	int btn2 = (GPIO_ReadValue(1) >> 31) & 0x01;
	if (btn2 == 0) {
		clear();
	}
}

void launchMode(void) {
	if (enterLaunch == 1) {
		oled_clearScreen(OLED_COLOR_BLACK);
		oled_putString(0, 0, launchOLED, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		tempSens();
		oled_acc();
		UART_Send(LPC_UART3, (uint8_t *) launchUART, strlen(launchUART),
				BLOCKING);
		enterStationary = 1;
		enterReturn = 1;
		launchTicks = getTicks();
	}
	if (enterLaunch == 1 || getTicks() - launchTicks >= (uint32_t) 10000) {
		launchTicks = getTicks();
		sendUART();
		enterLaunch = 0;
	}
	if (receive == 1) {
		sendUART();
		receive = 0;
	}
	tempSens();
	oled_acc();
	if (startRed == 1 && startBlue == 0) { //only temp warning
		tempRedLight();
	} else if (startRed == 0 && startBlue == 1) { //only acc warning
		accBlueLight();
	} else if (startRed == 1 && startBlue == 1) { //both warnings
		bothLights();
	}
	if (sw3 == 1) {
		toggleLau();
	}
	int btn2 = (GPIO_ReadValue(1) >> 31) & 0x01;
	if (btn2 == 0) {
		clear();
	}
}

void returnMode(void) {
	if (enterReturn == 1) {
		oled_clearScreen(OLED_COLOR_BLACK);
		oled_putString(0, 0, returningOLED, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		clearWarning();
		lsla();
		UART_Send(LPC_UART3, (uint8_t *) returningUART, strlen(returningUART),
				BLOCKING);
		enterStationary = 1;
		enterLaunch = 1;
	}
	if (enterReturn == 1 || getTicks() - lightTicks >= (uint32_t) 10000) {
		lightTicks = getTicks();
		sendLightUART();
		enterReturn = 0;
	}
	//rptReturn();
	if (sw3 == 1) {
		toggleRet();
	}

	int btn2 = (GPIO_ReadValue(1) >> 31) & 0x01;
	if (btn2 == 0) {
		clear();
	}
	lsla();
	if (light == 1) {
		obstacleNear();
	} else {
		obstacleAvoid();
	}
	if (receive == 1) {
		sendLightUART();
		receive = 0;
	}
}

int main(void) {
	initAll();

	while (1) {
		if (launch == 0 && returning == 0) {
			stationaryMode();
		} else if (launch == 1 && returning == 0) {
			launchMode();
		} else if (launch == 0 && returning == 1) {
			returnMode();
		}

	}
}
