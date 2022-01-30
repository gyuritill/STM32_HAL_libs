/*
 * tm1637.h
 *
 *  Created on: Jan 23, 2022
 *      Author: György
 *
 *      The DIO and CLK pins shall be configured as open-drain outputs
 *      The TM1637 module has 10k pull-ups on both lines
 */

#ifndef INC_TM1637_H_
#define INC_TM1637_H_

#if defined (STM32F103xB)
#include "stm32f1xx_hal.h" /* needed for GPIO */
#elif defined (STM32F411xE)
#include "stm32f4xx_hal.h" /* needed for GPIO */
#elif defined (STM32L476xx)
#include "stm32l4xx_hal.h" /* needed for GPIO */
#elif defined (STM32G474xx)
#include "stm32g4xx_hal.h" /* needed for GPIO */
#endif

/* defines */
#define TM1637_DATA_CMD		0x40
#define TM1637_ADDR_CMD		0xC0
#define TM1637_DISP_CTRL	0x80
#define TM1637_DISP_ON		0x08

#define TM1637_MINUS_SIGN	0x40	// segment "G"
#define TM1637_DOTS_DIGIT	0x01	// digit 2 (from left)
#define TM1637_DOTS_SEG		0x80	// segment "DP"
#define TM1637_BRGHT_MASK	0x07
#define TM1637_DIGIT_MASK	0x03
#define TM1637_DIGIT_COUNT	0x04
#define TM1637_CMD_MASK		0x0F

/*
 * DEVICE STRUCT
 */
typedef struct TM1637_LED{
	GPIO_TypeDef *clk_Port;		// CLK
	uint16_t clk_Pin;
	GPIO_TypeDef *dio_Port;		// DIO
	uint16_t dio_Pin;
	uint8_t displayEnabled;		// 0: off, 1: on
	uint8_t	brightness;			// 0..7 -> 1/16, 2/16, 4/16, 10/16, 11/16, 12/16, 13/16, 14/16
	uint8_t leadingZero;		// 0: without leading zeroes, 1: with leading zeroes
} TM1637_DEVICE;

/*
 * Initialisation
 */
void TM1637_Init(TM1637_DEVICE *dev, GPIO_TypeDef *clk_Port, uint16_t clk_Pin, GPIO_TypeDef *dio_Port, uint16_t dio_Pin);

/*
 * High-level functions
 */
void TM1637_showNumberDec(TM1637_DEVICE *dev, uint16_t num);
void TM1637_showNumberHex(TM1637_DEVICE *dev, uint16_t num);
void TM1637_showClock(TM1637_DEVICE *dev, uint8_t hour, uint8_t minute);

void TM1637_setBrightness(TM1637_DEVICE *dev, uint8_t brightness);
void TM1637_enableDisplay(TM1637_DEVICE *dev, uint8_t enable);
void TM1637_enableLeadingZeroes(TM1637_DEVICE *dev, uint8_t enable);
void TM1637_clearDisplay(TM1637_DEVICE *dev);

void TM1637_setSegments(TM1637_DEVICE *dev, uint8_t *data, uint8_t length, uint8_t pos);

/*
 * Low-level functions
 */
void TM1637_writeStart(TM1637_DEVICE *dev);
uint8_t TM1637_writeByte(TM1637_DEVICE *dev, uint8_t data);
void TM1637_writeStop(TM1637_DEVICE *dev);
void TM1637_bitDelay(void);

#endif /* INC_TM1637_H_ */
