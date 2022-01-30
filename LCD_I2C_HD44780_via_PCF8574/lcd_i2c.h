/*
 * lcd_i2c.h
 *  LCD I2C driver for STM32F1xx
 *  Created on: Nov 6, 2021
 *      Author: György
 */

#ifndef INC_LCD_I2C_H_
#define INC_LCD_I2C_H_

#if defined (STM32F103xB)
#include "stm32f1xx_hal.h" /* needed for I2C */
#elif defined (STM32F411xE)
#include "stm32f4xx_hal.h" /* needed for I2C */
#elif defined (STM32L476xx)
#include "stm32l4xx_hal.h" /* needed for I2C */
#elif defined (STM32G474xx)
#include "stm32g4xx_hal.h" /* needed for I2C */
#endif

/* defines */
#define PCF8574_I2C_ADDRESS 	(0x20 << 1)
#define PCF8574A_I2C_ADDRESS 	(0x38 << 1)
#define HD44780_PIN_RS			0
#define HD44780_PIN_RW			1
#define HD44780_PIN_E			2
#define HD44780_PIN_BL			3
#define HD44780_PIN_DATA		4

/* lcd instructions */
#define HD44780_INSTR_CLEAR_DISPLAY		0x01
#define HD44780_INSTR_RETURN_HOME		0x02
#define HD44780_INSTR_ENTRY_MODE		0x04
#define HD44780_BIT_INCREMENT_ADDR		0x02	// 0: decrement
#define HD44780_BIT_AUTO_DISPLAY_SHIFT	0x01	// 0: no auto display shift
#define HD44780_INSTR_DISPLAY_CONTROL	0x08
#define HD44780_BIT_DISPLAY_ON			0x04	// 0: display off
#define HD44780_BIT_CURSOR_ON			0x02	// 0: cursor off
#define HD44780_BIT_CURSOR_BLINK		0x01	// 0: cursor steady
#define HD44780_INSTR_CURSOR_SHIFT		0x10
#define HD44780_BIT_DISPLAY_SHIFT		0x08	// 0: cursor move
#define HD44780_BIT_SHIFT_RIGHT			0x04	// 0: shift left
#define HD44780_INSTR_FUNCTION_SET		0x20
#define HD44780_BIT_8_BIT_BUS			0x10	// 0: 4-bit bus
#define HD44780_BIT_2_LINES				0x08	// 0: 1 line
#define HD44780_BIT_5x10_DOTS			0x04	// 0: 5x7 dots
#define HD44780_INSTR_SET_CG_ADDRESS	0x40
#define HD44780_BIT_CG_ADDRESS_MASK		0x3F
#define HD44780_INSTR_SET_DD_ADDRESS	0x80
#define HD44780_BIT_DD_ADDRESS_MASK		0x7F

/*
 * DEVICE STRUCT
 */
typedef struct LCD{
	/* I2C handle */
	I2C_HandleTypeDef *i2cHandle;

	/* I2C address */
	uint8_t address;

	/* state */
	uint8_t controlBits;	// DCB display, cursor, blink
	uint8_t backlight;

} LCD_I2C_DEVICE;

/*
 * HIGH-LEVEL
 * address: A2, A1, A0 pins (0-7)
 */
HAL_StatusTypeDef LCD_I2C_Init(LCD_I2C_DEVICE *dev, I2C_HandleTypeDef *i2cHandle, uint8_t address);
HAL_StatusTypeDef LCD_I2C_clear(LCD_I2C_DEVICE *dev, uint8_t data);
HAL_StatusTypeDef LCD_I2C_displayOn(LCD_I2C_DEVICE *dev);
HAL_StatusTypeDef LCD_I2C_displayOff(LCD_I2C_DEVICE *dev);
HAL_StatusTypeDef LCD_I2C_setCursorHome(LCD_I2C_DEVICE *dev);
HAL_StatusTypeDef LCD_I2C_setCursorPos(LCD_I2C_DEVICE *dev, uint8_t column, uint8_t row);
HAL_StatusTypeDef LCD_I2C_print(LCD_I2C_DEVICE *dev, char *string);
HAL_StatusTypeDef LCD_I2C_cursorOn(LCD_I2C_DEVICE *dev);
HAL_StatusTypeDef LCD_I2C_cursorOff(LCD_I2C_DEVICE *dev);
HAL_StatusTypeDef LCD_I2C_blinkOn(LCD_I2C_DEVICE *dev);
HAL_StatusTypeDef LCD_I2C_blinkOff(LCD_I2C_DEVICE *dev);
HAL_StatusTypeDef LCD_I2C_backlightOn(LCD_I2C_DEVICE *dev);
HAL_StatusTypeDef LCD_I2C_backlightOff(LCD_I2C_DEVICE *dev);
HAL_StatusTypeDef LCD_I2C_createChar(LCD_I2C_DEVICE *dev, uint8_t charNum, uint8_t *data);

/*
 * LOW-LEVEL
 */
HAL_StatusTypeDef PCF8574_write(LCD_I2C_DEVICE *dev, uint8_t data);
HAL_StatusTypeDef LCD_I2C_writeCommand(LCD_I2C_DEVICE *dev, uint8_t data);
HAL_StatusTypeDef LCD_I2C_writeCharacter(LCD_I2C_DEVICE *dev, uint8_t data);

#endif /* INC_LCD_I2C_H_ */
