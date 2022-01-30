/*
 * lcd_i2c.c
 *
 *  Created on: Nov 6, 2021
 *      Author: György
 */

#include "lcd_i2c.h"

/*
 * HIGH-LEVEL
 * address: A2, A1, A0 pins (0-7)
 */
HAL_StatusTypeDef LCD_I2C_Init(LCD_I2C_DEVICE *dev, I2C_HandleTypeDef *i2cHandle, uint8_t address){
	HAL_StatusTypeDef status=HAL_OK;
	dev->i2cHandle = i2cHandle;
	dev->address = PCF8574_I2C_ADDRESS + (address << 1);
	dev->backlight = 1;
	dev->controlBits = 0b100;	// DCB display, cursor, blink
	HAL_Delay(15);
	status |= PCF8574_write(dev, 0x30 | 1<<HD44780_PIN_E);
	status |= PCF8574_write(dev, 0x30);
	HAL_Delay(5);
	status |= PCF8574_write(dev, 0x30 | 1<<HD44780_PIN_E);
	status |= PCF8574_write(dev, 0x30);
	HAL_Delay(1);
	status |= PCF8574_write(dev, 0x30 | 1<<HD44780_PIN_E);
	status |= PCF8574_write(dev, 0x30);
	HAL_Delay(1);
	status |= PCF8574_write(dev, 0x20 | 1<<HD44780_PIN_E);
	status |= PCF8574_write(dev, 0x20);
	HAL_Delay(1);
	// 4-bit, 2-lines, 5x7 pixel
	status |= LCD_I2C_writeCommand(dev, HD44780_INSTR_FUNCTION_SET | HD44780_BIT_2_LINES);
	HAL_Delay(1);
	// display off, cursor off, blink off
	status |= LCD_I2C_writeCommand(dev, HD44780_INSTR_DISPLAY_CONTROL);
	HAL_Delay(1);
	// display clear
	status |= LCD_I2C_writeCommand(dev, HD44780_INSTR_CLEAR_DISPLAY);
	HAL_Delay(1);
	// auto increment, no shift
	status |= LCD_I2C_writeCommand(dev, HD44780_INSTR_ENTRY_MODE | HD44780_BIT_INCREMENT_ADDR);
	HAL_Delay(1);
	// display on, cursor off, blink off
	status |= LCD_I2C_writeCommand(dev, HD44780_INSTR_DISPLAY_CONTROL | HD44780_BIT_DISPLAY_ON);
	HAL_Delay(1);
	// return cursor and LCD to home
	status |= LCD_I2C_writeCommand(dev, HD44780_INSTR_RETURN_HOME);
	HAL_Delay(2);
	return status;
}
HAL_StatusTypeDef LCD_I2C_clear(LCD_I2C_DEVICE *dev, uint8_t data){
	HAL_StatusTypeDef status=HAL_OK;
	status |= LCD_I2C_writeCommand(dev, HD44780_INSTR_CLEAR_DISPLAY);
	HAL_Delay(1);
	return status;
}
HAL_StatusTypeDef LCD_I2C_displayOn(LCD_I2C_DEVICE *dev){
	HAL_StatusTypeDef status=HAL_OK;
	status |= LCD_I2C_writeCommand(dev, (HD44780_INSTR_DISPLAY_CONTROL | dev->controlBits) | HD44780_BIT_DISPLAY_ON);
	HAL_Delay(1);
	if(status == HAL_OK){
		dev->controlBits |= HD44780_BIT_DISPLAY_ON;
	}
	return status;
}
HAL_StatusTypeDef LCD_I2C_displayOff(LCD_I2C_DEVICE *dev){
	HAL_StatusTypeDef status=HAL_OK;
	status |= LCD_I2C_writeCommand(dev, (HD44780_INSTR_DISPLAY_CONTROL | dev->controlBits) & ~HD44780_BIT_DISPLAY_ON);
	HAL_Delay(1);
	if(status == HAL_OK){
		dev->controlBits &= ~HD44780_BIT_DISPLAY_ON;
	}
	return status;
}
HAL_StatusTypeDef LCD_I2C_setCursorHome(LCD_I2C_DEVICE *dev){
	HAL_StatusTypeDef status=HAL_OK;
	status |= LCD_I2C_writeCommand(dev, HD44780_INSTR_RETURN_HOME);
	HAL_Delay(1);
	return status;
}
HAL_StatusTypeDef LCD_I2C_setCursorPos(LCD_I2C_DEVICE *dev, uint8_t column, uint8_t row){
	HAL_StatusTypeDef status=HAL_OK;
	uint8_t ddAddr = 0x40*(row & 0x1) + 20*((row & 0x2)>>1) + (column & 0x1F);
	status |= LCD_I2C_writeCommand(dev, HD44780_INSTR_SET_DD_ADDRESS | ddAddr);
	HAL_Delay(1);
	return status;
}
HAL_StatusTypeDef LCD_I2C_print(LCD_I2C_DEVICE *dev, char *string){
	HAL_StatusTypeDef status=HAL_OK;
	for(uint8_t i=0;i<255;i++){
		char ch = string[i];
		if(ch == '\0') break;
		status |= LCD_I2C_writeCharacter(dev, string[i]);
	}
	return status;
}
HAL_StatusTypeDef LCD_I2C_cursorOn(LCD_I2C_DEVICE *dev){
	HAL_StatusTypeDef status=HAL_OK;
	status |= LCD_I2C_writeCommand(dev, (HD44780_INSTR_DISPLAY_CONTROL | dev->controlBits) | HD44780_BIT_CURSOR_ON);
	HAL_Delay(1);
	if(status == HAL_OK){
		dev->controlBits |= HD44780_BIT_CURSOR_ON;
	}
	return status;
}
HAL_StatusTypeDef LCD_I2C_cursorOff(LCD_I2C_DEVICE *dev){
	HAL_StatusTypeDef status=HAL_OK;
	status |= LCD_I2C_writeCommand(dev, (HD44780_INSTR_DISPLAY_CONTROL | dev->controlBits) & ~HD44780_BIT_CURSOR_ON);
	HAL_Delay(1);
	if(status == HAL_OK){
		dev->controlBits &= ~HD44780_BIT_CURSOR_ON;
	}
	return status;
}
HAL_StatusTypeDef LCD_I2C_blinkOn(LCD_I2C_DEVICE *dev){
	HAL_StatusTypeDef status=HAL_OK;
	status |= LCD_I2C_writeCommand(dev, (HD44780_INSTR_DISPLAY_CONTROL | dev->controlBits) | HD44780_BIT_CURSOR_BLINK);
	HAL_Delay(1);
	if(status == HAL_OK){
		dev->controlBits |= HD44780_BIT_CURSOR_BLINK;
	}
	return status;
}
HAL_StatusTypeDef LCD_I2C_blinkOff(LCD_I2C_DEVICE *dev){
	HAL_StatusTypeDef status=HAL_OK;
	status |= LCD_I2C_writeCommand(dev, (HD44780_INSTR_DISPLAY_CONTROL | dev->controlBits) & ~HD44780_BIT_CURSOR_BLINK);
	HAL_Delay(1);
	if(status == HAL_OK){
		dev->controlBits &= ~HD44780_BIT_CURSOR_BLINK;
	}
	return status;
}
HAL_StatusTypeDef LCD_I2C_backlightOn(LCD_I2C_DEVICE *dev){
	HAL_StatusTypeDef status=HAL_OK;
	status |= PCF8574_write(dev, 1<<HD44780_PIN_BL);
	HAL_Delay(1);
	if(status == HAL_OK){
		dev->backlight = 1;
	}
	return status;
}
HAL_StatusTypeDef LCD_I2C_backlightOff(LCD_I2C_DEVICE *dev){
	HAL_StatusTypeDef status=HAL_OK;
	status |= PCF8574_write(dev, 0<<HD44780_PIN_BL);
	HAL_Delay(1);
	if(status == HAL_OK){
		dev->backlight = 0;
	}
	return status;
}
HAL_StatusTypeDef LCD_I2C_createChar(LCD_I2C_DEVICE *dev, uint8_t charNum, uint8_t *data){
	HAL_StatusTypeDef status=HAL_OK;
	status |= LCD_I2C_writeCommand(dev, HD44780_INSTR_SET_CG_ADDRESS | (charNum & 7) << 3);
	for(uint8_t i=0;i<7;i++){
		status |= LCD_I2C_writeCharacter(dev, (data[i] & 0x1F));
	}
	HAL_Delay(1);
	return status;
}

/*
 * LOW-LEVEL
 */
HAL_StatusTypeDef PCF8574_write(LCD_I2C_DEVICE *dev, uint8_t data){
	return HAL_I2C_Master_Transmit(dev->i2cHandle , dev->address, &data, 1, HAL_MAX_DELAY);
}
HAL_StatusTypeDef LCD_I2C_writeCommand(LCD_I2C_DEVICE *dev, uint8_t data){
	HAL_StatusTypeDef status=HAL_OK;
	status |= PCF8574_write(dev, (data & 0xF0) | (dev->backlight>0)<<HD44780_PIN_BL | 1<<HD44780_PIN_E);
	status |= PCF8574_write(dev, (data & 0xF0) | (dev->backlight>0)<<HD44780_PIN_BL);
	HAL_Delay(1);
	status |= PCF8574_write(dev, (data & 0x0F)<<4 | (dev->backlight>0)<<HD44780_PIN_BL | 1<<HD44780_PIN_E);
	status |= PCF8574_write(dev, (data & 0x0F)<<4 | (dev->backlight>0)<<HD44780_PIN_BL);
	HAL_Delay(1);
	return status;
}
HAL_StatusTypeDef LCD_I2C_writeCharacter(LCD_I2C_DEVICE *dev, uint8_t data){
	HAL_StatusTypeDef status=HAL_OK;
	status |= PCF8574_write(dev, (data & 0xF0) | (dev->backlight>0)<<HD44780_PIN_BL | 1<<HD44780_PIN_RS | 1<<HD44780_PIN_E);
	status |= PCF8574_write(dev, (data & 0xF0) | (dev->backlight>0)<<HD44780_PIN_BL | 1<<HD44780_PIN_RS);
	HAL_Delay(1);
	status |= PCF8574_write(dev, (data & 0x0F)<<4 | (dev->backlight>0)<<HD44780_PIN_BL | 1<<HD44780_PIN_RS | 1<<HD44780_PIN_E);
	status |= PCF8574_write(dev, (data & 0x0F)<<4 | (dev->backlight>0)<<HD44780_PIN_BL | 1<<HD44780_PIN_RS);
	HAL_Delay(1);
	return status;
}
