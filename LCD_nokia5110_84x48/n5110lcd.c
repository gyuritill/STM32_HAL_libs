/*
 * n5110lcd.c
 *
 *  Created on: Nov 27, 2021
 *      Author: György
 */

#include "n5110lcd.h"

/*
 * Initialisation
 */
void N5110_Init_GPIO(N5110LCD_DEVICE *dev, GPIO_TypeDef *sce_Port, uint16_t sce_Pin, GPIO_TypeDef *rst_Port, uint16_t rst_Pin, GPIO_TypeDef *dc_Port,
	uint16_t dc_Pin, GPIO_TypeDef *din_Port, uint16_t din_Pin, GPIO_TypeDef *sclk_Port, uint16_t sclk_Pin, GPIO_TypeDef *led_Port, uint16_t led_Pin){
	dev->sce_Port = sce_Port;
	dev->sce_Pin = sce_Pin;
	dev->rst_Port = rst_Port;
	dev->rst_Pin = rst_Pin;
	dev->dc_Port = dc_Port;
	dev->dc_Pin = dc_Pin;
	dev->din_Port = din_Port;
	dev->din_Pin = din_Pin;
	dev->sclk_Port = sclk_Port;
	dev->sclk_Pin = sclk_Pin;
	dev->led_Port = led_Port;
	dev->led_Pin = led_Pin;
	HAL_GPIO_WritePin(dev->rst_Port, dev->rst_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(dev->rst_Port, dev->rst_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(dev->sce_Port, dev->sce_Pin, GPIO_PIN_SET);
	N5110_writeCommand(dev, CMD_FUNCTION_SET | 1 << CMD_FUNCTION_BIT_H);	// LCD extended commands
	N5110_writeCommand(dev, CMDX_SET_VOP | 0x38);							// set LCD Vop(Contrast) to 3.06V+0x38*0.06V=6.42V
	N5110_writeCommand(dev, CMDX_TEMP_CONTROL | CMDX_TC0_1MV_PER_K);		// set temperature coefficient to 1mV/K
	N5110_writeCommand(dev, CMDX_BIAS_SYSTEM | CMDX_BS3_1_TO_40);			// set LCD bias mode to 1:40
	N5110_writeCommand(dev, CMD_FUNCTION_SET | 0 << CMD_FUNCTION_BIT_H);	// LCD basic commands
	N5110_writeCommand(dev, CMD_DISPLAY_CONTROL | 1 << CMD_DISPLAY_D);		// LCD normal
	N5110_clearScreen(dev);
	dev->graphicMode = 1 << CMD_DISPLAY_D;
	dev->textMode = NORMAL_TEXT;
	dev->posX = 0;
	dev->posY = 0;
}

/*
 * High-level functions
 */
void N5110_switchOnScreen(N5110LCD_DEVICE *dev){
	dev->graphicMode |= 1 << CMD_DISPLAY_D;
	N5110_writeCommand(dev, CMD_DISPLAY_CONTROL | (dev->graphicMode & 0x5));
}

void N5110_switchOffScreen(N5110LCD_DEVICE *dev){
	dev->graphicMode &= ~(1 << CMD_DISPLAY_D);
	N5110_writeCommand(dev, CMD_DISPLAY_CONTROL | (dev->graphicMode & 0x5));
}

void N5110_positiveScreen(N5110LCD_DEVICE *dev){
	dev->graphicMode &= ~(1 << CMD_DISPLAY_E);
	N5110_writeCommand(dev, CMD_DISPLAY_CONTROL | (dev->graphicMode & 0x5));
}

void N5110_negativeScreen(N5110LCD_DEVICE *dev){
	dev->graphicMode |= 1 << CMD_DISPLAY_E;
	N5110_writeCommand(dev, CMD_DISPLAY_CONTROL | (dev->graphicMode & 0x5));
}

void N5110_positiveText(N5110LCD_DEVICE *dev){
	dev->textMode = 0;
}

void N5110_negativeText(N5110LCD_DEVICE *dev){
	dev->textMode = 1;
}

void N5110_putChar(N5110LCD_DEVICE *dev, char ch){
	for(uint8_t i = 0; i < FONT_WIDTH; i++){
		if(dev->textMode){
			N5110_writeData(dev, ~(ASCII[ch - ASCII_START][i]));	// inverted
		}else{
			N5110_writeData(dev, ASCII[ch - ASCII_START][i]);	// normal
		}
	}
}

void N5110_print(N5110LCD_DEVICE *dev, char *str){
  while(*str){
	  N5110_putChar(dev, *str++);
  }
}

void N5110_setPos(N5110LCD_DEVICE *dev, uint8_t x, uint8_t y){
	y += x / LCD_WIDTH;
	x %= LCD_WIDTH;
	y %= LCD_HEIGHT;
	N5110_writeCommand(dev, CMD_SET_X_ADDRESS | x);
	N5110_writeCommand(dev, CMD_SET_Y_ADDRESS | y);
}

void N5110_clearScreen(N5110LCD_DEVICE *dev){
	for(uint16_t i = 0; i < LCD_SIZE; i++){
		N5110_writeData(dev, 0x00);
	}
}

void N5110_backlightOn(N5110LCD_DEVICE *dev){
	HAL_GPIO_WritePin(dev->led_Port, dev->led_Pin, GPIO_PIN_SET);
}

void N5110_backlightOff(N5110LCD_DEVICE *dev){
	HAL_GPIO_WritePin(dev->led_Port, dev->led_Pin, GPIO_PIN_RESET);
}

/*
 * Low-level functions
 */
void N5110_writeCommand(N5110LCD_DEVICE *dev, uint8_t command){
	HAL_GPIO_WritePin(dev->dc_Port, dev->dc_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(dev->sce_Port, dev->sce_Pin, GPIO_PIN_RESET);
	for(uint8_t i=0; i<8; i++){
		HAL_GPIO_WritePin(dev->din_Port, dev->din_Pin, (command >> (7 - i)) & 0x1);
		HAL_GPIO_WritePin(dev->sclk_Port, dev->sclk_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dev->sclk_Port, dev->sclk_Pin, GPIO_PIN_RESET);
	}
	HAL_GPIO_WritePin(dev->sce_Port, dev->sce_Pin, GPIO_PIN_SET);
}
void N5110_writeData(N5110LCD_DEVICE *dev, uint8_t data){
	HAL_GPIO_WritePin(dev->dc_Port, dev->dc_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(dev->sce_Port, dev->sce_Pin, GPIO_PIN_RESET);
	for(uint8_t i=0; i<8; i++){
		HAL_GPIO_WritePin(dev->din_Port, dev->din_Pin, (data >> (7 - i)) & 0x1);
		HAL_GPIO_WritePin(dev->sclk_Port, dev->sclk_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dev->sclk_Port, dev->sclk_Pin, GPIO_PIN_RESET);
	}
	HAL_GPIO_WritePin(dev->sce_Port, dev->sce_Pin, GPIO_PIN_SET);
}
