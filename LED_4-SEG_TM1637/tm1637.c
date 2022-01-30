/*
 * tm1637.c
 *
 *  Created on: Jan 23, 2022
 *      Author: György
 */

#include "tm1637.h"

//
//      A
//     ---
//  F |   | B
//     -G-
//  E |   | C
//     ---
//      D
const uint8_t digitToSegment[] = {
 // XGFEDCBA
  0b00111111,    // 0
  0b00000110,    // 1
  0b01011011,    // 2
  0b01001111,    // 3
  0b01100110,    // 4
  0b01101101,    // 5
  0b01111101,    // 6
  0b00000111,    // 7
  0b01111111,    // 8
  0b01101111,    // 9
  0b01110111,    // A
  0b01111100,    // b
  0b00111001,    // C
  0b01011110,    // d
  0b01111001,    // E
  0b01110001     // F
  };

/*
 * Initialisation
 */
void TM1637_Init(TM1637_DEVICE *dev, GPIO_TypeDef *clk_Port, uint16_t clk_Pin, GPIO_TypeDef *dio_Port, uint16_t dio_Pin){
	dev->clk_Port = clk_Port;
	dev->clk_Pin = clk_Pin;
	dev->dio_Port = dio_Port;
	dev->dio_Pin = dio_Pin;
	dev->displayEnabled = 1;
	dev->brightness = 7;
	dev->leadingZero = 0;

	HAL_GPIO_WritePin(dio_Port, dio_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(clk_Port, clk_Pin, GPIO_PIN_SET);
}

/*
 * High-level functions
 */
void TM1637_showNumberDec(TM1637_DEVICE *dev, uint16_t num){
	uint8_t data[4];
	uint8_t zeroFlag = 0;
	for(uint8_t i = 0; i < 4; i++){
		data[3-i] = num % 10;
		num /= 10;
	}
	for(uint8_t i = 0; i < 4; i++){
		if(data[i] == 0 && dev->leadingZero == 0 && zeroFlag == 0){
			data[i] = 0x00;
		}else{
			data[i] = digitToSegment[data[i]];
			zeroFlag = 1;
		}
		num /= 10;
	}
	TM1637_setSegments(dev, data, 4, 0);
}
void TM1637_showNumberHex(TM1637_DEVICE *dev, uint16_t num){
	uint8_t data[4];
	uint8_t zeroFlag = 0;
	for(uint8_t i = 0; i < 4; i++){
		data[3-i] = num % 0x10;
		num /= 0x10;
	}
	for(uint8_t i = 0; i < 4; i++){
		if(data[i] == 0 && dev->leadingZero == 0 && zeroFlag == 0){
			data[i] = 0x00;
		}else{
			data[i] = digitToSegment[data[i]];
			zeroFlag = 1;
		}
		num /= 10;
	}
	TM1637_setSegments(dev, data, 4, 0);
}
void TM1637_showClock(TM1637_DEVICE *dev, uint8_t hour, uint8_t minute){
	uint8_t data[4];
	data[0] = digitToSegment[hour / 10];
	data[1] = digitToSegment[hour % 10];
	data[2] = digitToSegment[minute / 10];
	data[3] = digitToSegment[minute % 10];
	data[TM1637_DOTS_DIGIT] |= TM1637_DOTS_SEG;
	TM1637_setSegments(dev, data, 4, 0);
}

void TM1637_setBrightness(TM1637_DEVICE *dev, uint8_t brightness){
	dev->brightness = brightness & TM1637_BRGHT_MASK;
}
void TM1637_enableDisplay(TM1637_DEVICE *dev, uint8_t enable){
	dev->displayEnabled = enable ? 1 : 0;
}
void TM1637_enableLeadingZeroes(TM1637_DEVICE *dev, uint8_t enable){
	dev->leadingZero = enable ? 1 : 0;
}
void TM1637_clearDisplay(TM1637_DEVICE *dev){
	uint8_t data[4] = {TM1637_MINUS_SIGN, TM1637_MINUS_SIGN, TM1637_MINUS_SIGN, TM1637_MINUS_SIGN};
	TM1637_setSegments(dev, data, TM1637_DIGIT_COUNT, 0);
}

void TM1637_setSegments(TM1637_DEVICE *dev, uint8_t *data, uint8_t length, uint8_t pos){
	if(pos + length > TM1637_DIGIT_COUNT){
		length = TM1637_DIGIT_COUNT - pos;
	}
	TM1637_writeStart(dev);
	TM1637_writeByte(dev, TM1637_DATA_CMD);
	TM1637_writeStop(dev);

	TM1637_writeStart(dev);
	TM1637_writeByte(dev, TM1637_ADDR_CMD + (pos & TM1637_DIGIT_MASK));
	for(uint8_t i = 0; i < length; i++){
		TM1637_writeByte(dev, data[i]);
	}
	TM1637_writeStop(dev);

	TM1637_writeStart(dev);
	TM1637_writeByte(dev, TM1637_DISP_CTRL + (dev->brightness & TM1637_BRGHT_MASK) + (dev->displayEnabled ? TM1637_DISP_ON : 0x00));
	TM1637_writeStop(dev);
}

/*
 * Low-level functions
 */
void TM1637_writeStart(TM1637_DEVICE *dev){
	HAL_GPIO_WritePin(dev->dio_Port, dev->dio_Pin, GPIO_PIN_RESET);
	TM1637_bitDelay();
}
uint8_t TM1637_writeByte(TM1637_DEVICE *dev, uint8_t data){
	uint8_t ack = 0;
	for(uint8_t i = 0; i < 8; i++){
		HAL_GPIO_WritePin(dev->clk_Port, dev->clk_Pin, GPIO_PIN_RESET);	// pull CLK to LOW
		TM1637_bitDelay();
		HAL_GPIO_WritePin(dev->dio_Port, dev->dio_Pin, (data & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);	// set DIO to data bit value, start with LSB
		TM1637_bitDelay();
		HAL_GPIO_WritePin(dev->clk_Port, dev->clk_Pin, GPIO_PIN_SET);	// release CLK to HIGH
		TM1637_bitDelay();
		data >>= 1;	// shift data right for next bit
	}
	HAL_GPIO_WritePin(dev->clk_Port, dev->clk_Pin, GPIO_PIN_RESET);	// pull CLK to LOW
	TM1637_bitDelay();
	HAL_GPIO_WritePin(dev->dio_Port, dev->dio_Pin, GPIO_PIN_SET);		// release DIO to HIGH, to let the TM1637 output it's ACK
	TM1637_bitDelay();
	ack = HAL_GPIO_ReadPin(dev->dio_Port, dev->dio_Pin);
	HAL_GPIO_WritePin(dev->dio_Port, dev->dio_Pin, GPIO_PIN_RESET);	// pull DIO to LOW
	TM1637_bitDelay();
	HAL_GPIO_WritePin(dev->clk_Port, dev->clk_Pin, GPIO_PIN_SET);		// release CLK to HIGH
	TM1637_bitDelay();
	HAL_GPIO_WritePin(dev->clk_Port, dev->clk_Pin, GPIO_PIN_RESET);	// pull CLK to LOW
	TM1637_bitDelay();
	return ack;
}
void TM1637_writeStop(TM1637_DEVICE *dev){
	HAL_GPIO_WritePin(dev->dio_Port, dev->dio_Pin, GPIO_PIN_RESET);
	TM1637_bitDelay();
	HAL_GPIO_WritePin(dev->clk_Port, dev->clk_Pin, GPIO_PIN_SET);
	TM1637_bitDelay();
	HAL_GPIO_WritePin(dev->dio_Port, dev->dio_Pin, GPIO_PIN_SET);
	TM1637_bitDelay();
}

void TM1637_bitDelay(void){
	HAL_Delay(1);
}

