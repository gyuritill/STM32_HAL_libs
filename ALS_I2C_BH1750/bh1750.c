/*
 * bh1750.c
 *
 *  Created on: Feb 4, 2022
 *      Author: György
 */

#include "bh1750.h"

/*
 * Initialisation
 */
void BH1750_Init(BH1750_DEVICE *dev, I2C_HandleTypeDef *i2cHandle, uint8_t addrHL){
	dev->i2cHandle = i2cHandle;
	dev->address = addrHL ? BH1750_ADDR_HIGH : BH1750_ADDR_LOW;
	dev->mode = BH1750_RES1;
	dev->measTime = BH1750_MT_DEFAULT;
}
/*
 * High-level functions
 */
void BH1750_powerDown(BH1750_DEVICE *dev){
	uint8_t data = BH1750_POWER_DOWN;
	BH1750_writeCommand(dev, &data);
}
void BH1750_powerOn(BH1750_DEVICE *dev){
	uint8_t data = BH1750_POWER_ON;
	BH1750_writeCommand(dev, &data);
}
void BH1750_reset(BH1750_DEVICE *dev){
	uint8_t data = BH1750_RESET;
	BH1750_writeCommand(dev, &data);
}
void BH1750_startContHighRes1(BH1750_DEVICE *dev){
	uint8_t data = BH1750_CONT_HRES1;
	BH1750_writeCommand(dev, &data);
	dev->mode = BH1750_RES1;
}
void BH1750_startContHighRes2(BH1750_DEVICE *dev){
	uint8_t data = BH1750_CONT_HRES2;
	BH1750_writeCommand(dev, &data);
	dev->mode = BH1750_RES2;
}
void BH1750_startContLowRes(BH1750_DEVICE *dev){
	uint8_t data = BH1750_CONT_LRES;
	BH1750_writeCommand(dev, &data);
	dev->mode = BH1750_RES1;
}
void BH1750_startOnceHighRes1(BH1750_DEVICE *dev){
	uint8_t data = BH1750_ONCE_HRES1;
	BH1750_writeCommand(dev, &data);
	dev->mode = BH1750_RES1;
}
void BH1750_startOnceHighRes2(BH1750_DEVICE *dev){
	uint8_t data = BH1750_ONCE_HRES2;
	BH1750_writeCommand(dev, &data);
	dev->mode = BH1750_RES2;
}
void BH1750_startOnceLowRes(BH1750_DEVICE *dev){
	uint8_t data = BH1750_ONCE_LRES;
	BH1750_writeCommand(dev, &data);
	dev->mode = BH1750_RES1;
}
void BH1750_setMeasTime(BH1750_DEVICE *dev, uint16_t ms){
	uint16_t measTime;
	measTime = ms / BH1750_MT_DEF_MS * (float)BH1750_MT_DEFAULT;
	if(measTime < BH1750_MT_MIN){
		measTime = BH1750_MT_MIN;
	}
	if(measTime > BH1750_MT_MAX){
		measTime = BH1750_MT_MAX;
	}
	uint8_t data;
	data = BH1750_MT_HIGH_CMD;
	data |= (measTime >> BH1750_MT_HIGH_SHFT) & BH1750_MT_HIGH_MASK;
	BH1750_writeCommand(dev, &data);
	data = BH1750_MT_LOW_CMD;
	data |= measTime & BH1750_MT_LOW_MSK;
	BH1750_writeCommand(dev, &data);
	dev->measTime = measTime;
}

float BH1750_readBrightness(BH1750_DEVICE *dev){
	uint16_t data;
	float brightness;
	BH1750_readValue(dev, &data);
	if(dev->mode == BH1750_RES1){
		brightness = data / 1.2 * BH1750_MT_DEFAULT / dev->measTime;
	}else{
		brightness = data / 2.4 * BH1750_MT_DEFAULT / dev->measTime;
	}
	return brightness;
}
/*
 * LOW-LEVEL I2C
 */
HAL_StatusTypeDef BH1750_readValue(BH1750_DEVICE *dev, uint16_t *data16){
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t data8[2];
	status |= HAL_I2C_Master_Receive(dev->i2cHandle, dev->address << 1, data8, 2, HAL_MAX_DELAY);
	*data16 = data8[0] << 8 | data8[1];
	return status;
}
HAL_StatusTypeDef BH1750_writeCommand(BH1750_DEVICE *dev, uint8_t *cmd){
	HAL_StatusTypeDef status = HAL_OK;
	status |= HAL_I2C_Master_Transmit(dev->i2cHandle, dev->address << 1, cmd, 1, HAL_MAX_DELAY);
	return status;
}

