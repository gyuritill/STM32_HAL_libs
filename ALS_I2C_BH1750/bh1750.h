/*
 * bh1750.h
 *
 *  Created on: Feb 4, 2022
 *      Author: György
 */

#ifndef INC_BH1750_H_
#define INC_BH1750_H_

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
#define BH1750_ADDR_HIGH	0x5C
#define BH1750_ADDR_LOW		0x23
#define BH1750_POWER_DOWN	0x00
#define BH1750_POWER_ON		0x01
#define BH1750_RESET		0x03
#define BH1750_CONT_HRES1	0x10
#define BH1750_CONT_HRES2	0x11
#define BH1750_CONT_LRES	0x13
#define BH1750_ONCE_HRES1	0x20
#define BH1750_ONCE_HRES2	0x21
#define BH1750_ONCE_LRES	0x23
#define BH1750_MT_HIGH_CMD	0x40
#define BH1750_MT_HIGH_MASK	0x07
#define BH1750_MT_HIGH_SHFT	0x05
#define BH1750_MT_LOW_CMD	0x60
#define BH1750_MT_LOW_MSK	0x1F
#define BH1750_MT_MIN		31
#define BH1750_MT_DEFAULT	69
#define BH1750_MT_DEF_MS	120.0
#define BH1750_MT_MAX		254
#define BH1750_RES1			0
#define BH1750_RES2			1

/*
 * DEVICE STRUCT
 */
typedef struct BH1750_ALS{
	I2C_HandleTypeDef *i2cHandle;
	uint8_t address;
	uint8_t measTime;	// default: 69
	uint8_t mode;		// 0: 1lx resolution, 1: 0.5lx resolution
} BH1750_DEVICE;

/*
 * Initialisation
 */
void BH1750_Init(BH1750_DEVICE *dev, I2C_HandleTypeDef *i2cHandle, uint8_t addrHL);

/*
 * High-level functions
 */
void BH1750_powerDown(BH1750_DEVICE *dev);
void BH1750_powerOn(BH1750_DEVICE *dev);
void BH1750_reset(BH1750_DEVICE *dev);
void BH1750_startContHighRes1(BH1750_DEVICE *dev);
void BH1750_startContHighRes2(BH1750_DEVICE *dev);
void BH1750_startContLowRes(BH1750_DEVICE *dev);
void BH1750_startOnceHighRes1(BH1750_DEVICE *dev);
void BH1750_startOnceHighRes2(BH1750_DEVICE *dev);
void BH1750_startOnceLowRes(BH1750_DEVICE *dev);
void BH1750_setMeasTime(BH1750_DEVICE *dev, uint16_t ms);	// ms: 54..441, default 120

float BH1750_readBrightness(BH1750_DEVICE *dev);
/*
 * LOW-LEVEL I2C
 */
HAL_StatusTypeDef BH1750_readValue(BH1750_DEVICE *dev, uint16_t *buf);
HAL_StatusTypeDef BH1750_writeCommand(BH1750_DEVICE *dev, uint8_t *cmd);

#endif /* INC_BH1750_H_ */
