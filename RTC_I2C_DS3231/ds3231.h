/*
 * ds3231.h
 * Maxim DS3231 RTC I2C driver for STM32F1xx
 *  Created on: Oct 26, 2021
 *      Author: Till György
 */

#ifndef DS3231_I2C_DRIVER_H
#define DS3231_I2C_DRIVER_H

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
#define DS3231_I2C_ADDRESS (0x68 << 1)
#define ENABLED						1
#define DISABLED					0
#define ALARM_MASK_EVERY_SECOND		0xF
#define ALARM_MASK_SECONDS_MATCH	0xE
#define ALARM_MASK_MINUTES_MATCH	0xC
#define ALARM_MASK_HOURS_MATCH		0x8
#define ALARM_MASK_DAY_DATE_MATCH	0x0
#define ALARM1_INT_MASK				0x01
#define ALARM2_INT_MASK				0x02
#define OSF_MASK					0x80

/* registers */
#define DS3231_REG_SECONDS			0x00
#define DS3231_REG_MINUTES			0x01
#define DS3231_REG_HOURS			0x02
#define DS3231_REG_DAY				0x03
#define DS3231_REG_DATE				0x04
#define DS3231_REG_MONTH			0x05
#define DS3231_REG_YEAR				0x06
#define DS3231_REG_ALARM1_SECONDS	0x07
#define DS3231_REG_ALARM1_MINUTES	0x08
#define DS3231_REG_ALARM1_HOURS		0x09
#define DS3231_REG_ALARM1_DAY		0x0A
#define DS3231_REG_ALARM2_MINUTES	0x0B
#define DS3231_REG_ALARM2_HOURS		0x0C
#define DS3231_REG_ALARM2_DAY		0x0D
#define DS3231_REG_CONTROL			0x0E
#define DS3231_REG_STATUS			0x0F
#define DS3231_REG_AGING			0x10
#define DS3231_REG_TEMP_MSB			0x11
#define DS3231_REG_TEMP_LSB			0x12

/*
 * DATETIME STRUCT
 */
typedef struct DT{
	/* time */
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t ampm;
	uint8_t n24;

	/* date */
	uint8_t dayOfWeek;
	uint8_t dayOfMonth;
	uint8_t month;
	uint8_t century;
	uint16_t year;

} DS3231_DATETIME;

/*
 * ALARM STRUCT
 */
typedef struct AL{
	/* time */
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t ampm;
	uint8_t n24;

	/* date */
	uint8_t nDT;
	uint8_t dayOfWeek;
	uint8_t dayOfMonth;

	/* mask */
	uint8_t mask;

} DS3231_ALARM;

/*
 * STATUS STRUCT
 */
typedef struct ST{
	/* status */
	uint8_t OSF;	// 1: oscillator stopped
	uint8_t busy;	// 1: busy
	uint8_t alarm1;	// 1: alarm on
	uint8_t alarm2;	// 1: alarm on

} DS3231_STATUS;

/*
 * DEVICE STRUCT
 */
typedef struct DS{
	/* I2C handle */
	I2C_HandleTypeDef *i2cHandle;

	/* oscillator stop flag */
	uint8_t OSF;

} DS3231_DEVICE;

/*
 * INITIALISATION
 */
HAL_StatusTypeDef DS3231_Init(DS3231_DEVICE *dev, I2C_HandleTypeDef *i2cHandle);

/*
 * DATA READ
 */
HAL_StatusTypeDef DS3231_getDateTime(DS3231_DEVICE *dev, DS3231_DATETIME *dateTime);
HAL_StatusTypeDef DS3231_getTemperature(DS3231_DEVICE *dev, double *tempC);
HAL_StatusTypeDef DS3231_getAlarm1(DS3231_DEVICE *dev, DS3231_ALARM *alarm);
HAL_StatusTypeDef DS3231_getAlarm2(DS3231_DEVICE *dev, DS3231_ALARM *alarm);
HAL_StatusTypeDef DS3231_getStatus(DS3231_DEVICE *dev, DS3231_STATUS *status);

/*
 * DATA WRITE
 */
HAL_StatusTypeDef DS3231_setDateTime(DS3231_DEVICE *dev, DS3231_DATETIME *dateTime);
HAL_StatusTypeDef DS3231_setAlarm1(DS3231_DEVICE *dev, DS3231_ALARM *alarm);
HAL_StatusTypeDef DS3231_setAlarm2(DS3231_DEVICE *dev, DS3231_ALARM *alarm);
HAL_StatusTypeDef DS3231_enableAlarm1(DS3231_DEVICE *dev, uint8_t enabled);
HAL_StatusTypeDef DS3231_enableAlarm2(DS3231_DEVICE *dev, uint8_t enabled);
HAL_StatusTypeDef DS3231_clearOSF(DS3231_DEVICE *dev);
HAL_StatusTypeDef DS3231_clearAlarm1(DS3231_DEVICE *dev);
HAL_StatusTypeDef DS3231_clearAlarm2(DS3231_DEVICE *dev);

/*
 * LOW-LEVEL I2C
 */
HAL_StatusTypeDef DS3231_readRegister(DS3231_DEVICE *dev, uint8_t reg, uint8_t *buf);
HAL_StatusTypeDef DS3231_readRegisters(DS3231_DEVICE *dev, uint8_t reg, uint8_t *buf, uint8_t len);
HAL_StatusTypeDef DS3231_writeRegister(DS3231_DEVICE *dev, uint8_t reg, uint8_t *buf);
HAL_StatusTypeDef DS3231_writeRegisters(DS3231_DEVICE *dev, uint8_t reg, uint8_t *buf, uint8_t len);

/*
 * HELPER
 */
uint8_t monthStr2Num(char *monthString);
char *monthNum2StrLong(uint8_t monthNum);
char *monthNum2Str(uint8_t monthNum);
uint8_t dayStr2Num(char *dayString);
char *dayNum2StrLong(uint8_t dayNum);
char *dayNum2Str(uint8_t dayNum);

#endif /* INC_DS3231_H_ */
