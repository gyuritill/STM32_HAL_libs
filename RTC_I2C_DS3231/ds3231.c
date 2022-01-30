/*
 * ds3231.c
 * Maxim DS3231 RTC I2C driver
 *  Created on: Oct 26, 2021
 *      Author: Till György
 */

#include "ds3231.h"

/*
 * INITIALISATION
 */
HAL_StatusTypeDef DS3231_Init(DS3231_DEVICE *dev, I2C_HandleTypeDef *i2cHandle){
	dev->i2cHandle = i2cHandle;
	HAL_StatusTypeDef status=HAL_OK;
	uint8_t data;

	// nEOSC=0:			osc on
	// BBSQW=0:			square-wave off
	// CONV=0:			no temperature conversion
	// RS2=0, RS1=0:	don't care
	// INTCN=1:			nINT/SQW pin is interrupt
	// A2IE=0, A1IE=0:	alarm interrupts off
	data = 0x04;
	status |= DS3231_writeRegister(dev, DS3231_REG_CONTROL, &data);

	status |= DS3231_readRegister(dev, DS3231_REG_STATUS, &data);
	dev->OSF = data & 0x80;

	// OSF=0:			clear oscillator stop flag
	// 000:				not used
	// EN32kHz=0:		disable 32kHz output
	// BSY=0:			clear busy flag
	// A2F=0, A1F=0:	clear alarm interrupt flags
	data = 0x00;
	status |= DS3231_writeRegister(dev, DS3231_REG_STATUS, &data);
	return status;
}

/*
 * DATA READ
 */
HAL_StatusTypeDef DS3231_getDateTime(DS3231_DEVICE *dev, DS3231_DATETIME *dateTime){
	HAL_StatusTypeDef status=HAL_OK;
	uint8_t data[7];
	status |= DS3231_readRegisters(dev, DS3231_REG_SECONDS, &data[0], 7);
	dateTime->seconds = ((data[0] & 0x70)>>4)*10 + (data[0] & 0x0f);
	dateTime->minutes = ((data[1] & 0x70)>>4)*10 + (data[1] & 0x0f);
	dateTime->n24 = (data[2] & 0x40)>>6;
	if(dateTime->n24){
		dateTime->ampm = (data[2] & 0x20)>>5;
		dateTime->hours = ((data[2] & 0x10)>>4)*10 + (data[2] & 0x0F);
	}else{
		dateTime->hours = ((data[2] & 0x30)>>4)*10 + (data[2] & 0x0F);
	}
	dateTime->dayOfWeek = (data[3] & 0x07);
	dateTime->dayOfMonth = ((data[4] & 0x30)>>4)*10 + (data[4] & 0x0F);
	dateTime->month = ((data[5] & 0x10)>>4)*10 + (data[5] & 0x0F);
	dateTime->year = 2000 + ((data[6] & 0xF0)>>4)*10 + (data[6] & 0x0F);
	return status;
}
HAL_StatusTypeDef DS3231_getTemperature(DS3231_DEVICE *dev, double *tempC){
	HAL_StatusTypeDef status=HAL_OK;
	uint8_t data[2];
	status |= DS3231_readRegisters(dev, DS3231_REG_TEMP_MSB, &data[0], 2);
	*tempC = (int)data[0] + (data[1]>>6)/4.0;
	return status;
}
HAL_StatusTypeDef DS3231_getAlarm1(DS3231_DEVICE *dev, DS3231_ALARM *alarm){
	HAL_StatusTypeDef status=HAL_OK;
	uint8_t data[4];
	status |= DS3231_readRegisters(dev, DS3231_REG_ALARM1_SECONDS, &data[0], 4);
	alarm->seconds = ((data[0] & 0x70)>>4)*10 + (data[0] & 0x0f);
	alarm->minutes = ((data[1] & 0x70)>>4)*10 + (data[1] & 0x0f);
	alarm->n24 = (data[2] & 0x40)>>6;
	if(alarm->n24){
		alarm->ampm = (data[2] & 0x20)>>5;
		alarm->hours = ((data[2] & 0x10)>>4)*10 + (data[2] & 0x0F);
	}else{
		alarm->hours = ((data[2] & 0x30)>>4)*10 + (data[2] & 0x0F);
	}
	alarm->nDT = (data[3] & 0x40)>>6;
	if(alarm->nDT){
		alarm->dayOfWeek = (data[3] & 0x0F);
	}else{
		alarm->dayOfMonth = ((data[3] & 0x30)>>4)*10 + (data[3] & 0x0F);
	}
	alarm->mask = ((data[3] & 0x80)>>4) + ((data[2] & 0x80)>>5) + ((data[1] & 0x80)>>6) + ((data[0] & 0x80)>>7);
	return status;
}
HAL_StatusTypeDef DS3231_getAlarm2(DS3231_DEVICE *dev, DS3231_ALARM *alarm){
	HAL_StatusTypeDef status=HAL_OK;
	uint8_t data[3];
	status |= DS3231_readRegisters(dev, DS3231_REG_ALARM2_MINUTES, &data[0], 3);
	alarm->minutes = ((data[0] & 0x70)>>4)*10 + (data[0] & 0x0f);
	alarm->n24 = (data[1] & 0x40)>>6;
	if(alarm->n24){
		alarm->ampm = (data[1] & 0x20)>>5;
		alarm->hours = ((data[1] & 0x10)>>4)*10 + (data[1] & 0x0F);
	}else{
		alarm->hours = ((data[1] & 0x30)>>4)*10 + (data[1] & 0x0F);
	}
	alarm->nDT = (data[2] & 0x40)>>6;
	if(alarm->nDT){
		alarm->dayOfWeek = (data[2] & 0x0F);
	}else{
		alarm->dayOfMonth = ((data[2] & 0x30)>>4)*10 + (data[2] & 0x0F);
	}
	alarm->mask = ((data[2] & 0x80)>>4) + ((data[1] & 0x80)>>5) + ((data[0] & 0x80)>>6);
	return status;
}
HAL_StatusTypeDef DS3231_getStatus(DS3231_DEVICE *dev, DS3231_STATUS *deviceStatus){
	HAL_StatusTypeDef status=HAL_OK;
	uint8_t data;
	status |= DS3231_readRegister(dev, DS3231_REG_STATUS, &data);
	deviceStatus->OSF = data & 0x80;
	deviceStatus->alarm1 = data & 0x01;
	deviceStatus->alarm2 = data & 0x02;
	deviceStatus->busy = data & 0x04;
	return status;
}

/*
 * DATA WRITE
 */
HAL_StatusTypeDef DS3231_setDateTime(DS3231_DEVICE *dev, DS3231_DATETIME *dateTime){
	HAL_StatusTypeDef status=HAL_OK;
	uint8_t data;
	data = ((dateTime->seconds / 10 & 0x7) << 4) + (dateTime->seconds % 10 & 0xF);
	status |= DS3231_writeRegister(dev, DS3231_REG_SECONDS, &data);
	data = ((dateTime->minutes / 10 & 0x7) << 4) + (dateTime->minutes % 10 & 0xF);
	status |= DS3231_writeRegister(dev, DS3231_REG_MINUTES, &data);
	if(dateTime->n24){
		data = 0x40 + ((dateTime->ampm & 0x1) << 5) + ((dateTime->hours / 10 & 0x1) << 4) + (dateTime->hours % 10 & 0xF);
	}else{
		data = ((dateTime->hours / 10 & 0x3) << 4) + (dateTime->hours % 10 & 0xF);
	}
	status |= DS3231_writeRegister(dev, DS3231_REG_HOURS, &data);
	data = dateTime->dayOfWeek &0x7;
	status |= DS3231_writeRegister(dev, DS3231_REG_DAY, &data);
	data = ((dateTime->dayOfMonth / 10 & 0x3) << 4) + (dateTime->dayOfMonth % 10 & 0xF);
	status |= DS3231_writeRegister(dev, DS3231_REG_DATE, &data);
	data = ((dateTime->century & 0x1) << 7) + ((dateTime->month / 10 & 0x1) << 4) + (dateTime->month % 10 & 0xF);
	status |= DS3231_writeRegister(dev, DS3231_REG_MONTH, &data);
	data = (((dateTime->year-2000) / 10 & 0xF) << 4) + ((dateTime->year-2000) % 10 & 0xF);
	status |= DS3231_writeRegister(dev, DS3231_REG_YEAR, &data);
	return status;
}
HAL_StatusTypeDef DS3231_setAlarm1(DS3231_DEVICE *dev, DS3231_ALARM *alarm){
	HAL_StatusTypeDef status=HAL_OK;
	uint8_t data;
	data = ((alarm->mask & 0x1) << 7) + ((alarm->seconds / 10 & 0x7) << 4) + (alarm->seconds % 10 & 0xF);
	status |= DS3231_writeRegister(dev, DS3231_REG_ALARM1_SECONDS, &data);
	data = ((alarm->mask & 0x2) << 7) + ((alarm->minutes / 10 & 0x7) << 4) + (alarm->minutes % 10 & 0xF);
	status |= DS3231_writeRegister(dev, DS3231_REG_ALARM1_MINUTES, &data);
	if(alarm->n24){
		data = ((alarm->mask & 0x4) << 7) + 0x40 + ((alarm->ampm & 0x1) << 5) + ((alarm->minutes / 10 & 0x1) << 4) + (alarm->minutes % 10 & 0xF);
	}else{
		data = ((alarm->mask & 0x4) << 7) + ((alarm->minutes / 10 & 0x3) << 4) + (alarm->minutes % 10 & 0xF);
	}
	status |= DS3231_writeRegister(dev, DS3231_REG_ALARM1_HOURS, &data);
	if(alarm->nDT){
		data = ((alarm->mask & 0x8) << 7) + 0x40 + (alarm->dayOfWeek % 10 & 0xF);
	}else{
		data = ((alarm->mask & 0x8) << 7) + ((alarm->dayOfMonth / 10 & 0x3) << 4) + (alarm->dayOfMonth % 10 & 0xF);
	}
	status |= DS3231_writeRegister(dev, DS3231_REG_ALARM1_DAY, &data);
	return status;
}
HAL_StatusTypeDef DS3231_setAlarm2(DS3231_DEVICE *dev, DS3231_ALARM *alarm){
	HAL_StatusTypeDef status=HAL_OK;
	uint8_t data;
	data = ((alarm->mask & 0x2) << 7) + ((alarm->minutes / 10 & 0x7) << 4) + (alarm->minutes % 10 & 0xF);
	status |= DS3231_writeRegister(dev, DS3231_REG_ALARM2_MINUTES, &data);
	if(alarm->n24){
		data = ((alarm->mask & 0x4) << 7) + 0x40 + ((alarm->ampm & 0x1) << 5) + ((alarm->minutes / 10 & 0x1) << 4) + (alarm->minutes % 10 & 0xF);
	}else{
		data = ((alarm->mask & 0x4) << 7) + ((alarm->minutes / 10 & 0x3) << 4) + (alarm->minutes % 10 & 0xF);
	}
	status |= DS3231_writeRegister(dev, DS3231_REG_ALARM2_HOURS, &data);
	if(alarm->nDT){
		data = ((alarm->mask & 0x8) << 7) + 0x40 + (alarm->dayOfWeek % 10 & 0xF);
	}else{
		data = ((alarm->mask & 0x8) << 7) + ((alarm->dayOfMonth / 10 & 0x3) << 4) + (alarm->dayOfMonth % 10 & 0xF);
	}
	status |= DS3231_writeRegister(dev, DS3231_REG_ALARM2_DAY, &data);
	return status;
}
HAL_StatusTypeDef DS3231_enableAlarm1(DS3231_DEVICE *dev, uint8_t enabled){
	HAL_StatusTypeDef status=HAL_OK;
	uint8_t data;
	status |= DS3231_readRegister(dev, DS3231_REG_CONTROL, &data);
	if(enabled){
		data |= ALARM1_INT_MASK;
	}else{
		data &= ~ALARM1_INT_MASK;
	}
	status |= DS3231_writeRegister(dev, DS3231_REG_CONTROL, &data);
	return status;
}
HAL_StatusTypeDef DS3231_enableAlarm2(DS3231_DEVICE *dev, uint8_t enabled){
	HAL_StatusTypeDef status=HAL_OK;
	uint8_t data;
	status |= DS3231_readRegister(dev, DS3231_REG_CONTROL, &data);
	if(enabled){
		data |= ALARM2_INT_MASK;
	}else{
		data &= ~ALARM2_INT_MASK;
	}
	status |= DS3231_writeRegister(dev, DS3231_REG_CONTROL, &data);
	return status;
}
HAL_StatusTypeDef DS3231_clearAlarm1(DS3231_DEVICE *dev){
	HAL_StatusTypeDef status=HAL_OK;
	uint8_t data;
	status |= DS3231_readRegister(dev, DS3231_REG_STATUS, &data);
	data &= ~ALARM1_INT_MASK;
	status |= DS3231_writeRegister(dev, DS3231_REG_STATUS, &data);
	return status;
}
HAL_StatusTypeDef DS3231_clearAlarm2(DS3231_DEVICE *dev){
	HAL_StatusTypeDef status=HAL_OK;
	uint8_t data;
	status |= DS3231_readRegister(dev, DS3231_REG_STATUS, &data);
	data &= ~ALARM2_INT_MASK;
	status |= DS3231_writeRegister(dev, DS3231_REG_STATUS, &data);
	return status;
}
HAL_StatusTypeDef DS3231_clearOSF(DS3231_DEVICE *dev){
	HAL_StatusTypeDef status=HAL_OK;
	uint8_t data;
	status |= DS3231_readRegister(dev, DS3231_REG_STATUS, &data);
	data &= ~OSF_MASK;
	status |= DS3231_writeRegister(dev, DS3231_REG_STATUS, &data);
	return status;
}
/*
 * LOW-LEVEL I2C
 */
HAL_StatusTypeDef DS3231_readRegister(DS3231_DEVICE *dev, uint8_t reg, uint8_t *pBuf){
	return HAL_I2C_Mem_Read(dev->i2cHandle, DS3231_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, pBuf, 1, HAL_MAX_DELAY);
}
HAL_StatusTypeDef DS3231_readRegisters(DS3231_DEVICE *dev, uint8_t reg, uint8_t *pBuf, uint8_t len){
	return HAL_I2C_Mem_Read(dev->i2cHandle, DS3231_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, pBuf, len, HAL_MAX_DELAY);
}
HAL_StatusTypeDef DS3231_writeRegister(DS3231_DEVICE *dev, uint8_t reg, uint8_t *pBuf){
	return HAL_I2C_Mem_Write(dev->i2cHandle, DS3231_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, pBuf, 1, HAL_MAX_DELAY);
}
HAL_StatusTypeDef DS3231_writeRegisters(DS3231_DEVICE *dev, uint8_t reg, uint8_t *pBuf, uint8_t len){
	return HAL_I2C_Mem_Write(dev->i2cHandle, DS3231_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, pBuf, len, HAL_MAX_DELAY);
}

/*
 * HELPER
 */
uint8_t monthStr2Num(char *monthString){
	uint8_t monthNum;
	switch(monthString[0]){
	case 'J':			// Jan, Jun, Jul
		if(monthString[1] == 'a'){
			monthNum = 1;
		}else if(monthString[2] == 'n'){
			monthNum = 6;
		}else{
			monthNum = 7;
		}
		break;
	case 'F':			// Feb
		monthNum = 2;
		break;
	case 'M':			// Mar, May
		if(monthString[2] == 'r'){
			monthNum = 3;
		}else{
			monthNum = 5;
		}
		break;
	case 'A':			// Apr, Aug
		if(monthString[2] == 'r'){
			monthNum = 4;
		}else{
			monthNum = 8;
		}
		break;
	case 'S':			// Sep
		monthNum = 9;
		break;
	case 'O':			// Oct
		monthNum = 10;
		break;
	case 'N':			// Nov
		monthNum = 11;
		break;
	case 'D':			// Dec
		monthNum = 12;
		break;
	default:
		monthNum = 0;
	}
	return monthNum;
}

uint8_t dayStr2Num(char *dayString){
	uint8_t dayNum;
	switch(dayString[1]){
	case 'o':			// Mon
		dayNum = 1;
		break;
	case 'u':			// Tue, Sun
		if(dayString[2] == 'e'){
			dayNum = 2;
		}else{
			dayNum = 7;
		}
		break;
	case 'e':			// Wed
		dayNum = 3;
		break;
	case 'h':			// Thu
		dayNum = 4;
		break;
	case 'r':			// Fri
		dayNum = 5;
		break;
	case 'a':			// Sat
		dayNum = 6;
		break;
	default:
		dayNum = 0;
	}
	return dayNum;
}

char *monthNum2StrLong(uint8_t monthNum){
	if(monthNum == 1) return "January";
	else if(monthNum == 2) return "February";
	else if(monthNum == 3) return "March";
	else if(monthNum == 4) return "April";
	else if(monthNum == 5) return "May";
	else if(monthNum == 6) return "June";
	else if(monthNum == 7) return "July";
	else if(monthNum == 8) return "August";
	else if(monthNum == 9) return "September";
	else if(monthNum == 10) return "October";
	else if(monthNum == 11) return "November";
	else if(monthNum == 12) return "December";
	return NULL;
}

char *monthNum2Str(uint8_t monthNum){
	if(monthNum == 1) return "Jan";
	else if(monthNum == 2) return "Feb";
	else if(monthNum == 3) return "Mar";
	else if(monthNum == 4) return "Apr";
	else if(monthNum == 5) return "May";
	else if(monthNum == 6) return "Jun";
	else if(monthNum == 7) return "Jul";
	else if(monthNum == 8) return "Aug";
	else if(monthNum == 9) return "Sep";
	else if(monthNum == 10) return "Oct";
	else if(monthNum == 11) return "Nov";
	else if(monthNum == 12) return "Dec";
	return NULL;
}

char *dayNum2StrLong(uint8_t dayNum){
	if(dayNum == 1) return "Monday";
	else if(dayNum == 2) return "Tuesday";
	else if(dayNum == 3) return "Wednesday";
	else if(dayNum == 4) return "Thursday";
	else if(dayNum == 5) return "Friday";
	else if(dayNum == 6) return "Saturday";
	else if(dayNum == 7) return "Sunday";
	return NULL;
}

char *dayNum2Str(uint8_t dayNum){
	if(dayNum == 1) return "Mon";
	else if(dayNum == 2) return "Tue";
	else if(dayNum == 3) return "Wed";
	else if(dayNum == 4) return "Thu";
	else if(dayNum == 5) return "Fri";
	else if(dayNum == 6) return "Sat";
	else if(dayNum == 7) return "Sun";
	return NULL;
}
