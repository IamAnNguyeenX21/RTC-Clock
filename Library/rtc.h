/*
 * rtc.h
 *
 *  Created on: Apr 11, 2024
 *      Author: an nguyen
 */

#ifndef NEWLIB_RTC_H_
#define NEWLIB_RTC_H_

#include "stm32f1xx_hal.h"

#define RTC_ADDR	0x68<<1

typedef struct
{
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint8_t date;
	uint8_t day;
	uint8_t month;
	uint8_t year;

}DateTime;

typedef struct
{
	I2C_HandleTypeDef *rtc;
	DateTime *time;
}RTC_Typedef;

typedef enum
{
	Start_Set,
	End_set,
}Set_Button_Status;
uint8_t Decimal2BCD(uint8_t num);
uint8_t BCD2Decimal(uint8_t num);
void RTC_ReadTime (RTC_Typedef *rtc, DateTime *dt);
void RTC_WriteTime(RTC_Typedef *rtc, DateTime *dt);
void RTC_Init (RTC_Typedef *rtc,I2C_HandleTypeDef *i2c, DateTime *dt);
#endif /* NEWLIB_RTC_H_ */
