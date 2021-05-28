/*
  RTC library for Arduino SAMD/E51.
  Copyright (c) 2015 Arduino LLC. All right reserved.
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#include <time.h>
#include "RTC_E51.h"

#define EPOCH_TIME_OFF      946684800  // This is 1st January 2000, 00:00:00 in epoch time
#define EPOCH_TIME_YEAR_OFF 100        // years since 1900

// Default date & time after reset
#define DEFAULT_YEAR    2000    // 2000..2063
#define DEFAULT_MONTH   1       // 1..12
#define DEFAULT_DAY     1       // 1..31
#define DEFAULT_HOUR    0       // 1..23
#define DEFAULT_MINUTE  0       // 0..59
#define DEFAULT_SECOND  0       // 0..59

voidFuncPtr RTC_callBack = NULL;

RTC_E51::RTC_E51()
{
  _configured = false;
}

void RTC_E51::begin(bool resetTime)
{
  uint16_t tmp_reg = 0;
  
  MCLK->APBAMASK.reg |= MCLK_APBAMASK_RTC; // turn on digital interface clock

  config32kOSC();

  // If the RTC is in clock mode and the reset was
  // not due to POR or BOD, preserve the clock time
  // POR causes a reset anyway, BOD behaviour is?
  bool validTime = false;
  RTC_MODE2_CLOCK_Type oldTime;

	if ((!resetTime) && (RSTC->RCAUSE.reg & (RSTC_RCAUSE_SYST | RSTC_RCAUSE_WDT | RSTC_RCAUSE_EXT))) {
		if (RTC->MODE2.CTRLA.reg & RTC_MODE2_CTRLA_MODE_CLOCK) {
			validTime = true;
			oldTime.reg = RTC->MODE2.CLOCK.reg;
		}
	}
		
	tmp_reg |= RTC_MODE2_CTRLA_MODE_CLOCK; // set clock operating mode
	tmp_reg |= RTC_MODE2_CTRLA_PRESCALER_DIV1024; // set prescaler to 1024 for MODE2
	tmp_reg &= ~RTC_MODE2_CTRLA_MATCHCLR; // disable clear on match
	tmp_reg &= ~RTC_MODE2_CTRLA_CLKREP; // 24h time representation //According to the datasheet RTC_MODE2_CTRL_CLKREP = 0 for 24h
	RTC->MODE2.CTRLA.reg = tmp_reg;
	while (RTCisSyncing());
	
	NVIC_EnableIRQ(RTC_IRQn); // enable RTC interrupt
	NVIC_SetPriority(RTC_IRQn, 0x00);

	RTC->MODE2.INTENSET.reg					|= RTC_MODE2_INTENSET_ALARM0; // enable alarm interrupt
	RTC->MODE2.Mode2Alarm[0].MASK.bit.SEL    = MATCH_OFF; // default alarm match is off (disabled)
	while (RTCisSyncing());

	RTCenable();
	RTCresetRemove();

  if ((!resetTime) && (validTime) && (oldTime.reg != 0L)) {
		RTC->MODE2.CLOCK.reg = oldTime.reg;
	}
	else {
		RTC->MODE2.CLOCK.reg = RTC_MODE2_CLOCK_YEAR(DEFAULT_YEAR - 2000) |
								RTC_MODE2_CLOCK_MONTH(DEFAULT_MONTH) |
								RTC_MODE2_CLOCK_DAY(DEFAULT_DAY) |
								RTC_MODE2_CLOCK_HOUR(DEFAULT_HOUR) |
								RTC_MODE2_CLOCK_MINUTE(DEFAULT_MINUTE) |
								RTC_MODE2_CLOCK_SECOND(DEFAULT_SECOND);
	}
	
	while (RTCisSyncing());

	_configured = true;
}


void RTC_Handler(void)
{
  if (RTC_callBack != NULL) {
    RTC_callBack();
  }
  RTC->MODE2.INTFLAG.reg = RTC_MODE2_INTFLAG_ALARM0; // must clear flag at end
}

void RTC_E51::enableAlarm(Alarm_Match match)
{
  if (_configured) {
    RTC->MODE2.Mode2Alarm[0].MASK.bit.SEL = match;
    while (RTCisSyncing())
      ;
  }
}

void RTC_E51::disableAlarm()
{
  if (_configured) {
    RTC->MODE2.Mode2Alarm[0].MASK.bit.SEL = 0x00;
    while (RTCisSyncing())
      ;
  }
}

void RTC_E51::attachInterrupt(voidFuncPtr callback)
{
  RTC_callBack = callback;
}

void RTC_E51::detachInterrupt()
{
  RTC_callBack = NULL;
}

void RTC_E51::standbyMode()
{
  // Entering standby mode when connected
  // via the native USB port causes issues.
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  __WFI();
}

/*
 * Get Functions
 */

uint8_t RTC_E51::getSeconds()
{
  RTCreadRequest();
  return RTC->MODE2.CLOCK.bit.SECOND;
}

uint8_t RTC_E51::getMinutes()
{
  RTCreadRequest();
  return RTC->MODE2.CLOCK.bit.MINUTE;
}

uint8_t RTC_E51::getHours()
{
  RTCreadRequest();
  return RTC->MODE2.CLOCK.bit.HOUR;
}

uint8_t RTC_E51::getDay()
{
  RTCreadRequest();
  return RTC->MODE2.CLOCK.bit.DAY;
}

uint8_t RTC_E51::getMonth()
{
  RTCreadRequest();
  return RTC->MODE2.CLOCK.bit.MONTH;
}

uint8_t RTC_E51::getYear()
{
  RTCreadRequest();
  return RTC->MODE2.CLOCK.bit.YEAR;
}

uint8_t RTC_E51::getAlarmSeconds()
{
  return RTC->MODE2.Mode2Alarm[0].ALARM.bit.SECOND;
}

uint8_t RTC_E51::getAlarmMinutes()
{
  return RTC->MODE2.Mode2Alarm[0].ALARM.bit.MINUTE;
}

uint8_t RTC_E51::getAlarmHours()
{
  return RTC->MODE2.Mode2Alarm[0].ALARM.bit.HOUR;
}

uint8_t RTC_E51::getAlarmDay()
{
  return RTC->MODE2.Mode2Alarm[0].ALARM.bit.DAY;
}

uint8_t RTC_E51::getAlarmMonth()
{
  return RTC->MODE2.Mode2Alarm[0].ALARM.bit.MONTH;
}

uint8_t RTC_E51::getAlarmYear()
{
  return RTC->MODE2.Mode2Alarm[0].ALARM.bit.YEAR;
}

/*
 * Set Functions
 */

void RTC_E51::setSeconds(uint8_t seconds)
{
  if (_configured) {
    RTC->MODE2.CLOCK.bit.SECOND = seconds;
    while (RTCisSyncing())
      ;
  }
}

void RTC_E51::setMinutes(uint8_t minutes)
{
  if (_configured) {
    RTC->MODE2.CLOCK.bit.MINUTE = minutes;
    while (RTCisSyncing())
      ;
  }
}

void RTC_E51::setHours(uint8_t hours)
{
  if (_configured) {
    RTC->MODE2.CLOCK.bit.HOUR = hours;
    while (RTCisSyncing())
      ;
  }
}

void RTC_E51::setTime(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
  if (_configured) {
    setSeconds(seconds);
    setMinutes(minutes);
    setHours(hours);
  }
}

void RTC_E51::setDay(uint8_t day)
{
  if (_configured) {
    RTC->MODE2.CLOCK.bit.DAY = day;
    while (RTCisSyncing())
      ;
  }
}

void RTC_E51::setMonth(uint8_t month)
{
  if (_configured) {
    RTC->MODE2.CLOCK.bit.MONTH = month;
    while (RTCisSyncing())
      ;
  }
}

void RTC_E51::setYear(uint8_t year)
{
  if (_configured) {
    RTC->MODE2.CLOCK.bit.YEAR = year;
    while (RTCisSyncing())
      ;
  }
}

void RTC_E51::setDate(uint8_t day, uint8_t month, uint8_t year)
{
  if (_configured) {
    setDay(day);
    setMonth(month);
    setYear(year);
  }
}

void RTC_E51::setAlarmSeconds(uint8_t seconds)
{
  if (_configured) {
    RTC->MODE2.Mode2Alarm[0].ALARM.bit.SECOND = seconds;
    while (RTCisSyncing())
      ;
  }
}

void RTC_E51::setAlarmMinutes(uint8_t minutes)
{
  if (_configured) {
    RTC->MODE2.Mode2Alarm[0].ALARM.bit.MINUTE = minutes;
    while (RTCisSyncing())
      ;
  }
}

void RTC_E51::setAlarmHours(uint8_t hours)
{
  if (_configured) {
    RTC->MODE2.Mode2Alarm[0].ALARM.bit.HOUR = hours;
    while (RTCisSyncing())
      ;
  }
}

void RTC_E51::setAlarmTime(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
  if (_configured) {
    setAlarmSeconds(seconds);
    setAlarmMinutes(minutes);
    setAlarmHours(hours);
  }
}

void RTC_E51::setAlarmDay(uint8_t day)
{
  if (_configured) {
    RTC->MODE2.Mode2Alarm[0].ALARM.bit.DAY = day;
    while (RTCisSyncing())
      ;
  }
}

void RTC_E51::setAlarmMonth(uint8_t month)
{
  if (_configured) {
    RTC->MODE2.Mode2Alarm[0].ALARM.bit.MONTH = month;
    while (RTCisSyncing())
      ;
  }
}

void RTC_E51::setAlarmYear(uint8_t year)
{
  if (_configured) {
    RTC->MODE2.Mode2Alarm[0].ALARM.bit.YEAR = year;
    while (RTCisSyncing())
      ;
  }
}

void RTC_E51::setAlarmDate(uint8_t day, uint8_t month, uint8_t year)
{
  if (_configured) {
    setAlarmDay(day);
    setAlarmMonth(month);
    setAlarmYear(year);
  }
}

uint32_t RTC_E51::getEpoch()
{
  RTCreadRequest();
  RTC_MODE2_CLOCK_Type clockTime;
  clockTime.reg = RTC->MODE2.CLOCK.reg;

  struct tm tm;

  tm.tm_isdst = -1;
  tm.tm_yday = 0;
  tm.tm_wday = 0;
  tm.tm_year = clockTime.bit.YEAR + EPOCH_TIME_YEAR_OFF;
  tm.tm_mon = clockTime.bit.MONTH - 1;
  tm.tm_mday = clockTime.bit.DAY;
  tm.tm_hour = clockTime.bit.HOUR;
  tm.tm_min = clockTime.bit.MINUTE;
  tm.tm_sec = clockTime.bit.SECOND;

  return mktime(&tm);
}

uint32_t RTC_E51::getY2kEpoch()
{
  return (getEpoch() - EPOCH_TIME_OFF);
}

void RTC_E51::setAlarmEpoch(uint32_t ts)
{
  if (_configured) {
    if (ts < EPOCH_TIME_OFF) {
      ts = EPOCH_TIME_OFF;
    }

    time_t t = ts;
    struct tm* tmp = gmtime(&t);

    setAlarmDate(tmp->tm_mday, tmp->tm_mon + 1, tmp->tm_year - EPOCH_TIME_YEAR_OFF);
    setAlarmTime(tmp->tm_hour, tmp->tm_min, tmp->tm_sec);
  }
}

void RTC_E51::setEpoch(uint32_t ts)
{
  if (_configured) {
    if (ts < EPOCH_TIME_OFF) {
      ts = EPOCH_TIME_OFF;
    }

    time_t t = ts;
    struct tm* tmp = gmtime(&t);

    RTC->MODE2.CLOCK.bit.YEAR = tmp->tm_year - EPOCH_TIME_YEAR_OFF;
    RTC->MODE2.CLOCK.bit.MONTH = tmp->tm_mon + 1;
    RTC->MODE2.CLOCK.bit.DAY = tmp->tm_mday;
    RTC->MODE2.CLOCK.bit.HOUR = tmp->tm_hour;
    RTC->MODE2.CLOCK.bit.MINUTE = tmp->tm_min;
    RTC->MODE2.CLOCK.bit.SECOND = tmp->tm_sec;

    while (RTCisSyncing())
      ;
  }
}

void RTC_E51::setY2kEpoch(uint32_t ts)
{
  if (_configured) {
    setEpoch(ts + EPOCH_TIME_OFF);
  }
}

/*
 * Private Utility Functions
 */

/* Configure the 32768Hz Oscillator */
void RTC_E51::config32kOSC() 
{
  #ifdef CRYSTALLESS
	OSC32KCTRL->RTCCTRL.reg = OSC32KCTRL_RTCCTRL_RTCSEL_ULP1K;
	OSC32KCTRL->OSCULP32K.reg = OSC32KCTRL_OSCULP32K_EN32K |
                  OSC32KCTRL_OSCULP32K_EN1K |
                  ((_calib & 0x3F) << 14);          // Says don't change, mine is 2x slow without a calibration...
  #else
	/* Selecting the XOSC32k  --> 1k output as RTCclock src*/
	OSC32KCTRL->RTCCTRL.reg = OSC32KCTRL_RTCCTRL_RTCSEL_XOSC1K;
	while( (OSC32KCTRL->STATUS.reg & OSC32KCTRL_STATUS_XOSC32KRDY) == 0 );
	/* XOSC32K activated in startup.c but activating again in case USER-code stopped it */
	/* Enabling the "1k" output */
	OSC32KCTRL->XOSC32K.reg =   OSC32KCTRL_XOSC32K_XTALEN|
								OSC32KCTRL_XOSC32K_ENABLE |
								OSC32KCTRL_XOSC32K_EN32K |
								OSC32KCTRL_XOSC32K_EN1K |
								OSC32KCTRL_XOSC32K_CGM_XT |
								OSC32KCTRL_XOSC32K_RUNSTDBY |
								OSC32KCTRL_XOSC32K_ONDEMAND |
								OSC32KCTRL_XOSC32K_STARTUP(3);
  #endif
}

/* Synchronise the CLOCK register for reading*/
inline void RTC_E51::RTCreadRequest() {
	if (_configured) {
			RTC->MODE2.CTRLA.bit.CLOCKSYNC = 1;
		while (RTCisSyncing());
	}
}

/* Wait for sync in write operations */
inline bool RTC_E51::RTCisSyncing()
{
	return (RTC->MODE2.SYNCBUSY.reg );
}

void RTC_E51::RTCenable()
{
		RTC->MODE2.CTRLA.reg |= RTC_MODE2_CTRLA_ENABLE; // enable RTC
	while (RTCisSyncing());
}

void RTC_E51::RTCresetRemove()
{
		RTC->MODE2.CTRLA.reg &= ~RTC_MODE2_CTRLA_SWRST; // software reset remove
  while (RTCisSyncing())
    ;
}