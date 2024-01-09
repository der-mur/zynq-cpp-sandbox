/******************************************************************************
 * @Title		:	Triple Timer Counter Driver Class Header file
 * @Filename	:	ttc.h
 * @Author		:	Derek Murray
 * @Origin Date	:	07/12/2023
 * @Version		:	1.0.0
 * @Compiler	:	arm-none-eabi-gcc
 * @Target		: 	Xilinx Zynq-7000
 * @Platform	: 	Digilent Zybo-Z7-20
 *
 * ------------------------------------------------------------------------
 *
 * Copyright (C) 2023  Derek Murray
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
******************************************************************************/

#ifndef TTC_H_
#define TTC_H_

/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/

#include <cstdint>
#include "../system/settings.h"


/*****************************************************************************/
/********************************* Aliases ***********************************/
/*****************************************************************************/

using device_reg = std::uint32_t volatile;

// Interrupt handler function pointer
using p_TtcIntrHandler = void (*) (void);




/*****************************************************************************/
/***************************** Class Interface *******************************/
/*****************************************************************************/


/*-------------------------------------------------------------*/
/*--------------- Single Timer Instance -----------------------*/
/*-------------------------------------------------------------*/

/* Class for an individual timer/counter i.e. three of these exist in
 * each Triple Timer Counter (TTC). The user can create and used these
 *  single objects, or create a Triple Timer Counter object that contains
 *  all three single timer objects. (See class Ttc below for the latter option. */


class TtcSingleTimer
{
public:
	/* --- CONSTRUCTORS --------- */
	TtcSingleTimer() {};
	TtcSingleTimer(std::uint16_t ttc_id, std::uint16_t timer_id); 	// ttc_id = 0 or 1; timer_id = 0, 1 or 2.

	/* --- PUBLIC METHODS --------- */

	/* --- Enable and reset methods --- */
	void enableTtc(void);
	void disableTtc(void);
	void resetCounter(void); // Resets the counter by writing 1 to bit 4; automatically cleared.

	/* --- Output waveform methods --- */
	void setOutputWaveformOnOff(bool on_off); // true = output enabled, false = output disabled
	void setOutputWaveformPolarity(bool polarity); // true = active high, false = active low

	/* --- Mode ---
	 * overflow_interval_mode: false = overflow, true = interval.
	 * match_mode: false = disable match mode, true = enable match mode.
	 * incr_dec: false = timer increments, true = timer decrements. */
	void setMode(bool overflow_interval_mode, bool match_mode, bool decrement);

	/* Enable/Disable the prescaler and set the value. */
	void setPrescaler(bool enable, std::uint16_t value);

	/* Set the interval value for reset and restart in interval mode. */
	void setIntervalValue(std::uint16_t interval);


	/* Set the match values to be used in match mode. */
	void setMatchValues(std::uint16_t match0, std::uint16_t match1, std::uint16_t match2);

	void setMatchValuesF(double match0_seconds, double match1_seconds, double match2_seconds);

	/* Get the counter value from COUNTER register */
	std::uint16_t getCount(void);

	/* Interrupt Methods */
	std::uint8_t getInterruptStatus(void);
	void clearInterruptStatus(void);
	void setInterruptEnable(std::uint8_t intr_enable);

	// Set/Get user interrupt handler
	void setUserIntrHandler(p_TtcIntrHandler p_handler);
	p_TtcIntrHandler getUserIntrHandler(void);


	// Individual Get
	bool intervalIntrDetected(void);
	bool match0IntrDetected(void);
	bool match1IntrDetected(void);
	bool match2IntrDetected(void);
	bool countOverflowIntrDetected(void);
	bool eventTimerOverflowIntrDetected(void);

	// Individual Clear
	bool clearIntervalIntr(void);
	bool clearMatch0Intr(void);
	bool clearMatch1Intr(void);
	bool clearMatch2Intr(void);
	bool clearCountOverflowIntr(void);
	bool clearEventTimerOverflowIntr(void);


	// test
	void printDetails(void);



private:
	// General parameters
	std::uint32_t m_base_addr {XPAR_PS7_TTC_0_BASEADDR};
	std::uint16_t m_ttc_id {0};
	std::uint16_t m_timer_id {0};

	p_TtcIntrHandler p_UserInterruptHandler;

	// Registers
	device_reg *p_CLOCK_CONTROL_REG {nullptr};
	device_reg *p_COUNT_CONTROL_REG {nullptr};
	device_reg *p_COUNT_VALUE_REG {nullptr};
	device_reg *p_INTERVAL_VALUE_REG {nullptr};
	device_reg *p_MATCH0_VALUE_REG {nullptr};
	device_reg *p_MATCH1_VALUE_REG {nullptr};
	device_reg *p_MATCH2_VALUE_REG {nullptr};
	device_reg *p_INTR_STATUS_REG {nullptr};
	device_reg *p_INTR_ENABLE_REG {nullptr};
	device_reg *p_EVENT_CONTROL_REG {nullptr};
	device_reg *p_EVENT_COUNT_REG {nullptr};

};




/*-------------------------------------------------------------*/
/*--------------- Triple Timer Counter ------------------------*/
/*-------------------------------------------------------------*/

/* All three TTCs can be implemented in a single class if desired.
 * (An array of TtcSingleTimer objects) */

class Ttc
{
public:
	/* --- CONSTRUCTORS --------- */
	//Ttc() {}
	Ttc(std::uint16_t ttc_id);


	void printDetails(bool all, uint16_t ttc_id);

private:
	TtcSingleTimer TtcTimerArray[sys::ttc::n_timers_per_ttc];
};


#endif // TTC_H_
