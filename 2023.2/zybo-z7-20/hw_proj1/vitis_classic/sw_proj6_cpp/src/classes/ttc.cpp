/******************************************************************************
 * @Title		:	Triple Timer Counter Driver Class Source file
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

/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/

#include "ttc.h"
#include "../system/ps_reg.h"



/*****************************************************************************/
/******************************* Namespaces **********************************/
/*****************************************************************************/

using namespace ps::reg::offset::ttc;
using namespace sys::ttc;




/*****************************************************************************/
/************************** Constant Definitions *****************************/
/*****************************************************************************/

// TTC0_0 setting used for all TTCs. Would need to update if different settings used per TTC.
constexpr double ttc_period_ns = (1/static_cast<double>(sys::clock::ttc0_freq_hz));




/*****************************************************************************/
/******************** Register Bit Positions and Masks ***********************/
/*****************************************************************************/
// Not all registers/bits implemented in this test code.

/* Reg 0: CLOCK_CONTROL */
static constexpr std::uint8_t prescaler_enable_bit_pos {0};
static constexpr std::uint8_t prescaler_value_bit_pos {1};
static constexpr std::uint8_t prescaler_value_mask {0x1E}; // [Bits 4:1]
static constexpr std::uint8_t clock_source_bit_pos {5};
static constexpr std::uint8_t ext_clock_edge_bit_pos {6};


/* Reg 1: COUNT_CONTROL */
static constexpr std::uint8_t disable_counter_bit_pos {0};
static constexpr std::uint8_t interval_overflow_mode_bit_pos {1};
static constexpr std::uint8_t decrement_bit_pos {2};
static constexpr std::uint8_t match_mode_bit_pos {3};
static constexpr std::uint8_t reset_counter_bit_pos {4};
static constexpr std::uint8_t output_wave_enable_bit_pos {5};
static constexpr std::uint8_t output_wave_polarity_bit_pos {6};




/* Reg 7: INTERRUPT_STATUS; Reg 8: INTERRUPT_ENABLE */
static constexpr std::uint8_t interval_intr_bit_pos {0};
static constexpr std::uint8_t match1_intr_bit_pos {1};
static constexpr std::uint8_t match2_intr_bit_pos {2};
static constexpr std::uint8_t match3_intr_bit_pos {3};
static constexpr std::uint8_t counter_overflow_intr_bit_pos {4};
static constexpr std::uint8_t event_timer_overflow_intr_bit_pos {5};


/* Reg 9: EVENT_CONTROL */
static constexpr std::uint8_t event_enable_timer_bit_pos {0};
static constexpr std::uint8_t event_count_hi_lo_bit_pos {1};
static constexpr std::uint8_t event_overflow_mode_bit_pos {2};




/*****************************************************************************/
/*************** Single Timer Instance ***********************/
/*****************************************************************************/

/* ----------------------------------*/
/* --- Constructor ------------------*/
/* ----------------------------------*/

 // TtcSingleTimer::TtcSingleTimer(std::uint16_t ttc_id, std::uint16_t timer_id) 	// ttc_id = 0 or 1; timer_id = 0, 1 or 2.
TtcSingleTimer::TtcSingleTimer(std::uint16_t ttc_id, std::uint16_t timer_id)
{
	m_ttc_id = ttc_id;
	m_timer_id = timer_id;
	m_base_addr = ttc_base[ (n_timers_per_ttc * m_ttc_id) + m_timer_id ];


	p_CLOCK_CONTROL_REG = reinterpret_cast<device_reg*>(m_base_addr + CLOCK_CONTROL);
	p_COUNT_CONTROL_REG = reinterpret_cast<device_reg*>(m_base_addr + COUNT_CONTROL);
	p_COUNT_VALUE_REG = reinterpret_cast<device_reg*>(m_base_addr + COUNT_VALUE);
	p_INTERVAL_VALUE_REG = reinterpret_cast<device_reg*>(m_base_addr + INTERVAL_VALUE);
	p_MATCH0_VALUE_REG = reinterpret_cast<device_reg*>(m_base_addr + MATCH0_VALUE);
	p_MATCH1_VALUE_REG = reinterpret_cast<device_reg*>(m_base_addr + MATCH1_VALUE);
	p_MATCH2_VALUE_REG = reinterpret_cast<device_reg*>(m_base_addr + MATCH2_VALUE);
	p_INTR_STATUS_REG = reinterpret_cast<device_reg*>(m_base_addr + INTERRUPT_STATUS);
	p_INTR_ENABLE_REG = reinterpret_cast<device_reg*>(m_base_addr + INTERRUPT_ENABLE);
	p_EVENT_CONTROL_REG = reinterpret_cast<device_reg*>(m_base_addr + EVENT_CONTROL);
	p_EVENT_COUNT_REG = reinterpret_cast<device_reg*>(m_base_addr + EVENT_COUNT);

};


 /* ----------------------------------*/
 /* --- Public Methods ---------------*/
 /* ----------------------------------*/

/* --- Enable and reset methods --- */
 // true = counter enabled (= running), false = disabled
void TtcSingleTimer::enableTtc(void){
	// Clear the disable bit to enable the TTC:
	*p_COUNT_CONTROL_REG &= ~(1 << disable_counter_bit_pos);
}

void TtcSingleTimer::disableTtc(void){
	// Set the disable bit to disable the TTC:
	*p_COUNT_CONTROL_REG |= (1 << disable_counter_bit_pos);
}

// Resets the counter by writing 1; automatically cleared.
void TtcSingleTimer::resetCounter(void){
	*p_COUNT_CONTROL_REG |= (1 << reset_counter_bit_pos);
}

/* --- Output waveform methods --- */
// true = output enabled, false = output disabled
void TtcSingleTimer::setOutputWaveformOnOff(bool on_off){
	// Bit is active low i.e. clear bit to enable the TTC output.
	if (on_off) { *p_COUNT_CONTROL_REG &= ~(1 << output_wave_enable_bit_pos); }
	else { *p_COUNT_CONTROL_REG |= (1 << output_wave_enable_bit_pos); }
}

// true = active high, false = active low
void TtcSingleTimer::setOutputWaveformPolarity(bool polarity){
	if (polarity == true) { *p_COUNT_CONTROL_REG |= (1 << output_wave_polarity_bit_pos); }
	else { *p_COUNT_CONTROL_REG &= ~(1 << output_wave_polarity_bit_pos); }
}


/* --- Mode ---
 * overflow_interval_mode: false = overflow, true = interval.
 * match_mode: false = disable match mode, true = enable match mode.
 * decrement: false = timer increments, true = timer decrements. */
void TtcSingleTimer::setMode(bool overflow_interval_mode, bool match_mode, bool decrement){

	// std::uint32_t mode {0};

	if(overflow_interval_mode) { *p_COUNT_CONTROL_REG |= (1 << interval_overflow_mode_bit_pos); }
	if(match_mode) { *p_COUNT_CONTROL_REG |= (1 << match_mode_bit_pos); }
	if(decrement) { *p_COUNT_CONTROL_REG |= (1 << decrement_bit_pos); }

}

/* Enable/Disable the prescaler and set the value. */
void TtcSingleTimer::setPrescaler(bool enable, std::uint16_t value){

	// Set the prescaler value:
	value = value << prescaler_value_bit_pos;
	*p_CLOCK_CONTROL_REG &= value;

	// Enable/Disable prescaler:
	if (enable) { *p_CLOCK_CONTROL_REG |= (1 << prescaler_enable_bit_pos); }
	else { *p_CLOCK_CONTROL_REG &= ~(1 << prescaler_enable_bit_pos); }



}

/* Set the interval value for reset and restart in interval mode. */
void TtcSingleTimer::setIntervalValue(std::uint16_t interval){
	*p_INTERVAL_VALUE_REG = interval;
}


/* Set the match values to be used in match mode. */
void TtcSingleTimer::setMatchValues(std::uint16_t match0, std::uint16_t match1, std::uint16_t match2){

	*p_MATCH0_VALUE_REG = match0;
	*p_MATCH1_VALUE_REG = match1;
	*p_MATCH2_VALUE_REG = match2;

}

// double version:
void TtcSingleTimer::setMatchValuesF(double match0_seconds, double match1_seconds, double match2_seconds){

	*p_MATCH0_VALUE_REG = static_cast<std::uint16_t>( match0_seconds / ttc_period_ns );
	*p_MATCH1_VALUE_REG = static_cast<std::uint16_t>( match1_seconds / ttc_period_ns );
	*p_MATCH2_VALUE_REG = static_cast<std::uint16_t>( match2_seconds / ttc_period_ns );
}

/* Get the counter value from COUNTER register */
std::uint16_t TtcSingleTimer::getCount(void){

	return static_cast<std::uint16_t>(*p_COUNT_VALUE_REG);
}

/* Interrupt Methods */
std::uint8_t TtcSingleTimer::getInterruptStatus(void){

	return static_cast<std::uint8_t>(*p_INTR_STATUS_REG);

}

void TtcSingleTimer::clearInterruptStatus(void){

	std::uint32_t status = *p_INTR_STATUS_REG;
	*p_INTR_STATUS_REG &= status;

}

void TtcSingleTimer::setInterruptEnable(std::uint8_t intr_enable)
{
	*p_INTR_ENABLE_REG = intr_enable;
}



/* User interrupt handler set/get */
void TtcSingleTimer::setUserIntrHandler(p_TtcIntrHandler p_handler){
	p_UserInterruptHandler = p_handler;
}

p_TtcIntrHandler TtcSingleTimer::getUserIntrHandler(){
	return p_UserInterruptHandler;
}


void TtcSingleTimer::printDetails(void)
{
	printf("m_ttc_id: %u\n", m_ttc_id);
	printf("m_timer_id: %u\n", m_timer_id);
	printf("m_base_addr: 0x%08x\n\n", m_base_addr);

	printf("CLOCK_CONTROL_REG = 0x%08x\n", *p_CLOCK_CONTROL_REG);
	printf("COUNT_CONTROL_REG = 0x%08x\n", *p_COUNT_CONTROL_REG);
	printf("COUNT_VALUE_REG = 0x%08x\n", *p_COUNT_VALUE_REG);
	printf("INTERVAL_VALUE_REG = 0x%08x\n", *p_INTERVAL_VALUE_REG);
	printf("MATCH0_VALUE_REG = 0x%08x\n", *p_MATCH0_VALUE_REG);
	printf("MATCH1_VALUE_REG = 0x%08x\n", *p_MATCH1_VALUE_REG);
	printf("MATCH2_VALUE_REG = 0x%08x\n", *p_MATCH2_VALUE_REG);
	printf("INTR_STATUS_REG = 0x%08x\n", *p_INTR_STATUS_REG);
	printf("INTR_ENABLE_REG = 0x%08x\n", *p_INTR_ENABLE_REG);
	printf("EVENT_CONTROL_REG = 0x%08x\n", *p_EVENT_CONTROL_REG);
	printf("EVENT_COUNT_REG = 0x%08x\n", *p_EVENT_COUNT_REG);

}





/*****************************************************************************/
/************************** Triple Timer Counter *****************************/
/*****************************************************************************/
// This class represents a single Triple Timer Counter, which contains
// three TtcSingleTimer objects.


/* ----------------------------------*/
/* --- Constructor ------------------*/
/* ----------------------------------*/

Ttc::Ttc(std::uint16_t ttc_id)
{

	for(int timer_id=0; timer_id < n_timers_per_ttc; timer_id++)
	{
		TtcTimerArray[timer_id] = TtcSingleTimer(ttc_id, timer_id);
	}
}

/* ----------------------------------*/
/* --- Public Methods ---------------*/
/* ----------------------------------*/

void Ttc::printDetails(bool all, uint16_t timer_id){
	if(!all)
	{
		TtcTimerArray[timer_id].printDetails();
	}
	else
	{
		for(int t_id=0; t_id < n_timers_per_ttc; t_id++)
		{
			TtcTimerArray[t_id].printDetails();
		}
	}
}


