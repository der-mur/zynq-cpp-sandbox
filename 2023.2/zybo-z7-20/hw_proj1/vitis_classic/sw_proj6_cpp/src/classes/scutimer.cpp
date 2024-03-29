/******************************************************************************
 * @Title		:	SCU Timer Driver Class Source file
 * @Filename	:	scutimer.cpp
 * @Author		:	Derek Murray
 * @Origin Date	:	11/12/2023
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

#include "scutimer.h"
#include <cstdint>

#include "../system/settings.h"


/*****************************************************************************/
/********************************* Aliases ***********************************/
/*****************************************************************************/

using device_reg = std::uint32_t volatile;


/*****************************************************************************/
/******************************* Namespaces **********************************/
/*****************************************************************************/

using namespace ps::reg::offset::scutimer;
using namespace ps::reg::offset::scuwdt;


/*****************************************************************************/
/************************* Constant Definitions ******************************/
/*****************************************************************************/

constexpr double timer_clk_period_ns = (1/static_cast<double>(sys::clock::core0_freq_hz))*2;


/*****************************************************************************/
/******************** Register Bit Positions and Masks ***********************/
/*****************************************************************************/

// Load Register bit positions and masks
static constexpr std::uint8_t timer_enable_bit_pos {0}; // bit 0 in CONTROL_REG
static constexpr std::uint8_t auto_reload_bit_pos {1}; // bit 1 in CONTROL_REG
static constexpr std::uint8_t irq_enable_bit_pos {2}; // bit 2 in CONTROL_REG

static constexpr std::uint8_t prescaler_bit_pos {8}; // Starts at bit 8 in CONTROL_REG
static constexpr std::uint16_t prescaler_mask {0xFF00}; // [15:8] in CONTROL_REG

static constexpr std::uint8_t watchdog_mode_bit_pos {3}; // bit 3 in CONTROL_REG




/* ----------------------------------*/
/* --- Constructor ------------------*/
/* ----------------------------------*/
ScuTimer::ScuTimer(std::uint32_t base)
{
	p_LOAD_REG = reinterpret_cast<device_reg*>(base + LOAD);
	p_COUNTER_REG = reinterpret_cast<device_reg*>(base + COUNTER);
	p_CONTROL_REG = reinterpret_cast<device_reg*>(base + CONTROL);
	p_ISR_REG = reinterpret_cast<device_reg*>(base + ISR);

}



/* ----------------------------------*/
/* --- Public Methods ---------------*/
/* ----------------------------------*/

void ScuTimer::configure(double time_seconds, bool auto_reload, bool irq_enable)
{
	ScuTimer::stop();
	ScuTimer::setLoad(time_seconds);
	ScuTimer::setAutoReload(auto_reload);
	ScuTimer::setIrqEnable(irq_enable);
}

void ScuTimer::start(void){
	*p_CONTROL_REG |= (1 << timer_enable_bit_pos);
}


void ScuTimer::stop(void){
	*p_CONTROL_REG &= ~(1 << timer_enable_bit_pos);
}


bool ScuTimer::getIntrStatus(void){
	// ISR reg just has bit 0 implemented. If bit 0 is set, interrupt has occurred:
	return *p_ISR_REG & 0x1;
}


void ScuTimer::clearIntrStatus(void){
	// Clear interrupt by writing 1 to interrupt status register (i.e. write 1 to bit 0) :
	*p_ISR_REG = 0x1;
}

void ScuTimer::waitTimerExpired(void){

	do {} while (ScuTimer::getIntrStatus() == false);
	ScuTimer::clearIntrStatus();
	ScuTimer::start();
}


/* User interrupt handler set/get */
void ScuTimer::setUserIntrHandler(p_ScuTimerIntrHandler p_handler){
	p_UserInterruptHandler = p_handler;
}

p_ScuTimerIntrHandler ScuTimer::getUserIntrHandler(){
	return p_UserInterruptHandler;
}


// test
void ScuTimer::printRegisters(void){

	printf("LOAD = 0x%08x\n", *p_LOAD_REG);
	printf("COUNTER = 0x%08x\n", *p_COUNTER_REG);
	printf("CONTROL = 0x%08x\n", *p_CONTROL_REG);
	printf("ISR = 0x%08x\n", *p_ISR_REG);

}



// Load Register
void ScuTimer::setLoad(double time_seconds){

	std::uint32_t load_value = static_cast<std::uint32_t>(time_seconds / timer_clk_period_ns);
	if (load_value == 0) { load_value++; }
	*p_LOAD_REG = load_value;
}

uint32_t ScuTimer::getLoad(void){
	return *p_LOAD_REG;
}


// Control Reg
void ScuTimer::setAutoReload(bool val){
	if (val) { *p_CONTROL_REG |= (1 << auto_reload_bit_pos); }
	else { *p_CONTROL_REG &= ~(1 << auto_reload_bit_pos); }
}

void ScuTimer::setIrqEnable(bool val){
	if (val) { *p_CONTROL_REG |= (1 << irq_enable_bit_pos); }
	else { *p_CONTROL_REG &= ~(1 << irq_enable_bit_pos); }
}

// !!! CHECK THIS FUNCTION !!!
void ScuTimer::setPrescaler(std::uint8_t prescaler){
	if (prescaler == 0) { *p_CONTROL_REG &= ~(prescaler_mask); }
	else { *p_CONTROL_REG |= (1 << prescaler_bit_pos); }
}

void ScuTimer::clearPrescaler(void){
	*p_CONTROL_REG &= ~(prescaler_mask);
}

uint16_t ScuTimer::getPrescaler(void){
	return (*p_CONTROL_REG & prescaler_mask) >> prescaler_bit_pos;
}
