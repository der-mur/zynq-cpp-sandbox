/******************************************************************************
 * @Title		:	SCU Watchdog Timer Driver Class Source file
 * @Filename	:	scuwdt.cpp
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

#include "scuwdt.h"

#include "../system/settings.h"


/*****************************************************************************/
/********************************* Aliases ***********************************/
/*****************************************************************************/

using device_reg = std::uint32_t volatile;



/*****************************************************************************/
/******************************* Namespaces **********************************/
/*****************************************************************************/

using namespace ps::reg::offset::scuwdt;



/*****************************************************************************/
/******************** Register Bit Positions and Masks ***********************/
/*****************************************************************************/

static constexpr std::uint8_t watchdog_mode_bit_pos {3}; // bit 3 in CONTROL_REG




/* ----------------------------------*/
/* --- Constructor ------------------*/
/* ----------------------------------*/

ScuWdt::ScuWdt(std::uint32_t base) : ScuTimer::ScuTimer(base)
{
	p_RESET_STATUS = reinterpret_cast<device_reg*>(base + RESET_STATUS);
	p_WATCHDOG_DISABLE = reinterpret_cast<device_reg*>(base + WATCHDOG_DISABLE);
}


/* ----------------------------------*/
/* --- Public Methods ---------------*/
/* ----------------------------------*/

void ScuWdt::configure(double time_seconds, bool auto_reload, bool irq_enable)
{
	ScuTimer::configure(time_seconds, auto_reload, irq_enable);
	ScuWdt::setWatchdogMode();
}

void ScuWdt::setWatchdogMode(void){
	*p_CONTROL_REG |= (1 << watchdog_mode_bit_pos);
}

void ScuWdt::clearWatchdogMode(void){
	*p_WATCHDOG_DISABLE = 0x12345678;
	*p_WATCHDOG_DISABLE = 0x87654321;
}

void ScuWdt::restart(void){
	*p_LOAD_REG = *p_LOAD_REG;
}

void ScuWdt::printRegisters(void){
	ScuTimer::printRegisters();
	printf("RESET_STATUS = 0x%08x\n", *p_RESET_STATUS);
	printf("WATCHDOG_DISABLE = 0x%08x\n",  *p_WATCHDOG_DISABLE);
}

