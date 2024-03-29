/******************************************************************************
 * @Title		:	SCU Watchdog Timer Driver Class Header file
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

#ifndef SCUWDT_H
#define SCUWDT_H


/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/

#include "scutimer.h"


/*****************************************************************************/
/********************************* Aliases ***********************************/
/*****************************************************************************/

using device_reg = std::uint32_t volatile;


/*****************************************************************************/
/***************************** Class Interface *******************************/
/*****************************************************************************/

// ScuWdt is almost identical to the ScuTimer, with some extra settings or WDT mode.
// So we can easily inherit from ScuTimer:
class ScuWdt : public ScuTimer
{
  public:
	/* --- CONSTRUCTOR --------- */
	ScuWdt(std::uint32_t base);

	/* --- METHODS ------------- */

	void configure(double time_seconds, bool auto_reload=true, bool irq_enable=true);

	void setWatchdogMode(void);
	void clearWatchdogMode(void);
	void restart(void);

	// test
	void printRegisters(void);


  private:
	// Registers exclusive to watchdog mode:
	device_reg *p_RESET_STATUS {nullptr};
	device_reg *p_WATCHDOG_DISABLE {nullptr};

};

#endif // SCUWDT_H
