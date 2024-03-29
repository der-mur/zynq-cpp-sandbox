/******************************************************************************
 * @Title		:	SCU Timer Driver Class Header file
 * @Filename	:	scutimer.h
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

#ifndef SCUTIMER_H
#define SCUTIMER_H

/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/

#include <cstdint>

#include "../system/ps_reg.h"


/*****************************************************************************/
/********************************* Aliases ***********************************/
/*****************************************************************************/

using device_reg = std::uint32_t volatile;


// Interrupt handler function pointer
using p_ScuTimerIntrHandler = void (*) (void);



/*****************************************************************************/
/***************************** Class Interface *******************************/
/*****************************************************************************/



class ScuTimer
{
  public:
	/* --- CONSTRUCTORS --------- */
	ScuTimer(std::uint32_t base);

	/* --- PUBLIC METHODS --------- */

	void configure(double time_seconds, bool auto_reload=true, bool irq_enable=true);
	void start(void); // Set the enable bit in CONTROL REG
	void stop(void); // Clear the enable bit in CONTROL REG
	bool getIntrStatus(void);
	void clearIntrStatus(void);
	void waitTimerExpired(void);


	void setLoad(double time_seconds);
	uint32_t getLoad(void);

	void setPrescaler(std::uint8_t prescaler);
	void clearPrescaler(void);
	uint16_t getPrescaler(void);

	void setAutoReload(bool val); // 'true' = enable, 'false' = disable AutoReload
	void setIrqEnable(bool val); // 'true' = enable, 'false' = disable IRQ


	// Set/Get user interrupt handler
	void setUserIntrHandler(p_ScuTimerIntrHandler p_handler);
	p_ScuTimerIntrHandler getUserIntrHandler(void);

	// test
	void printRegisters(void);

  protected: // Protected, as the ScuWdt timer class needs to inherit from this class.
	// Timer registers
	device_reg *p_LOAD_REG { nullptr };
	device_reg *p_COUNTER_REG { nullptr };
	device_reg *p_CONTROL_REG { nullptr };
	device_reg *p_ISR_REG { nullptr};

	p_ScuTimerIntrHandler p_UserInterruptHandler;

};


#endif // SCUTIMER_H






