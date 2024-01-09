/******************************************************************************
 * @Title		:	Tasks
 * @Filename	:	tasks.cpp
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

#include "tasks.h"

#include <cstdint>
#include "system/settings.h" // for sys::timing and access to common namespace.


/*****************************************************************************/
/************************** Variable Declarations ****************************/
/*****************************************************************************/

/* Slow down LED toggle rates, when loop rate is very fast */
static constexpr std::uint32_t LED1_TOGGLE_COUNT = 2500; // Count max (LED toggles when value is reached)
static constexpr std::uint32_t LED2_TOGGLE_COUNT = 5000; // Count max (LED toggles when value is reached)

// Variables used in the tasks:
static volatile std::uint32_t led1_count = 0; // Task 1
static volatile std::uint32_t led2_count = 0; // Task 2


/*****************************************************************************/
/******************************* Namespaces **********************************/
/*****************************************************************************/

using namespace board::gpio::pin_names::axi;
using namespace board::gpio::pin_names::ps;
using namespace common;




/*****************************************************************************
 * Function: task1()
 *//**
 *
 * @brief		Simple task to toggle LED1. The led1_count variable is needed
 * 				to slow down the toggle rate when the code is running at a
 * 				very fast rate, as in that case the LED might appear to be
 * 				always on.
 *
 * @return		None.
 *
 * @note		None.
 *
******************************************************************************/

void task1(PsGpio *p_PsGpio, AxiGpio *p_AxiGpio)
{

	p_PsGpio->writePin(PMOD_JF_PIN4, GpioOperation::Set);		/// TEST SIGNAL

	led1_count++;

	if (led1_count >= LED1_TOGGLE_COUNT)
	{
		p_AxiGpio->writeCh1Pin(LED1, GpioOperation::Toggle);
		led1_count = 0;
	}


	/* Dummy delay for test purposes */
	for (std::uint32_t idx = 0; idx <= 50; idx++) {
		p_PsGpio->writePin(PMOD_JF_PIN4, GpioOperation::Set);		/// TEST SIGNAL
	}


	p_PsGpio->writePin(PMOD_JF_PIN4, GpioOperation::Clear);	/// TEST SIGNAL

}




/*****************************************************************************
 * Function: task2()
 *//**
 *
 * @brief		Simple task to toggle LED2. The led2_count variable is needed
 * 				to slow down the toggle rate when the code is running at a
 * 				very fast rate, as in that case the LED might appear to be
 * 				always on.
 *
 * @return		None.
 *
 * @note		None.
 *
******************************************************************************/


void task2(PsGpio *p_PsGpio, AxiGpio *p_AxiGpio)
{

	p_PsGpio->writePin(PMOD_JF_PIN7, GpioOperation::Set);		/// TEST SIGNAL

	led2_count++;

	if (led2_count >= LED2_TOGGLE_COUNT)
	{
		p_AxiGpio->writeCh1Pin(LED2, GpioOperation::Toggle);
		led2_count = 0;
	}


	/* Dummy delay for test purposes */
	for (std::uint32_t idx = 0; idx <= 50; idx++) {
		p_PsGpio->writePin(PMOD_JF_PIN7, GpioOperation::Set);		/// TEST SIGNAL
	}


	p_PsGpio->writePin(PMOD_JF_PIN7, GpioOperation::Clear);	/// TEST SIGNAL

}

/****** End functions *****/

/****** End of File **********************************************************/
