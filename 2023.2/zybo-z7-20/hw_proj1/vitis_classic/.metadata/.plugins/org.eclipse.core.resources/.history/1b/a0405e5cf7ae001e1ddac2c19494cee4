/******************************************************************************
 * @Title		:	Zynq Fundamentals Software Project 4
 * @Filename	:	system_config.h
 * @Author		:	Derek Murray
 * @Origin Date	:	05/12/2023
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

#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/

#include "system/settings.h" // for sys::timing and access to common namespace.

#include "classes/axi_gpio.h"
#include "classes/ps_gpio.h"
#include "classes/scuwdt.h"
#include "classes/ttc.h"




/*****************************************************************************/
/******************************* Namespaces **********************************/
/*****************************************************************************/

using namespace common; // for Status


/*****************************************************************************/
/******************************* Structs *************************************/
/*****************************************************************************/

// Struct for drivers that need to be shared.
struct DriverPointers{
	ScuWdt* p_ScuWdtCore0 {nullptr};
	AxiGpio* p_AxiGpio0 {nullptr};
	PsGpio* p_PsGpio {nullptr};
	TtcSingleTimer* p_TtcSingleTimer0_0 {nullptr};
};



/*****************************************************************************/
/************************** Function Prototypes ******************************/
/*****************************************************************************/

// Called by main.cpp to initialise the system:
Status sys_init(DriverPointers* p_DriverPointers);


/* Task functions */
bool getTask1TriggerState(void);
bool getTask2TriggerState(void);


#endif // SYSTEM_CONFIG_H
