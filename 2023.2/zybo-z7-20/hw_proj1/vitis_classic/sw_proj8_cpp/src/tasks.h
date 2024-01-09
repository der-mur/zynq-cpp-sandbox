/******************************************************************************
 * @Title		:	Tasks
 * @Filename	:	tasks.h
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

#ifndef SRC_TASKS_H_
#define SRC_TASKS_H_

/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/

// Interface files
#include "classes/ps_gpio.h"
#include "classes/axi_gpio.h"



/*****************************************************************************/
/************************** Function Prototypes ******************************/
/*****************************************************************************/
/*=== Task logic === */
/* Tasks */
void task1(PsGpio *p_PsGpio, AxiGpio *p_AxiGpio);
void task2(PsGpio *p_PsGpio, AxiGpio *p_AxiGpio);


#endif /* SRC_TASKS_H_ */
