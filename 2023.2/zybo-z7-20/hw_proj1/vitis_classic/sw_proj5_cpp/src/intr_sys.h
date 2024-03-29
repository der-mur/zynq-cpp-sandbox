/******************************************************************************
 * @Title		:	Interrupt System (Header File)
 * @Filename	:	intr_sys.h
 * @Author		:	Derek Murray
 * @Origin Date	:	18/12/2023
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

#ifndef SRC_INTR_SYS_H_
#define SRC_INTR_SYS_H_

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/

/* Low-level Xilinx */
#include "xscugic.h"
#include "xil_exception.h"



/* Must also include any files for drivers which will be added to intr sys: */
// c++ classes:
#include "classes/scutimer.h"


/*****************************************************************************/
/************************** Constant Definitions *****************************/
/*****************************************************************************/

#define INTR_SYS_DEBUG						0

/* Device ID */
#define PS7_SCUGIC_DEVICE_ID		XPAR_PS7_SCUGIC_0_DEVICE_ID


/* Interrupt IDs (from "xparameters_ps.h") */
#define SCU_TIMER_INTR_ID			XPS_SCU_TMR_INT_ID // SCU Private Timer interrupt, 29U


/* Interrupt priorities/triggers */

/* SCU PRIVATE TIMER */
#define SCU_TIMER_INTR_PRI			(0xA0)
#define SCU_TIMER_INTR_TRIG			(0x03)




/*****************************************************************************/
/************************** Variable Declarations ****************************/
/*****************************************************************************/



/*****************************************************************************/
/************************ Macros (Inline Functions) **************************/
/*****************************************************************************/

/* Ensure xil_printf.h is included: */
#ifndef SRC_XIL_PRINTF_H
#define SRC_XIL_PRINTF_H
#endif

#define printf 		xil_printf 	/* Small foot-print printf function */



/*****************************************************************************/
/************************** Function Prototypes ******************************/
/*****************************************************************************/

/* Device Initialization */
int xScuGicInit(void);


/* Interrupt configuration */
int addScuTimerToInterruptSystem(ScuTimer* p_ScuTimerCore0); // called by system_config.cpp



void enableInterrupts(void);
void disableInterrupts(void);

#ifdef __cplusplus
}
#endif

#endif /* SRC_INTR_SYS_H_ */
