/******************************************************************************
 * @Title		:	Zynq Fundamentals Software Project 6
 * @Filename	:	sw_proj6_main.cpp
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



// Enable debug prints:
#define MAIN_DEBUG 		1

/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/

#include "system_config.h"
#include "tasks.h"


/*****************************************************************************/
/******************************* Namespaces **********************************/
/*****************************************************************************/

using namespace board::gpio::pin_names::axi;
using namespace board::gpio::pin_names::ps;
using namespace common;



/*****************************************************************************/
/******************************* Constants ***********************************/
/*****************************************************************************/


/* ------------------------- RUN-TIME PARAMETERS ---------------------- */

/* Toggle rate for LED0 if initialization fails */
static constexpr std::uint32_t INIT_FAIL_LOOP_DELAY		= 10000000;

/* Slow down LED toggle rates, when loop rate is very fast */

static constexpr std::uint32_t LED4_TOGGLE_COUNT = 10000;


/*****************************************************************************/
/************************** Function Prototypes ******************************/
/*****************************************************************************/




/*****************************************************************************/
/********************************** MAIN *************************************/
/*****************************************************************************/

int main()
{

	// ********************************************************************************* //
	// *****  INITIALIZATION PHASE *****
	// ********************************************************************************* //

#if MAIN_DEBUG
	printf("\n\r-----------------------------------------------------------\n\r");
	printf("----------- Zynq Fundamentals Software Project 6 ----------\n\r");
	printf("-----------------------------------------------------------\n\r");
	printf("Title: Program Timing using Triple Timer Counter.\r\n");
	printf("Architecture: FG/BG Polled State Machine.\r\n");
	printf("Timing: Triple Timer Counter 0, Wave 0.\r\n\r\n");

	printf("==== Initialising Drivers ====\r\n");
#endif

	/* Struct to store driver pointers. */
	DriverPointers DriverPointersInst;
	DriverPointers *p_DriverPointers = &DriverPointersInst;


	/* Variable for initialisation status */
	Status init_status;

	/* Run initialisation and update DriverPointersInst struct by reference. */
	init_status = sys_init(p_DriverPointers);

	if (init_status == Status::Success){
		p_DriverPointers->p_AxiGpio0->writeCh1Pin(LED0, GpioOperation::Set);
		#if MAIN_DEBUG
			printf("\n\rSystem ready: LED0 should be on.\n\r");
		#endif
	}
	else {
		#if MAIN_DEBUG
			printf("\n\r!!! LED 0 FLASHING: INITIALIZATION FAILED !!!\n\r");
		#endif
		while(1) { // Stay in this loop

		/* Toggle LED to show init failed, and 'kick' WDT
		 * so that debug can be carried out e.g. via XSCT.
		 * (Assumes that WDT was not the failing component.) */
		p_DriverPointers->p_AxiGpio0->writeCh1Pin(LED0, GpioOperation::Toggle);

		p_DriverPointers->p_ScuWdtCore0->restart();

		std::uint32_t delay = 0U;
		for (delay = 0; delay < INIT_FAIL_LOOP_DELAY; delay++)
			{}
		}
	}


	// ********************************************************************************* //
	// *****   MAIN PROGRAM [TASK STATE MACHINE ARCHITECTURE] *****
	// ********************************************************************************* //

	#if MAIN_DEBUG
		printf("\n\rRunning main program:\n\r");
		printf("LED 1: Task 1.\n\r");
		printf("LED 2: Task 2.\n\r");
		printf("LED 4: Watchdog timer.\n\r");
		printf("\n\r-----------------------------------------------------------\n\r\n\r");
	#endif


	for(;;) // Infinite loop
	{


		/* Use to slow down LED4 toggle rate, when loop rate is very fast */
		static std::uint32_t led4_count = 0U;

		/* Flags for WDT to know that task completed */
		static std::uint32_t task1_complete;
		static std::uint32_t task2_complete;


		static enum {
			INIT,
			TASK1,
			TASK2,
			SERVICE_WDT
		} state = INIT;


		/* ----- (1) INITIALIZATION  ----------------------------------- */
		switch (state) {
			case INIT:
				p_DriverPointers->p_TtcSingleTimer0_0->enableTtc();
				task1_complete = 0U;
				task2_complete = 0U;
				state = TASK1;
				break;




			/* ----- (2) RUN TASKS 1 AND 2 ----------------------------------- */
			/* For each task case:
			 * (a) Wait for task trigger signal.
			 * (b) Call the task.
			 * (c) When task returns, set 'taskX_complete' signal.
			 * (d) Set the next state.*/

			case TASK1:
				while (!getTask1TriggerState())
					{} // wait for trigger
				task1(p_DriverPointers->p_PsGpio, p_DriverPointers->p_AxiGpio0);
				task1_complete = 1U;
				state = TASK2;
				break;


			case TASK2:
				while (!getTask2TriggerState())
					{} // wait for trigger
				task2(p_DriverPointers->p_PsGpio, p_DriverPointers->p_AxiGpio0);
				task2_complete = 1;
				state = SERVICE_WDT;

				break;



			/* ----- (3) SERVICE WDT----------------------------------------- */
			/* Main aim is to check that task1 and task2 have completed.
			 * If they have, the watchdog will be serviced, the task_complete
			 * signals will be cleared, and the sequencer will go back to
			 * TASK1 state. If the tasks have not completed,then the sequencer
			 * stays in this state, the watchdog will not be serviced, and the
			 * system eventually resets.
			 *
			 * The LED4 count logic is used to slow the visible LED toggle rate
			 * when the system is running very fast.  */

			case SERVICE_WDT:
				p_DriverPointers->p_PsGpio->writePin(PMOD_JF_PIN8, GpioOperation::Set);	// TEST SIGNAL

				if ( (task1_complete == 1U) && (task2_complete == 1U) )
				{
					led4_count++;
					if (led4_count == LED4_TOGGLE_COUNT)
					{
						p_DriverPointers->p_PsGpio->writePin(LED4, GpioOperation::Toggle);
						led4_count = 0;
					}
					task1_complete = 0U;
					task2_complete = 0U;
					/* --- PET WATCHDOG --- */
					p_DriverPointers->p_ScuWdtCore0->restart();

					state = TASK1;
				}
				p_DriverPointers->p_PsGpio->writePin(PMOD_JF_PIN8, GpioOperation::Clear);	// TEST SIGNAL
				break;

			} /* End switch */

	}

	return 0;

}
