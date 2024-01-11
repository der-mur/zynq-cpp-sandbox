/******************************************************************************
 * @Title		:	Zynq Fundamentals Software Project 4
 * @Filename	:	system_config.cpp
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

#define SYS_CONFIG_DEBUG 	1 // C-style debug prints

/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/

#include "system_config.h"


#ifdef __cplusplus
extern "C"
{
	#include "intr_sys.h" // intr_sys.c/h is C code.
}
#endif // __cplusplus


/*****************************************************************************/
/******************************* Namespaces **********************************/
/*****************************************************************************/

using namespace sys::timing; // Scutimer and Scuwdt settings
using namespace sys::ps_gpio; // PS GPIO settings
using namespace sys::ps_uart; // Tx/Rx buffer sizes

using namespace board::gpio::pin_names::ps;




/*****************************************************************************/
/******************************** Aliases ************************************/
/*****************************************************************************/

using device_reg = std::uint32_t volatile;




/************************** Variable Definitions ****************************/

/* Task trigger signals; Scope is local to this file.
 * Accessor functions are used to give access to other files. */
static bool trigger_task1;
static bool trigger_task2;




/*===========================================================================*/
/*     CREATE STATIC OBJECTS FOR ALL DRIVERS                                 */
/*===========================================================================*/

static ScuWdt	ScuWdtCore0(ps::reg::base_addr::scuwdt0);
static AxiGpio	AxiGpio0(axi::reg::base_addr::gpio0);
static PsGpio	PsGpio0;
static TtcSingleTimer TtcSingleTimer0_0(0, 0); // TTC 0, timer 0




/*****************************************************************************/
/****************** File-Scope Function Prototypes ***************************/
/*****************************************************************************/
// Declare the interrupt handlers in this scope:
void Ttc0_0_IntrHandler(void);


// Function to show the status of current boot
// i.e. it might indicate that WDT triggered:
static void checkRebootStatus(void);







/*****************************************************************************
 * Function: sys_init()
 *//**
 *
 * @brief		Initialises the system.
 *
 * @return Status (Success or Error)
 *
******************************************************************************/

Status sys_init(DriverPointers* p_DriverPointers){

	/* ----------------------------------------------------------------*/
	/* First, display the state of any reboot.                         */
	/* ----------------------------------------------------------------*/
#if SYS_CONFIG_DEBUG
	checkRebootStatus();
#endif


	/* ----------------------------------------------------------------*/
	/* Initialise the GIC. We use the original C code for this task.   */
	/* ----------------------------------------------------------------*/

	int gic_init_status = xScuGicInit(); // Status: 0 = Success, 1 = Failure.



	/* ----------------------------------------------------------------*/
	/* Create pointers to all static driver objects.                   */
	/* ----------------------------------------------------------------*/

	ScuWdt* p_ScuWdtCore0 = &ScuWdtCore0;
	AxiGpio* p_AxiGpio0 = &AxiGpio0;
	PsGpio* p_PsGpio = &PsGpio0;
	TtcSingleTimer* p_TtcSingleTimer0_0 = &TtcSingleTimer0_0;


/* Print some info */
#if SYS_CONFIG_DEBUG
	if (!gic_init_status) 		{ printf("SCUGIC initialisation succeeded.\n\r"); }
	else						{ printf("SCUGIC initialisation failed.\n\r"); }

	if (p_ScuWdtCore0) 			{ printf("SCUWDT initialisation succeeded.\n\r"); }
	else						{ printf("SCUWDT initialisation failed.\n\r"); }

	if (p_AxiGpio0) 			{ printf("AXI GPIO initialisation succeeded.\n\r"); }
	else						{ printf("AXI GPIO initialisation failed.\n\r"); }

	if (p_PsGpio) 				{ printf("PS7 GPIO initialisation succeeded.\n\r"); }
	else						{ printf("PS7 GPIO initialisation failed.\n\r"); }

	if (p_TtcSingleTimer0_0) 	{ printf("TTC0-0 initialisation succeeded.\n\r"); }
	else						{ printf("TTC0-0 initialisation failed.\n\r"); }
#endif



	/* ----------------------------------------------------------------*/
	/* Set system timing                                               */
	/* ----------------------------------------------------------------*/

	/* Disable the TTC before configuring it. */
	p_TtcSingleTimer0_0->disableTtc();

	/* Mode: Enable Match mode. */
	bool overflow_or_interval {false}; // If false, then timer works in interval mode
	bool match_mode {true};
	bool decrement {false};

	p_TtcSingleTimer0_0->setMode(overflow_or_interval, match_mode, decrement);


	/* Set the match values (see system/settings.h for the values).
	 * Note we are using an overloaded function, so use a cast
	 * to ensure there is no ambiguity. */
	p_TtcSingleTimer0_0->setMatchValues((double) match0_seconds,
										(double) match1_seconds,
										(double) match2_seconds);

	/* Set interrupts: Match 0 (bit 1) and Match 1 (bit 2) => 0x6. */
	p_TtcSingleTimer0_0->setInterruptEnable(0x06);

	/* Enable the TTC output. */
	p_TtcSingleTimer0_0->setOutputWaveformOnOff(true);

	/* Set the user interrupt handler and add it to the interrupt system. */
	p_TtcSingleTimer0_0->setUserIntrHandler(Ttc0_0_IntrHandler);
	addTtc0ToInterruptSystem(p_TtcSingleTimer0_0);



	/* ----------------------------------------------------------------*/
	/* Initialise and start the watchdog                               */
	/* ----------------------------------------------------------------*/

	p_ScuWdtCore0->configure(sys::timing::scuwdt0_timeout_seconds); // (see system/settings.h)
	p_ScuWdtCore0->start();



	/* ----------------------------------------------------------------*/
	/* Configure the processing system GPIO bank.                      */
	/* (We just need to configure bank 0.)                             */
	/* ----------------------------------------------------------------*/

	p_PsGpio->writeBankData(0, 0x0, GpioOperation::Clear);
	p_PsGpio->setBankDirection(0, bank_gpio_output[0]);
	p_PsGpio->setBankOutputEnable(0, bank_gpio_output[0]);




//////////////////////////////////////////////////////////////////////////////////

	/* ----------------------------------------------------------------*/
	/*    Enable interrupts if the GIC was initialised successfully.   */
	/* ----------------------------------------------------------------*/

	if (gic_init_status == XST_SUCCESS) { enableInterrupts(); }



	/* ----------------------------------------------------------------*/
	/* Update the DriverPointers struct (if pointers are valid)        */
	/* ----------------------------------------------------------------*/

	if (p_AxiGpio0) { p_DriverPointers->p_AxiGpio0 = p_AxiGpio0; }
	if (p_ScuWdtCore0) { p_DriverPointers->p_ScuWdtCore0 = p_ScuWdtCore0; }
	if (p_PsGpio) { p_DriverPointers->p_PsGpio = p_PsGpio; }
	if (p_TtcSingleTimer0_0) { p_DriverPointers->p_TtcSingleTimer0_0 = p_TtcSingleTimer0_0; }




	/* ----------------------------------------------------------------*/
    /* Check result of initialisation and return to calling function   */
	/* ----------------------------------------------------------------*/

	if( (p_AxiGpio0) && (p_TtcSingleTimer0_0) && (p_ScuWdtCore0) && (p_PsGpio) // Check that pointers are valid.
		&& (gic_init_status == XST_SUCCESS)) // Check that GIC was initialised.
	{
		return common::Status::Success;
	}
	else
	{
		return common::Status::Error;
	}

}








/*****************************************************************************
 * Function: Ttc0_0_IntrHandler()
 *//**
 *
 * @brief		Interrupt handler for the TTC0 Timer. Used in this project
 * 				for triggering tasks in the main code.
 *
 *
 * @details		The basic aim of this function is to assert trigger_task1
 * 				and trigger_task2 at the correct timing intervals. TTC0 is
 * 				configured in Match mode, and match registers 0 and 1 are used:
 * 				MATCH0 = trigger task 1.
 * 				MATCH1 = trigger task 2.
 *
 * 				The basic flow every time an interrupt occurs is:
 * 				(1) Read the TTC0 interrupt status.
 * 				(2) Clear the interrupt.
 * 				(3) If interrupt status = MATCH0:
 * 						(a) Assert trigger_task1.
 * 				(4) Else if interrupt status = MATCH1:
 * 						(a) Assert trigger_task2.
 * 						(b) Reset TTC0 count so that the task sequence can
 * 							start again.
 *
 *
 * @note		Match values are defined in ttc0_if.h
 *
****************************************************************************/

void Ttc0_0_IntrHandler(void){


	PsGpio0.writePin(PMOD_JF_PIN1, GpioOperation::Set); /// SET TEST SIGNAL: TIMNG INTERRUPT ///

	trigger_task1 = false;
	trigger_task2 = false;



	/* Read and clear TTC0 interrupts */
	std::uint32_t status_event {0};
	status_event = TtcSingleTimer0_0.getInterruptStatus();
	TtcSingleTimer0_0.clearInterruptStatus();


	/* Assert trigger_taskX depending on the MATCH interrupt. */
	if (0 != (0x2 & status_event)) // 0x2 = Match 0 interrupt in TTC interrupt status register
	{
		PsGpio0.writePin(PMOD_JF_PIN2, GpioOperation::Set); /// SET TEST SIGNAL: TRIGGER TASK 1 ///

		trigger_task1 = true;

		PsGpio0.writePin(PMOD_JF_PIN2, GpioOperation::Clear); /// CLEAR TEST SIGNAL: TRIGGER TASK 1 ///
	}
	else if (0 != (0x4 & status_event)) // 0x4 = Match 1 interrupt in TTC interrupt status register
	{
		PsGpio0.writePin(PMOD_JF_PIN3, GpioOperation::Set); /// SET TEST SIGNAL: TRIGGER TASK 2 ///

		trigger_task2 = true;
		TtcSingleTimer0_0.resetCounter();

		PsGpio0.writePin(PMOD_JF_PIN3, GpioOperation::Clear); /// CLEAR TEST SIGNAL: TRIGGER TASK 2 ///
	}
	else
		{}


	PsGpio0.writePin(PMOD_JF_PIN1, GpioOperation::Clear); /// CLEAR TEST SIGNAL: TIMNG INTERRUPT ///

}


/*****************************************************************************
 * Function: getTask1TriggerState()
 *//**
 *
 * @brief		Accessor function to allow top-level main() code to read
 * 				global trigger_task1 status.
 *
 * @return		Current value of trigger state.
 *
 * @note		None.
 *
******************************************************************************/

bool getTask1TriggerState(void)
{
	return trigger_task1;
}



/*****************************************************************************
 * Function: getTask2TriggerState()
 *//**
 *
 * @brief		Accessor function to allow top-level main() code to read
 * 				trigger_task2 status.
 *
 * @return		Current value of trigger state.
 *
 * @note		None.
 *
******************************************************************************/
bool getTask2TriggerState(void)
{
	return trigger_task2;
}





/*****************************************************************************
 * Function: checkRebootStatus()
 *//**
 *
 * @brief		Prints out the boot status. Useful to see if the watchdog
 *				caused the previous sutdown.
 *
******************************************************************************/

/*------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------
SLCR_REBOOT_STATUS_REG 				0xF8000258U

SLCR_RS_BOOTROM_ERR_CODE_MASK	 	0x0000FFFFU
SLCR_RS_SWDT_RST_MASK				0x00010000U
SLCR_RS_AWDT0_RST_MASK				0x00020000U
SLCR_RS_AWDT1_RST_MASK				0x00040000U
SLCR_RS_SLC_RST_MASK				0x00080000U
SLCR_RS_DBG_RST_MASK				0x00100000U
SLCR_RS_SRST_B_MASK					0x00200000U
SLCR_RS_POR_MASK					0x00400000U
SLCR_RS_RESV_MASK					0x00800000U
SLCR_RS_REBOOT_STATE_MASK			0xFF000000U
------------------------------------------------------------------------------*/

static void checkRebootStatus(void){

	device_reg *p_slcr_reboot_sts = reinterpret_cast<device_reg*>(0xF8000258);

	std::uint32_t slcr_reboot_sts = *p_slcr_reboot_sts;


	printf("\r\n-----------------------------------------------------------\r\n");
	printf("SLCR Reboot Status Register: \r\n");
	printf("SWDT_RST  = %x\r\n", ( (slcr_reboot_sts & 0x00010000) != 0) );
	printf("AWDT0_RST = %x\r\n", ( (slcr_reboot_sts & 0x00020000) != 0) );
	printf("AWDT1_RST = %x\r\n", ( (slcr_reboot_sts & 0x00040000) != 0) );
	printf("SLC_RST   = %x\r\n", ( (slcr_reboot_sts & 0x00080000) != 0) );
	printf("DBG_RST   = %x\r\n", ( (slcr_reboot_sts & 0x00100000) != 0) );;
	printf("SRST_B    = %x\r\n", ( (slcr_reboot_sts & 0x00200000) != 0) );
	printf("POR       = %x\r\n", ( (slcr_reboot_sts & 0x00400000) != 0) );
	printf("(Note: Power-cycle (POR) required to clear this register.)\r\n");
	printf("-----------------------------------------------------------\r\n\r\n");
}
