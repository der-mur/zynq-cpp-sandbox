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






/*****************************************************************************/
/******************************* CONSTANTS ***********************************/
/*****************************************************************************/

/* ========= Buffers for UART/Command handling ========= */

/* Uart Buffer for sending data to host (see settings.h for buffer size) */
static std::uint8_t TxBuffer [tx_buffer_size] = {0};

/* Uart Buffer for receiving data from host (see settings.h for buffer size) */
static std::uint8_t RxBuffer [rx_buffer_size] = {0};




/*===========================================================================*/
/*     CREATE STATIC OBJECTS FOR ALL DRIVERS                                 */
/*===========================================================================*/

static ScuWdt	ScuWdtCore0(ps::reg::base_addr::scuwdt0);
static AxiGpio	AxiGpio0(axi::reg::base_addr::gpio0);
static PsGpio	PsGpio0;
static TtcSingleTimer TtcSingleTimer0_0(0, 0); // TTC 0, timer 0
static PsUart PsUart1(1, TxBuffer, tx_buffer_size, RxBuffer, rx_buffer_size);


//  CREATE STATIC OBJECT FOR COMMAND HANDLER
static CmdHandler CmdHandler0;



/*****************************************************************************/
/****************** File-Scope Function Prototypes ***************************/
/*****************************************************************************/
// Declare the interrupt handlers in this scope:
void PsUart1IntrHandler(void);
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
	PsUart* p_PsUart1 = &PsUart1;


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

	if (p_PsUart1) 				{ printf("Ps7 Uart1 initialisation succeeded.\n\r"); }
	else						{ printf("Ps7 Uart1  initialisation failed.\n\r"); }
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


	/* Set the match values (see system/settings.h for the values). */
	p_TtcSingleTimer0_0->setMatchValuesF(match0_seconds, match1_seconds, match2_seconds);

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



	/* ----------------------------------------------------------------*/
	/* Configure the Uart. It will be used for the command handler     */
	/* (Note that most of the init is done in the constructor.)        */
	/* ----------------------------------------------------------------*/

	/* First, ensure all interrupts are disabled. */
	p_PsUart1->disableAll_Interrupts();

	/* Enable rx_trig interrupt (= bit 0). Note that the Tx FIFO Empty
	 * Interrupt (Bit 3) is set when the interrupt handler is called. */
	p_PsUart1->enableInterrupts(0x1); // rx_trig, bit 0

	/* Set the user interrupt handler and add it to the interrupt system. */
	p_PsUart1->setUserIntrHandler(PsUart1IntrHandler);
	addUart1ToInterruptSystem(p_PsUart1);



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
 * Function: PsUart1IntrHandler()
 *//**
 *
 * @brief		Interrupt handler for PS7 UART1.
 *
 * @details		This is a very stripped down interrupt handler which just works
 * 				in this particular test application. There are two interrupts
 * 				enabled in the code: Rx FIFO Trigger (Bit 0) and Tx FIFO Empty
 * 				Interrupt (Bit 3). The interrupt handler checks for these two events
 * 				and ignores the rest.
 *
 * 				Note that the Tx FIFO Empty Interrupt (Bit 3) is set during
 * 				the PsUart::sendBytes() function in the PsUart class.
 *
 *
 * @note
 *
****************************************************************************/

void PsUart1IntrHandler(void)
 {


	/* 1. Read the enabled interrupts. */
	std::uint16_t enabled_interrupts {0};
	enabled_interrupts = PsUart1.readEnabledinterrupts();

	/* 2. Get the interrupt status. */
	std::uint16_t intr_status {0};
	intr_status = PsUart1.getinterruptStatus();

	/* 3. Clear interrupts. */
	PsUart1.clearInterrupts(intr_status);

	/* 4. Get the interrupt status based on the enabled interrupts. */
	uint16_t enabled_intr_status = intr_status & enabled_interrupts;


	// --------------------------------------------------------------------------------- //
	// Check if there was a Rx event i.e. was Rx Trigger interrupt detected? (Bit 0)
	// --------------------------------------------------------------------------------- //
	if ( (enabled_intr_status & 0x1) == 0x1 )
	{

		PsGpio0.writePin(PMOD_JF_PIN9, GpioOperation::Set); /// SET TEST SIGNAL: SET UART RX INTR ///

		/* === RX FROM HOST === */
		/* Get the data received from the host */
		PsUart1.recvBytes();

		/* Call function to handle the data */
		CmdHandler0.handleCommand(PsUart1.getRxBufferPtr(), PsUart1.getTxBufferPtr());

		// Dummy delay
		for (int i=0; i<=100; i++) {}

		/* === TX TO HOST === */
		/* Send the response data to the host.
		 * Note that sendBytes() will enable the Tx FIFO Empty Interrupt (Bit 3). */
		PsUart1.sendBytes();


		PsGpio0.writePin(PMOD_JF_PIN9, GpioOperation::Clear); /// SET TEST SIGNAL: CLEAR UART RX INTR ///

	}

	// --------------------------------------------------------------------------------- //
	// Check if there was a Tx event i.e. was Tx Empty Interrupt detected? (Bit 3).
	// --------------------------------------------------------------------------------- //
	else if ( (enabled_intr_status & 0x0008) == 0x0008 )
	{
		PsGpio0.writePin(PMOD_JF_PIN10, GpioOperation::Set); /// SET TEST SIGNAL: SET UART TX INTR ///

		// Dummy delay
		for (int i=0; i<=100; i++) {}

		PsGpio0.writePin(PMOD_JF_PIN10, GpioOperation::Clear); /// SET TEST SIGNAL: Clear UART TX INTR ///
	}


	// --------------------------------------------------------------------------------- //
	// All other events ignored for now.
	// --------------------------------------------------------------------------------- //
	else {}




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
