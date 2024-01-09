/******************************************************************************
 * @Title		:	PS UART Driver Class Source file
 * @Filename	:	axi_gpio.cpp
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

#include "ps_uart.h"
#include <cstdint>

#include "../system/ps_reg.h"

/*****************************************************************************/
/********************************* Aliases ***********************************/
/*****************************************************************************/

using device_reg = std::uint32_t volatile;



/*****************************************************************************/
/******************************* Namespaces **********************************/
/*****************************************************************************/

using namespace ps::reg; // for register offsets



/*****************************************************************************/
/******************** Register Bit Positions and Masks ***********************/
/*****************************************************************************/
// Not all fields/registers covered.

/* --- Reg 0: CONTROL --- */
static constexpr std::uint8_t enable_disable_mask {0x3c};
static constexpr std::uint8_t uart_tx_rx_disable {0x28};
static constexpr std::uint8_t uart_tx_rx_enable {0x14};
static constexpr std::uint8_t uart_tx_rx_reset {0x03};

/* ------------------------------- */



/* --- Reg 1: Mode --- */
static constexpr std::uint8_t clk_source_bit_pos {0};
static constexpr std::uint8_t char_length_bit_pos {1};
static constexpr std::uint16_t char_length_mask {0x06}; // number of data bits, [2:1]
static constexpr std::uint8_t parity_bit_pos {3};
static constexpr std::uint16_t parity_mask {0x38}; // parity type, [5:3]
static constexpr std::uint8_t nstop_bit_pos {6};
static constexpr std::uint16_t nstop_mask {0xC0}; // number of stop bits, [7:6]
static constexpr std::uint8_t loopback_bit_pos {8};
static constexpr std::uint16_t loopback_mask {0x300}; // loopback mode, [9:8]
/* ------------------------------- */



/* --- Interrupt Registers: IER, IDR, IMR, ISR --- */
static constexpr std::uint16_t all_interrupts_mask {0x1FFF};

static constexpr std::uint16_t rx_trig_mask {0x0001};
static constexpr std::uint16_t rx_empty_mask {0x0002};
static constexpr std::uint16_t rx_full_mask {0x0004};
static constexpr std::uint16_t tx_empty_mask {0x0008};
//----
static constexpr std::uint16_t tx_full_mask {0x0010};
static constexpr std::uint16_t rx_overflow_mask {0x0020};
static constexpr std::uint16_t rx_framing_err_mask {0x0040};
static constexpr std::uint16_t rx_parity_err_mask {0x0080};
//----
static constexpr std::uint16_t rx_timeout_err_mask {0x0100};
static constexpr std::uint16_t delta_modem_sts_mask {0x0200};
static constexpr std::uint16_t tx_fifo_trig_mask {0x0400};
static constexpr std::uint16_t tx_fifo_nearly_full_mask {0x0800};
//----
static constexpr std::uint16_t tx_overflow_mask {0x1000};
/* ------------------------------- */



/* --- Channel Status register --- */
static constexpr std::uint32_t csr_rx_fifo_trig_mask				{0x0001};
static constexpr std::uint32_t csr_rx_fifo_empty_mask				{0x0002};
static constexpr std::uint32_t csr_rx_fifo_full_mask				{0x0004};
static constexpr std::uint32_t csr_tx_fifo_empty_mask				{0x0008};
//----
static constexpr std::uint32_t csr_tx_fifo_full_mask				{0x0010};
static constexpr std::uint32_t csr_rx_active_mask					{0x0400};
static constexpr std::uint32_t csr_tx_active_mask					{0x0800};
static constexpr std::uint32_t csr_rx_flow_delay_trig_mask			{0x1000};
//---
static constexpr std::uint32_t csr_tx_fifo_trig_mask				{0x2000};
static constexpr std::uint32_t csr_tx_fifo_nearly_full_mask			{0x4000};
/* ------------------------------- */







/* ----------------------------------*/
/* --- Constructor ------------------*/
/* ----------------------------------*/

PsUart::PsUart(uint8_t uart_id, uint8_t *p_TxBuffer, uint8_t tx_buffer_size, uint8_t *p_RxBuffer, uint8_t rx_buffer_size)
{
	if (uart_id == 1) { m_base_addr = base_addr::uart1; }

	// Registers
	p_CONTROL_REG = reinterpret_cast<device_reg*>(m_base_addr + offset::uart::CONTROL);
	p_MODE_REG = reinterpret_cast<device_reg*>(m_base_addr + offset::uart::MODE);
	p_INTR_EN_REG = reinterpret_cast<device_reg*>(m_base_addr + offset::uart::INTR_EN);
	p_INTR_DIS_REG = reinterpret_cast<device_reg*>(m_base_addr + offset::uart::INTR_DIS);

	p_INTR_MASK_REG = reinterpret_cast<device_reg*>(m_base_addr + offset::uart::INTR_MASK);
	p_INTR_STS_REG = reinterpret_cast<device_reg*>(m_base_addr + offset::uart::INTR_STS);
	p_BAUD_GEN_REG = reinterpret_cast<device_reg*>(m_base_addr + offset::uart::BAUD_GEN);
	p_RX_TIMEOUT_REG = reinterpret_cast<device_reg*>(m_base_addr + offset::uart::RX_TIMEOUT);

	p_RX_FIFO_TRIG_LVL_REG = reinterpret_cast<device_reg*>(m_base_addr + offset::uart::RX_FIFO_TRIG_LV);
	p_MODEM_CTRL_REG = reinterpret_cast<device_reg*>(m_base_addr + offset::uart::MODEM_CTRL);
	p_MODEM_STS_REG = reinterpret_cast<device_reg*>(m_base_addr + offset::uart::MODEM_STS);
	p_CHANNEL_RAW_STS_REG = reinterpret_cast<device_reg*>(m_base_addr + offset::uart::CHANNEL_RAW_STS);

	p_FIFO_REG = reinterpret_cast<device_reg*>(m_base_addr + offset::uart::FIFO);
	p_BAUD_RATE_DIV_REG = reinterpret_cast<device_reg*>(m_base_addr + offset::uart::BAUD_RATE_DIV);
	p_FLOW_DELAY_REG = reinterpret_cast<device_reg*>(m_base_addr + offset::uart::FLOW_DELAY);
	p_TX_FIFO_TRIG_LVL_REG = reinterpret_cast<device_reg*>(m_base_addr + offset::uart::TX_FIFO_TRIG_LVL);

	p_RX_FIFO_BYTE_STS_REG = reinterpret_cast<device_reg*>(m_base_addr + offset::uart::RX_FIFO_BYTE_STS);


	// Reset:
	PsUart::reset();

	// Set buffer details:
	PsUart::initRxBuffer(p_RxBuffer, rx_buffer_size);
	PsUart::initTxBuffer(p_TxBuffer, tx_buffer_size);

	/* By default, lets set a Rx timeout of 8. Probably not needed in this simple program.
	 * Can re-visit if necessary. */
	PsUart::setRxFifoTimeout(8);

}


/* ----------------------------------*/
/* --- Public Methods ---------------*/
/* ----------------------------------*/

void PsUart::disable(void){
	// (CONTROL_REG & (~0x3C)) | 0x28
	std::uint32_t reg_val = *p_CONTROL_REG;
	*p_CONTROL_REG = (reg_val & ~enable_disable_mask) | uart_tx_rx_disable;
}

void PsUart::reset(void){
	*p_CONTROL_REG |= uart_tx_rx_reset;
}

void PsUart::enable(void){
	std::uint32_t reg_val = *p_CONTROL_REG;
	*p_CONTROL_REG = (reg_val & ~enable_disable_mask) | uart_tx_rx_enable;
}

void PsUart::initRxBuffer(uint8_t *p_buffer, uint8_t sz)
{
	RxBuffer.p_Buffer = p_buffer;
	RxBuffer.size = sz;

	PsUart::setRxFifoTrigLevel(sz);
}

void PsUart::initTxBuffer(uint8_t *p_buffer, uint8_t sz)
{
	TxBuffer.p_Buffer = p_buffer;
	TxBuffer.size = sz;

	PsUart::setTxFifoTrigLevel(sz);
}




// Transfer the bytes from the UART HW FIFO to the SW RxBuffer (i.e. *p_buffer)
std::uint8_t PsUart::recvBytes(void)
{

	// 1. Disable interrupts:
	std::uint16_t enabled_interrupts {0};
	enabled_interrupts = PsUart::readEnabledinterrupts();
	PsUart::disableInterrupts(enabled_interrupts);



	/* 2. RECEIVE DATA */
	// While byte count less than buffer size and the FIFO is not empty, transfer receive data.
	std::uint8_t byte_count {0};

	while ( (byte_count < RxBuffer.size) && (PsUart::checkRxFifoEmpty() == false) )
	{
		// Transfer the data from the FIFO into the (software) RxBuffer:
		RxBuffer.p_Buffer[byte_count] = *p_FIFO_REG;
		byte_count++;
	} // End receive data.


	// Restore original interrupt setting
	PsUart::enableInterrupts(enabled_interrupts);

	return byte_count;
}




std::uint8_t PsUart::sendBytes(void)
{

	/* 1. Read the enabled interrupts to re-enable at the end of the fn. */
	std::uint16_t enabled_interrupts {0};
	enabled_interrupts = PsUart::readEnabledinterrupts();

	// 2. Disable Tx Interrupts before sending data.
	std::uint32_t intr_disable_val =  tx_empty_mask | tx_full_mask;
	PsUart::disableInterrupts(intr_disable_val);


	/* 3. SEND DATA */
	/* If the TX FIFO is full, send nothing; otherwise send the bytes in the Tx Buffer */
	std::uint8_t byte_count {0};

	while ( (checkTxFifoFull() == false) && (byte_count < TxBuffer.size) )
	{
		// Transfer the data from the UART (hardware) FIFO into the (software) RxBuffer:
		*p_FIFO_REG = TxBuffer.p_Buffer[byte_count];
		byte_count++;
	} // End send data.


	/* 4. Re-enable interrupts (and make sure tx_empty is enabled) */
	std::uint32_t val_to_write = enabled_interrupts |  tx_empty_mask;
	PsUart::enableInterrupts(val_to_write);

	return byte_count;

}

std::uint8_t* PsUart::getRxBufferPtr(void){
	return PsUart::RxBuffer.p_Buffer;
}

std::uint8_t* PsUart::getTxBufferPtr(void){
	return PsUart::TxBuffer.p_Buffer;
}

std::uint8_t PsUart::getRxBufferSize(void){
	return PsUart::RxBuffer.size;
}

std::uint8_t PsUart::getTxBufferSize(void){
	return PsUart::TxBuffer.size;
}


//--------------------------------


void PsUart::configureGeneralSettings(common::n_data_bits data, common::parity par, common::n_stop_bits stop)
{
	setNumberDataBits(data);
	setParity(par);
	setNumberStopBits(stop);
}



void PsUart::setNumberDataBits(common::n_data_bits ndata){

	std::uint16_t data_bits_val {0};

	switch (ndata){
		case common::n_data_bits::six:
			data_bits_val = 3;
			break;

		case common::n_data_bits::seven:
			data_bits_val = 2;
			break;

		case common::n_data_bits::eight:
			data_bits_val = 0;
			break;

		default:
			data_bits_val = 0;
			break;
	}

	data_bits_val = data_bits_val << char_length_bit_pos;
	*p_MODE_REG |= data_bits_val;

}


void PsUart::setNumberStopBits(common::n_stop_bits nstop){

	std::uint16_t stop_bits_val {0};

	switch (nstop){
		case common::n_stop_bits::one:
			stop_bits_val = 0;
			break;

		case common::n_stop_bits::onePtFive:
			stop_bits_val = 1;
			break;

		case common::n_stop_bits::two:
			stop_bits_val = 2;
			break;

		default:
			stop_bits_val = 0;
			break;
	}

	stop_bits_val = stop_bits_val << nstop_bit_pos;
	*p_MODE_REG |= stop_bits_val;

}

void PsUart::setParity(common::parity par){

	std::uint16_t parity_val {0};

	switch (par){
		case common::parity::even:
			parity_val = 0;
			break;

		case common::parity::odd:
			parity_val = 1;
			break;

		case common::parity::forced_zero:
			parity_val = 2;
			break;

		case common::parity::forced_one:
			parity_val = 3;
			break;

		case common::parity::none:
			parity_val = 4;
			break;

		default:
			parity_val = 0;
			break;
	}

	parity_val = parity_val << parity_bit_pos;
	*p_MODE_REG |= parity_val;

}


// Rx FIFO
void PsUart::setRxFifoTimeout(std::uint8_t timeout){
	*p_RX_TIMEOUT_REG = timeout;
}

void PsUart::setRxFifoTrigLevel(std::uint8_t nbytes){
	*p_RX_FIFO_TRIG_LVL_REG = nbytes;
}


// Tx FIFO
void PsUart::setTxFifoTrigLevel(std::uint8_t nbytes){
	*p_TX_FIFO_TRIG_LVL_REG = nbytes;
}



// Channel Status register
std::uint16_t PsUart::readChannelStatusRegister(void){
	return *p_CHANNEL_RAW_STS_REG;
}

// Check if Rx FIFO is empty, return true if so:
bool PsUart::checkRxFifoEmpty(void){
	return (*p_CHANNEL_RAW_STS_REG & csr_rx_fifo_empty_mask) == csr_rx_fifo_empty_mask;
}

// Check if Tx FIFO is full, return true if so:
bool PsUart::checkTxFifoFull(void){
	return (*p_CHANNEL_RAW_STS_REG & csr_tx_fifo_full_mask) == csr_tx_fifo_full_mask;
}



// Interrupt Methods

void PsUart::enableInterrupts(std::uint16_t intr_en_val){
	*p_INTR_EN_REG = intr_en_val;
}

void PsUart::disableInterrupts(std::uint16_t intr_dis_val){
	*p_INTR_DIS_REG = intr_dis_val;
}

void PsUart::disableAll_Interrupts(void){
	*p_INTR_DIS_REG = all_interrupts_mask;
}

std::uint16_t PsUart::readEnabledinterrupts(void){
	return *p_INTR_MASK_REG;
}

std::uint16_t PsUart::getinterruptStatus(void){
	return *p_INTR_STS_REG;
}


void PsUart::clearInterrupts(std::uint16_t intr_clear_val){
	*p_INTR_STS_REG = intr_clear_val;
}

void PsUart::setUserIntrHandler(p_UartIntrHandler p_handler)
{
	p_UserInterruptHandler = p_handler;
}

p_UartIntrHandler PsUart::getUserIntrHandler(void)
{
	return p_UserInterruptHandler;
}

// test
void PsUart::printDetails(void){
	printf("m_base_addr: 0x%08x\n", m_base_addr);

	printf("UART Control Register: 0x%08x\n", *p_CONTROL_REG);
	printf("UART Mode Register: 0x%08x\n", *p_MODE_REG);
	printf("Interrupt Enable Register: 0x%08x\n", *p_INTR_EN_REG);
	printf("Interrupt Disable Register: 0x%08x\n", *p_INTR_DIS_REG);

	printf("Interrupt Mask Register: 0x%08x\n", *p_INTR_MASK_REG);
	printf("Channel Interrupt Status Reg: 0x%08x\n", *p_INTR_STS_REG);
	printf("Baud Rate Generator Register: 0x%08x\n", *p_BAUD_GEN_REG);
	printf("Receiver Timeout Register: 0x%08x\n", *p_RX_TIMEOUT_REG);

	printf("Receiver FIFO Trigger Level Reg: 0x%08x\n", *p_RX_FIFO_TRIG_LVL_REG);
	printf("Modem Control Register: 0x%08x\n", *p_MODEM_CTRL_REG);
	printf("Modem Status Register: 0x%08x\n", *p_MODEM_STS_REG);
	printf("Channel Status Reg: 0x%08x\n", *p_CHANNEL_RAW_STS_REG);

	printf("Transmit and Receive FIFO Reg: 0x%08x\n", *p_FIFO_REG);
	printf("Baud Rate Divider Reg: 0x%08x\n", *p_BAUD_RATE_DIV_REG);
	printf("Flow Control Delay Reg: 0x%08x\n", *p_FLOW_DELAY_REG);
	printf("Transmitter FIFO Trigger Level: 0x%08x\n", *p_TX_FIFO_TRIG_LVL_REG);

}



