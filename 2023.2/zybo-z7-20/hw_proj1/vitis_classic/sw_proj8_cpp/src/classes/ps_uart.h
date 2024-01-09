/******************************************************************************
 * @Title		:	PS UART Driver Class Header file
 * @Filename	:	axi_gpio.h
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

#ifndef UART_H_
#define UART_H_


/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/

#include <cstdint>

#include "../system/common.h"


/*****************************************************************************/
/********************************* Aliases ***********************************/
/*****************************************************************************/

using device_reg = std::uint32_t volatile;

// Interrupt handler function pointer
using p_UartIntrHandler = void (*)(void);


/*****************************************************************************/
/***************************** Class Interface *******************************/
/*****************************************************************************/

class PsUart{
  public:
	/* --- CONSTRUCTOR --------- */
	PsUart(uint8_t uart_id, uint8_t *TxBuffer, uint8_t tx_buffer_size, uint8_t *RxBuffer, uint8_t rx_buffer_size);

	/* --- METHODS ------------- */

	// Enable/Disable/Reset
	void enable(void);
	void disable(void);
	void reset(void);


	// General
	void configureBaudRate(void); // TBD
	void configureGeneralSettings(common::n_data_bits data, common::parity par, common::n_stop_bits stop);

	void setNumberDataBits(common::n_data_bits ndata);
	void setNumberStopBits(common::n_stop_bits nstop);
	void setParity(common::parity par);



	// Send/Receive
	void initRxBuffer(uint8_t *p_buffer, uint8_t sz);
	void initTxBuffer(uint8_t *p_buffer, uint8_t sz);
	void setRxFifoTimeout(std::uint8_t timeout);
	void setRxFifoTrigLevel(std::uint8_t nbytes);
	void setTxFifoTrigLevel(std::uint8_t nbytes);
	std::uint8_t sendBytes(void); // Transmit whatever bytes are in the UART FIFO
	std::uint8_t recvBytes(void); // Receive whatever bytes are in the UART FIFO

	std::uint8_t* getRxBufferPtr(void);
	std::uint8_t* getTxBufferPtr(void);
	std::uint8_t getRxBufferSize(void);
	std::uint8_t getTxBufferSize(void);



	// Channel Status Register
	std::uint16_t readChannelStatusRegister(void);
	bool checkRxFifoEmpty(void);
	bool checkTxFifoFull(void);



	// Interrupts:
	void enableInterrupts(std::uint16_t intr_en_val);
	void disableInterrupts(std::uint16_t intr_dis_val);
	void disableAll_Interrupts(void);

	std::uint16_t readEnabledinterrupts(void);
	std::uint16_t getinterruptStatus(void);
	void clearInterrupts(std::uint16_t intr_clear_val);

	// Set/Get user interrupt handler
	void setUserIntrHandler(p_UartIntrHandler p_handler);
	p_UartIntrHandler getUserIntrHandler(void);


	// test
	void printDetails(void);






  private:
	std::uint32_t m_base_addr {0xE0000000};

	// Buffer details. Configured when constructor is called.
	struct Buffer{
		std::uint8_t *p_Buffer;
		std::uint8_t size; // 1-64 bytes
	};
	Buffer RxBuffer;
	Buffer TxBuffer;

	// Pointer to the user interrupt handler, which is declared and defined at a higher level.
	p_UartIntrHandler p_UserInterruptHandler;

	// Registers
	device_reg *p_CONTROL_REG {nullptr};
	device_reg *p_MODE_REG {nullptr};
	device_reg *p_INTR_EN_REG {nullptr};
	device_reg *p_INTR_DIS_REG {nullptr};

	device_reg *p_INTR_MASK_REG {nullptr};
	device_reg *p_INTR_STS_REG {nullptr};
	device_reg *p_BAUD_GEN_REG {nullptr};
	device_reg *p_RX_TIMEOUT_REG {nullptr};

	device_reg *p_RX_FIFO_TRIG_LVL_REG {nullptr};
	device_reg *p_MODEM_CTRL_REG {nullptr};
	device_reg *p_MODEM_STS_REG {nullptr};
	device_reg *p_CHANNEL_RAW_STS_REG {nullptr};

	device_reg *p_FIFO_REG {nullptr}; // ???
	device_reg *p_BAUD_RATE_DIV_REG {nullptr};
	device_reg *p_FLOW_DELAY_REG {nullptr};
	device_reg *p_TX_FIFO_TRIG_LVL_REG {nullptr};

	device_reg *p_RX_FIFO_BYTE_STS_REG {nullptr};

};


#endif // UART_H_
