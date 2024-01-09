/******************************************************************************
 * @Title		:	Processing System Drivers Register File
 * @Filename	:	ps_reg.h
 * @Author		:	Derek Murray
 * @Origin Date	:	07/12/2023
 * @Version		:	1.0.0
 * @Compiler	:	arm-none-eabi-gcc
 * @Target		: 	Xilinx Zynq-7000
 * @Platform	: 	Digilent Zybo-Z7-20
 *
 * @brief		:	Contains Processing System (PS) register base addresses and offsets.
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

#ifndef PS_REG_H_
#define PS_REG_H

/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/

// Include C file:
#ifdef __cplusplus
  extern "C" {
    #include "xparameters.h"
  }
#endif // __cplusplus

#include <cstdint>



/*****************************************************************************/
/******************************* Namespaces **********************************/
/*****************************************************************************/

namespace ps
{
  namespace reg
  {
  	  namespace base_addr
	  {
  	  	  constexpr std::uint32_t gpio				= XPAR_PS7_GPIO_0_BASEADDR;
  	  	  constexpr std::uint32_t scuwdt0			= XPAR_SCUWDT_0_BASEADDR;
  	  	  constexpr std::uint32_t scutimer0			= XPAR_XSCUTIMER_0_BASEADDR;
  	  	  constexpr std::uint32_t uart0				= 0xE0000000;
  	  	  constexpr std::uint32_t uart1				= XPAR_PS7_UART_1_BASEADDR;

  	  	  constexpr std::uint32_t ttc0_base			= XPAR_PS7_TTC_0_BASEADDR;
  	  	  constexpr std::uint32_t ttc1_base			= XPAR_PS7_TTC_1_BASEADDR;
  	  	  constexpr std::uint32_t ttc2_base			= XPAR_PS7_TTC_2_BASEADDR;
  	  	  constexpr std::uint32_t ttc3_base			= XPAR_PS7_TTC_3_BASEADDR;
  	  	  constexpr std::uint32_t ttc4_base			= XPAR_PS7_TTC_4_BASEADDR;
  	  	  constexpr std::uint32_t ttc5_base			= XPAR_PS7_TTC_5_BASEADDR;


	  }

  	  // REGISTER OFFSETS
  	  namespace offset
	  {
      	  // SCUTIMER OFFSETS (also used by derived ScuWdt class).
		namespace scutimer
		{
			constexpr std::uint8_t LOAD 				= 0;
			constexpr std::uint8_t COUNTER 				= 0x04;
			constexpr std::uint8_t CONTROL 				= 0x08;
			constexpr std::uint8_t ISR					= 0x0C;
		} // scutimer

		// SCUWDT OFFSETS (extra
		namespace scuwdt
		{
			constexpr std::uint8_t RESET_STATUS 		= 0x10;
			constexpr std::uint8_t WATCHDOG_DISABLE 	= 0x14;
		} // scutimer

		namespace gpio
		{
			constexpr std::uint8_t MASK_DATA_LSW		= 0;	// 0x8, 0x10, 0x18
			constexpr std::uint8_t MASK_DATA_MSW		= 0x4; 	// 0xC, 0x14, 0x1C
			constexpr std::uint8_t DATA_WRITE			= 0x40; // 0x44, 0x48, 0x4C
			constexpr std::uint8_t DATA_READ			= 0x60;	// 0x64, 0x68, 0x6C
			constexpr std::uint16_t DIR_MODE			= 0x204; // 0x244, 0x284, 0x2C4
			constexpr std::uint16_t OUTPUT_ENABLE		= 0x208;
			constexpr std::uint16_t INTR_MASK			= 0x20C;
			constexpr std::uint16_t INTR_ENABLE_UNMASK	= 0x210;
			constexpr std::uint16_t INTR_DISABLE_MASK	= 0x214;
			constexpr std::uint16_t INTR_STATUS			= 0x218;
			constexpr std::uint16_t INTR_TYPE			= 0x21C;
			constexpr std::uint16_t INTR_POLARITY		= 0x220;
			constexpr std::uint16_t INTR_ANY_EDGE_SENSE = 0x224;

			constexpr std::uint8_t DIR_MODE_BANK_OFFSET	= 0x40; // 0x40 OFFSET PER BANK
		}

		namespace ttc
		{
			constexpr std::uint8_t CLOCK_CONTROL			= 0;
			constexpr std::uint8_t COUNT_CONTROL			= 0xC;
			constexpr std::uint8_t COUNT_VALUE				= 0x18;
			constexpr std::uint8_t INTERVAL_VALUE			= 0x24;
			constexpr std::uint8_t MATCH0_VALUE				= 0x30;
			constexpr std::uint8_t MATCH1_VALUE				= 0x3C;
			constexpr std::uint8_t MATCH2_VALUE				= 0x48;
			constexpr std::uint8_t INTERRUPT_STATUS			= 0x54;
			constexpr std::uint8_t INTERRUPT_ENABLE			= 0x60;
			constexpr std::uint8_t EVENT_CONTROL			= 0x6C;
			constexpr std::uint8_t EVENT_COUNT				= 0x78;
		} // ttc

		namespace uart
		{
			constexpr std::uint8_t	CONTROL						= 0;
			constexpr std::uint8_t	MODE						= 0x4;
			constexpr std::uint8_t	INTR_EN						= 0x8;
			constexpr std::uint8_t	INTR_DIS					= 0xC;

			constexpr std::uint8_t	INTR_MASK					= 0x10;
			constexpr std::uint8_t	INTR_STS					= 0x14;
			constexpr std::uint8_t	BAUD_GEN					= 0x18;
			constexpr std::uint8_t	RX_TIMEOUT					= 0x1C;

			constexpr std::uint8_t	RX_FIFO_TRIG_LV				= 0x20;
			constexpr std::uint8_t	MODEM_CTRL					= 0x24;
			constexpr std::uint8_t	MODEM_STS					= 0x28;
			constexpr std::uint8_t	CHANNEL_RAW_STS				= 0x2C;

			constexpr std::uint8_t	FIFO						= 0x30;
			constexpr std::uint8_t	BAUD_RATE_DIV				= 0x34;
			constexpr std::uint8_t	FLOW_DELAY					= 0x38;
			constexpr std::uint8_t	TX_FIFO_TRIG_LVL			= 0x44;

			constexpr std::uint8_t	RX_FIFO_BYTE_STS			= 0x48;
		} // uart

	  } // offset

  } // reg
} // ps

#endif // PS_REG_H_


