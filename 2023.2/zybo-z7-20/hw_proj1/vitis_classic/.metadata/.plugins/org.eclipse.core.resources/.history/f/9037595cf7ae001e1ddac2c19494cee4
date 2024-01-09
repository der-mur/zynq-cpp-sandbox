/******************************************************************************
 * @Title		:	AXI Drivers Register File
 * @Filename	:	axi_reg.h
 * @Author		:	Derek Murray
 * @Origin Date	:	07/12/2023
 * @Version		:	1.0.0
 * @Compiler	:	arm-none-eabi-gcc
 * @Target		: 	Xilinx Zynq-7000
 * @Platform	: 	Digilent Zybo-Z7-20
 *
 * @brief		:	Contains AXI register base addresses and offsets.
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

#ifndef AXI_REG_H
#define AXI_REG_H

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

namespace axi
{
  namespace reg
  {
  	  namespace base_addr
	  {
      	  constexpr std::uint32_t gpio0			= std::uint32_t(XPAR_GPIO_0_BASEADDR);
      	  constexpr std::uint32_t spi0			= std::uint32_t(XPAR_SPI_0_BASEADDR);
	  } // base_addr



  	  // REGISTER OFFSETS
  	  namespace offset
	  {
  	  	  // GPIO OFFSETS
  	  	  namespace gpio
		  {
			constexpr std::uint16_t CH1_DATA		= 0;
			constexpr std::uint16_t CH1_TRI			= 0x0004;
			constexpr std::uint16_t CH2_DATA		= 0x0008;
			constexpr std::uint16_t CH2_TRI			= 0x000C;

			constexpr std::uint16_t GBL_INTR_EN		= 0x011C;
			constexpr std::uint16_t INTR_STATUS		= 0x0120;
			constexpr std::uint16_t INTR_ENABLE		= 0x0128;
		  }  // gpio
  	  } // offset


  } // reg
} // axi


#endif // AXI_REG_H
