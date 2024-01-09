/******************************************************************************
 * @Title		:	Common definition file
 * @Filename	:	common.h
 * @Author		:	Derek Murray
 * @Origin Date	:	07/12/2023
 * @Version		:	1.0.0
 * @Compiler	:	arm-none-eabi-gcc
 * @Target		: 	Xilinx Zynq-7000
 * @Platform	: 	Digilent Zybo-Z7-20
 *
 * @brief		:	Contains common definition.
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

#ifndef COMMON_H_
#define COMMON_H_

/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/

#ifdef __cplusplus
extern "C"
{
	#include "xil_printf.h"
	#define printf 		xil_printf 	/* Small foot-print printf function */
}
#endif // __cplusplus


/*****************************************************************************/
/******************************* Namespaces **********************************/
/*****************************************************************************/

namespace common
{
	enum class GpioOperation {Set, Clear, Toggle};
	enum class BankType {mio, emio};
	enum class Direction {Input, Output};

	enum class Status {Warning = -1, Error, Success};

	// Standard UART setting types
	enum class n_data_bits : std::uint16_t { six = 0, seven, eight};
	enum class parity : std::uint16_t  { even = 0, odd, forced_zero, forced_one, none };
	enum class n_stop_bits : std::uint16_t  { one = 0, onePtFive, two };
	enum class loopback_mode : std::uint16_t  { normal = 0, auto_echo, local, remote };

}

#endif // COMMON_H_
