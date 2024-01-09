/******************************************************************************
 * @Title		:	AXI GPIO Driver Class Source file
 * @Filename	:	axi_gpio.h
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


/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/

#include "axi_gpio.h"
#include <cstdint>


/*****************************************************************************/
/********************************* Aliases ***********************************/
/*****************************************************************************/

using device_reg = std::uint32_t volatile;



/*****************************************************************************/
/******************************* Namespaces **********************************/
/*****************************************************************************/

using namespace axi::reg; // for register offsets




/* ----------------------------------*/
/* --- Constructor ------------------*/
/* ----------------------------------*/

AxiGpio::AxiGpio(std::uint32_t base)
{
	p_CH1_DATA_REG = reinterpret_cast<device_reg*>(base + offset::gpio::CH1_DATA);
	p_CH1_TRI_REG = reinterpret_cast<device_reg*>(base + offset::gpio::CH1_TRI);
	p_CH2_DATA_REG = reinterpret_cast<device_reg*>(base + offset::gpio::CH2_DATA);
	p_CH2_TRI_REG = reinterpret_cast<device_reg*>(base + offset::gpio::CH2_TRI);
 }




/* ----------------------------------*/
/* --- Public Methods ---------------*/
/* ----------------------------------*/

/* Channel 1 */

void AxiGpio::setCh1_Data(std::uint32_t value){
	*p_CH1_DATA_REG = value;
}

std::uint32_t AxiGpio::getCh1_Data(void){
	return *p_CH1_DATA_REG;
}

void AxiGpio::setCh1_Dir(std::uint32_t value){
	*p_CH1_TRI_REG = value;
}

std::uint32_t AxiGpio::getCh1_Dir(void){
	return *p_CH1_TRI_REG;
}


void AxiGpio::writeCh1Pin(std::uint16_t pin_number, common::GpioOperation op){

	// Set the bit to alter based on the pin number:
	std::uint32_t data {0};
	data |= (1 << pin_number);

	switch (op){
		case common::GpioOperation::Set:
			*p_CH1_DATA_REG |= data;
			break;

		case common::GpioOperation::Clear:
			*p_CH1_DATA_REG &= ~data;
			break;

		case common::GpioOperation::Toggle:
			*p_CH1_DATA_REG ^= data;
			break;
		default:
			break;
	}
}

/* End channel 1 */



/* Channel 2 */

void AxiGpio::setCh2_Data(std::uint32_t value){
	*p_CH2_DATA_REG = value;
}

std::uint32_t AxiGpio::getCh2_Data(void){
	return *p_CH2_DATA_REG;
}

void AxiGpio::setCh2_Dir(std::uint32_t value){
	*p_CH2_TRI_REG = value;
}

std::uint32_t AxiGpio::getCh2_Dir(void){
	return *p_CH2_TRI_REG;
}

void AxiGpio::writeCh2Pin(std::uint16_t pin_number, common::GpioOperation op){

	// Set the bit to alter based on the pin number:
	std::uint32_t data {0};
	data |= (1 << pin_number);

	switch (op){
		case common::GpioOperation::Set:
			*p_CH2_DATA_REG |= data;
			break;

		case common::GpioOperation::Clear:
			*p_CH2_DATA_REG &= ~data;
			break;

		case common::GpioOperation::Toggle:
			*p_CH2_DATA_REG ^= data;
			break;
		default:
			break;
	}
}

/* End channel 2 */
