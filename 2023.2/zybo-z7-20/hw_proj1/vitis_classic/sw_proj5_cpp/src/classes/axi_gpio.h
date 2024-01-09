/******************************************************************************
 * @Title		:	AXI GPIO Driver Class Header file
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

#ifndef AXI_GPIO_H
#define AXI_GPIO_H

/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/

#include "../system/axi_reg.h"
#include "../system/common.h"



/*****************************************************************************/
/********************************* Aliases ***********************************/
/*****************************************************************************/

using device_reg = std::uint32_t volatile;



/*****************************************************************************/
/***************************** Class Interface *******************************/
/*****************************************************************************/


class AxiGpio
{
  public:
	/* --- CONSTRUCTOR --------- */
	AxiGpio(std::uint32_t base);

	/* --- METHODS ------------- */
	/* --- Channel 1 ----------- */
	void setCh1_Dir(std::uint32_t value);
	void setCh1_Data(std::uint32_t value);
	std::uint32_t getCh1_Dir(void);
	std::uint32_t getCh1_Data(void);
	void writeCh1Pin(std::uint16_t pin_number, common::GpioOperation op);


	/* --- Channel 2 ----------- */
	void setCh2_Dir(std::uint32_t value);
	void setCh2_Data(std::uint32_t value);
	std::uint32_t getCh2_Dir(void);
	std::uint32_t getCh2_Data(void);
	void writeCh2Pin(std::uint16_t pin_number, common::GpioOperation op);


  private:
	device_reg *p_CH1_DATA_REG { nullptr };
	device_reg *p_CH1_TRI_REG { nullptr };
	device_reg *p_CH2_DATA_REG { nullptr };
	device_reg *p_CH2_TRI_REG { nullptr };

};

#endif // AXI_GPIO_H
