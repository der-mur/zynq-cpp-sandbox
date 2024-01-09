/******************************************************************************
 * @Title		:	PS7 GPIO Driver Class Header file
 * @Filename	:	ps_gpio.h
 * @Author		:	Derek Murray
 * @Origin Date	:	07/12/2023
 * @Version		:	1.0.0
 * @Compiler	:	arm-none-eabi-gcc
 * @Target		: 	Xilinx Zynq-7000
 * @Platform	: 	Digilent Zybo-Z7-20
 *
 * @brief		:
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

#ifndef PS7_GPIO_H_
#define PS7_GPIO_H_

/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/

#include "../system/settings.h"


/*****************************************************************************/
/******************************* Namespaces **********************************/
/*****************************************************************************/

using namespace common;


/*****************************************************************************/
/********************************* Aliases ***********************************/
/*****************************************************************************/

using device_reg = std::uint32_t volatile;




/*****************************************************************************/
/***************************** Class Interface *******************************/
/*****************************************************************************/

class PsGpioBank
{
  public:
	// Constructors
	PsGpioBank() {}
	PsGpioBank(std::uint16_t bank_number);

	// Methods
	void setBankDirection(std::uint32_t direction);
	void setBankOutputEnable(std::uint32_t output_enable);
	void writeBankData(std::uint32_t data, common::GpioOperation op);

	void writeMaskedDataLower16(std::uint32_t mask, std::uint32_t data);
	void writeMaskedDataUpper16(std::uint32_t mask, std::uint32_t data);

	std::uint32_t readBankData(void);



	// temp
	void printDetails(void);

  private:
	// General parameters
	std::uint16_t m_bank_number {0};
	std::uint32_t m_bank_addr {0};
	BankType m_bank_type {BankType::mio };
	std::uint32_t m_bank_size_hex {0x0};
	std::uint32_t m_bank_size_offset {0};
	std::uint32_t m_bank_gpio_allowed {0};


	// Register pointers
	device_reg *p_MASK_DATA_LSW_REG { nullptr };
	device_reg *p_MASK_DATA_MSW_REG { nullptr };
	device_reg *p_DATA_WRITE_REG { nullptr };
	device_reg *p_DATA_READ_REG { nullptr };
	device_reg *p_DIR_MODE_REG { nullptr };
	device_reg *p_OUTPUT_ENABLE_REG { nullptr };
	device_reg *p_INTR_MASK_REG { nullptr };
	device_reg *p_INTR_ENABLE_UNMASK_REG { nullptr };
	device_reg *p_INTR_DISABLE_MASK_REG { nullptr };
	device_reg *p_INTR_STATUS_REG { nullptr };
	device_reg *p_INTR_TYPE_REG{ nullptr };
	device_reg *p_INTR_POLARITY_REG { nullptr };
	device_reg *p_INTR_ANY_EDGE_SENSE_REG { nullptr };

};


class PsGpio
{
  public:
	// Constructor
	PsGpio();

	void setBankDirection(std::uint16_t bank_number, std::uint32_t direction);
	void setBankOutputEnable(std::uint16_t bank_number, std::uint32_t output_enable);
	void writeBankData(std::uint16_t bank_number, std::uint32_t data, common::GpioOperation op);

	void writePin(std::uint16_t pin_number, common::GpioOperation op);

	// test
	void printDetails(std::uint16_t bank_number);


  private:
	std::int16_t getBankNumberFromPin(std::uint16_t pin);
	PsGpioBank PsGpioBankArray[sys::ps_gpio::n_banks];


};
#endif // PS7_GPIO_H_

