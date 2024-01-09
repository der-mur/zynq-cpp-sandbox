/******************************************************************************
 * @Title		:	PS7 GPIO Driver Class Source file
 * @Filename	:	ps_gpio.cpp
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

/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/

#include "ps_gpio.h"

#include "../system/ps_reg.h"

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

using namespace sys::ps_gpio; // Gpio bank details
using namespace ps::reg::offset::gpio; // Gpio registers
using namespace common;


/*****************************************************************************/
/********************************* Aliases ***********************************/
/*****************************************************************************/

using device_reg = std::uint32_t volatile;




/*****************************************************************************/
/****************************** PsGpioBank ***********************************/
/*****************************************************************************/


/* ----------------------------------*/
/* --- Constructor ------------------*/
/* ----------------------------------*/

PsGpioBank::PsGpioBank(std::uint16_t bank_number)
{
	// General settings
	m_bank_number = bank_number;
	m_bank_addr = bank_base_addr[bank_number];
	m_bank_type = bank_type[bank_number];
	m_bank_size_hex = bank_size_hex[bank_number];
	m_bank_size_offset = bank_size_offsets[bank_number];
	m_bank_gpio_allowed = bank_gpio_allowed[bank_number];

	// Register pointers
	p_MASK_DATA_LSW_REG = reinterpret_cast<device_reg*>(m_bank_addr + MASK_DATA_LSW);
	p_MASK_DATA_MSW_REG = reinterpret_cast<device_reg*>(m_bank_addr + MASK_DATA_MSW);
	p_DATA_WRITE_REG = reinterpret_cast<device_reg*>(m_bank_addr + DATA_WRITE - (m_bank_number*0x4));
	p_DATA_READ_REG = reinterpret_cast<device_reg*>(m_bank_addr + DATA_READ - (m_bank_number*0x4));

	p_DIR_MODE_REG = reinterpret_cast<device_reg*>(m_bank_addr + DIR_MODE + (m_bank_number*0x38));
	p_OUTPUT_ENABLE_REG = reinterpret_cast<device_reg*>(m_bank_addr + OUTPUT_ENABLE + (m_bank_number*0x38));
	p_INTR_MASK_REG = reinterpret_cast<device_reg*>(m_bank_addr + INTR_MASK + (m_bank_number*0x38));
	p_INTR_ENABLE_UNMASK_REG = reinterpret_cast<device_reg*>(m_bank_addr + INTR_ENABLE_UNMASK + (m_bank_number*0x38));
	p_INTR_DISABLE_MASK_REG = reinterpret_cast<device_reg*>(m_bank_addr + INTR_DISABLE_MASK + (m_bank_number*0x38));
	p_INTR_STATUS_REG = reinterpret_cast<device_reg*>(m_bank_addr + INTR_STATUS + (m_bank_number*0x38));
	p_INTR_TYPE_REG = reinterpret_cast<device_reg*>(m_bank_addr + INTR_TYPE + (m_bank_number*0x38));
	p_INTR_POLARITY_REG = reinterpret_cast<device_reg*>(m_bank_addr + INTR_POLARITY + (m_bank_number*0x38));
	p_INTR_ANY_EDGE_SENSE_REG = reinterpret_cast<device_reg*>(m_bank_addr + INTR_ANY_EDGE_SENSE + (m_bank_number*0x38));


}


/* ----------------------------------*/
/* --- Public Methods ---------------*/
/* ----------------------------------*/

void PsGpioBank::setBankDirection(std::uint32_t direction){
	std::uint32_t modifed_dir = m_bank_size_hex & m_bank_gpio_allowed & direction;
	*p_DIR_MODE_REG |= modifed_dir; // 0: input; 1: output
}


void PsGpioBank::setBankOutputEnable(std::uint32_t output_enable){
	std::uint32_t modifed_oe = m_bank_size_hex & m_bank_gpio_allowed & output_enable;
	*p_OUTPUT_ENABLE_REG |= modifed_oe; // 0: disabled; 1: enabled
}

void PsGpioBank::writeBankData(std::uint32_t data, common::GpioOperation op){
	//std::uint32_t modifed_data = m_bank_size & m_bank_gpio_allowed & data;

	switch (op){
	case GpioOperation::Set:
		*p_DATA_WRITE_REG |= data;
		break;

	case GpioOperation::Clear:
		*p_DATA_WRITE_REG &= ~data;
		break;

	case GpioOperation::Toggle:
		*p_DATA_WRITE_REG ^= data;
		break;
	default:
		break;
	}
}


void PsGpioBank::writeMaskedDataLower16(std::uint32_t mask, std::uint32_t data){
	std::uint32_t write_data = (mask << 16) | data;
	*p_MASK_DATA_LSW_REG = write_data;
}

void PsGpioBank::writeMaskedDataUpper16(std::uint32_t mask, std::uint32_t data){
	std::uint32_t write_data = (mask << 16) | data;
	*p_MASK_DATA_MSW_REG = write_data;
}

std::uint32_t PsGpioBank::readBankData(void){
	return *p_DATA_READ_REG;
}




// test
void PsGpioBank::printDetails(void)
{
	printf("bank_number: %i\n", m_bank_number);
	printf("bank_type: %x\n", (uint32_t)m_bank_type);
	printf("bank_size: 0x%08x\n", m_bank_size_hex);
	printf("bank_addr: 0x%08x\n", m_bank_addr);
	printf("bank_gpio_enabled: 0x%08x\n", m_bank_gpio_allowed);

	printf("MASK_DATA_LSW_REG: 0x%08x\n", *p_MASK_DATA_LSW_REG);
	printf("MASK_DATA_MSW_REG: 0x%08x\n", *p_MASK_DATA_MSW_REG);
	printf("DATA_WRITE_REG: 0x%08x\n", *p_DATA_WRITE_REG);
	printf("DATA_READ_REG: 0x%08x\n", *p_DATA_READ_REG);
	printf("DIR_MODE_REG: 0x%08x\n", *p_DIR_MODE_REG);
	printf("OUTPUT_ENABLE_REG: 0x%08x\n", *p_OUTPUT_ENABLE_REG);
}





/*****************************************************************************/
/******************************* PsGpio **************************************/
/*****************************************************************************/

/* ----------------------------------*/
/* --- Constructor ------------------*/
/* ----------------------------------*/

PsGpio::PsGpio(){
	for(int i=0; i <= n_banks; i++)
	{
		PsGpioBankArray[i] = PsGpioBank(i);
	}
}


/* ----------------------------------*/
/* --- Public Methods ---------------*/
/* ----------------------------------*/

void PsGpio::setBankDirection(std::uint16_t bank_number, std::uint32_t direction)
{
	PsGpioBankArray[0].setBankDirection(direction);
}


void PsGpio::setBankOutputEnable(std::uint16_t bank_number, std::uint32_t output_enable)
{
	PsGpioBankArray[0].setBankOutputEnable(output_enable);
}


void PsGpio::writeBankData(std::uint16_t bank_number, std::uint32_t data, common::GpioOperation op)
{
	PsGpioBankArray[bank_number].writeBankData(data, op);
}


void PsGpio::printDetails(std::uint16_t bank_number)
{
	PsGpioBankArray[bank_number].printDetails();
}


void PsGpio::writePin(std::uint16_t pin_number, common::GpioOperation op){

	// Set the bit to alter based on the pin number:
	std::uint32_t data {0};
	data |= (1 << pin_number);


	// Get the bank number associated with the pin_number:
	std::int16_t bank_number = 0;

	if (pin_number >= 0 && pin_number <= bank_size_offsets[0]) bank_number = 0;
	else if (pin_number <= bank_size_offsets[1]) bank_number = 1;
	else if (pin_number <= bank_size_offsets[2]) bank_number = 2;
	else if (pin_number <= bank_size_offsets[3]) bank_number = 3;
	else bank_number = -1; // Return -1 if out of range

	// Only carry out the operation if the bank_number is in range (i.e. != -1):
	if(bank_number >= 0) { PsGpioBankArray[bank_number].writeBankData(data, op); }
}




/* ----------------------------------*/
/* --- Private Methods ---------------*/
/* ----------------------------------*/

/* Bank size: Bank 0: 32-bits; Bank 1: 22-bits; Bank 2: 32-bits; Bank 3: 32-bits
   Bank 0: [0:31]; Bank 1: [32:53]; Bank 2: [54:85]; Bank 3: [86:117]; */
std::int16_t PsGpio::getBankNumberFromPin(std::uint16_t pin)
{

	if (pin >= 0 && pin <= bank_size_offsets[0]) return 0;
	else if (pin <= bank_size_offsets[1]) return 1;
	else if (pin <= bank_size_offsets[2]) return 2;
	else if (pin <= bank_size_offsets[3]) return 3;
	else return -1; // Return -1 if out of range

}


