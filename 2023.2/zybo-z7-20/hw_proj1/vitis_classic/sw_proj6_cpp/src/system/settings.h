/******************************************************************************
 * @Title		:	Settings file
 * @Filename	:	settings.h
 * @Author		:	Derek Murray
 * @Origin Date	:	07/12/2023
 * @Version		:	1.0.0
 * @Compiler	:	arm-none-eabi-gcc
 * @Target		: 	Xilinx Zynq-7000
 * @Platform	: 	Digilent Zybo-Z7-20
 *
 * @brief		:	Contains system and board settings.
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

#ifndef SETTINGS_H_
#define SETTINGS_H_

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
#include "common.h"

/*****************************************************************************/
/******************************* Namespaces **********************************/
/*****************************************************************************/

using namespace common;


namespace sys
{
	namespace clock
	{
  		constexpr std::uint32_t core0_freq_hz				= XPAR_PS7_CORTEXA9_0_CPU_CLK_FREQ_HZ;
  		constexpr std::uint32_t ttc0_freq_hz				= XPAR_XTTCPS_0_TTC_CLK_FREQ_HZ;
	} // clock

	namespace timing
	{
		// constexpr double scutimer0_time_seconds			= 10e-6; 	// 10us
		constexpr double scuwdt0_timeout_seconds			= 10; 		// 10s

		// TTC0-0 task timing
		constexpr double match0_seconds					= 20e-6;	// 20us
		constexpr double match1_seconds					= 50e-6;	// 50us
		constexpr double match2_seconds					= 0;

		constexpr std::uint16_t match0					= 2222;		// 20us (9ns x 2222)
		constexpr std::uint16_t match1					= 5556;		// 50us (9ns x 5556)
		constexpr std::uint16_t match2					= 0;		// n/a
	}

	namespace ttc
	{

	  constexpr std::uint8_t n_ttc				= 2;
	  constexpr std::uint8_t n_timers_per_ttc	= 3;
	  constexpr std::uint32_t ttc_base[6]		= {	XPAR_PS7_TTC_0_BASEADDR,
  			  	  	  	  	  	  	  	  	  	  	XPAR_PS7_TTC_1_BASEADDR,
													XPAR_PS7_TTC_2_BASEADDR,
													XPAR_PS7_TTC_3_BASEADDR,
													XPAR_PS7_TTC_4_BASEADDR,
													XPAR_PS7_TTC_5_BASEADDR };
  	} // ttc

	namespace ps_uart // Buffer sizes also used for command handler
	{
		constexpr std::uint8_t tx_buffer_size	= 4; // 4 bytes for command handler transmit side
		constexpr std::uint8_t rx_buffer_size	= 10; // 10 bytes for command handler receive side
	}





	namespace ps_gpio
	{
		constexpr std::uint8_t n_banks				= 4; // Four GPIO banks in Zynq7000

		// Bank 0: MIO; Bank 1: MIO; Bank 2: EMIO; Bank 3: EMIO;
		constexpr BankType bank_type[n_banks]		= { BankType::mio, BankType::mio, BankType::emio, BankType::emio };

		// Bank 0: Pins 0,7,9,10,11,12,13,14,15; Bank 1: 50, 51; Bank 2: All pins; Bank 3: All pins
		constexpr std::uint32_t bank_gpio_allowed[n_banks] = { 0x0000FE81, 0x00060000, 0xFFFFFFFF, 0xFFFFFFFF };

		// Bank size: Bank 0: 32-bits; Bank 1: 22-bits; Bank 2: 32-bits; Bank 3: 32-bits
		constexpr std::uint32_t	bank_size_hex[n_banks] = { 0xFFFFFFFF, 0x003FFFFF, 0xFFFFFFFF, 0xFFFFFFFF };
		constexpr std::uint32_t	bank_size_offsets[n_banks] = { 32, 53, 85, 117 };

		// Bank 0: Pins 0,7,9,10,11,12,13,14,15 set to output.
		constexpr std::uint32_t bank_gpio_output[n_banks] = { 0x0000FE81, 0x00000000, 0x00000000, 0x00000000 };

		constexpr std::uint32_t bank_base_addr[n_banks] = { XPAR_PS7_GPIO_0_BASEADDR,
															XPAR_PS7_GPIO_0_BASEADDR + 0x8,
															XPAR_PS7_GPIO_0_BASEADDR + 0x10,
															XPAR_PS7_GPIO_0_BASEADDR + 0x18 };

	} // ps_gpio
} // sys



namespace board
{
	namespace gpio
	{
		namespace pin_names
		{
			namespace axi
			{
				// CHANNEL 1 (ALL OUTPUTS)
				constexpr std::uint8_t LED0				= 0;
				constexpr std::uint8_t LED1				= 1;
				constexpr std::uint8_t LED2				= 2;
				constexpr std::uint8_t LED3				= 3;
				constexpr std::uint8_t PMOD_JE_PIN1		= 4;
				constexpr std::uint8_t PMOD_JE_PIN2		= 5;
				constexpr std::uint8_t PMOD_JE_PIN3		= 6;
				constexpr std::uint8_t PMOD_JE_PIN4		= 7;

				// CHANNEL 2 (ALL INPUTS)
				constexpr std::uint8_t BTN0				= 0;
				constexpr std::uint8_t BTN1				= 1;
				constexpr std::uint8_t BTN2				= 2;
				constexpr std::uint8_t BTN3				= 3;
				constexpr std::uint8_t SW0				= 4;
				constexpr std::uint8_t SW1				= 5;
				constexpr std::uint8_t SW2				= 6;
				constexpr std::uint8_t SW3				= 7;
				constexpr std::uint8_t PMOD_JE_PIN7		= 8;
				constexpr std::uint8_t PMOD_JE_PIN8		= 9;
				constexpr std::uint8_t PMOD_JE_PIN9		= 10;
				constexpr std::uint8_t PMOD_JE_PIN10	= 11;
			} // axi

			namespace ps
			{
				constexpr std::uint8_t LED4				= 7;	/** MIO 7 (!!! OUTPUT) */
				constexpr std::uint8_t PMOD_JF_PIN1		= 13;	/** MIO13, PMOD JF PIN 1*/
				constexpr std::uint8_t PMOD_JF_PIN2		= 10;	/** MIO10, PMOD JF PIN 2*/
				constexpr std::uint8_t PMOD_JF_PIN3		= 11;	/** MIO11, PMOD JF PIN 3*/
				constexpr std::uint8_t PMOD_JF_PIN4		= 12;	/** MIO12, PMOD JF PIN 4*/
				constexpr std::uint8_t PMOD_JF_PIN7		= 0;	/** MIO0, PMOD JF PIN 7*/
				constexpr std::uint8_t PMOD_JF_PIN8		= 9;	/** MIO9, PMOD JF PIN 8*/
				constexpr std::uint8_t PMOD_JF_PIN9		= 14;	/** MIO14, PMOD JF PIN 9*/
				constexpr std::uint8_t PMOD_JF_PIN10	= 15;	/** MIO15, PMOD JF PIN 10*/
				constexpr std::uint8_t BTN4				= 50;	/** MIO 50 (!!! INPUT) */
				constexpr std::uint8_t BTN5				= 51;	/** MIO 51 (!!! INPUT) */
			} // ps
		} // pin_names
	} // gpio
} // board

#endif // SETTINGS_H_
