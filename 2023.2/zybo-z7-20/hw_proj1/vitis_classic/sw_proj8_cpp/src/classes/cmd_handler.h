/******************************************************************************
 * @Title		:	Command Handler (Header File)
 * @Filename	:	cmd_handler.h
 * @Author		:	Derek Murray
 * @Origin Date	:	12/12/2023
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


#ifndef CMD_HANDLER_H_
#define CMD_HANDLER_H_


/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/

#include <cstdint>




/*****************************************************************************/
/***************************** Class Interface *******************************/
/*****************************************************************************/

class CmdHandler
{
  public:
	/* --- CONSTRUCTOR --------- */
	CmdHandler();

	enum command{ WRITE_WORD = 0x00D3, READ_WORD = 0x00D4};


	/* --- METHODS ------------- */
	/* Main function to be used by comms block ISR */
	void handleCommand(std::uint8_t *rx_buffer, std::uint8_t *tx_buffer);




  private:

	/* -------- Command frame structure -------*/
	/*	-----------------------------------------
	*	| 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
	*	-----------------------------------------
	*	|  CMD  |    FIELD 1    |    FIELD 2    |
	*	-----------------------------------------*/

	struct cmdFrame {
		std::uint16_t cmd {0};
		std::uint32_t field1 {0};
		std::uint32_t field2 {0};
	} CmdFrame ;


	/* --- METHODS ------------- */
	/* Functions internal to the command handler */
	void decodeRxData(std::uint8_t *rx_buffer);
	void executeCommand(std::uint8_t *tx_buffer);
	void setResponseBytes(std::uint8_t *tx_buffer, uint32_t tx_data);

};




#endif // CMD_HANDLER_H_
