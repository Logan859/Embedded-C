/*
 * Modbus_Slave.c
 *
 *  Created on: Feb 20, 2022
 *      Author: DELL 5558
 */

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "Modbus_Slave.h"

volatile uint8_t uart_rx_byte = 0;

//Holding Register Array
static unsigned int HoldingRegisters[] =
{
	24288,	30717,	52317,	58320,	53844,	7596,	50869,	64892,	58209,	15124,	//0-9
	38131,	16852,	62343,	25973,	11963,	49434,	31745,	48278,	50955,	50027,	//10-19
	62940,	38520,	51144,	54693,	60394,	14244,	62402,	62647,	65457,	56695,	//20-29
	40971,	29608,	37659,	60486,	3321,	769,	52458,	61584,	55023,	61523,	//30-39
	50744,	51336,	3649,	48678,	29801,	43147,	1325,	32661,	49277,	32353,	//40-49
	48858,	41522,	32167,	13748,	24501,	2375,	6957,	45371,	53417,	34799,	//50-59
	51057,	3290,	13510,	48962,	65080,	22597,	38672,	14315,	48621,	49290,	//60-69
	30178,	1793,	13119,	54566,	1570,	31355,	1754,	21305,	5750,	25520,	//70-79
	23688,	54689,	64511,	17349,	33911,	31220,	64245,	17176,	43586,	30999,	//80-89
	12761,	42338,	62682,	7424,	6981,	36057,	63189,	4897,	52723,	61279,	//90-99
};


//Input Register Array
static unsigned int InputRegisters[] =
{
	24288,	30717,	52317,	58320,	53844,	7596,	50869,	64892,	58209,	15124,	//0-9
	38131,	16852,	62343,	25973,	11963,	49434,	31745,	48278,	50955,	50027,	//10-19
	62940,	38520,	51144,	54693,	60394,	14244,	62402,	62647,	65457,	56695,	//20-29
	40971,	29608,	37659,	60486,	3321,	769,	52458,	61584,	55023,	61523,	//30-39
	50744,	51336,	3649,	48678,	29801,	43147,	1325,	32661,	49277,	32353,	//40-49
	48858,	41522,	32167,	13748,	24501,	2375,	6957,	45371,	53417,	34799,	//50-59
	51057,	3290,	13510,	48962,	65080,	22597,	38672,	14315,	48621,	49290,	//60-69
	30178,	1793,	13119,	54566,	1570,	31355,	1754,	21305,	5750,	25520,	//70-79
	23688,	54689,	64511,	17349,	33911,	31220,	64245,	17176,	43586,	30999,	//80-89
	12761,	42338,	62682,	7424,	6981,	36057,	63189,	4897,	52723,	61279,	//90-99
};

static unsigned char Coils[] =
{
	0b110011,	0b101111,	0b10001100,	0b10001010,	0b100011,	0b10110,	0b1011011,	0b10001110,	0b11101011,	0b1001111,		//0-9
	0b0,		0b11101001,	0b10011011,	0b11001111,	0b1001101,	0b100110,	0b10001101,	0b11010111,	0b10011010,	0b11001101,		//10-19
	0b11011011,	0b100010,	0b1111001,	0b1111100,	0b1001101,	0b10000011,	0b1010110,	0b10000001,	0b11011110,	0b11110110,		//20-29
	0b1110111,	0b111010,	0b1111111,	0b100101,	0b11111110,	0b1,		0b101,		0b1100100,	0b10110011,	0b10010000,		//30-39
	0b11010110,	0b10110,	0b1110101,	0b1011101,	0b1011010,	0b11100,	0b11010110,	0b1111011,	0b10011110,0b11111010		//40-49
};

static unsigned char DiscreteInputs[] =
{
	0b00110011,	0b00101111,	0b10001100,	0b10001010,	0b00100011,	0b00010110,	0b01011011,	0b10001110,	0b11101011,	0b01001111,		//0-9
	0b00000000,	0b11101001,	0b10011011,	0b11001111,	0b01001101,	0b00100110,	0b10001101,	0b11010111,	0b10011010,	0b11001101,		//10-19
	0b11011011,	0b0100010,	0b01111001,	0b01111100,	0b01001101,	0b10000011,	0b01010110,	0b10000001,	0b11011110,	0b11110110,		//20-29
	0b01110111,	0b0111010,	0b01111111,	0b00100101,	0b11111110,	0b00000001,	0b00000101,	0b01100100,	0b10110011,	0b10010000,		//30-39
	0b11010110,	0b0010110,	0b01110101,	0b01011101,	0b01011010,	0b00011100,	0b11010110,	0b01111011,	0b10011110, 0b11111010		//40-49
};

unsigned int CharCount;

void MB_Init(void)
{
	/* Set Modbus Slave ID */
	MY_SLAVE_ID = 1U;

	/* Clear all counters */
	ClearModbusCounters();

	/* Get size of arrays */
	HoldingRegSize       = (sizeof(HoldingRegisters) / sizeof(HoldingRegisters[0]));
	InputRegSize         = (sizeof(InputRegisters) / sizeof(InputRegisters[0]));
	CoilsRegsize         = (sizeof(Coils) / sizeof(Coils[0]));
	DiscreteInputRegsize = (sizeof(DiscreteInputs) / sizeof(DiscreteInputs[0]));

	/* Enable Hardware Interrupts for UART and Timers */
	MBUartInterrupts(&MB_UART_OBJECT);
	MBEnableTimeInterrupts(&TIMER_A_OBJECT);
	MBEnableTimeInterrupts(&TIMER_B_OBJECT);
}

//Starting Modbus function to check the PDU received
void CheckMBPDU(void)
{
	CharCount = 0;
	CharCount = TotalCharsReceived;
	TotalCharsReceived = 0;
	if (CharCount >= 4u)
	{
		//Check inbound frame CRC
		unsigned int crcvalue = CRC16(data_in, (CharCount - 2));

		if ((data_in[CharCount - 2] == (unsigned char) (crcvalue)) &      //lower byte at higher register
			(data_in[CharCount - 1] == (unsigned char) (crcvalue >> 8)))  //higher byte at lower register
		{
			BusMsgCount += 1;	//increment bus message counter
			if ((data_in[0] == MY_SLAVE_ID) | (data_in[0] == 0u))
			{
				SlaveMsgCount += 1;                         //Increment Slave message count
				SlaveNoRspCount += data_in[0] == 0 ? 1 : 0; //STEP 2: Check function code
				switch (data_in[1])
				{
					case 0x01:
						MBProcessBitsRead(Coils, CoilsRegsize);			//read coils
					break;

					case 0x02:
						MBProcessBitsRead(DiscreteInputs, DiscreteInputRegsize);//read discrete inputs
					break;

					case 0x03:
						MBProcessRegisterRead(HoldingRegisters, HoldingRegSize);//read holding register
					break;

					case 0x04:
						MBProcessRegisterRead(InputRegisters, InputRegSize);//read input register
					break;

					case 0x05:
						MBForceSingleCoil(Coils, CoilsRegsize);	//Write single coil
					break;

					case 0x06:
						MBPresetSingleRegister(HoldingRegisters, HoldingRegSize);//write single register
					break;

					case 0x08:
						MBProcessDiagnostics();
					break;

					case 0x0f:
						MBForceMultipleCoils(Coils, CoilsRegsize);//Write multiple coils
					break;

					case 0x10:
						MBPresetMultipleRegisters(HoldingRegisters, HoldingRegSize);//write multiple registers
					break;

					default:
					{
						MBException(0x01);             //Illegal function code 01
						MBSendData(ResponseFrameSize); //send data if not broadcast command
					}
					break;
				}
			}
		}
		else
		{
			BusCommErrCount += 1; //Increment bus communication error counter
		}
	}
	else
	{
		BusCommErrCount += 1; //Increment bus communication error counter
	}
}

//Count the number of registers
unsigned int MBRegisterCount(void)
{
	return (data_in[5] | (data_in[4] << 8));
}

//Get Starting Modbus Address
unsigned int MBStartAddress(void)	//Return requested start address
{
	return (data_in[3] | (data_in[2] << 8));
}

//Send data over USART (RS232)
void MBSendData(unsigned char count)	//Send final data over UART
{
	if (data_in[0] != 0)
	{
#if 0
		for (unsigned char c = 0; c < count; c++)
		{
			while (!( USART2->ISR & (1 << 7u)))
			{
			};	//wait till transmit buffer is empty
			USART2->TDR = data_in[c];
		}
#else
		if(count > 0)
		{
			MBSendDataOnUART((uint8_t *)data_in, count);
		}
#endif
	}
}

/*************** Append CRC ***************/
//C function to append CRC to Slave Modbus response PDU
void AppendCRCtoMBRegister(unsigned char packtop)//crc is calculated from slave id to last data byte
{
	unsigned short crcvalue = 0;
	crcvalue = CRC16(data_in, packtop);
	data_in[packtop] = (unsigned char) (crcvalue);//lower byte at higher register
	data_in[packtop + 1] = (unsigned char) (crcvalue >> 8);	//higher byte at lower register
	ResponseFrameSize = packtop + 2;
}
//************** Append CRC ***************

//**************  EXCEPTION  **************
//Write Exception code
void MBException(unsigned char exceptionCode)	//Exception code
{
	SlaveExErrCount += 1;			//Increment Slave Exception Error count
	data_in[1] |= 0x80;	//setting MSB of the function code (the exception flag)
	data_in[2] = exceptionCode; //Exception code. Also the last byte containing dat
	ResponseFrameSize = 3;		// 3 bytes to send. No crc calculation.
}
//**************  EXCEPTION  **************

//*********Modbus Register Read Operations*************
//C function related to Analog Read operations (Function code 03, 04)
void MBProcessRegisterRead(unsigned int *InArr, unsigned int InArrSize)
{
	//First check what is the count of registers requested
	unsigned int RegCount = MBRegisterCount();

	//| SLAVE_ID | FUNCTION_CODE | RETURN BYTES COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE            | N*2 BYTES | 2 BYTES |
	//So our final requested data should fit in above 256 size, so data should be max 256-6 bytes
	//As a safeguard we are also checking with maximum limits of query as per modbus function (m584 controller)
	if ((RegCount >= 1u) & (RegCount * 2 <= 251u) & (RegCount <= 125u))
	{
		//to check if the requested start and end addresses are available in out controller
		//Get to know the starting address of the requested data
		unsigned int StAddress = MBStartAddress();
		unsigned int EndAddress = StAddress + RegCount - 1u;

		//We will simply check if the end address is inside the size of our holding register
		if ((StAddress >= 0u) & (EndAddress <= (InArrSize - 1u)))
		{
			//Process the request
			data_in[2] = (unsigned char) (RegCount * 2);//fill the byte count in the data array
			AppendDatatoMBRegister(StAddress, RegCount, HoldingRegisters,
					data_in);	//fill data in the data register
			AppendCRCtoMBRegister(3 + RegCount * 2);
		}
		else
			MBException(0x02);	//Exception code 02 = ILLEGAL DATA ADDRESS
	}
	else
		MBException(0x03);	//Exception code 03 = ILLEGAL DATA VALUE

	MBSendData(ResponseFrameSize);		//send response if not broadcast message
}

//Append Data from unsigned integer Array to Modbus PDU during Read Operation
void AppendDatatoMBRegister(unsigned int StAddr, unsigned int count,
		unsigned int *inreg, volatile unsigned char *outreg)
{
	for (unsigned char c = 0; c < count; c++)
	{
		*(outreg + 3 + c * 2) = (unsigned char) (*(inreg + StAddr + c) >> 8);//MSB IN HIGHER BYTE
		*(outreg + 3 + (c * 2 + 1)) = (unsigned char) (*(inreg + StAddr + c));//LSB IN LOWER BYTE
	}
}
//*********Modbus Register Read Operations*************

//*********Modbus Register Write Operations*************
//C function to Write Single Analog register (Function code 06)
void MBPresetSingleRegister(unsigned int *InArr, unsigned int InArrSize)
{
	unsigned int StAddress = MBStartAddress();
	//exception code 03 cannot be done for function code 06
	//check of exception code 02
	if ((StAddress) <= (InArrSize - 1u))
	{
		*(InArr + StAddress) = (unsigned int) (data_in[4] << 8)
				| (unsigned int) (data_in[5]);	//MSB FIRST AND THEN LSB IS ORed
		ResponseFrameSize = CharCount;
	}
	else
		MBException(0x02);	//Exception code 02 = ILLEGAL DATA ADDRESS

	MBSendData(ResponseFrameSize);		//send response if not broadcast message
}

//C function to Write Multiple Analog registers (Function code 16)
void MBPresetMultipleRegisters(unsigned int *InArr, unsigned int InArrSize)
{

	//Get to know the starting address of the requested data
	unsigned int StAddress = MBStartAddress();
	unsigned int RegCount = MBRegisterCount();
	unsigned int EndAddress = StAddress + RegCount - 1;

	//| SLAVE_ID | FUNCTION_CODE | RETURN BYTES COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE            | N*2 BYTES | 2 BYTES |
	//So our final requested data should fit in above 256 size, so data should be max 256-6 bytes
	//As a safeguard we are also checking with maximum limits of query as per modbus function (m584 controller)
	if ((RegCount >= 1u) & (RegCount <= 100u) & (data_in[6] == RegCount * 2))//for fc16, number of bytes requested is embedded in modbus message
	{
		//We will simply check if the end address is inside the size of our holding register
		if ((StAddress >= 0) & (EndAddress <= (InArrSize - 1u)))
		{
			//Process the request
			WriteMBRegistertoData(StAddress, RegCount, data_in,
					HoldingRegisters);
			ResponseFrameSize = CharCount;
		}
		else
			MBException(0x02);	//Exception code 02 = ILLEGAL DATA ADDRESS
	}
	else
		MBException(0x03);	//Exception code 03 = ILLEGAL DATA VALUE

	MBSendData(ResponseFrameSize);		//send response if not broadcast message
}

//C function to Store Analog Write operations to unsigned integer Variable array
void WriteMBRegistertoData(unsigned int StAddr, unsigned int count,
		volatile unsigned char *inreg, unsigned int *outreg)
{
	for (unsigned char c = 0; c < count; c++)
		*(outreg + StAddr + c) = (unsigned int) (*(inreg + 7 + (c * 2)) << 8)
				| (unsigned int) (*(inreg + 7 + (c * 2) + 1));
}
//*********Modbus Register Write Operations*************

//*********Modbus Discrete Read Operations*************
//C function related to Discrete Read operations (Function code 01, 02)
void MBProcessBitsRead(unsigned char *InArr, unsigned int InArrSize)
{
	//First check what is the count of bits requested
	unsigned int BitCount = MBRegisterCount();
	//As a safeguard we are also checking with maximum limits of query as per modbus function (m584 controller)

	unsigned char ByteCount = (
			(BitCount <= 8u) ?
					1u :
					((BitCount % 8u == 0) ?
							(BitCount / 8u) : ((BitCount / 8u) + 1u)));
	//| SLAVE_ID | FUNCTION_CODE | RETURN BYTES COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE            | N BYTES   | 2 BYTES |
	if ((BitCount >= 1u) & (BitCount <= 2000u))
	{
		//to check if the requested start and end addresses are available in out controller
		//As an example we configure 50 holding registers = 100 bytes of data array HoldingRegister
		//Get to know the starting address of the requested data
		unsigned int StAddress = MBStartAddress();			//start coil address
		unsigned int EndAddress = StAddress + BitCount - 1;	//end coil address

		if (EndAddress <= ((InArrSize * 8) - 1))
		{
			//Process the request
			data_in[2] = (unsigned char) ByteCount;	//fill the byte count in the data array

			//Ex. if master request 4 coil statuses, this means that 1 register is response.
			//We need to clear the remaining 4 bits of the data if there is any.
			//Else there will be error
			unsigned char regtoclear = (
					(BitCount < 8u) ?
							3u :
							((BitCount % 8u > 0) ?
									((unsigned char) (BitCount / 8u + 3u)) : 0));
			//clearing last byte of response array
			if (regtoclear > 0)
				data_in[(unsigned char) regtoclear] = 0x00;

			AppendBitsToRegisters(StAddress, BitCount, InArr, data_in);
			AppendCRCtoMBRegister(3 + ByteCount);
		}
		else
			MBException(0x02);		//Exception code 02 = ILLEGAL DATA ADDRESS
	}
	else
		MBException(0x03);			//Exception code 03 = ILLEGAL DATA VALUE

	MBSendData(ResponseFrameSize);		//send response if not broadcast message
}

//C function to append array bits to modbus pDU
void AppendBitsToRegisters(unsigned int StAddr, unsigned int count,
		unsigned char *inreg, volatile unsigned char *outreg)
{
	for (unsigned int c = 0; c < count; c++)
	{
		if (*(inreg + ((StAddr + c) / 8u))
				& (1 << ((StAddr + c) - (((StAddr + c) / 8u) * 8u))))//if in outreg array, bit is 1?
		{
			*(outreg + 3 + (c / 8)) |= (1 << (c - ((c / 8) * 8)));//then set bit 1 in target array
		}
		else
			*(outreg + 3 + (c / 8)) &= ~(1 << (c - ((c / 8) * 8)));	//else clear the bit in target array
	}
}
//*********Modbus Discrete Read Operations*************

//*********Modbus Discrete Write Operations*************
//C function for writing one coil (Function code 05)
void MBForceSingleCoil(unsigned char *InArr, unsigned int InArrSize)
{
	unsigned int StAddress = MBStartAddress();
	unsigned int outputvalue = MBRegisterCount();//using existing c function instead of creating a new one
	if ((outputvalue == 0x0000) | (outputvalue == 0xff00))
	{
		if (StAddress <= ((InArrSize * 8) - 1))
		{
			//Not calling separate function. Writing here itself
			if (outputvalue == 0xff00)
			{
				*(InArr + (StAddress / 8)) |= (1u
						<< (StAddress - ((StAddress / 8) * 8)));
			}
			else
				*(InArr + (StAddress / 8)) &= ~(1u
						<< (StAddress - ((StAddress / 8) * 8)));
			ResponseFrameSize = CharCount;
		}
		else
			MBException(0x02);		//Exception code 02 = ILLEGAL DATA ADDRESS
	}
	else
		MBException(0x03);		//Exception code 03 = ILLEGAL DATA VALUE

	MBSendData(ResponseFrameSize);		//send response if not broadcast message
}

//C function for writing multiple coils (Function code 15)
void MBForceMultipleCoils(unsigned char *InArr, unsigned int InArrSize)
{
	//First check what is the count of bits requested
	unsigned int BitCount = MBRegisterCount();
	unsigned int StAddress = MBStartAddress();

	//As a safeguard we are also checking with maximum limits of query as per modbus function (m584 controller)
	unsigned char ByteCount = (
			(BitCount <= 8u) ?
					1u :
					((BitCount % 8u == 0) ?
							(BitCount / 8u) : ((BitCount / 8u) + 1u)));
	//| SLAVE_ID | FUNCTION_CODE | RETURN BYTES COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE            | N BYTES   | 2 BYTES |
	if ((BitCount >= 1u) & (BitCount <= 800u) & (ByteCount == data_in[6]))
	{
		//to check if the requested start and end addresses are available in out controller
		//As an example we configure 50 holding registers = 100 bytes of data array HoldingRegister
		//Get to know the starting address of the requested data
		//start coil address
		unsigned int EndAddress = StAddress + BitCount - 1;	//end coil address

		if ((StAddress >= 0) & (EndAddress <= ((InArrSize * 8) - 1)))
		{
			//Process the requests
			WriteMBRegistersToBits(StAddress, BitCount, data_in, InArr);
			ResponseFrameSize = CharCount;
		}
		else
			MBException(0x02);	//Exception code 02 = ILLEGAL DATA ADDRESS
	}
	else
		MBException(0x03);	//Exception code 03 = ILLEGAL DATA VALUE

	MBSendData(ResponseFrameSize);		//send response if not broadcast message
}

//C function to store MODBUS master's discrete signals to Coil array
void WriteMBRegistersToBits(unsigned int StAddr, unsigned int count,
		volatile unsigned char *inreg, unsigned char *outreg)
{
	for (unsigned int c = 0; c < count; c++)
	{
		if (*(inreg + 7 + (c / 8u)) & (1 << (c - ((c / 8u) * 8u))))	//if in outreg array, bit is 1?
		{
			*(outreg + ((StAddr + c) / 8)) |= (1u
					<< ((StAddr + c) - (((StAddr + c) / 8) * 8)));
		}
		else
			*(outreg + ((StAddr + c) / 8)) &= ~(1u
					<< ((StAddr + c) - (((StAddr + c) / 8) * 8)));//else clear the bit in target array
	}
}
//*********Modbus Discrete Write Operations*************

void MBProcessDiagnostics(void)
{
	unsigned int subcode = MBStartAddress();//using existing c function instead of creating new one
	unsigned int Data = MBRegisterCount();	//using existing c function
	switch (subcode)
	{
		case 0x00:			//Return Query Data
			ResponseFrameSize = CharCount;
		break;

		case 0x01:			//Restart Communications Option
		break;

		case 0x02://Return Diagnostic Register - No diagnostic register in modbus slave
		break;

		case 0x03://Change ASCII Input Delimiter - Not for our application as we are using Modbus RTU
		break;

		case 0x04:			//Force Listen Only Mode
		break;

		case 0x0a:			//Clear Counters and Diagnostic Register
		{
			if (Data == 0x0000)
			{
				ClearModbusCounters();
				ResponseFrameSize = CharCount;
			}
			else
			{
				MBException(0x03);			//Exception code 03 = ILLEGAL DATA VALUE
			}
		}
		break;

		case 0x0b:			//Return Bus Message Count
		case 0x0c:			//Return Bus Communication Error Count
		case 0x0d:			//Return Bus Exception Error Count
		case 0x0e:			//Return Slave Message Count
		case 0x0f:			//Return Slave No Response Count
		case 0x10:			//Return Slave NAK Count
		case 0x11:			//Return Slave Busy Count
		case 0x012:			//Return Bus Character Overrun Count
		{
			if (Data == 0x0000)
			{
				unsigned int counterval = ReturnDiagCounter(subcode);
				data_in[4] = (counterval << 8);
				data_in[5] = (counterval);
				ResponseFrameSize = CharCount;
			}
			else
			{
				MBException(0x03);			//Exception code 03 = ILLEGAL DATA VALUE
			}
		}
		break;

		case 0x14:			//Clear Overrun Counter and Flag
		{
			if (Data == 0x0000)
			{
				BusChrOvrCount = 0;
				ResponseFrameSize = CharCount;
			}
			else
			{
				MBException(0x03);			//Exception code 03 = ILLEGAL DATA VALUE
			}
			break;
		}

		default:
		{
			MBException(0x01); //Illegal function code 01
			MBSendData(ResponseFrameSize);		//send data if not broadcast command
		}
		break;
	}
	MBSendData(CharCount);	//echo back the data
}

unsigned int ReturnDiagCounter(unsigned int scode)
{
	switch (scode)
	{
		case 0x0b:
			return BusMsgCount;
		break;

		case 0x0c:
			return BusCommErrCount;
		break;

		case 0x0d:
			return SlaveExErrCount;
		break;

		case 0x0e:
			return SlaveMsgCount;
		break;

		case 0x0f:
			return SlaveNoRspCount;
		break;

		case 0x10:
			return SlaveNAKCount;
		break;

		case 0x11:
			return SlaveBusyCount;
		break;

		case 0x12:
			return BusChrOvrCount;
		break;
	}
	return 0;
}

void ClearModbusCounters(void)
{
	BusMsgCount = 0;		//Bus Message Count
	BusCommErrCount = 0; 	//Bus Communication Error Count
	SlaveExErrCount = 0;	//Bus Exception Error Count
	SlaveMsgCount = 0;		//Slave Message Count
	SlaveNoRspCount = 0;	//Slave No Response Count
	SlaveNAKCount = 0;		//Slave NAK Count
	SlaveBusyCount = 0;		//Slave Busy Count
	BusChrOvrCount = 0;		//Bus Character Overrun Count
}

/* Table of CRC values for high–order byte */
static unsigned char auchCRCHi[] =
{
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
	0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40
} ;

/* Table of CRC values for low–order byte */
static char auchCRCLo[] =
{
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
	0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
	0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
	0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
	0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
	0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
	0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
	0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
	0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
	0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
	0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
	0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
	0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
	0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
	0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
	0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
	0x40
};

unsigned short CRC16(volatile unsigned char *puchMsg, unsigned short usDataLen) /* The function returns the CRC as a unsigned short type   */
{
	//*puchMsg =  /* message to calculate CRC upon */
	//usDataLen =  /* quantity of bytes in message  */

	unsigned char uchCRCHi = 0xFF;  /* high byte of CRC initialized      */
	unsigned char uchCRCLo = 0xFF;  /* low byte of CRC initialized       */
	unsigned uIndex;                /* will index into CRC lookup table  */
	while (usDataLen--)             /* pass through message buffer       */
	{
		uIndex   = uchCRCLo ^ *puchMsg++; /* calculate the CRC  */
		uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex];
		uchCRCHi = auchCRCLo[uIndex];
	}
	return (uchCRCHi << 8 | uchCRCLo);
}

/******************************************************************************/
/*                 Peripheral Interrupt and Request APIs                      */
/******************************************************************************/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &MB_UART_OBJECT)
	{
		__HAL_TIM_DISABLE(&TIMER_A_OBJECT);
		__HAL_TIM_DISABLE(&TIMER_B_OBJECT);

		__HAL_TIM_SET_COUNTER(&TIMER_A_OBJECT, 0U);
		__HAL_TIM_SET_COUNTER(&TIMER_B_OBJECT, 0U);

		if (DataPos >= MB_PDU_SIZE)
		{
			DataPos = 0;
		}
		else
		{
			data_in[DataPos] = uart_rx_byte;
			DataPos += 1;
			TotalCharsReceived = DataPos;
		}

		__HAL_TIM_ENABLE(&TIMER_A_OBJECT);
		__HAL_TIM_ENABLE(&TIMER_B_OBJECT);

		MBUartInterrupts(huart); // Re-enable Rx Interrupts
		// Do we need to call MBEnableTimeInterrupts()?
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim == &TIMER_A_OBJECT)
	{
		/* STOP THE 1.5CHAR TIMER */
		//__HAL_TIM_DISABLE(&TIMER_A_OBJECT);
		__HAL_TIM_SET_COUNTER(&TIMER_A_OBJECT, 0U);

		/* clear the Indexer and start over */
		DataPos=0U;

		__HAL_TIM_CLEAR_FLAG(&TIMER_A_OBJECT, 0xFF);
	}
	else if(htim == &TIMER_B_OBJECT)
	{
		/* STOP THE 3.5CHAR TIMER */
		//__HAL_TIM_DISABLE(&TIMER_B_OBJECT);

		/* clear the Indexer */
		DataPos=0U;

		/* Process Modbus frame */
		CheckMBPDU();

		__HAL_TIM_CLEAR_FLAG(&TIMER_B_OBJECT, 0xFF);
	}
	else
	{
		;
	}
}

void MBEnableTimeInterrupts(TIM_HandleTypeDef* htim)
{
	HAL_TIM_Base_Start_IT(htim);
}

void MBUartInterrupts(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(huart,(uint8_t *)&uart_rx_byte, 1U);
}

void MBSendDataOnUART(uint8_t *data, uint32_t size)
{
	//=> RS485 GPIO toggle ?
	HAL_UART_Transmit(&MB_UART_OBJECT, data, size, 500);
	//=> RS485 GPIO toggle ?

	MBUartInterrupts(&MB_UART_OBJECT); // Re-enable Rx Interrupts
}

/******************************************************************************/
