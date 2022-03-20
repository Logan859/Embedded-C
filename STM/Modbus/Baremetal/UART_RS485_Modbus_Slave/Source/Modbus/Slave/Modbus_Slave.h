/*
 * Modbus_Slave.h
 *
 *  Created on: Feb 20, 2022
 *      Author: DELL 5558
 */

#ifndef MODBUS_SLAVE_MODBUS_SLAVE_H_
#define MODBUS_SLAVE_MODBUS_SLAVE_H_

#define MCO_Pin               GPIO_PIN_0
#define MCO_GPIO_Port         GPIOF

#define USART2_TX_Pin         GPIO_PIN_2
#define USART2_TX_GPIO_Port   GPIOA
#define USART2_RX_Pin         GPIO_PIN_3
#define USART2_RX_GPIO_Port   GPIOA

#define TMS_Pin               GPIO_PIN_13
#define TMS_GPIO_Port         GPIOA

#define TCK_Pin               GPIO_PIN_14
#define TCK_GPIO_Port         GPIOA

#define MB_UART_OBJECT        huart2
#define TIMER_A_OBJECT        htim6
#define TIMER_B_OBJECT        htim7

#define MB_PDU_SIZE           256U

unsigned char MY_SLAVE_ID;

volatile unsigned char ResponseFrameSize;
volatile unsigned char data_in[MB_PDU_SIZE];
volatile unsigned int  DataPos;
volatile unsigned int  TotalCharsReceived;

unsigned int HoldingRegSize;
unsigned int InputRegSize;
unsigned int CoilsRegsize;
unsigned int DiscreteInputRegsize;

//Diagnostic counters
unsigned int BusMsgCount;		//Bus Message Count
unsigned int BusCommErrCount;	//Bus Communication Error Count
unsigned int SlaveExErrCount;	//Bus Exception Error Count
unsigned int SlaveMsgCount;		//Slave Message Count
unsigned int SlaveNoRspCount;	//Slave No Response Count
unsigned int SlaveNAKCount;		//Slave NAK Count
unsigned int SlaveBusyCount;	//Slave Busy Count
unsigned int BusChrOvrCount;	//Bus Character Overrun Count

void MB_Init(void);
void CheckMBPDU(void);
void ClearModbusCounters(void);
void AppendDatatoMBRegister(unsigned int StAddr,unsigned int count, unsigned int *inreg, volatile unsigned char *outreg);
void AppendCRCtoMBRegister(unsigned char packtop);
void AppendBitsToRegisters(unsigned int StAddr, unsigned int count, unsigned char *inreg, volatile unsigned char *outreg);
void WriteMBRegistertoData(unsigned int StAddr,unsigned int count, volatile unsigned char *inreg, unsigned int *outreg);
void WriteMBRegistersToBits(unsigned int StAddr, unsigned int count, volatile unsigned char *inreg,unsigned char *outreg);
void MBProcessDiagnostics(void);
void MBException(unsigned char exceptionCode);
void MBSendData(unsigned char count);
void MBProcessRegisterRead(unsigned int *InArr, unsigned int InArrSize);
void MBProcessBitsRead(unsigned char *InArr, unsigned int InArrSize);
void MBPresetSingleRegister(unsigned int *InArr, unsigned int InArrSize);
void MBPresetMultipleRegisters(unsigned int *InArr, unsigned int InArrSize);
void MBForceSingleCoil(unsigned char *InArr, unsigned int InArrSize);
void MBForceMultipleCoils(unsigned char *InArr, unsigned int InArrSize);
void MBSendDataOnUART(uint8_t *data, uint32_t size);
void MBUartInterrupts(UART_HandleTypeDef *huart);
void MBEnableTimeInterrupts(TIM_HandleTypeDef* htim);

unsigned int MBRegisterCount(void);
unsigned int MBStartAddress(void);
unsigned int ReturnDiagCounter(unsigned int scode);
unsigned short CRC16 (volatile unsigned char *puchMsg, unsigned short usDataLen );

#endif /* MODBUS_SLAVE_MODBUS_SLAVE_H_ */
