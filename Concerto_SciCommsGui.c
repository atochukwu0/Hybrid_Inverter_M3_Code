//============================================================================
//============================================================================
//
// FILE:	SciCommsGui.c
//
// TITLE:	GP Comms kernel as an interface to external GUI
// 
// Version: 22 April 2009 - Release 1.2 - Internal Release (BRL)
//============================================================================
//============================================================================
//#include "DSP28x_Project.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_ipc.h"
#include "inc/hw_ram.h"
#include "inc/hw_uart.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/flash.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/ipc.h"
#include "driverlib/ram.h"
#include "driverlib/uart.h"
#include "utils/memcopy.h"

#define	PktSize				6
#define	CmdNumber			16
#define	MAX_CMD_NUM			8


// Function prototypes for Command RECEIVE State machine
// ------------------------------------------------------------
void GetCmdByte(void);
void EchoCmdByte(void);
void GetSizeByte(void);
void EchoSizeByte(void);
void GetDataByte(void);
void EchoDataByte(void);
void PackWord(void);
void PackLong(void);
void PackArray(void);
void CmdInterpreter(void);

// Function prototypes for Command Interpreter and dispatcher
// ------------------------------------------------------------
void LifePulseTsk(void);		// 0
void TextSet(void);				// 1
void ButtonSet(void);			// 2
void SliderSet(void);			// 3
void VariableGet(void);			// 4
void ArrayGet(void);			// 5
void DataGet(void);				// 6
void TextLongSet(void);			// 7
void TextLongGet(void);			// 8

void SendData(void);

// Variable declarations
void (*RcvTaskPointer)(void); 			// State pointer for Command Packet Receive
void (*CmdDispatcher[CmdNumber])(void);	// Array of pointers to Function (i.e. tasks)

extern int *varSetTxtList[];
extern int *varSetTxtLongList[];
extern int *varSetBtnList[];
extern int *varSetSldrList[];
extern int *varGetList[];
extern short int *arrayGetList[];
extern int *dataGetList[];

//int16 -> short int
extern int CommsOKflg, SerialCommsTimer;

//Uint16 -> unsigned short int
unsigned int	LowByteFlag, SendTaskPtr,ByteNum=3;
unsigned int	RxChar, RxWord;
unsigned int  RxLong;
unsigned int	CmdPacket[PktSize];
unsigned int	TaskDoneFlag, NumWords, wordsLeftToGet;

unsigned int dataOut;
int *memDataPtr;

unsigned short int dataOut_1;
short int *memDataPtr_1;

int  RcvTskPtrShdw;	// for debug

int 	delayer;

int	MemGetPtr;
//Uint32 -> unsigned int
unsigned int	MemGetAddress;
int	MemGetAmount;

unsigned long Temp;


void SCIA_Init()
{    
	int j = 0;
	
	RcvTaskPointer = &GetCmdByte;			// Initialize the CmdPacket Rcv Handler state machine ptr
	RcvTskPtrShdw = 1;						// DEBUG
	SendTaskPtr = 0;						// Init to 1st state
	LowByteFlag = 1;						// Start with LSB during Byte-to-Word packing

	dataOut = 0;
	*memDataPtr = 0;

	RcvTskPtrShdw = 0;	// for debug

	delayer = 0;

	MemGetPtr = 0;
	MemGetAddress = 0x00000000;
	MemGetAmount = 0;

	// clear Command Packet
	for (j=0; j<PktSize; j++)
	{
		CmdPacket[j] = 0x0;
	}

	j=0;
	
// init all dispatch Tasks                   
 	CmdDispatcher[0] = LifePulseTsk;             
 	CmdDispatcher[1] = TextSet;                  
 	CmdDispatcher[2] = ButtonSet;                
 	CmdDispatcher[3] = SliderSet;                
 	CmdDispatcher[4] = VariableGet;              
 	CmdDispatcher[5] = ArrayGet;                 
 	CmdDispatcher[6] = DataGet;                  
 	CmdDispatcher[7] = TextLongSet;               
 	CmdDispatcher[8] = TextLongGet;               
                                                                 
}

//===========================================================
// Host Command RECEIVE and DISPATCH State Machine
//===========================================================

//======== SM Entry Point =================
void SerialHostComms()
{		
	(*RcvTaskPointer)();	// Call routine pointed to by state pointer
}
//=========================================

void GetCmdByte(void) // Task 1
{
	if (!(HWREG(UART0_BASE + UART_O_FR) & UART_FR_RXFE))//(SciaRegs.SCIRXST.bit.RXRDY == 1)	// check if a char has been received
	{
		RxChar = HWREG(UART0_BASE  + UART_O_DR);//SciaRegs.SCIRXBUF.all;
		RcvTaskPointer = &EchoCmdByte;		// point to next state
		SerialCommsTimer = 0;
		//RcvTskPtrShdw = 2;				// DEBUG
		EchoCmdByte();
	}                                                                      
	else if(HWREG(UART0_BASE  + UART_O_RSR) & UART_RSR_BE)//(SciaRegs.SCIRXST.bit.BRKDT == 1 )//|| SerialCommsTimer > 2500) //~2 s timeout
	{	
	// If break detected or serialport times out, reset SCI
		//--- Needed by some serialports when code is run with an emulator  
 		//SciaRegs.SCICCR.all =0x0007;   		// 1 stop bit,  No loopback             
  	                                 		// No parity,8 char bits,           
  	                                  		// async mode, idle-line protocol   
 		//SciaRegs.SCICTL1.all =0x0003;  		// enable TX, RX, internal SCICLK,      
  	                                  		// Disable RX ERR, SLEEP, TXWAKE    
 		//SciaRegs.SCICTL2.all =0x0000;                                                                                
                                                                              
 		//SciaRegs.SCICTL1.all =0x0027;		// Relinquish SCI from Reset        
    	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED), 38400,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));
		//asm(" RPT#8 || NOP");
		//---

		SendTaskPtr = 0;					// Init to 1st state	
		SerialCommsTimer = 0;
								                                      
		CommsOKflg = 0;
		RcvTaskPointer = &GetCmdByte;		// go back and wait for new CMD                       
	}
}

void EchoCmdByte(void) // Task 2
{
    if(!(HWREG(UART0_BASE + UART_O_FR) & UART_FR_BUSY)) //(SciaRegs.SCICTL2.bit.TXRDY == 1)		// is TXBUF empty ?, i.e. TXRDY = 1
    {
    	HWREG(UART0_BASE  + UART_O_DR) = RxChar;//SciaRegs.SCITXBUF=RxChar;			// if yes, echo back the received char
    	CmdPacket[0] = RxChar;
		RcvTaskPointer = &GetSizeByte;//GetSizeByte;
		//RcvTskPtrShdw = 3;					// DEBUG
		//RcvTaskPointer = &GetCmdByte;		// Un-comment for simple echo test
		SerialCommsTimer = 0;						// Reset Time-out timer
	}

}

void GetSizeByte(void) // Task 3
{
	if(!(HWREG(UART0_BASE + UART_O_FR) & UART_FR_RXFE))//(SciaRegs.SCIRXST.bit.RXRDY == 1)	// check if a char has been received
	{
		RxChar = HWREG(UART0_BASE  + UART_O_DR);//SciaRegs.SCIRXBUF.all;

		RcvTaskPointer = &EchoSizeByte;		// point to next state
		//RcvTskPtrShdw = 4;					// DEBUG
		EchoSizeByte();
	}
    else if (HWREG(UART0_BASE  + UART_O_RSR) & UART_RSR_BE)//(SciaRegs.SCIRXST.bit.BRKDT == 1 )//|| SerialCommsTimer > 2500) //~2 s timeout
	{	
	// If break detected or serialport times out, reset SCI
		//--- Needed by some serialports when code is run with an emulator  
 		//SciaRegs.SCICCR.all =0x0007;   		// 1 stop bit,  No loopback             
  	                                 		// No parity,8 char bits,           
  	                                  		// async mode, idle-line protocol   
 		//SciaRegs.SCICTL1.all =0x0003;  		// enable TX, RX, internal SCICLK,      
  	                                  		// Disable RX ERR, SLEEP, TXWAKE    
 		//SciaRegs.SCICTL2.all =0x0000;                                                                                
                                                                              
 		//SciaRegs.SCICTL1.all =0x0027;		// Relinquish SCI from Reset        
    	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED), 38400,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));
		//asm(" RPT#8 || NOP");
		//---

		SendTaskPtr = 0;					// Init to 1st state	
		SerialCommsTimer = 0;
								                                      
		CommsOKflg = 0;
		RcvTaskPointer = &GetCmdByte;		// go back and wait for new CMD                       
	}
	else if (SerialCommsTimer > 1000)		// 1000*1mS = 1.0 sec timeout
	{
		CommsOKflg = 0;
		RcvTaskPointer = &GetCmdByte;		// Abort, go back wait for new CMD
		SerialCommsTimer = 0;
	}
}

void EchoSizeByte(void) // Task 4
{
    if(!(HWREG(UART0_BASE + UART_O_FR) & UART_FR_BUSY)) //(SciaRegs.SCICTL2.bit.TXRDY == 1)		// is TXBUF empty ?, i.e. TXRDY = 1
    {
    	HWREG(UART0_BASE  + UART_O_DR) = RxChar;//SciaRegs.SCITXBUF=RxChar;			// if yes, echo back the received char
    	CmdPacket[1] = RxChar;
		RcvTaskPointer = &GetDataByte;
		RxLong=0;
		//RcvTskPtrShdw = 5;				// DEBUG
		//RcvTaskPointer = &GetCmdByte;		// Un-comment for Test
		SerialCommsTimer = 0;				// Reset Time-out timer
    }
}

void GetDataByte(void) // Task 5
{
	if(!(HWREG(UART0_BASE + UART_O_FR) & UART_FR_RXFE))//(SciaRegs.SCIRXST.bit.RXRDY == 1)	// check if a char has been received
	{
		RxChar = HWREG(UART0_BASE  + UART_O_DR);//SciaRegs.SCIRXBUF.all;
		RcvTaskPointer = &EchoDataByte;		// point to next state
		//RcvTskPtrShdw = 6;				// DEBUG
		EchoDataByte();				
	}
	else if (HWREG(UART0_BASE  + UART_O_RSR) & UART_RSR_BE)//(SciaRegs.SCIRXST.bit.BRKDT == 1 )//|| SerialCommsTimer > 2500) //~2 s timeout
	{	
	// If break detected or serialport times out, reset SCI
		//--- Needed by some serialports when code is run with an emulator  
 		//SciaRegs.SCICCR.all =0x0007;   		// 1 stop bit,  No loopback             
  	                                 		// No parity,8 char bits,           
  	                                  		// async mode, idle-line protocol   
 		//SciaRegs.SCICTL1.all =0x0003;  		// enable TX, RX, internal SCICLK,      
  	                                  		// Disable RX ERR, SLEEP, TXWAKE    
 		//SciaRegs.SCICTL2.all =0x0000;                                                                                
                                                                              
 		//SciaRegs.SCICTL1.all =0x0027;		// Relinquish SCI from Reset        
    	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED), 38400,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));
		//asm(" RPT#8 || NOP");
		//---

		SendTaskPtr = 0;					// Init to 1st state	
		SerialCommsTimer = 0;
								                                      
		CommsOKflg = 0;
		RcvTaskPointer = &GetCmdByte;		// go back and wait for new CMD                       
	}
	else if (SerialCommsTimer > 500)		// 1000*1mS = 1 sec timeout
	{
		CommsOKflg = 0;
		RcvTaskPointer = &GetCmdByte;		// Abort, go back wait for new CMD
		SerialCommsTimer = 0;
	}
	
}

void EchoDataByte(void) // Task 6
{
    if(!(HWREG(UART0_BASE + UART_O_FR) & UART_FR_BUSY)) //(SciaRegs.SCICTL2.bit.TXRDY == 1)		// is TXBUF empty ?, i.e. TXRDY = 1
    {
    	HWREG(UART0_BASE  + UART_O_DR) = RxChar;//SciaRegs.SCITXBUF=RxChar;			// if yes, echo back the received char
		RcvTaskPointer = &PackWord;
		if(CmdPacket[0] == 7)
		  RcvTaskPointer = &PackLong;
		//RcvTskPtrShdw = 7;				// DEBUG
    }
}
void PackLong(void) // expects LSB first then MSB // Task 7
{
	if(ByteNum>0)
	{ 
		RxLong += RxChar<<((3-ByteNum)*8);
		ByteNum--;
		RcvTaskPointer = &GetDataByte;
		//RcvTskPtrShdw = 5;				// DEBUG*
		GetDataByte();
	}
	else
	{
		RxLong += RxChar;
		ByteNum=3;
		//CmdPacket[2] = RxWord;				// store data in packet
		RcvTaskPointer = &CmdInterpreter;
		//RcvTskPtrShdw = 8;				// DEBUG
		TaskDoneFlag = 0; 					// indicate new task underway	
	}
}
void PackWord(void) // expects LSB first then MSB // Task 7
{
	if(LowByteFlag == 1)
	{ 
		RxWord = RxChar;
		LowByteFlag = 0;
		RcvTaskPointer = &GetDataByte;
		//RcvTskPtrShdw = 5;				// DEBUG
		GetDataByte();
	}
	else
	{
		RxWord = RxWord | (RxChar<<8);
		LowByteFlag = 1;
		CmdPacket[2] = RxWord;				// store data in packet
		RcvTaskPointer = &CmdInterpreter;
		//RcvTskPtrShdw = 8;				// DEBUG
		TaskDoneFlag = 0; 					// indicate new task underway	
	}
}

void CmdInterpreter(void) // Task 8
{
	if (TaskDoneFlag == 0)
	{
	 	(*CmdDispatcher[ CmdPacket[0] ] )();  	// dispatch Task					
	}

	// Incase Task never finishes 
	if (SerialCommsTimer > 2500)			// 2500*1mS = 2.5 sec timeout
	{
		CommsOKflg = 0;
		RcvTaskPointer = &GetCmdByte;		// Abort, go back wait for new CMD
		SerialCommsTimer = 0;
	}
	if (TaskDoneFlag == 1)
	{
		RcvTaskPointer = &GetCmdByte;
	 	//RcvTskPtrShdw = 1;   				// DEBUG
	}
}

//===========================================================
// Slave Tasks commanded by Host
//===========================================================
void LifePulseTsk(void) 	// CmdPacket[0] = 0
{
	
	if (CmdPacket[2]==0x0000 && CmdPacket[1]==0x00)  //LED2-ON
	{
		/*
		#if DSP2802x_DEVICE   
		GpioDataRegs.GPASET.bit.GPIO12=1;		
		#else
		GpioDataRegs.GPASET.bit.GPIO31=1;
		#endif
		*/
	}
	if (CmdPacket[2]==0x0001 && CmdPacket[1]==0x00)   //LED2-OFF
	{/*
		#if DSP2802x_DEVICE
		GpioDataRegs.GPACLEAR.bit.GPIO12=1;
		#else
		GpioDataRegs.GPACLEAR.bit.GPIO31=1;
		#endif
		*/		
	}
	if (CmdPacket[2]==0x0002 && CmdPacket[1]==0x00)  //LED2-Toggle
	{/*
		#if DSP2802x_DEVICE
		GpioDataRegs.GPATOGGLE.bit.GPIO12=1;
		#else
		GpioDataRegs.GPATOGGLE.bit.GPIO31=1;
		#endif
		*/
	}

	CommsOKflg = 1;
	SerialCommsTimer = 0;
	TaskDoneFlag = 1;
}
//-----------------------------------------------------------
void TextSet(void) // CmdPacket[0] = 1
{	
	
	*varSetTxtList[CmdPacket[1]] = CmdPacket[2];

	TaskDoneFlag = 1; // indicate Task execution is complete
}
//-----------------------------------------------------------
void ButtonSet(void) 	// CmdPacket[0] = 2
{
	*varSetBtnList[CmdPacket[1]] = CmdPacket[2];

	TaskDoneFlag = 1; // indicate Task execution is complete
}
//-----------------------------------------------------------
void SliderSet(void) 	// CmdPacket[0] = 3
{
	*varSetSldrList[CmdPacket[1]] = CmdPacket[2];

	TaskDoneFlag = 1; // indicate Task execution is complete
}
//-----------------------------------------------------------
void VariableGet(void)		// CmdPacket[0] = 4
{
	SendData();
}
//-----------------------------------------------------------
//Send a Uint16 array one element at a time
void ArrayGet(void)			// CmdPacket[0] = 5
{
	SendData();	
}
//-----------------------------------------------------------
void DataGet(void)			// CmdPacket[0] = 6
{
	switch(MemGetPtr)
	{
		case 0:
			MemGetAddress = CmdPacket[2];
			MemGetPtr = 1;

			wordsLeftToGet = 1;
			SendTaskPtr = 1;
			TaskDoneFlag = 1;
			break;

		case 1:
			Temp = CmdPacket[2];
			MemGetAddress = MemGetAddress + (Temp<<16);
			memDataPtr = (int*)MemGetAddress;
			dataOut = *memDataPtr;		
			SendData();		

			if(TaskDoneFlag == 1)
			{
				MemGetPtr = 0;
			}
			break;
		}

	//TaskDoneFlag = 1; 	// indicate Task execution is complete
}
//-----------------------------------------------------------
void TextLongSet(void)		// CmdPacket[0] = 7
{
	*varSetTxtLongList[CmdPacket[1]]=RxLong;
	TaskDoneFlag = 1; 		// indicate Task execution is complete
}
//-----------------------------------------------------------
void TextLongGet(void)		// CmdPacket[0] = 8
{
	TaskDoneFlag = 1; 		// indicate Task execution is complete
}
//-----------------------------------------------------------

void SendData(void)
{
	/*switch(SendTaskPtr)
	{
	case 0:  //initialization 
		if(CmdPacket[0] == 0x04)
		{
			memDataPtr = (int *) varGetList[CmdPacket[1]];
		}
		else
		{
			memDataPtr = (int *) arrayGetList[CmdPacket[1]];
		}
		
		dataOut = *memDataPtr;
		wordsLeftToGet = CmdPacket[2];
		//Note that case 0 rolls into case 1 (no break)

	case 1:  //send LSB
		if(wordsLeftToGet > 0)
		{
			if (!(HWREG(UART0_BASE + UART_O_FR) & UART_FR_BUSY)) //(SciaRegs.SCICTL2.bit.TXRDY == 1)
			{
				HWREG(UART0_BASE  + UART_O_DR) = dataOut & 0x000000FF;//SciaRegs.SCITXBUF = dataOut & 0x000000FF;
				SendTaskPtr = 2;
			}
			else
			{
				TaskDoneFlag = TaskDoneFlag;
				break;
			}
		}
		else
		{
			SendTaskPtr = 0;
			TaskDoneFlag = 1;
			break;
		}		

	case 2: //send MSB
		if(!(HWREG(UART0_BASE + UART_O_FR) & UART_FR_BUSY)) //(SciaRegs.SCICTL2.bit.TXRDY == 1)
		{
			HWREG(UART0_BASE  + UART_O_DR) = (dataOut>>8 & 0x000000FF);//SciaRegs.SCITXBUF = (dataOut>>8 & 0x000000FF);

			memDataPtr = memDataPtr + 1;
			dataOut = *memDataPtr;
			wordsLeftToGet = wordsLeftToGet - 1;
			SendTaskPtr = 1;
		}
		break;
	}*/
	
	switch(SendTaskPtr)
	{
	case 0:  //initialization 
		if(CmdPacket[0] == 0x04)
		{
			memDataPtr_1 =  (short int *)varGetList[CmdPacket[1]];
		}
		else
		{
			memDataPtr_1 =  arrayGetList[CmdPacket[1]];
		}
		
		dataOut_1 = *memDataPtr_1;
		wordsLeftToGet = CmdPacket[2];
		//Note that case 0 rolls into case 1 (no break)

	case 1:  //send LSB
		if(wordsLeftToGet > 0)
		{
			if (!(HWREG(UART0_BASE + UART_O_FR) & UART_FR_BUSY)) //(SciaRegs.SCICTL2.bit.TXRDY == 1)
			{
				HWREG(UART0_BASE  + UART_O_DR) = dataOut_1 & 0x000000FF;//SciaRegs.SCITXBUF = dataOut & 0x000000FF;
				SendTaskPtr = 2;
			}
			else
			{
				TaskDoneFlag = TaskDoneFlag;
				break;
			}
		}
		else
		{
			SendTaskPtr = 0;
			TaskDoneFlag = 1;
			break;
		}		

	case 2: //send MSB
		if(!(HWREG(UART0_BASE + UART_O_FR) & UART_FR_BUSY)) //(SciaRegs.SCICTL2.bit.TXRDY == 1)
		{
			HWREG(UART0_BASE  + UART_O_DR) = (dataOut_1>>8 & 0x000000FF);//SciaRegs.SCITXBUF = (dataOut>>8 & 0x000000FF);

			memDataPtr_1 = memDataPtr_1 + 1;
			dataOut_1 = *memDataPtr_1;
			wordsLeftToGet = wordsLeftToGet - 1;
			SendTaskPtr = 1;
		}
		break;
	}
	
}




















