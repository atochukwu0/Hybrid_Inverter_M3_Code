//#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
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

#include "utils/uartstdio.h"

#include "Solar_DC_AC_IPC.h"

#define	PktSizeLin				2

// Prototype statements for functions found within this file.
void PackTxBuffersLin(void);
void UnpackRxBuffersLin(void);
void SetupLin(void);
void error(void);

// Global variables
unsigned int sdataALin;     // Send data for SCI-A
unsigned int rdataALin;     // Received data for SCI-A

unsigned int iLin;			  //Generic iterator
/*
extern int Gui_wPanelVolt;
extern int Gui_wPanelCurrent;
extern int Gui_wBoostOutputVolt;
extern int Gui_wPanelOutputPower;
extern int Gui_wMPPT_Status;
extern int Gui_wSysMode;
extern int Gui_wLLCOutputVolt;
extern int Gui_wTurnOn;
extern int Gui_wTurnOff;
*/

void (*RcvTaskPointerLin)(void); 			// State pointer for Command Packet Receive
unsigned int	CmdPacketLin[PktSizeLin];


void GetCmdEchoByteLin(void);
void SendIndexByteLin(void);
void GetIndexEchoByteLin(void);
void GetDataByteLin(void);
void EchoDataByteLin(void);
void SendDataByteLin(void);
void GetCompleteByteLin(void);
void ReadyForDataLin(void);


void Inquiry_Panel_Voltage(void);
void Inquiry_Panel_Current(void);
void Inquiry_Boost_Voltage(void);
void Inquiry_Panel_Output_Power(void);
void Inquiry_MPPT_Status(void);
void Send_Running_Mode(void);
void Send_LLC_Output_Voltage(void);
void Send_Turn_On_Command(void);
void Send_Turn_Off_Command(void);
void Send_Turn_On_MPPT_Command(void);
void Send_Turn_Off_MPPT_Command(void);

int wTestcnt;

//extern unsigned int wSendTurnOnCmd;
//extern unsigned int wSendTurnOffCmd;
extern unsigned int wNoSciRevIntCnt;

extern int Gui_DC_Board_Connect;
int wErrorCnt;

//extern FSuperFlag;
extern union FSysFlags{
	
	unsigned int	word;
	
	struct {
		unsigned int	FwTurnOn:1;
		unsigned int	FwTurnOff:1;
		
		unsigned int  FwWarning:1;
		unsigned int  FwFault:1;
		
		unsigned int  FwClearFaultFlag:1;
		unsigned int  FwOpRlyStatus:1;
		unsigned int  FwPwmStatus:1;
		
		unsigned int  FwAutoStartOnEn:1;
		unsigned int  FwTurnOnConditionOk:1;
		unsigned int  FwAutoStart:1;
		
		unsigned int   resv:6;
		}BIT;
}FSuperFlag;
//extern  union FSysFlags FSuperFlag;


//LIN Communication
//======== SM Entry Point =================
void SerialHostCommsLin()
{		
	(*RcvTaskPointerLin)();	// Call routine pointed to by state pointer
}
//=========================================


void GetCmdEchoByteLin(void)
{
	UnpackRxBuffersLin();
	if (CmdPacketLin[0] == rdataALin)
	{
		if(CmdPacketLin[0] <= 0x2)
		{
			RcvTaskPointerLin = &SendIndexByteLin;		// point to next state
			SendIndexByteLin();
		}
		else if(CmdPacketLin[0] > 0x2)
		{
			RcvTaskPointerLin = &GetCmdEchoByteLin;		// point to next state
			
			if(CmdPacketLin[0] == 0x3)	//Slave echo the turn on cmd
			{
				//wSendTurnOnCmd = 0;
				MtoC_Message1.C28_wSendTurnOnCmd_Clear = 1;
			}
			else if(CmdPacketLin[0] == 0x4)
			{
				//wSendTurnOffCmd = 0;
				MtoC_Message1.C28_wSendTurnOffCmd_Clear = 1;
				//FSuperFlag.BIT.FwTurnOff = 1;
			}
			
			// clear Command Packet
			for (iLin=0; iLin<PktSizeLin; iLin++)
			{
				CmdPacketLin[iLin] = 0x0;
			}
			sdataALin = 0x0;
   			rdataALin = 0x0;
		}
		
		//MtoC_Message1.M3_Gui_DC_Board_Connect = 1;
		wErrorCnt = 0;
	}
	else
	{
		if(((wErrorCnt++) >= 4))
		{
			//MtoC_Message1.M3_Gui_DC_Board_Connect = 0;
		}
		error();		
	}
		
}


void SendIndexByteLin(void)
{
	sdataALin = CmdPacketLin[1];
	PackTxBuffersLin();
	RcvTaskPointerLin = &GetIndexEchoByteLin;		// point to next state
}


void GetIndexEchoByteLin(void)
{
	UnpackRxBuffersLin();
	if(CmdPacketLin[1] == rdataALin)
	{
		if(CmdPacketLin[0] == 0x1)
		{
			//RcvTaskPointer = &GetDataByte;		// point to next state
			RcvTaskPointerLin = &ReadyForDataLin;
			ReadyForDataLin();			
		}
		else if(CmdPacketLin[0] == 0x2)
		{
			RcvTaskPointerLin = &SendDataByteLin;		// point to next state
			SendDataByteLin();
		}
	}
	
	else
		error();
}

void ReadyForDataLin(void)
{
	sdataALin = CmdPacketLin[1];
	PackTxBuffersLin();
	RcvTaskPointerLin = &GetDataByteLin;		// point to next state
}


void GetDataByteLin(void)
{
	UnpackRxBuffersLin();
	switch(CmdPacketLin[1])
	{
		case 0: MtoC_Message1.DCDC_Gui_wPanelVolt = rdataALin;
				break;
		case 1: MtoC_Message1.DCDC_Gui_wPanelCurrent = rdataALin;
				break;
		case 2: MtoC_Message1.DCDC_Gui_wBoostOutputVolt = rdataALin;
				break;
		case 3: MtoC_Message1.DCDC_Gui_wPanelOutputPower = rdataALin;
				break;
		case 4: MtoC_Message1.DCDC_Gui_wMPPT_Status = rdataALin;
				break;
		default:
				break;
	}
	
	RcvTaskPointerLin = &EchoDataByteLin;		// point to next state
	EchoDataByteLin();
}


void EchoDataByteLin(void)
{
	sdataALin = 0x00ff;
	PackTxBuffersLin();
	// clear Command Packet
	for (iLin=0; iLin<PktSizeLin; iLin++)
	{
		CmdPacketLin[iLin] = 0x0;
	}
	
	rdataALin = 0x0;
	sdataALin = 0x0;
	
	RcvTaskPointerLin = &GetCmdEchoByteLin;		// point to next state
}


void SendDataByteLin(void)
{
	switch(CmdPacketLin[1])
	{
		case 0: sdataALin = CtoM_Message1.C28_Gui_wSysMode;
				break;
		case 1: sdataALin = CtoM_Message1.C28_Gui_wLLCOutputVolt;
				break;
		default: 
				break;
	}
	PackTxBuffersLin();
	RcvTaskPointerLin = &GetCompleteByteLin;		// point to next state
}


void GetCompleteByteLin(void)
{
	UnpackRxBuffersLin();
	if(rdataALin == 0x00ff)
	{
		for (iLin=0; iLin<PktSizeLin; iLin++)
		{
			CmdPacketLin[iLin] = 0x0;
		}
		
		rdataALin = 0x0;
		sdataALin = 0x0;
	
		RcvTaskPointerLin = &GetCmdEchoByteLin;		// point to next state
	}
	
	else
		error();
}

//High priority LIN ISR.
void 
LinIntHandler(void)
{	
    unsigned long ulStatus;

    // Get the interrrupt status.
    ulStatus = UARTIntStatus(UART1_BASE, 1);

    // Clear the asserted interrupts.
    UARTIntClear(UART1_BASE, ulStatus);

    // Loop while there are characters in the receive FIFO.
    /*
    while(UARTCharsAvail(UART1_BASE))
    {
        // Read the next character from the UART and write it back to the UART.
        UARTCharPutNonBlocking(UART1_BASE,
                               UARTCharGetNonBlocking(UART1_BASE));
    }*/
    
	wTestcnt++;
	wNoSciRevIntCnt = 0;

	//RXINT
	SerialHostCommsLin();
}



//Pack array words into LINTD buffer bytes
//sending high byte first.
void PackTxBuffersLin(void)
{
	//LinaRegs.LINTD0.bit.TD0 = sdataALin >> 8;
	//LinaRegs.LINTD0.bit.TD1 = sdataALin &  0x00FF;
	//if(TX_BUFFER_EMPTY)
    {
    	HWREG(UART1_BASE  + UART_O_DR) = sdataALin >> 8;
    	HWREG(UART1_BASE  + UART_O_DR) = sdataALin &  0x00FF;
    }
}

//Move data from LINRD buffers to data array.
void UnpackRxBuffersLin(void)
{
	unsigned int ReadData;
	ReadData = HWREG(UART1_BASE  + UART_O_DR);//LinaRegs.LINRD0.all;
	rdataALin = ReadData & 0xFF;
	ReadData = HWREG(UART1_BASE  + UART_O_DR);//LinaRegs.LINRD0.all;
	rdataALin = (rdataALin<<8)|(ReadData & 0xFF);
	/*
	rdataA[1] = ReadData & 0x0000FFFF;
	ReadData = LinaRegs.LINRD1.all;
	rdataA[2] = (ReadData & 0xFFFF0000) >> 16;
	rdataA[3] = ReadData & 0x0000FFFF;
	*/
}

//Verify correct data transmission and
//increment test data.

void SetupLin(void)
{
   	//Initialize and Enable BLIN SCI module
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED), 9600,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));
                         
    // Set the FIFO interrupt levels.
    UARTFIFOEnable(UART1_BASE);
	UARTFIFOLevelSet(UART1_BASE, UART_FIFO_TX1_8,UART_FIFO_RX1_8);
                         
    //SCIA_Init();

    // Enable the UART interrupt.
    IntRegister(INT_UART1, LinIntHandler);
    IntEnable(INT_UART1);
    UARTIntEnable(UART1_BASE, UART_INT_RX);
}



void Inquiry_Panel_Voltage(void)
{
	
	CmdPacketLin[0] = 1;
	CmdPacketLin[1] = 0;
	sdataALin = CmdPacketLin[0];
	PackTxBuffersLin();
}

void Inquiry_Panel_Current(void)
{
	CmdPacketLin[0] = 1;
	CmdPacketLin[1] = 1;
	sdataALin = CmdPacketLin[0];
	PackTxBuffersLin();
}

void Inquiry_Boost_Voltage(void)
{
	CmdPacketLin[0] = 1;
	CmdPacketLin[1] = 2;
	sdataALin = CmdPacketLin[0];
	PackTxBuffersLin();
}

void Inquiry_Panel_Output_Power(void)
{
	CmdPacketLin[0] = 1;
	CmdPacketLin[1] = 3;
	sdataALin = CmdPacketLin[0];
	PackTxBuffersLin();
}

void Inquiry_MPPT_Status(void)
{
	CmdPacketLin[0] = 1;
	CmdPacketLin[1] = 4;
	sdataALin = CmdPacketLin[0];
	PackTxBuffersLin();
}

void Send_Running_Mode(void)
{
	CmdPacketLin[0] = 2;
	CmdPacketLin[1] = 0;
	sdataALin = CmdPacketLin[0];
	PackTxBuffersLin();
}

void Send_LLC_Output_Voltage(void)
{
	CmdPacketLin[0] = 2;
	CmdPacketLin[1] = 1;
	sdataALin = CmdPacketLin[0];
	PackTxBuffersLin();
}

void Send_Turn_On_Command(void)
{
	CmdPacketLin[0] = 3;
	sdataALin = CmdPacketLin[0];
	PackTxBuffersLin();
}

void Send_Turn_Off_Command(void)
{
	CmdPacketLin[0] = 4;
	sdataALin = CmdPacketLin[0];
	PackTxBuffersLin();
}

void Send_Turn_On_MPPT_Command(void)
{
	CmdPacketLin[0] = 5;
	sdataALin = CmdPacketLin[0];
	PackTxBuffersLin();
}

void Send_Turn_Off_MPPT_Command(void)
{
	CmdPacketLin[0] = 6;
	sdataALin = CmdPacketLin[0];
	PackTxBuffersLin();
}



void error(void)
{
	//asm("     ESTOP0"); // Test failed!! Stop!
	//for (;;);
	RcvTaskPointerLin = &GetCmdEchoByteLin;	 // Initialize the CmdPacket Rcv Handler state machine
}
