//###########################################################################
// FILE:   uart_echo.c
// TITLE:  Example for reading data from and writing data to the UART in
//         an interrupt driven fashion.
//###########################################################################
// $TI Release: F28M35x Driver Library v100 $
// $Release Date: October 12, 2011 $
//###########################################################################
//Major change to data acquisition structure
//Used structures to share data between processors, Used start flag to sync pointers.


#define ACKed   0xBBBB
#define NotACKed 0x6666

#define ChargingCurrent_MAX                 -5
#define ChargingCurrent_MIN                 -20
#define gridFeedCurrent_MAX                 20
#define gridFeedCurrent_MIN                 1
#define UPSVoltage_MAX                      400
#define UPSVoltage_MIN                      200
#define OP_I_Limit_MAX                      50
#define OP_I_Limit_MIN                      1
#define LINE_V_Over_Hard_Limit_MAX          450
#define LINE_V_Over_Hard_Limit_MIN          100
#define LINE_V_Over_Moderate_Limit_MAX      440
#define LINE_V_Over_Moderate_Limit_MIN      100
#define LINE_V_Over_Soft_Limit_MAX          400
#define LINE_V_Over_Soft_Limit_MIN          100
#define LINE_V_Under_Limit_MAX              400
#define LINE_V_Under_Limit_MIN              100
#define DC_V_Under_UPS_Shut_Limit_MAX       400
#define DC_V_Under_UPS_Shut_Limit_MIN       100
#define DC_V_Under_Limit_MAX                400
#define DC_V_Under_Limit_MIN                100
#define DC_V_Grid_tie_start_Limit_MAX       400
#define DC_V_Grid_tie_start_Limit_MIN       100
#define DC_V_Grid_tie_stop_Limit_MAX        400
#define DC_V_Grid_tie_stop_Limit_MIN        100
#define DC_V_Chrg_Under_Limit_MAX           400
#define DC_V_Chrg_Under_Limit_MIN           100
#define DC_V_Chrg_Over_Limit_MAX            450
#define DC_V_Chrg_Over_Limit_MIN            100
#define DC_V_Chrg_Over_Limit_Offset_MAX     400
#define DC_V_Chrg_Over_Limit_Offset_MIN     100
#define max_chrg_current_MAX                -5
#define max_chrg_current_MIN                -20
#define min_chrg_current_MAX                -5
#define min_chrg_current_MIN                -20
#define LINE_V_Threshold_MAX                400
#define LINE_V_Threshold_MIN                200

#define OrangePiDataFramMaxSize 65

#define M3_MASTER 0
#define C28_MASTER 1

#define EEPROM_ADDRESS 0x50

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_i2c.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "inc/hw_sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/i2c.h"
#include "driverlib/gpio.h"
#include "driverlib/flash.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "inc/hw_ipc.h"
#include "inc/hw_ram.h"
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"
#include "driverlib/ipc.h"
#include "driverlib/ram.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "utils/ustdlib.h"
#include "utils/uartstdio.h"
#include "utils/memcopy.h"
#include "Solar_DC_AC_IPC.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "delay.h"
#include "eeprom.h"
#include "memoryAddress.h"

// Variables that need to shared with C28x and M3
// m3 owned memory region
struct MtoC_Message MtoC_Message1;
#pragma DATA_SECTION(MtoC_Message1,"MtoC_MsgRAM");
struct CtoM_Message CtoM_Message1;
#pragma DATA_SECTION(CtoM_Message1,"CtoM_MsgRAM");

enum states {
    BOOT,
    OFF,
    STANDBY,
    UPS,
    GRID_POWER_NO_CHARGING,
    GRID_POWER_CHARGING,
    GRID_TIED_SOLAR,
    GRID_TIED_PEAK,
    ERROR,
    GRID_ABNORMAL,
    DC_BUS_PRECHARGING
};

enum off_conditions{
    UNKNOWN,
    DC_VOLTAGE_UNDER_UPS_SHUT_LIMIT,
    DC_VOLTAGE_UNDER_LIMIT,
    OTHER_FAULT_SHUTDOWN,
    USER_INITIATED,
};

typedef struct {
    unsigned char startByte1;
    unsigned char startByte2;
    float BatteryVoltage;
    float BatteryCurrent;
    float GridVoltage;
    float GridCurrent;
    float HomeVoltage;
    float HomeCurrent;
    float PowerFactor;
    unsigned short CurrentState;
    unsigned short ReasonState;
    unsigned short ACK;
    unsigned char crc;
} SendingMsg_t;

typedef struct {
    float ChargingCurrent;
    float gridFeedCurrent;
    float UPSVoltage;
} ReceiveMsg1_t;

typedef struct{
    float OP_I_Limit ;
    float LINE_V_Over_Hard_Limit ;
    float LINE_V_Over_Moderate_Limit ;
    float LINE_V_Over_Soft_Limit ;
    float LINE_V_Under_Limit ;

    float DC_V_Under_UPS_Shut_Limit;
    float DC_V_Under_Limit ;
    float DC_V_Grid_tie_start_Limit;
    float DC_V_Grid_tie_stop_Limit ;
    float DC_V_Chrg_Under_Limit ;
    float DC_V_Chrg_Over_Limit;
    float DC_V_Chrg_Over_Limit_Offset;

    float max_chrg_current ;
    float min_chrg_current;
    float LINE_V_Threshold ;
} ReceiveMsg2_t;

typedef struct{
    float Currentsense_Gain;
    float OP_Voltagesense_Gain;
    float GRID_Voltagesense_Gain;
    float DC_VOLTAGESENSE_GAIN;
    float DC_VOLTAGESENSE_OFFSET ;
    float GRID_VOLTAGE_OFFSET ;
    float OP_VOLTAGE_OFFSET;
    float OP_CURRENT_OFFSET;
    float IP_CURRENT_OFFSET;
} ReceiveMsg3_t;

typedef struct {
    unsigned char Peak;
    unsigned char CanstValue;

} ReceiveMsg4_t;

struct Message {
    float f1;
    float f2;
    float f3;
    float f4;
    float f5;
    float f6;

    unsigned short i1;
    unsigned short i2;
    unsigned short count;

    unsigned char crc1;
    unsigned char crc2;
};

//Should match struct in C28 code including variable order, enums have different lengths
struct CtoMData {                                           /**/
    unsigned long long Pw;                                  /**/
    unsigned short start_flag;                              /**/
    float BatVoltage;                                       /**/
    float BatCurrent;                                       /**/
    float GridVoltage;                                      /**/
    float GridCurrent;                                      /**/
    float InverterVoltage;                                  /**/
    float InverterCurrent;                                  /**/
    float PowerFactor;                                      /**/
    short SystemState_16bit;                                /**/
    short off_condition_16bit;                              /**/
    unsigned short ReasonState;                             /**/
};
//Should match struct in C28 code including variable order
struct MtoCData {                                           /**/
    unsigned long long Pr;                                  /**/
    unsigned short solar_available;                         /**/
    unsigned short is_peak_time;                            /**/
    unsigned short peak_enabled;                            /**/
    unsigned short op_power;                                /**/
    ReceiveMsg1_t basic_configuration;                      /**/
    enum off_conditions last_off_condition;                 /**/
};

unsigned char crc=0x00;
unsigned char Addr = 0;

volatile struct Message M2;

struct Message  base[256];
#pragma DATA_SECTION(base,"SHARERAMS2");

volatile struct CtoMData CtoMvar;
#pragma DATA_SECTION(CtoMvar,"SHARERAMS3");

volatile struct MtoCData MtoCvar;
#pragma DATA_SECTION(MtoCvar,"SHARERAMS4");

const unsigned char SendMsgSize = sizeof(SendingMsg_t);
const unsigned char Msg1Size = sizeof(ReceiveMsg1_t);
const unsigned char Msg2Size = sizeof(ReceiveMsg2_t);
const unsigned char Msg3Size = sizeof(ReceiveMsg3_t);
const unsigned char Msg4Size = sizeof(ReceiveMsg4_t);
unsigned char RecieveData[OrangePiDataFramMaxSize] = {};
unsigned char RecievedDataCount, MsgSize;
enum states SystemState,LastSystemState;
enum off_conditions off_condition;

SendingMsg_t SendingMsg;
ReceiveMsg1_t tempReceiveMsg1;
ReceiveMsg1_t ReceiveMsg1;
ReceiveMsg2_t tempReceiveMsg2;
ReceiveMsg2_t ReceiveMsg2;
ReceiveMsg3_t tempReceiveMsg3;
ReceiveMsg3_t ReceiveMsg3;
ReceiveMsg4_t ReceiveMsg4;


static volatile unsigned long g_ulFlags;
//#ifdef FLASH
#define DLOG_SIZE 400	// Uncomment for FLASH configuration only
//#else
//#define DLOG_SIZE 150
//#endif

// These are defined by the linker (see device linker command file)
extern unsigned long RamfuncsLoadStart;
extern unsigned long RamfuncsLoadEnd;
extern unsigned long RamfuncsRunStart;

//void master_ram_init_control_m0m1_msgram_memories(void);
//void master_ram_init_control_L0_L4_memories(void);

extern void SCIA_Init();
extern void SerialHostComms();
int ftoa(float value, char *buf, char decimalPoints);

//int	SerialCommsTimer;
//int	CommsOKflg;

//Variable used for data transfer
volatile int base_read_index =0;
volatile char *character_pointer;
volatile int index=0;
volatile char serial_print_char;

volatile long count=0,count2=0,data=0xFFFF,data2=0xFFFF;
volatile int index_drawn=0,edit_mode=0,edit_row_index=0,editvar=0;
volatile int Enter_pressed=0,Back_pressed=0,INCRE_pressed=0,DECRE_pressed=0;


//*****************************************************************************
// The error routine that is called if the driver library encounters an error.
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}

#endif

void SendUARTInverterStructure();
unsigned char CRC8bit(unsigned char*data, unsigned char length);
void validateReceiveMsg1();
void validateReceiveMsg2();
void validateReceiveMsg3();
void updateRecievingData();

volatile int LED = 0;
//*****************************************************************************
// The interrupt handler for the first timer interrupt.
//*****************************************************************************
void Timer0IntHandler(void)
{
    // Clear the timer interrupt.
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //**********Data transfer
    if(CtoMvar.start_flag==1)
        MtoCvar.Pr=0;
    while(CtoMvar.Pw > MtoCvar.Pr){
        //      while(UARTCharPutNonBlocking(UART0_BASE,serial_print_char)){
        Addr = MtoCvar.Pr % 256;//0xFF;//& 0xFF;
        M2 = base[Addr];
        UARTCharPut(UART0_BASE,0xAA);   //Send Beginning
        UARTCharPut(UART0_BASE,0x55);
        while(index<30){
            serial_print_char = *(char*)(((char*)(&M2))+ index);
            UARTCharPut(UART0_BASE,serial_print_char);
            index++;
        }
        crc=M2.crc1 ^ M2.crc2;
        UARTCharPut(UART0_BASE,crc);
        index = 0;
        MtoCvar.Pr++;

    }

    // Toggle the flag for the first timer.
    HWREGBITW(&g_ulFlags, 0) ^= 1;

}

void Timer1IntHandler(void){
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);     // Clear the timer interrupt.
    HWREGBITW(&g_ulFlags, 1) ^= 1;                      // Toggle the flag for the second timer.
}

void UARTIntHandler(void){
    unsigned long ulStatus;
    ulStatus = UARTIntStatus(UART0_BASE, true);     // Get the interrupt status.
    UARTIntClear(UART0_BASE, ulStatus);             // Clear the asserted interrupts.
}

void UART1IntHandler(void){
    unsigned long ulStatus;
    ulStatus = UARTIntStatus(UART1_BASE, true);     // Get the interrupt status.
    UARTIntClear(UART1_BASE, ulStatus);             // Clear the asserted interrupts.

    while(UARTCharsAvail(UART1_BASE)){              // Loop while there are characters in the receive FIFO.
        // Read the next character from the UART and write it back to the UART.
        // UARTCharPutNonBlocking(UART1_BASE,
        RecieveData[RecievedDataCount] = UARTCharGetNonBlocking(UART1_BASE);
        RecievedDataCount++;
    }

    if(RecievedDataCount > OrangePiDataFramMaxSize)
        RecievedDataCount = 0;

    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER1_BASE, TIMER_A);
}

void Timer3IntHandler(void){
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);     // Clear the timer interrupt.
    HWREGBITW(&g_ulFlags, 1) ^= 1;      // Toggle the flag for the second timer.
    RecievedDataCount = 0;
    TimerDisable(TIMER1_BASE, TIMER_A);
    updateRecievingData();
}

////*****************************************************************************
//// Send a string to the UART.
////*****************************************************************************
//void
//UARTSend(const unsigned char *pucBuffer, unsigned long ulCount)
//{
//    // Loop while there are more characters to send.
//    while(ulCount--)
//    {
//        // Write the next character to the UART.
//        UARTCharPutNonBlocking(UART0_BASE, *pucBuffer++);
//    }
//}

void UART1Send(const unsigned char *pucBuffer, unsigned long ulCount) {
    while(ulCount--){        // Loop while there are more characters to send.
        UARTCharPut(UART1_BASE, *pucBuffer++);
    }
}


int main(void) {

    // Disable Protection
    HWREG(SYSCTL_MWRALLOW) =  0xA5A5A5A5;

    // Tells M3 Core the vector table is at the beginning of C0 now.
    HWREG(NVIC_VTABLE) = 0x20005000;

    // Setup main clock tree for 75MHz - M3 and 150MHz - C28x
    SysCtlClockConfigSet(SYSCTL_SYSDIV_1 | SYSCTL_M3SSDIV_2 | SYSCTL_USE_PLL |
                         (SYSCTL_SPLLIMULT_M & 0x0F));

    MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
    // Call Flash Initialization to setup flash wait states
    // This function must reside in RAM
    FlashInit();

    // Enable all GPIOs
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);

    // Enable the peripherals used
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    //Select core for controlling GPIO
    GPIOPinConfigureCoreSelect(GPIO_PORTA_BASE, 0xFF, GPIO_PIN_C_CORE_SELECT);
    GPIOPinConfigureCoreSelect(GPIO_PORTB_BASE, 0x0F, GPIO_PIN_C_CORE_SELECT);  // Two pins used for I2C
    GPIOPinConfigureCoreSelect(GPIO_PORTC_BASE, 0x7F, GPIO_PIN_C_CORE_SELECT);  // 1 pin used by M3 for blink LED
    GPIOPinConfigureCoreSelect(GPIO_PORTD_BASE, 0xFF, GPIO_PIN_C_CORE_SELECT);
    GPIOPinConfigureCoreSelect(GPIO_PORTE_BASE, 0xCF, GPIO_PIN_C_CORE_SELECT);  // Two pins used by M3 for UART
    GPIOPinConfigureCoreSelect(GPIO_PORTF_BASE, 0xDF, GPIO_PIN_C_CORE_SELECT);  // 1 switch for menu usage
    GPIOPinConfigureCoreSelect(GPIO_PORTG_BASE, 0xFF, GPIO_PIN_C_CORE_SELECT);
    GPIOPinConfigureCoreSelect(GPIO_PORTH_BASE, 0xFF, GPIO_PIN_C_CORE_SELECT);
    GPIOPinConfigureCoreSelect(GPIO_PORTJ_BASE, 0x8F, GPIO_PIN_C_CORE_SELECT);  // 3 switches for menu usage


    GPIOPadConfigSet(GPIO_PORTA_BASE, 0xFF, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTB_BASE, 0x0F, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTC_BASE, 0x7F, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTD_BASE, 0xFF, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTE_BASE, 0xCF, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTF_BASE, 0xDF, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTG_BASE, 0xFF, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTH_BASE, 0xFF, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, 0x8F, GPIO_PIN_TYPE_STD_WPU);

    //  GPIODirModeSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_DIR_MODE_IN);  //Set the PA6 as input


    //  Set GPIO E4 and E5 as UART0 pins.
    GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    GPIOPinConfigure(GPIO_PE4_U0RX);
    GPIOPinConfigure(GPIO_PE5_U0TX);
    //  Set GPIO B4 and B5 as UART1 pins.
    GPIOPinConfigure(GPIO_PB4_U1RX);
    GPIOPinConfigure(GPIO_PB5_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    //  Set up pins for I2C
    //  Unlock GPIO This has to be done because this pin is a NMI
    GPIOPinUnlock(GPIO_PORTB_BASE, GPIO_PIN_7);//Function was not available in V100, had to copy from v220
    GPIOPinConfigure(GPIO_PB6_I2C0SDA);
    GPIOPinConfigure(GPIO_PB7_I2C0SCL);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7);   //GPIO14 | GPIO15

    //  Set up the Pin for LED
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7);
    //  Set up the pins for SW
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE,GPIO_PIN_5);   //GPIO37
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_4);   //GPIO60
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_5);   //GPIO61
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_6);   //GPIO62

    SysCtlReleaseSubSystemFromReset(SYSCTL_CONTROL_SYSTEM_RES_CNF);

    // Details of how c28 uses these memory sections is defined
    // in the c28 linker file.(28M35H52C1_RAM_lnk.cmd)
    RamMReqSharedMemAccess((S0_ACCESS | S1_ACCESS |S2_ACCESS | S3_ACCESS ),C28_MASTER);

    IPCMtoCBootControlSystem(CBROM_MTOC_BOOTMODE_BOOT_FROM_FLASH);

    // Enable processor interrupts.
    IntMasterEnable();

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.  For this example we will use a data rate of 100kbps.
//    I2CMasterEnable(I2C0_MASTER_BASE);
    I2CMasterInitExpClk(I2C0_MASTER_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED), false);

//    int status = EEPROMByteWrite(I2C0_MASTER_BASE, 0x50, 0x0000, 205);
    data = EEPROMByteRead(I2C0_MASTER_BASE, EEPROM_ADDRESS , LAST_OFF_CONDITION_ADDR);
    if(data<0 || data>4){ //4 is the no of elements in off_conditions enum
        MtoCvar.last_off_condition = UNKNOWN;
    }
    else{
        MtoCvar.last_off_condition = (enum off_conditions)data;
    }

    // Configure the UART for 115,200, 8-N-1 operation.
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED), 8000000,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                UART_CONFIG_PAR_NONE));
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED), 9600,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                UART_CONFIG_PAR_NONE));

    // Enable the UART interrupt.
    IntRegister(INT_UART0, UARTIntHandler);
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX | UART_INT_CTS);
    UARTFIFOLevelSet(UART0_BASE,UART_FIFO_TX1_8 ,UART_FIFO_RX1_8);

    IntRegister(INT_UART1, UART1IntHandler);
    IntEnable(INT_UART1);
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);

    // Configure the two 32-bit periodic timers.
    TimerConfigure(TIMER0_BASE, TIMER_CFG_32_BIT_PER);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_32_BIT_PER);
    TimerLoadSet(TIMER0_BASE, TIMER_A, (SysCtlClockGet(SYSTEM_CLOCK_SPEED)/10000)); //10kHz timer interrupt , used for data transfer
    TimerLoadSet(TIMER1_BASE, TIMER_A, (SysCtlClockGet(SYSTEM_CLOCK_SPEED)/10));    // 2Hz timer interrupt , used for button de-bounce

    // Setup the interrupts for the timer timeouts.
    IntEnable(INT_TIMER0A);
    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    IntRegister(INT_TIMER0A, Timer0IntHandler);
    IntRegister(INT_TIMER1A, Timer3IntHandler);

    SendingMsg.startByte1 = 0xAA;
    SendingMsg.startByte2 = 0x55;

    MtoCvar.Pr = 0;
    MtoCvar.solar_available=0;
    MtoCvar.is_peak_time=0;
    MtoCvar.peak_enabled=0;
    MtoCvar.op_power=0;

    // Enable the timers.
    TimerEnable(TIMER0_BASE, TIMER_A);
    //TimerEnable(TIMER1_BASE, TIMER_A);

    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, ~0); //Turn OFF LED

    //Loop forever
    while(1){

        SystemState=(enum states)CtoMvar.SystemState_16bit;
        off_condition=(enum off_conditions)CtoMvar.off_condition_16bit;

        //Update EEPROM when inverter is switched off
        if((SystemState == OFF) && (LastSystemState != OFF) && (LastSystemState != BOOT)){
            EEPROMByteWrite(I2C0_MASTER_BASE, EEPROM_ADDRESS, LAST_OFF_CONDITION_ADDR, (unsigned char)off_condition);
            data2= EEPROMByteRead(I2C0_MASTER_BASE, EEPROM_ADDRESS , LAST_OFF_CONDITION_ADDR);
        }

        LastSystemState=SystemState;

        MtoCvar.basic_configuration = ReceiveMsg1;

        if(SendingMsg.CurrentState != SystemState || SendingMsg.ReasonState != CtoMvar.ReasonState)
            SendUARTInverterStructure();

        count++;
        if ((count%1000000)==0)
        {
            if(LED==1){
                LED = 0;
                GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, ~0); // Turn off LED
                SendUARTInverterStructure();
            }
            else{
                LED = 1;
                GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0); // Turn on LED
            }

        }

    }
}


int ftoa(float value, char *buf, char decimalPoints){
    int ipart = (int)value;
    float fpart = value - (float)ipart;
    ltoa(ipart,buf);
    int i = strlen(buf);

    // check for display option after point
    if (decimalPoints != 0)
    {
        buf[i] = '.';

        // Get the value of fraction part up to given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart = fpart * pow(10, decimalPoints);
        ltoa((int)fpart, buf+i+1);
    }
    return 1;
}

void SendUARTInverterStructure()
{
    unsigned char count,size = SendMsgSize;
    unsigned char *tempdata;//[SendMsgSize];
    //memcpy(tempdata,&SendingMsg.BV,SendMsgSize-5);
    SendingMsg.BatteryVoltage = CtoMvar.BatVoltage;
    SendingMsg.BatteryCurrent = sqrt(CtoMvar.BatCurrent);
    SendingMsg.GridVoltage = sqrt(CtoMvar.GridVoltage);
    SendingMsg.GridCurrent = sqrt(CtoMvar.GridCurrent);
    SendingMsg.HomeVoltage = CtoMvar.InverterVoltage;
    SendingMsg.HomeCurrent = sqrt(CtoMvar.InverterCurrent);
    SendingMsg.PowerFactor = CtoMvar.PowerFactor;
    SendingMsg.CurrentState = SystemState;
    SendingMsg.ReasonState = CtoMvar.ReasonState;
    SendingMsg.crc = CRC8bit(&SendingMsg.BatteryVoltage, (SendMsgSize-6));
    tempdata = &SendingMsg.startByte1;
    count = 0;
    while(size--)
    {
        UARTCharPut(UART1_BASE, tempdata[count]);
        //UARTCharPut(UART0_BASE, data[count]);
        count++;
    }
    //  if(SendingMsg.ACK == ACKed)
    //  {
    //      SendingMsg.ACK = NotACKed;
    //  }
    SendingMsg.ACK = 0;//ACKed;
}

unsigned char CRC8bit(unsigned char*data, unsigned char length)
{
    unsigned char j;
    unsigned short temp1,temp2 = 0;
    for(j = 0; j<length; j+=2 )
    {
        temp1 = data[j]|(data[j+1]<<8);
        temp2 = temp2^temp1;
    }
    return (temp2 & 0xFF)^(temp2>>8);

}

void updateRecievingData() {
    //unsigned char i= 0;//,tempcount = 0;;
    //  while(tempcount<60 && RecievedDataCount > 0)
    //  {
    if(RecieveData[0] == 0xAA && RecieveData[1] == 0x55){
        switch (RecieveData[2]){
            case 1:
                if(RecieveData[Msg1Size + 3] == CRC8bit(&RecieveData[3],Msg1Size)){
                    memcpy(&tempReceiveMsg1,&RecieveData[3],Msg1Size);
                    validateReceiveMsg1();
                    if(SendingMsg.ACK == ACKed)
                        memcpy(&ReceiveMsg1,&RecieveData[3],Msg1Size);
                    break;
                }
                else{
                    SendingMsg.ACK = NotACKed;
                    break;
                }
            case 2:
                if(RecieveData[Msg2Size + 3] == CRC8bit(&RecieveData[3],Msg2Size)){
                    memcpy(&tempReceiveMsg2,&RecieveData[3],Msg2Size);
                    validateReceiveMsg2();
                    if(SendingMsg.ACK == ACKed)
                        memcpy(&ReceiveMsg2,&RecieveData[3],Msg2Size);
                    break;
                }
                else{
                    SendingMsg.ACK = NotACKed;
                    break;
                }
            case 3:
                if(RecieveData[Msg3Size + 3] == CRC8bit(&RecieveData[3],Msg3Size)){
                    //memcpy(&tempReceiveMsg3,&RecieveData[3],Msg3Size);
                    //DataValidationReceiveMsg3();
                    //if(SendingMsg.ACK == ACKed)
                    memcpy(&ReceiveMsg3,&RecieveData[3],Msg3Size);
                    SendingMsg.ACK = ACKed;
                    break;
                }
                else{
                    SendingMsg.ACK = NotACKed;
                    break;
                }
            case 4:
                if(RecieveData[Msg4Size + 3] == CRC8bit(&RecieveData[3],Msg4Size)){
                    memcpy(&ReceiveMsg4,&RecieveData[3],Msg4Size);
                    //SendingMsg.ACK = ACKed;
                    break;
                }
                else{
                    //SendingMsg.ACK = NotACKed;
                    break;
                }

            default :
                MsgSize = 0;
                break;
        }

    }

}

void validateReceiveMsg1() {
    if(tempReceiveMsg1.ChargingCurrent <= ChargingCurrent_MAX &&  tempReceiveMsg1.ChargingCurrent >= ChargingCurrent_MIN)
        SendingMsg.ACK = ACKed;
    else
        SendingMsg.ACK = NotACKed;

    if(tempReceiveMsg1.UPSVoltage <= UPSVoltage_MAX && tempReceiveMsg1.UPSVoltage >= UPSVoltage_MIN)
        SendingMsg.ACK = ACKed;
    else
        SendingMsg.ACK = NotACKed;

    if(tempReceiveMsg1.gridFeedCurrent <= gridFeedCurrent_MAX && tempReceiveMsg1.gridFeedCurrent >= gridFeedCurrent_MIN)
        SendingMsg.ACK = ACKed;
    else
        SendingMsg.ACK = NotACKed;
}

void validateReceiveMsg2() {
    if(tempReceiveMsg2.DC_V_Chrg_Over_Limit <= DC_V_Chrg_Over_Limit_MAX && tempReceiveMsg2.DC_V_Chrg_Over_Limit >= DC_V_Chrg_Over_Limit_MIN)
        SendingMsg.ACK = ACKed;
    else
        SendingMsg.ACK = NotACKed;

    if(tempReceiveMsg2.DC_V_Chrg_Over_Limit_Offset <= DC_V_Chrg_Over_Limit_Offset_MAX && tempReceiveMsg2.DC_V_Chrg_Over_Limit_Offset >= DC_V_Chrg_Over_Limit_Offset_MIN)
        SendingMsg.ACK = ACKed;
    else
        SendingMsg.ACK = NotACKed;

    if(tempReceiveMsg2.DC_V_Chrg_Under_Limit <= DC_V_Chrg_Under_Limit_MAX && tempReceiveMsg2.DC_V_Chrg_Under_Limit >= DC_V_Chrg_Under_Limit_MIN)
        SendingMsg.ACK = ACKed;
    else
        SendingMsg.ACK = NotACKed;

    if(tempReceiveMsg2.DC_V_Grid_tie_start_Limit <= DC_V_Grid_tie_start_Limit_MAX && tempReceiveMsg2.DC_V_Grid_tie_start_Limit >= DC_V_Grid_tie_start_Limit_MIN)
        SendingMsg.ACK = ACKed;
    else
        SendingMsg.ACK = NotACKed;

    if(tempReceiveMsg2.DC_V_Grid_tie_stop_Limit <= DC_V_Grid_tie_stop_Limit_MAX && tempReceiveMsg2.DC_V_Grid_tie_stop_Limit >= DC_V_Grid_tie_stop_Limit_MIN)
        SendingMsg.ACK = ACKed;
    else
        SendingMsg.ACK = NotACKed;

    if(tempReceiveMsg2.DC_V_Under_Limit <= DC_V_Under_Limit_MAX && tempReceiveMsg2.DC_V_Under_Limit >= DC_V_Under_Limit_MIN)
        SendingMsg.ACK = ACKed;
    else
        SendingMsg.ACK = NotACKed;

    if(tempReceiveMsg2.DC_V_Under_UPS_Shut_Limit <= DC_V_Under_UPS_Shut_Limit_MAX && tempReceiveMsg2.DC_V_Under_UPS_Shut_Limit >= DC_V_Under_UPS_Shut_Limit_MIN)
        SendingMsg.ACK = ACKed;
    else
        SendingMsg.ACK = NotACKed;

    if(tempReceiveMsg2.LINE_V_Over_Hard_Limit <= LINE_V_Over_Hard_Limit_MAX && tempReceiveMsg2.LINE_V_Over_Hard_Limit >= LINE_V_Over_Hard_Limit_MIN)
        SendingMsg.ACK = ACKed;
    else
        SendingMsg.ACK = NotACKed;

    if(tempReceiveMsg2.LINE_V_Over_Moderate_Limit <= LINE_V_Over_Moderate_Limit_MAX && tempReceiveMsg2.LINE_V_Over_Moderate_Limit >= LINE_V_Over_Moderate_Limit_MIN)
        SendingMsg.ACK = ACKed;
    else
        SendingMsg.ACK = NotACKed;

    if(tempReceiveMsg2.LINE_V_Over_Soft_Limit <= LINE_V_Over_Soft_Limit_MAX && tempReceiveMsg2.LINE_V_Over_Soft_Limit >= LINE_V_Over_Soft_Limit_MIN)
        SendingMsg.ACK = ACKed;
    else
        SendingMsg.ACK = NotACKed;

    if(tempReceiveMsg2.LINE_V_Threshold <= LINE_V_Threshold_MAX && tempReceiveMsg2.LINE_V_Threshold >= LINE_V_Threshold_MIN)
        SendingMsg.ACK = ACKed;
    else
        SendingMsg.ACK = NotACKed;

    if(tempReceiveMsg2.LINE_V_Under_Limit <= LINE_V_Under_Limit_MAX && tempReceiveMsg2.LINE_V_Under_Limit >= LINE_V_Under_Limit_MIN)
        SendingMsg.ACK = ACKed;
    else
        SendingMsg.ACK = NotACKed;

    if(tempReceiveMsg2.OP_I_Limit <= OP_I_Limit_MAX && tempReceiveMsg2.OP_I_Limit >= OP_I_Limit_MIN)
        SendingMsg.ACK = ACKed;
    else
        SendingMsg.ACK = NotACKed;

    if(tempReceiveMsg2.max_chrg_current <= max_chrg_current_MAX && tempReceiveMsg2.max_chrg_current >= max_chrg_current_MIN)
        SendingMsg.ACK = ACKed;
    else
        SendingMsg.ACK = NotACKed;

    if(tempReceiveMsg2.min_chrg_current <= min_chrg_current_MAX && tempReceiveMsg2.min_chrg_current >= min_chrg_current_MIN)
        SendingMsg.ACK = ACKed;
    else
        SendingMsg.ACK = NotACKed;
}

