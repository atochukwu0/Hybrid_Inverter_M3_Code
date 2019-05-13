//###########################################################################
// FILE:   uart_echo.c
// TITLE:  Example for reading data from and writing data to the UART in
//         an interrupt driven fashion.
//###########################################################################
// $TI Release: F28M35x Driver Library v100 $
// $Release Date: October 12, 2011 $
//###########################################################################
//Major change to data aqquasition structure
//Used structures to share data between processors, Used start flag to sync pointers.

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

#include "inc/hw_ipc.h" // TRD 2018/04/27
#include "inc/hw_ram.h"


#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/flash.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/gpio.h"
#include "driverlib/ipc.h"
#include "driverlib/ram.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"


#include "utils/ustdlib.h"
#include "utils/uartstdio.h"
#include "utils/memcopy.h"
#include "Solar_DC_AC_IPC.h"
#include "LiquidCrystal_PCF8574.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define M3_MASTER 0
#define C28_MASTER 1

#define LCD_ADDRESS 0x27

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
    GRID_TIED,
    ERROR
};

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

struct CtoMData {

    unsigned long long Pw;
    unsigned short start_flag;

};

struct MtoCData {
    unsigned long long Pr;
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


volatile long count=0,count2=0;
volatile int index_drawn=0,edit_mode=0,edit_row_index=0,editvar=0;
volatile int Enter_pressed=0,Back_pressed=0,INCRE_pressed=0,DECRE_pressed=0;

volatile enum pages {
    HOME,
    TEST,
    NETWORK_SETTINGS,
    VOLTAGE_SETTINGS,
    CURRENT_SETTINGS,
    DELAY_SETTINGS
} currentPage;


//Initialize variable,these needed to be update otherwise in real application
char *Datetime="2019/05/02 19:40";

//*****************************************************************************
// The error routine that is called if the driver library encounters an error.
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}

#endif

volatile int LED = 0;
//*****************************************************************************
// The interrupt handler for the first timer interrupt.
//*****************************************************************************
void
Timer0IntHandler(void)
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
        UARTCharPut(UART0_BASE,0xAA);   //Send Begining
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

//*****************************************************************************
// The interrupt handler for the second timer interrupt.
//*****************************************************************************
void
Timer1IntHandler(void)
{
    // Clear the timer interrupt.
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    // Toggle the flag for the second timer.
    HWREGBITW(&g_ulFlags, 1) ^= 1;

}

//*****************************************************************************
// The UART interrupt handler.
//*****************************************************************************
void
UARTIntHandler(void)
{
    unsigned long ulStatus;

    // Get the interrrupt status.
    ulStatus = UARTIntStatus(UART0_BASE, true);

    // Clear the asserted interrupts.
    UARTIntClear(UART0_BASE, ulStatus);

    // Loop while there are characters in the receive FIFO.

}

//*****************************************************************************
// Send a string to the UART.
//*****************************************************************************
void
UARTSend(const unsigned char *pucBuffer, unsigned long ulCount)
{
    // Loop while there are more characters to send.
    while(ulCount--)
    {
        // Write the next character to the UART.
        UARTCharPutNonBlocking(UART0_BASE, *pucBuffer++);
    }
}



int
main(void)
{

    // Disable Protection
    HWREG(SYSCTL_MWRALLOW) =  0xA5A5A5A5;

    // Tells M3 Core the vector table is at the beginning of C0 now.
    HWREG(NVIC_VTABLE) = 0x20005000;


    // Setup main clock tree for 75MHz - M3 and 150MHz - C28x
    SysCtlClockConfigSet(SYSCTL_SYSDIV_1 | SYSCTL_M3SSDIV_2 | SYSCTL_USE_PLL |
                         (SYSCTL_SPLLIMULT_M & 0x0F));

    MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
    // Call Flash Initialization to setup flash waitstates
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
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);


    //Select core for controlling GPIO
    GPIOPinConfigureCoreSelect(GPIO_PORTA_BASE, 0xFF, GPIO_PIN_C_CORE_SELECT);
    GPIOPinConfigureCoreSelect(GPIO_PORTB_BASE, 0x3F, GPIO_PIN_C_CORE_SELECT);  // Two pins used for I2C
    GPIOPinConfigureCoreSelect(GPIO_PORTC_BASE, 0x7F, GPIO_PIN_C_CORE_SELECT);  // 1 pins used byt M3 for blink LED
    GPIOPinConfigureCoreSelect(GPIO_PORTD_BASE, 0xFF, GPIO_PIN_C_CORE_SELECT);
    GPIOPinConfigureCoreSelect(GPIO_PORTE_BASE, 0xCF, GPIO_PIN_C_CORE_SELECT); // Two pins used by M3 for UART
    GPIOPinConfigureCoreSelect(GPIO_PORTF_BASE, 0xDF, GPIO_PIN_C_CORE_SELECT); // 1 switch for menu usage
    GPIOPinConfigureCoreSelect(GPIO_PORTG_BASE, 0xFF, GPIO_PIN_C_CORE_SELECT);
    GPIOPinConfigureCoreSelect(GPIO_PORTH_BASE, 0xFF, GPIO_PIN_C_CORE_SELECT);
    GPIOPinConfigureCoreSelect(GPIO_PORTJ_BASE, 0x8F, GPIO_PIN_C_CORE_SELECT);  // 3 switches for menu usage


    GPIOPadConfigSet(GPIO_PORTA_BASE, 0xFF, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTB_BASE, 0x3F, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTC_BASE, 0x7F, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTD_BASE, 0xFF, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTE_BASE, 0xCF, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTF_BASE, 0xDF, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTG_BASE, 0xFF, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTH_BASE, 0xFF, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, 0x8F, GPIO_PIN_TYPE_STD_WPU);

//    GPIODirModeSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_DIR_MODE_IN);  //Set the PA6 as input


    // Set GPIO E4 and E5 as UART pins.
    GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    GPIOPinConfigure(GPIO_PE4_U0RX);
    GPIOPinConfigure(GPIO_PE5_U0TX);

    //Set up pins for I2C
    //Unlock GPIO This has to be done because this pin is a NMI
    GPIOPinUnlock(GPIO_PORTB_BASE, GPIO_PIN_7); //This function was not initialy available on v100 files. Copied the functions to v100 files from v220 files
    GPIOPinConfigure(GPIO_PB6_I2C0SDA);
    GPIOPinConfigure(GPIO_PB7_I2C0SCL);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    // Set up the Pin for LED
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7);
    //Set up the pins for SW

    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE,GPIO_PIN_5);
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_4);
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_5);
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_6);


    SysCtlReleaseSubSystemFromReset(SYSCTL_CONTROL_SYSTEM_RES_CNF);




 //   master_ram_init_control_m0m1_msgram_memories();
 //   master_ram_init_control_L0_L4_memories();


//    RamMReqSharedMemAccess((S6_ACCESS),C28_MASTER);
//    RamMReqSharedMemAccess((S7_ACCESS),M3_MASTER);
    // assign S2 and S3 of the shared ram for use by the c28
    // Details of how c28 uses these memory sections is defined
    // in the c28 linker file.(28M35H52C1_RAM_lnk.cmd)
    RamMReqSharedMemAccess((S0_ACCESS | S1_ACCESS |S2_ACCESS | S3_ACCESS ),C28_MASTER);
//    RamMReqSharedMemAccess((S4_ACCESS | S7_ACCESS),M3_MASTER);
//    volatile unsigned long ulLoop;
//
//    for(ulLoop = 0; ulLoop < 2000000; ulLoop++)
//    {
//    }

    //Vieri/20111123/For buffer data get
//    RamMReqSharedMemAccess((S5_ACCESS),C28_MASTER);

    IPCMtoCBootControlSystem(CBROM_MTOC_BOOTMODE_BOOT_FROM_FLASH);

    // Enable processor interrupts.
    IntMasterEnable();

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.  For this example we will use a data rate of 100kbps.
    I2CMasterEnable(I2C0_MASTER_BASE);
    I2CMasterInitExpClk(I2C0_MASTER_BASE, SysCtlClockGet(
                            SYSTEM_CLOCK_SPEED), false);

    // Tell the master module what address it will place on the bus when
    // communicating with the slave.  Set the address to SLAVE_ADDRESS
    // (as set in the slave module).  The receive parameter is set to false
    // which indicates the I2C Master is initiating a writes to the slave.  If
    // true, that would indicate that the I2C Master is initiating reads from
    // the slave.
    I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, LCD_ADDRESS, false);

    // Configure the UART for 115,200, 8-N-1 operation.
     UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED), 8000000,
                         (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                          UART_CONFIG_PAR_NONE));

     // Enable the UART interrupt.
     IntRegister(INT_UART0, UARTIntHandler);
     IntEnable(INT_UART0);
     UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX | UART_INT_CTS);
     UARTFIFOLevelSet(UART0_BASE,UART_FIFO_TX1_8 ,UART_FIFO_RX1_8);

    // Configure the two 32-bit periodic timers.
    TimerConfigure(TIMER0_BASE, TIMER_CFG_32_BIT_PER);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_32_BIT_PER);
    TimerLoadSet(TIMER0_BASE, TIMER_A, (SysCtlClockGet(SYSTEM_CLOCK_SPEED)/10000)); //10kHz timer interrupt , used for data transfer
    TimerLoadSet(TIMER1_BASE, TIMER_A, (SysCtlClockGet(SYSTEM_CLOCK_SPEED)/4));    // 4Hz timer interrupt , used for button de-bounce

    // Setup the interrupts for the timer timeouts.
    IntEnable(INT_TIMER0A);
    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    IntRegister(INT_TIMER0A, Timer0IntHandler);
    IntRegister(INT_TIMER1A, Timer1IntHandler);

    MtoCvar.Pr = 0;

    // Enable the timers.
    TimerEnable(TIMER0_BASE, TIMER_A);
    TimerEnable(TIMER1_BASE, TIMER_A);

    //Loop forever
    while(1){

        // Toggle the LED.
        if (count>10)
    	{
    	    if(LED==0){
    			LED = 1;
    			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, ~0);
    		}
    		else{
    			LED = 0;
    			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0);
    		}
    		count=0;
    	}
    	else
    		count++;

        delayMicroseconds(10000);

    }
}


int ftoa(float value, char *buf, char decimalPoints)
{
    int ipart = (int)value;
    float fpart=value-(float)ipart;
    ltoa(ipart,buf);
    int i=strlen(buf);

    // check for display option after point
    if (decimalPoints != 0)
    {
        buf[i] = '.';

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart = fpart * pow(10, decimalPoints);
        ltoa((int)fpart,buf+i+1);
    }
    return 1;
}
