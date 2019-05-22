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

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_i2c.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_ipc.h" // TRD 2018/04/27
#include "inc/hw_ram.h"
#include "inc/hw_can.h"


#include "driverlib/i2c.h"
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
#include "driverlib/can.h"


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
    GRID_TIED_SOLAR,
    GRID_TIED_PEAK,
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
    float Vdc;
    float Vgrid;
    float Vout;
    enum states system_state;
};

struct MtoCData {
    unsigned long long Pr;
    unsigned short solar_available;
    unsigned short is_peaktime;
    unsigned short peak_enabled;
    unsigned short op_power;
    unsigned short battery_voltage;
    unsigned short HFactor;
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
#define DLOG_SIZE 400	// uncomment for FLASH configuration only
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
void CANIntHandler(void);
void decodeRXFrame(unsigned char *ucRXData, unsigned int uiMsgIndex);
void sendTXFrame(uint8_t mode);
void requestBatteryData(uint8_t type);

//int	SerialCommsTimer;
//int	CommsOKflg;

/********************CAN related variables*******************************************/
unsigned char ucTXMsgData[8], ucRXMsgData[8], ucRXMsgData_1db[8];                   //
tCANMsgObject sTXCANMessage;                                                        //
tCANMsgObject sRXCANMessage_7bb;                                                    //
tCANMsgObject sRXCANMessage_1db;                                                    //
                                                                                    //
uint8_t index_7bb = 0;                                                              //
uint8_t mode_7bb = 0;                                                               //
uint8_t cell_index = 0;                                                             //
                                                                                    //
uint16_t cellVoltages[96];                                                          //
uint32_t voltage = 0;                                                               //
uint16_t raw_battery_current = 0;                                                   //
uint16_t battery_current = 0;                                                       //
float battery_current_float = 0;                                                    //
uint32_t HFactor = 0;                                                               //
uint32_t SOC = 0;                                                                   //
uint32_t capacity = 0;                                                              //
uint8_t shuntStatuses[12];                                                          //
/*   [int 0,         int 2,   .......... int 11]                                    //
     [0bxxxxxxxx, 0bxxxxxxxx, .......... 0bxxxxxxxx]                                //
     These 96 bits indicates shunt statuses in ascending order */                   //
                                                                                    //
/************************************************************************************/

//Variable used for data transfer
volatile int base_read_index =0;
volatile char *character_pointer;
volatile int index=0;
volatile char serial_print_char;

//Variables for the menu
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
char CAN_UART_buffer[6];

volatile int LED = 0;

//*****************************************************************************
// The error routine that is called if the driver library encounters an error.
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}

#endif


void CANIntHandler(void){

    unsigned long ulStatus;
    // Read the CAN interrupt status to find the cause of the interrupt
    ulStatus = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);
    // If the cause is a controller status interrupt, then get the status
    if(ulStatus == CAN_INT_INT0ID_STATUS){

        ulStatus = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);
        //ltoa(ulStatus, CAN_UART_buffer);
        //UARTCharPut(UART0_BASE, CAN_UART_buffer);
        //UARTCharPut(UART0_BASE, '\n');
        if (ulStatus == CAN_STATUS_RXOK){
            //ltoa(ulStatus, CAN_UART_buffer);
            //UARTCharPut(UART0_BASE, CAN_UART_buffer);
            //UARTCharPut(UART0_BASE, '\n');
            ulStatus = CANStatusGet(CAN0_BASE, CAN_STS_NEWDAT) ;    //This shit returns 32bit bitmap for all message objects at once.
           // msg obj id 4 sets 4th bit, msg obj id 5 sets 5th bit likewise
           if (ulStatus & 0x00000002){
                CANMessageGet(CAN0_BASE, 2, &sRXCANMessage_7bb, true);
                //UARTprintf("Receiving message from object 2: %02X %02X %02X %02X %02X %02X %02X %02X\n", ucRXMsgData[0], ucRXMsgData[1], ucRXMsgData[2], ucRXMsgData[3], ucRXMsgData[4], ucRXMsgData[5], ucRXMsgData[6], ucRXMsgData[7]);
                decodeRXFrame(ucRXMsgData, 0x7bb);
                memset(ucRXMsgData, 0, sizeof(ucRXMsgData));
           }

           else if (ulStatus & 0x00000004){
                CANMessageGet(CAN0_BASE, 3, &sRXCANMessage_1db, true);
                //UARTprintf("Receiving message from object 3: %02X %02X %02X %02X %02X %02X %02X %02X\n", ucRXMsgData_1db[0], ucRXMsgData_1db[1], ucRXMsgData_1db[2], ucRXMsgData_1db[3], ucRXMsgData_1db[4], ucRXMsgData_1db[5], ucRXMsgData_1db[6], ucRXMsgData_1db[7]);
                decodeRXFrame(ucRXMsgData_1db, 0x1db);
                memset(ucRXMsgData_1db, 0, sizeof(ucRXMsgData_1db));
           }

        }

        CANIntClear(CAN0_BASE, 2);
        CANIntClear(CAN0_BASE, 3);

    }
}


void decodeRXFrame(unsigned char *ucRXData, unsigned int uiMsgIndex){

    if(uiMsgIndex==0x7bb){

        if (mode_7bb == 1){

            if(index_7bb == 4){
                HFactor = (ucRXData[2]<< 8) + ucRXData[3];
                SOC = (ucRXData[5]<<16) + (ucRXData[6]<<8) + (ucRXData[7]);
                index_7bb++;
            }

            else if(index_7bb == 5){
                capacity = (ucRXData[2]<<16) + (ucRXData[3]<<8) + (ucRXData[4]);
                index_7bb = 0;
                //CANIntDisable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
                return;
            }

            else if(index_7bb > 5){
                index_7bb = 0;
                //CANIntDisable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
                return;
            }

            else{
                index_7bb++;
            }

            sendTXFrame(4);

        }

        else if(mode_7bb == 2){

            if(index_7bb == 0){
                cellVoltages[0] = (ucRXData[4]<<8) + ucRXData[5];
                cellVoltages[1] = (ucRXData[6]<<8) + ucRXData[7];
                cell_index = 2;
                index_7bb++;
            }

            else if(index_7bb == 27){
                cellVoltages[93] = (ucRXData[1]<<8) + ucRXData[2];
                cellVoltages[94] = (ucRXData[3]<<8) + ucRXData[4];
                cellVoltages[95] = (ucRXData[5]<<8) + ucRXData[6];
                cell_index = 0;
                index_7bb = 0;
                //CANIntDisable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
                return;
            }

            else if(index_7bb > 27){
                cell_index = 0;
                index_7bb = 0;
                //CANIntDisable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
                return;
            }

            else if((index_7bb%2) == 0){
                cellVoltages[cell_index] = cellVoltages[cell_index] + ucRXData[1];
                cellVoltages[cell_index + 1] = (ucRXData[2]<<8) + ucRXData[3];
                cellVoltages[cell_index + 2] = (ucRXData[4]<<8) + ucRXData[5];
                cellVoltages[cell_index + 3] = (ucRXData[6]<<8) + ucRXData[7];
                cell_index = cell_index + 4;
                index_7bb++;
            }

            else if((index_7bb%2) != 0){
                cellVoltages[cell_index] = (ucRXData[1]<<8) + ucRXData[2];
                cellVoltages[cell_index + 1] = (ucRXData[3]<<8) + ucRXData[4];
                cellVoltages[cell_index + 2] = (ucRXData[5]<<8) + ucRXData[6];
                cellVoltages[cell_index + 3] = (ucRXData[7]<<8);
                cell_index = cell_index + 3;
                index_7bb++;
            }

            sendTXFrame(4);

         }

        else if (mode_7bb == 3){

            if (index_7bb == 0){
                shuntStatuses[0] = ((ucRXData[4] & 0xF) << 4) + (ucRXData[5] & 0xF);
                shuntStatuses[1] = ((ucRXData[6] & 0xF) << 4) + (ucRXData[7] & 0xF);
                index_7bb++;
            }

            else if(index_7bb == 1){
                shuntStatuses[2] = ((ucRXData[1] & 0xF) << 4) + (ucRXData[2] & 0xF);
                shuntStatuses[3] = ((ucRXData[3] & 0xF) << 4) + (ucRXData[4] & 0xF);
                shuntStatuses[4] = ((ucRXData[5] & 0xF) << 4) + (ucRXData[6] & 0xF);
                shuntStatuses[5] = ((ucRXData[7] & 0xF) << 4);
                index_7bb++;
            }

            else if(index_7bb == 2){
                shuntStatuses[5] = shuntStatuses[5] + (ucRXData[1] & 0xF);
                shuntStatuses[6] = ((ucRXData[2] & 0xF) << 4) + (ucRXData[3] & 0xF);
                shuntStatuses[7] = ((ucRXData[4] & 0xF) << 4) + (ucRXData[5] & 0xF);
                shuntStatuses[8] = ((ucRXData[6] & 0xF) << 4) + (ucRXData[7] & 0xF);
                index_7bb++;
            }

            else if (index_7bb == 3){
                shuntStatuses[9] = ((ucRXData[1] & 0xF) << 4) + (ucRXData[2] & 0xF);
                shuntStatuses[10] = ((ucRXData[3] & 0xF) << 4) + (ucRXData[4] & 0xF);
                shuntStatuses[11] = ((ucRXData[5] & 0xF) << 4) + (ucRXData[6] & 0xF);
                index_7bb = 0;
                //CANIntDisable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
                return;
            }

            else {
                index_7bb = 0;
                //CANIntDisable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
                return;
            }

            sendTXFrame(4);

        }

    }

    else if(uiMsgIndex==0x1db){
        voltage = ((ucRXData[2] << 2) + (ucRXData[3] >> 6)) * 5;    //Original value from can message is voltage * 2. Therefore multiplied by 5 to get voltage * 10.
        raw_battery_current = ((ucRXData[0]  << 3) + (ucRXData[1] >> 13)) << 5;
        battery_current = (raw_battery_current/32) * 5;           //Original value from can message is current * 2. Therefore multiplied by 5 to get current * 10.
        battery_current_float = raw_battery_current/64.0;
        //Disabling CAN Interrupt here gives bad results when reading other parameters
        //CANIntDisable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
    }

}


void sendTXFrame(uint8_t mode){

    if(mode == 1){
        ucTXMsgData[0] = 0x02;
        ucTXMsgData[1] = 0x21;
        ucTXMsgData[2] = 0x01;
        ucTXMsgData[3] = 0xff;
        ucTXMsgData[4] = 0xff;
        ucTXMsgData[5] = 0xff;
        ucTXMsgData[6] = 0xff;
        ucTXMsgData[7] = 0xff;
    }
    else if(mode == 2){
        ucTXMsgData[0] = 0x02;
        ucTXMsgData[1] = 0x21;
        ucTXMsgData[2] = 0x02;
        ucTXMsgData[3] = 0xff;
        ucTXMsgData[4] = 0xff;
        ucTXMsgData[5] = 0xff;
        ucTXMsgData[6] = 0xff;
        ucTXMsgData[7] = 0xff;
    }
    else if(mode == 3){
        ucTXMsgData[0] = 0x02;
        ucTXMsgData[1] = 0x21;
        ucTXMsgData[2] = 0x06;
        ucTXMsgData[3] = 0xff;
        ucTXMsgData[4] = 0xff;
        ucTXMsgData[5] = 0xff;
        ucTXMsgData[6] = 0xff;
        ucTXMsgData[7] = 0xff;
    }
    else if(mode == 4){
        ucTXMsgData[0] = 0x30;
        ucTXMsgData[1] = 0x01;
        ucTXMsgData[2] = 0x00;
        ucTXMsgData[3] = 0xff;
        ucTXMsgData[4] = 0xff;
        ucTXMsgData[5] = 0xff;
        ucTXMsgData[6] = 0xff;
        ucTXMsgData[7] = 0xff;
    }

    CANMessageSet(CAN0_BASE, 2, &sRXCANMessage_7bb, MSG_OBJ_TYPE_RX);
    CANMessageSet(CAN0_BASE, 1, &sTXCANMessage, MSG_OBJ_TYPE_TX);
}


void requestBatteryData(uint8_t type){
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
    if (type != 4){
        index_7bb = 0;
        cell_index = 0;
        mode_7bb = type;
        sendTXFrame(type);
    }
}


void Timer0IntHandler(void){        // Timer 0 ISR
    // Clear the timer interrupt.
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //Data transfer
    if(CtoMvar.start_flag==1)
            MtoCvar.Pr=0;
    while(CtoMvar.Pw > MtoCvar.Pr){
        //while(UARTCharPutNonBlocking(UART0_BASE,serial_print_char)){
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

void Timer1IntHandler(void) {       // Timer 1 ISR
    // Clear the timer interrupt.
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    if(!GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_4))
        DECRE_pressed = 1;
    if(!GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_5))
        Enter_pressed = 1;
    if(!GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_6))
        Back_pressed = 1;
    if(!GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_5))
        INCRE_pressed = 1;

    // Toggle the flag for the second timer.
    HWREGBITW(&g_ulFlags, 1) ^= 1;

}

void UARTIntHandler(void){	    //UART ISR
    unsigned long ulStatus;
    // Get the interrupt status.
    ulStatus = UARTIntStatus(UART0_BASE, true);
    // Clear the asserted interrupts.
    UARTIntClear(UART0_BASE, ulStatus);
    // Loop while there are characters in the receive FIFO.
}

void UARTSend(const unsigned char *pucBuffer, unsigned long ulCount){	//Send a string to the UART
    while(ulCount--)        // Loop while there are more characters to send.
        UARTCharPutNonBlocking(UART0_BASE, *pucBuffer++);   // Write the next character to the UART.
}

int a=54325;
char buffer[20];
float b=4562.26;


int main(void) {

    // Disable Protection
    HWREG(SYSCTL_MWRALLOW) =  0xA5A5A5A5;

    // Tells M3 Core the vector table is at the beginning of C0 now.
    HWREG(NVIC_VTABLE) = 0x20005000;

    // Setup main clock tree for 75MHz - M3 and 150MHz - C28x
    SysCtlClockConfigSet(SYSCTL_SYSDIV_1 | SYSCTL_M3SSDIV_2 | SYSCTL_USE_PLL | (SYSCTL_SPLLIMULT_M & 0x0F));

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

    //Select core for controlling GPIO
    GPIOPinConfigureCoreSelect(GPIO_PORTA_BASE, 0xFF, GPIO_PIN_C_CORE_SELECT);
    GPIOPinConfigureCoreSelect(GPIO_PORTB_BASE, 0x3F, GPIO_PIN_C_CORE_SELECT);  // Two pins used for I2C
    GPIOPinConfigureCoreSelect(GPIO_PORTC_BASE, 0x7F, GPIO_PIN_C_CORE_SELECT);  // 1 pins used by M3 for blink LED
    GPIOPinConfigureCoreSelect(GPIO_PORTD_BASE, 0xFF, GPIO_PIN_C_CORE_SELECT);
    GPIOPinConfigureCoreSelect(GPIO_PORTE_BASE, 0xCC, GPIO_PIN_C_CORE_SELECT);  // Two pins for UART & another two for CAN - aroshaD
    GPIOPinConfigureCoreSelect(GPIO_PORTF_BASE, 0xDF, GPIO_PIN_C_CORE_SELECT);  // 1 switch for menu usage
    GPIOPinConfigureCoreSelect(GPIO_PORTG_BASE, 0xFF, GPIO_PIN_C_CORE_SELECT);
    GPIOPinConfigureCoreSelect(GPIO_PORTH_BASE, 0xFF, GPIO_PIN_C_CORE_SELECT);
    GPIOPinConfigureCoreSelect(GPIO_PORTJ_BASE, 0x8F, GPIO_PIN_C_CORE_SELECT);  // 3 switches for menu usage

    GPIOPadConfigSet(GPIO_PORTA_BASE, 0xFF, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTB_BASE, 0x3F, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTC_BASE, 0x7F, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTD_BASE, 0xFF, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTE_BASE, 0xCC, GPIO_PIN_TYPE_STD_WPU); //Changed from 0xCF to 0xCC to release CAN pins - aroshaD
    GPIOPadConfigSet(GPIO_PORTF_BASE, 0xDF, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTG_BASE, 0xFF, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTH_BASE, 0xFF, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, 0x8F, GPIO_PIN_TYPE_STD_WPU);

    //Set GPIO E4 and E5 as UART pins.
    GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    GPIOPinConfigure(GPIO_PE4_U0RX);
    GPIOPinConfigure(GPIO_PE5_U0TX);

    //Setup pins for CAN
    GPIOPinConfigure(GPIO_PE0_CAN0RX);
    GPIOPinConfigure(GPIO_PE1_CAN0TX);
    GPIOPinTypeCAN(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //Setup pins for I2C
    //Unlock GPIO This has to be done because this pin is a NMI
    GPIOPinUnlock(GPIO_PORTB_BASE, GPIO_PIN_7);
    GPIOPinConfigure(GPIO_PB6_I2C0SDA);
    GPIOPinConfigure(GPIO_PB7_I2C0SCL);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7);   //GPIO14 | GPIO15

    // Set up the Pin for LED
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7);

    //Set up the pins for SW
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE,GPIO_PIN_5);   //GPIO37
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_4);   //GPIO60
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_5);   //GPIO61
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_6);   //GPIO62

    // Enable the peripherals used
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);

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

//    Vieri/20111123/For buffer data get
//    RamMReqSharedMemAccess((S5_ACCESS),C28_MASTER);

    IPCMtoCBootControlSystem(CBROM_MTOC_BOOTMODE_BOOT_FROM_FLASH);

    // Enable processor interrupts.
    IntMasterEnable();

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.  For this example we will use a data rate of 100kbps.
    I2CMasterEnable(I2C0_MASTER_BASE);
    I2CMasterInitExpClk(I2C0_MASTER_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED), false);

    // Tell the master module what address it will place on the bus when
    // communicating with the slave.  Set the address to SLAVE_ADDRESS
    // (as set in the slave module).  The receive parameter is set to false
    // which indicates the I2C Master is initiating a writes to the slave.  If
    // true, that would indicate that the I2C Master is initiating reads from
    // the slave.
    I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, LCD_ADDRESS, false);

    //CAN Configuration - aroshaD
    CANInit(CAN0_BASE);
    CANClkSourceSelect(CAN0_BASE, CAN_CLK_M3);
    CANBitRateSet(CAN0_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED), 500000);
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
    IntRegister(INT_CAN0INT0, CANIntHandler);
    IntPrioritySet(INT_CAN0INT0, 5);
    IntEnable(INT_CAN0INT0);
    CANEnable(CAN0_BASE);

    // Configure the UART for 115,200, 8-N-1 operation.
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED), 8000000, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    // Enable the UART interrupt.
    IntRegister(INT_UART0, UARTIntHandler);
    IntPrioritySet(INT_UART0, 4);       //4 is the highest possible priority defined in interrupt.c - aroshaD
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX | UART_INT_CTS);
    UARTFIFOLevelSet(UART0_BASE,UART_FIFO_TX1_8 ,UART_FIFO_RX1_8);

    // Configure the two 32-bit periodic timers.
    TimerConfigure(TIMER0_BASE, TIMER_CFG_32_BIT_PER);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_32_BIT_PER);
    TimerLoadSet(TIMER0_BASE, TIMER_A, (SysCtlClockGet(SYSTEM_CLOCK_SPEED)/10000)); //10kHz timer interrupt , used for data transfer
    TimerLoadSet(TIMER1_BASE, TIMER_A, (SysCtlClockGet(SYSTEM_CLOCK_SPEED)/2));    // 2Hz timer interrupt , used for button de-bounce
    // Setup the interrupts for the timer timeouts.
    IntEnable(INT_TIMER0A);
    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    IntRegister(INT_TIMER0A, Timer0IntHandler);
    IntRegister(INT_TIMER1A, Timer1IntHandler);
    IntPrioritySet(INT_TIMER0A, 4);
    IntPrioritySet(INT_TIMER1A, 4);
    // Enable the timers.
    TimerEnable(TIMER0_BASE, TIMER_A);
    TimerEnable(TIMER1_BASE, TIMER_A);

/*************************************CAN Frames*****************************************/
    *(unsigned long *)ucTXMsgData = 0;                                                  //
    sTXCANMessage.ulMsgID = 0x79b;                          // CAN message ID           //
    sTXCANMessage.ulMsgIDMask = 0;                                                      //
    sTXCANMessage.ulFlags = MSG_OBJ_NO_FLAGS;               // no Interrupts for TX     //
    sTXCANMessage.ulMsgLen = sizeof(ucTXMsgData);                                       //
    sTXCANMessage.pucMsgData = ucTXMsgData;                                             //
                                                                                        //
    *(unsigned long *)ucRXMsgData = 0;                                                  //
    sRXCANMessage_7bb.ulMsgID = 0x7bb;                      // CAN message ID           //
    sRXCANMessage_7bb.ulMsgIDMask = 0x7bb;                  // mask for RX              //
    sRXCANMessage_7bb.ulFlags = MSG_OBJ_RX_INT_ENABLE;      // enable interrupt on RX   //
    sRXCANMessage_7bb.ulMsgLen = sizeof(ucRXMsgData);                                   //
    sRXCANMessage_7bb.pucMsgData = ucRXMsgData;                                         //
                                                                                        //
    *(unsigned long *)ucRXMsgData_1db = 0;                                              //
    sRXCANMessage_1db.ulMsgID = 0x1db;                      // CAN message ID - use 1   //
    sRXCANMessage_1db.ulMsgIDMask = 0x1db;                  // mask for RX              //
    sRXCANMessage_1db.ulFlags = MSG_OBJ_RX_INT_ENABLE;      // enable interrupt on RX   //
    sRXCANMessage_1db.ulMsgLen = sizeof(ucRXMsgData_1db);                               //
    sRXCANMessage_1db.pucMsgData = ucRXMsgData_1db;                                     //
                                                                                        //
    CANMessageSet(CAN0_BASE, 3, &sRXCANMessage_1db, MSG_OBJ_TYPE_RX);                   //
/****************************************************************************************/



    MtoCvar.Pr = 0;
    MtoCvar.solar_available=0;
    MtoCvar.is_peaktime=0;
    MtoCvar.peak_enabled=0;
    MtoCvar.op_power=0;

    currentPage=HOME;
    MtoCvar.peak_enabled=initializeLCD(20, 4, LCD_5x8DOTS);
    MtoCvar.peak_enabled=setBacklightLCD(255);
    MtoCvar.peak_enabled=homeLCD();
    MtoCvar.peak_enabled=clearLCD();

//    cursorLCD();
//    blinkLCD();
//    printLCD(0,0,"JLanka");
//    ltoa(a,buffer);
//    printLCD(0,1,buffer);
//    ftoa(b,buffer,2);
//    printLCD(0,2,buffer);

    MtoCvar.peak_enabled=printLCD(1,0,"Solar flag: ");
    MtoCvar.peak_enabled=printLCD(1,1,"Peak flag: ");
    MtoCvar.peak_enabled=printLCD(1,2,"OP Power: ");
    ltoa(MtoCvar.solar_available,buffer);
    MtoCvar.peak_enabled=printLCD(14,0,buffer);
    ltoa(MtoCvar.is_peaktime,buffer);
    MtoCvar.peak_enabled=printLCD(14,1,buffer);
    ltoa(MtoCvar.op_power,buffer);
    MtoCvar.peak_enabled=printLCD(14,2,buffer);



    //Loop forever
    while(1){

//        printLCD(0,0," ");
//        printLCD(0,1," ");
//        printLCD(0,2," ");
//        printLCD(0,edit_row_index,">");



        if(edit_mode){
            ltoa(editvar,buffer);
            MtoCvar.peak_enabled= printLCD(14,edit_row_index,buffer);
            MtoCvar.peak_enabled=setCursorLCD(14, edit_row_index);
            MtoCvar.peak_enabled=blinkLCD();
        }
        else{
            switch(edit_row_index){
             case 0:
                 editvar=MtoCvar.solar_available;
                 break;
             case 1:
                 editvar=MtoCvar.is_peaktime;
                 break;
             case 2:
                 editvar=MtoCvar.op_power;
                 break;
             default:
                 break;
             }
        }

        if(DECRE_pressed){
            DECRE_pressed=0;
            if(edit_mode && editvar>0){
                editvar--;
            }
            else{
                if(edit_row_index<3){
                    edit_row_index++;
                    MtoCvar.peak_enabled=printLCD(0,0," ");
                    MtoCvar.peak_enabled=printLCD(0,1," ");
                    MtoCvar.peak_enabled=printLCD(0,2," ");
                    MtoCvar.peak_enabled=printLCD(0,edit_row_index,">");
                }
            }
        }
        if(INCRE_pressed){
            INCRE_pressed=0;
            if(edit_mode){
                editvar++;
            }
            else
                if(edit_row_index>0){
                    edit_row_index--;
                    MtoCvar.peak_enabled=printLCD(0,0," ");
                    MtoCvar.peak_enabled=printLCD(0,1," ");
                    MtoCvar.peak_enabled=printLCD(0,2," ");
                    MtoCvar.peak_enabled=printLCD(0,edit_row_index,">");
                }

        }
        if(Enter_pressed){
            Enter_pressed=0;
            if(!edit_mode){
            edit_mode=1;
            }
            else{
                edit_mode=0;
                MtoCvar.peak_enabled=noBlinkLCD();
                switch(edit_row_index){
                case 0:
                    MtoCvar.solar_available=editvar;
                    break;
                case 1:
                    MtoCvar.is_peaktime=editvar;
                    break;
                case 2:
                    MtoCvar.op_power=editvar;
                    break;
                default:
                    break;

                }
            }
        }
        if(Back_pressed){
            Back_pressed=0;
            edit_mode=0;
            MtoCvar.peak_enabled=noBlinkLCD();
        }


        // Toggle the LED.
        if (count>10)
    	{
            count2++;
            ltoa(count2,buffer);
            MtoCvar.peak_enabled= printLCD(2,3,buffer);

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


        requestBatteryData(1);
        delayMicroseconds(100000);
        MtoCvar.battery_voltage = voltage;
        /*ltoa(voltage, CAN_UART_buffer);
        int i = 0;
        for (i=0; i<6; i++){
            UARTCharPut(UART0_BASE, CAN_UART_buffer[i]);
            CAN_UART_buffer[i] = 0;
        }
        UARTCharPut(UART0_BASE, '\n');
        ltoa(battery_current, CAN_UART_buffer);
        for (i=0; i<6; i++){
            UARTCharPut(UART0_BASE, CAN_UART_buffer[i]);
            CAN_UART_buffer[i] = 0;
        }
        UARTCharPut(UART0_BASE, '\n');
        UARTCharPut(UART0_BASE, '#');
        UARTCharPut(UART0_BASE, '#');
        UARTCharPut(UART0_BASE, '#');
        UARTCharPut(UART0_BASE, '\n');*/

        MtoCvar.HFactor = HFactor;
        /*ltoa(HFactor, CAN_UART_buffer);
        for (i=0; i<6; i++){
            UARTCharPut(UART0_BASE, CAN_UART_buffer[i]);
            CAN_UART_buffer[i] = 0;
        }
        UARTCharPut(UART0_BASE, '\n');
        ltoa(SOC, CAN_UART_buffer);
        for (i=0; i<6; i++){
            UARTCharPut(UART0_BASE, CAN_UART_buffer[i]);
            CAN_UART_buffer[i] = 0;
        }
        UARTCharPut(UART0_BASE, '\n');
        UARTCharPut(UART0_BASE, '*');
        UARTCharPut(UART0_BASE, '*');
        UARTCharPut(UART0_BASE, '*');
        UARTCharPut(UART0_BASE, '\n');*/
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
