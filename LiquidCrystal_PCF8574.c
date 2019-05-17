//Chinthaka Jayarathne 2019/3/22

#include "LiquidCrystal_PCF8574.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "inc/hw_nvic.h"
#include "inc/hw_sysctl.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/flash.h"
#include "utils/uartstdio.h"
#include <string.h>
#include <stdio.h>
#include <inttypes.h>

/// Definitions on how the PCF8574 is connected to the LCD

/// These are Bit-Masks for the special signals and background light
#define PCF_RS  0x01
#define PCF_RW  0x02
#define PCF_EN  0x04
#define PCF_BACKLIGHT 0x08

// Definitions on how the PCF8574 is connected to the LCD
// These are Bit-Masks for the special signals and Background
#define RSMODE_CMD  0
#define RSMODE_DATA 1

#define true 1
#define false 0


// When the display powers up, it is configured as follows:
//
// 1. Display clear
// 2. Function set: 
//    DL = 1; 8-bit interface data 
//    N = 0; 1-line display 
//    F = 0; 5x8 dot character font 
// 3. Display on/off control: 
//    D = 0; Display off 
//    C = 0; Cursor off 
//    B = 0; Blinking off 
// 4. Entry mode set: 
//    I/D = 1; Increment by 1 
//    S = 0; No shift 



char initializeLCD(uint8_t cols, uint8_t lines, uint8_t dotsize) {
  // cols ignored !
  _numlines = lines;

  _displayfunction = 0;

  if (lines > 1) {
    _displayfunction |= LCD_2LINE;
  }

  // for some 1 line displays you can select a 10 pixel high font
  if ((dotsize != 0) && (lines == 1)) {
    _displayfunction |= LCD_5x10DOTS;
  }

  // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
  // according to datasheet, we need at least 40ms after power rises above 2.7V
  // before sending commands. Arduino can turn on way befor 4.5V so we'll wait 50
  char is_sucessful;

  // initializing th display
  is_sucessful=_write2I2C(0x00, 0, false);
  if(is_sucessful==0)
      return 0;
  delayMicroseconds(50000); 

  // put the LCD into 4 bit mode according to the hitachi HD44780 datasheet figure 26, pg 47
  is_sucessful=_sendNibble(0x03, RSMODE_CMD);
  if(is_sucessful==0)
      return 0;
  delayMicroseconds(4500); 
  is_sucessful=_sendNibble(0x03, RSMODE_CMD);
  if(is_sucessful==0)
      return 0;
  delayMicroseconds(4500); 
  is_sucessful=_sendNibble(0x03, RSMODE_CMD);
  if(is_sucessful==0)
      return 0;
  delayMicroseconds(150);
  // finally, set to 4-bit interface
  is_sucessful=_sendNibble(0x02, RSMODE_CMD);
  if(is_sucessful==0)
      return 0;
  // finally, set # lines, font size, etc.
  is_sucessful=_command(LCD_FUNCTIONSET | _displayfunction);
  if(is_sucessful==0)
      return 0;
  // turn the display on with no cursor or blinking default
  _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;  
  is_sucessful=displayLCD();
  if(is_sucessful==0)
      return 0;
  // clear it off
  is_sucessful=clearLCD();
  if(is_sucessful==0)
      return 0;
  // Initialize to default text direction (for romance languages)
  _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  // set the entry mode
  is_sucessful=_command(LCD_ENTRYMODESET | _displaymode);
  if(is_sucessful==0)
      return 0;
  return 1;
}

/********** high level commands, for the user! */
char clearLCD()
{
  char is_sucessful=_command(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
  if(is_sucessful==0)
      return 0;
  delayMicroseconds(2000);  // this command takes a long time!
  return 1;
}

char homeLCD()
{
  char is_sucessful=_command(LCD_RETURNHOME);  // set cursor position to zero
  if(is_sucessful==0)
      return 0;
  delayMicroseconds(2000);  // this command takes a long time!
  return 1;
}


/// Set the cursor to a new position. 
char setCursorLCD(uint8_t col, uint8_t row)
{
  int row_offsets[] = { 0x00, 0x40, 0x14, 0x54   };
  if ( row >= _numlines ) {
    row = _numlines-1;    // we count rows starting w/0
  }

  char is_sucessful=_command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
  if(is_sucessful==0)
      return 0;
  return 1;
}

// Turn the display on/off (quickly)
char noDisplayLCD() {
  _displaycontrol &= ~LCD_DISPLAYON;
  char is_sucessful=_command(LCD_DISPLAYCONTROL | _displaycontrol);
  if(is_sucessful==0)
      return 0;
  return 1;
}
char displayLCD() {
  _displaycontrol |= LCD_DISPLAYON;
  char is_sucessful=_command(LCD_DISPLAYCONTROL | _displaycontrol);
  if(is_sucessful==0)
      return 0;
  return 1;
}

// Turns the underline cursor on/off
char noCursorLCD() {
  _displaycontrol &= ~LCD_CURSORON;
  char is_sucessful=_command(LCD_DISPLAYCONTROL | _displaycontrol);
  if(is_sucessful==0)
      return 0;
  return 1;
}
char cursorLCD() {
  _displaycontrol |= LCD_CURSORON;
  char is_sucessful=_command(LCD_DISPLAYCONTROL | _displaycontrol);
  if(is_sucessful==0)
      return 0;
  return 1;
}

// Turn on and off the blinking cursor
char noBlinkLCD() {
  _displaycontrol &= ~LCD_BLINKON;
  char is_sucessful=_command(LCD_DISPLAYCONTROL | _displaycontrol);
  if(is_sucessful==0)
      return 0;
  return 1;
}
char blinkLCD() {
  _displaycontrol |= LCD_BLINKON;
  char is_sucessful=_command(LCD_DISPLAYCONTROL | _displaycontrol);
  if(is_sucessful==0)
      return 0;
  return 1;
}

// These commands scroll the display without changing the RAM
char scrollDisplayLeftLCD(void) {
  char is_sucessful=_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
  if(is_sucessful==0)
      return 0;
  return 1;
}
char scrollDisplayRightLCD(void) {
  char is_sucessful=_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
  if(is_sucessful==0)
      return 0;
  return 1;
}

// This is for text that flows Left to Right
char leftToRightLCD(void) {
  _displaymode |= LCD_ENTRYLEFT;
  char is_sucessful=_command(LCD_ENTRYMODESET | _displaymode);
  if(is_sucessful==0)
      return 0;
  return 1;
}

// This is for text that flows Right to Left
char rightToLeftLCD(void) {
  _displaymode &= ~LCD_ENTRYLEFT;
  char is_sucessful=_command(LCD_ENTRYMODESET | _displaymode);
  if(is_sucessful==0)
      return 0;
  return 1;
}

// This will 'right justify' text from the cursor
char autoscrollLCD(void) {
  _displaymode |= LCD_ENTRYSHIFTINCREMENT;
  char is_sucessful=_command(LCD_ENTRYMODESET | _displaymode);
  if(is_sucessful==0)
      return 0;
  return 1;
}

// This will 'left justify' text from the cursor
char noAutoscrollLCD(void) {
  _displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
  char is_sucessful=_command(LCD_ENTRYMODESET | _displaymode);
  if(is_sucessful==0)
      return 0;
  return 1;
}


/// Setting the brightness of the background display light.
/// The backlight can be switched on and off.
/// The current brightness is stored in the private _backlight variable to have it available for further data transfers.
char setBacklightLCD(uint8_t brightness) {
  _backlight = brightness;
  // send no data but set the background-pin right;
  char is_sucessful=_write2I2C(0x00, RSMODE_DATA, false);
  if(is_sucessful==0)
      return 0;
  return 1;
} // setBacklight

//Print a string on LCD starting from given location
char printLCD(uint8_t col, uint8_t row, char *text)
{
    int i;
    setCursorLCD(col,row);
    size_t n = strlen(text);
    char is_sucessful;
    for(i=0;i<n;i++)
    {
        is_sucessful=_send(text[i],RSMODE_DATA);
        if(is_sucessful==0)
            return 0;
    }
    return 1;
}
//// Allows us to fill the first 8 CGRAM locations
//// with custom characters
//void createCharLCD(uint8_t location, uint8_t charmap[]) {
//  location &= 0x7; // we only have 8 locations 0-7
//  _command(LCD_SETCGRAMADDR | (location << 3));
//  for (int i=0; i<8; i++) {
//    write(charmap[i]);
//  }
//}
//
//
//inline size_t write(uint8_t value) {
//  _send(value, RSMODE_DATA);
////  return 1; // assume success
//}

/* ----- low level functions ----- */

inline char _command(uint8_t value) {
  char is_sucessful=_send(value, RSMODE_CMD);
  return is_sucessful;
} // _command()


// write either command or data
char _send(uint8_t value, uint8_t mode) {
  // separate the 4 value-nibbles
  uint8_t valueLo = value    & 0x0F;
  uint8_t valueHi = value>>4 & 0x0F;

  char is_sucessful1=_sendNibble(valueHi, mode);
  char is_sucessful2=_sendNibble(valueLo, mode);
  if(is_sucessful1 && is_sucessful2)
      return 1;
  else
      return 0;

} // _send()


// write a nibble / halfByte with handshake
char _sendNibble(uint8_t halfByte, uint8_t mode) {
  char is_sucessful1=_write2I2C(halfByte, mode, true);
  delayMicroseconds(1);    // enable pulse must be >450ns
  char is_sucessful2=_write2I2C(halfByte, mode, false);
  delayMicroseconds(40);
  if(is_sucessful1 && is_sucessful2)
      return 1;
  else
      return 0;
}


// private function to change the PCF8674 pins to the given value
char _write2I2C(uint8_t halfByte, uint8_t mode, uint8_t enable) {
  // map the given values to the hardware of the I2C schema
  unsigned int failCount=0;
  uint8_t i2cData = halfByte << 4;
  if (mode > 0) i2cData |= PCF_RS;
  // PCF_RW is never used.
  if (enable > 0) i2cData |= PCF_EN;
  if (_backlight > 0) i2cData |= PCF_BACKLIGHT;

  I2CMasterDataPut(I2C0_MASTER_BASE, i2cData);
  I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_SINGLE_SEND);
  while(I2CMasterBusy(I2C0_MASTER_BASE))
  {
      failCount++;
      if(failCount>1000)
          return 0;
  }
  return 1;
} // write2Wire

// Micro seconds delay implementaton
void delayMicroseconds(long microseconds)
{
    if (microseconds>0)
    {
        long const tick= 25*microseconds;
        SysCtlDelay(tick);
    }

}

// The End.
