///
/// \file LiquidCrystal_PCF8574.h
/// \brief LiquidCrystal library with PCF8574 I2C adapter.
///
//Chinthaka Jayarathne 2019/3/22



#ifndef LiquidCrystal_PCF8574_h
#define LiquidCrystal_PCF8574_h

#include <inttypes.h>

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00


void initializeLCD(uint8_t cols, uint8_t rows, uint8_t dotsize);
void clearLCD();
void homeLCD();
void noDisplayLCD();
void displayLCD();
void noBlinkLCD();
void blinkLCD();
void noCursorLCD();
void cursorLCD();
void scrollDisplayLeftLCD();
void scrollDisplayRightLCD();
void leftToRightLCD();
void rightToLeftLCD();
void autoscrollLCD();
void noAutoscrollLCD();

void setBacklightLCD(uint8_t brightness);
void setCursorLCD(uint8_t col, uint8_t row);
void printLCD(uint8_t col, uint8_t row, char *text);

//void createCharLCD(uint8_t location, uint8_t[]);
//virtual size_t write(uint8_t);
//using Print::write;

// low level functions
void _command(uint8_t);
void _send(uint8_t value, uint8_t mode);
void _sendNibble(uint8_t halfByte, uint8_t mode);
void _write2I2C(uint8_t halfByte, uint8_t mode, uint8_t enable);
void delayMicroseconds(long microseconds); // Implementation of microseconds delay

// NEW:
uint8_t _backlight;   ///< the backlight intensity

uint8_t _displayfunction; ///< lines and dots mode
uint8_t _displaycontrol;  ///< cursor, display, blink flags
uint8_t _displaymode;     ///< left2right, autoscroll

uint8_t _numlines;        ///< The number of rows the display supports.

#endif
