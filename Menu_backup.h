/*
 * Menu.h
 *
 *  Created on: May 3, 2019
 *      Author: Chinthaka
 */

#ifndef MENU_BACKUP_H_
#define MENU_BACKUP_H_

#include "LiquidCrystal_PCF8574.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

//flags for switch press detection
char buffer[20];
char tempbuffer[20];
volatile int Next_pressed=0,Prev_pressed=0,INCRE_pressed=0,DECRE_pressed=0;



typedef enum {
    None,
    Configuration_Settings_edit1,
    Configuration_Settings_edit2,
    Configuration_Settings_edit3,
    Voltage_Settings1_edit1,
    Voltage_Settings1_edit2,
    Voltage_Settings1_edit3,
    Voltage_Settings1_edit4,
    Voltage_Settings2_edit1,
    Voltage_Settings2_edit2,
    Voltage_Settings2_edit3,
    Voltage_Settings2_edit4,
    Current_Settings1_edit1,
    Current_Settings1_edit2,
    Current_Settings1_edit3
}task;

typedef enum {
    id_HOME,
    id_Configuration_Settings,
    id_Configuration_Settings_select1,
    id_Configuration_Settings_select2,
    id_Configuration_Settings_select3,
    id_Network_Settings,
    id_Voltage_Settings1,
    id_Voltage_Settings1_select1,
    id_Voltage_Settings1_select2,
    id_Voltage_Settings1_select3,
    id_Voltage_Settings1_select4,
    id_Voltage_Settings2,
    id_Voltage_Settings2_select1,
    id_Voltage_Settings2_select2,
    id_Voltage_Settings2_select3,
    id_Voltage_Settings2_select4,
    id_Current_Settings1,
    id_Current_Settings1_select1,
    id_Current_Settings1_select2,
    id_Current_Settings1_select3,
    id_DelaySettings
} menu_id;

struct menu {
   menu_id ID;
   struct menu *next;
   struct menu *prev;
   struct menu *down;
   struct menu *up;
   task DoTask; //Changed function pointer to hold enum value
} HOME,
Configuration_Settings,
Configuration_Settings_select1,
Configuration_Settings_select2,
Configuration_Settings_select3,
Network_Settings,
Voltage_Settings1,
Voltage_Settings1_select1,
Voltage_Settings1_select2,
Voltage_Settings1_select3,
Voltage_Settings1_select4,
Voltage_Settings2,
Voltage_Settings2_select1,
Voltage_Settings2_select2,
Voltage_Settings2_select3,
Voltage_Settings2_select4,
Current_Settings1,
Current_Settings1_select1,
Current_Settings1_select2,
Current_Settings1_select3,
DelaySettings,
*currentMenu;

void BuildMenu(struct menu *currentMenu, menu_id MenuID, task DoTask, struct menu *prevNode, struct menu *nextNode,struct menu *upNode,struct menu *downNode)
{
    currentMenu->ID = MenuID;
    currentMenu->prev = prevNode;
    currentMenu->next = nextNode;
    currentMenu->up = upNode;
    currentMenu->down = downNode;
    currentMenu->DoTask = DoTask;
}

void RenderIndex(menu_id ID){
    switch(ID){
    case id_HOME:
        clearLCD();
        printLCD(0,1,"MODE:");
        printLCD(0,2,"Vdc    Vgrid  Vout");
        break;
    case id_Configuration_Settings:
        clearLCD();
        printLCD(1,0,"Solar flag: ");
        printLCD(1,1,"Peak flag: ");
        printLCD(1,2,"OP Power:        kW");
        break;
    case id_Network_Settings:
        clearLCD();
        printLCD(1,0,"MQTT server: ");
        printLCD(1,1,"WiFi SSID: ");
        printLCD(1,2,"WiFi Pwd: ");
        break;
    case id_Voltage_Settings1:
        clearLCD();
        printLCD(1,0,"GRD OVR V H:      V");
        printLCD(1,1,"GRD OVR V M:      V");
        printLCD(1,2,"GRD OVR V L:      V");
        printLCD(1,3,"GRD UNDR V:       V");
        break;
    case id_Voltage_Settings2:
        clearLCD();
        printLCD(1,0,"DC UPS ST V:      V");
        printLCD(1,1,"DC UNDR V:        V");
        printLCD(1,2,"DC CHRG V H:      V");
        printLCD(1,3,"DC CHRG V L:      V");
        break;
    case id_Current_Settings1:
        clearLCD();
        printLCD(1,0,"I CHRG MAX:       A");
        printLCD(1,1,"I CHRG MIN:       A");
        printLCD(1,2,"I OP LIMIT:       A");
//        printLCD(1, 3, "Id: ");
        break;
    case id_DelaySettings:
        clearLCD();
        printLCD(1,0,"Delay a: ");
        printLCD(1,1,"Delay b: ");
        printLCD(1,2,"Delay c: ");
        printLCD(1, 3, "Delay d: ");
        break;
    }
}

int ltoa_zp(int value,char *buf,unsigned int string_length){
    char tempbuffer[20];
    int j;

    ltoa(value,tempbuffer);
    int no_of_zeros= string_length-strlen(tempbuffer);
    for(j=0;j<no_of_zeros;j++){
        buf[j]='0';
    }
    strcpy(buf+no_of_zeros,tempbuffer);
    return 1;
}

int ftoa(float value, char *buf, unsigned char decimalPoints , unsigned char integer_length)
{
    int ipart = (int)value;
    float fpart=value-(float)ipart;
    ltoa_zp(ipart,buf,integer_length);
    unsigned char i=strlen(buf);

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


unsigned int update_int(unsigned int lower_limit, unsigned int upper_limit, unsigned int disp_col, unsigned int disp_row, unsigned int disp_length, unsigned int value ){
    setCursorLCD(disp_col+disp_length-1, disp_row);
    blinkLCD();
    while(1){
        if(INCRE_pressed){
            INCRE_pressed=0;
            if(value<upper_limit){
                value+=1;
                ltoa_zp(value,buffer,1);
                printLCD(disp_col, disp_row, buffer);
                setCursorLCD(disp_col+disp_length-1, disp_row);
            }
        }
        if(DECRE_pressed){
            DECRE_pressed=0;
            if(value>lower_limit){
                value-=1;
                ltoa_zp(value,buffer,1);
                printLCD(disp_col, disp_row, buffer);
                setCursorLCD(disp_col+disp_length-1, disp_row);
            }
        }
        if(Next_pressed){
            Next_pressed=0;
            noBlinkLCD();
            return value;

        }
    }
}

//Update floats
float update_float(float value, float lower_limit, float upper_limit, unsigned char disp_col, unsigned char disp_row,unsigned char ipart_length, float step){
    unsigned char cursor_column;
    float init_value=value;
    if(step<1){
        cursor_column=disp_col+ipart_length+1;
    }
    else {
        cursor_column=disp_col+ipart_length-1;
    }
    setCursorLCD(cursor_column, disp_row);
    blinkLCD();
    while(1){
        if(INCRE_pressed){
            INCRE_pressed=0;
            if(value<upper_limit){
                value+=step;
                ftoa(value,buffer,1,ipart_length);
                printLCD(disp_col, disp_row, buffer);
                setCursorLCD(cursor_column, disp_row);
            }
        }
        if(DECRE_pressed){
            DECRE_pressed=0;
            if(value>lower_limit){
                value-=step;
                ftoa(value,buffer,1,ipart_length);
                printLCD(disp_col, disp_row, buffer);
                setCursorLCD(cursor_column, disp_row);
            }
        }
        if(Next_pressed){
            Next_pressed=0;
            noBlinkLCD();
            return value;
        }
        if(Prev_pressed){
            Prev_pressed=0;
            ftoa(init_value,buffer,1,ipart_length);
            printLCD(disp_col, disp_row, buffer);
            noBlinkLCD();
            return init_value;
        }

    }
}
#endif /* MENU_BACKUP_H_ */
