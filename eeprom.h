/*
 * eeprom.h
 *
 *  Created on: Jul 21, 2019
 *      Author: Chinthaka
 */

#ifndef EEPROM_H_
#define EEPROM_H_
#include "driverlib/i2c.h"
#include "delay.h"

//Write a Byte to a given address, returns 0 if successful and error code if not
unsigned long EEPROMByteWrite(unsigned long ulBase,unsigned char ucSlaveAddr,unsigned short eepromAddr,unsigned char data){
    unsigned long status = 0xFFFF;
    unsigned char addrLSB=(unsigned char)(eepromAddr & 0xFF);
    unsigned char addrMSB=(unsigned char)(eepromAddr >> 8);

    I2CMasterSlaveAddrSet(ulBase, ucSlaveAddr, false);
    I2CMasterDataPut(ulBase,addrMSB);
    I2CMasterControl(ulBase,I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(ulBase)){}
    I2CMasterDataPut(ulBase, addrLSB);
    I2CMasterControl(ulBase,I2C_MASTER_CMD_BURST_SEND_CONT);
    while(I2CMasterBusy(ulBase)){}
    I2CMasterDataPut(ulBase, data);
    I2CMasterControl(ulBase,I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusy(ulBase)){}
    delayMicroseconds(5000);

    status = I2CMasterErr(ulBase);
    if(status != 0){
        I2CMasterControl(ulBase,I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
    }
    return status;
}

//Read the byte in the immediate address, return data if  successfull or return eror code which is minus
long EEPROMImmediateAddrByteRead(unsigned long ulBase,unsigned char ucSlaveAddr){
    long status = 0xFFFF;
    unsigned long data;
    I2CMasterSlaveAddrSet(ulBase, ucSlaveAddr, true);
    I2CMasterControl(ulBase,I2C_MASTER_CMD_SINGLE_RECEIVE);
    while(I2CMasterBusy(ulBase)){}
    status = I2CMasterErr(ulBase);
    if(status != 0){
        return (status*-1);
    }
    else{
        data= I2CMasterDataGet(ulBase);
        return data;
    }

}

//Read the byte in the given address, return data if  successful or return eror code which is minus
long EEPROMByteRead(unsigned long ulBase,unsigned char ucSlaveAddr,unsigned short eepromAddr){
    unsigned char addrLSB=(unsigned char)(eepromAddr & 0xFF);
    unsigned char addrMSB=(unsigned char)(eepromAddr >> 8);

    I2CMasterSlaveAddrSet(ulBase, ucSlaveAddr, false);
    I2CMasterDataPut(ulBase, addrMSB);
    I2CMasterControl(ulBase,I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(ulBase)){}
    I2CMasterDataPut(ulBase, addrLSB);
    I2CMasterControl(ulBase,I2C_MASTER_CMD_BURST_SEND_CONT);
    while(I2CMasterBusy(ulBase)){}
    return EEPROMImmediateAddrByteRead(ulBase,ucSlaveAddr);
}




#endif /* EEPROM_H_ */
