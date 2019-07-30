/*
 * delay.h
 *
 *  Created on: Jul 21, 2019
 *      Author: Chinthaka
 */

#ifndef DELAY_H_
#define DELAY_H_

#include "driverlib/sysctl.h"

// Micro seconds delay implementaton
void delayMicroseconds(long microseconds)
{
    if (microseconds>0)
    {
        long const tick= 25*microseconds;
        SysCtlDelay(tick);
    }

}



#endif /* DELAY_H_ */
