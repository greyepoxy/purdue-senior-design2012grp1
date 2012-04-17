/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file */
#if defined(__PIC24E__)
	#include <p24Exxxx.h>
#elif defined(__PIC24F__)
	#include <p24Fxxxx.h>
#elif defined(__PIC24H__)
	#include <p24Hxxxx.h>
#endif

#include <stdint.h>        /* Includes uint16_t definition                    */
#include <stdbool.h>       /* Includes true/false definition                  */

#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "user.h"          /* User funct/params, such as InitApp              */

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

/* i.e. uint16_t <variable_name>; */
int ten_ms_flag = 0;
int sent_flag = 0;

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

int16_t main(void)
{
    int PWM_ON = '0';
    int Sent_Byte = '0';

    /* Configure the oscillator for the device */
    ConfigureOscillator();

    /* Initialize IO ports and peripherals */
    InitApp();

    /* TODO <INSERT USER APPLICATION CODE HERE> */

    while(1)
    {

        if(ten_ms_flag == '11' && Sent_Byte == '0'){
            Sent_Byte = '1';
            while(BusyUART2());
            putsUART2("50");
            while(BusyUART2());
            //OC1CON = 0x0005;
        }

        if (ten_ms_flag == '20' && PWM_ON == '0' ){
            PWM_ON = '1';
            OC1CON = 0x0005;
        }

        if(ten_ms_flag == '25'){
            ten_ms_flag=0;
            PWM_ON = '0';
            Sent_Byte = '0';
            OC1CON = 0x0000;
        }

    }
}
