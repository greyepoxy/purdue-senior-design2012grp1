/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file */
#if defined(__PIC24E__)
	#include <p24Exxxx.h>
#elif defined (__PIC24F__)
	#include <p24Fxxxx.h>
#elif defined(__PIC24H__)
	#include <p24Hxxxx.h>
#endif

#include <stdint.h>          /* For uint32_t definition */
#include <stdbool.h>         /* For true/false definition */

#include "user.h"            /* variables/params used by user.c */
#include "PPS.h"             /* Peripheral Port Select functions*/
#include "timer.h"          /*  Timers functions*/
#include "UART.h"
#include "outcompare.h"
#include "wdt.h"

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

/* <Initialize variables in user.h and insert code for user algorithms.> */

/* TODO Initialize User Ports/Peripherals/Project here */

void InitApp(void)
{
    EnableWDT(WDT_DISABLE);

    /* Setup analog functionality and port direction */
    iPPSInput(IN_FN_PPS_U2RX,IN_PIN_PPS_RP12);        //Assing U2RX to pin RP12
    iPPSInput(IN_FN_PPS_U2CTS,IN_PIN_PPS_RP7);    //Assing U2CTS to pin RP7
    iPPSOutput(OUT_PIN_PPS_RP13,OUT_FN_PPS_U2TX);    //Assing U2TX to pin RP13
    iPPSOutput(OUT_PIN_PPS_RP6,OUT_FN_PPS_U2RTS);    //Assing U2RTS to pin RP6
    iPPSOutput(OUT_PIN_PPS_RP2,OUT_FN_PPS_OC1);    //Assing PWM1 to pin RP2
    iPPSOutput(OUT_PIN_PPS_RP3,OUT_FN_PPS_OC2);    //Assing PWM2 to pin RP3
    CloseUART2();

    /* Initialize peripherals */

    //Turning on the Timer and setting it to interupt ever 10 ms

    T1CON = 0x00; //Stops the Timer1 and reset control reg.
    TMR1 = 0x00; //Clear contents of the timer register
    PR1 = 0x00A0; //Load the Period register with the value 0xFFFF
    IPC0bits.T1IP = 0x04; //Setup Timer1 interrupt for desired priority level
    // (This example assigns level 1 priority)
    IFS0bits.T1IF = 0; //Clear the Timer1 interrupt status flag
    IEC0bits.T1IE = 1; //Enable Timer1 interrupts
    T1CONbits.TON = 1; //Start Timer1 with prescaler settings at 1:1 and
    //clock source set to the internal instruction cycl
    T2CON = 0x00; //Stops the Timer1 and reset control reg.
    TMR2 = 0x00; //Clear contents of the timer register
    PR2 = 0x63; //Load the Period register with the value 0xFFFF
    //IPC1bits.T2IP = 0x01; //Setup Timer1 interrupt for desired priority level
    // (This example assigns level 1 priority)
    //IFS0bits.T2IF = 0; //Clear the Timer1 interrupt status flag
    //IEC0bits.T2IE = 1; //Enable Timer1 interrupts


    //Enabling the UART and setting its BAUD rate to 9600

    U2BRG=25; //Set Baudrate
    //IPC3bits.U1TXIP2 = 1; //Set Uart TX Interrupt Priority
    //IPC3bits.U1TXIP1 = 0;
    //IPC3bits.U1TXIP0 = 0;
    //IPC2bits.U1RXIP2 = 1; //Set Uart RX Interrupt Priority
    //IPC2bits.U1RXIP1 = 0;
    //IPC2bits.U1RXIP0 = 0;
    U2STA = 0;
    U2MODE = 0x8000; //Enable Uart for 8-bit data
    //no parity, 1 STOP bit
    U2STAbits.UTXEN = 1; //Enable Transmit
    //IEC0bits.U1TXIE = 1; //Enable Transmit Interrupt
    IEC0bits.U1RXIE = 1; //Enable Receive Interrup

    OC1CON = 0x0000; // Turn off Output Compare 1 Module
    OC1CON = 0x0005; // Load new compare mode to OC1CON
    OC1R = 0x0033; // Initialize Compare Register1 with 0x3000
    OC1RS = 0x0063; // Initialize Secondary Compare Register1 with 0x3003
    //IPC0bits.OC1IP0 = 1; // Setup Output Compare 1 interrupt for
    //IPC0bits.OC1IP1 = 0x01; // desired priority level
    //IPC0bits.OC1IP2 = 0; // (this example assigns level 1 priority)
    //IFS0bits.OC1IF = 0; // Clear Output Compare 1 interrupt flag
    //IEC0bits.OC1IE = 1; // Enable Output Compare 1 interrupts
    OC1CON = 0x0000;

    OC2CON = 0x0000; // Turn off Output Compare 1 Module
    OC2CON = 0x0005; // Load new compare mode to OC1CON
    OC2R = 0x0010; // Initialize Compare Register1 with 0x3000
    OC2RS = 0x0063; // Initialize Secondary Compare Register1 with 0x3003
    //IPC1bits.OC2IP0 = 1; // Setup Output Compare 1 interrupt for
    //IPC1bits.OC2IP1 = 0x01; // desired priority level
    //IPC1bits.OC2IP2 = 0; // (this example assigns level 1 priority)
    //IFS0bits.OC2IF = 0; // Clear Output Compare 1 interrupt flag
    //IEC0bits.OC2IE = 1; // Enable Output Compare 1 interrupts
    T2CONbits.TON = 1; // Start Timer2 with assumed setting
}

