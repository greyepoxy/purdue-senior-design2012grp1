/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#include <plib.h>            /* Include to use PIC32 peripheral libraries     */

/******************************************************************************/
/* Configuration Bits                                                         */
/*                                                                            */
/* Refer to 'C32 Configuration Settings' under the Help > Contents            */
/* > C32 Toolchain in MPLAB X IDE for available PIC32 Configurations.  For    */
/* additional information about what the hardware configurations mean in      */
/* terms of device operation, refer to the device datasheet 'Special Features'*/
/* chapter.                                                                   */
/*                                                                            */
/******************************************************************************/

/* Fill in your configuration bits here.  The general style is shown below.
The Debug Configuration bit is handline by MPLAB and should not be embedded
in the configuration macro.*/

/* TODO Fill in your configuration bits here.  The general style is below:    */

#if 1

// Config settings
// POSCMOD = HS, FNOSC = PRIPLL, FWDTEN = OFF
// PLLIDIV = DIV_2, PLLMUL = MUL_16
// PBDIV = 8 (default)
// Main clock = 8MHz /2 * 16    = 80MHz
// Peripheral clock = 80MHz /8  =  10MHz

// Configuration Bit settings
// SYSCLK = 80 MHz (8MHz Crystal/ FPLLIDIV * FPLLMUL / FPLLODIV)
// PBCLK = 10 MHz
// Primary Osc w/PLL (XT+,HS+,EC+PLL)
// WDT OFF
// FCSM (fail-safe clock moniter) disabled
// clock switching disabled
// Other options are don't care

#pragma config UPLLEN   = ON			// USB PLL Enabled
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
#pragma config FPLLMUL  = MUL_20        // PLL Multiplier
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_2         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config FUSBIDIO = OFF			// USB USID selection
#pragma config FVBUSONIO = OFF			// USB VBUS selection

// Fine with the defaults on all of theses
//#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
//#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
//#pragma config OSCIOFNC = OFF           // CLKO Enable
//#pragma config IESO     = OFF           // Internal/External Switch-over
//#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
//#pragma config CP       = OFF           // Code Protect
//#pragma config BWP      = OFF           // Boot Flash Write Protect
//#pragma config PWP      = OFF           // Program Flash Write Protect
//#pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select

#endif
