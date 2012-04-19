/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#include <plib.h>            /* Include to use PIC32 peripheral libraries     */
#include <sys/attribs.h>     /* For __ISR definition                          */
#include <stdint.h>          /* For uint32_t definition                       */
#include <stdbool.h>         /* For true/false definition                     */
#include "interrupts.h"
#include "user.h"

#define T2_TICK             40 //gives an interrupt every 1us
#define ACCEL_PRI           0x01
#define GYRO_PRI            0x02
#define MAG_PRI             0x03
#define DIS_PRI             0x04
#define CAM_PRI             0x05


/******************************************************************************/
/* Interrupt Vector Options                                                   */
/******************************************************************************/
/*                                                                            */
/* VECTOR NAMES:                                                              */
/*                                                                            */
/* _CORE_TIMER_VECTOR          _COMPARATOR_2_VECTOR                           */
/* _CORE_SOFTWARE_0_VECTOR     _UART_2A_VECTOR                                */
/* _CORE_SOFTWARE_1_VECTOR     _I2C_2A_VECTOR                                 */
/* _EXTERNAL_0_VECTOR          _SPI_2_VECTOR                                  */
/* _TIMER_1_VECTOR             _SPI_2A_VECTOR                                 */
/* _INPUT_CAPTURE_1_VECTOR     _I2C_4_VECTOR                                  */
/* _OUTPUT_COMPARE_1_VECTOR    _UART_3_VECTOR                                 */
/* _EXTERNAL_1_VECTOR          _UART_2_VECTOR                                 */
/* _TIMER_2_VECTOR             _SPI_3A_VECTOR                                 */
/* _INPUT_CAPTURE_2_VECTOR     _I2C_3A_VECTOR                                 */
/* _OUTPUT_COMPARE_2_VECTOR    _UART_3A_VECTOR                                */
/* _EXTERNAL_2_VECTOR          _SPI_4_VECTOR                                  */
/* _TIMER_3_VECTOR             _I2C_5_VECTOR                                  */
/* _INPUT_CAPTURE_3_VECTOR     _I2C_2_VECTOR                                  */
/* _OUTPUT_COMPARE_3_VECTOR    _FAIL_SAFE_MONITOR_VECTOR                      */
/* _EXTERNAL_3_VECTOR          _RTCC_VECTOR                                   */
/* _TIMER_4_VECTOR             _DMA_0_VECTOR                                  */
/* _INPUT_CAPTURE_4_VECTOR     _DMA_1_VECTOR                                  */
/* _OUTPUT_COMPARE_4_VECTOR    _DMA_2_VECTOR                                  */
/* _EXTERNAL_4_VECTOR          _DMA_3_VECTOR                                  */
/* _TIMER_5_VECTOR             _DMA_4_VECTOR                                  */
/* _INPUT_CAPTURE_5_VECTOR     _DMA_5_VECTOR                                  */
/* _OUTPUT_COMPARE_5_VECTOR    _DMA_6_VECTOR                                  */
/* _SPI_1_VECTOR               _DMA_7_VECTOR                                  */
/* _I2C_3_VECTOR               _FCE_VECTOR                                    */
/* _UART_1A_VECTOR             _USB_1_VECTOR                                  */
/* _UART_1_VECTOR              _CAN_1_VECTOR                                  */
/* _SPI_1A_VECTOR              _CAN_2_VECTOR                                  */
/* _I2C_1A_VECTOR              _ETH_VECTOR                                    */
/* _SPI_3_VECTOR               _UART_4_VECTOR                                 */
/* _I2C_1_VECTOR               _UART_1B_VECTOR                                */
/* _CHANGE_NOTICE_VECTOR       _UART_6_VECTOR                                 */
/* _ADC_VECTOR                 _UART_2B_VECTOR                                */
/* _PMP_VECTOR                 _UART_5_VECTOR                                 */
/* _COMPARATOR_1_VECTOR        _UART_3B_VECTOR                                */
/*                                                                            */
/* Refer to the device specific .h file in the C32 Compiler                   */
/* pic32mx\include\proc directory for a complete Vector and IRQ mnemonic      */
/* listings for the PIC32 device.                                             */
/*                                                                            */
/* PRIORITY OPTIONS:                                                          */
/*                                                                            */
/* (default) IPL0AUTO, IPL1, IPL2, ... IPL7 (highest)                         */
/*                                                                            */
/* Example Shorthand Syntax                                                   */
/*                                                                            */
/* void __ISR(<Vector Name>,<PRIORITY>) user_interrupt_routine_name(void)     */
/* {                                                                          */
/*     <Clear Interrupt Flag>                                                 */
/* }                                                                          */
/*                                                                            */
/* For more interrupt macro examples refer to the C compiler User Guide in    */
/* the C compiler /doc directory.                                             */
/*                                                                            */
/******************************************************************************/
/* Interrupt Routines                                                         */
/******************************************************************************/

void __ISR(_SPI_3_VECTOR, IPL2SOFT) SPI3Handler(void){
    
    INTClearFlag(INT_SPI3RX);

    if(count = 0)
        x_pix = SPI3BUF;
    else if(count = 1){
        y_pix = SPI3BUF;
        count = 0;
        Insertion_Sort(CAM_PRI);
    }
}


void __ISR(_TIMER_1_VECTOR, IPL2SOFT) Timer1Handler(void)
{

    // clear the interrupt flag
    mT1ClearIntFlag();
    // Set the timer1 flag
    timer1Flag = 1;

    /* USB TEST STUFF
    // clear the interrupt flag
    mT1ClearIntFlag();
	//Toggle the LED	mPORTDToggleBits(BIT_7);


	//Toggle LED on Flag
	if (LEDOnFlag == 0)
		LEDOnFlag = 1;
	else
		LEDOnFlag = 0;*/
}

void __ISR(_TIMER_2_VECTOR, IPL2SOFT) Timer2Handler(void)
{
 char charArray[6];
    // clear the interrupt flag
    mT2ClearIntFlag();
    // increment tcount
    tcount++;
    	int offset = 8 * ((~ReadActiveBufferADC10() & 0x01));
	an15Data = ReadADC10(offset);
        //convIntToString(an15Data, charArray);
        //bufferSpaces(charArray);
        //WriteString(charArray);
        if(tcount >= 90000){
            INTEnable(INT_T2, INT_DISABLED);
            //mAD1IntEnable(INT_DISABLED);
            Insertion_Sort(DIS_PRI);
        }
        if(an15Data >= 530){
            INTEnable(INT_T2, INT_DISABLED);
            //mAD1IntEnable(INT_DISABLED);
            Insertion_Sort(DIS_PRI);
        }

}

void __ISR(_CHANGE_NOTICE_VECTOR, IPL2SOFT) ChangeNotice_Handler(void)
{
    unsigned int temp;

    // clear the mismatch condition
    temp = mPORTDRead();

    // clear the interrupt flag
    mCNClearIntFlag();

    if (temp == 0x0B77)
    {
        // .. things to do .. toggle the button flag
    calibration = 1;
    //mPORTAToggleBits(BIT_0);
    }
}

// ADC10 interrupt handler

void __ISR(_ADC_VECTOR, IPL2SOFT) IntAdc10Handler(void)
{
   
        mAD1ClearIntFlag();
	// determine which buffer is idle and create an offset
	int offset = 8 * ((~ReadActiveBufferADC10() & 0x01));
	an15Data = ReadADC10(offset);

        if(tcount >= 11754){
            ConfigIntTimer2(T2_INT_OFF | T1_INT_PRIOR_2);
            //mAD1IntEnable(INT_DISABLED);
            Insertion_Sort(DIS_PRI);
        }
        if(an15Data >= 600){
            ConfigIntTimer2(T2_INT_OFF | T1_INT_PRIOR_2);
            //mAD1IntEnable(INT_DISABLED);
            Insertion_Sort(DIS_PRI);
        }
}

// UART 2 interrupt handler
// it is set at priority level 2 with software context saving
void __ISR(_UART2_VECTOR, IPL2SOFT) IntUart2Handler(void)
{
    // Is this an RX interrupt?
    if (INTGetFlag(INT_SOURCE_UART_RX(UART2)))
    {
        char temp = UARTGetDataByte(UART2);
        // Echo what we just received.
        PutCharacter(temp);

        // Clear the RX interrupt Flag
        INTClearFlag(INT_SOURCE_UART_RX(UART2));

        // Toggle LED to indicate UART activity
        //mPORTAToggleBits(BIT_7);
    }

    // Transmit complete send next byte in buffer
    if (INTGetFlag(INT_SOURCE_UART_TX(UART2)))
    {
        //Clear the TX interrupt Flag
        INTClearFlag(INT_SOURCE_UART_TX(UART2));

        //if buffer empty exit
        if (TBufferHead == TBufferTail) {
			INTEnable(INT_SOURCE_UART_TX(UART2), INT_DISABLED);
            return;
		}
        //else transmit next character
        else
        {
            PutCharacter(rs232TBuffer[TBufferHead]);
            TBufferHead = (TBufferHead + 1)% TBufferSize;
        }

    }
}
void __ISR(_UART_3_VECTOR, IPL2SOFT) IntUart3Handler(void)
{
    // Is this an RX interrupt?
    if (INTGetFlag(INT_SOURCE_UART_RX(UART3)))
    {
        int temp = UARTGetDataByte(UART3);
        // Echo what we just received.
        //PutCharacter(temp);
        int test = temp;
        if(test == 0x35){
                //Configure the T2 timer which counts the number of us till the ultrasonics appears on the ATD to 1 us
                //OpenTimer2(T2_ON | T2_PS_1_4, 1); //one second interrupts
                INTEnable(INT_T2, INT_ENABLED);
                //mAD1IntEnable(INT_ENABLED);
        }

        // Clear the RX interrupt Flag
        INTClearFlag(INT_SOURCE_UART_RX(UART3));

        // Toggle LED to indicate UART activity
        //mPORTAToggleBits(BIT_7);
    }

}
void __ISR(_EXTERNAL_1_VECTOR, IPL2SOFT) ExtINT1Handler(void)
{
    // clear the interrupt flag
    INTClearFlag(INT_VECTOR_EX_INT(INT_INT1));
	// disables the interrupt for now
	DisableINT1;

    // set the read sensor flag
    Insertion_Sort(ACCEL_PRI);
    readAccelFlag = 1;
}

void __ISR(_EXTERNAL_0_VECTOR, IPL2SOFT) ExtINT0Handler(void)
{
    // clear the interrupt flag
    INTClearFlag(INT_VECTOR_EX_INT(INT_INT0));
	// disables the interrupt for now
	DisableINT0;

    // set the read sensor flag
    readMagFlag = 1;
    Insertion_Sort(MAG_PRI);
}

void __ISR(_EXTERNAL_4_VECTOR, IPL2SOFT) ExtINT4Handler(void)
{
    // clear the interrupt flag
    INTClearFlag(INT_VECTOR_EX_INT(INT_INT4));
	// disables the interrupt for now
	DisableINT4;

    // set the read sensor flag
    readGyroFlag = 1;
    Insertion_Sort(GYRO_PRI);
}

void PutCharacter(const char character)
{
  while (!UARTTransmitterIsReady(UART2));

  UARTSendDataByte(UART2, character);

  //while (!UARTTransmissionHasCompleted(UART2));
}
