/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#include <plib.h>            /* Include to use PIC32 peripheral libraries     */
#include <sys/attribs.h>     /* For __ISR definition                          */
#include <stdint.h>          /* For uint32_t definition                       */
#include <stdbool.h>         /* For true/false definition                     */
#include "interrupts.h"      /* For flags*/

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

/* TODO Add interrupt routine code here. */
// configure the CN interrupt handler
void __ISR(_CHANGE_NOTICE_VECTOR, IPL2SOFT) ChangeNotice_Handler(void)
{
    unsigned int temp;

    // clear the mismatch condition
    temp = mPORTDRead();

    // clear the interrupt flag
    mCNClearIntFlag();

    //if (temp == 0xFFFE)
    //{
        // .. things to do .. toggle the button flag
    buttonFlag = 1;
    //mPORTAToggleBits(BIT_0);
    //}
}

// ADC10 interrupt handler
void __ISR(_ADC_VECTOR, IPL2SOFT) IntAdc10Handler(void)
{
	mAD1ClearIntFlag();
	// determine which buffer is idle and create an offset
	int offset = 8 * ((~ReadActiveBufferADC10() & 0x01));
	an15Data = ReadADC10(offset);
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

void __ISR(_TIMER_1_VECTOR, IPL2SOFT) Timer1Handler(void)
{
    // clear the interrupt flag
    mT1ClearIntFlag();
    // Set the timer1 flag
    timer1Flag = 1;
}

void __ISR(_EXTERNAL_1_VECTOR, IPL2SOFT) ExtINT1Handler(void)
{
    // clear the interrupt flag
    INTClearFlag(INT_VECTOR_EX_INT(INT_INT1));
	// disables the interrupt for now
	DisableINT1;

    // set the read sensor flag
    readAccelFlag = 1;
}

void __ISR(_EXTERNAL_2_VECTOR, IPL2SOFT) ExtINT2Handler(void)
{
    // clear the interrupt flag
    INTClearFlag(INT_VECTOR_EX_INT(INT_INT2));
	// disables the interrupt for now
	DisableINT2;

    // set the read sensor flag
    readMagFlag = 1;
}

void __ISR(_EXTERNAL_3_VECTOR, IPL2SOFT) ExtINT3Handler(void)
{
    // clear the interrupt flag
    INTClearFlag(INT_VECTOR_EX_INT(INT_INT3));
	// disables the interrupt for now
	DisableINT3;

    // set the read sensor flag
    readGyroFlag = 1;
}

void PutCharacter(const char character)
{
  while (!UARTTransmitterIsReady(UART2));

  UARTSendDataByte(UART2, character);

  //while (!UARTTransmissionHasCompleted(UART2));
}

