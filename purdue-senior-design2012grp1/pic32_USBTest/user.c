/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#include <plib.h>            /* Include to use PIC32 peripheral libraries     */
#include <stdint.h>          /* For uint32_t definition                       */
#include <stdbool.h>         /* For true/false definition*/
#include <math.h>
#include "system.h"
#include "user.h"            /* variables/params used by user.c               */
#include "interrupts.h"
#include "HardwareProfile - PIC32MX534F064H.h"
#include "USB/usb.h"
#include "USB/usb_function_generic.h"

/******************************************************************************/
/* Global Declarations                                                        */
/******************************************************************************/
/* i.e. uint32_t <variable_name>; */
#define	GetSystemClock()              (80000000ul)
#define	GetPeripheralClock()          (GetSystemClock()/(1 << OSCCONbits.PBDIV))
#define DESIRED_BAUDRATE    (9600)      //The desired BaudRate
// I2C Constants
#define I2C_CLOCK_FREQ      100000
#define I2C_BUS             I2C1
#define ACCEL_ADDRESS       0x1D    // 0b011101 MMA8452 address
/* Set the scale below either 2, 4 or 8*/
#define ACCEL_SCALE			2 // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.
/* Set the output data rate below. Value should be between 0 and 7*/
#define ACCEL_DATARATE		0 // 0=800Hz, 1=400, 2=200, 3=100, 4=50, 5=12.5, 6=6.25, 7=1.56
#define MAG_ADDRESS			0x0E	// address of MAG3110
#define GYRO_ADDRESS		0x69	// address of ITG-3200 0xb1101001
#define CHAR_ARRAY_LENGTH	6
// Timer 1 tick rate
#define T1_PRESCALE         512 //256
#define T1_TICK       		(SYS_FREQ/PB_DIV/T1_PRESCALE)


/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

/* TODO Initialize User Ports/Peripherals/Project here */

void InitApp(void)
{
        UINT32 actualI2CClock;
        int i;
 // Initialize debug messages (when supported)
    //DBINIT();

    /* TODO <INSERT USER APPLICATION CODE HERE> */
    //OpenTimer2(T2_ON | T2_PS_1_1, 0x00F9);
    /* Enable OC | 32 bit Mode  | Timer2 is selected | Continuous O/P   | OC Pin High , S Compare value, Compare value*/
    //OpenOC1( OC_ON | OC_TIMER_MODE16 | OC_TIMER2_SRC | OC_CONTINUE_PULSE | OC_LOW_HIGH , 0x0000, 0x007C );

        for(i =0; i<27; i++){
            data[i] = 0;}

        data[0] = 0x01;
        data[7] = 0x02;
        data[14] = 0x03;
        data[21] = 0x04;
        data[24] = 0x05;

    //PORTSetPinsDigitalOut(IOPORT_A, BIT_0);
    //PORTSetPinsDigitalIn(IOPORT_D, BIT_6);
	PORTSetPinsDigitalOut(IOPORT_D, BIT_7);
	PORTSetPinsDigitalOut(IOPORT_D, BIT_5);
	PORTSetPinsDigitalOut(IOPORT_D, BIT_6);
	mPORTDClearBits(BIT_7 | BIT_6 | BIT_5);

    //mPORTAClearBits(BIT_0);
    mCNOpen(CN_ON | CN_IDLE_CON, CN7_ENABLE, CN7_PULLUP_ENABLE);
    //temp = mPORTDRead();
    ConfigIntCN(CHANGE_INT_ON | CHANGE_INT_PRI_2);

    //mPORTAClearBits(BIT_7); 		// Turn off RA7 on startup.
    //mPORTASetPinsDigitalOut(BIT_7);	// Make RA7 as output.



    // Explorer-16 uses UART2 to connect to the PC.
    // This initialization assumes 10MHz Fpb clock. If it changes,
    // you will have to modify baud rate initializer.
    UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(UART2, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(UART2, GetPeripheralClock(), DESIRED_BAUDRATE);
    UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART2 RX Interrupt
    INTSetVectorPriority(INT_VECTOR_UART(UART2), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(UART2), INT_SUB_PRIORITY_LEVEL_0);
    INTEnable(INT_SOURCE_UART_RX(UART2), INT_ENABLED);
    INTEnable(INT_SOURCE_UART_TX(UART2), INT_ENABLED);

        // Explorer-16 uses UART2 to connect to the PC.
    // This initialization assumes 10MHz Fpb clock. If it changes,
    // you will have to modify baud rate initializer.
    UARTConfigure(UART3, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(UART3, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(UART3, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(UART3, GetPeripheralClock(), DESIRED_BAUDRATE);
    UARTEnable(UART3, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART2 RX Interrupt
    INTSetVectorPriority(INT_VECTOR_UART(UART3), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(UART3), INT_SUB_PRIORITY_LEVEL_0);
    INTEnable(INT_SOURCE_UART_RX(UART3), INT_ENABLED);

    // Configure the T1 timer which says when to send the accel data to the computer
    OpenTimer1(T1_ON | T1_PS_1_256, T1_TICK); //one second interrupts
    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);

    OpenTimer2(T2_ON | T2_PS_1_4, 1); //one second interrupts
    ConfigIntTimer2(T2_INT_OFF | T1_INT_PRIOR_2);


    //Enable Camera Peripheral and Linear regulators
    mPORTDSetBits(BIT_6 | BIT_5);
    WriteString("CAM ON \n\r");

	CloseADC10();
	//Set up analog to digital converter for port pin AN15
	// define setup parameters for OpenADC10
	// 				Turn module on | ouput in integer | trigger mode auto | enable autosample
	#define ATDPARAM1  ADC_MODULE_ON | ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON
	// 				ADC ref external    | disable offset test    | disable scan mode | perform 2 samples | use dual buffers | use alternate mode
	#define ATDPARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_1 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_OFF
	//				  use ADC internal clock | set sample time
	#define ATDPARAM3  ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_31
	//               set AN15 as analog inputs
	#define ATDPARAM4	ENABLE_AN15_ANA
	// do not assign channels to scan
	#define ATDPARAM5	SKIP_SCAN_AN15

	// use ground as neg ref for A | use AN4 for input A
	SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN15); // configure to sample AN15
	OpenADC10( ATDPARAM1, ATDPARAM2, ATDPARAM3, ATDPARAM4, ATDPARAM5 ); // configure ADC using the parameters defined above;
	//mAD1SetIntPriority(INT_PRIORITY_LEVEL_2);
	//mAD1SetIntSubPriority(INT_SUB_PRIORITY_LEVEL_0);

	EnableADC10(); // Enable the ADC



    // Configure I2C
    I2CConfigure(I2C_BUS, I2C_ENABLE_SLAVE_CLOCK_STRETCHING|I2C_ENABLE_HIGH_SPEED);
    actualI2CClock = I2CSetFrequency(I2C1, GetPeripheralClock(), I2C_CLOCK_FREQ);
    if ( abs(actualI2CClock-I2C_CLOCK_FREQ) > I2C_CLOCK_FREQ/10 )
    {
        DBPRINTF("I2C1 clock frequency (%ld) error exceeds 10%%\n", actualClock);
    }
    I2CEnable(I2C_BUS, TRUE);
	I2CClearStatus(I2C_BUS, I2C_TRANSMITTER_FULL);

    // Configure external interrupts on INT1 and INT2
    PORTSetPinsDigitalIn(IOPORT_D, BIT_0 | BIT_8);
	ConfigINT1(EXT_INT_PRI_2 | RISING_EDGE_INT | EXT_INT_ENABLE);
	ConfigINT0(EXT_INT_PRI_2 | RISING_EDGE_INT | EXT_INT_ENABLE);
    //Configure external interrupts on INT3
    PORTSetPinsDigitalIn(IOPORT_D, BIT_11);
	ConfigINT4(EXT_INT_PRI_2 | RISING_EDGE_INT | EXT_INT_ENABLE);

    /*Configure Multivector Interrupt Mode.  Using Single Vector Mode
    is expensive from a timing perspective, so most applications
    should probably not use a Single Vector Mode*/
    INTEnableSystemMultiVectoredInt();
    // configure for multi-vectored mode
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
    // enable interrupts
    INTEnableInterrupts();

    //Configure I2C bus
    // Configure Accelerometer
	UINT8 tOutData;
	if (I2CSingleByteRead(ACCEL_ADDRESS, 0x0D, &tOutData))
	{
		if (tOutData == 0x2A)
		{
			DBPRINTF("OMG it works!!!\n");
			if (!initMMA8452(ACCEL_SCALE, ACCEL_DATARATE)) {
				WriteString("Failed to initilize Accelerometer");
				return;
			}
		}
		else
			WriteString("Failed to verify Accelerometer");
	}
	if (I2CSingleByteRead(MAG_ADDRESS,  0x07 ,&tOutData))
	{
		if (tOutData == 0xC4)
		{
			DBPRINTF("OMG it works!!!\n");
			if (!initMAG3110()) {
				WriteString("Failed to initilize Magnetometer");
				return;
			}
		}
		else
			WriteString("Failed to verify Magnetometer");
	}
	if (I2CSingleByteRead(GYRO_ADDRESS, 0x00, &tOutData))
	{
		if (tOutData == GYRO_ADDRESS)
		{
			DBPRINTF("OMG it works!!!\n");
			if (!initITG3200()) {
				WriteString("Failed to initilize Gyroscope");
				return;
			}
		}
		else
			WriteString("Failed to verify Gyroscope");
	}
	WriteString("Type as you will :/\n\r");



}

void Insertion_Sort(int flag){

    //Highest prioerity is 1
    int temp;
    int i;

    rear++;
    rear = rear % 5;
    i= rear;

        while(flag < queue[i] || (i+1) % 5 == front){
            queue[(i+1) % 5] = queue[i];
            i--;
            if(i == -1)
                i += 5;
        }
    queue[(i) % 5] = flag;
}

// helper functions
void WriteString(const char *string)
{
  while (*string != '\0')
    {
        WriteChar(*string);
        string++;
    }
}

void WriteChar(const char c)
{
    //buffer full, wait until space opens
        while (TBufferHead == ((TBufferTail + 1) % TBufferSize));
        //add to buffer next character
        rs232TBuffer[TBufferTail] = c;
        TBufferTail = (TBufferTail + 1) % TBufferSize;
        //Set interrupt if transmitter is ready
        if (UARTTransmitterIsReady(UART2)) {
            INTSetFlag(INT_SOURCE_UART_TX(UART2));
		}
		//If transmitter interrupt disabled, enable
		if (!INTGetEnable(INT_SOURCE_UART_TX(UART2)))
				INTEnable(INT_SOURCE_UART_TX(UART2), INT_ENABLED);
}

void bufferSpaces(char t[])
{
	int i, y;
	for (i = 0; (t[i] != '\0') && (i != CHAR_ARRAY_LENGTH - 1); i++);
	if (i == CHAR_ARRAY_LENGTH - 1)
		return;
	for (y = 1;i >= 0; i--, y++)
		t[CHAR_ARRAY_LENGTH - y] = t[i];
	for (;y <= CHAR_ARRAY_LENGTH; y++)
		t[CHAR_ARRAY_LENGTH - y] = ' ';
}

/* converts a base 10 int to a string an returns
 converted int with '\0' char at the end of the converted string
 requires that char array has enough space to hold int
 (and negative sign if negative). returns length in chars of converted int*/
unsigned int convIntToString(int i, char t[])
{
	unsigned int temp, cnt = 0, k;
	BOOL neg = false;
	//special case i equals 0
	if (i == 0) {
		t[0] = '0';
		t[1] = '\0';
		return 1;
	}
	if (i < 0) {
		neg = true;
		i = ~i;
	}
	for (k = 0; i > 0; k++) {
		temp = i % 10;
		t[k] = temp + '0';
		i /= 10;
		cnt++;
	}
	if (neg)
		t[k] = '-';
	else
		t[k] = ' ';
	cnt++;
	t[cnt] = '\0';
	//reverse array
	for (k = 0; k < cnt/2; k++) {
		temp = t[k];
		t[k] = t[cnt - k - 1];
		t[cnt - k - 1] = temp;
	}
	return cnt;
}

/* Converts a base 10 float to a string and returns converted float with
 '\0' car at the end of th string.
 Requires that char array has enough space to hold float (and negative
 sign if negative).
 Converts to n decimal places
 returns length in chars of converted float*/
unsigned int convFloatToString(float f, unsigned int n, char t[])
{
	unsigned int temp, cnt = 0, k;
	int i;
	BOOL neg = false;
	i = (int)(f * pow(10, n));
	if (i < 0) {
		neg = true;
		i = ~i;
	}
	for (k = 0; i > 0 || k <= n + 1; k++) {
		temp = i % 10;
		t[k] = temp + '0';
		i /= 10;
		cnt++;
		if (k == 1) {
			k++;
			t[k] = '.';
			cnt++;
		}
	}
	if (neg)
		t[k] = '-';
	else
		t[k] = ' ';
	t[cnt] = '\0';
	//reverse array
	for (k=0; k < cnt / 2; k++) {
		temp = t[k];
		t[k] = t[cnt - k - 1];
		t[cnt - k - 1] = temp;
	}
	return cnt;
}

BOOL I2CStartTransfer( BOOL restart)
{
    I2C_STATUS  status;
    // Send the Start (or Restart) signal
    if(restart)
    {
        I2CRepeatStart(I2C_BUS);
    }
    else
    {
         // Wait for the bus to be idle, then start the transfer
        while( !I2CBusIsIdle(I2C_BUS) );
        if(I2CStart(I2C_BUS) != I2C_SUCCESS)
        {
            DBPRINTF("Error: Bus collision during transfer Start\n");
            return FALSE;
        }
    }

    // Wait for the signal to complete
    do
    {
        status = I2CGetStatus(I2C_BUS);
		if (status & I2C_ARBITRATION_LOSS)
			return FALSE;

    } while ( !(status & I2C_START) );

    return TRUE;
}

BOOL TransmitOneByte( UINT8 data )
{
    // Wait for the transmitter to be ready
    while(!I2CTransmitterIsReady(I2C_BUS));

    // Transmit the byte
    if(I2CSendByte(I2C_BUS, data) == I2C_MASTER_BUS_COLLISION)
    {
        DBPRINTF("Error: I2C Master Bus Collision\n");
        return FALSE;
    }

    // Wait for the transmission to finish
    while(!I2CTransmissionHasCompleted(I2C_BUS));
    // Verify that the byte was acknowledged
    if(!I2CByteWasAcknowledged(I2C_BUS))
    {
        DBPRINTF("Error: Sent byte was not acknowledged\n");
        return FALSE;
    }

    return TRUE;
}

BOOL RecieveOneByte(UINT8 * data)
{
    if(I2CReceiverEnable(I2C_BUS, TRUE) == I2C_RECEIVE_OVERFLOW)
    {
        DBPRINTF("Error: I2C Receive Overflow\n");
        return FALSE;
    }
    else
    {
        while(!I2CReceivedDataIsAvailable(I2C_BUS));
        *(data) = I2CGetByte(I2C_BUS);
    }

    return TRUE;
}

void StopTransfer( void )
{
    I2C_STATUS  status;

    // Send the Stop signal
	I2CStop(I2C_BUS);

    // Wait for the signal to complete
    do
    {
        status = I2CGetStatus(I2C_BUS);

    } while ( !(status & I2C_STOP) );
}

BOOL I2CSingleByteWrite(UINT8 target, UINT8 regAddress, UINT8 data)
{
    if (!I2CStartTransfer( FALSE ))
        return FALSE;
    I2C_7_BIT_ADDRESS slaveAddress;
    I2C_FORMAT_7_BIT_ADDRESS(slaveAddress, target, I2C_WRITE);
    if (!TransmitOneByte(slaveAddress.byte))
        return FALSE;
    if (!TransmitOneByte(regAddress))
        return FALSE;
    if (!TransmitOneByte(data))
        return FALSE;
    StopTransfer();
    return TRUE;
}

BOOL I2CMultByteWrite(UINT8 target, UINT8 regAddress, UINT numData, UINT8 *data)
{
    if (!I2CStartTransfer( FALSE ))
        return FALSE;
    I2C_7_BIT_ADDRESS slaveAddress;
    I2C_FORMAT_7_BIT_ADDRESS(slaveAddress, target, I2C_WRITE);
    if (!TransmitOneByte(slaveAddress.byte))
        return FALSE;
    if (!TransmitOneByte(regAddress))
        return FALSE;
	int i;
	for (i = 0; i < numData; i++) {
		if (!TransmitOneByte(data[i]))
			return FALSE;
	}
    StopTransfer();
    return TRUE;
}

BOOL I2CSingleByteRead(UINT8 target, UINT8 regAddress, UINT8* outData)
{
    if (!I2CStartTransfer( FALSE ))
        return FALSE;
    I2C_7_BIT_ADDRESS slaveAddress;
    I2C_FORMAT_7_BIT_ADDRESS(slaveAddress, target, I2C_WRITE);
    if (!TransmitOneByte(slaveAddress.byte))
        return FALSE;
    if (!TransmitOneByte(regAddress))
        return FALSE;
    if (!I2CStartTransfer( TRUE ))
        return FALSE;
    I2C_FORMAT_7_BIT_ADDRESS(slaveAddress, target, I2C_READ);
    if (!TransmitOneByte(slaveAddress.byte))
        return FALSE;
    if (!RecieveOneByte(outData))
        return FALSE;
    //Send NAK, and wait for it to finish
    I2CAcknowledgeByte(I2C_BUS, FALSE);
    while(!I2CAcknowledgeHasCompleted(I2C_BUS));
    StopTransfer();
    return TRUE;
}

BOOL I2CMultByteRead(UINT8 target, UINT8 regAddress, UINT numData, UINT8* outDataArray)
{
    if (!I2CStartTransfer( FALSE ))
        return FALSE;
    I2C_7_BIT_ADDRESS slaveAddress;
    I2C_FORMAT_7_BIT_ADDRESS(slaveAddress, target, I2C_WRITE);
    if (!TransmitOneByte(slaveAddress.byte))
        return FALSE;
    if (!TransmitOneByte(regAddress))
        return FALSE;
    if (!I2CStartTransfer( TRUE ))
        return FALSE;
    I2C_FORMAT_7_BIT_ADDRESS(slaveAddress, target, I2C_READ);
    if (!TransmitOneByte(slaveAddress.byte))
        return FALSE;
	int i;
	numData -= 1;
	for (i = 0; i < numData; i++)
	{
		//read a byte
		if (!RecieveOneByte(&(outDataArray[i])))
			return FALSE;
		//Send AK, and wait for it to finish
		I2CAcknowledgeByte(I2C_BUS, TRUE);
		while(!I2CAcknowledgeHasCompleted(I2C_BUS));
	}
	//Read last byte
	if (!RecieveOneByte(&(outDataArray[numData])))
			return FALSE;
    //Send NAK, and wait for it to finish
    I2CAcknowledgeByte(I2C_BUS, FALSE);
    while(!I2CAcknowledgeHasCompleted(I2C_BUS));
    StopTransfer();
    return TRUE;
}

/* Initialize the MMA8452 registers */
BOOL initMMA8452(UINT8 fsr, UINT8 dataRate)
{
	UINT8 accel_ctrl_reg1;
	// Must be in standby to change registers
	if (!I2CSingleByteRead(ACCEL_ADDRESS, 0x2A, &accel_ctrl_reg1))
		return false;
	accel_ctrl_reg1 &= ~0x01;
	if (!I2CSingleByteWrite(ACCEL_ADDRESS, 0x2A, accel_ctrl_reg1))
		return false;

	/* Set up the full scale range to 2, 4, or 8g. */
	if ((fsr==2)||(fsr==4)||(fsr==8)) {
		if (!I2CSingleByteWrite(ACCEL_ADDRESS, 0x0E, fsr >> 2))
			return false;
	}
	else {
		if (!I2CSingleByteWrite(ACCEL_ADDRESS, 0x0E, 0))
			return false;
	}

	/* Setup the 3 data rate bits, from 0 to 7 */
	accel_ctrl_reg1 &= ~(0x38);
	if (dataRate <= 7)
		accel_ctrl_reg1 |= (dataRate << 3);
	if (!I2CSingleByteWrite(ACCEL_ADDRESS, 0x2A, accel_ctrl_reg1))
		return false;

	/* Set up interrupt 1 */
	// Active high, push-pull interrupts
	if (!I2CSingleByteWrite(ACCEL_ADDRESS, 0x2C, 0x02))
		return false;
	// DRDY int enabled
	if (!I2CSingleByteWrite(ACCEL_ADDRESS, 0x2D, 0x01))
		return false;
	// DRDY on INT1
	if (!I2CSingleByteWrite(ACCEL_ADDRESS, 0x2E, 0x01))
		return false;

	// Set to active to start reading
	accel_ctrl_reg1 |= 0x01;
	if (!I2CSingleByteWrite(ACCEL_ADDRESS, 0x2A, accel_ctrl_reg1))
		return false;

	return true;
}

BOOL initMAG3110(void)
{
	// enable auto resets
	if (!I2CSingleByteWrite(MAG_ADDRESS, 0x11, 0x80))
		return false;
	// set ODR = 40 hz, OSR = 1, enable sensor
	if (!I2CSingleByteWrite(MAG_ADDRESS, 0x10, 0x21))
		return false;

	return true;
}

BOOL initITG3200(void)
{
	// Set sample rate divider to 5 setting the sample frequency to 166 Hz
	if (!I2CSingleByteWrite(GYRO_ADDRESS, 0x15, 0x05))
		return false;
	// set fsr to +/- 2000 (3), and dlpf_cfg to 3
	if (!I2CSingleByteWrite(GYRO_ADDRESS, 0x16, 0x18))
		return false;
	// set interrupt status to RAW_RDY_EN
	if (!I2CSingleByteWrite(GYRO_ADDRESS, 0x17, 0x35))
		return false;
	// set clock source to pll with internal x gyro reference
	if (!I2CSingleByteWrite(GYRO_ADDRESS, 0x3E, 0x00))
		return false;


	return true;
}
