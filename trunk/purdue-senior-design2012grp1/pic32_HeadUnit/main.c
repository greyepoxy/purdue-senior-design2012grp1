/******************************************************************************/
/*  Files to Include                                                          */
/******************************************************************************/

#include <plib.h>           /* Include to use PIC32 peripheral libraries      */
#include <stdint.h>         /* For uint32_t definition                        */
#include <stdbool.h>        /* For true/false definition                      */
#include <math.h>			/* Math functions*/

#include "system.h"         /* System funct/params, like osc/periph config    */
#include "user.h"           /* User funct/params, such as InitApp             */
#include "interrupts.h"     /* Interrupt flags                                */

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

/* i.e. uint32_t <variable_name>; */
#define	GetSystemClock()              (80000000ul)
#define	GetPeripheralClock()          (GetSystemClock()/(1 << OSCCONbits.PBDIV))
#define DESIRED_BAUDRATE    (9600)      //The desired BaudRate
// Timer 1 tick rate
#define T1_PRESCALE         256
#define T1_TICK       		(SYS_FREQ/PB_DIV/T1_PRESCALE)
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
int accel_x;
int accel_y;
int accel_z;
INT16 mag_x, mag_y, mag_z;
INT16 gyro_x, gyro_y, gyro_z;
volatile char rs232TBuffer[TBufferSize];

void WriteString(const char *);
void WriteChar(const char);
//void WriteFloat(float f, unsigned int, unsigned int);
unsigned int convIntToString(int, char *);
unsigned int convFloatToString(float, unsigned int, char *);
void bufferSpaces(char *);
BOOL I2CStartTransfer(BOOL);
void StopTransfer( void );
BOOL TransmitOneByte( UINT8 );
BOOL I2CSingleByteWrite(UINT8, UINT8, UINT8);
BOOL I2CMultByteWrite(UINT8, UINT8, UINT, UINT8*);
BOOL I2CSingleByteRead(UINT8, UINT8, UINT8*);
BOOL I2CMultByteRead(UINT8, UINT8, UINT, UINT8*);
BOOL initMMA8452(UINT8, UINT8);
BOOL initMAG3110(void);
BOOL initITG3200(void);


/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

int32_t main(void)
{

	char charArray[CHAR_ARRAY_LENGTH];
     unsigned int temp;
     UINT32 actualI2CClock;
     // Set interrupt flags
     buttonFlag = 0;
     timer1Flag = 0;
     readAccelFlag = 0;
	 readGyroFlag = 0;
	 readMagFlag = 0;
     //rs232TBuffer[32];
     TBufferHead = 0;
     TBufferTail = 0;
     accel_x = 0;
     accel_y = 0;
     accel_z = 0;
	 mag_x = 0;
	 mag_y = 0;
	 mag_z = 0;
	 gyro_x = 0;
	 gyro_y = 0;
	 gyro_z = 0;


#ifndef PIC32_STARTER_KIT
    /*The JTAG is on by default on POR.  A PIC32 Starter Kit uses the JTAG, but
    for other debug tool use, like ICD 3 and Real ICE, the JTAG should be off
    to free up the JTAG I/O */
    DDPCONbits.JTAGEN = 0;
#endif

    //OSCConfig( OSC_POSC_PLL, OSC_PLL_MULT_20, OSC_PLL_POST_2, OSC_FRC_POST_1 );
    

    /*Refer to the C32 peripheral library compiled help file for more
    information on the SYTEMConfig function.
    
    This function sets the PB divider, the Flash Wait States, and the DRM
    /wait states to the optimum value.  It also enables the cacheability for
    the K0 segment.  It could has side effects of possibly alter the pre-fetch
    buffer and cache.  It sets the RAM wait states to 0.  Other than
    the SYS_FREQ, this takes these parameters.  The top 3 may be '|'ed
    together:
    
    SYS_CFG_WAIT_STATES (configures flash wait states from system clock)
    SYS_CFG_PB_BUS (configures the PB bus from the system clock)
    SYS_CFG_PCACHE (configures the pCache if used)
    SYS_CFG_ALL (configures the flash wait states, PB bus, and pCache)*/

    /* TODO Add user clock/system configuration code if appropriate.  */
    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

    /* Initialize I/O and Peripherals for application */
    InitApp();

    // Initialize debug messages (when supported)
    DBINIT();

    /* TODO <INSERT USER APPLICATION CODE HERE> */
    //OpenTimer2(T2_ON | T2_PS_1_1, 0x00F9);
    /* Enable OC | 32 bit Mode  | Timer2 is selected | Continuous O/P   | OC Pin High , S Compare value, Compare value*/
    //OpenOC1( OC_ON | OC_TIMER_MODE16 | OC_TIMER2_SRC | OC_CONTINUE_PULSE | OC_LOW_HIGH , 0x0000, 0x007C );

    PORTSetPinsDigitalOut(IOPORT_A, BIT_0);
    PORTSetPinsDigitalIn(IOPORT_D, BIT_6);
    mPORTAClearBits(BIT_0);
    mCNOpen(CN_ON | CN_IDLE_CON, CN15_ENABLE, CN15_PULLUP_ENABLE);
    temp = mPORTDRead();
    ConfigIntCN(CHANGE_INT_ON | CHANGE_INT_PRI_2);

    mPORTAClearBits(BIT_7); 		// Turn off RA7 on startup.
    mPORTASetPinsDigitalOut(BIT_7);	// Make RA7 as output.

    // Explorer-16 uses UART2 to connect to the PC.
    // This initialization assumes 10MHz Fpb clock. If it changes,
    // you will have to modify baud rate initializer.
    UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(UART2, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(UART2, GetPeripheralClock(), DESIRED_BAUDRATE);
    UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART2 RX Interrupt
    INTEnable(INT_SOURCE_UART_RX(UART2), INT_ENABLED);
    INTEnable(INT_SOURCE_UART_TX(UART2), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(UART2), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(UART2), INT_SUB_PRIORITY_LEVEL_0);

    // Configure the T1 timer which says when to send the accel data to the computer
    OpenTimer1(T1_ON | T1_PS_1_256, T1_TICK);
    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);

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
    PORTSetPinsDigitalIn(IOPORT_E, BIT_8 | BIT_9);
	ConfigINT1(EXT_INT_PRI_2 | RISING_EDGE_INT | EXT_INT_ENABLE);
	ConfigINT2(EXT_INT_PRI_2 | RISING_EDGE_INT | EXT_INT_ENABLE);
	// Configure external interrupts on INT3
	PORTSetPinsDigitalIn(IOPORT_A, BIT_14);
	ConfigINT3(EXT_INT_PRI_2 | RISING_EDGE_INT | EXT_INT_ENABLE);

    /*Configure Multivector Interrupt Mode.  Using Single Vector Mode
    is expensive from a timing perspective, so most applications
    should probably not use a Single Vector Mode*/
    INTEnableSystemMultiVectoredInt();
    // configure for multi-vectored mode
    //INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
    // enable interrupts
    //INTEnableInterrupts();

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
				return(0);
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
				return(0);
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
				return(0);
			}
		}
		else
			WriteString("Failed to verify Gyroscope");
	}

    while(buttonFlag != 1)
    {
		if (timer1Flag == 1) {
			//Accelerometer Data
			WriteString("\rAccel-> x: ");
			convIntToString(accel_x, charArray);
			bufferSpaces(charArray);
			WriteString(charArray);
			WriteString(" y: ");
			convIntToString(accel_y, charArray);
			bufferSpaces(charArray);
			WriteString(charArray);
			WriteString(" z: ");
			convIntToString(accel_z, charArray);
			bufferSpaces(charArray);
			WriteString(charArray);
			//Magnetometer Data
			WriteString(" Mag-> x: ");
			convIntToString(mag_x, charArray);
			bufferSpaces(charArray);
			WriteString(charArray);
			WriteString(" y: ");
			convIntToString(mag_y, charArray);
			bufferSpaces(charArray);
			WriteString(charArray);
			WriteString(" z: ");
			convIntToString(mag_z, charArray);
			bufferSpaces(charArray);
			WriteString(charArray);
			timer1Flag = 0;
			//Gyroscope Data
			WriteString(" Gyro-> x: ");
			convIntToString(gyro_x, charArray);
			bufferSpaces(charArray);
			WriteString(charArray);
			WriteString(" y: ");
			convIntToString(gyro_y, charArray);
			bufferSpaces(charArray);
			WriteString(charArray);
			WriteString(" z: ");
			convIntToString(gyro_z, charArray);
			bufferSpaces(charArray);
			WriteString(charArray);
			timer1Flag = 0;
		}
		if (readAccelFlag == 1) {
			//Do i2c operations here to read accelerometer
			UINT8 rawData[6]; // x/y/z accel register data stored here
			// Read the six raw data registers into data array
			I2CMultByteRead(ACCEL_ADDRESS, 0x01, 6, &rawData[0]);
			
			int tData, i;
			/* loop to calculate 12-bit ADC and g value for each axis */
			for (i=0; i<6; i+=2)
			{
				// Turn the MSB and LSB into a 12-bit value
				tData = ((rawData[i] << 8) | rawData[i+1]) >> 4;
				// If the number is negative, we have to make it so manually (no 12-bit data type)
				if (rawData[i] > 0x7F)
					tData -= 4096;
				if (i == 0)
					accel_x = tData;
				else if (i == 2)
					accel_y = tData;
				else
					accel_z = tData;
			}
			readAccelFlag = 0;
			EnableINT1;
		}
		if (readMagFlag == 1) {
			//Do i2c operations here to read magnetometer
			UINT8 rawData[6];
			I2CMultByteRead(MAG_ADDRESS, 0x01, 6, &rawData[0]);
			
			INT16 tData;
			int i;
			/* loop to calculate 16-bit ADC values for each axis */
			for (i=0; i<6; i+=2)
			{
				// Turn the MSB and LSB into a 16-bit value
				tData = ((rawData[i] << 8) | rawData[i+1]);
				if (i == 0)
					mag_x = tData;
				else if (i == 2)
					mag_y = tData;
				else
					mag_z = tData;
			}
			readMagFlag = 0;
			EnableINT2;
		}
		if (readGyroFlag == 1) {
			//Do i2c operations here to read gyroscope
			UINT8 rawData[6]; // x/y/z velocity register data stored here
			// Read the six raw data registers into data array
			I2CMultByteRead(GYRO_ADDRESS, 0x1D, 6, &rawData[0]);

			INT16 tData;
			int i;
			/* loop to calculate 16-bit ADC values for each axis */
			for (i=0; i<6; i+=2)
			{
				// Turn the MSB and LSB into a 16-bit value
				tData = ((rawData[i] << 8) | rawData[i+1]);
				if (i == 0)
					gyro_x = tData;
				else if (i == 2)
					gyro_y = tData;
				else
					gyro_z = tData;
			}
			readGyroFlag = 0;
			EnableINT3;
		}
    }

	CloseINT1();
	CloseINT2();
	CloseINT3();
    //CloseOC1();

    return(1);
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
        if (UARTTransmitterIsReady(UART2))
            INTSetFlag(INT_SOURCE_UART_TX(UART2));
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
	// Set sample rate divider to 7
	if (!I2CSingleByteWrite(GYRO_ADDRESS, 0x15, 0x07))
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
