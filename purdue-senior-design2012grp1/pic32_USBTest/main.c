/******************************************************************************/
/*  Files to Include                                                          */
/******************************************************************************/

#include <plib.h>           /* Include to use PIC32 peripheral libraries      */
#include <stdint.h>         /* For uint32_t definition                        */
#include <stdbool.h>        /* For true/false definition                      */

#include "system.h"         /* System funct/params, like osc/periph config    */
#include "user.h"           /* User funct/params, such as InitApp             */

#include "HardwareProfile - PIC32MX534F064H.h"
#include "USB/usb.h"
#include "USB/usb_function_generic.h"
#include "interrupts.h"

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

// Timer 1 tick rate
#define T1_PRESCALE         256
#define T1_TICK (SYS_FREQ/PB_DIV/T1_PRESCALE)
/* i.e. uint32_t <variable_name>; */
//User application buffer for receiving and holding OUT packets sent from the host
unsigned char OUTPacket[64];
//User application buffer for sending IN packets to the host
unsigned char INPacket[64];

USB_HANDLE USBGenericOutHandle;
USB_HANDLE USBGenericInHandle;

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
#define ACCEL_PRI           0x01
#define GYRO_PRI            0x02
#define MAG_PRI             0x03
#define DIS_PRI             0x04
#define CAM_PRI             0x05
int accel_x;
int accel_y;
int accel_z;
INT16 mag_x, mag_y, mag_z;
INT16 gyro_x, gyro_y, gyro_z;
volatile char rs232TBuffer[TBufferSize];
USB_HANDLE USBGenericInHandle;

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
BOOL neg;

/** PRIVATE PROTOTYPES *********************************************/
static void InitializeSystem(void);
void ProcessIO(void);

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

int32_t main(void)
{
    front = 0;
    rear = 0;
    char charArray[CHAR_ARRAY_LENGTH];
    char c_flag = 0;
    unsigned int temp;

    // Set interrupt flags
    buttonFlag = 0;
    timer1Flag = 0;
    readAccelFlag = 0;
    readGyroFlag = 0;
    readMagFlag = 0;
    rs232TBuffer[32];
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
    checkatd = 0;
    tcount = 0;
    distance = 0;

    InitializeSystem();

    #if defined(USB_INTERRUPT)
        USBDeviceAttach();
    #endif

    while(1)
    {
		// Application-specific tasks.
		// Application related code may be added here, or in the ProcessIO() function.
        if(front != rear){
            c_flag = queue[front];
            queue[front] = 0;
            front = (front + 1) % 5;
        }
        ProcessIO();
        if (timer1Flag == 1) {
                        //Accelerometer Data
			/*WriteString("\rAccel-> x: ");
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
			mPORTDToggleBits(BIT_7);
			convIntToString(an15Data, charArray);
			bufferSpaces(charArray);
			WriteString("\r\n");
			WriteString(charArray);*/
			timer1Flag = 0;
	}
	if (c_flag == ACCEL_PRI) {
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
                        data[0] = 0x11;
                        data[1] = (char)(accel_x >> 8);
                        data[2] = (char)accel_x;
                        data[3] = (char)(accel_y >> 8);
                        data[4] = (char)accel_y;
                        data[5] = (char)(accel_z >> 8);
                        data[6] = (char)accel_z;
			EnableINT1;
	}
	if (c_flag == MAG_PRI) {
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
                        data[14] = 0x12;
                        data[15] = (char)(mag_x >>8);
                        data[16] = (char)mag_x;
                        data[17] = (char)(mag_y >> 8);
                        data[18] = (char)mag_y;
                        data[19] = (char)(mag_z >> 8);
                        data[20] = (char)mag_z;
			
			EnableINT0;
	}
	if (c_flag == GYRO_PRI) {
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

                        data[7] = 0x13;
                        data[8] = (char)(gyro_x >> 8);
                        data[9] = (char)gyro_x;
                        data[10] = (char)(gyro_y >> 8);
                        data[11] = (char)gyro_y;
                        data[12] = (char)(gyro_z >> 8);
                        data[13] = (char)gyro_z;
			
			EnableINT4;
	}
        if(c_flag == DIS_PRI){
            distance = ((340290 * tcount)/1000000) - 140;
            convIntToString(distance, charArray);
            bufferSpaces(charArray);
            WriteString(charArray);
            tcount = 0;
            data[21] = 0x14;
            data[22] = (char)(distance >> 8);
            data[23] = (char) distance;

        }

        if(c_flag == CAM_PRI){
            data[24] = 0x15;
            data[25] = 0x00;
            data[26] = 0x00;
        }
   }//end while
}

static void InitializeSystem(void)
{
	#ifndef PIC32_STARTER_KIT
		/*The JTAG is on by default on POR.  A PIC32 Starter Kit uses the JTAG, but
		for other debug tool use, like ICD 3 and Real ICE, the JTAG should be off
		to free up the JTAG I/O */
		DDPCONbits.JTAGEN = 0;
	#endif

	// some random register set by usb main > <
	AD1PCFG = 0xFFFF;

    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

    /* Initialize I/O and Peripherals for application */
    InitApp();

	// LED port
	//PORTSetPinsDigitalOut(IOPORT_D, BIT_7);
	// Board Control Bits
	//PORTSetPinsDigitalOut(IOPORT_D, BIT_5);
	//PORTSetPinsDigitalOut(IOPORT_D, BIT_6);
	//mPORTDClearBits(BIT_7 | BIT_6 | BIT_5);
	LEDOnFlag = 0;


	//Enable Camera Peripheral and Linear regulators
	//mPORTDSetBits(BIT_6 | BIT_5);

	// Configure the T1 timer which says when to send the accel data to the computer
    OpenTimer1(T1_ON | T1_PS_1_256, T1_TICK); //one second interrupts
    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);

//	The USB specifications require that USB peripheral devices must never source
//	current onto the Vbus pin.  Additionally, USB peripherals should not source
//	current on D+ or D- when the host/hub is not actively powering the Vbus line.
//	When designing a self powered (as opposed to bus powered) USB peripheral
//	device, the firmware should make sure not to turn on the USB module and D+
//	or D- pull up resistor unless Vbus is actively powered.  Therefore, the
//	firmware needs some means to detect when Vbus is being powered by the host.
//	A 5V tolerant I/O pin can be connected to Vbus (through a resistor), and
// 	can be used to detect when Vbus is high (host actively powering), or low
//	(host is shut down or otherwise not supplying power).  The USB firmware
// 	can then periodically poll this I/O pin to know when it is okay to turn on
//	the USB module/D+/D- pull up resistor.  When designing a purely bus powered
//	peripheral device, it is not possible to source current on D+ or D- when the
//	host is not actively providing power on Vbus. Therefore, implementing this
//	bus sense feature is optional.  This firmware can be made to use this bus
//	sense feature by making sure "USE_USB_BUS_SENSE_IO" has been defined in the
//	HardwareProfile.h file.
    #if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
    #endif

//	If the host PC sends a GetStatus (device) request, the firmware must respond
//	and let the host know if the USB peripheral device is currently bus powered
//	or self powered.  See chapter 9 in the official USB specifications for details
//	regarding this request.  If the peripheral device is capable of being both
//	self and bus powered, it should not return a hard coded value for this request.
//	Instead, firmware should check if it is currently self or bus powered, and
//	respond accordingly.  If the hardware has been configured like demonstrated
//	on the PICDEM FS USB Demo Board, an I/O pin can be polled to determine the
//	currently selected power source.  On the PICDEM FS USB Demo Board, "RA2"
//	is used for	this purpose.  If using this feature, make sure "USE_SELF_POWER_SENSE_IO"
//	has been defined in HardwareProfile.h, and that an appropriate I/O pin has been mapped
//	to it in HardwareProfile.h.
    #if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN;	// See HardwareProfile.h
    #endif

	USBGenericOutHandle = 0;
	USBGenericInHandle = 0;

    USBDeviceInit();	//usb_device.c.  Initializes USB module SFRs and firmware
    					//variables to known states.

    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
}//end InitializeSystem

void ProcessIO(void)
{
    int i;

    //User Application USB tasks below.
    //Note: The user application should not begin attempting to read/write over the USB
    //until after the device has been fully enumerated.  After the device is fully
    //enumerated, the USBDeviceState will be set to "CONFIGURED_STATE".
    if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) return;


    if(!USBHandleBusy(USBGenericOutHandle))		//Check if the endpoint has received any data from the host.
    {
        switch(OUTPacket[0])
        {
            case 0x80:
             for(i=0;i < 27; i++){
                    INPacket[i] = data[i];
             }
             if(!USBHandleBusy(USBGenericInHandle))
	        {
		            //The endpoint was not "busy", therefore it is safe to write to the buffer and arm the endpoint.
	                //The USBGenWrite() function call "arms" the endpoint (and makes the handle indicate the endpoint is busy).
	                //Once armed, the data will be automatically sent to the host (in hardware by the SIE) the next time the
	                //host polls the endpoint.  Once the data is successfully sent, the handle (in this case USBGenericInHandle)
	                //will indicate the the endpoint is no longer busy.
					USBGenericInHandle = USBGenWrite(USBGEN_EP_NUM,(BYTE*)&INPacket,USBGEN_EP_SIZE);
                }
             data[0] = 0x01;
             data[7] = 0x02;
             data[14] = 0x03;
             data[21] = 0x04;
             data[24] = 0x05;
             break;
        }
    }

    //As the device completes the enumeration process, the USBCBInitEP() function will
    //get called.  In this function, we initialize the user application endpoints (in this
    //example code, the user application makes use of endpoint 1 IN and endpoint 1 OUT).
    //The USBGenRead() function call in the USBCBInitEP() function initializes endpoint 1 OUT
    //and "arms" it so that it can receive a packet of data from the host.  Once the endpoint
    //has been armed, the host can then send data to it (assuming some kind of application software
    //is running on the host, and the application software tries to send data to the USB device).

    //If the host sends a packet of data to the endpoint 1 OUT buffer, the hardware of the SIE will
    //automatically receive it and store the data at the memory location pointed to when we called
    //USBGenRead().  Additionally, the endpoint handle (in this case USBGenericOutHandle) will indicate
    //that the endpoint is no longer busy.  At this point, it is safe for this firmware to begin reading
    //from the endpoint buffer, and processing the data.  In this example, we have implemented a few very
    //simple commands.  For example, if the host sends a packet of data to the endpoint 1 OUT buffer, with the
    //first byte = 0x80, this is being used as a command to indicate that the firmware should "Toggle LED(s)".
    //if(!USBHandleBusy(USBGenericOutHandle))		//Check if the endpoint has received any data from the host.
    //{
    //    switch(OUTPacket[0])					//Data arrived, check what kind of command might be in the packet of data.
    //    {
    //        case 0x80:  //Toggle LED(s) command from PC application, stop/start blinking LED.
	//			if (INTGetEnable(INT_T1))
	//				INTEnable(INT_T1, INT_DISABLED);
	//			else
	//				INTEnable(INT_T1, INT_ENABLED);
//
  //              break;
    //        case 0x81:  //Get push LED state command from PC application.
      //          INPacket[0] = 0x81;				//Echo back to the host PC the command we are fulfilling in the first byte.  In this case, the Get Pushbutton State command.
	//			if(LEDOnFlag == 1)	//LED on
	//			{
	//				INPacket[1] = 0x01;
	//			}
	//			else							//LED off
	//			{
	//				INPacket[1] = 0x00;
	//			}
				//Now check to make sure no previous attempts to send data to the host are still pending.  If any attemps are still
				//pending, we do not want to write to the endpoint 1 IN buffer again, until the previous transaction is complete.
				//Otherwise the unsent data waiting in the buffer will get overwritten and will result in unexpected behavior.
         //       if(!USBHandleBusy(USBGenericInHandle))
	  //          {
		            //The endpoint was not "busy", therefore it is safe to write to the buffer and arm the endpoint.
	                //The USBGenWrite() function call "arms" the endpoint (and makes the handle indicate the endpoint is busy).
	                //Once armed, the data will be automatically sent to the host (in hardware by the SIE) the next time the
	                //host polls the endpoint.  Once the data is successfully sent, the handle (in this case USBGenericInHandle)
	                //will indicate the the endpoint is no longer busy.
	//				USBGenericInHandle = USBGenWrite(USBGEN_EP_NUM,(BYTE*)&INPacket,USBGEN_EP_SIZE);
          //      }
            //    break;
        //}

        //Re-arm the OUT endpoint for the next packet:
	    //The USBGenRead() function call "arms" the endpoint (and makes it "busy").  If the endpoint is armed, the SIE will
	    //automatically accept data from the host, if the host tries to send a packet of data to the endpoint.  Once a data
	    //packet addressed to this endpoint is received from the host, the endpoint will no longer be busy, and the application
	    //can read the data which will be sitting in the buffer.
        //USBGenericOutHandle = USBGenRead(USBGEN_EP_NUM,(BYTE*)&OUTPacket,USBGEN_EP_SIZE);
    //}
}//end ProcessIO

