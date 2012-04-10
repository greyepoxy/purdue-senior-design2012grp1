//------------------------------------------------------------------------------
//		TCM8230 Functions
//		Author: Ryan Hannah
//		Date:	3/18/2012
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//		Header Files
//------------------------------------------------------------------------------
	#include "TCM8230.h"
	#include <AT91SAM9XE256.H>

//------------------------------------------------------------------------------
//		Global Variables
//------------------------------------------------------------------------------
	#define C_Addr	0x3C
	#define TWCK 400000

//------------------------------------------------------------------------------
//		Function Definitions
//------------------------------------------------------------------------------

	//------------------------------------------------------------------------------
	//		Camera_Init
	//		Initilizes the camera to 15fps, RGB, and Black and White
	//		Also begins image capture
	//		Variables:
	//				pTwi - the TWI perfieral instance that is to be used for
	//					   communication.
	//------------------------------------------------------------------------------
		void Camera_Init (AT91S_TWI *pTwi) {

			//Initilzing the TWI periferial and configuiring it as a master
			TWI_ConfigureMaster(pTwi, TWCK, 18432000);

			//Writing 0x80 to register 0x02 to set the frames per second to 15
			TWI_StartWrite(pTwi, C_Addr, 0x02, 1, 0x81);
			while(!TWI_ByteSent(pTwi));
			TWI_SendSTOPCondition(pTwi);

			//Writing 0x03 to register 0x03 to set the camera to RGB and Black&White.
			//Also starts image capture.
			TWI_StartWrite(pTwi, C_Addr, 0x03, 1, 0x22);
			while(!TWI_ByteSent(pTwi));
			TWI_SendSTOPCondition(pTwi);

		}
