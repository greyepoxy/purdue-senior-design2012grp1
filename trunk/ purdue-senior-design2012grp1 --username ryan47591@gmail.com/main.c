//------------------------------------------------------------------------------
//		Main Camera Unit Code
//		Author: Ryan Hannah
//		Date:	3/18/2012
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//		Headers
//------------------------------------------------------------------------------
#include "header files/TCM8230.h"
#include "header files/isi.h"

//------------------------------------------------------------------------------
//		Local Definitions
//------------------------------------------------------------------------------
#define PINS_ISI BOARD_ISI_PIO_CTRL1,\
                 BOARD_ISI_PIO_CTRL2,\
                 BOARD_ISI_TWCK,\
                 BOARD_ISI_TWD,\
                 BOARD_ISI_MCK,\
                 BOARD_ISI_VSYNC,\
                 BOARD_ISI_HSYNC,\
                 BOARD_ISI_PCK,\
                 BOARD_ISI_PINS_DATA

//------------------------------------------------------------------------------
//		Global Variables
//------------------------------------------------------------------------------
static const Pin pins[] = {PINS_TWI0, PINS_ISI};


//------------------------------------------------------------------------------
//		Main Function
//------------------------------------------------------------------------------
int main(void) {
	//Initlizing the output pins for TWI 
	PIO_Configure(pins, PIO_LISTSIZE(pins));
	ISI_Enable();
	ISI_Init();

	//Initlizing TCM8230MD to begin image capture
	Camera_Init(AT91C_BASE_TWI0);
	
	while(1);
}
