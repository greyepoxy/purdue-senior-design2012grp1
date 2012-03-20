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
#include "header files/board.h"

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
//		Interupt Handlers
//------------------------------------------------------------------------------
__irq void ISI_Handler(void){
	int i;
	
	for(i=0; i<1; i++);
}


//------------------------------------------------------------------------------
//		Main Function
//------------------------------------------------------------------------------
int main(void) {
	//Initlizing the output pins for TWI 
	PIO_Configure(pins, PIO_LISTSIZE(pins));

	//setting the ISI
	AT91C_BASE_ISI->ISI_CR1 = 0x00001000;
	AT91C_BASE_ISI->ISI_CR2 = 0xC280B1E0;

	//enabling interupts
	AIC_ConfigureIT(AT91C_ID_ISI,AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE,(void (*)())ISI_Handler);
	AT91C_BASE_ISI->ISI_IER = 0x00000001;
	AIC_EnableIT(AT91C_ID_ISI);

	//Initlizing TCM8230MD to begin image capture
	Camera_Init(AT91C_BASE_TWI0);
	
	while(1);
}
