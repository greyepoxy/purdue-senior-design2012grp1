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
//		Global Variables
//------------------------------------------------------------------------------
static const Pin pins[] = {PINS_TWI0, BOARD_ISI_PINS_DATA, BOARD_ISI_PCK,BOARD_ISI_HSYNC,BOARD_ISI_VSYNC,BOARD_ISI_TWD,BOARD_ISI_TWCK,BOARD_ISI_PIO_CTRL2,BOARD_ISI_PIO_CTRL1,BOARD_ISI_MCK};
ISI_FrameBufferDescriptors FbList;
static const int Buffer = 0x003000A0;
static const int Fb_offset = ((128*96*2)/8); 

//------------------------------------------------------------------------------
//		Interupt Handlers
//------------------------------------------------------------------------------
__irq void ISI_Handler(void){
	int i;
	
	for(i=0; i<1; i++);
	AT91C_BASE_ISI->ISI_SR = 0x00000000;
}


//------------------------------------------------------------------------------
//		Main Function
//------------------------------------------------------------------------------
int main(void) {
	//Initlizing the output pins for TWI 
	PIO_Configure(pins, PIO_LISTSIZE(pins));

	//setting the ISI
	AT91C_BASE_ISI->ISI_CR1 = 0x02009000;
	AT91C_BASE_ISI->ISI_CR2 = 0xC080B860;
	AT91C_BASE_ISI->ISI_CDBA = 0x00302000;
	AT91C_BASE_ISI->ISI_PPFBD = (int)&FbList;

	//enabling interupts
	AIC_ConfigureIT(AT91C_ID_ISI,AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE,(void (*)())ISI_Handler);
	AIC_EnableIT(AT91C_ID_ISI);
	AT91C_BASE_ISI->ISI_IER = 0x0000081;

	//Intilizing the FBD to buffer cam data
	FbList.Current = Buffer;
	FbList.Next = (int)&FbList;	


	//Initlizing TCM8230MD to begin image capture
	Camera_Init(AT91C_BASE_TWI0);
	
	while(1);
}
