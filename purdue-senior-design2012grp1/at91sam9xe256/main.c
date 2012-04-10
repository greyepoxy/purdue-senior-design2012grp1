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
ISI_FrameBufferDescriptors FbList[2];
static const int Buffer = 0x00301000;
//static const int Fb_offset = ((128*96*2)/8);
static int frame_ready_flag;
#define DMA_BEGIN  0x00301000;
//static char Matrix[64][96];
//unsigned char volatile * const startofdma = (unsigned char *)DMA_BEGIN;


//------------------------------------------------------------------------------
//		Interupt Handlers
//------------------------------------------------------------------------------
__irq void ISI_Handler(void){
	int status;
	status = AT91C_BASE_ISI->ISI_SR;
	if (status&0x00000080 == 0x00000080)	
	frame_ready_flag = 1;
	else if(status&0x00000001 == 0x00000001)
	frame_ready_flag = 0;
	else
	frame_ready_flag = 0;
	*AT91C_AIC_EOICR = AT91C_BASE_ISI->ISI_SR;
}

//------------------------------------------------------------------------------
//		Main Function
//------------------------------------------------------------------------------
int main(void) {
	int i = 0,j;
	unsigned char byte1 = 0, byte2 = 0;
	unsigned char byte3 = 0, byte4 = 0;
	unsigned char *startofdma;
	int temp = 0;
	startofdma = (unsigned char *)DMA_BEGIN;


	//Initlizing the output pins for TWI and enabling and turning on the reset of cam
	AT91C_BASE_PIOA -> PIO_OER = AT91C_PIO_PA3;
	AT91C_BASE_PIOA -> PIO_CODR = AT91C_PIO_PA3;
	PIO_Configure(pins, PIO_LISTSIZE(pins));


	//Intilizing the FBD to buffer cam data
	for(i = 0; i <= 1; i++){
	FbList[i].Current = Buffer;
	FbList[i].Next = (int)&FbList[i+1];
	};
	FbList[i-1].Next = (int)&FbList[0];

	i=0;

	//enabling interupts
	AIC_ConfigureIT(AT91C_ID_ISI,AT91C_AIC_PRIOR_HIGHEST,(void (*)())ISI_Handler);
	AIC_EnableIT(AT91C_ID_ISI);
	AT91C_BASE_ISI->ISI_IER = 0x0000081;

	//setting the ISI
	AT91C_BASE_ISI->ISI_PPFBD = (int)&FbList;
	AT91C_BASE_ISI->ISI_CR1 = 0x02000000;
    AT91C_BASE_ISI->ISI_CR2 = 0xC080B060;	

  	//clearing the frame ready flag
	frame_ready_flag = 0;
	
	//waiting for 100 cycles of ext clk for cam
	for(j =0; j<8000; j++);

	//turning off reset on cam
	AT91C_BASE_PIOA -> PIO_SODR = AT91C_PIO_PA3;

	//waiting for 2000 cycles of ext clk for cam
	for(j =0; j<20000; j++);

	//Initlizing TCM8230MD to begin image capture
	Camera_Init(AT91C_BASE_TWI0);
	
	while(1){
		//check to see if 
		if(frame_ready_flag == 1) {
			i++;
			frame_ready_flag = 0;

			//enabling interupts
			AIC_ConfigureIT(AT91C_ID_ISI,AT91C_AIC_PRIOR_HIGHEST,(void (*)())ISI_Handler);
			AIC_EnableIT(AT91C_ID_ISI);
			AT91C_BASE_ISI->ISI_IER = 0x0000001;

			//setting the ISI
			AT91C_BASE_ISI->ISI_PPFBD = (int)&FbList;
			AT91C_BASE_ISI->ISI_CR1 = 0x02000000;
    		AT91C_BASE_ISI->ISI_CR2 = 0xC080B060;

			byte1 = startofdma[0];
			byte2 = *startofdma;
			//byte2 = startofdma[1];
			byte3 = startofdma[2];
			byte4 = startofdma[3];
			
			

		};
	};
}
