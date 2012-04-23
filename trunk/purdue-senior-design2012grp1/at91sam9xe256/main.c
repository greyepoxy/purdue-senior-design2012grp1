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
#include "header files/spi.h"               

//------------------------------------------------------------------------------
//		Global Variables
//------------------------------------------------------------------------------
static const Pin pins[] = {PINS_SPI0, PINS_TWI0, BOARD_ISI_PINS_DATA, BOARD_ISI_PCK,BOARD_ISI_HSYNC,BOARD_ISI_VSYNC,BOARD_ISI_TWD,BOARD_ISI_TWCK,BOARD_ISI_PIO_CTRL2,BOARD_ISI_PIO_CTRL1,BOARD_ISI_MCK};
ISI_FrameBufferDescriptors FbList[2];
static const int Buffer = 0x00301000;
//static const int Fb_offset = ((128*96*2)/8);
static int frame_ready_flag;
#define DMA_BEGIN  0x00301000;
unsigned char Matrix[(64*96)];//[4076];//[24576];//[64][96];//[128][96];//[24576];
unsigned int pixaccum;


//------------------------------------------------------------------------------
//		Interupt Handlers
//------------------------------------------------------------------------------
__irq void ISI_Handler(void){
	int status;
	status = AT91C_BASE_ISI->ISI_SR;
	if ((status&0x00000080) == 0x00000080) {
		frame_ready_flag = 1;
	}
	else if((status&0x00000001) == 0x00000001) {
		frame_ready_flag = 0;
	}
	else {
		frame_ready_flag = 0;
	}
	*AT91C_AIC_EOICR = AT91C_BASE_ISI->ISI_SR;
}

//------------------------------------------------------------------------------
//		Main Function
//------------------------------------------------------------------------------
int main(void) {
	int i = 0,j,k,h;
	unsigned char *startofdma;
	unsigned char byte1, byte2, rbyte1, rbyte2;
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
	AT91C_BASE_ISI->ISI_CR1 = 0x02000700;//0x02000000;
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

			// POINT THE BUFFER TO CRAP
			for(i = 0; i <= 1; i++){
				FbList[i].Current = 0x10000000;
				FbList[i].Next = (int)&FbList[i+1];
			}
			FbList[i-1].Next = (int)&FbList[0];

			//enabling interupts
			AIC_ConfigureIT(AT91C_ID_ISI,AT91C_AIC_PRIOR_HIGHEST,(void (*)())ISI_Handler);
			AIC_EnableIT(AT91C_ID_ISI);
			AT91C_BASE_ISI->ISI_IER = 0x0000001;

			//setting the ISI
			AT91C_BASE_ISI->ISI_PPFBD = (int)&FbList;
			AT91C_BASE_ISI->ISI_CR1 = 0x02000700;
    		AT91C_BASE_ISI->ISI_CR2 = 0xC080B060;


			// START UNPACKING/PACKING THE RED MATRIX


			k = 0;	//K IS THE POINTER TO THE NEXT SPOT IN THE DMA	  
			j =	0;
			for(i = 0; i < 6144; i++)//while(i < 24576)//28672)// 12288)			 //SIZE OF ONE MATRIX (64X96)(x2x2)
			{
				byte1 = startofdma[k];
				k++;
				byte2 = startofdma[k];
				k++;
				byte1 = byte1 >> 4;
				byte1 = byte1 | 0xF0;
				byte2 = byte2 << 4;
				byte2 = byte2 | 0x0F;
				rbyte1 = byte1 & byte2;  //rbyte1 has first pixel of RGB dualbyte
					
				byte1 = startofdma[k];
				k++;
				byte2 = startofdma[k];
				k++;
				byte1 = byte1 >> 4;
				byte1 = byte1 | 0xF0;
				byte2 = byte2 << 4;
				byte2 = byte2 | 0x0F;
				rbyte2 = byte1 & byte2; //rbyte2 has second pixel of RGB dualbyte
			   
			    rbyte1 = rbyte1 & 0xF8;
				Matrix[i] = rbyte1;
				
				

			
			}	//AFTER THIS LOOP, THE MATRIX OF RED IS FILLED

			
			pixaccum = 0;
			rbyte1 = 0;
			Matrix[830] = 0;
			Matrix[829] = 0;
			for(i = (10 * 64); i < (93*64); i++)
			{
				pixaccum = pixaccum + Matrix[i];
			 	if(Matrix[i] > rbyte1)
				{
				 	rbyte1 = Matrix[i];
					h = i;
				}	
				
			}	 //AFTER THIS LOOP, byte1 HAS THE ARRAY INDEX OF THE BRIGHTEST PIXEL
				 //OF MAX VALUE PIXEL IN THE ARRAY
			h = h + (10 * 64);
			i = h % 64;
			j = h / 64;
			i = 64 - i;
			j = 48 - j;

			if((pixaccum < 220000) && (rbyte1 > 80))//220000 based on experimental data
			{
				//send i,j over SPI
				SPI_Write(AT91C_BASE_SPI0, 0, (char)j);
				while(!SPI_IsFinished(AT91C_BASE_SPI0));
				SPI_Write(AT91C_BASE_SPI0, 0, (char)i);
			}


												                                                            
			// POINT BUFFER BACK TO THE RIGHT SPOT
			for(i = 0; i <= 1; i++){
				FbList[i].Current = Buffer;
				FbList[i].Next = (int)&FbList[i+1];
			};
			FbList[i-1].Next = (int)&FbList[0];
			i=0;
			

		};
	};
}
