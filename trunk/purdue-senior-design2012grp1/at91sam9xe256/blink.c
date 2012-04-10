//-------- Period Blinking Code ------

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

#include "header files/pit.h"
#include "header files/board.h"
#include "header files/aic.h"

#define CLOCK_FREQ  961097            /* Main clock frequency in Hz         */

volatile char tflag = 0;
volatile long ticount = 0;

void wait(void) {
	while(tflag == 0);
	tflag = 0;
}

__irq void PIT_Handler (void)  {
	if (ticount == 100) {
		ticount = 0;
		tflag = 1;
	} else{
	ticount++;}
	*AT91C_AIC_EOICR = AT91C_BASE_PITC->PITC_PIVR;
}

int main (void) {

AT91C_BASE_PIOA -> PIO_OER = AT91C_PIO_PA25;
AT91C_BASE_PIOA -> PIO_SODR = AT91C_PIO_PA25;



PIT_Enable();
PIT_Init(1000,18);


AIC_ConfigureIT(AT91C_ID_SYS,AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE,(void (*)())PIT_Handler);
PIT_EnableIT();
AIC_EnableIT(AT91C_ID_SYS);


while(1) {
  AT91C_BASE_PIOA -> PIO_CODR = AT91C_PIO_PA25;
  wait();
  AT91C_BASE_PIOA -> PIO_SODR = AT91C_PIO_PA25;
  wait();
  }
}
