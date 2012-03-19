//-------- I2C Test Code ------

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

#include "header files/twi.h"
#include "header files/board.h"
#include "header files/aic.h"
#include "header files/pio.h"
#include <AT91SAM9XE256.H>

#define C_Addr	0x3C
#define TWCK 400000


static const Pin pins[] = {PINS_TWI0};


int main (void) {

//Initlizing the output pins for TWI 
PIO_Configure(pins, PIO_LISTSIZE(pins));

//Initilzing the TWI periferial and configuiring it as a master
AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_TWI0;
TWI_ConfigureMaster(AT91C_BASE_TWI0, TWCK, 18432000);


//Writing 0x80 to register 0x02 to set the frames per second to 15
TWI_StartWrite(AT91C_BASE_TWI0, C_Addr, 0x02, 1, 0x80);
while(!TWI_ByteSent(AT91C_BASE_TWI0));
TWI_SendSTOPCondition(AT91C_BASE_TWI0);

//Writing 0x03 to register 0x03 to set the camera to RGB and Black&White.
//Also starts image capture.
TWI_StartWrite(AT91C_BASE_TWI0, C_Addr, 0x03, 1, 0x03);
while(!TWI_ByteSent(AT91C_BASE_TWI0));
TWI_SendSTOPCondition(AT91C_BASE_TWI0);

while(1);
}
