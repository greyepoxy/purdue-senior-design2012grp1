/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/

/* TODO Application specific user parameters used in user.c may go here */

/******************************************************************************/
/* User Function Prototypes                                                    /
/******************************************************************************/

/* TODO User level functions prototypes (i.e. InitApp) go here */

void InitApp(void);         /* I/O and Peripheral Initialization */
void Insertion_Sort(int);
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


