//-----------------------------------------------------------------------------
// F350_FlashPrimitives.c
//-----------------------------------------------------------------------------
// Copyright 2004 Silicon Laboratories, Inc.
//
// This program contains several useful utilities for writing and updating
// FLASH memory.
//
// AUTH: BW & GP
// DATE: 21 JUL 04
//
// Target: C8051F35x
// Tool chain: KEIL C51 7.06
//
// Release 1.0
//

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------

#include "F350_FlashPrimitives.h"
#include <c8051F350.h>

//-----------------------------------------------------------------------------
// Structures, Unions, Enumerations, and Type Definitions
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------

// FLASH read/write/erase routines
void FLASH_ByteWrite (FLADDR addr, char byte);
unsigned char FLASH_ByteRead (FLADDR addr);
void FLASH_PageErase (FLADDR addr);
 void OSCILLATOR_Init (void);
//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// FLASH Routines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// FLASH_ByteWrite
//-----------------------------------------------------------------------------
//
// This routine writes <byte> to the linear FLASH address <addr>.
//
void FLASH_ByteWrite (FLADDR addr, char byte)
{
   bit EA_SAVE = EA;                   // preserve EA
   char xdata * data pwrite;           // FLASH write pointer

   EA = 0;                             // disable interrupts

   // change clock speed to slow, then restore later

   VDM0CN = 0x80;                      // enable VDD monitor


   RSTSRC = 0x02;                      // enable VDD monitor as a reset source

   pwrite = (char xdata *) addr;

   FLKEY  = 0xA5;                      // Key Sequence 1
   FLKEY  = 0xF1;                      // Key Sequence 2
   PSCTL |= 0x01;                      // PSWE = 1


   VDM0CN = 0x80;                      // enable VDD monitor


   RSTSRC = 0x02;                      // enable VDD monitor as a reset source

   *pwrite = byte;                     // write the byte

   PSCTL &= ~0x01;                     // PSWE = 0

   EA = EA_SAVE;                       // restore interrupts
}

//-----------------------------------------------------------------------------
// FLASH_ByteRead
//-----------------------------------------------------------------------------
//
// This routine reads a <byte> from the linear FLASH address <addr>.
//

unsigned char FLASH_ByteRead (FLADDR addr)
{
   bit EA_SAVE = EA;                   // preserve EA
   char code * data pread;             // FLASH read pointer
   unsigned char byt;

   EA = 0;                             // disable interrupts

   pread = (char code *) addr;

   byt = *pread;                      // read the byte

   EA = EA_SAVE;                       // restore interrupts

   return byt;
}
  
//-----------------------------------------------------------------------------
// FLASH_PageErase 
//-----------------------------------------------------------------------------
//
// This routine erases the FLASH page containing the linear FLASH address
// <addr>.
//
void FLASH_PageErase (FLADDR addr)
{
   bit EA_SAVE = EA;                   // preserve EA
   char xdata * data pwrite;           // FLASH write pointer

   EA = 0;                          // disable interrupts
   // change clock speed to slow, then restore later

   VDM0CN = 0x80;                      // enable VDD monitor


   RSTSRC = 0x02;                   // enable VDD monitor as a reset source

   pwrite = (char xdata *) addr;

   FLKEY  = 0xA5;                   // Key Sequence 1
   FLKEY  = 0xF1;                   // Key Sequence 2
   PSCTL |= 0x03;                   // PSWE = 1; PSEE = 1


   VDM0CN = 0x80;                      // enable VDD monitor


   RSTSRC = 0x02;                   // enable VDD monitor as a reset source
   *pwrite = 0;                     // initiate page erase

   PSCTL &= ~0x03;                  // PSWE = 0; PSEE = 0

   EA = EA_SAVE;                    // restore interrupts
}


/*
void main(void)
{
	FLASH_ByteWrite (0x4000,0x5a);
	FLASH_ByteWrite (0x4001,0x5a);
	FLASH_ByteWrite (0x4004,0x5a);
FLASH_ByteWrite (0x4005,0x5a);


}*/
//-----------------------------------------------------------------------------
// Timer2_Init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : 
//   1)  int counts - calculated Timer overflow rate
//                    range is postive range of integer: 0 to 32767
//
// Configure Timer2 to 16-bit auto-reload and generate an interrupt at 
// interval specified by <counts> using SYSCLK as its time base.
//
//-----------------------------------------------------------------------------
/*
void Timer2_Init (int counts)
{
   TMR2CN  = 0x00;                     // Stop Timer2; Clear TF2;
                                       // use SYSCLK/12 as timebase
   CKCON   = 0x30;                     // Timer2 clocked based on SYSCLK

   TMR2RL  = -counts;                  // Init reload values
   TMR2    = 0xffff;                   // Set to reload immediately
   ET2     = 1;                        // Enable Timer2 interrupts
   TR2     = 1;                        // Start Timer2
} */

//-----------------------------------------------------------------------------
// Interrupt Service Routines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Timer2_ISR
//-----------------------------------------------------------------------------
//
//This routine changes the state of the LED whenever Timer2 overflows.
//
//-----------------------------------------------------------------------------
/*
void Timer2_ISR (void) interrupt 5
{
   TF2H = 0;                           // Clear Timer2 interrupt flag
   LED = ~LED;                         // Change state of LED
} */

/*
void RTD (void)
{


unsigned char addr;




RTC0ADR = 0x0e;
RTC0DAT = 0x00;
while((RTC0ADR & 0x80) == 0x80);
RTC0ADR = 0x0f;

for (addr =0; addr < 64; addr++)
{
	RTC0ADR |= 0x80;
while((RTC0ADR & 0x80) == 0x80);
//ram_data[addr]=RTC0DAT ;

}
 
}	  */
 /*
void RTD1 (void)
{


unsigned char addr;




while((RTC0ADR & 0x80) == 0x80);

	RTC0ADR  = 0x00;
	RTC0ADR |= 0x80;

	while((RTC0ADR & 0x80) == 0x80);
		ram_da[0] = RTC0DAT ;


    while((RTC0ADR & 0x80) == 0x80);
       RTC0ADR  = 0x01;
       RTC0ADR |= 0x80;

    while((RTC0ADR & 0x80) == 0x80);
       ram_da[1] = RTC0DAT ;


    while((RTC0ADR & 0x80) == 0x80);
       RTC0ADR  = 0x02;
       RTC0ADR |= 0x80; 

    while((RTC0ADR & 0x80) == 0x80);
       ram_da[2] = RTC0DAT ;


while((RTC0ADR & 0x80) == 0x80);
RTC0ADR = 0x03;
RTC0ADR |= 0x80;
while((RTC0ADR & 0x80) == 0x80);
ram_da[3] =RTC0DAT ;

;
while((RTC0ADR & 0x80) == 0x80);
RTC0ADR = 0x04;
RTC0ADR |= 0x80;
while((RTC0ADR & 0x80) == 0x80);
ram_da[4] =RTC0DAT ;


while((RTC0ADR & 0x80) == 0x80);
RTC0ADR = 0x05;
RTC0ADR |= 0x80;
while((RTC0ADR & 0x80) == 0x80);
ram_da[5] =RTC0DAT ;
RTC0ADR = 0x06;
RTC0ADR |= 0x80;
while((RTC0ADR & 0x80) == 0x80);
ram_da[6] =RTC0DAT ;


RTC0ADR = 0x07;
RTC0ADR |= 0x80;
while((RTC0ADR & 0x80) == 0x80);
ram_da[7] =RTC0DAT ;


}
*/
/*
void add1(char l)
{
    k1.two[3] = k1.two[2];
	k1.two[2] = 0;	
	k1.one += rez.tiki[l];
	k1.one += rez1.tiki[l];    //
	new.tiki[l] = k1.two[3];    // запись результата
}
  */
	/*
void add_data(void)
{
			//	rez.rezult[0]= 0x8f38cb00;
			//	rez.rezult[1]=0x5a3;

			//	rez1.rezult[0]= 0x517d5d00;
			//	rez1.rezult[1]=0x1;	

	k1.one = 0;
	add1(3);
	add1(2);
	add1(1);
	add1(0);
	add1(7);
	add1(6);
	add1(5);
	add1(4);
	rez1.rezult[1] = new.rezult[1];
	rez1.rezult[0] = new.rezult[0];				 
}	  */
