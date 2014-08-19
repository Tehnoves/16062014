#include <c8051f410.h>     
#include <stdio.h>
#include <string.h>
#include <intrins.h> 
#include <math.h>
#include <stdlib.h>
#include <ctype.h>



#include "F350_FlashPrimitives.h"
unsigned int msek;
unsigned char sek,taut,reset_sek;
bit flag_taut,flag_dop;

 void Timer2_ISR (void) interrupt 5

	{
 	TF2H = 0;
 																//	TR2=0;
	msek++;

	if (flag_taut)
		{
			taut++;
			if ( taut > 10)  
				{   taut =0;
					flag_dop = 1;
					flag_taut = 0;	
				}
		}	
	if ((msek >320) | reset_sek  )										 // 	  &(sek == 0)
		{
			reset_sek = 0;										//	new_av=~new_av;
		//	flag_sek = 0;
			msek = 0;
		sek=1;
		 //  	P03 = ~P03;								// тестовая подсветка
		}
		
 
 	}
void main(void)


{
PCA0MD &= ~0x40;  
 OSCICN    = 0x87;
    IE        = 0xA0;
	 TMR2CN    = 0x04;
	TMR2L     = 0x4a;  //0x4a;   ///0x3e;
	TMR2H     = 0xa0;  //0xa0;   //0X50;			 // b
	TMR2RLH   = 0xa0;  //0X50;
	TMR2RLL   = 0x4a;  //0X3e;   
	     
	TR2 = 1;           // Timer0 enabled
	while (1);

}