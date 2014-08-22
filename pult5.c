/* 
 * File:   pult2.c
 * Author: Козырев С.А.
 *  07.03.14 c:\1\pult2
 * pic16f1847 технологический пульт
 * написание под MPLAB X8C на 
 * PICkit3
 * 
 * 
 * 
 * V1.0 11.03.14
 * V2.0 15.03.14   многопроцессорный обмен  
 * V2.1 16.03.14   разбор команд мастера  и отправка пакета    
 * V2.2 29.03.14
 * V2.2 30.03.14   19:27 pult нужно отразить период 
 * V2.2 31.03.14   15:00 отразили DAC
 * V2.3 31.03.14   таймер для записи в EEPROM
 * 17.06.14
 * 07/08.14 убрали зависания по разбору строки, WDT,
 * 19.08.14 двойная буферизация
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <xc.h> 					// include standard header file

// set Config bits
#pragma config FOSC=INTOSC, PLLEN=OFF, WDTE=OFF, MCLRE=ON,
#pragma config CLKOUTEN=OFF, IESO=OFF, FCMEN=OFF,CP=OFF, CPD=OFF,BOREN=OFF
#pragma config WRT=OFF,STVREN=ON,BORV=LO,LVP=OFF

																				// set Config bits для 16F886
																				//.#pragma config FOSC=INTOSC, PLLEN=OFF, WDTE=OFF, MCLRE=ON,
																				//#pragma config CLKOUTEN=OFF, IESO=OFF, FCMEN=OFF,CP=OFF, CPD=OFF,BOREN=OFF
		
		
																					// для 16F1823
																					//
																				//#pragma config CP=OFF,CPD=OFF,BOR4V=BOR40V,WDTE=OFF
																				//#pragma config WRT=OFF,DEBUG=ON,LVP=OFF,CPD=OFF,FOSC=  INTRC_NOCLKOUT, MCLRE =	ON //BORV=LO,STVREN=ON

																							//  остатки от 16F1822 Microchip 14f_1822_ADC
																							//__CONFIG    _CONFIG1, _LVP_OFF & _FCMEN_ON & _IESO_OFF & _BOR_OFF & _CPD_OFF & _CP_OFF & _MCLRE_ON & 
																							//_PWRTE_ON & _WDT_OFF & _INTRC_OSC_NOCLKOUT
																							//	__CONFIG    _CONFIG2, _WRT_OFF & _BOR21V
																							// Definitions
#define _XTAL_FREQ  16000000        // this is used by the __delay_ms(xx) and __delay_us(xx) functions
#define temp     	-(4000)   

#define ok_     			0x01      // 1
#define bad_crc_     		0x02      // 2
//#define period_     			0x80      // 8
//#define key_     			0x20      // 8

#define	state_	  		0x08											//  есть\нет изменение состояния         999  -  0x3e7    ~ 10 bit      4 0x08 
#define	assig_		    0x100											//  установить изменения состояния по запросу инженерной панели         9 0x100
#define	zapr_	    	0x200												//  запросить (flash инженерной панели)                                10 0x200
//#define															//  запросить БП             0x210                                                                                                           
//#define															//  запросить ключи          0x220                                         ?????????    0x04
//#define															//  запросить частотник      0x240   0x204
//#define															//  запросиь вкл\откл        0x280
																//
#define	power_		0x10												//  есть\нет изменение состояния БП                       5  0x10        0x18
#define	period_		0x80												//  есть\нет изменение состояния ключи                    6  0x80        0x88
#define	key_		0x20												//  есть\нет изменение состояния ключи                    6  0x20        0x28
#define	frec1_   	0x40													//  есть\нет изменение состояния частотник                7  0x40        0x48
#define	frec2_	    0x04													//  есть\нет изменение состояния частотник                3  0x04        0x0c
//#define															//  есть\нет изменение состояния вкл\откл                 8  0x80







//#define ok_          0x01			//
//#define bad_crc_     0x02			//  ош четности
//#define onn          0x04			//  включи
//#define command_ok   0x08       	//  значения присвоены
//#define diag         0x01			//  дай состояние
//#define assigment    0x02			//  присвоить значения




	bit kuku;
	
	
	unsigned char enable,flag,crc_ok,sekon,fist,fl200,fl100,ok,ok3,fl1,fl2,sekond;  //  ,ok4,ok2
	unsigned char ok_command_pult,ok_command_uart;   //  22.08.14
	unsigned char ass_command_pult,ass_command_uart; // проверить на потерю переменной
																							//unsigned char on_command;
    unsigned char power_command_uart;
	unsigned char key_command_uart;
	unsigned char frec1_command_uart;
	unsigned char frec2_command_uart;
	unsigned char state_command_uart;
																							//unsigned char zapr_command;
	unsigned char period_command_uart;
	unsigned char power_command_pult;
	unsigned char key_command_pult;
	unsigned char frec1_command_pult;
	unsigned char frec2_command_pult;
	unsigned char state_command_pult,state_command_uart;
																							//unsigned char zapr_command;
	unsigned char period_command_pult;
	unsigned char taut;
									
	unsigned char  start_taut;
			unsigned char  sek4,msec4;
			unsigned char  msec40;
	unsigned char right,left ,takt,takt2, takt22,takt1;
	unsigned char   *tr_buf,flag_read,*tr_bu,flag_ok,flag_write,*tr_bu3,flag_razborka,flag_usart;
	unsigned int   msec,m100,m200;
   	unsigned char  sek2,tmp;   // sek,
	unsigned char key,key_ok,key_state,flag_xvost,flag_peredacha,flag_zanyato1,flag_zanyato2;   //   ,on_state
	unsigned char  a,a1,a2,a3,a4,a5;						//
														//  a4   // частота 2  flash
														//  a3	 // частота 
														//  a2	 // напряжение ключа
														//  a1   // период ключа  
														//
	unsigned char  a11,a22,a33,a44,left2,right2,a55,m3;						//
														//  a44  // частота 2 значение    текущее значение
														//
														//
														//
	unsigned char  a,a222,a333,a444,a555,j;			//  получено от инженерного пульта
	int a111;													//
//	bit right,left;
union  Crr
	 {
   		unsigned int Int;
   		unsigned char Char[2];
   	 };
	unsigned char 	t12,t13;             // отладочные айты
										 //unsigned char 	ca;	
	unsigned char 	byte_cnt; 
	//unsigned char 	diagnostica;
	union Crr  		Crc_send,Crc1_send,Crc2_send;
	//unsigned char  	buf[40];
	//unsigned char 	buf2[40];
	unsigned char 	buf1[40];
										 //	unsigned char  buf3[40];
 										 //	const unsigned char  tes[] = "#,34,001,+250.1,+12.3,-23.4, +8.927689\r\n"; 
  
																//	unsigned char  buf1[40]="#,29,000,71\r\n"; 	
																//	unsigned char  bu[40]="#,21,288,71,80,42,61,1,61096\r\n"; 				
	unsigned char  	bu[40];
	unsigned char 	temp3[40];
	//unsigned char 	temp2[40];
																// "#,00,001,43868,43868\r\n"
	unsigned char 	i,nn;	
	unsigned char 	i2;
	unsigned char 	txt[7],str[7];   /// char xdata txt[7],str[7];
    unsigned char 	bvminus;
	unsigned char 	decimal;    //  точка
	unsigned char  pusto_pos[]="00,00,00,00,0,";  
	unsigned int crc;
	
	int FastCRC16(char crcData, int crcReg);
	unsigned char diagnostika(void);
	
	
const unsigned int   crc16LUT[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};   
	void ind (void);
	void interrupt Interrupt(void);


	int main(int argc, char** argv);


int FastCRC16(char crcData, int crcReg)
	{  unsigned char table;
		table=((char)(crcReg >> 8)) ^ crcData;
		crcReg=((crcReg << 8) ^ crc16LUT[table]);
		return(crcReg);
	}

void null() {
 		LATAbits.LATA0 = 1;	     			//	GPIO |= (1<<dclk);
  		asm("nop");
  		LATAbits.LATA0 = 0;					//	GPIO &= ~(1<< dclk);
  		asm("nop");									//	asm nop
  		LATAbits.LATA7 = 0;					//	GPIO &= ~(1<< din);
  		asm("nop");									//	asm nop
  }

void edin () {
 		LATAbits.LATA0 = 1;	     			// GPIO |= (1<<dclk);
 		asm("nop"); 
 		LATAbits.LATA0 = 0;					// GPIO &= ~(1<< dclk);
 		asm("nop");									// asm nop
 		LATAbits.LATA7 = 1;					// GPIO |= (1<< din);
  		asm("nop");									//asm nop
}
void minus() {
     null();
      null();
      null();
      null();
      null();
      edin();
      null();
      }

 void zero () {
      edin();
      edin();
      edin();
      edin();
      edin();
      null();
      edin();
 }
 void one() {
      null();
      edin();//null();
      edin();//null();
      null();
      null();// edin();
      null();
      null();// edin();
 }
 void two() {
      edin();
      null();
      edin();
      edin();
      null();
      edin();
      edin();
 }
 void tree() {
      edin();
      edin();//null();
      edin();//null();
      edin();
      null();//edin();
      edin();
      null();//edin();
 }
 void four() {
      null();
      edin();
      edin();//null();
      null();
      edin();
      edin();
      null();//edin();
 }
 void five() {
      edin();
      edin();
      null();
      edin();
      edin();
      edin();
      null();
 }
 void six() {
      edin();
      edin();
      null();//edin();
      edin();
      edin();
      edin();
      edin();//null();
 }
 void seven() {
      null();//edin();
      edin();//null();
      edin();//null();
      edin();//null();
     null();//edin();
      null();
      null();//edin();
 }
 void eight() {
      edin();
      edin();
      edin();
      edin();
      edin();
      edin();
      edin();
 }
 void nine() {
      edin();
      edin();
      edin();//edin();
      edin();
      edin();
      edin();//edin();
      null();//edin();
 }
 void pusto() {
      null();
      null();
      null();
      null();
      null();
      null();
      null();
 }

 void cas(unsigned char qwe) {

      switch (qwe)
           {
          case  0x30: zero();	break;
          case  0x31: one();	break;
          case  0x32: two();	break;
          case  0x33: tree();	break;
          case  0x34: four();	break;
          case  0x35: five();	break;
          case  0x36: six(); 	break;
          case  0x37: seven(); 	break;
          case  0x38: eight();	break;
          case  0x39: nine();	break;
          case  0x20: pusto();	break;
          case  '-': minus();	break;
           }
 }



 void indik (void)
 {
if ((ok)| (ok3))
			{
	       sprintf(str,"%3.3u",a); 
	  	    txt[3] = str[0];
			txt[4] = str[1];
			txt[5] = str[2];
			}

    cas(txt[5]);
    if (bvminus == 1)
        {
                 if (decimal == 1)
                       null();
                 else
                      edin();
        }
        else
            edin();


    if (txt[4] == ' ')
         txt[4] = '0';
    cas(txt[4]);
    null();              // точка
    null();              // точка
    cas(txt[3]);
    null();



    LATAbits.LATA6 = 1;                   // GPIO |= (1<<load);
    asm("nop");									// asm nop
    asm("nop");									// asm nop
    asm("nop");									// asm nop
    asm("nop");									//  asm nop
    LATAbits.LATA6 = 0;                   // GPIO &= ~(1<< load);

 }




														//  a4   // частота 2 текущее значение
														//  a3	 // частота 
														//  a2	 // напряжение ключа
														//  a1   // период ключа  

 
	//**********************************************
	//
	//    передача мастеру
	//
	//**********************************************

unsigned char diagnostika(void)
	{ unsigned char te;
		te=0;
		if (ok_command==1)
			te=te | ok_;
		else
			te &= ~ok_;
			
		if (key_command_pult==1)
			{
			te = te | key_ ;
		
							//((key & 0x02) == 0x02)
			}
	//	else
	//		te &= ~key_;
			
		if (crc !=	Crc2_send.Int )
			te |= bad_crc_;
		//else 
			//te&= ~bad_crc_;
			
		if ((state_command_pult==1) | (state_command_uart==1))
			{
			te = te | state_;
		//	state_command=0;
			}
		//else
			//te &= ~state_;	
			
		if (period_command_pult==1)
			{
			te = te | period_;
			period_command_pult = 0;
			}
		//else
			//te &= ~period_;	
			
		if (power_command_pult==1)
			{
			te = te | power_;
			power_command_pult = 0;
			}
		//else
			//te &= ~power_;		
		
		if (frec1_command_pult==1)
			{
			te = te | frec1_;
			frec1_command_pult = 0;
			}
		//else
			//te &= ~frec1_;	
		if (frec2_command_pult==1)
			{
			te = te | frec2_;
			frec2_command_pult = 0;
			}
		//else
			//te &= ~frec2_;	
			return  (te);	
	}

	//**********************************************
	//
	//  получение команды от мастера
	//
	//**********************************************

void comand( int dia)
	{
		

	

		if (( dia & ok_) == ok_)  ////////////////////// ok
				ok_command = 1;	
		else
			ok_command = 0;		
			
		if (( dia & power_) == power_)  ///////////////// power
				{ass_command = 1;	
				power_command_uart = 1;
				}	
		else
			power_command_uart = 0;	


		if (( dia & key_) == key_)   ///////////////////// key
					{ass_command = 1;
				key_command_uart = 1;
					}	
		else
			key_command_uart = 0;

		if (( dia & frec1_) == frec1_)   /////////////////  frec1
					{
				frec1_command_uart = 1;	
					}
		else
			frec1_command_uart = 0;	
		if (( dia & frec2_) == frec2_)  /////////  frec2
					{ass_command = 1;
				frec2_command_uart = 1;
					}	
		else
			frec2_command_uart = 0;
		//if (( dia & zapr_) == zapr_)
				//zapr_command = 1;	
		//else
			//zapr_command = 0;	
		if (( dia & state_) == state_)  //////////////// state
				state_command_uart = 1;	
		else
			state_command_uart = 0;	
		if (( dia & period_) == period_)  /////// period
					{ass_command = 1;
				period_command_uart = 1;
					}	
		else
			period_command_uart = 0;	
			
		if (( dia & assig_) == assig_)  					// ass
				{ass_command = 1;
				state_command_uart = 1;
				//power_command = 1;
				//key_command = 1;
				//period_command = 1;
				//frec1_command = 1;	
				//frec2_command = 1;	
				}
        else
				{
				ass_command = 0;	
				ok_command = 1;
				}
	
			
			}


	//**********************************************
	//
	//  			ОТВЕТ МАСТЕРУ
	//
	//**********************************************

															//  есть\нет изменение состояния
															//  установить изменения состояния по запросу инженерной панели
															//  запросить flash инженерной панели 
															//  запросить БП                                                                                                                                юд
															//  запросить ключи
															//  запросить частотник
															//  запросиь вкл\откл
															//
															//  есть\нет изменение состояния БП
															//  есть\нет изменение состояния ключи
															//  есть\нет изменение состояния частотник
															//  есть\нет изменение состояния вкл\откл
															
														
														
														
																								
					//										
					// #,23,001,	96,		16,		45,	51,	  1,1		2345\r\n"		
					//			частота*10   dac    f1  f2   on\off		 crc
					//
							
void otv(void)
		{
		unsigned char ij;				
	
		strcpy(buf1,"1,00,");
														//i  = diagnostika(); 
		sprintf(temp3,"%#0.3u,",(int)diagnostika());	
		strcat(buf1,temp3);
														//sprintf(temp3,"%#0.5u,",Crc2_send.Int); 
		 state_command_pult = 1;
		state_command_uart = 1;										//strcat(buf1,temp3);
		if (  (state_command_pult == 1) | (state_command_uart == 1)|(ass_command_ok == 1) )                                   //  | (on_state == 1)
			{
				sprintf(temp3,"%0.3d,",(int)round(((1.0/a11)*10000)));		//  a1   // период ключа  
					strcat(buf1,temp3);
					sprintf(temp3,"%0.2u,",(int)a22);			//  a2	 // напряжение ключа
					strcat(buf1,temp3);
					sprintf(temp3,"%0.2u,",(int)m3);            //  a3	 // частота a33
					strcat(buf1,temp3);
					sprintf(temp3,"%0.2u,",(int)a44); 			//  a4   // частота 2 текущее значение
					strcat(buf1,temp3);
					sprintf(temp3,"%1u,",(int)a5);				// on\off
					strcat(buf1,temp3);
					
					state_command_pult = 0;
					state_command_uart = 0;
				//	on_state_pult = 0;
				ass_command_ok =0;
			}
			else					//    unsigned char  pusto_pos[]="00,00,00,00,0,";  
					{	
						strcat(buf1,pusto_pos);	
						ok_command = 1; 
					}
				ok_command = 1; 	
		ij  = strlen(buf1);
  		sprintf(temp3,"%#0.2u,",ij);
		
	    buf1[2] = temp3[0];
		buf1[3] = temp3[1];
		
		Crc1_send.Int = 0;
		for (i =0;i<ij-1;i++)
		 	Crc1_send.Int=FastCRC16(buf1[i], Crc1_send.Int);
		sprintf(temp3,"%#0.5u\n\r",Crc1_send.Int); 
		strcat(buf1,temp3);
		tr_bu = &buf1; 
																	//LATBbits.LATB3= 0;  // включение RS485 на передачу 
			TXREG = *tr_bu;     	// приготовили к передаче  первый байт
																	//ten = 1;
			TXSTAbits.TXEN = 0X01; 
			/*
		if (flag_usart ==0)
		{
		PIE1bits.TXIE=0x00;
			if (flag_xvost)															//
		  	{	// 2
				//if (!flag_peredacha)
					{flag_zanyato1 = 1;
					strcpy(buf,buf1);      //1
			
					flag_zanyato1 = 0;}
				//else
					//{flag_zanyato2 = 1;
					//strcpy(buf2,buf1);		//2
					//flag_zanyato2 =  0;}
			}
		  else
		  	{	// 1
			//	if (flag_peredacha)
					{flag_zanyato2 = 1;
					strcpy(buf2,buf1);   // 2
					flag_zanyato2 = 0;
				//	flag_xvost = ~flag_xvost;
}
				//else
				//	{flag_zanyato1 = 1;
				//	strcpy(buf,buf1);   // 1
				//	flag_zanyato1 = 0;}
			}								//ij=strlen(buf1);
				flag_xvost = ~flag_xvost;	
				//ij=strlen(buf1);
				PIE1bits.TXIE=0x01;	
		}	*/
		}

														//  a4   // частота 2 текущее значение
														//  a3	 // частота 
														//  a2	 // напряжение ключа
														//  a1   // период ключа  
		
		
		
		
		
	/*	
	unsigned char proverka_nomera(void)
	{ unsigned char i;
	if ((((temp3[0]>= '0') & (temp3[0] <='9')) & ((temp3[1] >= '0') & (temp3[3] <= '9')))& ((i =atoi(temp3)) < 40) )
		return (i);
	else 
		return (0);
	}	
	*/
		
	//**********************************************
	//
	//
	//
	//**********************************************

unsigned char razborka2(void)

			{
			unsigned int tem;
			char * r;
			
				tr_buf =bu;
				r = strrchr(tr_buf,',');	
 				strncpy(temp3,r+1,5);	   // neiie?iaaee inoaoie iia ','
				temp3[5] =0;
				crc = atoi(temp3);
				if (crc == 0)
					return (0);
				r = strchr(tr_buf,',');

				strncpy(temp3,r+1,2); // neiie?iaaee inoaoie iinea ','
				
				temp3[2] = 0;
				 nn = atoi(temp3);
				//nn = proverka_nomera();
				if (nn == 0)
					return (0);
				
				
				
     			//nn = atoi(temp3);            // iieo?eee ?enei

				Crc2_send.Int = 0;
				crc_ok =0;
  				for (i =0;i<nn-1;i++)
		 				Crc2_send.Int=FastCRC16(tr_buf[i], Crc2_send.Int);
				if (crc !=	Crc2_send.Int)
					return (0);
				crc_ok = 1;
				
			
				r = strchr(r+1,',');
				strncpy(temp3,r+1,3);
				temp3[3] = 0;
				comand(atoi(temp3));
								//	comand(tem);

		//	if (ass_command==1)
				{
						ass_command_ok == 0;
					 	r = strchr(r+1,',');


					//	if (period_command_uart == 1)
							{
								strncpy(temp3,r+1,3);   /////???????????????????????????????????? 3 | 2~~~~~~~~
								temp3[3] = 0;
								a111 = atoi(temp3);					//  a1   // период ключа  
								a111 = (int)round((1.0/a111)*10000);
								period_command_uart =1;
								//if ((a111 != a11) )
									{
										//	eeprom_write(2,a111);
										//	a11 = a111;
										//	a1 = a11;
										ass_command_ok == 1;
									}	
							}	
				 																										//	i=osn_chastota;
					
						r = strchr(r+1,',');
					//	if (power_command_uart == 1)
								{
									strncpy(temp3,r+1,2);
									temp3[2] = 0;
									a222 = atoi(temp3);	           //  a2	 // напряжение ключа
									//if (a222 != a22)
										{
											//	eeprom_write(3,a222);
											//a22 = a222;
											//	a2 = a22;
											ass_command_ok == 1;
										}	
								}
					
						r = strchr(r+1,',');
				//		if (frec1_command_uart == 1)
							{	 	
								strncpy(temp3,r+1,2);
								temp3[2] = 0;
								a333 = atoi(temp3);	 
								//if (a333 != a33)
										{
										//	eeprom_write(4,a333);
										//	a33 = a333;
										//	a3 = a33;
											ass_command_ok == 1;
										}	
								//  a3	 // частота 
							}																											//i=vsp_period;
			
						
						r = strchr(r+1,',');
				//		if (frec2_command_uart == 1)
							{	 	
								strncpy(temp3,r+1,2);
								temp3[2] = 0;
								a444 = atoi(temp3);	  
								//if (a444 != a44)
										{
										//	eeprom_write(5,a444);
										//	a44 = a444;
										//	a4 = a44;
												ass_command_ok == 1;
										}		
								//  a4   // частота 2 текущее значение   
							}
						r = strchr(r+1,',');			
				///		if (key_command_uart == 1)
							{	 
					
								strncpy(temp3,r+1,1);
								temp3[1] = 0;
								a555 = atoi(temp3);	                   // on\off
								//if (a555 != a55)
										{
											//	eeprom_write(3,a222);
											//a55 = a555;
											//	a2 = a22;
											ass_command_ok == 1;
										}	
							}	
						ass_command = 1;
						ok_command = 1;
				
																															//	eeprom_write(0x01,0x055);
				/*
				
			if (takt == 1)
					{
						a = a1;
						decimal = 0;	
						ind();
					}
			else if (takt == 2)
					{
						a = a2;
						decimal = 1;	
						ind();
					}
			else if (takt == 3)
					{
						a = a3;
						decimal = 1;	
						ind();
					}	
			else if (takt == 4)
					{
						a = a4;
						decimal = 1;	
						ind();
					}	*/
}
			otv();
//memset(bu, 0x00, sizeof(bu));
																																		//i=vsp_zazor;
				return (1);	
			}

	void interrupt Interrupt()
{
		if ((PIE3bits.TMR6IE==1)&(PIR3bits.TMR6IF == 1 ))  //////////////////// ВТОРОЙ ТАЙМЕР	
		{
			PIR3bits.TMR6IF = 0;
				CLRWDT();
		}

		if (PIR3bits.TMR4IF == 1 )  //////////////////// ВТОРОЙ ТАЙМЕР
		{  PIR3bits.TMR4IF = 0;
			if (start_taut == 1)
			{
			msec4++;
			if (msec4 >= 100)
				{
									//	flsek = 1;
									//	sek++;
							
				//	if ((fist) | (sekond))
						{sek4++;
							if (sek4 >=3) 
								{//	fist = 0;
									sek4 = 0;
									taut = 1;
									start_taut = 0;
									
								}
						}
					msec4 = 0;
				}
			}	
		}

	if (PIR1bits.TMR2IF == 1 )  //////////////////// ВТОРОЙ ТАЙМЕР
		{  PIR1bits.TMR2IF = 0;
			msec++;
			if (fl100)
				{
				m100++;
	   			if (m100 >=10)
	   				{	m100 = 0;
						fl1 = 1;     //есть 100мсек
					}
				}
	        if (fl200)
	   		   {	m200++;
	   			    if (m200 > 50)     // есть 1 сек
	   				   {	m200 = 0;
								//	fl200 = 0;
							fl2 = 1;
						}
				}
			if (msec >= 100)
				{
									//	flsek = 1;
									//	sek++;
							
					if ((fist) | (sekond))
						{sek2++;
							if (sek2 >= 5) 
								{//	fist = 0;
									sek2 = 0;
									fl100 = 1;  // считаем быстро
									fl200 = 0;  // ситаем медленно
								}
						}
					msec = 0;
				}
		}

	if (INTCONbits.IOCIF )  ////////////////////// СМЕНА СОСТОЯНИЯ
		{
										//IOCBFbits.IOCBF0 = 0;
			 if (IOCBFbits.IOCBF0 == 1)
					{
						if (IOCBNbits.IOCBN0 == 1)
							{
								IOCBNbits.IOCBN0 = 0;
								IOCBPbits.IOCBP0 = 1;	
								right = 0;	
							}
						else if (IOCBPbits.IOCBP0 == 1)
							{ 
								IOCBNbits.IOCBN0 = 1;
								IOCBPbits.IOCBP0 = 0;		
								right = 1;
								right2 = 1;
							}
						IOCBFbits.IOCBF0 = 0;
					}
			if (IOCBFbits.IOCBF3 == 1)
					{
						if (IOCBNbits.IOCBN3 == 1)
							{
								IOCBNbits.IOCBN3 = 0;
								IOCBPbits.IOCBP3 = 1;		
								left = 0;
							}
						else if (IOCBPbits.IOCBP3 == 1)
							{ 
								IOCBNbits.IOCBN3 = 1;
								IOCBPbits.IOCBP3 = 0;		
								left = 1;
								left2 = 1;
							}
						IOCBFbits.IOCBF3 = 0;
					}
			if (IOCBFbits.IOCBF4 == 1)
					{
						if (IOCBNbits.IOCBN4 == 1)
							{
								IOCBNbits.IOCBN4 = 0;
								IOCBPbits.IOCBP4 = 1;		
								takt1 = 1;
							}
						else if (IOCBPbits.IOCBP4 == 1)
							{ 
								IOCBNbits.IOCBN4 = 1;
								IOCBPbits.IOCBP4 = 0;		
								takt2 = 1 ;
							}
						IOCBFbits.IOCBF4 = 0;
					}
			 if (IOCBFbits.IOCBF5 == 1)
					{
						if (IOCBNbits.IOCBN5 == 1)
							{
								IOCBNbits.IOCBN5 = 0;
								IOCBPbits.IOCBP5 = 1;	
								key_state = 0;
							key_ok = 1;
							}
						else if (IOCBPbits.IOCBP5 == 1)
							{ 
								IOCBNbits.IOCBN5 = 1;
								IOCBPbits.IOCBP5 = 0;		
								key_state = 1;
							}
						IOCBFbits.IOCBF5 = 0;
					}		
									//IOCBPbits.IOCBP = 0;

									//	IOCBNbits.IOCBN == 1
									//IOCBFbits.IOCBF0 = 0;
		}
 /*    if (PIR1bits.TMR1IF)  // there is only one interrupt vector so you should check to verify what caused the interrupt

		{
		
			TMR1L=(0xff & temp);
			TMR1H =(0xff & (temp >> 8));  
 			flag = 1;
//	tak();																	//	PORTBbits.RB2=~PORTBbits.RB2;
            PIR1bits.TMR1IF=0;        												// ...and then clear the interrupt flag before exiting
		}*/
    if (PIR1bits.TXIF) 	////////////////////// USART передатчик
			{   
																				//if (flag_write == 1)
			  // t12 = *tr_bu;
 				if (*tr_bu != 0)  // && (TXSTA.bits.TRMT == 0)
	
				{
			    if (*tr_bu != 0x00)
					{
																	//	t12 = *tr_bu;	
																	//TXREG = 0x53;	
						TXREG = *tr_bu++;  //++
																	//	t13 = *tr_bu;
						
						j++;
						if (j > 0)
							TXSTAbits.TX9D  = 0;		
					}
				else 	
					{
																			//	if (flag_ok == 1)
														//	t12 = *tr_bu;
							TXREG = *tr_bu;

					}
				 }
			
				else if (*tr_bu == 0) 
																		//	if (flag_write == 1) 
						if (TXSTAbits.TRMT==1)
							{
																		//	t12 = *tr_bu;	
																		//	TXREG = *tr_bu;
								flag_write = 0;
								TXSTAbits.TXEN = 0X0; 
								flag_usart = 0;
								 state_command_pult = 0;   //////////////////////////// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
																		///////?????????????????????LATAbits.LATA4  = 0; 	// включили приемник
								LATAbits.LATA4  = 0; 	// включили приемник

							}
						else 
							{
							//	if (flag_write == 0)
									{
										//t12 = *tr_bu;	
																//	TXREG = *tr_bu;
									//	flag_write = 0;
																//	TXSTAbits.TXEN = 0X0; 
									}	
							}	
			}
	if (PIR1bits.RCIF)   /////////////////// USART   приемник
			{


				if (byte_cnt ==0)
					{
						tr_bu3[byte_cnt]=RCREG;
																	//t12 =tr_bu3[byte_cnt];

						if (tr_bu3[byte_cnt]=='2')  				// это пульт
							{  	RCSTAbits.ADDEN = 0X00; 			//1= Selects 9-bit reception		
								byte_cnt++;
	  
							}
	   
					}
				else
					{

						tr_bu3[byte_cnt] = RCREG;
																	//t12 = tr_bu3[byte_cnt];
						if ((tr_bu3[byte_cnt] != '\r') &(byte_cnt < 40  ))
							{byte_cnt++;
							}
						else
							{	tr_bu3[++byte_cnt] = 0;
								flag_read = 1;
								RCSTAbits.ADDEN = 0X01;  			//1= Selects 9-bit reception		
								byte_cnt = 0;
								LATAbits.LATA4  = 1; 				// включили передатчик
								TXSTAbits.TX9D  = 1;	
								j = 0;
									
								tr_bu = tr_buf;   					//&buf;  // буфер передатчика
																	//t12   = *tr_bu;	
									flag_usart = 1;	
									
						//			razborka2();
									
									
									/*
								flag_peredacha= ~flag_peredacha;
								
								if (flag_peredacha)
										if (!flag_zanyato1)
											{																//  		   !flag_peredacha	   buf		  (1)	перезапись		|  передача    buf2
												tr_buf = &buf;													//			    flag_peredacha	   buf2		  (0)					|			   buf
																		///		flag_peredacha= ~flag_peredacha;								//																	|
											}																//																	|
										else																//			  flag_zanyato1		   									|
											tr_buf = &buf2;													//			  flag_zanyato2
		 
								else																		//
										if (!flag_zanyato2)													//
											{																//			   flag_xvost		xvost	 	buf
												tr_buf = &buf2;												//			  !flag_xvost		 			buf2
																		///		   	flag_peredacha= ~flag_peredacha;							//
											}																//
										else																//
											tr_buf = &buf;	
											
											//tr_buf = &buf;	
											tr_bu = tr_buf; 
																	//LATBbits.LATB3= 0;  // включение RS485 на передачу 
											TXREG = *tr_bu;     	// приготовили к передаче  первый байт
																	//ten = 1;
											TXSTAbits.TXEN = 0X01; 	// разрешили передачу байта						
											
								*/			
							}								
					}
			}
	}








	void ind (void)
		{	sprintf(str,"%3.3u",a); 
	   		txt[3] = str[0];
			txt[4] = str[1];
			txt[5] = str[2];
		//	txt[0                                                                                                                                                                                                                                                                                                                                                                          ] = '1';
		//	txt[1] = '2';
		//	txt[2] = '3';
		//	txt[3] = '0';
	//		txt[4] = '0';
	//		txt[5] = '0';
		//	txt[6] = '7';
		    indik();
		}

	void initc(void)
		{
				a =  0;
			fist = 0;
			ok =0;
			//ok4 =1;
			ok3 = 0;
			//ok2 = 1;
			right = 1;
			left = 1;
			key = 1;
			takt = 1;	
			sekond = 0;
			sek2 = 0;
			decimal = 1;
			bvminus = 1;
//			ca = 1;	
			takt1 = 0; 
			takt2= 1;
			takt22 = 0;
			key_state = 1;
			key_ok =0;
			start_taut = 0;
			 sek4 = 0;
			 msec4 = 0;
			 taut =0;
			m3 =0;
			flag_usart = 0;
			tmp = eeprom_read(1);	
		if ((tmp  != 0x55))
			{
				eeprom_write(1,0x55);   // код присутствия flash
				eeprom_write(2,8);		// период ключа
				eeprom_write(3,60);		// напряжение ключа
				eeprom_write(4,50);		// частота 1
				eeprom_write(5,50);		// частота 2
				a1 = eeprom_read(2);		// период ключа
				a11 = a1;
				a2 = eeprom_read(3);		// напряжение ключа
				a22 = a2;
				a3 = eeprom_read(4);		// частота 1 
				a33 = a3;
				a4 = eeprom_read(5);		// частота 2
				a44 = a4;	
			}
		else 
			{ 
				a1 = eeprom_read(2);	// период ключа
				if ((a1 > 100) | (a1 <20 ))
					a1 =20 ;
				a11 = a1;
				a2 = eeprom_read(3);	// напряжение ключа
				if ((a2 > 62) | (a2 <30 ))
					a2 =30 ;
				a22 = a2;
				a3 = eeprom_read(4);	// частота 1
				if ((a3 > 80) | (a3  <20 ))
					a3 =20 ;
				a33 = a3;
				a4 = eeprom_read(5);    // частота 2
				if ((a4 > 80) | (a4  <20 ))
					a4 =20 ;
				a44 = a4;
			}
			a = a1;								// период ключа

			decimal = 0;
		}

	/*
	* 
	*/

	int main(int argc, char** argv) {

//byte_cnt = 3;
//j =++byte_cnt;
/*
temp3[0] = '5';
temp3[1]='0';
temp3[2]='0';
temp3[3]=0;

while (1)
{
	a111=atoi(temp3);;
	//a111 = (int)round((1.0/a111)*10000);
}*/
    OSCCONbits.SCS    = 0x02;    //set the SCS bits to select internal oscillator block
    OSCCONbits.IRCF   = 0x0f;   // 16mHz
    OSCCONbits.SPLLEN = 0x00;  // pll dicable

    
     // PORT C Assignments
	PORTA  = 0;
	PORTB  = 0;
	LATA   = 0;
	LATB   = 0;
	ANSELB = 0;
	ANSELA = 0;


    TRISBbits.TRISB0 = 1;	// RB0 = кнопка
    TRISBbits.TRISB1 = 1;	// RB1 = это пириемник RX
    TRISBbits.TRISB2 = 0;	// RB2 = это передатчик TX
    TRISBbits.TRISB3 = 1;	// RB3 = кнопка 
    TRISBbits.TRISB4 = 1;	// RB4   кнопка 
    TRISBbits.TRISB5 = 1;	// RB5 = кнопка
    TRISBbits.TRISB6 = 0;	// RB6 = 
    TRISBbits.TRISB7 = 0;	// RB7 = 



	TRISAbits.TRISA0 = 0;	// RA0 = CLK
    TRISAbits.TRISA1 = 0;	// RA1 = светодиод
    TRISAbits.TRISA2 = 0;	// RA2 = светодиод
    TRISAbits.TRISA3 = 0;	// RA3 = светодиод  
    TRISAbits.TRISA4 = 0;	// RA4 = это управление RX485    0- прием 1- передача
    TRISAbits.TRISA5 = 0;	// RA5 = 
    TRISAbits.TRISA6 = 0;	// RA6 = LOAD
    TRISAbits.TRISA7 = 0;	// RA7 = DIN
    
	//TMR1L=(0xff & temp);
	//TMR1H =(0xff & (temp >> 8));     
	//T1CONbits.T1CKPS = 0x00;					// 00= 1:1 Prescale value
	//T1CONbits.TMR1ON = 0X01;					// включить таймер
	T4CONbits.T4OUTPS =0x0F;
	T4CONbits.T4CKPS =0x03;
	PR4=38;  //40
	T4CONbits.TMR4ON=1;
	
	WDTCONbits.WDTPS = 0x06;   //  00110= 1:2048 (Interval 64 ms typ)
	WDTCONbits.SWDTEN = 0x01;  //  1= WDT is turned on
	
	T6CONbits.T6OUTPS  = 0x0f;  // 1111= 1:16 Postscaler
	T6CONbits.T6CKPS   = 0x03;  // 11= Prescaler is 64
	PR6                = 200;  //40
	T6CONbits.TMR6ON   = 1;	
	
	T2CONbits.T2OUTPS =0x0F;
	T2CONbits.T2CKPS =0x03;
	PR2=38;  //40
	T2CONbits.TMR2ON=1;
	msec = 0;

	BAUDCONbits.BRG16  = 0X01; 		// 1= 16-bit Baud Rate Generator is used	

	SPBRGL=(0xff & 206);                 // 414  19200
	SPBRGH =(0xff & (206 >> 8));   

		
//	SPBRG =  414; //;  //       33  1666*2  206


//	TXSTAbits.CSRC = 0X01;						// Asynchronous mode: Don’t care
	TXSTAbits.TX9  = 0X01;						// 0= Selects 8-bit transmission
	TXSTAbits.SYNC = 0X00;						// 0= Asynchronous mode
	TXSTAbits.BRGH = 0X01;						// 1= High speed
	TXSTAbits.TX9D=1;							// Can be address/data bit or a parity bit
	
												 // RCSTAbits.RX9  = 0X0; 
	 


		PIE3bits.TMR4IE=1; 
	PIE1bits.TMR2IE=1;               // разрешили прерыване таймера 2
	PIE3bits.TMR6IE=1;
	INTCONbits.IOCIE = 1;											//	PIR1=0;
    INTCONbits.PEIE  = 1;          	// Enable peripheral interrupt
    INTCONbits.GIE   = 1;           	// enable global interrupt
	//PIE1bits.TMR1IE  = 0x01;      	// enable the Timer 1 parator interrupt

	PIE1bits.RCIE   = 0x01; 		// разрешение прерывания по приемнику
	PIE1bits.TXIE=0x01; 			//  разрешили прерывание от передатчика
																									//	i = data_eeprom_read(0x01);

	RCSTAbits.CREN = 0X01; 			// 1= Enables receiver 
	RCSTAbits.SPEN = 0X01;			// Asynchronous mode: Don’t care
	RCSTAbits.RX9 = 0X01; 			// 1= Selects 9-bit reception		
	RCSTAbits.ADDEN	=1;
	
	flag_razborka = 0;
									//	tr_bu = &master;
	
//	TXSTAbits.TXEN = 0X01; 			// 1= Transmit enabled
	IOCBNbits.IOCBN0 = 1;
	IOCBNbits.IOCBN3 = 1;
    IOCBNbits.IOCBN4 = 1;
	IOCBNbits.IOCBN5 = 1;

	initc();
		
		ind();
		LATAbits.LATA1 = 0;	
		LATAbits.LATA3 = 1;	
		LATAbits.LATA2 = 1;	
		tr_bu3 = &bu;			
		while (1)
			{
			if (flag_read == 1)
				{
					razborka2();	
					flag_read = 0;
			}
				else
				state_command_uart = 1;    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

					/*
				//while(1)
			memset(bu, 0x00, sizeof(bu));
			//for (i=0;i<40;i++)
			//		bu[i]  = 0;
	
			LATAbits.LATA4  = 0; 	// включили приемник
			tr_bu3 = &bu;			// буфер приема
			flag_read = 0;			// обнулили флаг окончания передачи
		//	while (flag_read != 1);
		*/
					asm("nop");

				
{				//	otv();
/*
					tr_bu = &buf;  // буфер передатчика
					t12   = *tr_bu;
																//j=*tr_bu;
																//j++;
																//t12 = *tr_bu;
					flag_write = 1; // флаг конца передачаи
					flag_ok    = 1;
																						//	 TXSTAbits.TXEN = 0X01; 
					flag_razborka  = 0;  
					TXSTAbits.TX9  = 1;
					j = 0;
					LATAbits.LATA4 = 0; // включение RS485 на передачу 
					TXREG = *tr_bu;     // приготовили к передаче  первый байт
					TXSTAbits.TXEN = 0X0; // разрешили передачу байта

				

																				//PORTBbits.RB3=0;  // это приемо передатчик
																				//LATBbits.LATB3 = 0;   // это прием
				while (flag_read != 0);    // ждем конца передачи

		//for (i=0;i<200;i++)
		//	asm("nop");
			*/	
}			
			
					if ((takt1==1) & (takt22 == 0))  // мы нажали клавишу и ее еще не обрабатывали
													// идет обработка вверх-низ			
						{
						takt++;
						if (takt == 5)
							takt =1;
						takt22 = 1;   // мы ее сейчас обработаем
						takt2 = 0;    // готовимся отпустить ее
						if (takt == 1)
							{
								LATAbits.LATA1 = 0;	
								LATAbits.LATA3 = 1;	
								LATAbits.LATA2 = 1;	
								a4 =a;												// частота 2
								if (a4 != a44)
									{
										eeprom_write(5,a4);                       // частота 2  
										a44 = a4;									// частота 2
										state_command_pult = 1;
										period_command_pult = 1;
									}
								a = a1;												// период ключа
								decimal = 0;										// точка
								ind();
							}		
						else if (takt ==2)
							{
								LATAbits.LATA1 = 1;	
								LATAbits.LATA3 = 0;	
								LATAbits.LATA2 = 0;	
								a1 = a;										// период ключа
								if (a1 != a11)
									{
										eeprom_write(2,a1);					// период ключа
										a11 = a1;							// период ключа
										state_command_pult = 1;
										power_command_pult = 1;
									}
					
								a = a2;							// напряжение ключа
								decimal = 1;					// нет точка
								ind();
							}
						else if (takt == 3)
							{
								LATAbits.LATA1 = 0;	
								LATAbits.LATA3 = 0;	
								LATAbits.LATA2 = 1;	
								a2 = a;							// напряжение ключа
								if (a2 != a22)
									{
										eeprom_write(3,a2);		// напряжение ключа
										a22 = a2;				// напряжение ключа
										state_command_pult = 1;
										frec1_command_pult = 1;
									}	
					
								a = a3;							// частота 1 
								decimal = 1;					// нет точка
								ind();
							}
						else if (takt == 4)
							{
								LATAbits.LATA1 = 1;	
								LATAbits.LATA3 = 1;	
								LATAbits.LATA2 = 0;		
								a3 = a;							// частота 1 
								if (a3 != a33)
									{
										eeprom_write(4,a3);    // частота 1 
										a33 = a3;              // частота 1 
										state_command_pult = 1;
										frec2_command_pult = 1;
									}
								a = a4;							// частота 2
								decimal = 1;					// нет точка
								ind();
							}
						}
					else if ((takt2 ==1 ) & (takt22 == 1)) 
						{
						
							takt1 = 0;
							takt22 = 0;
						}
					
				asm("nop");
				
				if (((right== 1) & (right2 ==1)) | ((left== 1) & (left2 ==1)))     // обрабатываем вправо-влево !!!!!!!!! определяем время и место записи в EEPROM
																					// необходимо дождаться отпускания клавиши и 2сек и писать в EEPROM
					{
						right2 = 0;   // это неправильно !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! организовать таймер на 2сек
						left2 = 0;
						start_taut =1;
						
					}
				
					if (taut == 1)
						{if (takt ==1 )
							{		
									state_command_pult = 1;
									period_command_pult = 1;
									a1 = a;
									a11 = a1;
									eeprom_write(2,a1);					// период ключа
							}
						else if (takt ==2 )
							{
									state_command_pult = 1;
									power_command_pult = 1;
									a2 = a;
									a22 = a2;
									eeprom_write(3,a2);					// период ключа
							}
						else if (takt ==3)
							{
									state_command_pult = 1;
									frec1_command_pult = 1;
									a3 = a;
									a33 = a3;
									eeprom_write(4,a3);					// период ключа
							}
						else if (takt ==4)
							{
									state_command_pult = 1;
									frec2_command_pult = 1;
									a4 = a;
									a44 = a4;
									eeprom_write(5,a4);					// период ключа
							}
							taut  = 0;
						}
	
										//****************************
										//
										// движение вправо
										//
										//****************************
		  if ((right == 0) && (fist==0))  //нажали больше идем медленно
		  	{
			 fl100 = 0;
			 fl200 =1;
			 fist = 1;
			 start_taut = 0;
			 sek4 = 0;
			 msec4 = 0;
			 
			}
	
		if ((right == 1)	& (fist))   //отжали больше идем 
			{  fist = 0;
				fl100 = 0;	
				fl200 = 0;
				sek2 = 0;
				m3 = 0;
			}	
									//************************************
									//
									// движение влево
									//
									//************************************

		  if  ((left == 0) & (sekond==0))   //нажали меньше идем медленно
			 {
				fl100 = 0;
				fl200 =1;       // пошли медленно
				sekond = 1;
				start_taut = 0;
				sek4 = 0;
				msec4 = 0;
			}
	
		if ((left == 1)	& (sekond==1))     //отжали все сбросили
			{   sekond = 0;
				fl100 = 0;	
				fl200 = 0;
				sek2 = 0;
				m3 = 0;
			}//                       или медленно или быстро				
 		if ((!right) &(((fl100)   & (fl1)) | ((fl2))))   // нажали больше и 
	   		{
										
			  	{
			a++;                                                     

			if (takt == 1)
				{
					decimal = 0;
					if (a > 100)
						a = 20;
				}
			else if (takt == 2)
				{
					decimal = 1;
					if ( a >62)
						a = 30;
				}
			else if (takt == 3)
				{
					m3 = 2;
					decimal = 1;
					if ( a > 80)
						a = 20;
				}
			else if ( takt == 4)
				{
					decimal = 1;
					if ( a > 80)
						a = 20;
				}

			ind();
			fl2 = 0;
			fl1 = 0;
			}
			}

	   else if 	((!left) &(((fl100)   & (fl1)) | ( (fl2))))   //(fl200) &
			 {
																  	{
			a--;
				if (takt == 1)
				{
					decimal = 0;
					if (a < 20)
						a = 100;
				}
			else if (takt == 2)
				{
					decimal = 1;
					if (a < 30)
						a = 62;
				}
			else if (takt == 3)
				{
					m3 = 1;
					decimal = 1;
					if ( a < 20)
						a = 80;
				}
			else if ( takt == 4)
				{
					
					decimal = 1;
					if ( a < 20)
						a = 80;
				}

		
			ind();
			
			fl2 = 0;
			fl1 = 0;
			}
			}
			
		//*******************************************
		//
		// присвоение полученноко по USART
		//
		//*******************************************


		{	if ((state_command_uart ==1 ) )
				{ ok_command_pult = 0;
					state_command_pult = 1;
				}
			if ((period_command_uart ==1 ) & (ass_command_uart==1))
				{ 
					if (takt != 1)
					// /*	
					{ 
					
						if ((a111 != a11) )       // a11   текущее
							{                                     // a1  flash
								eeprom_write(2,a111);                 // a111 получено
								a11 = a111;
								a1 = a11;
								
							}	
									
					}
					else    //*/
						{	
							if (a111 != a)
							a = a111;
							eeprom_write(2,a111);
							ind();
							start_taut =1;
							
						}
				ok_command_pult = 1;
				state_command_pult = 1;		
					
				}
				if ((a111 != a11) & (state_command_uart = 1) & (ok_command_uart = 0))
					{
						assign_command_pult = 1;
						period_command_pult = 1;
					}
				if ((a111 == a11) & (state_command_uart = 1) & (ok_command_uart = 1))
					{
						assign_command_pult = 0;
						period_command_pult = 0;
					}
				/*
				if (power_command_uart == 1)
							{	
				
								if ((takt != 2) & (a22 != a222))
									{
										a22 = a222;
										a2 = a222;
										eeprom_write(3,a2);
										if (a2 != a22)
											{
														//eeprom_write(3,a2);    // частота 1 
												a22 = a2;              // частота 1 
											}
									}
								else if (a22 != a222)
									{
										ind();
										a = a222;
										start_taut =1;
										/*
										if (a2 != a22)
											{
													//eeprom_write(3,a2);    // частота 1 
												a22 = a2;              // частота 1 
											}	
									}
								power_command_uart = 0;	
							}	
				
			if (frec1_command_uart == 1)
				{  
					if (takt != 3)
					/*	{
							a = a333;
							ind();
							a3 = a333;
							if (a3 != a33)
								{
									//eeprom_write(4,a3);    // частота 1 
									a33 = a3;              // частота 1 
								}
						}
					else
						{
							a3 = a333;
							if (a3 != a33)
							{
								//eeprom_write(4,a3);    // частота 1 
								a33 = a3;              // частота 1 
							}	
						} 
					frec1_command_uart = 0;	
				}		
			if (frec2_command_uart == 1)
				{  
					if  (takt != 4)
						/*{
							a = a444;
							ind();
							a4 = a444;
							if (a4 != a44)
								{
									//eeprom_write(5,a4);    // частота 1 
									a44 = a4;              // частота 1 
								}
						}
					else   
						{
							a4 = a444;
							if (a4 != a44)
							{
								//eeprom_write(5,a4);    // частота 1 
								a44 = a4;              // частота 1 
							}	
						}
					frec2_command_uart = 0;	
				}			
		
	if ( (key_command_uart == 1))   //  пришло по сети
		{
			if ( a55 != a555)
				a55 = a555;
			if ( a55 == 1 )
				{
					//	LATBbits.LATB6 = 0;
					key = 2;
					//key_ok = 1;
				}
			else  
				{
					//  LATBbits.LATB6 = 1;	
 					key = 1;
					//key_ok = 1;
				}
				//
				//
			key_command_uart = 0;	
		}
	if (key_ok == 1)  // нажали кнопку
		{
			if (key_state == 0)
				{
					key++;
					if ((key & 0x01) == 0x01)
						{
							// вкл сеть  LATBbits.LATB6 = 0;	
							key_state = 1;
							a5 = 1;
						}
					else
						{
							// откл сеть  LATBbits.LATB6 = 1;	
							key_state = 1;
							a5 = 0;
						}
				}
			key_ok = 0;	
		}*/
	
				
		}
		}



 	 return (EXIT_SUCCESS);
	}
// union both {
// unsigned char byte;
// struct {
// unsigned b0:1, b1:1, b2:1, b3:1, b4:1, b5:1, b6:1, b7:1;
// } bitv;
// } var;
// This allows you to access byteas a whole (using var.byte), or any bit within that 
// variable independently (using var.bitv.b0through var.bitv.b7).


//   #define testbit(var, bit) ((var) & (1 <<(bit)))
//   #define setbit(var, bit) ((var) |= (1 << (bit)))
//   #define clrbit(var, bit) ((var) &= ~(1 << (bit)))