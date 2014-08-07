/* 
 * File:   frec.c
 * Author: Козырев С.А.
 *  21.03.14 c:\03\frec
 * pic16f1847 блок управления Частотником
 * написание под MPLAB X8C на 
 * PICkit3
 * 
 * 
 * 
 * V1.0 21.03.14
 * v2.0 05.04.14  частотник работает 21:00
 * v2.0 06.04.14  07:44
 * 19.06.14
 * 15.07.14
 * 07/08.14 убрали зависания по разбору строки, WDT,
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <xc.h> 					// include standard header file

// set Config bits
#pragma config FOSC=INTOSC, PLLEN=OFF, WDTE=ON, MCLRE=ON,
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
//#define dac_     			0x80      // 8
//#define key_     			0x20      // 8

#define	state_	  		0x08											//  есть\нет изменение состояния         999  -  0x3e7    ~ 10 bit      4 0x08 
#define	assig_		    0x100											//  установить изменения состояния по запросу инженерной панели         9 0x100
#define	zapr_	    	0x200												//  запросить (flash инженерной панели)                                10 0x200
//#define															//  запросить БП             0x210                                                                                                           
//#define															//  запросить ключи          0x220                                         ?????????    0x04
//#define															//  запросить частотник      0x240   0x204
//#define															//  запросиь вкл\откл        0x280
																//
#define	off_		0x10												//  есть\нет изменение состояния БП                       5  0x10        0x18
																//  есть\нет изменение состояния БП                       5  0x10        0x18
#define	on_	        0x20					            	//  есть\нет изменение состояния вкл\откл                 6  0x20        0x28
#define	plus_   	0x40	
#define	minus_		0x80												//  есть\нет изменение состояния ключи                    6  0x80        0x88
												//  есть\нет изменение состояния ключи                    6  0x20        0x28
//#define	frec1_   	0x40													//  есть\нет изменение состояния частотник                7  0x40        0x48
//#define	frec2_	    0x04													//  есть\нет изменение состояния частотник                3  0x04        0x0c
//#define															//  есть\нет изменение состояния вкл\откл                 8  0x80








//#define command_ok   0x08       	//  значения присвоены
//#define diag         0x01			//  дай состояние
//#define assigment    0x02			//  присвоить значения

	//unsigned char	flag_xvost_temper,flag_xvost_adc;
	//unsigned char 	flag_zanyato_adc1,flag_zanyato_adc2,flag_peredacha_adc;
	
	//unsigned char 	flag_zanyato_temper1,flag_zanyato_temper2,flag_peredacha_temper;
	
	
	bit kuku;
	unsigned char selector;
	
	
	unsigned char fist,second,msec;
	unsigned char enable,flag,crc_ok,ass_command,ok_command;
    unsigned char minus_command,plus_command,state_command,on_command,off_command;
	
	unsigned char   *tr_buf,flag_read,*tr_bu,flag_ok,flag_write,*tr_bu3,flag_razborka;
	
   
	unsigned char  a1,a2,a3,a4;						        //  a5   // реверс
														//  a4   // пуск
														//  a3	 // стоп 
														//  a2	 // -
														//  a1   // +  
														//
	unsigned char  a11,a22,a33,a44;					//
														//  a44  // частота 2 значение flash
														//
														//
														//
	unsigned char  j;			//  получено от инженерного пульта
												//
										//	bit right,left;
union  Crr
	 {
   		unsigned int Int;
   		unsigned char Char[2];
   	 };
	unsigned char 	t12,t13;             // отладочные айты
										 //unsigned char 	ca;	
	unsigned char 	byte_cnt; 
	unsigned char 	diagnostica;
	union Crr  		Crc_send,Crc1_send,Crc2_send;
	unsigned char  	buf[40];
										 //	unsigned char  buf3[40];
 										 //	const unsigned char  tes[] = "#,34,001,+250.1,+12.3,-23.4, +8.927689\r\n"; 
  
																//	unsigned char  buf1[40]="#,29,000,71,120,12345\r\n"; 	
																//	unsigned char  bu[40]="#,12,288,1,0,1,0,61096\r\n"; 				
	unsigned char  	bu[40];
	unsigned char 	temp3[40];
	unsigned char 	temp2[40];
																// "#,00,001,43868,43868\r\n"
	unsigned char 	i,nn;	
	unsigned char 	i2;
	unsigned char 	txt[7],str[7];   /// char xdata txt[7],str[7];
   
	
	unsigned int crc;
	
	int FastCRC16(char crcData, int crcReg);
	unsigned char diagnostika(void);



unsigned char kuk;

	
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

	void interrupt Interrupt(void);


	int main(int argc, char** argv);


int FastCRC16(char crcData, int crcReg)
	{  unsigned char table;
		table=((char)(crcReg >> 8)) ^ crcData;
		crcReg=((crcReg << 8) ^ crc16LUT[table]);
		return(crcReg);
	}




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
	if (crc !=	Crc2_send.Int )
			te |= bad_crc_;

	//	else
	//		te &= ~ok_;
		/*	
		if (on_command==1)
			{
			te = te | on_ ;
		
							//((key & 0x02) == 0x02)
			}
	//	else
	//		te &= ~key_;
		if (off_command==1)
			{
			te = te | off_ ;
			}
		if (crc !=	Crc2_send.Int )
			te |= bad_crc_;
	//	else 
	//		te&= ~bad_crc_;
			
		if (state_command==1)
			{
			te = te | state_;
		//	state_command=0;
			}
	//	else
	//		te &= ~state_;	
			
		if (plus_command==1)
			{
			te = te | plus_;
	//		dac_command = 0;
			}
	//	else
	//		te &= ~plus_;	
			
		if (minus_command==1)
			{
			te = te | minus_;
		//	minus_command = 0;
			}
	//	else
	//		te &= ~minus_;		
		*/

			return  (te);	
	}

	//**********************************************
	//
	//  получение команды от мастера
	//
	//**********************************************

void comand( int dia)
	{
		if (( dia & assig_) == assig_)  					// assigment_
				{ass_command = 1;
				plus_command = 1;
				on_command = 1;
				minus_command = 1;
				off_command = 1;
				}
        else
				ass_command = 0;	


	
/*
		if (( dia & ok_) == ok_)
				ok_command = 1;	
		else
			ok_command = 0;		
		if (( dia & plus_) == plus_)
				{ass_command = 1;	
				plus_command = 1;
				}	
		else
			plus_command = 0;	


		if (( dia & on_) == on_)
					{ass_command = 1;
				on_command = 1;
					}	
		else
			on_command = 0;

	
		if (( dia & off_) == off_)
				off_command = 1;	
		else
			off_command = 0;	
		if (( dia & state_) == state_)
				state_command = 1;	
		else
			state_command = 0;	
		if (( dia & minus_) == minus_)
					{ass_command = 1;
				minus_command = 1;
					}	
		else
			minus_command = 0;	
*/
	}


	//**********************************************
	//
	//
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
															
 void otv(void)
		{
		unsigned char ij;
	
		strcpy(buf,"1,00,");
		i  = diagnostika(); 
		sprintf(temp3,"%#0.3u,",(int)diagnostika());	
		strcat(buf,temp3);
		ij  = strlen(buf);
  		sprintf(temp3,"%#0.2u,",(int)ij);
		
	    buf[2] = temp3[0];
		buf[3] = temp3[1];
		
		Crc1_send.Int = 0;
		for (i =0;i<ij-1;i++)
		 	Crc1_send.Int=FastCRC16(buf[i], Crc1_send.Int);
		sprintf(temp3,"%#0.5u\n\r\0",Crc1_send.Int); 
		strcat(buf,temp3);
		//sprintf(temp3,"%#0.5u,",Crc2_send.Int); 
		//strcat(buf,temp3);
	//	if (  (state_command == 1) )
			{
			//		sprintf(temp3,"%0.1u,",a11);    // +
				//	strcat(buf,temp3);
				
				//	sprintf(temp3,"%0.1u,",a22);   // -
				//	strcat(buf,temp3);
				
					
				//	sprintf(temp3,"%0.1u,",a33);   // on
				//	strcat(buf,temp3);
					
					
				//	sprintf(temp3,"%0.1u,",a44);   // off
				//	strcat(buf,temp3);
					
				
			}

	
										//ij=strlen(buf1);
		}
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

				strncpy(temp3,r+1,2);	   // neiie?iaaee inoaoie iinea ','
					   // neiie?iaaee aieiao
				temp3[2] = 0;
				
				nn = proverka_nomera();
				if (nn == 0)
					return (0);
			//	 nn = atoi(temp3);
				
				
     		//	nn = atoi(temp3);            // iieo?eee ?enei

				Crc2_send.Int = 0;
				crc_ok =0;
  				for (i =0;i<nn-1;i++)
		 				Crc2_send.Int=FastCRC16(tr_buf[i], Crc2_send.Int);
			//	if (crc !=	Crc2_send.Int)
			//		return (0);
				crc_ok = 1;
				r = strchr(r+1,',');
				strncpy(temp3,r+1,3);
				temp3[3] = 0;
				comand(atoi(temp3));
				
												//	comand(tem);

		//	if (ass_command==1)
				{
						
					 	r = strchr(r+1,',');
				//		if (on_command == 1)
					 		{
								strncpy(temp3,r+1,1);   /////???????????????????????????????????? 3 | 2~~~~~~~~
								temp3[1] = 0;
								a11 = atoi(temp3);	
								//ass_command == 1;
							}							//i=diagnostica;
						r = strchr(r+1,',');
					//	if (off_command == 1)
					 		{
								strncpy(temp3,r+1,1);   /////???????????????????????????????????? 3 | 2~~~~~~~~
								temp3[1] = 0;
								a22 = atoi(temp3);	
								//ass_command == 1;
							}							//i=diagnostica;
						r = strchr(r+1,',');																						//	i=osn_chastota;
				//	if (plus_command == 1)
					 		{
								strncpy(temp3,r+1,1);   /////???????????????????????????????????? 3 | 2~~~~~~~~
								temp3[1] = 0;
								a33 = atoi(temp3);	
								//ass_command == 1;
							}							//i=diagnostica;
						r = strchr(r+1,',');
					//	if (minus_command == 1)
					 		{
								strncpy(temp3,r+1,1);   /////???????????????????????????????????? 3 | 2~~~~~~~~
								temp3[1] = 0;
								a44 = atoi(temp3);	
								//ass_command == 1;
							}							//i=diagnostica;
						
				 		ass_command = 1;
						ok_command = 1;
				
				}
	//		otv();

																																		//i=vsp_zazor;
				return (1);	
			}

	void interrupt Interrupt()
{

	if ((PIE3bits.TMR6IE==1)&(PIR3bits.TMR6IF == 1 ))  //////////////////// ВТОРОЙ ТАЙМЕР	
		{
			PIR3bits.TMR6IF = 0;
				//CLRWDT();
		}




	if (PIR1bits.TMR2IF == 1 )  //////////////////// ВТОРОЙ ТАЙМЕР
		{PIR1bits.TMR2IF = 0;
		if (fist == 1)
			{
			msec++;
	
	
	   if (msec >= 2)
	   		{
															//	flsek = 1;
															//	sek++;
				/*			
				if ((fist) )
					{sek2++;
						if (sek2 >= 5) 
							{//	fist = 0;
					 			sek2 = 0;
								fl100 = 1;
								fl200 = 0; 
							}
					}*/
				fist = 0;
				second = 1;	
				msec = 0;
			}
			}
		}  
/*
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
								sekond = 1;
							}
						else if (IOCBPbits.IOCBP0 == 1)
							{ 
								IOCBNbits.IOCBN0 = 1;
								IOCBPbits.IOCBP0 = 0;		
								right = 1;
								sekond2 = 1;
							}
						IOCBFbits.IOCBF0 = 0;
					}
		}	*/
											/*   if (PIR1bits.TMR1IF)  // there is only one interrupt vector so you should check to verify what caused the interrupt

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
			   t12 = *tr_bu;
 				if (*tr_bu != 0)  // && (TXSTA.bits.TRMT == 0)
	
				{
			    if (*tr_bu != 0x00)
					{
						t12 = *tr_bu;	
																	//TXREG = 0x53;	
						TXREG = *tr_bu++;  //++
						t13 = *tr_bu;
						
						j++;
						if (j > 0)
							TXSTAbits.TX9D  = 0;		
					}
				else 	
					{
																			//	if (flag_ok == 1)
						{
							t12 = *tr_bu;
							TXREG = *tr_bu;

							flag_write =0;
						}
																		//	else
						{
																		//	TXSTAbits.TXEN = 0X0; 
							flag_write = 0;
						}
					}
				 }
			
				else if (*tr_bu == 0) 
					//if (flag_write == 1) 
						if (TXSTAbits.TRMT==1)
							{
								t12 = *tr_bu;	
																		//	TXREG = *tr_bu;
								flag_write = 0;
								TXSTAbits.TXEN = 0X0; 
								LATBbits.LATB3  = 0; 
									//raborka_who(BP);
							}
						else 
							{
							//	if (flag_write == 0)
									{
										t12 = *tr_bu;	
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
	 					t12 =tr_bu3[byte_cnt];

						if (tr_bu3[byte_cnt]=='5')
							{  	RCSTAbits.ADDEN = 0X00; 			//1= Selects 9-bit reception		
								byte_cnt++;
	  
							}
	   
					}
				else
					{
						
						tr_bu3[byte_cnt] = RCREG;
						t12 = tr_bu3[byte_cnt];
						if ((tr_bu3[byte_cnt] != '\r') &(byte_cnt < 40  ))
							{byte_cnt++;
							}
						else
							{	tr_bu3[++byte_cnt] = 0;
								flag_read = 1;
								  	RCSTAbits.ADDEN = 0X01;  			//1= Selects 9-bit reception		
								byte_cnt = 0;
								LATBbits.LATB3  = 1; 	// включили передатчик
								TXSTAbits.TX9D  = 1;	
								j = 0;
razborka2();
								otv();
								tr_bu = &buf;
							//	tr_bu = tr_buf; 
																	//LATBbits.LATB3= 0;  // включение RS485 на передачу 
											TXREG = *tr_bu;     // приготовили к передаче  первый байт
																	//ten = 1;
											TXSTAbits.TXEN = 0X01; 
							}								
					}
			}
	}




	void initc(void)
		{
		
			a1 =0;
			a2 =0;
			a3 = 0;
			a4 = 0;	
			second = 0;
			fist = 0;
			msec = 0;
		
		}


	/*
	* 
	*/

	int main(int argc, char** argv) {
/*
while (1)
{
		temper = 105;
		a33 = 100;
	
			if ((temper > a33+a33/10) | (temper < a33 - a33/10))
				a33 = temper;


// adcsum = 51;	
// temper=sea(rr);

}
*/
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


    TRISBbits.TRISB0 = 0;	// RB0 = вход
    TRISBbits.TRISB1 = 1;	// RB1 = это пириемник RX
    TRISBbits.TRISB2 = 0;	// RB2 = это передатчик TX
    TRISBbits.TRISB3 = 0;	// RB3 = 
    TRISBbits.TRISB4 = 0;	// RB4   светодиод 
    TRISBbits.TRISB5 = 0;	// RB5 = светодиод
    TRISBbits.TRISB6 = 0;	// RB6 = 
    TRISBbits.TRISB7 = 0;	// RB7 = 



	TRISAbits.TRISA0 = 0;	// RA0 = Реверс
    TRISAbits.TRISA1 = 0;	// RA1 + частота
    TRISAbits.TRISA2 = 0;	//  
    TRISAbits.TRISA3 = 0;	//    
    TRISAbits.TRISA4 = 0;	// RA4 - частота
    TRISAbits.TRISA5 = 0;	//  
    TRISAbits.TRISA6 = 0;	// RA6 стоп  двигатель
    TRISAbits.TRISA7 = 0;	// RA7 пуск двигатель
	
/*	
		LATAbits.LATA7 = 1;	// on
LATAbits.LATA7 = 0; 
LATAbits.LATA6 = 1;   // off
LATAbits.LATA6 = 0; 
LATAbits.LATA7 = 1;	
	while (1)
	{	//LATBbits.LATB5 = 0;
		//LATBbits.LATB5 = 1;
	LATAbits.LATA1 = 0;  //++
		LATAbits.LATA1 = 1;
LATAbits.LATA1 = 0;  //++
	LATAbits.LATA4 = 0;  //--
	LATAbits.LATA4 = 1;
	LATAbits.LATA4 = 0;  //--
	//	LATAbits.LATA0= 0;  // revers
	//	LATAbits.LATA0 = 1;
	//	 //on
	//	LATAbits.LATA7 = 1;
	//	LATAbits.LATA7 = 0;  //++
	//	LATAbits.LATA6 = 0;  //off
	//	LATAbits.LATA6 = 1;
	//	LATAbits.LATA6 = 0;  //++	

	}
*/
	T2CONbits.T2OUTPS  = 0x0F;
	T2CONbits.T2CKPS   = 0x01;     // 500usec
	PR2                = 38;  					//40
	T2CONbits.TMR2ON   = 1;
	//ADCON1bits.ADPREF=0x03;      // VREF+ is connected to internal Fixed Voltage Reference (FVR) module
	
	//TMR1L=(0xff & temp);
	//TMR1H =(0xff & (temp >> 8));     
	//T1CONbits.T1CKPS = 0x00;					// 00= 1:1 Prescale value
	//T1CONbits.TMR1ON = 0X01;					// включить таймер

	T6CONbits.T6OUTPS  = 0x0f;  // 1111= 1:16 Postscaler
	T6CONbits.T6CKPS   = 0x03;  // 11= Prescaler is 64
	PR6                = 200;  //40
	T6CONbits.TMR6ON   = 1;	
	
	
	BAUDCONbits.BRG16  = 0X01; 		// 1= 16-bit Baud Rate Generator is used	

	SPBRGL=(0xff & 206);								// !!!!!!!!!!!!! это 9600 не забудь вернуть 19200
	SPBRGH =(0xff & (206 >> 8));   

		
//	SPBRG =  414; //;  //       33  1666*2  206


//	TXSTAbits.CSRC = 0X01;						// Asynchronous mode: Don’t care
	TXSTAbits.TX9  = 0X01;						// 0= Selects 8-bit transmission
	TXSTAbits.SYNC = 0X00;						// 0= Asynchronous mode
	TXSTAbits.BRGH = 0X01;						// 1= High speed
	TXSTAbits.TX9D=1;							// Can be address/data bit or a parity bit
	
												 // RCSTAbits.RX9  = 0X0; 
	 

	//PIE3bits.TMR6IE=1;     //1= Enables the TMR4 to PR4 Match interrup  
	//PIE3bits.TMR4IE=1;     //1= Enables the TMR4 to PR4 Match interrup  
	//PIE1bits.TMR2IE=1;               // разрешили прерыване таймера 2
	WDTCONbits.WDTPS = 0x06;   //  00110= 1:2048 (Interval 64 ms typ)
	WDTCONbits.SWDTEN = 0x01;  //  1= WDT is turned on
//	INTCONbits.IOCIE = 1;											//	PIR1=0;
    INTCONbits.PEIE  = 1;          	// Enable peripheral interrupt
	PIE3bits.TMR6IE=1;
    INTCONbits.GIE   = 1;           	// enable global interrupt
//	PIE1bits.TMR1IE  = 0x01;      	// enable the Timer 1 parator interrupt
	PIE1bits.TMR2IE=1;               // разрешили прерыване таймера 2
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
	//IOCBNbits.IOCBN0 = 1;
	

	initc();
		
	
		//LATAbits.LATA1 = 0;	
		//LATAbits.LATA3 = 1;	
		//LATAbits.LATA2 = 1;	
		//PIE3bits.TMR6IE=1;     //1= Enables the TMR4 to PR4 Match interrup  

			LATBbits.LATB3  = 0; 	// включили приемник
			tr_bu3 = &bu;			// буфер приема
			flag_read = 0;							//	while (1)
			
			//razborka2();		
				while (1)
			{
			
			//if (flag_read == 1)
			//	{
			//razborka2();
				//flag_read = 0;
				//}
		//		else
		//			state_command_uart = 1;  
			
			
							//while(1)
	//		memset(bu, 0x00, sizeof(bu));
			//for (i=0;i<40;i++)
			//		bu[i]  = 0;
	
  		//	LATBbits.LATB3  = 0; 	// включили приемник
		//	tr_bu3 = &bu;			// буфер приема
		//	flag_read = 0;			// обнулили флаг окончания передачи
		//	while (flag_read != 1);
		
					asm("nop");

				
{					
/*
					otv();
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
					LATBbits.LATB3 = 0; // включение RS485 на передачу 
					TXREG = *tr_bu;     // приготовили к передаче  первый байт
					TXSTAbits.TXEN = 0X0; // разрешили передачу байта
*/
				

																				//PORTBbits.RB3=0;  // это приемо передатчик
																				//LATBbits.LATB3 = 0;   // это прием
	///			while (flag_read != 0);    // ждем конца передачи

		//for (i=0;i<200;i++)
			asm("nop");
				
}			
			
				
					
			asm("nop");

			
		if (second == 1)
			{
					LATAbits.LATA6 = 0;	
					LATAbits.LATA7 = 0;
					second = 0;		
			}
		//*******************************************
		//
		// присвоение полученноко по USART
		//
		//*******************************************

	//	if (ass_command == 1)   //  пришло по сети
	
		{
			if (a1 != a11)
			{
					a1 = a11;
			if ( a1 == 1 )
				{
					//on_command = 1;
					//off_command = 0;
					//a1 = a11;
					//fist = 0;
					LATAbits.LATA7 = 1;	
					fist = 1;
					// нажать +
				}
		//	else
				{
						//on_command = 0;					//a11 = a11;
				//	fist = 0;
				//	LATAbits.LATA7 = 0;	
				}
			}	
		if (a2 != a22)
			{	
				a2 = a22;		
			if ( a2 == 1 )
				{
					//off_command = 1;
					//a2 = a22;
					//sekond = 0;
					LATAbits.LATA6 = 1;	
					fist = 1;
					// ажать -
				}
		//	else
				{
					//off_command = 1;	
					//a2 = a22;
				//	fist = 0;
			//		LATAbits.LATA6 = 0;	
				}	
			}
		if (a3 != a33)
			{
				a3 = a33;
			if (a3 == 1)
				{
					//a3 = a33;
					//tree = 0;
					LATAbits.LATA1 = 1;		
					// пуск
				}
			else
				{
					//a3 = a33;
					//fist = 0;
					LATAbits.LATA1 = 0;	
				}		
			}
		if (a4 != a44)
			{	
				a4 = a44;	
			if (a4 == 1)
				{
					//a4 = a44;
				
					LATAbits.LATA4 = 1;	
					// стоп
				}	
			else
				{
					//a4 = a44;
					//fist = 0;
					LATAbits.LATA4 = 0;	
				}		
			}	
				//
				//
			
				
				//
				//
			//on_command = 0;	
			ass_command =0; 
		}
	
		
				

		}



 	 return (EXIT_SUCCESS);
	}
