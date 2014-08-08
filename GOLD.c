/* 
 * File:   gold.c
 * Author: Козырев С.А.
 *  12.02.14 c:\1\pt1000
 * 
 * написание под MPLAB X8C на 16F1827 
 * PICkit2 , PICkit3
 * 
 * 
 * 
 * V1.0 18.22.14
 * V2.0 26.02.14
 * V2.1 08.04.14
 * 16.06.14
 * 15.07.14
 * 07/08.14 убрали зависания по разбору строки, WDT, 
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <xc.h> // include standard header file

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
#define temp     -(4000)   
#define ok_     0x01
#define bad_crc_     0x02
#define on_     0x04

#define ok_          0x01			//
#define bad_crc_     0x02			//  ош четности
#define onn          0x04			//  включи
#define command_ok   0x08       	//  значения присвоены
#define diag         0x01			//  дай состояние
#define assigment    0x02			//  присвоить значения

//int tic_razborka;
bit enable,flag,crc_ok,ass,on_command,ok_command;
//unsigned char flag_razborka;
union  Crr
	 {
   		unsigned int Int;
   		unsigned char Char[2];
   	 };

unsigned char byte_cnt; 
 
unsigned char takt;

unsigned char t1,tt1;
unsigned char t2,tt2;
unsigned char t3,tt3;
unsigned int t0,tt0;

unsigned char i,nn;
unsigned char j;
 unsigned char t12,t13;	
unsigned char i2;
unsigned char delay;
//unsigned char dmss;
unsigned char diagnostica;
									//	float  temper_float, temper1_float,temper2_float,temper3_float;
									//	int  temper_int, temper1_int,temper2_int,temper3_int;
union Crr osn_chastota,new_osn_chastota;     // частота импульса 
unsigned char   osn_period,new_osn_period;       // период основного импульса 
	
unsigned char vsp_period,new_vsp_period;    // частота импульса 
unsigned char vsp_zazor,new_vsp_zazor;     // зазор между импульсами 



    union Crr  Crc_send,Crc1_send,Crc2_send;
							 
	unsigned char   *tr_buf,flag_read,*tr_bu,flag_ok,flag_write,*tr_bu3,flag_razborka;
	unsigned char  buf1[40]="#,00,001,43868,43868\r\n";
  const unsigned char  master[] = "#,23,001,450,50,10,02,43868\r\n"; 	

										//unsigned char  buf[40];
	unsigned char  buf[40];
										//	unsigned char  buf3[40];
 										 //	const unsigned char  tes[] = "#,34,001,+250.1,+12.3,-23.4, +8.927689\r\n"; 
  
										//	const unsigned char  otvet[] = "#,00,001,43868,43868\r\n"; 

	unsigned char temp3[40];
	unsigned char temp2[40];

		
	unsigned int crc;
									//	int Mm;
									//	unsigned int AnalogValue;       // used to store ADC result after capture
									// unsigned int ADCValue;
									//unsigned int Read_ADC_Value(void);

									//const  unsigned int code crc16LUT[256];
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
	//
	//
	//**********************************************

unsigned char diagnostika(void)
	{ unsigned char te;
		te=0;
		if (ok_command)
			te=te | ok_;
		else
			te &= ~ok_;
			
		if (on_command)
			te = te | on_;
		else
			te &= ~on_;
			
		if (crc !=	Crc2_send.Int )
			te |= bad_crc_;
		else 
			te&= ~bad_crc_;
			
			return  (te);	
	}


	//**********************************************
	//
	//
	//
	//**********************************************

void comand(unsigned  int dia)
	{
		if (( dia & assigment) == assigment)
				ass = 1;
        else
				ass = 0;	
		if (( dia & onn) == onn)
				on_command = 1;	
		else
			on_command = 0;	

	}


	//**********************************************
	//
	//
	//
	//**********************************************

void otv(void)
		{
		unsigned char ij;
		strcpy(buf1,"1,00,");
																//	i  = diagnostika(); 
		sprintf(temp3,"%#0.3u,",(int)diagnostika());	
		strcat(buf1,temp3);
		sprintf(temp3,"%#0.5u,",Crc2_send.Int); 
		strcat(buf1,temp3);
		ij  = strlen(buf1);
  		sprintf(temp3,"%#0.2u,",ij);
		
	    buf1[2] = temp3[0];
		buf1[3] = temp3[1];
		
		Crc1_send.Int = 0;
		for (i =0;i<ij-1;i++)
		 	Crc1_send.Int=FastCRC16(buf1[i], Crc1_send.Int);
		sprintf(temp3,"%#0.5u\n\r",Crc1_send.Int); 
		strcat(buf1,temp3);
										//ij=strlen(buf1);
		}
		
		
/*		
unsigned char proverka_nomera(void)
	{ unsigned char i;
  //  i = atoi(temp2);
	if ((((temp2[0]>= '0') & (temp2[0] <='9')) & ((temp2[1] >= '0') & (temp2[2] <= '9')))& ((i =atoi(temp2)) < 40) )
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
				tr_buf =buf;
				i = strrchr(tr_buf,',')-tr_buf;	
 				strncpy(temp3,tr_buf+i+1,5);	   // взяли присланную контрольную сумму
				temp3[5] =0;
				crc = atoi(temp3);
				if (crc == 0)
					return (0);

				i = strchr(tr_buf,',')-tr_buf;

				strcpy(temp3,tr_buf+i+1);	   // neiie?iaaee inoaoie iinea ','
				i2 = strchr(temp3,',')-temp3;	   // iaoee ne. ','
				strncpy(temp2,temp3,i2);	   // neiie?iaaee aieiao
				temp2[i2] = 0;
				
			//	nn = proverka_nomera();
				nn = atoi(temp2);
				if (nn == 0)
					return (0);
				// nn = atoi(temp2);
				
     			//nn = atoi(temp2);            // iieo?eee ?enei

				Crc2_send.Int = 0;
				crc_ok =0;
  				for (i =0;i<nn-1;i++)
		 				Crc2_send.Int=FastCRC16(tr_buf[i], Crc2_send.Int);
				if (crc !=	Crc2_send.Int)
					{

						tem =crc;
						return (0);
					}
				crc_ok = 1;
				strcpy(temp3,temp3+i2+1);
				i2 = strchr(temp3,',')-temp3;
				strncpy(temp2,temp3,i2);
				temp2[i2] = 0;
				comand(atoi(temp2));
								//	comand(tem);

				if (ass)
						{
																														//i=diagnostica;
							strcpy(temp3,temp3+i2+1);
							i2 = strchr(temp3,',')-temp3;
							strncpy(temp2,temp3,i2);
							temp2[i2] = 0;
							new_osn_chastota.Int = atoi(temp2);
																																	//	i=osn_chastota;
							strcpy(temp3,temp3+i2+1);
							i2 = strchr(temp3,',')-temp3;
							strncpy(temp2,temp3,i2);
							temp2[i2] = 0;
							new_osn_period = atoi(temp2);	
																																	//i=osn_period;
							strcpy(temp3,temp3+i2+1);
							i2 = strchr(temp3,',')-temp3;
							strncpy(temp2,temp3,i2);
							temp2[i2] = 0;
							new_vsp_period = atoi(temp2);	
																																			//i=vsp_period;

							strcpy(temp3,temp3+i2+1);
							i2 = strchr(temp3,',')-temp3;
							strncpy(temp2,temp3,i2);
							temp2[i2] = 0;
							new_vsp_zazor = atoi(temp2);	
							ass = 0;
							ok_command = 1;
																																		//	eeprom_write(0x01,0x055);
							if(osn_chastota.Int != new_osn_chastota.Int)
								{
									eeprom_write(0x02,new_osn_chastota.Char[0]);
									eeprom_write(0x03,new_osn_chastota.Char[1]);
									osn_chastota.Char[0] = eeprom_read(0x02);       // частота импульса 
									osn_chastota.Char[1] = eeprom_read(0x03);       // частота импульса 
									t0 =  osn_chastota.Int;
									tt0 = 0;
								}
							if(osn_period != new_osn_period)
								{
									eeprom_write(0x04,new_osn_period);  //3
									osn_period   = eeprom_read(0x04);       // период основного импульса 
									t1 =  osn_period;
										tt1 = 0;	
								}
							if(vsp_period != new_vsp_period)
								{
									eeprom_write(0x06,new_vsp_period);	  //5
									vsp_period   = eeprom_read(0x06);       // частота импульса 
										t2 =  vsp_period;
											tt2 = 0;
								}
							
							if(vsp_zazor != new_vsp_zazor)
								{
									eeprom_write(0x05,new_vsp_zazor);  // 4
									 vsp_zazor    = eeprom_read(0x05);
									t3 =  vsp_zazor;
									tt3 = 0;
								}
					

						}
			otv();//++++++++++++++++++++++++++

																																		//i=vsp_zazor;
				return (1);	
			}


	//**********************************************
	//
	//
	//
	//**********************************************
/*
unsigned char eeprom_read1(char *  addr)
		{	
			EEADRL =   0x01;
			EECON1bits.CFGS=0;
			EECON1bits.EEPGD=0;
			EECON1bits.RD=1;
			return (EEDATL);
		}
*/

	//**********************************************
	//
	//
	//
	//**********************************************

void tak(void)
	{	switch (takt)
			{
			case 1:
				tt1++;
				if (tt1 > t1)
					{	 
						takt++;
						tt1 =0;
						LATAbits.LATA2 = 0;
					//	LATAbits.LATA3 = 0;
						LATAbits.LATA6 = 0;
						LATAbits.LATA7 = 0;
					}
			break;
			case 2:
				tt2++;
				if (tt2 > t2)
					{	
						takt++;
						tt2 = 0;
			
						LATAbits.LATA2 = 0;
					//	LATAbits.LATA3 = 1;
						LATAbits.LATA6 = 0;  // 0
						LATAbits.LATA7 = 1;  //1

					}
			break;
			case 3:
				tt3++;
				if (tt3 > t3)
					{
						takt++;
						tt3 = 0;
						LATAbits.LATA2 = 0;
					//	LATAbits.LATA3 = 0;
						LATAbits.LATA6 = 0;
						LATAbits.LATA7 = 0;
					}
			break;
			case 4:
				tt0++;
				if (tt0 > t0)
					{	
						takt = 1;
						tt0 = 0;
						LATAbits.LATA2 = 1;
					//	LATAbits.LATA3 = 0;
						LATAbits.LATA6 = 1; //1
						LATAbits.LATA7 = 0;  //0
					}
			break;
			}
		flag = 0;
	}

	//**********************************************
	//
	//
	//
	//**********************************************

	void interrupt Interrupt()
{  
	if ((PIE3bits.TMR6IE==1)&(PIR3bits.TMR6IF == 1 ))  //////////////////// ВТОРОЙ ТАЙМЕР	
		{
			PIR3bits.TMR6IF = 0;
				CLRWDT();
		}
     if (PIR1bits.TMR1IF)  // there is only one interrupt vector so you should check to verify what caused the interrupt
		{
		
			TMR1L=(0xff & temp);
			TMR1H =(0xff & (temp >> 8));  
 			flag = 1;
			tak();																	//	PORTBbits.RB2=~PORTBbits.RB2;
            PIR1bits.TMR1IF=0;        												// ...and then clear the interrupt flag before exiting
		}
    if (PIR1bits.TXIF) 	
			{   
																				//if (flag_write == 1)
				t12 = *tr_bu;
 				if (*tr_bu != 0)  // && (TXSTA.bits.TRMT == 0)
						{
							if (*tr_bu != 0x00)
								{
									t12 = *tr_bu;	
									TXREG = *tr_bu++;
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
							}
						else if (*tr_bu == 0)
							{
								if (flag_write == 0)
									{
																//t12 = *tr_bu;	
																//	TXREG = *tr_bu;
										flag_write = 0;
																//	TXSTAbits.TXEN = 0X0; 
									}	
							}	
			}
	if (PIR1bits.RCIF) 
			{


				if (byte_cnt ==0)
					{
						tr_bu3[byte_cnt]=RCREG;
						t12 =tr_bu3[byte_cnt];

						if (tr_bu3[byte_cnt]=='4')
							{
								RCSTAbits.ADDEN = 0X00;
								byte_cnt++;
	  						}
	   
					}
				else
					{

						tr_bu3[byte_cnt] = RCREG;
						t12 =tr_bu3[byte_cnt];
						if ((tr_bu3[byte_cnt] != '\r') &(byte_cnt < 40  ))
							{
								byte_cnt++;
							}
						else
							{   tr_bu3[++byte_cnt] = 0;
								flag_read = 1;
								RCSTAbits.ADDEN = 0X01;  			//1= Selects 9-bit reception
								TXSTAbits.TX9D  = 1;
								byte_cnt = 0;
								j = 0;

							}								
					}
			}
	}


	//**********************************************
	//
	//
	//
	//**********************************************
void read_ee(void)
		{
			i = eeprom_read(0x01);
			if (i != 0x35)
				{
				eeprom_write(0x01,0x055);
			    new_osn_chastota.Int=200;	
				eeprom_write(0x02,new_osn_chastota.Char[0]);
				eeprom_write(0x03,new_osn_chastota.Char[1]);
				eeprom_write(0x04,50);
				eeprom_write(0x05,12);	
				eeprom_write(0x06,20);
				}

				
		    osn_chastota.Char[0] = eeprom_read(0x02);     // частота импульса 
		    osn_chastota.Char[1] = eeprom_read(0x03);     // частота импульса 	
            osn_period   = eeprom_read(0x04);       // период основного импульса 
	
	        vsp_period   = eeprom_read(0x06);    // частота импульса 
	        vsp_zazor    = eeprom_read(0x05);	
  
		}


	//**********************************************
	//
	//
	//
	//**********************************************

void initc(void)
		{

			read_ee();
			t0 =  osn_chastota.Int;
			t1 =  osn_period;
			t2 =  vsp_period;
			t3 =  vsp_zazor;
			tt0 = 0;
			tt1 = 0;
			tt2 = 0;
			tt3 = 0;
			takt = 4;
			enable= 1;
			LATAbits.LATA2 = 0;
			LATAbits.LATA3 = 0;
			LATAbits.LATA6 = 0;
			LATAbits.LATA7 = 0;
			LATAbits.LATA0 = 1;
			on_command = 0;
			ok_command = 0;
			ass = 0;
		    flag = 0;
		}

	/*
	* 
	*/

	int main(int argc, char** argv) {

    

	TMR1L=(0xff & temp);
	TMR1H =(0xff & (temp >> 8));
 
    OSCCONbits.SCS=0x10;    //set the SCS bits to select internal oscillator block
    OSCCONbits.IRCF=0x0f;   // 16mHz
    OSCCONbits.SPLLEN=0x00;  // pll dicable

    
     // PORT C Assignments
	PORTA = 0;
	PORTB = 0;
	LATA = 0;
	LATB = 0;
	ANSELB=0;
	ANSELA =0;


    TRISBbits.TRISB0 = 0;	// RB0 = 
    TRISBbits.TRISB1 = 1;	// RB1 = это пириемник RX
    TRISBbits.TRISB2 = 0;	// RB2 = это передатчик TX
    TRISBbits.TRISB3 = 0;	// RB3 = это управление RX485    0- прием 1- передача
    TRISBbits.TRISB4 = 0;	// RB4
    TRISBbits.TRISB5 = 0;	// RB5 = 
    TRISBbits.TRISB6 = 0;	// RB6 = 
    TRISBbits.TRISB7 = 0;	// RB7 = 



	TRISAbits.TRISA0 = 0;	// RA0 = 
    TRISAbits.TRISA1 = 0;	// RA1 = 
    TRISAbits.TRISA2 = 0;	// RA2 = 
    TRISAbits.TRISA3 = 0;	// RA3 =  
    TRISAbits.TRISA4 = 0;	// RA4 = 
    TRISAbits.TRISA5 = 0;	// RA5 = 
    TRISAbits.TRISA6 = 0;	// RA6 = 
    TRISAbits.TRISA7 = 0;	// RA7 = 


//	ANSELAbits.ANSA2 = 1;
	FVRCONbits.CDAFVR = 0X10;
	FVRCONbits.FVREN = 0X01;


//	DACCON0bits.DACPSS = 0X0;
//	DACCON0bits.DACOE = 1;
//	DACCON0bits.DACLPS = 1;
//	DACCON0bits.DACEN = 1;

//	DACCON0bits.DACNSS = 0;
//	while (1)
	{
`//	DACCON1 = i++;
	}
	//
	// Set up ADC
	//
	
									//	ANSELB=0;
									//	ANSELA =0;
									//	ANSELAbits.ANSA2=1;	
									//	ANSELAbits.ANSA4=1;	

 	initc();
	
	T6CONbits.T6OUTPS  = 0x0f;  // 1111= 1:16 Postscaler
	T6CONbits.T6CKPS   = 0x03;  // 11= Prescaler is 64
	PR6                = 200;  //40
	T6CONbits.TMR6ON   = 1;	
	
	
	TMR1L=(0xff & temp);
	TMR1H =(0xff & (temp >> 8));     
	T1CONbits.T1CKPS = 0x00;					// 00= 1:1 Prescale value
	T1CONbits.TMR1ON = 0X01;					// включить таймер
	

	TXSTAbits.CSRC = 0X01;						// Asynchronous mode: Don’t care
	TXSTAbits.TX9  = 0X01;						// 0= Selects 8-bit transmission
	TXSTAbits.SYNC = 0X00;						// 0= Asynchronous mode
	TXSTAbits.BRGH = 0X01;						// 1= High speed

	
	  RCSTAbits.RX9  = 0X01; 
	 

	BAUDCONbits.BRG16  = 0X01; 					// 1= 16-bit Baud Rate Generator is used
	//SPBRG =  206; //206;  //       33
	SPBRGL=(0xff & 206);								// !!!!!!!!!!!!! это 9600 не забудь вернуть 19200
	SPBRGH =(0xff & (206 >> 8));   
	//
	//  Прерывания
	//
	
											//	PIR1=0;
    INTCONbits.PEIE = 1;          	// Enable peripheral interrupt
	WDTCONbits.WDTPS = 0x06;   //  00110= 1:2048 (Interval 64 ms typ)  15.07.14 ````````````````
	WDTCONbits.SWDTEN = 0x01;  //  1= WDT is turned on                 15.07.14  ```````````````
	
	PIE3bits.TMR6IE=1;
    INTCONbits.GIE  = 1;           	// enable global interrupt
	PIE1bits.TMR1IE = 0x01;      	// enable the Timer 1 parator interrupt

	PIE1bits.RCIE   = 0x01; 		// разрешение прерывания по приемнику

																									//	i = data_eeprom_read(0x01);

	RCSTAbits.CREN = 0X01; 			//1= Enables receiver 
	RCSTAbits.SPEN = 0X01;			// Asynchronous mode: Don’t care
	RCSTAbits.RX9 = 0X01; 			// 1= Selects 9-bit reception		
	RCSTAbits.ADDEN	=1;
	flag_razborka = 0;
//	tr_bu = &master;
	
	//TXSTAbits.TXEN = 0X01; 			// 1= Transmit enabled


							//t12 = *tr_bu;
							//j=0;
	TXREG = *tr_bu++;
							// j++;
	flag_write = 1;
	
	LATAbits.LATA1 = 1;				// выключили IR2110
    LATAbits.LATA3 = 1;
																	//LATBbits.LATB3 = 1;     /// это передача
																	//	PORTBbits.RB3=1;
	PIE1bits.TXIE=0x01; 			//  разрешили прерывание от передатчика
	WDTCONbits.WDTPS = 0x06;   //  00110= 1:2048 (Interval 64 ms typ)
	WDTCONbits.SWDTEN = 0x01;  //  1= WDT is turned on
	while (1)	
		{
			if (on_command)
				{
					LATAbits.LATA3 = 0;	
					LATAbits.LATA1 = 0;	
				}
			else
				{ LATAbits.LATA3 = 1;
				LATAbits.LATA1 = 1;	
				}	
		//if (flag)
		//	tak();


			for (i=0;i<40;i++)
					buf[i]  = 0;
			LATBbits.LATB3 = 0; 	// включили приемник
			tr_bu3 = &buf;			// буфер приема
			flag_read = 0;			// обнулили флаг окончания передачи
			while (flag_read != 1); // ждем окончания посылки

												//		TXREG = '1';
												//di();

												//if ((flag_write == 0)& (flag_razborka==0)) // & (flag_read = 1))
			{
				razborka2();      // разбор полученой посылки
					
																	//if (flag_read == 1)				//(flag_razborka == 1)
				{	


					tr_bu = &buf1;  // буфер передатчика

																//j=*tr_bu;
																//j++;
																//t12 = *tr_bu;
					flag_write = 1; // флаг конца передачаи
					flag_ok = 1;
																						//	 TXSTAbits.TXEN = 0X01; 
					flag_razborka = 0;  

					LATBbits.LATB3 = 1; // включение RS485 на передачу 
					TXREG = *tr_bu;     // приготовили к передаче  первый байт
					TXSTAbits.TXEN = 0X01; // разрешили передачу байта

				}

																				//PORTBbits.RB3=0;  // это приемо передатчик
																				//LATBbits.LATB3 = 0;   // это прием
				while (flag_write != 0);    // ждем конца передачи

																				//	flag_razborka=1;
																				//flag_read =0;
			}
			//otv();																	//ei();
		}

   
																				//		CLRWDT();
	

    return (EXIT_SUCCESS);
	}

