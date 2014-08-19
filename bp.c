/* 
 * File:   bp.c
 * Author: Козырев С.А.
 *  16.03.14 d:\bp
 * pic16f1847 блок управления БП
 * написание под MPLAB X8C на 
 * PICkit3
 * 
 * 
 * 
 * V1.0 16.03.14
 * V2.0 21.03.14 
 * V2.1 28.03.14
 * V2.1 29.03.14   поехало 8:00
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

//#define	otladka
#undef otladka																		
#define _XTAL_FREQ  16000000        // this is used by the __delay_ms(xx) and __delay_us(xx) functions
#define temp     	-(40000)   

#define ok_     			0x01      // 1
#define bad_crc_     		0x02      // 2
#define	ready_	            0x04
#define	state_	  		    0x08									//  есть\нет изменение состояния                        999  -  0x3e7    ~ 10 bit      4 0x08 
#define dac_	         	0x10									//  есть\нет изменение состояния БП                       5  0x10        0x18
#define	key_	        	0x20					            	//  есть\нет изменение состояния вкл\откл                 6  0x20        0x28
#define	temper_   	        0x40							    	//  есть\нет изменение состояния температура              7  0x40        0x48
#define	adc_		        0x80									//  есть\нет изменение состояния напряжение               6  0x80        0x88

#define	assig_		    0x100											//  установить изменения состояния по запросу инженерной панели         9 0x100
#define	zapr_	    	0x200												//  запросить (flash инженерной панели)                                10 0x200
											
#define TEMPERATURE_UNDER -55               // Значение температуры, возвращаемое если сумма результатов АЦП больше первого значения таблицы 
#define TEMPERATURE_OVER 125             	// Значение температуры, возвращаемое если сумма результатов АЦП меньше последнего значения таблицы 
#define TEMPERATURE_TABLE_START -55        	// Значение температуры соответствующее первому значению таблицы 
#define TEMPERATURE_TABLE_STEP 5           	// Шаг таблицы  

	unsigned char	flag_xvost_temper;   // выбор буфера оцифровки температуры     поменять на bit
	unsigned char	flag_xvost_adc;		 // выбор буфера оцифровки напряжения      поменять на bit
	unsigned char 	flag_zanyato_adc1,flag_zanyato_adc2,flag_peredacha_adc;
	unsigned char 	flag_zanyato_temper1,flag_zanyato_temper2,flag_peredacha_temper,flag_usart;
	
	
	bit kuku;
	unsigned char selector,z;       // выбор канала АЦП
	
	unsigned int AnalogValue11,AnalogValue12;       // значение оцифровки для напряжения
													// unsigned char AnalogCount1;
	unsigned int AnalogValue21,AnalogValue22;       // значение оцифровки для температуры
													// unsigned char AnalogCount2;
	
	unsigned char ten,eleven;

	#ifdef otladka
		unsigned char 	s11,s12,s21,s22;
	#endif

	unsigned char crc_ok;   // bit!!!!!!
	unsigned char ass_command,ok_command;  // bit!!!!!  ,on_command
	unsigned char fist,fl100,sekond,sekond2,ready_command;  // enable,flag,sekon,,fl200,ok2,ok3,ok4,ok,fl1,fl2
    unsigned char adc_command,key_command,state_command,temper_command,dac_command;
	unsigned char right,left ,takt,takt2, takt22,takt1;
	char * ttr_buf;
	unsigned char   *tr_buf,flag_read,*tr_bu,flag_ok,flag_write,*tr_bu3,flag_razborka;
	unsigned int   msec,m100,m200;
   	unsigned char  sek2,tmp;   // sek,
	unsigned char key,key_ok,key_state,flag_xvost,flag_peredacha,flag_zanyato1,flag_zanyato2;
	unsigned char  a1,a2,a3,a4,a5;						//  a5      on/off 
														//  a4   // ready
														//  a3	 // temper 
														//  a2	 // ADC
														//  a1   // DAC  
														//
	unsigned char  a11,a22,a33,a44,a55;					//
														//  a44  // частота 2 значение flash
														//
														//
														//
	unsigned char  a111,a555,j;			//  получено от инженерного пульта
	 int temper;													//
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
	
	unsigned char  	buf[40];			// это передача
	unsigned char 	buf2[40];
	unsigned char 	buf1[40];
										 //	unsigned char  buf3[40];
 										 //	const unsigned char  tes[] = "#,34,001,+250.1,+12.3,-23.4, +8.927689\r\n"; 
  
																//	unsigned char  buf1[40]="#,29,000,71,120,12345\r\n"; 	
																	unsigned char  bu[40]="#,a21424301,21056\r\n";       // 			"#,34,001,+050,67,30,1,1,27689\n\r\0"; 
																	//	unsigned char  bu[40]="#,21,424,30,1,21056\r\n";       // 
	unsigned char  pusto[]="+000,00,00,0,0,";  																
	unsigned char  	bu[40];										// это прием
	unsigned char 	temp3[40];

																// "#,00,001,43868,43868\r\n"
	unsigned char 	i,nn;	
	unsigned char 	i2;
	unsigned char 	txt[7],str[7];   /// char xdata txt[7],str[7];
    unsigned char 	bvminus;
	unsigned char 	decimal;    //  точка
	
	unsigned int crc;
	
	int FastCRC16(char crcData, int crcReg);
	int diagnostika(void);


const unsigned int termo_table[] = {
    1005, 998, 989, 978, 964, 947, 926, 900,
    871,  837, 798, 756, 710, 662, 612, 562,
    512,  463, 417, 373, 333, 296, 262, 232,
    205,  181, 160, 141, 125, 110, 98,  87,
    77,   69,  61,  55,  49
};	
unsigned int adcsum;	
unsigned char kuk;
 unsigned char rr = (sizeof(termo_table) / sizeof(termo_table[0])) - 1;
	
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
//	void ind (void);
	void interrupt Interrupt(void);


	int main(int argc, char** argv);


int FastCRC16(char crcData, int crcReg)
	{  unsigned char table;
		table=((char)(crcReg >> 8)) ^ crcData;
		crcReg=((crcReg << 8) ^ crc16LUT[table]);
		return(crcReg);
	}

 

//   	sprintf(temp3,"%0.3d,",(int)round(((1.0/a11)*10000)));
	//*****************************************************************************
	//
	//    передача мастеру 
    //
    // "#,34,001,+050,   67,  30,  1,     1,   27689\n\r\0"; 
	//           temper, adc, dac, ready, vkl, crc
	//
	//     ok_          - у меня все хорошо - передаю принятый crc и свой скс
	//     bad_crc      - ошибка crc
	//     zapros_      - у меня изменение состояния temper, adc, dac, ready, vkl 
	//     assign_      - провел присвоения dac,vkl
	//     state_       - мое состояние temper, adc, dac, ready, vkl 
	//
	//*****************************************************************************

int diagnostika(void)
	{ 
		int te;
		te=0;
		if (  (state_command == 1) |  (adc_command == 1) | (dac_command == 1) | (temper_command == 1) | (ready_command == 1) | (key_command == 1 ))
			{
		 	state_command = 1;
			te = te | state_;
}
		else					//if (ok_command==1)
			te=te | ok_;

		///////////////////////////////////	
		if (key_command==1)
			{
				te = te | key_ ;
				key_command = 0;
			}

		////////////////////////////////////	
		if (crc !=	Crc2_send.Int )
			te |= bad_crc_;

		////////////////////////////////////	
		te = te | dac_;
		////////////////////////////////////	
		if (dac_command==1)
			{
			te = te | dac_;
			dac_command = 0;								//		dac_command = 0;
			}
		////////////////////////////////////

		if (adc_command==1)
			{
			te = te | adc_;
			adc_command = 0;
			}
	
		te = te | temper_; ///////////////////////////???????
		if (temper_command==1)
			{
			te = te | temper_;
			temper_command = 0;
			}
		
		if (ready_command==1)
			{
			te = te | ready_;
			ready_command = 0;
			}

			return  (te);	
	}

	//**********************************************
	//
	//  получение команды от мастера
	//
	//**********************************************

void comand( int dia)
	{
		


	

/*		if (( dia & ok_) == ok_)
				ok_command = 1;	
		else
			ok_command = 0;		
*/
		/////////////////////////////////////////////////////////
		//if (( dia & adc_) == adc_)
		//		{ass_command = 1;	
		//		adc_command = 1;
		//		}	
		//else
		//	adc_command = 0;	
		/////////////////////////////////////////////////////////

		if (( dia & key_) == key_)
					{ass_command = 1;
				key_command = 1;
					}	
		else
			key_command = 0;

		////////////////////////////////////////////////////////
		/*if (( dia & zapr_) == zapr_)
				zapr_command = 1;	
		else
			zapr_command = 0;*/
		////////////////////////////////////////////////////////	
		if (( dia & state_) == state_)
				state_command = 1;	
		else
			state_command = 0;	
		/////////////////////////////////////////////////////////
		if (( dia & dac_) == dac_)
					{ass_command = 1;
				dac_command = 1;
					}	
		else
			dac_command = 0;	
		if (( dia & assig_) == assig_)  					// assigment_
				{ass_command = 1;
			//	adc_command = 1;
			//	key_command = 1;
			//	dac_command = 1;
			
				}
        else
				ass_command = 0;		
			
	}


	//**********************************************
	//
	//
	//
	//**********************************************
	//    передача мастеру 
    //
    // "#,34,001,+050,   67,  30,  1,     1,   27689\n\r\0"; 
	//           temper, adc, dac, ready, vkl, crc
	//				a3    a2   a1    a4    a5
	//     ok_          - у меня все хорошо - передаю принятый crc и свой скс
	//     bad_crc      - ошибка crc
	//     zapros_      - у меня изменение состояния temper, adc, dac, ready, vkl 
	//     assign_      - провел присвоения dac,vkl
	//     state_       - мое состояние temper, adc, dac, ready, vkl 
	//
	//*****************************************************************************


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
		int ij;
	
		strcpy(buf1,"1,00,");
	//	i  = diagnostika(); 
		sprintf(temp3,"%#3.3u,",(int)diagnostika());	
		strcat(buf1,temp3);
	  
 state_command = 1;
		if (  (state_command == 1) )       // если дай состояние
			{
					sprintf(temp3,"%+0.3u,",(int)temper);   // temper
					strcat(buf1,temp3);
					sprintf(temp3,"%0.3u,",(int)a2);   // power  "%0.2u,"   ADC реальное напряжение
					strcat(buf1,temp3);
					sprintf(temp3,"%0.2u,",(int)a1);   // DAC   // запомнить EEPROM
					strcat(buf1,temp3);
					sprintf(temp3,"%0.1u,",(int)a4);   // ready
					strcat(buf1,temp3);
					sprintf(temp3,"%0.1u,",(int)a5);   // on/off
					strcat(buf1,temp3);	
	     
					
			    
			}
		else
			{	
			strcat(buf1,pusto);	
			ok_command = 0; 
			}
		ij  = strlen(buf1);
  		sprintf(temp3,"%#0.2u,",ij);
		
	    buf1[2] = temp3[0];
		buf1[3] = temp3[1];
		
		Crc1_send.Int = 0;
		for (i =0;i<ij-1;i++)
		 	Crc1_send.Int=FastCRC16(buf1[i], Crc1_send.Int);
		sprintf(temp3,"%#0.5u\n\r",Crc1_send.Int); 
		strcat(buf1,temp3);
		if (flag_usart ==0)
		{
		PIE1bits.TXIE=0x00
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
			PIE1bits.TXIE=0x01	
		}		
		}

	//**********************************************
	//   char * strchr (const char * s, int c)
	//  
	//
	//**********************************************
    // "#,34,001, 30     1,   27689\n\r\0"; 
	//            dac,  vkl, crc
	//
	//     ok_          - у меня все хорошо - передаю принятый crc и свой скс
	//     bad_crc      - ошибка crc
	//     zapros_      - у меня изменение состояния temper, adc, dac, ready, vkl 
	//     assign_      - провел присвоения dac,vkl
	//     state_       - мое состояние temper, adc, dac, ready, vkl 
	//
/*	
unsigned char proverka_nomera(void)
	{ unsigned char i;
	if ((((temp3[0]>= '0') & (temp3[0] <='9')) & ((temp3[1] >= '0') & (temp3[3] <= '9')))& (i =atoi(temp3) < 40)) 
		return (i);
	else 
		return (0);
	}
*/	
	
unsigned char razborka2(void)

			{
			unsigned int tem;
			char * r;

				ttr_buf = &bu;
			
		
				r = strrchr(ttr_buf,',');	// поиск с конца
 				strncpy(temp3,r+1,5);	   // это наверное контрольная сумма
				temp3[5] =0;
				crc = atoi(temp3);  // если буква то crc = 0;
				if (crc == 0)
					return (0);
				r = strchr(ttr_buf,',');
				strncpy(temp3,r+1,2);		// это количество байтов
				temp3[2] = 0;
											// нужна проверка что это цифры и число меньше 40
				//nn = proverka_nomera();
			
				 nn = atoi(temp3);
				if (nn == 0)
					return (0);
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

			
		//		if (ass_command==1)  ///////////////////////////////////
					{
			
						r = strchr(r+1,',');				// DAC
					//	if (dac_command == 1)						// 17.06.14
							{
								strncpy(temp3,r+1,2);
								temp3[2] = 0;
								a11 = (atoi(temp3));
							}

						r = strchr(r+1,',');				// on/off
					//	if (key_command == 1)		// 17.06.14		
							{	 
								strncpy(temp3,r+1,1);
 								temp3[1] = 0;
								a55 = (atoi(temp3));
							}
					 																									//i=diagnostica;
					
				 		ass_command = 1;
						//ok_command = 1;
					
		
					}
	//		otv();

																																		//i=vsp_zazor;
				return (1);	
			}

	void interrupt Interrupt()
{


	if ((PIE1bits.ADIE==1)&(PIR1bits.ADIF == 1 ))  //////////////////// ОЦИФРОВКА	
		{
		PIR1bits.ADIF = 0;	
				if ((selector & 0x01) == 0x01)
					{		// температура
					
					if (flag_xvost_temper)
							{
							
							if (flag_peredacha_temper)
									
									{	flag_zanyato_temper2 	= 1;
										AnalogValue21 			= ADRESH << 8;         // get the 2 msbs of the result and rotate 8 bits to the left
										AnalogValue21 			=	AnalogValue21 + ADRESL;   // now add the low 8 bits of the resut into our return variable
										#ifdef otladka
											AnalogValue21 =s21++;
										#endif
										flag_zanyato_temper2 	=  0;
									}
									
									
							else
									{	flag_zanyato_temper1 	= 1;
										AnalogValue22 			= ADRESH << 8;         // get the 2 msbs of the result and rotate 8 bits to the left
										AnalogValue22 			= AnalogValue22 + ADRESL;   // now add the low 8 bits of the resut into our return variable
										#ifdef otladka
											AnalogValue22 =s22++;
										#endif	
										flag_xvost_temper 		= ~flag_xvost_temper;
										flag_zanyato_temper1 	= 0;
									}	
							}	  	
					else
							{	
							
							// 1
							if (flag_peredacha_temper)
									{	flag_zanyato_temper2 	= 1;
										AnalogValue22 			= ADRESH << 8;         // get the 2 msbs of the result and rotate 8 bits to the left
									    AnalogValue22 			= AnalogValue22 + ADRESL;   // now add the low 8 bits of the resut into our return variable
										#ifdef otladka
											AnalogValue22 =s22++;
										#endif	
										flag_zanyato_temper2 	= 0;
									}
							else
									{	flag_zanyato_temper1 	= 1;
										AnalogValue21 			= ADRESH << 8;         // get the 2 msbs of the result and rotate 8 bits to the left
										AnalogValue21 			= AnalogValue21 + ADRESL;   // now add the low 8 bits of the resut into our return variable
										#ifdef otladka	
										
											AnalogValue21 = s21++;
										#endif	
										flag_zanyato_temper1 	= 0;
										flag_xvost_temper 		= ~flag_xvost_temper;
									}	
								
							}
					}
				
				else
				
								
					{		// напряжение
					
						if (flag_xvost_adc)
							{
								if (flag_peredacha_adc)
								
									{	flag_zanyato_adc2 = 1;
										AnalogValue12 = ADRESH << 8;         // get the 2 msbs of the result and rotate 8 bits to the left
										AnalogValue12 =	AnalogValue12 + ADRESL;   // now add the low 8 bits of the resut into our return variable
										#ifdef otladka
											AnalogValue12 = s12++;
										#endif	
										flag_zanyato_adc2 =  0;
									}
								else
									{	flag_zanyato_adc1 = 1;
										AnalogValue11 = ADRESH << 8;         // get the 2 msbs of the result and rotate 8 bits to the left
										AnalogValue11 = 	AnalogValue11 + ADRESL;   // now add the low 8 bits of the resut into our return variable
										#ifdef otladka
											AnalogValue11 = s11++;	
										#endif
										flag_xvost_adc = ~flag_xvost_adc;
										flag_zanyato_adc1 = 0;
									}
							}
						else
							{	
								
							// 1
								if (flag_peredacha_adc)
									{flag_zanyato_adc2 = 1;
									AnalogValue11 = ADRESH << 8;         // get the 2 msbs of the result and rotate 8 bits to the left
									AnalogValue11 =	AnalogValue11 + ADRESL;   // now add the low 8 bits of the resut into our return variable
									#ifdef otladka	
										AnalogValue11 = s11++;	
									#endif	
									flag_zanyato_adc2 = 0;
								}
								else
									{flag_zanyato_adc1 = 1;
									AnalogValue12 = ADRESH << 8;         // get the 2 msbs of the result and rotate 8 bits to the left
									AnalogValue12 = 	AnalogValue12 + ADRESL;   // now add the low 8 bits of the resut into our return variable
									#ifdef otladka
										AnalogValue12 = s12++;
									#endif
									flag_zanyato_adc1 = 0;}	
									flag_xvost_adc = ~flag_xvost_adc;
								
							}
					
				  		 					
	
					}
		
		}






	if ((PIE3bits.TMR6IE==1)&(PIR3bits.TMR6IF == 1 ))  //////////////////// ВТОРОЙ ТАЙМЕР	
		{
			PIR3bits.TMR6IF = 0;
			TMR6 = 0;
			TMR4 = 0;
				T4CONbits.TMR4ON=1;	
				T6CONbits.TMR6ON=0;	
				selector++;
				if ((selector & 0x01) == 0x01)
					{
							ADCON0bits.CHS =0x01;		// ADC is on	
														//	ANSELAbits.ANSA1=1;		// Select A0 as analog input pin for potentiometer input
														//AnalogValue1 = Read_ADC_Value();
					}
				else
					{
								ADCON0bits.CHS =0x00;		//
															//	ANSELAbits.ANSA0=1;		// Select A0 as analog input pin for potentiometer input
															//AnalogValue2 = Read_ADC_Value();
					}	
															//	PIE3bits.TMR6IE=0;     //1= Enables the TMR4 to PR4 Match interrup  
				PIE3bits.TMR4IE=1;     //1= Enables the TMR4 to PR4 Match interrup  	
		}
	if((PIE3bits.TMR4IE==1)& (PIR3bits.TMR4IF == 1 ))  //////////////////// ВТОРОЙ ТАЙМЕР	
		{
			PIR3bits.TMR4IF = 0;   // 	PR4=40;  //40
				TMR4 = 0;
				TMR6 = 0;	
				T4CONbits.TMR4ON=0;	
			
															//selector++;
															//		if ((selector & 0x02) == 0x02)
															//			{
						  
    						ADCON0bits.GO = 1;  

															//						ANSELAbits.ANSA1=1;		// Select A0 as analog input pin for potentiometer input
															//				AnalogValue2 = Read_ADC_Value();
															//		}
															//	else
															//		{
							  
															//				ADCON0bits.GO = 1;  
															//				ANSELAbits.ANSA0=1;		// Select A0 as analog input pin for potentiometer input
															//				AnalogValue1 = Read_ADC_Value();
															//		}	
					T6CONbits.TMR6ON=1;	
															//	PIE3bits.TMR6IE=1;     //1= Enables the TMR4 to PR4 Match interrup  
															//	PIE3bits.TMR4IE=0;     //1= Enables the TMR4 to PR4 Match interrup  
		}



	if (PIR1bits.TMR2IF == 1 )  //////////////////// ВТОРОЙ ТАЙМЕР
		{PIR1bits.TMR2IF = 0;
			msec++;
	
	
	   if (msec >= 100)
	   		{
															//	flsek = 1;
															//	sek++;
							
				if ((fist) | (sekond))
					{sek2++;
						if (sek2 >= 5) 
							{//	fist = 0;
					 			sek2 = 0;
								fl100 = 1;
								//fl200 = 0; 
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
		}	
											   if (PIR1bits.TMR1IF)  // there is only one interrupt vector so you should check to verify what caused the interrupt

											{
		
												TMR1L=(0xff & temp);
												TMR1H =(0xff & (temp >> 8));  
												//flag = 1;
												//	tak();																	//	PORTBbits.RB2=~PORTBbits.RB2;
											PIR1bits.TMR1IF=0;    
											CLRWDT();
											// ...and then clear the interrupt flag before exiting
											}
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
						TXREG = *tr_bu++;  //*tr_bu++
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
							TXREG = *tr_bu++;  //*tr_bu;

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
						if (TXSTAbits.TRMT==1)  // буфер передачи пуст
							{
								t12 = *tr_bu;	
																		//	TXREG = *tr_bu;
								flag_write = 0;
								TXSTAbits.TXEN = 0X0; 
								flag_usart = 0;
								ten = 0;
					state_command = 0;   ////////////////////////////
				     ////////////////////////////
								LATBbits.LATB3 = 0;
								//LATAbits.LATA4  = 0; 	// включили приемник
#ifdef otladka
							
								if (flag_peredacha)
										if (!flag_zanyato1)
												{																//  		   !flag_peredacha	   buf		  (1)	перезапись		|  передача    buf2
													tr_buf = &buf;													//			    flag_peredacha	   buf2		  (0)					|			   buf
													flag_peredacha= ~flag_peredacha;								//																	|
												}																//																	|
										else																//			  flag_zanyato1		   									|
												tr_buf = &buf2;													//			  flag_zanyato2
		 
								else																		//
										if (!flag_zanyato2)													//
												{																//			   flag_xvost		xvost	 	buf
													tr_buf = &buf2;												//			  !flag_xvost		 			buf2
												   	flag_peredacha= ~flag_peredacha;							//
												}																//
										else																//
												tr_buf = &buf;
#endif

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

						if (tr_bu3[byte_cnt]=='3')       // 3 это БП
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
							{   tr_bu3[++byte_cnt] = 0;
								flag_read = 1;
								RCSTAbits.ADDEN = 0X01;  			//1= Selects 9-bit reception		
								byte_cnt = 0;
							    LATBbits.LATB3 =1;	
													//	LATAbits.LATA4  = 1; 	// включили передатчик
								TXSTAbits.TX9D  = 1;	
								j = 0;


					tr_bu = tr_buf;   //&buf;  // буфер передатчика
					t12   = *tr_bu;
																//j=*tr_bu;
																//j++;
				/*												//t12 = *tr_bu;
					flag_write = 1; // флаг конца передачаи
					flag_ok    = 1;
																						//	 TXSTAbits.TXEN = 0X01; 
					flag_razborka  = 0;  
					TXSTAbits.TX9  = 1;
					j = 0;
					LATBbits.LATB3= 0;  // включение RS485 на передачу 
					TXREG = *tr_bu;     // приготовили к передаче  первый байт
					ten = 1;
					TXSTAbits.TXEN = 0X01; // разрешили передачу байта
			*/





								flag_usart = 1;
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
											tr_bu = tr_buf; 
								//LATBbits.LATB3= 0;  // включение RS485 на передачу 
								TXREG = *tr_bu;     // приготовили к передаче  первый байт
								//ten = 1;
								TXSTAbits.TXEN = 0X01; // разрешили передачу байта	
							}								
					}
					z++;
			}
	}




	void initc(void)
		{
		
			fist 	= 0;
														//	ok 		=0;
														//ok4 	=1;
														//ok3 	= 0;
														//ok2 	= 1;
			right 	= 1;
			left 	= 1;
			key 	= 1;
			takt 	= 1;	
			sekond 	= 0;
			ready_command 	= 0;
			adc_command 	= 0;
			dac_command 	= 0;
			state_command 	= 0;
			//zapr_command 	= 0;
			ass_command = 0;
			//on_command =0;
			key_command = 0;
			sekond2 = 0;
			sek2 = 0;
			decimal = 1;
			bvminus = 1;
			takt1 = 0; 
			takt2= 1;
			takt22 = 0;
			key_state = 1;
			key_ok =0;
			flag_usart = 0;

			a5 = 0;
#ifdef  otladka
			s11 = 10;
			s12 = 30;
			s21 = 50;
			s22 = 70;
#endif

			tmp = eeprom_read(1);	
		if ((tmp  != 0x35))
			{
				eeprom_write(1,0x35);   // код присутствия flash
				eeprom_write(2,0x10);		// период ключа
			
				a1 = eeprom_read(2);		// период ключа
				a11 = a1;
			
			}
		else 
			{ 
				a1 = eeprom_read(2);	// период ключа
				a11 = a1;
			
			}
		

			decimal = 0;
		}

 int sea(unsigned int rr)
	{
		unsigned char m,r,l=0;

		int z,z1,z2,res;
		r = rr;

		if (adcsum >= termo_table[0])
   				return (-55);
		else if (adcsum <= termo_table[rr])
				return (+125);
		else
				{
					while ((r - l) > 1) {
   							  m = (l + r) >> 1;
   							 z=termo_table[m];
    						if (adcsum > termo_table[m])
	 							{
      								r = m;
    							 } 
							else 
								{
      								l = m;
    							}
  							}
		if (adcsum >=termo_table[l])
 			return l * TEMPERATURE_TABLE_STEP + TEMPERATURE_TABLE_START;
		res = TEMPERATURE_TABLE_START + r * TEMPERATURE_TABLE_STEP;
		if (termo_table[l]-termo_table[r])
				{

					z = ( (adcsum - termo_table[r]) ); 

					z1 =(int)ceil((termo_table[l]-termo_table[r])/5.0); 

					z2 =(z/z1);
					if (r < 11)
						res-=z2;
					else
						{

							res-=z2;
						}

					return (res); 
				}
		else
			return res;
			}
	}

/*
const unsigned int termo_table[] = {
    1005, 998, 989, 978, 964, 947, 926, 900,
    871,  837, 798, 756, 710, 662, 612, 562,
    512,  463, 417, 373, 333, 296, 262, 232,
    205,  181, 160, 141, 125, 110, 98,  87,
    77,   69,  61,  55,  49
};	

	
*/


	/*
	* 
	*/

	int main(int argc, char** argv) {

	razborka2();

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



	TRISAbits.TRISA0 = 1;	// RA0 = АЦП аналоговый
    TRISAbits.TRISA1 = 1;	// RA1 = NTC 10k  аналоговый
    TRISAbits.TRISA2 = 0;	// RA2 = ЦАП аналоговый
    TRISAbits.TRISA3 = 1;	// RA3 =  готовности  
    TRISAbits.TRISA4 = 0;	// RA4 = выход на включение БП
    TRISAbits.TRISA5 = 0;	// RA5 = 
    TRISAbits.TRISA6 = 0;	// RA6 = 
    TRISAbits.TRISA7 = 0;	// RA7 =
	
	
	ANSELAbits.ANSA0	= 1;		// Select A0 as analog input pin for potentiometer input
																			//ANSELAbits.ANSA1=1;		// Select A1 as analog input pin for potentiometer input
	ADCON0bits.CHS		= 0x00;	// This selects which analog input to use for the ADC conversion
    ADCON0bits.CHS		= 0x01;	// This selects which analog input to use for the ADC conversion
	ANSELAbits.ANSA1	= 1;
	ANSELAbits.ANSA0	= 1;
	ADCON0bits.ADON		= 1;			// ADC is on
    ADCON1bits.ADCS		= 0x02;		// select ADC conversion clock select as Fosc/32
    ADCON1bits.ADFM		= 0x01;       // results are right justified
	ADCON1bits.ADNREF	= 0;      // 0=VREF- is connected to VSS
	ADCON1bits.ADPREF	= 0;      // 00=VREF+ is connected to VDD
								//ADCON1bits.ADPREF=0x03;      // VREF+ is connected to internal Fixed Voltage Reference (FVR) module
	
								TMR1L=(0xff & temp);
								TMR1H =(0xff & (temp >> 8));     
								T1CONbits.T1CKPS = 0x03;					// 00= 1:1 Prescale value
								T1CONbits.TMR1ON = 0X01;					// включить таймер


	FVRCONbits.ADFVR   = 0x02;


	FVRCONbits.CDAFVR  = 0x02;
    FVRCONbits.FVREN   = 0x01;
	DACCON0bits.DACPSS = 0X2;    //00=VDD
								//01=VREF+
								//10= FVR Buffer2 output
								//11= Reserved, do not use
	DACCON0bits.DACOE  = 1;		// 1= DAC voltage level is also an output on the DACOUT pin
	DACCON0bits.DACLPS = 1;     // 1= DAC Positive reference source selected
	DACCON0bits.DACEN  = 1;		// 1= DAC is enabled

	DACCON0bits.DACNSS = 0;		// 0=VS
	T2CONbits.T2OUTPS  = 0x0F;
	T2CONbits.T2CKPS   = 0x03;
	PR2                = 38;  					//40
	T2CONbits.TMR2ON   = 1;

	T6CONbits.T6OUTPS  = 0x02;
	T6CONbits.T6CKPS   = 0x00;
	PR6                = 200;  //40
	T6CONbits.TMR6ON   = 1;	


	T4CONbits.T4OUTPS  = 0x00;
	T4CONbits.T4CKPS   = 0x00;
	PR4                = 70;  //40
	T4CONbits.TMR4ON   = 1;	




	msec = 0;

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
	PIE1bits.TMR2IE=1;               // разрешили прерыване таймера 2
	PIE1bits.ADIE = 1;
	INTCONbits.IOCIE = 1;											//	PIR1=0;
    INTCONbits.PEIE  = 1;          	// Enable peripheral interrupt
    INTCONbits.GIE   = 1;           	// enable global interrupt
	PIE1bits.TMR1IE  = 0x01;      	// enable the Timer 1 parator interrupt

	PIE1bits.RCIE   = 0x01; 		// разрешение прерывания по приемнику
	PIE1bits.TXIE=0x01; 			//  разрешили прерывание от передатчика
																									//	i = data_eeprom_read(0x01);

	RCSTAbits.CREN = 0X01; 			// 1= Enables receiver 
	RCSTAbits.SPEN = 0X01;			// Asynchronous mode: Don’t care
	RCSTAbits.RX9 = 0X01; 			// 1= Selects 9-bit reception		
	RCSTAbits.ADDEN	=1;
	
	flag_razborka = 0;
									//	tr_bu = &master;
	

	IOCBNbits.IOCBN0 = 1;
	

	initc();
		
	
//		LATAbits.LATA1 = 0;	
//		LATAbits.LATA3 = 1;	
//		LATAbits.LATA2 = 1;	
		PIE3bits.TMR6IE=1;     //1= Enables the TMR4 to PR4 Match interrup  

											//	while (1)


		
			LATBbits.LATB3  = 0; 	// включили приемник
			tr_bu3 = &bu;			// буфер приема
			flag_read = 0;	


			{
				ANSELAbits.ANSA0=1;		// Select A0 as analog input pin for potentiometer input
										//	AnalogValue1 = Read_ADC_Value();
			}
			//razborka2();		
		while (1)
			{
			if (flag_read == 1)
				{
						razborka2();
						flag_read = 0;
				}
			
			flag_peredacha_temper =1;
			if (flag_xvost_temper)
					adcsum = AnalogValue22;	
			else
					adcsum = AnalogValue21;
			temper =sea(rr); //adcsum; 
			flag_peredacha_temper =0;
			
			
			
 			if ((temper > a33) | (temper < a33))
				a33 = temper;
			if ( a3 != a33)
				{
				temper_command = 1;	
				a3 = a33;
				}
				
				
				
			
			flag_peredacha_adc =1;
			if (flag_xvost_adc)
				a22 = (unsigned char) round(AnalogValue12/10.0);   //   /10;
			else
				a22 = (unsigned char) round(AnalogValue11/10.0);    //  /10;
			if ((a22 > a2) | (a22 < a2))
					{
						adc_command = 1;
						a2 = a22;
					}
			//otv();
			flag_peredacha_adc =0;
	otv();
#ifdef otladka

	if (eleven == 0)
	{		
			if (flag_peredacha)
		   		if (!flag_zanyato1)
					{																//  		   !flag_peredacha	   buf		  (1)	перезапись		|  передача    buf2
		   			tr_buf = &buf;													//			    flag_peredacha	   buf2		  (0)					|			   buf
					flag_peredacha= ~flag_peredacha;								//																	|
					}																//																	|
				 else																//			  flag_zanyato1		   									|
				 	tr_buf = &buf2;													//			  flag_zanyato2
		    else																		//
		   		if (!flag_zanyato2)													//
					{																//			   flag_xvost		xvost	 	buf
		   				tr_buf = &buf2;												//			  !flag_xvost		 			buf2
					   	flag_peredacha= ~flag_peredacha;							//
					}																//
				else																//
						tr_buf = &buf;	
eleven = 1;
}
#endif

	
							//while(1)
	//		memset(bu, 0x00, sizeof(bu));
			//for (i=0;i<40;i++)
			//		bu[i]  = 0;
	
  		///	LATAbits.LATA4  = 0; 	// включили приемник
		//	tr_bu3 = &bu;			// буфер приема
		//	flag_read = 0;			// обнулили флаг окончания передачи
		//	while (flag_read != 1);
		
					asm("nop");

				
{	


#ifdef otladka
			
if (ten ==0)
{
				tr_bu = tr_buf;   //&buf;  // буфер передатчика
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
					LATBbits.LATB3= 0;  // включение RS485 на передачу 
					TXREG = *tr_bu;     // приготовили к передаче  первый байт
					ten = 1;
					TXSTAbits.TXEN = 0X01; // разрешили передачу байта
				
}


				
#endif
																				//PORTBbits.RB3=0;  // это приемо передатчик
																				//LATBbits.LATB3 = 0;   // это прием
	///			while (flag_read != 0);    // ждем конца передачи

	//	for (i=0;i<200;i++)
			asm("nop");
				
}			
			/*
					if ((takt1==1) & (takt22 == 0))
					{
						takt++;
						if (takt == 5)
							takt =1;
						takt22 = 1;
						takt2 = 0;
			if (takt == 1)
					{
						LATAbits.LATA1 = 0;	
						LATAbits.LATA3 = 1;	
						LATAbits.LATA2 = 1;	
																			//		a4 =a;												// частота 2
						if (a4 != a44)
							{
								eeprom_write(5,a4);                       	// частота 2  
								a44 = a4;									// частота 2
								state_command = 1;
								dac_command = 1;
							}
																			//		a = a1;												// период ключа
																			//		decimal = 0;										// точка
																			//		ind();
					}
		
		
			
					}
					else if ((takt2 ==1 ) & (takt22 == 1)) 
					{
						
						takt1 = 0;
						takt22 = 0;
					}
					*/
				asm("nop");

			
		//*******************************************
		//
		// присвоение полученноко по USART
		//
		//*******************************************

//	if (ass_command == 1)
		{
			//if (dac_command ==1 )
			if (a11 != a1 )
				{ 
					a11 = a1;
				//	DACCON1 = 0x10;
					DACCON1 = a11;
					dac_command =0;	
					if (a111 != a11)
							{
								eeprom_write(2,a111);
								a111 = a11;
								dac_command=1;
							}	

				}
		
		}
//	if ( (key_command == 1))   //  пришло по сети   // 17.06.14
		{
			if ( a5 != a55)
				a5     = a55;
			
				fist   = 1;
				sekond = 0;
			if ( a5 == 1 )
				{
					LATBbits.LATB6 = 0;
					fist  = 1;  // это по UART
					a4    = 2;
					a44  = a4;
					sek2  = 0;
					fl100 = 0;
				}
			else
				{
					LATBbits.LATB6 = 1;	
					sek2  = 0;
					fl100 = 0;
															////////key = 1;
															//key_ok = 1;
				}
				
				//
				//
			key_command = 0;	
		}
	if  ((fl100==1 ) |  (sekond == 1)|(sekond2 == 1))
		{
			if ( sekond == 1)   //готовность 0
					{
						if (fl100 == 1)      // таймаут
							{
								// не готов
								a4    = 0;
								fl100 = 0;
								
							}
						//else				// все нормально
							//{
								// готов
								//a4 = 1;
							//}
						if (a4 != a44)
							{
								ready_command = 1;
								a44 = a4;
							}	
						sekond = 0;	
						fist   = 0;
						sek2   = 0;
					}
			if (sekond2 == 1)   // готовность 1
				{
					a4 = right;
					sekond2 = 0;
					if (a4 != a44)
							{
								ready_command = 1;
								a44 = a4;
							}
				}
			if (fl100 == 1)
				{
					// не готов
								a4 = 0;
								fl100 = 0;
								sekond = 0;	
								fist = 0;
								sek2 = 0;
								a44 =a4;
								ready_command = 1;
				}
				
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


/* 
// Значение температуры, возвращаемое если сумма результатов АЦП больше первого значения таблицы
#define TEMPERATURE_UNDER -550
// Значение температуры, возвращаемое если сумма результатов АЦП меньше последнего значения таблицы
#define TEMPERATURE_OVER 1250
// Значение температуры соответствующее первому значению таблицы
#define TEMPERATURE_TABLE_START -550
// Шаг таблицы 
#define TEMPERATURE_TABLE_STEP 50




Таблица суммарного значения АЦП в зависимости от температуры. От большего значения к меньшему
   Для построения таблицы использованы следующие парамертры:
     R1(T1): 10кОм(25°С)
     Таблица R/T характеристик: EPCOS R/T:1010; B25/100:3530K
     Схема включения: A
     Ra: 10кОм
     Напряжения U0/Uref: 3.3В/3.3В
*//*
const temperature_table_entry_type termo_table[] PROGMEM = {
    1005, 998, 989, 978, 964, 947, 926, 900,
    871, 837, 798, 756, 710, 662, 612, 562,
    512, 463, 417, 373, 333, 296, 262, 232,
    205, 181, 160, 141, 125, 110, 98, 87,
    77, 69, 61, 55, 49
};

// Функция вычисляет значение температуры в десятых долях градусов Цельсия
// в зависимости от суммарного значения АЦП.
int16_t calc_temperature(temperature_table_entry_type adcsum) {
  temperature_table_index_type l = 0;
  temperature_table_index_type r = (sizeof(termo_table) / sizeof(termo_table[0])) - 1;
  temperature_table_entry_type thigh = TEMPERATURE_TABLE_READ(r);
  
  // Проверка выхода за пределы и граничных значений
  if (adcsum <= thigh) {
    #ifdef TEMPERATURE_UNDER
      if (adcsum < thigh) 
        return TEMPERATURE_UNDER;
    #endif
    return TEMPERATURE_TABLE_STEP * r + TEMPERATURE_TABLE_START;
  }
  temperature_table_entry_type tlow = TEMPERATURE_TABLE_READ(0);
  if (adcsum >= tlow) {
    #ifdef TEMPERATURE_OVER
      if (adcsum > tlow)
        return TEMPERATURE_OVER;
    #endif
    return TEMPERATURE_TABLE_START;
  }

  // Двоичный поиск по таблице
  while ((r - l) > 1) {
    temperature_table_index_type m = (l + r) >> 1;
    temperature_table_entry_type mid = TEMPERATURE_TABLE_READ(m);
    if (adcsum > mid) {
      r = m;
    } else {
      l = m;
    }
  }
  temperature_table_entry_type vl = TEMPERATURE_TABLE_READ(l);
  if (adcsum >= vl) {
    return l * TEMPERATURE_TABLE_STEP + TEMPERATURE_TABLE_START;
  }
  temperature_table_entry_type vr = TEMPERATURE_TABLE_READ(r);
  temperature_table_entry_type vd = vl - vr;
  int16_t res = TEMPERATURE_TABLE_START + r * TEMPERATURE_TABLE_STEP; 
  if (vd) {
    // Линейная интерполяция
    res -= ((TEMPERATURE_TABLE_STEP * (int32_t)(adcsum - vr) + (vd >> 1)) / vd);
  }
  return res;
}

*/