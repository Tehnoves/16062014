//***************************************
//
//	Проект А
//	     24.01.14
//  V2.0 21.03.14
//	V2.1 28.03.14
//	V2.1 29.03.14   8:00 поехал БП
//  V2.1 30.03.14   19:27 pult
//  V2.2 31.03.14   15:00 отразили DAC
//  V2.2 31.03.14   15:00 20:00 выводим период
//   17.06.14
//   02.06.14
//   03.06.14
//   15.08.14
//***************************************




#include <c8051f410.h>                 // SFR declarations
//#include <c8051f350.h>                       // SFR declarations
#include <stdio.h>
#include <string.h>
#include <intrins.h> 
#include <math.h>
#include <stdlib.h>
#include <ctype.h>



#include "F350_FlashPrimitives.h"

#define MASTER	     		(0x31) 
#define BP	     			(0x33) 
#define PULT	     		(0x32) 
#define KLUCY	     		(0x34)
#define CHASTOTA1     		(0x35)  
#define CHASTOTA2     		(0x36)



#define  LCD_ADDR     (0x70)                      // Device addresses (7 bits, lsb is a don't care)
#define  SYSCLK          24500000     // SYSCLK frequency in Hz
#define  LCD_WD         (0x01)
#define  SMB_MTSTA      0xE0                 // (MT) start transmitted
#define  SMB_MTDB       0xC0                 // (MT) data byte transmitted
#define  SMB_MRDB       0x80                 // (MR) data byte received
#define  LCD_WC         (0x00)
#define  SMB_FREQUENCY  50000                // Target SCL clock rate
#define  Normal_font 1
#define  Double_font 2
#define  Triple_font 3 
#define  PWM_START 			20000
#define  WRITE          0x00                 // SMBus WRITE command
#define  READ           0x01                 // SMBus READ command


#define MAX_PERIOD 			50
#define MIN_PERIOD 			35

#define MAX_ZAZOR 			5
#define MIN_ZAZOR 			1


#define MAX_VSP_PERIOD 			20
#define MIN_VSP_PERIOD 			2


#define MAX_CHASTOTA 			500
#define MIN_CHASTOTA 			100

sfr16 TMR3RL   = 0x92;                 // Timer3 reload value
sfr16 TMR3     = 0x94;                 // Timer3 counter

#define ok_          0x01
#define bad_crc_     0x02
#define onn          0x04
#define command_ok   0x08
#define diag         0x01
#define assigment    0x02

#define   ok_     			0x01      // *111
#define   bad_crc_     		0x02      // *
#define	ready_	            0x04      //  *
#define	state_	  		    0x08	  // *								//  есть\нет изменение состояния                        999  -  0x3e7    ~ 10 bit      4 0x08 
#define   dac_	         	0x10	  // *								//  есть\нет изменение состояния БП                       5  0x10        0x18
#define	key_	        	0x20	  //  *				            	//  есть\нет изменение состояния вкл\откл                 6  0x20        0x28
#define	temper_   	        0x40	  //  *						    	//  есть\нет изменение состояния температура              7  0x40        0x48
#define	adc_		        0x80	  //  *								//  есть\нет изменение состояния напряжение               6  0x80        0x88

#define	assig_		    0x100		//									//  установить изменения состояния по запросу инженерной панели         9 0x100
#define	zapr_	    	0x200       //

#define	power_		0x10												//  есть\нет изменение состояния БП                       5  0x10        0x18
#define	period_		0x80												//  есть\нет изменение состояния ключи                    6  0x80        0x88
#define	key_		0x20												//  есть\нет изменение состояния ключи                    6  0x20        0x28
#define	frec1_   	0x40													//  есть\нет изменение состояния частотник                7  0x40        0x48
#define	frec2_	    0x04	
#define	off_		0x10												//  есть\нет изменение состояния БП                       5  0x10        0x18
																//  есть\нет изменение состояния БП                       5  0x10        0x18
#define	on_	        0x20					            	//  есть\нет изменение состояния вкл\откл                 6  0x20        0x28
#define	plus_   	0x40	
#define	minus_		0x80		

		int xdata period_11;
		unsigned char xdata napravlenie,selector,frec1,frec2;
		unsigned char xdata dac_11,frec1_11,frec2_11,vkl_pult_11,adc_pult_11,vkl_pult__11,dac_12;
		bit new_ok_pult;
		bit new_crc_pult;
		bit new_key_pult;
		bit new_state_pult;
		bit new_dac_pult;
		bit new_frec1_pult;
		bit new_frec2_pult;
		bit new_period_pult;
		bit new_ass_pult;
		bit  LCD_COMM;     
		bit ass_command_bp;
		bit reset_sek;
		bit zapet_peredachi,zapret_priema;   //   pzu,
		
		bit dac_command_pult;			// ?
		bit key_command_pult;			// ?
		bit ass_command_pult;			// 
		bit state_command_pult;			// ?
		bit period_command_pult;		// ?
		bit ok_command_pult;		//
		bit badcrc_command_pult;		//
		
		
		
		bit assign_bp_11;
		bit ass_command_frec1;
		bit state_command_frec1;
		bit on_command_frec1;
		bit off_command_frec;
		bit plus_command_frec;
		bit minus_command_frec;
		
	 	bit crc,stop_priem_pult,stop_peredacha_pult;  // 19.08.14 20:06
		bit ready_key_bp,ready_temper_bp;
		bit new_ok_bp,new_crc_bp,new_key_bp,new_state_bp,new_dac_bp,new_temper_bp,new_adc_bp,new_ready_bp,ass_bp;
		bit vkl_bp,vkl_bp_11,vkl_kl,vkl_kl_11,vkl_frec,vkl_frec_11;	    // ???????????????????
		bit assign_bp,fist_bp;
   



   unsigned char  xdata on_frec1,off_frec1,plus_frec1,minus_frec1,cikl_pult; // 19.08.14 20:06
   unsigned char xdata i,ekr,adc,adc_11,ready_bp,ready_bp_11;
   unsigned char xdata dac,dac_111; // задание напряжения
   char xdata temper,temper_11;
	//
    // данные для хранения пакета передачи
	//
  unsigned char xdata sek;
	unsigned int xdata msek;
	unsigned char xdata temper_char[8], temper1_char[8],temper2_char[8],temper3_char[8];
	//float xdata temper_float, temper1_float,temper2_float,temper3_float;
	//int xdata temper_int, temper1_int,temper2_int,temper3_int;
	//unsigned char diagnostica;

int a9;
    unsigned char idata byte_cnt,tr_ok,read_ok,sost_rd,byte_cnt1,ok_read;
	unsigned char   *tr_buf;
	unsigned char xdata buf1[40];
	unsigned char xdata buf[40];
	unsigned char xdata buf3[40];
	unsigned char xdata buf2[40];
	////////////////////////////unsigned char *buf2; 
	unsigned char code plus[]="+\0";
   //const unsigned char code tes[] = "#,34,001,+250.1,+12.3,-23.4, +8.927689\n\r";  
	 const unsigned char code tes1[] = "#,34,001,+050,67,30,1,1,27a89\n\r\0"; 
	char xdata xvost[40];

	 char xdata golova[18],buf_[18];
    unsigned char xdata aaa_int;
	 unsigned int xdata aaa_kuku,aaa2_int;
	 unsigned int xdata aaa3_int;
///	 float  xdata aaa1;
	 int xdata aaa;
	 unsigned int xdata aaa1,aaa2,bbb3;
	 unsigned char xdata bbb;
	   unsigned char xdata z,z1,row0_down,row1_down,row2_down,row3_down,tara_down,z2,tip;

	int xdata osn_chastota;     // частота импульса 
	int xdata osn_period;       // период основного импульса 
	
	int xdata vsp_period;    // частота импульса 
	int xdata vsp_zazor;     // зазор между импульсами 

	float xdata test;
	int  xdata i_test;
	  unsigned char xdata null_down,minus_down,plus_down;	
  unsigned char xdata minus_press,plus_press;
	unsigned char xdata xx,yy,bunker;
	unsigned char xdata buffer[2];
    unsigned char kod,kodd,row,kl,commanda;
	unsigned xdata PWM = PWM_START;
	unsigned char xdata decim,sostojanie,komanda;

	//  запись - чтение ПЗУ
	//	вывод на экран текста
	//	вывод на экран графики
	//	работа с клавиатурой
	//	знакогенератор
	//
   extern const unsigned char code klav [3][5];
   extern  const unsigned char code TABL [155][6];

	unsigned char xdata taut;	
   bit av,bk,sv,ok,flag_taut,flag_sek,zad_bk,ko_ok,refresh,flag_dop;
bit new_av,new_bk,new_sv,new_ok,new_ko_ok, new_state,new_key	;	
    bit nachalo;
   bit b_new,b_old;
   bit start_u,null_u,minus_u,plus_u,stop_u;
   bit perebor;
	bit  takt;
  	bit SMB_BUSY = 0;   
   	bit SMB_SINGL_SENDCON;
 	bit SMB_SENDCON; 
 	bit SMB_SINGL_BYTE;
 	bit SMB_RW;                                  // Software flag to indicate the
                                            	 // direction of the current transfer
 	bit SMB_ACKPOLL;  
	bit di_;
	bit as_;
	bit on_1;   
	                         
//	  bit di_,on_,as_;
 	unsigned char* pSMB_DATA_IN;                 	// Global pointer for SMBus data
    	                                         	// All receive data is written here

 	unsigned char SMB_SINGLEBYTE_OUT;            // Global holder for single byte writes

 	unsigned char* pSMB_DATA_OUT;                // Global pointer for SMBus data.
            	                                 // All transmit data is read from here

	 unsigned char SMB_DATA_LEN;                  // Global holder for number of bytes
    	                                         // to send or receive in the current
        	                                     // SMBus transfer
 	unsigned char TARGET;                        // Target SMBus slave address

	unsigned char LCD_CON;
	unsigned char xdata LCD_out_buf[132];
	char xdata number[3];
	
	
	
	
    void LCD_CLR(unsigned char row);
    void LCD_init(void);
    void LCD_print(unsigned char Line,unsigned char  Pos, const unsigned char *str,unsigned char Font,unsigned char Invers);
    void ochistka_ekrana(void);
	void SMBUS_Write_Arrey( unsigned char Len,unsigned char* dat);
    void LCD_Write_Arrey(unsigned char con,bit cycle_con ,unsigned char len,unsigned char* dat);
	void pereschet(void);
	void PCA_init (void);
	void Timer2_Init (void);
	void redaktor2(void);
	void ppmm(void);
	unsigned char razborka_klucy(void);
	unsigned char razborka_bp(void);
	unsigned char razborka_pult(void);
	//void otv(void);
   void otv_bp(void);
		void otv_pult(void);
			void otv_klucy(void);
				void otv_frec1(void);
			unsigned char razborka_frec1(void);

	
	
	sbit LCD_RST =P0^2;
	sbit P20 =P2^0;
	sbit P03 = P0^6;	  // вес С   20
 sbit P21 =P2^1;
 sbit P22 =P2^2;
 sbit P23 =P2^3;
 sbit P24 =P2^4;
 sbit P25 =P2^5;
 sbit P26 =P2^6;
 sbit P27 =P2^7;
/////////////////////////////////////////////////////////////////////////////
							// unsigned char xdata rez1,rez3,rez4,rez5,op;
							//  unsigned int rez2;

	void otv_bp(void);
	
union zap
   {  char ch[4];
   long lo;
   };


 FLADDR xdata addr; 

 union rezult
	{ unsigned long Lon;
	  float Rea; 
	  unsigned char Ch[4];	
	};
 union rezult xdata rezultat,rezultat_itog;




 union  Crr
 {
   unsigned int Int;
   unsigned char Char[2];
   
 };
 union  Crr2
 {
   int Int;
   char Char[2];
   
 };
 union Crr xdata Crc_send,Crc1_send,Crc2_send;
							 
							 
							 
						
 struct constanta 
   {
	unsigned char cod;
	int zad_osn_chastota;		  // целое
	int zad_osn_period;
	int zad_vsp_period;    // частота импульса 
	int zad_vsp_zazor;     // зазор между импульсами 
	unsigned char zad_dac;

	
  };
 struct constanta coc;


 union global 
	{ struct constanta co;
	  char con[sizeof(coc)];
	};


 union global xdata AA;


/////////////////////////////////////
   const char code init_lcd[21] = {	
	 							0x01,  					//  page 000
								0x0e,  					//  page 110
								0x12,  //  mux 1/34		0x12
								0x84,  //  BIAS	1:65
								0x06,  //  normal mode
								0x24,  // set current
								0x0c,  // mirror
								0x01,  					//  page 000
								0x10,	// power-down gorizont
								0x0b,					//  page 011
								0x58,	// TRS    BRS
								0x05,	// DM =1;
								0x01,					//  page 000
								0x0d,					//  page 101
								0x09,	// mul factor
								//0x06,
								0x04,	// vlcd low
								0xb5,	// napriagenie
								0x05,	// start generator
								0x01, 					//  page 000
								0x0b, 					//  page 011
								0x04};	// direct mode///////////////////
								 /*
  								0x20,  //  vert,power-down
								0x0e,  //  page 110
								0x05,  //  normal mode
								0x12,  //  mux 1/34		0x12
								0x84,  //  BIAS	1:65
								0x24,
								0x08,
								0x01,  //  page 000
								0x0d,  //  page 101
								0x08,  //  volt multipl 3
								0x17,  //  K of temper	2	 0x22
								0x04,  //  VLCD HIGHT
								0xb8,
								0x05}; //  VLCD = 12.02	  0xe8
								   */
   const unsigned char code row0[] = {131,'e','p',147,'o',170,' ','=',' ','0','0','0',0};
   const unsigned char code row1[] = {167,150,147,154,'e',150,'.','=',' ','0','0','0',0};
   const unsigned char code row2[] = {127,'a',146,'o','p',' ',' ','=',' ','0','0','0',0};
   const unsigned char code row3[] = {167,150,'.',143,'a',156,' ','=',' ','0','0','0',0};



   const unsigned char code row4[] = {' ',' ',123,150,'o',149,' ',149,150,161,155,'e',148,' ',0};	 // Блок ключей
   const unsigned char code row5[] = {'A',142,'a','p',147,162,' ',' ',' ',' ','+',0};	 //	 Авария
   const unsigned char code row6[] = {'B',149,150,'.',' ',' ',' ',' ',' ',' ','+',0};	 //	 Вкл
   const unsigned char code row7[] = {'O',154,149,150,'.',' ',' ',' ',' ',' ',' ',0};	 //	 Откл
   const unsigned char code row8[] = {'H','e',154,' ','c',142,162,146,147,' ','+',0};	 //	 Нет связи
   const unsigned char code row9[] = {124,'o',154,'o',142,' ',' ',' ',' ',' ','+',0,0};	 //	 Готово


   // const unsigned char code row14[] = {'1','2','3','4',153,147,154,'a',152,147,162,' ',0};
   const unsigned char code row14[] = {' ',' ',123,150,'o',149,' ',153,147,154,'a',152,147,162,' ',0};	 // Блок питания
   const unsigned char code row15[] = {'T','e',151,153,'p',' ','+','1','2','5',179,'C',0};	 //	 Температура
   const unsigned char code row16[] = {'H','a',153,'p',162,145,' ','6','6',142,' ',0};	 //	 Напряжение
   const unsigned char code row17[] = {'D','A','C',' ',' ',' ',' ','5','9',142,' ',0};	 //	 Цап
   const unsigned char code row18[] = {'B',149,150,' ',' ',' ','+',0};	 //	 Вкл\Откл
   const unsigned char code row19[] = {124,'o',154,'o',142,' ','+',0};	 //	 Готово
   const unsigned char code row20[] = {'H','e',154,' ','c',142,162,146,147,' ','+',0};	 //	 Нет связи

/*		union  
{
    unsigned int Int;
    unsigned char Char[2];
}xdata Crc_send,Crc1_send,Crc2_send;  */

	unsigned char xdata rez3,rez4,rez5,op;
	unsigned int xdata rez2,rez1;


extern unsigned int code crc16LUT[256];
int FastCRC16(char crcData, int crcReg);

int FastCRC16(char crcData, int crcReg)
{  unsigned char table;
   table=((char)(crcReg >> 8)) ^ crcData;
   crcReg=((crcReg << 8) ^ crc16LUT[table]);
   return(crcReg);
}

 void delay(char op)
{
	char q;
	for (q=op;q>0;q--);


}
/*
#define MASTER	     		(0x31) 
#define BP	     			(0x32) 
#define PULT	     		(0x32) 
#define KLUCY	     		(0x34)
#define CHASTOTA1     		(0x35)  
#define CHASTOTA2     		(0x35)
*/
	void otv_who(char kuda)
		{
			switch (kuda)
				{
				case PULT:
					{
					napravlenie = PULT;
					otv_pult();	
					break;
					}
				case KLUCY:
					{
					napravlenie = KLUCY;
					otv_klucy();	
					break;
					}
				case BP:
					{
					napravlenie = BP;
					otv_bp();	
					break;
					}
				case CHASTOTA1:
					{
					  otv_frec1();
					break;
					}
				case CHASTOTA2: 
					{
					break;
					}
				}	
		}
		void raborka_who(char kuda)
		{
			switch (kuda)
				{
				case PULT:
					{
					//napravlenie = PULT;
					razborka_pult();
					break;
					}
				case KLUCY:
					{
					//napravlenie = KLUCY;
					razborka_klucy();	
					break;
					}
				case BP:
					{
					//napravlenie = BP;
					razborka_bp();	
					break;
					}
				case CHASTOTA1:
					{
					razborka_frec1();
					break;
					}
				case CHASTOTA2: 
					{
					break;
					}
				}
				_nop_();	
		}

			 
	//**********************
	//
	//
	//	 чтение FLASH
	//
	//
	//**********************

	void init_read(void)
		{
		char i;
			for (i=0;i<sizeof(AA);i++)
   				AA.con[i]=0;
		for (i=0;i<sizeof(AA);i++)
   				AA.con[i]=FLASH_ByteRead ((FLADDR)addr+i);
		osn_chastota  = AA.co.zad_osn_chastota ;
		osn_period = AA.co.zad_osn_period;
		vsp_period=AA.co.zad_vsp_period;
		vsp_zazor = AA.co.zad_vsp_zazor;
		dac = AA.co.zad_dac;
		}
				 

	  
	//********************************
	//
	//	  запись FLASH
	//
	//********************************

	void init_write(void)
		{  
		char I;
   
		AA.co.zad_osn_chastota = osn_chastota;
		AA.co.zad_osn_period = osn_period;
		AA.co.zad_vsp_period = vsp_period;
		AA.co.zad_vsp_zazor = vsp_zazor;
		AA.co.zad_dac = dac;
		AA.co.cod = 0x55;
		FLASH_PageErase((FLADDR) addr);
		for (I=0;I<sizeof(AA);I++)
				FLASH_ByteWrite ((FLADDR)addr+I,AA.con[I]);

	}	  




  	//***********************************
	//
	//	Присваиваем базовые коэффициенты
	//
	//***********************************
									   	 	//float osn_chastota;     // частота импульса 
											//float osn_period;       // период основного импульса 
	
											//float vsp_period;    // частота импульса 
											//float vsp_zazor;     // зазор между импульсами 



			void init_const(void)
			{   osn_chastota = 500;   // 2~8Hz
				osn_period   = 50;	   // 35~50mcek
				vsp_period   = 8;	   // 2~10mcek
				vsp_zazor    = 2;
				dac = 30;
			}


	void init_data(void)
		{

			 init_read();
			 if ( AA.co.cod != 0x53)
 					{init_const();
					init_write();
					init_read();}
 
		}

 void TIMER3_Init (void)
{
   TMR3CN  = 0x00;                     // Stop Timer3; Clear flags;
                                       // use SYSCLK/12 as timebase

   CKCON  &= ~0xC0;                    // Timer3 clocked based on T3XCLK;

   // Init reload values
   TMR3RL  = -(SYSCLK / 12 / 1000/5);

   TMR3    = 0xffff;                   // Set to reload immediately
   EIE1    |= 0x80;                    // Enable Timer3 interrupts

   TMR3CN  |= 0x04;                    // Start Timer3
}

void initc(void)
	{
		OSCICN    = 0x87;
							//    P0MDOUT   = 0x37;
							//XBR0      = 0x05;
							//XBR1      = 0xC0;

		P0MDOUT   = 0xFF;
		P1MDOUT   = 0xff; //0x1F;
		P2MDOUT   = 0x7F;
		XBR0      = 0x05;
		XBR1      = 0x40;
							//   P1MAT     = 0x1F;
		P1MASK    = 0xff;  //0xE0;   
		PCA_init();

		SCON0     = 0x30;   
		TMOD      = 0x22;
		CKCON     = 0x05;			   //				  0x05	   0x08
										// CKCON     = 0x01;
		TH0       = 0xAE;
		TH1       = 0x60;			   //	   0x60	19200	 0x96	 9600
		TR0 =1;
		TR1 =1;




	  /*

	TMOD      = 0x22;
    CKCON     = 0x0c;
    TH0       = 0xAE;
    TH1       = 0x60;
	TR0 =1;
	TR1 =1;	 */
    TIMER3_Init ();
	Timer2_Init ();

									// SCON0     = 0x30;
	SMB0CF    = 0xd0;
								   ///////

	EIE1      = 0x91;
    EIE2      = 0x02;
    IE        = 0xb0;
//	while (1);  ///////////////////////////
	 	kodd = 1;
	 addr=0x7c00-512;
	 init_data();	
	}

   	//*********************
	//
	// код нажатой клавиши
	//
	//*********************

	void down(void)
		{
						//    static unsigned char j;
						//	if (j > 19)
						//		j = 0;
  						//  stroka[j++] =  klav[row-1][kod-1];
						//	kl =  stroka[j-1];
  						//  if (stroka[j-1]==0x0d)
    					//    j = 0;
		kl =  klav[row-1][kod-1];
		takt = 1; 
	}


	//***************************
	//
	//	Борьба с дребезгом
	//
	//
	//***************************

 void PCA_init (void)
 {	decim = 0;
	PCA0CPL0 = (0xff & PWM);
	PCA0CPH0 = ( 0xff & (PWM >>8));
	PCA0L = 0;
	PCA0H = 0;
	PCA0CPM0 =0x049;
	EIE1 |= 0x10;

 }

 void SMBUS_Write_Arrey( unsigned char Len,unsigned char* dat)
{
   while (SMB_BUSY);                         // Wait for SMBus to be free.
   SMB_BUSY = 1;                             // Claim SMBus (set to busy)

   // Set SMBus ISR parameters
   TARGET = LCD_ADDR;                     // Set target slave address
   SMB_RW = WRITE;                           // Mark next transfer as a write
   SMB_ACKPOLL = 1;                          // Enable Acknowledge Polling (The ISR
                                             // will automatically restart the 
                                             // transfer if the slave does not 
                                             // acknowledge its address.

   pSMB_DATA_OUT = dat;     // The outgoing data pointer points to
                                             // the <dat> variable.

   SMB_DATA_LEN = Len;

   // Initiate SMBus Transfer
   STA = 1;
}


   void LCD_SET_XY(unsigned char X,unsigned char Y)
{
	while (SMB_BUSY);

		LCD_out_buf[0]=0xb0|Y;
		LCD_out_buf[1]=0x00|(X & 0X0F);
		LCD_out_buf[2]=0x10|(X >> 4);
					



//	LCD_out_buf[0]=0x01;
//	LCD_out_buf[1]=0x40|Y;
//	LCD_out_buf[2]=0x80|X;
	LCD_Write_Arrey(LCD_WC,1,3,&LCD_out_buf);
}



   void LCD_print(unsigned char Line,unsigned char  Pos, const unsigned char *str,unsigned char Font,unsigned char Invers)
{   char i;
    LCD_SET_XY(Pos,Line);
	if(Font ==Normal_font)//Normal_font)
	{           
		while(*str)
		{
				while (SMB_BUSY);
				for(i=0;i<6;i++)
				{
				   if(Invers) LCD_out_buf[i]=~(TABL[*str-0x20][i]);
                   else LCD_out_buf[i]=TABL[*str-0x20][i];
				}
				*str++; 
			LCD_Write_Arrey(LCD_WD,0,6,&LCD_out_buf);
			while (SMB_BUSY);
		}
	} else if(Font ==Double_font)
	{
	 	while(*str)
		{
			switch(*str++)
			{
			 	case '0': 
				{		 while (SMB_BUSY);
						 LCD_out_buf[0]=0xFC; LCD_out_buf[1]=0xFC; LCD_out_buf[2]=0x03;
						 LCD_out_buf[3]=0x03; LCD_out_buf[4]=0xC3; LCD_out_buf[5]=0xC3;							 
						 LCD_out_buf[6]=0x33; LCD_out_buf[7]=0x33; LCD_out_buf[8]=0xFC;
						 LCD_out_buf[9]=0xFC; LCD_out_buf[10]=0; LCD_out_buf[11]=0;
						 LCD_Write_Arrey(LCD_WD,0,12,&LCD_out_buf);
						 while (SMB_BUSY);
						 LCD_SET_XY(Pos,++Line);
						 while (SMB_BUSY);
						 LCD_out_buf[0]=0x0f; LCD_out_buf[1]=0x0F; LCD_out_buf[2]=0x33;
						 LCD_out_buf[3]=0x33; LCD_out_buf[4]=0x30; LCD_out_buf[5]=0x30;							 
						 LCD_out_buf[6]=0x30; LCD_out_buf[7]=0x30; LCD_out_buf[8]=0x0F;
						 LCD_out_buf[9]=0x0F; LCD_out_buf[10]=0; LCD_out_buf[11]=0;
						 LCD_Write_Arrey(LCD_WD,0,12,&LCD_out_buf);
				break;}
			}
		}	  
	}
}	



  void LCD_Write_Arrey(unsigned char con,bit cycle_con ,unsigned char len,unsigned char* dat)
	{
	while (SMB_BUSY);                         // Wait for SMBus to be free.
	LCD_COMM = con;
	SMB_SENDCON=1;
	SMB_SINGL_SENDCON=cycle_con; 
	SMBUS_Write_Arrey(len,dat);
	}



	void LCD_init(void)
 		
		 {  char i=0;
			int j =0;

   //			LCD_RST=1;
   	//		LCD_RST=0;
		_nop_();
				while (++j<150)
   			while(++i);
   			LCD_RST=1;
   while (SMB_BUSY);
			LCD_out_buf[0]=0xe2;
	
			LCD_Write_Arrey(LCD_WC,1,1,&LCD_out_buf);
   		
				i=0;
				j=0;
   		 while (++j<150)
   		 while(++i);   
			 
			 while (SMB_BUSY);
					LCD_out_buf[0]=0xeb;
					LCD_out_buf[1]=0x81;
					LCD_out_buf[2]=120;
					LCD_out_buf[3]=0xc6;
					LCD_out_buf[4]=0xaf;
					LCD_Write_Arrey(LCD_WC,1,5,&LCD_out_buf);
			
  			}






	  	void ochistka_ekrana(void)

	{
	
//	while (1);
	LCD_CLR(0);
     LCD_CLR(1);
	 LCD_CLR(2);
	 LCD_CLR(3);
	 LCD_CLR(4);
	 LCD_CLR(5);
	 LCD_CLR(6);
	 LCD_CLR(7);
	}
  	//************************
	//
	// печать на экран в место
	//
	//************************
			   /*
void  print_int(unsigned char n_bunker,unsigned char typ_viv)
    {
        switch (n_bunker)
        {
            case 1:
                if (typ_viv == 0)   
                    LCD_print(5,8*6, &tek_ves1,1,0);
                else    
                    LCD_print(5,15*6, &zad_ves1,1,0);
                    break;
            case 2:
                if (typ_viv == 0)
                    {LCD_print(4,8*6, &tek_ves2,1,0);}
                else
                    {LCD_print(4,15*6, &zad_ves2,1,0);}
                    break;
            case 3:     
                if (typ_viv == 0)
                    {LCD_print(3,8*6, &tek_ves3,1,0);}
                else
                    {LCD_print(3,15*6, &zad_ves3,1,0);}
                    break;
			 case 4:     
                if (typ_viv == 0)
                    {if (buf3[0] == '+') 
                    	LCD_print(0,5*6, &buf3+1,1,0);
					 else
					 	{buf3[1] = buf3[2];
						buf3[2] = buf3[3];
						buf3[3] = buf3[4];
						LCD_print(0,5*6, &buf3,1,0);}}
                else
					if (buf3[0] == '+') 
                    	LCD_print(0,16*6, &buf3+1,1,0);
					 else
					 	{buf3[1] = buf3[2];
						buf3[2] = buf3[3];
						buf3[3] = buf3[4];
						LCD_print(0,16*6, &buf3,1,0);}
                    break;
					
        }

    }		 */



			 
	// int osn_chastota;     // частота импульса 
	// int osn_period;       // период основного импульса 
	
	// int vsp_period;    // частота импульса 
	// int vsp_zazor;     // зазор между импульсами 


	

	//***********************************
	//
	//	 базовые кэф. в буффер
	//
	//***********************************

void rdd(void)

	{
	if (bunker == 1)
           { if (nachalo)
				{ 
				aaa = osn_chastota;
		   	 	sprintf(xvost,"%0.3d",aaa);
				buffer[0] = xvost[0];
				buffer[1] = 0;
		
		   		nachalo = 0;
				}
		   		else 
           		buffer[0] =xvost[xx]; 
		   }
          else if (bunker == 2)
            { if (nachalo)
				{ 
				aaa = osn_period;
		   	 	sprintf(xvost,"%0.3d",aaa);
				buffer[0] = xvost[0];
				buffer[1] = 0;
		
		   		nachalo = 0;
				}
		   		else 
           		buffer[0] =xvost[xx]; 
			 }
          else if (bunker == 3)
            {
			  if (nachalo)
				{ 
				aaa = vsp_zazor;
		   	 	sprintf(xvost,"%0.3d",aaa);
				buffer[0] = xvost[0];
				buffer[1] = 0;
		
		   		nachalo = 0;
				}
		   		else 
           		buffer[0] =xvost[xx]; 
			}
		  else if (bunker == 4)
            {  if (nachalo)
				{ 
				aaa = vsp_period;
		   	 	sprintf(xvost,"%0.3d",aaa);
				buffer[0] = xvost[0];
				buffer[1] = 0;
			
		   		nachalo = 0;
				}
		   		else 
           		buffer[0] =xvost[xx]; 
			
		}	  
	}



   	//*****************************
	//
	//
	//
	//*****************************

	void invertt(unsigned char inv)

	{	
		    rdd();
		    buffer[1] = 0;
			
			LCD_print(yy,(12*6 + (xx)*6), &buffer,1,inv);
			
			
			takt = 0;
	}
	// int osn_chastota;     // частота импульса 
	// int osn_period;       // период основного импульса 
	
	// int vsp_period;    // частота импульса 
	// int vsp_zazor;     // зазор между импульсами 




	//*********************************
	//
	//	 задание в буфер
	//
	//*********************************

void rd(void)

	{	  
		  if (bunker == 1)
           {aaa = osn_chastota;
		   	 	sprintf(xvost,"%0.3d",aaa);
				buffer[0] = xvost[0];
			}
          else if (bunker == 2)
            { 	aaa = osn_period;
		   	 	sprintf(xvost,"%0.3d",aaa);
				buffer[0] = xvost[0];
			}
          else if (bunker == 3)
            { aaa = vsp_zazor;
		   	 	sprintf(xvost,"%0.3d",aaa);
				buffer[0] = xvost[0];
			}	
		 
		   else if (bunker == 4)
		   		{ 	aaa = vsp_period;
		   	 	sprintf(xvost,"%0.3d",aaa);
				buffer[0] = xvost[0]; 
				}	
	}




	//*******************************
	//
	//
	//
	//*******************************

	void invert(unsigned char inv)

	{	
		    rd();
		    buffer[1] = 0;
			LCD_print(yy,(12*6 + (xx)*6), &buffer,1,inv);
			takt = 0;
	}

  void LCD_CLR(unsigned char row)
{
    LCD_SET_XY(0,row);
	while (SMB_BUSY);                         // Wait for SMBus to be free.
	SMB_SINGL_BYTE=1;
	LCD_out_buf[0]=0x00;
	LCD_Write_Arrey(LCD_WD,1,133,&LCD_out_buf);
    LCD_SET_XY(0,row);
}

//
//LCD_LINE(3,0,10);
//
//
void LCD_LINE1(unsigned char Line, unsigned char  Pos ,unsigned char Number)
	{
unsigned char ii;
    LCD_SET_XY(Pos,Line);

	          
		
		{
				while (SMB_BUSY);
				for(ii=0;ii<Number;ii++)
				{
				  
                   LCD_out_buf[ii]=0xff;
				}
				; 
			LCD_Write_Arrey(LCD_WD,0,Number,&LCD_out_buf);
			while (SMB_BUSY);  
		
			 LCD_SET_XY(Number-1,Line);
			 	while (SMB_BUSY);
		   for(ii=0;ii<(128-Number);ii++)
				{
				  
                   LCD_out_buf[ii]=0x80;
				}
				ii = 128-Number; 
			LCD_Write_Arrey(LCD_WD,0,ii,&LCD_out_buf);
			while (SMB_BUSY);  
		}

	}
void LCD_LINE(unsigned char Line, unsigned char  Pos ,unsigned char Number)
	{
unsigned char i;
    LCD_SET_XY(Pos,Line);

	          
		
		{
				while (SMB_BUSY);
				for(i=0;i<Number;i++)
				{
				  
                   LCD_out_buf[i]=0xff;
				}
				; 
			LCD_Write_Arrey(LCD_WD,0,Number,&LCD_out_buf);
			while (SMB_BUSY);  
		}

	}
    void wrr(void)

		{  
	
			/*
		    if (bunker == 1)
		   	{	
				 strcpy (golova, xvost);
					golova[xx] = buffer[0];
			 		aaa = atoi(golova);

			 		if ((aaa > MAX_CHASTOTA) || (aaa < MIN_CHASTOTA) )	
			 			perebor = 1;
			  		else 
			  		 	perebor = 0;
			 		if (!perebor)
							xvost[xx]=buffer[0];
		  	  }
	      else if (bunker == 2)
            	{	strcpy (golova, xvost);
					golova[xx] = buffer[0];
			 		aaa = atoi(golova);

			 		if ((aaa > MAX_PERIOD) || (aaa < MIN_PERIOD) )	
			 			perebor = 1;
			  		else 
			  		 	perebor = 0;
			 		if (!perebor)
							xvost[xx]=buffer[0];
				
				}
	      else if (bunker == 3)
            	{  strcpy (golova, xvost);
					golova[xx] = buffer[0];
			 		aaa = atoi(golova);

			 		if ((aaa > MAX_ZAZOR) || (aaa < MIN_ZAZOR) )	//
			 			perebor = 1;
			  		else 
			  			perebor = 0;
			 		if (!perebor)
						xvost[xx]=buffer[0];
					
				}
			 else if (bunker == 4)
            	{  strcpy (golova, xvost);
					golova[xx] = buffer[0];
			 		aaa = atoi(golova);

			 		if ((aaa > MAX_VSP_PERIOD) || (aaa < MIN_VSP_PERIOD) )	//
			 			perebor = 1;
			  		else 
			  			perebor = 0;
			 		if (!perebor)
						xvost[xx]=buffer[0];
					
				}
				  */

				 xvost[xx]=buffer[0];
				 aaa = atoi(xvost);

			if (yy == 0)
				{//if (!perebor)
						LCD_print(0,((12)*6 ), &xvost,1,0);
				}
			else  if (yy == 1)
				{//if (!perebor)
						LCD_print(1,((12)*6 ), &xvost,1,0);
				 }
			else if ( yy == 2)
					{//	if (!perebor)
							LCD_print(2,((12)*6 ), &xvost,1,0);
					}
			else if (yy == 3)
					{//	if (!perebor)
						 	LCD_print(3,((12)*6 ), &xvost,1,0);

					}
			
	     //  pereschet();
		}	 

void wrr_plus(void)
	{
			wrr();
		//	if (  !perebor)
				{
					xx++;		
											//if ((x == 6) && !perebor)
			 								//	xx++;
					if (xx	== 3)
							xx=0;
				
			   }
		   
				
		//   if (!perebor)
		    invertt(1);
	}

// редактор блока питания	
// редактор блока питания	
// редактор блока питания	
// редактор блока питания

	
   void redaktor_bp(void)                            
   		{
		   unsigned char b,y0; 
			y0=yy;
			b = bunker;
			ass_command_pult = 1;
			period_command_pult = 0;
			bunker=1;
			sek = 1; 												//////////////////
			ekr= 3;
			refresh=1;
		   while (bunker != 0)
				   {
					  if (takt)
					  		{ if (kl == 'b')
								{    bunker = 0;
								}
							  else if (kl < 'a')
								{	
									if (kl == '1')
										on_1 = 1;	 
									else if (kl == '4')
										on_1 = 0;

							    }	
						      takt =0;
							}


								if (( new_state_pult)  )   					//  & (! new_crc_bp)// bit new_ok_bp,new_crc_bp,new_key_bp,new_state_bp,new_dac_bp,new_temper_bp,new_adc_bp,new_ready_bp,ass_bp;
									{
																	//if (dac != dac_11 )
												{
												
													
													sprintf(xvost,"%0.2u",(int)(dac_11));   // flash
													LCD_print(3,((10)*6 ), &xvost,1,0);
													if (dac != dac_11)
														{
																dac = dac_11;
																init_write();
														}
													new_dac_pult = 0;
												}
									}
																					//////   new_ok_bp,new_crc_bp,new_state_bp    //////
						if (( new_state_bp) & crc )   								//  & (! new_crc_bp)// bit new_ok_bp,new_crc_bp,new_key_bp,new_state_bp,new_dac_bp,new_temper_bp,new_adc_bp,new_ready_bp,ass_bp;
							{					 								// bit ready_key_bp,ready_temper_bp;
																								//	if (new_adc_bp)
								if (adc != adc_11)
									{
										adc = adc_11;
																								//adc_pult_11 = adc;
										sprintf(xvost,"%0.2u",(int)adc_11);
							     		LCD_print(2,((10)*6 ), &xvost,1,0);
										new_adc_bp = 0;
									}
								if (new_temper_bp)
									{
																								//temper = temper_11;
										sprintf(xvost,"%+0.3d",(int)temper_11);
										LCD_print(1,((9)*6 ), &xvost,1,0);
										new_temper_bp = 0;
									}
								if (temper < 80)
									ready_temper_bp = 1;
								else
									ready_temper_bp = 0;
							
								if (new_ready_bp)
									{
																								//ready_bp = ready_bp_11;
										if (ready_bp == 0)
											{   
												xvost[0] = '0';
												xvost[1] = 0;
												LCD_print(5,((9)*6 ), &xvost,1,0);
												ready_key_bp = 0;
											}
										else if (ready_bp == 1)
											{
												xvost[0] = '1';
												xvost[1] = 0;
												LCD_print(5,((9)*6 ), &xvost,1,0);
												ready_key_bp = 1;
											}
										else if (ready_bp == 2)
											{
												xvost[0] = '*';
												xvost[1] = 0;
												LCD_print(5,((9)*6 ), &xvost,1,0);
												ready_key_bp = 0;
											}
											new_ready_bp = 0;
									}
								if (new_key_bp)
									{
										//vkl_bp = vkl_bp_11;
										if (vkl_bp_11 == 0)
											{   if (ready_bp == 0)
													{
														xvost[0] = ' ';
														xvost[1] = '0';
														xvost[2] = ' ';
														xvost[3] = 0;
													}												
												else if ( ready_bp == 2)
													{
														xvost[0] = '(';
														xvost[1] = '0';
														xvost[2] = '}';
														xvost[3] = 0;
														LCD_print(5,((9)*6 ), &xvost,1,0);
													}
												else if (vkl_bp == 1)
													{	xvost[0] = ' ';
														xvost[1] = '1';
														xvost[2] = ' ';
														xvost[3] = 0;
														LCD_print(5,((9)*6 ), &xvost,1,0);
													}
										
											}	
										assign_bp_11 = 0;	// ??????????????????????????
									}
							}

						 if (pzu)                     // ИНЖЕНЕРНЫЙ --- ТЕХНОЛОГИЧЕСКИЙ ПУЛЬТЫ первый раз
							 {
								if  (period_11 == osn_chastota)
									pzu = 0; // ждем передачу к пульту
							 }
						 else
							 {
							//	if (period_11!= osn_chastota)					///   04.07.14
										{
										
													osn_chastota = period_11;
											//		init_write();
													init_write(); 
												  
										}	
							}	
								if (dac != dac_11)
									{
										dac = dac_11;
										init_write();
									}				
					 	if (  (flag_dop == 1))			//	   нормальный конец прмема	  (flag_taut == 0) &
					 		{	new_sv =1;
								aaa2++;
								sprintf(xvost,"%0.5d",aaa2);
																						//  LCD_print(1,((15)*6 ), &xvost,1,0);
								   
								flag_dop = 0;
								ppmm();
							}
						if ((read_ok == 1))		//	 & (flag_taut ==1)					//		  конец прмема	 ??
					 		{   read_ok = 0;
								byte_cnt1=0;
							   	taut = 0;
								flag_taut = 0;
								flag_dop = 0;
																						//	raborka_who(CHASTOTA1);
										 
								if (selector== 1)   // БЛОК ПИТАНИЯ
									{	raborka_who(PULT);   }
								else if (selector ==2)
									{	raborka_who(KLUCY);        }
								else if (selector ==3)
									{	raborka_who(BP);    		}	
								else if (selector ==4)
									{	raborka_who(CHASTOTA1);   		}		
																				//	razborka_bp();
																				//	ppmm();
								aaa1++;
								sprintf(xvost,"%0.5d",aaa1);
																				/////	  LCD_print(1,((1)*6 ), &xvost,1,0);
								sprintf(xvost,"%0.5d",bbb3);
																				/////	    LCD_print(1,((8)*6 ), &xvost,1,0);
								new_sv = 0;
								//reset_sek = 1;							//sek = 0;
								sek = 1;   
											msek =0; 
							}
						if ((read_ok == 0) & (flag_taut == 0))        // АВАРИЯ ПРИЕМА ёёёёёёёёёёёёё
										reset_sek = 1;	
						if ( 	(sek   == 0x01) &(tr_ok==1))		   		//	передали и прошел тайм аут приема		(sek   == 0x01) &
			 				{   //reset_sek = 1;
									 sek = 0;				// поменять $ на | или прошла разборка
													msek =0;											//	delay(200);
																			//	delay(200);
																			//	memset(buf2, 0x00, sizeof(buf2));
																			//otv_who(CHASTOTA1);
								   
								if (selector== 1)
									{
										selector++;
										otv_who(KLUCY);
									}
								else if (selector == 2)
									{
										selector++;
										otv_who(BP);
									}
								else if (selector == 3)
									{
										selector++;
										otv_who(CHASTOTA1);
									}	
								else if (selector == 4)
									{
										selector= 1;
										otv_who(PULT);
									}	   
																						/*	
																						if (selector== 1)
																							{
																							otv_who(BP);
																							}
																						else if (selector == 2)
																							{
																							otv_who(PULT);
																							}
																							*/
								//sek = 0;
																						// otv_bp();
							    P20 = 1;
								S0MODE =1;
								MCE0 =1;
								TB80 = 1;
							    tr_buf = &buf1;
							    flag_taut = 1;
								flag_dop = 0;
								ok_read = 1;tr_ok = 0;

							    TI0 = 1;
							//	flag_taut = 1;      // запустили таймаут приема
							//	flag_dop = 0;

							}   
						else if ((sek  != 0x01) & (tr_ok == 1) &  (ok_read == 1))		  // ??????
							{	read_ok = 0;
								byte_cnt1=0;
								ok_read = 0;
							}

				

						}
			bunker=b;
			yy=y0;	

		}
 // оперативный экран ключей
  // оперативный экран ключей
  // оперативный экран ключей
   // оперативный экран ключей

   void redaktor2(void)  // оперативный экран ключей

		{  unsigned char b,y0; 
			y0=yy;
			b = bunker;
			sek = 1; //////////////////
			ekr= 2;
			refresh=1;
		   while (bunker != 0)
				   {
					    if (takt)
					  		{ 	if (kl == 'b')
									{    bunker = 0;}
								else if (kl < 'a')
									{	
										if (kl == '1')
											on_1 = 1;	 
										else if (kl == '4')
											on_1 = 0;

									}	
								takt =0;
						    }
						if (dac != dac_11)
							{
								dac = dac_11;
								init_write();
							}	
						if (period_11!= osn_chastota)					///   04.07.14
										{
										
													osn_chastota = period_11;
													init_write();
												
												  
										}		
					 	if (  (flag_dop == 1))			//	   нормальный конец прмема	  (flag_taut == 0) &
					 		{	new_sv =1;
								aaa2++;
								sprintf(xvost,"%0.5d",aaa2);
								LCD_print(1,((15)*6 ), &xvost,1,0);
								   
								flag_dop = 0;
								ppmm();
							}
					 	if ((read_ok == 1) )		//	 & (flag_taut ==1)					//		  конец прмема	 ??
					 		{   read_ok = 0;
							   	taut = 0;
								flag_taut = 0;
								flag_dop = 0;
								
								if (selector== 1)    //   REDAKTOR2
									{	raborka_who(CHASTOTA1);  }
								else if (selector ==2)
									{	raborka_who(PULT);    }
								else if (selector ==3)
									{	raborka_who(BP);  }	
								else if (selector ==4)
									{	raborka_who(KLUCY); }		
								
								ppmm();
								aaa1++;
								sprintf(xvost,"%0.5d",aaa1);
								LCD_print(1,((1)*6 ), &xvost,1,0);
								sprintf(xvost,"%0.5d",bbb3);
								LCD_print(1,((8)*6 ), &xvost,1,0);
								new_sv = 0;
								reset_sek = 1;    //sek = 0;   //  `````````````````````````````````````````
							}
						if ((read_ok == 0) & (flag_taut == 0))        // АВАРИЯ ПРИЕМА ёёёёёёёёёёёёё
										reset_sek = 1;	
					 	if ((sek   == 0x01) & (tr_ok==1))		   		//	передали и прошел тайм аут приема
			 				{		reset_sek = 1;
																		//	delay(200);
																		//	delay(200);
							
							
								if (selector== 1)
										{
											selector++;
											otv_who(PULT);
										}
									else if (selector == 2)
										{
											selector++;
											otv_who(BP);
										}
									else if (selector == 3)
										{
											selector++;
											otv_who(KLUCY);
										}	
									else if (selector == 4)
										{
											selector= 1;
											otv_who(CHASTOTA1);
										}	
								
									P20 = 1;
									S0MODE =1;
									MCE0 =1;
									TB80 = 1;
									tr_buf = &buf1;
									tr_ok = 0;
									TI0 = 1;
								//	flag_taut = 1;
								//	flag_dop = 0;
									ok_read = 1;

							}   
						else if ((sek  != 0x01) & (tr_ok == 1) &  (ok_read == 1))		  // ??????
							{	 read_ok = 0;
								 byte_cnt1=0;
								 ok_read = 0;
							}

				//   ppmm();

				   }
			  		bunker=b;
					yy=y0;	
		 }


	// int osn_chastota;     // частота импульса 
	// int osn_period;       // период основного импульса 
	
	// int vsp_period;    // частота импульса 
	// int vsp_zazor;     // зазор между импульсами 

// ввод данных на ключи	
// ввод данных на ключи	
// ввод данных на ключи	
// ввод данных на ключи	
	
void redaktor(void)   // ввод данных на ключи

		{  unsigned char b,y0; 
			y0=yy;
			b = bunker;
			ekr  = 1;
			sek = 1; //////////////////

			bunker=1;
			aaa = osn_chastota;
		   	sprintf(xvost,"%0.3d",aaa);
			buffer[0] = xvost[0];
			buffer[1] = 0;
		 	nachalo = 0;
																													//nachalo = 1;
    		xx = 0;
			yy = 0;
			invert(1);
			while (bunker != 0)

				{
				   if (takt)
				        { 
										// редактор										

							if (kl < 'a')                    //  ЭТО РЕДАКТОР ЭТО РЕДАКТОР ЭТО РЕДАКТОР ЭТО РЕДАКТОР
								{	buffer[0] = kl;  
									wrr_plus();
								}                            //  ЭТО РЕДАКТОР ЭТО РЕДАКТОР ЭТО РЕДАКТОР ЭТО РЕДАКТОР


										 // после редактора переход на новую строку


						    else if (kl == 'd')		        //  ЭТО КОНЕЦ ВВОДА ЭТО КОНЕЦ ВВОДАЭТО КОНЕЦ ВВОДА ЭТО КОНЕЦ ВВОДА 
								{  		wrr();				//  EEPROM EEPRO EEPROMM EEPROM
										xx = 0;
										if ( bunker == 1)
											{  	
													if ((aaa >= MAX_CHASTOTA) || (aaa <= MIN_CHASTOTA) )	
														{	 _nop_();
															aaa = osn_chastota;  // 19.08.14 20:13 ???????
															sprintf(xvost,"%0.3d",aaa);   // блокировать прием
															LCD_print(0,((12)*6 ), &xvost,1,0);
														}
													else
														{
														//stop_priem_pult=1;    //20.08.14 5:20
														osn_chastota = aaa;
												
														}
													yy++;
													bunker++;
													period_command_pult=1; // 22.08.14
													ass_command_pult = 1;
													
													if (AA.co.zad_osn_chastota != osn_chastota)		
														init_write();
											}
										else if (bunker == 2)
											{  	
													if ((aaa >= MAX_PERIOD) || (aaa <= MIN_PERIOD) )	
														{	 _nop_();
															aaa = osn_period;
															sprintf(xvost,"%0.3d",aaa);
															LCD_print(1,((12)*6 ), &xvost,1,0);
														}
													else
														osn_period = aaa;
													yy++;
													bunker++;
													if  (AA.co.zad_osn_period != osn_period)
														init_write();
											}
										else if (bunker == 3)
											{  	
																						
													if ((aaa >= MAX_ZAZOR) || (aaa <= MIN_ZAZOR))	
															{	 _nop_();
																aaa = vsp_zazor; 
																sprintf(xvost,"%0.3d",aaa);
																LCD_print(2,((12)*6 ), &xvost,1,0);
															}
													else
															vsp_zazor = aaa;	
													yy++;
													bunker++;
													if (AA.co.zad_vsp_zazor != vsp_zazor)
														init_write();
											}
									   	else if (bunker == 4)
											{  	
													if ((aaa >= MAX_VSP_PERIOD) || (aaa <= MIN_VSP_PERIOD))	
															{	 _nop_();
																aaa = vsp_period;
																sprintf(xvost,"%0.3d",aaa);
																LCD_print(3,((12)*6 ), &xvost,1,0);
															}
													else
															vsp_period = aaa;
													yy = 0;
													bunker = 1;
													if  ( AA.co.zad_vsp_period != vsp_period)
														init_write();
											}
										pereschet();
										xx=0;
										invert(1);
								}                               //  ЭТО КОНЕЦ ВВОДА ЭТО КОНЕЦ ВВОДАЭТО КОНЕЦ ВВОДА ЭТО КОНЕЦ ВВОДА 


										//  уходим

							else if (kl == 'b')                //  ЭТО СМЕНА ЭКРАНА  ЭТО СМЕНА ЭКРАНА ЭТО СМЕНА ЭКРАНА
								{ 	wrr();
									xx = 0;
								   if (bunker ==1)
										{
									  	if ((aaa > MAX_CHASTOTA) || (aaa < MIN_CHASTOTA) )	  // 19.08.14 20:13 ???????
													{	 _nop_();								// блокировать прием
														aaa = osn_chastota;
														sprintf(xvost,"%0.3d",aaa);
														LCD_print(0,((12)*6 ), &xvost,1,0);
													}
										else
													{
													stop_priem_pult=1;    //20.08.14 5:20
													osn_chastota = aaa;
													
													}
													period_command_pult=1; // 22.08.14
													ass_command_pult = 1;
									}				
								   else if (bunker == 2) 
										if ((aaa > MAX_PERIOD) || (aaa < MIN_PERIOD) )	
													{	 _nop_();
														aaa = osn_period;
														sprintf(xvost,"%0.3d",aaa);
														LCD_print(1,((12)*6 ), &xvost,1,0);
													}
										else
									    				osn_period  = aaa;
								   else if (bunker == 3) 
										if ((aaa > MAX_ZAZOR) || (aaa < MIN_ZAZOR))	
													{	 _nop_();
														aaa = vsp_zazor; 
														sprintf(xvost,"%0.3d",aaa);
														LCD_print(2,((12)*6 ), &xvost,1,0);
													}
										else
									    			vsp_zazor  = aaa;
								   else if (bunker == 4) 
										if ((aaa > MAX_VSP_PERIOD) || (aaa < MIN_VSP_PERIOD))	
													{	 _nop_();
														aaa = vsp_period;
														sprintf(xvost,"%0.3d",aaa);
														LCD_print(3,((12)*6 ), &xvost,1,0);
													}
										else
										 				vsp_period  = aaa;
								   pereschet();
								   invertt(0);
								   bunker = 0;

								}                         //  ЭТО СМЕНА ЭКРАНА  ЭТО СМЕНА ЭКРАНА ЭТО СМЕНА ЭКРАНА 
								
								
									
						  takt = 0;
						}	// КОНЕЦ РЕДАКТОРА КОНЕЦ РЕДАКТОРА КОНЕЦ РЕДАКТОРА КОНЕЦ РЕДАКТОРА КОНЕЦ РЕДАКТОРА КОНЕЦ РЕДАКТОРА
					  	//?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????12345
						
					// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&|
					// анализ обмена с пультом                                             |
					//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|
					//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|
					
				// 	признак клавы
				if ((period_command_pult) & (new_ok_pult == 1) & (period_11 == osn_chastota))
								{
																	// проверили if (period_11!= osn_chastota)
									period_command_pult = 0;      	// убрали period_command_pult
									ass_command_pult    = 0;        // убрали ass_command_pult
									state_command_pult  = 1;      	// установили state_command_pult = 1
									
								}
				// присвоение технологического пульта
				if ((new_ass_pult) & (period_11 != osn_chastota)   )
								
								{
															if (bunker == 1)     // мы в редакторе на этой строке
																	{	nachalo = 1;
																		xx = 0;
																		osn_chastota = period_11; // отдать пульту
																		aaa = osn_chastota;       // блокировать прием
																	//	sprintf(xvost,"%0.3d",osn_chastota);
																	
																	  
																	}	
															else
																	{
																		osn_chastota = period_11;   //???????19.08.14 13:42
																	//	sprintf(xvost,"%0.3d",osn_chastota);
																		
																	}
																	
															sprintf(xvost,"%0.3d",osn_chastota);		
															LCD_print(0,((12)*6 ), &xvost,1,0);	
															rdd();
															pereschet();
															invertt(1);
															                                //new_state_pult =0;              //*//
															state_command_pult  = 1;        //*//
															ok_command_pult =1;             //*//
															init_write(); 
								}
				// подтверждение приема  моей частоты от технологического пульта	(признак клавы)				
				if ((period_11 == osn_chastota) & (new_ok_pult) & (new_state_pult) )
								{
									ok_command_pult = 0;
								}
					
				if (period_11!= osn_chastota)					/// // 19.08.14 20:13 ???????
								{
									
								}								
											
				
							
					//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|
					//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|		
							
							
					if (dac != dac_11)
							{
								dac = dac_11;
								init_write();
							}		
					if (  (flag_dop == 1))			//	   нормальный конец примема	  (flag_taut == 0) &
										{	new_sv =1;
											aaa2++;
											sprintf(xvost,"%0.5d",aaa2);
											LCD_print(5,((13)*6 ), &xvost,1,0);
											flag_dop = 0;
											ppmm();
										}
					if ((read_ok == 1) )		//	10mcek и нет тайм аута    ёёёёёёёёёёё   & (flag_taut ==1)					//		  конец прмема	 ??
										{   read_ok = 0;
											taut = 0;
											flag_taut = 0;
											flag_dop = 0;
									//		  raborka_who(PULT);
										//	  reset_sek = 1;

											 if (selector== 1)  
												  	{ 
														if ((stop_priem_pult == 1)) //20.08.14 5:20
															{
																cikl_pult = 0;
																stop_priem_pult = 0;
															}
													raborka_who(PULT);        }	
											 	else if (selector ==2)
													 	{ raborka_who(KLUCY);     }	
												else if (selector ==3)
												{ raborka_who(BP);      }

											  	if (selector== 4)          // ИНЖЕНЕРНЫЙ ПУЛЬТ
												{ raborka_who(CHASTOTA1); }
								 
											/*
											if (selector== 1)          // ИНЖЕНЕРНЫЙ ПУЛЬТ
												{ raborka_who(CHASTOTA1); }
											else if (selector ==2)
												{ raborka_who(PULT);      }
											else if (selector ==3)
												{ raborka_who(BP);        }	
											else if (selector ==4)
												{ raborka_who(KLUCY);     }	*/	
										
											ppmm();
											aaa1++;
											sprintf(xvost,"%0.5d",aaa1);
											LCD_print(5,((1)*6 ), &xvost,1,0);
											new_sv = 0;
										//	reset_sek = 1;	
											sek = 1;   
											msek =0;              // 11.07.14 ёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёёё
										}
				//	if ((read_ok == 0) & (flag_taut == 0))        // АВАРИЯ ПРИЕМА ёёёёёёёёёёёёё
										//reset_sek = 1;
					//				{	sek =1;
					//					msek =0; 
					//					 }
					if ((sek   == 0x01) & (tr_ok==1))		   		//	20 mcek  передали и прошел тайм аут приема ЁЁЁЁЁЁЁЁЁЁЁЁЁЁЁЁЁЁЁЁЁЁ
										{		   sek = 0;				// поменять $ на | или прошла разборка
													msek =0;					//	delay(200);
											//	reset_sek = 1;					//	delay(200);
										   
								/*		if (selector== 1)
												{

													selector++;
													otv_who(KLUCY);
													
												}
											else if (selector == 2)
												{
														selector= 1;
												//	selector++;
													otv_who(BP);
												}

												*/

											 if (selector == 1)
												{
													selector++;
													otv_who(KLUCY);
												}
												else if (selector == 2)
												{
													selector++;
													otv_who(BP);
												}	 
	
													
											else if (selector == 3)
												{
													selector++;
													otv_who(CHASTOTA1);
												}	
											else if (selector == 4)
												{
													selector= 1;
													if ((stop_priem_pult == 1) & (cikl_pult != 4))  //20.08.14 5:20
														{
															cikl_pult++;
														}
													otv_who(PULT);
												}	 	 

										//	 otv_who(PULT);

											//	aaa1++;
											P20 = 1;
											S0MODE =1;
											MCE0 =1;
											TB80 = 1;
											tr_buf = &buf1;
											tr_ok = 0;
											TI0 = 1;
											
											ok_read = 1;                      // ёёёёёёёёёёёёёёёёёёёёёёёёёёёёёё

										}   
					else if ((sek  != 0x01) & (tr_ok == 1) &  (ok_read == 1))		  // 20mcek   прошла передача и таймаут не окончен??????
										{	 read_ok = 0;                             // опережение окна
											 byte_cnt1=0;
											 ok_read = 0;
										}


				

				}
		



			if ((AA.co.zad_osn_chastota != osn_chastota) | (AA.co.zad_osn_period != osn_period) | ( AA.co.zad_vsp_period != vsp_period) | (AA.co.zad_vsp_zazor != vsp_zazor))
										{
											init_write();                           // 4 секунды ЁЁЁЁЁЁЁЁЁЁЁЁЁЁЁЁЁЁЁЁЁЁЁЁЁ
											as_=1;
										}
			bunker=b;
			yy=y0;	
																					//	good_bye();
		}
	

void pereschet(void)
	{		unsigned char i,j,m;

					LCD_CLR(7);
    				LCD_CLR(6);
	 				LCD_CLR(5);
					LCD_CLR(4);
   
		  			i = ceil(osn_period*128/osn_chastota);
					LCD_LINE1(6,0,i);
					j = ceil(vsp_zazor*128/osn_chastota);
					m = ceil(vsp_period*128/osn_chastota);
					LCD_LINE(7,i+j,m);
					
					_nop_();
	}


 void ekran1(void)
{ 	   		

				ochistka_ekrana();
				LCD_print(0,3*6, &row0,1,0);  
				LCD_print(1,3*6, &row1,1,0);  
				LCD_print(2,3*6, &row2,1,0);  
				LCD_print(3,3*6, &row3,1,0);  
													// int osn_chastota;     // частота импульса 
													// int osn_period;       // период основного импульса 
	
													// int vsp_period;    // частота импульса 
													// int vsp_zazor;     // зазор между импульсами 
		   
				
					
					sprintf(xvost,"%0.3d",osn_chastota);
					LCD_print(0,((12)*6 ), &xvost,1,0);

					sprintf(xvost,"%0.3d",osn_period);
					LCD_print(1,((12)*6 ), &xvost,1,0);

					sprintf(xvost,"%0.3d",vsp_zazor);
					LCD_print(2,((12)*6 ), &xvost,1,0);

					sprintf(xvost,"%0.3d",vsp_period);
					LCD_print(3,((12)*6 ), &xvost,1,0);
					pereschet();
					
					redaktor();
					_nop_();
 }


	void  pp (void)
		{ 	xvost[0]='+';
					xvost[1] = 0;
		}

		void mm(void)
			{  	xvost[0]='-';
					xvost[1] = 0;
			}
		void nu(void)
			{  	xvost[0]=' ';
					xvost[1] = 0;
			}

		 void ppmm(void)
		 { if (new_sv == 0)
				{ 	
						{  if ((av != new_av) |(refresh))
							if (ekr == 2)
							{
							if (new_av)
								pp();
							else
								mm();
							LCD_print(1,((13)*6 ), &xvost,1,0);
							}
							av = new_av;
						}
					if ((bk != new_bk)|(refresh))
						{	if (ekr == 2)
							{
					 		if (new_bk)
								pp();
							else
								mm();
							LCD_print(2,((13)*6 ), &xvost,1,0);	
							}			
						}

					 if ((bk != new_bk)|(refresh))
						{	 if (ekr == 2)
							{
					 		if (new_bk)
								mm();
							else
								pp();
							LCD_print(3,((13)*6 ), &xvost,1,0);
							}
							bk=new_bk;

						}
				
						{	
						 	   if (ekr == 2)
							{
								pp();
						
							LCD_print(2,((13)*6 ), &xvost,1,0);
							}
							sv=new_sv;
						}

					if ((ok != new_ok) |(refresh))
						{	 if (ekr == 2)
							{
							if (new_ok)
								pp();
							else
								mm();
							LCD_print(1,((13)*6 ), &xvost,1,0);
							}
							ok=new_ok;
						}
						refresh = 0;
					}
					else
						{	
						
					if (ekr == 2)
							{	
					nu();	
					
					LCD_print(1,((13)*6 ), &xvost,1,0);

					
					LCD_print(2,((13)*6 ), &xvost,1,0);
				
				
					LCD_print(3,((13)*6 ), &xvost,1,0);
				   		
					LCD_print(5,((13)*6 ), &xvost,1,0);
						
					mm();
					LCD_print(4,((13)*6 ), &xvost,1,0);
					}
						sv=new_sv;
				
						refresh =1;
						}

		 }
   void ekran_bp(void)
{ 	  
					

				ochistka_ekrana();
				LCD_print(0,3*6, &row14,1,0);  
				LCD_print(1,3*6, &row15,1,0);  
				LCD_print(2,3*6, &row16,1,0);  
				LCD_print(3,3*6, &row17,1,0); 
				LCD_print(4,3*6, &row18,1,0);  
				LCD_print(5,3*6, &row19,1,0); 
			   	LCD_print(6,3*6, &row20,1,0); 
			
					 temper = 25;
			     	sprintf(xvost,"%+0.3d",(int)temper);
					LCD_print(1,((9)*6 ), &xvost,1,0);

								
					sprintf(xvost,"%0.2u",(int)adc);
					LCD_print(2,((10)*6 ), &xvost,1,0);

									//  dac = 31;
					sprintf(xvost,"%0.2u",(int)dac);
					LCD_print(3,((10)*6 ), &xvost,1,0);


					 xvost[0] = '-';
					 xvost[1] = 0;
					//sprintf(xvost,"%0.3d",vsp_period);
					LCD_print(4,((9)*6 ), &xvost,1,0);
				

					 xvost[0] = '+';
					 xvost[1] = 0;
					//sprintf(xvost,"%0.3d",vsp_period);
					LCD_print(5,((9)*6 ), &xvost,1,0);
				

					

			
					_nop_();

													// int osn_chastota;     // частота импульса 
													// int osn_period;       // период основного импульса 
	
													// int vsp_period;    // частота импульса 
													// int vsp_zazor;     // зазор между импульсами 
		   
				redaktor_bp();
					
				//  while (1);
					_nop_();
 }

    void ekran2(void)
{ 	  
					

				ochistka_ekrana();
				LCD_print(0,3*6, &row4,1,0);  
				LCD_print(1,3*6, &row5,1,0);  
				LCD_print(2,3*6, &row6,1,0);  
				LCD_print(3,3*6, &row7,1,0); 
				LCD_print(4,3*6, &row8,1,0);  
				LCD_print(5,3*6, &row9,1,0); 
				ppmm();	



													// int osn_chastota;     // частота импульса 
													// int osn_period;       // период основного импульса 
	
													// int vsp_period;    // частота импульса 
													// int vsp_zazor;     // зазор между импульсами 
		   
				redaktor2();
					
				
					_nop_();
 }




			/*
   unsigned char razborka1(void)
   		
		{
			unsigned char temp[40],temp2[40];
			unsigned char i,i2,nn;
			unsigned int crc;
		//	tr_buf =&tes;
			if (strlen(tr_buf) > 1)
				{ 
				
				  i = strrpos(tr_buf,',');
				  strncpy(temp,tr_buf+i+1+5,5);	   // скопировали остаток после ','
				  temp[5] =0;
				  crc = atoi(temp);


				  i = strpos(tr_buf,',');

				  strcpy(temp,tr_buf+i+1);	   // скопировали остаток после ','
				  i2 = strpos(temp,',');	   // нашли сл. ','
				  strncpy(temp2,temp,i2);	   // скопировали голову
				  temp2[i2] = 0;
     			  nn = atoi(temp2);            // получили число

				Crc2_send.Int = 0;
  				for (i =0;i<nn-1;i++)
		 				Crc2_send.Int=FastCRC16(tr_buf[i], Crc2_send.Int);
				 if (crc !=	Crc2_send.Int)
				 	return (0);

				  strcpy(temp,temp+i2+1);
				  i2 = strpos(temp,',');
				  strncpy(temp2,temp,i2);
				  temp2[i2] = 0;
				  diagnostica = atoi(temp2);

				  strcpy(temp,temp+i2+1);
				  i2 = strpos(temp,',');
				  strncpy(temp2,temp,i2);
				  temp2[i2] = 0;
				  temper_float = atof(temp2);

				  strcpy(temp,temp+i2+1);
				  i2 = strpos(temp,',');
				  strncpy(temp2,temp,i2);
				  temp2[i2] = 0;
				  temper1_float = atof(temp2);

				  strcpy(temp,temp+i2+1);
				  i2 = strpos(temp,',');
				  strncpy(temp2,temp,i2);
				  temp2[i2] = 0;
				  temper2_float = atof(temp2);

				  strcpy(temp,temp+i2+1);
				  i2 = 5;
				 // i2 = strpos(temp,',');
				  strncpy(temp2,temp,i2);
				  temp2[i2] = 0;
				  temper3_float = atof(temp2);


				_nop_();
				return (1);	
				}

		}
			 */

	//   	sprintf(temp3,"%0.3d,",(int)round(((1.0/a11)*10000)));
	//*****************************************************************************
	//
	//    получение посылки от БП																																	передача мастеру 
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
			 
			 
			 
			 
//#define   ok_     			0x01      // *
//#define   bad_crc_     		0x02      // *
//#define	ready_	            0x04      //  *
//#define	state_	  		    0x08	  // *								//  есть\нет изменение состояния                        999  -  0x3e7    ~ 10 bit      4 0x08 
//#define   dac_	         	0x10	  // *								//  есть\нет изменение состояния БП                       5  0x10        0x18
//#define	key_	        	0x20	  //  *				            	//  есть\нет изменение состояния вкл\откл                 6  0x20        0x28
//#define	temper_   	        0x40	  //  *						    	//  есть\нет изменение состояния температура              7  0x40        0x48
//#define	adc_		        0x80	  //  *								//  есть\нет изменение состояния напряжение               6  0x80        0x88

//#define	assig_		    0x100		//									//  установить изменения состояния по запросу инженерной панели         9 0x100
//#define	zapr_	    	0x200       //



//#define ok_          0x01					
//#define bad_crc_     0x02
//#define onn          0x04
//#define command_ok   0x08
//#define diag         0x01
//#define assigment    0x02
//   bit new_ok_bp,new_crc_bp,new_key_bp,new_state_bp,new_dac_bp,new_temper_bp,new_adc_bp,new_ready_bp,ass_bp;

					//										
					// #,23,001,	96,		16,		45,	51,	  1,1		2345\r\n"		
					//			частота*10   dac    f1  f2   on\off		 crc
					//
					
					// bit new_ok_pult,new_crc_pult,new_key_pult,new_state_pult,new_dac_pult,new_frec1_pult,new_frec2_pult,new_period_pult;
					
					
/*				#define ok_     			0x01      // 1
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
#define	frec2_	    0x04		
*/					
		void diagnoz_pult(int di)
		{
			if ((di & assig_) == assig_)
				{
				new_ass_pult = 1;  		// вообще не при делах
										//ass_command_pult = 0;
										//state_command_pult =1;  //  всегда 1
				}
			
			//////////////////////////////       ok
			if ((di & ok_) == ok_)
				new_ok_pult = 1;
			 else
			 	new_ok_pult = 0;	
			//////////////////////////////	     crc
			if ((di & bad_crc_)== bad_crc_)
				new_crc_pult = 1;
			else										  
			 	new_crc_pult = 0;
			/////////////////////////////        key
			if ((di & key_)== key_)
				new_key_pult = 1;
			else
		   			new_key_pult = 0;
			////////////////////////////		state		state_
			 if ((di & state_)== state_)        
				new_state_pult = 1;
			 else
				new_state_pult = 0;
			///////////////////////////	       dac
			if ((di & dac_)== dac_)
				new_dac_pult = 1;
			 else
				new_dac_pult = 0;	 
			///////////////////////////	      f1
			if ((di & frec1_)== frec1_)
				new_frec1_pult = 1;
			 else
				new_frec1_pult = 0;	 
			///////////////////////////	     f2
			if ((di & frec2_)== frec2_)
				new_frec2_pult = 1;
			 else
				new_frec2_pult = 0;	 
			///////////////////////////	    period
			if ((di & period_)== period_)
				new_period_pult = 1;
			 else
				new_period_pult = 0;	 
			///////////////////////////	 
		}


		void diagnoz_bp(int di)
		{
			
			if ((di & ok_) == ok_)
				new_ok_bp = 1;
			 else
			 	new_ok_bp = 0;	
			//////////////////////////////			  ok_
			if ((di & bad_crc_)== bad_crc_)
				new_crc_bp = 1;
			else										  
			 	new_crc_bp = 0;
			/////////////////////////////			crc
			if ((di & key_)== key_)
				new_key_bp = 1;
			else
		   			new_key_bp = 0;
			////////////////////////////			key
			 if ((di & state_)== state_)
				new_state_bp = 1;
			 else
				new_state_bp = 0;
			///////////////////////////	 		  state
			if ((di & dac_)== dac_)
				new_dac_bp = 1;
			 else
				new_dac_bp = 0;	 
			///////////////////////////	    
			if ((di & temper_)== temper_)
				new_temper_bp = 1;
			 else
				new_temper_bp = 0;	 
			///////////////////////////	 
			if ((di & adc_)== adc_)
				new_adc_bp = 1;
			 else
				new_adc_bp = 0;	 
			///////////////////////////	 
			if ((di & ready_)== ready_)
				new_ready_bp = 1;
			 else
				new_ready_bp = 0;	 
			///////////////////////////	 
		}

				 
	void diagnoz(int di)
		{
			if ((di & ok_) == ok_)
				new_ok = 1;
			 else
			 	new_ok = 0;	
			if ((di & bad_crc_)== bad_crc_)
				new_sv = 1;
			 else										  
			 		new_sv = 0;
		 if ((di & onn)== onn)
				new_bk = 1;
		   else
		   			new_bk = 0;
			 if ((di & command_ok)== command_ok)
				new_ko_ok = 1;
			 else
				 new_ko_ok = 0;
		}
	// temper_11,adc_11,dac_11,ready_bp_11,vkl_bp_11
	

unsigned char razborka_frec1(void)

  		{	 
  		 char *i;
		 unsigned char nn;
		
					//                                 
		 	        // #,17,001,	1,		0,		1,	  0,	  	2345\r\n"		
					//			    on      off    plus  minus   	 crc
  	 																						//Crc_send.Int= 0;
		if (strlen(buf2) > 1)			   // если пакет не нулевой длины
			{
				i = strrchr(buf2,',');
	     		strncpy(buf3,i+1,5);
				 buf3[5]=0;
				 rez2 =  atoi(buf3);
				 Crc2_send.Int = 0;
				 i = strchr(buf2,',');
				// i = strchr(i+1,',');
				 strncpy(buf3,i+1,2);
				 buf3[2] = 0;
			 if (!(   (isdigit(buf3[0]) & isdigit(buf3[1])) & ((nn = atoi(buf3)) < 40)     ))
					return (0);
				 nn  = atoi(buf3);

				//  проверить принятую
				
				// проверить переданную   что делать


  	             for (op =0;op<nn-1;op++)
		            Crc2_send.Int=FastCRC16(buf2[op], Crc2_send.Int);

											// sprintf(buf3,"%#0.5u",Crc2_send.Int); 
											// strncpy(buf3,buf2+15,5);
										    //  buf3[5] =0;
										    //  rez2 =  atoi(buf3);
											//   period_11,dac_11,frec1_11,frec2_11,vkl_pult_11;
		        if( rez2  == Crc2_send.Int)
			       {

														//	i = strchr(buf2,',');
						i = strchr(i+1,',');
						strncpy(buf3,i+1,3);
						buf3[3] = 0;
						diagnoz_pult(atoi(buf3));   // разборка команд полученного пакета от pult
			/*			
						i = strchr(i+1,',');
						strncpy(buf3,i+1,3);
						buf3[4] = 0;
						period_11 = (atoi(buf3));  /// ??????????????????????????????

						i = strchr(i+1,',');
						strncpy(buf3,i+1,2);
						buf3[2] = 0;
						dac_11 = (atoi(buf3));  ///   ?????????????????????

						i = strchr(i+1,',');
						strncpy(buf3,i+1,2);
						buf3[2] = 0;
						frec1_11 = (atoi(buf3));
						frec1 = frec1_11;

						i = strchr(i+1,',');
						strncpy(buf3,i+1,2);
						buf3[2] = 0;
						frec2_11 = (atoi(buf3));
						frec2 = frec2_11;
						i = strchr(i+1,',');
						strncpy(buf3,i+1,1);
						buf3[2] = 0;
						vkl_pult_11 = (atoi(buf3));
*/




					 /*
			          if (rez1 == Crc1_send.Int)
			  	          {
				            new_av = 0;
							strncpy(buf3,buf2+5,3);
							buf3[3]=0;
							diagnoz(atoi(buf3));
							if (new_ko_ok ==1)
								as_ = 0;
						  }
					  else 
							{  	new_av = 1;
								bbb3++;
							}
				   */
																									//buf3[6] =0;
			 										//						LCD_print(2,((12)*6 ), &buf3,1,0);
				   }
		   		else
		   			{	new_av = 1;
						bbb3++;
						return (0);
					}
	
			return (1);
			}
		else
			return (0);
	  }
		





	
	unsigned char razborka_pult(void)

  		{	 
  		 char *i;
		 unsigned char nn;
		 // const unsigned char code tes1[] = "#,34,001,+050,67,30,1,1,27689\n\r "; 
		 // unsigned char  bu[40]="#,21,288,71,80,42,61,1,61096\r\n"; 
		 //                                 
		 	        // #,23,001,	96,		16,		45,	51,	  1,     	2345\r\n"		
					//			частота*10   dac    f1  f2   on\off		 crc
  	 																						//Crc_send.Int= 0;
		if (strlen(buf2) > 1)			   // если пакет не нулевой длины
			{//	i = strlen(tes1);
				i = strrchr(buf2,',');
	     		strncpy(buf3,i+1,5);
				 buf3[5]=0;
				 rez2 =  atoi(buf3);
				 Crc2_send.Int = 0;
				 i = strchr(buf2,',');
				// i = strchr(i+1,',');
				 strncpy(buf3,i+1,2);
				 buf3[2] = 0;
				 			 if (!(   (isdigit(buf3[0]) & isdigit(buf3[1])) & ((nn = atoi(buf3)) < 40)     ))
					return (0);
				 nn  = atoi(buf3);

				//  проверить принятую
				
				// проверить переданную   что делать


  	             for (op =0;op<nn-1;op++)
		            Crc2_send.Int=FastCRC16(buf2[op], Crc2_send.Int);

			  	// sprintf(buf3,"%#0.5u",Crc2_send.Int); 
			    // strncpy(buf3,buf2+15,5);
			   //  buf3[5] =0;
			   //  rez2 =  atoi(buf3);
				//   period_11,dac_11,frec1_11,frec2_11,vkl_pult_11;
		        if( rez2  == Crc2_send.Int)
			       {
						badcrc_command_pult = 0;
					 //	i = strchr(buf2,',');
						i = strchr(i+1,',');
						strncpy(buf3,i+1,3);
						buf3[3] = 0;
						diagnoz_pult(atoi(buf3));   // разборка команд полученного пакета от pult
						i = strchr(i+1,',');
						strncpy(buf3,i+1,3);
						buf3[4] = 0;
						if (stop_priem_pult!=1)               //20.08.14 5:20 
							period_11 = (atoi(buf3));      ///

						i = strchr(i+1,',');
						strncpy(buf3,i+1,2);
						buf3[2] = 0;
						dac_11 = (atoi(buf3));

						i = strchr(i+1,',');
						strncpy(buf3,i+1,2);
						buf3[2] = 0;
						frec1_11 = (atoi(buf3));
						frec1 = frec1_11;

						i = strchr(i+1,',');
						strncpy(buf3,i+1,2);
						buf3[2] = 0;
						frec2_11 = (atoi(buf3));
						frec2 = frec2_11;
						i = strchr(i+1,',');
						strncpy(buf3,i+1,1);
						buf3[1] = 0;
						vkl_pult_11 = (atoi(buf3));





					 /*
			          if (rez1 == Crc1_send.Int)
			  	          {
				            new_av = 0;
							strncpy(buf3,buf2+5,3);
							buf3[3]=0;
							diagnoz(atoi(buf3));
							if (new_ko_ok ==1)
								as_ = 0;
						  }
					  else 
							{  	new_av = 1;
								bbb3++;
							}
				   */
																									//buf3[6] =0;
			 										//						LCD_print(2,((12)*6 ), &buf3,1,0);
				   }
		   		else
		   			{	new_av = 1;
						bbb3++;
						badcrc_command_pult=1;
						return (0);
					}
	
			return (1);
			}
		else
			return (0);
	  }
		
		
		
		// temper_11,adc_11,dac_11,ready_bp_11,vkl_bp_11
					//										
					// #,23,001,	96,		16,		45,	51,	  1,1		2345\r\n"		
					//			частота*10   dac    f1  f2   on\off		 crc
					//
	unsigned char razborka_bp(void)

  		{	 
  		 char *i;
		 unsigned char nn;
		 // const unsigned char code tes1[] = "#,34,001,+050,67,30,1,1,27689\n\r "; 
  	 	//	buf2 = &tes1;																				//Crc_send.Int= 0;
		if (strlen(buf2) > 1)			   // если пакет не нулевой длины
			{//	i = strlen(tes1);
				i = strrchr(buf2,',');
	     		strncpy(buf3,i+1,5);
				 buf3[5]=0;
				 rez2 =  atoi(buf3);
				 Crc2_send.Int = 0;


				 i = strchr(buf2,',');
				// i = strchr(i+1,',');
				 strncpy(buf3,i+1,2);
				 buf3[2] = 0;
				 	 if (!(   (isdigit(buf3[0]) & isdigit(buf3[1])) & ((nn = atoi(buf3)) < 40)     ))
					return (0);
				// nn  = atoi(buf3);

				//  проверить принятую
				
				// проверить переданную   что делать

				  crc = 0;
  	             for (op =0;op<nn-1;op++)
		            Crc2_send.Int=FastCRC16(buf2[op], Crc2_send.Int);

			  	// sprintf(buf3,"%#0.5u",Crc2_send.Int); 
			    // strncpy(buf3,buf2+15,5);
			   //  buf3[5] =0;
			   //  rez2 =  atoi(buf3);

		        if( rez2  == Crc2_send.Int)
			       {
						crc = 1;
					 //	i = strchr(buf2,',');
						i = strchr(i+1,',');
						strncpy(buf3,i+1,3);
						buf3[3] = 0;
						diagnoz_bp(atoi(buf3));   // разборка команд полученного пакета от БП
						i = strchr(i+1,',');
						strncpy(buf3,i+1,4);
						buf3[4] = 0;
						temper_11 = (atoi(buf3));   // температура

						i = strchr(i+1,',');
						strncpy(buf3,i+1,2);
						buf3[2] = 0;
						adc_11 = (atoi(buf3));      // реальное напряжение

						i = strchr(i+1,',');
						strncpy(buf3,i+1,2);
 						buf3[2] = 0;
						dac_12 = (atoi(buf3));      // DAC на БП  +30  !!!!!!!!!!!!!!!!!!!!

						i = strchr(i+1,',');
						strncpy(buf3,i+1,2);
						buf3[2] = 0;
						ready_bp_11 = (atoi(buf3));

						i = strchr(i+1,','); 
						strncpy(buf3,i+1,1);
						buf3[2] = 0;
						vkl_bp_11 = (atoi(buf3));
// что дальше делать будем?
					// если вкл и не готов то придерживаем пускостальным
					//
					//
					//
					//
					//
					



					 /*
			          if (rez1 == Crc1_send.Int)
			  	          {
				            new_av = 0;
							strncpy(buf3,buf2+5,3);
							buf3[3]=0;
							diagnoz(atoi(buf3));
							if (new_ko_ok ==1)
								as_ = 0;
						  }
					  else 
							{  	new_av = 1;
								bbb3++;
							}
				   */
																									//buf3[6] =0;
			 										//						LCD_print(2,((12)*6 ), &buf3,1,0);
				   }
		   		else
		   			{	new_av = 1;
						bbb3++;
						return (0);
					}
	
			return (1);
			}
		else
			return (0);
	  }






   unsigned char razborka_klucy(void)

  		{	 
//  		unsigned char cnt=0,i;

  	 																						//Crc_send.Int= 0;
		if (strlen(buf2) > 1)			   // если пакет не нулевой длины
			{
			   strncpy(buf3,buf2+2,2);
				 buf3[2]=0;
				 	 if (!(   (isdigit(buf3[0]) & isdigit(buf3[1])) & ((rez2 = atoi(buf3)) < 40)     ))
					return (0);
				//rez2 =  atoi(buf3);
					Crc2_send.Int = 0;
  	for (op =0;op<rez2-1;op++)
		 Crc2_send.Int=FastCRC16(buf2[op], Crc2_send.Int);
			  	sprintf(buf3,"%#0.5u",Crc2_send.Int); 
			  strncpy(buf3,buf2+15,5);
			  buf3[5] =0;
			  rez2 =  atoi(buf3);

		  if( rez2  == Crc2_send.Int)
			   {
			   	strncpy(buf3,buf2+9,5);
				 buf3[5] =0;
			  rez1 =  atoi(buf3);
			  if (rez1 == Crc1_send.Int)
			  	{
				new_av = 0;
				strncpy(buf3,buf2+5,3);
				buf3[3]=0;
				diagnoz(atoi(buf3));
				if (new_ko_ok ==1)
					as_ = 0;
				}
				else 
					{  new_av = 1;
						bbb3++;
						return (0);
					}
				
																									//buf3[6] =0;
			 										//						LCD_print(2,((12)*6 ), &buf3,1,0);
				}
		   else
		   		{new_av = 1;
				bbb3++;
				}
		//	tr_ok = 0;
		//  	byte_cnt = 0;


													//	buf3[5] =0x0a;
													//buf3[6] = 0xd;
													//tr_buf = &xvost;
		
													//	TI0 = 1;
													//while (!tr_ok);
													//	_nop_();
			return (1);
			}
		else
			return (0);
	  }

	 unsigned char diagnostika(void)
	{ unsigned char te;
		te=0;
		//	di_ =~di_;
		if (di_)
			te=te | diag;
		else
			te &= ~diag;

		// on_ = ~on_;
		 if (on_)
			te = te | onn;
		 else
		 	te &= ~onn;
		 if (as_)
		 	te = te | assigment;
		 else
		 	te &= ~assigment;
		return  (te);
	}
	
	//**********************************************
	//
	//  передача команд для Частотника 													получение команды от мастера
	//
	// 	"#,21,288, 71,    80,  42, 61,     1,   61096\r\n"; 		
	//           period, dac,  f1  f2     vkl, crc
	//
	//     ok_          - у меня все хорошо - передаю принятый crc и свой скс
	//     bad_crc      - ошибка crc
	//     zapros_      - у меня изменение состояния temper, adc, dac, ready, vkl 
	//     assign_      - провел присвоения dac,vkl
	//     state_       - мое состояние temper, adc, dac, ready, vkl 
	//**********************************************

	
 int comand_frec1(void)
	{  int te;
		te=0;
		
		/////////////////////////////////////////////////////////
	
	
		if (ass_command_frec1 == 1)// определитьЁЁЁЁЁЁЁЁЁЁЁ

				{	
				te = te | assig_;
											//te = te | state_;		
				}
		if (state_command_frec1 == 1) //~~~~~~~~~~~~~~~~~~// определитьЁЁЁЁЁЁЁЁЁЁЁ

				{	
				te = te | state_;
											//te = te | state_;		
				}
		
		/////////////////////////////////////////////////////////

		if (on_command_frec1)
					{
						te = te | on_;
					
						
					}

		////////////////////////////////////////////////////////
		
		////////////////////////////////////////////////////////	
		
		/////////////////////////////////////////////////////////
		if (off_command_frec)
					{
					te = te | off_;
					
					//period_command_pult = 0;   // определитьЁЁЁЁЁЁЁЁЁЁЁ
				
					}	
		if (plus_command_frec)
					{
					te = te | plus_;
					
					//period_command_pult = 0;   // определитьЁЁЁЁЁЁЁЁЁЁЁ
				
					}	
		if (minus_command_frec)
					{
					te = te | minus_;
					
					//period_command_pult = 0;   // определитьЁЁЁЁЁЁЁЁЁЁЁ
				
					}				
		return  (te);
	}	
	

	//**********************************************
	//
	//  передача команд для Пульта 													получение команды от мастера
	//
	// 	"#,21,288, 71,    80,  42, 61,     1,   61096\r\n"; 		
	//           period, dac,  f1  f2     vkl, crc
	//
	//     ok_          - у меня все хорошо - передаю принятый crc и свой скс
	//     bad_crc      - ошибка crc
	//     zapros_      - у меня изменение состояния temper, adc, dac, ready, vkl 
	//     assign_      - провел присвоения dac,vkl
	//     state_       - мое состояние temper, adc, dac, ready, vkl 
	//**********************************************

	
 int comand_pult(void)     // 22.08.14     9:20
	{  int te;
		te=0;
					
		if (badcrc_command_pult)// определитьЁЁЁЁЁЁЁЁЁЁЁ
				te = te | bad_crc_;
		
		if (ok_command_pult == 1)// определитьЁЁЁЁЁЁЁЁЁЁЁ
				te = te | ok_;
	
	
		if (ass_command_pult == 1)// определитьЁЁЁЁЁЁЁЁЁЁЁ
				te = te | assig_;
		
		if (state_command_pult == 1) //~~~~~~~~~~~~~~~~~~// определитьЁЁЁЁЁЁЁЁЁЁЁ
				te = te | state_;
						
		if (key_command_pult)
					te = te | key_;
		
		if (period_command_pult)								//  не применяется висяк
				te = te | period_;
		
		
		return  (te);
	}	
	
	
	//**********************************************
	//
	//  передача команд для БП 													получение команды от мастера
	//  "#,34,001,+050,   67,  30,  1,     1,   27689\n\r\0"; 
	//           temper, adc, dac, ready, vkl, crc
	//
	//     ok_          - у меня все хорошо - передаю принятый crc и свой скс
	//     bad_crc      - ошибка crc
	//     zapros_      - у меня изменение состояния temper, adc, dac, ready, vkl 
	//     assign_      - провел присвоения dac,vkl
	//     state_       - мое состояние temper, adc, dac, ready, vkl 
	//**********************************************
 int comand_bp(void)
	{  int te;
		te=0;
		
		/////////////////////////////////////////////////////////
	
	
		if (dac_command_pult | key_command_pult | !ready_temper_bp | new_crc_bp)

				{	
				te = te | assig_;
				te = te | state_;		
				}
		else
				te = te | state_;
				//te = te | assig_;	
		
		/////////////////////////////////////////////////////////

		if (key_command_pult)
					{
						te = te | key_;
					}

		////////////////////////////////////////////////////////
		
		////////////////////////////////////////////////////////	
		
		/////////////////////////////////////////////////////////
		if (dac_command_pult)
					{
					te = te | dac_;
					dac_command_pult = 0;
					
					ass_command_bp = 1;
			
					}	
		
		
		return  (te);
	}
//#define   ok_     			0x01      // *
//#define   bad_crc_     		0x02      // *
//#define	ready_	            0x04      //  *
//#define	state_	  		    0x08	  // *								//  есть\нет изменение состояния                        999  -  0x3e7    ~ 10 bit      4 0x08 
//#define   dac_	         	0x10	  // *								//  есть\нет изменение состояния БП                       5  0x10        0x18
//#define	key_	        	0x20	  //  *				            	//  есть\нет изменение состояния вкл\откл                 6  0x20        0x28
//#define	temper_   	        0x40	  //  *						    	//  есть\нет изменение состояния температура              7  0x40        0x48
//#define	adc_		        0x80	  //  *								//  есть\нет изменение состояния напряжение               6  0x80        0x88

//#define	assig_		    0x100		//									//  установить изменения состояния по запросу инженерной панели         9 0x100
//#define	zapr_	    	0x200       //



//#define ok_          0x01
//#define bad_crc_     0x02
//#define onn          0x04
//#define command_ok   0x08
//#define diag         0x01
//#define assigment    0x02
//   bit new_ok_bp,new_crc_bp,new_key_bp,new_state_bp,new_dac_bp,new_temper_bp,new_adc_bp,new_ready_bp,ass_bp;

	//**********************************************
	//
	//  передача команд для БП 													получение команды от мастера
	//  "#,34,001,+050,   67,  30,  1,     1,   27689\n\r\0"; 
	//           temper, adc, dac, ready, vkl, crc
	//
	//     ok_          - у меня все хорошо - передаю принятый crc и свой скс
	//     bad_crc      - ошибка crc
	//     zapros_      - у меня изменение состояния temper, adc, dac, ready, vkl 
	//     assign_      - провел присвоения dac,vkl
	//     state_       - мое состояние temper, adc, dac, ready, vkl 
	//**********************************************	
												// temper_11,adc_11,dac_11,ready_bp_11,vkl_bp_11
	void otv_bp(void)
		{
			unsigned int  ij;
			//unsigned int i;
			strcpy(buf1,"3,00,");
			//i  = comand_bp(); 
			sprintf(buf3,"%#3.3d,",(int)comand_bp());	// пакет команд для БП
			strcat(buf1,buf3);
		
			//dac_11 =71;                           ////////?????????????????????????????????????
			sprintf(buf3,"%2.2d,",(int)(dac-30));
			strcat(buf1,buf3);
			ready_bp_11 =1;
			sprintf(buf3,"%1.1d,",(int)ready_bp_11);
			strcat(buf1,buf3);
			vkl_bp_11 =1;
			sprintf(buf3,"%1.1d,",(int)vkl_bp_11);
			strcat(buf1,buf3);

																			//	sprintf(buf3,"%#0.5u,",Crc2_send.Int); 
																					//	strcat(buf1,temp3);
			ij  = strlen(buf1);
			sprintf(buf3,"%#2.2u,",ij);
		
			buf1[2] = buf3[0];
			buf1[3] = buf3[1];
			Crc1_send.Int = 0;
			for (i =0;i<ij-1;i++)
				Crc1_send.Int=FastCRC16(buf1[i], Crc1_send.Int);
			sprintf(buf3,"%#5.5u\n\r",(int)Crc1_send.Int); 
			strcat(buf1,buf3);
			ij=strlen(buf1);



		}

		//**********************************************
	//
	//  передача команд для Частотника 													получение команды от мастера
	//  
	//           
	//				//                                 
		 	        // #,17,001,	1,		0,		1,	  0,	  	2345\r\n"		
					//			    on      off    plus  minus   	 crc
	//     ok_          - у меня все хорошо - передаю принятый crc и свой скс
	//     bad_crc      - ошибка crc
	//     zapros_      - у меня изменение состояния temper, adc, dac, ready, vkl 
	//     assign_      - провел присвоения dac,vkl
	//     state_       - мое состояние temper, adc, dac, ready, vkl 
	//**********************************************	
												// temper_11,adc_11,dac_11,ready_bp_11,vkl_bp_11
	void otv_frec1(void)
		{
			unsigned int  ij;
			//unsigned int i;
			strcpy(buf1,"5,00,");
			//i  = comand_bp(); 
			ass_command_frec1 = 1;
			sprintf(buf3,"%#3.3d,",(int)comand_frec1());	// пакет команд для БП
			strcat(buf1,buf3);
			if ( vkl_pult_11 == 1)
			on_frec1 =1;    
			else
			 on_frec1 =0;                       ////////?????????????????????????????????????
			sprintf(buf3,"%1.1d,",(int)on_frec1);
			strcat(buf1,buf3);
			
			if ( vkl_pult_11 == 1)
			off_frec1 =0;
			else
			off_frec1 =1;
			
			sprintf(buf3,"%1.1d,",(int)off_frec1);
			strcat(buf1,buf3);
			if (frec1_11==0)
				{minus_frec1 =0;
				plus_frec1 =0;
				}
			else if (frec1_11==1)
				{
				minus_frec1 =1;
				plus_frec1 =0;
				}
			else if (frec1_11==2)
				{plus_frec1 =1;
				minus_frec1 =0;
				}
			sprintf(buf3,"%1.1d,",(int)plus_frec1);
			strcat(buf1,buf3);
		//	minus_frec1 =0;
			sprintf(buf3,"%1.1d,",(int)minus_frec1);
			strcat(buf1,buf3);
																			//	sprintf(buf3,"%#0.5u,",Crc2_send.Int); 
																					//	strcat(buf1,temp3);
			ij  = strlen(buf1);
			sprintf(buf3,"%#2.2u,",ij);
		
			buf1[2] = buf3[0];
			buf1[3] = buf3[1];
			Crc1_send.Int = 0;
			for (i =0;i<ij-1;i++)
				Crc1_send.Int=FastCRC16(buf1[i], Crc1_send.Int);
			sprintf(buf3,"%#5.5u\n\r",(int)Crc1_send.Int); 
			strcat(buf1,buf3);
			ij=strlen(buf1);



		}	
		
		
	//**********************************************
	//
	//  передача команд для БП 													получение команды от мастера
	//  "#,34,001,+050,   67,  30,  1,     1,   27689\n\r\0"; 
	//           temper, adc, dac, ready, vkl, crc
	//
	//     ok_          - у меня все хорошо - передаю принятый crc и свой скс
	//     bad_crc      - ошибка crc
	//     zapros_      - у меня изменение состояния temper, adc, dac, ready, vkl 
	//     assign_      - провел присвоения dac,vkl
	//     state_       - мое состояние temper, adc, dac, ready, vkl 
	//**********************************************	
												// temper_11,adc_11,dac_11,ready_bp_11,vkl_bp_11
												// period_11,power_pult_11,frec1_11,frec2_11,vkl_pult__11
	//void otv_bp(void)	
		
		
	void otv_pult(void)
		{
			unsigned int  ij;
			//unsigned int i;
			strcpy(buf1,"2,00,");
			//i  = comand_bp(); 
			sprintf(buf3,"%#0.3d,",(int)comand_pult());	// пакет команд для БП
			strcat(buf1,buf3);
			
			sprintf(buf3,"%0.3d,",(int)osn_chastota);          ////	 period_11
			strcat(buf1,buf3);
			sprintf(buf3,"%0.2d,",(int)adc_pult_11);
			strcat(buf1,buf3);
			//frec1 = 50;  20.08.14 7:54
			sprintf(buf3,"%0.2d,",(int)frec1);
			strcat(buf1,buf3);
			//frec2 = 50;  20.08.14 7:54
			sprintf(buf3,"%0.2d,",(int)frec2);
			strcat(buf1,buf3);
			vkl_pult__11=1;
			sprintf(buf3,"%0.1d,",(int)vkl_pult__11);
			strcat(buf1,buf3);
			
			
																						//	sprintf(buf3,"%#0.5u,",Crc2_send.Int); 
																					//	strcat(buf1,temp3);
			ij  = strlen(buf1);
			sprintf(buf3,"%#0.2u,",ij);
		
			buf1[2] = buf3[0];
			buf1[3] = buf3[1];
			Crc1_send.Int = 0;
			for (i =0;i<ij-1;i++)
				Crc1_send.Int=FastCRC16(buf1[i], Crc1_send.Int);
			sprintf(buf3,"%#0.5u\n\r",Crc1_send.Int); 
			strcat(buf1,buf3);
			ij=strlen(buf1);



		}

	 

   void otv_klucy(void)
		{
		unsigned int  ij;
		unsigned int i;
		strcpy(buf1,"4,00,");
		i  = diagnostika(); 
		sprintf(buf3,"%#0.3u,",i);	
		strcat(buf1,buf3);
					///  osn_chastota=421;
			sprintf(buf3,"%0.3d,",osn_chastota);
					strcat(buf1,buf3);
					sprintf(buf3,"%0.2d,",osn_period);
					strcat(buf1,buf3);
					sprintf(buf3,"%0.2d,",vsp_period);
					strcat(buf1,buf3);
					sprintf(buf3,"%0.2d,",vsp_zazor);
					strcat(buf1,buf3);


	//	sprintf(buf3,"%#0.5u,",Crc2_send.Int); 
	//	strcat(buf1,temp3);
		ij  = strlen(buf1);
  		sprintf(buf3,"%#0.2u,",ij);
		
	    buf1[2] = buf3[0];
		buf1[3] = buf3[1];
		Crc1_send.Int = 0;
		for (i =0;i<ij-1;i++)
		 	Crc1_send.Int=FastCRC16(buf1[i], Crc1_send.Int);
		sprintf(buf3,"%#0.5u\n\r",Crc1_send.Int); 
		strcat(buf1,buf3);
	ij=strlen(buf1);



		}

     void main (void)

{
    	PCA0MD &= ~0x40;  
		
	//	LCD_SET_XY(120,2);
	//		a9=0;
	//a9  |= (1<<2);
//	a9  &= ~(1<<2);
	//	temper = -5;
	//	aaa = 25;
	//	sprintf(xvost,"%+3.3d",(int)temper);	
		  
//	otv2();
	 // razborka_bp();


	   di_=1;
	   on_1=0;
	   as_=1;
	initc();
	 tr_ok=1;
	sek =0;
	taut = 0;


	 av=1;
	 bk=1;
	 sv=0;
	 ok=1;
	 ko_ok=1;
 new_av=1;
 new_bk=1;
 new_sv=0;
 new_ok=1;
 zad_bk = 0;
 refresh = 0;	
 aaa2=0;
 aaa1=0;
 flag_dop = 0;
 bbb3=0;
 selector = 1;
 // period_11 =  osn_chastota;
     adc =33;
  adc_pult_11 = adc;
 // pzu = 1;                      // заблокировать прием с пульта до окончания передачи
  zapet_peredachi=1;
  zapret_priema=1;
 // while (1);	//////////////////////////////////////////////////////

/*
	 while (1)
	 {
	 if (tr_ok ==1)
	 {	
	
	 aaa++;
	 otv();
		//	sprintf(xvost,"%0.5d",aaa);
			P20 = 1;
			tr_ok = 0;
		  	byte_cnt = 0;
			//xvost[5] =0x0a;
			//xvost[6] = 0xd;
			tr_buf = &buf1;

			TI0 = 1;
		//	while (!tr_ok);
	 }
	 }
	 
	 {
	 if (tr_ok == 1)
	 {
	
	 byte_cnt=0;
	 tr_ok=0;
		// while (1);
//	otv();
	 tr_buf = &buf1;
	TI0= 1;
	}
	 }
	 */
	 ;
	 	LCD_init();
		
	//	while (SMB_BUSY);
	//				LCD_out_buf[0]=0xb0;
	//				LCD_out_buf[2]=0x10;
	//				
	//				LCD_Write_Arrey(LCD_WC,1,3,&LCD_out_buf);
	//				while (SMB_BUSY);
	//						 for(a9=0;a9<(132);a9++)
	//					LCD_out_buf[a9]=0x00;
						//SMB_SINGL_BYTE=1;
	//				
						LCD_Write_Arrey(LCD_WD,0,132,&LCD_out_buf);
						_nop_();
		
	//	while (1);

		{	
		//		P20 = 1;
			
		}	
		//	    P20 = 0;
			////////////////////////////////////
	while(1)
		{
	//	   ekran_bp();
	//	ekran2();
				ekran1();
		  	 if (((sek  & 0x01) == 0x01) & (tr_ok))
			 		{  //otv();
							P20 = 1;
							tr_buf = &buf1;
							 TI0 = 1;

					}		
		 
		}
		bbb = 0;  

	while (1)
		{

		   if (read_ok)

		  	{
				read_ok = 0;
				razborka_klucy();
			}



		   	bbb++;
			sprintf(xvost,"%0.5u",bbb);
			tr_ok = 0;
		  	byte_cnt = 0;
			xvost[5] = 0x0a;
			xvost[6] = 0xd;
			tr_buf = &xvost;
		  	TI0 = 1;

			while (!tr_ok);
		}

	  //////////////////////////////////////


}


void		 video(void)
		{ 
		if (null_down == 3)
			{_nop_();}
		else if (null_down == 1)				// &&&&&&& клавиша вниз отпущена &&&&&&&
		  	{	null_down =3;
			  stop_u = 0;
			}
		else if (null_down == 0)				// ::::::: нажата как минимум клавиша вниз ::::::::::
			{ 
			null_down = 3;
			stop_u = 1;
			}
//****************************************88
		if (minus_down == 3)
			{_nop_();}
		else if (minus_down == 1)				// &&&&&&& клавиша вниз отпущена &&&&&&&
		  	{	minus_down =3;
			  start_u = 0;
			}
		else if (minus_down == 0)				// ::::::: нажата как минимум клавиша вниз ::::::::::
			{ 
			minus_down = 3;
			start_u = 1;
			}
//*****************************************
		 if (plus_down == 3)
			{_nop_();}
		else if (plus_down == 1)				// &&&&&&& клавиша вниз отпущена &&&&&&&
		  	{	plus_down =3;
			  plus_u = 0;
			}
		else if (plus_down == 0)				// ::::::: нажата как минимум клавиша вниз ::::::::::
			{ 
			plus_down = 3;
			plus_u = 1;
			}
 //*****************************************
		 if (minus_press == 3)
			{_nop_();}
		else if (minus_press == 1)				// &&&&&&& клавиша вниз отпущена &&&&&&&
		  	{	minus_press =3;
			  minus_u = 0;
			}
		else if (minus_press == 0)				// ::::::: нажата как минимум клавиша вниз ::::::::::
			{ 
			minus_press = 3;
			minus_u = 1;
			}
 //****************************************0
		  if (plus_press == 3)
			{_nop_();}
		else if (plus_press == 1)				// &&&&&&& клавиша вниз отпущена &&&&&&&
		  	{	plus_press =3;
			  null_u = 0;
			}
		else if (plus_press == 0)				// ::::::: нажата как минимум клавиша вниз ::::::::::
			{ 
			plus_press = 3;
			null_u = 1;
			}
	}	



  void uart_int(void) interrupt 4
{

unsigned char z;

 if (RI0)
  {
   	if (byte_cnt1 ==0)
		{
			buf2[byte_cnt1]=SBUF0;
			RI0=0;

			if (buf2[byte_cnt1]==(0x31))
				{
						byte_cnt1++;
						MCE0 =0;

															/*

														  if (buf2[byte_cnt1]=='#')
														  {
														 byte_cnt1++;
														  MCE0 =0;
															   */
													//	  flagi =1;
				}
		   
		}
   else
		{ 
			buf2[byte_cnt1]=SBUF0;
			RI0=0;
			byte_cnt1++;
		   	if((SBUF0==0x0d) || ( byte_cnt1==40 ))
				 {	 
					  buf2[byte_cnt1]=0;
					  byte_cnt1=0;
					  read_ok=1;
					  P20=1;
				 }
		}
	
  }
  
  
  else
		{
			TI0=0;
			if(byte_cnt ==1)
				TB80 = 0;
														//	S0MODE =0;

			if((tr_buf[byte_cnt-1]==0x0d) & (byte_cnt != 0))
			   {	byte_cnt=0; 
																//TI0=1;
																//	S0MODE=1;
					MCE0  = 1;   //1
					TB80  = 1;
					tr_ok = 1;
					P20   = 0;
					flag_taut = 1;      // запустили таймаут приема
					flag_dop = 0;
				}
		   else
			   {
					i =tr_buf[byte_cnt];
					SBUF0=i;
					byte_cnt++;
				}
														//  while(!TI0);

		  z = buf1[0];
		}
}


	//************************
	//
	//	наверное дребезг
	//
	//************************

 void PCA_ISR (void) interrupt 11
{	CCF0 = 0;
	CR = 0;
	PCA0H = 0;
	PCA0L = 0;
	decim++;
	if (decim == 10)
	{EIE2 |= 0x02;
	decim = 0;}
	else
	{CR = 1;}
	

}
 //------------------------------------------------------------------------------------
// SMBus Interrupt Service Routine (ISR)
//------------------------------------------------------------------------------------
//
// SMBus ISR state machine
// - Master only implementation - no slave or arbitration states defined
// - All incoming data is written starting at the global pointer <pSMB_DATA_IN>
// - All outgoing data is read from the global pointer <pSMB_DATA_OUT>
//
void SMBus_ISR (void) interrupt 7
{
   bit FAIL = 0;                             // Used by the ISR to flag failed
                                             // transfers

   static unsigned char i;                            // Used by the ISR to count the
                                             // number of data bytes sent or
                                             // received

   static bit SEND_START = 0;                // Send a start

   switch (SMB0CN & 0xF0)                    // Status vector
   {
      // Master Transmitter/Receiver: START condition transmitted.
      case SMB_MTSTA:
         SMB0DAT = TARGET;                   // Load address of the target slave
         SMB0DAT |= SMB_RW;                  // Load R/W bit
				 if(LCD_COMM)
					SMB0DAT  |= (1<<1);
				 else 
					SMB0DAT  &= ~(1<<1);
				 /*
				 GPIO |= (1<<dclk);
  asm nop
  GPIO &= ~(1<< dclk);
  asm nop
				 */
				 
				 
				 
         STA = 0;                            // Manually clear START bit
         i = 0;                              // reset data byte counter
         break;

      // Master Transmitter: Data byte (or Slave Address) transmitted
      case SMB_MTDB:
         if (ACK)                            // Slave Address or Data Byte 
         {                                   // Acknowledged?
            if (SEND_START)
            {
               STA = 1;
               SEND_START = 0;
               break;
            }/*
            if(SMB_SENDCON)                  // Are we sending the control byte?
            {
               SMB_SENDCON = 0;              // Clear flag
               SMB0DAT = LCD_CON ;            // send control byte
               break;
             }*/

            if (SMB_RW==WRITE)               // Is this transfer a WRITE?
            {

               if (i < SMB_DATA_LEN)         // Is there data to send?
               {
			      if(SMB_SINGL_BYTE) SMB0DAT = pSMB_DATA_OUT[0];

				  else	SMB0DAT = pSMB_DATA_OUT[i]; // send data byte
                  i++; 
			//	  if(SMB_SINGL_SENDCON)SMB_SENDCON = 1;              
               }
               else
               {
                 STO = 1;                    // set STO to terminte transfer
                 SMB_BUSY = 0;               // clear software busy flag
				 SMB_SINGL_BYTE=0;
				 SMB_SENDCON = 0;              
               }
            }
            else {}                          // If this transfer is a READ,
                                             // then take no action. Slave
                                             // address was transmitted. A
                                             // separate 'case' is defined
                                             // for data byte recieved.
         }
         else                                // If slave NACK,
         {
            if(SMB_ACKPOLL)
            {
               STA = 1;                      // Restart transfer
            }
            else
            {
               FAIL = 1;                     // Indicate failed transfer
            }                                // and handle at end of ISR
         }
         break;

      // Master Receiver: byte received
      case SMB_MRDB:
         if ( i < SMB_DATA_LEN )             // Is there any data remaining?
         {
            *pSMB_DATA_IN = SMB0DAT;         // Store received byte
            pSMB_DATA_IN++;                  // Increment data in pointer
            i++;                             // Increment number of bytes received
            ACK = 1;                         // Set ACK bit (may be cleared later
                                             // in the code)

         }

         if (i == SMB_DATA_LEN)              // This is the last byte
         {
            SMB_BUSY = 0;                    // Free SMBus interface
            ACK = 0;                         // Send NACK to indicate last byte
                                             // of this transfer
            STO = 1;                         // Send STOP to terminate transfer
		 }

         break;

      default:
         FAIL = 1;                           // Indicate failed transfer
                                             // and handle at end of ISR
         break;
   }

   if (FAIL)                                 // If the transfer failed,
   {
      SMB0CN &= ~0x40;                       // Reset communication
      SMB0CN |= 0x40;
      SMB_BUSY = 0;                          // Free SMBus
   }

   SI=0;                                     // clear interrupt flag
}

	//***********************************************************
	//
	//	У 411 процессора идет прерывание по изменению состояния
	//  цифрового порта. Мы анализируем состояние клавиш  клавиатуры
	// и датчиков
	//
	//***********************************************************

    void mask_int (void) interrupt INTERRUPT_PORT_MATCH
     {
	 z2 = P1;
	 z=(P1 & P1MASK);            		//   сохранили новое значение порта
	 z1 = (P1MAT & P1MASK);		 		//	 сохранили старое значение порта
	 P1MAT = z;							//   подготовили новое старое
			b_new = (z & 0x80);		 		//	 выделили P1.0 
		b_old = (z1 & 0x80);	 		//   выделили сохраненный P1.0
										//   если в порту клавиша нажата 
	 	if (b_new)	 
										   
			{  if  (b_old)  			//  мы ее и не отпускали
				  {_nop_();}			//   ничего не изменилось
				else
				  {row0_down =1;
                  TMR3CN |= 0x04;}		//	 0 --> 1		 мы ее отпустили  включение TR3
			}
		else 
		 	{	if ( b_old )
					{row0_down =0;
                    TMR3CN &= 0xfb;		 // остановили сканирование клавиатуры
                    row = 1;
                    down();}		//	  1 --> 0		 мы ее нажали
				else
					{_nop_();}			//	  ничего не изменилось
			 }



		b_new = (z & 0x40);		 		//	 выделили P1.1 
		b_old = (z1 & 0x40);	 		//   выделили сохраненный P1.6
	 	if ((b_new))	    			//   если в порту клавиша нажата 
			{  if ( b_old )  			//  мы ее и не отпускали
					{_nop_();}			//   ничего не изменилось
				else
					{row1_down=1;
                    TMR3CN |= 0x04;}			//	 0 --> 1		 мы ее отпустили
			}
		else 
		 	{	if (b_old )
					{row1_down=0;
                    TMR3CN &= 0xfb;
                    row = 2;
                    down();}			//	  1 --> 0		 мы ее нажали
				else
					{_nop_();}			//	  ничего не изменилось
			 }




			 b_new = (z & 0x20);		 		//	 выделили P1.2 
		b_old = (z1 & 0x20);	 		//   выделили сохраненный P1.2
	 	if (b_new )	    				//   если в порту клавиша нажата 
			{  if (b_old )  			//  мы ее и не отпускали
					{_nop_();}			//   ничего не изменилось
				else
					{row2_down=1;
                    TMR3CN |= 0x04;}		//	 0 --> 1		 мы ее отпустили
			}
		else 
		 	{	if  (b_old )
					{row2_down=0;
                    TMR3CN &= 0xfb;
                    row = 3;
                    down();}//	  1 --> 0		 мы ее нажали
				else
					{_nop_();}//	  ничего не изменилось
			 }

		 	
		b_new = (z & 0x10);		 		//	 выделили P1.2 
		b_old = (z1 & 0x10);	 		//   выделили сохраненный P1.2
	 	if (b_new )	    				//   если в порту клавиша нажата 
			{  if (b_old )  			//  мы ее и не отпускали
					{_nop_();}			//   ничего не изменилось
				else
					{ //pusk_down=1;
				//		if (null_down==3)
					  null_down=1;
				//	   else
				//	  selector_down_otl=1;
                    }		//	 0 --> 1		 мы ее отпустили
			}
		else 
		 	{	if  (b_old )
					{ //pusk_down=0;
				
					  null_down=0;
					 
                    	}//	  1 --> 0		 мы ее нажали
				else
					{_nop_();}//	  ничего не изменилось
			 }
	  
	  		b_new = (z & 0x08);		 		//	 выделили P1.2 
		b_old = (z1 & 0x08);	 		//   выделили сохраненный P1.2
	 	if (b_new )	    				//   если в порту клавиша нажата 
			{  if (b_old )  			//  мы ее и не отпускали
					{_nop_();}			//   ничего не изменилось
				else
					{ //pusk_down=1;
				//		if (null_down==3)
					  minus_down=1;
				//	   else
				//	  selector_down_otl=1;
                    }		//	 0 --> 1		 мы ее отпустили
			}
		else 
		 	{	if  (b_old )
					{ //pusk_down=0;
				
					  minus_down=0;
					 
                    	}//	  1 --> 0		 мы ее нажали
				else
					{_nop_();}//	  ничего не изменилось
			 }
	  	b_new = (z & 0x04);		 		//	 выделили P1.2 
		b_old = (z1 & 0x04);	 		//   выделили сохраненный P1.2
	 	if (b_new )	    				//   если в порту клавиша нажата 
			{  if (b_old )  			//  мы ее и не отпускали
					{_nop_();}			//   ничего не изменилось
				else
					{ //pusk_down=1;
				//		if (null_down==3)
					  plus_down=1;
				//	   else
				//	  selector_down_otl=1;
                    }		//	 0 --> 1		 мы ее отпустили
			}
		else 
		 	{	if  (b_old )
					{ //pusk_down=0;
				
					  plus_down=0;
					 
                    	}//	  1 --> 0		 мы ее нажали
				else
					{_nop_();}//	  ничего не изменилось
			 }

		b_new = (z & 0x02);		 		//	 выделили P1.2 
		b_old = (z1 & 0x02);	 		//   выделили сохраненный P1.2
	 	if (b_new )	    				//   если в порту клавиша нажата 
			{  if (b_old )  			//  мы ее и не отпускали
					{_nop_();}			//   ничего не изменилось
				else
					{ //pusk_down=1;
				//		if (null_down==3)
					  minus_press = 1;
				//	   else
				//	  selector_down_otl=1;
                    }		//	 0 --> 1		 мы ее отпустили
			}
		else 
		 	{	if  (b_old )
					{ //pusk_down=0;
				
					  minus_press=0;
					 
                    	}//	  1 --> 0		 мы ее нажали
				else
					{_nop_();}//	  ничего не изменилось
			 }

	 	b_new = (z & 0x01);		 		//	 выделили P1.2 
		b_old = (z1 & 0x01);	 		//   выделили сохраненный P1.2
	 	if (b_new )	    				//   если в порту клавиша нажата 
			{  if (b_old )  			//  мы ее и не отпускали
					{_nop_();}			//   ничего не изменилось
				else
					{ //pusk_down=1;
				//		if (null_down==3)
					  plus_press = 1;
				//	   else
				//	  selector_down_otl=1;
                    }		//	 0 --> 1		 мы ее отпустили
			}
		else 
		 	{	if  (b_old )
					{ //pusk_down=0;
				
					  plus_press=0;
					 
                    	}//	  1 --> 0		 мы ее нажали
				else
					{_nop_();}//	  ничего не изменилось
			 }


    //P0MASK    = 0x80;
							// а правильно ли это ??????????????????????????????
	EIE2 &= ~(1<<1);		//   запрет прерывания от изменения порта на время дребезга
	CR = 1;					// включили счетчик PCA
video();
}  
 	//**************************
	//
	// Сканирование клавиатуры
	//
	//
	//**************************



 void TIMER3_ISR (void) interrupt 14
{
   TMR3CN &= ~0x80;                    // Clear Timer3 Flags

switch (kodd)	
{
	case 0x01:
		{P25 = 1;
		 P21 = 0x00;
		 kodd = 0x02;
         kod = 1;
		 break;
		}
	case 0x02:
		{ P21 = 1;
		  P22 = 0;		//@@@@@@@@@@
		  kodd = 0x03;
          kod = 2;
		   break;
		}
	case 0x03:
		{  P22 = 1;	//@@@@@@@@@@@@
		 P23 = 0;
		  kodd = 0x4;
          kod = 3;
		   break;
		}
	case 0x4:
		{	 P23 = 1;
		P24 = 0;
		  kodd = 0x5;
          kod = 4;

		   break;
		 }
	case 0x5:
	{	 P24 = 1;
		P25 = 0;
		
		 kodd = 0x01;
         kod = 5;
		  break;
		 }
}

  
   
}
 void Timer2_Init (void)
	{
    TMR2CN    = 0x04;
	TMR2L     = 0x4a;  //0x4a;   ///0x3e;
	TMR2H     = 0xa0;  //0xa0;   //0X50;			 // b
	TMR2RLH   = 0xa0;  //0X50;
	TMR2RLL   = 0x4a;  //0X3e;   
	     
	TR2 = 1;           // Timer0 enabled
	}

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
	if ((msek >20) | reset_sek  )										 // 	  &(sek == 0)
		{
			reset_sek = 0;										//	new_av=~new_av;
			flag_sek = 0;
			msek = 0;
		sek=1;
		   	P03 = ~P03;								// тестовая подсветка
		}
		
 
 	}


 unsigned int code crc16LUT[256] = {
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
  const unsigned char code klav [3][5]=
                {{'a','0','3','2','1'},
                 {'b','.','6','5','4'},
                 {'c','d','9','8','7'}};   
  const unsigned char code TABL [155][6]=
                {{ 0x00,0x000,0x000,0x000,0x00, 00},
                  {0x00,0x000,0x04f,0x000,0x00, 00},  //;!
                  {0x00,0x007,0x000,0x007,0x00, 00},  //;"
                  {0x14,0x07f,0x014,0x07f,0x14, 00},  //;#
                  {0x24,0x02A,0x07F,0x02A,0x12, 00},  //;$
                  {0x23,0x013,0x008,0x064,0x62, 00},  //;%
                  {0x36,0x049,0x055,0x022,0x40, 00},  //;&
                  {0x00,0x005,0x003,0x000,0x00, 00},  //;'								 
                  {0x00,0x01c,0x022,0x041,0x00, 00},  //;(
                  {0x00,0x041,0x022,0x01c,0x00, 00},  //;)
                  {0x14,0x008,0x03e,0x008,0x14, 00},  //;*
                  {0x08,0x008,0x03e,0x008,0x08, 00},  //;+
                  {0x00,0x050,0x030,0x000,0x00, 00},  //;,
                  {0x08,0x008,0x008,0x008,0x08, 00},  //;-
                  {0x00,0x060,0x060,0x000,0x00, 00},  //;,
                  {0x20,0x010,0x008,0x004,0x02, 00},  //;/
                  {0x3E,0x051,0x049,0x045,0x3E, 00},  //;0
                  {0x00,0x042,0x07f,0x040,0x00, 00},  //;1
                  {0x42,0x061,0x051,0x049,0x46, 00},  //;2
                  {0x21,0x041,0x045,0x04b,0x31, 00},  //;3
                  {0x18,0x014,0x012,0x07f,0x10, 00},  //;4
                  {0x27,0x045,0x045,0x045,0x39, 00},  //;5
                  {0x3c,0x04a,0x049,0x049,0x30, 00},  //;6
                  {0x01,0x071,0x009,0x005,0x03, 00},  //;7
                  {0x36,0x049,0x049,0x049,0x36, 00},  //;8
                  {0x06,0x049,0x049,0x025,0x1e, 00},  //;9
                  {0x00,0x036,0x036,0x000,0x00, 00},  //;:
                  {0x00,0x056,0x036,0x000,0x00, 00},  //;;
                  {0x08,0x014,0x022,0x041,0x00, 00},  //;<
                  {0x14,0x014,0x014,0x014,0x14, 00},  //;=
                  {0x00,0x041,0x022,0x014,0x08, 00},  //;>
                  {0x02,0x001,0x051,0x009,0x06, 00},  //;?
                  {0x32,0x049,0x079,0x041,0x3e, 00},  //;@
                  {0x7e,0x011,0x011,0x011,0x7e, 00},  //;A
                  {0x7f,0x049,0x049,0x049,0x36, 00},  //;B
                  {0x3e,0x041,0x041,0x041,0x22, 00},  //;C
                  {0x7f,0x041,0x041,0x022,0x1c, 00},  //;D
                  {0x7f,0x049,0x049,0x049,0x41, 00},  //;E
                  {0x7f,0x009,0x009,0x009,0x01, 00},  //;F
                  {0x3e,0x041,0x049,0x049,0x3a, 00},  //;G40
                  {0x7f,0x008,0x008,0x008,0x7f, 00},  //;H
                  {0x00,0x041,0x07f,0x041,0x00, 00},  //;I
                  {0x20,0x040,0x041,0x03f,0x01, 00},  //;J
                  {0x7f,0x008,0x014,0x022,0x41, 00},  //;K
                  {0x7f,0x040,0x040,0x040,0x40, 00},  //;L
                  {0x7f,0x002,0x00c,0x002,0x7f, 00},  //;M
                  {0x7f,0x004,0x008,0x010,0x7f, 00},  //;N
                  {0x3e,0x041,0x041,0x041,0x3e, 00},  //;O
                  {0x7f,0x009,0x009,0x009,0x06, 00},  //;P
                  {0x3e,0x041,0x051,0x021,0x5e, 00},  //;Q50
                  {0x7f,0x009,0x019,0x029,0x46, 00},  //;R
                  {0x46,0x049,0x049,0x049,0x31, 00},  //;S
                  {0x01,0x001,0x07f,0x001,0x01, 00},  //;T
                  {0x3f,0x040,0x040,0x040,0x3f, 00},  //;U
                  {0x1f,0x020,0x040,0x020,0x1f, 00},  //;V
                  {0x3f,0x040,0x070,0x040,0x3f, 00},  //;W
                  {0x63,0x014,0x008,0x014,0x63, 00},  //;X
                  {0x07,0x008,0x070,0x008,0x07, 00},  //;Y
                  {0x61,0x051,0x049,0x045,0x43, 00},  //;Z
                  {0x00,0x07f,0x041,0x041,0x00, 00},  //;[60
                  {0x02,0x004,0x008,0x010,0x20, 00},  //;
                  {0x00,0x041,0x041,0x07f,0x00, 00},  //;]
                  {0x04,0x002,0x001,0x002,0x04, 00},  //;~
                  {0x40,0x040,0x040,0x040,0x40, 00},  //;_
                  {0x00,0x001,0x002,0x000,0x00, 00},  //;`
                  {0x20,0x054,0x054,0x054,0x78, 00},  //;a
                  {0x7f,0x044,0x044,0x044,0x38, 00},  //;b
                  {0x38,0x044,0x044,0x044,0x20, 00},  //;c
                  {0x38,0x044,0x044,0x048,0x7f, 00},  //;d
                  {0x38,0x054,0x054,0x054,0x18, 00},  //;e70
                  {0x08,0x07e,0x009,0x001,0x02, 00},  //;f
                  {0x0c,0x052,0x052,0x052,0x3e, 00},  //;g
                  {0x7f,0x008,0x004,0x004,0x78, 00},  //;h
                  {0x00,0x044,0x07d,0x040,0x00, 00},  //;i
                  {0x20,0x040,0x044,0x03d,0x00, 00},  //;j
                  {0x7f,0x010,0x028,0x044,0x00, 00},  //;k
                  {0x00,0x041,0x07f,0x040,0x00, 00},  //;l
                  {0x7c,0x004,0x018,0x004,0x78, 00},  //;m
                  {0x7c,0x008,0x004,0x004,0x7c, 00},  //;n
                  {0x38,0x044,0x044,0x044,0x38, 00},  //;o80
                  {0x7c,0x014,0x014,0x014,0x08, 00},  //;p
                  {0x08,0x014,0x014,0x014,0x7c, 00},  //;q
                  {0x7c,0x008,0x004,0x004,0x08, 00},  //;r
                  {0x48,0x054,0x054,0x054,0x20, 00},  //;s
                  {0x04,0x03f,0x044,0x040,0x20, 00},  //;t
                  {0x3c,0x040,0x040,0x020,0x7c, 00},  //;u
                  {0x1c,0x020,0x040,0x020,0x1c, 00},  //;v
                  {0x3c,0x040,0x020,0x040,0x3c, 00},  //;w
                  {0x44,0x028,0x010,0x028,0x44, 00},  //;x
                  {0x0c,0x050,0x050,0x050,0x3c, 00},  //;y90
                  {0x44,0x064,0x054,0x04C,0x44, 00},  //;z
                  {0x7f,0x049,0x049,0x049,0x33, 00},  //;Ѓ
                  {0x7f,0x001,0x001,0x001,0x03, 00},  //;ѓ
                  {0x7c,0x055,0x054,0x055,0x00, 00},  //;…
                  {0x77,0x008,0x07f,0x008,0x77, 00},  //;†
                  {0x41,0x049,0x049,0x049,0x36, 00},  //;‡
                  {0x7f,0x010,0x008,0x004,0x7f, 00},  //;€
                  {0x7c,0x021,0x012,0x009,0x7c, 00},  //;©
                  {0x20,0x041,0x03f,0x001,0x7f, 00},  //;‹
                  {0x7f,0x001,0x001,0x001,0x7f, 00},  //;Џ100
                  {0x47,0x028,0x010,0x008,0x07, 00},  //;“
                  {0x1c,0x022,0x07f,0x022,0x1c, 00},  //;”
                  {0x07,0x008,0x008,0x008,0x7f, 00},  //;—
                  {0x7f,0x040,0x07f,0x040,0x7f, 00},  //;
                  {0x01,0x07f,0x048,0x048,0x30, 00},  //;љ
                  {0x7f,0x048,0x030,0x000,0x7f, 00},  //;›
                  {0x22,0x041,0x049,0x049,0x3e, 00},  //;ќ
                  {0x7f,0x008,0x03e,0x041,0x3e, 00},  //;ћ
                  {0x46,0x029,0x019,0x009,0x7f, 00},  //;џ
                  {0x3C,0x04A,0x04A,0x049,0x31, 00},  //;Ў110
                  {0x7c,0x054,0x054,0x028,0x00, 00},  //;ў
                  {0x7c,0x004,0x004,0x004,0x0c, 00},  //;Ј
                  {0x38,0x055,0x054,0x055,0x18, 00},  //;Ґ
                  {0x6c,0x010,0x07c,0x010,0x6c, 00},  //;¦
                  {0x44,0x044,0x054,0x054,0x28, 00},  //;§
                  {0x7c,0x020,0x010,0x008,0x7c, 00},  //;Ё
                  {0x78,0x042,0x024,0x012,0x78, 00},  //;©
                  {0x7c,0x010,0x028,0x044,0x00, 00},  //;Є
                  {0x20,0x044,0x03c,0x004,0x7c, 00},  //;«
                  {0x7c,0x008,0x010,0x008,0x7c, 00},  //;¬120
                  {0x7c,0x010,0x010,0x010,0x7c, 00},  //;­
                  {0x7c,0x004,0x004,0x004,0x7c, 00},  //;Ї
                  {0x04,0x004,0x07c,0x004,0x04, 00},  //;в
                  {0x0c,0x010,0x010,0x010,0x7c, 00},  //;з
                  {0x7c,0x040,0x07c,0x040,0x7c, 00},  //;и
                  {0x04,0x07c,0x050,0x050,0x20, 00},  //;к
                  {0x7c,0x050,0x050,0x020,0x7c, 00},  //;л
                  {0x7c,0x050,0x050,0x020,0x00, 00},  //;м
                  {0x28,0x044,0x054,0x054,0x38, 00},  //;н
                  {0x7c,0x010,0x038,0x044,0x38, 00},  //;о130
                  {0x08,0x054,0x034,0x014,0x7C, 00},  //;п
                  {0x10,0x028,0x044,0x010,0x28, 00},  //;<<
                  {0x28,0x010,0x044,0x028,0x10, 00},  //;>>
                  {0x04,0x002,0x07f,0x002,0x04, 00},  //;  ‘’ђ…‹ЉЂ ‚‚…ђ•
                  {0x10,0x020,0x07f,0x020,0x10, 00},  //;  бваҐ«Є  ў­Ё§
                  {0xe0,0x051,0x04f,0x041,0xff, 00}, //;//„
                  {0x7f,0x040,0x040,0x040,0xff, 00}, //;//–
                  {0x7f,0x040,0x07f,0x040,0xff, 00}, //;//™
                  {0xe0,0x054,0x04c,0x044,0xfc, 00}, //;//¤
                  {0x30,0x048,0x0fe,0x048,0x30, 00}, //;//д140
                  {0x7c,0x040,0x040,0x040,0xfc, 00}, //;//ж
                  {0x7C,0x040,0x07C,0x040,0xFC, 00}, //;й
                  {0x08,0x01c,0x02a,0x008,0x0F, 00}, //;       ўЄ
                  {0x17,0x008,0x014,0x01a,0x7d, 00}, //;1/4
                  {0x17,0x008,0x044,0x056,0x7d, 00}, //;1/3
                  {0x17,0x008,0x034,0x056,0x5d, 00}, //;1/2
                  {0x00,0x0d8,0x0a8,0x098,0x00, 00}, //;   2       Ё­¤ҐЄб
                  {0x0e,0x00a,0x00e,0x000,0x00, 00 }, //   gradus
                  {0x40,0x32,0x09,0x3E,0x40,0x00},   //ALFA
                  {0x90,0x0f8,0x080,0x000,0x00, 00  }, //  1         Ё­¤ҐЄб
                  {0x70,0x028,0x024,0x028,0x70, 00 }, // 150    a    181    д § 
                  {0x7c,0x054,0x054,0x06c,0x00, 00},// ;   b  182      д § 
                  {0x7c,0x044,0x044,0x044,0x00, 00},// ;   c  183      д § 
                  {0x7C,0x044,0x044,0x044,0x00, 00},// ;   0 Ё­ўҐаб­®Ґ
				  {0x30,0x028,0x024,0x028,0x30, 00}};//	
	//#######################################################################################
