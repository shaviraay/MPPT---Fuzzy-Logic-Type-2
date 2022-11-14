// this file modified by eka prasetyono november 2015
// Program study electro intrustry engineering
// eka@pens.ac.id

#include "stm32f4xx_hal.h"
#include <stdio.h>
char data_f[7];

//*** <<< Use Configuration Wizard in Context Menu >>> ***
//
//=========================================================================== LCD Text Configuration
// <e0> Text LCD Configuration <modified by eka>
//   <o1.0..2> LCD DATA PORT
//     <i> Default: PORT = GPIOB
//        <0=> GPIOA (sometimes used for ADC)
//        <1=> GPIOB (set as the alternate function of JTAG pin)
//        <2=> GPIOC (only for Chip 64 pin and above)
//        <3=> GPIOD (only for Chip 100 pin and above)
//        <4=> GPIOE (only for Chip 100 pin and above)
//        <5=> GPIOF (only for Chip 100 pin and above)
//        <6=> GPIOG (only for Chip 144 pin and above)
//   <o2.0..3> Data Position  (LCD D4-D7) 
//     <i> Default: Data = Bit 0-3
//        <0=> Data = Bit 0-3
//        <1=> Data = Bit 1-4  
//        <2=> Data = Bit 2-5
//        <3=> Data = Bit 3-6
//        <4=> Data = Bit 4-7
//        <5=> Data = Bit 5-8
//        <6=> Data = Bit 6-9
//        <7=> Data = Bit 7-10
//        <8=> Data = Bit 8-11 
//        <9=> Data = Bit 9-12
//        <10=> Data = Bit 10-13
//        <11=> Data = Bit 11-14
//        <12=> Data = Bit 12-15
//   <o3.0..2> LCD RS PORT
//     <i> Default: RS_PORT = GPIOB
//        <0=> GPIOA (sometimes used for ADC)
//        <1=> GPIOB (set as the alternate function of JTAG pin)
//        <2=> GPIOC (only for Chip 64 pin and above)
//        <3=> GPIOD (only for Chip 100 pin and above)
//        <4=> GPIOE (only for Chip 100 pin and above)
//        <5=> GPIOF (only for Chip 100 pin and above)
//        <6=> GPIOG (only for Chip 144 pin and above)
//   <o4.0..15> LCD RS BIT
//     <i> Default: RS_BIT = Bit 4 
//		  <0x0001=> RS = Bit 0
//		  <0x0002=> RS = Bit 1
//		  <0x0004=> RS = Bit 2
//		  <0x0008=> RS = Bit 3
//		  <0x0010=> RS = Bit 4
//		  <0x0020=> RS = Bit 5
//		  <0x0040=> RS = Bit 6
//		  <0x0080=> RS = Bit 7
//		  <0x0100=> RS = Bit 8
//		  <0x0200=> RS = Bit 9
//		  <0x0400=> RS = Bit 10
//		  <0x0800=> RS = Bit 11
//		  <0x1000=> RS = Bit 12
//		  <0x2000=> RS = Bit 13
//		  <0x4000=> RS = Bit 14
//		  <0x8000=> RS = Bit 15
//   <o5.0..2> LCD RW PORT
//     <i> Default: RW_PORT = GPIOB
//        <0=> GPIOA (sometimes used for ADC)
//        <1=> GPIOB (set as the alternate function of JTAG pin)
//        <2=> GPIOC (only for Chip 64 pin and above)
//        <3=> GPIOD (only for Chip 100 pin and above)
//        <4=> GPIOE (only for Chip 100 pin and above)
//        <5=> GPIOF (only for Chip 100 pin and above)
//        <6=> GPIOG (only for Chip 144 pin and above)
//   <o6.0..15> LCD RW BIT
//     <i> Default: RW_BIT = Bit 5 
//		  <0x0001=> RW = Bit 0
//		  <0x0002=> RW = Bit 1
//		  <0x0004=> RW = Bit 2
//		  <0x0008=> RW = Bit 3
//		  <0x0010=> RW = Bit 4
//		  <0x0020=> RW = Bit 5
//		  <0x0040=> RW = Bit 6
//		  <0x0080=> RW = Bit 7
//		  <0x0100=> RW = Bit 8
//		  <0x0200=> RW = Bit 9
//		  <0x0400=> RW = Bit 10
//		  <0x0800=> RW = Bit 11
//		  <0x1000=> RW = Bit 12
//		  <0x2000=> RW = Bit 13
//		  <0x4000=> RW = Bit 14
//		  <0x8000=> RW = Bit 15
//	<o7.0..2> LCD Enable PORT
//     <i> Default: Enable_PORT = GPIOB
//        <0=> GPIOA (sometimes used for ADC)
//        <1=> GPIOB (set as the alternate function of JTAG pin)
//        <2=> GPIOC (only for Chip 64 pin and above)
//        <3=> GPIOD (only for Chip 100 pin and above)
//        <4=> GPIOE (only for Chip 100 pin and above)
//        <5=> GPIOF (only for Chip 100 pin and above)
//        <6=> GPIOG (only for Chip 144 pin and above)
//   <o8.0..15> LCD Enable BIT
//     <i> Default: Enable_BIT = Bit 5 
//		  <0x0001=> Enable = Bit 0
//		  <0x0002=> Enable = Bit 1
//		  <0x0004=> Enable = Bit 2
//		  <0x0008=> Enable = Bit 3
//		  <0x0010=> Enable = Bit 4
//		  <0x0020=> Enable = Bit 5
//		  <0x0040=> Enable = Bit 6
//		  <0x0080=> Enable = Bit 7
//		  <0x0100=> Enable = Bit 8
//		  <0x0200=> Enable = Bit 9
//		  <0x0400=> Enable = Bit 10
//		  <0x0800=> Enable = Bit 11
//		  <0x1000=> Enable = Bit 12
//		  <0x2000=> Enable = Bit 13
//		  <0x4000=> Enable = Bit 14
//		  <0x8000=> Enable = Bit 15
// </e> End Text LCD Configuration


//*** <<< end of configuration section >>>    ***


#define __LCD_CHAR            		         	1              //  harus 1
#define __LCD_DATA_PORT          	0x000000003              //  port lcd data bite 4-7
#define __LCD_DATA_BIT          	0x000000004              //  posisi Data 
#define __LCD_RS_PORT            	0x000000003              //  PORT RS
#define __LCD_RS_BIT            	0x000000001              //  posisi Bit RS
#define __LCD_RW_PORT            	0x000000003              //  PORT RW
#define __LCD_RW_BIT            	0x000000002              //  posisi Bit RW
#define __LCD_EN_PORT            	0x000000003              //  PORT Enable
#define __LCD_EN_BIT            	0x000000004              //  posisi Bit Enable


// Deklarasikan Port sambungan LCD 
#if __LCD_CHAR
// inisalisasi LCD karakter 
    #if ((__LCD_DATA_PORT & 3)==0)	
		#define LCD_PORT   	GPIOA->ODR
		#define LCD_DATA_PORT   	GPIOA
	#elif ((__LCD_DATA_PORT & 2)==1)
		#define LCD_PORT   	GPIOB->ODR
		#define LCD_DATA_PORT   	GPIOB
	#elif ((__LCD_DATA_PORT & 7)==2)
		#define LCD_PORT   	GPIOC->ODR
		#define LCD_DATA_PORT   	GPIOC
	#elif ((__LCD_DATA_PORT & 7)==3)
		#define LCD_PORT   	GPIOD->ODR
		#define LCD_DATA_PORT   	GPIOD
	#elif ((__LCD_DATA_PORT & 7)==4)
		#define LCD_PORT   	GPIOE->ODR
		#define LCD_DATA_PORT   	GPIOE
	#elif ((__LCD_DATA_PORT & 7)==5)
		#define LCD_PORT   	GPIOF->ODR
    #elif ((__LCD_DATA_PORT & 7)==6)
		#define LCD_PORT   	GPIOG->ODR
		#define LCD_DATA_PORT   	GPIOG
  #endif	

	#if ((__LCD_RS_PORT & 7)==0)	
		#define LCD_RS		GPIOA
	#elif ((__LCD_RS_PORT & 7)==1)
		#define LCD_RS   	GPIOB
	#elif ((__LCD_RS_PORT & 7)==2)
		#define LCD_RS   	GPIOC
	#elif ((__LCD_RS_PORT & 7)==3)
		#define LCD_RS   	GPIOD
	#elif ((__LCD_RS_PORT & 7)==4)
		#define LCD_RS   	GPIOE
	#elif ((__LCD_RS_PORT & 7)==5)
		#define LCD_RS   	GPIOF
    #elif ((__LCD_RS_PORT & 7)==6)
		#define LCD_RS   	GPIOG
    #endif	

	#if ((__LCD_RW_PORT & 7)==0)	
		#define LCD_RW   	GPIOA
	#elif ((__LCD_RW_PORT & 7)==1)
		#define LCD_RW   	GPIOB
	#elif ((__LCD_RW_PORT & 7)==2)
		#define LCD_RW   	GPIOC
	#elif ((__LCD_RW_PORT & 7)==3)
		#define LCD_RW   	GPIOD
	#elif ((__LCD_RW_PORT & 7)==4)
		#define LCD_RW   	GPIOE
	#elif ((__LCD_RW_PORT & 7)==5)
		#define LCD_RW   	GPIOF
    #elif ((__LCD_RW_PORT & 7)==6)
		#define LCD_RW   	GPIOG
  #endif


	#if ((__LCD_EN_PORT & 7)==0)	
		#define LCD_EN   	GPIOA
	#elif ((__LCD_EN_PORT & 7)==1)
		#define LCD_EN   	GPIOB
	#elif ((__LCD_EN_PORT & 7)==2)
		#define LCD_EN   	GPIOC
	#elif ((__LCD_EN_PORT & 7)==3)
		#define LCD_EN   	GPIOD
	#elif ((__LCD_EN_PORT & 7)==4)
		#define LCD_EN   	GPIOE
	#elif ((__LCD_EN_PORT & 7)==5)
		#define LCD_EN   	GPIOF
    #elif ((__LCD_EN_PORT & 7)==6)
		#define LCD_EN   	GPIOG
  #endif	
//
									  
//#define lcd_wait_const 	stm32_GetPCLK1() / 3538461 //5538461
//#define lcd_wait_const 	SystemCoreClock / 4038461
#define lcd_wait_const 336000000 /2074784   //15574784    //1769227  //8074784  //2074784
#define lcd_delay		10000	 

void wait_lcd(unsigned long int xx)
{  xx*= lcd_wait_const;	
   while(xx--);
}

void lcd_en_clk(void)
{		HAL_GPIO_WritePin(GPIOD,__LCD_EN_BIT,GPIO_PIN_SET);  
	 	wait_lcd(10);					 			// tunggu 2us
		HAL_GPIO_WritePin(LCD_EN,__LCD_EN_BIT,GPIO_PIN_RESET);  
}

void lcd_ins(unsigned short xx)
{		HAL_GPIO_WritePin(LCD_RS,__LCD_RS_BIT,GPIO_PIN_RESET); 
		LCD_PORT  = (LCD_PORT & ~(0xF << __LCD_DATA_BIT)) | (xx<<__LCD_DATA_BIT);	// kirim data
		lcd_en_clk();
}

void lcd_ins2(unsigned short xx)
{		HAL_GPIO_WritePin(LCD_RS,__LCD_RS_BIT,GPIO_PIN_SET);  
		LCD_PORT  = (LCD_PORT & ~(0xF << __LCD_DATA_BIT)) | (xx<<__LCD_DATA_BIT);	// kirim data
		lcd_en_clk();
}

void lcd_cmd (unsigned char cmd)
{		lcd_ins ((cmd>>4) & 0x0F);	// kirim nibble high	
		lcd_ins (cmd & 0x0F);		// kirim nibble low
		wait_lcd(50);				// tunggu 0.05ms
}

void lcd_data (unsigned char dat)
{		lcd_ins2 ((dat>>4) & 0x0F);	// kirim nibble high	
		lcd_ins2 (dat & 0x0F);		// kirim nibble low
		wait_lcd(50);				// tunggu 0.05ms
}
					  
void lcd_reset(void)
{		LCD_PORT |= (0xF<<__LCD_DATA_BIT);
		HAL_GPIO_WritePin(LCD_RS,__LCD_RS_BIT,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(LCD_EN,__LCD_EN_BIT,GPIO_PIN_SET); 
		wait_lcd(20000);			// tunggu 20ms
		lcd_ins(3);					// reset #1
		wait_lcd(15000);			// tunggu 15ms
		lcd_ins(3);					// reset #2
		wait_lcd(5000);				// tunggu 5ms
		lcd_ins(3);					// reset #3
		wait_lcd(5000);				// tunggu 5ms
		lcd_ins(2);					// set data transfer 4 bit						
		wait_lcd(5000);				// tunggu 5ms
}

void lcd_init(void)
{		
	GPIO_InitTypeDef GPIO_InitStructure; 
	/********************** Init GPIO LCD *************************/
 	// aktivasi RS bit
	GPIO_InitStructure.Pin = __LCD_RS_BIT;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_RS, &GPIO_InitStructure);
	// aktivasi RW bit
	GPIO_InitStructure.Pin = __LCD_RW_BIT;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_RW, &GPIO_InitStructure);
	// aktivasi EN bit
	GPIO_InitStructure.Pin = __LCD_EN_BIT;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_EN, &GPIO_InitStructure);
	// aktivasi Data bit
  GPIO_InitStructure.Pin = 0xF << __LCD_DATA_BIT;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_DATA_PORT, &GPIO_InitStructure);
	// mode write
	HAL_GPIO_WritePin(LCD_RW,__LCD_RW_BIT,GPIO_PIN_RESET);
/********************** End GPIO LCD ********************/

// inisialisasi LCD dimulai
	  lcd_reset();
		lcd_cmd(0x28);				//LCD yang digunakan  = data 4 bit, 2 baris, 5x7 dots
		wait_lcd(1000);		
		lcd_cmd(0x0c);				//Display ON cursor OFF
		wait_lcd(1000);		
		lcd_cmd(0x06);				//Set entry mode - auto increement 
		wait_lcd(1000);		
		lcd_cmd(0x01);
		wait_lcd(2000);		
		lcd_cmd(0x80);
		wait_lcd(2000);		
}

void lcd_gotoxy(unsigned char x, unsigned char y)
{		HAL_GPIO_WritePin(LCD_RS,__LCD_RS_BIT,GPIO_PIN_RESET);
	switch(y){
		case 0:lcd_cmd(0x00 +0x80+ x); 		break;
		case 1:lcd_cmd(0x40 +0x80+ x);			break;
		case 2:lcd_cmd(0x14 +0x80+ x);     break;
		case 3:lcd_cmd(0x54 +0x80+ x);     break;
	}
}

void lcd_clear(void)
{	 lcd_cmd(1);
	 wait_lcd(2000);				// tunggu 2ms
}

// menampilkan string ke lCD

void lcd_puts(const char *xx)
{	while(*xx) lcd_data(*xx++);
}

void lcd_Test(unsigned char xx)
{		HAL_GPIO_WritePin(LCD_EN,__LCD_EN_BIT,GPIO_PIN_RESET); 
	  HAL_GPIO_WritePin(LCD_RS,__LCD_RS_BIT,GPIO_PIN_SET);
	  LCD_PORT  = (LCD_PORT & ~(0xF << __LCD_DATA_BIT)) | (xx<<__LCD_DATA_BIT);	// kirim data	
}


// menampilkan data integer ke LCD

void lcd_float6(float xx)
{
	sprintf(data_f,"%3.4f",xx);
	lcd_data(data_f[0]);
	lcd_data(data_f[1]);
	lcd_data(data_f[2]);
	lcd_data(data_f[3]);
	lcd_data(data_f[4]);
	lcd_data(data_f[5]);
}

// menampilkan data integer ke LCD

void lcd_bit(unsigned char xx)
{  	if(xx==0)
 	lcd_data('0');
   	else
	lcd_data('1');
}

void lcd_sen(unsigned char xx)
{  unsigned char i;
   for(i=0;i<4;i++)
   {	if (!(xx&(1<<i))) 	lcd_data('0');
   	else         		lcd_data('1');
   }
}

void lcd_us(unsigned short xx){
   lcd_data((xx%10000)/1000 + 0x30);
   lcd_data((xx%1000)/100 + 0x30);
   lcd_data((xx%100)/10 + 0x30);
   lcd_data(xx%10 + 0x30);
}

void lcd_uint16(unsigned short xx)
{  lcd_data(xx/10000 + 0x30);
   lcd_data((xx%10000)/1000 + 0x30);
   lcd_data((xx%1000)/100 + 0x30);
   lcd_data((xx%100)/10 + 0x30);
   lcd_data(xx%10 + 0x30);
}

void lcd_uint8(unsigned short xx){
   lcd_data((xx%1000)/100 + 0x30);
   lcd_data((xx%100)/10 + 0x30);
   lcd_data(xx%10 + 0x30);
}

void lcd_int16(short int xx) 
{  if(xx<0)
   {   lcd_data('-');
       xx=-xx;
   }
   else lcd_data(' ');	      
   lcd_uint16(xx); 
}

void lcd_int8(short int xx){
	if(xx<0){
		lcd_data('-');
		xx=-xx;
	}
   	else lcd_data(' ');	      
   	lcd_uint8(xx);
}

// menampilkan data dalam bentuk hexa desimal

void lcd_hex (unsigned char xx)
{  if (xx<10) lcd_data(xx + 48);	   	// menampilkan angka
   else       lcd_data(xx + 55);		// menampilkan huruf
} 

void lcd_bin (unsigned char xx)
{  unsigned char i;
   for(i=0;i<8;i++)
   {	if (!(xx&(1<<i))) 	lcd_data('0');
   	else         		lcd_data('1');
   }
}

void lcd_bin16 (unsigned char xx)
{  unsigned char i;
   for(i=0;i<16;i++)
   {	if (!(xx&(1<<i))) 	lcd_data('0');
   	else         		lcd_data('1');
   }
}

void lcd_hex8 (unsigned char xx)
{  lcd_hex(xx>>4);
   lcd_hex(xx & 15);
} 


void lcd_hex16(unsigned short int xx) 
{  lcd_hex((xx>>12) & 0xF);
   lcd_hex((xx>>8) & 0xF);
   lcd_hex((xx>>4) & 0xF);
   lcd_hex(xx & 0xF);
}

void lcd_hex32(unsigned long int xx) 
{  lcd_hex16((xx>>16) & 0xFFFF);
   lcd_hex16(xx & 0xFFFF);
}

#endif
