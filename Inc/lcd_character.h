/* rutin LCD matrix text 
   oleh Eko Henfri B
   copyright(c) ERRC 2011
*/

#ifndef __LCD_TEXT_
#define __LCD_TEXT_

extern void lcd_Test(unsigned char xx);
extern void wait_lcd(unsigned long int xx);
extern void lcd_ins(unsigned short xx);
extern void lcd_ins2(unsigned short xx);
extern void lcd_cmd(unsigned char cmd);
extern void lcd_data(unsigned char dat);
extern void lcd_reset(void);
extern void lcd_init(void);
extern void lcd_gotoxy(unsigned char x, unsigned char y);
extern void lcd_clear(void);
extern void lcd_bit(unsigned char xx);
// menampilkan string ke lCD
extern void lcd_puts(const char *xx);
// menampilkan data integer ke LCD
extern void lcd_uint16(unsigned short xx);
extern void lcd_int16 (short int xx); 
extern void lcd_int8 (short int xx); 
extern void lcd_float6(float xx);
extern void lcd_sen(unsigned char xx);
extern void lcd_us(unsigned short xx);
extern void lcd_uint8(unsigned short xx);
// menampilkan data dalam bentuk hexa desimal
extern void lcd_bin (unsigned char xx);
extern void lcd_bin16 (unsigned char xx);
extern void lcd_hex (unsigned char xx);
extern void lcd_hex8 (unsigned char xx);
extern void lcd_hex16(unsigned short int xx); 
extern void lcd_hex32(unsigned long int xx); 

// animasi tampilan string dari LCD
// xx = pesan yang ditampilkan
// x = baris dari lcd yang di tulis	, jika x = 1 -> baris = 2, selain itu baris = 1 
extern void lcd_tengah(unsigned char kolom, const char *xx, unsigned char x);
extern void lcd_hapus_tengah(unsigned char kolom, unsigned char x);
extern void lcd_pinggir(unsigned char kolom, const char *xx, unsigned char x);
extern void lcd_hapus_pinggir(unsigned char kolom, unsigned char x);
extern void lcd_kiri(unsigned char kolom, const char *xx, unsigned char x);
extern void lcd_hapus_kiri(unsigned char kolom, unsigned char x);
extern void lcd_kanan(unsigned char kolom, const char *xx, unsigned char x);
extern void lcd_hapus_kanan(unsigned char kolom, unsigned char x);
#endif
