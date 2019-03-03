#ifndef _TFT_1_8_H
#define _TFT_1_8_H
#include "sys.h"
extern const unsigned char gImage_qqp[3200];

typedef unsigned char uchar;

void TFT1_8_init(void);
void LCD_WR_CMD(uchar index, unsigned int val);
void set_data(uchar data);
void lcd_rs(uchar v);
void lcd_cs(uchar v);
void lcd_wr(uchar v);
void lcd_rst(uchar v);
void flash_rst(uchar v);
void flash_cs(uchar v);
void LCD_WR_REG(uchar index);
void LCD_MM();
void LCD_WR_Data(unsigned int val);
#endif
