#ifndef MOCK_LCD_H
#define MOCK_LCD_H
#include <stdint.h>
#define WHITE 1u
#define BLACK 0u
#define CYAN 2u
#define DARK_GREY 3u
typedef enum { PIN_ON_TOP } TFT_ORIENTATION;
void tft_init(TFT_ORIENTATION,uint16_t,uint16_t,uint16_t,uint16_t);
void tft_printc(uint8_t,uint8_t,const char *);
void tft_printu(uint8_t,uint8_t,uint32_t);
void tft_printl(uint8_t,uint8_t,int64_t);
void tft_prints(uint8_t,uint8_t,const char *,...);
void tft_update(uint32_t);
#endif
