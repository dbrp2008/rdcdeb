#ifndef MOCK_LCD_H
#define MOCK_LCD_H
#include <stdint.h>
void tft_printc(uint8_t,uint8_t,const char *);
void tft_prints(uint8_t,uint8_t,const char *,...);
void tft_update(uint32_t);
#endif
