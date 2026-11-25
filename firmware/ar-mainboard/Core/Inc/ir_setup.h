#ifndef IR_SETUP_H
#define IR_SETUP_H
#include <stdbool.h>
#include <stdint.h>

/* The exact three thresholds/polarities shown on the setup TFT, L/C/R. */
typedef struct { uint16_t t[3]; bool black_high[3]; } IR_Profile;

/* Stationary calibration UI. The caller owns hardware initialization. */
void IRSetup_Init(uint32_t now);
/* False leaves *applied untouched. On true, stop polling setup until reboot. */
bool IRSetup_Update(uint32_t now, IR_Profile *applied);
/* The wheel encoder counts (right J1, left J3) for the setup page to show (digital page only; ignored otherwise).
 * Until this is called the page shows "--". */
void IRSetup_Wheels(uint32_t right, uint32_t left); /* raw encoder pulses (any speed) */
/* bottom row: the powered wheel check (wheel_spin.h). wheel 'L'/'R', or 0 before any hold; live = spinning now */
void IRSetup_Spin(char wheel, bool live, uint32_t pulses);
#endif
