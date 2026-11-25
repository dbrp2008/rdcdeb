#ifndef TR_AUTO_APP_H
#define TR_AUTO_APP_H

/* Hardware side of the autonomous routine (see tr_auto.h): reads the IR / ultrasonic / encoders, runs the state
 * machine and drives the robot through the TRDrive hooks. Controlled with the MainBoard buttons:
 *   BTN2 (on the drive page): open the AUTO page; BTN2 again = START (3 s countdown); BTN2 while running = STOP
 *   BTN1 on the AUTO page (not running): switch to the LINE TEST (see tr_linetest.h); BTN1 again: back to the drive page
 * Any big joystick movement stops the routine and hands control back. */

#include <stdbool.h>

void TRAutoApp_Update(void);
bool TRAutoApp_PageActive(void);
bool TRAutoApp_Running(void); /* the routine or the line test is driving */
void TRAutoApp_DrawPage(void);
void TRAutoApp_Btn1(void);
void TRAutoApp_Btn2(void);

#endif
