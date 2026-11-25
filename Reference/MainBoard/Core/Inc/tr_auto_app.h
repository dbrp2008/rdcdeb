#ifndef TR_AUTO_APP_H
#define TR_AUTO_APP_H

/* Hardware side of the autonomous routine (see tr_auto.h): reads the IR / ultrasonic / encoders, runs the state
 * machine and drives the robot through the TRDrive hooks. Controlled with the MainBoard buttons:
 *   BTN2 (on the drive page): open the AUTO page; BTN2 again = START (3 s countdown); BTN2 while running = STOP
 *   BTN1 on the AUTO page (not running): next mode: R1 (blocks 1 + 2) -> R2 (far-right blocks) -> REC -> PLAY ->
 *        PLAY MIRROR -> LINE TEST (tr_linetest.h) -> back to the drive page
 *   REC  (tr_record.h): B2 = start recording HERE (the start pose of every replay), drive the run with the joystick,
 *        B2 = stop, stand still, B2 = save to flash (tr_store.h: kept through power-off and normal re-flashing)
 *   PLAY / PLAY MIRROR: B2 = replay the saved run (3 s countdown) from the same start pose; MIRROR = the other field side
 * Any big joystick movement stops the routine and hands control back. */

#include <stdbool.h>

void TRAutoApp_Update(void);
bool TRAutoApp_PageActive(void);
bool TRAutoApp_Running(void); /* the routine or the line test is driving */
void TRAutoApp_DrawPage(void);
void TRAutoApp_Btn1(void);
void TRAutoApp_Btn2(void);

#endif
