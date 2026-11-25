#ifndef TR_CONTROLS_H
#define TR_CONTROLS_H

/**
 * Titan Robot controls (joystick). Sends over USART1 (HC-05) every TR_SEND_PERIOD_MS:
 *   M+0.50/-0.25\n  left stick  X (strafe right +), Y (forward +)
 *   R-0.40\n        right stick X (rotate clockwise +)
 *   GO / GC / GS\n  Button 8 only = open, Button 7 only = close (swap with MECH_SWAP_78), otherwise (release / both) = stop
 *   S0..S9\n        Speed level 1..10 (S0 = level 1): Button 5 = one step up, Button 6 = one step down (10 levels, default 4)
 *   H0 / H1 / HZ\n  Button 1 toggles heading-relative driving (H1), Button 4 zeroes the heading (HZ)
 *   YL / YR / YU / Y0\n  Button 2 = turn 90 left, Button 3 = 90 right, both = 180
 * Sticks are read from ADC1 PA0 (LX), PA1 (LY), PA2 (RX) by polling; no CubeMX ADC setup is required.
 */

#define TR_SEND_PERIOD_MS 50U

/** Set the button pins to pull-up (buttons pull the pin LOW when pressed). Call once before the main loop. */
void TRControls_Init(void);

/** Call every main-loop pass: reads the buttons, sends the packet when due, updates the TFT. */
void TRControls_Update(void);

#endif
