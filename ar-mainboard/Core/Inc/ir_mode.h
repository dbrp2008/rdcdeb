#ifndef IR_MODE_H
#define IR_MODE_H
/* The three front line IRs (L PA2, C PC2, R PC3).
 * 1 (default): DIGITAL modules, like the side IRs L2/R2 - 1 on black, 0 on the floor, threshold set by each module's
 *   own knob. The pins are plain GPIO inputs, no pull (MainBoard.ioc: GPIO_Input, ADC1 removed). There is no boot
 *   white/black capture: B1+B2 on the IR page goes straight to drive. IR_ReadAll gives 4095 for black, 0 for white,
 *   so the drive's 2-scan average and threshold (IR_DIGITAL_T) work unchanged: "filtered" black = black on both scans.
 * 0: the old analog sensors on ADC1 (needs ADC1 IN2/IN12/IN13 back in the ioc) with the boot calibration. */
#ifndef IR_DIGITAL
#define IR_DIGITAL 1
#endif
#define IR_DIGITAL_BLACK 4095u
#define IR_DIGITAL_T 2048u /* > this = black: 1 of 2 scans black averages to exactly 2048 = white */
#endif
