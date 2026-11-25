#ifndef CALIBRATION_APP_H
#define CALIBRATION_APP_H
/* Entry points used by CubeMX main. Drive starts in stationary IR setup;
 * the separately linked EncoderBench implementation never enables drive. */
void CalibrationApp_Init(void);
void CalibrationApp_Update(void);
#endif
