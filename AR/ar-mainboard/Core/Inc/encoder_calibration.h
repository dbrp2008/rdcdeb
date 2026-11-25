#ifndef ENCODER_CALIBRATION_H
#define ENCODER_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

/* These are raw rising-edge counts, not signed distance or proven travel. */
void EncoderCalibration_Init(uint32_t now_ms, bool pressed, uint32_t j1_edges, uint32_t j3_edges);
void EncoderCalibration_Update(uint32_t now_ms, bool pressed, uint32_t j1_edges, uint32_t j3_edges);
uint32_t EncoderCalibration_J1(uint32_t edges);
uint32_t EncoderCalibration_J3(uint32_t edges);

#endif
