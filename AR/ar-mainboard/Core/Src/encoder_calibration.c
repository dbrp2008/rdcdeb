#include "encoder_calibration.h"

#define BUTTON_DEBOUNCE_MS 30u

static uint32_t j1_baseline, j3_baseline, changed_ms;
static bool candidate, stable, armed;

void EncoderCalibration_Init(uint32_t now_ms, bool pressed, uint32_t j1_edges, uint32_t j3_edges) {
  j1_baseline = j1_edges;
  j3_baseline = j3_edges;
  candidate = stable = pressed;
  armed = !pressed;
  changed_ms = now_ms;
}

void EncoderCalibration_Update(uint32_t now_ms, bool pressed, uint32_t j1_edges, uint32_t j3_edges) {
  if (pressed != candidate) {
    candidate = pressed;
    changed_ms = now_ms;
  }
  if (candidate != stable && (uint32_t)(now_ms - changed_ms) >= BUTTON_DEBOUNCE_MS)
    stable = candidate;
  if (!stable) armed = true;
  if (stable && armed) {
    j1_baseline = j1_edges;
    j3_baseline = j3_edges;
    armed = false;
  }
}

uint32_t EncoderCalibration_J1(uint32_t edges) { return edges - j1_baseline; }
uint32_t EncoderCalibration_J3(uint32_t edges) { return edges - j3_baseline; }
