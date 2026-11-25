#ifndef LINE_DRIVE_H
#define LINE_DRIVE_H
#include <stdbool.h>

/* Shared physical wheel commands. Forward is positive on either wheel;
 * the board, not a route controller, applies the wiring polarity. */
typedef struct { float left, right; } LineWheels;
/* All F1/F2 motion uses 90% of the base duty. Apply once in the controller
 * (or the F2 branch-pivot pair), never in the board writer shared with F3. */
#define F12_DUTY(base_pct) ((base_pct) * 0.90f)

#define LINE_NOMINAL_PERCENT 25.0f
#define LINE_STEER_MEAN_PERCENT 18.75f
#define LINE_STEER_BIAS_PERCENT 7.75f
/* F1/F2: strict thresholds (equality is clear). F3 has its own immediate stop. */
#define DRIVE_MM_LABEL_(n) #n "MM"
#define DRIVE_MM_LABEL(n) DRIVE_MM_LABEL_(n)
#define DRIVE_US1_STOP_MM   60  /* inhibit start; when running latch the approach */
#define DRIVE_US1_FINISH_MM 55  /* F1/F2 only: armed and currently all white on return */

/* Internal bit order is L/C/R = bits 0/1/2, not printed binary order. */
enum { LINE_LEFT = 1u, LINE_CENTRE = 2u, LINE_RIGHT = 4u };
static inline bool Line_MultipleBlack(unsigned mask) {
  return (mask & (mask - 1u)) != 0u; /* two or more sensors black */
}
static inline LineWheels Line_ForwardSplit(unsigned mask, float inside, float outside) {
  if (mask == LINE_LEFT) return (LineWheels){inside, outside};
  if (mask == LINE_RIGHT) return (LineWheels){outside, inside};
  return (LineWheels){LINE_NOMINAL_PERCENT, LINE_NOMINAL_PERCENT};
}
/* Base steering (unscaled for F3): 11% inside / 26.5% outside. */
static inline LineWheels Line_Forward(unsigned mask) {
  return Line_ForwardSplit(mask, LINE_STEER_MEAN_PERCENT - LINE_STEER_BIAS_PERCENT,
                           LINE_STEER_MEAN_PERCENT + LINE_STEER_BIAS_PERCENT);
}
#endif
