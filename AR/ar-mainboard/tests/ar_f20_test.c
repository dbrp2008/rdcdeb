/* Host test: F20 (20 cm forward) goes STRAIGHT with badly matched motors. Differential-drive model: each wheel's speed follows
 * its command with a lag, one motor is weaker, one starts later, the encoders count whole edges. Heading and sideways offset
 * come from the real wheel travel. (It cannot catch different wheel SIZES: equal edges then still curve - AR_DRIFT_TRIM_PERMILLE.) */
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979
#endif
#include <stdio.h>
#include <string.h>
#include "ar_nav.h"
static int fails;
#define CHECK(c) do{ if(!(c)){printf("FAIL line %d: %s\n",__LINE__,#c);fails++;} }while(0)
static int run(double gain_l, double gain_r, int delay_l_ms, int delay_r_ms) {
  AR_Nav n; AR_Nav_Init(&n); AR_Nav_SetKey(&n, AR_KEY_F20);
  uint32_t t = 1000; AR_Nav_Start(&n, t);
  double vl = 0, vr = 0, el = 0, er = 0, x = 0, y = 0, th = 0, worst = 0; AR_Cmd c = {0, 0}; uint32_t moving_since = 0;
  const double k = 141.0 / 35.0 / 1000.0;                 /* edges per ms per percent: ~25 cm/s at 35 % */
  const double mm_per_edge = 10.0 / AR_EDGES_PER_CM, track = AR_TRACK_MM;
  for (int i = 0; i < 20000; i++) {
    t += 2;
    if ((c.left || c.right) && !moving_since) moving_since = t;
    const double tl = (moving_since && t - moving_since >= (uint32_t)delay_l_ms) ? c.left * k * gain_l : 0;
    const double tr = (moving_since && t - moving_since >= (uint32_t)delay_r_ms) ? c.right * k * gain_r : 0;
    vl += (tl - vl) * 2.0 / 80.0; vr += (tr - vr) * 2.0 / 80.0;   /* 80 ms motor lag (also the roll after the stop) */
    const double dl = vl * 2, dr = vr * 2;                      /* edges this tick */
    el += fabs(dl); er += fabs(dr);
    const double sl = dl * mm_per_edge, sr = dr * mm_per_edge, ds = (sl + sr) / 2, dth = (sl - sr) / track; /* + = turning right */
    x += ds * cos(th + dth / 2); y += ds * sin(th + dth / 2); th += dth;
    if (fabs(th) > worst) worst = fabs(th);
    AR_Nav_SetWheelOdometer(&n, (uint32_t)el, (uint32_t)er);
    AR_Line none = {0, 0, 0};
    c = AR_Nav_Step(&n, t, none, -1, false);
    if (n.phase == AR_PH_DONE && fabs(vl) < 1e-4 && fabs(vr) < 1e-4) break;
  }
  const double deg = th * 180 / M_PI;
  printf("  motors L x%.2f R x%.2f, start delay L %d R %d ms: %5.1f mm forward, %+5.1f mm sideways, heading %+5.2f deg (worst %4.2f)\n",
         gain_l, gain_r, delay_l_ms, delay_r_ms, x, y, deg, worst * 180 / M_PI);
  CHECK(n.phase == AR_PH_DONE);
  CHECK(fabs(x - AR_F20_CM * 10.0) < 12);    /* 20 cm (+- the coast) */
  CHECK(fabs(y) < 4.0);                      /* straight: under 4 mm off over 20 cm */
  CHECK(fabs(deg) < 1.5);
  return 0;
}
int main(void) {
  run(1.00, 1.00, 0, 0);
  run(0.85, 1.00, 0, 0);   /* left motor 15 % weaker */
  run(1.00, 0.80, 0, 0);   /* right motor 20 % weaker */
  run(1.00, 1.00, 40, 0);  /* left starts 40 ms late */
  run(0.85, 1.00, 0, 40);  /* both */
  if (fails) { printf("AR F20 FAILED (%d)\n", fails); return 1; }
  printf("AR F20 STRAIGHT PASS\n");
  return 0;
}
