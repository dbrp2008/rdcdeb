/* Closed-loop host sim of F1 going HOME by turning 180 (function1.c with F1_RETURN_TURN): out along the line to the key,
 * target hold, back off (wheels matched as the board does), pivot until the centre IR is on the line, follow it home,
 * cross the start-zone outline (the IRs need not trip together), straight into the zone, stop at the zone's wall (US1).
 * World (mm): the line on y = 0 from the zone outline (x -20..0) to the key at x = 1500; the AR starts with its axle at
 * x = -200 facing +x; the zone's back wall at x = -450. 3 analog IRs 25 mm apart and 2 outer digital IRs 60 mm out, all on
 * the front bar AR_IR_AHEAD mm ahead of the axle. Each IR has its own sensitivity (knob): it reads black up to bias mm
 * off the tape's edge (negative: it needs to be further onto it).
 * Args: [IR ahead of axle mm] [left motor gain in reverse] [IR bias spread mm] [start heading deg] */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "function1.h"

#define TRACK 233.0
#define MM_PER_PULSE (3.14159265 * 65.0 / 115.0)
typedef struct { double x, y, th, vl, vr; int32_t j1, j3; double el, er; } Bot;
static double rev_left = 1.0, ahead = 130.0, bias[5];
static int us_dead; /* US1 gives no valid reading going home (the wall backup must stop it) */
/* dead encoders: from the first tick in phase freeze_phase, the wheels give no more pulses */
static int freeze_phase = -1, frozen; static const char *last_end; static double end_s;
static void bot_tick(Bot *b, LineWheels cmd) {
  /* the zone's wall (x -450): the front (the IR bar) cannot pass it - the wheels stall */
  if (cos(b->th) < -0.7 && b->x + ahead * cos(b->th) <= -450 && cmd.left + cmd.right > 0) { b->vl = b->vr = 0; return; }
  b->vl += (cmd.left * 10.0 * (cmd.left < 0 ? rev_left : 1.0) - b->vl) * 0.15;
  b->vr += (cmd.right * 10.0 - b->vr) * 0.15;
  const double dl = b->vl * 0.005, dr = b->vr * 0.005, ds = (dl + dr) / 2, dth = (dr - dl) / TRACK;
  b->x += ds * cos(b->th + dth / 2); b->y += ds * sin(b->th + dth / 2); b->th += dth;
  b->el += fabs(dl) / MM_PER_PULSE; b->er += fabs(dr) / MM_PER_PULSE;
  if (frozen) b->el = b->er = 0.0;
  while (b->er >= 1.0) { b->er -= 1.0; if (cmd.right > 0) b->j1++; else if (cmd.right < 0) b->j1--; else if (b->vr > 0) b->j1++; else b->j1--; }
  while (b->el >= 1.0) { b->el -= 1.0; if (cmd.left > 0) b->j3--; else if (cmd.left < 0) b->j3++; else if (b->vl > 0) b->j3--; else b->j3++; }
}
/* black at (x, y) for IR i: the line (y = 0, 0 <= x <= 1500, 20 mm wide) or the outline (x -20..0, |y| < 325) */
static double y_sum; static int y_n; /* where the good runs end, across the line (the zone shift) */
static int dead_outer_r, bend; /* the outer right IR never reads black; a 40 mm bump in the line 10-50 cm from the zone */
static double line_y(double x) { return bend && x > 100 && x < 500 ? 40.0 * sin(3.14159265 * (x - 100) / 400) : 0.0; }
static int black(double x, double y, int i) {
  const double e = 10.0 + bias[i];
  if (i == 4 && dead_outer_r) return 0;
  if (x >= 0 && x <= 1500 && fabs(y - line_y(x)) <= e) return 1;
  if (x >= -20 - bias[i] && x <= 0 + bias[i] && fabs(y) < 325) return 1;
  return 0;
}
static double skew = 0; /* IR bar not parallel to the axle: mm further ahead per mm to the right */
static const double lat[5] = {25, 0, -25, 60, -60}; /* L, C, R, outer L, outer R (+ = left of the AR) */
static void irs(const Bot *b, unsigned *mask, int *sl, int *sr) {
  const double fx = cos(b->th), fy = sin(b->th), lx = -fy, ly = fx;
  int v[5];
  for (int i = 0; i < 5; i++) { const double ai = ahead - lat[i] * skew; /* skewed IR bar: + skew = right end further ahead */
    v[i] = black(b->x + ai * fx + lat[i] * lx, b->y + ai * fy + lat[i] * ly, i); }
  *mask = (unsigned)(v[0] | (v[1] << 1) | (v[2] << 2)); *sl = v[3]; *sr = v[4];
}
static int us_mm(const Bot *b) { /* forward: the key face (x 1500) going out, the zone wall (x -450) going home */
  const double fx = cos(b->th), sx = b->x + ahead * fx, sy = b->y + ahead * sin(b->th);
  if (fx > 0.7 && fabs(sy) < 80) return (int)((1500 - sx) / fx);
  if (fx < -0.7 && us_dead) return 9999;
  if (fx < -0.7 && fabs(sy) < 300) return (int)((sx + 450) / -fx);
  return 2000;
}

static int run(double ahead_mm, double rev_gain, double spread, double start_deg, int verbose) {
  ahead = ahead_mm; rev_left = rev_gain;
  for (int i = 0; i < 5; i++) bias[i] = spread * ((i * 37 % 11) / 5.0 - 1.0); /* fixed, unequal knobs */
  Bot b; memset(&b, 0, sizeof b); b.x = -200; b.th = start_deg * 3.14159265 / 180.0;
  /* the board's totals run since boot (NOT reset per run, as on the AR): it feeds them before the start */
  static double odo = 0, arc = 0, hdg = 0;
  F1_Odometry((float)odo, (float)arc); F1_Heading((float)hdg);
  F1_Init();
  uint32_t now = 1000; int32_t oj1 = 0, oj3 = 0;
  bool straight = false; int32_t sj1 = 0, sj3 = 0;
  unsigned mask; int sl, sr; irs(&b, &mask, &sl, &sr);
  F1_Start(now, mask, false); frozen = 0;
  const char *end = NULL; double pivot_end = 0; int pivoted = 0;
  for (int t = 0; t < 12000 && !end; t++, now += 5) {
    { const int32_t a = b.j1 - oj1, c = b.j3 - oj3; oj1 = b.j1; oj3 = b.j3;
      const double dr = abs(a) * MM_PER_PULSE, dl = abs(c) * MM_PER_PULSE; odo += 0.5 * (dl + dr); arc += dl + dr;
      F1_Odometry((float)odo, (float)arc);
      hdg += ((double)a - (-(double)c)) * MM_PER_PULSE / TRACK * 57.29578; /* right forward + / left forward (J3 inverted) */
      F1_Heading((float)hdg); }
    if (t % 2 == 0) { irs(&b, &mask, &sl, &sr); F1_SideLevels(sl, sr); F1_Scan(now, now, mask, mask); }
    const int us = us_mm(&b);
    if (t % 10 == 0) (void)F1_TargetPing(now, (uint32_t)us);
    end = F1_Advance(now);
    if (end) break;
    const F1_Phase ph = F1_PhaseAt(now);
    if (ph == F1_PIVOT) pivoted = 1;
    if (pivoted == 1 && ph == F1_RETURN) { pivot_end = fmod(b.th * 180 / 3.14159265 + 720.0, 360.0); pivoted = 2; }
    F1_Command c = F1_CommandAt(now, (uint32_t)us);
    LineWheels w = {0, 0};
    if (c.request == F1_STOP) { end = c.status; break; }
    if (c.request == F1_MOVE) {
      w = c.wheels;
      if (F1_EncoderStraight(now)) { /* the board's wheel matching */
        if (!straight) { straight = true; sj1 = b.j1; sj3 = b.j3; }
        const double dr = abs(b.j1 - sj1), dl = abs(b.j3 - sj3);
        double kl = 1 - 0.08 * (dl - dr), kr = 1 - 0.08 * (dr - dl);
        kl = kl < 0.3 ? 0.3 : (kl > 1 ? 1 : kl); kr = kr < 0.3 ? 0.3 : (kr > 1 ? 1 : kr);
        w.left *= kl; w.right *= kr;
      } else straight = false;
    }
    if (freeze_phase >= 0 && (int)ph == freeze_phase) frozen = 1;
    bot_tick(&b, w);
    end_s = t * 0.005;
  }
  last_end = end;
  for (int i = 0; i < 60; i++) bot_tick(&b, (LineWheels){0, 0});
  const double hd = fmod(b.th * 180 / 3.14159265 + 720.0, 360.0);
  if (verbose && end && !us_dead) { y_sum += b.y; ++y_n; } /* facing home (-x), left of the line is -y */
  if (verbose)
    printf("  IR %3.0f mm ahead, left motor x%.2f in reverse, IR knobs +-%.0f mm, start %+3.0f deg: %-16s at (%5.0f, %+4.0f), "
           "heading %5.1f (180 = home), pivot ended at %5.1f\n",
           ahead_mm, rev_gain, spread, start_deg, end ? end : "(time)", b.x, b.y, hd, pivot_end);
  const int ok = end && !strcmp(end, us_dead ? "HOME AT WALL" : "TARGET LINE END") && b.x < -150 && b.x > -330 && fabs(b.y) < 60 &&
                 fabs(hd - 180) < 12 &&
                 fabs(pivot_end - 180) < 25; /* the pivot ends on the line's EDGE (closer IRs: earlier); the forward follow straightens it */
  return ok;
}

int main(void) {
  setvbuf(stdout, NULL, _IONBF, 0);
  int fails = 0;
  const double aheads[] = {100, 130, 160}, revs[] = {1.0, 0.85}, spreads[] = {0, 4}, starts[] = {0, 3, -3};
  for (unsigned a = 0; a < 3; a++)
    for (unsigned r = 0; r < 2; r++)
      for (unsigned s = 0; s < 2; s++)
        for (unsigned h = 0; h < 3; h++)
          if (!run(aheads[a], revs[r], spreads[s], starts[h], 1)) fails++;
  printf("  - the measured AR: IRs 120 mm ahead of the right wheel, 105 mm of the left (bar skewed), both signs:\n");
  for (int sg = -1; sg <= 1; sg += 2) {
    skew = sg * 15.0 / TRACK;
    for (unsigned r = 0; r < 2; r++)
      for (unsigned h = 0; h < 3; h++)
        if (!run(112.5, revs[r], 4, starts[h], 1)) fails++;
  }
  skew = 0;
  dead_outer_r = 1; printf("  - the outer right IR never sees the outline (finish on the zone's wall near home):\n");
  for (unsigned h = 0; h < 3; h++) if (!run(130, 1.0, 4, starts[h], 1)) fails++;
  dead_outer_r = 0; bend = 1; printf("  - a 4 cm bump in the line 10-50 cm before the zone (followed, not cut straight):\n");
  for (unsigned h = 0; h < 3; h++) if (!run(130, 0.85, 4, starts[h], 1)) fails++;
  bend = 0; us_dead = 1; printf("  - US1 never sees the wall going home (stops when the wheels stall against it):\n");
  for (unsigned h = 0; h < 3; h++) if (!run(130, 1.0, 4, starts[h], 1)) fails++;
  us_dead = 0;
  printf("  - an encoder dies mid-run: a time limit stops it:\n");
  { const struct { F1_Phase ph; const char *name, *want; } at[] = {
      {F1_PIVOT, "the 180 pivot", "LINE NOT FOUND"}, {F1_RETURN, "the return", "HOME BY TIME"}};
    for (unsigned i = 0; i < 2; i++) {
      freeze_phase = (int)at[i].ph; (void)run(130, 1.0, 0, 0, 0); freeze_phase = -1;
      const int ok = last_end && !strcmp(last_end, at[i].want);
      printf("    encoders dead from %-13s: %-16s after %5.2f s %s\n", at[i].name, last_end ? last_end : "(never stopped)", end_s,
             ok ? "ok" : "FAIL");
      if (!ok) fails++;
      if (at[i].ph == F1_PIVOT && F1_LastPivotDeg() > 5.0f) { printf("    FAIL: screen pivot angle %.0f with dead encoders\n", F1_LastPivotDeg()); fails++; }
    } }
  { (void)run(130, 1.0, 4, starts[0], 1); /* the screen's pivot readout keeps the angle the line was picked up at */
    const float d = F1_LastPivotDeg();
    const int ok = d >= F1_PIVOT_MIN_DEG && d <= F1_PIVOT_MAX_DEG;
    printf("  - screen PIVOT readout after a good run: %.0f deg %s\n", d, ok ? "ok" : "FAIL");
    if (!ok) fails++; }
  { const double y = y_n ? y_sum / y_n : 0.0; /* F1_ZONE_SHIFT_MM: the AR ends left of the line, square (heading checked per run) */
    const int ok = y_n > 10 && (F1_ZONE_SHIFT_MM > 0.0f ? (y < -15.0 && y > -50.0) : fabs(y) < 15.0);
    printf("  - into the zone shifted %.0f mm left of the line on average (%d runs, F1_ZONE_SHIFT_MM %.0f) %s\n", -y, y_n,
           (double)F1_ZONE_SHIFT_MM, ok ? "ok" : "FAIL");
    if (!ok) fails++; }
  printf(fails ? "F1 HOME BY TURNING: %d FAILED\n" : "F1 home by turning 180: out, hold, back off, pivot onto the line, follow home, "
                 "outline (IRs tripping unevenly), straight into the zone, stop at the wall PASS\n", fails);
  return fails != 0;
}
