/* Closed-loop host sim of F2 going HOME by turning 180 (function1.c, F2_RETURN_TURN): out along the bottom line, the F2
 * right turn at the T (function2.c itself, driven as calibration_app.c does: 8 cm past the T, then the 5:3 pivot), up the vertical line to the
 * top key, hold, back off, pivot 180 (F1's turn), down the vertical line, LEFT turn at the T (the mirror of the outbound
 * one), home along the bottom line, outline, straight into the zone, stop at the wall.
 * World (mm, from the ARA map at the rulebook's 300 mm start zone): x = west of the start-zone outline (x -20..0), y = SOUTH
 * (so north, the vertical line, is on the AR's right going out). Bottom line y = 0; the T's vertical line x = 782 up to
 * the top key's face at y = -841; the diagonal leaves it at y = -716 toward the south-east; the U-curve joins the bottom line
 * at x = 483 from the north; the big curve's tip is at (931, -716). The zone's wall at x = -450. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "function1.h"
#include "function2.h"

#define PI 3.14159265358979
#define TRACK 233.0
#define MM_PER_PULSE (PI * 65.0 / 115.0)
typedef struct { double x, y, th, vl, vr; int32_t j1, j3; double el, er; } Bot; /* th: heading, 0 = +x (west), +PI/2 = +y (south) */
static double rev_left = 1.0, ahead = 130.0, bias[5];
static int us_dead;
/* dead encoders: from the first tick whose status contains freeze_on, the wheels give no more pulses */
static const char *freeze_on, *last_end; static int frozen; static double end_s;
static void bot_tick(Bot *b, LineWheels cmd) {
  if (cos(b->th) < -0.7 && b->x + ahead * cos(b->th) <= -450 && cmd.left + cmd.right > 0) { b->vl = b->vr = 0; return; }
  b->vl += (cmd.left * 10.0 * (cmd.left < 0 ? rev_left : 1.0) - b->vl) * 0.15;
  b->vr += (cmd.right * 10.0 - b->vr) * 0.15;
  /* (x west, y south) is right-handed like (east, north): a left turn increases th (west -> south) */
  const double dl = b->vl * 0.005, dr = b->vr * 0.005, ds = (dl + dr) / 2, dth = (dr - dl) / TRACK;
  b->x += ds * cos(b->th + dth / 2); b->y += ds * sin(b->th + dth / 2); b->th += dth;
  b->el += fabs(dl) / MM_PER_PULSE; b->er += fabs(dr) / MM_PER_PULSE;
  if (frozen) b->el = b->er = 0.0;
  while (b->er >= 1.0) { b->er -= 1.0; if (cmd.right > 0) b->j1++; else if (cmd.right < 0) b->j1--; else if (b->vr > 0) b->j1++; else b->j1--; }
  while (b->el >= 1.0) { b->el -= 1.0; if (cmd.left > 0) b->j3--; else if (cmd.left < 0) b->j3++; else if (b->vl > 0) b->j3--; else b->j3++; }
}
static double seg(double px, double py, double ax, double ay, double bx, double by) {
  const double vx = bx - ax, vy = by - ay, t0 = ((px - ax) * vx + (py - ay) * vy) / (vx * vx + vy * vy), t = t0 < 0 ? 0 : (t0 > 1 ? 1 : t0);
  return hypot(px - ax - t * vx, py - ay - t * vy);
}
static int black(double x, double y, int i) {
  const double e = 10.0 + bias[i];
  if (x >= -20 - bias[i] && x <= 0 + bias[i] && fabs(y) < 325) return 1;                    /* the zone outline */
  if (seg(x, y, 0, 0, 2000, 0) <= e) return 1;                                              /* bottom line */
  if (seg(x, y, 782, 0, 782, -870) <= e) return 1;                                          /* the T's vertical line */
  if (seg(x, y, 782, -716, 782 - 0.894 * 600, -716 + 0.447 * 600) <= e) return 1;           /* the diagonal */
  { const double r = hypot(x - 483, y + 200); if (x >= 483 && fabs(r - 200) <= e) return 1; } /* U-curve (west half) */
  if (seg(x, y, 161, -400, 483, -400) <= e) return 1;                                       /* ...its top arm */
  { const double r = hypot(x - 931, y + 331), ph = atan2(x - 931, -(y + 331));              /* big curve, from its tip */
    if (fabs(r - 385) <= e && ph >= -0.05 && ph <= 2.6) return 1; }
  return 0;
}
static double skew = 0; /* IR bar not parallel to the axle: mm further ahead per mm to the right */
static const double lat[5] = {25, 0, -25, 60, -60}; /* L, C, R, outer L, outer R (+ = left of the AR) */
static void irs(const Bot *b, unsigned *mask, int *sl, int *sr) {
  const double fx = cos(b->th), fy = sin(b->th), lx = -fy, ly = fx; /* left of the heading (going west: south) */
  int v[5];
  for (int i = 0; i < 5; i++) { const double ai = ahead - lat[i] * skew; /* skewed IR bar: + skew = right end further ahead */
    v[i] = black(b->x + ai * fx + lat[i] * lx, b->y + ai * fy + lat[i] * ly, i); }
  *mask = (unsigned)(v[0] | (v[1] << 1) | (v[2] << 2)); *sl = v[3]; *sr = v[4];
}
static int us_mm(const Bot *b) { /* the top key's face going north; the zone's wall going home */
  const double fx = cos(b->th), fy = sin(b->th), sx = b->x + ahead * fx, sy = b->y + ahead * fy;
  if (fy < -0.7 && fabs(sx - 782) < 80 && sy > -841) return (int)((sy + 841) / -fy);
  if (fx < -0.7 && us_dead) return 9999;
  if (fx < -0.7 && fabs(sy) < 300) return (int)((sx + 450) / -fx);
  return 2000;
}

static int run(double ahead_mm, double rev_gain, double spread, double start_deg, int verbose) {
  ahead = ahead_mm; rev_left = rev_gain;
  for (int i = 0; i < 5; i++) bias[i] = spread * ((i * 37 % 11) / 5.0 - 1.0);
  Bot b; memset(&b, 0, sizeof b); b.x = -200; b.th = start_deg * PI / 180.0;
  static double odo = 0, arc = 0, hdg = 0; /* since boot, as on the board */
  F1_Odometry((float)odo, (float)arc); F1_Heading((float)hdg);
  F1_Init();
  uint32_t now = 1000; int32_t oj1 = 0, oj3 = 0;
  unsigned mask; int sl, sr; irs(&b, &mask, &sl, &sr);
  F1_StartPerScan(now, mask);
  /* function2.c's outbound right turn at the T (3rd crossing), fed and driven as calibration_app.c does */
  F2_Init(); F2_Start(sl || sr); frozen = 0;
  int prev_side = sl || sr; bool adv_on = false, piv_on = false; double adv_odo = 0, piv_arc0 = 0; int32_t aj1 = 0, aj3 = 0;
  bool straight = false; int32_t sj1 = 0, sj3 = 0;
  const char *end = NULL; double t_turn_end = 0; int t_seen = 0;
  for (int t = 0; t < 16000 && !end; t++, now += 5) {
    { const int32_t a = b.j1 - oj1, c = b.j3 - oj3; oj1 = b.j1; oj3 = b.j3;
      const double dr = abs(a) * MM_PER_PULSE, dl = abs(c) * MM_PER_PULSE; odo += 0.5 * (dl + dr); arc += dl + dr;
      F1_Odometry((float)odo, (float)arc);
      hdg += ((double)a - (-(double)c)) * MM_PER_PULSE / TRACK * 57.29578;
      F1_Heading((float)hdg); }
    const F1_Phase ph0 = F1_PhaseAt(now);
    if (t % 2 == 0) {
      irs(&b, &mask, &sl, &sr); F1_SideLevels(sl, sr);
      { const int hi = sl || sr; const BranchSample bs = {now, now, hi != 0, hi && !prev_side}; prev_side = hi;
        if (ph0 == F1_OUTBOUND || (ph0 == F1_LAUNCH && !F2_Turning())) F2_Update(bs, mask); }
      F1_Scan(now, now, mask, mask);
    }
    F1_BranchTurning(F2_Turning());
    F2_Tick(now); /* the board's time backups, every drive tick */
    if (F2_Failed()) { end = F2_Status(); break; }
    const int us = us_mm(&b);
    if (t % 10 == 0 && !F2_Turning()) (void)F1_TargetPing(now, (uint32_t)us);
    end = F1_Advance(now);
    if (end) break;
    LineWheels w = {0, 0}; const char *lab = "F2 TURN RIGHT";
    const int f2_out = F1_PhaseAt(now) == F1_OUTBOUND;
    if (f2_out && F2_Advancing()) { if (!adv_on) { adv_on = true; adv_odo = odo; aj1 = b.j1; aj3 = b.j3; } F2_Advanced((float)(odo - adv_odo)); }
    else adv_on = false;
    if (f2_out && F2_Advancing()) { /* straight past the T, wheels matched on the encoders */
      const double dr = abs(b.j1 - aj1), dl = abs(b.j3 - aj3);
      double kl = 1 - 0.08 * (dl - dr), kr = 1 - 0.08 * (dr - dl);
      kl = kl < 0.3 ? 0.3 : (kl > 1 ? 1 : kl); kr = kr < 0.3 ? 0.3 : (kr > 1 ? 1 : kr);
      w = (LineWheels){F12_DUTY(LINE_NOMINAL_PERCENT) * kl, F12_DUTY(LINE_NOMINAL_PERCENT) * kr}; lab = F2_Status();
    } else if (f2_out && F2_Turning()) {
      if (!piv_on) { piv_on = true; piv_arc0 = arc; }
      F2_PivotAngle((float)((arc - piv_arc0) / TRACK * 57.29578));
      if (F2_Failed()) { end = F2_Status(); break; }
      w = (LineWheels){F2_LEFT_FORWARD_PCT, -F2_RIGHT_REVERSE_PCT}; F2_PivotCommanded(); lab = F2_Status();
    }
    else {
      piv_on = false;
      F1_Command c = F1_CommandAt(now, (uint32_t)us);
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
      lab = c.status;
      if (c.status && !strcmp(c.status, "HOME: T LEFT")) t_seen = 1;
      else if (t_seen == 1) { t_seen = 2; t_turn_end = fmod(b.th * 180 / PI + 720.0, 360.0); }
    }
    if (getenv("TRACE")) {
      static const char *last = NULL; (void)0;
      if (lab != last && (!last || !lab || strcmp(lab, last))) {
        printf("    t=%6.2f %-18s x %6.0f y %+5.0f hdg %5.1f mask %u%u%u side %d%d\n", t * 0.005, lab ? lab : "-", b.x, b.y,
               fmod(b.th * 180 / PI + 720.0, 360.0), mask & 1, (mask >> 1) & 1, (mask >> 2) & 1, sl, sr);
        last = lab;
      }
    }
    if (freeze_on && lab && strstr(lab, freeze_on)) frozen = 1;
    bot_tick(&b, w);
    end_s = t * 0.005;
  }
  last_end = end;
  for (int i = 0; i < 60; i++) bot_tick(&b, (LineWheels){0, 0});
  const double hd = fmod(b.th * 180 / PI + 720.0, 360.0);
  if (verbose)
    printf("  IR %3.0f mm ahead, left x%.2f in reverse, knobs +-%.0f, start %+2.0f deg: %-17s at (%5.0f, %+4.0f) heading %5.1f "
           "(180 = home), T turn ended at %5.1f\n", ahead_mm, rev_gain, spread, start_deg, end ? end : "(time)", b.x, b.y, hd, t_turn_end);
  return end && !strcmp(end, us_dead ? "HOME AT WALL" : "TARGET LINE END") && b.x < -150 && b.x > -330 && fabs(b.y) < 60 &&
         fabs(hd - 180) < 12;
}

int main(void) {
  setvbuf(stdout, NULL, _IONBF, 0);
  if (getenv("ONE")) return !run(getenv("AHEAD") ? atof(getenv("AHEAD")) : 130, 1.0, 0, 0, 1);
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
  us_dead = 1; printf("  - US1 never sees the wall going home:\n");
  for (unsigned h = 0; h < 3; h++) if (!run(130, 1.0, 4, starts[h], 1)) fails++;
  us_dead = 0;
  printf("  - an encoder dies mid-run: a timeout stops it (no driving / spinning away):\n");
  { static const char *const at[][2] = {{"F2 PAST THE T", "F2 ADV TIMEOUT"}, {"F2 TURN RIGHT", "F2 TURN TIMEOUT"},
                                        {"HOME: PAST THE T", "HOME T TIMEOUT"}, {"HOME: T LEFT", "HOME T TIMEOUT"}};
    for (unsigned i = 0; i < 4; i++) {
      freeze_on = at[i][0]; (void)run(130, 1.0, 0, 0, 0); freeze_on = NULL;
      const int ok = last_end && !strcmp(last_end, at[i][1]);
      printf("    encoders dead from \"%s\": %-16s after %5.2f s %s\n", at[i][0], last_end ? last_end : "(never stopped)", end_s,
             ok ? "ok" : "FAIL");
      if (!ok) fails++;
    } }
  printf(fails ? "F2 HOME BY TURNING: %d FAILED\n" : "F2 home by turning 180: out, right at the T, top key, back off, pivot, down, "
                 "LEFT at the T, home, outline, into the zone, stop at the wall PASS\n", fails);
  return fails != 0;
}
