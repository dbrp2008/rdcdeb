/* Closed-loop host sim of F3 (function1.c, F1_StartF3): the top-right key by the right wall, on the field photo's map.
 * Out along the bottom line, keep right onto the U, round it, east along its top arm to the V, advance + pivot LEFT up
 * the short up-right leg, advance + pivot RIGHT at its top bend, east to the key (US1), hold, back off, pivot CCW onto the
 * up-right leg, down to the V, advance + pivot RIGHT onto the U's top arm, keep left round the U onto the bottom line,
 * outline, straight into the zone, stop at the wall.
 * World (mm, measured off the photo at ~0.78 mm/px - the AR's 233 mm track and the F2 sim's 782 mm to the vertical line
 * agree): x = WEST of the start-zone outline (x -20..0), y = SOUTH. Bottom line y = 0; the U (centre (483, -200), r 200,
 * its west half) leaves it tangentially at x 483 and comes back east as its top arm y = -400 to the V at (152, -400); the
 * diagonal arrives at the V from the vertical line (782, -716); the up-right leg runs from the V to the bend (0, -675),
 * 61 deg left of east; the east leg runs to x = -180, the key (a stone) at x <= -190. A box stands by the bend
 * (x 16..164, y -897..-647): US1 must not take it for the key. The zone's wall at x = -450. MAP_SCALE stretches the whole
 * map (the photo's scale is the least certain number here). */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "function1.h"

#define PI 3.14159265358979
#define TRACK 233.0
#define MM_PER_PULSE (PI * 65.0 / 115.0)
typedef struct { double x, y, th, vl, vr; int32_t j1, j3; double el, er; } Bot; /* th: heading, 0 = +x (west), +PI/2 = +y (south) */
static double rev_left = 1.0, ahead = 115.0, bias[5], sc = 1.0, skew = 0;
static int us_dead;
static const char *freeze_on, *last_end; static int frozen; static double end_s;
static double hold_x, hold_y; static int held;
/* faults on the up-right leg (from its pivot's end, mm): 1 = outer-right IR high for one sample at 160, 2 = no tape under
 * any IR for 40 mm from 170 (a worn / lifted patch), 3 = the outer-right IR never sees anything on the up leg */
static int fault; static double up_at = -1; /* where the IR bar was during the target hold */
static void bot_tick(Bot *b, LineWheels cmd) {
  if (cos(b->th) < -0.7 && b->x + ahead * cos(b->th) <= -450 && fabs(b->y) < 300 && cmd.left + cmd.right > 0) { b->vl = b->vr = 0; return; }
  /* the key's stone stops the front (the IR bar ~ the front): the wheels stall against it */
  if (cos(b->th) < -0.7 && b->x + ahead * cos(b->th) <= -190 * sc && fabs(b->y + 675 * sc) < 90 && cmd.left + cmd.right > 0) {
    b->vl = b->vr = 0; return; }
  b->vl += (cmd.left * 10.0 * (cmd.left < 0 ? rev_left : 1.0) - b->vl) * 0.15;
  b->vr += (cmd.right * 10.0 - b->vr) * 0.15;
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
#define S(v) ((v) * sc)
static int black(double x, double y, int i) {
  const double e = 10.0 + bias[i];
  if (x >= -20 - bias[i] && x <= 0 + bias[i] && fabs(y) < 160) return 1;                         /* the zone outline (west side) */
  if (x <= 0 && x >= -300 && (fabs(y + 150) <= e || fabs(y - 150) <= e)) return 1;              /* ...its north / south sides */
  if (seg(x, y, 0, 0, 2000, 0) <= e) return 1;                                                    /* bottom line */
  if (seg(x, y, S(782), 0, S(782), S(-870)) <= e) return 1;                                       /* the vertical line */
  if (seg(x, y, S(782), S(-716), S(152), S(-400)) <= e) return 1;                                 /* the diagonal */
  { const double r = hypot(x - S(483), y + S(200)); if (x >= S(483) && fabs(r - S(200)) <= e) return 1; } /* U (west half) */
  if (seg(x, y, S(152), S(-400), S(483), S(-400)) <= e) return 1;                                 /* ...its top arm, to the V */
  if (fault == 4) { /* the bend taped as a curve (radius 120 mm) instead of a sharp corner */
    const double cx = S(-70.6), cy = S(-555.2), p1x = S(34.2), p1y = S(-613.1), p2x = S(-70.7), p2y = S(-675), r = S(120);
    if (seg(x, y, S(152), S(-400), p1x, p1y) <= e || seg(x, y, p2x, p2y, S(-180), S(-675)) <= e) return 1;
    const double ax = p1x - cx, ay = p1y - cy, bx = p2x - cx, by = p2y - cy, qx = x - cx, qy = y - cy, ab = ax * by - ay * bx;
    if (fabs(hypot(qx, qy) - r) <= e && (ax * qy - ay * qx) * ab >= 0 && (qx * by - qy * bx) * ab >= 0) return 1;
    return 0;
  }
  if (seg(x, y, S(152), S(-400), S(0), S(-675)) <= e) return 1;                                   /* the up-right leg */
  if (seg(x, y, S(0), S(-675), S(-180), S(-675)) <= e) return 1;                                  /* the east leg to the key */
  return 0;
}
static const double lat[5] = {25, 0, -25, 60, -60}; /* L, C, R, outer L, outer R (+ = left of the AR) */
static void irs(const Bot *b, unsigned *mask, int *sl, int *sr) {
  const double fx = cos(b->th), fy = sin(b->th), lx = -fy, ly = fx;
  int v[5];
  for (int i = 0; i < 5; i++) { const double ai = ahead - lat[i] * skew;
    v[i] = black(b->x + ai * fx + lat[i] * lx, b->y + ai * fy + lat[i] * ly, i); }
  *mask = (unsigned)(v[0] | (v[1] << 1) | (v[2] << 2)); *sl = v[3]; *sr = v[4];
}
static int inbox(double x, double y, double x0, double x1, double y0, double y1) { return x >= x0 && x <= x1 && y >= y0 && y <= y1; }
/* US1 at the IR bar, a +-8 deg beam: the key (stone), the box by the bend, the zone's wall */
static int us_mm(const Bot *b) {
  int best = 2000;
  for (int k = -1; k <= 1; k++) {
    const double a = b->th + k * 8.0 * PI / 180.0, fx = cos(a), fy = sin(a), sx = b->x + ahead * cos(b->th), sy = b->y + ahead * sin(b->th);
    for (int d = 0; d < 1500; d += 4) {
      const double x = sx + d * fx, y = sy + d * fy;
      if (inbox(x, y, -400, S(-190), S(-760), S(-600)) || inbox(x, y, S(16), S(164), S(-897), S(-647)) ||
          (!us_dead && x <= -450 && fabs(y) < 300)) { if (d < best) best = d; break; }
    }
  }
  return best;
}

static const char *bend_txt(void) {
  static char t[40]; int up, deg; char k; bool r;
  if (!F1_F3Bend(&up, &k, &deg, &r)) return "-";
  snprintf(t, sizeof t, "%d mm up (%c), %d deg%s", up, k, deg, r ? ", retried" : "");
  return t;
}
static int run(double ahead_mm, double rev_gain, double spread, double start_deg, int verbose) {
  ahead = ahead_mm; rev_left = rev_gain; held = 0; up_at = -1;
  for (int i = 0; i < 5; i++) bias[i] = spread * ((i * 37 % 11) / 5.0 - 1.0);
  Bot b; memset(&b, 0, sizeof b); b.x = -200; b.th = start_deg * PI / 180.0;
  static double odo = 0, arc = 0, hdg = 0; /* since boot, as on the board */
  F1_Odometry((float)odo, (float)arc); F1_Heading((float)hdg);
  F1_Init();
  uint32_t now = 1000; int32_t oj1 = 0, oj3 = 0;
  unsigned mask; int sl, sr; irs(&b, &mask, &sl, &sr);
  F1_StartF3(now, mask, sl || sr); frozen = 0;
  bool straight = false; int32_t sj1 = 0, sj3 = 0;
  const char *end = NULL;
  for (int t = 0; t < 24000 && !end; t++, now += 5) {
    { const int32_t a = b.j1 - oj1, c = b.j3 - oj3; oj1 = b.j1; oj3 = b.j3;
      const double dr = abs(a) * MM_PER_PULSE, dl = abs(c) * MM_PER_PULSE; odo += 0.5 * (dl + dr); arc += dl + dr;
      F1_Odometry((float)odo, (float)arc);
      hdg += ((double)a - (-(double)c)) * MM_PER_PULSE / TRACK * 57.29578;
      F1_Heading((float)hdg); }
    if (t % 2 == 0) {
      irs(&b, &mask, &sl, &sr);
      if (fault && up_at >= 0) {
        const double u = odo - up_at;
        if (fault == 1 && u >= 160 && u < 162) sr = 1;
        if (fault == 2 && u >= 170 && u < 210) mask = 0;
        if ((fault == 3 || fault == 4) && u < 400) sr = 0;
      }
      F1_SideLevels(sl, sr); F1_Scan(now, now, mask, mask);
    }
    const int us = us_mm(&b);
    if (t % 12 == 0) (void)F1_TargetPing(now, (uint32_t)us);
    end = F1_Advance(now);
    if (end) break;
    LineWheels w = {0, 0};
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
    } else if (F1_PhaseAt(now) == F1_TARGET_HOLD && !held) {
      held = 1; hold_x = b.x + ahead * cos(b.th); hold_y = b.y + ahead * sin(b.th);
    }
    const char *lab = c.status;
    if (lab && !strcmp(lab, "F3: UP") && up_at < 0) up_at = odo;
    if (getenv("FINE") && t * 0.005 > atof(getenv("FINE")) && t * 0.005 < atof(getenv("FINE")) + 1.5 && t % 4 == 0) printf("      t=%6.2f %-16s x %6.0f y %+5.0f hdg %5.1f mask %u%u%u side %d%d L %.1f R %.1f\n", t * 0.005, lab ? lab : "-", b.x, b.y, fmod(b.th * 180 / PI + 720.0, 360.0), mask & 1, (mask >> 1) & 1, (mask >> 2) & 1, sl, sr, w.left, w.right);
    if (getenv("TRACE")) {
      static const char *last = NULL;
      if (lab != last && (!last || !lab || strcmp(lab, last))) {
        printf("    t=%6.2f %-18s x %6.0f y %+5.0f hdg %5.1f mask %u%u%u side %d%d us %d\n", t * 0.005, lab ? lab : "-", b.x, b.y,
               fmod(b.th * 180 / PI + 720.0, 360.0), mask & 1, (mask >> 1) & 1, (mask >> 2) & 1, sl, sr, us);
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
  /* the key: the bar stopped at the stone's face, on the east leg */
  const int at_key = held && hold_x < S(-120) && hold_x > S(-190) - 10 && fabs(hold_y - S(-675)) < 50;
  if (verbose)
    printf("  scale %.2f, IR %3.0f mm ahead, left x%.2f in reverse, knobs +-%.0f, start %+2.0f deg: key %s (%4.0f, %+4.0f), "
           "%-16s at (%5.0f, %+4.0f) heading %5.1f, pivot %3.0f, bend %s\n", sc, ahead_mm, rev_gain, spread, start_deg,
           at_key ? "ok" : "MISSED", held ? hold_x : 0.0, held ? hold_y : 0.0, end ? end : "(time)", b.x, b.y, hd,
           (double)F1_LastPivotDeg(), bend_txt());
  return at_key && end && !strcmp(end, us_dead ? "HOME AT WALL" : "TARGET LINE END") && b.x < -150 && b.x > (us_dead ? -345 : -330) &&
         fabs(b.y) < 60 && fabs(hd - 180) < 12;
}

int main(void) {
  setvbuf(stdout, NULL, _IONBF, 0);
  if (getenv("ONE")) {
    sc = getenv("SCALE") ? atof(getenv("SCALE")) : 1.0; fault = getenv("FAULT") ? atoi(getenv("FAULT")) : 0;
    return !run(getenv("AHEAD") ? atof(getenv("AHEAD")) : 115, getenv("REV") ? atof(getenv("REV")) : 1.0,
                getenv("SPREAD") ? atof(getenv("SPREAD")) : 0, getenv("START") ? atof(getenv("START")) : 0, 1);
  }
  int fails = 0, n = 0;
  const double aheads[] = {100, 115, 130}, revs[] = {1.0, 0.85}, spreads[] = {0, 4}, starts[] = {0, 3, -3}, scales[] = {0.93, 1.0, 1.07};
  for (unsigned m = 0; m < 3; m++) {
    sc = scales[m];
    for (unsigned a = 0; a < 3; a++)
      for (unsigned r = 0; r < 2; r++)
        for (unsigned s = 0; s < 2; s++)
          for (unsigned h = 0; h < 3; h++) { n++; if (!run(aheads[a], revs[r], spreads[s], starts[h], 1)) fails++; }
  }
  sc = 1.0;
  printf("  - the measured AR: IRs 120 mm ahead of the right wheel, 105 mm of the left (bar skewed), both signs:\n");
  for (int sg = -1; sg <= 1; sg += 2) {
    skew = sg * 15.0 / TRACK;
    for (unsigned r = 0; r < 2; r++)
      for (unsigned h = 0; h < 3; h++) { n++; if (!run(112.5, revs[r], 4, starts[h], 1)) fails++; }
  }
  skew = 0;
  us_dead = 1; printf("  - US1 never sees the zone's wall going home:\n");
  for (unsigned h = 0; h < 3; h++) { n++; if (!run(115, 1.0, 4, starts[h], 1)) fails++; }
  us_dead = 0;
  { static const char *const what[5] = {"", "the outer-right IR glitches high once at 160 mm up the short leg",
                                       "40 mm of the short leg with no tape (170-210 mm up): an early bend, turned back, retried",
                                       "the outer-right IR misses the east leg: the centre + left white backup",
                                       "the bend taped round (r 120 mm) and the outer-right IR misses it: the follower takes it"};
    for (fault = 1; fault <= 4; fault++) {
      printf("  - %s:\n", what[fault]);
      for (unsigned a = 0; a < 3; a++) for (unsigned h = 0; h < 3; h += 2) { n++; if (!run(aheads[a], 1.0, 4, starts[h], 1)) fails++; }
    }
    fault = 0; }
  printf("  - an encoder dies mid-run: a limit stops it (no driving / spinning away):\n");
  { static const char *const at[][2] = {{"F3: PAST CORNER", "F3 ADV TIMEOUT"}, {"F3: V LEFT", "F3 TURN TIMEOUT"},
                                        {"F3: BEND RIGHT", "F3 TURN TIMEOUT"}, {"HOME: V RIGHT", "F3 TURN TIMEOUT"}};
    for (unsigned i = 0; i < 4; i++) {
      freeze_on = at[i][0]; (void)run(115, 1.0, 0, 0, 0); freeze_on = NULL;
      const int ok = last_end && !strcmp(last_end, at[i][1]);
      printf("    encoders dead from \"%s\": %-16s after %5.2f s %s\n", at[i][0], last_end ? last_end : "(never stopped)", end_s,
             ok ? "ok" : "FAIL");
      if (!ok) fails++;
    } }
  printf(fails ? "F3 TOP-RIGHT KEY: %d of %d FAILED\n" : "F3 top-right key: bottom line, keep right round the U, V left, bend right, "
                 "key, back off, pivot, down, V right, round the U, home, zone, wall (%d runs) PASS\n", fails ? fails : n, n);
  return fails != 0;
}
