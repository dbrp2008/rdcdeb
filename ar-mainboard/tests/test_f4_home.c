/* Closed-loop host sim of F4 (function1.c, F1_StartF4): the bottom-left key below the 8, on the field map (105.png,
 * ~3.6 px/cm). Out along the bottom line to the F1 stone (US1), stop, encoder legs held on the heading: north, west past
 * the stone, through the gap between the T's bar and the barrier's top, west across the 1 and the 8, over its west edge,
 * south to the key (US1), hold, back off; home the same legs backwards, south to the bottom line, pivot onto it, home like
 * F1 (outline, zone, wall).
 * World (mm): x = WEST of the start-zone outline, y = SOUTH, the bottom line at y = 0 (map: x = (1065 - px) / 0.36,
 * y = (py - 400) / 0.36). Every thin black line on the map is a thin WALL: the AR's rectangle (270 wide, 117 ahead of the
 * axle, 133 behind) must never touch one, nor the F1 stone. MAP_SCALE (sc) stretches the whole map about the zone.
 * The encoders: a pulse at each fixed edge on the wheel (1.78 mm apart), its sign from the command, as on the board. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "function1.h"

#define PI 3.14159265358979
#define TRACK 233.0
#define MM_PER_PULSE (PI * 65.0 / 115.0)
#define FRONT 117.0
#define BACK 133.0
#define HALF_W 135.0
typedef struct { double x, y, th, vl, vr; int32_t j1, j3; double el, er; int dl_last, dr_last; } Bot; /* th: 0 = +x (west), +PI/2 = +y (south) */
static double pull = 1.0; /* right wheel forward gain: < 1 = the AR pulls right (onto the curves) */
static double rev_left = 1.0, ahead = 115.0, bias[5], sc = 1.0, skew = 0, enc_scale = 1.0;
static int us_dead;
static const char *freeze_on, *last_end; static int frozen, no_coast; static double roll = 0.15; /* roll: speed lag per 5 ms at 0 % (lower = rolls on further) */ static double end_s; /* no_coast: the board before SideIR_CountCoast */
static double hold_x, hold_y; static int held;
static double min_clear; static char hit_what[96]; static const char *cur_leg;
#define S(v) ((v) * sc)

/* the thin walls (x0, y0, x1, y1), map mm */
static const double walls[][4] = {
  {1642, -917, 1642, -619}, /* the T's stem, from the top wall */
  {1486, -612, 1792, -612}, /* the T's bar */
  {2253, -147, 2253, 153},  /* the barrier: the line down to the bottom edge (the bottom line ends here) */
  {2097, -153, 2406, -153}, /* ...its lower bar */
  {2103, -461, 2103, -153}, /* ...the line up to its top (the gap's south-west side) */
  {2103, -307, 2414, -307}, /* ...its upper bar */
  {0, -917, 2819, -917},    /* the top wall */
};
static int inbox(double x, double y, double x0, double x1, double y0, double y1) { return x >= x0 && x <= x1 && y >= y0 && y <= y1; }
static double track_k = 1.0; /* the real track / F1_TRACK_MM: > 1 = every turn comes out short */
static double stone_dx; static int box_on; /* box_on: a box left on the west leg (US1 must stop F4 short of it) */ /* the mock stone moved along the line (+ = west, toward the barrier) */
static int stone(double x, double y) { return inbox(x, y, S(1500) + stone_dx, S(1620) + stone_dx, -45, 45); }         /* the F1 stone */
static int key(double x, double y) { return inbox(x, y, S(2569), S(2647), S(47), S(125)); }    /* the key, by the bottom edge */

static double seg(double px, double py, double ax, double ay, double bx, double by) {
  const double vx = bx - ax, vy = by - ay, t0 = ((px - ax) * vx + (py - ay) * vy) / (vx * vx + vy * vy), t = t0 < 0 ? 0 : (t0 > 1 ? 1 : t0);
  return hypot(px - ax - t * vx, py - ay - t * vy);
}
/* the AR's rectangle against the walls and the F1 stone: the smallest gap (mm, < 0 = a hit) */
static void check_body(const Bot *b) {
  const double fx = cos(b->th), fy = sin(b->th);
  for (unsigned w = 0; w < sizeof walls / sizeof walls[0]; w++) {
    const double x0 = S(walls[w][0]), y0 = S(walls[w][1]), x1 = S(walls[w][2]), y1 = S(walls[w][3]), L = hypot(x1 - x0, y1 - y0);
    if (seg(b->x, b->y, x0, y0, x1, y1) > 400) continue; /* far off */
    for (double t = 0; t <= L; t += 4) {
      const double px = x0 + (x1 - x0) * t / L - b->x, py = y0 + (y1 - y0) * t / L - b->y;
      const double u = px * fx + py * fy, v = -px * fy + py * fx; /* along / across the AR */
      const double du = u > FRONT ? u - FRONT : (u < -BACK ? -BACK - u : 0), dv = fabs(v) > HALF_W ? fabs(v) - HALF_W : 0;
      const double c = (du > 0 || dv > 0) ? hypot(du, dv) : -fmin(fmin(FRONT - u, u + BACK), HALF_W - fabs(v));
      if (c < min_clear) { min_clear = c; snprintf(hit_what, sizeof hit_what, "wall %u at (%.0f, %.0f) on \"%s\"", w, px + b->x, py + b->y, cur_leg ? cur_leg : "-"); }
    }
  }
  for (double u = -BACK; u <= FRONT; u += 5) for (double v = -HALF_W; v <= HALF_W; v += 5) {
    if (fabs(v) < HALF_W - 1 && u > -BACK + 1 && u < FRONT - 1) continue; /* the outline is enough */
    const double x = b->x + u * fx - v * fy, y = b->y + u * fy + v * fx;
    if (stone(x, y) && min_clear > -1) { min_clear = -1; snprintf(hit_what, sizeof hit_what, "the F1 stone at (%.0f, %.0f)", x, y); }
  }
}

static void bot_tick(Bot *b, LineWheels cmd) {
  const double fx = cos(b->th), fy = sin(b->th);
  const int fwd = cmd.left + cmd.right > 0;
  /* the front stops at the zone's wall and at the key (the wheels stall against it) */
  if (fwd && fx < -0.7 && b->x + FRONT * fx <= -450 && fabs(b->y) < 300) { b->vl = b->vr = 0; return; }
  if (fwd && (key(b->x + FRONT * fx, b->y + FRONT * fy) || key(b->x + FRONT * fx - 30 * fy, b->y + FRONT * fy + 30 * fx) ||
              key(b->x + FRONT * fx + 30 * fy, b->y + FRONT * fy - 30 * fx))) { b->vl = b->vr = 0; return; }
  b->vl += (cmd.left * 10.0 * (cmd.left < 0 ? rev_left : 1.0) - b->vl) * (cmd.left ? 0.15 : roll);
  b->vr += (cmd.right * 10.0 * (cmd.right > 0 ? pull : 1.0) - b->vr) * (cmd.right ? 0.15 : roll);
  const double dl = b->vl * 0.005, dr = b->vr * 0.005, ds = (dl + dr) / 2, dth = (dr - dl) / (TRACK * track_k);
  b->x += ds * cos(b->th + dth / 2); b->y += ds * sin(b->th + dth / 2); b->th += dth;
  /* el / er: each wheel's angle in pulses (signed). A pulse at every edge it passes, either way; its sign from the
   * command as on the board; at 0 % the last commanded direction (SideIR_CountCoast, F4 only), or none (no_coast). Frozen = the encoder gives no pulses. */
  const double l0 = b->el, r0 = b->er;
  b->el += dl / MM_PER_PULSE * enc_scale; b->er += dr / MM_PER_PULSE * enc_scale;
  if (!frozen) {
    if (cmd.right) b->dr_last = cmd.right > 0 ? 1 : -1;
    if (cmd.left) b->dl_last = cmd.left > 0 ? 1 : -1;
    for (long k = labs((long)floor(b->er) - (long)floor(r0)); k > 0; k--) /* at 0 %: the last commanded direction */
      if (cmd.right || !no_coast) b->j1 += cmd.right > 0 ? 1 : cmd.right < 0 ? -1 : b->dr_last;
    for (long k = labs((long)floor(b->el) - (long)floor(l0)); k > 0; k--)
      if (cmd.left || !no_coast) b->j3 -= cmd.left > 0 ? 1 : cmd.left < 0 ? -1 : b->dl_last;
  }
}
static int black(double x, double y, int i) {
  const double e = 10.0 + bias[i];
  if (x >= -20 - bias[i] && x <= 0 + bias[i] && fabs(y) < 160) return 1;                         /* the zone outline (west side) */
  if (x <= 0 && x >= -300 && (fabs(y + 150) <= e || fabs(y - 150) <= e)) return 1;              /* ...its north / south sides */
  if (seg(x, y, 0, 0, S(2244), 0) <= e) return 1;                                                 /* the bottom line */
  if (seg(x, y, S(778), 0, S(778), S(-917)) <= e) return 1;                                       /* the vertical line */
  if (seg(x, y, S(778), S(-708), S(167), S(-417)) <= e) return 1;                                 /* the diagonal */
  { const double r = hypot(x - S(500), y + S(210)); if (x >= S(500) && fabs(r - S(210)) <= e) return 1; } /* the lower U */
  if (seg(x, y, S(167), S(-417), S(500), S(-420)) <= e) return 1;                                 /* ...its top arm */
  { const double r = hypot(x - S(947), y + S(346)); if (x >= S(947) && y <= 0 && fabs(r - S(346)) <= e) return 1; } /* the C (leaves the bottom line tangentially, as on the map) */
  { const double r = hypot(x - S(1472), y + S(611)); if (fabs(r - S(236)) <= e) return 1; }      /* the circle */
  if (seg(x, y, S(2251), S(-722), S(2251), S(-453)) <= e + 2) return 1;                           /* the 1 */
  if (seg(x, y, S(2404), S(-718), S(2404), S(-463)) <= e || seg(x, y, S(2612), S(-718), S(2612), S(-463)) <= e ||
      seg(x, y, S(2404), S(-718), S(2612), S(-718)) <= e || seg(x, y, S(2404), S(-463), S(2612), S(-463)) <= e ||
      seg(x, y, S(2404), S(-590), S(2612), S(-590)) <= e) return 1;                                /* the 8 */
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
/* US1 at the front, a +-8 deg beam: the F1 stone, the key, the thin walls, the zone's wall */
static int us_mm(const Bot *b) {
  int best = 2000;
  for (int k = -1; k <= 1; k++) {
    const double a = b->th + k * 8.0 * PI / 180.0, fx = cos(a), fy = sin(a), sx = b->x + FRONT * cos(b->th), sy = b->y + FRONT * sin(b->th);
    for (int d = 0; d < 1500 && d < best; d += 3) {
      const double x = sx + d * fx, y = sy + d * fy;
      int hit = stone(x, y) || key(x, y) || (box_on && inbox(x, y, 1800, 1900, -330, -200)) || (!us_dead && x <= -450 && fabs(y) < 300);
      for (unsigned w = 0; !hit && w < sizeof walls / sizeof walls[0]; w++)
        hit = seg(x, y, S(walls[w][0]), S(walls[w][1]), S(walls[w][2]), S(walls[w][3])) <= 3;
      if (hit) { best = d; break; }
    }
  }
  return best;
}
static const char *info_txt(void) {
  static char t[80]; int us, dr; unsigned xo, xh, leg, k;
  if (!F1_F4Info(&us, &xo, &xh, &leg, &k, &dr)) return "-";
  snprintf(t, sizeof t, "stone US1 %d, K%u D%d, crossings %u/%u, leg %u", us, k, dr, xo, xh, leg);
  return t;
}
static int run(double ahead_mm, double rev_gain, double spread, double start_deg, int verbose) {
  ahead = ahead_mm; rev_left = rev_gain; held = 0; min_clear = 1e9; hit_what[0] = 0;
  for (int i = 0; i < 5; i++) bias[i] = spread * ((i * 37 % 11) / 5.0 - 1.0);
  Bot b; memset(&b, 0, sizeof b); b.x = -200; b.th = start_deg * PI / 180.0;
  static double odo = 0, arc = 0, hdg = 0; /* since boot, as on the board */
  F1_Odometry((float)odo, (float)arc); F1_Heading((float)hdg);
  F1_Init();
  uint32_t now = 1000; int32_t oj1 = 0, oj3 = 0;
  unsigned mask; int sl, sr; irs(&b, &mask, &sl, &sr);
  F1_StartF4(now, mask, sl || sr); frozen = 0;
  bool straight = false; int32_t sj1 = 0, sj3 = 0;
  const char *end = NULL;
  double gap_clear = 1e9;
  for (int t = 0; t < 40000 && !end; t++, now += 5) {
    { const int32_t a = b.j1 - oj1, c = b.j3 - oj3; oj1 = b.j1; oj3 = b.j3;
      const double dr = abs(a) * MM_PER_PULSE, dl = abs(c) * MM_PER_PULSE; odo += 0.5 * (dl + dr); arc += dl + dr;
      F1_Odometry((float)odo, (float)arc);
      hdg += ((double)a - (-(double)c)) * MM_PER_PULSE / TRACK * 57.29578;
      F1_Heading((float)hdg); }
    if (t % 2 == 0) { irs(&b, &mask, &sl, &sr); F1_SideLevels(sl, sr); F1_Scan(now, now, mask, mask); }
    static int us = 2000;
    if (t % 12 == 0) { us = us_mm(&b); (void)F1_TargetPing(now, (uint32_t)us); } /* a ping every 60 ms */
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
      held = 1; hold_x = b.x + FRONT * cos(b.th); hold_y = b.y + FRONT * sin(b.th);
    }
    const char *lab = c.status;
    if (t % 4 == 0) { /* the whole run's closest approach (min_clear, hit_what) and the closest in the gap */
      const double m0 = min_clear; char w0[96]; memcpy(w0, hit_what, sizeof w0);
      min_clear = 1e9; cur_leg = lab; check_body(&b);
      if (lab && strstr(lab, "THRU GAP") && min_clear < gap_clear) gap_clear = min_clear;
      if (m0 <= min_clear) { min_clear = m0; memcpy(hit_what, w0, sizeof w0); } }
    if (getenv("TRACE")) {
      static const char *last = NULL;
      if (lab != last && (!last || !lab || strcmp(lab, last))) {
        printf("    t=%6.2f %-18s x %6.0f y %+5.0f hdg %6.1f mask %u%u%u side %d%d us %d\n", t * 0.005, lab ? lab : "-", b.x, b.y,
               b.th * 180 / PI, mask & 1, (mask >> 1) & 1, (mask >> 2) & 1, sl, sr, us);
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
  /* the key: the front stopped at its face, square to it */
  const int at_key = held && fabs(hold_x - S(2608)) < 39 && hold_y > S(47) - 25 && hold_y < S(47) + 5;
  const int clear = min_clear > 0;
  if (verbose)
    printf("  scale %.2f, IR %3.0f ahead, left x%.2f rev, knobs +-%.0f, start %+2.0f, enc x%.2f: key %s (%4.0f, %+4.0f), "
           "gap clear %3.0f, min clear %4.0f%s%s, %-16s at (%5.0f, %+4.0f) hdg %5.1f, %s\n", sc, ahead_mm, rev_gain, spread,
           start_deg, enc_scale, at_key ? "ok" : "MISSED", held ? hold_x : 0.0, held ? hold_y : 0.0, gap_clear, min_clear,
           clear ? " at " : " HIT ", hit_what, end ? end : "(time)", b.x, b.y, hd, info_txt());
  return at_key && clear && end && !strcmp(end, us_dead ? "HOME AT WALL" : "TARGET LINE END") && b.x < -150 &&
         b.x > (us_dead ? -345 : -330) && fabs(b.y) < 60 && fabs(hd - 180) < 12;
}

int main(void) {
  setvbuf(stdout, NULL, _IONBF, 0);
  if (getenv("ONE")) {
    sc = getenv("SCALE") ? atof(getenv("SCALE")) : 1.0; enc_scale = getenv("ENC") ? atof(getenv("ENC")) : 1.0;
    skew = getenv("SKEW") ? atof(getenv("SKEW")) / TRACK : 0;
    return !run(getenv("AHEAD") ? atof(getenv("AHEAD")) : 115, getenv("REV") ? atof(getenv("REV")) : 1.0,
                getenv("SPREAD") ? atof(getenv("SPREAD")) : 0, getenv("START") ? atof(getenv("START")) : 0, 1);
  }
  int fails = 0, n = 0;
  const double aheads[] = {100, 115, 130}, revs[] = {1.0, 0.9}, spreads[] = {0, 4}, starts[] = {0, 3, -3};
  const double scales[] = {0.97, 1.0, 1.03};
  for (unsigned m = 0; m < 3; m++) {
    sc = scales[m];
    for (unsigned a = 0; a < 3; a++)
      for (unsigned r = 0; r < 2; r++)
        for (unsigned s = 0; s < 2; s++)
          for (unsigned h = 0; h < 3; h++) { n++; if (!run(aheads[a], revs[r], spreads[s], starts[h], 1)) fails++; }
  }
  sc = 1.0;
  printf("  - the encoders read 1%% long / short (wheel size): distances and turns both off:\n");
  for (int e = -1; e <= 1; e += 2) {
    enc_scale = 1.0 + e * 0.01;
    for (unsigned r = 0; r < 2; r++) for (unsigned h = 0; h < 3; h++) { n++; if (!run(115, revs[r], 4, starts[h], 1)) fails++; }
  }
  enc_scale = 1.0;
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
  printf("  - an encoder dies mid-run: a limit stops it (no driving / spinning away):\n");
  { static const char *const at[] = {"F4: TURN NORTH", "F4: WEST", "F4: THRU GAP", "F4: TO THE 8", "HOME: BACK NORTH", "HOME: EAST"};
    for (unsigned i = 0; i < sizeof at / sizeof at[0]; i++) {
      freeze_on = at[i]; (void)run(115, 1.0, 0, 0, 0); freeze_on = NULL;
      const int ok = last_end && !strcmp(last_end, "F4 STALLED") && end_s < 60;
      printf("    encoders dead from \"%s\": %-16s after %5.2f s %s\n", at[i], last_end ? last_end : "(never stopped)", end_s,
             ok ? "ok" : "FAIL");
      if (!ok) fails++;
    } }
  if (getenv("F4_FULL")) { /* (F4_FULL=1 only: ~5 min) harsher, as the F3 sim: reported, not gating (the gap leaves ~38 mm a side; the home legs turn a net 180 deg on the
     * encoders, so a 2 % wheel-size error is ~3.6 deg on the way back) */
    int sf = 0, sn = 0;
    printf("  - harsher (reported only): left wheel 15%% weak in reverse, map +-3%%, IRs 100-130:\n");
    for (unsigned m = 0; m < 3; m++) { sc = scales[m];
      for (unsigned a = 0; a < 3; a++) for (unsigned s = 0; s < 2; s++) for (unsigned h = 0; h < 3; h++) {
        sn++; if (!run(aheads[a], 0.85, spreads[s], starts[h], 0)) sf++; } }
    sc = 1.0;
    printf("  - harsher (reported only): encoders 2%% long / short:\n");
    for (int e = -1; e <= 1; e += 2) { enc_scale = 1.0 + e * 0.02;
      for (unsigned r = 0; r < 2; r++) for (unsigned h = 0; h < 3; h++) { sn++; if (!run(115, r ? 0.85 : 1.0, 4, starts[h], 0)) sf++; } }
    enc_scale = 1.0;
    printf("    harsher: %d of %d runs fail (not part of the pass)\n", sf, sn);
    /* the first field run (it hit the barrier): the board dropped every pulse at 0 %, so each stop's roll-on was lost */
    for (unsigned k = 0; k < 3; k++) {
      static const double rolls[3] = {0.15, 0.05, 0.025}; /* rolls on ~5, ~15, ~30 mm from full speed (~2, ~5, ~10 deg turning) */
      roll = rolls[k];
      int cf = 0, nf = 0, cn = 0;
      for (unsigned r = 0; r < 2; r++) for (unsigned h = 0; h < 3; h++) {
        cn++; if (!run(115, revs[r], 4, starts[h], 0)) cf++;
        no_coast = 1; if (!run(115, revs[r], 4, starts[h], k == 1 && !r && !h)) nf++; no_coast = 0; }
      printf("  - rolls on more after each stop (lag %.3f): %d of %d fail; the board without coast counting (first field "
             "run): %d of %d\n", roll, cf, cn, nf, cn);
    }
    roll = 0.15;
  }
  { /* the second field run: it followed the C (it leaves the bottom line tangentially) and turned there. An AR that pulls
     * right sits on the line's right edge, where the curves peel off: gating */
    printf("  - the AR pulls right (right wheel 3 / 6 / 9 %% weaker forward), the C and the U leave the line tangentially:\n");
    static const double pulls[3] = {0.97, 0.94, 0.91};
    for (unsigned k = 0; k < 3; k++) { pull = pulls[k];
      for (unsigned r = 0; r < 2; r++) for (unsigned h = 0; h < 3; h++) { n++; if (!run(115, revs[r], 4, starts[h], 1)) fails++; } }
    pull = 1.0;
  }
  { /* the third field run: a mock stone only roughly placed, so the stone-to-barrier distance is off: the barrier face
     * (US1) must end the west leg wherever it is */
    printf("  - the stone moved along the line (-200 / -100 / +100 / +200 mm):\n");
    static const double dxs[4] = {-200, -100, 100, 200};
    for (unsigned k = 0; k < 4; k++) { stone_dx = dxs[k];
      for (unsigned r = 0; r < 2; r++) for (unsigned h = 0; h < 3; h++) { n++; if (!run(115, revs[r], 4, starts[h], 1)) fails++; } }
    stone_dx = 0;
  }
  if (getenv("F4_FULL")) { /* (F4_FULL=1 only) turns off on the real floor (track width / scrub): reported - how exact the turns must be */
    static const double ks[6] = {0.94, 0.97, 0.985, 1.015, 1.03, 1.06};
    for (unsigned k = 0; k < 6; k++) { track_k = ks[k]; int tf = 0, tn = 0;
      for (unsigned r = 0; r < 2; r++) for (unsigned h = 0; h < 3; h++) { tn++; if (!run(115, revs[r], 4, starts[h], 0)) tf++; }
      printf("  - turns %s by %.1f%% (a 90 comes out %.1f): %d of %d fail (reported only)\n", ks[k] > 1 ? "short" : "long",
             100 * (ks[k] > 1 ? ks[k] - 1 : 1 - ks[k]), 90.0 / ks[k], tf, tn); }
    track_k = 1.0;
  }
  { /* a box on the west leg: US1 stops it short (no driving into it) */
    box_on = 1; (void)run(115, 1.0, 4, 0, 0); box_on = 0;
    const int ok = last_end && !strcmp(last_end, "F4 US1 BLOCKED");
    printf("  - a box left on the west leg: %s %s\n", last_end ? last_end : "(never stopped)", ok ? "ok" : "FAIL");
    if (!ok) fails++;
  }
  printf(fails ? "F4 BOTTOM-LEFT KEY: %d of %d FAILED\n" : "F4 bottom-left key: bottom line to the F1 stone, north, west, "
                 "through the gap, across the 1 and the 8, south to the key, back the same way, bottom line, home, zone, wall "
                 "(%d runs, no wall touched) PASS\n", fails ? fails : n, n);
  return fails != 0;
}
