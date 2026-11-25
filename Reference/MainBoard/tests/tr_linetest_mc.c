/* Randomised (Monte-Carlo) stress test of the TR line-following test, with the messy parts of a real robot.
 * Build: gcc -O2 -ICore/Inc -o tr_linetest_mc tests/tr_linetest_mc.c Core/Src/tr_linetest.c -lm      Run: ./tr_linetest_mc [runs]
 *
 * Each run draws: IR spot size (2..12 mm), sideways creep (-5..+5 % of forward travel), a slow heading drift from unequal wheels,
 * IR flicker within 2 mm of the tape edge, one tick of IR delay, a sideways-encoder scale error (+-8 %) and noise, a start that is
 * up to 6 mm off-centre and 2 degrees off-angle, a branch (none / left / right) at a random distance, and a start box
 * (none / all dark / light with a 20 mm black outline / borderline red that flickers black-white) whose edge is 5..80 mm ahead of the IRs and tilted up to 10 degrees.
 * Pass conditions per run:  no branch -> 0 junctions, stops at the wall, made it most of the way;
 *                           branch    -> exactly 1 junction, turned the right way, ended on the branch near its wall.
 *                           never FAULT. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "tr_linetest.h"

static uint32_t rng = 12345u;
static int dbg = 0; /* print the state changes of the run being traced */
static double urand(void) { rng = rng * 1664525u + 1013904223u; return (double)(rng >> 8) / 16777216.0; } /* 0..1 */
static double uni(double a, double b) { return a + (b - a) * urand(); }

static double segd(double x, double y, double ax, double ay, double bx, double by) {
  double dx = bx - ax, dy = by - ay, t = ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy);
  t = t < 0 ? 0 : (t > 1 ? 1 : t);
  return hypot(x - (ax + t * dx), y - (ay + t * dy));
}

typedef struct { double brake; double spot, creep, yaw_drift, enc_scale, off_y, off_hd; int branch; double bx; int box; double box_e, box_tan; } Case;

/* distance from (x,y) to the nearest tape centre-line */
static double tape_dist(const Case *c, double x, double y, double main_end) {
  double d = segd(x, y, 0, 0, main_end, 0);
  if (c->branch > 0) d = fmin(d, segd(x, y, c->bx, 0, c->bx, 1500));
  if (c->branch < 0) d = fmin(d, segd(x, y, c->bx, 0, c->bx, -1500));
  return d;
}
static int ir_read(const Case *c, double x, double y, double main_end) {
  /* the start box: its front edge is box_e ahead of the IRs' start, tilted (box_tan). box 1 = the whole box reads black;
   * box 2 = a light box (red reflects IR) with a 20 mm black outline along the edge; box 3 = a borderline red that flickers */
  if (c->box) {
    const double e = c->box_e + y * c->box_tan;
    if (c->box == 1 && x < e + c->spot) return 1;
    if (c->box == 3 && x < e) return urand() < 0.5;
    if (c->box == 2 && x < e + c->spot && x > e - 20 - c->spot) return 1;
  }
  const double edge = 10.0 + c->spot, d = tape_dist(c, x, y, main_end);
  if (fabs(d - edge) < 2.0) return urand() < 0.5; /* flicker near the threshold */
  return d < edge;
}

static int run_one(const Case *c, char *why, size_t whylen) {
  const double main_end = c->branch ? 2600 : 2000;
  double px = 100, py = c->off_y, hd = c->off_hd, vx = 0, vy = 0, odo = 0, sodo_meas = 0;
  TRLine a;
  TRLine_Init(&a);
  uint32_t now = 1000, next_us = 0, turn_end = 0;
  TRLine_Start(&a, now);
  int turn_state = 0, l_prev = 0, r_prev = 0;
  double turn_target = 0, hd_bias = 0;
  for (int i = 0; i < 20000; i++) {
    now += 5;
    if (turn_state == 1 && (int32_t)(now - turn_end) >= 0) { hd = turn_target + uni(-1.5, 1.5); turn_state = 2; } /* turns are a bit imprecise */
    const double r = hd * M_PI / 180.0, fx = cos(r), fy = -sin(r), lx = -fy, ly = fx;
    TRAuto_In in;
    memset(&in, 0, sizeof in);
    /* one tick of IR delay; with MC_LATCH the IR is sampled every 5 ms and "black at any time since the last loop" is reported */
#ifdef MC_LATCH
    static int l_latch = 0, r_latch = 0;
    l_latch |= l_prev; r_latch |= r_prev;
    in.left = l_latch; in.right = r_latch;
#else
    in.left = l_prev; in.right = r_prev;
#endif
    l_prev = ir_read(c, px + 200 * fx + 25 * lx, py + 200 * fy + 25 * ly, main_end);
    r_prev = ir_read(c, px + 200 * fx - 25 * lx, py + 200 * fy - 25 * ly, main_end);
    in.us_fresh = (int32_t)(now - next_us) >= 0;
    if (in.us_fresh) next_us = now + 50;
    {
      const double sxp = px + 200 * fx, syp = py + 200 * fy;
      double d = -1;
      if (fx > 0.95) d = (main_end + 100) - sxp;
      else if (fy > 0.95) d = 1600 - syp;
      else if (fy < -0.95) d = syp + 1600;
      in.us_mm = (d > 0 && d < 2000) ? (int32_t)(d + uni(-8, 8)) : -1;
    }
    in.dist_mm = (float)odo;
    in.strafe_mm = (float)sodo_meas;
    hd_bias += c->yaw_drift * 0.005;                 /* the heading estimate drifts a little too */
    in.heading_deg = (float)(hd + hd_bias);
    in.heading_sign = -1.0f;
    in.turn_state = (uint8_t)turn_state;
    in.now = now;
    static TRAuto_Out o;
    const TRL_State before = a.st;
#ifndef MC_LOOP_MS
#define MC_LOOP_MS 5
#endif
    if (i % (MC_LOOP_MS / 5) == 0) {
      TRLine_Step(&a, &in, &o);
#ifdef MC_LATCH
      l_latch = r_latch = 0;
#endif
    } else { o.turn_req = false; o.zero_heading = false; }
    if (dbg && a.st != before) printf("    t=%6.3f %-10s -> %-10s  L%d R%d  x=%6.1f y=%+6.1f hd=%+5.1f phase=%u gap=%.1f\n", (now - a.t0) / 1000.0, TRLine_StateText(before), TRLine_StateText(a.st), in.left, in.right, px, py, hd, a.phase, a.gap_mm);
    if (o.zero_heading) { hd_bias = -hd; }
    if (o.turn_req && turn_state != 1) { turn_state = 1; turn_end = now + 1500; turn_target = hd + o.turn_deg; }
    if (turn_state == 2 && a.st != TRL_TURN) turn_state = 0;
    const double cx = (o.active && turn_state != 1) ? o.x : 0, cy = (o.active && turn_state != 1) ? o.y : 0, step = 600.0 / 130.0 * 0.005 * c->brake;
    vx += fmax(-step, fmin(step, cx - vx));
    vy += fmax(-step, fmin(step, cy - vy));
    const double sx = vx * 1.037 * 5, sy = vy * 1.037 * 5, rx = -lx, ry = -ly;
    px += sy * fx + sx * rx; py += sy * fy + sx * ry;
    odo += sy;
    sodo_meas += sx * c->enc_scale + uni(-0.05, 0.05); /* encoder scale error + noise */
    px += c->creep * fabs(sy) * lx; py += c->creep * fabs(sy) * ly; /* sideways creep while driving */
    /* the heading-hold command rotates the robot: w = 1 is about 100 deg/s (60 rpm wheels, 0.273 m radius); with heading_sign
     * -1 a negative w turns clockwise (heading +) */
    if (o.active && turn_state != 1) hd += -o.w * 100.0 * 0.005;
    hd += c->yaw_drift * 0.005 * (fabs(sy) > 0 ? 1 : 0); /* unequal wheels turn it slowly while it drives */
    if (!TRLine_IsRunning(&a)) break;
  }
  if (a.st == TRL_FAULT) { snprintf(why, whylen, "FAULT"); return 0; }
  if (!c->branch) {
    if (a.junctions != 0) { snprintf(why, whylen, "false junction (%u)", a.junctions); return 0; }
    if (a.st != TRL_END) { snprintf(why, whylen, "did not reach the wall (%s, x=%.0f)", TRLine_StateText(a.st), px); return 0; }
    return 1;
  }
  if (a.junctions != 1) { snprintf(why, whylen, "junctions %u (expected 1)", a.junctions); return 0; }
  const double want_hd = c->branch > 0 ? -90 : 90;
  if (fabs(hd - want_hd) > 12) { snprintf(why, whylen, "heading %.0f, expected %.0f", hd, want_hd); return 0; }
  if (fabs(px - c->bx) > 50) { snprintf(why, whylen, "ended %.0f mm off the branch", px - c->bx); return 0; }
  if (a.st != TRL_END) { snprintf(why, whylen, "did not reach the branch wall (%s)", TRLine_StateText(a.st)); return 0; }
  return 1;
}

int main(int argc, char **argv) {
  const int runs = argc > 1 ? atoi(argv[1]) : 600;
  const int trace = argc > 2 ? atoi(argv[2]) : -1; /* run index to trace */
  int fails = 0, shown = 0;
  unsigned long drifts = 0;
  for (int k = 0; k < runs; k++) {
    Case c;
    c.spot = uni(2, 12);
#ifdef MC_SLOW_BRAKE
    c.brake = uni(0.3, 1.0);            /* the real TR may stop more slowly than the 600 rpm/s ramp: it overruns lines */
#else
    c.brake = 1.0;
#endif
    c.creep = uni(-0.05, 0.05);
    c.yaw_drift = uni(-0.6, 0.6);      /* deg/s */
    c.enc_scale = uni(0.92, 1.08);
    c.off_y = uni(-6, 6);
    c.off_hd = uni(-2, 2);
    c.branch = (k % 3 == 0) ? 0 : (k % 3 == 1 ? 1 : -1);
    c.bx = uni(700, 1300);
    c.box = (k / 3) % 4;               /* none / dark box / light box with a black outline / flickering box */
    c.box_e = 300 + uni(5, 80);        /* the IRs start at x = 300, inside the box */
    c.box_tan = tan(uni(-10, 10) * M_PI / 180.0);
    char why[96] = "";
    const uint32_t seed = rng;
    dbg = (k == trace);
    if (!run_one(&c, why, sizeof why)) {
      fails++;
      if (shown < 12) {
        printf("  FAIL run %d (seed %u): %s | spot %.1f creep %+.3f yaw %+.2f enc %.2f off %+.1f/%+.1f branch %d at %.0f box %d\n", k, seed, why,
               c.spot, c.creep, c.yaw_drift, c.enc_scale, c.off_y, c.off_hd, c.branch, c.bx, c.box);
        shown++;
      }
    }
    (void)drifts;
  }
  printf("TR LINE TEST MONTE-CARLO: %d / %d runs passed (%.1f %%)\n", runs - fails, runs, 100.0 * (runs - fails) / runs);
  return fails != 0;
}
