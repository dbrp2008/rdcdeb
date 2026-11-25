/* Host tests of the fixes (default build, F1_LEGACY 0): F1 steering, the 101 latch release, the retrace hand-off, and
 * retrace.c driving a simulated AR home. Build: cc -ICore/Inc tests/test_wm_fixes.c Core/Src/function1.c Core/Src/retrace.c -lm */
#include "function1.h"
#include "retrace.h"
#include "function2.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static int near(float a, float b) { return fabsf(a - b) < 0.01f; }
static F1_Command cmd_at(uint32_t now) { assert(F1_Advance(now) == NULL); return F1_CommandAt(now, 200); }
static void scan(uint32_t now, unsigned mask) { F1_Scan(now, now, mask, mask); }
static void expect(uint32_t now, float l, float r) {
  F1_Command c = cmd_at(now);
  assert(c.request == F1_MOVE);
  if (!near(c.wheels.left, F12_DUTY(l)) || !near(c.wheels.right, F12_DUTY(r))) {
    printf("  t=%u got %.3f / %.3f want %.3f / %.3f (%s)\n", (unsigned)now, c.wheels.left, c.wheels.right, F12_DUTY(l), F12_DUTY(r), c.status);
    assert(0);
  }
}
#define S LINE_NOMINAL_PERCENT
#define HI (LINE_STEER_MEAN_PERCENT - LINE_STEER_BIAS_PERCENT)
#define HO (LINE_STEER_MEAN_PERCENT + LINE_STEER_BIAS_PERCENT)
#define GI (LINE_STEER_MEAN_PERCENT - 0.5f * LINE_STEER_BIAS_PERCENT)
#define GO (LINE_STEER_MEAN_PERCENT + 0.5f * LINE_STEER_BIAS_PERCENT)

static void start(void) { F1_Init(); F1_Start(0, LINE_CENTRE, false); (void)cmd_at(800); } /* past the 750 ms launch */

static void steering(void) {
  start();
  scan(810, 2); expect(810, S, S);                    /* 010 straight */
  scan(820, 3); expect(820, GI, GO);                  /* 110: gentle left (was straight) */
  scan(830, 1); expect(830, HI, HO);                  /* 100: hard left (as before) */
  scan(840, 6); expect(840, GO, GI);                  /* 011: gentle right (was straight) */
  scan(850, 4); expect(850, HO, HI);                  /* 001: hard right (as before) */
  scan(860, 7); expect(860, S, S);                    /* 111 crossing: straight */
  /* lost right after slipping out LEFT: seek left, slower, for F1_LOST_TURN_MS, then straight */
  scan(870, 1); expect(870, HI, HO);
  scan(880, 0); expect(880, 6.0f, 18.0f);
  assert(!strcmp(F1_CommandAt(1000, 200).status, "LOST: SEEK LEFT"));
  expect(880 + F1_LOST_TURN_MS - 1u, 6.0f, 18.0f);
  expect(880 + F1_LOST_TURN_MS, S, S);
  /* lost while CENTRED (a gap in the tape): straight on, as before */
  scan(2000, 2); expect(2000, S, S);
  scan(2010, 0); expect(2010, S, S);
  /* lost after slipping out RIGHT: seek right */
  scan(2100, 6); scan(2110, 0); expect(2110, 18.0f, 6.0f);
  printf("F1 steering: 110/011 gentle, 100/001 hard, lost line seeks the side it slipped to (centred gap: straight) PASS\n");
}

static void split_latch(void) {
  start();
  scan(810, 5); expect(810, HI, HO);                  /* 101: hold left */
  scan(820, 4); expect(820, HI, HO);                  /* 001 alone does NOT release it (centre not black) */
  scan(830, 6); expect(830, GO, GI);                  /* 011: centre black -> released, gentle RIGHT (old: kept turning left) */
  scan(840, 5); expect(840, HI, HO);                  /* 101 again */
  scan(850, 0);
  expect(840 + F1_SPLIT_MAX_MS - 1u, HI, HO);          /* the latch (set at 840) still holds ... */
  expect(840 + F1_SPLIT_MAX_MS + 5u, 18.0f, 6.0f);     /* ... released by the time limit (old: left forever); the line was
                                                          last seen slipping RIGHT (011 at 830) and is lost: seek right */
  expect(850 + F1_LOST_TURN_MS, S, S);                 /* and straight once the lost-line search time is over */
  printf("F1 101 latch: released by any centre-black scan or after %u ms PASS\n", (unsigned)F1_SPLIT_MAX_MS);
}

static void retrace_handoff(void) {
  start();
  scan(810, 2); expect(810, S, S);
  assert(F1_TargetPing(900, 50));                      /* a valid ping under 60 mm: approach (350 ms) */
  (void)cmd_at(1260);                                  /* approach over: hold starts */
  assert(F1_PhaseAt(1260) == F1_TARGET_HOLD);
  assert(F1_Advance(2260) == NULL);                    /* 1 s hold over */
  assert(F1_PhaseAt(2260) == F1_TARGET_REVERSE);       /* F1 turns home: back off first */
  F1_Command c = F1_CommandAt(2270, 200);
  assert(c.request == F1_MOVE && c.wheels.left < 0 && c.wheels.right < 0 && F1_EncoderStraight(2270));
  F1_Stop();
  assert(!F1_FinishArmed());                           /* no armed three-black finish for the next run */
  /* F2 also turns home (F2_RETURN_TURN; 0 = the retrace) */
  F1_Init(); F1_StartPerScan(0, LINE_CENTRE); (void)cmd_at(800); scan(810, 2);
  assert(F1_TargetPing(900, 50)); (void)cmd_at(1260); assert(F1_Advance(2260) == NULL);
  assert(F1_PhaseAt(2260) == (F2_RETURN_TURN ? F1_TARGET_REVERSE : F1_RETRACE));
  c = F1_CommandAt(2270, 200);
  assert(F2_RETURN_TURN ? (c.request == F1_MOVE && !strcmp(c.status, "HOME: BACK OFF"))
                        : (c.request == F1_KEEP_STOPPED && !strcmp(c.status, "RETRACE HOME")));
  F1_Stop();
  printf("F1 / F2 home: after the target hold both back off to turn home (F2_RETURN_TURN); next run not armed PASS\n");
}

/* ---- retrace.c on a simulated two-wheel AR ---- */
typedef struct { double x, y, th, vl, vr, pl, pr; int32_t j1, j3; double el, er; double count_scale; } Bot; /* el/er: fractional pulses */
#define TRACK 233.0
#define MM_PER_PULSE (3.14159265 * 65.0 / 115.0)
static double rev_left_gain = 1.0; /* the left motor this much weaker in REVERSE only (field: the return did not go straight) */
static void bot_tick(Bot *b, LineWheels cmd, double left_gain) {
  /* duty -> speed (10 mm/s per %), first-order lag; the left motor weaker by left_gain (and by rev_left_gain in reverse) */
  b->vl += (cmd.left * 10.0 * left_gain * (cmd.left < 0 ? rev_left_gain : 1.0) - b->vl) * 0.15;
  b->vr += (cmd.right * 10.0 - b->vr) * 0.15;
  const double dl = b->vl * 0.005, dr = b->vr * 0.005, ds = (dl + dr) / 2, dth = (dr - dl) / TRACK;
  b->x += ds * cos(b->th + dth / 2); b->y += ds * sin(b->th + dth / 2); b->th += dth;
  /* ELCP counting: pulses only while that wheel is commanded, signed by the command (J1 = right, J3 = left) */
  const double cs = b->count_scale > 0 ? b->count_scale : 1.0; /* > 1: the wheels slip (count more than the AR moves) */
  b->el += fabs(dl) / MM_PER_PULSE * cs; b->er += fabs(dr) / MM_PER_PULSE * cs;
  while (b->er >= 1.0) { b->er -= 1.0; if (cmd.right > 0) b->j1++; else if (cmd.right < 0) b->j1--; }
  while (b->el >= 1.0) { b->el -= 1.0; if (cmd.left > 0) b->j3--; else if (cmd.left < 0) b->j3++; } /* J3 is wired inverted */
}
static Retrace tr;
static double retrace_sim(double left_gain) {
  Bot b; memset(&b, 0, sizeof b);
  uint32_t now = 0;
  Retrace_Start(&tr, now, b.j1, b.j3);
  /* outbound: straight, gentle left curve, line-follow zig-zag, F2-style right pivot, straight; then the target stop */
  for (int i = 0; i < 1400; i++) {
    LineWheels w;
    if (i < 400) w = (LineWheels){22.5f, 22.5f};
    else if (i < 700) w = (LineWheels){13.4f, 20.4f};
    else if (i < 900) w = (i / 20) % 2 ? (LineWheels){9.9f, 23.9f} : (LineWheels){23.9f, 9.9f};
    else if (i < 1060) w = (LineWheels){20.0f, -16.0f};
    else w = (LineWheels){22.5f, 22.5f};
    Retrace_Record(&tr, now, w, b.j1, b.j3);
    bot_tick(&b, w, left_gain);
    now += 5;
  }
  for (int i = 0; i < 300; i++) { Retrace_Record(&tr, now, (LineWheels){0, 0}, b.j1, b.j3); bot_tick(&b, (LineWheels){0, 0}, left_gain); now += 5; }
  const double ox = b.x, oy = b.y, oth = b.th;
  int ticks = 0;
  LineWheels w;
  while (Retrace_Step(&tr, now, b.j1, b.j3, &w) && ticks < 20000) { bot_tick(&b, w, left_gain); now += 5; ticks++; }
  for (int i = 0; i < 100; i++) bot_tick(&b, (LineWheels){0, 0}, left_gain); /* settle */
  const double err = hypot(b.x, b.y), herr = fabs(b.th) * 180.0 / 3.14159265;
  printf("  retrace (left motor x%.2f, x%.2f in reverse): out to (%.0f, %.0f) %.0f deg, %u breadcrumbs; home %.1f mm from the start, heading %.1f deg off, %s\n",
         left_gain, rev_left_gain, ox, oy, oth * 180 / 3.14159265, tr.n, err, herr, tr.state == RETRACE_DONE ? "DONE" : "NOT DONE");
  assert(tr.state == RETRACE_DONE && !tr.overflow);
  assert(err < 40.0 && herr < 4.0);
  return err;
}
static void retrace_stall(void) {
  Bot b; memset(&b, 0, sizeof b);
  uint32_t now = 0;
  Retrace_Start(&tr, now, 0, 0);
  for (int i = 0; i < 400; i++) { Retrace_Record(&tr, now, (LineWheels){22.5f, 22.5f}, b.j1, b.j3); bot_tick(&b, (LineWheels){22.5f, 22.5f}, 1.0); now += 5; }
  LineWheels w; int ticks = 0;
  while (Retrace_Step(&tr, now, b.j1, b.j3, &w) && ticks < 20000) { now += 5; ticks++; } /* wheels blocked: counts never move */
  assert(tr.state == RETRACE_STALLED && ticks * 5 <= (int)RETRACE_STALL_MS + 10);
  printf("  blocked on the way home: HOME STALLED after %d ms PASS\n", ticks * 5);
}

/* the start zone's outline: a 20 mm black band across the start heading, 50 mm in front of the IRs at the start; the 3
 * IRs are 150 mm ahead of the wheel axle, 25 mm apart */
static unsigned outline_mask(const Bot *b) {
  unsigned m = 0u;
  for (int i = 0; i < 3; i++) {
    const double side = (i - 1) * 25.0; /* L, C, R = bits 0, 1, 2: left is +y */
    const double ix = b->x + 150.0 * cos(b->th) - (-side) * sin(b->th), iy = b->y + 150.0 * sin(b->th) + (-side) * cos(b->th);
    (void)iy;
    if (ix >= 200.0 && ix <= 220.0) m |= 1u << i;
  }
  return m;
}
static double last_x, last_y;
static double anchor_run(double slip_home, int use_edge) {
  Bot b; memset(&b, 0, sizeof b);
  uint32_t now = 0;
  Retrace_Start(&tr, now, b.j1, b.j3);
  for (int i = 0; i < 1400; i++) {
    LineWheels w;
    if (i < 400) w = (LineWheels){22.5f, 22.5f};
    else if (i < 700) w = (LineWheels){13.4f, 20.4f};
    else w = (LineWheels){22.5f, 22.5f};
    Retrace_Record(&tr, now, w, b.j1, b.j3);
    bot_tick(&b, w, 1.0);
    if (use_edge && i % 2 == 0) Retrace_Line(&tr, outline_mask(&b));
    now += 5;
  }
  for (int i = 0; i < 100; i++) { Retrace_Record(&tr, now, (LineWheels){0, 0}, b.j1, b.j3); bot_tick(&b, (LineWheels){0, 0}, 1.0); now += 5; }
  assert(!use_edge || tr.edge_marked);
  b.count_scale = slip_home;
  LineWheels w; int ticks = 0;
  while (Retrace_Step(&tr, now, b.j1, b.j3, &w) && ticks < 20000) {
    bot_tick(&b, w, 1.0);
    if (use_edge && ticks % 2 == 0) Retrace_Line(&tr, outline_mask(&b));
    now += 5; ticks++;
  }
  for (int i = 0; i < 100; i++) bot_tick(&b, (LineWheels){0, 0}, 1.0);
  assert(tr.state == RETRACE_DONE);
  if (use_edge) assert(tr.anchored);
  last_x = b.x; last_y = b.y;
  return hypot(b.x, b.y);
}
static void anchor_tests(void) {
  for (int k = 0; k < 3; k++) {
    const double slip = k == 0 ? 1.06 : (k == 1 ? 0.95 : 1.0);
    const double without = anchor_run(slip, 0), wx = last_x, with = anchor_run(slip, 1);
    printf("  start-zone checkpoint, wheels counting %+3.0f %% on the way home: home %5.1f mm off without it (%+5.1f along the start line), %4.1f mm with it (%+4.1f along, %+4.1f sideways)\n",
           (slip - 1.0) * 100.0, without, wx, with, last_x, last_y);
    /* it fixes the DISTANCE (to within the IR scan lag, a few mm); heading error from slip in the turns stays */
    assert(fabs(last_x) < 10.0 && with < 15.0);
    if (k < 2) assert(with < without / 4.0);
  }
  printf("Retrace: the start-zone outline corrects the distance on the way home (also when the trail runs out first) PASS\n");
}

static void f2_window(void) {
  uint32_t t = 0;
  F2_Init();
  F2_Start(false);
  F2_Update((BranchSample){.at = t += 10, .high = true}, 0); F2_Update((BranchSample){.at = t += 10}, 0);
  F2_Update((BranchSample){.at = t += 80, .first_rise = t - 5, .rose = true}, 0);
  F2_Update((BranchSample){.at = t += 80, .first_rise = t - 5, .rose = true}, 0);
  assert(F2_Turning() && F2_Advancing());
  F2_Advanced(F2_T_OUT_ADVANCE_MM); /* 8 cm past the T: now the pivot */
  F2_PivotCommanded();
  F2_PivotAngle(5.0f);
  F2_Update((BranchSample){.at = t += 10}, 0);                 /* white */
  F2_Update((BranchSample){.at = t += 80}, 0);                 /* the 75 ms gap */
  F2_PivotAngle(30.0f);
  F2_Update((BranchSample){.at = t += 10}, LINE_CENTRE);       /* tape at 30 deg: too early (the old line / a crossing) */
  assert(F2_Turning());
  F2_Update((BranchSample){.at = t += 10}, 0);
  F2_PivotAngle(80.0f);
  F2_Update((BranchSample){.at = t += 10}, LINE_CENTRE);       /* tape at 80 deg: the branch */
  assert(!F2_Turning() && !F2_Failed());
  /* no tape at all: stops at the window's end instead of spinning */
  F2_Init(); F2_Start(false); t = 0;
  F2_Update((BranchSample){.at = t += 10, .high = true}, 0); F2_Update((BranchSample){.at = t += 10}, 0);
  F2_Update((BranchSample){.at = t += 80, .first_rise = t - 5, .rose = true}, 0);
  F2_Update((BranchSample){.at = t += 80, .first_rise = t - 5, .rose = true}, 0);
  F2_Advanced(F2_T_OUT_ADVANCE_MM);
  F2_PivotCommanded();
  F2_PivotAngle(F2_TURN_MAX_DEG - 1.0f); assert(F2_Turning());
  F2_PivotAngle(F2_TURN_MAX_DEG + 1.0f); assert(!F2_Turning() && F2_Failed() && !strcmp(F2_Status(), "F2 TURN NO TAPE"));
  printf("F2 pivot: tape before %.0f deg ignored, branch taken in the window, stops at %.0f deg with no tape PASS\n",
         (double)F2_TURN_MIN_DEG, (double)F2_TURN_MAX_DEG);
}

int main(void) {
  setvbuf(stdout, NULL, _IONBF, 0);
  steering();
  split_latch();
  retrace_handoff();
  retrace_sim(1.0);
  retrace_sim(0.9);
  rev_left_gain = 0.85; retrace_sim(1.0); rev_left_gain = 0.7; retrace_sim(1.0); rev_left_gain = 1.0;
  printf("Retrace: each wheel held to its own recorded pulses - a motor weaker in reverse still goes home PASS\n");
  retrace_stall();
  anchor_tests();
  f2_window();
  printf("Retrace home: straight, curve, zig-zag and pivot reversed on the encoders, weaker motor too PASS\n");
  return 0;
}
