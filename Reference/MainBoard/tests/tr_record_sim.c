/* Host simulation of teach and replay (tr_record.c).
 * Build: gcc -ICore/Inc -o tr_record_sim tests/tr_record_sim.c Core/Src/tr_record.c -lm
 * A scripted "driver" drives the simulated TR while the recorder logs it; the TR is put back at the start and the player
 * replays the recording. The robot model disagrees with the code on purpose: the encoders read 3 % long, the heading estimate
 * reads 2 % long, a rotation command turns 15 % less than the code assumes, the wheels follow the command with the drive's
 * ramp. Checks: the replayed path stays on the recorded one, ends where it ended, the arm follows; mirrored replay; a
 * recording faster than the replay limit; a blocked robot stops with a fault. */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "tr_record.h"
#ifndef M_PI
#define M_PI 3.14159265358979
#endif
static int fails;
#define CHECK(c) do{ if(!(c)){printf("FAIL line %d: %s\n",__LINE__,#c);fails++;} }while(0)

typedef struct { double X, Y, h, vf, vs, wr, odo_f, odo_s, arm; } Robot; /* true field pose (X fwd-at-start, Y right), heading deg cw */
#define ENC 1.03     /* encoder scale error */
#define HSC 1.02     /* heading estimate scale error */
#define WEFF 0.85    /* rotation command is 15 % weaker than assumed */
/* one 5 ms tick: body velocity command (mm/s, + forward / + right), rotation command w (drive convention), arm command */
static void tick(Robot *r, double cf, double cs, double w, int arm, int blocked) {
  const double step = 600.0 / 130.0 * 1037.0 * 0.005; /* ramp: mm/s per tick */
  r->vf += fmax(-step, fmin(step, cf - r->vf));
  r->vs += fmax(-step, fmin(step, cs - r->vs));
  const double wr_cmd = -w * TRR_DEG_S_PER_W * WEFF;    /* heading_sign -1: w > 0 turns counter-clockwise */
  r->wr += (wr_cmd - r->wr) * 0.1;                      /* rotation lag */
  if (blocked) { r->vf = r->vs = 0; r->wr = 0; }
  const double df = r->vf * 0.005, ds = r->vs * 0.005, hr = r->h * M_PI / 180;
  r->X += df * cos(hr) - ds * sin(hr);
  r->Y += df * sin(hr) + ds * cos(hr);
  r->h += r->wr * 0.005;
  r->odo_f += df * ENC;
  r->odo_s += ds * ENC;
  if (arm > 0) r->arm -= 3; else if (arm < 0) r->arm += 3;
  if (r->arm < 1611) r->arm = 1611;
  if (r->arm > 2500) r->arm = 2500;
}
static TRAuto_In mk(const Robot *r, uint32_t now) {
  TRAuto_In in; memset(&in, 0, sizeof in);
  in.dist_mm = (float)r->odo_f; in.strafe_mm = (float)r->odo_s; in.heading_deg = (float)(r->h * HSC);
  in.heading_sign = -1.0f; in.arm_us = (uint16_t)r->arm; in.now = now;
  return in;
}
/* the driver: a field-frame path (in the start frame), ~11 s */
static void driver(double t, double speed, double *vX, double *vY, double *wcw, int *arm) {
  *vX = *vY = *wcw = 0; *arm = 0;
  if (t < 0.0) return;                                         /* waiting */
  if (t < 2.0) *vX = speed;                                   /* forward */
  else if (t < 3.5) *vY = speed * 0.6;                        /* strafe right */
  else if (t < 5.5) { *vX = speed * 0.7; *wcw = 45; }         /* forward while turning 90 deg clockwise */
  else if (t < 6.5) *arm = 1;                                  /* close the arm (stopped) */
  else if (t < 9.0) { *vX = speed * 0.5; *vY = -speed * 0.5; } /* diagonal (field frame) */
  else if (t < 10.0) *arm = -1;                                /* open */
}
static TRR_Recorder rec;
static double recX[3000], recY[3000]; static int recN;
static void record(double speed, Robot *r) {
  memset(r, 0, sizeof *r); r->arm = 2500;
  uint32_t now = 1000;
  TRR_RecStart(&rec, now, 0, 0, 0);
  recN = 0;
  for (int i = 0; i < 400 + 2400 + 300; i++) { /* 2 s waiting (walking to the controller), the 12 s run, 1.5 s waiting */
    double vX, vY, wcw; int arm;
    driver(i < 400 ? -1.0 : (i - 400) * 0.005, speed, &vX, &vY, &wcw, &arm);
    const double hr = r->h * M_PI / 180, cf = vX * cos(hr) + vY * sin(hr), cs = -vX * sin(hr) + vY * cos(hr);
    /* the driver turns with the stick: w such that the TRUE rate is wcw (the human closes the loop by eye) */
    tick(r, cf, cs, -wcw / (TRR_DEG_S_PER_W * WEFF), arm, 0);
    now += 5;
    TRAuto_In in = mk(r, now);
    TRR_RecStep(&rec, now, in.dist_mm, in.strafe_mm, in.heading_deg, in.arm_us);
    if (i % 4 == 0 && recN < 3000) { recX[recN] = r->X; recY[recN] = r->Y; recN++; }
  }
  TRR_RecStop(&rec);
}
static double dist_to_rec(double x, double y, int mirror) {
  double best = 1e9;
  for (int k = 0; k < recN; k++) { const double d = hypot(x - recX[k], y - (mirror ? -recY[k] : recY[k])); if (d < best) best = d; }
  return best;
}
static void replay(const char *name, int mirror, int block_at_ms, double end_tol, double dev_tol, int expect_fault) {
  Robot r; memset(&r, 0, sizeof r); r.arm = 2500;
  TRR_Player p; uint32_t now = 50000;
  CHECK(TRR_PlayStart(&p, rec.buf, rec.n, mirror != 0, now, 0, 0, 0));
  double max_dev = 0; uint32_t t0 = now; int ticks = 0;
  while (TRR_PlayRunning(&p) && ticks < 20000) {
    now += 5; ticks++;
    TRAuto_In in = mk(&r, now);
    TRAuto_Out o; TRR_PlayStep(&p, &in, &o);
    const double cf = o.active ? o.y * TRR_FULL_SCALE_MM_S : 0, cs = o.active ? o.x * TRR_FULL_SCALE_MM_S : 0;
    tick(&r, cf, cs, o.active ? o.w : 0, o.active ? o.arm : 0, block_at_ms >= 0 && (int)(now - t0) > block_at_ms);
    const double dv = dist_to_rec(r.X, r.Y, mirror); if (dv > max_dev) max_dev = dv;
  }
  const TRR_Sample *last = &rec.buf[rec.n - 1];
  const double lastX = recX[recN - 1], lastY = (mirror ? -1 : 1) * recY[recN - 1];
  const double end_err = hypot(r.X - lastX, r.Y - lastY);
  printf("  %-26s %-13s in %5.2f s (recorded %5.2f s): path deviation max %5.1f mm, end %5.1f mm off, heading %+6.1f deg (recorded %+6.1f), arm %4.0f us (recorded %u)\n",
         name, TRR_PlayStateText(p.st), (now - t0) / 1000.0, rec.n * TRR_PERIOD_MS / 1000.0, max_dev, end_err, r.h, (mirror ? -1 : 1) * last->hdg_ddeg / 10.0 / HSC, r.arm, last->arm_us);
  if (expect_fault) { CHECK(p.st == TRR_FAULT); return; }
  CHECK(p.st == TRR_DONE);
  CHECK(max_dev < dev_tol);
  CHECK(end_err < end_tol);
  CHECK(fabs(r.h - (mirror ? -1 : 1) * last->hdg_ddeg / 10.0 / HSC) < 3.0);
  CHECK(fabs(r.arm - last->arm_us) < 40);
}
int main(void) {
  Robot r;
  record(600.0, &r);
  printf("  recorded %u samples, fastest %.0f mm/s, ended at (%.0f, %.0f) heading %.1f\n", rec.n, rec.vmax_mm_s, r.X, r.Y, r.h);
  CHECK(rec.n >= 450 && rec.n <= 560);  /* the 2 s / 1.5 s of waiting and the still end of the run are trimmed */
  replay("replay", 0, -1, 25, 40, 0);
  replay("mirrored replay", 1, -1, 25, 40, 0);
  replay("blocked after 3 s", 0, 3000, 0, 0, 1);
  record(1400.0, &r);   /* recorded faster than the replay may go: the clock slows, it must still follow the path */
  printf("  recorded %u samples, fastest %.0f mm/s (limit %.0f)\n", rec.n, rec.vmax_mm_s, TRR_VMAX_MM_S);
  replay("too-fast recording", 0, -1, 25, 60, 0);
  /* never moved: nothing kept */
  { TRR_RecStart(&rec, 0, 0, 0, 0); for (uint32_t t = 0; t < 2000; t += 5) TRR_RecStep(&rec, t, 0, 0, 0, 2500); TRR_RecStop(&rec); CHECK(rec.n == 0); }
  /* the pose tracker on its own: a square driven with the robot turning in place at each corner returns to the start */
  { TRR_Pose ps; TRR_PoseStart(&ps, 0, 0, 0); float f = 0, h = 0;
    for (int side = 0; side < 4; side++) { for (int i = 0; i < 100; i++) { f += 10; TRR_PoseUpdate(&ps, f, 0, h); } for (int i = 0; i < 90; i++) { h += 1; TRR_PoseUpdate(&ps, f, 0, h); } }
    printf("  pose tracker, 1 m square: back at (%.1f, %.1f)\n", ps.x, ps.y); CHECK(fabs(ps.x) < 1 && fabs(ps.y) < 1); }
  printf(fails ? "%d FAILED\n" : "TR RECORD / REPLAY SIM PASS\n", fails);
  return fails != 0;
}
