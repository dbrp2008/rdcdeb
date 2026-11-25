#ifndef RETRACE_H
#define RETRACE_H
#include <stdbool.h>
#include <stdint.h>
#include "line_drive.h"

/* Return home by RETRACING the outbound run on the wheel encoders (no line, no turn-around, no US). On the way out every
 * wheel command is recorded as "breadcrumbs": about every RETRACE_CRUMB_PULSES encoder pulses (both wheels together, ~1 cm)
 * the average left / right duty that was commanded AND how many pulses EACH wheel turned. Going home the breadcrumbs are
 * replayed from the last to the first with both wheels REVERSED, and each wheel is held to its own recorded pulses (the one
 * that gets ahead is slowed): the same wheel turns back the same number of turns, so a motor that is weaker in reverse, a
 * lower battery or unequal wheels do not bend the path (field: the power-only replay did not reverse straight) - straight
 * parts, steering corrections and pivots (e.g. the F2 branch turn) alike. Pure logic: no HAL; the caller passes the signed ELCP
 * pulse counts (which follow the commanded direction) and the time. */

#ifndef RETRACE_MAX_CRUMBS
#define RETRACE_MAX_CRUMBS 1024u /* x RETRACE_CRUMB_PULSES: ~7 m of travel */
#endif
#ifndef RETRACE_CRUMB_PULSES
#define RETRACE_CRUMB_PULSES 12u /* J1 + J3 pulses per breadcrumb (~1 cm of each wheel at ~115 pulses per 20 cm turn) */
#endif
#ifndef RETRACE_CRUMB_MAX_MS
#define RETRACE_CRUMB_MAX_MS 150u /* ...or this long, whichever comes first */
#endif
#ifndef RETRACE_STALL_MS
#define RETRACE_STALL_MS 1000u /* no encoder pulse for this long while reversing: stop (stuck) */
#endif
#ifndef RETRACE_MIN_PERCENT
#define RETRACE_MIN_PERCENT 12.0f /* never reverse a wheel slower than this (it would stall), unless it was stopped */
#endif

typedef struct {
  int16_t left_x10, right_x10; /* average commanded duty, 0.1 % */
  uint16_t pulses;             /* |dJ1| + |dJ3| while it was commanded */
  uint16_t ms;                 /* how long it lasted (used when it moved no encoder, e.g. a stalled start) */
  uint16_t pl, pr;             /* |dJ3| (left wheel) and |dJ1| (right wheel) pulses: replayed WHEEL TURNS, not just power */
} RetraceCrumb;

/* Start-zone checkpoint: the zone is outlined in black. On the way out the front IRs cross that outline (all three black)
 * within the first RETRACE_EDGE_MAX_PULSES: the distance to that moment is noted. Reversing home they cross it again; when
 * they LEAVE it (back inside the zone) the AR is exactly that distance from the start, whatever the encoders drifted over
 * the trip: the rest of the replay is corrected to match. Only accepted within RETRACE_EDGE_WINDOW_PULSES of where the
 * trail expects it (a crossing elsewhere on the field is ignored). */
#ifndef RETRACE_WHEEL_K
#define RETRACE_WHEEL_K 3.0f /* per-wheel matching: the wheel ahead (fraction of its breadcrumb) is slowed by K x the lead */
#endif
#ifndef RETRACE_EDGE_MAX_PULSES
#define RETRACE_EDGE_MAX_PULSES 400u /* both wheels together: ~35 cm per wheel */
#endif
#ifndef RETRACE_EDGE_WINDOW_PULSES
#define RETRACE_EDGE_WINDOW_PULSES 120u /* ~10 cm per wheel */
#endif

typedef enum { RETRACE_RECORDING = 0, RETRACE_RETURNING, RETRACE_DONE, RETRACE_STALLED, RETRACE_PARTIAL } RetraceState;

typedef struct {
  RetraceState state;
  RetraceCrumb crumbs[RETRACE_MAX_CRUMBS];
  uint16_t n;                  /* closed breadcrumbs */
  bool overflow;               /* ran out of breadcrumbs: the return stops early (RETRACE_PARTIAL) */
  /* the breadcrumb being recorded */
  float sum_left, sum_right;   /* duty x ms */
  uint32_t acc_ms, acc_pulses, last_ms;
  uint32_t acc_pl, acc_pr;     /* per-wheel pulses in the breadcrumb being recorded */
  int32_t last_j1, last_j3;
  bool have_last;
  float cur_left, cur_right;   /* last command (held until the next call) */
  /* returning */
  int32_t k;                   /* breadcrumb being replayed (counts down) */
  int32_t done_pulses;         /* pulses done in the current breadcrumb (negative after a checkpoint that asked for more) */
  int32_t done_pl, done_pr;    /* ...per wheel (carried into the next breadcrumb like done_pulses) */
  uint32_t crumb_start_ms, last_pulse_ms;
  /* start-zone checkpoint */
  uint32_t recorded_pulses;    /* total in the closed breadcrumbs */
  bool edge_marked, edge_in, anchored;
  uint32_t edge_mark;          /* pulses from the start to where the front IRs reached the outline on the way out */
  int32_t anchor_fix;          /* the correction applied there (pulses, + = it was further than the encoders said) */
} Retrace;

void Retrace_Start(Retrace *r, uint32_t now, int32_t j1, int32_t j3);
/* Every control tick on the way out: the command just sent and the current pulse counts. Zero commands are not recorded. */
void Retrace_Record(Retrace *r, uint32_t now, LineWheels sent, int32_t j1, int32_t j3);
/* Every filtered IR scan (bits L/C/R): finds the start-zone outline on the way out and uses it on the way home. */
void Retrace_Line(Retrace *r, unsigned filtered);
/* Every control tick on the way home: the command to send now. Returns false when finished (r->state says how). */
bool Retrace_Step(Retrace *r, uint32_t now, int32_t j1, int32_t j3, LineWheels *cmd);
#endif
