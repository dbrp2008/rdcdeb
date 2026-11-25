/* Link with function1.c alone: no HAL, simulated board or other routes. */
#include "function1.h"
#include "f12_duties.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void wheels(uint32_t now, float left, float right) {
  assert(F1_Advance(now) == NULL);
  F1_Command cmd = F1_CommandAt(now, 200);
  assert(cmd.request == F1_MOVE && duty_is(cmd.wheels.left,left) && duty_is(cmd.wheels.right,right));
}
static void launch(uint32_t t)   { wheels(t, DUTY_LAUNCH, DUTY_LAUNCH); }
static void straight(uint32_t t) { wheels(t, DUTY_STRAIGHT, DUTY_STRAIGHT); }
static void reverse(uint32_t t)  { wheels(t, -DUTY_REVERSE, -DUTY_REVERSE); }
static void ccw(uint32_t t)      { wheels(t, -DUTY_CCW, DUTY_CCW); }
static void holding(uint32_t now) {
  assert(!F1_Advance(now));
  F1_Command cmd=F1_CommandAt(now,200);
  assert(cmd.request==F1_KEEP_STOPPED && !cmd.wheels.left && !cmd.wheels.right);
  assert(!strcmp(cmd.status,"TARGET 60MM HOLD") && F1_PhaseAt(now)==F1_TARGET_HOLD);
}
static void heading(uint32_t now, unsigned dir) {
  const float left  = dir == LINE_LEFT ? DUTY_INSIDE : dir == LINE_RIGHT ? DUTY_OUTSIDE : DUTY_STRAIGHT;
  const float right = dir == LINE_LEFT ? DUTY_OUTSIDE : dir == LINE_RIGHT ? DUTY_INSIDE : DUTY_STRAIGHT;
  wheels(now, left, right);
}
static void scan(uint32_t now, unsigned mask) { F1_Scan(now, now, mask, mask); }
static void outbound(void) {
  F1_Init();
  assert(F1_CommandAt(0, 200).request == F1_KEEP_STOPPED && !F1_TargetPing(0, 39));
  F1_Start(100, 2, true); /* centre tape, but an edge pin is already high */
  F1_SideSample((BranchSample){.at=110, .high=true});
  for (unsigned mask=0; mask<8; ++mask) scan(111+mask, mask);
  assert(F1_BranchCount() == 0); /* none of the eight analog masks counts */
  F1_SideSample((BranchSample){.at=120}); F1_SideSample((BranchSample){.at=195});
  F1_SideSample((BranchSample){.at=205, .high=true}); /* sampled high, no IRQ needed */
  assert(F1_BranchCount() == 1);
  F1_SideSample((BranchSample){.at=210});
  F1_SideSample((BranchSample){.at=290, .rose=true, .first_rise=285}); /* pulse already low */
  assert(F1_BranchCount() == 2);
  scan(295, 6);
  launch(849); straight(850); /* no turn on crossing two */
  scan(860, 1); heading(860, LINE_LEFT);
  scan(870, 4); heading(870, LINE_RIGHT);
  scan(880, 0); straight(880); /* white clears right immediately */
  straight(10099); straight(10100); /* 10 s from start is not a stop */
  F1_Stop(); scan(10200, 3); F1_SideSample((BranchSample){.at=10200});
  F1_SideSample((BranchSample){.at=10300, .rose=true, .first_rise=10295});
  assert(F1_BranchCount() == 2 && F1_CommandAt(10300, 200).request == F1_KEEP_STOPPED);
}
/* Every filtered mask from straight, left and right. Raw is deliberately the
 * opposite: it cannot select or release a forward correction. */
static void steering(void) {
  const unsigned prior[] = {2, 1, 4};
  /* Numeric mask order is L=1, C=2, R=4. Test all 24 transitions. */
  static const unsigned rule[8] = {2, 1, 2, 2, 4, 1, 2, 2}; /* absent a prior split */
  for (unsigned i = 0; i < 3; ++i) for (unsigned mask = 0; mask < 8; ++mask) {
    F1_Init(); F1_Start(0, 2, false);
    scan(749, 5); launch(749); /* 101 cannot steer during launch */
    scan(750, prior[i]);
    F1_Scan(760, 760, mask ^ 7u, mask); /* two observations between command reads */
    const unsigned expected = rule[mask];
    heading(760, expected); heading(765, expected);
    const char *label = expected == 1 ? "RUN LEFT" : expected == 4 ? "RUN RIGHT" :
                        mask == 0 ? "NO LINE STRAIGHT" : mask == 2 ? "RUN STRAIGHT" : "BRANCH STRAIGHT";
    assert(!strcmp(F1_CommandAt(765, 200).status, label));
    scan(770, 0); heading(770, mask == 5u ? LINE_LEFT : LINE_CENTRE);
    /* No command on the split: one filtered 101 is enough. Raw 010 and all
     * other filtered masks (including 110, 011 and 111) cannot release it. */
    F1_Scan(780,780,2,5); F1_Scan(790,790,2,mask);
    heading(790, mask == 2u ? LINE_CENTRE : LINE_LEFT);
    /* No command on release either. Raw-only 101 cannot immediately re-latch. */
    F1_Scan(800,800,5,2); F1_Scan(810,810,5,4); heading(810, LINE_RIGHT);
    scan(820, 0); straight(820);
  }
  scan(821, 5); heading(821, LINE_LEFT);
  F1_Stop(); F1_Start(825, 0, false); straight(1575); /* no split carried between runs */
  scan(1580, 5); heading(1580, LINE_LEFT);
  assert(F1_TargetPing(1600, 59)); scan(1610, 4); heading(1949, LINE_LEFT);
  assert(!strcmp(F1_CommandAt(1949,200).status,"TARGET APPROACH"));
  holding(1950); reverse(2950); ccw(3450); /* no 010 was needed to stop or reverse */
  scan(3460, 0); scan(4450, 4); ccw(4450);
  scan(4810, 5); scan(4890, 0); straight(4900); /* old outbound/alignment splits did not arm return */
  scan(4910, 4); heading(4910, LINE_RIGHT);
  scan(4920, 6); straight(4920); /* 001 -> 011 -> 000: right, straight, straight */
  scan(4925, 0); straight(4925);
  scan(4930, 1); heading(4930, LINE_LEFT);
  F1_Scan(4940, 4940, 1, 3); /* filtered 100 -> 110, even with raw left only */
  straight(4940);
  scan(4950, 0); straight(4950); /* 110 cleared left; white cannot restore it */
  scan(4960, 4); heading(4960, LINE_RIGHT);
  F1_Scan(4965, 4965, 4, 5); /* return: filtered 101 starts a left hold even with raw 001 */
  heading(4965, LINE_LEFT); assert(!strcmp(F1_CommandAt(4965, 200).status, "TARGET FOLLOW L"));
  scan(4968, 0); heading(4968, LINE_LEFT);
  scan(4970, 2); scan(4980, 0); straight(4980); /* exact centre cleared the hold between commands */
  scan(4990, 4); heading(4990, 4);
  F1_Scan(5000, 5000, 4, 7); /* return: without a split, filtered 111 still clears an ordinary right correction */
  heading(5000, 2); scan(5010, 0); heading(5010, 2);
  assert(!strcmp(F1_CommandAt(5010, 200).status, "TARGET FOLLOW") && F1_BranchCount() == 0);
  F1_Stop(); F1_Start(5350, 0, false); /* armed before stopping the last run */
  scan(6100, 5); heading(6100, LINE_LEFT); /* armed 101 still follows; only 111 is the finish */
  scan(6110, 7); wheels(6110, -DUTY_STRAIGHT, -DUTY_STRAIGHT); /* finish overrides a held correction */
}
/* Include the latest mask actually used to steer on entering a forward leg,
 * not a transient from open-loop launch. Cross tick wrap and the launch
 * boundary while an early US1 approach owns the forward wheels. */
static void split_boundaries(void) {
  const uint32_t t=UINT32_MAX-735u;
  F1_Init(); F1_Start(t,2,false);
  scan(t+730u,5); launch(t+749u); heading(t+750u,LINE_LEFT);
  scan(t+760u,4); heading(t+760u,LINE_LEFT); scan(t+770u,2); straight(t+770u);
  F1_Init(); F1_Start(t,2,false); scan(t+650u,5); scan(t+690u,0);
  assert(F1_TargetPing(t+700u,59)); straight(t+700u); /* an old launch split is not queued */
  scan(t+710u,5); scan(t+720u,0); heading(t+749u,LINE_LEFT); heading(t+750u,LINE_LEFT);
  scan(t+760u,6); heading(t+1049u,LINE_LEFT); holding(t+1050u);
  F1_Init(); F1_StartPerScan(t,2); scan(t+750u,5); straight(t+750u);
  scan(t+760u,4); heading(t+760u,LINE_RIGHT); /* F2 never inherits F1's 101 rule */
  puts("F1 split: filtered 101 holds left until exact 010; no launch history, no F2 latch, approach and stop priority PASS");
}
static void turnaround(void) {
  uint32_t t = UINT32_MAX - 2200u; /* tick wrap during the pivot */
  F1_Init(); F1_Start(t - 10u, 2, false);
  assert(!F1_TargetPing(t - 1u, 60)); launch(t-1u); /* equality is clear */
  assert(F1_TargetPing(t, 59) && F1_PhaseAt(t)==F1_TARGET_APPROACH);
  F1_Start(t + 20u, 7, false); /* cannot restart while approaching */
  straight(t); scan(t+70u,1); heading(t+70u,1);
  assert(!F1_TargetPing(t+100u,59) && !F1_TargetPing(t+300u,200));
  scan(t+349u,0); straight(t+349u);
  assert(!strcmp(F1_CommandAt(t+349u,200).status,"TARGET APPROACH"));
  holding(t+350u); holding(t+1349u);
  reverse(t + 1350u); reverse(t + 1849u);
  uint32_t spin = t + 1850u;
  ccw(spin);
  scan(spin + 10u, 0);
  F1_Scan(spin + 990u, spin + 1003u, 4, 4); /* started too early */
  ccw(spin + 1003u); assert(F1_PhaseAt(spin + 1003u) == F1_PIVOT);
  F1_Scan(spin + 1010u, spin + 1010u, 4, 1); /* different sensors */
  ccw(spin + 1010u); assert(F1_PhaseAt(spin + 1010u) == F1_PIVOT);
  scan(spin + 1020u, 4);
  uint32_t picked = spin + 1030u; /* a ping finished after the accepted scan */
  ccw(picked); assert(F1_PhaseAt(picked) == F1_ALIGN);
  ccw(picked + 449u); heading(picked + 450u, LINE_RIGHT);
  scan(picked + 460u, 6); straight(picked + 460u); /* 011 clears right */
  scan(picked + 465u, 7); /* actual return marker, not the pickup or a partial mask */
  scan(picked + 470u, 0); straight(picked + 470u); straight(picked + 969u);
  assert(!F1_TargetPing(picked + 970u, 54) && F1_Advance(picked + 970u) == NULL);
  F1_Command end = F1_CommandAt(picked + 970u, 54); /* enters range while still white */
  assert(end.request == F1_STOP && strcmp(end.status, "TARGET LINE END") == 0);
  assert(F1_FinishArmed() && F1_PhaseAt(picked + 970u) == F1_IDLE);

  uint32_t next = picked + 1000u;
  F1_Start(next, 2, false); assert(F1_FinishArmed() && F1_BranchCount() == 0);
  scan(next + 10u, 7); wheels(next + 10u, -DUTY_STRAIGHT, -DUTY_STRAIGHT);
  scan(next + 20u, 4); wheels(next + 20u, -DUTY_FINISH_INNER, -DUTY_STRAIGHT);
  scan(next + 30u, 1); wheels(next + 209u, -DUTY_STRAIGHT, -DUTY_FINISH_INNER);
  const char *reason = F1_Advance(next + 210u);
  assert(reason && strcmp(reason, "FINISH 3 BLACK") == 0 && !F1_FinishArmed());
}
/* Put BBB in every pre-return manoeuvre, including the qualifying pivot scan.
 * At the return boundary, the cached sample can steer but cannot arm the end. */
static uint32_t turn_to_return(uint32_t target, unsigned last_align) {
  assert(F1_TargetPing(target, 59));
  target += 350u; holding(target);
  scan(target + 10u, 7); reverse(target + 1000u);
  scan(target + 1010u, 7); ccw(target + 1500u);
  scan(target + 1510u, 0); scan(target + 2500u, 7);
  ccw(target + 2500u);
  scan(target + 2949u, last_align); ccw(target + 2949u);
  straight(target + 2950u); assert(F1_PhaseAt(target + 2950u) == F1_RETURN);
  return target + 2950u;
}
/* Isolate the finish predicate with a supplied fresh range. New-ping
 * latching and the intersection with this predicate are replayed separately. */
static void still_returning(uint32_t now, uint32_t mm) {
  assert(!F1_Advance(now) && F1_PhaseAt(now) == F1_RETURN);
  assert(F1_CommandAt(now, mm).request == F1_MOVE);
}
static void return_gate(bool per_scan) {
  const uint32_t target = 0u - 4500u - 3300u; /* final filtered white arrives exactly at tick zero */
  F1_Init();
  if (per_scan) F1_StartPerScan(target - 1500u, 7);
  else F1_Start(target - 1500u, 7, false);
  scan(target - 700u, 7); scan(target - 690u, 0); straight(target - 1u);
  const uint32_t r = turn_to_return(target, 7);
  F1_Scan(r - 1u, r + 1u, 7, 7); /* started in alignment, delivered on return */
  scan(r + 10u, 0); still_returning(r + 10u, 54); still_returning(r + 550u, 54);
  for (unsigned mask = 1; mask < 7; ++mask) {
    const uint32_t t = r + 600u + (mask - 1u) * 550u;
    F1_Scan(t, t, 7, mask); still_returning(t, 54); /* raw BBB cannot arm; none of these is filtered BBB */
    scan(t + 10u, 0); still_returning(t + 10u, 54); still_returning(t + 510u, 54);
  }
  F1_Scan(r + 3990u, r + 3990u, 0, 7); /* filtered, not raw BBB; latch it even between commands */
  scan(r + 3991u, 0); still_returning(r + 3991u, 55); still_returning(r + 4495u, 55);
  F1_Scan(r + 4498u, r + 4498u, 0, 2); still_returning(r + 4498u, 54); /* raw/past white is insufficient */
  F1_Scan(r + 4499u, 0u, 7, 0); /* filtered WWW at tick zero; no second BBB or dwell */
  const uint32_t mm = per_scan ? 39u : 54u;
  assert(!F1_TargetPing(0u, mm)); /* at 39 mm this is a finish, not a new hold */
  assert(!F1_Advance(0u));
  const F1_Command end = F1_CommandAt(0u, mm);
  assert(end.request == F1_STOP && !strcmp(end.status, "TARGET LINE END"));
  assert(F1_PhaseAt(0u) == F1_IDLE && F1_BranchCount() == 0u);
}
static void long_return(void) {
  F1_Init(); F1_Start(0,2,false);
  uint32_t r=turn_to_return(800,7);
  scan(r+10u,7); scan(r+20u,0); still_returning(r+20u,60);
  scan(r+30u,2); r=turn_to_return(r+31u,0); /* repeated turnaround needs a new marker */
  scan(r+10u,0); still_returning(r+10u,54); still_returning(r+10000u,60);
  scan(r+15000u,2); still_returning(r+15000u,60);
  still_returning(r+25000u,60); /* no timeout after first black either */
  scan(r+25010u,0); still_returning(r+25010u,54); /* previous leg's 111 cannot end this leg */
  scan(r+26000u,7); scan(r+26010u,0);
  F1_Command end=F1_CommandAt(r+26010u,54);
  assert(end.request==F1_STOP && !strcmp(end.status,"TARGET LINE END"));
}
/* Fast-forward the portable controller only. Board freshness is exercised
 * separately with serviced ADC scans and pings. Neither untimed leg should
 * depend on the age of a run-start timestamp after the HAL tick rolls over. */
static void long_tick_wrap(void) {
  F1_Init(); F1_Start(100,4,false); heading(850,4);
  const uint32_t step=1u<<30;
  for(unsigned i=1;i<=4;++i) heading(101u+i*step,4); /* no second launch at tick 101 */
  uint32_t r=turn_to_return(301,2);
  scan(r+step,1); heading(r+step,1);
  scan(r+2u*step,4); heading(r+2u*step,4);
  scan(r+3u*step,7); heading(r+3u*step,2);
  scan(r+1u,0); assert(F1_CommandAt(r+1u,54).request==F1_STOP);
}
static void approach_priority(void) {
  /* The finish can arm and qualify DURING approach, even on its deadline.
   * Filtered white + 55 is not a finish, but a new valid 54 is. */
  for(unsigned f2=0; f2<2; ++f2) for(unsigned delta=349;delta<=350; ++delta) {
    F1_Init(); if(f2) F1_StartPerScan(0,2); else F1_Start(0,2,false);
    uint32_t t=turn_to_return(800,2)+10u;
    assert(F1_TargetPing(t,55)); scan(t+10u,7); scan(t+20u,0);
    F1_Command c=F1_CommandAt(t+348u,55); assert(c.request==F1_MOVE && duty_is(c.wheels.left,DUTY_STRAIGHT) && duty_is(c.wheels.right,DUTY_STRAIGHT));
    assert(!F1_TargetPing(t+delta,54) && !F1_Advance(t+delta));
    c=F1_CommandAt(t+delta,54); assert(c.request==F1_STOP && !strcmp(c.status,"TARGET LINE END"));
  }
  /* White at 54 without a fresh return marker really is an ordinary target;
   * at 55, even marked white is not a finish. Neither bypasses the delay. */
  for (unsigned marked=0; marked<2; ++marked) {
    F1_Init(); F1_Start(0,2,false); uint32_t t=turn_to_return(800,7)+20u;
    if (marked) scan(t-10u,7);
    scan(t,0); const unsigned mm=marked?55u:54u; assert(F1_TargetPing(t,mm));
    F1_Command c=F1_CommandAt(t+349u,mm); assert(c.request==F1_MOVE && !strcmp(c.status,"TARGET APPROACH"));
    c=F1_CommandAt(t+350u,mm); assert(c.request==F1_KEEP_STOPPED && F1_PhaseAt(t+350u)==F1_TARGET_HOLD);
  }
  /* Unserviced deadline: never backdate the hold or jump straight to reverse. */
  uint32_t t=UINT32_MAX-100u; F1_Init(); F1_Start(t,2,false); assert(F1_TargetPing(t,59));
  heading(t+349u,2); holding(t+1600u); holding(t+2599u); reverse(t+2600u);
  /* Old 10-second boundary falls exactly at reverse entry, inside the hold,
   * inside the approach, or before even detecting the target. */
  const unsigned near[]={8650,8900,9800,20000};
  for(unsigned i=0;i<sizeof near/sizeof *near;++i) {
    F1_Init(); F1_Start(0,2,false); straight(near[i]-1u);
    assert(F1_TargetPing(near[i],35)); straight(near[i]+349u);
    holding(near[i]+350u); holding(near[i]+1349u);
    reverse(near[i]+1350u); reverse(near[i]+1849u); ccw(near[i]+1850u);
  }
  /* A second target late on return still gets its own hold and reverse. */
  F1_Init(); F1_Start(0,2,false); uint32_t r=turn_to_return(800,0);
  assert(F1_TargetPing(r+9990u,55)); straight(r+10000u); holding(r+10340u); reverse(r+11340u);
  puts("F1/F2 approach: 59/60 gate, 349/350 ms, immediate finish priority and late targets through reverse past 10 s PASS");
}
int main(void) {
  return_gate(false); return_gate(true); long_return(); long_tick_wrap();
  puts("Return finish (F1/F2): fresh post-CCW 111 arms; current filtered 000 and US1 <55 finish without dwell; repeat turnaround, long legs and tick wrap PASS");
  approach_priority(); steering(); split_boundaries(); outbound(); turnaround();
  puts("Standalone F1 controller: ordinary steering, held split, launch, crossings, US1 turnaround, return, finish and deadlines without board/F2/F3 PASS");
}
