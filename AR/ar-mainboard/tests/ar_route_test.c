/* Host test of the hardcoded route (MID / TOP / ALL). Scripted sensors + a crude odometer; no hardware. */
#include <stdio.h>
#include <string.h>
#include "ar_nav.h"
static int fails;
#define CHECK(c) do{ if(!(c)){printf("FAIL line %d: %s\n",__LINE__,#c);fails++;} }while(0)
static AR_Line L(int l,int c,int r){AR_Line s={l,c,r};return s;}
static uint32_t t, odo_f; /* odo in 1/1000 edge */
static AR_Cmd last;
static AR_Cmd tick(AR_Nav*n,AR_Line s,int us,int fresh){
  t+=5; int a=last.left<0?-last.left:last.left, b=last.right<0?-last.right:last.right;
  odo_f += (uint32_t)((a+b)/2*5*10);           /* 0.05 edge per ms per percent */
  AR_Nav_SetOdometer(n, odo_f/1000);
  last=AR_Nav_Step(n,t,s,us,fresh); return last;
}
static void run(AR_Nav*n,AR_Line s,int ms,int us){ for(int i=0;i<ms;i+=5) tick(n,s,us,(i%60)==0); }
static int in(AR_Nav*n,const char*name){ return strcmp(AR_Nav_PhaseText(n),name)==0; }
/* drive straight on the line until the step name changes (or ms run out); optional us trigger once odo moved far enough */
static void until_name(AR_Nav*n,const char*name,int max_ms,uint32_t trig_odo,uint32_t base){
  for(int i=0;i<max_ms && in(n,name);i+=5){
    int us = (n->odo-base>=trig_odo)?50:500; tick(n,L(0,1,0),us,(i%60)==0);
  }
}
/* pivot: sensors off the line for a while, then centre on the new line */
static void do_pivot(AR_Nav*n,const char*name,int off_ms){
  int i=0; for(; i<off_ms && in(n,name); i+=5) tick(n,L(0,0,0),-1,0);
  for(int k=0;k<40 && in(n,name);k++) tick(n,L(0,1,0),-1,0);
}
static void full(AR_Key key,int stages_done){
  AR_Nav n; AR_Nav_Init(&n); t=1000; odo_f=0; last=(AR_Cmd){0,0};
  AR_Nav_SetKey(&n,key); AR_Nav_Start(&n,t);
  run(&n,L(0,1,0),AR_COUNTDOWN_MS+50,-1); CHECK(n.phase==AR_PH_OUT);
  CHECK(in(&n,"MID TO KEY"));
  run(&n,L(0,1,0),300,500); CHECK(last.left>0&&last.right>0);
  /* curve branch on the right while the centre is on the line: ignored, no turn */
  tick(&n,L(0,1,1),500,0); CHECK(last.left==AR_BASE_PCT&&last.right==AR_BASE_PCT);
  run(&n,L(0,1,0),500,500);
  /* junction J */
  run(&n,L(1,1,1),30,500); run(&n,L(0,1,0),AR_CROSS_ADVANCE_MS+20,500);
  uint32_t base=n.odo;
  /* drive on to the key (US triggers after ~1000 edges from J) */
  until_name(&n,"MID TO KEY",30000,1000,base); CHECK(in(&n,"MID GRAB"));
  uint32_t out=n.out_edges; CHECK(out>=900 && out<1300);
  run(&n,L(0,1,0),AR_GRAB_PAUSE_MS+AR_STEP_SETTLE_MS+30,50); CHECK(in(&n,"MID REVERSE"));
  uint32_t o0=n.odo; run(&n,L(0,1,0),AR_STEP_SETTLE_MS+100,-1); CHECK(last.left<0&&last.right<0);
  tick(&n,L(1,1,0),-1,0); /* line on the left while reversing: right wheel reverses more slowly */
  CHECK(last.left==-AR_BASE_PCT && last.right>last.left);
  for(int i=0;i<20000 && in(&n,"MID REVERSE");i+=5) tick(&n,L(0,1,0),-1,0);
  uint32_t back=n.odo>o0 ? n.odo-o0 : 0;
  CHECK(back>=out && back<out+30);   /* reversed the same distance (within the step before it stops) */
  if (key==AR_KEY_MID){ CHECK(n.phase==AR_PH_DONE); CHECK(last.left==0&&last.right==0); CHECK(in(&n,"ROUTE PART DONE")); return; }
  run(&n,L(0,1,0),AR_STEP_SETTLE_MS+20,-1); CHECK(in(&n,"TURN NORTH"));
  run(&n,L(0,1,0),50,-1); CHECK(last.left>0&&last.right<0);              /* pivots right */
  /* centre still on the old line, never off: keep turning */
  run(&n,L(0,1,0),AR_PIVOT_MIN_MS+400,-1); CHECK(in(&n,"TURN NORTH"));
  do_pivot(&n,"TURN NORTH",300); CHECK(in(&n,"TOP TO KEY"));
  run(&n,L(0,1,0),AR_STEP_SETTLE_MS+20,500);
  base=n.odo; until_name(&n,"TOP TO KEY",30000,800,base); CHECK(in(&n,"TOP GRAB"));
  run(&n,L(0,1,0),AR_GRAB_PAUSE_MS+AR_STEP_SETTLE_MS+30,50); CHECK(in(&n,"TOP REVERSE"));
  o0=n.odo; run(&n,L(0,1,0),AR_STEP_SETTLE_MS+5,-1);
  for(int i=0;i<20000 && in(&n,"TOP REVERSE");i+=5) tick(&n,L(0,1,0),-1,0);
  uint32_t want=(uint32_t)(AR_TOP_BACK_CM*AR_EDGES_PER_CM); CHECK(n.odo-o0>=want && n.odo-o0<want+30);
  if (key==AR_KEY_TOP2){ CHECK(n.phase==AR_PH_DONE); return; }
  run(&n,L(0,1,0),AR_STEP_SETTLE_MS+20,-1); CHECK(in(&n,"TURN DIAGONAL"));
  /* 116 degrees: must not finish before the minimum pivot time even if a line shows up early */
  run(&n,L(0,0,0),20,-1); tick(&n,L(0,1,0),-1,0); CHECK(in(&n,"TURN DIAGONAL"));
  do_pivot(&n,"TURN DIAGONAL",800); CHECK(in(&n,"DIAGONAL"));
  o0=n.odo; run(&n,L(0,1,0),AR_STEP_SETTLE_MS+20,-1);
  for(int i=0;i<30000 && in(&n,"DIAGONAL");i+=5) tick(&n,L(0,1,0),-1,0);
  want=(uint32_t)(AR_DIAG_CM*AR_EDGES_PER_CM); CHECK(n.odo-o0>=want && n.odo-o0<want+30);
  run(&n,L(0,1,0),AR_STEP_SETTLE_MS+20,-1); CHECK(in(&n,"TURN UP-RIGHT"));
  tick(&n,L(0,0,0),-1,0); CHECK(last.left<0&&last.right>0);              /* pivots LEFT */
  do_pivot(&n,"TURN UP-RIGHT",500); CHECK(in(&n,"UP-RIGHT"));
  run(&n,L(0,1,0),AR_STEP_SETTLE_MS+20,-1);
  for(int i=0;i<30000 && in(&n,"UP-RIGHT");i+=5) tick(&n,L(0,1,0),-1,0);
  run(&n,L(0,1,0),AR_STEP_SETTLE_MS+20,-1); CHECK(in(&n,"TURN EAST"));
  do_pivot(&n,"TURN EAST",300); CHECK(in(&n,"RIGHT TO KEY"));
  run(&n,L(0,1,0),AR_STEP_SETTLE_MS+20,500);
  base=n.odo; until_name(&n,"RIGHT TO KEY",30000,400,base);
  CHECK(n.phase==AR_PH_GRAB);                                            /* right key: generic grab, turn round, home */
  run(&n,L(0,1,0),AR_GRAB_PAUSE_MS+20,-1); CHECK(n.phase==AR_PH_TURNAROUND);
  do_pivot(&n,"TURN AROUND",400); CHECK(n.phase==AR_PH_RETURN);
  run(&n,L(0,1,0),400,-1); run(&n,L(0,0,0),AR_HOME_LOST_MS+50,-1); CHECK(n.phase==AR_PH_HOME_ADVANCE);
  run(&n,L(0,0,0),AR_HOME_ADVANCE_MS+20,-1); CHECK(n.phase==AR_PH_DONE); CHECK(in(&n,"DONE AT HOME"));
  (void)stages_done;
}
int main(void){
  full(AR_KEY_MID,1); printf("  MID route ok\n");
  full(AR_KEY_TOP2,2); printf("  MID+TOP route ok\n");
  full(AR_KEY_ALL,3); printf("  ALL route ok\n");
  /* a step that never completes faults instead of running forever */
  { AR_Nav n; AR_Nav_Init(&n); t=1000; odo_f=0; last=(AR_Cmd){0,0};
    AR_Nav_SetKey(&n,AR_KEY_MID); AR_Nav_Start(&n,t); run(&n,L(0,1,0),AR_COUNTDOWN_MS+50,-1);
    run(&n,L(0,1,0),AR_STEP_MAX_MS+200,500); CHECK(n.phase==AR_PH_FAULT); CHECK(in(&n,"FAULT STEP"));
    CHECK(last.left==0&&last.right==0); }
  /* lost line on the way to the middle key faults */
  { AR_Nav n; AR_Nav_Init(&n); t=1000; odo_f=0; last=(AR_Cmd){0,0};
    AR_Nav_SetKey(&n,AR_KEY_MID); AR_Nav_Start(&n,t); run(&n,L(0,1,0),AR_COUNTDOWN_MS+400,500);
    run(&n,L(0,0,0),AR_END_LOST_MS+300,500); CHECK(n.phase==AR_PH_FAULT); }
  if(fails){printf("AR ROUTE FAILED (%d)\n",fails);return 1;}
  printf("AR ROUTE PASS\n"); return 0;
}
