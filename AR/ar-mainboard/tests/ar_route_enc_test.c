/* Host test of the ENCODER-ONLY hardcoded route (AR_ROUTE_USE_IR 0). IR input is always "nothing seen". */
#include <stdio.h>
#include <string.h>
#include "ar_nav.h"
static int fails;
#define CHECK(c) do{ if(!(c)){printf("FAIL line %d: %s\n",__LINE__,#c);fails++;} }while(0)
static uint32_t t; static double el, er; /* wheel edges */
static AR_Cmd last; static int stalled; static int coast_cfg, coast_left, prev_moving; /* simulated roll after a stop: coast_cfg ticks of 1 edge each */ /* 1 = wheels held still (pushing against something) */ static double rmul=0.97; /* right wheel 3% slow */
static AR_Line none={0,0,0};
static void tick(AR_Nav*n,int us,int fresh){
  t+=5; double a=last.left<0?-last.left:last.left, b=last.right<0?-last.right:last.right;
  if(!stalled){ el+=a*0.05*5; er+=b*0.05*5*rmul; }
  { int moving=(last.left||last.right); if(!moving&&prev_moving) coast_left=coast_cfg; prev_moving=moving;
    if(!moving&&coast_left>0){ el+=1; er+=1; coast_left--; } }
  AR_Nav_SetWheelOdometer(n,(uint32_t)el,(uint32_t)er);
  last=AR_Nav_Step(n,t,none,us,fresh);
}
static void run(AR_Nav*n,int ms,int us){ for(int i=0;i<ms;i+=5) tick(n,us,(i%60)==0); }
static int in(AR_Nav*n,const char*s){ return strcmp(AR_Nav_PhaseText(n),s)==0; }
/* wait for a step to finish; the US reads "stone close" once the mean travelled distance passes trig */
static double leg(AR_Nav*n,const char*name,double trig_edges){
  double s0=(el+er)/2, drift=0; int i=0;
  for(;i<60000 && in(n,name);i+=5){ double m=(el+er)/2-s0; tick(n,m>=trig_edges?50:900,(i%60)==0); }
  drift=(el-er); return drift;
}
#include <math.h>
#define COAST ((AR_COAST_CM+AR_REVERSE_TRIM_CM)*AR_EDGES_PER_CM)
#define NEARB(b,w) ((b)>=(w)-COAST-2 && (b)<=(w)+8)   /* legs end COAST early, never more than a few edges late */
/* Straight 158 cm leg with one motor weaker/stronger: how far does the AR end up sideways? (yaw from the wheel counts) */
static double lateral_cm(double rmul_, double *worst_yaw_deg){
  AR_Nav n; AR_Nav_Init(&n); t=1000; el=er=0; last=(AR_Cmd){0,0}; rmul=rmul_;
  AR_Nav_SetKey(&n,AR_KEY_MID); AR_Nav_Start(&n,t); run(&n,AR_COUNTDOWN_MS+50,900);
  double lat=0, worst=0, track_edges=AR_TRACK_MM/10.0*AR_EDGES_PER_CM, s0=(el+er)/2;
  for(int i=0;i<40000 && in(&n,"MID TO KEY");i+=5){
    double e0=el, f0=er; double m=(el+er)/2-s0;
    tick(&n,m>=158*AR_EDGES_PER_CM?50:900,(i%60)==0);
    double yaw=(el-er)/track_edges;                 /* + = turned right (left wheel ahead) */
    double ds=((el-e0)+(er-f0))/2/AR_EDGES_PER_CM;  /* cm this tick */
    lat+=ds*sin(yaw); if(fabs(yaw)>worst) worst=fabs(yaw);
  }
  *worst_yaw_deg=worst*180/3.14159265; return lat;
}
int main(void){
  { double w; const double r[]={0.6,0.75,0.9,1.1,1.25,1.5};
    for(int k=0;k<6;k++){ double lat=lateral_cm(r[k],&w); printf("  wheel ratio %.2f: sideways %.1f cm, worst heading %.1f deg\n",r[k],lat,w);
      CHECK(fabs(lat)<4.0); CHECK(w<6.0);} rmul=0.97; }
  AR_Nav n; AR_Nav_Init(&n); t=1000; el=er=0; last=(AR_Cmd){0,0};
  AR_Nav_SetKey(&n,AR_KEY_ALL); CHECK(!AR_Nav_NeedsIR(&n));
  AR_Nav_Start(&n,t); run(&n,AR_COUNTDOWN_MS+50,900); CHECK(in(&n,"MID TO KEY"));
  /* 158 cm to the stone, right wheel 3% slow: balance keeps the wheel counts close */
  double d=leg(&n,"MID TO KEY",158*AR_EDGES_PER_CM); CHECK(in(&n,"MID GRAB")); CHECK(d>-30&&d<30);
  run(&n,AR_GRAB_PAUSE_MS+AR_STEP_SETTLE_MS+30,50); CHECK(in(&n,"MID REVERSE"));
  double m0=(el+er)/2; run(&n,AR_STEP_SETTLE_MS+20,-1); CHECK(last.left<0&&last.right<0);
  leg(&n,"MID REVERSE",1e9); double back=(el+er)/2-m0, want=AR_MID_BACK_CM*AR_EDGES_PER_CM;
  CHECK(NEARB(back,want)); CHECK(in(&n,"TURN NORTH"));
  /* 90 degrees: each wheel travels pi*track/4 */
  m0=(el+er)/2; run(&n,AR_STEP_SETTLE_MS+10,-1); CHECK(last.left>0&&last.right<0);
  { int vmax=0, vmin=100, v0=-1; for(int i=0;i<4000 && in(&n,"TURN NORTH");i+=5){ tick(&n,-1,0); int a=last.left<0?-last.left:last.left; if(v0<0)v0=a; if(a>vmax)vmax=a; if(a&&a<vmin)vmin=a; }
    CHECK(v0<AR_PIVOT_PCT-5);                       /* soft start: begins gently */
    CHECK(vmax>v0);                                 /* speeds up from the gentle start */
    CHECK(vmin<=AR_PIVOT_START_PCT+8); }            /* starts low (the slow last stretch is checked in the TRN4 run) */
  double piv=(el+er)/2-m0, pw=3.14159*AR_TRACK_MM/10/4*AR_EDGES_PER_CM;
  CHECK(piv>=pw-2&&piv<pw+20); CHECK(in(&n,"TOP TO KEY"));
  run(&n,AR_STEP_SETTLE_MS+10,900); leg(&n,"TOP TO KEY",84*AR_EDGES_PER_CM); CHECK(in(&n,"TOP GRAB"));
  run(&n,AR_GRAB_PAUSE_MS+AR_STEP_SETTLE_MS+30,50); CHECK(in(&n,"TOP REVERSE"));
  leg(&n,"TOP REVERSE",1e9); CHECK(in(&n,"TURN DIAGONAL"));
  leg(&n,"TURN DIAGONAL",1e9); CHECK(in(&n,"DIAGONAL")); leg(&n,"DIAGONAL",1e9); CHECK(in(&n,"TURN UP-RIGHT"));
  run(&n,AR_STEP_SETTLE_MS+10,-1); CHECK(last.left<0&&last.right>0);                    /* pivots left */
  leg(&n,"TURN UP-RIGHT",1e9); CHECK(in(&n,"UP-RIGHT")); leg(&n,"UP-RIGHT",1e9); CHECK(in(&n,"TURN EAST"));
  leg(&n,"TURN EAST",1e9); CHECK(in(&n,"RIGHT TO KEY")); run(&n,AR_STEP_SETTLE_MS+10,900);
  double r0=(el+er)/2; leg(&n,"RIGHT TO KEY",24*AR_EDGES_PER_CM); double approach=(el+er)/2-r0; CHECK(in(&n,"RIGHT GRAB"));
  /* mirror home: reverse exactly the approach, unturn, back the up-right segment, unturn, back the diagonal, unturn, then the TOP return */
  run(&n,AR_GRAB_PAUSE_MS+AR_STEP_SETTLE_MS+30,50); CHECK(in(&n,"RIGHT BACK"));
  r0=(el+er)/2; run(&n,AR_STEP_SETTLE_MS+5,-1); CHECK(last.left<0&&last.right<0); leg(&n,"RIGHT BACK",1e9);
  { double back=(el+er)/2-r0; CHECK(NEARB(back,(double)n.leg_edges)); CHECK(fabs(n.leg_edges-approach)<12); }
  CHECK(in(&n,"UNTURN EAST")); run(&n,AR_STEP_SETTLE_MS+5,-1); CHECK(last.left<0&&last.right>0); leg(&n,"UNTURN EAST",1e9);   /* left */
  CHECK(in(&n,"BACK UP-RIGHT")); leg(&n,"BACK UP-RIGHT",1e9);
  CHECK(in(&n,"UNTURN")); run(&n,AR_STEP_SETTLE_MS+5,-1); CHECK(last.left>0&&last.right<0); leg(&n,"UNTURN",1e9);                /* right */
  CHECK(in(&n,"BACK DIAGONAL")); leg(&n,"BACK DIAGONAL",1e9);
  CHECK(in(&n,"UNTURN DIAG")); leg(&n,"UNTURN DIAG",1e9);
  CHECK(in(&n,"TOP BACK TO J")); leg(&n,"TOP BACK TO J",1e9); leg(&n,"TURN EAST",1e9); leg(&n,"TOP HOME",1e9);
  CHECK(n.phase==AR_PH_DONE); CHECK(in(&n,"DONE AT HOME")); CHECK(last.left==0&&last.right==0);
  /* partial-start modes begin in the right pose / step */
  { AR_Nav m; AR_Nav_Init(&m); t=1000; el=er=0; last=(AR_Cmd){0,0}; rmul=1.0; AR_Nav_SetKey(&m,AR_KEY_TOP1); AR_Nav_Start(&m,t);
    run(&m,AR_COUNTDOWN_MS+50,900); CHECK(in(&m,"TURN NORTH")); CHECK(!strcmp(AR_Nav_KeyText(&m),"TOP1"));
    leg(&m,"TURN NORTH",1e9); CHECK(in(&m,"TOP TO KEY")); run(&m,AR_STEP_SETTLE_MS+10,900);
    double a0=(el+er)/2; leg(&m,"TOP TO KEY",84*AR_EDGES_PER_CM); double approach=(el+er)/2-a0; CHECK(in(&m,"TOP GRAB")); run(&m,AR_GRAB_PAUSE_MS+AR_STEP_SETTLE_MS+30,50);
    CHECK(in(&m,"TOP BACK TO J"));                                      /* one straight reverse, no 13 cm stop in between */
    double m0=(el+er)/2; run(&m,AR_STEP_SETTLE_MS+5,-1); leg(&m,"TOP BACK TO J",1e9);
    double back=(el+er)/2-m0; CHECK(NEARB(back,approach));              /* exactly what the approach travelled */
    CHECK(in(&m,"TURN EAST")); run(&m,AR_STEP_SETTLE_MS+10,-1); CHECK(last.left>0&&last.right<0); leg(&m,"TURN EAST",1e9);
    CHECK(in(&m,"TOP HOME")); m0=(el+er)/2; run(&m,AR_STEP_SETTLE_MS+5,-1); CHECK(last.left>0&&last.right>0); leg(&m,"TOP HOME",1e9);
    back=(el+er)/2-m0; want=AR_J_TO_SZ_CM*AR_EDGES_PER_CM; CHECK(NEARB(back,want));
    CHECK(m.phase==AR_PH_DONE); CHECK(in(&m,"DONE AT HOME")); CHECK(last.left==0&&last.right==0); }
  { AR_Nav m; AR_Nav_Init(&m); t=1000; el=er=0; last=(AR_Cmd){0,0}; rmul=1.0; AR_Nav_SetKey(&m,AR_KEY_MID); AR_Nav_Start(&m,t);
    run(&m,AR_COUNTDOWN_MS+50,900); leg(&m,"MID TO KEY",100*AR_EDGES_PER_CM); run(&m,AR_GRAB_PAUSE_MS+AR_STEP_SETTLE_MS+30,50);
    leg(&m,"MID REVERSE",1e9); CHECK(in(&m,"MID HOME"));
    double m0=(el+er)/2; run(&m,AR_STEP_SETTLE_MS+5,-1); CHECK(last.left<0&&last.right<0);
    { int minspd=100; for(int i=0;i<60000 && in(&m,"MID HOME");i+=5){ tick(&m,-1,0); int a=last.left<0?-last.left:last.left; if(a&&a<minspd)minspd=a; } CHECK(minspd<=AR_SLOW_SPEED_PCT+2); } /* slowed for the last stretch */
    double back=(el+er)/2-m0, want=AR_J_TO_SZ_CM*AR_EDGES_PER_CM; CHECK(NEARB(back,want));
    CHECK(m.phase==AR_PH_DONE); CHECK(in(&m,"DONE AT HOME")); }
  { AR_Nav m; AR_Nav_Init(&m); t=1000; el=er=0; last=(AR_Cmd){0,0}; AR_Nav_SetKey(&m,AR_KEY_RGT1); AR_Nav_Start(&m,t);
    run(&m,AR_COUNTDOWN_MS+50,900); CHECK(in(&m,"TURN DIAGONAL"));
    leg(&m,"TURN DIAGONAL",1e9); leg(&m,"DIAGONAL",1e9); leg(&m,"TURN UP-RIGHT",1e9); leg(&m,"UP-RIGHT",1e9); leg(&m,"TURN EAST",1e9);
    CHECK(in(&m,"RIGHT TO KEY")); run(&m,AR_STEP_SETTLE_MS+10,900); leg(&m,"RIGHT TO KEY",24*AR_EDGES_PER_CM);
    CHECK(in(&m,"RIGHT GRAB")); }
  { AR_Nav m; AR_Nav_Init(&m); t=1000; el=er=0; last=(AR_Cmd){0,0}; rmul=1.0; AR_Nav_SetKey(&m,AR_KEY_TRN4); AR_Nav_Start(&m,t);
    run(&m,AR_COUNTDOWN_MS+50,900); CHECK(in(&m,"TURN 1 OF 4")); CHECK(!strcmp(AR_Nav_KeyText(&m),"TRN4"));
    double m0=(el+er)/2; leg(&m,"TURN 1 OF 4",1e9); leg(&m,"TURN 2 OF 4",1e9); leg(&m,"TURN 3 OF 4",1e9); CHECK(in(&m,"TURN 4 OF 4")); leg(&m,"TURN 4 OF 4",1e9);
    double each=((el+er)/2-m0)/4, pw=3.14159*AR_TRACK_MM/10/4*AR_EDGES_PER_CM; CHECK(each>=pw-2&&each<pw+10);
    CHECK(m.phase==AR_PH_DONE); CHECK(in(&m,"ROUTE PART DONE")); CHECK(last.left==0&&last.right==0); }
  /* pivot with one wheel 15 % weaker: the wheels must still travel (nearly) equal distances, so the AR turns on its own centre */
  { AR_Nav m; AR_Nav_Init(&m); t=1000; el=er=0; last=(AR_Cmd){0,0}; rmul=0.85; AR_Nav_SetKey(&m,AR_KEY_TRN4); AR_Nav_Start(&m,t);
    run(&m,AR_COUNTDOWN_MS+50,900); CHECK(in(&m,"TURN 1 OF 4")); run(&m,AR_STEP_SETTLE_MS+5,-1);
    double l0=el, r0=er; leg(&m,"TURN 1 OF 4",1e9); double dl=el-l0, dr=er-r0; CHECK(fabs(dl-dr)<=8);
    rmul=0.97; }
  /* the AR pushes against the stone and the US never reads close (no echo): the stalled wheels count as arrived */
  { AR_Nav m; AR_Nav_Init(&m); t=1000; el=er=0; last=(AR_Cmd){0,0}; rmul=1.0; stalled=0; AR_Nav_SetKey(&m,AR_KEY_TOP1); AR_Nav_Start(&m,t);
    run(&m,AR_COUNTDOWN_MS+50,900); leg(&m,"TURN NORTH",1e9); CHECK(in(&m,"TOP TO KEY")); run(&m,AR_STEP_SETTLE_MS+10,900);
    double s0=(el+er)/2; for(int i=0;i<4000 && (el+er)/2-s0<40*AR_EDGES_PER_CM;i+=5) tick(&m,-1,0);   /* drive 40 cm, no echo */
    CHECK(in(&m,"TOP TO KEY")); stalled=1;
    int ms=0; for(;ms<3000 && in(&m,"TOP TO KEY");ms+=5) tick(&m,-1,0);
    CHECK(in(&m,"TOP GRAB")); CHECK(ms>=(int)AR_STALL_MS-20 && ms<=(int)AR_STALL_MS+200); stalled=0; }
  /* the AR keeps rolling for 200 ms after each stop: the next turn waits until the wheels are still */
  { AR_Nav m; AR_Nav_Init(&m); t=1000; el=er=0; last=(AR_Cmd){0,0}; rmul=1.0; coast_cfg=40; prev_moving=0; coast_left=0;
    AR_Nav_SetKey(&m,AR_KEY_TRN4); AR_Nav_Start(&m,t); run(&m,AR_COUNTDOWN_MS+50,900);
    leg(&m,"TURN 1 OF 4",1e9); CHECK(in(&m,"TURN 2 OF 4"));
    int ms=0; for(;ms<2000 && last.left==0 && last.right==0;ms+=5) tick(&m,-1,0);
    CHECK(ms>=200+(int)AR_STILL_MS-20); CHECK(ms<=(int)AR_STEP_SETTLE_MAX_MS+20);
    CHECK(m.coast_max>=38); CHECK(m.settle_max_ms>=200);
    coast_cfg=0; }
  /* no roll: the settle stays at the minimum */
  { AR_Nav m; AR_Nav_Init(&m); t=1000; el=er=0; last=(AR_Cmd){0,0}; coast_cfg=0; prev_moving=0; coast_left=0;
    AR_Nav_SetKey(&m,AR_KEY_TRN4); AR_Nav_Start(&m,t); run(&m,AR_COUNTDOWN_MS+50,900);
    leg(&m,"TURN 1 OF 4",1e9); int ms=0; for(;ms<2000 && last.left==0 && last.right==0;ms+=5) tick(&m,-1,0);
    CHECK(ms>=(int)AR_STEP_SETTLE_MS-10 && ms<=(int)AR_STEP_SETTLE_MS+20); }
  /* a step that never completes faults */
  { AR_Nav m; AR_Nav_Init(&m); t=1000; el=er=0; last=(AR_Cmd){0,0}; AR_Nav_SetKey(&m,AR_KEY_MID); AR_Nav_Start(&m,t);
    run(&m,AR_COUNTDOWN_MS+AR_STEP_MAX_MS+300,900); CHECK(m.phase==AR_PH_FAULT); CHECK(last.left==0&&last.right==0); }
  if(fails){printf("AR ROUTE ENCODER FAILED (%d)\n",fails);return 1;}
  printf("AR ROUTE ENCODER PASS\n"); return 0;
}
