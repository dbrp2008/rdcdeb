/* Host test of the straight right-key route (AR_RIGHT_STRAIGHT 1, encoder-only): 90 deg east then straight to the key, mirrored home. */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "ar_nav.h"
static int fails;
#define CHECK(c) do{ if(!(c)){printf("FAIL line %d: %s\n",__LINE__,#c);fails++;} }while(0)
static uint32_t t; static double el, er; static AR_Cmd last; static AR_Line none={0,0,0};
static void tick(AR_Nav*n,int us,int fresh){
  t+=5; double a=last.left<0?-last.left:last.left, b=last.right<0?-last.right:last.right;
  el+=a*0.05*5; er+=b*0.05*5*0.98;
  AR_Nav_SetWheelOdometer(n,(uint32_t)el,(uint32_t)er);
  last=AR_Nav_Step(n,t,none,us,fresh);
}
static void run(AR_Nav*n,int ms,int us){ for(int i=0;i<ms;i+=5) tick(n,us,(i%60)==0); }
static int in(AR_Nav*n,const char*s){ return strcmp(AR_Nav_PhaseText(n),s)==0; }
static double leg(AR_Nav*n,const char*name,double trig){
  double s0=(el+er)/2; for(int i=0;i<60000 && in(n,name);i+=5){ double m=(el+er)/2-s0; tick(n,trig>=1e8?-1:(m>=trig?50:900),(i%60)==0); } return (el+er)/2-s0; }  /* no trigger: the US sees nothing (no echo) */
int main(void){
  AR_Nav n; AR_Nav_Init(&n); t=1000; el=er=0; last=(AR_Cmd){0,0};
  AR_Nav_SetKey(&n,AR_KEY_RGT1); AR_Nav_Start(&n,t); run(&n,AR_COUNTDOWN_MS+50,900);
  CHECK(in(&n,"TURN DIAGONAL"));                                       /* first pivot: now a plain 90 deg right turn */
  double m0=(el+er)/2; run(&n,AR_STEP_SETTLE_MS+10,900); CHECK(last.left>0&&last.right<0); leg(&n,"TURN DIAGONAL",1e9);
  double piv=(el+er)/2-m0, pw=3.14159*AR_TRACK_MM/10/4*AR_EDGES_PER_CM; CHECK(piv>=pw-2 && piv<pw+14);
  CHECK(in(&n,"RIGHT TO KEY"));                                        /* straight to the key: no zig-zag steps */
  run(&n,AR_STEP_SETTLE_MS+10,900); double approach=leg(&n,"RIGHT TO KEY",100*AR_EDGES_PER_CM);
  CHECK(approach>=100*AR_EDGES_PER_CM && approach<100*AR_EDGES_PER_CM+300); CHECK(in(&n,"RIGHT GRAB"));
  run(&n,AR_GRAB_PAUSE_MS+AR_STEP_SETTLE_MS+30,50); CHECK(in(&n,"RIGHT BACK"));
  m0=(el+er)/2; run(&n,AR_STEP_SETTLE_MS+5,-1); CHECK(last.left<0&&last.right<0); leg(&n,"RIGHT BACK",1e9);
  { double back=(el+er)/2-m0; CHECK(back<=approach+8 && back>=approach-(AR_COAST_CM+AR_REVERSE_TRIM_CM)*AR_EDGES_PER_CM-3); }
  CHECK(in(&n,"UNTURN EAST")); run(&n,AR_STEP_SETTLE_MS+5,-1); CHECK(last.left<0&&last.right>0); leg(&n,"UNTURN EAST",1e9);   /* left 90 */
  CHECK(in(&n,"TOP BACK TO J"));                                       /* straight to the TOP return: no unturn legs in between */
  leg(&n,"TOP BACK TO J",1e9); CHECK(in(&n,"TURN EAST")); leg(&n,"TURN EAST",1e9); CHECK(in(&n,"TOP HOME")); leg(&n,"TOP HOME",1e9);
  CHECK(n.phase==AR_PH_DONE); CHECK(in(&n,"DONE AT HOME")); CHECK(last.left==0&&last.right==0);
  /* ALL: the turning point after the top key comes from the US (distance to the top stone grows while reversing) */
  { AR_Nav m; AR_Nav_Init(&m); t=1000; el=er=0; last=(AR_Cmd){0,0};
    AR_Nav_SetKey(&m,AR_KEY_ALL); AR_Nav_Start(&m,t); run(&m,AR_COUNTDOWN_MS+50,900);
    leg(&m,"MID TO KEY",150*AR_EDGES_PER_CM); run(&m,AR_GRAB_PAUSE_MS+AR_STEP_SETTLE_MS+30,50);
    leg(&m,"MID REVERSE",1e9); leg(&m,"TURN NORTH",1e9); CHECK(in(&m,"TOP TO KEY"));
    leg(&m,"TOP TO KEY",84*AR_EDGES_PER_CM); CHECK(in(&m,"TOP GRAB")); run(&m,AR_GRAB_PAUSE_MS+AR_STEP_SETTLE_MS+30,50);
    CHECK(in(&m,"TOP REVERSE"));
    double s0=(el+er)/2; int sawslow=0;
    for(int i=0;i<20000 && in(&m,"TOP REVERSE");i+=5){
      double rev_mm=((el+er)/2-s0)/AR_EDGES_PER_CM*10.0;
      tick(&m,(int)(95+rev_mm),1);   /* stone was 95 mm away at the stop; a reading every tick (the simulated AR is ~6x faster than the real one) */
      int a=last.left<0?-last.left:last.left; if(a&&a<=AR_SLOW_SPEED_PCT+(int)AR_BALANCE_MAX_PCT) sawslow=1; }
    double rev_mm=((el+er)/2-s0)/AR_EDGES_PER_CM*10.0;
    CHECK(in(&m,"TURN DIAGONAL"));                                       /* then the 90 deg turn east */
    CHECK(rev_mm>=200-AR_TOP_TURN_LEAD_MM-95-5 && rev_mm<=200-95+60);    /* stopped about AR_TOP_TURN_MM from the stone */
    CHECK(m.top_back_edges>0); CHECK(sawslow);
    leg(&m,"TURN DIAGONAL",1e9); leg(&m,"RIGHT TO KEY",100*AR_EDGES_PER_CM); run(&m,AR_GRAB_PAUSE_MS+AR_STEP_SETTLE_MS+30,50);
    leg(&m,"RIGHT BACK",1e9); leg(&m,"UNTURN EAST",1e9); CHECK(in(&m,"TOP BACK TO J"));
    double j0=(el+er)/2; leg(&m,"TOP BACK TO J",1e9); double back=(el+er)/2-j0;
    double want=(double)m.top_edges-(double)m.top_back_edges;          /* the rest of the way down to J */
    CHECK(back<=want+8 && back>=want-(AR_COAST_CM+AR_REVERSE_TRIM_CM)*AR_EDGES_PER_CM-3);
    leg(&m,"TURN EAST",1e9); leg(&m,"TOP HOME",1e9); CHECK(m.phase==AR_PH_DONE); CHECK(in(&m,"DONE AT HOME")); }
  /* the US never gives a usable reading: the reverse stops at the encoder limit */
  { AR_Nav m; AR_Nav_Init(&m); t=1000; el=er=0; last=(AR_Cmd){0,0};
    AR_Nav_SetKey(&m,AR_KEY_ALL); AR_Nav_Start(&m,t); run(&m,AR_COUNTDOWN_MS+50,900);
    leg(&m,"MID TO KEY",150*AR_EDGES_PER_CM); run(&m,AR_GRAB_PAUSE_MS+AR_STEP_SETTLE_MS+30,50);
    leg(&m,"MID REVERSE",1e9); leg(&m,"TURN NORTH",1e9); leg(&m,"TOP TO KEY",84*AR_EDGES_PER_CM); run(&m,AR_GRAB_PAUSE_MS+AR_STEP_SETTLE_MS+30,50);
    CHECK(in(&m,"TOP REVERSE")); double s0=(el+er)/2;
    for(int i=0;i<20000 && in(&m,"TOP REVERSE");i+=5) tick(&m,-1,(i%40)==0);
    double rev=(el+er)/2-s0; CHECK(rev>=AR_TOP_TURN_MAX_CM*AR_EDGES_PER_CM-2 && rev<=AR_TOP_TURN_MAX_CM*AR_EDGES_PER_CM+15); }
  /* TOP2: middle key and top key in one run, then home (no 13 cm stop, straight back to J, east into the zone) */
  { AR_Nav m; AR_Nav_Init(&m); t=1000; el=er=0; last=(AR_Cmd){0,0};
    AR_Nav_SetKey(&m,AR_KEY_TOP2); CHECK(!strcmp(AR_Nav_KeyText(&m),"TOP2")); AR_Nav_Start(&m,t); run(&m,AR_COUNTDOWN_MS+50,900);
    CHECK(in(&m,"MID TO KEY")); leg(&m,"MID TO KEY",150*AR_EDGES_PER_CM); CHECK(in(&m,"MID GRAB")); run(&m,AR_GRAB_PAUSE_MS+AR_STEP_SETTLE_MS+30,50);
    CHECK(in(&m,"MID REVERSE")); leg(&m,"MID REVERSE",1e9);
    CHECK(in(&m,"TURN NORTH")); leg(&m,"TURN NORTH",1e9);
    CHECK(in(&m,"TOP TO KEY")); leg(&m,"TOP TO KEY",84*AR_EDGES_PER_CM); CHECK(in(&m,"TOP GRAB")); run(&m,AR_GRAB_PAUSE_MS+AR_STEP_SETTLE_MS+30,50);
    CHECK(in(&m,"TOP BACK TO J")); leg(&m,"TOP BACK TO J",1e9);
    CHECK(in(&m,"TURN EAST")); leg(&m,"TURN EAST",1e9);
    CHECK(in(&m,"TOP HOME")); leg(&m,"TOP HOME",1e9);
    CHECK(m.phase==AR_PH_DONE); CHECK(in(&m,"DONE AT HOME")); CHECK(last.left==0&&last.right==0); }
  /* TOP from the start zone: west to J, north to the top key, straight back to J, east home */
  { AR_Nav m; AR_Nav_Init(&m); t=1000; el=er=0; last=(AR_Cmd){0,0};
    AR_Nav_SetKey(&m,AR_KEY_TOP); CHECK(!strcmp(AR_Nav_KeyText(&m),"TOP")); AR_Nav_Start(&m,t); run(&m,AR_COUNTDOWN_MS+50,900);
    CHECK(in(&m,"TO J")); double j=leg(&m,"TO J",1e9); CHECK(fabs(j-AR_SZ_TO_J_CM*AR_EDGES_PER_CM)<12);   /* 900 = nothing seen: encoder distance */
    CHECK(in(&m,"TURN NORTH")); run(&m,AR_STEP_SETTLE_MS+10,900); CHECK(last.left>0&&last.right<0); leg(&m,"TURN NORTH",1e9);
    CHECK(in(&m,"TOP TO KEY")); double a=leg(&m,"TOP TO KEY",84*AR_EDGES_PER_CM); CHECK(in(&m,"TOP GRAB")); run(&m,AR_GRAB_PAUSE_MS+AR_STEP_SETTLE_MS+30,50);
    CHECK(in(&m,"TOP BACK TO J")); double b=leg(&m,"TOP BACK TO J",1e9); CHECK(b<=a+8 && b>=a-20);
    CHECK(in(&m,"TURN EAST")); leg(&m,"TURN EAST",1e9); CHECK(in(&m,"TOP HOME")); leg(&m,"TOP HOME",1e9);
    CHECK(m.phase==AR_PH_DONE); CHECK(in(&m,"DONE AT HOME")); }
  /* RGT from the start zone: west to J, north until the US sees the top stone at AR_TOP_TURN_MM, east to the right key, home */
  { AR_Nav m; AR_Nav_Init(&m); t=1000; el=er=0; last=(AR_Cmd){0,0};
    AR_Nav_SetKey(&m,AR_KEY_RGT); CHECK(!strcmp(AR_Nav_KeyText(&m),"RGT")); AR_Nav_Start(&m,t); run(&m,AR_COUNTDOWN_MS+50,900);
    leg(&m,"TO J",1e9); CHECK(in(&m,"TURN NORTH")); leg(&m,"TURN NORTH",1e9);
    CHECK(in(&m,"TO TURN POINT")); run(&m,AR_STEP_SETTLE_MS+10,900);
    double s0=(el+er)/2;
    for(int i=0;i<20000 && in(&m,"TO TURN POINT");i+=5){
      double gone_mm=((el+er)/2-s0)/AR_EDGES_PER_CM*10.0; tick(&m,(int)(700-gone_mm),1); }   /* top stone 700 mm ahead at the start */
    double gone_mm=((el+er)/2-s0)/AR_EDGES_PER_CM*10.0;
    CHECK(in(&m,"TURN DIAGONAL")); CHECK(gone_mm>=700-200-AR_TOP_TURN_LEAD_MM-20 && gone_mm<=700-200+40);   /* stopped ~200 mm from the stone (sim AR ~6x fast: +30 mm roll) */
    leg(&m,"TURN DIAGONAL",1e9); CHECK(in(&m,"RIGHT TO KEY")); leg(&m,"RIGHT TO KEY",100*AR_EDGES_PER_CM); run(&m,AR_GRAB_PAUSE_MS+AR_STEP_SETTLE_MS+30,50);
    leg(&m,"RIGHT BACK",1e9); leg(&m,"UNTURN EAST",1e9); CHECK(in(&m,"TOP BACK TO J"));
    double back=leg(&m,"TOP BACK TO J",1e9); CHECK(back<=(double)m.top_edges+8 && back>=(double)m.top_edges-20);   /* back down exactly the J -> turn point leg */
    leg(&m,"TURN EAST",1e9); leg(&m,"TOP HOME",1e9); CHECK(m.phase==AR_PH_DONE); CHECK(in(&m,"DONE AT HOME")); }
  /* RGT with no top stone in view: the north leg stops at the encoder limit */
  { AR_Nav m; AR_Nav_Init(&m); t=1000; el=er=0; last=(AR_Cmd){0,0};
    AR_Nav_SetKey(&m,AR_KEY_RGT); AR_Nav_Start(&m,t); run(&m,AR_COUNTDOWN_MS+50,900);
    leg(&m,"TO J",1e9); leg(&m,"TURN NORTH",1e9); double g=leg(&m,"TO TURN POINT",1e9);
    CHECK(g>=AR_J_TO_TURN_MAX_CM*AR_EDGES_PER_CM-2 && g<=AR_J_TO_TURN_MAX_CM*AR_EDGES_PER_CM+40); CHECK(in(&m,"TURN DIAGONAL")); }
  /* J by the US: driving west from the start zone, the middle stone is 1500 mm ahead; stop when the US reads AR_J_US_MM */
  { AR_Nav m; AR_Nav_Init(&m); t=1000; el=er=0; last=(AR_Cmd){0,0};
    AR_Nav_SetKey(&m,AR_KEY_TOP); AR_Nav_Start(&m,t); run(&m,AR_COUNTDOWN_MS+50,-1); CHECK(in(&m,"TO J"));
    double s0=(el+er)/2; for(int i=0;i<20000 && in(&m,"TO J");i+=5){ double g=((el+er)/2-s0)/AR_EDGES_PER_CM*10.0; tick(&m,(int)(1500-g),1); }
    double g=((el+er)/2-s0)/AR_EDGES_PER_CM*10.0; CHECK(in(&m,"TURN NORTH"));
    CHECK(g>=1500-AR_J_US_MM-AR_TOP_TURN_LEAD_MM-10 && g<=1500-AR_J_US_MM+40); }          /* stopped ~AR_J_US_MM from the stone */
  /* the US says J far outside the encoder window: the hard limit stops it */
  { AR_Nav m; AR_Nav_Init(&m); t=1000; el=er=0; last=(AR_Cmd){0,0};
    AR_Nav_SetKey(&m,AR_KEY_TOP); AR_Nav_Start(&m,t); run(&m,AR_COUNTDOWN_MS+50,-1);
    double s0=(el+er)/2; for(int i=0;i<20000 && in(&m,"TO J");i+=5){ double g=((el+er)/2-s0)/AR_EDGES_PER_CM*10.0; tick(&m,(int)(2000-g),1); }   /* seen from 500 mm on (under 1.5 m), J by the US would be at 1305 mm */
    double g=((el+er)/2-s0)/AR_EDGES_PER_CM*10.0; CHECK(g>=(AR_SZ_TO_J_CM+AR_J_WINDOW_CM)*10-5 && g<=(AR_SZ_TO_J_CM+AR_J_WINDOW_CM)*10+40); }
  /* reversing from the middle key (TOP2): back until the US reads AR_J_US_MM from the middle stone */
  { AR_Nav m; AR_Nav_Init(&m); t=1000; el=er=0; last=(AR_Cmd){0,0};
    AR_Nav_SetKey(&m,AR_KEY_TOP2); AR_Nav_Start(&m,t); run(&m,AR_COUNTDOWN_MS+50,900);
    leg(&m,"MID TO KEY",150*AR_EDGES_PER_CM); run(&m,AR_GRAB_PAUSE_MS+AR_STEP_SETTLE_MS+30,50); CHECK(in(&m,"MID REVERSE"));
    double s0=(el+er)/2; for(int i=0;i<20000 && in(&m,"MID REVERSE");i+=5){ double r=((el+er)/2-s0)/AR_EDGES_PER_CM*10.0; tick(&m,(int)(95+r),1); }
    double r=((el+er)/2-s0)/AR_EDGES_PER_CM*10.0; CHECK(in(&m,"TURN NORTH"));
    CHECK(r>=AR_J_US_MM-95-AR_TOP_TURN_LEAD_MM-10 && r<=AR_J_US_MM-95+40); }               /* stone was 95 mm away at the grab */
  if(fails){printf("AR ROUTE STRAIGHT FAILED (%d)\n",fails);return 1;}
  printf("AR ROUTE STRAIGHT PASS\n"); return 0;
}
