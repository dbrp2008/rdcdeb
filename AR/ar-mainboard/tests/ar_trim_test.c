/* Host test of the drift trim (compile with -DAR_DRIFT_TRIM_PERMILLE=20) and the STR straight-line test mode. */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "ar_nav.h"
static int fails;
#define CHECK(c) do{ if(!(c)){printf("FAIL line %d: %s\n",__LINE__,#c);fails++;} }while(0)
static uint32_t t; static double el, er; static AR_Cmd last; static AR_Line none={0,0,0};
static void tick(AR_Nav*n){
  t+=5; double a=last.left<0?-last.left:last.left, b=last.right<0?-last.right:last.right;
  el+=a*0.05*5; er+=b*0.05*5;
  AR_Nav_SetWheelOdometer(n,(uint32_t)el,(uint32_t)er);
  last=AR_Nav_Step(n,t,none,-1,0);
}
static int in(AR_Nav*n,const char*s){ return strcmp(AR_Nav_PhaseText(n),s)==0; }
int main(void){
  AR_Nav n; AR_Nav_Init(&n); t=1000; el=er=0; last=(AR_Cmd){0,0};
  AR_Nav_SetKey(&n,AR_KEY_STR); CHECK(!strcmp(AR_Nav_KeyText(&n),"STR")); AR_Nav_Start(&n,t);
  for(int i=0;i<(int)AR_COUNTDOWN_MS+50;i+=5) tick(&n);
  CHECK(in(&n,"STRAIGHT TEST"));
  double l0=el, r0=er; int first=-1, vmax=0;
  for(int i=0;i<60000 && in(&n,"STRAIGHT TEST");i+=5){ tick(&n); int a=last.left<0?-last.left:last.left; if(a&&first<0) first=a; if(a>vmax) vmax=a; }
  CHECK(first>0 && first<=AR_STRAIGHT_START_PCT+(int)AR_BALANCE_MAX_PCT/3+2);   /* sets off gently */
  CHECK(vmax>=AR_BASE_PCT-3);                                                   /* then reaches the leg speed */
  double dl=el-l0, dr=er-r0, mean=(dl+dr)/2;
  CHECK(n.phase==AR_PH_DONE); CHECK(in(&n,"ROUTE PART DONE"));
  CHECK(fabs(mean-AR_STR_TEST_CM*AR_EDGES_PER_CM)<20);                  /* drove the test distance */
  double want=1.0+AR_DRIFT_TRIM_PERMILLE/1000.0;                          /* right edges = left edges x (1 + trim) */
  CHECK(fabs(dr/dl-want)<0.006);
  printf("  STR: left %.0f right %.0f edges, ratio %.4f (trim %d permille)\n",dl,dr,dr/dl,AR_DRIFT_TRIM_PERMILLE);
  /* F20: 20 cm forward, then stop */
  AR_Nav_SetKey(&n,AR_KEY_F20); CHECK(!strcmp(AR_Nav_KeyText(&n),"F20")); AR_Nav_Start(&n,t);
  for(int i=0;i<(int)AR_COUNTDOWN_MS+50;i+=5) tick(&n);
  CHECK(in(&n,"FORWARD 20 CM"));
  l0=el; r0=er;
  for(int i=0;i<60000 && n.phase!=AR_PH_DONE && n.phase!=AR_PH_FAULT;i+=5) tick(&n);
  for(int i=0;i<500;i+=5) tick(&n); /* stopped: nothing more moves */
  mean=((el-l0)+(er-r0))/2;
  CHECK(n.phase==AR_PH_DONE);
  CHECK(fabs(mean-AR_F20_CM*AR_EDGES_PER_CM)<AR_EDGES_PER_CM*1.5);       /* 20 cm (the coast allowance ends it ~1 cm early here: no roll in this model) */
  printf("  F20: %.0f edges = %.1f cm\n",mean,mean/AR_EDGES_PER_CM);
  if(fails){printf("AR TRIM FAILED (%d)\n",fails);return 1;}
  printf("AR TRIM PASS\n"); return 0;
}
