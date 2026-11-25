#include <stdio.h>
#include <string.h>
#include "ar_nav.h"
static int fails; static uint32_t t;
#define CHECK(c) do{ if(!(c)){printf("FAIL line %d: %s\n",__LINE__,#c);fails++;} }while(0)
static AR_Line L(int l,int c,int r){AR_Line s={l,c,r};return s;}
static AR_Cmd step(AR_Nav*n,AR_Line s,int us,int fresh){ t+=5; return AR_Nav_Step(n,t,s,us,fresh); }
static void run(AR_Nav*n,AR_Line s,int ms,int us){ for(int i=0;i<ms;i+=5){ AR_Cmd c=step(n,s,us,(i%60)==0); (void)c; } }
int main(void){
  AR_Nav n; AR_Nav_Init(&n); t=1000;
  CHECK(n.phase==AR_PH_IDLE); AR_Cmd c=step(&n,L(0,1,0),-1,0); CHECK(c.left==0&&c.right==0);
  /* ===== Key A ===== */
  AR_Nav_SetKey(&n,AR_KEY_A); AR_Nav_Start(&n,t); CHECK(n.phase==AR_PH_COUNTDOWN);
  run(&n,L(0,0,0),1500,-1); CHECK(n.phase==AR_PH_COUNTDOWN);                 /* still waiting */
  run(&n,L(0,0,0),700,-1); CHECK(n.phase==AR_PH_OUT);                         /* now moving */
  c=step(&n,L(0,0,0),-1,0); CHECK(c.left>0&&c.right>0);                      /* creeps forward to find the line */
  for(int i=0;i<10;i++) run(&n,L(0,1,0),50,-1);                              /* on the line */
  c=step(&n,L(0,1,0),-1,0); CHECK(c.left==AR_BASE_PCT&&c.right==AR_BASE_PCT);
  /* curved branch on the right, centre still on the line: ignored on Key A */
  c=step(&n,L(0,1,1),-1,0); CHECK(c.left==AR_BASE_PCT&&c.right==AR_BASE_PCT);
  /* drifts right of the line: still steers back */
  c=step(&n,L(0,0,1),-1,0); CHECK(c.right==0&&c.left>0);
  /* the CROSS */
  run(&n,L(1,1,1),10,-1); CHECK(n.sub==AR_SUB_FOLLOW);                       /* too short */
  run(&n,L(0,1,0),30,-1);
  run(&n,L(1,1,1),30,-1); CHECK(n.sub==AR_SUB_ADVANCE);
  c=step(&n,L(1,1,1),-1,0); CHECK(c.left>0&&c.right>0);                      /* drives on over the junction */
  run(&n,L(0,1,0),AR_CROSS_ADVANCE_MS+20,-1); CHECK(n.sub==AR_SUB_PIVOT);
  c=step(&n,L(0,1,0),-1,0); CHECK(c.left>0&&c.right<0);                      /* pivots RIGHT */
  run(&n,L(0,1,0),AR_PIVOT_MIN_MS+50,-1); CHECK(n.sub==AR_SUB_PIVOT);          /* centre still on, but never went off: keep turning */
  run(&n,L(0,0,0),100,-1); CHECK(n.sub==AR_SUB_PIVOT);
  run(&n,L(0,1,0),20,-1); CHECK(n.sub==AR_SUB_FOLLOW&&n.leg==1);             /* found the vertical line */
  run(&n,L(0,1,0),500,-1);
  /* branch (diagonal) on the right of the vertical line: ignored */
  c=step(&n,L(0,1,1),-1,0); CHECK(c.left==AR_BASE_PCT&&c.right==AR_BASE_PCT);
  /* approaches the key: ultrasonic */
  run(&n,L(0,1,0),300,500); CHECK(n.phase==AR_PH_OUT);
  run(&n,L(0,1,0),200,AR_STOP_MM-20); CHECK(n.phase==AR_PH_GRAB);
  c=step(&n,L(0,1,0),-1,0); CHECK(c.left==0&&c.right==0);
  run(&n,L(0,1,0),AR_GRAB_PAUSE_MS+20,-1); CHECK(n.phase==AR_PH_TURNAROUND);
  c=step(&n,L(0,1,0),-1,0); CHECK(c.left<0&&c.right>0);                      /* turns LEFT (AR_TURNAROUND_DIR = -1) */
  run(&n,L(0,0,0),300,-1); run(&n,L(0,1,0),20,-1); CHECK(n.phase==AR_PH_RETURN&&n.leg==0);
  run(&n,L(0,1,0),400,-1);
  c=step(&n,L(1,1,0),-1,0); CHECK(c.left==AR_BASE_PCT&&c.right==AR_BASE_PCT); /* diagonal joins on the left: ignored */
  run(&n,L(1,1,1),30,-1); CHECK(n.sub==AR_SUB_ADVANCE);
  run(&n,L(0,1,0),AR_CROSS_ADVANCE_MS+20,-1); c=step(&n,L(0,1,0),-1,0); CHECK(c.left<0&&c.right>0); /* pivots LEFT */
  run(&n,L(0,0,0),350,-1); run(&n,L(0,1,0),20,-1); CHECK(n.sub==AR_SUB_FOLLOW&&n.leg==1);
  run(&n,L(0,1,0),500,-1);
  run(&n,L(0,0,0),AR_HOME_LOST_MS-100,-1); CHECK(n.phase==AR_PH_RETURN);
  run(&n,L(0,0,0),200,-1); CHECK(n.phase==AR_PH_HOME_ADVANCE);
  c=step(&n,L(0,0,0),-1,0); CHECK(c.left>0&&c.right>0);
  run(&n,L(0,0,0),AR_HOME_ADVANCE_MS+20,-1); CHECK(n.phase==AR_PH_DONE);
  c=step(&n,L(0,1,0),-1,0); CHECK(c.left==0&&c.right==0);
  printf("  Key A run: ends in %s\n",AR_Nav_PhaseText(&n));
  /* ===== switch to Key B with the button, run again ===== */
  AR_Nav_SetKey(&n,AR_KEY_B); CHECK(n.key==AR_KEY_B);
  AR_Nav_Start(&n,t); run(&n,L(0,0,0),AR_COUNTDOWN_MS+50,-1); CHECK(n.phase==AR_PH_OUT);
  run(&n,L(0,1,0),500,-1);
  c=step(&n,L(0,1,1),-1,0); CHECK(c.left==AR_BASE_PCT&&c.right==0);          /* right-hand branch: takes it */
  run(&n,L(0,1,0),300,-1);
  /* corner to the left: line seen only on the left, then lost: swings left */
  c=step(&n,L(1,0,0),-1,0); CHECK(c.left==0&&c.right==AR_BASE_PCT);
  c=step(&n,L(0,0,0),-1,0); CHECK(c.left<0&&c.right>0);
  run(&n,L(0,1,0),300,-1);
  run(&n,L(0,1,0),200,AR_STOP_MM-10); CHECK(n.phase==AR_PH_GRAB);
  run(&n,L(0,1,0),AR_GRAB_PAUSE_MS+20,-1); run(&n,L(0,0,0),300,-1); run(&n,L(0,1,0),20,-1); CHECK(n.phase==AR_PH_RETURN);
  run(&n,L(0,1,0),400,-1); run(&n,L(0,0,0),AR_HOME_LOST_MS+50,-1); CHECK(n.phase==AR_PH_HOME_ADVANCE);
  run(&n,L(0,0,0),AR_HOME_ADVANCE_MS+20,-1); CHECK(n.phase==AR_PH_DONE);
  /* ===== faults ===== */
  AR_Nav_Start(&n,t); run(&n,L(0,0,0),AR_COUNTDOWN_MS+50,-1); run(&n,L(0,1,0),200,-1);
  run(&n,L(0,0,0),AR_LOST_FAULT_MS+3000,-1); CHECK(n.phase==AR_PH_FAULT); printf("  lost-line on Key B ends in %s\n",AR_Nav_PhaseText(&n));
  /* start-zone: never saw the line, nothing close: no false 'arrived' */
  AR_Nav_Init(&n); AR_Nav_Start(&n,t); run(&n,L(0,0,0),AR_COUNTDOWN_MS+50,-1); run(&n,L(0,0,0),AR_END_LOST_MS+200,AR_STOP_MM-10);
  CHECK(n.phase==AR_PH_OUT);
  run(&n,L(0,0,0),AR_LOST_FAULT_MS,-1); CHECK(n.phase==AR_PH_FAULT);
  /* abort */
  AR_Nav_Start(&n,t); AR_Nav_Abort(&n); CHECK(n.phase==AR_PH_IDLE);
  printf(fails?"%d FAILED\n":"AR NAV ALL PASS\n",fails); return fails!=0;
}
