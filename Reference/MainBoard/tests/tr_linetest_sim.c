/* Host simulation of the TR line-following test. Build: gcc -ICore/Inc -o tr_linetest_sim tests/tr_linetest_sim.c Core/Src/tr_linetest.c -lm
 * World (mm, y = left of the start heading): main line y=0 from x=0 to MAIN_END; optional branch x=800 going LEFT (y 0..1500).
 * Walls end both lines (x = MAIN_END + 100, y = 1600): the front ultrasonic (200 mm ahead of the centre) sees them.
 * The robot creeps sideways (DRIFT x forward travel) to force drift corrections. Velocity follows the command with the
 * real acceleration limit (600 rpm/s at 130 rpm full scale), so it cannot stop instantly. Build with -DTRL_PROBE_SPEED=0.35f to
 * stress the braking logic (all cases must still pass). */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "tr_linetest.h"
static int fails;
#define CHECK(c) do{ if(!(c)){printf("FAIL line %d: %s\n",__LINE__,#c);fails++;} }while(0)
static double px,py,hd,odo,sodo,vx,vy; static int branch, main_end; static uint32_t next_us; static double maxy; static int follow_on_black;
static double segd(double x,double y,double ax,double ay,double bx,double by){double dx=bx-ax,dy=by-ay;double t=((x-ax)*dx+(y-ay)*dy)/(dx*dx+dy*dy);t=t<0?0:(t>1?1:t);return hypot(x-(ax+t*dx),y-(ay+t*dy));}
static double spot = 0; /* IR spot radius: the IR sees black when the tape (10 mm half-width) is within 10 + spot mm */
static int black(double x,double y){ return segd(x,y,0,0,main_end,0)<10+spot || (branch>0 && segd(x,y,800,0,800,1500)<10+spot) || (branch<0 && segd(x,y,800,0,800,-1500)<10+spot); }  /* branch +1 left, -1 right */
static void run(int with_branch,int end_x,double drift,TRLine*a,int*state_turned){
  branch=with_branch; main_end=end_x; px=100;py=0;hd=0;odo=sodo=vx=vy=0; next_us=0; maxy=0; follow_on_black=0;
  TRLine_Init(a); uint32_t now=1000; TRLine_Start(a,now);
  int turn_state=0; uint32_t turn_end=0; double turn_target=0; *state_turned=0;
  for(int i=0;i<12000;i++){ now+=5;
    if(turn_state==1&&(int32_t)(now-turn_end)>=0){hd=turn_target;turn_state=2;}
    double r=hd*M_PI/180.0, fx=cos(r), fy=-sin(r), lx=-fy, ly=fx;   /* heading clockwise positive */
    TRAuto_In in; memset(&in,0,sizeof in);
    in.left=black(px+200*fx+25*lx,py+200*fy+25*ly); in.right=black(px+200*fx-25*lx,py+200*fy-25*ly);
    { in.us_fresh=(int32_t)(now-next_us)>=0; if(in.us_fresh) next_us=now+50;
      double sxp=px+200*fx, syp=py+200*fy; double d=-1;
      if(fx>0.9) d=(main_end+100)-sxp; else if(fy>0.9) d=1600-syp; else if(fy<-0.9) d=syp+1600;   /* east / north / south walls */
      in.us_mm=(d>0&&d<2000)?(int32_t)d:-1; }
    in.dist_mm=(float)odo; in.strafe_mm=(float)sodo; in.heading_deg=(float)hd; in.heading_sign=-1.0f; in.turn_state=(uint8_t)turn_state; in.now=now;
    TRL_State before=a->st; TRAuto_Out o; TRLine_Step(a,&in,&o);
    { static uint32_t check_at=0; if(before==TRL_PROBE&&a->st==TRL_FOLLOW) check_at=now+100;   /* a correction just ended: 100 ms later (slide settled) */
      if(check_at&&(int32_t)(now-check_at)>=0){ check_at=0; if(a->st==TRL_FOLLOW&&(in.left||in.right)) follow_on_black++; } }   /* ...the line must be between the IRs */
    if(fabs(py)>maxy) maxy=fabs(py);
#ifdef TRACE
    if(before!=a->st) printf("      t=%6.3f %-10s -> %-10s L%d R%d x=%6.1f y=%+6.1f hd=%+5.1f\n",(now-a->t0)/1000.0,TRLine_StateText(before),TRLine_StateText(a->st),in.left,in.right,px,py,hd);
#endif
#ifdef DBG
    if(before==TRL_PROBE&&a->st!=TRL_PROBE){ double r2=hd*M_PI/180.0; double off = (fabs(hd)<1)? py : (hd<0? -(px-800) : (px-800)); (void)r2;
      printf("    t=%5.2f end probe -> %s  offset %+5.1f mm  gap %4.1f  vy %+.3f\n",(now-a->t0)/1000.0,TRLine_StateText(a->st),off,a->gap_mm,vx); }
#endif
    if(o.zero_heading) hd=0;
    if(o.turn_req&&turn_state!=1){turn_state=1;turn_end=now+1500;turn_target=hd+o.turn_deg;*state_turned=1;}
    if(turn_state==2&&a->st!=TRL_TURN) turn_state=0;
    /* velocity with the acceleration limit: full scale 1037 mm/s, 600/130 of full scale per second */
    double cx=o.active&&turn_state!=1?o.x:0, cy=o.active&&turn_state!=1?o.y:0, step=600.0/130.0*0.005;
    vx+= fmax(-step,fmin(step,cx-vx)); vy+= fmax(-step,fmin(step,cy-vy));
    double sx=vx*1.037*5, sy=vy*1.037*5;                  /* mm this tick: sideways (+right), forward */
    double rx=-lx, ry=-ly;
    px+=sy*fx+sx*rx; py+=sy*fy+sx*ry; odo+=sy; sodo+=sx;
    px+=drift*fabs(sy)*lx; py+=drift*fabs(sy)*ly;         /* creeps to the LEFT while driving forward */
    if(!TRLine_IsRunning(a)) break;
  }
}
int main(void){
  TRLine a; int turned;
  /* 1: drift + a branch to the left: corrects the drift, turns left at the branch, follows it to its end */
  run(1,1500,0.03,&a,&turned);
  printf("  branch: %s, drifts %u, junctions %u, ended at (%.0f,%.0f) heading %.0f\n",TRLine_StateText(a.st),a.drifts,a.junctions,px,py,hd);
  CHECK(a.st==TRL_END); CHECK(a.drifts>=1); CHECK(a.junctions==1); CHECK(turned);
  CHECK(fabs(hd+90)<1); CHECK(fabs(px-800)<40); CHECK(py>1250);  /* on the branch, near its far end */
  /* 2: drift only, no branch: never mistakes the drift for a junction, stops at the end of the line */
  run(0,2000,0.03,&a,&turned);
  printf("  drift only: %s, drifts %u, junctions %u, ended at (%.0f,%.0f)\n",TRLine_StateText(a.st),a.drifts,a.junctions,px,py);
  CHECK(a.st==TRL_END); CHECK(a.drifts>=2); CHECK(a.junctions==0); CHECK(!turned); CHECK(px>1700);
  /* 3: drift the other way */
  run(0,2000,-0.03,&a,&turned);
  printf("  drift right: %s, drifts %u, junctions %u\n",TRLine_StateText(a.st),a.drifts,a.junctions);
  CHECK(a.st==TRL_END); CHECK(a.drifts>=2); CHECK(a.junctions==0);
  /* 4: branch to the RIGHT: turns right */
  run(-1,1500,0.03,&a,&turned);
  printf("  right branch: %s, drifts %u, junctions %u, ended at (%.0f,%.0f) heading %.0f\n",TRLine_StateText(a.st),a.drifts,a.junctions,px,py,hd);
  CHECK(a.junctions==1); CHECK(fabs(hd-90)<1); CHECK(fabs(px-800)<40); CHECK(py<-1250);
  /* 5: a stronger drift (6 %) is still corrected and never taken for a junction */
  run(0,2000,0.06,&a,&turned);
  printf("  6%% drift: %s, drifts %u, junctions %u, ended at (%.0f,%.0f)\n",TRLine_StateText(a.st),a.drifts,a.junctions,px,py);
  CHECK(a.st==TRL_END); CHECK(a.junctions==0); CHECK(fabs(py)<30);
  /* 6: realistic IR spot (4 mm) and only a 1 % drift on a 2 m line: the corrections must not bounce between the two IRs
   * (on the robot, 40 corrections on a short straight line = sliding side to side) */
  for(int sp=4; sp<=12; sp+=4){ spot=sp; run(0,2000,0.01,&a,&turned);
  printf("  spot %d mm, 1%% drift: %s, drifts %u, junctions %u, gap %.0f mm, ended at (%.0f,%.0f)\n",sp,TRLine_StateText(a.st),a.drifts,a.junctions,a.gap_mm,px,py);
  CHECK(a.st==TRL_END); CHECK(a.junctions==0); CHECK(a.drifts<=(sp<=8?6:10));   /* a 6 mm gap needs a correction every ~3 mm of creep */
  run(1,1500,0.03,&a,&turned);
  printf("  spot %d mm, branch: %s, drifts %u, junctions %u, heading %.0f\n",sp,TRLine_StateText(a.st),a.drifts,a.junctions,hd);
  CHECK(a.junctions==1); CHECK(fabs(hd+90)<1); if(sp<=8) CHECK(a.drifts<=8); }   /* a 12 mm spot leaves only a ~6 mm gap: 3 % drift reaches an IR every few cm */
  spot=0;
  /* faster slide (more momentum when it reverses) with a small gap: after every correction the line must end between
   * the IRs, never touching the other one (the slide back used to count the momentum the wrong way as progress) */
  spot=10; run(0,2000,0.03,&a,&turned);
  printf("  stress: %s, drifts %u, junctions %u, gap %.0f mm, max |y| %.1f\n",TRLine_StateText(a.st),a.drifts,a.junctions,a.gap_mm,maxy);
  CHECK(a.st==TRL_END); CHECK(a.junctions==0); CHECK(follow_on_black==0);
  spot=0;
  printf(fails?"%d FAILED\n":"TR LINE TEST SIM PASS\n",fails); return fails!=0;
}
