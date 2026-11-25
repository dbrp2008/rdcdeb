/* Host simulation of the autonomous routine. Build: gcc -ICore/Inc -o tr_auto_sim tests/tr_auto_sim.c Core/Src/tr_auto.c Core/Src/tr_linetest.c -lm Args: [us_noise_mm] [seed] [start zone: 0 white, 1 reads black, 2 flickers] */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
static int noise_mm = 0; /* +- ultrasonic noise */
#include "tr_auto.h"
#ifndef SENS_MM
#define SENS_MM TRA_ADVANCE_MM /* the real IR / ultrasonic distance ahead of the turning centre (the code assumes TRA_ADVANCE_MM) */
#endif
static int fails;
#define CHECK(c) do{ if(!(c)){printf("FAIL line %d: %s\n",__LINE__,#c);fails++;} }while(0)
/* ---- world (mm): bottom line y=0, vertical x=1250 (0..1500), short line y=800 (x 250..1250); blocks at (1250,800), (250,800) ---- */
static double last_b2dist=0; static double px,py,hd; static double odo, sodo; /* forward and sideways encoder distance */ static int arm_us; static double arm_f;
static int turn_state; static double vxr,vyr; static int turn1_x_set; static double turn1_start_x,turn1_end_x,max_tip_x;
static uint32_t turn_end; static double turn_target;
#define TIP_MM 250 /* front of the arm, ahead of the turning centre (estimate) */
static int b1_carried, b2_carried; static double b1x=1807,b1y=1236,b2x=726,b2y=1236;
static double segd(double x,double y,double ax,double ay,double bx,double by){double dx=bx-ax,dy=by-ay;double t=((x-ax)*dx+(y-ay)*dy)/(dx*dx+dy*dy);t=t<0?0:(t>1?1:t);double qx=ax+t*dx,qy=ay+t*dy;return hypot(x-qx,y-qy);}
static int box_mode=0; /* 0 none, 1 dark start zone (x<650), 2 borderline: flickers inside the zone */
static int black(double x,double y){if(x<650&&box_mode==1)return 1; if(x<650&&box_mode==2)return rand()%2; return segd(x,y,650,0,4000,0)<10||segd(x,y,1807,0,1807,2600)<10||segd(x,y,726,1236,1807,1236)<10;}
static void front(double*fx,double*fy){double r=hd*M_PI/180.0;*fx=cos(r);*fy=-sin(r);}
static int us_mm(void){double fx,fy;front(&fx,&fy);double sx=px+SENS_MM*fx,sy=py+SENS_MM*fy;int best=-1;
  double bx[2]={b1x,b2x},by[2]={b1y,b2y};int carried[2]={b1_carried,b2_carried};
  for(int i=0;i<2;i++){ if(carried[i])continue; double rx=bx[i]-sx,ry=by[i]-sy; double along=rx*fx+ry*fy, lat=fabs(-rx*fy+ry*fx); if(along>0&&lat<80){int d=(int)(along-70); if(best<0||d<best)best=d;} } return best;}
int main(int argc,char**argv){ if(argc>1){noise_mm=atoi(argv[1]); srand(argc>2?atoi(argv[2]):1);} if(argc>3) box_mode=atoi(argv[3]);
  TRAuto a; memset(&a,0,sizeof a); TRAuto_Init(&a); uint32_t now=1000; px=290;py=0;hd=0;odo=0;arm_us=2500;arm_f=2500;
  TRAuto_Start(&a,now);
  int state_seen[64]={0}; uint32_t next_us=0; int us_val=-1; int us_fresh=0; double max_lat=0;
  for(int i=0;i<6000;i++){ now+=5;
    /* turn simulation: takes 1.5 s */
    if(turn_state==1&&(int32_t)(now-turn_end)>=0){hd=turn_target;turn_state=2;}
    TRAuto_In in; memset(&in,0,sizeof in);
    double fx,fy; front(&fx,&fy); double lx=-fy,ly=fx;
    double lsx=px+SENS_MM*fx+25*lx,lsy=py+SENS_MM*fy+25*ly,rsx=px+SENS_MM*fx-25*lx,rsy=py+SENS_MM*fy-25*ly;
    in.left=black(lsx,lsy); in.right=black(rsx,rsy);
    if((int32_t)(now-next_us)>=0){next_us=now+50;us_val=us_mm(); if(us_val>=0&&noise_mm) us_val+= (rand()%(2*noise_mm+1))-noise_mm; us_fresh=1;}else us_fresh=0;
    in.us_mm=us_val; in.us_fresh=us_fresh; in.dist_mm=(float)odo; in.strafe_mm=(float)sodo; in.heading_deg=(float)hd; in.heading_sign=-1.0f; in.turn_state=turn_state;
    in.arm_us=(uint16_t)arm_us; in.arm_closed_us=1611; in.arm_open_us=2500; in.now=now;
    TRAuto_Out o; TRAuto_Step(&a,&in,&o);
    state_seen[a.st]=1;
    if(o.zero_heading) hd=0;
    if(o.turn_req&&turn_state!=1){turn_state=1;turn_end=now+1500;turn_target=hd+o.turn_deg;}
    else if(turn_state==2&&!o.turn_req) {}
    if(turn_state==2 && a.st!=TRA_TURN1&&a.st!=TRA_TURN2&&a.st!=TRA_TURN3) turn_state=0;
    /* arm */
    if(o.arm>0)arm_f-=600*0.005; else if(o.arm<0)arm_f+=600*0.005; if(arm_f<1611)arm_f=1611; if(arm_f>2500)arm_f=2500; arm_us=(int)arm_f;
    /* motion: speed ratio x 1037 mm/s (130 rpm, speed level 5) */
    /* motion with the drive's ramp: 600 rpm/s normally, 200 rpm/s once a turn runs (the drive stops commanding x/y then,
     * so a moving TR coasts while it turns: in the sim the heading jumps at the end of the turn, so it coasts along the old one) */
    { const double tx=(o.active&&turn_state!=1)?o.x:0, ty=(o.active&&turn_state!=1)?o.y:0;
      const double step=(turn_state==1?200.0:600.0)/130.0*0.005;
      vxr+=fmax(-step,fmin(step,tx-vxr)); vyr+=fmax(-step,fmin(step,ty-vyr));
      double v=0.798*1.3*5.0; double sx=vxr*v, sy=vyr*v;
      double rx=-lx,ry=-ly; /* right vector */
      px+= (sy*fx + sx*rx); py+= (sy*fy + sx*ry); odo += sy; sodo += sx; } /* forward and sideways travel */
    if(turn_state==1&&!turn1_x_set){turn1_x_set=1;turn1_start_x=px;}
    if(turn_state!=1&&turn1_x_set==1){turn1_x_set=2;turn1_end_x=px;}
    if(a.st<=TRA_TURN1){ double f=px+TIP_MM*fx; if(f>max_tip_x) max_tip_x=f; }
    /* block carried */
    if(a.st==TRA_CARRY&&!b1_carried){b1_carried=1;} if(a.st==TRA_DROP||a.st>=TRA_TURN2) { if(b1_carried==1){ b1_carried=2; b1x=px+(SENS_MM+140)*fx; b1y=py+(SENS_MM+140)*fy; /* the block stays ~140 mm ahead of the IR sensors (70 mm US face distance + half a block) */ } }
    if(a.st==TRA_GRAB2||a.st==TRA_DONE) b2_carried=1;
    if(a.st==TRA_DONE||a.st==TRA_FAULT||a.st==TRA_TIMEUP) { {double fx2,fy2;front(&fx2,&fy2);last_b2dist=fabs(726+70-(px-SENS_MM));printf("  block 2 distance at the end: %d mm\n",(int)last_b2dist);} printf("  ended %s at t=%.1f s, pos (%.0f,%.0f) heading %.0f odo %.0f  [why \"%s\" at %d mm, last IR black L %d %d %d R %d %d %d, drifts %u]\n",TRAuto_StateText(a.st),(now-a.t0)/1000.0,px,py,hd,odo,a.why,a.fault_mm,a.l_on[(a.l_n+0)%3],a.l_on[(a.l_n+1)%3],a.l_on[(a.l_n+2)%3],a.r_on[(a.r_n+0)%3],a.r_on[(a.r_n+1)%3],a.r_on[(a.r_n+2)%3],a.search_drifts); break; }
  }
  printf("  T at x=1807: turn 1 started with the centre at x=%.0f, ended at x=%.0f (%+.0f mm); front of the TR reached x=%.0f\n",turn1_start_x,turn1_end_x,turn1_end_x-1807,max_tip_x);
  CHECK(fabs(turn1_end_x-1807)<25);   /* turned over the T, not past it */
  CHECK(a.st==TRA_DONE);
  for(int s=TRA_EAST;s<=TRA_GRAB2;s++) if(s!=TRA_ADV2) CHECK(state_seen[s]); /* ADV2 is skipped when the sensors are already past J2 */
  printf("  block 1 dropped at (%.0f, %.0f) (SA edge at y=1712)\n",b1x,b1y);
  CHECK(b1y>1236+476+70);                          /* fully inside the SA: >70 mm past its edge */
  CHECK(fabs(fabs(b1x-1807)-TRA_DROP_SHIFT_MM)<30); /* slid TRA_DROP_SHIFT_MM sideways off the line (into one SA) before the drop */
  CHECK(fabs(px-(726+70+SENS_MM))<120 && fabs(py-1236)<120);   /* ended next to block 2 */
  CHECK(arm_us<=1630);
  printf(fails?"%d FAILED\n":"TR AUTO ALL PASS\n",fails); return fails!=0;
}
