/* Host simulation of autonomous MODE 2: the two blocks on the far-right vertical line -> both into the bottom-right SA.
 * Build: gcc -ICore/Inc -o tr_auto2_sim tests/tr_auto2_sim.c Core/Src/tr_auto.c Core/Src/tr_linetest.c -lm
 * Args: [encoder scale error, e.g. 1.03] [IR / ultrasonic distance ahead of the turning centre, mm (default TRA_ADVANCE_MM)]
 * World in mm from the rulebook map (650 mm TR start zone as the ruler): X east from the start wall, Y north from the bottom
 * line. The drive has its real ramp (600 rpm/s, 200 rpm/s while turning) and coasts during a turn; a 90 deg turn takes 1.5 s. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "tr_auto.h"
static int fails;
#define CHECK(c) do{ if(!(c)){printf("FAIL line %d: %s\n",__LINE__,#c);fails++;} }while(0)
static double seg(double x,double y,double ax,double ay,double bx,double by){double dx=bx-ax,dy=by-ay,t=((x-ax)*dx+(y-ay)*dy)/(dx*dx+dy*dy);t=t<0?0:(t>1?1:t);return hypot(x-(ax+t*dx),y-(ay+t*dy));}
/* lines (centre lines of 20 mm tape) */
static int black(double x,double y){
  return seg(x,y,650,0,3509,0)<10 || seg(x,y,1804,0,1804,1220)<10 || seg(x,y,721,1220,1804,1220)<10 ||  /* bottom line, T1, short line */
         seg(x,y,3165,0,3165,2859)<10 || seg(x,y,3165,1813,4686,1813)<10 || seg(x,y,4686,1220,4686,2412)<10; /* T2, horizontal, far right */
}
#define WALL_X 4978.0   /* right field wall */
#define BARRIER_Y 2520.0 /* the barrier north of the top block */
#define SA_X0 4192.0
#define SA_X1 4969.0
#define SA_Y0 (-424.0)  /* bottom-right scoring area (south edge = image edge: at least this deep) */
#define SA_Y1 (-47.0)
int main(int argc,char**argv){
  const double enc = argc>1 ? atof(argv[1]) : 1.0;
  const double sens = argc>2 ? atof(argv[2]) : (double)TRA_ADVANCE_MM;
  double px=290,py=0,hd=0,vx=0,vy=0,odo=0,sodo=0;      /* hd: 0 = east, -90 = north (clockwise positive) */
  double ax=4686,ay=2412,bx=4686,by=1220; int carrying=0, dropped=0;
  double arm=2500; int turn_state=0; uint32_t turn_end=0; double turn_target=0;
  double min_wall=1e9, min_barrier=1e9, min_b=1e9;
  TRAuto a; memset(&a,0,sizeof a); TRAuto_Init(&a); TRAuto_SetMode(&a,2);
  uint32_t now=1000, next_us=0; int us_val=-1;
  TRAuto_Start(&a,now);
  for(int i=0;i<8000;i++){ now+=5;
    if(turn_state==1&&(int32_t)(now-turn_end)>=0){hd=turn_target;turn_state=2;}
    const double r=hd*M_PI/180, fx=cos(r), fy=-sin(r), lx=-fy, ly=fx;   /* forward, left */
    TRAuto_In in; memset(&in,0,sizeof in);
    in.left=black(px+sens*fx+25*lx,py+sens*fy+25*ly); in.right=black(px+sens*fx-25*lx,py+sens*fy-25*ly);
    in.us_fresh=(int32_t)(now-next_us)>=0;
    if(in.us_fresh){ next_us=now+50; us_val=-1;
      const double sx=px+sens*fx, sy=py+sens*fy; const double bxs[2]={ax,bx}, bys[2]={ay,by};
      for(int k=0;k<2;k++){ if(k==0&&carrying) continue; const double rx=bxs[k]-sx, ry=bys[k]-sy, along=rx*fx+ry*fy, lat=fabs(-rx*fy+ry*fx);
        if(along>0&&lat<80){ int d=(int)(along-70); if(us_val<0||d<us_val) us_val=d; } }
      if(carrying) us_val=0; /* the carried block sits right in front of the sensor */ }
    in.us_mm=us_val; in.dist_mm=(float)odo; in.strafe_mm=(float)sodo; in.heading_deg=(float)hd; in.heading_sign=-1.0f;
    in.turn_state=(uint8_t)turn_state; in.arm_us=(uint16_t)arm; in.arm_closed_us=1611; in.arm_open_us=2500; in.now=now;
    TRAuto_Out o; TRAuto_Step(&a,&in,&o);
    if(o.zero_heading) hd=0;
    if(o.turn_req&&turn_state!=1){turn_state=1;turn_end=now+(uint32_t)(1500*fabs(o.turn_deg)/90);turn_target=hd+o.turn_deg;}
    if(turn_state==2&&a.st!=TRB_TURN1&&a.st!=TRB_TURN2) turn_state=0;
    if(o.arm>0) arm-=600*0.005; else if(o.arm<0) arm+=600*0.005;
    if(arm<1611) arm=1611;
    if(arm>2500) arm=2500;
    { const double tx=(o.active&&turn_state!=1)?o.x:0, ty=(o.active&&turn_state!=1)?o.y:0, step=(turn_state==1?200.0:600.0)/130.0*0.005;
      vx+=fmax(-step,fmin(step,tx-vx)); vy+=fmax(-step,fmin(step,ty-vy));
      const double sx=vx*1.037*5, sy=vy*1.037*5;       /* mm this tick, right / forward */
      px+=sy*fx-sx*lx; py+=sy*fy-sx*ly; odo+=sy*enc; sodo+=sx*enc; }
    /* the carried block and the pushed one */
    if(!carrying && !dropped && a.st==TRB_MOVE2) carrying=1;
    if(carrying){ ax=px+(sens+140)*fx; ay=py+(sens+140)*fy;
      if(fy<-0.9 && fabs(ax-bx)<100 && ay-140<by) by=ay-140; /* pushing south: the lower block stays just ahead */ }
    if(carrying && a.st==TRA_DONE){ carrying=0; dropped=1; }
    if(!carrying && !dropped && a.st<TRB_SOUTH) { const double db=hypot(px-bx,py-by); if(db<min_b) min_b=db; } /* robot centre vs the lower block before pushing */
    if(turn_state==1){ const double reach=carrying?sens+140+70:460;  /* corner of the robot, or the front of the carried block */
      if(WALL_X-px-reach<min_wall) min_wall=WALL_X-px-reach; if(BARRIER_Y-py-reach<min_barrier) min_barrier=BARRIER_Y-py-reach; }
    if(a.st==TRA_DONE||a.st==TRA_FAULT||a.st==TRA_TIMEUP) break;
  }
  printf("  mode 2 (encoder x%.2f, sensors %.0f mm ahead): %s at t=%.1f s%s%s, robot (%.0f,%.0f)\n",enc,sens,TRAuto_StateText(a.st),(now-a.t0)/1000.0,
         a.why[0]?" why ":"",a.why,px,py);
  printf("  top block (%.0f,%.0f), lower block (%.0f,%.0f); SA X %.0f..%.0f Y %.0f..%.0f\n",ax,ay,bx,by,SA_X0,SA_X1,SA_Y0,SA_Y1);
  printf("  turning clearance: right wall %.0f mm, barrier %.0f mm; closest to the lower block before pushing %.0f mm; T2 found at %d (%s), slides %d\n",
         min_wall,min_barrier,min_b,a.t_found_mm,a.t_by==1?"OLD":"NEW",a.align_mm);
  CHECK(a.st==TRA_DONE);
  CHECK(ax>SA_X0+70&&ax<SA_X1-70&&ay>SA_Y0+70&&ay<SA_Y1-70);   /* top block fully inside the SA */
  CHECK(bx>SA_X0+70&&bx<SA_X1-70&&by>SA_Y0+70&&by<SA_Y1-70);   /* lower block too */
  CHECK(min_wall>20 && min_barrier>20);                       /* never swings into the right wall / barrier while turning */
  CHECK(min_b>272+70+20);                                     /* never brushes the lower block on the way to the top one */
  printf(fails?"%d FAILED\n":"TR AUTO MODE 2 PASS\n",fails); return fails!=0;
}
