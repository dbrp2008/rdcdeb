/* Host simulation of autonomous MODE 2: the two blocks on the far-right vertical line -> both into the bottom-right SA.
 * Build: gcc -ICore/Inc -o tr_auto2_sim tests/tr_auto2_sim.c Core/Src/tr_auto.c Core/Src/tr_linetest.c -lm
 * Args: [encoder scale error, e.g. 1.03] [IR / ultrasonic distance ahead of the turning centre, mm (default TRA_ADVANCE_MM)]
 *       env TURN_UNDER=deg: the left turn at the second T comes out this much short (the wheels report the full 90)
 *       env DIAG_EXTRA=mm: the diagonal ends this much further east than commanded (an overshoot past the line)
 *       [unseen rotation while sliding sideways FAST (>= 0.7), deg per metre (- = anticlockwise; less when slower): wheel slip
 *        the encoders cannot see. Field: ~10-15 deg anticlockwise after the 1.48 m diagonal = about -8]
 * World in mm from the rulebook map (650 mm TR start zone as the ruler): X east from the start wall, Y north from the bottom
 * line. The drive has its real ramp (600 rpm/s, 200 rpm/s while turning) and coasts during a turn; a 90 deg turn takes 1.5 s. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "tr_auto.h"
#define TR_ARM_AUTO_US_PER_S 1500.0 /* = tr_arm.h (which needs the HAL): the gripper speed while the routine drives it */
static int fails;
#define CHECK(c) do{ if(!(c)){printf("FAIL line %d: %s\n",__LINE__,#c);fails++;} }while(0)
static double seg(double x,double y,double ax,double ay,double bx,double by){double dx=bx-ax,dy=by-ay,t=((x-ax)*dx+(y-ay)*dy)/(dx*dx+dy*dy);t=t<0?0:(t>1?1:t);return hypot(x-(ax+t*dx),y-(ay+t*dy));}
/* lines (centre lines of 20 mm tape) */
#define BLACK_X0 (4686.0+336.0)
#define GRAY_X0 (4686.0+307.0) /* the gray strip inside the black boundary: the IRs read it as BLACK (field). From a photo with
                                * the 139 mm block as the ruler: gray from ~307 mm east of the far-right line's centre, black ~336 */
static double sz_dev; /* sideways in / just out of the start zone */
static int green_black; static double gbx=-1e9, gby=-1e9, gtx=-1e9, gty=-1e9; /* env GREEN=1: the uncovered green squares read black */
static int box_mode, box_right_ir; /* env BOX=3: the start box (x<650) reads dark to the RIGHT IR only; BOX=1: to both */
static int black(double x,double y){
  if(x<650 && box_mode==1) return 1;
  if(x<650 && box_mode==3) return box_right_ir;
  if(x>=GRAY_X0) return 1;
  if(green_black && ((fabs(x-gbx)<69.5&&fabs(y-gby)<69.5)||(fabs(x-gtx)<69.5&&fabs(y-gty)<69.5))) return 1;
  return seg(x,y,650,0,3509,0)<10 || seg(x,y,1804,0,1804,1220)<10 || seg(x,y,721,1220,1804,1220)<10 ||  /* bottom line, T1, short line */
         seg(x,y,3226,0,3226,2859)<10 || seg(x,y,3226,1813,4686,1813)<10 || seg(x,y,4686,1213,4686,2413)<10; /* T2, horizontal, far right
         (T2 at 4686-1460: the MEASURED 146 cm centre to centre; the far-right line ends ~10 cm past each block) */
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
  const double slip_deg_per_m = argc>3 ? atof(argv[3]) : 0.0;
  double hd_err=0; /* true heading - what the encoders report */
  double px=290,py=0,hd=0,vx=0,vy=0,odo=0,sodo=0;      /* hd: 0 = east, -90 = north (clockwise positive) */
  /* top block: its south face MEASURED 50 cm north of the horizontal line (1813) -> centre 1813+500+70; the lower block
   * assumed the same distance south (not measured yet) */
  double ax=4686,ay=getenv("AY")?atof(getenv("AY")):2383,bx=4686,by=getenv("BY")?atof(getenv("BY")):1243;
  green_black=getenv("GREEN")?atoi(getenv("GREEN")):0; box_mode=getenv("BOX")?atoi(getenv("BOX")):0; const double bx0=bx, by0=by, ay0=ay; int carrying=0, dropped=0;
  double arm=2500; int turn_state=0; uint32_t turn_end=0; double turn_target=0;
  double grab_hd=0, grab_hd_enc=0; double min_fp=1e9, min_wall=1e9, min_barrier=1e9, min_b=1e9, max_side=-1e9, grab_off=-1e9; int us_saw=0;
  TRAuto a; memset(&a,0,sizeof a); TRAuto_Init(&a); TRAuto_SetMode(&a,2);
  uint32_t now=1000, next_us=0; int us_val=-1;
  TRAuto_Start(&a,now);
  for(int i=0;i<8000;i++){ now+=5;
    if(turn_state==1&&(int32_t)(now-turn_end)>=0){hd=turn_target;turn_state=2;
      if(a.st==TRB_TURN1&&getenv("TURN_UNDER")){ const double u=atof(getenv("TURN_UNDER")); hd+=u; hd_err+=u; } } /* the turn
      at the second T comes out short (field: 80-85 deg): physically u deg less, the wheels report the full 90 */
    const double r=hd*M_PI/180, fx=cos(r), fy=-sin(r), lx=-fy, ly=fx;   /* forward, left */
    TRAuto_In in; memset(&in,0,sizeof in);
    box_right_ir=0; in.left=black(px+sens*fx+25*lx,py+sens*fy+25*ly); box_right_ir=1; in.right=black(px+sens*fx-25*lx,py+sens*fy-25*ly); box_right_ir=0;
    if(getenv("RSTUCK")) in.right=1; /* the right IR reads black everywhere (knob too sensitive / fault) */
    if(px<650+300 && a.st<=TRB_SEARCH && fabs(py)>sz_dev) sz_dev=fabs(py);
    in.us_fresh=(int32_t)(now-next_us)>=0;
    if(in.us_fresh){ next_us=now+50; us_val=-1;
      const double sx=px+sens*fx, sy=py+sens*fy; const double bxs[2]={ax,bx}, bys[2]={ay,by};
      for(int k=0;k<2;k++){ if(k==0&&carrying) continue; const double rx=bxs[k]-sx, ry=bys[k]-sy, along=rx*fx+ry*fy, lat=fabs(-rx*fy+ry*fx);
        if(along>0&&lat<80){ int d=(int)(along-70); if(us_val<0||d<us_val) us_val=d; } }
      if(carrying) us_val=0; /* the carried block sits right in front of the sensor */ }
    in.us_mm=us_val; in.dist_mm=(float)odo; in.strafe_mm=(float)sodo; in.heading_deg=(float)(hd-hd_err); in.heading_sign=-1.0f;
    in.turn_state=(uint8_t)turn_state; in.arm_us=(uint16_t)arm; in.arm_closed_us=1611; in.arm_open_us=2500; in.now=now;
    TRAuto_Out o; const TRAuto_State st_before=a.st; TRAuto_Step(&a,&in,&o);
    { static uint32_t t_in=1000; if(a.st!=st_before){ if(getenv("TIMES")) printf("  %-16s %5.2f s (t=%5.2f)\n",TRAuto_StateText(st_before),(now-t_in)/1000.0,(now-1000)/1000.0); t_in=now; } } /* TIMES=1: time per state */
    if(st_before==TRB_MOVE1 && a.st!=TRB_MOVE1 && getenv("DIAG_EXTRA")) px+=atof(getenv("DIAG_EXTRA"));
    if(o.zero_heading){ hd=0; hd_err=0; }
    if(o.turn_req&&turn_state!=1){turn_state=1;turn_end=now+(uint32_t)(1500*fabs(o.turn_deg)/90*(o.turn_frac>0?0.15+0.85*0.5/o.turn_frac:1.0));turn_target=hd+o.turn_deg;} /* faster
      turns: the ramp part (~15 %) stays */
    if(turn_state==2&&a.st!=TRB_TURN1&&a.st!=TRB_TURN2) turn_state=0;
    if(o.arm>0) arm-=TR_ARM_AUTO_US_PER_S*0.005; else if(o.arm<0) arm+=TR_ARM_AUTO_US_PER_S*0.005;
    if(arm<1611) arm=1611;
    if(arm>2500) arm=2500;
    { const double tx=(o.active&&turn_state!=1)?o.x:0, ty=(o.active&&turn_state!=1)?o.y:0, step=(turn_state==1?200.0:600.0)/130.0*0.005;
      vx+=fmax(-step,fmin(step,tx-vx)); vy+=fmax(-step,fmin(step,ty-vy));
      const double sx=vx*1.037*5, sy=vy*1.037*5;       /* mm this tick, right / forward */
      px+=sy*fx-sx*lx; py+=sy*fy-sx*ly; odo+=sy*enc; sodo+=sx*enc;
      if(o.active&&turn_state!=1){ const double dw=o.w*-1.0*130.0*0.005; hd+=dw; }   /* w = 1: ~130 deg/s, + turns anticlockwise */
      const double slip=slip_deg_per_m*fabs(sx)/1000.0*fmin(1.0,fabs(vx)/0.7); hd+=slip; hd_err+=slip; } /* full slip at 0.7+ */
    /* the carried block and the pushed one */
    if(!carrying && !dropped && a.st==TRB_GRAB && grab_off<-1e8){ grab_off=px-ax; grab_hd=hd+90; grab_hd_enc=hd-hd_err+90; }
    if(a.st==TRB_NORTH && in.us_fresh && us_val>=0 && us_val<200) us_saw=1; /* the ultrasonic saw the top block on the approach */ /* sideways offset at the grab (+ = robot east of block) */
    if(!carrying && !dropped && a.st==TRB_MOVE2) { carrying=1; gtx=4686; gty=ay0; } /* the top block's square uncovered */
    if(hypot(bx-bx0,by-by0)>140) { gbx=bx0; gby=by0; } /* the lower block pushed off its square */
    if(carrying){ ax=px+(sens+140)*fx; ay=py+(sens+140)*fy;
      if(fy<-0.9 && fabs(ax-bx)<100 && ay-140<by) by=ay-140; /* pushing south: the lower block stays just ahead */ }
    if(!carrying && !dropped && a.st<TRB_SOUTH && turn_state!=1) { /* the TR's footprint (back 290, front 365, half-width 272 mm) vs the lower block (+-70) */
      const double rx=bx-px, ry=by-py, u=rx*fx+ry*fy, v=rx*lx+ry*ly; const double gu = u>0 ? u-365-70 : -u-290-70, gv=fabs(v)-272-70;
      const double g = gu>gv?gu:gv; if(g<min_fp) min_fp=g; }
    if(carrying && a.st==TRA_DONE){ carrying=0; dropped=1; }
    if(!carrying && !dropped && a.st<TRB_SOUTH) { const double db=hypot(px-bx,py-by); if(db<min_b) min_b=db; } /* robot centre vs the lower block before pushing */
    if(turn_state==1){ const double reach=carrying?sens+140+70:460;  /* corner of the robot, or the front of the carried block */
      if(WALL_X-px-reach<min_wall) min_wall=WALL_X-px-reach; if(BARRIER_Y-py-reach<min_barrier) min_barrier=BARRIER_Y-py-reach; }
    if(turn_state!=1){ /* the TR's right-most point (half-width 272 mm sideways, 365 mm to the front when facing east) */
      const double reach = fabs(fx)>0.7 ? 365 : 272;
      if(!getenv("DIAG_EXTRA") || a.st>TRB_ALIGN1) { if(px+reach>max_side) max_side=px+reach; } } /* an injected overshoot: from after FIND LINE */
    if(a.st==TRA_DONE||a.st==TRA_FAULT||a.st==TRA_TIMEUP) break;
  }
  printf("  mode 2 (encoder x%.2f, sensors %.0f mm ahead): %s at t=%.1f s%s%s, robot (%.0f,%.0f)\n",enc,sens,TRAuto_StateText(a.st),((a.t_end?a.t_end:now)-a.t0)/1000.0,
         a.why[0]?" why ":"",a.why,px,py);
  printf("  top block (%.0f,%.0f), lower block (%.0f,%.0f); SA X %.0f..%.0f Y %.0f..%.0f\n",ax,ay,bx,by,SA_X0,SA_X1,SA_Y0,SA_Y1);
  printf("  turning clearance: right wall %.0f mm, barrier %.0f mm; closest to the lower block before pushing %.0f mm; T2 found at %d (%s), slides %d\n",
         min_wall,min_barrier,min_b,a.t_found_mm,a.t_by==1?"OLD":"NEW",a.align_mm);
  printf("  heading at the grab %+.1f deg off the line (the encoders thought %+.1f); slip %.1f deg/m sideways; square-up measured %+.1f deg (result %d)\n", grab_hd, grab_hd_enc, slip_deg_per_m, a.sq_deg, a.sq_result);
  printf("  closest the TR got to the right field edge: %.0f mm (to the black, photo: %.0f mm); grab off-centre %+.0f mm; FIND LINE %s the gray\n",
         WALL_X-max_side, BLACK_X0-max_side, grab_off, a.gray_seen?"found and rejected":"never met");
  printf("  sideways in the start zone: %.0f mm\n", sz_dev);
  CHECK(sz_dev < 30);                                          /* no sideways run into the start zone's wall */
  CHECK(fabs(grab_off) < 25);
  if(slip_deg_per_m==0) CHECK(fabs(grab_hd) < 3.0);            /* square to the line at the grab (no slip on the diagonal) */                                  /* the gripper closes near the block's centre */
  CHECK(us_saw);                                               /* the ultrasonic saw the block (it was centred on the approach) */
  CHECK(BLACK_X0-max_side > 0);                                /* never into the black on the right (gray allowed) */
  CHECK(a.st==TRA_DONE);
  CHECK(ax>SA_X0+70&&ax<SA_X1-70&&ay>SA_Y0+70&&ay<SA_Y1-70);   /* top block fully inside the SA */
  CHECK(bx>SA_X0+70&&bx<SA_X1-70&&by>SA_Y0+70&&by<SA_Y1-70);   /* lower block too */
  CHECK(min_wall>20 && min_barrier>20);                       /* never swings into the right wall / barrier while turning */
  if(getenv("TIMES")) printf("  TR body to the lower block: closest %.0f mm\n",min_fp);
  CHECK(min_fp>15);                                           /* the TR's body (back corner) never touches the lower block */
  CHECK(min_b>272+70+20);                                     /* never brushes the lower block on the way to the top one */
  printf(fails?"%d FAILED\n":"TR AUTO MODE 2 PASS\n",fails); return fails!=0;
}
