/* Actual rising-edge callbacks: neither background nor deadzone is supplied. */
#include "encoder_monitor.h"
#include "main.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

uint32_t mock_primask;
static uint32_t tick, raw_l, raw_r;
uint32_t HAL_GetTick(void) { assert(mock_primask); return tick; }
static void window(uint32_t ms, uint32_t l, uint32_t r) {
  raw_l += l; raw_r += r;
  while (l--) HAL_GPIO_EXTI_Callback(ELCP_J3_FB_Pin);
  while (r--) HAL_GPIO_EXTI_Callback(ELCP_J1_FB_Pin);
  tick += ms;
}
static EncoderReading sample(EncoderMode mode) {
  const uint32_t saved = mock_primask;
  const EncoderReading s = EncoderMonitor_Read();
  assert(mock_primask == saved && s.mode == mode);
  assert(s.left_j3.edges == raw_l && s.right_j1.edges == raw_r);
  return s;
}
static void fields(EncoderReading s, uint32_t l, uint32_t r, int64_t nl, int64_t nr, int64_t cl, int64_t cr) {
  assert(s.left_j3.pps == l && s.right_j1.pps == r);
  assert(s.left_j3.net_pps == nl && s.right_j1.net_pps == nr);
  assert(s.left_j3.net_edges == cl && s.right_j1.net_edges == cr);
}
static void idle(EncoderReading s, uint32_t l, uint32_t r) {
  assert(s.left_j3.idle_pps == l && s.right_j1.idle_pps == r);
  assert(!!s.left_j3.deadzone_pps == !!l && !!s.right_j1.deadzone_pps == !!r);
}
static void profile(EncoderReading s, unsigned l, unsigned ld, unsigned r, unsigned rd) {
  idle(s,l,r); assert(s.left_j3.deadzone_pps==ld && s.right_j1.deadzone_pps==rd);
}
/* Deliver edges uniformly, not all at the start of an interval. A 50 ms poll
 * can contribute evidence, but cannot close a 200 ms NET window on its own. */
static void span(unsigned ms, unsigned left, unsigned right) {
  unsigned at=0, l=0, r=0;
  while (at<ms) {
    unsigned dt=ms-at<50 ? ms-at : 50; at+=dt;
    unsigned a=(uint64_t)at*left/ms, b=(uint64_t)at*right/ms;
    window(dt,a-l,b-r); EncoderMonitor_Poll(tick); l=a; r=b;
  }
}
static void learn(unsigned l, unsigned r) {
  /* Feed the signals; NEVER press BTN1 here. A wheel may remain unqualified. */
  span(3000,3*l,3*r);
  idle(sample(l && r ? ENCODER_MEASURING : l || r ? ENCODER_PARTIAL : ENCODER_SETTLING),l,r);
}
static void calibration(void) {
  tick=UINT32_MAX-2000u; EncoderMonitor_Init();
  window(1100,1100,44); fields(sample(ENCODER_SETTLING),1000,40,0,0,0,0);
  for (unsigned i=0;i<60;++i) {
    /* Each wheel has a 200 ms double across the middle two blocks. The
     * final, clean windows can lock automatically despite the outliers. */
    window(50,i>=23 && i<27 ? 60 : 30,i>=20 && i<24 ? 30 : 15);
    EncoderMonitor_Poll(tick);
    if (i<59) idle(sample(ENCODER_SETTLING),0,0);
    if (i==27) { assert(sample(ENCODER_SETTLING).left_j3.pps==1050); assert(!EncoderMonitor_StartIdleSample()); }
  }
  EncoderReading s=sample(ENCODER_MEASURING); profile(s,600,5,300,5); fields(s,600,300,0,0,0,0);
  span(150,108,36); window(49,35,11); fields(sample(ENCODER_MEASURING),600,300,0,0,0,0);
  window(1,1,1); fields(sample(ENCODER_MEASURING),720,240,120,-60,24,-12);
  window(100,888,999); mock_primask=1; assert(EncoderMonitor_StartIdleSample());
  assert(mock_primask==1); mock_primask=0;
  learn(200,150); fields(sample(ENCODER_MEASURING),200,150,0,0,24,-12);
  span(200,16,42); fields(sample(ENCODER_MEASURING),80,210,-120,60,0,0);
  EncoderMonitor_Init(); s=sample(ENCODER_SETTLING); idle(s,0,0); fields(s,0,0,0,0,0,0);
  puts("Encoder auto-lock: independent clean majorities, raw spike visibility, 199/200 ms boundary and BTN1 re-arm PASS");
}
static void irregular(void) {
  EncoderMonitor_Init();
  span(2400,240,480); idle(sample(ENCODER_SETTLING),0,0);
  window(3000,304,600); idle(sample(ENCODER_SETTLING),0,0); /* gap breaks pending evidence, not 15 fabricated rates */
  for (unsigned i=0;i<3;++i) {window(297,i==2?31:30,i==2?60:59); sample(ENCODER_SETTLING);}
  for (unsigned i=0;i<12;++i) {window(200,20,40); sample(i==11 ? ENCODER_MEASURING : ENCODER_SETTLING);}
  profile(sample(ENCODER_MEASURING),101,6,200,5); /* 331 edges / 3291 ms, not an unweighted block mean */
  for (unsigned i=0;i<3;++i) {
    window(317,64,61); fields(sample(ENCODER_MEASURING),201,192,i==2?100:0,-7,i==2?95:0,-(int64_t)((i+1)*2400/1000));
  }
  puts("Encoder timing: real fresh windows after a long poll gap, duration-weighted calibration, delayed signed rates PASS");
}
static void bands(void) {
  EncoderMonitor_Init();
  for (unsigned block=0;block<5;++block) for (unsigned k=0;k<3;++k) {
    /* Identical 1200 PPS carriers, but repeatable +/-60 on J3 vs +/-5
     * on J1. Discard a full double and don't set DZ from a lone +/-70. */
    unsigned d=block==1?14:12;
    unsigned l=k==0?240-d:k==1?240+d:240;
    if (block==2 && k==1) l=480;
    span(200,l,k==0?239:k==1?241:240);
  }
  profile(sample(ENCODER_MEASURING),1200,60,1200,5);
  /* Same +75 PPS on both: three of the four probes rise by only +40,
   * inside J3's learned band; only J1 may take the fast positive path. */
  for (unsigned k=0;k<4;++k) span(50,k==3?69:62,k==3?69:62);
  EncoderReading s=sample(ENCODER_MEASURING); fields(s,1275,1275,0,75,0,15);
  assert(s.left_j3.rise_pending && !s.right_j1.rise_pending);
  span(200,240,240); fields(sample(ENCODER_MEASURING),1200,1200,0,0,0,15);
  span(200,252,239); fields(sample(ENCODER_MEASURING),1260,1195,0,0,0,15);
  span(200,228,241); fields(sample(ENCODER_MEASURING),1140,1205,0,0,0,15); /* equality, both signs */
  span(600,3*227,3*243); fields(sample(ENCODER_MEASURING),1135,1215,-65,15,-39,24);
  span(6000,0,0); s=sample(ENCODER_MEASURING); profile(s,1200,60,1200,5); fields(s,0,0,0,0,-39,24);
  span(6000,9000,5400); s=sample(ENCODER_MEASURING); profile(s,1200,60,1200,5);
  fields(s,1500,900,300,-300,1761,-1776); /* do not learn steady motion, even after signal loss */
  assert(EncoderMonitor_StartIdleSample()); profile(sample(ENCODER_SETTLING),0,0,0,0);
  for (unsigned i=0;i<5;++i) {span(200,240,228);span(200,240,252);span(200,240,240);}
  s=sample(ENCODER_MEASURING); profile(s,1200,5,1200,60); fields(s,1200,1200,0,0,1761,-1776);
  span(200,232,248); fields(sample(ENCODER_MEASURING),1160,1240,-40,0,1753,-1776);
  puts("Encoder learned bands: same idle but +/-60 vs +/-5, probe and motion gating, full signed credit, swap on manual re-arm only PASS");
}
static void settling(void) {
  EncoderMonitor_Init(); span(6000,0,0); profile(sample(ENCODER_SETTLING),0,0,0,0);
  for (unsigned i=0;i<10;++i) {
    /* Wild 200 ms samples average to a deceptively constant 300 per 600 ms. */
    span(200,30,40); span(200,90,40); span(200,60,40);
    idle(sample(i<4?ENCODER_SETTLING:ENCODER_PARTIAL),0,i<4?0:200);
  }
  for(unsigned i=0;i<5;++i) {
    span(600,240,156); /* J3 settles without a button; locked J1 measures +60 throughout. */
    EncoderReading s=EncoderMonitor_Read(); assert(s.right_j1.net_edges==36*(i+1) && s.right_j1.idle_pps==200);
    if(i<3) assert(!s.left_j3.idle_pps && !s.left_j3.net_edges);
  }
  EncoderReading s=sample(ENCODER_MEASURING); profile(s,400,5,200,5); fields(s,400,260,0,60,0,180);
  EncoderMonitor_Init(); span(2400,1440,720); /* old majority at the instant of a step / disconnect */
  span(600,540,0); profile(sample(ENCODER_SETTLING),0,0,0,0);
  for(unsigned i=0;i<5;++i) {span(600,540,0); assert(!EncoderMonitor_Read().right_j1.idle_pps);}
  profile(sample(ENCODER_PARTIAL),900,5,0,0);
  EncoderMonitor_Init();
  for(unsigned i=0;i<9;++i) { /* each block is quiet but its center drifts by 5 PPS every block */
    span(600,354+i*3,354+i*3); profile(sample(ENCODER_SETTLING),0,0,0,0);
  }
  /* Even one dead window breaks continuity before lock, but never erases
   * another wheel's evidence. Exactly two windows between polls is a gap. */
  EncoderMonitor_Init(); span(2400,720,480);
  window(400,120,80); idle(sample(ENCODER_SETTLING),0,0);
  span(2400,720,480); span(200,0,40); span(400,120,80);
  profile(sample(ENCODER_PARTIAL),0,0,200,5);
  span(2400,720,600); profile(sample(ENCODER_PARTIAL),0,0,200,5);
  span(200,60,50); s=sample(ENCODER_MEASURING); profile(s,300,5,200,5);
  fields(s,300,250,0,50,0,130);
  puts("Encoder stability: silence, within-block oscillation, drift and stale plateau do not qualify; late wheel locks independently PASS");
}
static unsigned random_step(unsigned *s) { return *s = *s * 1664525u + 1013904223u; }
static unsigned burst_edges(unsigned t, unsigned ms, unsigned off, unsigned len,
                            unsigned idle_rate, unsigned *phase) {
  unsigned start=t>off?t:off, end=t+ms<off+len?t+ms:off+len;
  *phase += idle_rate * (ms + (end>start?end-start:0));
  unsigned n=*phase/1000; *phase%=1000; return n;
}
static void startup_bursts(void) {
  /* A double partly in two blocks can leave a small shoulder beside a
   * rejected outlier. Vary fractional phase AND service time; check every
   * subsequent window for false motion, not just the displayed integer BG. */
  for(unsigned trial=0;trial<384;++trial) {
    EncoderMonitor_Init(); unsigned rng=trial;
    unsigned lp=random_step(&rng)%1000, rp=random_step(&rng)%1000;
    unsigned off=random_step(&rng)%2800, len=50+random_step(&rng)%151;
    for(unsigned t=0;t<12000;) {
      unsigned ms=200+random_step(&rng)%21;
      window(ms,burst_edges(t,ms,off,len,606,&lp),burst_edges(t,ms,2800-off,len,2400,&rp)); t+=ms;
      EncoderReading s=EncoderMonitor_Read();
      assert(s.left_j3.edges==raw_l && s.right_j1.edges==raw_r);
      assert(!s.left_j3.net_pps && !s.left_j3.net_edges && !s.right_j1.net_pps && !s.right_j1.net_edges);
      if(t>=6000) {
        assert(s.mode==ENCODER_MEASURING);
        assert(s.left_j3.idle_pps>=604 && s.left_j3.idle_pps<=608);
        assert(s.right_j1.idle_pps>=2398 && s.right_j1.idle_pps<=2402);
        assert(s.left_j3.deadzone_pps>=5 && s.left_j3.deadzone_pps<=10);
        assert(s.right_j1.deadzone_pps>=5 && s.right_j1.deadzone_pps<=10);
      }
    }
  }
  puts("Encoder startup: 384 phase/timing replays reject transient doubles without false counts after settling PASS");
}
static void bursts(void) {
  /* Sweep every millisecond offset, both wires. Even a split or diluted
   * burst must not briefly publish NET/s or leak into COUNT at rest. */
  unsigned trials=0;
  for (unsigned len=50;len<=200;len+=50) for (unsigned offset=0;offset<200;++offset) {
    EncoderMonitor_Init(); learn(606,304);
    unsigned lphase=0,rphase=0, lpeak=0,rpeak=0;
    for (unsigned t=0;t<1000;++t) {
      lphase+=606u * (1u+(t>=200+offset && t<200+offset+len));
      rphase+=304u * (1u+(t>=400+offset && t<400+offset+len));
      window(1,lphase/1000,rphase/1000); lphase%=1000; rphase%=1000;
      EncoderMonitor_Poll(tick);
      if (t%200==199) {
        EncoderReading s=sample(ENCODER_MEASURING); idle(s,606,304);
        assert(!s.left_j3.net_pps && !s.right_j1.net_pps && !s.left_j3.net_edges && !s.right_j1.net_edges);
        if(s.left_j3.pps>lpeak)lpeak=s.left_j3.pps;
        if(s.right_j1.pps>rpeak)rpeak=s.right_j1.pps;
      }
    }
    assert(lpeak>606 && rpeak>304); ++trials;
  }
  printf("Encoder live: %u phase/duration replays, double pulses in RAW/s but no false signed rate/count on either wheel PASS\n",trials);
}
static void jitter(void) {
  /* Independent, near-double bursts with irregular foreground service. No
   * Read in between polls to create artificial bins. Check every publication. */
  for(unsigned offset=0;offset<200;++offset) {
    EncoderMonitor_Init(); learn(606,304);
    unsigned lp=0,rp=0,hl=0,hr=0; bool saw_l=false,saw_r=false;
    for(unsigned t=0;t<2000;++t) {
      bool l=(t>=177+offset && t<377+offset) || (t>=954 && t<1114);
      bool r=t>=900+offset && t<1100+offset;
      lp+=l?1113:606; rp+=r?635:304; /* not exact factors of two */
      window(1,lp/1000,rp/1000); lp%=1000; rp%=1000;
      if(t%17!=0 && t%23!=0 && t!=1999) continue;
      EncoderMonitor_Poll(tick); EncoderReading s=sample(ENCODER_MEASURING);
      assert(!s.left_j3.net_edges && !s.left_j3.net_pps && !s.right_j1.net_edges && !s.right_j1.net_pps);
      saw_l |= s.left_j3.rise_pending; saw_r |= s.right_j1.rise_pending;
      if(s.left_j3.pps>hl)hl=s.left_j3.pps;
      if(s.right_j1.pps>hr)hr=s.right_j1.pps;
    }
    assert(saw_l && saw_r && hl>606 && hr>304);
  }
  EncoderMonitor_Init(); learn(600,300);
  span(200,240,60); assert(sample(ENCODER_MEASURING).left_j3.rise_pending);
  span(200,144,60); /* regular moderate motion does not release an earlier high spike */
  fields(sample(ENCODER_MEASURING),720,300,120,0,24,0);
  puts("Encoder guard: jittered near-double and repeated/simultaneous bursts, distinct HOLD and clean recovery PASS");
}
static void reversal(void) {
  EncoderMonitor_Init(); learn(600,300);
  span(200,240,72); fields(sample(ENCODER_MEASURING),1200,360,0,60,0,12);
  span(200,120,48); fields(sample(ENCODER_MEASURING),600,240,0,-60,0,0); /* discard only J3 candidate */
  span(200,240,120); fields(sample(ENCODER_MEASURING),1200,600,0,0,0,0);
  span(200,240,120); fields(sample(ENCODER_MEASURING),1200,600,600,300,240,120); /* catch-up once; rate NOT doubled */
  fields(sample(ENCODER_MEASURING),1200,600,600,300,240,120);
  span(200,360,180); fields(sample(ENCODER_MEASURING),1800,900,1200,600,480,240); /* no hard speed ceiling */
  span(2000,600,300); fields(sample(ENCODER_MEASURING),300,150,-300,-150,-120,-60);
  span(200,240,120); fields(sample(ENCODER_MEASURING),1200,600,0,0,-120,-60);
  span(200,0,30); fields(sample(ENCODER_MEASURING),0,150,0,-150,-120,-90); /* silence/reversal throw away held credit */
  span(200,240,120); assert(sample(ENCODER_MEASURING).left_j3.rise_pending);
  assert(EncoderMonitor_StartIdleSample()); EncoderReading fresh=sample(ENCODER_SETTLING);
  assert(!fresh.left_j3.rise_pending && !fresh.right_j1.rise_pending);
  span(3000,0,600); idle(sample(ENCODER_PARTIAL),0,200); span(200,140,50);
  fields(sample(ENCODER_PARTIAL),700,250,0,50,-120,-80); /* recal never releases old deferred credit */
  assert(EncoderMonitor_StartIdleSample()); learn(400,0);
  span(200,64,99); fields(sample(ENCODER_PARTIAL),320,495,-80,0,-136,-80);
  puts("Encoder signs: ordinary response in 200 ms, sustained >=2x fully credited once, reversal, silence and missing idle PASS");
}
static void fractions(void) {
  EncoderMonitor_Init(); learn(101,51); profile(sample(ENCODER_MEASURING),101,6,51,6);
  window(1000,107,45); fields(sample(ENCODER_MEASURING),107,45,0,0,0,0);
  window(1000,95,57); fields(sample(ENCODER_MEASURING),95,57,0,0,0,0); /* inclusive learned band */
  /* 1.5 s: +/-9.5 edges. PPS truncates to the band; compare unrounded. */
  for(unsigned i=1;i<=3;++i) {
    window(1500,161,67); fields(sample(ENCODER_MEASURING),107,44,i==3?6:0,-6,i==3?28:0,-(int64_t)(i*19/2));
  }
  window(1000,101,51); fields(sample(ENCODER_MEASURING),101,51,0,0,28,-28);
  assert(EncoderMonitor_StartIdleSample()); learn(101,51); /* preserve the half edges too */
  for(unsigned i=1;i<=3;++i) {
    window(1500,142,86); fields(sample(ENCODER_MEASURING),94,57,-6,i==3?6:0,(3-i)*19/2,i==3?0:-28);
  }
  puts("Encoder integration: signed fractions preserved through confirmation and quiet windows; opposite motions cancel PASS");
}
static void wide(void) {
  EncoderMonitor_Init(); learn(1001,600);
  for (unsigned i=0;i<3;++i) {
    window(1000,4300000u,600); fields(sample(ENCODER_MEASURING),4300000u,600,i==2?4298999:0,0,i==2?12896997:0,0);
  }
  window(4294967000u,1,1); fields(sample(ENCODER_MEASURING),0,0,-1000,-599,12896998LL-1001LL*4294967,-2576980199LL);
  window(4294967000u,1,1); fields(sample(ENCODER_MEASURING),0,0,-1000,-599,-(int64_t)UINT32_MAX,-(int64_t)UINT32_MAX);
  span(1000,1101,700); fields(sample(ENCODER_MEASURING),1105,700,104,100,100-(int64_t)UINT32_MAX,100-(int64_t)UINT32_MAX);
  puts("Encoder magnitude: wide numerators, delayed/tick wrap, saturating signed counts and recovery PASS");
}
int main(int argc,char **argv) {
  assert(argc==2);
  if(!strcmp(argv[1],"calibration")) calibration();
  else if(!strcmp(argv[1],"irregular")) irregular();
  else if(!strcmp(argv[1],"bands")) bands();
  else if(!strcmp(argv[1],"settling")) settling();
  else if(!strcmp(argv[1],"startup-bursts")) startup_bursts();
  else if(!strcmp(argv[1],"bursts")) bursts();
  else if(!strcmp(argv[1],"jitter")) jitter();
  else if(!strcmp(argv[1],"reversal")) reversal();
  else if(!strcmp(argv[1],"fractions")) fractions();
  else {assert(!strcmp(argv[1],"wide"));wide();}
}
