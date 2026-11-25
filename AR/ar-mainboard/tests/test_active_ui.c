#include "main.h"
#include "us_capture.h"
#include "ir_sensor.h"
#include "elcp_motor/elcp_motor.h"
#include "ar_merged_app.h"
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
GPIO_TypeDef gpioa,gpiob; RCC_Type rcc; TIM_Type tim2;
volatile uint32_t elcp_j1_fb_edges,elcp_j3_fb_edges,elcp_j1_fb_min_dt_us,elcp_j3_fb_min_dt_us,elcp_j1_fb_rejected,elcp_j3_fb_rejected;
void ELCP_FB_ResetStats(void){elcp_j1_fb_min_dt_us=elcp_j3_fb_min_dt_us=0xFFFFFFFFu;elcp_j1_fb_rejected=elcp_j3_fb_rejected=0;}
static uint32_t now; static unsigned pressed; static int black, fail_adc, fail_us, fail_motor, init_count, starts, stops, draws;static bool inited[2];static float speed[2];static char row[10][100];
bool mock_btn(int x){return (pressed & (unsigned)x)!=0;}
uint32_t HAL_GetTick(void){return now;}
void IR_Sensor_Init(void){}
HAL_StatusTypeDef IR_ReadAll(IR_RawScan*s){if(fail_adc)return HAL_ERROR;for(int i=0;i<3;i++)s->raw[i]=(black==1||(black==2&&i==1))?3000:1000;return HAL_OK;}
bool US_Init(void){return true;}
uint8_t US_InitCode(void){return 0;}
uint32_t US_ShotCount(void){return 0;}
uint32_t US_IrqCount(void){return 0;}
uint8_t US_EchoSeenMask(void){return 0;}
void US_DebugSnapshot(US_Debug*d){*d=(US_Debug){0};}
void US_SetEnabled(uint8_t m){(void)m;}
void US_Update(uint32_t n){(void)n;}
bool US_Take(US_Id id,US2_Reading*r){static uint32_t t[3];if(now-t[id]<60)return false;t[id]=now;*r=(US2_Reading){(id==US_FC&&fail_us)?US2_NO_RISE:US2_OK,1000,id==US_FC?500u:700u};return true;}
void ELCP_Motor_Init(ELCP_Motor_ID id){init_count++;inited[id]=!(fail_motor&&id==MOTOR_J3);}
bool ELCP_Motor_IsInitialized(ELCP_Motor_ID id){return inited[id];}
void ELCP_Motor_SetSpeed(ELCP_Motor_ID id,float value){assert(inited[0]&&inited[1]);speed[id]=value;if(value!=0)starts++;}
void ELCP_Motor_Stop(ELCP_Motor_ID id){speed[id]=0;stops++;}
void tft_printc(uint8_t x,uint8_t y,const char*s){assert(x==0&&y<10);snprintf(row[y],sizeof(row[y]),"%s",s);}
void tft_prints(uint8_t x,uint8_t y,const char*fmt,...){assert(x==0&&y<10);va_list ap;va_start(ap,fmt);vsnprintf(row[y],sizeof(row[y]),fmt,ap);va_end(ap);}
void tft_update(uint32_t x){(void)x;assert(speed[0]==0&&speed[1]==0);draws++;}
static void run(unsigned ms){for(unsigned i=0;i<ms;i+=10){now+=10;AR_MergedApp_Update();}}
static void tap(unsigned x){pressed=x;run(100);pressed=0;run(120);}
static void next(void){pressed=3;run(1150);pressed=0;run(150);}
static void prepare(void){AR_MergedApp_Init();run(120);tap(1);run(1500);assert(strstr(row[5],"WHITE:1"));black=1;tap(2);run(1500);assert(strstr(row[5],"BLACK:1"));black=2;run(100);next();next();assert(strstr(row[0],"NAV RUN"));}
int main(int argc,char**argv){assert(argc==2);int test=atoi(argv[1]);
if(test==7){AR_MergedApp_Init();run(200);next();next();tap(1);run(2500);assert(starts==0&&strstr(row[1],"CHECK IR CAL/ADC"));puts("Uncalibrated start blocked PASS");return 0;}
if(test==4)fail_motor=1;
prepare();
if(test==10){next();next();next();assert(strstr(row[0],"WHEELS OFF"));
  pressed=1;for(int i=0;i<50;i++){now+=10;elcp_j3_fb_edges+=2;AR_MergedApp_Update();}assert(speed[MOTOR_J3]>0&&speed[MOTOR_J1]==0);
  pressed=2;now+=10;AR_MergedApp_Update();assert(speed[MOTOR_J3]==0&&speed[MOTOR_J1]>0);            /* switching button: first stops, then runs the other */
  for(int i=0;i<30;i++){now+=10;elcp_j1_fb_edges+=3;AR_MergedApp_Update();}
  pressed=3;now+=10;AR_MergedApp_Update();assert(speed[MOTOR_J3]==0&&speed[MOTOR_J1]==0);          /* both buttons: nothing runs */
  pressed=0;run(300);assert(strstr(row[3],"J3 E   98")&&strstr(row[4],"J1 E   90"));
  puts("Wheel test screen PASS");return 0;}
if(test==11){tap(2);tap(2);assert(strstr(row[0],"KEY MID")); /* A -> B -> MID */
  fail_us=1;run(100);tap(1);run(2600);                                                     /* US reads "no echo": the route still drives */
  assert(starts>0&&speed[0]!=0&&speed[1]!=0&&!strstr(row[1],"US2 FAULT"));puts("Route ignores no-echo PASS");return 0;}
if(test==9){next();next();run(300);assert(strstr(row[0],"ALL US TEST")&&strstr(row[1],"500 mm")&&strstr(row[2],"700 mm")&&strstr(row[3],"700 mm"));puts("All-US test screen PASS");return 0;}
if(test==5){fail_us=1;run(100);tap(1);run(2500);assert(starts==0&&strstr(row[1],"CHECK US2"));puts("US2 start interlock PASS");return 0;}
if(test==6){black=0;run(100);tap(1);run(2500);assert(starts==0&&strstr(row[1],"CENTRE ON LINE"));puts("Centre-on-line start interlock PASS");return 0;}
if(test==4){fail_motor=1;}tap(1);if(test==8){black=0;run(100);assert(starts==0&&strstr(row[1],"LINE LOST START"));puts("Countdown line-loss interlock PASS");return 0;}int start_draws=draws;run(1800);assert(starts==0&&init_count==2);run(500);if(test==4){assert(init_count>=2&&starts==0&&speed[0]==0&&speed[1]==0);assert(strstr(row[1],"MOTOR INIT FAIL"));}else {assert(init_count>=2&&starts>0&&speed[0]!=0&&speed[1]!=0);assert(draws==start_draws);if(test==1){pressed=1;run(10);assert(speed[0]==0&&speed[1]==0);pressed=0;run(200);int old=starts;run(2500);assert(starts==old);assert(strstr(row[1],"USER STOP"));}if(test==2){fail_adc=1;run(20);assert(speed[0]==0&&speed[1]==0&&strstr(row[1],"IR FAULT STOP"));}if(test==3){fail_us=1;run(100);assert(speed[0]==0&&speed[1]==0&&strstr(row[1],"US2 FAULT STOP"));}}
printf("Active navigation interlock case %d PASS\n",test);}
