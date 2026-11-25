#include "calibration_app.h"
#include "encoder_monitor.h"
#include "lcd/lcd.h"
#include "main.h"
#include "stm32f4xx_it.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

/* Real IRQ vectors and application entry points; no motor, PWM-start, ADC,
 * flash storage or ultrasonic implementation is linked. BTN1 is active low. */
GPIO_TypeDef gpiob, gpioc;
DMA_HandleTypeDef hdma_spi1_tx;
SPI_HandleTypeDef hspi1;
uint32_t mock_primask;
static uint32_t tick, preloaded, outputs, raw_l, raw_r;
static uint16_t pending;
static unsigned in_irq, frames, pin_writes;
static bool down;
static char drawing[10][17], row[10][17];

void HAL_IncTick(void) { ++tick; }
uint32_t HAL_GetTick(void) { assert(!in_irq); return tick; }
void HAL_SPI_IRQHandler(SPI_HandleTypeDef *h) { (void)h; assert(0); }
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *h) { (void)h; assert(0); }
static unsigned capture_started;
void EncCapture_Start(void) { ++capture_started; }
void HAL_GPIO_EXTI_IRQHandler(uint16_t pin) {
  if (pending & pin) { pending &= (uint16_t)~pin; HAL_GPIO_EXTI_Callback(pin); }
}
static void dispatch(uint16_t pins) {
  assert(!mock_primask); raw_l += !!(pins & ELCP_J3_FB_Pin); raw_r += !!(pins & ELCP_J1_FB_Pin);
  pending |= pins & (IR_L2_Pin | IR_R2_Pin); in_irq = 1; /* side IRs: real EXTI vectors */
  EXTI4_IRQHandler(); EXTI9_5_IRQHandler();
  /* encoder FB: timer capture (enc_capture.c, register-level) hands each filtered edge to the same callback */
  if (pins & ELCP_J3_FB_Pin) HAL_GPIO_EXTI_Callback(ELCP_J3_FB_Pin);
  if (pins & ELCP_J1_FB_Pin) HAL_GPIO_EXTI_Callback(ELCP_J1_FB_Pin);
  in_irq = 0; assert(!pending);
}
static void pulses(unsigned l, unsigned r) {
  while (l--) dispatch(ELCP_J3_FB_Pin);
  while (r--) dispatch(ELCP_J1_FB_Pin);
}
static unsigned pwm(GPIO_TypeDef *port, uint32_t pin) {
  if (port == &gpiob && pin == (1u << 14)) return 1u;
  assert(port == &gpioc && pin == (1u << 7)); return 2u;
}
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState level) {
  assert(!in_irq && level == GPIO_PIN_RESET); ++pin_writes; preloaded |= pwm(port, pin);
}
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *io) {
  const unsigned bit = pwm(port, io->Pin);
  assert(!in_irq && (preloaded & bit) && io->Mode == GPIO_MODE_OUTPUT_PP && io->Pull == GPIO_NOPULL);
  outputs |= bit;
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin) {
  assert(outputs == 3u && !in_irq && !mock_primask && port == &gpiob && pin == (1u << 1));
  return down ? GPIO_PIN_RESET : GPIO_PIN_SET;
}
static void clear_drawing(void) {
  for (unsigned y=0; y<10; ++y) { memset(drawing[y], ' ', 16); drawing[y][16]='\0'; }
}
void tft_init(TFT_ORIENTATION o, uint16_t bg, uint16_t fg, uint16_t a, uint16_t h) {
  assert(outputs == 3u && pin_writes == 2u && !mock_primask && !in_irq);
  assert(o == PIN_ON_TOP && bg == BLACK && fg == WHITE && a == CYAN && h == DARK_GREY);
  dispatch(ELCP_J3_FB_Pin); tick += 150u; clear_drawing(); /* edges during TFT startup are live */
}
void tft_printc(uint8_t x, uint8_t y, const char *s) {
  assert(!in_irq && !mock_primask && outputs==3u && y<10u && x+strlen(s)<=16u);
  memcpy(drawing[y]+x, s, strlen(s));
}
void tft_printl(uint8_t x, uint8_t y, int64_t n) {
  char text[12]; const int len=snprintf(text,sizeof text,"%lld",(long long)n);
  assert(len>0 && len<(int)sizeof text); tft_printc(x,y,text);
}
void tft_update(uint32_t ms) {
  assert(ms==0 && !in_irq && !mock_primask); ++frames; memcpy(row,drawing,sizeof row);
  for (unsigned y=0; y<10; ++y) for (int x=15; x>=0 && row[y][x]==' '; --x) row[y][x]='\0';
  clear_drawing();
}
static void run(unsigned ms) { while (ms--) { ++tick; CalibrationApp_Update(); } }
/* Realistic pulse timing matters for the short windows and transient guard. */
static void stream(unsigned ms, unsigned lp, unsigned rp) {
  static unsigned l,r;
  while(ms--) { l+=lp; r+=rp; pulses(l/1000,r/1000); l%=1000; r%=1000; run(1); }
}
static void text(unsigned y, const char *s) { assert(strcmp(row[y],s)==0); }
static EncoderReading state(EncoderMode mode) {
  const EncoderReading r=EncoderMonitor_Read(); assert(r.mode==mode);
  assert(r.left_j3.edges==raw_l && r.right_j1.edges==raw_r && !mock_primask);
  return r;
}
static void tap(void) { down=false; run(40); down=true; run(31); }
int main(void) {
  dispatch(IR_L2_Pin|IR_R2_Pin); dispatch(IR_R2_Pin|ELCP_J3_FB_Pin);
  dispatch(ELCP_J3_FB_Pin|ELCP_J1_FB_Pin); dispatch(0); state(ENCODER_SETTLING);
  tick=UINT32_MAX-1800u; down=false; CalibrationApp_Init(); CalibrationApp_Update();
  assert(raw_l==3u && raw_r==1u && frames==1u); text(0,"ENCODER NO DRIVE");
  text(1,"AUTO: KEEP STILL"); text(2,"J3 WAIT STABLE"); text(4,"NET/s --"); text(5,"COUNT 0");
  /* No press at all. A startup double is visible in RAW/s, not learned as idle or DZ. */
  stream(400,480,250); stream(200,960,250); text(3,"RAW/s 960"); text(4,"NET/s --");
  for(unsigned i=0;i<4;++i) {
    stream(200,465,250); stream(200,495,250); stream(i==3?199:200,480,250);
  }
  state(ENCODER_SETTLING); stream(1,480,250); state(ENCODER_MEASURING); text(1,"READY BTN1:RECAL");
  text(2,"J3 BG480 +/-15"); text(6,"J1 BG250 +/-5"); text(5,"COUNT 0"); text(9,"COUNT 0");
  stream(200,960,250); text(3,"RAW/s 960"); text(4,"NET/s 0 HOLD"); text(5,"COUNT 0");
  stream(200,480,250); text(4,"NET/s 0"); text(5,"COUNT 0");
  stream(200,600,150); text(4,"NET/s +120"); text(5,"COUNT +24");
  stream(800,600,150); text(3,"RAW/s 600"); text(7,"RAW/s 150");
  text(4,"NET/s +120"); text(5,"COUNT +120"); text(8,"NET/s -100"); text(9,"COUNT -100");
  stream(1000,240,450); text(4,"NET/s -240"); text(5,"COUNT -120");
  text(8,"NET/s +200"); text(9,"COUNT +100"); /* both signed counts crossed zero */
  run(1000); state(ENCODER_MEASURING); text(4,"NET/s 0"); text(5,"COUNT -120"); /* silence does not re-arm a locked profile */

  down=true; run(12); down=false; run(40); state(ENCODER_MEASURING); /* bounce ignored */
  down=true; run(30); state(ENCODER_MEASURING); run(1); state(ENCODER_SETTLING);
  text(1,"AUTO: KEEP STILL"); text(2,"J3 WAIT STABLE"); text(5,"COUNT -120");
  down=false; stream(40,0,200); down=true; stream(31,0,200); /* busy press cannot restart capture */
  stream(3000-71,0,200); state(ENCODER_PARTIAL); text(1,"WAIT J3: STILL");
  text(2,"J3 WAIT STABLE"); text(4,"NET/s --"); text(6,"J1 BG200 +/-5"); text(9,"COUNT +100");
  stream(1000,400,180); text(3,"RAW/s 400"); text(5,"COUNT -120"); text(9,"COUNT +80");
  tap(); state(ENCODER_SETTLING); /* manual re-arm is available even from partial readiness */
  down=false; stream(2990,1001,201);
  down=true; stream(10,1001,201); text(1,"READY BTN1:RECAL"); text(2,"J3 BG1001 +/-6"); text(6,"J1 BG201 +/-6");
  run(31); state(ENCODER_MEASURING); /* debounce of that last press straddled READY: not queued */
  pulses(1,0); tick+=4294967000u; CalibrationApp_Update();
  text(3,"RAW/s 0"); text(4,"NET/s -1000"); text(5,"COUNT-4294967295"); /* no clipped digit */
  text(8,"NET/s 0"); text(9,"COUNT +80"); state(ENCODER_MEASURING);
  /* Values too wide to share 16 columns alternate without clipping. */
  tap(); state(ENCODER_SETTLING); down=false;
  for(unsigned b=0;b<5;++b) {stream(200,98500,0); stream(200,101500,0); stream(200,100000,0);}
  state(ENCODER_PARTIAL); text(1,"WAIT J1: STILL"); text(6,"J1 WAIT STABLE");
  bool dz_page=((tick/2000u)&1u)!=0; text(2,dz_page?"J3 DZ 1500":"J3 BG 100000");
  stream(2000,100000,0); text(2,dz_page?"J3 BG 100000":"J3 DZ 1500");
  assert(pin_writes==2u && outputs==3u); /* no later PWM/DIR or motor action */
  assert(capture_started == 1u); /* the bench starts the FB capture once */
  puts("Passive EncoderBench: real side EXTI + FB capture, hands-free per-wheel BG +/- DZ, partial readiness, BTN1 re-arm and signed TFT with no drive PASS");
}
