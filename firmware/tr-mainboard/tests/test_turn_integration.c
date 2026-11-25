/* Compile actual TRDrive implementation with mock CAN/UART and time. */
#include "mocks/host_hal.h"
#include <assert.h>
#include <math.h>
#include <string.h>
#include "../Core/Src/tr_drive.c"

CAN_HandleTypeDef hcan1, hcan2;
UART_HandleTypeDef huart1;
static uint32_t tick;
static uint8_t mask1=3, mask2=4;
static int16_t rpm[3];
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *p, uint16_t n, uint32_t t)
{ (void)h; (void)p; (void)n; (void)t; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *p, uint16_t n)
{ (void)h; (void)p; (void)n; return HAL_OK; }
HAL_StatusTypeDef rm_can_init(CAN_HandleTypeDef *h) { (void)h; return HAL_OK; }
HAL_StatusTypeDef rm_ctrl_loop(CAN_HandleTypeDef *h) { (void)h; return HAL_OK; }
void rm_can_get_status(CAN_HandleTypeDef *h, RM_CAN_Status *s)
{ (void)h; memset(s, 0, sizeof(*s)); s->init_ok=1; }
uint8_t rm_get_feedback_mask(CAN_HandleTypeDef *h, uint32_t age)
{ (void)age; return h==&hcan1 ? mask1 : mask2; }
void rm_set_current_setpoint(CAN_HandleTypeDef *h, RM_Motor m, float c)
{ (void)h; (void)m; (void)c; }
int16_t rm_get_actual_vel(CAN_HandleTypeDef *h, RM_Motor m)
{ (void)h; return rpm[m]; }
uint8_t rm_get_temperature(CAN_HandleTypeDef *h, RM_Motor m)
{ (void)h; (void)m; return 25; }
void TRArm_Init(void) {}
void TRArm_Update(int8_t c) { (void)c; }
uint16_t TRArm_GetPulseUs(void) { return 2500; }
static void send(const char *s) { assert(parse_line(s, (uint8_t)strlen(s))); }
static void update(void) { ++tick; TRDrive_Update(); }
static int stopped_targets(void)
{ return fabsf(status.target_rpm[0]) < 0.001f && fabsf(status.target_rpm[1]) < 0.001f && fabsf(status.target_rpm[2]) < 0.001f; }
int main(void)
{
  TRDrive_Init();
  send("H0\n"); send("M+0.00/+0.00\n"); send("R+0.00\n"); send("Y0\n"); update();
  send("YR\n"); update(); assert(status.turn_state == TR_TURN_REJECT && stopped_targets());
  send("Y0\n"); send("HZ\n"); update(); assert(status.heading_valid);
  send("YR\n"); update(); assert(status.turn_state == TR_TURN_RUN);
  assert(status.target_rpm[0] > 0 && status.target_rpm[1] > 0 && status.target_rpm[2] > 0);
  send("YR\n"); update(); assert(status.turn_state == TR_TURN_RUN);
  heading.degrees = 90.0f; update(); assert(status.turn_state == TR_TURN_DONE && stopped_targets());
  send("YR\n"); update(); assert(status.turn_state == TR_TURN_DONE); /* held request cannot repeat */
  send("Y0\n"); send("YL\n"); update(); assert(status.turn_state == TR_TURN_RUN);
  send("YX\n"); update(); assert(status.turn_state == TR_TURN_ABORT && stopped_targets());
  send("YL\n"); update(); assert(status.turn_state == TR_TURN_ABORT); /* YX requires fresh idle */
  send("Y0\n"); send("YU\n"); update(); assert(status.turn_state == TR_TURN_RUN);
  send("M+0.50/+0.00\n"); update(); assert(status.turn_state == TR_TURN_ABORT && stopped_targets());
  send("M+0.00/+0.00\n"); send("M+0.00/+0.00\n"); send("M+0.00/+0.00\n");
  send("Y0\n"); update(); send("YL\n"); update(); assert(status.turn_state == TR_TURN_RUN);
  mask2=0; update(); assert(status.turn_state == TR_TURN_ABORT && stopped_targets());
  mask2=4; send("Y0\n"); send("HZ\n"); update(); assert(status.heading_valid);
  send("YL\n"); update(); assert(status.turn_state == TR_TURN_RUN);
  for (int i=0; i<13; i++) {
    tick += 49;
    send("Y0\n"); send("H0\n"); send("M+0.00/+0.00\n"); send("R+0.00\n"); update();
  }
  assert(status.turn_state == TR_TURN_NOFB && stopped_targets());
  send("YR\n"); update(); assert(status.turn_state == TR_TURN_RUN);
  for (int i=0; i<8; i++) {
    tick += 49;
    send("H0\n"); send("M+0.00/+0.00\n"); send("R+0.00\n"); update();
  }
  assert(status.turn_state == TR_TURN_ABORT && stopped_targets()); /* stale Y heartbeat */
  return 0;
}
