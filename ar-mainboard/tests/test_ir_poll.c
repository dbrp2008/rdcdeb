#include "mocks_adc/host_hal.h"
#include "adc.h"
#include "ir_sensor.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
ADC_HandleTypeDef hadc1;
int adc_device,gpioa_device,gpioc_device,dma_device,mock_overrun;
static int channel_index, selected, active, starts, polls, reads, stops, configs, fail_index, fail_stage;
static unsigned pins_a, pins_c;
enum { NONE, CONFIG_FAIL, START_FAIL, POLL_FAIL, STOP_FAIL };
void HAL_GPIO_Init(void *p, GPIO_InitTypeDef *g){assert(g->Mode==GPIO_MODE_ANALOG && g->Pull==GPIO_NOPULL);if(p==GPIOA) pins_a=g->Pin;else if(p==GPIOC)pins_c=g->Pin;else assert(0);}
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef*h,ADC_ChannelConfTypeDef*c){static const int exp[3]={2,12,13};assert(h==&hadc1&&!active&&channel_index<3);assert(c->Channel==(unsigned)exp[channel_index]);assert(c->Rank==1&&c->SamplingTime==84);configs++;if(fail_stage==CONFIG_FAIL && channel_index==fail_index)return HAL_ERROR;selected=c->Channel;return HAL_OK;}
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef*h){assert(h==&hadc1&&!active);starts++;if(fail_stage==START_FAIL && channel_index==fail_index)return HAL_ERROR;active=1;return HAL_OK;}
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef*h,uint32_t t){assert(h==&hadc1&&active&&t==IR_ADC_TIMEOUT_MS);polls++;return fail_stage==POLL_FAIL&&channel_index==fail_index?HAL_TIMEOUT:HAL_OK;}
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef*h){assert(h==&hadc1&&active);reads++;return (uint32_t)(selected+100);}
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef*h){assert(h==&hadc1);stops++;active=0;if(fail_stage==STOP_FAIL&&channel_index==fail_index)return HAL_ERROR;channel_index++;return HAL_OK;}
static void reset(void){channel_index=selected=active=starts=polls=reads=stops=configs=0;fail_index=-1;fail_stage=NONE;}
int main(void){IR_Sensor_Init();assert(pins_a==GPIO_PIN_2 && pins_c==(GPIO_PIN_2|GPIO_PIN_3));IR_RawScan out={{9,8,7}},old=out;assert(IR_ReadAll(NULL)==HAL_ERROR);reset();assert(IR_ReadAll(&out)==HAL_OK);assert(out.raw[0]==102&&out.raw[1]==112&&out.raw[2]==113);assert(configs==3&&starts==3&&polls==3&&reads==3&&stops==3&&!active);
for(int s=CONFIG_FAIL;s<=STOP_FAIL;s++)for(int i=0;i<3;i++){reset();out=old;fail_stage=s;fail_index=i;HAL_StatusTypeDef r=IR_ReadAll(&out);assert(r==(s==POLL_FAIL?HAL_TIMEOUT:HAL_ERROR));assert(memcmp(&out,&old,sizeof out)==0&&!active);assert(configs==i+1);assert(polls==i+(s==POLL_FAIL||s==STOP_FAIL));assert(reads==i+(s==STOP_FAIL));assert(stops==i+(s!=CONFIG_FAIL));}
puts("IR polling channels, sequencing, timeouts, failures and GPIO PASS");}
