#include "ar_nav.h"
#include <assert.h>
#include <stdio.h>
int main(void){AR_Nav n; AR_Nav_Init(&n);AR_Nav_SetKey(&n,AR_KEY_B);AR_Nav_Start(&n,0);
AR_Line line={0,1,0}, lost={0,0,0}; AR_Nav_Step(&n,2001,line,500,true); assert(n.phase==AR_PH_OUT);
AR_Nav_Step(&n,2010,line,90,true);assert(n.us_close==1);AR_Nav_Step(&n,2070,line,-1,true);assert(n.us_close==0&&n.phase==AR_PH_OUT);
AR_Nav_Step(&n,2080,lost,500,true);AR_Cmd c=AR_Nav_Step(&n,2080+AR_END_LOST_MS,lost,500,true);assert(n.phase==AR_PH_FAULT&&n.fault==AR_FAULT_LOST&&c.left==0&&c.right==0);
puts("Navigation rejects stale-close confirmation and faults on lost key line PASS");}
