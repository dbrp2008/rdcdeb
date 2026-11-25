#ifndef TESTS_DRIVE_PROBE_H
#define TESTS_DRIVE_PROBE_H
#include "drive_plan.h"
/* Host-only: Simulate pins a single route. Expect only guards callbacks;
 * the real F1 <-> F2 selector runs without any override. */
void DriveProbe_Simulate(DriveFunction function);
void DriveProbe_Expect(DriveFunction function);
unsigned DriveProbe_Cycles(void);
void DriveProbe_Check(unsigned expected_boots);
#endif
