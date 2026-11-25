#include "drive_plan.h"
#include "function1.h" /* F1_LEGACY */

DriveFunction DrivePlan_InitialFunction(void) { return DRIVE_F1; }

DriveFunction DrivePlan_NextFunction(DriveFunction selected) {
  switch (selected) {
    case DRIVE_F1: return DRIVE_F2;
#if F1_LEGACY
    case DRIVE_F2: return DRIVE_F1;
    default: return DRIVE_DISABLED; /* F3 and the passive bench are not menu entries */
#else
    case DRIVE_F2: return DRIVE_F3;
    case DRIVE_F3: return DRIVE_F4; /* F3 = the top-right key (F1_StartF3) */
    case DRIVE_F4: return DRIVE_F1; /* F4 = the bottom-left key below the 8 (F1_StartF4, sim only) */
    default: return DRIVE_DISABLED; /* the passive bench is not a menu entry */
#endif
  }
}
