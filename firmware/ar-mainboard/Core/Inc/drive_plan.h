#ifndef DRIVE_PLAN_H
#define DRIVE_PLAN_H

typedef enum { DRIVE_DISABLED = 0, DRIVE_F1, DRIVE_F2, DRIVE_F3, DRIVE_F4 } DriveFunction;

/* Calibration selects F1. Only a released B1+B2 chord changes selection;
 * an unknown route fails closed. Starting or stopping never advances it. */
DriveFunction DrivePlan_InitialFunction(void);
DriveFunction DrivePlan_NextFunction(DriveFunction selected);
#endif
