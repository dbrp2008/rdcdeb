#ifndef FUNCTION2_H
#define FUNCTION2_H
#include "branch_latch.h"
#include "line_drive.h"
#include <stdbool.h>
#include <stdint.h>

/* F2 counts only the digital side pair. Filtered analog tape controls line
 * reacquisition, never BR. Motor polarity and interlocks remain on the board. */
#define F2_TURN_AT_BRANCH 3u
/* Branch pivot, physical left forward : right reverse = 5:3 (warmaster 4566284: forward 15.75%, reverse 9.45%) */
#define F2_PIVOT_UNIT_PCT    F12_DUTY(3.5f)
#define F2_LEFT_FORWARD_PCT  (5.0f * F2_PIVOT_UNIT_PCT)
#define F2_RIGHT_REVERSE_PCT (3.0f * F2_PIVOT_UNIT_PCT)

/* (not F1_LEGACY) Encoder window on the branch pivot, which still ends on the tape: tape seen before F2_TURN_MIN_DEG is
 * ignored (a crossing / the old line), and with no tape by F2_TURN_MAX_DEG the run stops ("F2 TURN NO TAPE") instead of
 * spinning. The angle comes from both wheels' pulses (F2_PivotAngle). Set F2_TURN_EXPECT_DEG to the real branch angle. */
#ifndef F2_TURN_EXPECT_DEG
#define F2_TURN_EXPECT_DEG 90.0f
#endif
#ifndef F2_TURN_MIN_DEG
#define F2_TURN_MIN_DEG (0.6f * F2_TURN_EXPECT_DEG)
#endif
#ifndef F2_TURN_MAX_DEG
#define F2_TURN_MAX_DEG (1.3f * F2_TURN_EXPECT_DEG)
#endif

/* (not F1_LEGACY) At the T (3rd crossing) drive this far straight past it before pivoting. Pivoting where the side IR
 * sees the vertical line leaves the IRs' circle tangent to it: the centre IR stays near the line, never sees white then
 * black, and the turn runs out of window (sim: 0 of 3). F2_T_OUT_ADVANCE_MM past, the centre IR crosses the line at
 * acos(1 - advance / d) into the pivot for IR-to-axle d: 73 deg at the measured ~112 mm (60 at 160, 78 at 100), inside
 * the 54..117 deg window. The same move the return uses at the T (F2_T_ADVANCE_MM). 0 = pivot at once (old). */
#ifndef F2_T_OUT_ADVANCE_MM
#define F2_T_OUT_ADVANCE_MM 80.0f
#endif

void F2_Init(void);
bool F2_Advancing(void);       /* driving straight past the T before the pivot (the board holds the wheels equal) */
void F2_Advanced(float mm);    /* while advancing: mm driven since the T was seen; starts the pivot when far enough */
/* Time backups (a dead encoder never ends the advance or trips the pivot's angle limit): sized for a slow AR, see
 * function1.h. On expiry F2_Failed() and the status says which. */
#ifndef F2_ADVANCE_MAX_MS
#define F2_ADVANCE_MAX_MS 3000u /* past the T (F2_T_OUT_ADVANCE_MM): 27 mm/s minimum */
#endif
#ifndef F2_TURN_MAX_MS
#define F2_TURN_MAX_MS 6000u    /* the right pivot onto the branch (max window 117 deg): ~20 deg/s minimum */
#endif
void F2_Tick(uint32_t now);    /* every drive tick while F2 runs: fails the run if the advance or pivot overruns */
/* While turning: the pivot angle so far from the encoders (degrees; < 0 = unknown, the window is then not used). */
void F2_PivotAngle(float deg);
bool F2_Failed(void); /* stopped by the window (no tape by F2_TURN_MAX_DEG) */
void F2_Start(bool either_side_high);
void F2_Stop(const char *reason);
void F2_Update(BranchSample side, unsigned filtered_line); /* fresh IR scan */
/* Call after writing both pivot commands. Idempotent; a turn requested in
 * launch cannot use pre-pivot IR as evidence that the centre left the tape. */
void F2_PivotCommanded(void);
unsigned F2_BranchCount(void);
bool F2_Turning(void); /* pivot requested, not yet completed; board still checks interlocks */
bool F2_Done(void);
const char *F2_Status(void);
#endif
