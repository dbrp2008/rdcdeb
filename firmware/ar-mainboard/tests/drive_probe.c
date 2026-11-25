/* GNU host-linker probes. Not part of any firmware target. An inactive
 * callback (including Init, Stop, a status read, or a scan) fails the test. */
#include "drive_probe.h"
#include "function2.h"
#include "function3.h"
#include <assert.h>

static DriveFunction expected = DRIVE_F1;
static unsigned boots, cycles, f2_calls, f3_calls;
static bool pinned;

void DriveProbe_Simulate(DriveFunction function) {
  assert(boots == 0); /* only synthetic host runs override the boot plan */
  pinned = true; expected = function;
}
/* Guard the allowed callbacks; this does NOT select a route on the board. */
void DriveProbe_Expect(DriveFunction function) {
  assert(!pinned && function == expected);
}
DriveFunction __real_DrivePlan_InitialFunction(void);
DriveFunction __wrap_DrivePlan_InitialFunction(void) {
  ++boots;
  const DriveFunction production = __real_DrivePlan_InitialFunction();
  assert(production == DRIVE_F1);
  if (!pinned) expected = production;
  return expected;
}
DriveFunction __real_DrivePlan_NextFunction(DriveFunction selected);
DriveFunction __wrap_DrivePlan_NextFunction(DriveFunction selected) {
  assert(selected == expected);
  ++cycles;
  const DriveFunction next = __real_DrivePlan_NextFunction(selected);
  if (!pinned) {
    assert((selected == DRIVE_F1 && next == DRIVE_F2) || (selected == DRIVE_F2 && next == DRIVE_F1));
    expected = next; /* guard callbacks against the REAL menu result */
  }
  return pinned ? selected : next;
}
unsigned DriveProbe_Cycles(void) { return cycles; }
void DriveProbe_Check(unsigned expected_boots) {
  assert(boots > 0 && boots == expected_boots); /* never reselect on a button or sensor event */
  if (pinned) assert((f2_calls != 0) == (expected == DRIVE_F2));
  else assert(f2_calls > 0 && cycles > 0); /* real selector exercised both functions */
  assert((f3_calls != 0) == (expected == DRIVE_F3));
}

static void probe_f2(void) { assert(expected == DRIVE_F2); ++f2_calls; }
static void probe_f3(void) { assert(expected == DRIVE_F3); ++f3_calls; }
#define WATCH_VOID(probe, name, params, args) \
  void __real_##name params; \
  void __wrap_##name params { probe(); __real_##name args; }
#define WATCH_VALUE(probe, type, name, params, args) \
  type __real_##name params; \
  type __wrap_##name params { probe(); return __real_##name args; }
WATCH_VOID(probe_f2, F2_Init, (void), ())
WATCH_VOID(probe_f2, F2_Start, (bool branch), (branch))
WATCH_VOID(probe_f2, F2_Update, (BranchSample side, unsigned filtered), (side, filtered))
WATCH_VOID(probe_f2, F2_PivotCommanded, (void), ())
WATCH_VOID(probe_f2, F2_Stop, (const char *why), (why))
WATCH_VALUE(probe_f2, unsigned, F2_BranchCount, (void), ())
WATCH_VALUE(probe_f2, bool, F2_Turning, (void), ())
WATCH_VALUE(probe_f2, bool, F2_Done, (void), ())
WATCH_VALUE(probe_f2, const char *, F2_Status, (void), ())
WATCH_VOID(probe_f3, F3_Reset, (void), ())
WATCH_VOID(probe_f3, F3_Scan, (uint32_t t, unsigned raw, unsigned filtered), (t, raw, filtered))
WATCH_VALUE(probe_f3, F3_Command, F3_CommandAt, (uint32_t t), (t))
WATCH_VALUE(probe_f3, unsigned, F3_RightTurns, (void), ())
