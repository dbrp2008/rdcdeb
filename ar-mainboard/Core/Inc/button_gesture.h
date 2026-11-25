#ifndef BUTTON_GESTURE_H
#define BUTTON_GESTURE_H
#include <stdbool.h>
#include <stdint.h>

/* Release recognizer shared by calibration and drive. B2's raw motor stop
 * is independent: the board never waits for this debounce to stop a run. */
#define GESTURE_DEBOUNCE_MS 30u
enum { KEY_B1 = 1u, KEY_B2 = 2u, KEY_BOTH = KEY_B1 | KEY_B2 };
typedef enum { GESTURES_OFF, GESTURES_CHORD_ONLY, GESTURES_ALL } GestureMode;
typedef struct {
  unsigned raw, seen, qualified;
  uint32_t changed;
  bool armed, chord_only;
} ButtonGesture;

/* Emit at most one key mask after both keys have been up for 30 ms. Any
 * sight of the other key cancels a single; only a stable overlap can cycle.
 * There is no deadline for adding the second key or releasing the first. */
static inline unsigned ButtonGesture_Release(ButtonGesture *g, uint32_t now,
                                             unsigned keys, GestureMode mode) {
  if (keys != g->raw) { g->raw = keys; g->changed = now; }
  const bool stable = (uint32_t)(now - g->changed) >= GESTURE_DEBOUNCE_MS;
  if (!g->armed || mode == GESTURES_OFF) {
    g->seen = g->qualified = 0u;
    g->chord_only = false;
    g->armed = mode != GESTURES_OFF && keys == 0u && stable;
    return 0u; /* consume boot-held keys and presses begun during a capture */
  }
  if (keys && mode == GESTURES_CHORD_ONLY) g->chord_only = true; /* never queue a mid-run B1 */
  g->seen |= keys;
  if (!stable) return 0u;
  if (keys) {
    if (keys == g->seen) g->qualified = keys; /* staggered release cannot downgrade a chord */
    return 0u;
  }
  const unsigned result = (g->qualified == g->seen && (!g->chord_only || g->seen == KEY_BOTH))
                          ? g->seen : 0u;
  g->seen = g->qualified = 0u;
  g->chord_only = false;
  return result;
}
#endif
