#include "encoder_monitor.h"
#include "encoder_bench_config.h"
#include "main.h"

/* One ISR writer per rising-edge counter. Never filter, clear, or time edges here. */
static volatile uint32_t left_edges, right_edges;
void HAL_GPIO_EXTI_Callback(uint16_t pin) {
  if (pin == ELCP_J3_FB_Pin) ++left_edges;       /* PC8, not PB6 on EXTI9_5 */
  else if (pin == ELCP_J1_FB_Pin) ++right_edges; /* PB15 */
}

#define CHECK_U32(x) _Static_assert((uint64_t)(x) <= UINT32_MAX && (x) == (uint32_t)(x), \
                                    #x " must be a whole unsigned 32-bit value")
CHECK_U32(ENCODER_IDLE_SAMPLE_MS);
CHECK_U32(ENCODER_NET_WINDOW_MS);
_Static_assert(ENCODER_IDLE_SAMPLE_MS >= 1000u, "sample idle for at least one second");
_Static_assert(ENCODER_NET_WINDOW_MS >= 4u && ENCODER_NET_WINDOW_MS <= 1000u, "rate window must be 4..1000 ms");
#define IDLE_BLOCKS 5u
#define BLOCK_MS (ENCODER_IDLE_SAMPLE_MS / IDLE_BLOCKS + (ENCODER_IDLE_SAMPLE_MS % IDLE_BLOCKS != 0u))
#define PROBE_MS ((ENCODER_NET_WINDOW_MS + 3u) / 4u)
#define MODERATE_PROBES 3u
#define CONFIRM_PROBES 6u
#define CONFIRM_WINDOWS 3u

typedef struct { uint32_t at, left, right; } Snapshot;
/* Mean plus the outward-rounded envelope of real NET-window rates. Never
 * infer subwindows from a delayed poll: jitter within such a gap is unknown. */
typedef struct { uint32_t edges, ms, low, high; } IdleBlock;
typedef struct { IdleBlock building, history[IDLE_BLOCKS]; unsigned next, used; } Learner;
typedef struct { uint8_t votes; bool high; } Evidence;
typedef struct { uint64_t held; uint8_t windows, votes; bool high; } Rise;
typedef struct {
  uint32_t raw_pps, idle_pps, deadzone_pps;
  int64_t net_pps, milli_edges;
  Learner learner;
  Evidence probe;       /* current rate window; probes never credit edges */
  Rise rise;            /* deferred positive credit, independent per wheel */
} Wheel;
static Snapshot rate_from, probe_from;
static Wheel j3, j1;

static Snapshot snapshot(void) {
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  const Snapshot s = {HAL_GetTick(), left_edges, right_edges};
  __set_PRIMASK(mask);
  return s;
}
static uint32_t capped(uint64_t rate) { return rate > UINT32_MAX ? UINT32_MAX : (uint32_t)rate; }
static uint64_t scaled(uint32_t edges) { return (uint64_t)edges * 1000u; }
static uint64_t distance(uint64_t a, uint64_t b) { return a > b ? a - b : b - a; }

static uint32_t larger(uint32_t a, uint32_t b) { return a > b ? a : b; }
static uint32_t ceil_div(uint32_t n, uint32_t d) { return n / d + (n % d != 0u); }
static uint32_t excursion(const IdleBlock *b, uint32_t center) {
  return larger(center > b->low ? center - b->low : 0u, b->high > center ? b->high - center : 0u);
}

/* Once full, h->next is the oldest complete block. */
static const IdleBlock *block_at(const Learner *h, unsigned i) {
  return &h->history[(h->next + i) % IDLE_BLOCKS];
}

static uint32_t median_of_five(const uint32_t values[IDLE_BLOCKS]) {
  uint32_t sorted[IDLE_BLOCKS];
  for (unsigned i = 0; i < IDLE_BLOCKS; ++i) {
    unsigned j = i;
    while (j && sorted[j-1] > values[i]) { sorted[j] = sorted[j-1]; --j; }
    sorted[j] = values[i];
  }
  return sorted[IDLE_BLOCKS / 2u];
}

/* Guard against both observed window jitter and edge quantization around a
 * block mean that is slightly different from the fitted whole-PPS idle. */
static uint32_t block_band(const IdleBlock *b, uint32_t idle, uint32_t quantum) {
  const uint64_t err = distance(scaled(b->edges), (uint64_t)idle * b->ms);
  const uint32_t resolution = capped(quantum + (err + b->ms - 1u) / b->ms);
  return larger(resolution, excursion(b, idle));
}

/* Relative eligibility limits stop an unstable signal from declaring its
 * own arbitrarily wide deadzone. Neither limit is a motion threshold. */
static void try_lock(Wheel *w) {
  const Learner *h = &w->learner;
  uint32_t centers[IDLE_BLOCKS], diffs[IDLE_BLOCKS];
  for (unsigned i = 0; i < IDLE_BLOCKS; ++i) {
    const IdleBlock *b = block_at(h, i);
    centers[i] = (uint32_t)((scaled(b->edges) + b->ms / 2u) / b->ms);
  }
  const uint32_t median = median_of_five(centers);
  if (!median) return;
  for (unsigned i = 0; i < IDLE_BLOCKS; ++i) diffs[i] = (uint32_t)distance(centers[i], median);
  /* A double can spill a few edges into an otherwise ordinary adjacent
   * block. Compare its center to the observed scatter as well as the rails. */
  const uint32_t center_quant = 2u * ceil_div(1000u, BLOCK_MS);
  const uint32_t fit_limit = larger(center_quant, capped(2ULL * median_of_five(diffs)));
  const uint32_t quantum = ceil_div(1000u, ENCODER_NET_WINDOW_MS);
  const uint32_t spread_limit = larger(2u * quantum, ceil_div(median, 16u));
  const uint32_t drift_limit = larger(center_quant, ceil_div(median, 64u));
  uint64_t edges = 0, elapsed = 0;
  unsigned kept = 0, mask = 0;
  uint32_t first = 0, last = 0;
  for (unsigned i = 0; i < IDLE_BLOCKS; ++i) {
    const IdleBlock *b = block_at(h, i);
    if (diffs[i] > fit_limit || diffs[i] > drift_limit || excursion(b, median) > spread_limit) continue;
    if (!kept) first = centers[i];
    last = centers[i]; ++kept; mask |= 1u << i;
    edges += b->edges; elapsed += b->ms;
  }
  /* A stale majority cannot lock a new step or a now-disconnected input.
   * Require support near the beginning AND two recent clean blocks, without drift. */
  const unsigned latest_two = 3u << (IDLE_BLOCKS - 2u);
  if (kept <= IDLE_BLOCKS / 2u || !(mask & 3u) || (mask & latest_two) != latest_two ||
      distance(first, last) > drift_limit) return;
  const uint32_t idle = (uint32_t)((edges * 1000u + elapsed / 2u) / elapsed);
  if (!idle) return;
  /* Second-largest accepted envelope: one lone peak cannot set the band. */
  uint32_t largest = 0, repeated = 0;
  for (unsigned i = 0; i < IDLE_BLOCKS; ++i) if (mask & (1u << i)) {
    const uint32_t peak = block_band(block_at(h, i), idle, quantum);
    if (peak >= largest) { repeated = largest; largest = peak; }
    else repeated = larger(repeated, peak);
  }
  w->idle_pps = idle;
  w->deadzone_pps = repeated;
}

static void learn_window(Wheel *w, uint32_t edges, uint32_t ms) {
  Learner *h = &w->learner;
  IdleBlock *b = &h->building;
  const uint64_t obs = scaled(edges);
  /* Missing carrier, an unobserved whole window or unrepresentable input:
   * require fresh support on THIS wheel; never re-use pre-gap evidence. */
  if (!edges || ms >= 2u * ENCODER_NET_WINDOW_MS ||
      edges > UINT32_MAX - b->edges || obs > (uint64_t)UINT32_MAX * ms) {
    *h = (Learner){0};
    return;
  }
  const uint32_t low = (uint32_t)(obs / ms), high = (uint32_t)((obs + ms - 1u) / ms);
  if (!b->ms || low < b->low) b->low = low;
  b->high = larger(b->high, high);
  b->edges += edges;
  b->ms += ms; /* before this window: < BLOCK_MS; this window: <= 2000 ms */
  if (b->ms < BLOCK_MS) return;
  h->history[h->next] = *b; *b = (IdleBlock){0};
  h->next = (h->next + 1u) % IDLE_BLOCKS;
  if (h->used < IDLE_BLOCKS) ++h->used;
  if (h->used == IDLE_BLOCKS) try_lock(w);
}

void EncoderMonitor_Init(void) {
  rate_from = probe_from = snapshot();
  j3 = j1 = (Wheel){0}; /* zero idle means auto-acquire, never zero as a fallback */
}
static void rearm(Wheel *w) {
  *w = (Wheel){.raw_pps = w->raw_pps, .milli_edges = w->milli_edges};
}
bool EncoderMonitor_StartIdleSample(void) {
  if (!j3.idle_pps && !j1.idle_pps) return false; /* both already looking */
  rate_from = probe_from = snapshot();
  rearm(&j3); rearm(&j1);
  return true;
}

/* A regular idle stream can fall either side of a short bin boundary by one
 * edge. Compare unrounded rates before deciding whether an observation votes. */
static bool outside_band(uint64_t deviation, uint32_t ms, uint32_t band) {
  return deviation > (uint64_t)band * ms && (ms >= 1000u || deviation > 1000u);
}
static bool high_rise(uint64_t excess, uint64_t expected) {
  return excess >= expected / 2u + expected % 2u; /* raw >= 1.5 x idle, without overflow */
}
static void observe(Wheel *w, uint32_t edges, uint32_t ms) {
  const uint64_t obs = scaled(edges), expected = (uint64_t)w->idle_pps * ms;
  if (!expected || obs <= expected || !outside_band(obs - expected, ms, w->deadzone_pps)) return;
  if (w->probe.votes < CONFIRM_PROBES) ++w->probe.votes;
  if (high_rise(obs - expected, expected)) w->probe.high = true;
}

/* A doubled burst lasting <= one normal rate window can touch two rate
 * windows, up to five quarter-window probes, but only two partial probes.
 * Three clearly moderate probes (no high probe) support ordinary motion now;
 * six positive probes or three successive positive windows confirm a rise.
 * Never turn one long delayed read into multiple votes. */
static bool accept_rise(Wheel *w, bool high, uint64_t *credit) {
  Rise *r = &w->rise;
  if (!high && w->probe.votes >= MODERATE_PROBES) {
    if (!r->high) *credit += r->held; /* include a mixed onset, not a preceding high spike */
    *r = (Rise){0};
    return true;
  }
  if (high && !r->high && r->windows == CONFIRM_WINDOWS) *r = (Rise){0}; /* a new high step */
  r->high |= high;
  if (r->windows < CONFIRM_WINDOWS) ++r->windows;
  if (r->votes < CONFIRM_PROBES) r->votes += w->probe.votes;
  if (r->windows < CONFIRM_WINDOWS && r->votes < CONFIRM_PROBES) {
    r->held += *credit; /* at most two positive windows, each <= UINT32_MAX * 1000 */
    return false;
  }
  *credit += r->held; r->held = 0; r->windows = CONFIRM_WINDOWS;
  return true;
}
static void measure(Wheel *w, uint32_t edges, uint32_t ms) {
  const uint64_t obs = scaled(edges), expected = (uint64_t)w->idle_pps * ms;
  const bool below = obs < expected;
  const uint64_t deviation = distance(obs, expected);
  w->net_pps = 0;
  if (!edges || !w->idle_pps || !outside_band(deviation, ms, w->deadzone_pps)) { w->rise = (Rise){0}; return; }
  uint64_t credit = deviation;
  if (below) w->rise = (Rise){0}; /* a reversal never releases a positive glitch */
  else if (!accept_rise(w, w->probe.high || high_rise(deviation, expected), &credit)) return;

  const int64_t sign = below ? -1 : 1;
  w->net_pps = sign * (int64_t)capped(deviation / ms); /* only this window, not the deferred count */
  const int64_t limit = (int64_t)UINT32_MAX * 1000;
  const uint64_t room = (uint64_t)(below ? limit + w->milli_edges : limit - w->milli_edges);
  w->milli_edges += sign * (int64_t)(credit < room ? credit : room);
}
static void close_window(Wheel *w, uint32_t edges, uint32_t ms) {
  w->raw_pps = capped(scaled(edges) / ms);
  if (w->idle_pps) measure(w, edges, ms);
  else learn_window(w, edges, ms); /* even the window that locks is never motion */
  w->probe = (Evidence){0};
}
static void advance(Snapshot now) {
  const uint32_t probe_ms = now.at - probe_from.at, ms = now.at - rate_from.at;
  if ((j3.idle_pps || j1.idle_pps) && probe_ms >= PROBE_MS) {
    observe(&j3, now.left - probe_from.left, probe_ms);
    observe(&j1, now.right - probe_from.right, probe_ms);
    probe_from = now;
  }
  if (ms >= ENCODER_NET_WINDOW_MS) {
    close_window(&j3, now.left - rate_from.left, ms);
    close_window(&j1, now.right - rate_from.right, ms);
    rate_from = probe_from = now;
  }
}
void EncoderMonitor_Poll(uint32_t now_ms) {
  if (now_ms - rate_from.at >= ENCODER_NET_WINDOW_MS ||
      ((j3.idle_pps || j1.idle_pps) && now_ms - probe_from.at >= PROBE_MS)) advance(snapshot());
}
static EncoderWheel reading(const Wheel *w, uint32_t edges) {
  return (EncoderWheel){.edges = edges, .pps = w->raw_pps, .idle_pps = w->idle_pps,
                        .deadzone_pps = w->deadzone_pps, .net_edges = w->milli_edges / 1000,
                        .net_pps = w->net_pps, .rise_pending = w->rise.held != 0};
}
EncoderReading EncoderMonitor_Read(void) {
  const Snapshot now = snapshot();
  advance(now);
  const EncoderMode mode = j3.idle_pps && j1.idle_pps ? ENCODER_MEASURING :
                           j3.idle_pps || j1.idle_pps ? ENCODER_PARTIAL : ENCODER_SETTLING;
  return (EncoderReading){.mode = mode, .left_j3 = reading(&j3, now.left),
                          .right_j1 = reading(&j1, now.right)};
}
