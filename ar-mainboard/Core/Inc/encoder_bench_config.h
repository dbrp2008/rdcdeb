#ifndef ENCODER_BENCH_CONFIG_H
#define ENCODER_BENCH_CONFIG_H

/* Passive EncoderBench only. Idle PPS and the symmetric deadzone are BOTH
 * measured independently on each wheel. Neither has a preset or fallback.
 * Five real blocks spanning at least this long must show a stable carrier. */
#ifndef ENCODER_IDLE_SAMPLE_MS
#define ENCODER_IDLE_SAMPLE_MS 3000u
#endif
/* RAW/s and NET/s use actual elapsed time. The learner observes jitter in
 * these same windows; four shorter probes check live rises for transients. */
#ifndef ENCODER_NET_WINDOW_MS
#define ENCODER_NET_WINDOW_MS 200u
#endif

#endif
