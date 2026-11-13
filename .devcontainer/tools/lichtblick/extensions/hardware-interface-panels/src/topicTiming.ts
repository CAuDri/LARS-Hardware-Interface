// Keep timestamps and periods only; never retain ROS message payloads.
export type TimingSample = { at: number; period?: number };
export type TimingSummary = {
  hz?: number; age?: number; mean?: number; min?: number; max?: number;
  jitter?: number; p95?: number; stale: boolean; gaps: number;
};
export class TopicTiming {
  samples: TimingSample[] = [];
  count = 0;
  lastArrival?: number;
  private lastStamp?: number;
  missingStamps = 0;
  clockResets = 0;
  truncated = false;
  constructor(private readonly capacity = 5000) {}

  add(at: number, stamp: number | undefined): void {
    this.count++;
    this.lastArrival = at;
    let period: number | undefined;
    if (stamp == undefined || !Number.isFinite(stamp)) {
      this.missingStamps++;
      this.lastStamp = undefined;
    } else {
      if (this.lastStamp != undefined) {
        if (stamp >= this.lastStamp) period = stamp - this.lastStamp;
        else this.clockResets++;
      }
      this.lastStamp = stamp;
    }
    this.samples.push({ at, period });
    if (this.samples.length > this.capacity) {
      this.samples.splice(0, Math.max(1, Math.floor(this.capacity / 4)));
      this.truncated = true;
    }
  }

  prune(cutoff: number): void {
    const first = this.samples.findIndex((sample) => sample.at >= cutoff);
    if (first < 0) this.samples = [];
    else if (first > 0) this.samples.splice(0, first);
  }

  rate(now: number, windowMs: number, staleMs: number): Pick<TimingSummary, "hz" | "age" | "stale"> {
    const age = this.lastArrival == undefined ? undefined : Math.max(0, now - this.lastArrival);
    const stale = age != undefined && age >= staleMs;
    const recent = this.samples.filter((sample) => sample.at >= now - windowMs && sample.at <= now);
    const first = recent[0];
    // Include the current quiet interval, so the rate decays before the stale timeout.
    const duration = first ? now - first.at : 0;
    const hz = age == undefined ? undefined : stale || recent.length === 0 ? 0
      : recent.length > 1 && duration > 0 ? (recent.length - 1) * 1000 / duration : undefined;
    return { hz, age, stale };
  }

  summarize(now: number, windowMs: number, staleMs: number, expectedMs = 0): TimingSummary {
    const { hz, age, stale } = this.rate(now, windowMs, staleMs);
    const recent = this.samples.filter((sample) => sample.at >= now - windowMs && sample.at <= now);
    const periods = recent.flatMap((sample) => sample.period == undefined ? [] : [sample.period]);
    if (periods.length === 0) return { hz, age, stale, gaps: 0 };
    periods.sort((a, b) => a - b);
    const mean = periods.reduce((sum, value) => sum + value, 0) / periods.length;
    const median = periods[Math.floor(periods.length / 2)] ?? 0;
    const baseline = expectedMs > 0 ? expectedMs : median;
    return { hz, age, stale, mean, min: periods[0], max: periods[periods.length - 1],
      jitter: Math.sqrt(periods.reduce((sum, value) => sum + (value - mean) ** 2, 0) / periods.length),
      p95: periods[Math.max(0, Math.ceil(periods.length * 0.95) - 1)],
      gaps: baseline > 0 ? periods.filter((period) => period > baseline * 1.5).length : 0 };
  }
}

// Preserve minima/maxima when reducing graph points, so short spikes remain visible.
export function envelope(samples: readonly TimingSample[], maxPoints = 800): TimingSample[] {
  if (samples.length <= maxPoints) return [...samples];
  const size = Math.ceil(samples.length / Math.max(1, Math.floor(maxPoints / 2)));
  const result: TimingSample[] = [];
  for (let i = 0; i < samples.length; i += size) {
    const bucket = samples.slice(i, i + size);
    let low = bucket[0]!; let high = low;
    for (const sample of bucket) {
      if ((sample.period ?? 0) < (low.period ?? 0)) low = sample;
      if ((sample.period ?? 0) > (high.period ?? 0)) high = sample;
    }
    if (low === high) result.push(low);
    else if (bucket.indexOf(low) < bucket.indexOf(high)) result.push(low, high);
    else result.push(high, low);
  }
  return result;
}
