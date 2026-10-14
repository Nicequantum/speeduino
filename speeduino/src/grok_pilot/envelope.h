// ---------------------------------------------------------------------------
// VENDORED FILE: do not edit here.
// Source:   https://github.com/Nicequantum/ahanu-ecu  firmware/envelope/envelope.h
// Revision: ahanu-ecu commit 942468a (branch feat/envelope-cpp, PR #12)
// SHA-256 of the upstream file (before this header was added):
//           df44d23b5167e467a0246a672c82dcd07d82bfeb8b46835b6d3fb578efb22531
// Why vendored: the fork has to build standalone with PlatformIO and Speeduino
// tooling, and it can't reach into its parent repo. ahanu-ecu stays the source
// of truth (Python reference + cross-check vectors). ahanu-ecu CI checks this
// copy against firmware/envelope/envelope.h. To update: copy the file and
// refresh this header.
// License: GPL-3.0 (same as Speeduino).
// ---------------------------------------------------------------------------
// Grok Pilot validation envelope: C++ port of src/grok_pilot/envelope.py.
//
// Teensy-friendly: header-only, no heap, no exceptions, no RTTI, no STL
// containers. All state is fixed-size and owned by the Envelope object.
// Deterministic: the same inputs give the same decisions. Arithmetic uses
// IEEE-754 double so results match the Python reference bit-for-bit. Build
// with -ffp-contract=off so the compiler cannot fuse multiply-add.
//
// Semantics mirror the Python reference exactly, including the order of
// checks. See docs/ENVELOPE-CPP-PORT.md for the mapping table. Any change
// here must also go into envelope.py and regenerate the shared vectors
// (tests/vectors/envelope_vectors.txt).
#pragma once

#include <math.h>
#include <stdint.h>

namespace grok_pilot {

enum class Table : uint8_t { VE = 0, SPARK = 1, AFR_TARGET = 2 };
static const int kNumTables = 3;
static const int kMaxBins = 16;        // Speeduino 3D tables are 16x16
static const int kMaxRateTouches = 4;  // compile-time ceiling for rate_limit_touches

enum class Decision : uint8_t { ACCEPTED = 0, CLAMPED = 1, REJECTED = 2 };

// Order and names match grok_pilot.envelope.Reason / DATA-SCHEMA v0.2.
enum class Reason : uint8_t {
  NONE = 0,
  FUEL_DELTA_EXCEEDED,
  SPARK_DELTA_EXCEEDED,
  SPARK_LIMIT,
  AFR_FLOOR,
  AFR_CEILING,
  VE_LIMIT,
  RATE_LIMITED,
  WIDEBAND_DISAGREE,     // reserved (set upstream by the cross-check)
  INSUFFICIENT_SAMPLES,
  STAGE_LOCKED,
  SENSOR_FAULT,          // reserved
  INVALID_VALUE,
  OFF_TABLE,
  WRONG_ENGINE,
  STALE_TIMESTAMP,
  STALE_BASIS,
  LEARNING_FROZEN,
  COUNT_
};

inline const char* reason_name(Reason r) {
  static const char* const names[] = {
      "none", "fuel_delta_exceeded", "spark_delta_exceeded", "spark_limit", "afr_floor", "afr_ceiling",
      "ve_limit", "rate_limited", "wideband_disagree", "insufficient_samples", "stage_locked", "sensor_fault",
      "invalid_value", "off_table", "wrong_engine", "stale_timestamp", "stale_basis", "learning_frozen"};
  unsigned i = static_cast<unsigned>(r);
  return i < static_cast<unsigned>(Reason::COUNT_) ? names[i] : "?";
}

inline const char* decision_name(Decision d) {
  return d == Decision::ACCEPTED ? "accepted" : d == Decision::CLAMPED ? "clamped" : "rejected";
}

// Defaults are identical to EnvelopeConfig in envelope.py.
struct EnvelopeConfig {
  int32_t engine_id = 0;
  int32_t rpm_bins = 16;
  int32_t map_bins = 16;
  double max_fuel_delta_pct = 5.0;
  double max_spark_delta_deg = 2.0;
  double spark_min_deg = 0.0;
  double spark_max_deg = 36.0;   // TODO: confirm for marinized 350 + Mallory/HEI (#9)
  double afr_floor = 10.5;
  double afr_ceiling = 16.0;
  double ve_min = 10.0;
  double ve_max = 150.0;
  int32_t min_evidence_samples = 50;
  int32_t rate_limit_touches = 3;   // must be <= kMaxRateTouches
  double rate_limit_window_s = 60.0;
  double max_age_s = 2.0;
  double max_future_skew_s = 0.5;
  bool clamp_deltas = true;
};

// table_code is an int (not Table) so out-of-range codes from the wire can be
// represented and rejected as OFF_TABLE, like an unknown table name in Python.
struct Proposal {
  int32_t engine_id;
  int32_t table_code;
  int32_t rpm_idx;
  int32_t map_idx;
  double value_before;
  double value_proposed;
  double timestamp_s;
  int32_t evidence_samples;
  uint32_t proposal_id;
};

struct Result {
  Decision decision;
  Reason reason;
  double value_before;
  double value_accepted;
  bool applied() const { return decision == Decision::ACCEPTED || decision == Decision::CLAMPED; }
};

// Optional stage lock, called as allowed(ctx, table_code, rpm_idx, map_idx).
typedef bool (*CellAllowedFn)(void* ctx, int32_t table_code, int32_t rpm_idx, int32_t map_idx);

class Envelope {
 public:
  explicit Envelope(const EnvelopeConfig& cfg) : frozen(false), cfg_(cfg) { clear(); }

  bool frozen;  // set by the wideband cross-check / curriculum abort

  const EnvelopeConfig& config() const { return cfg_; }

  // Returns false if the config can't be represented in fixed-size state.
  bool config_ok() const {
    return cfg_.rpm_bins > 0 && cfg_.rpm_bins <= kMaxBins && cfg_.map_bins > 0 && cfg_.map_bins <= kMaxBins &&
           cfg_.rate_limit_touches >= 0 && cfg_.rate_limit_touches <= kMaxRateTouches;
  }

  void clear() {
    for (int t = 0; t < kNumTables; ++t)
      for (int r = 0; r < kMaxBins; ++r)
        for (int m = 0; m < kMaxBins; ++m) {
          Cell& c = cells_[t][r][m];
          c.has_value = false;
          c.value = 0.0;
          c.has_last_ts = false;
          c.last_ts = 0.0;
          c.n_touches = 0;
          c.head = 0;
        }
  }

  void set_cell(Table t, int r, int m, double v) {
    if (in_range(static_cast<int32_t>(t), r, m)) {
      Cell& c = cells_[static_cast<int>(t)][r][m];
      c.value = v;
      c.has_value = true;
    }
  }

  bool get_cell(Table t, int r, int m, double* out) const {
    if (!in_range(static_cast<int32_t>(t), r, m)) return false;
    const Cell& c = cells_[static_cast<int>(t)][r][m];
    if (c.has_value && out) *out = c.value;
    return c.has_value;
  }

  Result evaluate(const Proposal& p, double now_s, CellAllowedFn allowed = nullptr, void* ctx = nullptr,
                  bool commit = true) {
    const EnvelopeConfig& c = cfg_;
    // 1. identity & structure
    if (p.engine_id != c.engine_id) return reject(p, Reason::WRONG_ENGINE);
    if (p.table_code < 0 || p.table_code >= kNumTables) return reject(p, Reason::OFF_TABLE);
    if (!(p.rpm_idx >= 0 && p.rpm_idx < c.rpm_bins && p.map_idx >= 0 && p.map_idx < c.map_bins) ||
        p.rpm_idx >= kMaxBins || p.map_idx >= kMaxBins)
      return reject(p, Reason::OFF_TABLE);
    if (!(isfinite(p.value_before) && isfinite(p.value_proposed) && isfinite(p.timestamp_s) && isfinite(now_s)))
      return reject(p, Reason::INVALID_VALUE);
    if (p.evidence_samples < 0) return reject(p, Reason::INVALID_VALUE);
    // 2. freshness
    if (now_s - p.timestamp_s > c.max_age_s || p.timestamp_s - now_s > c.max_future_skew_s)
      return reject(p, Reason::STALE_TIMESTAMP);
    Cell& cell = cells_[p.table_code][p.rpm_idx][p.map_idx];
    if (cell.has_last_ts && p.timestamp_s <= cell.last_ts) return reject(p, Reason::STALE_TIMESTAMP);
    if (cell.has_value && cell.value != p.value_before) return reject(p, Reason::STALE_BASIS);
    // 3. gates
    if (frozen) return reject(p, Reason::LEARNING_FROZEN);
    if (allowed && !allowed(ctx, p.table_code, p.rpm_idx, p.map_idx)) return reject(p, Reason::STAGE_LOCKED);
    if (p.evidence_samples < c.min_evidence_samples) return reject(p, Reason::INSUFFICIENT_SAMPLES);
    // 4. absolute limits (hard reject, never clamp)
    const double v = p.value_proposed, vb = p.value_before;
    const Table tb = static_cast<Table>(p.table_code);
    if (tb == Table::AFR_TARGET) {
      if (v < c.afr_floor) return reject(p, Reason::AFR_FLOOR);
      if (v > c.afr_ceiling) return reject(p, Reason::AFR_CEILING);
    } else if (tb == Table::SPARK) {
      if (!(c.spark_min_deg <= v && v <= c.spark_max_deg)) return reject(p, Reason::SPARK_LIMIT);
    } else {
      if (!(c.ve_min <= v && v <= c.ve_max) || vb <= 0) return reject(p, Reason::VE_LIMIT);
    }
    // 5. rate limit (ring buffer = Python deque; expire while ts - oldest >= window)
    while (cell.n_touches > 0 && p.timestamp_s - cell.touch[cell.head] >= c.rate_limit_window_s) {
      cell.head = static_cast<uint8_t>((cell.head + 1) % kMaxRateTouches);
      --cell.n_touches;
    }
    if (cell.n_touches >= c.rate_limit_touches) return reject(p, Reason::RATE_LIMITED);
    // 6. delta limits
    Decision decision = Decision::ACCEPTED;
    Reason reason = Reason::NONE;
    double out = v;
    bool has_lim = false;
    double lim = 0.0;
    Reason r = Reason::NONE;
    if (tb == Table::SPARK) {
      has_lim = true; lim = c.max_spark_delta_deg; r = Reason::SPARK_DELTA_EXCEEDED;
    } else if (tb == Table::VE) {
      has_lim = true; lim = fabs(vb) * c.max_fuel_delta_pct / 100.0; r = Reason::FUEL_DELTA_EXCEEDED;
    }
    if (has_lim && fabs(v - vb) > lim + 1e-9) {
      if (!c.clamp_deltas) return reject(p, r);
      out = vb + copysign(lim, v - vb);
      decision = Decision::CLAMPED;
      reason = r;
    }
    Result res = {decision, reason, vb, out};
    if (commit) {
      uint8_t tail = static_cast<uint8_t>((cell.head + cell.n_touches) % kMaxRateTouches);
      cell.touch[tail] = p.timestamp_s;
      ++cell.n_touches;
      cell.last_ts = p.timestamp_s;
      cell.has_last_ts = true;
      cell.value = out;
      cell.has_value = true;
    }
    return res;
  }

 private:
  struct Cell {
    double value;
    double last_ts;
    double touch[kMaxRateTouches];
    bool has_value;
    bool has_last_ts;
    uint8_t n_touches;
    uint8_t head;
  };

  bool in_range(int32_t t, int r, int m) const {
    return t >= 0 && t < kNumTables && r >= 0 && r < kMaxBins && m >= 0 && m < kMaxBins;
  }

  static Result reject(const Proposal& p, Reason reason) {
    double vb = isfinite(p.value_before) ? p.value_before : NAN;
    Result res = {Decision::REJECTED, reason, vb, vb};
    return res;
  }

  EnvelopeConfig cfg_;
  Cell cells_[kNumTables][kMaxBins][kMaxBins];
};

}  // namespace grok_pilot
