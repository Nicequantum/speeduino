#include "proposal.h"
#include <math.h>
#include <string.h>

namespace grok_pilot {

Rc decode(const uint8_t* p, size_t len, WireProposal* w) {
  if (p == nullptr || len != kProposalFrameLen) return Rc::BAD_LENGTH;
  if (p[0] != kProposalCommand) return Rc::BAD_COMMAND;
  if (p[1] != kProposalVersion) return Rc::BAD_VERSION;
  uint32_t crc = rd32(p + 22);
  if (crc != crc32_ieee(p, 22)) return Rc::BAD_CRC;
  if (p[7] != 0) return Rc::BAD_RESERVED;
  w->command = p[0]; w->version = p[1]; w->engine_id = p[2]; w->table_code = p[3];
  w->rpm_idx = p[4]; w->map_idx = p[5]; w->before_raw = p[6]; w->flags = p[7];
  uint32_t bits = rd32(p + 8);
  memcpy(&w->value_proposed, &bits, sizeof(float));   // IEEE-754 binary32, little-endian on the wire
  w->ts_ms = rd32(p + 12);
  w->evidence = rd16(p + 16);
  w->proposal_id = rd32(p + 18);
  w->crc = crc;
  return Rc::PROCESSED;
}

uint8_t table_page(int table_code) {
  return table_code == 0 ? kPageVE : table_code == 1 ? kPageIgn : kPageAFR;
}

double raw_to_units(int table_code, uint8_t raw) {
  if (table_code == 1) return (double)raw - kSparkOffset;   // spark: 1 deg/bit, -40 offset
  if (table_code == 2) return (double)raw / 10.0;            // AFR: 0.1/bit
  return (double)raw;                                        // VE: 1 %/bit
}

uint8_t units_to_raw_toward(int table_code, double units, uint8_t old_raw) {
  double x = table_code == 1 ? units + kSparkOffset : table_code == 2 ? units * 10.0 : units;
  const double eps = 1e-6;  // absorbs binary FP error (e.g. 14.7*10 = 146.99999...)
  double r;
  if (x > (double)old_raw) r = floor(x + eps);
  else if (x < (double)old_raw) r = ceil(x - eps);
  else r = old_raw;
  if ((x > old_raw && r < old_raw) || (x < old_raw && r > old_raw)) r = old_raw;  // never overshoot past old
  if (r < 0) r = 0;
  if (r > 255) r = 255;
  return (uint8_t)r;
}

Outcome ProposalHandler::handle(const uint8_t* payload, size_t len, uint32_t now_ms, bool eeprom_busy) {
  Outcome o = {Rc::PROCESSED, Decision::REJECTED, Reason::INVALID_VALUE, false, 0, 0, 0};
  WireProposal w;
  memset(&w, 0, sizeof w);
  Rc rc = decode(payload, len, &w);
  if (rc != Rc::PROCESSED) {
    o.rc = rc;
    log(o, nullptr, now_ms);
    return o;
  }
  o.proposal_id = w.proposal_id;
  if (eeprom_busy) {
    o.rc = Rc::BUSY_EEPROM;
    log(o, &w, now_ms);
    return o;
  }
  const bool cell_ok = w.table_code <= 2 && w.rpm_idx < 16 && w.map_idx < 16;
  if (cell_ok) {
    // Sync the envelope's view from the live table, so writes from any path
    // (TunerStudio 'M'/'W', EEPROM load) are visible and STALE_BASIS is real.
    uint8_t cur = tables_.get(tables_.ctx, table_page(w.table_code), cell_offset(w.rpm_idx, w.map_idx));
    o.old_raw = o.new_raw = cur;
    env_.set_cell(static_cast<Table>(w.table_code), w.rpm_idx, w.map_idx, raw_to_units(w.table_code, cur));
  }
  Proposal p;
  p.engine_id = w.engine_id;
  p.table_code = w.table_code;
  p.rpm_idx = w.rpm_idx;
  p.map_idx = w.map_idx;
  p.value_before = cell_ok ? raw_to_units(w.table_code, w.before_raw) : (double)w.before_raw;
  p.value_proposed = (double)w.value_proposed;
  p.timestamp_s = w.ts_ms / 1000.0;
  p.evidence_samples = w.evidence;
  p.proposal_id = w.proposal_id;
  Result r = env_.evaluate(p, now_ms / 1000.0);
  o.decision = r.decision;
  o.reason = r.reason;
  if (r.applied() && cell_ok) {
    uint8_t nr = units_to_raw_toward(w.table_code, r.value_accepted, o.old_raw);
    o.new_raw = nr;
    if (nr != o.old_raw) {
      tables_.set(tables_.ctx, table_page(w.table_code), cell_offset(w.rpm_idx, w.map_idx), nr);
      o.wrote = true;
    }
    // Keep the envelope's view equal to what's actually in the table.
    env_.set_cell(static_cast<Table>(w.table_code), w.rpm_idx, w.map_idx, raw_to_units(w.table_code, nr));
  }
  log(o, &w, now_ms);
  return o;
}

void ProposalHandler::encode_response(const Outcome& o, uint32_t now_ms, uint8_t out[kResponseLen]) {
  out[0] = static_cast<uint8_t>(o.rc);
  out[1] = static_cast<uint8_t>(o.decision);
  out[2] = static_cast<uint8_t>(o.reason);
  out[3] = o.wrote ? 1 : 0;
  out[4] = o.old_raw;
  out[5] = o.new_raw;
  wr32(out + 6, o.proposal_id);
  wr32(out + 10, now_ms);
}

void ProposalHandler::log(const Outcome& o, const WireProposal* w, uint32_t now_ms) {
  LogRecord& L = log_[head_];
  memset(&L, 0, sizeof L);
  L.proposal_id = o.proposal_id;
  L.ecu_ms = now_ms;
  L.rc = static_cast<uint8_t>(o.rc);
  L.decision = static_cast<uint8_t>(o.decision);
  L.reason = static_cast<uint8_t>(o.reason);
  L.wrote = o.wrote;
  L.old_raw = o.old_raw;
  L.new_raw = o.new_raw;
  if (w) {
    L.engine_id = w->engine_id; L.table_code = w->table_code; L.rpm_idx = w->rpm_idx; L.map_idx = w->map_idx;
    L.value_proposed = w->value_proposed;
  }
  head_ = (head_ + 1) % kLogCapacity;
  if (count_ < kLogCapacity) ++count_;
}

}  // namespace grok_pilot
