// Grok Pilot AI map-write proposal handler (pure logic, no Arduino deps).
//
// Wire protocol: docs in ahanu-ecu docs/PROPOSAL-WIRE-PROTOCOL.md. This file:
//   decode + structural checks + inner CRC32  ->  sync envelope from the real
//   table  ->  Envelope::evaluate  ->  byte rounding toward old value  ->
//   write through TableAccess only if applied  ->  RAM ring-buffer log.
// It never burns to EEPROM. Persistence needs an explicit operator 'b'.
//
// No heap, exceptions or RTTI. All state is fixed-size inside ProposalHandler.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "envelope.h"

namespace grok_pilot {

static const uint8_t kProposalCommand = 'Y';   // unused in 202501 comms.cpp, comms_legacy.cpp and speeduino.ini
static const uint8_t kProposalVersion = 1;
static const size_t kProposalFrameLen = 26;    // payload bytes incl. command byte and inner CRC
static const size_t kResponseLen = 14;
static const int kLogCapacity = 64;

// Speeduino pages that hold the 16x16 tables (pages.h) and their byte encodings.
static const uint8_t kPageVE = 2, kPageIgn = 3, kPageAFR = 5;
static const int kSparkOffset = 40;            // OFFSET_IGNITION

// Response status byte (payload[0] of the reply).
enum class Rc : uint8_t {
  PROCESSED = 0x00,        // decision/reason fields are valid
  BAD_LENGTH = 0xA1,
  BAD_VERSION = 0xA2,
  BAD_CRC = 0xA3,
  BAD_RESERVED = 0xA4,
  BUSY_EEPROM = 0xA5,      // an EEPROM burn is in progress; retry later (prevents writeAllConfig persisting AI writes)
  BAD_COMMAND = 0xA6,
};

struct WireProposal {
  uint8_t command, version, engine_id, table_code, rpm_idx, map_idx, before_raw, flags;
  float value_proposed;
  uint32_t ts_ms;
  uint16_t evidence;
  uint32_t proposal_id;
  uint32_t crc;
};

// IEEE 802.3 CRC-32 (reflected, poly 0xEDB88320, init/xorout 0xFFFFFFFF), the
// same as zlib.crc32 and Speeduino's FastCRC crc32. Bitwise, so it needs no table RAM.
inline uint32_t crc32_ieee(const uint8_t* p, size_t n) {
  uint32_t c = 0xFFFFFFFFu;
  for (size_t i = 0; i < n; ++i) {
    c ^= p[i];
    for (int k = 0; k < 8; ++k) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
  }
  return ~c;
}

inline uint16_t rd16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
inline uint32_t rd32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
inline void wr32(uint8_t* p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }

// Structural decode. Range and semantic checks (engine, table, indices,
// NaN/inf, limits) are left to the envelope so reason codes match DATA-SCHEMA.
Rc decode(const uint8_t* payload, size_t len, WireProposal* out);

// Encoding helpers (table byte <-> engineering units).
double raw_to_units(int table_code, uint8_t raw);
// Round toward old_raw so the quantized change never exceeds the accepted change.
uint8_t units_to_raw_toward(int table_code, double units, uint8_t old_raw);

// Abstract table access so the handler can be unit-tested off-target.
struct TableAccess {
  void* ctx;
  uint8_t (*get)(void* ctx, uint8_t page, uint16_t offset);
  void (*set)(void* ctx, uint8_t page, uint16_t offset, uint8_t value);
};
uint8_t table_page(int table_code);
// TS page offset of a 16x16 table cell: values start at 0, row = load (map) bin, col = rpm bin.
// ⚠️ Orientation follows the TS ini [16x16] convention; verify on the bench (ahanu-ecu #11).
inline uint16_t cell_offset(int rpm_idx, int map_idx) { return (uint16_t)(map_idx * 16 + rpm_idx); }

struct LogRecord {
  uint32_t proposal_id;
  uint32_t ecu_ms;
  uint8_t rc, engine_id, table_code, rpm_idx, map_idx;
  uint8_t decision, reason, wrote, old_raw, new_raw;
  float value_proposed;
};

struct Outcome {
  Rc rc;
  Decision decision;
  Reason reason;
  bool wrote;
  uint8_t old_raw, new_raw;
  uint32_t proposal_id;
};

class ProposalHandler {
 public:
  ProposalHandler(const EnvelopeConfig& cfg, TableAccess tables) : env_(cfg), tables_(tables), head_(0), count_(0) {}

  Envelope& envelope() { return env_; }

  // payload = full new-protocol payload starting at the command byte.
  // now_ms = ECU millis(). eeprom_busy = isEepromWritePending().
  Outcome handle(const uint8_t* payload, size_t len, uint32_t now_ms, bool eeprom_busy);

  static void encode_response(const Outcome& o, uint32_t now_ms, uint8_t out[kResponseLen]);

  int log_count() const { return count_; }
  // i = 0 is the oldest record still held.
  const LogRecord& log_at(int i) const { return log_[(head_ + kLogCapacity - count_ + i) % kLogCapacity]; }

 private:
  void log(const Outcome& o, const WireProposal* w, uint32_t now_ms);
  Envelope env_;
  TableAccess tables_;
  LogRecord log_[kLogCapacity];
  int head_, count_;
};

}  // namespace grok_pilot
