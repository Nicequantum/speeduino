#include <gtest/gtest.h>
#include <math.h>
#include <string.h>
#include <limits>

#include "proposal.h"

using namespace grok_pilot;

namespace {
struct FakeTables {
  uint8_t pages[6][288];
  int sets = 0;
  FakeTables() {
    memset(pages, 0, sizeof pages);
    for (int i = 0; i < 256; ++i) { pages[kPageVE][i] = 100; pages[kPageIgn][i] = 60; pages[kPageAFR][i] = 147; }
  }
  static uint8_t get(void* c, uint8_t pg, uint16_t off) { return static_cast<FakeTables*>(c)->pages[pg][off]; }
  static void set(void* c, uint8_t pg, uint16_t off, uint8_t v) {
    auto* t = static_cast<FakeTables*>(c);
    t->pages[pg][off] = v;
    ++t->sets;
  }
  TableAccess access() { return TableAccess{this, &FakeTables::get, &FakeTables::set}; }
};

struct Frame {
  uint8_t b[kProposalFrameLen];
  Frame(uint8_t table, uint8_t r, uint8_t m, uint8_t before_raw, float proposed, uint32_t ts_ms,
        uint16_t evidence = 100, uint8_t engine = 0, uint32_t pid = 7) {
    memset(b, 0, sizeof b);
    b[0] = kProposalCommand; b[1] = kProposalVersion; b[2] = engine; b[3] = table; b[4] = r; b[5] = m;
    b[6] = before_raw; b[7] = 0;
    uint32_t bits; memcpy(&bits, &proposed, 4); wr32(b + 8, bits);
    wr32(b + 12, ts_ms);
    b[16] = evidence & 0xFF; b[17] = evidence >> 8;
    wr32(b + 18, pid);
    reseal();
  }
  void reseal() { wr32(b + 22, crc32_ieee(b, 22)); }
};

const uint32_t NOW = 100000;  // ECU ms
}  // namespace

TEST(Crc32, MatchesZlibCheckValue) {
  const char* s = "123456789";
  EXPECT_EQ(crc32_ieee(reinterpret_cast<const uint8_t*>(s), 9), 0xCBF43926u);
  EXPECT_EQ(crc32_ieee(nullptr, 0), 0u);
}

TEST(Decode, GoodFrame) {
  Frame f(0, 3, 4, 100, 103.5f, NOW, 250, 0, 42);
  WireProposal w;
  ASSERT_EQ(decode(f.b, sizeof f.b, &w), Rc::PROCESSED);
  EXPECT_EQ(w.rpm_idx, 3); EXPECT_EQ(w.map_idx, 4); EXPECT_EQ(w.before_raw, 100);
  EXPECT_FLOAT_EQ(w.value_proposed, 103.5f); EXPECT_EQ(w.evidence, 250); EXPECT_EQ(w.proposal_id, 42u);
  EXPECT_EQ(w.ts_ms, NOW);
}

TEST(Decode, StructuralRejects) {
  WireProposal w;
  Frame f(0, 3, 4, 100, 103.f, NOW);
  EXPECT_EQ(decode(f.b, 25, &w), Rc::BAD_LENGTH);
  EXPECT_EQ(decode(f.b, 27, &w), Rc::BAD_LENGTH);
  EXPECT_EQ(decode(nullptr, 26, &w), Rc::BAD_LENGTH);
  Frame c = f; c.b[0] = 'M'; c.reseal(); EXPECT_EQ(decode(c.b, 26, &w), Rc::BAD_COMMAND);
  Frame v = f; v.b[1] = 2; v.reseal(); EXPECT_EQ(decode(v.b, 26, &w), Rc::BAD_VERSION);
  Frame r = f; r.b[7] = 1; r.reseal(); EXPECT_EQ(decode(r.b, 26, &w), Rc::BAD_RESERVED);
  for (int i = 0; i < 26; ++i) {  // any single-bit flip is caught
    if (i == 0 || i == 1) continue;  // command/version checked before CRC
    Frame x = f; x.b[i] ^= 0x10;
    EXPECT_EQ(decode(x.b, 26, &w), Rc::BAD_CRC) << i;
  }
}

TEST(Rounding, TowardOldValue) {
  // VE 1 %/bit
  EXPECT_EQ(units_to_raw_toward(0, 104.9, 100), 104);
  EXPECT_EQ(units_to_raw_toward(0, 95.1, 100), 96);
  EXPECT_EQ(units_to_raw_toward(0, 101.5, 30), 101);
  EXPECT_EQ(units_to_raw_toward(0, 31.5, 30), 31);   // 5 % of 30 = 1.5 -> 1
  EXPECT_EQ(units_to_raw_toward(0, 100.0, 100), 100);
  EXPECT_EQ(units_to_raw_toward(0, 100.4, 100), 100);  // sub-bit increase -> no change
  // spark 1 deg/bit, -40 offset
  EXPECT_EQ(units_to_raw_toward(1, 21.7, 60), 61);   // 20 deg -> 21.7 = raw 61.7 -> 61
  EXPECT_EQ(units_to_raw_toward(1, 18.3, 60), 59);
  EXPECT_DOUBLE_EQ(raw_to_units(1, 60), 20.0);
  // AFR 0.1/bit, FP error must not lose a bit
  EXPECT_EQ(units_to_raw_toward(2, 14.7, 140), 147);
  EXPECT_EQ(units_to_raw_toward(2, 14.7, 150), 147);
  EXPECT_EQ(units_to_raw_toward(2, 14.75, 140), 147);
  EXPECT_EQ(units_to_raw_toward(2, 14.65, 150), 147);
  EXPECT_DOUBLE_EQ(raw_to_units(2, 147), 14.7);
}

TEST(Handler, AcceptWritesCellRamOnly) {
  FakeTables t;
  ProposalHandler h(EnvelopeConfig(), t.access());
  Frame f(0, 3, 4, 100, 103.0f, NOW);
  Outcome o = h.handle(f.b, 26, NOW, false);
  EXPECT_EQ(o.rc, Rc::PROCESSED);
  EXPECT_EQ(o.decision, Decision::ACCEPTED);
  EXPECT_TRUE(o.wrote);
  EXPECT_EQ(t.pages[kPageVE][cell_offset(3, 4)], 103);
  EXPECT_EQ(t.pages[kPageVE][cell_offset(4, 3)], 100);  // only the addressed cell
  EXPECT_EQ(t.sets, 1);
}

TEST(Handler, ClampedThenRoundedTowardOld) {
  FakeTables t;
  t.pages[kPageVE][cell_offset(1, 1)] = 30;
  ProposalHandler h(EnvelopeConfig(), t.access());
  Frame f(0, 1, 1, 30, 60.0f, NOW);  // clamp to 31.5 -> byte 31
  Outcome o = h.handle(f.b, 26, NOW, false);
  EXPECT_EQ(o.decision, Decision::CLAMPED);
  EXPECT_EQ(o.reason, Reason::FUEL_DELTA_EXCEEDED);
  EXPECT_EQ(o.new_raw, 31);
  EXPECT_EQ(t.pages[kPageVE][cell_offset(1, 1)], 31);
}

TEST(Handler, SparkAndAfrEncodings) {
  FakeTables t;
  ProposalHandler h(EnvelopeConfig(), t.access());
  Frame s(1, 2, 2, 60, 22.0f, NOW);  // 20 -> 22 deg
  EXPECT_TRUE(h.handle(s.b, 26, NOW, false).wrote);
  EXPECT_EQ(t.pages[kPageIgn][cell_offset(2, 2)], 62);
  Frame a(2, 2, 2, 147, 13.5f, NOW);  // AFR target 14.7 -> 13.5
  Outcome o = h.handle(a.b, 26, NOW, false);
  EXPECT_EQ(o.decision, Decision::ACCEPTED);
  EXPECT_EQ(t.pages[kPageAFR][cell_offset(2, 2)], 135);
}

TEST(Handler, SubBitChangeAcceptedButNotWritten) {
  FakeTables t;
  ProposalHandler h(EnvelopeConfig(), t.access());
  Frame f(0, 3, 4, 100, 100.4f, NOW);
  Outcome o = h.handle(f.b, 26, NOW, false);
  EXPECT_EQ(o.decision, Decision::ACCEPTED);
  EXPECT_FALSE(o.wrote);
  EXPECT_EQ(t.sets, 0);
}

struct RejectCase { const char* name; Frame f; uint32_t now; bool busy; Rc rc; Reason reason; };

TEST(Handler, RejectsLeaveCellUntouched) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  Frame badcrc(0, 3, 4, 100, 103.f, NOW); badcrc.b[10] ^= 1;
  RejectCase cases[] = {
      {"bad_crc", badcrc, NOW, false, Rc::BAD_CRC, Reason::INVALID_VALUE},
      {"busy", Frame(0, 3, 4, 100, 103.f, NOW), NOW, true, Rc::BUSY_EEPROM, Reason::INVALID_VALUE},
      {"wrong_engine", Frame(0, 3, 4, 100, 103.f, NOW, 100, 1), NOW, false, Rc::PROCESSED, Reason::WRONG_ENGINE},
      {"off_table_table", Frame(3, 3, 4, 100, 103.f, NOW), NOW, false, Rc::PROCESSED, Reason::OFF_TABLE},
      {"off_table_rpm", Frame(0, 16, 4, 100, 103.f, NOW), NOW, false, Rc::PROCESSED, Reason::OFF_TABLE},
      {"off_table_map", Frame(0, 3, 255, 100, 103.f, NOW), NOW, false, Rc::PROCESSED, Reason::OFF_TABLE},
      {"nan", Frame(0, 3, 4, 100, nan, NOW), NOW, false, Rc::PROCESSED, Reason::INVALID_VALUE},
      {"inf", Frame(0, 3, 4, 100, inf, NOW), NOW, false, Rc::PROCESSED, Reason::INVALID_VALUE},
      {"huge", Frame(0, 3, 4, 100, 3.0e38f, NOW), NOW, false, Rc::PROCESSED, Reason::VE_LIMIT},
      {"negative", Frame(0, 3, 4, 100, -5.f, NOW), NOW, false, Rc::PROCESSED, Reason::VE_LIMIT},
      {"stale", Frame(0, 3, 4, 100, 103.f, NOW - 2001), NOW, false, Rc::PROCESSED, Reason::STALE_TIMESTAMP},
      {"future", Frame(0, 3, 4, 100, 103.f, NOW + 501), NOW, false, Rc::PROCESSED, Reason::STALE_TIMESTAMP},
      {"stale_basis", Frame(0, 3, 4, 99, 103.f, NOW), NOW, false, Rc::PROCESSED, Reason::STALE_BASIS},
      {"evidence", Frame(0, 3, 4, 100, 103.f, NOW, 49), NOW, false, Rc::PROCESSED, Reason::INSUFFICIENT_SAMPLES},
      {"spark_limit", Frame(1, 3, 4, 60, 37.f, NOW), NOW, false, Rc::PROCESSED, Reason::SPARK_LIMIT},
      {"afr_floor", Frame(2, 3, 4, 147, 10.4f, NOW), NOW, false, Rc::PROCESSED, Reason::AFR_FLOOR},
      {"afr_ceiling", Frame(2, 3, 4, 147, 16.1f, NOW), NOW, false, Rc::PROCESSED, Reason::AFR_CEILING},
  };
  for (auto& c : cases) {
    FakeTables t;
    FakeTables before = t;
    ProposalHandler h(EnvelopeConfig(), t.access());
    Outcome o = h.handle(c.f.b, 26, c.now, c.busy);
    EXPECT_EQ(o.rc, c.rc) << c.name;
    EXPECT_EQ(o.decision, Decision::REJECTED) << c.name;
    if (c.rc == Rc::PROCESSED) { EXPECT_EQ(o.reason, c.reason) << c.name; }
    EXPECT_FALSE(o.wrote) << c.name;
    EXPECT_EQ(t.sets, 0) << c.name;
    EXPECT_EQ(memcmp(t.pages, before.pages, sizeof t.pages), 0) << c.name;
    EXPECT_EQ(h.log_count(), 1) << c.name;
  }
}

TEST(Handler, FrozenRejects) {
  FakeTables t;
  ProposalHandler h(EnvelopeConfig(), t.access());
  h.envelope().frozen = true;
  Frame f(0, 3, 4, 100, 103.f, NOW);
  EXPECT_EQ(h.handle(f.b, 26, NOW, false).reason, Reason::LEARNING_FROZEN);
  EXPECT_EQ(t.sets, 0);
}

TEST(Handler, SyncsWithOtherWritePaths) {
  FakeTables t;
  ProposalHandler h(EnvelopeConfig(), t.access());
  Frame f1(0, 3, 4, 100, 103.f, NOW);
  ASSERT_TRUE(h.handle(f1.b, 26, NOW, false).wrote);
  t.pages[kPageVE][cell_offset(3, 4)] = 90;  // TunerStudio 'M' write behind our back
  Frame stale(0, 3, 4, 103, 105.f, NOW + 1);
  EXPECT_EQ(h.handle(stale.b, 26, NOW + 1, false).reason, Reason::STALE_BASIS);
  Frame fresh(0, 3, 4, 90, 92.f, NOW + 2);
  Outcome o = h.handle(fresh.b, 26, NOW + 2, false);
  EXPECT_EQ(o.decision, Decision::ACCEPTED);
  EXPECT_EQ(t.pages[kPageVE][cell_offset(3, 4)], 92);
}

TEST(Handler, ReplayAndRateLimit) {
  FakeTables t;
  ProposalHandler h(EnvelopeConfig(), t.access());
  uint8_t v = 100;
  for (uint32_t k = 0; k < 3; ++k) {
    Frame f(0, 3, 4, v, v + 1.f, NOW + k * 1000);
    Outcome o = h.handle(f.b, 26, NOW + k * 1000, false);
    ASSERT_TRUE(o.wrote) << k;
    v = o.new_raw;
  }
  Frame rl(0, 3, 4, v, v + 1.f, NOW + 3000);
  EXPECT_EQ(h.handle(rl.b, 26, NOW + 3000, false).reason, Reason::RATE_LIMITED);
  Frame rp(0, 3, 4, v, v + 1.f, NOW + 2000);  // older than last applied
  EXPECT_EQ(h.handle(rp.b, 26, NOW + 2000, false).reason, Reason::STALE_TIMESTAMP);
}

TEST(Handler, NeverBurns) {
  // The handler has no EEPROM dependency at all: the only side effect is TableAccess::set
  // (RAM). This test documents the contract: N accepted proposals -> N RAM sets, nothing else.
  FakeTables t;
  ProposalHandler h(EnvelopeConfig(), t.access());
  for (int r = 0; r < 8; ++r) {
    Frame f(0, r, 0, 100, 102.f, NOW);
    h.handle(f.b, 26, NOW, false);
  }
  EXPECT_EQ(t.sets, 8);
}

TEST(Handler, ResponseEncodingAndLog) {
  FakeTables t;
  ProposalHandler h(EnvelopeConfig(), t.access());
  Frame f(0, 3, 4, 100, 120.f, NOW, 100, 0, 0xA1B2C3D4);
  Outcome o = h.handle(f.b, 26, NOW, false);
  uint8_t out[kResponseLen];
  ProposalHandler::encode_response(o, NOW, out);
  EXPECT_EQ(out[0], 0x00);
  EXPECT_EQ(out[1], (uint8_t)Decision::CLAMPED);
  EXPECT_EQ(out[2], (uint8_t)Reason::FUEL_DELTA_EXCEEDED);
  EXPECT_EQ(out[3], 1);
  EXPECT_EQ(out[4], 100);
  EXPECT_EQ(out[5], 105);
  EXPECT_EQ(rd32(out + 6), 0xA1B2C3D4u);
  EXPECT_EQ(rd32(out + 10), NOW);
  ASSERT_EQ(h.log_count(), 1);
  EXPECT_EQ(h.log_at(0).proposal_id, 0xA1B2C3D4u);
  EXPECT_EQ(h.log_at(0).new_raw, 105);
}

TEST(Handler, LogRingWraps) {
  FakeTables t;
  ProposalHandler h(EnvelopeConfig(), t.access());
  for (uint32_t i = 0; i < kLogCapacity + 10; ++i) {
    Frame f(0, 0, 0, 100, 100.f, NOW, 100, 0, i);
    h.handle(f.b, 26, NOW, false);
  }
  EXPECT_EQ(h.log_count(), kLogCapacity);
  EXPECT_EQ(h.log_at(0).proposal_id, 10u);
  EXPECT_EQ(h.log_at(kLogCapacity - 1).proposal_id, (uint32_t)kLogCapacity + 9);
}
