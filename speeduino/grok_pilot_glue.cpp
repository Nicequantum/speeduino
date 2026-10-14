// Grok Pilot: binds the pure proposal handler (grok_pilot/proposal.*) to
// Speeduino's page tables. Teensy 4.1 only: the envelope's ~43 KB of static
// state doesn't fit the AVR/STM32 targets, and those builds don't change.
#include "globals.h"
#include "pages.h"
#include "storage.h"
#include "grok_pilot_glue.h"

#if defined(CORE_TEENSY41)
#include "src/grok_pilot/proposal.h"

#ifndef GROK_PILOT_ENGINE_ID
#define GROK_PILOT_ENGINE_ID 0   // 0 = port, 1 = starboard; set per ECU with -DGROK_PILOT_ENGINE_ID=1
#endif

namespace {
uint8_t gp_get(void*, uint8_t page, uint16_t offset) { return getPageValue(page, offset); }
void gp_set(void*, uint8_t page, uint16_t offset, uint8_t value) {
  setPageValue(page, offset, value);              // RAM only, same call as the TS 'M' write path
  deferEEPROMWritesUntil = micros() + EEPROM_DEFER_DELAY;
}
grok_pilot::EnvelopeConfig gp_config() {
  grok_pilot::EnvelopeConfig c;
  c.engine_id = GROK_PILOT_ENGINE_ID;
  return c;
}
grok_pilot::ProposalHandler gp_handler(gp_config(), grok_pilot::TableAccess{nullptr, &gp_get, &gp_set});
}  // namespace

uint16_t grokPilotHandleProposal(uint8_t *payload, uint16_t length)
{
  const uint32_t now = millis();
  grok_pilot::Outcome o = gp_handler.handle(payload, length, now, isEepromWritePending());
  grok_pilot::ProposalHandler::encode_response(o, now, payload);   // reply reuses the payload buffer
  return (uint16_t)grok_pilot::kResponseLen;
}
#endif
