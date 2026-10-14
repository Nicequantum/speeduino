#pragma once
#include <stdint.h>
#if defined(CORE_TEENSY41)
/** Grok Pilot AI proposal command ('Y'). Decodes payload (starting at the command
 *  byte), runs the validation envelope, writes the table cell in RAM if accepted,
 *  and writes the 14-byte response back into payload. Never burns to EEPROM.
 *  @return response length */
uint16_t grokPilotHandleProposal(uint8_t *payload, uint16_t length);
#endif
