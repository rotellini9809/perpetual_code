#pragma once
#include "storage/FlightLogDecoder.h"
#include "storage/FlightSampleStore.h"
#include <cassert>

// Host-test view of a committed v4 block: the reference decoder must accept it.
inline storage::flightlog::DecodedBlock decodeRaw(const storage::RawBlock &raw) {
  storage::flightlog::DecodedBlock block;
  const bool decoded = storage::flightlog::decodeBody(raw.body, block);
  assert(decoded && block.frames == raw.frames); (void)decoded;
  return block;
}
