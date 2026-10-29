#pragma once
#include "communication/CommandProtocol.h"
#include "storage/FlightSampleStore.h"
#include "storage/BinaryEncoding.h"
#include <cstring>

namespace protocol {
constexpr std::size_t PackedFlightMaxBytes = 448;
// Pure bounded wire codec: no flash IO, transfer lease or run/arming policy.
// XOR state resets for each reply; samples retain the v3 bit layout.
inline bool appendPackedFlightSamples(Json &data, const storage::SampleBlock &block, uint32_t offset) {
  static_assert(storage::FlightSampleStore::RecordBytes==76,"Packed protocol requires sample v3");
  if (offset>=block.count || block.count>block.samples.size()) return false;
  // XOR successive 32-bit words; omit unchanged words using a 32-bit mask.
  // Bounded payload fits existing JSON buffers even for incompressible data.
  uint8_t bytes[PackedFlightMaxBytes],previous[storage::FlightSampleStore::RecordBytes]={},record[storage::FlightSampleStore::RecordBytes];
  unsigned count=0,size=0;
  while (offset+count<block.count) {
    storage::FlightSampleStore::encodeSample(block.samples[offset+count],record);
    uint32_t mask=0; unsigned changed=0;
    for (unsigned word=0;word<storage::FlightSampleStore::RecordBytes/4;++word) {
      if (storage::binary::get32(record+word*4,4)!=storage::binary::get32(previous+word*4,4)) { mask|=uint32_t(1u<<word); ++changed; }
    }
    if (size+4+changed*4>sizeof(bytes)) break;
    storage::binary::put(bytes+size,mask,4); size+=4;
    for (unsigned word=0;word<storage::FlightSampleStore::RecordBytes/4;++word) if (mask&(1u<<word)) {
      storage::binary::put(bytes+size,storage::binary::get32(record+word*4,4)^storage::binary::get32(previous+word*4,4),4); size+=4;
    }
    memcpy(previous,record,sizeof(record)); ++count;
  }
  data.add(",\"block\":%lu,\"offset\":%lu,\"first\":%lu,\"block_count\":%u,\"count\":%u,\"crc32\":%lu,\"data\":\"",
           static_cast<unsigned long>(block.index),static_cast<unsigned long>(offset),static_cast<unsigned long>(block.firstSample+offset),unsigned(block.count),count,static_cast<unsigned long>(storage::binary::crc(bytes,size)));
  static const char alphabet[]="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  for (unsigned i=0;i<size;i+=3) {
    const uint32_t bits=(uint32_t(bytes[i])<<16) | (i+1<size ? uint32_t(bytes[i+1])<<8 : 0) | (i+2<size ? bytes[i+2] : 0);
    data.add("%c%c%c%c",alphabet[(bits>>18)&63],alphabet[(bits>>12)&63],i+1<size ? alphabet[(bits>>6)&63] : '=',i+2<size ? alphabet[bits&63] : '=');
  }
  data.add("\""); return true;
}
}
