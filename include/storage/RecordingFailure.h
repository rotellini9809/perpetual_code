#pragma once
#include "storage/RecordingTypes.h"
#include "storage/FlashDevice.h"
namespace storage {
struct RecordingFailure {
  RecorderError error = RecorderError::None;
  const char *component = "None", *cause = "None";
  FlashFailure flash;
};
}
