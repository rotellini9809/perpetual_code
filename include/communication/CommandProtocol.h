#pragma once
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cstdarg>
#include <cmath>
#include <limits>
#include <cerrno>

namespace protocol {
struct Tokens {
  char bytes[129] = {};
  char *values[12] = {};
  unsigned count = 0;
  bool valid = true;
  explicit Tokens(const char *text) {
    if (strlen(text) > 128) { valid = false; return; }
    strcpy(bytes, text); char *p = bytes;
    while (*p) {
      while (*p == ' ' || *p == '\t') ++p;
      if (!*p) break;
      if (count == 12) { valid = false; return; }
      if (*p == '"') {
        char *write = ++p; values[count++] = write;
        while (*p && *p != '"') {
          if (*p == '\\') { ++p; if (*p != '"' && *p != '\\') { valid = false; return; } }
          *write++ = *p++;
        }
        if (*p != '"') { valid = false; return; }
        ++p; if (*p && *p != ' ' && *p != '\t') { valid = false; return; }
        *write = 0;
      } else {
        values[count++] = p;
        while (*p && *p != ' ' && *p != '\t') { if (*p == '"' || *p == '\\') { valid = false; return; } ++p; }
        if (*p) *p++ = 0;
      }
    }
  }
  const char *operator[](unsigned i) const { return i < count ? values[i] : ""; }
  bool is(unsigned i, const char *s) const { return strcmp((*this)[i], s) == 0; }
};
struct Json {
  char bytes[768] = {}; std::size_t length = 0; bool ok = true;
  void add(const char *format, ...) {
    if (!ok) return;
    va_list args; va_start(args, format);
    const int n = vsnprintf(bytes + length, sizeof(bytes) - length, format, args); va_end(args);
    if (n < 0 || std::size_t(n) >= sizeof(bytes) - length) { ok = false; return; }
    length += std::size_t(n);
  }
  void value(float number) {
    if (std::isfinite(number)) add("%.9g", double(number)); else add("null");
  }
  void number(const char *name, float number) { add(",\"%s\":", name); value(number); }
  void quoted(const char *text) {
    add("\"");
    for (const char *p = text; *p; ++p) {
      if (*p == '"' || *p == '\\') add("\\%c", *p);
      else if (static_cast<unsigned char>(*p) < 0x20) add("\\u%04x", unsigned(static_cast<unsigned char>(*p)));
      else add("%c", *p);
    }
    add("\"");
  }
};
template<typename UInt> inline bool unsignedInteger(const char *text, UInt &value) {
  if (!*text) return false;
  value = 0;
  for (const char *p = text; *p; ++p) {
    if (*p < '0' || *p > '9') return false;
    const UInt digit = UInt(*p - '0');
    if (value > (std::numeric_limits<UInt>::max() - digit) / 10) return false;
    value = value * 10 + digit;
  }
  return true;
}
inline bool unsignedNumber(const char *text, uint32_t &value) { return unsignedInteger(text, value); }
inline bool unsignedNumber64(const char *text, uint64_t &value) { return unsignedInteger(text, value); }
inline bool finiteNumber(const char *text, float &value) {
  if (!*text) return false;
  char *end; errno = 0; value = std::strtof(text, &end);
  return !*end && errno != ERANGE && std::isfinite(value);
}
inline bool floating(const char *text, float &value) {
  if (!*text) return false;
  char *end; errno = 0; value = std::strtof(text, &end);
  return !*end && errno != ERANGE;
}
// Configuration validation errors may carry extra fields; other errors are compact.
inline bool finish(const char *command, const Json &data, const char *error, bool hasId, uint32_t id,
                   char *output, std::size_t capacity, bool includeErrorData = false) {
  if (!data.ok) error = "RESPONSE_TOO_LARGE";
  char identity[32] = {};
  if (hasId) snprintf(identity, sizeof(identity), "\"id\":%lu,", static_cast<unsigned long>(id));
  if (error) snprintf(output, capacity, "{%s\"type\":\"error\",\"command\":\"%s\",\"ok\":false,\"code\":\"%s\"%s}\n",
                      identity, command, error, includeErrorData && data.ok ? data.bytes : "");
  else snprintf(output, capacity, "{%s\"type\":\"reply\",\"command\":\"%s\",\"ok\":true%s}\n", identity, command, data.bytes);
  return true;
}

}
