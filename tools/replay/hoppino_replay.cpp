#include "configuration/RecoveryRuleEngine.h"
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace recovery;
namespace {
// Deliberately synthetic fixture. Change here to exercise another plan.
Configuration examplePlan() {
  Configuration c;
  c.ruleCount = 2;
  auto &d = c.rules[0]; d.id = 10; std::strcpy(d.name.data(), "Example apogee");
  d.conditionCount = 1; d.conditions[0].kind = ConditionKind::ApogeeDetected;
  d.assignmentCount = 2;
  d.assignments[0].channel = 1;
  d.assignments[1].kind = OutputKind::Servo; d.assignments[1].channel = 1;
  c.pyros[0].pulseMs = 100;
  c.servos[0].calibrated = true; c.servos[0].initialUs = 1000;
  c.servos[0].finalUs = 2000;
  auto &m = c.rules[1]; m.id = 20; std::strcpy(m.name.data(), "Example low descent");
  m.delayMs = 100;
  m.conditionCount = 2;
  m.conditions[0].kind = ConditionKind::Phase; m.conditions[0].phases = DescentMask;
  m.conditions[1].kind = ConditionKind::AltitudeComparison;
  m.conditions[1].comparison = Comparison::Less; m.conditions[1].value = 30;
  m.assignmentCount = 1; m.assignments[0].channel = 2;
  c.pyros[1].pulseMs = 100;
  return c;
}
std::vector<std::string> split(std::string line) {
  if (!line.empty() && line.back() == '\r') line.pop_back();
  if (line.find('"') != std::string::npos) throw std::runtime_error("Quoted CSV fields are unsupported");
  std::vector<std::string> fields;
  std::size_t start = 0;
  for (;;) {
    const auto end = line.find(',', start);
    fields.push_back(line.substr(start, end == std::string::npos ? end : end - start));
    if (end == std::string::npos) return fields;
    start = end + 1;
  }
}
uint64_t integer(const std::string &text) {
  if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos)
    throw std::runtime_error("Expected unsigned integer, got: " + text);
  std::size_t used = 0;
  const auto value = std::stoull(text, &used);
  if (used != text.size()) throw std::runtime_error("Invalid integer");
  return value;
}
float number(const std::string &text) {
  std::size_t used = 0;
  const float value = std::stof(text, &used);
  if (used != text.size()) throw std::runtime_error("Invalid numeric field: " + text);
  return value; // NaN/Infinity become explicitly invalid estimates below.
}
FlightPhase phase(uint64_t state) {
  switch (state) {
    case 0: return FlightPhase::Pad;
    case 1: return FlightPhase::Boost;
    case 2: return FlightPhase::Coast;
    case 3: case 4: return FlightPhase::Descent;
    case 5: return FlightPhase::Landed;
    default: throw std::runtime_error("Unsupported legacy state: " + std::to_string(state));
  }
}
const char *phaseName(FlightPhase p) {
  switch (p) {
    case FlightPhase::Pad: return "PAD";
    case FlightPhase::Boost: return "BOOST";
    case FlightPhase::Coast: return "COAST";
    case FlightPhase::Descent: return "DESCENT";
    case FlightPhase::Landed: return "LANDED";
    case FlightPhase::Boost2: return "BOOST 2";
    case FlightPhase::Coast2: return "COAST 2";
    case FlightPhase::Abort: return "ABORT";
  }
  return "UNKNOWN";
}
struct Row { uint64_t time; FlightPhase phase; float altitude; float velocity; };
std::vector<Row> readRows(const std::string &path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("Cannot open input: " + path);
  std::string line;
  if (!std::getline(in, line)) throw std::runtime_error("Empty CSV");
  auto header = split(line);
  if (header[0].compare(0, 3, "\xEF\xBB\xBF") == 0) header[0].erase(0, 3);
  std::map<std::string, std::size_t> columns;
  for (std::size_t i = 0; i < header.size(); ++i)
    if (!columns.emplace(header[i], i).second) throw std::runtime_error("Duplicate column: " + header[i]);
  // The dataset is microseconds (t_us); rules run on whole milliseconds. t_ms also works.
  const bool micros = columns.count("t_us") != 0;
  for (const char *name : {micros ? "t_us" : "t_ms", "z_est", "vz_est", "state"})
    if (!columns.count(name)) throw std::runtime_error(std::string("Missing column: ") + name);
  std::vector<Row> rows;
  std::size_t lineNumber = 1;
  while (std::getline(in, line)) {
    ++lineNumber;
    try {
      const auto fields = split(line);
      if (fields.size() != header.size()) throw std::runtime_error("Wrong column count");
      Row r;
      const auto stamp = integer(fields[columns.at(micros ? "t_us" : "t_ms")]);
      if (micros && stamp % 1000) throw std::runtime_error("t_us must be whole milliseconds");
      r.time = micros ? stamp / 1000 : stamp;
      r.phase = phase(integer(fields[columns.at("state")]));
      r.altitude = number(fields[columns.at("z_est")]);
      r.velocity = number(fields[columns.at("vz_est")]);
      if (!rows.empty() && r.time <= rows.back().time)
        throw std::runtime_error("Timestamps must strictly increase");
      rows.push_back(r);
    } catch (const std::exception &e) {
      throw std::runtime_error("CSV line " + std::to_string(lineNumber) + ": " + e.what());
    }
  }
  if (in.bad()) throw std::runtime_error("Input read failed");
  if (rows.empty()) throw std::runtime_error("No flight samples");
  return rows;
}
} // namespace
int main(int argc, char **argv) {
  try {
    if (argc != 3) throw std::runtime_error("Usage: hoppino_replay <input.csv> <new-events.csv>");
    // Never overwrite a recording or an existing export, including via path aliases.
    { std::ifstream existing(argv[2]); if (existing.good()) throw std::runtime_error("Output already exists; choose a new output path"); }
    const auto rows = readRows(argv[1]); // Validate entire input before opening output.
    const auto config = examplePlan();
    const ValidationLimits limits{1000, 2000, 1000, 60000};
    RuleEngine engine;
    if (!engine.begin(config, limits, 100)) throw std::runtime_error("Invalid replay fixture");
    std::ofstream out(argv[2]);
    if (!out) throw std::runtime_error("Cannot create output");
    out << "t_ms,source_t_ms,event,phase,rule_id,rule_name,output_kind,channel,mode\n";
    std::cout << "Hoppino estimate replay; EXAMPLE PLAN; VIRTUAL outputs only.\n";
    const uint64_t origin = rows.front().time;
    std::array<bool, MaxRules> reported{};
    bool landed = false;
    uint64_t landedAt = 0;
    std::size_t starts = 0;
    auto emit = [&](uint64_t t, uint64_t source, const char *event, const char *p,
                    RuleId id, const char *name, const char *kind, unsigned channel) {
      out << t << ',' << source << ',' << event << ',' << p << ',' << id << ','
          << name << ',' << kind << ',' << channel << ",VIRTUAL\n";
    };
    for (std::size_t n = 0; n < rows.size(); ++n) {
      const auto &r = rows[n]; const auto t = r.time - origin;
      if (r.phase == FlightPhase::Landed && !landed) { landed = true; landedAt = t; }
      // Explicit replay policy: 15-second post-landing window. No fabricated EOF time.
      if (landed && t - landedAt >= 15000) engine.closeStarts();
      FlightSnapshot s;
      s.sequence = n + 1; s.phase = r.phase; s.phaseValid = true; s.phaseMeasuredMs = t;
      s.altitude.value = r.altitude; s.altitude.valid = std::isfinite(r.altitude);
      s.altitude.sequence = n + 1; s.altitude.measuredMs = t;
      s.verticalVelocity = s.altitude; s.verticalVelocity.value = r.velocity;
      s.verticalVelocity.valid = std::isfinite(r.velocity);
      if (n == 0 || r.phase != rows[n - 1].phase) {
        emit(t, r.time, "phase", phaseName(r.phase), 0, "", "", 0);
        std::cout << t << " ms: " << phaseName(r.phase) << '\n';
      }
      const auto result = engine.evaluate(t, s);
      if (!result.accepted) throw std::runtime_error("Evaluator rejected sample");
      for (std::size_t i = 0; i < config.ruleCount; ++i) {
        if (reported[i] || !engine.rules()[i].triggered) continue;
        reported[i] = true;
        emit(t, r.time, "rule_trigger", phaseName(r.phase), config.rules[i].id,
             config.rules[i].name.data(), "", 0);
      }
      for (std::size_t i = 0; i < result.startCount; ++i) {
        const auto &a = result.starts[i];
        const char *name = "";
        for (std::size_t j = 0; j < config.ruleCount; ++j)
          if (config.rules[j].id == a.rule) name = config.rules[j].name.data();
        const char *kind = a.kind == OutputKind::Pyro ? "pyro" : "servo";
        emit(t, r.time, "output_start", phaseName(r.phase), a.rule, name, kind, a.channel);
        std::cout << t << " ms: VIRTUAL " << kind << ' ' << unsigned(a.channel) << " (" << name << ")\n";
        ++starts;
      }
    }
    std::size_t pending = 0;
    for (std::size_t i = 0; i < engine.actionCount(); ++i)
      if (engine.actions()[i].state == ActionState::Pending) ++pending;
    engine.end();
    out.flush(); if (!out) throw std::runtime_error("Output write failed");
    std::cout << rows.size() << " samples, " << starts << " virtual starts, " << pending
              << " pending actions cancelled at EOF.\nEvents: " << argv[2] << '\n';
    return 0;
  } catch (const std::exception &e) { std::cerr << "Replay failed: " << e.what() << '\n'; return 1; }
}
