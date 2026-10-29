#include "control/AirbrakeLaw.h"

namespace control {
// CUSTOM CONTROL AREA: implement the law here and extend LawState if needed.
// No blocking IO, delays, heap allocation or hardware calls. This is an API
// boundary, not a memory sandbox. There is deliberately no arbitrary flight law.
ControlOutput updateAirbrakes(const ControlInput &, LawState &) { return {}; }
}
