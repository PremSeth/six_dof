#include "src/encoder_retry.h"
#include <assert.h>
#include <initializer_list>

int main() {
  for (unsigned failures = 0; failures <= 3; ++failures) {
    uint32_t ms = 0;
    unsigned calls = 0, pauses = 0;
    bool ok = retryEncoder([&]() { return ++calls > failures; },
      [&]() { return ms; }, [&]() { ++pauses; ++ms; });
    assert(ok == (failures < 3));
    assert(calls == (failures < 3 ? failures + 1 : 3));
    assert(pauses == calls - 1);
  }
  // Slow transactions fail rather than receiving another full retry budget.
  for (bool reply : {false, true}) {
    uint32_t ms = 0;
    unsigned calls = 0;
    assert(!retryEncoder([&]() { ++calls; ms += 101; return reply; },
      [&]() { return ms; }, [&]() { ++ms; }));
    assert(calls == 1);
  }
  // A normal Wire timeout must leave time for a successful second attempt.
  {
    uint32_t ms = 0;
    unsigned calls = 0;
    assert(retryEncoder([&]() { ++calls; ms += calls == 1 ? 51 : 2; return calls == 2; },
      [&]() { return ms; }, [&]() { ++ms; }));
    assert(calls == 2);
  }
  // Unsigned elapsed-time arithmetic survives millis() rollover.
  uint32_t ms = UINT32_MAX;
  unsigned calls = 0;
  assert(retryEncoder([&]() { return ++calls == 2; },
    [&]() { return ms; }, [&]() { ++ms; }));
}
