#pragma once
#include <stdint.h>

// Three total attempts, with a 100 ms elapsed-time budget. An in-flight Wire
// transaction can exceed the budget; its own timeout still applies.
template <typename Read, typename Clock, typename Pause>
bool retryEncoder(Read read, Clock now, Pause pause) {
  const uint32_t started = now();
  for (unsigned attempt = 0; attempt < 3; ++attempt) {
    if (uint32_t(now() - started) >= 100) return false;
    const bool ok = read();
    if (uint32_t(now() - started) >= 100) return false;
    if (ok) return true;
    if (attempt < 2) pause();
  }
  return false;
}
