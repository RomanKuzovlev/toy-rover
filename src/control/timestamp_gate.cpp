#include "control/timestamp_gate.hpp"

namespace toy_rover::control
{
  bool TimestampGate::accept(
      std::chrono::nanoseconds message_timestamp,
      std::chrono::nanoseconds clock_timestamp)
  {
    if (last_clock_timestamp_ && clock_timestamp < *last_clock_timestamp_)
    {
      // A genuine simulation reset starts a new valid timestamp sequence.
      last_message_timestamp_.reset();
    }
    last_clock_timestamp_ = clock_timestamp;

    if (last_message_timestamp_ && message_timestamp <= *last_message_timestamp_)
    {
      return false;
    }

    last_message_timestamp_ = message_timestamp;
    return true;
  }
} // namespace toy_rover::control
