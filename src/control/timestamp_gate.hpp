#pragma once

#include <chrono>
#include <optional>

namespace toy_rover::control
{
  class TimestampGate
  {
  public:
    [[nodiscard]] bool accept(
        std::chrono::nanoseconds message_timestamp,
        std::chrono::nanoseconds clock_timestamp);

  private:
    std::optional<std::chrono::nanoseconds> last_message_timestamp_;
    std::optional<std::chrono::nanoseconds> last_clock_timestamp_;
  };
} // namespace toy_rover::control
