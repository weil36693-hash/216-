// Local opt-in extension: put each vehicle's Agent in its assigned ROS domain.
// With no LEAP_AGENT_DOMAIN_ID, preserve the upstream client's domain exactly.
#pragma once

#include <cstdlib>
#include <stdexcept>
#include <cstdint>

namespace eprosima {
namespace uxr {
inline int16_t fleet_domain(int16_t requested)
{
    const char* value = std::getenv("LEAP_AGENT_DOMAIN_ID");
    if (!value) {
        return requested;
    }
    char* end = nullptr;
    const long domain = std::strtol(value, &end, 10);
    if (value == end || *end != '\0' || domain < 1 || domain > 99) {
        throw std::runtime_error("LEAP_AGENT_DOMAIN_ID must be an integer in [1, 99]");
    }
    return static_cast<int16_t>(domain);
}
} // namespace uxr
} // namespace eprosima
