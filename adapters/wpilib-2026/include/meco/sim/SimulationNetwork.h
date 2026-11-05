#pragma once

#include <charconv>
#include <cstdlib>
#include <stdexcept>
#include <string_view>
#include <networktables/NetworkTableInstance.h>

namespace meco::sim
{
// If FRC_SIM_NT_PORT is set, restart the NT server on loopback with that NT4
// port and no NT3 port. Leave the server untouched when unset. Call once at
// startup before publishing topics. Invalid ports throw without stopping it.
inline void ConfigureSimulationNetwork(
    nt::NetworkTableInstance instance = nt::NetworkTableInstance::GetDefault())
{
    if (const char *text = std::getenv("FRC_SIM_NT_PORT"))
    {
        const std::string_view value{text};
        unsigned int port = 0;
        const auto parsed = std::from_chars(value.data(), value.data() + value.size(), port);
        if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size() || port < 1024 || port > 65535)
            throw std::invalid_argument("FRC_SIM_NT_PORT must be an integer in [1024,65535]");
        instance.StopServer();
        instance.StartServer("", "127.0.0.1", 0, port);
    }
}
} // namespace meco::sim
