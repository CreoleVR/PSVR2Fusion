#pragma once

#include <string>

namespace psvr2fusion {

// Writes "psvr2fusion: <message>" to SteamVR's vrserver log.
void Log(const char *fmt, ...);

std::string Format(const char *fmt, ...);

} // namespace psvr2fusion
