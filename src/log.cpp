#include "log.h"

#include <openvr_driver.h>

#include <cstdarg>
#include <cstdio>

namespace psvr2fusion {

static std::string VFormat(const char *fmt, va_list args) {
    va_list copy;
    va_copy(copy, args);
    int n = std::vsnprintf(nullptr, 0, fmt, copy);
    va_end(copy);
    if (n <= 0) return {};
    std::string s((size_t)n + 1, '\0');
    std::vsnprintf(&s[0], s.size(), fmt, args);
    s.resize((size_t)n);
    return s;
}

std::string Format(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    std::string s = VFormat(fmt, args);
    va_end(args);
    return s;
}

void Log(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    std::string s = "psvr2fusion: " + VFormat(fmt, args) + "\n";
    va_end(args);
    if (vr::VRDriverLog()) vr::VRDriverLog()->Log(s.c_str());
}

} // namespace psvr2fusion
