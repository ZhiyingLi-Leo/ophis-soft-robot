#include "serial.hpp"
#include <glob.h>
#include <cstdlib>
#include <set>
#include <sstream>
#include <stdexcept>

namespace robot {
std::vector<std::string> cylinder_port_candidates() {
    std::vector<std::string> paths;
    for (const char* pattern : {"/dev/cu.usbserial*", "/dev/cu.usbmodem*", "/dev/cu.SLAB_USBtoUART*", "/dev/cu.wchusbserial*"}) {
        glob_t result{};
        const int code = glob(pattern, 0, nullptr, &result);
        if (code == 0) for (size_t i = 0; i < result.gl_pathc; ++i) paths.emplace_back(result.gl_pathv[i]);
        globfree(&result);
        if (code != 0 && code != GLOB_NOMATCH) throw std::runtime_error("cannot enumerate USB serial ports");
    }
    return paths;
}
std::string discover_cylinder_port(const Config& config, const std::vector<std::string>& candidates) {
    std::vector<std::string> matches;
    std::set<std::string> visited;
    std::ostringstream diagnostics;
    char propulsion[4096];
    const std::string excluded = config.propulsion != "none" && realpath(config.propulsion_port.c_str(), propulsion)
        ? propulsion : "";
    for (const auto& path : candidates) {
        char resolved[4096];
        if (!realpath(path.c_str(), resolved)) continue;
        if (excluded == resolved || !visited.insert(resolved).second) continue;
        try {
            Serial port(path, config.cylinder_baud);
            // Identification only: no register writes, enable, homing or motion.
            for (const auto& c : config.cylinders) port.exchange(c.id, inspire::command(c.id, 0x30));
            matches.push_back(path);
        } catch (const std::exception& e) {
            diagnostics << "\n  " << path << ": " << e.what();
        }
    }
    if (matches.size() == 1) return matches.front();
    if (matches.empty()) throw std::runtime_error("no USB serial port replied for all three configured cylinder IDs; check power, IDs and baudrate" + diagnostics.str());
    std::string message = "multiple cylinder buses matched; set cylinder_port explicitly:";
    for (const auto& path : matches) message += "\n  " + path;
    throw std::runtime_error(message);
}
} // namespace robot
