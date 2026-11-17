#include "robot.hpp"
#include <algorithm>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace robot {
double seconds(Clock::time_point since) {
    return std::chrono::duration<double>(Clock::now() - since).count();
}
namespace {
std::string trim(std::string s) {
    auto a = s.find_first_not_of(" \t\r\n");
    return a == std::string::npos ? "" : s.substr(a, s.find_last_not_of(" \t\r\n") - a + 1);
}
double number(const std::string& s) {
    size_t n = 0;
    double v = std::stod(s, &n);
    if (n != s.size() || !std::isfinite(v)) throw std::invalid_argument("invalid number: " + s);
    return v;
}
int integer(const std::string& s) {
    double n = number(s);
    if (n != std::floor(n) || n < 0 || n > 1000000) throw std::invalid_argument("invalid integer: " + s);
    return static_cast<int>(n);
}
}
Config load_config(const std::string& path) {
    Config c;
    if (path.empty()) {
        // Simulation values only. Hardware has no inferred stroke or limits.
        for (int i = 0; i < 3; ++i) c.cylinders[i] = {i + 1, 16, 0, 16, 0.5};
        return c;
    }
    c.hardware = true;
    std::ifstream file(path);
    if (!file) throw std::runtime_error("cannot read config: " + path);
    std::map<std::string, std::string> values;
    std::string line;
    while (std::getline(file, line)) {
        line = trim(line.substr(0, line.find('#')));
        if (line.empty()) continue;
        auto eq = line.find('=');
        if (eq == std::string::npos) throw std::invalid_argument("expected key=value: " + line);
        auto key = trim(line.substr(0, eq)), value = trim(line.substr(eq + 1));
        if (!values.emplace(key, value).second) throw std::invalid_argument("duplicate config key: " + key);
    }
    auto take = [&](const std::string& key) {
        auto it = values.find(key);
        if (it == values.end() || it->second.empty()) throw std::invalid_argument("missing configuration: " + key);
        std::string v = it->second; values.erase(it); return v;
    };
    c.propulsion = take("propulsion");
    if (c.propulsion != "none") c.propulsion_port = take("propulsion_port");
    c.cylinder_port = take("cylinder_port");
    c.cylinder_baud = integer(take("cylinder_baud"));
    for (int i = 0; i < 3; ++i) {
        std::string key = "cylinder" + std::to_string(i + 1);
        std::string value = take(key);
        std::replace(value.begin(), value.end(), ',', ' ');
        std::istringstream in(value);
        std::array<std::string, 5> fields;
        for (auto& f : fields) if (!(in >> f)) throw std::invalid_argument(key + ": expected id,stroke,min,max,speed");
        std::string extra;
        if (in >> extra) throw std::invalid_argument(key + ": too many fields");
        c.cylinders[i] = {integer(fields[0]), number(fields[1]), number(fields[2]), number(fields[3]), number(fields[4])};
    }
    if (values.count("auto_zero")) {
        const auto value = take("auto_zero");
        if (value != "true" && value != "false") throw std::invalid_argument("auto_zero must be true or false");
        c.auto_zero = value == "true";
    }
    if (values.count("zero_tolerance_mm")) c.zero_tolerance_mm = number(take("zero_tolerance_mm"));
    if (values.count("max_speed")) c.max_speed = integer(take("max_speed"));
    if (values.count("acceleration")) c.acceleration = number(take("acceleration"));
    if (values.count("deadzone")) c.deadzone = number(take("deadzone"));
    if (!values.empty()) throw std::invalid_argument("unknown configuration: " + values.begin()->first);
    validate(c);
    return c;
}
bool valid_feedback(double mm, const CylinderConfig& c, double tolerance) {
    return std::isfinite(mm) && mm >= (c.minimum == 0 ? -tolerance : c.minimum) && mm <= c.maximum;
}
void validate(const Config& c) {
    if (!std::isfinite(c.zero_tolerance_mm) || c.zero_tolerance_mm < 0 || c.zero_tolerance_mm > 0.05)
        throw std::invalid_argument("zero_tolerance_mm must be between 0 and 0.05 mm");
    if (c.propulsion != "nano" && c.propulsion != "pcb" && c.propulsion != "none") throw std::invalid_argument("propulsion must be nano, pcb or none");
    if (c.max_speed < 1 || c.max_speed > 16000 || !std::isfinite(c.acceleration) || c.acceleration <= 0 ||
        !std::isfinite(c.deadzone) || c.deadzone < 0 || c.deadzone >= 1)
        throw std::invalid_argument("invalid propulsion speed, acceleration or deadzone");
    if (c.propulsion == "none" && !c.propulsion_port.empty())
        throw std::invalid_argument("propulsion_port must be omitted when propulsion=none");
    if (c.hardware && c.cylinder_port.empty()) throw std::invalid_argument("cylinder serial port is required");
    if (c.hardware && c.propulsion != "none" && (c.propulsion_port.empty() || c.propulsion_port == c.cylinder_port))
        throw std::invalid_argument("two distinct serial ports are required");
    if (c.cylinder_baud != 115200 && c.cylinder_baud != 57600 && c.cylinder_baud != 19200 && c.cylinder_baud != 921600)
        throw std::invalid_argument("unsupported cylinder baudrate");
    std::set<int> ids;
    for (const auto& x : c.cylinders) {
        if (c.auto_zero && x.minimum != 0)
            throw std::invalid_argument("auto_zero requires assembly minimum=0 for every cylinder");
        if (x.id < 1 || x.id > 254 || !ids.insert(x.id).second ||
            !std::isfinite(x.stroke) || x.stroke <= 0 || !std::isfinite(x.minimum) ||
            !std::isfinite(x.maximum) || x.minimum < 0 || x.minimum >= x.maximum || x.maximum > x.stroke ||
            !std::isfinite(x.speed) || x.speed <= 0 || std::floor(x.speed / x.stroke * 2000) < 1 ||
            std::floor(x.speed / x.stroke * 2000) > 32767 ||
            std::ceil(x.minimum / x.stroke * 2000) > std::floor(x.maximum / x.stroke * 2000))
            throw std::invalid_argument("invalid cylinder ID, stroke, limits or speed");
    }
}
double axis_value(double value, double dz) {
    if (!std::isfinite(value)) throw std::invalid_argument("non-finite input");
    value = std::clamp(value, -1.0, 1.0);
    return std::abs(value) <= dz ? 0 : std::copysign((std::abs(value) - dz) / (1 - dz), value);
}
int ramp(int current, int target, double dt, double acceleration) {
    if (dt <= 0) return current;
    int change = std::max(1, static_cast<int>(std::lround(acceleration * std::clamp(dt, 0.0, 0.05))));
    return current < target ? std::min(target, current + change) : std::max(target, current - change);
}
int position_code(double mm, const CylinderConfig& c) {
    if (!std::isfinite(mm) || mm < c.minimum || mm > c.maximum) throw std::invalid_argument("cylinder target outside limits");
    return std::clamp(static_cast<int>(std::lround(mm / c.stroke * 2000)),
                      static_cast<int>(std::ceil(c.minimum / c.stroke * 2000)),
                      static_cast<int>(std::floor(c.maximum / c.stroke * 2000)));
}
} // namespace robot
