#include "robot.hpp"
#include <cstdint>
#include <stdexcept>

namespace robot::inspire {
std::vector<unsigned char> command(int id, int op, const std::vector<unsigned char>& payload) {
    if (id < 1 || id > 254 || payload.size() > 250) throw std::invalid_argument("invalid cylinder frame");
    std::vector<unsigned char> out{0x55, 0xaa, static_cast<unsigned char>(payload.size() + 1),
                                   static_cast<unsigned char>(id), static_cast<unsigned char>(op)};
    out.insert(out.end(), payload.begin(), payload.end());
    unsigned sum = 0;
    for (size_t i = 2; i < out.size(); ++i) sum += out[i];
    out.push_back(static_cast<unsigned char>(sum));
    return out;
}
std::vector<unsigned char> registers(int id, int address, const std::vector<int>& values) {
    std::vector<unsigned char> payload;
    auto put = [&](int x) {
        if (x < 0 || x > 65535) throw std::invalid_argument("register value outside uint16");
        payload.push_back(x & 255); payload.push_back((x >> 8) & 255);
    };
    if (values.empty()) throw std::invalid_argument("empty register write");
    put(address);
    for (int x : values) put(x);
    return command(id, 0x32, payload);
}
bool extract(std::vector<unsigned char>& b, int id, int op, int address, State& s) {
    for (size_t i = 0; i + 20 <= b.size(); ++i) {
        auto p = b.data() + i;
        if (p[0] != 0xaa || p[1] != 0x55 || p[2] != 15 || p[3] != id || p[4] != op ||
            (p[5] | (p[6] << 8)) != address) continue;
        unsigned sum = 0;
        for (int j = 2; j < 19; ++j) sum += p[j];
        if ((sum & 255) != p[19]) continue;
        s.target = static_cast<int16_t>(p[7] | (p[8] << 8));
        s.position = static_cast<int16_t>(p[9] | (p[10] << 8));
        s.temperature = static_cast<int8_t>(p[17]); s.fault = p[18];
        b.erase(b.begin(), b.begin() + static_cast<long>(i + 20));
        return true;
    }
    if (b.size() > 512) b.erase(b.begin(), b.end() - 19);
    return false;
}
} // namespace robot::inspire
