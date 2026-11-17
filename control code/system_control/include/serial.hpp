#pragma once
#include "robot.hpp"

namespace robot {
std::vector<std::string> cylinder_port_candidates();
std::string discover_cylinder_port(const Config& config, const std::vector<std::string>& candidates);

class Serial {
public:
    Serial(const std::string& path, int baud);
    ~Serial();
    Serial(const Serial&) = delete;
    Serial& operator=(const Serial&) = delete;
    void write(const std::vector<unsigned char>& data);
    std::vector<unsigned char> read(int timeout_ms);
    std::string line(const std::string& command, const std::string& expected);
    inspire::State exchange(int id, const std::vector<unsigned char>& request, int op = 0x30, int address = 0);
private:
    int fd_ = -1;
    std::string lines_;
};
}
