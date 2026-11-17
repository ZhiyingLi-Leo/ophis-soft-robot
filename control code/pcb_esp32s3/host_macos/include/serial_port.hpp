#pragma once

#include <string>

namespace mac_control {

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();

    SerialPort(const SerialPort &) = delete;
    SerialPort &operator=(const SerialPort &) = delete;

    bool open_port(const std::string &path, int baud_rate, std::string &error);
    bool send_line(const std::string &line, std::string &error);
    std::string read_available(std::string &error);
    void close_port();
    bool is_open() const { return fd_ >= 0; }

private:
    int fd_ = -1;
};

} // namespace mac_control
