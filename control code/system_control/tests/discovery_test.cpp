#include "serial.hpp"
#include <iostream>
int main(int argc, char** argv) {
    try {
        auto config = robot::load_config("");
        config.propulsion = "none";
        std::vector<std::string> paths;
        for (int i = 1; i < argc; ++i) paths.emplace_back(argv[i]);
        std::cout << robot::discover_cylinder_port(config, paths) << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
