// Pinpoint which core record(s) in MAS/data/cores.ndjson crash OpenMagnetics::Core(json)
// (the load_cores type_error.304 that breaks MagneticAdviser). Prints name + line + error.
#include <iostream>
#include <fstream>
#include <string>
#include <nlohmann/json.hpp>
#include "constructive_models/Core.h"
using json = nlohmann::json;

int main(int argc, char** argv) {
    const std::string path = argc > 1 ? argv[1] : "/home/alf/OpenMagnetics/MKF/MAS/data/cores.ndjson";
    std::ifstream f(path);
    std::string line; int n = 0, bad = 0;
    while (std::getline(f, line)) {
        ++n;
        if (line.empty()) continue;
        json j;
        try { j = json::parse(line); }
        catch (const std::exception& e) { std::cout << "JSON-PARSE-FAIL line " << n << ": " << e.what() << "\n"; ++bad; continue; }
        const std::string name = j.value("name", std::string("?"));
        try { OpenMagnetics::Core c(j); }
        catch (const std::exception& e) {
            std::cout << "BAD line " << n << " [" << name << "]: " << e.what() << "\n";
            ++bad;
        }
    }
    std::cout << "total=" << n << " bad=" << bad << "\n";
    return 0;
}
