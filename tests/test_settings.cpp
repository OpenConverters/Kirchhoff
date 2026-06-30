// Kirchhoff::Settings — project-wide config singleton (mirrors MKF's Settings). Verify the singleton
// identity, the RunEngine default (SPICE), get/set round-trips, and reset().

#include <catch2/catch_test_macros.hpp>
#include "Settings.hpp"

using Kirchhoff::Settings;
using Kirchhoff::RunEngine;

TEST_CASE("Settings is a single shared instance", "[settings]") {
    CHECK(&Settings::GetInstance() == &Settings::GetInstance());
}

TEST_CASE("Settings defaults: SPICE run engine, in-process preferred", "[settings]") {
    Settings::GetInstance().reset();
    CHECK(Settings::GetInstance().get_run_engine() == RunEngine::SPICE);
    CHECK(Settings::GetInstance().get_prefer_in_process_ngspice() == true);
    CHECK(Settings::GetInstance().get_verbose() == false);
}

TEST_CASE("Settings get/set round-trip and reset", "[settings]") {
    auto& s = Settings::GetInstance();
    s.reset();
    s.set_run_engine(RunEngine::ANALYTICAL);
    s.set_prefer_in_process_ngspice(false);
    s.set_verbose(true);
    CHECK(s.get_run_engine() == RunEngine::ANALYTICAL);
    CHECK(s.get_prefer_in_process_ngspice() == false);
    CHECK(s.get_verbose() == true);
    // The change is visible through any handle to the singleton.
    CHECK(Settings::GetInstance().get_run_engine() == RunEngine::ANALYTICAL);
    s.reset();
    CHECK(s.get_run_engine() == RunEngine::SPICE);
    CHECK(s.get_prefer_in_process_ngspice() == true);
}
