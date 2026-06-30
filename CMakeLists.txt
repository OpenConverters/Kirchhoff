cmake_minimum_required(VERSION 3.15)
project(Kirchhoff LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

# PSMA-org sibling repos (PEAS/RAS/CAS/SAS/MAS/CIAS) are git submodules under deps/.
# After cloning Kirchhoff: `git submodule update --init` to fetch them.
set(PSMA_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/deps" CACHE PATH "Directory holding the PSMA-org sibling repos (submodules)")

# Pull in the per-family converter libs + the CIAS circuit converter (tests/pybind off here).
foreach(opt RAS_BUILD_TESTS RAS_BUILD_PYBIND CAS_BUILD_TESTS CAS_BUILD_PYBIND
            SAS_BUILD_TESTS SAS_BUILD_PYBIND MAS_BUILD_PYBIND CIAS_BUILD_TESTS CIAS_BUILD_PYBIND)
    set(${opt} OFF CACHE BOOL "" FORCE)
endforeach()

add_subdirectory("${PSMA_ROOT}/RAS"      "${CMAKE_BINARY_DIR}/RAS")
add_subdirectory("${PSMA_ROOT}/CAS"      "${CMAKE_BINARY_DIR}/CAS")
add_subdirectory("${PSMA_ROOT}/SAS"      "${CMAKE_BINARY_DIR}/SAS")
add_subdirectory("${PSMA_ROOT}/AAS"      "${CMAKE_BINARY_DIR}/AAS")
add_subdirectory("${PSMA_ROOT}/CTAS"     "${CMAKE_BINARY_DIR}/CTAS")
add_subdirectory("${PSMA_ROOT}/MAS/kirchhoff" "${CMAKE_BINARY_DIR}/MAS")
add_subdirectory("${PSMA_ROOT}/CIAS"     "${CMAKE_BINARY_DIR}/CIAS")

# Optional in-process ngspice runner (libngspice shared-library API). When ON, CMake locates
# libngspice + <ngspice/sharedspice.h> and src/NgspiceRunner.cpp is compiled with ENABLE_NGSPICE; when
# OFF (or the lib is absent) the runner stub throws and callers use the ngspice CLI. Default: auto —
# ON if the library is found. The same libngspice path feeds the future WASM in-browser run (P5).
option(ENABLE_NGSPICE "Build the in-process libngspice runner (src/NgspiceRunner.cpp)" OFF)
find_library(NGSPICE_LIB ngspice)
find_path(NGSPICE_INCLUDE_DIR ngspice/sharedspice.h)
if (NGSPICE_LIB AND NGSPICE_INCLUDE_DIR AND NOT DEFINED CACHE{ENABLE_NGSPICE_SET})
    set(ENABLE_NGSPICE ON CACHE BOOL "" FORCE)   # auto-enable when libngspice is present
    set(ENABLE_NGSPICE_SET ON CACHE INTERNAL "")
endif()

add_library(kirchhoff STATIC src/Flyback.cpp src/Boost.cpp src/Buck.cpp src/Forward.cpp src/TwoSwitchForward.cpp src/Sepic.cpp src/Cuk.cpp src/Zeta.cpp src/PushPull.cpp src/Psfb.cpp src/Ahb.cpp src/Acf.cpp src/Fsbb.cpp src/Pshb.cpp src/Dab.cpp src/IsolatedBuck.cpp src/IsolatedBuckBoost.cpp src/Weinberg.cpp src/Llc.cpp src/Src.cpp src/Cllc.cpp src/Clllc.cpp src/Pfc.cpp src/Vienna.cpp src/TasAssembler.cpp src/NgspiceRunner.cpp src/Analytical.cpp src/AnalyticalDsp.cpp)
target_include_directories(kirchhoff PUBLIC
    src
    "${PSMA_ROOT}/PEAS/src"
    "${PSMA_ROOT}/RAS/src" "${PSMA_ROOT}/CAS/src" "${PSMA_ROOT}/SAS/src"
    "${PSMA_ROOT}/AAS/src" "${PSMA_ROOT}/CTAS/src"
    "${PSMA_ROOT}/MAS/src" "${PSMA_ROOT}/CIAS/src"
    # MAS.hpp (the single generated MAS::Topology enum, used by src/Topology.hpp) is exported by
    # MAS_kirchhoff but does `#include "json.hpp"`, so consumers also need the nlohmann single-header
    # dir on the path (MAS_kirchhoff keeps that PRIVATE).
    "${nlohmann_json_SOURCE_DIR}/include/nlohmann")
target_link_libraries(kirchhoff PUBLIC RAS CAS SAS AAS CTAS MAS_kirchhoff CIAS nlohmann_json::nlohmann_json)
if (ENABLE_NGSPICE AND NGSPICE_LIB AND NGSPICE_INCLUDE_DIR)
    target_compile_definitions(kirchhoff PUBLIC ENABLE_NGSPICE)
    target_include_directories(kirchhoff PUBLIC "${NGSPICE_INCLUDE_DIR}")
    target_link_libraries(kirchhoff PUBLIC "${NGSPICE_LIB}")
    message(STATUS "Kirchhoff: in-process libngspice runner ENABLED (${NGSPICE_LIB})")
else()
    message(STATUS "Kirchhoff: in-process libngspice runner disabled (CLI fallback)")
endif()

add_executable(flyback_demo examples/flyback_demo.cpp)
target_link_libraries(flyback_demo PRIVATE kirchhoff)

add_executable(tas_flyback_demo examples/tas_flyback_demo.cpp)
target_link_libraries(tas_flyback_demo PRIVATE kirchhoff)

add_executable(boost_demo examples/boost_demo.cpp)
target_link_libraries(boost_demo PRIVATE kirchhoff)

add_executable(psfb_demo examples/psfb_demo.cpp)
target_link_libraries(psfb_demo PRIVATE kirchhoff)

add_executable(ahb_demo examples/ahb_demo.cpp)
target_link_libraries(ahb_demo PRIVATE kirchhoff)

add_executable(acf_demo examples/acf_demo.cpp)
target_link_libraries(acf_demo PRIVATE kirchhoff)

add_executable(fsbb_demo examples/fsbb_demo.cpp)
target_link_libraries(fsbb_demo PRIVATE kirchhoff)

add_executable(pshb_demo examples/pshb_demo.cpp)
target_link_libraries(pshb_demo PRIVATE kirchhoff)

# Catch2 (house test framework) — shared by all Kirchhoff C++ tests.
include(FetchContent)
FetchContent_Declare(Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.8.1)
FetchContent_MakeAvailable(Catch2)

add_executable(test_mkf_equivalence tests/test_mkf_equivalence.cpp)
target_compile_definitions(test_mkf_equivalence PRIVATE KIRCHHOFF_TEST_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests")
target_link_libraries(test_mkf_equivalence PRIVATE kirchhoff Catch2::Catch2WithMain)

add_executable(test_real_fidelity tests/test_real_fidelity.cpp)
target_link_libraries(test_real_fidelity PRIVATE kirchhoff Catch2::Catch2WithMain)

add_executable(test_real_magnetic tests/test_real_magnetic.cpp)
target_link_libraries(test_real_magnetic PRIVATE kirchhoff Catch2::Catch2WithMain)

enable_testing()
# Tests are Catch2 (house standard). The *_demo executables above are illustrative EXAMPLES, not
# tests — they stay buildable (so an API break fails the build) but are NOT registered with ctest;
# their behavior (boost/flyback design -> assemble -> simulate) is covered with tighter assertions by
# the Catch2 mkf_equivalence suite. (tas_schema_valid below is intentionally Python — it uses jsonschema.)
add_test(NAME mkf_equivalence COMMAND test_mkf_equivalence)
add_test(NAME real_fidelity COMMAND test_real_fidelity)
add_test(NAME real_magnetic COMMAND test_real_magnetic)

# --- PyKirchhoff: pybind11 module for the orchestrator ---
option(KIRCHHOFF_BUILD_PYBIND "Build the PyKirchhoff pybind11 module" ON)
if (KIRCHHOFF_BUILD_PYBIND AND NOT EMSCRIPTEN)
    include(FetchContent)
    FetchContent_Declare(pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git GIT_TAG v2.13.6)
    FetchContent_Declare(pybind11_json
        GIT_REPOSITORY https://github.com/pybind/pybind11_json.git)
    FetchContent_MakeAvailable(pybind11 pybind11_json)
    pybind11_add_module(PyKirchhoff src/bindings.cpp)
    target_include_directories(PyKirchhoff PRIVATE "${pybind11_json_SOURCE_DIR}/include")
    target_link_libraries(PyKirchhoff PRIVATE kirchhoff)

    # Schema-conformance of Kirchhoff's generated TAS docs (needs jsonschema + referencing).
    add_test(NAME tas_schema_valid
             COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_tas_schema.py)
    set_tests_properties(tas_schema_valid PROPERTIES
        ENVIRONMENT "PYTHONPATH=${CMAKE_BINARY_DIR};KIRCHHOFF_PSMA_ROOT=${PSMA_ROOT}")
endif()

add_executable(clllc_demo examples/clllc_demo.cpp)
target_link_libraries(clllc_demo PRIVATE kirchhoff)

add_executable(test_control_stage tests/test_control_stage.cpp)
target_link_libraries(test_control_stage PRIVATE kirchhoff Catch2::Catch2WithMain)

add_executable(pfc_demo examples/pfc_demo.cpp)
target_link_libraries(pfc_demo PRIVATE kirchhoff)

add_executable(test_pfc tests/test_pfc.cpp)
target_link_libraries(test_pfc PRIVATE kirchhoff Catch2::Catch2WithMain)

add_executable(test_ngspice_runner tests/test_ngspice_runner.cpp)
target_link_libraries(test_ngspice_runner PRIVATE kirchhoff Catch2::Catch2WithMain)

add_executable(test_analytical tests/test_analytical.cpp)
target_link_libraries(test_analytical PRIVATE kirchhoff Catch2::Catch2WithMain)

add_executable(test_ptp tests/test_ptp.cpp)
target_link_libraries(test_ptp PRIVATE kirchhoff Catch2::Catch2WithMain)

add_executable(test_analytical_dsp tests/test_analytical_dsp.cpp)
target_link_libraries(test_analytical_dsp PRIVATE kirchhoff Catch2::Catch2WithMain)

add_executable(test_settings tests/test_settings.cpp)
target_link_libraries(test_settings PRIVATE kirchhoff Catch2::Catch2WithMain)
# Lets the test locate the independent Python averaged-model oracle (tests/averaged_model.py).
target_compile_definitions(test_pfc PRIVATE KIRCHHOFF_TEST_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests")

add_executable(vienna_demo examples/vienna_demo.cpp)
target_link_libraries(vienna_demo PRIVATE kirchhoff)

add_executable(test_vienna tests/test_vienna.cpp)
target_link_libraries(test_vienna PRIVATE kirchhoff Catch2::Catch2WithMain)
target_compile_definitions(test_vienna PRIVATE KIRCHHOFF_TEST_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests")

# Second-backend (#2): CIAS -> LTspice dialect, structural + (Wine-gated) execution cross-check.
add_executable(test_ltspice_backend tests/test_ltspice_backend.cpp)
target_link_libraries(test_ltspice_backend PRIVATE kirchhoff Catch2::Catch2WithMain)

# Requirements-conformance gate: every topology must deliver its spec'd output voltage + power.
add_executable(test_requirements tests/test_requirements.cpp)
target_link_libraries(test_requirements PRIVATE kirchhoff Catch2::Catch2WithMain)
target_compile_definitions(test_requirements PRIVATE KIRCHHOFF_TEST_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests")
