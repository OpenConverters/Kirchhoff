cmake_minimum_required(VERSION 3.15)
project(Kirchhoff VERSION 1.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

# Emscripten disables C++ exception CATCHING by default (throws still abort). libKirchhoff's guarded()
# wrapper must actually catch, and it must be consistent across every TU (this lib + the submodule deps),
# so enable it globally for the WASM build at both compile and link. Applied before add_subdirectory so
# the submodule targets pick it up too. We use the NATIVE wasm EH (-fwasm-exceptions), NOT the legacy
# JS-based -fexceptions, because the in-browser ngspice (ENABLE_NGSPICE for wasm) is built with
# -fwasm-exceptions + -sSUPPORT_LONGJMP=wasm (WebLibMKF's recipe) and the two EH models cannot be mixed in
# one module — matching it here is what lets KH's wasm build link libngspice and simulate in the browser.
if (EMSCRIPTEN)
    add_compile_options(-fwasm-exceptions)
    add_link_options(-fwasm-exceptions -sSUPPORT_LONGJMP=wasm)
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
# Honour an explicit -DENABLE_NGSPICE=ON/OFF; only AUTO-enable (when libngspice is found) if the user did
# NOT specify it. A command-line -D populates the cache BEFORE this script runs, so DEFINED distinguishes an
# explicit choice from option()'s default — this is what keeps a fresh `-DENABLE_NGSPICE=OFF` from being
# silently forced back ON (no silent override of an explicit user choice).
if (DEFINED ENABLE_NGSPICE)
    set(_ngspice_user_set TRUE)
else()
    set(_ngspice_user_set FALSE)
endif()
option(ENABLE_NGSPICE "Build the in-process libngspice runner (src/NgspiceRunner.cpp)" OFF)
find_library(NGSPICE_LIB ngspice)
find_path(NGSPICE_INCLUDE_DIR ngspice/sharedspice.h)
if (NGSPICE_LIB AND NGSPICE_INCLUDE_DIR AND NOT _ngspice_user_set)
    set(ENABLE_NGSPICE ON CACHE BOOL "" FORCE)   # auto-enable only when unspecified and libngspice is present
endif()

# The converter-waveform DSP (FFT, create_waveform, sampled/processed/harmonics, complete_excitation)
# is NOT re-implemented in Kirchhoff — it lives once in MKF's self-contained WaveformProcessor (extracted
# out of Inputs.cpp; depends only on the MAS types + std, no magnetics/Settings). Kirchhoff compiles that
# single file directly (no dependency on the rest of MKF). MKF is vendored as the deps/MKF git submodule
# (pinned to a specific OpenMagnetics/MKF commit) so a fresh clone is reproducible: run
#   git submodule update --init deps/MKF
# MKF_DIR defaults to that submodule but stays overridable — point it at a live local MKF checkout for
# co-development (e.g. -DMKF_DIR=/home/alf/OpenMagnetics/MKF).
set(MKF_DIR "${CMAKE_CURRENT_SOURCE_DIR}/deps/MKF" CACHE PATH "Path to the MKF checkout (for the shared WaveformProcessor)")
set(MKF_WAVEFORM_PROCESSOR "${MKF_DIR}/src/processors/WaveformProcessor.cpp")
# Fail configure with a clear message instead of a cryptic mid-compile "file not found" when MKF_DIR does
# not point at a real checkout (e.g. the submodule was never initialised).
if (NOT EXISTS "${MKF_WAVEFORM_PROCESSOR}")
    message(FATAL_ERROR
        "MKF WaveformProcessor not found:\n  ${MKF_WAVEFORM_PROCESSOR}\n"
        "Kirchhoff compiles this one self-contained MKF file for its converter-waveform DSP. "
        "Initialise the submodule ('git submodule update --init deps/MKF') or point "
        "-DMKF_DIR=<path-to-MKF-checkout> at a local MKF clone.")
endif()

add_library(kirchhoff STATIC src/Flyback.cpp src/Boost.cpp src/Buck.cpp src/Forward.cpp src/TwoSwitchForward.cpp src/Sepic.cpp src/Cuk.cpp src/Zeta.cpp src/PushPull.cpp src/Psfb.cpp src/Ahb.cpp src/Acf.cpp src/Fsbb.cpp src/Pshb.cpp src/Dab.cpp src/IsolatedBuck.cpp src/IsolatedBuckBoost.cpp src/Weinberg.cpp src/Llc.cpp src/Src.cpp src/Cllc.cpp src/Clllc.cpp src/Pfc.cpp src/Vienna.cpp src/Cmc.cpp src/CmcSim.cpp src/Dmc.cpp src/DmcSim.cpp src/CurrentTransformer.cpp src/TasAssembler.cpp src/NgspiceRunner.cpp src/Analytical.cpp src/ConverterAnalytical.cpp src/ConverterExtract.cpp src/ComponentWaveforms.cpp src/DatasheetModels.cpp "${MKF_WAVEFORM_PROCESSOR}")
target_include_directories(kirchhoff PUBLIC src)
# The PSMA/MAS/json dependency headers are marked SYSTEM so the -Wall/-Wextra enabled below reports only
# Kirchhoff's OWN code, not warnings inside generated/vendored dependency headers (e.g. MAS.hpp).
target_include_directories(kirchhoff SYSTEM PUBLIC
    "${PSMA_ROOT}/PEAS/src"
    "${PSMA_ROOT}/RAS/src" "${PSMA_ROOT}/CAS/src" "${PSMA_ROOT}/SAS/src"
    "${PSMA_ROOT}/AAS/src" "${PSMA_ROOT}/CTAS/src"
    "${PSMA_ROOT}/MAS/src" "${PSMA_ROOT}/CIAS/src"
    # MAS.hpp (the single generated MAS::Topology enum, used by src/Topology.hpp) is exported by
    # MAS_kirchhoff but does `#include "json.hpp"`, so consumers also need the nlohmann single-header
    # dir on the path (MAS_kirchhoff keeps that PRIVATE).
    "${nlohmann_json_SOURCE_DIR}/include/nlohmann")
# MKF's src/ — only for the shared WaveformProcessor's own includes ("processors/WaveformProcessor.h",
# "support/Exceptions.h"), both self-contained (MAS types + std). PRIVATE so it's on the path for
# kirchhoff's own TUs (ConverterAnalytical.cpp + WaveformProcessor.cpp) but does NOT propagate to test
# binaries (no risk of an MKF header shadowing a Kirchhoff one).
target_include_directories(kirchhoff SYSTEM PRIVATE "${MKF_DIR}/src")
target_link_libraries(kirchhoff PUBLIC RAS CAS SAS AAS CTAS MAS_kirchhoff CIAS nlohmann_json::nlohmann_json)
# Compile Kirchhoff's OWN sources with -Wall -Wextra (dep headers are SYSTEM above, so this flags only our
# code). The vendored MKF WaveformProcessor is not ours to clean — suppress its warnings with a trailing -w.
if (NOT MSVC)
    target_compile_options(kirchhoff PRIVATE -Wall -Wextra)
    set_source_files_properties("${MKF_WAVEFORM_PROCESSOR}" PROPERTIES COMPILE_OPTIONS "-w")
endif()
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
        GIT_REPOSITORY https://github.com/pybind/pybind11_json.git GIT_TAG 0.2.15)
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

# --- libKirchhoff: the Emscripten/embind WASM module (the WebLibMKF converter-surface replacement) ---
# Built only under emscripten (emcmake cmake ...). Thin embind over Kirchhoff::api (KirchhoffApi.cpp is
# compiled directly here — it is NOT in the static core lib, so it can also go into the hidden-visibility
# shared lib below without a duplicate symbol).
if (EMSCRIPTEN)
    add_executable(libKirchhoff src/libKirchhoff.cpp src/KirchhoffApi.cpp)
    target_link_libraries(libKirchhoff PRIVATE kirchhoff)
    set_target_properties(libKirchhoff PROPERTIES OUTPUT_NAME "kirchhoff")
    target_link_options(libKirchhoff PRIVATE
        "--bind"
        "-sMODULARIZE=1" "-sEXPORT_NAME=createKirchhoffModule"
        "-sENVIRONMENT=web,worker,node" "-sALLOW_MEMORY_GROWTH=1"
        "-sEXPORT_ES6=1" "-sSINGLE_FILE=1"
        # ngspice is stack- and memory-hungry: match WebLibMKF's proven sizing (the emscripten default
        # 64 KB stack blows out mid-simulation -> "memory access out of bounds"). Harmless when ngspice
        # is off, required when it is on.
        "-sSTACK_SIZE=67108864" "-sINITIAL_MEMORY=134217728" "-sMAXIMUM_MEMORY=4gb")
endif()

# --- libKirchhoffApi: a native SHARED library exposing ONLY Kirchhoff::api (the string/JSON facade) with
# hidden internals. This is how MKF / WebLibMKF link Kirchhoff WITHOUT a symbol clash: KH and MKF BOTH
# generate their types into `namespace MAS`, so linking KH's objects directly into MKF would collide. Here
# the whole typed core + all deps are bundled into the .so and compiled -fvisibility=hidden; only the
# KH_API-marked string functions are exported, so KH's MAS:: symbols never cross the library boundary.
# Off by default (native tests/demos don't need it); MKF's ExternalProject turns it ON.
option(KIRCHHOFF_BUILD_SHARED_API "Build libKirchhoffApi.so (string API only, hidden internals)" OFF)
if (KIRCHHOFF_BUILD_SHARED_API AND NOT EMSCRIPTEN)
    add_library(KirchhoffApi SHARED src/KirchhoffApi.cpp)
    target_compile_definitions(KirchhoffApi PRIVATE KIRCHHOFF_API_EXPORTS)
    target_compile_options(KirchhoffApi PRIVATE -fvisibility=hidden -fvisibility-inlines-hidden)
    # Pull the ENTIRE static core (+ its deps) into the .so so nothing dangles, but keep it all hidden.
    target_link_libraries(KirchhoffApi PRIVATE
        -Wl,--whole-archive kirchhoff -Wl,--no-whole-archive
        RAS CAS SAS AAS CTAS MAS_kirchhoff CIAS nlohmann_json::nlohmann_json)
    if (ENABLE_NGSPICE AND NGSPICE_LIB)
        target_link_libraries(KirchhoffApi PRIVATE "${NGSPICE_LIB}")
    endif()
    # the static core was compiled WITHOUT -fvisibility=hidden, so hide its symbols at link with a version
    # script that exports only the C++ mangled Kirchhoff::api names (and the std/global new/delete the .so
    # must still export for its own allocations are kept local via the default localizer).
    target_link_options(KirchhoffApi PRIVATE
        "-Wl,--exclude-libs,ALL")   # localize every symbol pulled from the linked static archives
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

add_executable(test_converter_analytical tests/test_converter_analytical.cpp)
target_link_libraries(test_converter_analytical PRIVATE kirchhoff Catch2::Catch2WithMain)

add_executable(test_nrmse_gate tests/test_nrmse_gate.cpp)
target_link_libraries(test_nrmse_gate PRIVATE kirchhoff Catch2::Catch2WithMain)

add_executable(test_resonant_validation tests/test_resonant_validation.cpp)
target_link_libraries(test_resonant_validation PRIVATE kirchhoff Catch2::Catch2WithMain)

add_executable(test_extract tests/test_extract.cpp src/KirchhoffApi.cpp)
target_link_libraries(test_extract PRIVATE kirchhoff Catch2::Catch2WithMain)

add_executable(test_cmc tests/test_cmc.cpp src/KirchhoffApi.cpp)
target_link_libraries(test_cmc PRIVATE kirchhoff Catch2::Catch2WithMain)

add_executable(test_dmc_ct tests/test_dmc_ct.cpp src/KirchhoffApi.cpp)
target_link_libraries(test_dmc_ct PRIVATE kirchhoff Catch2::Catch2WithMain)

add_executable(test_component_waveforms tests/test_component_waveforms.cpp src/KirchhoffApi.cpp)
target_link_libraries(test_component_waveforms PRIVATE kirchhoff Catch2::Catch2WithMain)

add_executable(test_advanced tests/test_advanced.cpp)
target_link_libraries(test_advanced PRIVATE kirchhoff Catch2::Catch2WithMain)

add_executable(test_diagnostics tests/test_diagnostics.cpp)
target_link_libraries(test_diagnostics PRIVATE kirchhoff Catch2::Catch2WithMain)

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


# Native consumer of libKirchhoffApi.so (mimics MKF's bridge: string header only, no MAS types) — a
# regression guard that the isolated shared-API link keeps working. Built with the shared API.
if (KIRCHHOFF_BUILD_SHARED_API AND NOT EMSCRIPTEN)
    add_executable(api_consumer examples/api_consumer.cpp)
    target_include_directories(api_consumer PRIVATE src "${PSMA_ROOT}/PEAS/src")
    target_link_libraries(api_consumer PRIVATE KirchhoffApi)
endif()

# Enforce -Wall -Wextra on Kirchhoff's own tests + example demos too (their sources are already clean —
# this keeps them so). Dependency headers are marked SYSTEM above and Catch2 is external, so only our code
# is flagged. The binding modules (PyKirchhoff / libKirchhoff / KirchhoffApi) are intentionally excluded —
# their pybind11 / embind headers are noisy under -Wextra and carry no converter logic.
if (NOT MSVC)
    foreach(_kh_warn_target
            test_advanced test_analytical test_control_stage test_converter_analytical test_diagnostics
            test_extract test_ltspice_backend test_mkf_equivalence test_ngspice_runner test_nrmse_gate
            test_pfc test_ptp test_real_fidelity test_real_magnetic test_requirements
            test_resonant_validation test_settings test_vienna
            flyback_demo tas_flyback_demo boost_demo psfb_demo ahb_demo acf_demo fsbb_demo pshb_demo
            clllc_demo pfc_demo vienna_demo api_consumer)
        if (TARGET ${_kh_warn_target})
            target_compile_options(${_kh_warn_target} PRIVATE -Wall -Wextra)
        endif()
    endforeach()
endif()
