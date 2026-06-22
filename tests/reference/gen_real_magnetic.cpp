// Spike: design REAL magnetics for current topologies via MKF and export their ngspice subcircuits.
//   converter -> MagneticAdviser.get_advised_magnetic_from_converter -> Magnetic
//             -> CircuitSimulatorExporter(NGSPICE).export_magnetic_as_subcircuit
// (all magnetics math lives in MKF). Writes /tmp/<topology>_mag.subckt for the validation tool.

#include <iostream>
#include <fstream>
#include "converter_models/Boost.h"
#include "converter_models/Flyback.h"
#include "converter_models/SingleSwitchForward.h"
#include "advisers/MagneticAdviser.h"
#include "processors/CircuitSimulatorInterface.h"

using namespace OpenMagnetics;
using namespace MAS;

template <class Converter>
void export_for(Converter& conv, const std::string& tag, const std::string& outPath) {
    conv.process_design_requirements();
    MagneticAdviser adviser;
    auto results = adviser.get_advised_magnetic_from_converter(conv, 1);
    if (results.empty()) { std::cerr << tag << ": MagneticAdviser returned no magnetic\n"; return; }
    OpenMagnetics::Magnetic magnetic = results[0].first.get_magnetic();
    std::string subckt = CircuitSimulatorExporter(CircuitSimulatorExporterModels::NGSPICE)
        .export_magnetic_as_subcircuit(magnetic, 100e3, 25.0);
    std::ofstream(outPath) << subckt;
    std::cerr << tag << ": " << magnetic.get_reference() << "  -> " << outPath
              << " (" << subckt.size() << " bytes)\n";
}

int main() {
    {   // boost: single-winding inductor
        OpenMagnetics::Boost b;
        DimensionWithTolerance iv; iv.set_nominal(12.0); iv.set_minimum(11.4); iv.set_maximum(12.6);
        b.set_input_voltage(iv); b.set_diode_voltage_drop(0.0); b.set_efficiency(1.0);
        b.set_current_ripple_ratio(0.4);
        BaseOperatingPoint op; op.set_output_voltages({24.0}); op.set_output_currents({1.0});
        op.set_switching_frequency(100e3); op.set_ambient_temperature(25.0);
        b.set_operating_points({op});
        export_for(b, "boost", "/tmp/boost_mag.subckt");
    }
    {   // flyback: 2-winding transformer (primary + secondary)
        OpenMagnetics::Flyback f;
        DimensionWithTolerance iv; iv.set_nominal(48.0); iv.set_minimum(45.6); iv.set_maximum(50.4);
        f.set_input_voltage(iv); f.set_diode_voltage_drop(0.0); f.set_efficiency(1.0);
        f.set_current_ripple_ratio(0.4); f.set_maximum_duty_cycle(0.5);
        OpenMagnetics::FlybackOperatingPoint op; op.set_output_voltages({12.0}); op.set_output_currents({2.0});
        op.set_switching_frequency(100e3); op.set_ambient_temperature(25.0);
        f.set_operating_points(std::vector<OpenMagnetics::FlybackOperatingPoint>{op});
        export_for(f, "flyback", "/tmp/flyback_mag.subckt");
    }
    {   // single-switch forward: 3-winding transformer (primary + demag + secondary)
        OpenMagnetics::SingleSwitchForward fwd;
        DimensionWithTolerance iv; iv.set_nominal(48.0); iv.set_minimum(45.6); iv.set_maximum(50.4);
        fwd.set_input_voltage(iv); fwd.set_diode_voltage_drop(0.0); fwd.set_efficiency(1.0);
        fwd.set_current_ripple_ratio(0.4); fwd.set_duty_cycle(0.5);
        OpenMagnetics::ForwardOperatingPoint op; op.set_output_voltages({12.0}); op.set_output_currents({2.0});
        op.set_switching_frequency(100e3); op.set_ambient_temperature(25.0);
        fwd.set_operating_points({op});
        export_for(fwd, "forward", "/tmp/forward_mag.subckt");
    }
    return 0;
}
