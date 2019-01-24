#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

void eigen_geometry(pybind11::module& m);

void dart_common(pybind11::module& m);
void dart_math(pybind11::module& m);
void dart_dynamics(pybind11::module& m);
void dart_simulation(pybind11::module& m);
void dart_utils(pybind11::module& m);
void dart_gui(pybind11::module& m);

PYBIND11_MODULE(dartpy, m)
{
  py::module::import("numpy");

  m.doc() = "DART python bindings";

  eigen_geometry(m);

  dart_common(m);
  dart_math(m);
  dart_dynamics(m);
  dart_simulation(m);
  dart_utils(m);
  dart_gui(m);
}

} // namespace python
} // namespace dart
