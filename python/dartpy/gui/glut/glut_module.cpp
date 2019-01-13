#include <pybind11/pybind11.h>

namespace dart {
namespace python {

void SimWindow(pybind11::module& sm);

void dart_gui_glut(pybind11::module& m)
{
  auto sm = m.def_submodule("glut");

  SimWindow(sm);
}

} // namespace python
} // namespace dart
