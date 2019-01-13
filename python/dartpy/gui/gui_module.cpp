#include <pybind11/pybind11.h>

namespace dart {
namespace python {

void dart_gui_glut(pybind11::module& m);

void dart_gui(pybind11::module& m)
{
  auto sm = m.def_submodule("gui");

  dart_gui_glut(sm);
}

} // namespace python
} // namespace dart
