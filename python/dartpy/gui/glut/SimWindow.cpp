#include <dart/dart.hpp>
#include <dart/gui/glut/glut.hpp>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

void SimWindow(pybind11::module& m)
{
  ::pybind11::class_<dart::gui::glut::SimWindow>(m, "SimWindow")
      .def(py::init<>());
}

} // namespace python
} // namespace dart
