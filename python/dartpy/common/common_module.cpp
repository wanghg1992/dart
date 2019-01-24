#include <pybind11/pybind11.h>

namespace dart {
namespace python {

// void Subject(pybind11::module& sm);
void Uri(pybind11::module& sm);

void dart_common(pybind11::module& m)
{
  auto sm = m.def_submodule("common");

  //  Subject(sm);
  Uri(sm);
}

} // namespace python
} // namespace dart
