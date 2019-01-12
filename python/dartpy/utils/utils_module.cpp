#include <pybind11/pybind11.h>

namespace dart {
namespace python {

void DartLoader(pybind11::module& sm);

void dart_utils(pybind11::module& m)
{
  auto sm = m.def_submodule("utils");

  DartLoader(sm);
}

} // namespace python
} // namespace dart
