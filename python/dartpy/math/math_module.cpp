#include <pybind11/pybind11.h>

namespace dart {
namespace python {

void dart_math(pybind11::module& m)
{
  auto sm = m.def_submodule("math");

  void Random(pybind11::module& sm);
  Random(sm);
}

} // namespace python
} // namespace dart
