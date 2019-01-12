#include <pybind11/pybind11.h>

namespace dart {
namespace python {

void Random(pybind11::module& sm);

void dart_math(pybind11::module& m)
{
  auto sm = m.def_submodule("math");

  Random(sm);
}

} // namespace python
} // namespace dart
