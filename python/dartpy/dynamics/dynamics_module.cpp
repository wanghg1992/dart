#include <pybind11/pybind11.h>

namespace dart {
namespace python {

void Joint(pybind11::module& sm);
void Skeleton(pybind11::module& sm);

void dart_dynamics(pybind11::module& m)
{
  auto sm = m.def_submodule("dynamics");

  Joint(sm);

  Skeleton(sm);
}

} // namespace python
} // namespace dart
