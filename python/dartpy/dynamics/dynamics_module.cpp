#include <pybind11/pybind11.h>

namespace dart {
namespace python {

void Frame(pybind11::module& sm);
void BodyNode(pybind11::module& sm);
void Joint(pybind11::module& sm);
void MetaSkeleton(pybind11::module& sm);
void Skeleton(pybind11::module& sm);

void dart_dynamics(pybind11::module& m)
{
  auto sm = m.def_submodule("dynamics");

  Frame(sm);
  BodyNode(sm);
  Joint(sm);
  MetaSkeleton(sm);
  Skeleton(sm);
}

} // namespace python
} // namespace dart
