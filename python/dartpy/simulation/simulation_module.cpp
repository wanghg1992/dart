#include <pybind11/pybind11.h>

namespace dart {
namespace python {

void World(pybind11::module& sm);

void dart_simulation(pybind11::module& m)
{
  auto sm = m.def_submodule("simulation");

  World(sm);
}

} // namespace python
} // namespace dart
