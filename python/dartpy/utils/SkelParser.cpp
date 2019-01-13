#include <dart/dart.hpp>
#include <dart/utils/utils.hpp>
#include <pybind11/pybind11.h>

namespace dart {
namespace python {

void SkelParser(pybind11::module& m)
{
  auto sm = m.def_submodule("SkelParser");

  sm.def("readWorld", +[](const common::Uri& uri) -> simulation::WorldPtr {
    return utils::SkelParser::readWorld(uri);
  });
}

} // namespace python
} // namespace dart
