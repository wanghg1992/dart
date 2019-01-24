#include <dart/dart.hpp>
#include <pybind11/pybind11.h>

namespace dart {
namespace python {

void Subject(pybind11::module& m)
{
  ::pybind11::
      class_<dart::common::Subject, std::shared_ptr<dart::common::Subject>>(
          m, "Subject");
}

} // namespace python
} // namespace dart
