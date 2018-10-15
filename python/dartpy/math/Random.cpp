#include <dart/dart.hpp>
#include <pybind11/pybind11.h>

namespace dart {
namespace python {

void Random(pybind11::module& m)
{
  ::pybind11::class_<dart::math::Random >(m, "Random")
      .def(::pybind11::init<>())
      .def_static("setSeed", +[](unsigned int seed) { dart::math::Random::setSeed(seed); }, ::pybind11::arg("seed"))
      .def_static("getSeed", +[]() -> unsigned int { return dart::math::Random::getSeed(); })
      .def_static("uniform", +[](double min, double max) -> double { return dart::math::Random::uniform(min, max); }, ::pybind11::arg("min"), ::pybind11::arg("max"))
      ;
}

} // namespace python
} // namespace dart
