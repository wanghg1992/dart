#include <dart/dart.hpp>
#include <pybind11/pybind11.h>
#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"

namespace dart {
namespace python {

void Frame(pybind11::module& m)
{
  ::pybind11::class_<dart::dynamics::Frame>(m, "Frame")
      .def(
          "getTransform",
          +[](const dart::dynamics::Frame* self) -> Eigen::Isometry3d {
            return self->getTransform();
          })
      .def(
          "getTransform",
          +[](const dart::dynamics::Frame* self,
              const dart::dynamics::Frame* _withRespectTo)
              -> Eigen::Isometry3d {
            return self->getTransform(_withRespectTo);
          },
          ::pybind11::arg("withRespectTo"))
      .def(
          "getTransform",
          +[](const dart::dynamics::Frame* self,
              const dart::dynamics::Frame* withRespectTo,
              const dart::dynamics::Frame* inCoordinatesOf)
              -> Eigen::Isometry3d {
            return self->getTransform(withRespectTo, inCoordinatesOf);
          },
          ::pybind11::arg("withRespectTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getSpatialVelocity",
          +[](const dart::dynamics::Frame* self,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector6d {
            return self->getSpatialVelocity(_relativeTo, _inCoordinatesOf);
          },
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getSpatialVelocity",
          +[](const dart::dynamics::Frame* self, const Eigen::Vector3d& _offset)
              -> Eigen::Vector6d { return self->getSpatialVelocity(_offset); },
          ::pybind11::arg("offset"))
      .def(
          "getSpatialVelocity",
          +[](const dart::dynamics::Frame* self,
              const Eigen::Vector3d& _offset,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector6d {
            return self->getSpatialVelocity(
                _offset, _relativeTo, _inCoordinatesOf);
          },
          ::pybind11::arg("offset"),
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getLinearVelocity",
          +[](const dart::dynamics::Frame* self) -> Eigen::Vector3d {
            return self->getLinearVelocity();
          })
      .def(
          "getLinearVelocity",
          +[](const dart::dynamics::Frame* self,
              const dart::dynamics::Frame* _relativeTo) -> Eigen::Vector3d {
            return self->getLinearVelocity(_relativeTo);
          },
          ::pybind11::arg("relativeTo"))
      .def(
          "getLinearVelocity",
          +[](const dart::dynamics::Frame* self,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector3d {
            return self->getLinearVelocity(_relativeTo, _inCoordinatesOf);
          },
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getLinearVelocity",
          +[](const dart::dynamics::Frame* self, const Eigen::Vector3d& _offset)
              -> Eigen::Vector3d { return self->getLinearVelocity(_offset); },
          ::pybind11::arg("offset"))
      .def(
          "getLinearVelocity",
          +[](const dart::dynamics::Frame* self,
              const Eigen::Vector3d& _offset,
              const dart::dynamics::Frame* _relativeTo) -> Eigen::Vector3d {
            return self->getLinearVelocity(_offset, _relativeTo);
          },
          ::pybind11::arg("offset"),
          ::pybind11::arg("relativeTo"))
      .def(
          "getLinearVelocity",
          +[](const dart::dynamics::Frame* self,
              const Eigen::Vector3d& _offset,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector3d {
            return self->getLinearVelocity(
                _offset, _relativeTo, _inCoordinatesOf);
          },
          ::pybind11::arg("offset"),
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getAngularVelocity",
          +[](const dart::dynamics::Frame* self) -> Eigen::Vector3d {
            return self->getAngularVelocity();
          })
      .def(
          "getAngularVelocity",
          +[](const dart::dynamics::Frame* self,
              const dart::dynamics::Frame* _relativeTo) -> Eigen::Vector3d {
            return self->getAngularVelocity(_relativeTo);
          },
          ::pybind11::arg("relativeTo"))
      .def(
          "getAngularVelocity",
          +[](const dart::dynamics::Frame* self,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector3d {
            return self->getAngularVelocity(_relativeTo, _inCoordinatesOf);
          },
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getSpatialAcceleration",
          +[](const dart::dynamics::Frame* self,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector6d {
            return self->getSpatialAcceleration(_relativeTo, _inCoordinatesOf);
          },
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getSpatialAcceleration",
          +[](const dart::dynamics::Frame* self,
              const Eigen::Vector3d& _offset) -> Eigen::Vector6d {
            return self->getSpatialAcceleration(_offset);
          },
          ::pybind11::arg("offset"))
      .def(
          "getSpatialAcceleration",
          +[](const dart::dynamics::Frame* self,
              const Eigen::Vector3d& _offset,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector6d {
            return self->getSpatialAcceleration(
                _offset, _relativeTo, _inCoordinatesOf);
          },
          ::pybind11::arg("offset"),
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getLinearAcceleration",
          +[](const dart::dynamics::Frame* self) -> Eigen::Vector3d {
            return self->getLinearAcceleration();
          })
      .def(
          "getLinearAcceleration",
          +[](const dart::dynamics::Frame* self,
              const dart::dynamics::Frame* _relativeTo) -> Eigen::Vector3d {
            return self->getLinearAcceleration(_relativeTo);
          },
          ::pybind11::arg("relativeTo"))
      .def(
          "getLinearAcceleration",
          +[](const dart::dynamics::Frame* self,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector3d {
            return self->getLinearAcceleration(_relativeTo, _inCoordinatesOf);
          },
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getLinearAcceleration",
          +[](const dart::dynamics::Frame* self,
              const Eigen::Vector3d& _offset) -> Eigen::Vector3d {
            return self->getLinearAcceleration(_offset);
          },
          ::pybind11::arg("offset"))
      .def(
          "getLinearAcceleration",
          +[](const dart::dynamics::Frame* self,
              const Eigen::Vector3d& _offset,
              const dart::dynamics::Frame* _relativeTo) -> Eigen::Vector3d {
            return self->getLinearAcceleration(_offset, _relativeTo);
          },
          ::pybind11::arg("offset"),
          ::pybind11::arg("relativeTo"))
      .def(
          "getLinearAcceleration",
          +[](const dart::dynamics::Frame* self,
              const Eigen::Vector3d& _offset,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector3d {
            return self->getLinearAcceleration(
                _offset, _relativeTo, _inCoordinatesOf);
          },
          ::pybind11::arg("offset"),
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getAngularAcceleration",
          +[](const dart::dynamics::Frame* self) -> Eigen::Vector3d {
            return self->getAngularAcceleration();
          })
      .def(
          "getAngularAcceleration",
          +[](const dart::dynamics::Frame* self,
              const dart::dynamics::Frame* _relativeTo) -> Eigen::Vector3d {
            return self->getAngularAcceleration(_relativeTo);
          },
          ::pybind11::arg("relativeTo"))
      .def(
          "getAngularAcceleration",
          +[](const dart::dynamics::Frame* self,
              const dart::dynamics::Frame* _relativeTo,
              const dart::dynamics::Frame* _inCoordinatesOf)
              -> Eigen::Vector3d {
            return self->getAngularAcceleration(_relativeTo, _inCoordinatesOf);
          },
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getChildEntities",
          +[](const dart::dynamics::Frame* self)
              -> const std::set<const dart::dynamics::Entity*> {
            return self->getChildEntities();
          })
      .def(
          "getNumChildEntities",
          +[](const dart::dynamics::Frame* self) -> std::size_t {
            return self->getNumChildEntities();
          })
      .def(
          "getChildFrames",
          +[](const dart::dynamics::Frame* self)
              -> std::set<const dart::dynamics::Frame*> {
            return self->getChildFrames();
          })
      .def(
          "getNumChildFrames",
          +[](const dart::dynamics::Frame* self) -> std::size_t {
            return self->getNumChildFrames();
          })
      .def(
          "isShapeFrame",
          +[](const dart::dynamics::Frame* self) -> bool {
            return self->isShapeFrame();
          })
      .def(
          "isWorld",
          +[](const dart::dynamics::Frame* self) -> bool {
            return self->isWorld();
          })
      .def(
          "dirtyTransform",
          +[](dart::dynamics::Frame* self) { self->dirtyTransform(); })
      .def(
          "dirtyVelocity",
          +[](dart::dynamics::Frame* self) { self->dirtyVelocity(); })
      .def("dirtyAcceleration", +[](dart::dynamics::Frame* self) {
        self->dirtyAcceleration();
      });
}

} // namespace python
} // namespace dart
