#include "FlightControlUtils.h"
#include "QPInput.h"
#include "Robot.h"
#include "swig.h"
#include <Eigen/src/Core/Matrix.h>
#include <iDynTree/EigenHelpers.h>
#include <iDynTree/Transform.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Searchable.h>

namespace py = pybind11;

PYBIND11_MODULE(bindings, m)
{
    /************************************************************************/
    /*************************** Robot class ********************************/
    /************************************************************************/
    py::class_<Robot, std::shared_ptr<Robot>>(m, "Robot")
        .def(py::init<>())
        .def("configure",
             [](Robot& impl,
                const std::string modelFullPath,
                std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler>
                    parametersHandler,
                const std::vector<std::string>& refFrameExtWrenchList) {
                 std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> phWeak
                     = parametersHandler;
                 return impl.configure(modelFullPath, phWeak, refFrameExtWrenchList);
             })

        .def("setState",
             [](Robot& impl,
                py::object& wHb,
                py::object& baseVel,
                py::object& positionsInRad,
                py::object& velocitiesInRadS,
                py::EigenDRef<Eigen::VectorXd> jetThrustInNewton,
                std::vector<Eigen::Vector6d>& extWrenchesInNewton) {
                 // for wHb
                 iDynTree::Transform* wHbPtr
                     = pybind11::detail::swig_wrapped_pointer_to_pybind<iDynTree::Transform>(wHb);

                 // for baseVel
                 iDynTree::Twist* baseVelPtr
                     = pybind11::detail::swig_wrapped_pointer_to_pybind<iDynTree::Twist>(baseVel);

                 // for positionsInRad
                 iDynTree::VectorDynSize* positionsInRadPtr
                     = pybind11::detail::swig_wrapped_pointer_to_pybind<iDynTree::VectorDynSize>(
                         positionsInRad);

                 // for velocitiesInRadS
                 iDynTree::VectorDynSize* velocitiesInRadSPtr
                     = pybind11::detail::swig_wrapped_pointer_to_pybind<iDynTree::VectorDynSize>(
                         velocitiesInRadS);

                 return impl.setState(*wHbPtr,
                                      *baseVelPtr,
                                      *positionsInRadPtr,
                                      *velocitiesInRadSPtr,
                                      jetThrustInNewton,
                                      extWrenchesInNewton);
             }

             )
        .def("getBaseLinVel",
             [](Robot& impl) { return Eigen::Vector3d(impl.getBaseVel().getLinearVec3().data()); })
        .def("getBaseAngVel",
             [](Robot& impl) { return Eigen::Vector3d(impl.getBaseVel().getAngularVec3().data()); })
        .def("getBasePosition",
             [](Robot& impl) { return Eigen::Vector3d(impl.getBasePose().getPosition().data()); })
        .def("getBaseOrientation",
             [](Robot& impl) {
                 return Eigen::Vector3d(impl.getBasePose().getRotation().asRPY().data());
             })
        .def("getTotalMass", &Robot::getTotalMass)
        .def("getNJoints", &Robot::getNJoints)
        .def("getAxesList", &Robot::getAxesList)
        .def("getFloatingBaseFrameName", &Robot::getFloatingBaseFrameName)
        .def("getPositionCoM", &Robot::getPositionCoM)
        .def("getMomentum", &Robot::getMomentum, py::arg("inBodyCoord") = false)
        .def("getWorldTransform",
             [](Robot& impl, const std::string& frameName) {
                 iDynTree::Transform transform = impl.getWorldTransform(frameName);
                 auto homogeneousMatrix = transform.asHomogeneousTransform();
                 Eigen::Matrix4d eigenMatrix = iDynTree::toEigen(homogeneousMatrix);
                 return eigenMatrix;
             })
        .def("getMatrixAmomJets", &Robot::getMatrixAmomJets, py::arg("inBodyCoord") = false);

    /************************************************************************/
    /*********************** QP solver related classes **********************/
    /************************************************************************/

    // For the QPInput class
    py::class_<QPInput>(m, "QPInput")
        .def(py::init<>())
        .def("getRobot", &QPInput::getRobot)
        .def("setRobot", &QPInput::setRobot, py::arg("robot"))
        .def("getRobotReference", &QPInput::getRobotReference)
        .def("setRobotReference", &QPInput::setRobotReference, py::arg("robotReference"))
        .def("getVectorsCollectionServer", &QPInput::getVectorsCollectionServer)
        .def("setVectorsCollectionServer",
             &QPInput::setVectorsCollectionServer,
             py::arg("vectorsCollectionServer"))

        .def("getOutputQPThrust", &QPInput::getOutputQPThrust)
        .def("setOutputQPThrust",
             [](QPInput& self, py::EigenDRef<Eigen::VectorXd> outputQPThrust) {
                 self.setOutputQPThrust(outputQPThrust);
             })

        .def("getOutputQPJointsPosition", &QPInput::getOutputQPJointsPosition)
        .def("setOutputQPJointsPosition",
             [](QPInput& self, py::EigenDRef<Eigen::VectorXd> outputQPJointsPosition) {
                 self.setOutputQPJointsPosition(outputQPJointsPosition);
             })

        .def("getOutputQPThrustDot", &QPInput::getOutputQPThrustDot)
        .def("setOutputQPThrustDot",
             [](QPInput& self, py::EigenDRef<Eigen::VectorXd> outputQPThrustDot) {
                 self.setOutputQPThrustDot(outputQPThrustDot);
             })

        .def("getOutputQPJointsVelocity", &QPInput::getOutputQPJointsVelocity)
        .def("setOutputQPJointsVelocity",
             [](QPInput& self, py::EigenDRef<Eigen::VectorXd> outputQPJointsVelocity) {
                 self.setOutputQPJointsVelocity(outputQPJointsVelocity);
             })

        .def("getThrustReference", &QPInput::getThrustReference)
        .def("setThrustReference",
             [](QPInput& self, py::EigenDRef<Eigen::VectorXd> thrustReference) {
                 self.setThrustReference(thrustReference);
             })

        .def("getJointPosReference", &QPInput::getJointPosReference)
        .def("setJointPosReference",
             [](QPInput& self, py::EigenDRef<Eigen::VectorXd> jointPosReference) {
                 self.setJointPosReference(jointPosReference);
             })

        .def("getPosCoMReference", &QPInput::getPosCoMReference)
        .def("setPosCoMReference",
             [](QPInput& self, py::EigenDRef<Eigen::Vector3d> posCoMReference) {
                 self.setPosCoMReference(posCoMReference);
             })

        .def("getBaseRotReference",
             [](QPInput& self) {
                 return Eigen::Vector3d(self.getBaseRotReference().asRPY().data());
             })
        .def("setBaseRotReference",
             [](QPInput& self, py::object& baseRotReference) {
                 iDynTree::Rotation* baseRotReferencePtr
                     = pybind11::detail::swig_wrapped_pointer_to_pybind<iDynTree::Rotation>(
                         baseRotReference);
                 self.setBaseRotReference(*baseRotReferencePtr);
             })

        .def("getThrustJetControl", &QPInput::getThrustJetControl)
        .def("setThrustJetControl",
             [](QPInput& self, py::EigenDRef<Eigen::VectorXd> thrustJetControl) {
                 self.setThrustJetControl(thrustJetControl);
             })

        .def("getOutputQPSolution", &QPInput::getOutputQPSolution)
        .def("setOutputQPSolution",
             [](QPInput& self, py::EigenDRef<Eigen::VectorXd> outputQPSolution) {
                 self.setOutputQPSolution(outputQPSolution);
             })

        .def("getContactWrenchesReference", &QPInput::getContactWrenchesReference)
        .def("setContactWrenchesReference",
             [](QPInput& self, const std::vector<Eigen::Vector6d>& contactWrenchesReference) {
                 self.setContactWrenchesReference(contactWrenchesReference);
             })

        .def("getContactWrenchesFrameName", &QPInput::getContactWrenchesFrameName)
        .def("setContactWrenchesFrameName",
             [](QPInput& self, const std::vector<std::string>& contactWrenchesFrameName) {
                 self.setContactWrenchesFrameName(contactWrenchesFrameName);
             })

        .def("getSumExternalWrenchesInertialFrame", &QPInput::getSumExternalWrenchesInertialFrame)
        .def("setSumExternalWrenchesInertialFrame",
             [](QPInput& self, py::EigenDRef<Eigen::Vector6d> sumExternalWrenchesInertialFrame) {
                 self.setSumExternalWrenchesInertialFrame(sumExternalWrenchesInertialFrame);
             })

        .def("getFlightControllerPeriod", &QPInput::getFlightControllerPeriod)
        .def("setFlightControllerPeriod",
             &QPInput::setFlightControllerPeriod,
             py::arg("flightControllerPeriod"))

        .def("getMomentumReference", &QPInput::getMomentumReference)
        .def("setMomentumReference",
             [](QPInput& self, py::EigenDRef<Eigen::Vector6d> momentumReference) {
                 self.setMomentumReference(momentumReference);
             })

        .def("getMomentumDotReference", &QPInput::getMomentumDotReference)
        .def("setMomentumDotReference",
             [](QPInput& self, py::EigenDRef<Eigen::Vector6d> momentumDotReference) {
                 self.setMomentumDotReference(momentumDotReference);
             })

        .def("getMomentumDotDotReference", &QPInput::getMomentumDotDotReference)
        .def("setMomentumDotDotReference",
             [](QPInput& self, py::EigenDRef<Eigen::Vector6d> momentumDotDotReference) {
                 self.setMomentumDotDotReference(momentumDotDotReference);
             })

        .def("getFreezeAlphaGravity", &QPInput::getFreezeAlphaGravity)
        .def("setFreezeAlphaGravity",
             &QPInput::setFreezeAlphaGravity,
             py::arg("freezeAlphaGravity"))

        .def("getAlphaGravity", &QPInput::getAlphaGravity)
        .def("setAlphaGravity", &QPInput::setAlphaGravity, py::arg("alphaGravity"))

        .def("getThrustDesMPC", &QPInput::getThrustDesMPC)
        .def("setThrustDesMPC",
             [](QPInput& self, py::EigenDRef<Eigen::VectorXd> thrustDesMPC) {
                 self.setThrustDesMPC(thrustDesMPC);
             })

        .def("getThrustDotDesMPC", &QPInput::getThrustDotDesMPC)
        .def("setThrustDotDesMPC",
             [](QPInput& self, py::EigenDRef<Eigen::VectorXd> thrustDotDesMPC) {
                 self.setThrustDotDesMPC(thrustDotDesMPC);
             })

        .def("getThrottleMPC", &QPInput::getThrottleMPC)
        .def("setThrottleMPC",
             [](QPInput& self, py::EigenDRef<Eigen::VectorXd> throttleMPC) {
                 self.setThrottleMPC(throttleMPC);
             })

        .def("getEstimatedThrustDot", &QPInput::getEstimatedThrustDot)
        .def("setEstimatedThrustDot",
             [](QPInput& self, py::EigenDRef<Eigen::VectorXd> estimatedThrustDot) {
                 self.setEstimatedThrustDot(estimatedThrustDot);
             })

        .def("getRPYReference", &QPInput::getRPYReference)
        .def("getPosCoMReference", &QPInput::getPosCoMReference)

        .def("setEmptyVectorsCollectionServer",
             [](QPInput& self) {
                 std::shared_ptr<BipedalLocomotion::YarpUtilities::VectorsCollectionServer>
                     vectorsCollectionServer
                     = std::make_shared<BipedalLocomotion::YarpUtilities::VectorsCollectionServer>();
                 self.setVectorsCollectionServer(vectorsCollectionServer);
             })

        .def("setEmptyJetModel",
             [](QPInput& self) {
                 std::shared_ptr<JetModel> jetModel = std::make_shared<JetModel>();
                 self.setJetModel(jetModel);
             })
        .def("getRPYReference", &QPInput::getRPYReference)
        .def("getPosCoMReference", &QPInput::getPosCoMReference);

    /************************************************************************/
    /*********************** Mathematics related class **********************/
    /************************************************************************/

    py::class_<Eigen::Vector6d>(m, "EigenVector6d", py::buffer_protocol())
        .def(py::init([](py::buffer b) {
            typedef Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic> Strides;

            /* Request a buffer descriptor from Python */
            py::buffer_info info = b.request();

            /* Some basic validation checks ... */
            if (info.format != py::format_descriptor<Eigen::Vector6d::Scalar>::format())
                throw std::runtime_error("Incompatible format: expected a double array!");

            if (info.ndim != 1)
                throw std::runtime_error("Incompatible buffer dimension!");

            auto map = Eigen::Map<Eigen::Vector6d>(static_cast<Eigen::Vector6d::Scalar*>(info.ptr),
                                                   info.shape[0]);

            return Eigen::Vector6d(map);
        }));

    py::class_<Eigen::Vector3d>(m, "EigenVector3d", py::buffer_protocol())
        .def(py::init([](py::buffer b) {
            typedef Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic> Strides;

            /* Request a buffer descriptor from Python */
            py::buffer_info info = b.request();

            /* Some basic validation checks ... */
            if (info.format != py::format_descriptor<Eigen::Vector3d::Scalar>::format())
                throw std::runtime_error("Incompatible format: expected a double array!");

            if (info.ndim != 1)
                throw std::runtime_error("Incompatible buffer dimension!");

            auto map = Eigen::Map<Eigen::Vector3d>(static_cast<Eigen::Vector3d::Scalar*>(info.ptr),
                                                   info.shape[0]);

            return Eigen::Vector3d(map);
        }))
        .def("__getitem__", [](const Eigen::Vector3d& v, size_t i) {
            if (i >= 3)
                throw py::index_error();
            return v[i];
        });

    constexpr bool rowMajor = Eigen::VectorXd::Flags & Eigen::RowMajorBit;
    py::class_<Eigen::VectorXd>(m, "EigenVectorXd", py::buffer_protocol())
        .def(py::init([](py::buffer b) {
            typedef Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic> Strides;

            /* Request a buffer descriptor from Python */
            py::buffer_info info = b.request();

            /* Some basic validation checks ... */
            if (info.format != py::format_descriptor<Eigen::VectorXd::Scalar>::format())
                throw std::runtime_error("Incompatible format: expected a double array!");

            if (info.ndim != 1)
                throw std::runtime_error("Incompatible buffer dimension!");

            auto map = Eigen::Map<Eigen::VectorXd>(static_cast<Eigen::VectorXd::Scalar*>(info.ptr),
                                                   info.shape[0]);

            return Eigen::VectorXd(map);
        }));

    /************************************************************************/
    /************************** ResourceFinder class ************************/
    /************************************************************************/

    py::class_<yarp::os::ResourceFinder>(m, "ResourceFinder")
        .def(py::init<>())
        .def("findFileByName", [](yarp::os::ResourceFinder& self, const std::string& name) {
            return self.findFileByName(name);
        });

    /************************************************************************/
    /*************** General Functions not part of a  class *****************/
    /************************************************************************/

    m.def("readXMLFile", &readXMLFile, py::arg("pathToFile"), py::arg("parameterHandler"));
    m.def("useSystemClock", &yarp::os::Time::useSystemClock);
}
