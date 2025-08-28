#include "PyIMPCProblem.h"
#include <dataDrivenVSMPC/dataDrivenVariableSamplingMPC.h>
#include <iDynTree/Transform.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Searchable.h>

namespace py = pybind11;

PYBIND11_MODULE(bindingsMPC, m)
{

    py::class_<IMPCProblem, PyIMPCProblem>(m, "IMPCProblem")
        .def(py::init<>())
        .def("setCostAndConstraints",
             &IMPCProblem::setCostAndConstraints,
             py::arg("parametersHandler"),
             py::arg("mpcInput"));

    py::class_<DataDrivenVariableSamplingMPC, IMPCProblem>(m, "DataDrivenVariableSamplingMPC")
        .def(py::init<>())
        .def("configure",
             [](DataDrivenVariableSamplingMPC& self,
                std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler>
                    parametersHandler,
                QPInput& mpcInput) {
                 std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler>
                     parametersHandlerWeak = parametersHandler;
                 return self.configure(parametersHandlerWeak, mpcInput);
             })
        .def("setCostAndConstraints",
             &IMPCProblem::setCostAndConstraints,
             py::arg("parametersHandler"),
             py::arg("mpcInput"))
        .def("update", &IMPCProblem::update, py::arg("mpcInput"))
        .def("solveMPC", &DataDrivenVariableSamplingMPC::solveMPC)
        .def("getMPCSolution", &DataDrivenVariableSamplingMPC::getMPCSolution)
        .def("getJointsReferencePosition",
             [](DataDrivenVariableSamplingMPC& self) {
                 Eigen::VectorXd jointsReferencePosition;
                 jointsReferencePosition.resize(23);
                 self.getJointsReferencePosition(jointsReferencePosition);
                 return jointsReferencePosition;
             })
        .def("getThrottleReference",
             [](DataDrivenVariableSamplingMPC& self) {
                 Eigen::VectorXd throttleReference(4);
                 self.getThrottleReference(throttleReference);
                 return throttleReference;
             })
        .def("getThrustReference",
             [](DataDrivenVariableSamplingMPC& self) {
                 Eigen::VectorXd jointsReferenceVelocity(4);
                 self.getThrustReference(jointsReferenceVelocity);
                 return jointsReferenceVelocity;
             })
        .def("getFinalCoMPosition",
             [](DataDrivenVariableSamplingMPC& self) {
                 Eigen::Vector3d finalCoMPosition;
                 self.getFinalCoMPosition(finalCoMPosition);
                 return finalCoMPosition;
             })
        .def("getFinalLinMom",
             [](DataDrivenVariableSamplingMPC& self) {
                 Eigen::VectorXd finalLinMom;
                 self.getFinalLinMom(finalLinMom);
                 return finalLinMom;
             })
        .def("getFinalRPY",
             [](DataDrivenVariableSamplingMPC& self) {
                 Eigen::Vector3d finalRPY;
                 self.getFinalRPY(finalRPY);
                 return finalRPY;
             })
        .def("getFinalAngMom",
             [](DataDrivenVariableSamplingMPC& self) {
                 Eigen::VectorXd finalAngMom;
                 self.getFinalAngMom(finalAngMom);
                 return finalAngMom;
             })
        .def("getArtificialEquilibrium",
             [](DataDrivenVariableSamplingMPC& self) {
                 Eigen::VectorXd artificialEquilibrium;
                 artificialEquilibrium.resize(12);
                 self.getArtificialEquilibrium(artificialEquilibrium);
                 return artificialEquilibrium;
             })
        .def("getNStatesMPC", &DataDrivenVariableSamplingMPC::getNStatesMPC)
        .def("getNInputMPC", &DataDrivenVariableSamplingMPC::getNInputMPC)
        .def("setHankleMatrices",
             [](DataDrivenVariableSamplingMPC& self,
                const std::vector<std::vector<double>>& inputData,
                const std::vector<std::vector<double>>& outputData) {
                 return self.setHankleMatrices(inputData, outputData);
             })
        .def("getValueFunction", &DataDrivenVariableSamplingMPC::getValueFunction);
}
