#include "PyIMPCProblem.h"
#include <dataFusedMPC/dataFusedMPC.h>
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

    py::class_<DataFusedMPC, IMPCProblem>(m, "DataFusedMPC")
        .def(py::init<>())
        .def("configure",
             [](DataFusedMPC& self,
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
        .def("solveMPC", &DataFusedMPC::solveMPC)
        .def("getMPCSolution", &DataFusedMPC::getMPCSolution)
        .def("getJointsReferencePosition",
             [](DataFusedMPC& self) {
                 Eigen::VectorXd jointsReferencePosition;
                 jointsReferencePosition.resize(23);
                 self.getJointsReferencePosition(jointsReferencePosition);
                 return jointsReferencePosition;
             })
        .def("getThrottleReference",
             [](DataFusedMPC& self) {
                 Eigen::VectorXd throttleReference(4);
                 self.getThrottleReference(throttleReference);
                 return throttleReference;
             })
        .def("getThrustReference",
             [](DataFusedMPC& self) {
                 Eigen::VectorXd jointsReferenceVelocity(4);
                 self.getThrustReference(jointsReferenceVelocity);
                 return jointsReferenceVelocity;
             })
        .def("getFinalCoMPosition",
             [](DataFusedMPC& self) {
                 Eigen::Vector3d finalCoMPosition;
                 self.getFinalCoMPosition(finalCoMPosition);
                 return finalCoMPosition;
             })
        .def("getFinalLinMom",
             [](DataFusedMPC& self) {
                 Eigen::VectorXd finalLinMom;
                 self.getFinalLinMom(finalLinMom);
                 return finalLinMom;
             })
        .def("getFinalRPY",
             [](DataFusedMPC& self) {
                 Eigen::Vector3d finalRPY;
                 self.getFinalRPY(finalRPY);
                 return finalRPY;
             })
        .def("getFinalAngMom",
             [](DataFusedMPC& self) {
                 Eigen::VectorXd finalAngMom;
                 self.getFinalAngMom(finalAngMom);
                 return finalAngMom;
             })
        .def("getArtificialEquilibrium",
             [](DataFusedMPC& self) {
                 Eigen::VectorXd artificialEquilibrium;
                 artificialEquilibrium.resize(12);
                 self.getArtificialEquilibrium(artificialEquilibrium);
                 return artificialEquilibrium;
             })
        .def("getNStatesMPC", &DataFusedMPC::getNStatesMPC)
        .def("getNInputMPC", &DataFusedMPC::getNInputMPC)
        .def("setHankleMatrices",
             [](DataFusedMPC& self,
                const std::vector<std::vector<double>>& inputData,
                const std::vector<std::vector<double>>& outputData) {
                 return self.setHankleMatrices(inputData, outputData);
             })
        .def("getValueFunction", &DataFusedMPC::getValueFunction);
}
