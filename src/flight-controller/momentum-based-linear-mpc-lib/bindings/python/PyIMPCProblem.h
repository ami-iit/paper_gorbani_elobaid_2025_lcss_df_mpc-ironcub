#include <IMPCProblem/IMPCProblem.h>
#include <pybind11/chrono.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <IMPCProblem/IMPCProblem.h>

class PyIMPCProblem : public IMPCProblem
{
    using IMPCProblem::IMPCProblem;

    const bool setCostAndConstraints(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override
    {
        PYBIND11_OVERLOAD_PURE(const bool,
                               IMPCProblem,
                               setCostAndConstraints,
                               parametersHandler,
                               qpInput);
    }
};