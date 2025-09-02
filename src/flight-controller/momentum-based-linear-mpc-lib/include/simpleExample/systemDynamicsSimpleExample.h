
#ifndef SYSTEM_DYNAMIC_SIMPLE_EXAMPLE_H
#define SYSTEM_DYNAMIC_SIMPLE_EXAMPLE_H

#include "QPInput.h"
#include "TrajectoryManager.h"
#include <Eigen/Dense>
#include <IMPCProblem/systemDynamic.h>
#include <boost/core/demangle.hpp>

class subSys1SimpleExample : public DynamicTemplate
{
public:
    subSys1SimpleExample(const int nStates, const int nInput);
    ~subSys1SimpleExample() = default;

    const bool configure(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;

    const bool updateInitialStates(QPInput& qpInput) override;

private:
    double m_periodMPC;
    std::shared_ptr<BipedalLocomotion::YarpUtilities::VectorsCollectionServer>
        m_vectorsCollectionServer;
};

class SystemDynamicsSimpleExample
{
public:
    SystemDynamicsSimpleExample(const int nStates, const int nInput);
    ~SystemDynamicsSimpleExample() = default;

    const bool configure(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput);

    const bool updateDynamicMatrices(QPInput& qpInput);
    const bool getAMatrix(Eigen::Ref<Eigen::MatrixXd> A);
    const bool getBMatrix(Eigen::Ref<Eigen::MatrixXd> B);
    const bool getCVector(Eigen::Ref<Eigen::VectorXd> c);

private:
    int m_nStates;
    int m_nInput;
    Eigen::MatrixXd m_A;
    Eigen::MatrixXd m_B;
    Eigen::VectorXd m_c;
    std::vector<std::unique_ptr<DynamicTemplate>> m_vectorDynamic;
};

#endif // SYSTEM_DYNAMIC_SIMPLE_EXAMPLE_H
