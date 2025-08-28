
#ifndef SYSTEMDYNAMIC_H
#define SYSTEMDYNAMIC_H

#include "QPInput.h"
#include <Eigen/Dense>
#include <boost/core/demangle.hpp>

enum JointsLambda
{
    UNFILTERED = 0,
    CONSTANT = 1
};

class DynamicTemplate
{
public:
    DynamicTemplate(const int nStates, const int nInput);
    virtual ~DynamicTemplate() = default;

    virtual const bool configure(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput)
        = 0;

    virtual const bool updateInitialStates(QPInput& qpInput) = 0;
    const bool getAMatrixDynamicClass(Eigen::Ref<Eigen::MatrixXd> A);
    const bool getBMatrixDynamicClass(Eigen::Ref<Eigen::MatrixXd> B);
    const bool getCVectorDynamicClass(Eigen::Ref<Eigen::VectorXd> c);

protected:
    int m_nStates;
    int m_nInput;
    Eigen::MatrixXd m_ASystem;
    Eigen::MatrixXd m_BSystem;
    Eigen::VectorXd m_cSystem;
};

class DynamicTemplateVariableSampling
{
public:
    DynamicTemplateVariableSampling(const int nStates, const int nJoints, const int nThrottle);
    virtual ~DynamicTemplateVariableSampling() = default;

    virtual const bool configure(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput)
        = 0;

    virtual const bool updateInitialStates(QPInput& qpInput) = 0;
    const bool getAMatrixDynamicClass(Eigen::Ref<Eigen::MatrixXd> A);
    const bool getBJointsMatrixDynamicClass(Eigen::Ref<Eigen::MatrixXd> BJoints);
    const bool getBThrottleMatrixDynamicClass(Eigen::Ref<Eigen::MatrixXd> BThrottle);
    const bool getCVectorDynamicClass(Eigen::Ref<Eigen::VectorXd> c);

protected:
    int m_nStates;
    int m_nJoints;
    int m_nThrottle;
    Eigen::MatrixXd m_ASystem;
    Eigen::MatrixXd m_BSystemJoints;
    Eigen::MatrixXd m_BSystemThrottle;
    Eigen::VectorXd m_cSystem;
};

#endif // SYSTEMDYNAMIC_H
