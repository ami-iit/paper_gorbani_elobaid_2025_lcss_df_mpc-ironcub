#include "FlightControlUtils.h"
#include <simpleExample/SimpleExampleconstant.h>
#include <simpleExample/systemDynamicsSimpleExample.h>
#define EIGEN_INITIALIZE_MATRICES_BY_NAN

subSys1SimpleExample::subSys1SimpleExample(const int nStates, const int nInput)
    : DynamicTemplate(nStates, nInput)
{
}

const bool subSys1SimpleExample::configure(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{
    auto ptr = parametersHandler.lock();
    if (!ptr->getParameter("periodMPC", m_periodMPC))
    {
        yError() << "Parameter 'periodMPC' not found in the config file.";
        return false;
    }
    return true;
}

const bool subSys1SimpleExample::updateInitialStates(QPInput& qpInput)
{
    m_ASystem.setZero();
    m_BSystem.setZero();
    m_cSystem.setZero();
    Eigen::MatrixXd A;
    A.resize(2, 2);
    A << 0, 1, -2, -3;
    m_ASystem.setIdentity();
    m_ASystem += 0.01 * A;
    // std::cout << "A system: " << m_ASystem << std::endl;
    // B_system = [E B_{alpha}]
    m_BSystem << 1, 0, 0, 0, 1, 0;
    m_BSystem = m_BSystem * 0.01;

    return true;
}

SystemDynamicsSimpleExample::SystemDynamicsSimpleExample(const int nStates, const int nInput)
{
    m_nStates = nStates;
    m_nInput = nInput;
    m_A = Eigen::MatrixXd::Zero(m_nStates, m_nStates);
    m_B = Eigen::MatrixXd::Zero(m_nStates, m_nInput);
    m_c = Eigen::VectorXd::Zero(m_nStates);
}

const bool SystemDynamicsSimpleExample::configure(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{
    m_vectorDynamic.emplace_back(std::make_unique<subSys1SimpleExample>(m_nStates, m_nInput));

    for (auto& dynamic : m_vectorDynamic)
    {
        if (!dynamic->configure(parametersHandler, qpInput))
        {
            yError() << "Could not configure the dynamic.";
            return false;
        }
    }
    return true;
}

const bool SystemDynamicsSimpleExample::updateDynamicMatrices(QPInput& qpInput)
{
    for (auto& dynamic : m_vectorDynamic)
    {
        if (!dynamic->updateInitialStates(qpInput))
        {
            yError() << "Could not update the dynamic matrices of the class: "
                     << boost::core::demangle(typeid(*dynamic).name());
            return false;
        }
    }
    return true;
}

const bool SystemDynamicsSimpleExample::getAMatrix(Eigen::Ref<Eigen::MatrixXd> A)
{
    if (A.cols() != m_A.cols() || A.rows() != m_A.rows())
    {
        yError() << "SystemDynamicsSimpleExample::getAMatrix: The size of the input matrix is not "
                    "correct.";
        return false;
    }
    m_A.setZero();
    Eigen::MatrixXd A_dyn;
    A_dyn.resize(m_nStates, m_nStates);
    for (auto& dynamic : m_vectorDynamic)
    {
        dynamic->getAMatrixDynamicClass(A_dyn);
        m_A += A_dyn;
    }
    A = m_A;
    return true;
}

const bool SystemDynamicsSimpleExample::getBMatrix(Eigen::Ref<Eigen::MatrixXd> B)
{
    if (B.cols() != m_B.cols() || B.rows() != m_B.rows())
    {
        yError() << "SystemDynamicsSimpleExample::getBMatrix: The size of the input matrix is not "
                    "correct.";
        return false;
    }
    m_B.setZero();
    for (auto& dynamic : m_vectorDynamic)
    {
        Eigen::MatrixXd B_dyn;
        B_dyn.resize(m_nStates, m_nInput);
        dynamic->getBMatrixDynamicClass(B_dyn);
        m_B += B_dyn;
    }
    B = m_B;
    return true;
}

const bool SystemDynamicsSimpleExample::getCVector(Eigen::Ref<Eigen::VectorXd> c)
{
    if (c.size() != m_c.size())
    {
        yError() << "SystemDynamicsSimpleExample::getCVector: The size of the input vector is not "
                    "correct.";
        return false;
    }
    m_c.setZero();
    for (auto& dynamic : m_vectorDynamic)
    {
        Eigen::VectorXd c_dyn;
        c_dyn.resize(m_nStates);
        dynamic->getCVectorDynamicClass(c_dyn);
        m_c += c_dyn;
    }
    c = m_c;
    return true;
}
