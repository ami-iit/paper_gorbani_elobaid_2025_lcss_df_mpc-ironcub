#include <simpleExample/SimpleExampleconstant.h>
#include <simpleExample/costsSimpleExample.h>

namespace SIMPLE_EXAMPLE
{

TrackingCost::TrackingCost(const unsigned int nVar,
                           const int nStates,
                           const int nIter,
                           const int artificialEquilibriumInitPosition)
    : IQPCost(nVar)
{
    m_nStates = nStates;
    m_nIter = nIter;
    m_artificialEquilibriumInitPosition = artificialEquilibriumInitPosition;
    m_artificialInputInit = artificialEquilibriumInitPosition + N_U_1 + N_U_2;
    m_firstUpdate = true;
}

const bool TrackingCost::readConfigParameters(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{
    auto ptr = parametersHandler.lock();
    if (!ptr->getParameter("weightTracking", m_weightTracking))
    {
        yError() << "Parameter 'weightTracking' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("weightArtificialEquilibrium", m_weightArtificialEquilibrium))
    {
        yError() << "Parameter 'weightArtificialEquilibrium' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("weightR", m_weightR))
    {
        yError() << "Parameter 'weightR' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("nIter", m_nIter))
    {
        yError() << "Parameter 'nIter' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("referenceYAlpha", m_referenceYAlpha))
    {
        yError() << "Parameter 'referenceYAlpha' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("referenceYBeta", m_referenceYBeta))
    {
        yError() << "Parameter 'referenceYBeta' not found in the config file.";
        return false;
    }
    return true;
}

void TrackingCost::configureDynVectorsSize(QPInput& qpInput)
{
    m_hessian.setZero();
    m_gradient.setZero();
}

const bool TrackingCost::computeHessianAndGradient(QPInput& qpInput)
{
    if (m_firstUpdate)
    {
        for (int i = 0; i < m_nIter; i++)
        {
            // Y_{alpha} corresponds to x_{alpha}[1] since C_{alpha} = [0 1]
            m_hessian(i * m_nStates, i * m_nStates) += m_weightTracking[0];
            m_hessian(i * m_nStates, m_artificialEquilibriumInitPosition) -= m_weightTracking[0];
            m_hessian(m_artificialEquilibriumInitPosition, i * m_nStates) -= m_weightTracking[0];
            m_hessian(m_artificialEquilibriumInitPosition, m_artificialEquilibriumInitPosition)
                += m_weightTracking[0];

            m_hessian(m_nStates * (m_nIter + 1) + Y2Idx[0] + i * (N_U_1 + N_Y_2 + N_U_2),
                      m_nStates * (m_nIter + 1) + Y2Idx[0] + i * (N_U_1 + N_Y_2 + N_U_2))
                += m_weightTracking[0];
            m_hessian(m_nStates * (m_nIter + 1) + Y2Idx[0] + i * (N_U_1 + N_Y_2 + N_U_2),
                      m_artificialEquilibriumInitPosition + 1)
                -= m_weightTracking[0];
            m_hessian(m_artificialEquilibriumInitPosition + 1,
                      m_nStates * (m_nIter + 1) + Y2Idx[0] + i * (N_U_1 + N_Y_2 + N_U_2))
                -= m_weightTracking[0];
            m_hessian(m_artificialEquilibriumInitPosition + 1,
                      m_artificialEquilibriumInitPosition + 1)
                += m_weightTracking[0];

            m_hessian(m_nStates * (m_nIter + 1) + (N_U_1 + N_Y_2 + N_U_2) * i + U1Idx[0],
                      m_nStates * (m_nIter + 1) + (N_U_1 + N_Y_2 + N_U_2) * i + U1Idx[0])
                += m_weightR[0];
            m_hessian(m_nStates * (m_nIter + 1) + (N_U_1 + N_Y_2 + N_U_2) * i + U1Idx[0],
                      m_artificialEquilibriumInitPosition)
                -= m_weightR[0];
            m_hessian(m_artificialEquilibriumInitPosition,
                      m_nStates * (m_nIter + 1) + (N_U_1 + N_Y_2 + N_U_2) * i + U1Idx[0])
                -= m_weightR[0];
            m_hessian(m_artificialEquilibriumInitPosition, m_artificialEquilibriumInitPosition)
                += m_weightR[0];
            m_hessian(m_nStates * (m_nIter + 1) + (N_U_1 + N_Y_2 + N_U_2) * i + U2Idx[0],
                      m_nStates * (m_nIter + 1) + (N_U_1 + N_Y_2 + N_U_2) * i + U2Idx[0])
                += m_weightR[1];
            m_hessian(m_nStates * (m_nIter + 1) + (N_U_1 + N_Y_2 + N_U_2) * i + U2Idx[0],
                      m_artificialEquilibriumInitPosition + 1)
                -= m_weightR[1];
            m_hessian(m_artificialEquilibriumInitPosition + 1,
                      m_nStates * (m_nIter + 1) + (N_U_1 + N_Y_2 + N_U_2) * i + U2Idx[0])
                -= m_weightR[1];
            m_hessian(m_artificialEquilibriumInitPosition + 1,
                      m_artificialEquilibriumInitPosition + 1)
                += m_weightR[1];
        }
        m_hessian(m_artificialEquilibriumInitPosition, m_artificialEquilibriumInitPosition)
            += m_weightArtificialEquilibrium[0];
        m_hessian(m_artificialEquilibriumInitPosition + 1, m_artificialEquilibriumInitPosition + 1)
            += m_weightArtificialEquilibrium[1];

        m_gradient(m_artificialEquilibriumInitPosition)
            = -m_weightArtificialEquilibrium[0] * m_referenceYAlpha;
        m_gradient(m_artificialEquilibriumInitPosition + 1)
            = -m_weightArtificialEquilibrium[1] * m_referenceYBeta;

        m_firstUpdate = false;
    }
    return true;
}

const bool TrackingCost::configureVectorsCollectionServer(QPInput& qpInput)
{
    return true;
}

const bool TrackingCost::populateVectorsCollection(QPInput& qpInput,
                                                   const Eigen::Ref<Eigen::VectorXd> qpSolution)
{
    return true;
}

double TrackingCost::getConstantCost()
{
    double constantCost
        = 0.5 * m_weightArtificialEquilibrium[0] * m_referenceYAlpha * m_referenceYAlpha
          + 0.5 * m_weightArtificialEquilibrium[1] * m_referenceYBeta * m_referenceYBeta;
    return constantCost;
}

RegularizationGParametersCost::RegularizationGParametersCost(const unsigned int nVar,
                                                             const int gInitPosition,
                                                             const int nG)
    : IQPCost(nVar)
{
    m_gInitPosition = gInitPosition;
    m_nG = nG;
    m_firstUpdate = true;
}

const bool RegularizationGParametersCost::readConfigParameters(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{
    auto ptr = parametersHandler.lock();
    if (!ptr->getParameter("weightGParameters", m_weightGParameters))
    {
        yError() << "Parameter 'weightGParameters' not found in the config file.";
        return false;
    }
    return true;
}

void RegularizationGParametersCost::configureDynVectorsSize(QPInput& qpInput)
{
    m_hessian.setZero();
    m_gradient.setZero();
}

const bool RegularizationGParametersCost::computeHessianAndGradient(QPInput& qpInput)
{
    if (m_firstUpdate)
    {
        m_hessian.block(m_gInitPosition, m_gInitPosition, m_nG, m_nG)
            = m_weightGParameters * Eigen::MatrixXd::Identity(m_nG, m_nG);
        m_firstUpdate = false;
    }
    return true;
}

const bool RegularizationGParametersCost::configureVectorsCollectionServer(QPInput& qpInput)
{
    return true;
}

const bool RegularizationGParametersCost::populateVectorsCollection(
    QPInput& qpInput, const Eigen::Ref<Eigen::VectorXd> qpSolution)
{
    return true;
}

RegularizationSlackVariableCost::RegularizationSlackVariableCost(const unsigned int nVar,
                                                                 const int slackVarInitPosition,
                                                                 const int nSlackVar)
    : IQPCost(nVar)
{
    m_slackVarInitPosition = slackVarInitPosition;
    m_nSlackVar = nSlackVar;
    m_firstUpdate = true;
}

const bool RegularizationSlackVariableCost::readConfigParameters(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{
    auto ptr = parametersHandler.lock();
    if (!ptr->getParameter("weightSlackVariable", m_weightSlackVariable))
    {
        yError() << "Parameter 'weightSlackVariable' not found in the config file.";
        return false;
    }
    return true;
}

void RegularizationSlackVariableCost::configureDynVectorsSize(QPInput& qpInput)
{
    m_hessian.setZero();
    m_gradient.setZero();
}

const bool RegularizationSlackVariableCost::computeHessianAndGradient(QPInput& qpInput)
{
    if (m_firstUpdate)
    {
        m_hessian.block(m_slackVarInitPosition, m_slackVarInitPosition, m_nSlackVar, m_nSlackVar)
            = m_weightSlackVariable * Eigen::MatrixXd::Identity(m_nSlackVar, m_nSlackVar);
        m_firstUpdate = false;
    }
    return true;
}

const bool RegularizationSlackVariableCost::configureVectorsCollectionServer(QPInput& qpInput)
{
    return true;
}

const bool RegularizationSlackVariableCost::populateVectorsCollection(
    QPInput& qpInput, const Eigen::Ref<Eigen::VectorXd> qpSolution)
{
    return true;
}

} // namespace SIMPLE_EXAMPLE
