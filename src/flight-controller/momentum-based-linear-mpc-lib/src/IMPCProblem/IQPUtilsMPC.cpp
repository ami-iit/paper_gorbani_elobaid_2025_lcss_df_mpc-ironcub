#include <IMPCProblem/IQPUtilsMPC.h>

IQPConstraintMPCDynamic::IQPConstraintMPCDynamic(const unsigned int nVar,
                                                 const unsigned int nStates,
                                                 const unsigned int nInput,
                                                 const unsigned int nIter)
    : IQPConstraint(nVar, (nStates)*nIter)
{
    m_nStates = nStates;
    m_nInput = nInput;
    m_nIter = nIter;
    this->configureSizeDynamicsMatrices();
}

const bool IQPConstraintMPCDynamic::configureSizeDynamicsMatrices()
{
    m_A = Eigen::MatrixXd::Zero(m_nStates, m_nStates);
    m_B = Eigen::MatrixXd::Zero(m_nStates, m_nInput);
    m_c = Eigen::VectorXd::Zero(m_nStates);
    return true;
}

const bool IQPConstraintMPCDynamic::computeConstraintsMatrixAndBounds(QPInput& qpInput)
{
    if (!updateDynamicConstraints(qpInput))
    {
        yError() << "IQPConstraintMPCDynamic::computeConstraintsMatrixAndBounds: error in "
                    "updateDynamicConstraints";
        return false;
    }
    if (m_A.rows() != m_nStates || m_A.cols() != m_nStates || m_B.rows() != m_nStates
        || m_B.cols() != m_nInput || m_c.size() != m_nStates)
    {
        yError() << "IQPConstraintMPCDynamic::computeConstraintsMatrixAndBounds: "
                    "m_A.rows() != m_nStates || m_A.cols() != m_nStates || m_B.rows() != m_nStates "
                    "|| m_B.cols() != m_nInput || m_c.size() != m_nStates";
        return false;
    }
    m_linearMatrix.setZero();
    m_lowerBound.setZero();
    m_upperBound.setZero();
    for (int i = 0; i < m_nIter; i++)
    {
        m_linearMatrix.block(i * m_nStates, i * m_nStates, m_nStates, m_nStates) = m_A;
        m_linearMatrix.block(i * m_nStates, (i + 1) * m_nStates, m_nStates, m_nStates)
            = -Eigen::MatrixXd::Identity(m_nStates, m_nStates);
        m_linearMatrix.block(i * m_nStates,
                             m_nStates * (m_nIter + 1) + i * m_nInput,
                             m_nStates,
                             m_nInput)
            = m_B;
        m_lowerBound.segment(i * m_nStates, m_nStates) = -m_c;
        m_upperBound.segment(i * m_nStates, m_nStates) = -m_c;
    }
    return true;
}

IQPConstraintInitialState::IQPConstraintInitialState(const unsigned int nStates,
                                                     const unsigned int nVar)
    : IQPConstraint(nVar, nStates)
{
    m_nStates = nStates;
    this->configureSizeInitialState();
}

const bool IQPConstraintInitialState::configureSizeInitialState()
{
    m_initialState.resize(m_nStates);
    return true;
}

const bool IQPConstraintInitialState::computeConstraintsMatrixAndBounds(QPInput& qpInput)
{
    m_linearMatrix.setZero();
    m_lowerBound.setZero();
    m_upperBound.setZero();
    if (!updateInitialState(qpInput))
    {
        yError() << "IQPConstraintInitialState::computeConstraintsMatrixAndBounds: error in "
                    "updateInitialState";
        return false;
    }
    if (m_initialState.size() != m_nStates)
    {
        yError() << "IQPConstraintInitialState::computeConstraintsMatrixAndBounds: "
                    "m_initialState.size() != m_nStates";
        return false;
    }
    m_linearMatrix.block(0, 0, m_nStates, m_nStates).setIdentity();
    m_lowerBound.segment(0, m_nStates) = m_initialState;
    m_upperBound.segment(0, m_nStates) = m_initialState;
    return true;
}
