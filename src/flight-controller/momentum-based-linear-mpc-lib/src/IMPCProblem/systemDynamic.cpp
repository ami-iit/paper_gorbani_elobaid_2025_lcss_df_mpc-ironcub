#include <IMPCProblem/systemDynamic.h>

DynamicTemplate::DynamicTemplate(const int nStates, const int nInput)
{
    m_nStates = nStates;
    m_nInput = nInput;
    m_ASystem = Eigen::MatrixXd::Zero(m_nStates, m_nStates);
    m_BSystem = Eigen::MatrixXd::Zero(m_nStates, m_nInput);
    m_cSystem = Eigen::VectorXd::Zero(m_nStates);
}

const bool DynamicTemplate::getAMatrixDynamicClass(Eigen::Ref<Eigen::MatrixXd> A)
{
    if (A.cols() != m_ASystem.cols() || A.rows() != m_ASystem.rows())
    {
        yError() << "JetDynamic::getJetAMatrix: The size of the input matrix is not correct.";
        return false;
    }
    A = m_ASystem;
    return true;
}

const bool DynamicTemplate::getBMatrixDynamicClass(Eigen::Ref<Eigen::MatrixXd> B)
{
    if (B.cols() != m_BSystem.cols() || B.rows() != m_BSystem.rows())
    {
        yError() << "JetDynamic::getJetBMatrix: The size of the input matrix is not correct.";
        return false;
    }
    B = m_BSystem;
    return true;
}

const bool DynamicTemplate::getCVectorDynamicClass(Eigen::Ref<Eigen::VectorXd> c)
{
    if (c.size() != m_cSystem.size())
    {
        yError() << "JetDynamic::getJetCVector: The size of the input vector is not correct.";
        return false;
    }
    c = m_cSystem;
    return true;
}

DynamicTemplateVariableSampling::DynamicTemplateVariableSampling(const int nStates,
                                                                 const int nJoints,
                                                                 const int nThrottle)
{
    m_nStates = nStates;
    m_nJoints = nJoints;
    m_nThrottle = nThrottle;
    m_ASystem = Eigen::MatrixXd::Zero(m_nStates, m_nStates);
    m_BSystemJoints = Eigen::MatrixXd::Zero(m_nStates, m_nJoints);
    m_BSystemThrottle = Eigen::MatrixXd::Zero(m_nStates, m_nThrottle);
    m_cSystem = Eigen::VectorXd::Zero(m_nStates);
}

const bool DynamicTemplateVariableSampling::getAMatrixDynamicClass(Eigen::Ref<Eigen::MatrixXd> A)
{
    if (A.cols() != m_ASystem.cols() || A.rows() != m_ASystem.rows())
    {
        yError() << "JetDynamic::getJetAMatrix: The size of the input matrix is not correct.";
        return false;
    }
    A = m_ASystem;
    return true;
}

const bool
DynamicTemplateVariableSampling::getBJointsMatrixDynamicClass(Eigen::Ref<Eigen::MatrixXd> BJoints)
{
    if (BJoints.cols() != m_BSystemJoints.cols() || BJoints.rows() != m_BSystemJoints.rows())
    {
        yError() << "JetDynamic::getJetBJointsMatrix: The size of the input matrix is not correct.";
        return false;
    }
    BJoints = m_BSystemJoints;
    return true;
}

const bool DynamicTemplateVariableSampling::getBThrottleMatrixDynamicClass(
    Eigen::Ref<Eigen::MatrixXd> BThrottle)
{
    if (BThrottle.cols() != m_BSystemThrottle.cols()
        || BThrottle.rows() != m_BSystemThrottle.rows())
    {
        yError() << "JetDynamic::getJetBThrottleMatrix: The size of the input matrix is not "
                    "correct.";
        return false;
    }
    BThrottle = m_BSystemThrottle;
    return true;
}

const bool DynamicTemplateVariableSampling::getCVectorDynamicClass(Eigen::Ref<Eigen::VectorXd> c)
{
    if (c.size() != m_cSystem.size())
    {
        yError() << "JetDynamic::getJetCVector: The size of the input vector is not correct.";
        return false;
    }
    c = m_cSystem;
    return true;
}
