#include "FlightControlUtils.h"
#include <dataFusedMPC/DFconstant.h>
#include <dataFusedMPC/costsDFMPC.h>
#include <fstream>

namespace DFMPC
{

ReferenceTrackingCost::ReferenceTrackingCost(const unsigned int nVar,
                                             const unsigned int nStates,
                                             const unsigned int nIter)
    : IQPCost(nVar)
{
    m_nStates = nStates;
    m_nIter = nIter;
    m_weightCoMPos.resize(3);
    m_weightLinMom.resize(3);
    m_weightRPY.resize(3);
    m_weightAngMom.resize(3);
    m_initialCoMPos.resize(3);
    m_initialRPY.resize(3);
    m_inertia.setZero();
    m_W.setZero();
}

const bool ReferenceTrackingCost::readConfigParameters(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{
    auto ptr = parametersHandler.lock();
    bool ok = getParameterAndCheckSize(parametersHandler, "weightCoMPos", m_weightCoMPos);
    ok = ok && getParameterAndCheckSize(parametersHandler, "weightLinMom", m_weightLinMom);
    ok = ok && getParameterAndCheckSize(parametersHandler, "weightRPY", m_weightRPY);
    ok = ok && getParameterAndCheckSize(parametersHandler, "weightAngMom", m_weightAngMom);
    double periodMPC;
    if (!ptr->getParameter("periodMPC", periodMPC))
    {
        yError() << "Parameter 'periodMPC' not found in the config file.";
        return false;
    }
    auto paramHandlerYarpImpl
        = std::make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>();
    auto group = ptr->getGroup("POSITION_TRAJECTORY");
    if (group.lock() == nullptr)
    {
        yError() << "Group [TRAJECTORY_MANAGER] not found in the config file.";
        return false;
    }
    paramHandlerYarpImpl->setGroup("TRAJECTORY_MANAGER", group.lock());
    double periodMPCLargeSteps;
    if (!ptr->getParameter("periodMPCLargeSteps", periodMPCLargeSteps))
    {
        yError() << "Parameter 'periodMPCLargeSteps' not found in the config file.";
        return false;
    }
    double periodMPCSmallSteps;
    if (!ptr->getParameter("periodMPCSmallSteps", periodMPCSmallSteps))
    {
        yError() << "Parameter 'periodMPCSmallSteps' not found in the config file.";
        return false;
    }
    std::cout << "=============== large step period: " << periodMPCLargeSteps << std::endl;
    m_trajManager.configure(paramHandlerYarpImpl, 10);
    m_ratioSmallLargeStepsPeriod = round(periodMPCLargeSteps / periodMPCSmallSteps);

    return ok;
}

void ReferenceTrackingCost::configureDynVectorsSize(QPInput& qpInput)
{
    m_robot = qpInput.getRobot();
    m_vectorsCollectionServer = qpInput.getVectorsCollectionServer();
    m_Q.resize(m_nStates, m_nStates);
    m_Q.setZero();
    m_Q.block(CoMPosIdx[0], CoMPosIdx[0], CoMPosIdx.size(), CoMPosIdx.size())
        = m_weightCoMPos.asDiagonal();
    m_Q.block(linMomIdx[0], linMomIdx[0], linMomIdx.size(), linMomIdx.size())
        = m_weightLinMom.asDiagonal();
    m_Q.block(rpyIdx[0], rpyIdx[0], rpyIdx.size(), rpyIdx.size()) = m_weightRPY.asDiagonal();
    m_Q.block(angMomIdx[0], angMomIdx[0], angMomIdx.size(), angMomIdx.size())
        = m_weightAngMom.asDiagonal();
    m_stateReference.resize(m_nStates, m_nIter);
    m_stateReference.setZero();
    m_positionCoMReference.resize(3, m_nIter);
    m_linearMomentumReference.resize(3, m_nIter);
    m_RPYReference.resize(3, m_nIter);
    m_angularMomentumReference.resize(3, m_nIter);
    m_initialCoMPos = m_robot->getPositionCoM();
    m_initialRPY = iDynTree::toEigen(m_robot->getBasePose().getRotation().asRPY());
    this->updateInertiaMatrix();
    for (int i = 0; i < m_nIter; i++)
    {
        m_positionCoMReference.col(i)
            = m_initialCoMPos + m_trajManager.getCurrentValue("positionCoM");
        m_linearMomentumReference.col(i)
            = iDynTree::toEigen(m_robot->getBasePose().getRotation()).transpose()
              * m_robot->getTotalMass() * m_trajManager.getCurrentValue("velocityCoM");
        m_RPYReference.col(i) = m_initialRPY + m_trajManager.getCurrentValue("RPY");
        m_angularMomentumReference.col(i)
            = m_inertia * m_W * m_trajManager.getCurrentValue("RPYDot");
    }
    m_reduced_matrix.resize(m_positionCoMReference.rows(), m_positionCoMReference.cols() - 1);
    this->setPositionCoMReference(m_positionCoMReference);
    this->setLinearMomentumReference(m_linearMomentumReference);
    this->setRPYReference(m_RPYReference);
    this->setAngularMomentumReference(m_angularMomentumReference);
    m_counter = (m_ratioSmallLargeStepsPeriod - 1);
}

const bool ReferenceTrackingCost::computeHessianAndGradient(QPInput& qpInput)
{
    m_gradient.setZero();
    if (m_counter == (m_ratioSmallLargeStepsPeriod - 1))
    {
        m_trajManager.advanceTrajectory();
        Eigen::MatrixXd mat_reduced
            = m_positionCoMReference.block(0,
                                           1,
                                           m_positionCoMReference.rows(),
                                           m_positionCoMReference.cols() - 1);
        m_positionCoMReference << mat_reduced,
            m_initialCoMPos + m_trajManager.getCurrentValue("positionCoM");
        mat_reduced = m_linearMomentumReference.block(0,
                                                      1,
                                                      m_linearMomentumReference.rows(),
                                                      m_linearMomentumReference.cols() - 1);
        m_linearMomentumReference << mat_reduced,
            iDynTree::toEigen(m_robot->getBasePose().getRotation()).transpose()
                * m_robot->getTotalMass() * m_trajManager.getCurrentValue("velocityCoM");
        mat_reduced = m_RPYReference.block(0, 1, m_RPYReference.rows(), m_RPYReference.cols() - 1);
        m_RPYReference << mat_reduced, m_initialRPY + m_trajManager.getCurrentValue("RPY");
        this->updateInertiaMatrix();
        mat_reduced = m_angularMomentumReference.block(0,
                                                       1,
                                                       m_angularMomentumReference.rows(),
                                                       m_angularMomentumReference.cols() - 1);
        m_angularMomentumReference << mat_reduced,
            m_inertia * m_W * m_trajManager.getCurrentValue("RPYDot");

        this->setPositionCoMReference(m_positionCoMReference);
        this->setLinearMomentumReference(m_linearMomentumReference);
        this->setRPYReference(m_RPYReference);
        this->setAngularMomentumReference(m_angularMomentumReference);
        qpInput.setPosCoMReference(m_positionCoMReference.col(0));
        qpInput.setRPYReference(m_RPYReference.col(0));
        Eigen::Vector6d momentumReference;
        momentumReference.head(3) = m_linearMomentumReference.col(0);
        momentumReference.tail(3) = m_angularMomentumReference.col(0);
        qpInput.setMomentumReference(momentumReference);
        m_counter = 0;
    } else
    {
        m_counter++;
    }
    if (m_firstUpdate)
    {
        m_hessian.setZero();
        for (int i = 1; i < m_nIter + 1; i++)
        {
            m_hessian.block(i * m_nStates, i * m_nStates, m_nStates, m_nStates) = m_Q;
        }
        m_firstUpdate = false;
    }
    for (int i = 1; i < m_nIter + 1; i++)
    {
        m_gradient.segment(i * m_nStates, m_nStates) = -m_Q * m_stateReference.col(i - 1);
    }

    return true;
}

const bool
ReferenceTrackingCost::setPositionCoMReference(const Eigen::Ref<const Eigen::MatrixXd> CoMPosRef)
{
    if (CoMPosRef.rows() != 3 || CoMPosRef.cols() != m_nIter)
    {
        yError() << "The size of the input vector is not correct.";
        return false;
    }
    for (int i = 0; i < m_nIter; i++)
    {
        m_stateReference.block(CoMPosIdx[0], i, 3, 1) = CoMPosRef.col(i);
    }
    return true;
}

const bool
ReferenceTrackingCost::setLinearMomentumReference(const Eigen::Ref<const Eigen::MatrixXd> linMomRef)
{
    if (linMomRef.rows() != 3 || linMomRef.cols() != m_nIter)
    {
        yError() << "The size of the input vector is not correct.";
        return false;
    }
    for (int i = 0; i < m_nIter; i++)
    {
        m_stateReference.block(linMomIdx[0], i, 3, 1) = linMomRef.col(i);
    }
    return true;
}

const bool ReferenceTrackingCost::setRPYReference(const Eigen::Ref<const Eigen::MatrixXd> RPYRef)
{
    if (RPYRef.rows() != 3 || RPYRef.cols() != m_nIter)
    {
        yError() << "The size of the input vector is not correct.";
        return false;
    }
    for (int i = 0; i < m_nIter; i++)
    {
        m_stateReference.block(rpyIdx[0], i, 3, 1) = RPYRef.col(i);
    }
    return true;
}

const bool
ReferenceTrackingCost::setAngularMomentumReference(const Eigen::Ref<const Eigen::MatrixXd> angMomRef)
{
    if (angMomRef.rows() != 3 || angMomRef.cols() != m_nIter)
    {
        yError() << "The size of the input vector is not correct.";
        return false;
    }
    for (int i = 0; i < m_nIter; i++)
    {
        m_stateReference.block(angMomIdx[0], i, 3, 1) = angMomRef.col(i);
    }
    return true;
}

void ReferenceTrackingCost::updateInertiaMatrix()
{
    iDynTree::Position r;
    iDynTree::toEigen(r)
        = m_robot->getPositionCoM() - iDynTree::toEigen(m_robot->getBasePose().getPosition());
    iDynTree::Transform G_H_B;
    G_H_B.setPosition(r);
    G_H_B.setRotation(m_robot->getBasePose().getRotation());
    Eigen::MatrixXd M_b = m_robot->getMassMatrix().block(0, 0, 6, 6);
    auto m_RPY = iDynTree::toEigen(m_robot->getBasePose().getRotation().asRPY());
    m_W.setZero();
    m_W(0, 0) = 1.0;
    m_W(1, 1) = cos(m_RPY(0));
    m_W(2, 1) = -sin(m_RPY(0));
    m_W(0, 2) = -sin(m_RPY(1));
    m_W(1, 2) = cos(m_RPY(1)) * sin(m_RPY(0));
    m_W(2, 2) = cos(m_RPY(0)) * cos(m_RPY(1));
    m_inertia = (iDynTree::toEigen(G_H_B.asAdjointTransform()).transpose() * M_b
                 * iDynTree::toEigen(G_H_B.asAdjointTransform()))
                    .block(3, 3, 3, 3);
}

const bool ReferenceTrackingCost::configureVectorsCollectionServer(QPInput& qpInput)
{
    m_vectorsCollectionServer->populateMetadata("reference::CoMPos", {"x", "y", "z"});
    m_vectorsCollectionServer->populateMetadata("reference::linMom", {"x", "y", "z"});
    m_vectorsCollectionServer->populateMetadata("reference::RPY", {"roll", "pitch", "yaw"});
    m_vectorsCollectionServer->populateMetadata("reference::angMom", {"x", "y", "z"});
    return true;
}

const bool
ReferenceTrackingCost::populateVectorsCollection(QPInput& qpInput,
                                                 const Eigen::Ref<Eigen::VectorXd> qpSolution)
{
    m_vectorsCollectionServer->populateData("reference::CoMPos", m_positionCoMReference.col(0));
    m_vectorsCollectionServer->populateData("reference::linMom", m_linearMomentumReference.col(0));
    m_vectorsCollectionServer->populateData("reference::RPY", m_RPYReference.col(0));
    m_vectorsCollectionServer->populateData("reference::angMom", m_angularMomentumReference.col(0));

    return true;
}

RegualarizationCost::RegualarizationCost(const unsigned int nVar,
                                         const unsigned int nState,
                                         const unsigned int nJoints,
                                         const unsigned int nThrottle)
    : IQPCost(nVar)
{
    m_nStates = nState;
    m_nCtrlJoints = nJoints;
    m_nJets = nThrottle;
    m_firstUpdate = true;
}

const bool RegualarizationCost::readConfigParameters(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{
    auto ptr = parametersHandler.lock();
    if (!ptr->getParameter("nIter", m_nIter))
    {
        yError() << "Parameter 'nIter' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("nIterSmall", m_nSmallSteps))
    {
        yError() << "Parameter 'nIterSmall' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("weightThrottle", m_weightThrottle))
    {
        yError() << "Parameter 'weightThrottle' not found in the config file.";
        return false;
    }
    double weightThrust;
    if (!ptr->getParameter("weightThrust", weightThrust))
    {
        yError() << "Parameter 'weightThrust' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("controlHorizon", m_ctrlHorizon))
    {
        yError() << "Parameter 'controlHorizon' not found in the config file.";
        return false;
    }
    Eigen::VectorXd weightDeltaJoint;
    if (!ptr->getParameter("weightDeltaJoint", weightDeltaJoint))
    {
        yError() << "Parameter 'weightDeltaJoint' not found in the config file.";
        return false;
    }
    if (weightDeltaJoint.size() != deltaJointIdx.size())
    {
        yError() << "The size of the vector containing the weights for the joint deltas is not "
                    "correct.";
        return false;
    }
    m_weightDeltaJoint.resize(deltaJointIdx.size(), deltaJointIdx.size());
    m_weightDeltaJoint = weightDeltaJoint.asDiagonal();
    m_weightThrottleMatrix.resize(m_nJets, m_nJets);
    m_weightThrottleMatrix = m_weightThrottle * Eigen::MatrixXd::Identity(m_nJets, m_nJets);
    m_weightThrustMatrix = weightThrust * Eigen::MatrixXd::Identity(m_nJets, m_nJets);
    return true;
}

void RegualarizationCost::configureDynVectorsSize(QPInput& qpInput)
{
}

const bool RegualarizationCost::computeHessianAndGradient(QPInput& qpInput)
{
    if (m_firstUpdate)
    {
        m_hessian.setZero();
        m_gradient.setZero();
        for (int i = 0; i < m_ctrlHorizon; i++)
        {
            m_hessian.block(m_nStates * (m_nIter + 1) + i * m_nCtrlJoints,
                            m_nStates * (m_nIter + 1) + i * m_nCtrlJoints,
                            m_nCtrlJoints,
                            m_nCtrlJoints)
                = m_weightDeltaJoint;
        }
        for (int i = 0; i < m_ctrlHorizon - m_nSmallSteps; i++)
        {
            m_hessian.block(m_nStates * (m_nIter + 1) + m_ctrlHorizon * m_nCtrlJoints + i * m_nJets,
                            m_nStates * (m_nIter + 1) + m_ctrlHorizon * m_nCtrlJoints + i * m_nJets,
                            m_nJets,
                            m_nJets)
                += m_weightThrustMatrix;
            m_hessian.block(m_nStates * (m_nIter + 1) + m_ctrlHorizon * m_nCtrlJoints
                                + (i + 1) * m_nJets,
                            m_nStates * (m_nIter + 1) + m_ctrlHorizon * m_nCtrlJoints + i * m_nJets,
                            m_nJets,
                            m_nJets)
                -= m_weightThrustMatrix;
            m_hessian.block(m_nStates * (m_nIter + 1) + m_ctrlHorizon * m_nCtrlJoints + i * m_nJets,
                            m_nStates * (m_nIter + 1) + m_ctrlHorizon * m_nCtrlJoints
                                + (i + 1) * m_nJets,
                            m_nJets,
                            m_nJets)
                -= m_weightThrustMatrix;
            m_hessian.block(m_nStates * (m_nIter + 1) + m_ctrlHorizon * m_nCtrlJoints
                                + (i + 1) * m_nJets,
                            m_nStates * (m_nIter + 1) + m_ctrlHorizon * m_nCtrlJoints
                                + (i + 1) * m_nJets,
                            m_nJets,
                            m_nJets)
                += m_weightThrustMatrix;
            m_hessian.block(m_nStates * (m_nIter + 1) + m_ctrlHorizon * m_nCtrlJoints
                                + m_nJets * (m_ctrlHorizon - m_nSmallSteps + 1) + i * m_nJets,
                            m_nStates * (m_nIter + 1) + m_ctrlHorizon * m_nCtrlJoints
                                + m_nJets * (m_ctrlHorizon - m_nSmallSteps + 1) + i * m_nJets,
                            m_nJets,
                            m_nJets)
                += m_weightThrottleMatrix;
            m_hessian.block(m_nStates * (m_nIter + 1) + m_ctrlHorizon * m_nCtrlJoints
                                + m_nJets * (m_ctrlHorizon - m_nSmallSteps + 1) + (i + 1) * m_nJets,
                            m_nStates * (m_nIter + 1) + m_ctrlHorizon * m_nCtrlJoints
                                + m_nJets * (m_ctrlHorizon - m_nSmallSteps + 1) + i * m_nJets,
                            m_nJets,
                            m_nJets)
                -= m_weightThrottleMatrix;
            m_hessian.block(m_nStates * (m_nIter + 1) + m_ctrlHorizon * m_nCtrlJoints
                                + m_nJets * (m_ctrlHorizon - m_nSmallSteps + 1) + i * m_nJets,
                            m_nStates * (m_nIter + 1) + m_ctrlHorizon * m_nCtrlJoints
                                + m_nJets * (m_ctrlHorizon - m_nSmallSteps + 1) + (i + 1) * m_nJets,
                            m_nJets,
                            m_nJets)
                -= m_weightThrottleMatrix;
            m_hessian.block(m_nStates * (m_nIter + 1) + m_ctrlHorizon * m_nCtrlJoints
                                + m_nJets * (m_ctrlHorizon - m_nSmallSteps + 1) + (i + 1) * m_nJets,
                            m_nStates * (m_nIter + 1) + m_ctrlHorizon * m_nCtrlJoints
                                + m_nJets * (m_ctrlHorizon - m_nSmallSteps + 1) + (i + 1) * m_nJets,
                            m_nJets,
                            m_nJets)
                += m_weightThrottleMatrix;
        }
        m_firstUpdate = false;
    }
    return true;
}

const bool RegualarizationCost::configureVectorsCollectionServer(QPInput& qpInput)
{
    return true;
}

const bool
RegualarizationCost::populateVectorsCollection(QPInput& qpInput,
                                               const Eigen::Ref<Eigen::VectorXd> qpSolution)
{
    return true;
}

ThrottleInitialValueCost::ThrottleInitialValueCost(const unsigned int nVar,
                                                   const unsigned int nState,
                                                   const unsigned int nJoints,
                                                   const unsigned int nThrottle)
    : IQPCost(nVar)
{
    m_nStates = nState;
    m_nCtrlJoints = nJoints;
    m_nJets = nThrottle;
    m_firstUpdate = true;
}

const bool ThrottleInitialValueCost::readConfigParameters(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{
    auto ptr = parametersHandler.lock();
    if (!ptr->getParameter("nIter", m_nIter))
    {
        yError() << "Parameter 'nIter' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("weightInitialThrottle", m_weightThrottle))
    {
        yError() << "Parameter 'weightThrottle' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("weightInitialThrust", m_weightThrust))
    {
        yError() << "Parameter 'weightThrottle' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("controlHorizon", m_ctrlHorizon))
    {
        yError() << "Parameter 'controlHorizon' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("nIterSmall", m_nSmallSteps))
    {
        yError() << "Parameter 'nIterSmall' not found in the config file.";
        return false;
    }
    return true;
}

void ThrottleInitialValueCost::configureDynVectorsSize(QPInput& qpInput)
{
}

const bool ThrottleInitialValueCost::computeHessianAndGradient(QPInput& qpInput)
{
    if (m_firstUpdate)
    {
        m_hessian.block(m_nStates * (m_nIter + 1) + m_nCtrlJoints * m_ctrlHorizon,
                        m_nStates * (m_nIter + 1) + m_nCtrlJoints * m_ctrlHorizon,
                        m_nJets,
                        m_nJets)
            = m_weightThrust * Eigen::MatrixXd::Identity(m_nJets, m_nJets);
        m_hessian.block(m_nStates * (m_nIter + 1) + m_nCtrlJoints * m_ctrlHorizon
                            + m_nJets * (m_ctrlHorizon - m_nSmallSteps + 1),
                        m_nStates * (m_nIter + 1) + m_nCtrlJoints * m_ctrlHorizon
                            + m_nJets * (m_ctrlHorizon - m_nSmallSteps + 1),
                        m_nJets,
                        m_nJets)
            = m_weightThrottle * Eigen::MatrixXd::Identity(m_nJets, m_nJets);
        m_firstUpdate = false;
    }
    for (int i = 0; i < m_nJets; i++)
    {
        m_gradient(m_nStates * (m_nIter + 1) + m_nCtrlJoints * m_ctrlHorizon + i)
            = -m_weightThrust * standardizeThrust(qpInput.getThrustDesMPC()[i]);
        m_gradient(m_nStates * (m_nIter + 1) + m_nCtrlJoints * m_ctrlHorizon
                   + m_nJets * (m_ctrlHorizon - m_nSmallSteps + 1) + i)
            = -m_weightThrottle * standardizeThrottle(qpInput.getThrottleMPC()[i]);
    }
    return true;
}

const bool ThrottleInitialValueCost::configureVectorsCollectionServer(QPInput& qpInput)
{
    return true;
}

const bool
ThrottleInitialValueCost::populateVectorsCollection(QPInput& qpInput,
                                                    const Eigen::Ref<Eigen::VectorXd> qpSolution)
{
    return true;
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

ArtificialEquilibriumTrackingCost::ArtificialEquilibriumTrackingCost(
    const unsigned int nVar,
    const unsigned int nState,
    const unsigned int nIter,
    const unsigned int nArtificialEquilibriumStates,
    const unsigned int artificialEquilibriumStatesInitPosition)
    : IQPCost(nVar)
{
    m_nStates = nState;
    m_nIter = nIter;
    m_nArtificialEquilibriumStates = nArtificialEquilibriumStates;
    m_artificialEquilibriumStatesInitPosition = artificialEquilibriumStatesInitPosition;
    m_firstUpdate = true;
    m_weightCoMPos.resize(3);
    m_weightLinMom.resize(3);
    m_weightRPY.resize(3);
    m_weightAngMom.resize(3);
    m_initialCoMPos.resize(3);
    m_initialRPY.resize(3);
    m_inertia.setZero();
    m_W.setZero();
    m_artificialEquilibriumReference.resize(m_nArtificialEquilibriumStates);
}

const bool ArtificialEquilibriumTrackingCost::readConfigParameters(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{
    auto ptr = parametersHandler.lock();
    bool ok = getParameterAndCheckSize(parametersHandler, "weightCoMPos", m_weightCoMPos);
    ok = ok && getParameterAndCheckSize(parametersHandler, "weightLinMom", m_weightLinMom);
    ok = ok && getParameterAndCheckSize(parametersHandler, "weightRPY", m_weightRPY);
    ok = ok && getParameterAndCheckSize(parametersHandler, "weightAngMom", m_weightAngMom);
    auto paramHandlerYarpImpl
        = std::make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>();
    auto group = ptr->getGroup("POSITION_TRAJECTORY");
    if (group.lock() == nullptr)
    {
        yError() << "Group [TRAJECTORY_MANAGER] not found in the config file.";
        return false;
    }
    paramHandlerYarpImpl->setGroup("TRAJECTORY_MANAGER", group.lock());
    double periodMPC;
    if (!ptr->getParameter("periodMPC", periodMPC))
    {
        yError() << "Parameter 'periodMPC' not found in the config file.";
        return false;
    }
    double periodMPCLargeSteps;
    if (!ptr->getParameter("periodMPCLargeSteps", periodMPCLargeSteps))
    {
        yError() << "Parameter 'periodMPCLargeSteps' not found in the config file.";
        return false;
    }
    double periodMPCSmallSteps;
    if (!ptr->getParameter("periodMPCSmallSteps", periodMPCSmallSteps))
    {
        yError() << "Parameter 'periodMPCSmallSteps' not found in the config file.";
        return false;
    }
    m_trajManager.configure(paramHandlerYarpImpl, 1 / periodMPCLargeSteps);
    m_ratioSmallLargeStepsPeriod = round(periodMPCLargeSteps / periodMPCSmallSteps);
    return true;
}

void ArtificialEquilibriumTrackingCost::configureDynVectorsSize(QPInput& qpInput)
{
    m_robot = qpInput.getRobot();
    m_vectorsCollectionServer = qpInput.getVectorsCollectionServer();
    m_Q.resize(m_nArtificialEquilibriumStates, m_nArtificialEquilibriumStates);
    m_Q.setZero();
    m_Q.block(CoMPosIdx[0], CoMPosIdx[0], CoMPosIdx.size(), CoMPosIdx.size())
        = m_weightCoMPos.asDiagonal();
    m_Q.block(linMomIdx[0], linMomIdx[0], linMomIdx.size(), linMomIdx.size())
        = m_weightLinMom.asDiagonal();
    m_Q.block(rpyIdx[0], rpyIdx[0], rpyIdx.size(), rpyIdx.size()) = m_weightRPY.asDiagonal();
    m_Q.block(angMomIdx[0], angMomIdx[0], angMomIdx.size(), angMomIdx.size())
        = m_weightAngMom.asDiagonal();
    m_Q = 100 * m_Q;
    m_initialCoMPos = m_robot->getPositionCoM();
    m_initialRPY = iDynTree::toEigen(m_robot->getBasePose().getRotation().asRPY());
    this->updateInertiaMatrix();
    m_gradient.setZero();
    m_counter = (m_ratioSmallLargeStepsPeriod - 1);
}

const bool ArtificialEquilibriumTrackingCost::computeHessianAndGradient(QPInput& qpInput)
{
    if (m_counter == (m_ratioSmallLargeStepsPeriod - 1))
    {
        m_trajManager.advanceTrajectory();
        m_artificialEquilibriumReference.segment(CoMPosIdx[0], CoMPosIdx.size())
            = m_initialCoMPos + m_trajManager.getCurrentValue("positionCoM");
        m_artificialEquilibriumReference.segment(linMomIdx[0], linMomIdx.size())
            = iDynTree::toEigen(m_robot->getBasePose().getRotation()).transpose()
              * m_robot->getTotalMass() * m_trajManager.getCurrentValue("velocityCoM");
        m_artificialEquilibriumReference.segment(rpyIdx[0], rpyIdx.size())
            = m_initialRPY + m_trajManager.getCurrentValue("RPY");
        this->updateInertiaMatrix();
        m_artificialEquilibriumReference.segment(angMomIdx[0], angMomIdx.size())
            = m_inertia * m_W * m_trajManager.getCurrentValue("RPYDot");
        m_gradient.segment(m_artificialEquilibriumStatesInitPosition,
                           m_nArtificialEquilibriumStates)
            = -m_Q * m_artificialEquilibriumReference;
        qpInput.setPosCoMReference(
            m_artificialEquilibriumReference.segment(CoMPosIdx[0], CoMPosIdx.size()));
        qpInput.setRPYReference(m_artificialEquilibriumReference.segment(rpyIdx[0], rpyIdx.size()));
        Eigen::Vector6d momentumReference;
        momentumReference.head(3)
            = m_artificialEquilibriumReference.segment(linMomIdx[0], linMomIdx.size());
        momentumReference.tail(3)
            = m_artificialEquilibriumReference.segment(angMomIdx[0], angMomIdx.size());
        qpInput.setMomentumReference(momentumReference);
        m_counter = 0;
    } else
    {
        m_counter++;
    }
    if (m_firstUpdate)
    {
        m_hessian.setZero();
        m_hessian.block(m_artificialEquilibriumStatesInitPosition,
                        m_artificialEquilibriumStatesInitPosition,
                        m_nArtificialEquilibriumStates,
                        m_nArtificialEquilibriumStates)
            = m_Q;
        m_firstUpdate = false;
    }
    return true;
}

void ArtificialEquilibriumTrackingCost::updateInertiaMatrix()
{
    iDynTree::Position r;
    iDynTree::toEigen(r)
        = m_robot->getPositionCoM() - iDynTree::toEigen(m_robot->getBasePose().getPosition());
    iDynTree::Transform G_H_B;
    G_H_B.setPosition(r);
    G_H_B.setRotation(m_robot->getBasePose().getRotation());
    Eigen::MatrixXd M_b = m_robot->getMassMatrix().block(0, 0, 6, 6);
    auto m_RPY = iDynTree::toEigen(m_robot->getBasePose().getRotation().asRPY());
    m_W.setZero();
    m_W(0, 0) = 1.0;
    m_W(1, 1) = cos(m_RPY(0));
    m_W(2, 1) = -sin(m_RPY(0));
    m_W(0, 2) = -sin(m_RPY(1));
    m_W(1, 2) = cos(m_RPY(1)) * sin(m_RPY(0));
    m_W(2, 2) = cos(m_RPY(0)) * cos(m_RPY(1));
    m_inertia = (iDynTree::toEigen(G_H_B.asAdjointTransform()).transpose() * M_b
                 * iDynTree::toEigen(G_H_B.asAdjointTransform()))
                    .block(3, 3, 3, 3);
}

const bool ArtificialEquilibriumTrackingCost::configureVectorsCollectionServer(QPInput& qpInput)
{
    return true;
}

const bool ArtificialEquilibriumTrackingCost::populateVectorsCollection(
    QPInput& qpInput, const Eigen::Ref<Eigen::VectorXd> qpSolution)
{
    return true;
}

StateArtificialEquiliriumTrackCost::StateArtificialEquiliriumTrackCost(
    const unsigned int nVar,
    const unsigned int nState,
    const unsigned int nIter,
    const unsigned int nArtificialEquilibriumStates,
    const unsigned int artificialEquilibriumStatesInitPosition)
    : IQPCost(nVar)
{
    m_nArtificialEquilibriumStates = nArtificialEquilibriumStates;
    m_artificialEquilibriumStatesInitPosition = artificialEquilibriumStatesInitPosition;
    m_nStates = nState;
    m_nIter = nIter;
    m_firstUpdate = true;
    m_weightCoMPos.resize(3);
    m_weightLinMom.resize(3);
    m_weightRPY.resize(3);
    m_weightAngMom.resize(3);
}

const bool StateArtificialEquiliriumTrackCost::readConfigParameters(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{
    auto ptr = parametersHandler.lock();
    bool ok = getParameterAndCheckSize(parametersHandler, "weightCoMPos", m_weightCoMPos);
    ok = ok && getParameterAndCheckSize(parametersHandler, "weightLinMom", m_weightLinMom);
    ok = ok && getParameterAndCheckSize(parametersHandler, "weightRPY", m_weightRPY);
    ok = ok && getParameterAndCheckSize(parametersHandler, "weightAngMom", m_weightAngMom);
    return ok;
}

void StateArtificialEquiliriumTrackCost::configureDynVectorsSize(QPInput& qpInput)
{
    m_robot = qpInput.getRobot();
    m_vectorsCollectionServer = qpInput.getVectorsCollectionServer();
    m_Q.resize(m_nArtificialEquilibriumStates, m_nArtificialEquilibriumStates);
    m_Q.setZero();
    m_Q.block(CoMPosIdx[0], CoMPosIdx[0], CoMPosIdx.size(), CoMPosIdx.size())
        = m_weightCoMPos.asDiagonal();
    m_Q.block(linMomIdx[0], linMomIdx[0], linMomIdx.size(), linMomIdx.size())
        = m_weightLinMom.asDiagonal();
    m_Q.block(rpyIdx[0], rpyIdx[0], rpyIdx.size(), rpyIdx.size()) = m_weightRPY.asDiagonal();
    m_Q.block(angMomIdx[0], angMomIdx[0], angMomIdx.size(), angMomIdx.size())
        = m_weightAngMom.asDiagonal();
    m_hessian.setZero();
    m_gradient.setZero();
}

const bool StateArtificialEquiliriumTrackCost::computeHessianAndGradient(QPInput& qpInput)
{
    if (m_firstUpdate)
    {
        for (int i = 0; i < m_nIter + 1; i++)
        {
            m_hessian.block(i * m_nStates, i * m_nStates, m_nStates, m_nStates) += m_Q;
            m_hessian.block(i * m_nStates,
                            m_artificialEquilibriumStatesInitPosition,
                            m_nStates,
                            m_nArtificialEquilibriumStates)
                -= m_Q;
            m_hessian.block(m_artificialEquilibriumStatesInitPosition,
                            i * m_nStates,
                            m_nArtificialEquilibriumStates,
                            m_nStates)
                -= m_Q;
            m_hessian.block(m_artificialEquilibriumStatesInitPosition,
                            m_artificialEquilibriumStatesInitPosition,
                            m_nArtificialEquilibriumStates,
                            m_nArtificialEquilibriumStates)
                += m_Q;
        }
        m_firstUpdate = false;
    }
    return true;
}

const bool StateArtificialEquiliriumTrackCost::configureVectorsCollectionServer(QPInput& qpInput)
{
    return true;
}

const bool StateArtificialEquiliriumTrackCost::populateVectorsCollection(
    QPInput& qpInput, const Eigen::Ref<Eigen::VectorXd> qpSolution)
{
    return true;
}

} // namespace DFMPC
