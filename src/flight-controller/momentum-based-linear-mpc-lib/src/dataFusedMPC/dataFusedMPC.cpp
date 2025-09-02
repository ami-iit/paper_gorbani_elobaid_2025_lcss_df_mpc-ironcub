#include <FlightControlUtils.h>
#include <dataFusedMPC/DFconstant.h>
#include <dataFusedMPC/constraintsDFMPC.h>
#include <dataFusedMPC/costsDFMPC.h>
#include <dataFusedMPC/dataFusedMPC.h>

const bool DataFusedMPC::setCostAndConstraints(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{
    if (!m_hankleMatrixSet)
    {
        yError() << "DataFusedMPC::setCostAndConstraints: the Hankel matrices are "
                    "not set; please first call setHankleMatrices()";
        return false;
    }
    auto ptr = parametersHandler.lock();
    if (!ptr->getParameter("controlledJoints", m_controlledJoints))
    {
        yError() << "Parameter 'controlledJoints' not found in the config file.";
        return false;
    }
    m_nCtrlJoints = m_controlledJoints.size();
    if (m_nCtrlJoints != deltaJointIdx.size())
    {
        yError() << "The number of controlled joints defined in the systemDynamic.h file is "
                    "different from the size of the 'controlledJoints' parameter";
        return false;
    }
    if (!ptr->getParameter("nIter", m_nIter))
    {
        yError() << "Parameter 'nIter' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("nIterSmall", m_nIterSmall))
    {
        yError() << "Parameter 'nIterSmall' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("controlHorizon", m_ctrlHorizon))
    {
        yError() << "Parameter 'controlHorizon' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("HankleMatrixHorizon", m_horizonLenghtHankleMatrix))
    {
        yError() << "Parameter 'HankleMatrixHorizon' not found in the config file.";
        return false;
    }
    m_robot = qpInput.getRobot();
    m_nJets = m_robot->getNJets();
    m_nStates = angMomIdx[2] + 1;
    m_nInput = m_nCtrlJoints + m_nJets;
    int gParamNumber = m_inputData[0].size() - m_horizonLenghtHankleMatrix;

    // number of slack variables
    int slackVarNumber
        = m_nJets * (m_horizonLenghtHankleMatrix - (m_ctrlHorizon - m_nIterSmall + 1));

    // number of artificial equilibrium states variables
    m_nArtificialEquilibriumStates = m_nStates;

    // number of variables
    m_nVar = m_nStates * (m_nIter + 1) + m_nCtrlJoints * m_ctrlHorizon
             + 2 * m_nJets * (m_ctrlHorizon - m_nIterSmall + 1) + m_nJets * gParamNumber
             + slackVarNumber + m_nArtificialEquilibriumStates;
    m_gParamInitPosition = m_nStates * (m_nIter + 1) + m_nCtrlJoints * m_ctrlHorizon
                           + 2 * m_nJets * (m_ctrlHorizon - m_nIterSmall + 1);
    m_throttleInitPosition = m_nStates * (m_nIter + 1) + m_nCtrlJoints * m_ctrlHorizon
                             + m_nJets * (m_ctrlHorizon - m_nIterSmall + 1);
    int thrustInitPosition = m_nStates * (m_nIter + 1) + m_nCtrlJoints * m_ctrlHorizon;
    int slackVarInitPosition = m_nStates * (m_nIter + 1) + m_nCtrlJoints * m_ctrlHorizon
                               + 2 * m_nJets * (m_ctrlHorizon - m_nIterSmall + 1)
                               + m_nJets * gParamNumber;
    m_artificialEquilibriumStatesInitPosition = m_nStates * (m_nIter + 1)
                                                + m_nCtrlJoints * m_ctrlHorizon
                                                + 2 * m_nJets * (m_ctrlHorizon - m_nIterSmall + 1)
                                                + m_nJets * gParamNumber + slackVarNumber;
    std::cout << "thrust init position: " << thrustInitPosition
              << " throttle init position: " << m_throttleInitPosition << std::endl;
    m_jointSelectorVector.clear();
    for (auto joint : m_controlledJoints)
    {
        for (int i = 0; i < m_robot->getNJoints(); i++)
        {
            if (joint == m_robot->getJointName(i))
            {
                m_jointSelectorVector.emplace_back(i);
            }
        }
    }

    // resize the vectors
    m_jointsPositionReference.resize(m_robot->getNJoints());
    m_jointsPositionReference = m_robot->getJointPos();
    m_previousState.resize(m_nStates);
    m_QPSolution.resize(m_nVar);
    m_deltaJointsPositionReference.resize(m_nCtrlJoints);
    m_thrustReference.resize(m_nJets);
    m_throttleReference.resize(m_nJets);
    m_statesSolution.resize(m_nStates * (m_nIter + 1));
    m_inputSolution.resize(m_nCtrlJoints * m_nIter + m_nJets * (m_nIter - m_nIterSmall + 1));

    m_vectorCosts.emplace_back(
        std::make_unique<
            DFMPC::ArtificialEquilibriumTrackingCost>(m_nVar,
                                                      m_nStates,
                                                      m_nIter,
                                                      m_nArtificialEquilibriumStates,
                                                      m_artificialEquilibriumStatesInitPosition));
    m_vectorCosts.emplace_back(
        std::make_unique<
            DFMPC::StateArtificialEquiliriumTrackCost>(m_nVar,
                                                       m_nStates,
                                                       m_nIter,
                                                       m_nArtificialEquilibriumStates,
                                                       m_artificialEquilibriumStatesInitPosition));
    m_vectorCosts.emplace_back(
        std::make_unique<DFMPC::RegualarizationCost>(m_nVar, m_nStates, m_nCtrlJoints, m_nJets));
    m_vectorCosts.emplace_back(std::make_unique<DFMPC::ThrottleInitialValueCost>(m_nVar,
                                                                                 m_nStates,
                                                                                 m_nCtrlJoints,
                                                                                 m_nJets));
    m_vectorCosts.emplace_back(
        std::make_unique<DFMPC::RegularizationGParametersCost>(m_nVar,
                                                               m_gParamInitPosition,
                                                               m_nJets * gParamNumber));
    m_vectorCosts.emplace_back(
        std::make_unique<DFMPC::RegularizationSlackVariableCost>(m_nVar,
                                                                 slackVarInitPosition,
                                                                 slackVarNumber));
    m_vectorConstraints.emplace_back(
        std::make_unique<DFMPC::ConstraintSystemDynamicDF>(m_nVar,
                                                           m_nStates,
                                                           m_nCtrlJoints,
                                                           m_nJets,
                                                           m_nIter));
    m_vectorConstraints.emplace_back(
        std::make_unique<DFMPC::ConstraintInitialStateDF>(m_nStates, m_nVar));
    m_vectorConstraints.emplace_back(std::make_unique<DFMPC::ThrottleConstraintDD>(m_nVar,
                                                                                   m_nStates,
                                                                                   m_nIter,
                                                                                   m_nIterSmall,
                                                                                   m_ctrlHorizon));
    m_vectorConstraints.emplace_back(std::make_unique<DFMPC::ThrustContraintDD>(m_nVar,
                                                                                m_nStates,
                                                                                m_nIter,
                                                                                m_nIterSmall,
                                                                                m_ctrlHorizon));
    m_vectorConstraints.emplace_back(
        std::make_unique<DFMPC::HankleMatrixConstraint>(m_nVar,
                                                        m_nStates,
                                                        m_horizonLenghtHankleMatrix,
                                                        m_nJets,
                                                        gParamNumber,
                                                        m_gParamInitPosition,
                                                        m_throttleInitPosition,
                                                        thrustInitPosition,
                                                        m_inputData,
                                                        m_outputData));
    m_vectorConstraints.emplace_back(
        std::make_unique<
            DFMPC::ArtificialEquilibriumConstraint>(m_nVar,
                                                    m_nStates,
                                                    m_nIter,
                                                    m_nArtificialEquilibriumStates,
                                                    m_artificialEquilibriumStatesInitPosition));
    return true;
}

const bool DataFusedMPC::solveMPC()
{
    this->solve();
    if (this->getQPProblemStatus() == OsqpEigen::Status::Solved)
    {
        m_QPSolution = this->getSolution();

        // extract the solution
        m_statesSolution = m_QPSolution.head(m_nStates * (m_nIter + 1));
        m_inputSolution
            = m_QPSolution.segment(m_nStates * (m_nIter + 1),
                                   m_nCtrlJoints * m_ctrlHorizon
                                       + 2 * m_nJets * (m_ctrlHorizon - m_nIterSmall + 1));
        m_deltaJointsPositionReference = m_inputSolution.segment(0, m_nCtrlJoints);
        m_throttleReference = m_QPSolution.segment(m_throttleInitPosition, m_nJets);
        m_thrustReference = m_inputSolution.segment(m_nCtrlJoints * m_ctrlHorizon, m_nJets);
        m_finalState = m_statesSolution.tail(m_nStates);
        m_artificialEquilibrium = m_QPSolution.segment(m_artificialEquilibriumStatesInitPosition,
                                                       m_nArtificialEquilibriumStates);
        for (int i = 0; i < m_jointSelectorVector.size(); i++)
        {
            m_jointsPositionReference(m_jointSelectorVector[i])
                += m_deltaJointsPositionReference(i);
        }
    }

    return true;
}

const double DataFusedMPC::getValueFunction()
{
    if (this->getQPProblemStatus() == OsqpEigen::Status::Solved)
    {
        return this->getCostValue();
    } else
    {
        yError() << "DataFusedMPC::getValueFunction: the MPC problem is not "
                    "solved";
        return 0.0;
    }
}

const bool DataFusedMPC::setHankleMatrices(const std::vector<std::vector<double>>& inputData,
                                           const std::vector<std::vector<double>>& outputData)
{
    m_hankleMatrixSet = true;
    if ((inputData.size() != N_THRUSTS) || (outputData.size() != N_THRUSTS))
    {
        yError() << "DataFusedMPC::setHankleMatrices: the input and output vector "
                    "must have the size: "
                 << N_THRUSTS << " instead of " << inputData.size() << " and " << outputData.size();
        return false;
    }
    m_inputData = inputData;
    m_outputData = outputData;
    return true;
}

const bool DataFusedMPC::getMPCSolution(Eigen::Ref<Eigen::VectorXd> qpSolution)
{
    if (qpSolution.size() != m_nInput)
    {
        yError() << "DataFusedMPC::getMPCSolution: wrong size of the input vector";
        return false;
    }
    qpSolution = m_inputSolution;
    return true;
}

const bool DataFusedMPC::getJointsReferencePosition(Eigen::Ref<Eigen::VectorXd> jointsPosition)
{
    if (jointsPosition.size() != m_robot->getNJoints())
    {
        yError() << "DataFusedMPC::getJointsReferencePosition: wrong size of the "
                    "input "
                    "vector";
        return false;
    }
    jointsPosition = m_jointsPositionReference;
    return true;
}

const bool DataFusedMPC::getThrottleReference(Eigen::Ref<Eigen::VectorXd> throttle)
{
    if (throttle.size() != m_nJets)
    {
        yError() << "DataFusedMPC::getThrottleReference: wrong size of the input "
                    "vector";
        return false;
    }

    for (int i = 0; i < m_nJets; i++)
    {
        throttle(i) = destandardizeThrottle(m_throttleReference(i));
    }
    return true;
}

const bool DataFusedMPC::getThrustReference(Eigen::Ref<Eigen::VectorXd> thrust)
{
    if (thrust.size() != m_nJets)
    {
        yError() << "DataFusedMPC::getThrustReference: wrong size of the input "
                    "vector";
        return false;
    }
    for (int i = 0; i < m_nJets; i++)
    {
        thrust(i) = destandardizeThrust(m_thrustReference(i));
    }
    return true;
}

const bool DataFusedMPC::getFinalCoMPosition(Eigen::Ref<Eigen::VectorXd> finalCoMPosition)
{
    if (finalCoMPosition.size() != 3)
    {
        yError() << "DataFusedMPC::getFinalCoMPosition: wrong size of the input "
                    "vector";
        return false;
    }
    finalCoMPosition = m_finalState.segment(CoMPosIdx[0], CoMPosIdx.size());
    return true;
}

const bool DataFusedMPC::getFinalLinMom(Eigen::Ref<Eigen::VectorXd> finalLinMom)
{
    if (finalLinMom.size() != 3)
    {
        yError() << "DataFusedMPC::getFinalLinMom: wrong size of the input vector";
        return false;
    }
    finalLinMom = m_finalState.segment(linMomIdx[0], linMomIdx.size());
    return true;
}

const bool DataFusedMPC::getFinalRPY(Eigen::Ref<Eigen::VectorXd> finalRPY)
{
    if (finalRPY.size() != 3)
    {
        yError() << "DataFusedMPC::getFinalRPY: wrong size of the input vector";
        return false;
    }
    finalRPY = m_finalState.segment(rpyIdx[0], rpyIdx.size());
    return true;
}

const bool DataFusedMPC::getFinalAngMom(Eigen::Ref<Eigen::VectorXd> finalAngMom)
{
    if (finalAngMom.size() != 3)
    {
        yError() << "DataFusedMPC::getFinalAngMom: wrong size of the input vector";
        return false;
    }
    finalAngMom = m_finalState.segment(angMomIdx[0], angMomIdx.size());
    return true;
}

const bool DataFusedMPC::getArtificialEquilibrium(Eigen::Ref<Eigen::VectorXd> artificialEquilibrium)
{
    if (artificialEquilibrium.size() != m_nArtificialEquilibriumStates)
    {
        yError() << "DataFusedMPC::getArtificialEquilibrium: wrong size of the "
                    "input "
                    "vector";
        return false;
    }
    artificialEquilibrium = m_artificialEquilibrium;
    return true;
}

double DataFusedMPC::getNStatesMPC() const
{
    return m_nStates;
}

double DataFusedMPC::getNInputMPC() const
{
    return m_nInput;
}
