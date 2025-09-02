#include <simpleExample/SimpleExampleMPC.h>
#include <simpleExample/SimpleExampleconstant.h>
#include <simpleExample/constraintsSimpleExample.h>
#include <simpleExample/costsSimpleExample.h>

const bool SimpleExampleMPC::setCostAndConstraints(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{
    if (!m_hankleMatrixSet)
    {
        yError() << "SimpleExampleMPC::setCostAndConstraints: the Hankel matrices are "
                    "not set; please first call setHankleMatrices()";
        return false;
    }
    auto ptr = parametersHandler.lock();
    if (!ptr->getParameter("nIter", m_nIter))
    {
        yError() << "Parameter 'nIter' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("HankleMatrixHorizon", m_horizonLenghtHankleMatrix))
    {
        yError() << "Parameter 'HankleMatrixHorizon' not found in the config file.";
        return false;
    }
    m_nStates = xIdx.size();
    m_nInput = N_Y_2 + N_U_1 + N_U_2;
    m_gParamNumber = m_inputData.size() - m_horizonLenghtHankleMatrix;
    m_slackVarNumber = m_horizonLenghtHankleMatrix;
    int nArtificialEquilibriumStates = N_Y_1 + N_Y_2;
    int nArtificialEquilibriumInputs = N_U_1 + N_U_2;
    m_nVar = m_nStates * (m_nIter + 1) + m_nInput * m_nIter + m_gParamNumber + m_slackVarNumber
             + nArtificialEquilibriumStates + nArtificialEquilibriumInputs;
    m_gParamInitPosition = m_nStates * (m_nIter + 1) + m_nInput * m_nIter;
    int slackVarInitPosition = m_gParamInitPosition + m_gParamNumber;
    m_slackVarIntiPos = slackVarInitPosition;
    m_artificialEquilibriumInitPosition = slackVarInitPosition + m_slackVarNumber;
    m_artificialInputInitPosition
        = m_artificialEquilibriumInitPosition + nArtificialEquilibriumStates;
    m_artificialEq.resize(nArtificialEquilibriumStates);

    // resize the vectors
    m_previousState.resize(m_nStates);
    m_QPSolution.resize(m_nVar);
    m_statesSolution.resize(m_nStates * (m_nIter + 1));
    m_inputSolution.resize(m_nInput * m_nIter);
    m_gParameters.resize(m_gParamNumber);
    m_slackVariables.resize(m_slackVarNumber);

    m_vectorCosts.emplace_back(
        std::make_unique<SIMPLE_EXAMPLE::TrackingCost>(m_nVar,
                                                       m_nStates,
                                                       m_nIter,
                                                       m_artificialEquilibriumInitPosition));
    m_vectorCosts.emplace_back(
        std::make_unique<SIMPLE_EXAMPLE::RegularizationGParametersCost>(m_nVar,
                                                                        m_gParamInitPosition,
                                                                        m_gParamNumber));
    m_vectorCosts.emplace_back(
        std::make_unique<SIMPLE_EXAMPLE::RegularizationSlackVariableCost>(m_nVar,
                                                                          slackVarInitPosition,
                                                                          m_slackVarNumber));
    m_vectorConstraints.emplace_back(
        std::make_unique<SIMPLE_EXAMPLE::ConstraintSystemDynamicSimpleExample>(m_nVar,
                                                                               m_nStates,
                                                                               m_nInput,
                                                                               m_nIter));
    m_vectorConstraints.emplace_back(
        std::make_unique<SIMPLE_EXAMPLE::ConstraintInitialStateSimpleExample>(m_nStates, m_nVar));
    m_vectorConstraints.emplace_back(
        std::make_unique<
            SIMPLE_EXAMPLE::HankelMatrixConstraintSimpleExample>(m_nVar,
                                                                 m_nStates,
                                                                 m_horizonLenghtHankleMatrix,
                                                                 m_gParamNumber,
                                                                 m_gParamInitPosition,
                                                                 m_inputData,
                                                                 m_outputData));
    m_vectorConstraints.emplace_back(
        std::make_unique<SIMPLE_EXAMPLE::TerminalConstraint>(m_nVar,
                                                             m_nStates,
                                                             m_nIter,
                                                             m_artificialEquilibriumInitPosition));
    m_vectorConstraints.emplace_back(
        std::make_unique<SIMPLE_EXAMPLE::InputConstraint>(m_nVar, m_nStates, m_nIter));
    m_vectorConstraints.emplace_back(
        std::make_unique<SIMPLE_EXAMPLE::OutputConstraint>(m_nVar, m_nStates, m_nIter));
    return true;
}

const bool SimpleExampleMPC::solveMPC()
{
    bool ok = this->solve();
    if (this->getQPProblemStatus() == OsqpEigen::Status::Solved)
    {
        m_QPSolution = this->getSolution();

        // extract the solution
        m_statesSolution = m_QPSolution.head(m_nStates * (m_nIter + 1));
        m_inputSolution = m_QPSolution.segment(m_nStates * (m_nIter + 1), m_nInput * m_nIter);
        m_gParameters = m_QPSolution.segment(m_gParamInitPosition, m_gParamNumber);
        m_slackVariables = m_QPSolution.segment(m_slackVarIntiPos, m_slackVarNumber);
        m_finalState = m_statesSolution.tail(m_nStates);
        m_artificialEq = m_QPSolution.segment(m_artificialEquilibriumInitPosition, 2);
    }

    return ok;
}

const bool
SimpleExampleMPC::updateMPC(Eigen::Ref<Eigen::VectorXd> xInit, double uData, double yData)
{
    QPInput qpinput;
    if (xInit.size() != xIdx.size())
    {
        yError() << "SimpleExampleMPC::updateMPC: the size of the initial state is not correct. "
                    "Xinit size"
                 << xInit.size() << " xIdx size " << xIdx.size();
        return false;
    }
    for (auto& constraint : m_vectorConstraints)
    {
        // if the name of the constraint is ConstraintInitialStateSimpleExample update the initial
        // state
        if (std::string(typeid(*constraint).name())
            == "N14SIMPLE_EXAMPLE35ConstraintInitialStateSimpleExampleE")
        {
            // cast the constraint to the specific type
            auto ptr
                = std::dynamic_pointer_cast<SIMPLE_EXAMPLE::ConstraintInitialStateSimpleExample>(
                    std::shared_ptr<IQPConstraint>(constraint.get(), [](IQPConstraint*) {}));
            if (ptr)
            {
                ptr->updateInitialState(xInit);
            }
        }
        // update the hankel matrix initial state
        if (std::string(typeid(*constraint).name())
            == "N14SIMPLE_EXAMPLE35HankelMatrixConstraintSimpleExampleE")
        {
            // cast the constraint to the specific type
            auto ptr
                = std::dynamic_pointer_cast<SIMPLE_EXAMPLE::HankelMatrixConstraintSimpleExample>(
                    std::shared_ptr<IQPConstraint>(constraint.get(), [](IQPConstraint*) {}));
            if (ptr)
            {
                ptr->updateInitialInputOutput(uData, yData);
            }
        }
    }
    this->update(qpinput);
    return true;
}

const bool SimpleExampleMPC::getControlInput(Eigen::Ref<Eigen::VectorXd> controlInput)
{
    if (controlInput.size() != U1Idx.size() + U2Idx.size())
    {
        yError() << "SimpleExampleMPC::getControlInput: wrong size of the input vector";
        return false;
    }
    controlInput = m_inputSolution.segment(U1Idx[0], U1Idx.size() + U2Idx.size());
    return true;
}

const double SimpleExampleMPC::getValueFunction()
{
    if (this->getQPProblemStatus() == OsqpEigen::Status::Solved)
    {
        double J_opt = this->getCostValue();
        for (auto& cost : m_vectorCosts)
        {
            // if the name of the cost is TrackingCost add the constant cost
            if (std::string(typeid(*cost).name()) == "N14SIMPLE_EXAMPLE12TrackingCostE")
            {
                // cast the cost to the specific type
                auto ptr = std::dynamic_pointer_cast<SIMPLE_EXAMPLE::TrackingCost>(
                    std::shared_ptr<IQPCost>(cost.get(), [](IQPCost*) {}));
                if (ptr)
                {
                    J_opt += ptr->getConstantCost();
                }
            }
        }
        return J_opt;
    } else
    {
        yError() << "SimpleExampleMPC::getValueFunction: the MPC problem is not "
                    "solved";
        return 0.0;
    }
}

const bool SimpleExampleMPC::setHankleMatrices(const std::vector<double>& inputData,
                                               const std::vector<double>& outputData)
{
    // check that the size of inputData and outputData is the same
    if (inputData.size() != outputData.size())
    {
        yError() << "SimpleExampleMPC::setHankleMatrices: the size of inputData and outputData is "
                    "not the same.";
        return false;
    }
    m_inputData = inputData;
    m_outputData = outputData;
    m_hankleMatrixSet = true;
    return true;
}

const bool SimpleExampleMPC::getMPCSolution(Eigen::Ref<Eigen::VectorXd> qpSolution)
{
    if (qpSolution.size() != m_nInput)
    {
        yError() << "SimpleExampleMPC::getMPCSolution: wrong size of the input vector";
        return false;
    }
    qpSolution = m_inputSolution;
    return true;
}

double SimpleExampleMPC::getNStatesMPC() const
{
    return m_nStates;
}

double SimpleExampleMPC::getNInputMPC() const
{
    return m_nInput;
}

const bool SimpleExampleMPC::getArtificialEquilibrium(Eigen::Ref<Eigen::VectorXd> artificialEq)
{
    if (artificialEq.size() != m_artificialEq.size())
    {
        yError() << "artificial eq size error";
        return false;
    }
    artificialEq = m_artificialEq;
    return true;
}

Eigen::VectorXd SimpleExampleMPC::getY1Output() const
{
    Eigen::VectorXd y1Output;
    y1Output.resize(m_nIter);
    for (int i = 0; i < m_nIter; ++i)
    {
        y1Output(i) = m_statesSolution((i + 1) * m_nStates);
    }
    return y1Output;
}

Eigen::VectorXd SimpleExampleMPC::getY2Output() const
{
    Eigen::VectorXd y2Output;
    y2Output.resize(m_nIter);
    for (int i = 0; i < m_nIter; ++i)
    {
        y2Output(i) = m_inputSolution(i * m_nInput + Y2Idx[0]);
    }
    return y2Output;
}

Eigen::VectorXd SimpleExampleMPC::getU1Input() const
{
    Eigen::VectorXd u1Input;
    u1Input.resize(m_nIter);
    for (int i = 0; i < m_nIter; ++i)
    {
        u1Input(i) = m_inputSolution(i * m_nInput + U1Idx[0]);
    }
    return u1Input;
}

Eigen::VectorXd SimpleExampleMPC::getU2Input() const
{
    Eigen::VectorXd u2Input;
    u2Input.resize(m_nIter);
    for (int i = 0; i < m_nIter; ++i)
    {
        u2Input(i) = m_inputSolution(i * m_nInput + U2Idx[0]);
    }
    return u2Input;
}

Eigen::VectorXd SimpleExampleMPC::getGParameters() const
{
    return m_gParameters;
}

Eigen::VectorXd SimpleExampleMPC::getSlackVariables() const
{
    return m_slackVariables;
}
