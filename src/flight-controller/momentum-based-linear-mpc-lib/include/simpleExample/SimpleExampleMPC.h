
#ifndef SIMPLE_EXAMPLE_MPC_H
#define SIMPLE_EXAMPLE_MPC_H

#include <IMPCProblem/IMPCProblem.h>
#include <IMPCProblem/IQPUtilsMPC.h>
#include <IQPCost.h>

class SimpleExampleMPC : public IMPCProblem
{
public:
    const bool
    setHankleMatrices(const std::vector<double>& inputData, const std::vector<double>& outputData);

    const bool getMPCSolution(Eigen::Ref<Eigen::VectorXd> qpSolution);

    const bool getControlInput(Eigen::Ref<Eigen::VectorXd> controlInput);

    const bool solveMPC();

    const double getValueFunction();

    const bool updateMPC(Eigen::Ref<Eigen::VectorXd> xInit, double uData, double yData);

    double getNStatesMPC() const;

    double getNInputMPC() const;

    const bool getArtificialEquilibrium(Eigen::Ref<Eigen::VectorXd> artificialEq);

    Eigen::VectorXd getY1Output() const;

    Eigen::VectorXd getY2Output() const;

    Eigen::VectorXd getU1Input() const;

    Eigen::VectorXd getU2Input() const;

    Eigen::VectorXd getGParameters() const;

    Eigen::VectorXd getSlackVariables() const;

private:
    const bool setCostAndConstraints(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;

    int m_nStates;
    int m_nInput;
    int m_nIter;
    int m_gParamNumber;
    int m_horizonLenghtHankleMatrix;
    int m_slackVarNumber;
    int m_gParamInitPosition;
    int m_artificialEquilibriumInitPosition;
    int m_slackVarIntiPos;
    int m_artificialInputInitPosition;
    std::vector<double> m_inputData;
    std::vector<double> m_outputData;
    bool m_hankleMatrixSet{false};

    Eigen::VectorXd m_previousState;
    Eigen::VectorXd m_QPSolution;
    Eigen::VectorXd m_jointsPositionReference;
    Eigen::VectorXd m_deltaJointsPositionReference;
    Eigen::VectorXd m_thrustReference;
    Eigen::VectorXd m_throttleReference;
    Eigen::VectorXd m_statesSolution;
    Eigen::VectorXd m_inputSolution;
    Eigen::VectorXd m_gParameters;
    Eigen::VectorXd m_slackVariables;
    Eigen::VectorXd m_finalState;
    Eigen::VectorXd m_artificialEq;
};

#endif // SIMPLE_EXAMPLE_MPC_H
