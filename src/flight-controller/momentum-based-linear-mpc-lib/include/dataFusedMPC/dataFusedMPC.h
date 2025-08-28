
#ifndef DATA_DRIVEN_VARIABLE_SAMPLING_MPC_H
#define DATA_DRIVEN_VARIABLE_SAMPLING_MPC_H

#include <IMPCProblem/IMPCProblem.h>
#include <IMPCProblem/IQPUtilsMPC.h>
#include <IQPCost.h>

class DataFusedMPC : public IMPCProblem
{
public:
    const bool setHankleMatrices(const std::vector<std::vector<double>>& inptuData,
                                 const std::vector<std::vector<double>>& outputData);

    const bool getMPCSolution(Eigen::Ref<Eigen::VectorXd> qpSolution);

    const bool getJointsReferencePosition(Eigen::Ref<Eigen::VectorXd> jointsPosition);

    const bool getThrottleReference(Eigen::Ref<Eigen::VectorXd> throttle);

    const bool getThrustReference(Eigen::Ref<Eigen::VectorXd> thrust);

    const bool getFinalCoMPosition(Eigen::Ref<Eigen::VectorXd> finalCoMPosition);

    const bool getFinalLinMom(Eigen::Ref<Eigen::VectorXd> finalLinMom);

    const bool getFinalRPY(Eigen::Ref<Eigen::VectorXd> finalRPY);

    const bool getFinalAngMom(Eigen::Ref<Eigen::VectorXd> finalAngMom);

    const bool getArtificialEquilibrium(Eigen::Ref<Eigen::VectorXd> artificialEquilibrium);

    const bool solveMPC();

    const double getValueFunction();

    double getNStatesMPC() const;

    double getNInputMPC() const;

private:
    const bool setCostAndConstraints(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;

    int m_nStates;
    int m_nInput;
    int m_nIter;
    int m_ctrlHorizon;
    int m_nIterSmall;
    int m_nCtrlJoints;
    int m_nJets;
    int m_horizonLenghtHankleMatrix;
    int m_throttleInitPosition;
    int m_gParamInitPosition;
    int m_nArtificialEquilibriumStates;
    int m_artificialEquilibriumStatesInitPosition;
    std::vector<int> m_jointSelectorVector;
    std::vector<std::string> m_controlledJoints;
    std::vector<std::vector<double>> m_inputData;
    std::vector<std::vector<double>> m_outputData;
    bool m_hankleMatrixSet{false};
    std::shared_ptr<Robot> m_robot;

    Eigen::VectorXd m_previousState;
    Eigen::VectorXd m_QPSolution;
    Eigen::VectorXd m_jointsPositionReference;
    Eigen::VectorXd m_deltaJointsPositionReference;
    Eigen::VectorXd m_thrustReference;
    Eigen::VectorXd m_throttleReference;
    Eigen::VectorXd m_statesSolution;
    Eigen::VectorXd m_inputSolution;
    Eigen::VectorXd m_finalState;
    Eigen::VectorXd m_artificialEquilibrium;
};

#endif // DATA_DRIVEN_VARIABLE_SAMPLING_MPC_H
