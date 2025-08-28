#ifndef CONSTRAINT_DF_MPC_H
#define CONSTRAINT_DF_MPC_H

#include "systemDynamicsDFMPC.h"
#include <IMPCProblem/IQPUtilsMPC.h>

namespace DFMPC
{

class ConstraintSystemDynamicDF : public IQPConstraint
{
public:
    ConstraintSystemDynamicDF(
        const int nVar, const int nStates, const int nJoints, const int nThrottle, const int nIter);
    ~ConstraintSystemDynamicDF() = default;

    const bool readConfigParameters(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;

    void configureDynVectorsSize(QPInput& qpInput) override;

    const bool computeConstraintsMatrixAndBounds(QPInput& qpInput) override;

    const bool populateVectorsCollection(QPInput& qpInput,
                                         const Eigen::Ref<Eigen::VectorXd> qpSolution) override;

    const bool configureVectorsCollectionServer(QPInput& qpInput) override;

private:
    double warp_function(double t);

    bool m_updateThrottle;
    double m_counter;
    int m_nStates;
    int m_nJoints;
    int m_nThrottle;
    int m_nIter;
    int m_nSmallSteps;
    int m_ctrlHorizon;
    double m_deltaT;
    double m_deltaTSmallSteps;
    double m_deltaTLargeSteps;
    double m_beta1;
    double m_beta2;
    std::unique_ptr<SystemDynamicDF> m_systemDynamicDF;
    Eigen::MatrixXd m_A;
    Eigen::MatrixXd m_BJoints;
    Eigen::MatrixXd m_BThrottle;
    Eigen::VectorXd m_c;
};

class ConstraintInitialStateDF : public IQPConstraintInitialState
{
public:
    ConstraintInitialStateDF(const unsigned int nStates, const unsigned int nVar);
    const bool readConfigParameters(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;

    void configureDynVectorsSize(QPInput& qpInput) override;

    bool updateInitialState(QPInput& qpInput) override;

    const bool configureVectorsCollectionServer(QPInput& qpInput) override;

    const bool populateVectorsCollection(QPInput& qpInput,
                                         const Eigen::Ref<Eigen::VectorXd> qpSolution) override;

private:
    void unwrapRPY();

    Eigen::Vector3d m_initialCoMPos;
    Eigen::Vector3d m_initialLinMom;
    Eigen::Vector3d m_initialRPY;
    Eigen::Vector3d m_RPYinit;
    Eigen::Vector3d m_initialAngMom;
    Eigen::VectorXd m_initialThrust;
    Eigen::VectorXd m_initialThrustDot;
    Eigen::Vector3d m_rpyOld;
    Eigen::Vector3d m_rpyUnwrapped;
    double m_initialAlphaGravity;
    double m_periodMPC;
    int m_nStates;
    int m_nInput;
    int m_nIter;

    Eigen::Vector3d m_nTurns;
    std::shared_ptr<Robot> m_robot;
    std::shared_ptr<BipedalLocomotion::YarpUtilities::VectorsCollectionServer>
        m_vectorsCollectionServer;
};

class ThrottleConstraintDD : public IQPConstraint
{
public:
    ThrottleConstraintDD(const int nVar,
                         const int nStates,
                         const int nIter,
                         const int nSmallSteps,
                         const int ctrlHorizon);
    ~ThrottleConstraintDD() = default;

    const bool readConfigParameters(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;

    void configureDynVectorsSize(QPInput& qpInput) override;

    const bool computeConstraintsMatrixAndBounds(QPInput& qpInput) override;

    const bool populateVectorsCollection(QPInput& qpInput,
                                         const Eigen::Ref<Eigen::VectorXd> qpSolution) override;

    const bool configureVectorsCollectionServer(QPInput& qpInput) override;

private:
    int m_nIter;
    int m_ctrlHorizon;
    int m_nSmallSteps;
    int m_nStates;
    int m_nJoints;
    int m_nThrottle;
    int m_counter;
    int m_ratioSmallLargeStepsPeriod;
    double m_throttleMaxValue;
    double m_throttleMinValue;
    bool m_firstIteriation{true};

    std::shared_ptr<Robot> m_robot;
    std::shared_ptr<BipedalLocomotion::YarpUtilities::VectorsCollectionServer>
        m_vectorsCollectionServer;
};

class JointPositionConstraintDD : public IQPConstraint
{
public:
    JointPositionConstraintDD(const int nVar, const int nStates, const int nJoints, const int nIter);
    ~JointPositionConstraintDD() = default;

    const bool readConfigParameters(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;

    void configureDynVectorsSize(QPInput& qpInput) override;

    const bool computeConstraintsMatrixAndBounds(QPInput& qpInput) override;

    const bool populateVectorsCollection(QPInput& qpInput,
                                         const Eigen::Ref<Eigen::VectorXd> qpSolution) override;

    const bool configureVectorsCollectionServer(QPInput& qpInput) override;

private:
    int m_nIter;
    int m_ctrlHorizon;
    int m_nStates;
    int m_nJoints;
    int m_counter;
    bool m_firstIteriation;
    Eigen::VectorXd m_jointPositionMax;
    Eigen::VectorXd m_jointPositionMin;
    std::shared_ptr<Robot> m_robot;
    std::shared_ptr<BipedalLocomotion::YarpUtilities::VectorsCollectionServer>
        m_vectorsCollectionServer;
    std::shared_ptr<JetModel> m_jetModel;
};

class ThrustContraintDD : public IQPConstraint
{
public:
    ThrustContraintDD(const int nVar,
                      const int nStates,
                      const int nIter,
                      const int nSmallSteps,
                      const int ctrlHorizon);
    ~ThrustContraintDD() = default;

    const bool readConfigParameters(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;

    void configureDynVectorsSize(QPInput& qpInput) override;

    const bool computeConstraintsMatrixAndBounds(QPInput& qpInput) override;

    const bool populateVectorsCollection(QPInput& qpInput,
                                         const Eigen::Ref<Eigen::VectorXd> qpSolution) override;

    const bool configureVectorsCollectionServer(QPInput& qpInput) override;

private:
    int m_nIter;
    int m_ctrlHorizon;
    int m_nStates;
    int m_nJoints;
    int m_nThrottle;
    int m_nSmallSteps;
    int m_counter;
    int m_ratioSmallLargeStepsPeriod;
    double m_thrustMaxValue;
    double m_thrustMinValue;
    bool m_firstIteriation{true};

    std::shared_ptr<Robot> m_robot;
    std::shared_ptr<BipedalLocomotion::YarpUtilities::VectorsCollectionServer>
        m_vectorsCollectionServer;
};

class HankleMatrixConstraint : public IQPConstraint
{
public:
    HankleMatrixConstraint(const int nVar,
                           const int nStates,
                           const int horizonLenghtHankleMatrix,
                           const int nJets,
                           const int nG,
                           const int gInitPosition,
                           const int throttleInitPosition,
                           const int thrustInitPosition,
                           const std::vector<std::vector<double>>& inputData,
                           const std::vector<std::vector<double>>& outputData);
    ~HankleMatrixConstraint() = default;

    const bool readConfigParameters(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;

    void configureDynVectorsSize(QPInput& qpInput) override;

    const bool computeConstraintsMatrixAndBounds(QPInput& qpInput) override;

    const bool populateVectorsCollection(QPInput& qpInput,
                                         const Eigen::Ref<Eigen::VectorXd> qpSolution) override;

    const bool configureVectorsCollectionServer(QPInput& qpInput) override;

private:
    void updateHankleMatrix();

    bool
    isHankleMatrixPersistelyExciting(Eigen::MatrixXd& hankleMatrix, const double threshold = 0.001);

    int m_nIter;
    int m_nStates;
    int m_nG;
    int m_horizonLenghtHankleMatrix;
    bool m_firstIteriation{true};
    int m_initGPosition;
    int m_initThrottlePosition;
    int m_initThrustPosition;
    int m_nsmallSteps;
    int m_ctrlHorizonInput;
    int m_numCols;
    int m_counter;
    int m_initDataLength;
    int m_nJets;
    int m_ratioSmallLargeStepsPeriod;
    std::vector<std::vector<double>> m_inputData;
    std::vector<std::vector<double>> m_outputData;
    std::vector<Eigen::MatrixXd> m_hankleMatrixInput;
    std::vector<Eigen::MatrixXd> m_hankleMatrixOutput;
    std::vector<std::vector<double>> m_initialThrottleValuesVector;
    std::vector<std::vector<double>> m_initialThrustValuesVector;
    std::shared_ptr<Robot> m_robot;
    std::shared_ptr<BipedalLocomotion::YarpUtilities::VectorsCollectionServer>
        m_vectorsCollectionServer;
};

class ArtificialEquilibriumConstraint : public IQPConstraint
{
public:
    ArtificialEquilibriumConstraint(const int nVar,
                                    const int nStates,
                                    const int nIter,
                                    const int nArtificialEquilibriumStates,
                                    const int ArtificialEquilibriumStatesInitPosition);
    ~ArtificialEquilibriumConstraint() = default;

    const bool readConfigParameters(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;

    void configureDynVectorsSize(QPInput& qpInput) override;

    const bool computeConstraintsMatrixAndBounds(QPInput& qpInput) override;

    const bool populateVectorsCollection(QPInput& qpInput,
                                         const Eigen::Ref<Eigen::VectorXd> qpSolution) override;

    const bool configureVectorsCollectionServer(QPInput& qpInput) override;

private:
    int m_nIter;
    int m_nStates;
    int m_nArtificialEquilibriumStates;
    int m_ArtificialEquilibriumStatesInitPosition;
    bool m_firstIteration{true};
};

} // namespace DFMPC

#endif // CONSTRAINT_DF_MPC_H
