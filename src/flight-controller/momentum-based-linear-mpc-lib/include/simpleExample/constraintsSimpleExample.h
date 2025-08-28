#ifndef CONSTRAINT_SIMPLE_EXAMPLE_MPC_H
#define CONSTRAINT_SIMPLE_EXAMPLE_MPC_H

#include "systemDynamicsSimpleExample.h"
#include <IMPCProblem/IQPUtilsMPC.h>

namespace SIMPLE_EXAMPLE
{

class ConstraintSystemDynamicSimpleExample : public IQPConstraintMPCDynamic
{
public:
    ConstraintSystemDynamicSimpleExample(const unsigned int nVar,
                                         const unsigned int nStates,
                                         const unsigned int nInput,
                                         const unsigned int nIter);
    ~ConstraintSystemDynamicSimpleExample() = default;

    const bool readConfigParameters(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;

    void configureDynVectorsSize(QPInput& qpInput) override;

    bool updateDynamicConstraints(QPInput& qpInput) override;

    const bool configureVectorsCollectionServer(QPInput& qpInput) override;

    const bool populateVectorsCollection(QPInput& qpInput,
                                         const Eigen::Ref<Eigen::VectorXd> qpSolution) override;

private:
    std::shared_ptr<BipedalLocomotion::YarpUtilities::VectorsCollectionServer>
        m_vectorsCollectionServer;
    std::shared_ptr<SystemDynamicsSimpleExample> m_systemDynamic;
};

class ConstraintInitialStateSimpleExample : public IQPConstraintInitialState
{
public:
    ConstraintInitialStateSimpleExample(const unsigned int nStates, const unsigned int nVar);
    const bool readConfigParameters(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;

    void configureDynVectorsSize(QPInput& qpInput) override;

    bool updateInitialState(QPInput& qpInput) override;

    const bool updateInitialState(const Eigen::Ref<Eigen::VectorXd> xInit);

    const bool configureVectorsCollectionServer(QPInput& qpInput) override;

    const bool populateVectorsCollection(QPInput& qpInput,
                                         const Eigen::Ref<Eigen::VectorXd> qpSolution) override;
};

class HankelMatrixConstraintSimpleExample : public IQPConstraint
{
public:
    HankelMatrixConstraintSimpleExample(const int nVar,
                                        const int nStates,
                                        const int horizonLenghtHankleMatrix,
                                        const int nG,
                                        const int gInitPosition,
                                        const std::vector<double>& inputData,
                                        const std::vector<double>& outputData);
    ~HankelMatrixConstraintSimpleExample() = default;

    const bool readConfigParameters(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;

    void configureDynVectorsSize(QPInput& qpInput) override;

    const bool computeConstraintsMatrixAndBounds(QPInput& qpInput) override;

    const bool populateVectorsCollection(QPInput& qpInput,
                                         const Eigen::Ref<Eigen::VectorXd> qpSolution) override;

    const bool configureVectorsCollectionServer(QPInput& qpInput) override;

    const bool updateInitialInputOutput(double uData, double yData);

private:
    void updateHankleMatrix();

    bool
    isHankleMatrixPersistelyExciting(Eigen::MatrixXd& hankleMatrix, const double threshold = 0.001);

    int m_nIter;
    int m_nStates;
    int m_nG;
    int m_horizonLenghtHankleMatrix;
    bool m_firstIteriation{true};
    bool m_firstUpdateInitialState{true};
    int m_initGPosition;
    int m_initThrottlePosition;
    int m_initThrustPosition;
    int m_numCols;
    int m_initDataLength;
    std::vector<double> m_inputData;
    std::vector<double> m_outputData;
    Eigen::MatrixXd m_hankleMatrixInput;
    Eigen::MatrixXd m_hankleMatrixOutput;
    std::vector<double> m_initialInputValuesVector;
    std::vector<double> m_initialOutputValuesVector;
    std::shared_ptr<BipedalLocomotion::YarpUtilities::VectorsCollectionServer>
        m_vectorsCollectionServer;
};

class TerminalConstraint : public IQPConstraint
{
public:
    TerminalConstraint(const int nVar,
                       const int nStates,
                       const int nIter,
                       const int artificialEqInit);
    ~TerminalConstraint() = default;
    const bool readConfigParameters(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;

    void configureDynVectorsSize(QPInput& qpInput) override;

    const bool computeConstraintsMatrixAndBounds(QPInput& qpInput) override;

    const bool populateVectorsCollection(QPInput& qpInput,
                                         const Eigen::Ref<Eigen::VectorXd> qpSolution) override;

    const bool configureVectorsCollectionServer(QPInput& qpInput) override;

private:
    int m_nStates;
    int m_nIter;
    int m_artificialEqInit;
    int m_artificialInputInit;
};

class InputConstraint : public IQPConstraint
{
public:
    InputConstraint(const int nVar, const int nStates, const int nIter);
    ~InputConstraint() = default;

    const bool readConfigParameters(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;

    void configureDynVectorsSize(QPInput& qpInput) override;

    const bool computeConstraintsMatrixAndBounds(QPInput& qpInput) override;

    const bool populateVectorsCollection(QPInput& qpInput,
                                         const Eigen::Ref<Eigen::VectorXd> qpSolution) override;

    const bool configureVectorsCollectionServer(QPInput& qpInput) override;

private:
    int m_nStates;
    int m_nIter;
    int m_nInput;
};

class OutputConstraint : public IQPConstraint
{
public:
    OutputConstraint(const int nVar, const int nStates, const int nIter);
    ~OutputConstraint() = default;

    const bool readConfigParameters(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;

    void configureDynVectorsSize(QPInput& qpInput) override;

    const bool computeConstraintsMatrixAndBounds(QPInput& qpInput) override;

    const bool populateVectorsCollection(QPInput& qpInput,
                                         const Eigen::Ref<Eigen::VectorXd> qpSolution) override;

    const bool configureVectorsCollectionServer(QPInput& qpInput) override;

private:
    int m_nStates;
    int m_nIter;
    int m_nInput;
    std::vector<double> m_yMax;
    std::vector<double> m_yMin;
};

} // namespace SIMPLE_EXAMPLE

#endif // CONSTRAINT_SIMPLE_EXAMPLE_MPC_H
