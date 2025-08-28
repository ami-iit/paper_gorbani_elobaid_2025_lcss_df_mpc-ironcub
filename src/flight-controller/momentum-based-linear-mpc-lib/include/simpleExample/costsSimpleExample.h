#ifndef SIMPLE_EXAMPLE_COSTS
#define SIMPLE_EXAMPLE_COSTS

#include "TrajectoryManager.h"
#include <IMPCProblem/IQPUtilsMPC.h>
#include <IQPCost.h>

namespace SIMPLE_EXAMPLE
{

class TrackingCost : public IQPCost
{
public:
    TrackingCost(const unsigned int nVar,
                 const int nStates,
                 const int nIter,
                 const int artificialEquilibriumInitPosition);

    const bool readConfigParameters(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;

    void configureDynVectorsSize(QPInput& qpInput) override;

    const bool computeHessianAndGradient(QPInput& qpInput) override;

    const bool configureVectorsCollectionServer(QPInput& qpInput) override;

    const bool populateVectorsCollection(QPInput& qpInput,
                                         const Eigen::Ref<Eigen::VectorXd> qpSolution) override;

    double getConstantCost();

private:
    bool m_firstUpdate{true};
    int m_nStates;
    int m_artificialEquilibriumInitPosition;
    int m_artificialInputInit;
    int m_nIter;
    double m_referenceYAlpha;
    double m_referenceYBeta;
    std::vector<double> m_weightTracking;
    std::vector<double> m_weightArtificialEquilibrium;
    std::vector<double> m_weightR;
};

class RegularizationGParametersCost : public IQPCost
{
public:
    RegularizationGParametersCost(const unsigned int nVar, const int gInitPosition, const int nG);

    const bool readConfigParameters(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;

    void configureDynVectorsSize(QPInput& qpInput) override;

    const bool computeHessianAndGradient(QPInput& qpInput) override;

    const bool configureVectorsCollectionServer(QPInput& qpInput) override;

    const bool populateVectorsCollection(QPInput& qpInput,
                                         const Eigen::Ref<Eigen::VectorXd> qpSolution) override;

private:
    bool m_firstUpdate{true};
    int m_nG;
    int m_gInitPosition;
    double m_weightGParameters;
};

class RegularizationSlackVariableCost : public IQPCost
{
public:
    RegularizationSlackVariableCost(const unsigned int nVar,
                                    const int slackVarInitPosition,
                                    const int nSlackVar);

    const bool readConfigParameters(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;

    void configureDynVectorsSize(QPInput& qpInput) override;

    const bool computeHessianAndGradient(QPInput& qpInput) override;

    const bool configureVectorsCollectionServer(QPInput& qpInput) override;

    const bool populateVectorsCollection(QPInput& qpInput,
                                         const Eigen::Ref<Eigen::VectorXd> qpSolution) override;

private:
    bool m_firstUpdate{true};
    int m_nSlackVar;
    int m_slackVarInitPosition;
    double m_weightSlackVariable;
};

} // namespace SIMPLE_EXAMPLE

#endif // SIMPLE_EXAMPLE_COSTS
