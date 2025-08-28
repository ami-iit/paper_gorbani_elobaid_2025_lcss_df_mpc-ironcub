#ifndef COST_H
#define COST_H

#include "QPInput.h"
#include <BipedalLocomotion/YarpUtilities/VectorsCollectionServer.h>
#include <Eigen/Dense>

/**
 * @class IQPCost
 * @brief The IQPCost is an abstract class to define a cost for a Quadratic Programming (QP)
 * problem. IQPCost provides the methods to configure the Hessian matrix and gradient vector of the
 * cost function.
 */
class IQPCost
{
public:
    /**
     * @brief Constructs an IQPCost object with the specified number of variables.
     * @param nVar The number of optimization variables in the QP problem.
     */
    IQPCost(const unsigned int nVar);

    /**
     * @brief Default destructor.
     */
    virtual ~IQPCost() = default;

    /**
     * @brief Resizes the Hessian matrix and gradient vector to accomodate the cost introduced by
     * this object.
     * @return None.
     */
    void configureSizeHessianAndGradient();

    /**
     * @brief Virtual method Configures the dynamic vectors required for constraint computation.
     * @param qpInput Struct for collecting input parameters to pass to the QPProblem class.
     * @return None.
     */
    virtual void configureDynVectorsSize(QPInput& qpInput) = 0;

    /**
     * @brief Virtual method to read the configuration parameters of the QP problem.
     * @param parametersHandler Reference to the ParametersHandler object (to access configuration
     * parameters).
     * @return True if the configuration was successful, false otherwise.
     */
    virtual const bool readConfigParameters(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput)
        = 0;

    /**
     * @brief Computes the Hessian matrix and gradient vector for the given robot state.
     * @param qpInput Struct for collecting input parameters to pass to the QPProblem class.
     * @return True if the computation was successful, false otherwise.
     */
    virtual const bool computeHessianAndGradient(QPInput& qpInput) = 0;

    /**
     * @brief Configures the VectorsCollectionServer object to publish the data to be logged.
     * @return True if the computation was successful, false otherwise.
     */
    virtual const bool configureVectorsCollectionServer(QPInput& qpInput) = 0;

    /**
     * @brief Populates the VectorsCollection object with the data to be logged.
     * @param qpInput Struct for collecting input parameters to pass to the QPProblem class.
     * @param qpSolution The solution of the QP problem.
     * @return True if the computation was successful, false otherwise.
     */
    virtual const bool
    populateVectorsCollection(QPInput& qpInput, const Eigen::Ref<Eigen::VectorXd> qpSolution)
        = 0;
    /**
     * @brief Returns the Hessian matrix of the cost function.
     * @return The Hessian matrix of the cost function.
     */
    Eigen::Ref<Eigen::MatrixXd> getHessian();

    /**
     * @brief Returns the gradient vector of the cost function.
     * @return The gradient vector of the cost function.
     */
    Eigen::Ref<Eigen::VectorXd> getGradient();

protected:
    unsigned int m_nVar; /**< The number of variables in the QP problem. */
    Eigen::MatrixXd m_hessian; /**< The Hessian matrix of the cost function. */
    Eigen::VectorXd m_gradient; /**< The gradient vector of the cost function. */
};

#endif
