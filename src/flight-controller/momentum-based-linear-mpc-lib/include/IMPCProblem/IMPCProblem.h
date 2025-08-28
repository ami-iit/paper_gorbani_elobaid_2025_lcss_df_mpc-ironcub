#ifndef IMPCROBLEM_H
#define IMPCROBLEM_H

#include "IQPConstraint.h"
#include "IQPCost.h"
#include "QPInput.h"
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <boost/core/demangle.hpp>
#include <string>
#include <typeinfo>
#include <vector>
#include <yarp/os/LogStream.h>

/**
 * @class IMPCProblem
 * @brief An abstract class to define a Quadratic Programming (QP) problem.
 *
 * The IMPCProblem class provides an interface for configuring, updating, and solving a QP
 * problem. It stores the problem's variables, constraints, costs, and solution. Derived classes
 * can override the protected virtual functions to customize the behavior of the QP problem.
 */
class IMPCProblem
{
public:
    /**
     * @brief Configure the QP problem allocating the required memory for the variables,
     * constraints, and costs.
     * @param qpInput Reference to the Robot object (to access kino-dynamic information).
     * @param parametersHandler Reference to the ParametersHandler object (to access configuration
     * parameters).
     * @return True if the configuration was successful, false otherwise.
     */
    const bool configure(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput);

    /**
     * @brief Update the QP costs and constraints values given robot state.
     * @param qpInput Struct for collecting input parameters to pass to the QPProblem class.
     * @return True if the update was successful, false otherwise.
     */
    const bool update(QPInput& qpInput);

    /**
     * @brief Send the data to the YarpRobotLoggerDevice.
     * @return True if the solution was found, false otherwise.
     */
    const bool sendVectorsCollectionToLog(QPInput& qpInput);

    /**
     * @brief Returns the solution of the QP problem.
     * @return The solution of the QP problem.
     */
    Eigen::Ref<const Eigen::VectorXd> getSolution() const;

    /**
     * @brief Returns the exit flag of the QP problem
     * https://osqp.org/docs/interfaces/status_values.html#solver-errors.
     * @return The exit flag of the QP problem.
     */
    const OsqpEigen::ErrorExitFlag getQPProblemExitFlag();

    /**
     * @brief Returns the status of the QP problem
     * https://osqp.org/docs/interfaces/status_values.html#solver-status.
     * @return The status of the QP problem.
     */
    const OsqpEigen::Status getQPProblemStatus();

    /**
     * @brief Returns the number of optimization variables in the QP problem.
     * @return The number of optimization variables in the QP problem.
     */
    const unsigned int getNOptimizationVariables() const;

    /**
     * @brief Returns the number of constraints in the QP problem.
     * @return The number of constraints in the QP problem.
     */
    const unsigned int getNConstraints() const;

    /**
     * @brief Returns the Hessian matrix of the QP problem.
     * @return The Hessian matrix of the QP problem.
     */
    Eigen::Ref<const Eigen::MatrixXd> getHessian() const;

    /**
     * @brief Returns the gradient vector of the QP problem.
     * @return The gradient vector of the QP problem.
     */
    Eigen::Ref<const Eigen::VectorXd> getGradient() const;

    /**
     * @brief Returns the constraint matrix of the QP problem.
     * @return The constraint matrix of the QP problem.
     */
    Eigen::Ref<const Eigen::MatrixXd> getLinearConstraintMatrix() const;

    /**
     * @brief Returns the lower bound vector of the QP problem.
     * @return The lower bound vector of the QP problem.
     */
    Eigen::Ref<const Eigen::VectorXd> getLowerBound() const;

    /**
     * @brief Returns the upper bound vector of the QP problem.
     * @return The upper bound vector of the QP problem.
     */
    Eigen::Ref<const Eigen::VectorXd> getUpperBound() const;

    /**
     * @brief Returns the cost value x^T H x + g^T x of the QP problem.
     * @return The cost value x^T H x + g^T x of the QP problem.
     */
    double getCostValue() const;

    /**
     * @brief Virtual method to define the constraints and costs of the QP problem.
     * @param parametersHandler Reference to the ParametersHandler object (to access configuration
     * parameters).
     * @param qpInput Struct for collecting input parameters to pass to the QPProblem class.
     * @return True if the configuration was successful, false otherwise.
     */
    virtual const bool setCostAndConstraints(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput)
        = 0;

    /**
     * @brief Method to print the matrices of the QP problem.
     * @return None.
     */
    void printMatricesByTask() const;

    unsigned int m_nVar; /**< The number of optimization variables in the QP problem. */
    std::vector<std::unique_ptr<IQPConstraint>> m_vectorConstraints; /**< A vector containing the
                                                                        IQPConstraints objects of
                                                                        the QP problem. */
    std::vector<std::unique_ptr<IQPCost>> m_vectorCosts; /**< A vector containing the IQPCost
                                                            objects of the QP problem. */

    /**
     * @brief Enable the debug mode to log additional information.
     */
    void enableDebugLogMode();

protected:
    /**
     * @brief Solve the QP problem given the QP costs and constraints stored.
     * @return True if the solution was found, false otherwise.
     */
    const bool solve();

private:
    unsigned int m_nConstraints; /**< The number of constraints in the QP problem. */
    unsigned int m_countConstraints; /**< A counter to loop over the number of constraints. */
    Eigen::MatrixXd m_hessian; /**< The Hessian matrix of the QP problem. */
    Eigen::VectorXd m_gradient; /**< The gradient vector of the QP problem. */
    Eigen::MatrixXd m_linearMatrix; /**< The constraint matrix of the QP problem. */
    Eigen::VectorXd m_lowerBound; /**< The lower bound vector of the QP problem. */
    Eigen::VectorXd m_upperBound; /**< The upper bound vector of the QP problem. */

    OsqpEigen::Solver m_solver; /**< The solver of the OSQP problem. */
    Eigen::VectorXd m_outputQP; /**< The solution of the QP problem. */
    double m_costValue; /**< The cost value of the QP problem. */
    OsqpEigen::ErrorExitFlag m_exitFlagQPProblem; /**< The exit flag of the QP problem. */
    OsqpEigen::Status m_statusQPProblem; /**< The status of the QP problem. */
    bool m_debugModeActive; /**< A flag to enable/disable the debug mode. */
    bool m_firstUpdate{true};

    std::shared_ptr<BipedalLocomotion::YarpUtilities::VectorsCollectionServer>
        m_vectorsCollectionServer;
    const bool configureVectorsCollectionServerIQP(QPInput& qpInput);
    const std::string removeNamespace(const std::string& nameTag);
    const std::string convertNumberToStringWithDigits(const int& number, const int& digits);

    Eigen::SparseMatrix<double> m_hessianSparse; /**< The Hessian matrix of the QP problem
                                                (sparse version). */
    Eigen::SparseMatrix<double> m_linearMatrixSparse; /**< The constraint matrix of the QP
                                                         problem (sparse version). */
};

#endif /* end of include guard IMPCROBLEM_H */
