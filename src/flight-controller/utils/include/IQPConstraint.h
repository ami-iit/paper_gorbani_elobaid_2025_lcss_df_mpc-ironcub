#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "QPInput.h"
#include <BipedalLocomotion/YarpUtilities/VectorsCollectionServer.h>
#include <Eigen/Dense>

/**
 * @class IQPConstraint
 * @brief An abstract class to define a constraint for a Quadratic Programming (QP) problem.
 * IQPConstraint provides the methods to configure the constraint matrix and bounds vectors.
 */
class IQPConstraint
{
public:
    /**
     * @brief Constructs an IQPConstraint object with the specified number of variables and
     * constraints.
     * @param nVar The number of optimization variables in the QP problem.
     * @param nConstraints The number of constraint that are introduced by this object.
     */
    IQPConstraint(const unsigned int nVar, const unsigned int nConstraints = 0);

    /**
     * @brief Default destructor.
     */
    virtual ~IQPConstraint() = default;

    /**
     * @brief Resizes the constraint matrix and bounds vectors to accomodate the constraints
     * introduced by this object.
     * @return None.
     */
    void configureSizeConstraintMatrixAndBounds();

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
     * @brief Computes the constraint values for the given robot state.
     * @param qpInput Struct for collecting input parameters to pass to the QPProblem class.
     * @return True if the computation was successful, false otherwise.
     */
    virtual const bool computeConstraintsMatrixAndBounds(QPInput& qpInput) = 0;

    /**
     * @brief Returns the number of constraints in the problem.
     * @return The number of constraints in the problem.
     */
    const unsigned int getNConstraints() const;

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
     * @brief Returns the constraint matrix.
     * @return The constraint matrix.
     */
    Eigen::Ref<Eigen::MatrixXd> getLinearConstraintMatrix();

    /**
     * @brief Returns the lower bound vector.
     * @return The lower bound vector.
     */
    Eigen::Ref<Eigen::VectorXd> getLowerBound();

    /**
     * @brief Returns the upper bound vector.
     * @return The upper bound vector.
     */
    Eigen::Ref<Eigen::VectorXd> getUpperBound();

protected:
    unsigned int m_nConstraints; /**< The number of constraints in the problem. */
    unsigned int m_nVar; /**< The number of variables in the QP problem. */
    Eigen::MatrixXd m_linearMatrix; /**< The constraint matrix. */
    Eigen::VectorXd m_lowerBound; /**< The lower bound vector. */
    Eigen::VectorXd m_upperBound; /**< The upper bound vector. */
};

#endif
