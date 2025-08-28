/**
 * @file IQPConstraintMPCDynamic.h
 * @authors Davide Gorbani
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef IQP_CONSTRAINT_MPC_DYNAMIC_H
#define IQP_CONSTRAINT_MPC_DYNAMIC_H

#include "IQPConstraint.h"

class IQPConstraintMPCDynamic : public IQPConstraint
{
private:
    const bool configureSizeDynamicsMatrices();

public:
    IQPConstraintMPCDynamic(const unsigned int nVar,
                            const unsigned int nStates,
                            const unsigned int nInput,
                            const unsigned int nIter);

    virtual ~IQPConstraintMPCDynamic() = default;

    const bool computeConstraintsMatrixAndBounds(QPInput& qpInput) override;

    /**
     * @brief Virtual method to updated the matrices A, B and c of the dynamic constraints such that
     * x_{k+1} = A*x_k + B*u_k + c
     * @param qpInput The QPInput object
     * @return True if the update is successful, false otherwise
     */
    virtual bool updateDynamicConstraints(QPInput& qpInput) = 0;

protected:
    Eigen::MatrixXd m_A;
    Eigen::MatrixXd m_B;
    Eigen::VectorXd m_c;
    double m_nStates;
    double m_nInput;
    double m_nIter;
};

class IQPConstraintInitialState : public IQPConstraint
{
private:
    const bool configureSizeInitialState();

public:
    IQPConstraintInitialState(const unsigned int nStates, const unsigned int nVar);

    virtual ~IQPConstraintInitialState() = default;

    const bool computeConstraintsMatrixAndBounds(QPInput& qpInput) override;
    virtual bool updateInitialState(QPInput& qpInput) = 0;

protected:
    Eigen::VectorXd m_initialState;
    double m_nStates;
};

#endif // IQP_CONSTRAINT_MPC_DYNAMIC_H
