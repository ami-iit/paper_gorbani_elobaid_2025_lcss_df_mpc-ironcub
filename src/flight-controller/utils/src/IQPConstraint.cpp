#include "IQPConstraint.h"

IQPConstraint::IQPConstraint(const unsigned int nVar, const unsigned int nConstraints)
{
    if (nVar & (1 << (std::numeric_limits<unsigned int>::digits - 1)))
    {
        yError() << "QPConstraint::QPConstraint: The number of optimization variables in the QP "
                    "problem must be positive. Setting it to 0.";
        m_nVar = 0;
    } else
    {
        m_nVar = nVar;
    }
    if (nConstraints & (1 << (std::numeric_limits<unsigned int>::digits - 1)))
    {
        yError() << "QPConstraint::QPConstraint: The number of constraints in the QP problem must "
                    "be positive. Setting it to 0.";
        m_nConstraints = 0;
    } else
    {
        m_nConstraints = nConstraints;
    }
};

const unsigned int IQPConstraint::getNConstraints() const
{
    return m_nConstraints;
};

void IQPConstraint::configureSizeConstraintMatrixAndBounds()
{
    m_linearMatrix = Eigen::MatrixXd::Zero(m_nConstraints, m_nVar);
    m_lowerBound = Eigen::VectorXd::Zero(m_nConstraints);
    m_upperBound = Eigen::VectorXd::Zero(m_nConstraints);
};

Eigen::Ref<Eigen::MatrixXd> IQPConstraint::getLinearConstraintMatrix()
{
    return m_linearMatrix;
};

Eigen::Ref<Eigen::VectorXd> IQPConstraint::getLowerBound()
{
    return m_lowerBound;
};

Eigen::Ref<Eigen::VectorXd> IQPConstraint::getUpperBound()
{
    return m_upperBound;
};
