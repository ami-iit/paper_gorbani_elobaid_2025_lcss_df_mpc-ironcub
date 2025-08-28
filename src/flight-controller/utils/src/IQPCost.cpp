#include "IQPCost.h"

IQPCost::IQPCost(const unsigned int nVar)
{
    if (nVar & (1 << (std::numeric_limits<unsigned int>::digits - 1)))
    {
        yError() << "QPCost::QPCost: The number of variables in the QP problem must be positive. "
                    "Setting it to 0.";
        m_nVar = 0;
    } else
    {
        m_nVar = nVar;
    }
};

void IQPCost::configureSizeHessianAndGradient()
{
    m_hessian = Eigen::MatrixXd::Zero(m_nVar, m_nVar);
    m_gradient = Eigen::VectorXd::Zero(m_nVar);
};

Eigen::Ref<Eigen::MatrixXd> IQPCost::getHessian()
{
    return m_hessian;
};

Eigen::Ref<Eigen::VectorXd> IQPCost::getGradient()
{
    return m_gradient;
};
