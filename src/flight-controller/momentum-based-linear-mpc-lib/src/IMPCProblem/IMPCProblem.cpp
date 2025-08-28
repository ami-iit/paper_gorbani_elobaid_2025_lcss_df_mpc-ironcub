#include <IMPCProblem/IMPCProblem.h>

const bool IMPCProblem::configure(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{

    m_debugModeActive = false;

    if (!setCostAndConstraints(parametersHandler, qpInput))
    {
        yError() << "QPProblem::configure: error in setting the list of costs and constraints";
        return false;
    }
    for (auto& cost : m_vectorCosts)
    {
        if (!cost->readConfigParameters(parametersHandler, qpInput))
        {
            yError() << "QPProblem::configure: error in reading the configuration parameters of "
                     << boost::core::demangle(typeid(*cost).name());
            return false;
        }
    }
    for (auto& constraint : m_vectorConstraints)
    {
        if (!constraint->readConfigParameters(parametersHandler, qpInput))
        {
            yError() << "QPProblem::configure: error in reading the configuration parameters of "
                     << boost::core::demangle(typeid(*constraint).name());
            return false;
        }
    }

    // resize matrices
    m_hessian = Eigen::MatrixXd::Zero(m_nVar, m_nVar);
    m_gradient = Eigen::VectorXd::Zero(m_nVar);
    m_linearMatrix.resize(0, m_nVar);
    m_lowerBound.resize(0);
    m_upperBound.resize(0);

    m_hessian.setZero();
    m_gradient.setZero();
    m_linearMatrix.setZero();
    m_lowerBound.setZero();
    m_upperBound.setZero();

    // configure costs
    for (auto& cost : m_vectorCosts)
    {
        cost->configureDynVectorsSize(qpInput);
        cost->configureSizeHessianAndGradient();
    }
    for (auto& cost : m_vectorCosts)
    {
        if (!cost->computeHessianAndGradient(qpInput))
        {
            yError() << "QPProblem::configure: error in cost computation of "
                     << boost::core::demangle(typeid(*cost).name());
            return false;
        }
        yDebug() << "QPProblem::configure: computation of "
                 << boost::core::demangle(typeid(*cost).name()) << " done";

        if (cost->getHessian().rows() != m_nVar || cost->getHessian().cols() != m_nVar)
        {
            yError() << "QPProblem::configure: error in cost computation of "
                     << boost::core::demangle(typeid(*cost).name())
                     << " Hessian matrix has wrong size!";
            return false;
        }
        m_hessian += cost->getHessian();
        if (cost->getGradient().rows() != m_nVar)
        {
            yError() << "QPProblem::configure: error in cost computation of "
                     << boost::core::demangle(typeid(*cost).name())
                     << " gradient vector has wrong size!";
            return false;
        }
        m_gradient += cost->getGradient();
    }

    // configure constraints
    m_countConstraints = 0;
    for (auto& constraint : m_vectorConstraints)
    {
        constraint->configureDynVectorsSize(qpInput);
        constraint->configureSizeConstraintMatrixAndBounds();
        m_countConstraints += constraint->getNConstraints();
    }
    m_linearMatrix = Eigen::MatrixXd::Zero(m_countConstraints, m_nVar);
    m_lowerBound = Eigen::VectorXd::Zero(m_countConstraints);
    m_upperBound = Eigen::VectorXd::Zero(m_countConstraints);
    m_countConstraints = 0;
    for (auto& constraint : m_vectorConstraints)
    {
        if (!constraint->computeConstraintsMatrixAndBounds(qpInput))
        {
            yError() << "QPProblem::configure: error in constraint computation of "
                     << boost::core::demangle(typeid(*constraint).name());
            return false;
        }
        yDebug() << "QPProblem::configure: computation of "
                 << boost::core::demangle(typeid(*constraint).name()) << " done";

        if (constraint->getLinearConstraintMatrix().rows() != constraint->getNConstraints()
            || constraint->getLinearConstraintMatrix().cols() != m_nVar)
        {
            yError() << "QPProblem::configure: error in constraint computation of "
                     << boost::core::demangle(typeid(*constraint).name())
                     << " linear matrix has wrong size!";
            return false;
        }
        m_linearMatrix.block(m_countConstraints, 0, constraint->getNConstraints(), m_nVar)
            = constraint->getLinearConstraintMatrix();
        if (constraint->getLowerBound().rows() != constraint->getNConstraints())
        {
            yError() << "QPProblem::configure: error in constraint computation of "
                     << boost::core::demangle(typeid(*constraint).name())
                     << " lower bound vector has wrong size!";
            return false;
        }
        m_lowerBound.segment(m_countConstraints, constraint->getNConstraints())
            = constraint->getLowerBound();
        if (constraint->getUpperBound().rows() != constraint->getNConstraints())
        {
            yError() << "QPProblem::configure: error in constraint computation of "
                     << boost::core::demangle(typeid(*constraint).name())
                     << " upper bound vector has wrong size!";
            return false;
        }
        m_upperBound.segment(m_countConstraints, constraint->getNConstraints())
            = constraint->getUpperBound();
        m_countConstraints += constraint->getNConstraints();
    }

    // configure vectors collection server
    this->configureVectorsCollectionServerIQP(qpInput);

    m_nConstraints = m_countConstraints;
    m_solver.settings()->setWarmStart(true);
    m_solver.settings()->setVerbosity(false);
    m_solver.data()->setNumberOfVariables(m_nVar);
    m_solver.data()->setNumberOfConstraints(m_nConstraints);
    m_outputQP = Eigen::VectorXd::Zero(m_nVar);
    m_solver.settings()->setPolish(true);

    return true;
}

const bool IMPCProblem::update(QPInput& qpInput)
{
    if (m_firstUpdate)
    {
        m_hessian.setZero();
    }
    m_gradient.setZero();
    for (auto& cost : m_vectorCosts)
    {
        if (!cost->computeHessianAndGradient(qpInput))
        {
            const auto& costRef = *cost; // Dereference the pointer first
            std::string costTypeName = boost::core::demangle(typeid(costRef).name());
            yError() << "QPProblem::update: error in cost computation of " << costTypeName;
            return false;
        }
        if (m_firstUpdate)
        {
            m_hessian += cost->getHessian();
        }
        m_gradient += cost->getGradient();
    }
    if (m_firstUpdate)
    {
        m_firstUpdate = false;
    }
    m_countConstraints = 0;
    for (auto& constraint : m_vectorConstraints)
    {
        if (!constraint->computeConstraintsMatrixAndBounds(qpInput))
        {
            yError() << "QPProblem::update: error in constraint computation of "
                     << boost::core::demangle(typeid(*constraint).name());
            return false;
        }
        m_linearMatrix.block(m_countConstraints, 0, constraint->getNConstraints(), m_nVar)
            = constraint->getLinearConstraintMatrix();
        m_lowerBound.segment(m_countConstraints, constraint->getNConstraints())
            = constraint->getLowerBound();
        m_upperBound.segment(m_countConstraints, constraint->getNConstraints())
            = constraint->getUpperBound();
        m_countConstraints += constraint->getNConstraints();
    }
    return true;
}

const bool IMPCProblem::solve()
{
    m_exitFlagQPProblem
        = OsqpEigen::ErrorExitFlag::DataValidationError; /** Initialize
                                                            m_exitFlagQPProblem with
                                                            an error to prevent
                                                            m_exitFlagQPProblem from
                                                            being equal to
                                                            OsqpEigen::ErrorExitFlag::NoError
                                                            if code fails before
                                                            m_exitFlagQPProblem =
                                                            m_solver.solveProblem().
                                                          */

    // Convert matrices to sparse
    m_linearMatrixSparse = m_linearMatrix.sparseView();
    // m_hessianSparse = m_hessian.sparseView();

    // Debugging
    // std::cout << "Hessian: " << m_hessian << std::endl;
    // std::cout << "Gradient: " << m_gradient << std::endl;
    // std::cout << "Linear Matrix: " << m_linearMatrix << std::endl;
    // std::cout << "Lower Bound: " << m_lowerBound << std::endl;
    // std::cout << "Upper Bound: " << m_upperBound << std::endl;

    if (!m_solver.isInitialized())
    {
        m_hessianSparse = m_hessian.sparseView();
        yInfo() << "QPProblem::solve : initialising solver";
        if (!m_solver.data()->setHessianMatrix(m_hessianSparse))
        {
            yError() << "QPProblem::solve : error in setting hessian matrix";
            return false;
        }
        if (!m_solver.data()->setGradient(m_gradient))
        {
            yError() << "QPProblem::solve : error in setting gradient";
            return false;
        }
        if (!m_solver.data()->setLinearConstraintsMatrix(m_linearMatrixSparse))
        {
            yError() << "QPProblem::solve : error in setting linear matrix";
            return false;
        }
        if (!m_solver.data()->setLowerBound(m_lowerBound))
        {
            yError() << "QPProblem::solve : error in setting lower bound";
            return false;
        }
        if (!m_solver.data()->setUpperBound(m_upperBound))
        {
            yError() << "QPProblem::solve : error in setting upper bound";
            return false;
        }
        if (!m_solver.initSolver())
        {
            yError() << "QPProblem::solve : error in initializing solver";
            printMatricesByTask();
            return false;
        }
    } else
    {
        // if (!m_solver.updateHessianMatrix(m_hessianSparse))
        // {
        //     yError() << "QPProblem::solve : error in updating hessian matrix";
        //     return false;
        // }
        if (!m_solver.updateGradient(m_gradient))
        {
            yError() << "QPProblem::solve : error in updating gradient";
            return false;
        }
        if (!m_solver.updateLinearConstraintsMatrix(m_linearMatrixSparse))
        {
            yError() << "QPProblem::solve : error in updating linear matrix";
            return false;
        }
        if (!m_solver.updateBounds(m_lowerBound, m_upperBound))
        {
            yError() << "QPProblem::solve : error in updating bounds";
            return false;
        }
    }
    m_exitFlagQPProblem = m_solver.solveProblem();
    if (m_exitFlagQPProblem != OsqpEigen::ErrorExitFlag::NoError)
    {
        yError() << "QPProblem::solve : error in solving problem";
        return false;
    }
    m_statusQPProblem = m_solver.getStatus();
    if (m_statusQPProblem != OsqpEigen::Status::Solved
        && m_statusQPProblem != OsqpEigen::Status::SolvedInaccurate)
    {
        yWarning() << "QPProblem::solve : osqp was not able to find a feasible solution";
    }
    if (m_statusQPProblem == OsqpEigen::Status::SolvedInaccurate)
    {
        yWarning() << "QPProblem::solve : osqp found an inaccurate feasible solution.";
    }

    m_outputQP = m_solver.getSolution();
    return true;
}

const bool IMPCProblem::configureVectorsCollectionServerIQP(QPInput& qpInput)
{
    std::string nameTag;
    m_vectorsCollectionServer = qpInput.getVectorsCollectionServer();
    {
        if (m_debugModeActive)
        {
            m_vectorsCollectionServer->populateMetadata(nameTag + "::cost", {});
            m_vectorsCollectionServer->populateMetadata(nameTag + "::hessianDiagonal", {});
        }
    }
    for (auto& cost : m_vectorCosts)
    {
        if (m_debugModeActive)
        {
            nameTag = removeNamespace(boost::core::demangle(typeid(*cost).name()));
            m_vectorsCollectionServer->populateMetadata(nameTag + "::xHx_gx", {});
        }
        cost->configureVectorsCollectionServer(qpInput);
    }
    for (auto& constraint : m_vectorConstraints)
    {
        if (m_debugModeActive)
        {
            nameTag = removeNamespace(boost::core::demangle(typeid(*constraint).name()));
            for (unsigned int i = 0; i < constraint->getNConstraints(); i++)
            {
                m_vectorsCollectionServer->populateMetadata(
                    nameTag + "::constraint_"
                        + convertNumberToStringWithDigits(i,
                                                          std::to_string(
                                                              constraint->getNConstraints() - 1)
                                                              .length()),
                    {"lb", "Ax", "ub"});
            }
        }
        constraint->configureVectorsCollectionServer(qpInput);
    }
    yInfo() << "QPProblem::configureVectorsCollectionServerIQP : configuration vectors collection "
               "server done!";
    return true;
}

const bool IMPCProblem::sendVectorsCollectionToLog(QPInput& qpInput)
{
    std::string nameTag;
    m_vectorsCollectionServer = qpInput.getVectorsCollectionServer();
    {
        if (m_debugModeActive)
        {
            m_vectorsCollectionServer->populateData(nameTag + "::cost",
                                                    std::vector<double>{m_costValue});
            m_vectorsCollectionServer->populateData(nameTag + "::hessianDiagonal",
                                                    m_hessian.diagonal());
        }
    }
    double costValue;
    Eigen::VectorXd Ax;
    std::vector<double> lbAxUb;
    lbAxUb.resize(3);
    for (auto& cost : m_vectorCosts)
    {
        if (m_debugModeActive)
        {
            nameTag = removeNamespace(boost::core::demangle(typeid(*cost).name()));
            costValue = 0.5 * m_outputQP.transpose() * cost->getHessian() * m_outputQP;
            costValue += m_outputQP.transpose() * cost->getGradient();
            if (m_statusQPProblem != OsqpEigen::Status::Solved
                && m_statusQPProblem != OsqpEigen::Status::SolvedInaccurate)
            {
                costValue = 0.0;
            }
            m_vectorsCollectionServer->populateData(nameTag + "::xHx_gx",
                                                    std::vector<double>{costValue});
        }
        cost->populateVectorsCollection(qpInput, m_outputQP);
    }
    for (auto& constraint : m_vectorConstraints)
    {
        if (m_debugModeActive)
        {
            nameTag = removeNamespace(boost::core::demangle(typeid(*constraint).name()));
            Ax.resize(constraint->getNConstraints());
            Ax = constraint->getLinearConstraintMatrix() * m_outputQP;
            for (unsigned int i = 0; i < constraint->getNConstraints(); i++)
            {
                lbAxUb[0] = constraint->getLowerBound()(i);
                lbAxUb[1] = Ax(i);
                lbAxUb[2] = constraint->getUpperBound()(i);
                if (m_statusQPProblem != OsqpEigen::Status::Solved
                    && m_statusQPProblem != OsqpEigen::Status::SolvedInaccurate)
                {
                    lbAxUb[1] = 0.0;
                }
                m_vectorsCollectionServer
                    ->populateData(nameTag + "::constraint_"
                                       + convertNumberToStringWithDigits(i,
                                                                         std::to_string(
                                                                             constraint
                                                                                 ->getNConstraints()
                                                                             - 1)
                                                                             .length()),
                                   lbAxUb);
            }
        }
        constraint->populateVectorsCollection(qpInput, m_outputQP);
    }
    return true;
}

const std::string IMPCProblem::removeNamespace(const std::string& nameTag)
{
    std::string nameTagCopy = nameTag;
    size_t pos = nameTagCopy.find("::");
    if (pos != std::string::npos)
    {
        nameTagCopy.erase(0, pos + 2);
    }
    return nameTagCopy;
}

const std::string IMPCProblem::convertNumberToStringWithDigits(const int& number, const int& digits)
{
    std::string result = std::to_string(number);
    int numDigits = result.length();
    if (numDigits < digits)
    {
        result = std::string(digits - numDigits, '0') + result;
    }
    return result;
}

void IMPCProblem::enableDebugLogMode()
{
    m_debugModeActive = true;
    yWarning() << "QPProblem::enableDebugLogMode : debug mode enabled";
}

Eigen::Ref<const Eigen::VectorXd> IMPCProblem::getSolution() const
{
    return m_outputQP;
}

double IMPCProblem::getCostValue() const
{
    return m_costValue;
}

const OsqpEigen::ErrorExitFlag IMPCProblem::getQPProblemExitFlag()
{
    return m_exitFlagQPProblem;
}

const OsqpEigen::Status IMPCProblem::getQPProblemStatus()
{
    return m_statusQPProblem;
}

const unsigned int IMPCProblem::getNOptimizationVariables() const
{
    return m_nVar;
}

const unsigned int IMPCProblem::getNConstraints() const
{
    return m_nConstraints;
}

Eigen::Ref<const Eigen::MatrixXd> IMPCProblem::getHessian() const
{
    return m_hessian;
}

Eigen::Ref<const Eigen::VectorXd> IMPCProblem::getGradient() const
{
    return m_gradient;
}

Eigen::Ref<const Eigen::MatrixXd> IMPCProblem::getLinearConstraintMatrix() const
{
    return m_linearMatrix;
}

Eigen::Ref<const Eigen::VectorXd> IMPCProblem::getLowerBound() const
{
    return m_lowerBound;
}

Eigen::Ref<const Eigen::VectorXd> IMPCProblem::getUpperBound() const
{
    return m_upperBound;
}

void IMPCProblem::printMatricesByTask() const
{
    for (auto& cost : m_vectorCosts)
    {
        std::cout << "========= " << boost::core::demangle(typeid(*cost).name())
                  << " =========" << std::endl;
        std::cout << "Hessian:\n" << cost->getHessian() << std::endl;
        std::cout << "Gradient:\n" << cost->getGradient() << std::endl;
        std::cout << "=============================\n" << std::endl;
    }

    for (auto& constraint : m_vectorConstraints)
    {
        std::cout << "========= " << boost::core::demangle(typeid(*constraint).name())
                  << " =========" << std::endl;
        std::cout << "Linear Matrix:\n" << constraint->getLinearConstraintMatrix() << std::endl;
        std::cout << "Lower Bound:\n" << constraint->getLowerBound() << std::endl;
        std::cout << "Upper Bound:\n" << constraint->getUpperBound() << std::endl;
        std::cout << "=============================\n" << std::endl;
    }

    std::cout << "=========== Recap ===========" << std::endl;
    std::cout << "N Optimization Variables : " << m_nVar << std::endl;
    std::cout << "N Constraints : " << m_nConstraints << std::endl;
    std::cout << "Hessian:\n" << m_hessian << std::endl;
    std::cout << "Gradient:\n" << m_gradient << std::endl;
    std::cout << "Linear Matrix:\n" << m_linearMatrix << std::endl;
    std::cout << "Lower Bound:\n" << m_lowerBound << std::endl;
    std::cout << "Upper Bound:\n" << m_upperBound << std::endl;
    if (m_exitFlagQPProblem == OsqpEigen::ErrorExitFlag::NoError)
    {
        std::cout << "Solution:\n" << m_outputQP << std::endl;
    }
    std::cout << "=============================\n" << std::endl;
}
