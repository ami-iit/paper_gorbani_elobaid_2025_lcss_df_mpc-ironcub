#include <FlightControlUtils.h>
#include <simpleExample/SimpleExampleconstant.h>
#include <simpleExample/constraintsSimpleExample.h>
namespace SIMPLE_EXAMPLE
{

ConstraintSystemDynamicSimpleExample::ConstraintSystemDynamicSimpleExample(
    const unsigned int nVar,
    const unsigned int nStates,
    const unsigned int nInput,
    const unsigned int nIter)
    : IQPConstraintMPCDynamic(nVar, nStates, nInput, nIter)
{
    m_A.resize(nStates, nStates);
    m_B.resize(nStates, nInput);
    m_A.setZero();
    m_B.setZero();
    m_systemDynamic = std::make_shared<SystemDynamicsSimpleExample>(nStates, nInput);
}

const bool ConstraintSystemDynamicSimpleExample::readConfigParameters(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{
    auto ptr = parametersHandler.lock();
    m_systemDynamic->configure(parametersHandler, qpInput);
    return true;
}

void ConstraintSystemDynamicSimpleExample::configureDynVectorsSize(QPInput& qpInput)
{
    m_vectorsCollectionServer = qpInput.getVectorsCollectionServer();
}

bool ConstraintSystemDynamicSimpleExample::updateDynamicConstraints(QPInput& qpInput)
{
    m_systemDynamic->updateDynamicMatrices(qpInput);
    bool ok = m_systemDynamic->getAMatrix(m_A);
    ok = ok && m_systemDynamic->getBMatrix(m_B);
    ok = ok && m_systemDynamic->getCVector(m_c);
    return ok;
}

const bool ConstraintSystemDynamicSimpleExample::configureVectorsCollectionServer(QPInput& qpInput)
{
    return true;
}

const bool ConstraintSystemDynamicSimpleExample::populateVectorsCollection(
    QPInput& qpInput, const Eigen::Ref<Eigen::VectorXd> qpSolution)
{
    return true;
}

ConstraintInitialStateSimpleExample::ConstraintInitialStateSimpleExample(const unsigned int nStates,
                                                                         const unsigned int nVar)
    : IQPConstraintInitialState(nStates, nVar)
{
}

const bool ConstraintInitialStateSimpleExample::readConfigParameters(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{
    return true;
}

void ConstraintInitialStateSimpleExample::configureDynVectorsSize(QPInput& qpInput)
{
    m_initialState = Eigen::VectorXd::Ones(m_initialState.size());
}

bool ConstraintInitialStateSimpleExample::updateInitialState(QPInput& qpInput)
{
    return true;
}

const bool
ConstraintInitialStateSimpleExample::updateInitialState(const Eigen::Ref<Eigen::VectorXd> xInit)
{
    if (xInit.size() != m_nStates)
    {
        yError() << "ConstraintInitialStateSimpleExample::updateInitialState: the size of the "
                    "input vector is not correct.";
        return false;
    }
    m_initialState = xInit;
    return true;
}

const bool ConstraintInitialStateSimpleExample::configureVectorsCollectionServer(QPInput& qpInput)
{
    return true;
}

const bool ConstraintInitialStateSimpleExample::populateVectorsCollection(
    QPInput& qpInput, const Eigen::Ref<Eigen::VectorXd> qpSolution)
{
    return true;
}

HankelMatrixConstraintSimpleExample::HankelMatrixConstraintSimpleExample(
    const int nVar,
    const int nStates,
    const int horizonLenghtHankleMatrix,
    const int nG,
    const int gInitPosition,
    const std::vector<double>& inputData,
    const std::vector<double>& outputData)
    : IQPConstraint(nVar, 2 * horizonLenghtHankleMatrix)
{
    m_nStates = nStates;
    m_inputData = inputData;
    m_outputData = outputData;
    m_horizonLenghtHankleMatrix = horizonLenghtHankleMatrix;
    m_nG = nG;
    m_initGPosition = gInitPosition;
}

const bool HankelMatrixConstraintSimpleExample::readConfigParameters(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{
    auto ptr = parametersHandler.lock();
    if (!ptr->getParameter("nIter", m_nIter))
    {
        yError() << "Parameter 'nIter' not found in the config file.";
        return false;
    }
    m_initDataLength = m_horizonLenghtHankleMatrix - m_nIter;
    return true;
}

void HankelMatrixConstraintSimpleExample::configureDynVectorsSize(QPInput& qpInput)
{
    m_numCols = m_inputData.size() - m_horizonLenghtHankleMatrix;
    m_hankleMatrixInput.resize(m_horizonLenghtHankleMatrix, m_numCols);
    m_hankleMatrixOutput.resize(m_horizonLenghtHankleMatrix, m_numCols);
    this->updateHankleMatrix();
    m_linearMatrix.setZero();
    m_lowerBound.setZero();
    m_upperBound.setZero();
    m_initialInputValuesVector.resize(m_initDataLength);
    m_initialOutputValuesVector.resize(m_initDataLength);
}

const bool HankelMatrixConstraintSimpleExample::computeConstraintsMatrixAndBounds(QPInput& qpInput)
{
    if (m_firstIteriation)
    {
        m_linearMatrix.block(0, m_initGPosition, m_horizonLenghtHankleMatrix, m_numCols)
            = m_hankleMatrixInput;
        m_linearMatrix.block(m_horizonLenghtHankleMatrix,
                             m_initGPosition,
                             m_horizonLenghtHankleMatrix,
                             m_numCols)
            = m_hankleMatrixOutput;
        // set the constraints for the slack variables
        m_linearMatrix
            .block(m_horizonLenghtHankleMatrix,
                   m_initGPosition + m_nG,
                   m_horizonLenghtHankleMatrix,
                   m_horizonLenghtHankleMatrix)
            .setIdentity();
        for (int j = 0; j < m_nIter; j++)
        {
            m_linearMatrix(m_initDataLength + j,
                           m_nStates * (m_nIter + 1) + U2Idx[0] + j * (N_U_1 + N_Y_2 + N_U_2))
                = -1;
            m_linearMatrix(m_horizonLenghtHankleMatrix + m_initDataLength + j,
                           m_nStates * (m_nIter + 1) + Y2Idx[0] + j * (N_U_1 + N_Y_2 + N_U_2))
                = -1;
        }
        m_firstIteriation = false;
    }

    // update the hankle matrix
    for (int j = 0; j < m_initDataLength; j++)
    {
        m_upperBound[j] = m_initialInputValuesVector[j];
        m_lowerBound[j] = m_initialInputValuesVector[j];
        m_upperBound[m_horizonLenghtHankleMatrix + j] = m_initialOutputValuesVector[j];
        m_lowerBound[m_horizonLenghtHankleMatrix + j] = m_initialOutputValuesVector[j];
    }

    return true;
}

const bool HankelMatrixConstraintSimpleExample::updateInitialInputOutput(double uData, double yData)
{
    if (m_firstUpdateInitialState)
    {
        for (int i = 0; i < m_initDataLength; i++)
        {
            m_initialInputValuesVector[i] = uData;
            m_initialOutputValuesVector[i] = yData;
        }
        m_firstUpdateInitialState = false;
    } else
    {
        // remove the first element of the vectors
        m_initialInputValuesVector.erase(m_initialInputValuesVector.begin());
        m_initialOutputValuesVector.erase(m_initialOutputValuesVector.begin());
        // add the new element at the end of the vectors
        m_initialInputValuesVector.push_back(uData);
        m_initialOutputValuesVector.push_back(yData);
    }
    return true;
}

void HankelMatrixConstraintSimpleExample::updateHankleMatrix()
{
    for (int j = 0; j < m_horizonLenghtHankleMatrix; j++)
    {
        for (int k = 0; k < m_numCols; k++)
        {
            m_hankleMatrixInput(j, k) = m_inputData[k + j];
            m_hankleMatrixOutput(j, k) = m_outputData[k + j];
        }
    }
    // Check if the Hankle matrix is persistently exciting
    if (!isHankleMatrixPersistelyExciting(m_hankleMatrixInput))
    {
        yError() << "Input Hankle matrix is not persistently exciting. Please check the input "
                    "data.";
    }
    if (!isHankleMatrixPersistelyExciting(m_hankleMatrixOutput))
    {
        yError() << "Output Hankle matrix is not persistently exciting. Please check the output "
                    "data.";
    }
}

bool HankelMatrixConstraintSimpleExample::isHankleMatrixPersistelyExciting(
    Eigen::MatrixXd& hankelMatrix, const double tolerance)
{
    int max_possible_rank = std::min(hankelMatrix.rows(), hankelMatrix.cols());
    int order = 10;

    if (order > hankelMatrix.rows())
    {
        std::cerr << "Warning: Requested order (" << order
                  << ") for PE check exceeds Hankel matrix row dimension (" << hankelMatrix.rows()
                  << "). This check might not be meaningful." << std::endl;
        // Depending on interpretation, you might throw or just return false here
    }

    if (tolerance <= 0.0)
    {
        throw std::invalid_argument("Tolerance for PE check must be positive.");
    }

    // Handle empty matrix case
    if (hankelMatrix.rows() == 0 || hankelMatrix.cols() == 0)
    {
        // An empty matrix has rank 0. It's PE only if the required rank is 0.
        return (order == 0);
    }

    // --- Use ColPivHouseholderQR for Rank Computation ---
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(hankelMatrix);

    // Get the absolute values of the diagonal elements of the R factor.
    // The rank is the number of diagonal elements whose magnitude is > tolerance.
    // Eigen's internal representation might store R differently, but diagonal() on matrixQR() gives
    // access. Size of diagonal is min(rows, cols).
    const auto& R_diag_abs = qr.matrixQR().diagonal().cwiseAbs();

    // Count the number of diagonal elements above the *absolute* tolerance

    int computed_rank = 0;
    for (int i = 0; i < R_diag_abs.size(); ++i)
    {
        if (R_diag_abs(i) > tolerance)
        {
            computed_rank++;
        }
    }

    bool isPE = (computed_rank >= order);
    if (!isPE)
    {
        yError() << "Hankel matrix is not persistently exciting. Computed rank: " << computed_rank
                 << ", required rank: " << order;
    }

    // Check if the computed rank meets the required rank 'order'
    return isPE;
}

const bool HankelMatrixConstraintSimpleExample::configureVectorsCollectionServer(QPInput& qpInput)
{
    return true;
}

const bool HankelMatrixConstraintSimpleExample::populateVectorsCollection(
    QPInput& qpInput, const Eigen::Ref<Eigen::VectorXd> qpSolution)
{
    return true;
}

TerminalConstraint::TerminalConstraint(const int nVar,
                                       const int nStates,
                                       const int nIter,
                                       const int artificialEqInit)
    : IQPConstraint(nVar, 6)
{
    m_nStates = nStates;
    m_nIter = nIter;
    m_artificialEqInit = artificialEqInit;
    m_artificialInputInit = artificialEqInit + N_Y_1 + N_Y_2;
}

const bool TerminalConstraint::readConfigParameters(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{
    return true;
}

void TerminalConstraint::configureDynVectorsSize(QPInput& qpInput)
{
    m_lowerBound.setZero();
    m_upperBound.setZero();
    m_linearMatrix.setZero();
}

const bool TerminalConstraint::computeConstraintsMatrixAndBounds(QPInput& qpInput)
{
    m_linearMatrix(0, m_nStates * m_nIter) = 1;
    m_linearMatrix(0, m_artificialEqInit) = -1;
    m_linearMatrix(1, m_nStates * (m_nIter + 1) + (N_U_1 + N_Y_2 + N_U_2) * m_nIter + Y2Idx[0]) = 1;
    m_linearMatrix(1, m_artificialEqInit + 1) = -1;
    m_linearMatrix(2,
                   m_nStates * (m_nIter + 1) + (N_U_1 + N_Y_2 + N_U_2) * (m_nIter - 1) + Y2Idx[0])
        = 1;
    m_linearMatrix(2, m_artificialEqInit + 1) = -1;
    m_linearMatrix(3,
                   m_nStates * (m_nIter + 1) + (N_U_1 + N_Y_2 + N_U_2) * (m_nIter - 1) + U1Idx[0])
        = 1;
    m_linearMatrix(3, m_artificialInputInit) = -1;
    m_linearMatrix(4,
                   m_nStates * (m_nIter + 1) + (N_U_1 + N_Y_2 + N_U_2) * (m_nIter - 1) + U2Idx[0])
        = 1;
    m_linearMatrix(4, m_artificialInputInit + 1) = -1;
    m_linearMatrix(5, m_nStates * (m_nIter + 1) + (N_U_1 + N_Y_2 + N_U_2) * m_nIter + U2Idx[0]) = 1;
    m_linearMatrix(5, m_artificialInputInit + 1) = -1;
    return true;
}

const bool TerminalConstraint::configureVectorsCollectionServer(QPInput& qpInput)
{
    return true;
}

const bool
TerminalConstraint::populateVectorsCollection(QPInput& qpInput,
                                              const Eigen::Ref<Eigen::VectorXd> qpSolution)
{
    return true;
}

InputConstraint::InputConstraint(const int nVar, const int nStates, const int nIter)
    : IQPConstraint(nVar, nIter * (N_U_1 + N_U_2))
{
    m_nStates = nStates;
    m_nIter = nIter;
    m_nInput = N_U_1 + N_Y_2 + N_U_2;
}

const bool InputConstraint::readConfigParameters(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{
    return true;
}

void InputConstraint::configureDynVectorsSize(QPInput& qpInput)
{
    m_lowerBound.setZero();
    m_upperBound.setZero();
    m_linearMatrix.setZero();
}

const bool InputConstraint::computeConstraintsMatrixAndBounds(QPInput& qpInput)
{
    // Set the linear matrix for input constraints
    for (int i = 0; i < m_nIter; ++i)
    {
        m_linearMatrix(i, m_nStates * (m_nIter + 1) + U1Idx[0] + m_nInput * i) = 1; // U1
        m_linearMatrix(i + N_U_1 * m_nIter, m_nStates * (m_nIter + 1) + U2Idx[0] + m_nInput * i)
            = 1; // U2
        m_upperBound(i, 0) = 5;
        m_upperBound(i + N_U_1 * m_nIter, 0) = 5;
        m_lowerBound(i, 0) = -5;
        m_lowerBound(i + N_U_1 * m_nIter, 0) = -5;
    }
    return true;
}

const bool InputConstraint::configureVectorsCollectionServer(QPInput& qpInput)
{
    return true;
}

const bool InputConstraint::populateVectorsCollection(QPInput& qpInput,
                                                      const Eigen::Ref<Eigen::VectorXd> qpSolution)
{
    return true;
}

OutputConstraint::OutputConstraint(const int nVar, const int nStates, const int nIter)
    : IQPConstraint(nVar, nIter * (N_Y_1 + N_Y_2))
{
    m_nStates = nStates;
    m_nIter = nIter;
    m_nInput = N_U_1 + N_Y_2 + N_U_2;
}

const bool OutputConstraint::readConfigParameters(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{
    auto ptr = parametersHandler.lock();
    if (!ptr->getParameter("yMax", m_yMax))
    {
        yError() << "Failed to get parameter yMax";
        return false;
    }
    if (!ptr->getParameter("yMin", m_yMin))
    {
        yError() << "Failed to get parameter yMin";
        return false;
    }
    return true;
}

void OutputConstraint::configureDynVectorsSize(QPInput& qpInput)
{
    m_lowerBound.setZero();
    m_upperBound.setZero();
    m_linearMatrix.setZero();
}

const bool OutputConstraint::computeConstraintsMatrixAndBounds(QPInput& qpInput)
{
    // Set the linear matrix for output constraints
    for (int i = 0; i < m_nIter; ++i)
    {
        m_linearMatrix(i, i * m_nStates) = 1; // Y1
        m_linearMatrix(i + N_Y_1 * m_nIter, m_nStates * (m_nIter + 1) + Y2Idx[0] + m_nInput * i)
            = 1; // Y2
        m_upperBound(i) = m_yMax[0]; // Y1 max
        m_lowerBound(i) = m_yMin[0]; // Y1 min
        m_upperBound(i + N_Y_1 * m_nIter) = m_yMax[1]; // Y2 max
        m_lowerBound(i + N_Y_1 * m_nIter) = m_yMin[1]; // Y2 min
    }
    return true;
}

const bool OutputConstraint::configureVectorsCollectionServer(QPInput& qpInput)
{
    return true;
}

const bool OutputConstraint::populateVectorsCollection(QPInput& qpInput,
                                                       const Eigen::Ref<Eigen::VectorXd> qpSolution)
{
    return true;
}

} // namespace SIMPLE_EXAMPLE
