#include "QPInput.h"

const std::shared_ptr<Robot> QPInput::getRobot() const
{
    if (m_robot == nullptr)
    {
        yError() << "QPInput::getRobot : Robot pointer is null";
    }
    return m_robot;
}

const std::shared_ptr<Robot> QPInput::getRobotReference() const
{
    if (m_robotReference == nullptr)
    {
        yError() << "QPInput::getRobotReference : Robot pointer is null";
    }
    return m_robotReference;
}

void QPInput::setRobot(std::shared_ptr<Robot> robot)
{
    this->m_robot = robot;
    m_outputQPthrust.resize(robot->getNJets());
    m_outputQPthrustDot.resize(robot->getNJets());
    m_thrustReference.resize(robot->getNJets());
    m_outputQPJointsPosition.resize(robot->getNJoints());
    m_outputQPJointsVelocity.resize(robot->getNJoints());
    m_jointPosReference.resize(robot->getNJoints());
    yWarning() << "QPInput::setRobot : Robot pointer is set and robot variables are resized ";
}

void QPInput::setRobotReference(std::shared_ptr<Robot> robotReference)
{
    this->m_robotReference = robotReference;
}

const std::shared_ptr<BipedalLocomotion::YarpUtilities::VectorsCollectionServer>
QPInput::getVectorsCollectionServer() const
{
    if (m_vectorsCollectionServer == nullptr)
    {
        yError() << "QPInput::getVectorsCollectionServer : VectorsCollectionServer pointer "
                    "is null";
    }
    return m_vectorsCollectionServer;
}

void QPInput::setVectorsCollectionServer(
    std::shared_ptr<BipedalLocomotion::YarpUtilities::VectorsCollectionServer>
        vectorsCollectionServer)
{
    this->m_vectorsCollectionServer = vectorsCollectionServer;
}

void QPInput::setOutputQPThrust(const Eigen::Ref<const Eigen::VectorXd> outputQPThrust)
{
    if (m_robot == nullptr)
    {
        yError() << "QPInput::setOutputQPThrust : Robot pointer is null; please set "
                    "the robot pointer first";
    }
    if (outputQPThrust.size() != m_robot->getNJets())
    {
        yError() << "QPInput::setOutputQPThrust : outputQPThrust size is different from "
                    "the number of jets";
    }
    std::lock_guard<std::mutex> lock(m_outputQPthrustMutex);
    this->m_outputQPthrust = outputQPThrust;
}

const Eigen::Ref<const Eigen::VectorXd> QPInput::getOutputQPThrust()
{
    std::lock_guard<std::mutex> guard(m_outputQPthrustMutex);
    return m_outputQPthrust;
}

void QPInput::setOutputQPThrustDot(const Eigen::Ref<const Eigen::VectorXd> outputQPThrustDot)
{
    if (m_robot == nullptr)
    {
        yError() << "QPInput::setOutputQPThrustDot : Robot pointer is null; please set "
                    "the robot pointer first";
    }
    if (outputQPThrustDot.size() != m_robot->getNJets())
    {
        yError() << "QPInput::setOutputQPThrustDot : outputQPThrustDot size is different "
                    "from the number of jets";
    }
    this->m_outputQPthrustDot = outputQPThrustDot;
}

const Eigen::Ref<const Eigen::VectorXd> QPInput::getOutputQPThrustDot() const
{
    return m_outputQPthrustDot;
}

const Eigen::Ref<const Eigen::Vector3d> QPInput::getPosCoMReference()
{
    return m_posCoMReference;
}

void QPInput::setPosCoMReference(const Eigen::Ref<const Eigen::Vector3d> posCoMReference)
{
    m_posCoMReference = posCoMReference;
}

const Eigen::Ref<const Eigen::Vector3d> QPInput::getVelCoMReference() const
{
    return m_velCoMReference;
}

void QPInput::setVelCoMReference(const Eigen::Ref<const Eigen::Vector3d> velCoMReference)
{
    m_velCoMReference = velCoMReference;
}

const Eigen::Ref<const Eigen::Vector3d> QPInput::getRPYReference() const
{
    return m_RPYReference;
}

void QPInput::setRPYReference(const Eigen::Ref<const Eigen::Vector3d> RPYReference)
{
    m_RPYReference = RPYReference;
}

const Eigen::Ref<const Eigen::Vector3d> QPInput::getRPYDotReference() const
{
    return m_RPYDotReference;
}

void QPInput::setRPYDotReference(const Eigen::Ref<const Eigen::Vector3d> RPYDotReference)
{
    m_RPYDotReference = RPYDotReference;
}

void QPInput::setThrustJetControl(const Eigen::Ref<const Eigen::VectorXd> thrustJetControl)
{
    if (m_robot == nullptr)
    {
        yError() << "QPInput::setThrustJetControl : Robot pointer is null; please set the "
                    "robot pointer first";
    }
    if (thrustJetControl.size() != m_robot->getNJets())
    {
        yError() << "QPInput::setThrustJetControl : thrustJetControl size is different from "
                    "the number of jets";
    }
    this->m_thrustJetControl = thrustJetControl;
}

const Eigen::Ref<const Eigen::VectorXd> QPInput::getThrustJetControl() const
{
    return m_thrustJetControl;
}

void QPInput::setOutputQPSolution(const Eigen::Ref<const Eigen::VectorXd> outputQPSolution)
{
    this->m_outputQPSolution = outputQPSolution;
}

const Eigen::Ref<const Eigen::VectorXd> QPInput::getOutputQPSolution() const
{
    return m_outputQPSolution;
}

void QPInput::setThrustReference(const Eigen::Ref<const Eigen::VectorXd> thrustReference)
{
    if (m_robot == nullptr)
    {
        yError() << "QPInput::setThrustReference : Robot pointer is null; please set the "
                    "robot pointer first";
    }
    if (thrustReference.size() != m_robot->getNJets())
    {
        yError() << "QPInput::setThrustReference : thrustReference size is different from "
                    "the number of jets";
    }
    this->m_thrustReference = thrustReference;
}

const Eigen::Ref<const Eigen::VectorXd> QPInput::getThrustReference() const
{
    return m_thrustReference;
}

void QPInput::setOutputQPJointsVelocity(
    const Eigen::Ref<const Eigen::VectorXd> outputQPJointsVelocity)
{
    if (m_robot == nullptr)
    {
        yError() << "QPInput::setOutputQPJointsVelocity : Robot pointer is null; please "
                    "set the robot pointer first";
    }
    if (outputQPJointsVelocity.size() != m_robot->getNJoints())
    {
        yError() << "QPInput::setOutputQPJointsVelocity : outputQPJointsVelocity size is "
                    "different from the number of joints";
    }
    this->m_outputQPJointsVelocity = outputQPJointsVelocity;
}

const Eigen::Ref<const Eigen::VectorXd> QPInput::getOutputQPJointsVelocity() const
{
    return m_outputQPJointsVelocity;
}

void QPInput::setOutputQPJointsPosition(
    const Eigen::Ref<const Eigen::VectorXd> outputQPJointsPosition)
{
    if (m_robot == nullptr)
    {
        yError() << "QPInput::setOutputQPJointsPosition : Robot pointer is null; please "
                    "set the robot pointer first";
    }
    if (outputQPJointsPosition.size() != m_robot->getNJoints())
    {
        yError() << "QPInput::setOutputQPJointsPosition : outputQPJointsPosition size is "
                    "different from the number of joints";
    }
    this->m_outputQPJointsPosition = outputQPJointsPosition;
}

const Eigen::Ref<const Eigen::VectorXd> QPInput::getOutputQPJointsPosition() const
{
    return m_outputQPJointsPosition;
}

void QPInput::setJointPosReference(const Eigen::Ref<const Eigen::VectorXd> jointPosReference)
{
    if (m_robot == nullptr)
    {
        yError() << "QPInput::setJointPosReference : Robot pointer is null; please set the "
                    "robot pointer first";
    }
    if (jointPosReference.size() != m_robot->getNJoints())
    {
        yError() << "QPInput::setJointPosReference : jointPosReference size is different "
                    "from the number of joints";
    }
    this->m_jointPosReference = jointPosReference;
}

const Eigen::Ref<const Eigen::VectorXd> QPInput::getJointPosReference() const
{
    return m_jointPosReference;
}

const std::vector<Eigen::Vector6d>& QPInput::getContactWrenchesReference() const
{
    return m_contactWrenchesReference;
}

void QPInput::setContactWrenchesReference(
    const std::vector<Eigen::Vector6d>& contactWrenchesReference)
{
    this->m_contactWrenchesReference = contactWrenchesReference;
}

const std::vector<std::string>& QPInput::getContactWrenchesFrameName() const
{
    return m_contactWrenchesFrameName;
}

void QPInput::setContactWrenchesFrameName(const std::vector<std::string>& contactWrenchesFrameName)
{
    this->m_contactWrenchesFrameName = contactWrenchesFrameName;
}

void QPInput::setSumExternalWrenchesInertialFrame(
    const Eigen::Ref<const Eigen::Vector6d> sumExternalWrenchesInertialFrame)
{
    this->m_sumExternalWrenchesInertialFrame = sumExternalWrenchesInertialFrame;
}

const Eigen::Ref<const Eigen::Vector6d> QPInput::getSumExternalWrenchesInertialFrame() const
{
    return m_sumExternalWrenchesInertialFrame;
}

const double QPInput::getFlightControllerPeriod() const
{
    if (m_flightControllerPeriod <= 0.0)
    {
        throw yError() << "QPInput::getFlightControllerPeriod : Flight controller period is "
                          "not > 0. Did you set it?";
    }
    return m_flightControllerPeriod;
}

void QPInput::setFlightControllerPeriod(const double flightControllerPeriod)
{
    m_flightControllerPeriod = flightControllerPeriod;
}

const Eigen::Vector6d& QPInput::getMomentumReference() const
{
    return m_momentumReference;
}

void QPInput::setMomentumReference(const Eigen::Vector6d& momentumReference)
{
    m_momentumReference = momentumReference;
}

const Eigen::Vector6d& QPInput::getMomentumDotReference() const
{
    return m_momentumDotReference;
}

void QPInput::setMomentumDotReference(const Eigen::Vector6d& momentumDotReference)
{
    m_momentumDotReference = momentumDotReference;
}

const Eigen::Vector6d& QPInput::getMomentumDotDotReference() const
{
    return m_momentumDotDotReference;
}

void QPInput::setMomentumDotDotReference(const Eigen::Vector6d& momentumDotDotReference)
{
    m_momentumDotDotReference = momentumDotDotReference;
}

const iDynTree::Rotation& QPInput::getBaseRotReference() const
{
    return m_baseRotReference;
}

void QPInput::setBaseRotReference(const iDynTree::Rotation& baseRotReference)
{
    m_baseRotReference = baseRotReference;
}

void QPInput::setFreezeAlphaGravity(const bool freezeAlphaGravity)
{
    this->m_freezeAlphaGravity = freezeAlphaGravity;
}

const bool QPInput::getFreezeAlphaGravity() const
{
    return m_freezeAlphaGravity;
}

void QPInput::setAlphaGravity(const double alphaGravity)
{
    this->m_alphaGravity = alphaGravity;
}

const double QPInput::getAlphaGravity() const
{
    return m_alphaGravity;
}

const Eigen::Ref<const Eigen::VectorXd> QPInput::getThrustDesMPC() const
{
    return m_thrustDesMPC;
}

void QPInput::setThrustDesMPC(const Eigen::Ref<const Eigen::VectorXd> thrustDesMPC)
{
    if (m_robot == nullptr)
    {
        yError() << "QPInput::setThrustDesMPC : Robot pointer is null; please set the "
                    "robot pointer first";
    }
    if (thrustDesMPC.size() != m_robot->getNJets())
    {
        yError() << "QPInput::setThrustDesMPC : thrustDesMPC size is different from the "
                    "number of jets";
    }
    this->m_thrustDesMPC = thrustDesMPC;
}

const Eigen::Ref<const Eigen::VectorXd> QPInput::getThrustDotDesMPC() const
{
    return m_thrustDotDesMPC;
}

void QPInput::setThrustDotDesMPC(const Eigen::Ref<const Eigen::VectorXd> thrustDotDesMPC)
{
    if (m_robot == nullptr)
    {
        yError() << "QPInput::setThrustDotDesMPC : Robot pointer is null; please set the "
                    "robot pointer first";
    }
    if (thrustDotDesMPC.size() != m_robot->getNJets())
    {
        yError() << "QPInput::setThrustDotDesMPC : thrustDotDesMPC size is different from "
                    "the number of jets";
    }
    this->m_thrustDotDesMPC = thrustDotDesMPC;
}

const Eigen::Ref<const Eigen::VectorXd> QPInput::getEstimatedThrustDot() const
{
    return m_estimatedThrustDot;
}

void QPInput::setEstimatedThrustDot(const Eigen::Ref<const Eigen::VectorXd> estimatedThrustDot)
{
    if (m_robot == nullptr)
    {
        yError() << "QPInput::setEstimatedThrustDot : Robot pointer is null; please set the "
                    "robot pointer first";
    }
    if (estimatedThrustDot.size() != m_robot->getNJets())
    {
        yError() << "QPInput::setEstimatedThrustDot : estimatedThrustDot size is different "
                    "from the number of jets";
    }
    this->m_estimatedThrustDot = estimatedThrustDot;
}

const Eigen::Ref<const Eigen::VectorXd> QPInput::getThrottleMPC() const
{
    return m_throttleMPC;
}

void QPInput::setThrottleMPC(const Eigen::Ref<const Eigen::VectorXd> throttleMPC)
{
    if (m_robot == nullptr)
    {
        yError() << "QPInput::setThrottleMPC : Robot pointer is null; please set the robot "
                    "pointer first";
    }
    if (throttleMPC.size() != m_robot->getNJets())
    {
        yError() << "QPInput::setThrottleMPC : throttleMPC size is different from the number "
                    "of jets";
    }
    this->m_throttleMPC = throttleMPC;
}

void QPInput::setJetModel(std::shared_ptr<JetModel> jetModel)
{
    this->m_jetModel = jetModel;
}

const std::shared_ptr<JetModel> QPInput::getJetModel() const
{
    if (m_jetModel == nullptr)
    {
        yError() << "QPInput::getJetModel : JetModel pointer is null";
    }
    return m_jetModel;
}
