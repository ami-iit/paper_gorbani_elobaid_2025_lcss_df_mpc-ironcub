#include "Robot.h"
#include "FlightControlUtils.h"

bool Robot::configure(
    const std::string& modelFullPath,
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    const std::vector<std::string>& refFrameExtWrenchesList)
{
    // Read the parameters from the configuration file.
    auto ptr = parametersHandler.lock();
    if (!ptr->getParameter("robotPortName", m_robotPortName))
    {
        yError() << "Robot::configure() : Parameter 'robotPortName' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("axesList", m_axesList))
    {
        yError() << "Robot::configure() : Parameter 'axesList' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("jetsList", m_jetsList))
    {
        yError() << "Robot::configure() : Parameter 'jetsList' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("controlBoardsNames", m_controlBoardsNames))
    {
        yError() << "Robot::configure() : Parameter 'controlBoardsNames' not found in the config "
                    "file.";
        return false;
    }
    if (!ptr->getParameter("gravity", m_gravity))
    {
        yError() << "Robot::configure() : Parameter 'gravity' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("thresholdNormalForceRobotOnTheGround",
                           m_thresholdNormalForceRobotOnTheGround))
    {
        yError() << "Robot::configure() : Parameter 'thresholdNormalForceRobotOnTheGround' not "
                    "found in the config "
                    "file.";
        return false;
    }
    if (!ptr->getParameter("readZMPFromEstimator", m_readZMPFromEstimator))
    {
        yError() << "Robot::configure() : Parameter 'readZMPFromEstimator' not found in the config "
                    "file.";
        return false;
    }
    Eigen::Vector3d CoMOffset;
    if (!ptr->getParameter("CoMOffset", CoMOffset))
    {
        yWarning() << "Robot::configure() : Parameter 'CoMOffset' not found in the config file. "
                      "Setting it to zero.";
        CoMOffset << 0.0, 0.0, 0.0;
    }
    m_deltaCoM << CoMOffset, 1.0;

    // common robot files
    auto robotCommonGroup = ptr->getGroup("ROBOT_COMMON").lock();
    // check if the group is valid
    if (!robotCommonGroup)
    {
        yError() << "Robot::configure() : Group 'ROBOT_COMMON' not found in the config file.";
        return false;
    }

    if (!robotCommonGroup->getParameter("baseFrame", m_floatingBaseFrame))
    {
        yError() << "Robot::configure() : Parameter 'baseFrame' not found in the robot common "
                    "config "
                    "file.";
        return false;
    }

    // We use the iDynTree::ModelLoader class to extract from the URDF file
    // a model containing only the joint we are interested in controlling, and
    // in the same order with which we configured the remotecontrolboardremapper
    // device, to avoid complicated remapping between the vectors used in the YARP
    // devices and the one used by the iDynTree model.
    iDynTree::ModelLoader mdlLoader;
    if (!mdlLoader.loadReducedModelFromFile(modelFullPath, m_axesList))
    {
        yError() << "Robot::configure() : Unable to load reduced model from file" << modelFullPath;
        return false;
    }

    m_kinDynModel = std::make_unique<iDynTree::KinDynComputations>();

    // Once we loaded the model, we pass it to the KinDynComputations class to
    // compute dynamics quantities such as the vector of gravity torques
    if (!m_kinDynModel->loadRobotModel(mdlLoader.model()))
    {
        yError() << "Robot::configure() : Unable to load model with kinDynComputations";
        return false;
    }
    if (!m_kinDynModel->setFloatingBase(m_floatingBaseFrame))
    {
        yError() << "Robot::configure() : Unable to set floating base frame";
        return false;
    }

    const iDynTree::Model model = m_kinDynModel->model();
    yInfo() << "Robot::configure() : Model loaded from " << modelFullPath;

    m_nJoints = m_axesList.size();
    if (m_nJoints != model.getNrOfDOFs())
    {
        yError() << "Robot::configure() : The number of axes in the model (" << model.getNrOfDOFs()
                 << ") does not match the number of axes in the configuration file (" << m_nJoints
                 << ")";
        return false;
    }

    // Loop through the jetsList and calculate the corresponding jetsIndex
    m_nJets = m_jetsList.size();
    for (const auto& jetName : m_jetsList)
    {
        iDynTree::FrameIndex frameIndex = m_kinDynModel->getFrameIndex(jetName);
        if (frameIndex == iDynTree::FRAME_INVALID_INDEX)
        {
            yError() << "Robot::configure() : Frame " << jetName << " not found!";
            return false;
        }
        m_frameIndexJets.emplace_back(frameIndex);
    }
    if (m_frameIndexJets.size() > m_nJets)
    {
        yError() << "Robot::configure() : frameIndexJets are more than the number of jets.";
        return false;
    }

    // External wrenches
    m_nExtWrenches = refFrameExtWrenchesList.size();
    m_sumExtWrenches.setZero();
    m_ZMP.setZero();
    for (const auto& frameName : refFrameExtWrenchesList)
    {
        iDynTree::FrameIndex frameIndex = m_kinDynModel->getFrameIndex(frameName);
        if (frameIndex == iDynTree::FRAME_INVALID_INDEX)
        {
            yError() << "Robot::configure() : Frame " << frameName << " not found!";
            return false;
        }
        m_frameIndexExtWrenches.emplace_back(frameIndex);
    }
    if (m_frameIndexExtWrenches.size() > m_nExtWrenches)
    {
        yError() << "Robot::configure() : frameIndexExtWrenches are more than the number of "
                    "external wrenches.";
        return false;
    }

    // compute total mass
    m_totalMass = 0.0;

    // resize dynamic vectors and matrices
    m_jointPos.resize(m_nJoints);
    m_jointVel.resize(m_nJoints);
    m_jetThrusts.resize(m_nJets);
    m_massMatrix.resize(6 + m_nJoints, 6 + m_nJoints);
    m_centroidalMomentumMatrix.resize(6, 6 + m_nJoints);
    m_Jcom.resize(3, 6 + m_nJoints);
    m_J.resize(6, 6 + m_nJoints);
    m_JRel.resize(6, m_nJoints);
    m_AmomJets.resize(6, m_nJets);
    m_AmomJetsBody.resize(6, m_nJets);
    m_LambdaMomJets.resize(6, 6 + m_nJoints);
    m_LambdaMomExtWrenches.resize(6, 6 + m_nJoints);
    m_LambdaEW.resize(6, 6 + m_nJoints);
    m_Jri.resize(6, 6 + m_nJoints);
    m_SiJri.resize(6, 6 + m_nJoints);
    m_wHCoM.setIdentity();
    // initialize m_J_jets_stacked, initialize m_matrixOfJetArms, set m_jetsAxesLocalFrames, and
    // initialize m_matrixOfJetAxes.
    for (size_t i = 0; i < m_nJets; i++)
    {
        m_J_jets_stacked.emplace_back(Eigen::MatrixXd::Zero(6, 6 + m_nJoints));
        m_J_jets_body_frame.emplace_back(Eigen::MatrixXd::Zero(6, m_nJoints));
        m_matrixOfJetArms.emplace_back(Eigen::Vector3d::Zero());
        // Axes of the (local) jet frames along which the thrust is applied.
        // The sign indicates the thrust direction w.r.t. the frame axis.
        m_jetsAxesLocalFrames.emplace_back(iDynTree::Direction(0, 0, -1));
        m_matrixOfJetAxes.emplace_back(iDynTree::Direction(0, 0, -1));
    }
    // initialize m_J_extWrenches, m_extWrenchesBody, m_extWrenchesBodyMixed, and m_AmomExtWrenches.
    for (size_t i = 0; i < m_nExtWrenches; i++)
    {
        m_J_extWrenches.emplace_back(Eigen::MatrixXd::Zero(6, 6 + m_nJoints));
        m_extWrenchesBody.emplace_back(Eigen::Vector6d::Zero());
        m_extWrenchesMixed.emplace_back(Eigen::Vector6d::Zero());
        m_AmomExtWrenches.emplace_back(Eigen::Matrix6d::Zero());
    }
    return true;
}

bool Robot::setState(const iDynTree::Transform& wHb,
                     const iDynTree::Twist& baseVel,
                     const iDynTree::VectorDynSize& positionsInRad,
                     const iDynTree::VectorDynSize& velocitiesInRadS,
                     const Eigen::VectorXd& jetThrustInNewton,
                     const std::vector<Eigen::Vector6d>& extWrenchesInNewton)
{
    m_jointPos = iDynTree::toEigen(positionsInRad);
    m_jointVel = iDynTree::toEigen(velocitiesInRadS);
    m_wHb = wHb;
    m_baseVel = baseVel;
    m_jetThrusts = jetThrustInNewton;
    m_extWrenchesBody = extWrenchesInNewton;

    if (!m_kinDynModel->setRobotState(m_wHb, positionsInRad, m_baseVel, velocitiesInRadS, m_gravity))
    {
        yError() << "Robot::setState() : Unable to set robot state";
        return false;
    }
    if (!m_kinDynModel->getFreeFloatingMassMatrix(m_massMatrix))
    {
        yError() << "Robot::setState() : Unable to get mass matrix";
        return false;
    }
    m_momentum = iDynTree::toEigen(m_kinDynModel->getCentroidalTotalMomentum().asVector());
    m_wPcom = iDynTree::toEigen(m_kinDynModel->getCenterOfMassPosition());
    if (!m_kinDynModel->getCentroidalTotalMomentumJacobian(m_centroidalMomentumMatrix))
    {
        yError() << "Robot::setState() : Unable to get centroidal momentum matrix";
        return false;
    }
    if (!m_kinDynModel->getCenterOfMassJacobian(m_Jcom))
    {
        yError() << "Robot::setState() : Unable to get jacobian of center of mass";
        return false;
    }
    m_LambdaMomJets.setZero();
    // compute m_J_jets_stacked, m_matrixOfJetAxes, m_AmomJets and m_J_jets_body_frame
    for (size_t i = 0; i < m_nJets; i++)
    {
        if (!m_kinDynModel->getFrameFreeFloatingJacobian(m_frameIndexJets[i], m_J_jets_stacked[i]))
        {
            yError() << "Robot::setState() : Unable to get jacobian of jet frame "
                     << m_frameIndexJets[i];
            return false;
        }
        if (!m_kinDynModel->getRelativeJacobian(m_kinDynModel->getFrameIndex(m_floatingBaseFrame),
                                                m_frameIndexJets[i],
                                                m_J_jets_body_frame[i]))
        {
            yError() << "Robot::setState() : Unable to get relative jacobian of jet frame "
                     << m_frameIndexJets[i];
            return false;
        }

        m_wHCoM.block(0, 0, 3, 3) = iDynTree::toEigen(m_wHb.getRotation());
        m_wHCoM.block(0, 3, 3, 1) = this->getPositionCoM();
        m_wPCoMWithOffset = (m_wHCoM * m_deltaCoM).head(3);

        m_wHjet = m_kinDynModel->getWorldTransform(m_frameIndexJets[i]); // m_H is w_H_jet
        m_matrixOfJetAxes[i] = m_wHjet.getRotation() * m_jetsAxesLocalFrames[i];
        // m_matrixOfJetArms[i] = iDynTree::toEigen(m_wHjet.getPosition()) - m_wPcom;
        m_matrixOfJetArms[i] = iDynTree::toEigen(m_wHjet.getPosition()) - m_wPCoMWithOffset;

        // compute m_AmomJets (linear momentum part)
        m_AmomJets.block(0, i, 3, 1) = iDynTree::toEigen(m_matrixOfJetAxes[i]);
        // compute m_AmomJets (angular momentum part)
        m_skewMatrix = fromVecToSkew(m_matrixOfJetArms[i]);
        m_AmomJets.block(3, i, 3, 1) = m_skewMatrix * iDynTree::toEigen(m_matrixOfJetAxes[i]);
        // compute _LambdaMomJets (linear momentum part)
        m_Si.setZero();
        m_Si.block(3, 0, 3, 3) = fromVecToSkew(iDynTree::toEigen(m_matrixOfJetAxes[i]));
        m_Si.block(0, 3, 3, 3) = fromVecToSkew(iDynTree::toEigen(m_matrixOfJetAxes[i]));
        m_Si.block(3, 3, 3, 3) = fromVecToSkew(m_matrixOfJetArms[i])
                                 * fromVecToSkew(iDynTree::toEigen(m_matrixOfJetAxes[i]));
        m_Si *= m_jetThrusts[i]; // m_Si = m_jetThrusts[i] * m_Si;
        m_Jri = m_J_jets_stacked[i];
        m_Jri.topRows(3) -= m_Jcom;
        m_SiJri = m_Si * m_Jri;
        m_LambdaMomJets -= m_SiJri;
    }

    // compute quantities related to external wrenches: m_sumExtWrenches, m_ZMP, m_J_extWrenches,
    // m_AmomExtWrenches, m_LambdaMomExtWrenches
    m_sumExtWrenches.setZero();
    m_LambdaMomExtWrenches.setZero();
    for (size_t i = 0; i < m_nExtWrenches; i++)
    {
        m_wHextWrench = m_kinDynModel->getWorldTransform(m_frameIndexExtWrenches[i]);
        m_adjointMatrix = iDynTree::toEigen(m_wHextWrench.asAdjointTransformWrench());
        m_wrench = m_adjointMatrix * m_extWrenchesBody[i]; // move the wrench from the frame where
                                                           // it is applied to the world frame
        m_sumExtWrenches += m_wrench;
        // m_extWrenchesMixed
        m_wHextWrench.setPosition(iDynTree::Position::Zero());
        m_adjointMatrix = iDynTree::toEigen(m_wHextWrench.asAdjointTransformWrench());
        m_extWrenchesMixed[i] = m_adjointMatrix * m_extWrenchesBody[i];
        // m_J_extWrenches
        m_kinDynModel->getFrameFreeFloatingJacobian(m_frameIndexExtWrenches[i], m_J_extWrenches[i]);
        // m_AmomExtWrenches
        m_wHextWrench = m_kinDynModel->getWorldTransform(m_frameIndexExtWrenches[i]);
        m_p = m_wHextWrench.getPosition() - iDynTree::Position(m_wPcom);
        m_wHextWrench = iDynTree::Transform(iDynTree::Rotation::Identity(), m_p);
        m_AmomExtWrenches[i] = iDynTree::toEigen(m_wHextWrench.asAdjointTransformWrench());
        // m_LambdaMomExtWrenches
        m_LambdaEW.setZero();
        m_Jri = m_J_extWrenches[i];
        m_Jri.topRows(3) -= m_Jcom; // m_Jri = m_J_extWrenches[i] - m_Jcom;
        m_force = m_extWrenchesMixed[i].topRows(3);
        m_skewMatrix = fromVecToSkew(m_force);
        m_LambdaEW.bottomRows(3) = -m_skewMatrix * m_Jri.topRows(3);
        m_LambdaMomExtWrenches += m_LambdaEW;
    }

    m_isRobotOnTheGround = (m_sumExtWrenches[2] > m_thresholdNormalForceRobotOnTheGround);

    if (!m_readZMPFromEstimator && m_isRobotOnTheGround)
    {
        m_ZMP.setZero();
        if (m_sumExtWrenches[2] > m_thresholdNormalForceRobotOnTheGround)
        {
            m_ZMP[0] = (-m_sumExtWrenches[4] / m_sumExtWrenches[2]);
            m_ZMP[1] = (m_sumExtWrenches[3] / m_sumExtWrenches[2]);
        }
    }

    // compute body coordinate quantities
    m_H = iDynTree::Transform(m_wHb.getRotation().inverse(), iDynTree::Position::Zero());
    m_adjointMatrix = iDynTree::toEigen(m_H.asAdjointTransform());
    m_momentumBody = m_adjointMatrix * m_momentum;
    m_adjointMatrix = iDynTree::toEigen(m_H.asAdjointTransformWrench());
    m_AmomJetsBody = m_adjointMatrix * m_AmomJets;

    // compute total mass
    m_totalMass = m_massMatrix(0, 0);

    return true;
}

const bool Robot::setZMP(const Eigen::Ref<Eigen::Vector3d> zmp)
{
    if (zmp.size() != 3)
    {
        yError() << "Robot::setZMP() : input vector has wrong size";
        return false;
    }
    if (!m_readZMPFromEstimator)
    {
        yError() << "Robot::setZMP() : You cannot set ZMP! readZMPFromEstimator is false!";
        return false;
    }
    m_ZMP = zmp;
    return true;
}

// Getters

const size_t Robot::getNJoints() const
{
    return m_nJoints;
}

const size_t Robot::getNJets() const
{
    return m_nJets;
}

const size_t Robot::getNExtWrenches() const
{
    return m_nExtWrenches;
}

Eigen::Ref<const Eigen::VectorXd> Robot::getJointPos() const
{
    return m_jointPos;
}

Eigen::Ref<const Eigen::VectorXd> Robot::getJointVel() const
{
    return m_jointVel;
}

std::string Robot::getJointName(int jointPos) const
{
    return m_kinDynModel->model().getJointName(jointPos);
}

Eigen::Ref<const Eigen::VectorXd> Robot::getJetThrusts() const
{
    return m_jetThrusts;
}

const iDynTree::Transform Robot::getBasePose() const
{
    return m_wHb;
}

const iDynTree::Twist Robot::getBaseVel() const
{
    return m_baseVel;
}

const iDynTree::Vector3& Robot::getGravity() const
{
    return m_gravity;
}

Eigen::Ref<const Eigen::MatrixXd> Robot::getMassMatrix() const
{
    return m_massMatrix;
}

const double Robot::getTotalMass() const
{
    return m_totalMass;
}

Eigen::Ref<const Eigen::Vector6d> Robot::getMomentum(bool inBodyCoord) const
{
    if (inBodyCoord)
    {
        // From g[w]m_momentum to g[b]m_momentum
        return m_momentumBody;
    } else
    {
        return m_momentum;
    }
}

Eigen::Ref<const Eigen::Vector3d> Robot::getPositionCoM() const
{
    return m_wPcom;
}

Eigen::Ref<const Eigen::Vector3d> Robot::getPositionCoMWihtOffset() const
{
    return m_wPCoMWithOffset;
}

Eigen::Ref<const Eigen::MatrixXd> Robot::getCentroidalMomentumMatrix() const
{
    return m_centroidalMomentumMatrix;
}

Eigen::Ref<const Eigen::MatrixXd> Robot::getJacobianCoM() const
{
    return m_Jcom;
}

const std::vector<iDynTree::FrameIndex> Robot::getIndexesJetFrames() const
{
    return m_frameIndexJets;
}

const std::vector<Eigen::MatrixXd>& Robot::getJacobianJetsStacked() const
{
    return m_J_jets_stacked;
}

const std::vector<Eigen::MatrixXd>& Robot::getRelativeJacobianJetsBodyFrame() const
{
    return m_J_jets_body_frame;
}

const std::vector<iDynTree::Direction> Robot::getMatrixOfJetAxes() const
{
    return m_matrixOfJetAxes;
}

const std::vector<Eigen::Vector3d>& Robot::getMatrixOfJetArms() const
{
    return m_matrixOfJetArms;
}

Eigen::Ref<const Eigen::MatrixXd> Robot::getMatrixAmomJets(bool inBodyCoord) const
{
    if (inBodyCoord)
    {
        // From g[w]m_momentum to g[b]m_momentum
        return m_AmomJetsBody;
    } else
    {
        return m_AmomJets;
    }
}

Eigen::Ref<const Eigen::MatrixXd> Robot::getMatrixLambdaMomJets() const
{
    return m_LambdaMomJets;
}

const iDynTree::Transform Robot::getWorldTransform(const std::string& frameName)
{
    // TODO: improve to compute the transform inside setState and then return it here
    m_H = m_kinDynModel->getWorldTransform(m_kinDynModel->getFrameIndex(frameName));
    return m_H;
}

const iDynTree::Transform
Robot::getRelativeTransform(const std::string& refFrameName, const std::string& frameName)
{
    // TODO: improve to compute the transform inside setState and then return it here
    m_H = m_kinDynModel->getRelativeTransform(m_kinDynModel->getFrameIndex(refFrameName),
                                              m_kinDynModel->getFrameIndex(frameName));
    return m_H;
}

const Eigen::MatrixXd Robot::getJacobian(const std::string& frameName)
{
    // TODO: improve to compute the jacobian inside setState and then return it here
    m_J.setZero();
    if (!m_kinDynModel->getFrameFreeFloatingJacobian(m_kinDynModel->getFrameIndex(frameName), m_J))
    {
        yError() << "Unable to get jacobian of frame " << frameName << ", returning zero matrix";
    }
    return m_J;
}

Eigen::Ref<const Eigen::Vector6d> Robot::getSumExtWrenches() const
{
    return m_sumExtWrenches;
}

const std::vector<Eigen::Vector6d>& Robot::getExtWrenchesInMixedRepr() const
{
    return m_extWrenchesMixed;
}

const std::vector<Eigen::MatrixXd>& Robot::getJacobianExtWrenches() const
{
    return m_J_extWrenches;
}

const std::vector<Eigen::Matrix6d>& Robot::getMatrixAmomExtWrenches() const
{
    return m_AmomExtWrenches;
}

Eigen::Ref<const Eigen::MatrixXd> Robot::getMatrixLambdaMomExtWrenches() const
{
    return m_LambdaMomExtWrenches;
}

Eigen::Ref<const Eigen::Vector3d> Robot::getZMP() const
{
    return m_ZMP;
}

const std::vector<std::string>& Robot::getAxesList() const
{
    return m_axesList;
}

const std::vector<std::string>& Robot::getJetsList() const
{
    return m_jetsList;
}

const std::vector<std::string>& Robot::getControlBoardsNames() const
{
    return m_controlBoardsNames;
}

const std::string& Robot::getFloatingBaseFrameName() const
{
    return m_floatingBaseFrame;
}

const std::string& Robot::getRobotPortName() const
{
    return m_robotPortName;
}

const bool Robot::isRobotOnTheGround() const
{
    return m_isRobotOnTheGround;
}

const Eigen::MatrixXd
Robot::getRelativeJacobian(const std::string refFrameName, const std::string frameName)
{
    m_JRel.setZero();
    if (!m_kinDynModel->getRelativeJacobian(m_kinDynModel->getFrameIndex(refFrameName),
                                            m_kinDynModel->getFrameIndex(frameName),
                                            m_JRel))
    {
        yError() << "Unable to get relative jacobian of frame " << frameName << " w.r.t. frame "
                 << refFrameName << ", returning zero matrix";
    }
    return m_JRel;
}
