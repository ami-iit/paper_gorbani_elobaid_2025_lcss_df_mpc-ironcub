#ifndef ROBOT_H
#define ROBOT_H

#include <string>

#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <Eigen/Dense>
#include <iDynTree/Direction.h>
#include <iDynTree/EigenHelpers.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/MatrixDynSize.h>
#include <iDynTree/MatrixFixSize.h>
#include <iDynTree/Model.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/Position.h>
#include <iDynTree/Rotation.h>
#include <iDynTree/Transform.h>
#include <iDynTree/VectorDynSize.h>
#include <iDynTree/VectorFixSize.h>

#include <yarp/os/LogStream.h>

namespace Eigen
{
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
} // namespace Eigen

/**
 * @class Robot
 * @brief A class that stores kinematic and dynamic information of a flying robot powered by jets.
 */
class Robot
{

public:
    /**
     * @brief Configures the robot model.
     * @param modelFullPath The path to the robot model file.
     * @param parametersHandler Reference to the ParametersHandler object (to access configuration
     * parameters).
     * @param refFrameExtWrenchesList A vector that contains the reference frame names associated
     * with the external wrenches.
     * @return True if configuration is successful, false otherwise.
     */
    bool configure(
        const std::string& modelFullPath,
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        const std::vector<std::string>& refFrameExtWrenchesList);

    /**
     * @brief Sets the state of the robot.
     * @param wHb The homogenous transform representing the pose of the floating base.
     * @param baseVel The twist representing the velocity of the floating base link.
     * @param positionsInRad The joint positions in radians.
     * @param velocitiesInRadS The joint velocities in radians per second.
     * @param jetThrustInNewton The thrust of each jet in Newton.
     * @param extWrenchesInNewton A vector containing the external wrenches.
     * @return True if the state is set successfully, false otherwise.
     */
    bool setState(const iDynTree::Transform& wHb,
                  const iDynTree::Twist& baseVel,
                  const iDynTree::VectorDynSize& positionsInRad,
                  const iDynTree::VectorDynSize& velocitiesInRadS,
                  const Eigen::VectorXd& jetThrustInNewton,
                  const std::vector<Eigen::Vector6d>& extWrenchesInNewton);

    /**
     * @brief Set the ZMP of the robot in the world frame.
     * @param ZMP The ZMP of the robot in the world frame.
     * @return True if the ZMP is set successfully, false otherwise.
     */
    const bool setZMP(const Eigen::Ref<Eigen::Vector3d>);

    /**
     * @brief Gets the twist representing the velocity of the floating base.
     * @return The twist of the floating base.
     */
    const iDynTree::Twist getBaseVel() const;

    /**
     * @brief Gets the pose of the floating base in the world frame.
     * @return The pose of the floating base.
     */
    const iDynTree::Transform getBasePose() const;

    /**
     * @brief  Gets the transform of a specified frame in the world frame.
     * @param frameName The name of the frame.
     * @return The pose of the frame.
     */
    const iDynTree::Transform getWorldTransform(const std::string& frameName);

    /**
     * @brief Gets the relative transform between two frames.
     * @param refFrameName The name of the reference frame.
     * @param frameName The name of the target frame.
     * @return The relative transform between the reference and target frames.
     */
    const iDynTree::Transform
    getRelativeTransform(const std::string& refFrameName, const std::string& frameName);

    /**
     * @brief Gets the number of joints in the robot model.
     * @return The number of joints.
     */
    const size_t getNJoints() const;

    /**
     * @brief Gets the number of jets in the robot model.
     * @return The number of jets.
     */
    const size_t getNJets() const;

    /**
     * @brief Gets the number of external wrenches.
     * @return The number of external wrenches.
     */
    const size_t getNExtWrenches() const;

    /**
     * @brief Gets the total mass of the robot.
     * @return The total mass of the robot.
     */
    const double getTotalMass() const;

    /**
     * @brief Gets the joint positions in radians.
     * @return The joint positions in radians.
     */
    Eigen::Ref<const Eigen::VectorXd> getJointPos() const;

    /**
     * @brief Gets the joint velocities in radians per second.
     * @return The joint velocities in radians per second.
     */
    Eigen::Ref<const Eigen::VectorXd> getJointVel() const;

    /**
     * @brief Gets the joints list.
     * @return The joints list.
     */
    std::string getJointName(int jointPos) const;

    /**
     * @brief Gets the thrust of each jet in Newton.
     * @return The thrust of each jet in Newton.
     */
    Eigen::Ref<const Eigen::VectorXd> getJetThrusts() const;

    /**
     * @brief Gets the gravity vector in the world frame.
     * @return The gravity vector in the world frame.
     */
    const iDynTree::Vector3& getGravity() const;

    /**
     * @brief Gets the mass matrix of the robot.
     * @return The mass matrix of the robot.
     */
    Eigen::Ref<const Eigen::MatrixXd> getMassMatrix() const;

    /**
     * @brief Gets the centroidal momentum of the robot.
     * @param inBodyCoord If true, the momentum is expressed in the body frame, otherwise it is
     * expressed in the world frame.
     * @return The centroidal momentum of the robot.
     */
    Eigen::Ref<const Eigen::Vector6d> getMomentum(bool inBodyCoord = false) const;

    /**
     * @brief Gets the position of the center of mass of the robot in the world frame.
     * @return The position of the center of mass of the robot in the world frame.
     */
    Eigen::Ref<const Eigen::Vector3d> getPositionCoM() const;

    Eigen::Ref<const Eigen::Vector3d> getPositionCoMWihtOffset() const;

    /**
     * @brief Gets the centroidal momentum matrix of the robot.
     * @return The centroidal momentum matrix of the robot.
     */
    Eigen::Ref<const Eigen::MatrixXd> getCentroidalMomentumMatrix() const;

    /**
     * @brief Gets the center of mass jacobian of the robot.
     * @return The center of mass jacobian of the robot.
     */
    Eigen::Ref<const Eigen::MatrixXd> getJacobianCoM() const;

    /**
     * @brief Gets the jacobian of a specified frame.
     * @param frameName The name of the frame.
     * @return The jacobian of the frame.
     */
    const Eigen::MatrixXd getJacobian(const std::string& frameName);

    /**
     * @brief Gets the A matrix to compute the external wrenches applied in the center of mass.
     * A*thrust = wrench. The wrenches are expressend the world (or body) frame.
     * @param inBodyCoord If true, the wrenches are expressed in the body frame, otherwise they are
     * expressed in the world frame.
     * @return The A matrix of the momentum equation.
     */
    Eigen::Ref<const Eigen::MatrixXd> getMatrixAmomJets(bool inBodyCoord = false) const;

    /**
     * @brief Gets the matrix to obtain (dA/dt * thrust) if multiplied by ν (configuration
     * velocity).
     * @return The Λ matrix to satisfy Λ * ν = dA/dt * thrust.
     */
    Eigen::Ref<const Eigen::MatrixXd> getMatrixLambdaMomJets() const;

    /**
     * @brief Gets a vector containing the indexes of the frames of the jets.
     * @return A vector containing the indexes of the frames of the jets.
     */
    const std::vector<iDynTree::FrameIndex> getIndexesJetFrames() const;

    /**
     * @brief Gets a vector containing the axes of the jets in the world frames.
     * @return A vector containing the axes of the jets in the world frames.
     */
    const std::vector<iDynTree::Direction> getMatrixOfJetAxes() const;

    /**
     * @brief Gets a vector containing the free floating jacobian of each jet.
     * @return A vector containing the free floating jacobian of each jet.
     */
    const std::vector<Eigen::MatrixXd>& getJacobianJetsStacked() const;

    /**
     * @brief Gets a vector containing the relative jacobian between the body frame and the jets
     * frames.
     * @return A vector containing the relative jacobian between the body frame and the jets frames.
     */
    const std::vector<Eigen::MatrixXd>& getRelativeJacobianJetsBodyFrame() const;

    /**
     * @brief Gets a vector containing the distance between the jets and the center of mass of
     * the robot.
     * @return A vector containing the distance between the jets and the center of mass of the
     * robot.
     */
    const std::vector<Eigen::Vector3d>& getMatrixOfJetArms() const;

    /**
     * @brief Gets the sum wrench of external forces applied to the robot in the world frame.
     * @return The sum wrench of external forces applied to the robot in the world frame.
     */
    Eigen::Ref<const Eigen::Vector6d> getSumExtWrenches() const;

    /**
     * @brief Gets a vector containing the external wrenches in the frame where they are applied.
     * @return A vector containing the external wrenches in the frame where they are applied.
     */
    const std::vector<Eigen::Vector6d>& getExtWrenchesInMixedRepr() const;

    /**
     * @brief Gets a vector containing the free floating jacobian of the frames where the external
     * wrenches are applied.
     * @return A vector containing the free floating jacobian of the frames where the external
     * wrenches are applied.
     */
    const std::vector<Eigen::MatrixXd>& getJacobianExtWrenches() const;

    /**
     * @brief Gets a vector containing the A matrix to translate the wrenches (in mixed
     * representation) into a frame coincident with the robot's Center of Mass.
     * @return A vector containing the A matrix to translate the wrenches (in mixed
     * representation) into a frame coincident with the robot's Center of Mass.
     */
    const std::vector<Eigen::Matrix6d>& getMatrixAmomExtWrenches() const;

    /**
     * @brief Gets the matrix that multiplies ν to obtain Σ_i (dAi/dt * wrench_i).
     * Λ * ν = Σ_i (dAi/dt * wrench_i).
     * @return The matrix that multiplies ν to obtain Σ_i (dAi/dt * wrench_i).
     */
    Eigen::Ref<const Eigen::MatrixXd> getMatrixLambdaMomExtWrenches() const;

    /**
     * @brief Gets the ZMP of the robot in the world frame.
     * @return The ZMP of the robot in the world frame.
     */
    Eigen::Ref<const Eigen::Vector3d> getZMP() const;

    /**
     * @brief Get list of robot joint axes names.
     * @return Vector containing the joint axes names.
     */
    const std::vector<std::string>& getAxesList() const;

    /**
     * @brief Get list of robot jets names.
     * @return Vector containing the jets names.
     */
    const std::vector<std::string>& getJetsList() const;

    /**

     * @brief Get list of robot control boards.
     * @return Vector containing the control board names.
     */
    const std::vector<std::string>& getControlBoardsNames() const;

    /**
     * @brief Get floating base frame name.
     * @return The floating base frame name.
     */
    const std::string& getFloatingBaseFrameName() const;

    /**
     * @brief Get robot port name.
     * @return The robot port name.
     */
    const std::string& getRobotPortName() const;

    /**
     * @brief Get the relative jacobian between two frames.
     * @param refFrameName The name of the reference frame.
     * @param frameName The name of the target frame.
     * @return The relative jacobian between the reference and target frames.
     */
    const Eigen::MatrixXd
    getRelativeJacobian(const std::string refFrameName, const std::string frameName);

    /**
     * @brief Get a boolean that indicates if the robot is on the ground.
     * @return True if the robot is on the ground, false otherwise.
     */
    const bool isRobotOnTheGround() const;

private:
    size_t m_nJoints; /**< The number of joints in the robot model. */
    size_t m_nJets; /**< The number of jets in the robot model. */
    size_t m_nExtWrenches; /**< The number of external wrenches. */
    float m_totalMass; /**< The total mass of the robot. */
    std::string m_floatingBaseFrame; /**< The name of the frame representing the floating base link.
                                      */

    std::unique_ptr<iDynTree::KinDynComputations> m_kinDynModel; /**< The iDynTree
                                                   KinDynComputations object to compute model
                                                   quantities. */
    iDynTree::Transform m_wHb; /**< The homogenous transform representing the pose of the floating
                                  base. */
    Eigen::Matrix4d m_wHCoM; /**< The homogenous transform representing the pose of the center
                                    of mass in mixed. */
    iDynTree::Twist m_baseVel; /**< The twist representing the velocity of the floating base link.
                                */
    iDynTree::Vector3 m_gravity; /**< The gravity vector in the world frame. */
    Eigen::VectorXd m_jointPos; /**< The joint positions in radians. */
    Eigen::VectorXd m_jointVel; /**< The joint velocities in radians per second. */
    Eigen::VectorXd m_jetThrusts; /**< The thrust of each jet in Newton. */
    Eigen::Vector6d m_momentum; /**< The centroidal momentum of the robot in the interial frame. */
    Eigen::Vector6d m_momentumBody; /**< The centroidal momentum of the robot in the body frame. */
    Eigen::Vector3d m_wPcom; /**< The position of the center of mass of the robot in the world
                                frame. */
    Eigen::Vector3d m_wPCoMWithOffset; /**< The position of the center of mass of the robot in the
                                         world frame with an offset. */
    Eigen::Vector4d m_deltaCoM; /**< The offset of the center of mass of the robot. */
    Eigen::MatrixXd m_massMatrix; /**< The mass matrix of the robot. */
    Eigen::MatrixXd m_centroidalMomentumMatrix; /**< The centroidal momentum matrix of the robot. */
    Eigen::MatrixXd m_Jcom; /**< The center of mass jacobian of the robot. */
    Eigen::MatrixXd m_AmomJets; /**< The A matrix to compute the jet wrench applied in the
                                   center of mass. A*thrust = wrench. */
    Eigen::MatrixXd m_AmomJetsBody; /**< The A matrix to compute the jet wrench applied in
                                       the center of mass expressed in the body frame. A*thrust =
                                       wrench. */
    Eigen::MatrixXd m_LambdaMomJets; /**< The matrix that multiplies ν to obtain dA/dt * thrust.
                                        Λ * ν = dA/dt * thrust */
    std::vector<iDynTree::FrameIndex> m_frameIndexJets; /**< A vector containing the indexes of the
                                                           frames of the jets. */
    std::vector<Eigen::MatrixXd> m_J_jets_stacked; /**< A vector containing the free floating
                                                      jacobian of each jet. */
    std::vector<Eigen::MatrixXd> m_J_jets_body_frame; /**< A vector containing the relative jacobian
                                                         between the body frame and the jets frames.
                                                       */
    std::vector<iDynTree::Direction> m_jetsAxesLocalFrames; /**< A vector containing the axes of the
                                                               jets in the local frames. */
    std::vector<iDynTree::Direction> m_matrixOfJetAxes; /**< A vector containing the axes of the
                                                           jets in the world frame. */
    std::vector<Eigen::Vector3d> m_matrixOfJetArms; /**< A vector containing the distance between
                                                       the jets and the center of mass of the robot.
                                                     */
    Eigen::Vector6d m_sumExtWrenches; /**< The sum wrench of external forces applied to the robot in
                                         the world frame. */
    Eigen::Vector3d m_ZMP; /**< The ZMP of the robot in the world frame. */
    bool m_readZMPFromEstimator; /**< If true, the ZMP is read from the ZMP estimator. */
    std::vector<Eigen::Vector6d> m_extWrenchesBody; /**< A vector containing the external wrenches
                                                   in the frame where they are applied. */
    std::vector<Eigen::Vector6d> m_extWrenchesMixed; /**< A vector containing the external wrenches
                                                        in the frame where they are applied and
                                                        oriented as the inertial frame. */
    std::vector<iDynTree::FrameIndex> m_frameIndexExtWrenches; /**< A vector containing the indexes
                                                                  of the frames of the external
                                                                  wrenches. */
    std::vector<Eigen::MatrixXd> m_J_extWrenches; /**< A vector containing the free
                                                              floating jacobian of each external
                                                              wrench. */
    std::vector<Eigen::Matrix6d> m_AmomExtWrenches; /**< A vector containing the A matrix to
                                                       translate the wrenches into a frame
                                                       coincident with the robot's Center of Mass.
                                                       Ai * wrench_i. */

    Eigen::MatrixXd m_LambdaMomExtWrenches; /**< The matrix that multiplies ν to obtain Σ_i (dAi/dt
                                             * wrench_i).
                                             Λ * ν = Σ_i (dAi/dt * wrench_i) */

    double m_thresholdNormalForceRobotOnTheGround; /**< The threshold to consider a robot in contact
                                                      with the ground. */
    bool m_isRobotOnTheGround; /**< A boolean that indicates if the robot is on the ground. */

    std::string m_robotPortName; /**< The name of the robot port. */
    std::vector<std::string> m_axesList; /**< A vector that contains the joint axes names. */
    std::vector<std::string> m_jetsList; /**< A vector that contains the propeller names. */
    std::vector<std::string> m_controlBoardsNames; /**< A vector that contains the control board
                                                      names. */

    iDynTree::Position m_p; /**< A temporary position to allocate memory. */
    iDynTree::Rotation m_R; /**< A temporary rotation to allocate memory. */
    iDynTree::Transform m_H; /**< A temporary transform to allocate memory to compute transforms. */
    iDynTree::Transform m_wHjet; /**< A temporary transform to allocate memory to compute
                                    transforms. */
    iDynTree::Transform m_wHextWrench; /**< A temporary transform to allocate memory to compute
                                          transforms. */
    Eigen::Matrix3d m_skewMatrix; /**< A temporary matrix to create skewMatrix. */
    Eigen::Matrix6d m_adjointMatrix; /**< A temporary matrix to create adjointMatrix. */
    Eigen::MatrixXd m_J; /**< A temporary matrix to allocate memory to compute Jacobians. */
    Eigen::MatrixXd m_JRel; /**< A temporary matrix to allocate memory to compute the relative
                               Jacobians between two frames. */
    Eigen::Matrix6d m_Si; /**< A temporary matrix to allocate memory to compute m_LambdaMomJets. */
    Eigen::MatrixXd m_Jri; /**< A temporary matrix to allocate memory to compute m_LambdaMomJets. */
    Eigen::MatrixXd m_SiJri; /**< A temporary matrix to allocate memory to compute
                                m_LambdaMomJets. */
    Eigen::MatrixXd m_LambdaEW; /**< A temporary matrix to allocate memory to compute
                                 * m_LambdaMomExtWrenches. */
    Eigen::Vector6d m_wrench; /**< A temporary vector to allocate memory to compute a wrench*/
    Eigen::Vector3d m_force; /**< A temporary vector to allocate memory to compute a wrench*/
    Eigen::Vector3d m_torque; /**< A temporary vector to allocate memory to compute a wrench*/
};

#endif /* end of include guard ROBOT_H */
