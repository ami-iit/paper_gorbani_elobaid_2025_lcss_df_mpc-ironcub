#ifndef QPINPUT
#define QPINPUT

#include "JetModel.h"
#include "Robot.h"
#include <BipedalLocomotion/YarpUtilities/VectorsCollectionServer.h>

/**
 * @class QPInput
 * @brief A struct for collecting input parameters to pass to the QPProblem class.
 */
class QPInput
{
public:
    const std::shared_ptr<Robot> getRobot() const;
    void setRobot(std::shared_ptr<Robot> robot);

    const std::shared_ptr<Robot> getRobotReference() const;
    void setRobotReference(std::shared_ptr<Robot> robotReference);

    const std::shared_ptr<BipedalLocomotion::YarpUtilities::VectorsCollectionServer>
    getVectorsCollectionServer() const;
    void setVectorsCollectionServer(
        std::shared_ptr<BipedalLocomotion::YarpUtilities::VectorsCollectionServer>
            vectorsCollectionServer);

    const Eigen::Ref<const Eigen::VectorXd> getOutputQPThrust();
    void setOutputQPThrust(const Eigen::Ref<const Eigen::VectorXd> outputQPThrust);
    const Eigen::Ref<const Eigen::VectorXd> getOutputQPJointsPosition() const;
    void setOutputQPJointsPosition(const Eigen::Ref<const Eigen::VectorXd> outputQPJointsPosition);
    const Eigen::Ref<const Eigen::VectorXd> getOutputQPThrustDot() const;
    void setOutputQPThrustDot(const Eigen::Ref<const Eigen::VectorXd> outputQPThrustDot);
    const Eigen::Ref<const Eigen::VectorXd> getOutputQPJointsVelocity() const;
    void setOutputQPJointsVelocity(const Eigen::Ref<const Eigen::VectorXd> outputQPJointsVelocity);

    const Eigen::Ref<const Eigen::VectorXd> getThrustReference() const;
    void setThrustReference(const Eigen::Ref<const Eigen::VectorXd> thrustReference);
    const Eigen::Ref<const Eigen::VectorXd> getJointPosReference() const;
    void setJointPosReference(const Eigen::Ref<const Eigen::VectorXd> jointPosReference);
    const Eigen::Ref<const Eigen::Vector3d> getPosCoMReference();
    void setPosCoMReference(const Eigen::Ref<const Eigen::Vector3d> posCoMReference);
    const Eigen::Ref<const Eigen::Vector3d> getVelCoMReference() const;
    void setVelCoMReference(const Eigen::Ref<const Eigen::Vector3d> velCoMReference);
    const Eigen::Ref<const Eigen::Vector3d> getRPYReference() const;
    void setRPYReference(const Eigen::Ref<const Eigen::Vector3d> RPYReference);
    const Eigen::Ref<const Eigen::Vector3d> getRPYDotReference() const;
    void setRPYDotReference(const Eigen::Ref<const Eigen::Vector3d> RPYDotReference);
    const iDynTree::Rotation& getBaseRotReference() const;
    void setBaseRotReference(const iDynTree::Rotation& baseRotReference);

    const Eigen::Ref<const Eigen::VectorXd> getThrustJetControl() const;
    void setThrustJetControl(const Eigen::Ref<const Eigen::VectorXd> thrustJetControl);

    const Eigen::Ref<const Eigen::VectorXd> getOutputQPSolution() const;
    void setOutputQPSolution(const Eigen::Ref<const Eigen::VectorXd> outputQPSolution);

    const std::vector<Eigen::Vector6d>& getContactWrenchesReference() const;
    void setContactWrenchesReference(const std::vector<Eigen::Vector6d>& contactWrenchesReference);
    const std::vector<std::string>& getContactWrenchesFrameName() const;
    void setContactWrenchesFrameName(const std::vector<std::string>& contactWrenchesFrameName);
    void setSumExternalWrenchesInertialFrame(
        const Eigen::Ref<const Eigen::Vector6d> sumExternalWrenchesInertialFrame);
    const Eigen::Ref<const Eigen::Vector6d> getSumExternalWrenchesInertialFrame() const;
    const double getFlightControllerPeriod() const;
    void setFlightControllerPeriod(const double flightControllerPeriod);
    const Eigen::Vector6d& getMomentumReference() const;
    void setMomentumReference(const Eigen::Vector6d& momentumReference);
    const Eigen::Vector6d& getMomentumDotReference() const;
    void setMomentumDotReference(const Eigen::Vector6d& momentumDotReference);
    const Eigen::Vector6d& getMomentumDotDotReference() const;
    void setMomentumDotDotReference(const Eigen::Vector6d& momentumDotDotReference);
    void setFreezeAlphaGravity(const bool freezeAlphaGravity);
    const bool getFreezeAlphaGravity() const;

    void setAlphaGravity(const double alphaGravity);
    const double getAlphaGravity() const;

    const Eigen::Ref<const Eigen::VectorXd> getThrustDesMPC() const;
    void setThrustDesMPC(const Eigen::Ref<const Eigen::VectorXd> thrustDesMPC);
    const Eigen::Ref<const Eigen::VectorXd> getThrustDotDesMPC() const;
    void setThrustDotDesMPC(const Eigen::Ref<const Eigen::VectorXd> thrustDotDesMPC);
    const Eigen::Ref<const Eigen::VectorXd> getEstimatedThrustDot() const;
    void setEstimatedThrustDot(const Eigen::Ref<const Eigen::VectorXd> estimatedThrustDot);
    const Eigen::Ref<const Eigen::VectorXd> getThrottleMPC() const;
    void setThrottleMPC(const Eigen::Ref<const Eigen::VectorXd> throttleMPC);

    const std::shared_ptr<JetModel> getJetModel() const;
    void setJetModel(std::shared_ptr<JetModel> jetModel);

private:
    std::shared_ptr<Robot> m_robot; // nullptr is set by default
    std::shared_ptr<Robot> m_robotReference; // nullptr is set by default
    std::shared_ptr<BipedalLocomotion::YarpUtilities::VectorsCollectionServer>
        m_vectorsCollectionServer; // nullptr is set by default
    std::shared_ptr<JetModel> m_jetModel; // nullptr is set by default
    Eigen::Vector3d m_posCoMReference; // CoM position reference
    Eigen::Vector3d m_velCoMReference; // CoM velocity reference
    Eigen::Vector3d m_RPYReference; // base orientation reference
    Eigen::Vector3d m_RPYDotReference; // base orientation rate reference
    Eigen::VectorXd m_outputQPthrust; // Output QP thrust for each jet
    std::mutex m_outputQPthrustMutex; // Mutex for output QP thrust
    Eigen::VectorXd m_outputQPthrustDot; // Output QP thrust rate for each jet
    Eigen::VectorXd m_thrustReference; // Thrust reference for each jet
    Eigen::VectorXd m_thrustJetControl; // Thrust to be set in the jet control in case jet dynamics
    // is considered in the flight controller
    Eigen::VectorXd m_outputQPSolution; // Output QP solution
    Eigen::VectorXd m_outputQPJointsPosition; // Output QP joint position for each joint
    Eigen::VectorXd m_outputQPJointsVelocity; // Output QP joint velocity for each joint
    Eigen::VectorXd m_jointPosReference; // Joint position reference for each joint
    iDynTree::Rotation m_baseRotReference; // Desired base orientation
    std::vector<Eigen::Vector6d> m_contactWrenchesReference; // Reference contact wrenches
    std::vector<std::string> m_contactWrenchesFrameName; // Names of the contact wrenches
    Eigen::Vector6d m_sumExternalWrenchesInertialFrame; // Sum external wrenches in inertial frame
    double m_flightControllerPeriod{0.0}; // Period of the flight controller
    Eigen::Vector6d m_momentumReference; // Desired momentum
    Eigen::Vector6d m_momentumDotReference; // Desired momentum rate
    Eigen::Vector6d m_momentumDotDotReference; // Desired momentum acceleration
    bool m_freezeAlphaGravity; // Flag to freeze the gravity compensation term
    std::atomic<double> m_alphaGravity; // Percentage of gravity compensation
    Eigen::VectorXd m_thrustDesMPC; // Desired thrust for each jet computed by the MPC
    Eigen::VectorXd m_thrustDotDesMPC; // Desired thrust rate for each jet computed by the MPC
    Eigen::VectorXd m_throttleMPC; // Throttle computed by the MPC
    Eigen::VectorXd m_estimatedThrustDot; // Estimated thrust rate for each jet
};

#endif
