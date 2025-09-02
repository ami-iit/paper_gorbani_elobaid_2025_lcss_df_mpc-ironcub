/**
 * @file SystemDynamicVS.h
 * @authors Davide Gorbani
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef SYSTEM_DYNAMIC_DF_H
#define SYSTEM_DYNAMIC_DF_H

#include "QPInput.h"
#include "TrajectoryManager.h"
#include <Eigen/Dense>
#include <IMPCProblem/systemDynamic.h>
#include <boost/core/demangle.hpp>

class AngularMomentumDynamicDF : public DynamicTemplateVariableSampling
{
public:
    AngularMomentumDynamicDF(const int nStates, const int nJoints, const int nThrottle);
    ~AngularMomentumDynamicDF() = default;

    const bool configure(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;

    const bool updateInitialStates(QPInput& qpInput) override;

private:
    void computeLambdaAng(QPInput& qpInput);
    const Eigen::MatrixXd getRelativeJacobianCoM(std::string frameName);
    const bool computeAngularMomentumMatrices();
    void updateRPY();

    double m_periodMPC;
    std::vector<int> m_jointSelectorVector;
    Eigen::VectorXd m_B_omega_B;
    Eigen::Vector3d m_RPY;
    Eigen::Vector3d m_RPYDot;
    Eigen::Vector3d m_rpyInit;
    Eigen::VectorXd m_thrustMean;
    Eigen::Matrix3d m_wRb;
    Eigen::Matrix3d m_W;
    Eigen::Matrix3d m_WInverse;
    Eigen::Matrix3d m_WDot;
    Eigen::Matrix3d m_inertia;
    Eigen::MatrixXd m_lambdaAngB;
    Eigen::MatrixXd m_lambdaAng;
    Eigen::MatrixXd m_SiAngMom;
    std::vector<Eigen::MatrixXd> m_relJacobianInit;
    std::vector<iDynTree::Direction> m_matJetAxisInit;
    std::vector<Eigen::Vector3d> m_matJetArmsInit;

    iDynTree::Transform m_forcesTransform;

    std::shared_ptr<Robot> m_robot;
    std::shared_ptr<Robot> m_robotReference;
    std::shared_ptr<BipedalLocomotion::YarpUtilities::VectorsCollectionServer>
        m_vectorsCollectionServer;
    JointsLambda m_jointsLambda;
};

class LinearMomentumDynamicDF : public DynamicTemplateVariableSampling
{
public:
    LinearMomentumDynamicDF(const int nStates, const int nJoints, const int nThrottle);
    ~LinearMomentumDynamicDF() = default;

    const bool configure(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;

    const bool updateInitialStates(QPInput& qpInput) override;

private:
    const bool computeLambdaLin(QPInput& qpInput);
    const bool computeLinearMomentumMatrices(QPInput& qpInput);

    double m_periodMPC;
    Eigen::VectorXd m_B_omega_B;
    Eigen::VectorXd m_thrustMean;
    Eigen::MatrixXd m_A;
    Eigen::MatrixXd m_B;
    Eigen::MatrixXd m_lambda;
    Eigen::MatrixXd m_lambdaB;
    Eigen::Matrix3d m_wRb;
    std::vector<Eigen::MatrixXd> m_relJacobianInit;
    std::vector<iDynTree::Direction> m_matJetAxisInit;
    std::shared_ptr<Robot> m_robot;
    std::shared_ptr<Robot> m_robotReference;
    JointsLambda m_jointsLambda;
    TrajectoryManager m_trajectoryManager;
};

class SystemDynamicDF
{
public:
    SystemDynamicDF(const int nStates, const int nJoints, const int nThrottle);
    ~SystemDynamicDF() = default;

    const bool configure(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput);

    const bool updateDynamicMatrices(QPInput& qpInput);
    const bool getAMatrix(Eigen::Ref<Eigen::MatrixXd> A);
    const bool getBJointsMatrix(Eigen::Ref<Eigen::MatrixXd> B);
    const bool getBThrottleMatrix(Eigen::Ref<Eigen::MatrixXd> B);
    const bool getCVector(Eigen::Ref<Eigen::VectorXd> c);

private:
    int m_nStates;
    int m_nJoints;
    int m_nThrottle;
    Eigen::MatrixXd m_A;
    Eigen::MatrixXd m_BJoints;
    Eigen::MatrixXd m_BThrottle;
    Eigen::VectorXd m_c;
    std::vector<std::unique_ptr<DynamicTemplateVariableSampling>> m_vectorDynamic;
};

#endif // SYSTEM_DYNAMIC_DF_H
