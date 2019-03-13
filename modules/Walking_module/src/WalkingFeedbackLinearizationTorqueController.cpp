/**
 * @file TaskBasedTorqueSolver.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <iDynTree/yarp/YARPConfigurationsLoader.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/EigenSparseHelpers.h>

#include <FeedbackLinearizationTorqueController.hpp>
#include <Utils.hpp>

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXd;

bool FeedbackLinearizationTorqueController::initialize(const yarp::os::Searchable& config,
                                                       const int& actuatedDOFs)
{
    m_actuatedDOFs = actuatedDOFs;

    // resize matrices (generic)
    m_massMatrix.resize(m_actuatedDOFs + 6, m_actuatedDOFs + 6);
    m_generalizedBiasForces.resize(m_actuatedDOFs + 6);
    m_contactJacobian.resize(6 + 6, m_actuatedDOFs + 6);
    m_feetJacobian.resize(6 + 6, m_actuatedDOFs + 6);
    m_feetBiasAcceleration.resize(6 + 6);

    m_desiredJointAcceleration.resize(m_actuatedDOFs);
    m_desiredJointVelocity.resize(m_actuatedDOFs);
    m_desiredJointPosition.resize(m_actuatedDOFs);
    m_jointVelocity.resize(m_actuatedDOFs);
    m_jointPosition.resize(m_actuatedDOFs);
    m_jointRegularizationKp.resize(m_actuatedDOFs);
    m_jointRegularizationKd.resize(m_actuatedDOFs);

    m_forceGains.resize(6);

    m_selectionMatrix.resize(m_actuatedDOFs + 6, m_actuatedDOFs);
    m_selectionMatrix.zero();
    for(int i = 0; i < m_actuatedDOFs; i++)
        m_selectionMatrix(i + 6, i) = 1;

    // results
    m_desiredTorque.resize(m_actuatedDOFs);

    // check if the config is empty
    if(config.isNull())
    {
        yError() << "[initialize] Empty configuration for feedback linearization controller solver.";
        return false;
    }

    if(!YarpHelper::getVectorDynSizeFromSearchable(config, "joint_regularization_kp",
                                                   m_jointRegularizationKp))
        return false;

    if(!YarpHelper::getVectorDynSizeFromSearchable(config, "joint_regularization_kd",
                                                   m_jointRegularizationKd))
        return false;

    if(!YarpHelper::getVectorDynSizeFromSearchable(config, "force_gains", m_forceGains))
        return false;

    iDynTree::Vector3 linearKp, linearKd;
    double angularKp, angularKd, c0;

    if(!YarpHelper::getVectorFixSizeFromSearchable(config, "linear_kp", linearKp))
        return false;

    if(!YarpHelper::getVectorFixSizeFromSearchable(config, "linear_kd", linearKd))
        return false;


    if(!YarpHelper::getNumberFromSearchable(config, "angular_kp", angularKp))
        return false;

    if(!YarpHelper::getNumberFromSearchable(config, "angular_kd", angularKd))
        return false;

    if(!YarpHelper::getNumberFromSearchable(config, "c0", c0))
        return false;


    m_linearPID.setGains(linearKp, linearKd);
    m_rotationalPID.setGains(c0, angularKd, angularKp);

    return true;
}



void FeedbackLinearizationTorqueController::setMassMatrix(const iDynTree::MatrixDynSize& massMatrix)
{
    m_massMatrix = massMatrix;
}

void FeedbackLinearizationTorqueController::setGeneralizedBiasForces(const iDynTree::VectorDynSize& generalizedBiasForces)
{
    m_generalizedBiasForces = generalizedBiasForces;
}

void FeedbackLinearizationTorqueController::setDesiredJointTrajectory(const iDynTree::VectorDynSize& desiredJointPosition,
                                                      const iDynTree::VectorDynSize& desiredJointVelocity,
                                                      const iDynTree::VectorDynSize& desiredJointAcceleration)
{
    m_desiredJointPosition = desiredJointPosition;
    m_desiredJointVelocity = desiredJointVelocity;
    m_desiredJointAcceleration = desiredJointAcceleration;
}

void FeedbackLinearizationTorqueController::setJointState(const iDynTree::VectorDynSize& jointPosition,
                                                          const iDynTree::VectorDynSize& jointVelocity)
{
    m_jointPosition = jointPosition;
    m_jointVelocity = jointVelocity;
}

void FeedbackLinearizationTorqueController::setFeetJacobian(const iDynTree::MatrixDynSize &leftFootJacobian,
                                                            const iDynTree::MatrixDynSize &rightFootJacobian)
{
    iDynTree::toEigen(m_feetJacobian).block(0, 0, 6, m_actuatedDOFs + 6) = iDynTree::toEigen(leftFootJacobian);
    iDynTree::toEigen(m_feetJacobian).block(6, 0, 6, m_actuatedDOFs + 6) = iDynTree::toEigen(rightFootJacobian);

    if(m_doubleSupport)
        m_contactJacobian = m_feetJacobian;
    else
    {
        if(m_leftInContact)
        {
            iDynTree::toEigen(m_contactJacobian).block(0, 0, 6, m_actuatedDOFs + 6) = iDynTree::toEigen(leftFootJacobian);
            iDynTree::toEigen(m_contactJacobian).block(6, 0, 6, m_actuatedDOFs + 6).setZero();
        }
        else
        {
            iDynTree::toEigen(m_contactJacobian).block(0, 0, 6, m_actuatedDOFs + 6).setZero();
            iDynTree::toEigen(m_contactJacobian).block(6, 0, 6, m_actuatedDOFs + 6) = iDynTree::toEigen(rightFootJacobian);
        }
    }
}


void FeedbackLinearizationTorqueController::setFeetBiasAcceleration(const iDynTree::Vector6 &leftFootBiasAcceleration, const iDynTree::Vector6 &rightFootBiasAcceleration)
{
    iDynTree::toEigen(m_feetBiasAcceleration).block(0, 0, 6, 1) = iDynTree::toEigen(leftFootBiasAcceleration);
    iDynTree::toEigen(m_feetBiasAcceleration).block(6, 0, 6, 1) = iDynTree::toEigen(rightFootBiasAcceleration);
}

void FeedbackLinearizationTorqueController::setMeasuredWrench(const iDynTree::Wrench& leftWrench, const iDynTree::Wrench& rightWrench)
{
    m_leftWrench = leftWrench;
    m_rightWrench = rightWrench;
}

void FeedbackLinearizationTorqueController::setFeetState(const iDynTree::Transform& leftFootToWorldTransform,
                                                      const iDynTree::Twist& leftFootTwist,
                                                      const iDynTree::Transform& rightFootToWorldTransform,
                                                      const iDynTree::Twist& rightFootTwist)

{
    m_leftTwist = leftFootTwist;
    m_rightTwist = rightFootTwist;

    if(m_doubleSupport)
        return;

    const iDynTree::Twist &swingFootVelocity = rightFootTwist;
    const iDynTree::Transform &swingFootTranformation = rightFootToWorldTransform;

    if(!m_leftInContact)
    {
        const iDynTree::Twist &swingFootVelocity = leftFootTwist;
        const iDynTree::Transform &swingFootTranformation = leftFootToWorldTransform;
    }

    m_linearPID.setFeedback(swingFootVelocity.getLinearVec3(),
                            swingFootTranformation.getPosition());

    m_rotationalPID.setFeedback(swingFootVelocity.getAngularVec3(),
                                swingFootTranformation.getRotation());
}

void FeedbackLinearizationTorqueController::setDesiredFeetTrajectory(const iDynTree::Transform& leftFootToWorldTransform,
                                                                     const iDynTree::Twist& leftFootTwist,
                                                                     const iDynTree::Vector6& leftFootAcceleration,
                                                                     const iDynTree::Transform& rightFootToWorldTransform,
                                                                     const iDynTree::Twist& rightFootTwist,
                                                                     const iDynTree::Vector6& rightFootAcceleration)
{
    if(m_doubleSupport)
        return;

    const iDynTree::Vector6 &swingFootAcceleration = rightFootAcceleration;
    const iDynTree::Twist &swingFootVelocity = rightFootTwist;
    const iDynTree::Transform &swingFootTranformation = rightFootToWorldTransform;

    if(!m_leftInContact)
    {
        const iDynTree::Vector6 &swingFootAcceleration = leftFootAcceleration;
        const iDynTree::Twist &swingFootVelocity = leftFootTwist;
        const iDynTree::Transform &swingFootTranformation = leftFootToWorldTransform;
    }

    iDynTree::Vector3 swingFootLinearAcceleration, swingFootAngularAcceleration;
    swingFootLinearAcceleration(0) = swingFootAcceleration(0);
    swingFootLinearAcceleration(1) = swingFootAcceleration(1);
    swingFootLinearAcceleration(2) = swingFootAcceleration(2);

    swingFootAngularAcceleration(0) = swingFootAcceleration(3);
    swingFootAngularAcceleration(1) = swingFootAcceleration(4);
    swingFootAngularAcceleration(2) = swingFootAcceleration(5);


    m_linearPID.setDesiredTrajectory(swingFootLinearAcceleration,
                                     swingFootVelocity.getLinearVec3(),
                                     swingFootTranformation.getPosition());

    m_rotationalPID.setDesiredTrajectory(swingFootAngularAcceleration,
                                      swingFootVelocity.getAngularVec3(),
                                      swingFootTranformation.getRotation());
}

void FeedbackLinearizationTorqueController::setDesiredWrench(const iDynTree::Wrench &desiredLeftWrench, const iDynTree::Wrench &desiredRightWrench)
{
    m_desiredLeftWrench = desiredLeftWrench;
    m_desiredRightWrench = desiredRightWrench;
}

void FeedbackLinearizationTorqueController::setFeetState(bool leftInContact, bool rightInContact)
{
    if(leftInContact && rightInContact)
        m_doubleSupport = true;
    else
        m_doubleSupport = false;

    m_leftInContact = leftInContact;
    m_rightInContact = rightInContact;
}

void FeedbackLinearizationTorqueController::evaluatedDesiredTorque()
{
    auto selectionMatrix(iDynTree::toEigen(m_selectionMatrix));
    auto massMatrix(iDynTree::toEigen(m_massMatrix));
    auto generalizedBiasForce(iDynTree::toEigen(m_generalizedBiasForces));
    auto feetJacobian(iDynTree::toEigen(m_feetJacobian));
    auto contactJacobian(iDynTree::toEigen(m_contactJacobian));
    auto feetBiasAcceleration(iDynTree::toEigen(m_feetBiasAcceleration));

    auto M_b = massMatrix.block(0,0,6,6);
    auto M_b_inverse = M_b.inverse();
    auto M_s = massMatrix.block(6,6,m_actuatedDOFs,m_actuatedDOFs);
    auto M_bs = massMatrix.block(0,6,6,m_actuatedDOFs);
    auto M_inverse = massMatrix.inverse();

    auto h_b = generalizedBiasForce.block(0,0,6,1);
    auto h_s = generalizedBiasForce.block(6,0,m_actuatedDOFs,1);

    auto J_c_b_transpose = contactJacobian.transpose().block(0, 0, 6, 12);
    auto J_c_s_transpose = contactJacobian.transpose().block(6, 0, m_actuatedDOFs, 12);

    Eigen::VectorXd contactWrenches(12);
    contactWrenches.block(0, 0, 6, 1) = iDynTree::toEigen(m_desiredLeftWrench);
    contactWrenches.block(6, 0, 6, 1) = iDynTree::toEigen(m_desiredRightWrench);

    auto desiredTorque(iDynTree::toEigen(m_desiredTorque));

    MatrixXd identity12(12, 12);
    identity12.setIdentity();

    yInfo() << "sdafgidiugdiugvuias" << m_doubleSupport;

    auto lambda = feetJacobian * M_inverse * selectionMatrix;
    auto lambdaPseudoInverse = lambda.transpose() * (lambda * lambda.transpose() + identity12).inverse();

    MatrixXd identity(m_actuatedDOFs, m_actuatedDOFs);
    identity.setIdentity();
    auto nullProjection = (identity - lambdaPseudoInverse * lambda);

    Eigen::VectorXd a(12);
    if(m_doubleSupport)
    {
        a.block(0, 0, 6, 1) = iDynTree::toEigen(m_forceGains).asDiagonal()
            * (iDynTree::toEigen(m_desiredLeftWrench - m_leftWrench))
            - 0.0 * iDynTree::toEigen(m_leftTwist);

        a.block(6, 0, 6, 1) = iDynTree::toEigen(m_forceGains).asDiagonal()
            * (iDynTree::toEigen(m_desiredRightWrench - m_rightWrench))
            - 0.0 * iDynTree::toEigen(m_rightTwist);
    }
    else
    {
        if(m_leftInContact)
        {
            a.block(0, 0, 6, 1) = iDynTree::toEigen(m_forceGains).asDiagonal()
                * (iDynTree::toEigen(m_desiredLeftWrench - m_leftWrench))
                - 0.0 * iDynTree::toEigen(m_leftTwist);

            m_linearPID.evaluateControl();
            a.block(6, 0, 3, 1) = iDynTree::toEigen(m_linearPID.getControllerOutput());

            m_rotationalPID.evaluateControl();
            a.block(9, 0, 3, 1) = iDynTree::toEigen(m_rotationalPID.getControllerOutput());
        }
        else
        {
            m_linearPID.evaluateControl();
            a.block(0, 0, 3, 1) = iDynTree::toEigen(m_linearPID.getControllerOutput());

            m_rotationalPID.evaluateControl();
            a.block(3, 0, 3, 1) = iDynTree::toEigen(m_rotationalPID.getControllerOutput());

            a.block(6, 0, 6, 1) = iDynTree::toEigen(m_forceGains).asDiagonal()
                * (iDynTree::toEigen(m_desiredRightWrench - m_rightWrench))
                - 0.0 * iDynTree::toEigen(m_rightTwist);
        }
    }
    std::cerr << "contact wrenches "<< a.transpose() << std::endl;

    Eigen::VectorXd regularizationError(m_actuatedDOFs);
    regularizationError = iDynTree::toEigen(m_desiredJointAcceleration)
        + iDynTree::toEigen(m_jointRegularizationKd).asDiagonal() * (iDynTree::toEigen(m_desiredJointVelocity)
                                                                     -iDynTree::toEigen(m_jointVelocity))
        + iDynTree::toEigen(m_jointRegularizationKp).asDiagonal() * (iDynTree::toEigen(m_desiredJointPosition)
                                                                     -iDynTree::toEigen(m_jointPosition));

    desiredTorque = lambdaPseudoInverse * (-feetBiasAcceleration + feetJacobian * M_inverse *
                                           (generalizedBiasForce - contactJacobian.transpose() * contactWrenches) + a)
        + nullProjection * (h_s - M_bs.transpose() * M_b_inverse * h_b - (J_c_s_transpose - M_bs.transpose() * M_b_inverse * J_c_b_transpose)
                            * contactWrenches + (M_s - M_bs.transpose() * M_b_inverse * M_bs) * regularizationError);
}

const iDynTree::VectorDynSize& FeedbackLinearizationTorqueController::desiredTorque() const
{
    return m_desiredTorque;
}
