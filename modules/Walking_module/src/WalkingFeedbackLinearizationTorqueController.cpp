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

void FeedbackLinearizationTorqueController::setFeetVelocities(const iDynTree::Twist& left, const iDynTree::Twist& right)
{
    m_leftTwist = left;
    m_rightTwist = right;
}

void FeedbackLinearizationTorqueController::setDesiredWrench(const iDynTree::Wrench &desiredLeftWrench, const iDynTree::Wrench &desiredRightWrench)
{
    m_desiredLeftWrench = desiredLeftWrench;
    m_desiredRightWrench = desiredRightWrench;
}

void FeedbackLinearizationTorqueController::evaluatedDesiredTorque()
{
    auto selectionMatrix(iDynTree::toEigen(m_selectionMatrix));
    auto massMatrix(iDynTree::toEigen(m_massMatrix));
    auto generalizedBiasForce(iDynTree::toEigen(m_generalizedBiasForces));
    auto feetJacobian(iDynTree::toEigen(m_feetJacobian));
    auto feetBiasAcceleration(iDynTree::toEigen(m_feetBiasAcceleration));

    auto desiredTorque(iDynTree::toEigen(m_desiredTorque));

    auto massMatrixInverse = massMatrix.inverse();

    auto generalizedJacobianInverse = feetJacobian * massMatrixInverse * selectionMatrix;
    auto generalizedJacobianInversePseudoInverse = generalizedJacobianInverse.transpose()
        * (generalizedJacobianInverse * generalizedJacobianInverse.transpose()).inverse();

    MatrixXd identity(m_actuatedDOFs, m_actuatedDOFs);
    identity.setIdentity();
    auto nullProjection = (identity - generalizedJacobianInversePseudoInverse
                           * generalizedJacobianInverse);

    iDynTree::VectorDynSize a(12);

    yInfo() << "desired " <<m_desiredLeftWrench.toString() << " " << m_desiredRightWrench.toString();
    yInfo() << "measured " << m_leftWrench.toString() << " " << m_rightWrench.toString();

    auto c = m_desiredLeftWrench - m_leftWrench;
    auto d = m_desiredRightWrench - m_rightWrench;
    std::cerr << "error left wrench " << c.toString()<< std::endl;

    auto e = m_leftWrench - m_desiredLeftWrench;
    std::cerr << "error right wrench" << d.toString() << std::endl;

    iDynTree::toEigen(a).block(0, 0, 6, 1) = iDynTree::toEigen(m_forceGains).asDiagonal()
        * (iDynTree::toEigen(m_desiredLeftWrench - m_leftWrench))
        - iDynTree::toEigen(m_leftTwist);

    iDynTree::toEigen(a).block(6, 0, 6, 1) = iDynTree::toEigen(m_forceGains).asDiagonal()
        * (iDynTree::toEigen(m_desiredRightWrench - m_rightWrench))
        - iDynTree::toEigen(m_rightTwist);

    yInfo() << "a " << a.toString();

    iDynTree::VectorDynSize regularizationError(m_actuatedDOFs);
    iDynTree::toEigen(regularizationError) = iDynTree::toEigen(m_desiredJointAcceleration)
        + iDynTree::toEigen(m_jointRegularizationKd).asDiagonal() * (iDynTree::toEigen(m_desiredJointVelocity)
                                                                     -iDynTree::toEigen(m_jointVelocity))
        + iDynTree::toEigen(m_jointRegularizationKp).asDiagonal() * (iDynTree::toEigen(m_desiredJointPosition)
                                                                     -iDynTree::toEigen(m_jointPosition));


    iDynTree::VectorDynSize contactWrenches(12);
    iDynTree::toEigen(contactWrenches).block(0, 0, 6, 1) = iDynTree::toEigen(m_leftWrench);
    iDynTree::toEigen(contactWrenches).block(6, 0, 6, 1) = iDynTree::toEigen(m_rightWrench);

    desiredTorque = generalizedJacobianInversePseudoInverse * (feetBiasAcceleration
                                                               - feetJacobian * massMatrixInverse *
                                                               (generalizedBiasForce - feetJacobian.transpose() * iDynTree::toEigen(contactWrenches))
                                                               + iDynTree::toEigen(a))
        + nullProjection * iDynTree::toEigen(regularizationError);
}

const iDynTree::VectorDynSize& FeedbackLinearizationTorqueController::desiredTorque() const
{
    return m_desiredTorque;
}
