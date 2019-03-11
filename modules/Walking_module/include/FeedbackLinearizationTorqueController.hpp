/**
 * @file TaskBasedTorqueSolver.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef FEEDBACK_LINEARIZATION_TORQUE_CONTROLLER_HPP
#define FEEDBACK_LINEARIZATION_TORQUE_CONTROLLER_HPP

#include <iDynTree/Core/SpatialMomentum.h>

#include <iDynTree/Core/Wrench.h>
#include <iDynTree/Core/Twist.h>

#include <WalkingConstraint.hpp>

#include <TimeProfiler.hpp>

class FeedbackLinearizationTorqueController
{
    int m_actuatedDOFs;

    iDynTree::MatrixDynSize m_massMatrix;
    iDynTree::VectorDynSize m_generalizedBiasForces;
    iDynTree::MatrixDynSize m_selectionMatrix;

    iDynTree::MatrixDynSize m_feetJacobian;
    iDynTree::VectorDynSize m_feetBiasAcceleration;
    iDynTree::VectorDynSize m_forceGains;

    iDynTree::Wrench m_leftWrench;
    iDynTree::Wrench m_rightWrench;

    iDynTree::Twist m_leftTwist;
    iDynTree::Twist m_rightTwist;


    iDynTree::Wrench m_desiredLeftWrench;
    iDynTree::Wrench m_desiredRightWrench;

    iDynTree::VectorDynSize m_desiredTorque;

    iDynTree::VectorDynSize m_desiredJointPosition;
    iDynTree::VectorDynSize m_desiredJointVelocity;
    iDynTree::VectorDynSize m_desiredJointAcceleration;

    iDynTree::VectorDynSize m_jointPosition;
    iDynTree::VectorDynSize m_jointVelocity;

    iDynTree::VectorDynSize m_jointRegularizationKp;
    iDynTree::VectorDynSize m_jointRegularizationKd;

public:
    bool initialize(const yarp::os::Searchable& config, const int& actuatedDOFs);

    void setMassMatrix(const iDynTree::MatrixDynSize& massMatrix);

    void setGeneralizedBiasForces(const iDynTree::VectorDynSize& generalizedBiasForces);

    void setFeetJacobian(const iDynTree::MatrixDynSize& leftFootJacobian,
                         const iDynTree::MatrixDynSize& rightFootJacobian);

    void setFeetBiasAcceleration(const iDynTree::Vector6 &leftFootBiasAcceleration,
                                 const iDynTree::Vector6 &rightFootBiasAcceleration);

    void setMeasuredWrench(const iDynTree::Wrench& leftWrench, const iDynTree::Wrench& rightWrench);

    void setDesiredWrench(const iDynTree::Wrench& desiredLeftWrench,
                          const iDynTree::Wrench& desiredRightWrench);

    void setFeetVelocities(const iDynTree::Twist& left, const iDynTree::Twist& right);

    void setDesiredJointTrajectory(const iDynTree::VectorDynSize& desiredJointPosition,
                                   const iDynTree::VectorDynSize& desiredJointVelocity,
                                   const iDynTree::VectorDynSize& desiredJointAcceleration);

    void setJointState(const iDynTree::VectorDynSize& jointPosition,
                       const iDynTree::VectorDynSize& jointVelocity);

    void evaluatedDesiredTorque();

    const iDynTree::VectorDynSize& desiredTorque() const;
};

#endif
