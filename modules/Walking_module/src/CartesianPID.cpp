/**
 * @file CartesianPID.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// iDynTree
#include <iDynTree/Core/EigenHelpers.h>

#include <CartesianPID.hpp>

void RotationalPID::setGains(const double& c0, const double& c1, const double& c2)
{
    m_c0 = c0;
    m_c1 = c1;
    m_c2 = c2;
}

void RotationalPID::setDesiredTrajectory(const iDynTree::Vector3 &desiredAcceleration,
                                         const iDynTree::Vector3 &desiredVelocity,
                                         const iDynTree::Rotation &desiredOrientation)
{
    m_desiredAcceleration = desiredAcceleration;
    m_desiredVelocity = desiredVelocity;
    m_desiredOrientation = desiredOrientation;
}

void RotationalPID::setFeedback(const iDynTree::Vector3 &velocity,
                                const iDynTree::Rotation &orientation)
{
    m_velocity = velocity;
    m_orientation = orientation;
}

void RotationalPID::evaluateControl()
{
    Eigen::Vector3d error;
    error = iDynTree::unskew(iDynTree::toEigen(m_orientation * m_desiredOrientation.inverse()));

    Eigen::Vector3d dotError;
    Eigen::Matrix3d skewAngularVelocity = iDynTree::skew(iDynTree::toEigen(m_velocity));
    dotError = iDynTree::unskew(skewAngularVelocity *
                                iDynTree::toEigen(m_orientation * m_desiredOrientation.inverse())
                                -iDynTree::toEigen(m_orientation * m_desiredOrientation.inverse()) *
                                skewAngularVelocity);

    // evaluate the control law
    iDynTree::toEigen(m_controllerOutput) = iDynTree::toEigen(m_desiredAcceleration)
        - m_c0 * dotError
        - m_c1 * (iDynTree::toEigen(m_velocity) - iDynTree::toEigen(m_desiredVelocity))
        - m_c2 * error;
}

void LinearPID::setGains(const double& kp, const double& kd)
{
    m_kp = kp;
    m_kd = kd;
}

void LinearPID::setDesiredTrajectory(const iDynTree::Vector3 &desiredAcceleration,
                                     const iDynTree::Vector3 &desiredVelocity,
                                     const iDynTree::Vector3 &desiredPosition)
{
    m_desiredAcceleration = desiredAcceleration;
    m_desiredVelocity = desiredVelocity;
    m_desiredPosition = desiredPosition;
}

void LinearPID::setFeedback(const iDynTree::Vector3 &velocity,
                            const iDynTree::Vector3 &position)
{
    m_velocity = velocity;
    m_position = position;
}

void LinearPID::evaluateControl()
{

    Eigen::Vector3d error;
    error = iDynTree::toEigen(m_desiredPosition) - iDynTree::toEigen(m_position);
    Eigen::Vector3d dotError;
    dotError = iDynTree::toEigen(m_desiredVelocity) - iDynTree::toEigen(m_velocity);

    iDynTree::toEigen(m_controllerOutput) = iDynTree::toEigen(m_desiredAcceleration) +
        m_kp * error + m_kd * dotError;
}
