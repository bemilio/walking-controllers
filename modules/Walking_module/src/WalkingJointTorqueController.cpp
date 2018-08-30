/**
 * @file WalkingJointTorqueController.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// YARP
#include <yarp/os/LogStream.h>

// iDynTree
#include <iDynTree/Core/EigenHelpers.h>

#include <WalkingJointTorqueController.hpp>
#include <Utils.hpp>

bool WalkingJointTorqueController::initialize(const yarp::os::Searchable& config,
                                              const int& actuatedDOFs)
{
    m_actuatedDOFs = actuatedDOFs;

    // check if the config is empty
    if(config.isNull())
    {
        yError() << "[WalkingJointTorqueController::initialize] Empty configuration for "
                 << " joint torque controller.";
        return false;
    }

    // resize all vectors and matrices
    m_computedOutput.resize(m_actuatedDOFs);

    m_kp.resize(m_actuatedDOFs);
    m_kd.resize(m_actuatedDOFs);
    m_jointPosition.resize(m_actuatedDOFs);
    m_jointVelocity.resize(m_actuatedDOFs);
    m_desiredJointPosition.resize(m_actuatedDOFs);
    m_desiredJointVelocity.resize(m_actuatedDOFs);

    m_leftFootJacobianJoint.resize(6, m_actuatedDOFs);
    m_leftFootJacobianBase.resize(6, 6);
    m_rightFootJacobianJoint.resize(6, m_actuatedDOFs);
    m_rightFootJacobianBase.resize(6, 6);

    m_massMatrixJoint.resize(m_actuatedDOFs, m_actuatedDOFs);
    m_massMatrixBase.resize(6, 6);
    m_massMatrixJointBase.resize(m_actuatedDOFs, 6);
    m_massMatrixBaseJoint.resize(6, m_actuatedDOFs);

    m_generalizedBiasForcesJoint.resize(m_actuatedDOFs);
    m_generalizedBiasForcesBase.resize(6);

    // read  value from configuration file
    yarp::os::Value tempValue;
    tempValue = config.find("jointProportionalGains");
    if(!YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, m_kp))
    {
        yError() << "Initialization failed while reading jointProportionalGains vector.";
        return false;
    }

    tempValue = config.find("jointDerivativeGains");
    if(!YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, m_kd))
    {
        yError() << "Initialization failed while reading jointDerivativeGains vector.";
        return false;
    }

    // move in config file
    m_normalForceThreshold = 10;
    // if(!YarpHelper::getDoubleFromSearchable(config, "normalForceThreshold", m_normalForceThreshold))
    // {
    //     yError() << "Initialization failed while reading normalForceThreshold.";
    //     return false;
    // }

    return true;
}

bool WalkingJointTorqueController::setRobotState(const iDynTree::VectorDynSize& jointPosition,
                                                 const iDynTree::VectorDynSize& jointVelocity,
                                                 const iDynTree::Wrench& leftWrench,
                                                 const iDynTree::Wrench& rightWrench)
{
    if(jointPosition.size() != m_actuatedDOFs)
    {
        yError() << "[WalkingJointTorqueController::setRobotState] The size of the jointPosition "
                 << " vector is not coherent with the number of the actuated Joint";
        return false;
    }

    if(jointVelocity.size() != m_actuatedDOFs)
    {
        yError() << "[WalkingJointTorqueController::setRobotState] The size of the jointVelocity "
                 << " vector is not coherent with the number of the actuated Joint";
        return false;
    }

    m_jointPosition = jointPosition;
    m_jointVelocity = jointVelocity;

    m_leftWrench = leftWrench;
    m_rightWrench = rightWrench;

    return true;
}

bool WalkingJointTorqueController::setLeftFootJacobian(const iDynTree::MatrixDynSize& leftFootJacobian)
{
    if(leftFootJacobian.rows() != 6)
    {
        yError() << "[WalkingJointTorqueController::setLeftFootJacobian] The number of rows has to "
                 << "be equal to 6.";
        return false;
    }
    if(leftFootJacobian.cols() != m_actuatedDOFs + 6)
    {
        yError() << "[WalkingJointTorqueController::setLeftFootJacobian] the number of rows has to "
                 << "be equal to" << m_actuatedDOFs + 6;
        return false;
    }

    iDynTree::toEigen(m_leftFootJacobianBase) = iDynTree::toEigen(leftFootJacobian).block(0, 0,
                                                                                          6, 6);

    iDynTree::toEigen(m_leftFootJacobianJoint) = iDynTree::toEigen(leftFootJacobian).block(0, 6,
                                                                                           6, m_actuatedDOFs);

    return true;
}

bool WalkingJointTorqueController::setRightFootJacobian(const iDynTree::MatrixDynSize& rightFootJacobian)
{
    if(rightFootJacobian.rows() != 6)
    {
        yError() << "[WalkingJointTorqueController::setRightFootJacobian] The number of rows has to "
                 << "be equal to 6.";
        return false;
    }
    if(rightFootJacobian.cols() != m_actuatedDOFs + 6)
    {
        yError() << "[WalkingJointTorqueController::setRightFootJacobian] the number of columns has "
                 << "to be equal to" << m_actuatedDOFs + 6;
        return false;
    }

    iDynTree::toEigen(m_rightFootJacobianBase) = iDynTree::toEigen(rightFootJacobian).block(0, 0,
                                                                                            6, 6);

    iDynTree::toEigen(m_rightFootJacobianJoint) = iDynTree::toEigen(rightFootJacobian).block(0, 6,
                                                                                             6, m_actuatedDOFs);

    return true;
}

bool WalkingJointTorqueController::setMassMatrix(const iDynTree::MatrixDynSize& massMatrix)
{
    if(massMatrix.rows() != m_actuatedDOFs + 6)
    {
        yError() << "[WalkingJointTorqueController::setMassMatrix] The number of rows has to "
                 << "be equal to" << m_actuatedDOFs + 6;
        return false;
    }

    if(massMatrix.cols() != m_actuatedDOFs + 6)
    {
        yError() << "[WalkingJointTorqueController::setMassMatrix] The number of columns has to "
                 << "be equal to" << m_actuatedDOFs + 6;
        return false;
    }

    iDynTree::toEigen(m_massMatrixBase) = iDynTree::toEigen(massMatrix).block(0, 0, 6, 6);
    iDynTree::toEigen(m_massMatrixJoint) = iDynTree::toEigen(massMatrix).block(6, 6,
                                                                               m_actuatedDOFs,
                                                                               m_actuatedDOFs);
    iDynTree::toEigen(m_massMatrixJointBase) = iDynTree::toEigen(massMatrix).block(6, 0,
                                                                                   m_actuatedDOFs,
                                                                                   6);
    iDynTree::toEigen(m_massMatrixBaseJoint) = iDynTree::toEigen(massMatrix).block(0, 6,
                                                                                   6,
                                                                                   m_actuatedDOFs);
    return true;
}

bool WalkingJointTorqueController::setGeneralizedBiasForces(const iDynTree::VectorDynSize& generalizedBiasForces)
{
    if(generalizedBiasForces.size() != m_actuatedDOFs + 6)
    {
        yError() << "[WalkingJointTorqueController::setGeneralizedBiasForces] The size has to be "
                 << "equal to " << m_actuatedDOFs + 6;
        return false;
    }

    iDynTree::toEigen(m_generalizedBiasForcesBase) = iDynTree::toEigen(generalizedBiasForces).block(0, 0, 6, 1);

    iDynTree::toEigen(m_generalizedBiasForcesJoint) = iDynTree::toEigen(generalizedBiasForces).block(6, 0, m_actuatedDOFs, 1);
    return true;
}

bool WalkingJointTorqueController::setDesiredJointValues(const iDynTree::VectorDynSize& desiredJointPosition,
                                                         const iDynTree::VectorDynSize& desiredJointVelocity)
{
    if(desiredJointPosition.size() != m_actuatedDOFs)
    {
        yError() << "[WalkingJointTorqueController::setDesiredJointValues] The size of the "
                 << "jointPosition vector is not coherent with the number of the actuated Joint";
        return false;
    }

    if(desiredJointVelocity.size() != m_actuatedDOFs)
    {
        yError() << "[WalkingJointTorqueController::setDesiredJointValues] The size of the "
                 << "jointVelocity vector is not coherent with the number of the actuated Joint";
        return false;
    }

    m_desiredJointPosition = desiredJointPosition;
    m_desiredJointVelocity = desiredJointVelocity;

    return true;
}

bool WalkingJointTorqueController::evaluateControl()
{
    iDynTree::VectorDynSize fakeControl(m_actuatedDOFs);
    for(int i = 0; i < m_actuatedDOFs; i++)
        fakeControl(i) = m_kp(i) * (m_desiredJointPosition(i) - m_jointPosition(i)) +
            m_kd(i) * (m_desiredJointVelocity(i) - m_jointVelocity(i));


    // contact wrenches
    iDynTree::VectorDynSize contactWrenches(m_actuatedDOFs);
    contactWrenches.zero();
    if(m_rightWrench.getLinearVec3()(2) > m_normalForceThreshold)
    {
        // keep in mind the possibility to use pseudo inverse instead of standard inverse
        iDynTree::toEigen(contactWrenches) += -iDynTree::toEigen(m_rightFootJacobianJoint).transpose()
            * iDynTree::toEigen(m_rightWrench)
            + iDynTree::toEigen(m_massMatrixJointBase) * iDynTree::toEigen(m_massMatrixBase).inverse()
            * iDynTree::toEigen(m_rightFootJacobianBase).transpose()
            * iDynTree::toEigen(m_rightWrench);

    }

    if(m_leftWrench.getLinearVec3()(2) > m_normalForceThreshold)
    {
        // keep in mind the possibility to use pseudo inverse instead of standard inverse
        iDynTree::toEigen(contactWrenches) += -iDynTree::toEigen(m_leftFootJacobianJoint).transpose()
            * iDynTree::toEigen(m_leftWrench)
            + iDynTree::toEigen(m_massMatrixJointBase) * iDynTree::toEigen(m_massMatrixBase).inverse()
            * iDynTree::toEigen(m_leftFootJacobianBase).transpose()
            * iDynTree::toEigen(m_leftWrench);
    }

    // gravitational plus coriolis
    // keep in mind the possibility to use pseudo inverse instead of standard inverse
    iDynTree::VectorDynSize generalzedBiasForces(m_actuatedDOFs);
    iDynTree::toEigen(generalzedBiasForces) = iDynTree::toEigen(m_generalizedBiasForcesJoint)
        - iDynTree::toEigen(m_massMatrixJointBase) * iDynTree::toEigen(m_massMatrixBase).inverse()
        * iDynTree::toEigen(m_generalizedBiasForcesBase);

    // mass matrix
    iDynTree::MatrixDynSize massMatrix(m_actuatedDOFs, m_actuatedDOFs);
    iDynTree::toEigen(massMatrix) = iDynTree::toEigen(m_massMatrixJoint)
        - iDynTree::toEigen(m_massMatrixJointBase) * iDynTree::toEigen(m_massMatrixBase).inverse()
        * iDynTree::toEigen(m_massMatrixBaseJoint);


    // evaluate the joint torque
    iDynTree::toEigen(m_computedOutput) = iDynTree::toEigen(contactWrenches)
        + iDynTree::toEigen(generalzedBiasForces)
        + iDynTree::toEigen(massMatrix) * iDynTree::toEigen(fakeControl);


    return true;
}

bool WalkingJointTorqueController::getControllerOutput(iDynTree::VectorDynSize& controllerOutput)
{
    controllerOutput = m_computedOutput;
    return true;
}

iDynTree::VectorDynSize WalkingJointTorqueController::getJointError()
{
    iDynTree::VectorDynSize jointError(m_actuatedDOFs);
    iDynTree::toEigen(jointError) = iDynTree::toEigen(m_desiredJointPosition) - iDynTree::toEigen(m_jointPosition);

    return jointError;

}
