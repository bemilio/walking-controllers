/**
 * @file WalkingJointTorqueController.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_JOINT_TORQUE_CONTROLLER_HPP
#define WALKING_JOINT_TORQUE_CONTROLLER_HPP

// YARP
#include <yarp/os/Searchable.h>

// iDynTree
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Wrench.h>

class WalkingJointTorqueController
{
    int m_actuatedDOFs; /**< Number of actuated actuated DoF. */

    iDynTree::VectorDynSize m_computedOutput;

    iDynTree::VectorDynSize m_kp;
    iDynTree::VectorDynSize m_kd;

    iDynTree::VectorDynSize m_jointPosition;
    iDynTree::VectorDynSize m_jointVelocity;

    iDynTree::VectorDynSize m_desiredJointPosition;
    iDynTree::VectorDynSize m_desiredJointVelocity;

    iDynTree::Wrench m_leftWrench; /**< iDynTree vector that contains left foot wrench. */
    iDynTree::Wrench m_rightWrench; /**< iDynTree vector that contains right foot wrench. */

    // Kinematics quantities
    iDynTree::MatrixDynSize m_leftFootJacobianJoint; /**< Left foot Jacobian expressed using mixed
                                                        representation (it contains only the joint
                                                        submatrix). */
    iDynTree::MatrixDynSize m_leftFootJacobianBase; /**< Left foot Jacobian expressed using mixed
                                                       representation (it contains only the base
                                                       submatrix). */
    iDynTree::MatrixDynSize m_rightFootJacobianJoint; /**< Right foot Jacobian expressed using mixed
                                                         representation (it contains only the joint
                                                         submatrix). */
    iDynTree::MatrixDynSize m_rightFootJacobianBase; /**< Right foot Jacobian expressed using mixed
                                                        representation (it contains only the base
                                                        submatrix). */
    // Dynamics quantities
    iDynTree::MatrixDynSize m_massMatrixJoint; /**< Mass matrix containing the joint term (M_s). */
    iDynTree::MatrixDynSize m_massMatrixBase; /**< Mass matrix containing the base term (M_b). */
    iDynTree::MatrixDynSize m_massMatrixJointBase; /**< Mass matrix containing the coupling term (M_sb). */
    iDynTree::MatrixDynSize m_massMatrixBaseJoint; /**< Mass matrix containing the coupling term (M_bs). */

    iDynTree::VectorDynSize m_generalizedBiasForcesJoint; /**< Vector of generalized bias (gravity+coriolis) forces (only the joint term, h_s). */
    iDynTree::VectorDynSize m_generalizedBiasForcesBase; /**< Vector of generalized bias (gravity+coriolis) forces (only the base term, h_b). */

    double m_normalForceThreshold;

public:

    /**
     * Initialize the QP-IK problem.
     * @param config config of the QP-IK solver;
     * @param actuatedDOFs number of actuated DoF;
     * @return true/false in case of success/failure.
     */
    bool initialize(const yarp::os::Searchable& config,
                    const int& actuatedDOFs);

    /**
     * Set the robot state.
     * @param jointPosition vector of joint positions (in rad);
     * @param jointVelocity vector of joint velocity (in rad/sec);
     * @param leftWrench left foot contact wrench
     * @param rightWrench left foot contact wrench
     * @return true/false in case of success/failure.
     */
    bool setRobotState(const iDynTree::VectorDynSize& jointPosition,
                       const iDynTree::VectorDynSize& jointVelocity,
                       const iDynTree::Wrench& leftWrench,
                       const iDynTree::Wrench& rightWrench);


    /**
     * Set the Jacobian of the left foot
     * @param leftFootJacobian jacobian of the left foot (mixed representation)
     * @return true/false in case of success/failure.
     */
    bool setLeftFootJacobian(const iDynTree::MatrixDynSize& leftFootJacobian);

    /**
     * Set the Jacobian of the right foot
     * @param rightFootJacobian jacobian of the right foot (mixed representation)
     * @return true/false in case of success/failure.
     */
    bool setRightFootJacobian(const iDynTree::MatrixDynSize& rightFootJacobian);

    /**
     * Set the mass matrix
     * @oaram massMatrix mass matrix of the dynamic system
     * @return true/false in case of success/failure.
     */
    bool setMassMatrix(const iDynTree::MatrixDynSize& massMatrix);

    /**
     * Set the generalized bias forces
     * @oaram generalizedBiasForces generalized bias forces (gravity+coriolis)
     * @return true/false in case of success/failure.
     */
    bool setGeneralizedBiasForces(const iDynTree::VectorDynSize& generalizedBiasForces);

    /**
     * Set desired joint values
     * @param jointPosition desired joint position;
     * @param jointVelocity desired joint velocity
     * @return true/false in case of success/failure.
     */
    bool setDesiredJointValues(const iDynTree::VectorDynSize& desiredJointPosition,
                               const iDynTree::VectorDynSize& desiredJointVelocity);

    /**
     * Evaluate the control output.
     * @return true/false in case of success/failure
     */
    bool evaluateControl();

    /**
     * Get the controller output.
     * @param controllerOutput is the desired joint torque .
     * @return true/false in case of success/failure
     */
    bool getControllerOutput(iDynTree::VectorDynSize& controllerOutput);

    iDynTree::VectorDynSize getJointError();
};

#endif
