/**
 * @file WalkingTaskBasedTorqueController.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_TORQUE_CONTROL_OSQP_HPP
#define WALKING_TORQUE_CONTROL_OSQP_HPP

#include <OsqpEigen/OsqpEigen.h>

#include <CartesianPID.hpp>
#include <WalkingConstraint.hpp>

#include <TimeProfiler.hpp>

class WalkingTaskBasedTorqueController_osqp
{
    //todo
    std::unique_ptr<TimeProfiler> m_profiler; /**< Time profiler. */

    std::unique_ptr<OsqpEigen::Solver> m_optimizer{nullptr}; /**< QP solver. */

    bool m_isSolutionEvaluated;

    int m_actuatedDOFs;
    int m_numberOfVariables; /**<Number of variables in the QP problem (# of joints + 12) */
    int m_numberOfConstraints; /**<Number of constraints in the QP problem */

    iDynTree::MatrixDynSize m_inputMatrix;

    //
    Eigen::SparseMatrix<double> m_hessianEigen;
    Eigen::VectorXd m_gradient;

    iDynTree::Triplets m_constraintMatrixTriplet;
    iDynSparseMatrix m_constraintMatrix;
    Eigen::SparseMatrix<double>  m_constraintMatrixEigen;

    Eigen::VectorXd m_upperBound;
    Eigen::VectorXd m_lowerBound;

    Eigen::VectorXd m_solution;

    // Dynamical quantities
    iDynTree::MatrixDynSize m_massMatrix; /**< Mass matrix. */
    iDynTree::VectorDynSize m_generalizedBiasForces; /**< Generalized bias forces vector. */

    // Joint task
    iDynSparseMatrix m_jointRegularizationHessian;
    iDynSparseMatrix m_jointRegularizationGradient;

    iDynTree::VectorDynSize m_jointRegularizationWeights; /**< Vector containing the joint
                                                             regularization weights. */
    iDynTree::VectorDynSize m_jointRegularizationProportionalGains; /**< Vector containing the joint
                                                                       regularization gains
                                                                       (proportional). */
    iDynTree::VectorDynSize m_jointRegularizationDerivativeGains; /**< Vector containing the joint
                                                                     regularization gains
                                                                     (derivative). */

    iDynTree::VectorDynSize m_desiredJointPosition;
    iDynTree::VectorDynSize m_desiredJointVelocity;
    iDynTree::VectorDynSize m_desiredJointAcceleration;

    iDynTree::VectorDynSize m_jointPosition;
    iDynTree::VectorDynSize m_jointVelocity;

    iDynTree::VectorDynSize m_desiredJointAccelerationController;

    // TODO move in a class
    // Neck task
    std::unique_ptr<RotationalPID> m_neckOrientationController; /**< Pointer to the neck orientation
                                                                   controller */
    iDynTree::Rotation m_additionalRotation; /**< Additional rotation matrix (it is useful to rotate
                                                the desiredNeckOrientation rotation matrix). */
    iDynTree::Vector3 m_neckBiasAcceleration; /**< Neck bias acceleration \f$\dot{J} \nu \f$
                                                 (angular part). */

    iDynSparseMatrix m_neckHessian;
    iDynTree::MatrixDynSize m_neckHessianSubMatrix;
    iDynTree::Triplets m_neckHessianSubMatrixTriplets;

    iDynSparseMatrix m_neckGradient;
    iDynTree::MatrixDynSize m_neckGradientSubMatrix;
    iDynTree::Triplets m_neckGradientSubMatrixTriplets;

    iDynTree::MatrixDynSize m_neckJacobian; /**< Neck jacobian (mixed representation). */
    iDynTree::VectorDynSize m_neckOrientationWeight; /**< Neck weight matrix. */

    // this term is should be embedded somewhere
    iDynTree::MatrixDynSize m_leftFootJacobian;
    iDynTree::MatrixDynSize m_rightFootJacobian;

    // regularization task (input)
    iDynSparseMatrix m_inputRegularizationHessian;
    iDynSparseMatrix m_inputRegularizationGradient;

    // feet
    iDynTree::Transform m_leftFootToWorldTransform;
    iDynTree::Transform m_rightFootToWorldTransform;
    iDynTree::VectorDynSize m_leftFootBiasAcceleration;
    iDynTree::VectorDynSize m_rightFootBiasAcceleration;

    // com
    iDynTree::MatrixDynSize m_comJacobian;
    iDynTree::VectorDynSize m_comBiasAcceleration;

    // todo remove me
    iDynTree::Rotation m_desiredNeckOrientation;

    std::unordered_map<std::string, std::shared_ptr<Constraint>> m_constraints;


    bool instantiateCoMConstraint(const yarp::os::Searchable& config);

    bool instantiateFeetConstraint(const yarp::os::Searchable& config);

    void instantiateZMPConstraint();

    void instantiateSystemDynamicsConstraint();


    bool instantiateContactForcesConstraint(const yarp::os::Searchable& config);

    bool instantiateNeckSoftConstraint(const yarp::os::Searchable& config);

    bool instantiateRegularizationTaskConstraint(const yarp::os::Searchable& config);

    bool instantiateInputRegularizationConstraint(const yarp::os::Searchable& config);

    void instantiateConstPartInputMatrix();

    bool setHessianMatrix();

    bool setGradientVector();

    bool setLinearConstraintMatrix();

    bool setBounds();

    bool isSolutionFeasible();

public:
    bool initialize(const yarp::os::Searchable& config,
                    const int& actuatedDOFs,
                    const iDynTree::VectorDynSize& minJointTorque,
                    const iDynTree::VectorDynSize& maxJointTorque);


    bool setInitialValues(const iDynTree::VectorDynSize& jointTorque,
                          const iDynTree::Wrench& leftWrench,
                          const iDynTree::Wrench& rightWrench);

    bool setMassMatrix(const iDynTree::MatrixDynSize& massMatrix);

    bool setGeneralizedBiasForces(const iDynTree::VectorDynSize& generalizedBiasForces);

    bool setDesiredJointTrajectory(const iDynTree::VectorDynSize& desiredJointPosition,
                                   const iDynTree::VectorDynSize& desiredJointVelocity,
                                   const iDynTree::VectorDynSize& desiredJointAcceleration);


    bool setInternalRobotState(const iDynTree::VectorDynSize& jointPosition,
                               const iDynTree::VectorDynSize& jointVelocity);

    void setDesiredNeckTrajectory(const iDynTree::Rotation& desiredNeckOrientation,
                                  const iDynTree::Vector3& desiredNeckVelocity,
                                  const iDynTree::Vector3& desiredNeckAcceleration);

    void setNeckState(const iDynTree::Rotation& neckOrientation,
                      const iDynTree::Twist& neckVelocity);

    bool setNeckJacobian(const iDynTree::MatrixDynSize& neckJacobian);

    void setNeckBiasAcceleration(const iDynTree::Vector6 &neckBiasAcceleration);

    bool setDesiredFeetTrajectory(const iDynTree::Transform& desiredLeftFootToWorldTransform,
                                  const iDynTree::Transform& desiredRightFootToWorldTransform,
                                  const iDynTree::Twist& leftFootTwist,
                                  const iDynTree::Twist& rightFootTwist,
                                  const iDynTree::Twist& leftFootAcceleration,
                                  const iDynTree::Twist& rightFootAcceleration);

    bool setFeetState(const iDynTree::Transform& leftFootToWorldTransform,
                      const iDynTree::Twist& leftFootTwist,
                      const iDynTree::Transform& rightFootToWorldTransform,
                      const iDynTree::Twist& rightFootTwist);

    bool setFeetJacobian(const iDynTree::MatrixDynSize& leftFootJacobian,
                         const iDynTree::MatrixDynSize& rightFootJacobian);

    void setFeetBiasAcceleration(const iDynTree::Vector6 &leftFootBiasAcceleration,
                                 const iDynTree::Vector6 &rightFootBiasAcceleration);

    bool setDesiredCoMTrajectory(const iDynTree::Position& comPosition,
                                 const iDynTree::Vector3& comVelocity,
                                 const iDynTree::Vector3& comAcceleration);

    bool setCoMState(const iDynTree::Position& comPosition,
                     const iDynTree::Vector3& comVelocity);

    bool setCoMJacobian(const iDynTree::MatrixDynSize& comJacobian);

    bool setCoMBiasAcceleration(const iDynTree::Vector3 &comBiasAcceleration);

    bool setFeetState(const bool &leftInContact, const bool &rightInContact);

    bool setDesiredZMP(const iDynTree::Vector2& zmp);

    /**
     * Solve the optimization problem.
     * @return true/false in case of success/failure.
     */
    bool solve();

    /**
     * Get the solution of the optimization problem.
     * @param output joint torque
     * @return true/false in case of success/failure.
     */
    bool getSolution(iDynTree::VectorDynSize& output);

    iDynTree::Wrench getLeftWrench();

    iDynTree::Wrench getRightWrench();

    iDynTree::Vector2 getZMP();

    iDynTree::Vector3 getDesiredNeckOrientation();
};

#endif
