/**
 * @file WalkingConstraint.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_CONSTRAINT_HPP
#define WALKING_CONSTRAINT_HPP

// std
#include <memory>
#include <unordered_map>

// iDynTree
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Transform.h>

#include <Utils.hpp>
#include <CartesianPID.hpp>

#include <TimeProfiler.hpp>

/**
 * Constraint is an abstract class that embeds the generic constraint whose form is
 * g_l <= g <= g_u
 * notice: now it only implements linear constraint (in the future it will be extended)
 */
class Constraint
{
protected:

    int m_sizeOfConstraint;

    bool m_firstTime{true};

    int m_jacobianStartingRow; /**< Staring row of the jacobian sub-matrix.*/
    int m_jacobianStartingColumn; /**< Staring column of the jacobian sub-matrix.*/

    int m_hessianStartingRow; /**< Staring row of the jacobian sub-matrix.*/
    int m_hessianStartingColumn; /**< Staring column of the jacobian submatrix.*/

    /**
     * Set the size of the constraint.
     * @param sizeOfConstraint size of the constraint.
     */
    void setSizeOfConstraint(const int& sizeOfConstraint){m_sizeOfConstraint = sizeOfConstraint;};

public:

    /**
     * Evaluate Jacobian.
     */
    virtual void evaluateJacobian(Eigen::SparseMatrix<double>& jacobian) = 0;

    /**
     * Evaluate lower and upper bounds.
     */
    virtual void evaluateBounds(Eigen::VectorXd &upperBounds, Eigen::VectorXd &lowerBounds) = 0;

    /**
     * Set the jacobian and hessian starting row and column.
     * @param staringRow staring row of the jacobian sub-block;
     * @param staringColumn staring row of the jacobian sub-block.
     */
    void setSubMatricesStartingPosition(const int& startingRow, const int& startingColumn);

    /**
     * Get the number of constraint
     */
    int getNumberOfConstraints() {return m_sizeOfConstraint;};

    int getJacobianStartingRow() {return m_jacobianStartingRow;};

    int getJacobianStartingColumn() {return m_jacobianStartingColumn;};
};

/**
 * Linear constraint class. It handles the linear constraints.
 */
class LinearConstraint : public Constraint
{
};

/**
 * GenericCartesianConstraint is an abstract class useful to manage a generic Cartesian constraint
 * i.e. foot position and orientation, CoM position.
 */
class GenericCartesianConstraint : public LinearConstraint
{
    /**
     * Evaluate the desired acceleration. It depends on the type of constraint (Positional,
     * Rotational)
     */
    virtual void evaluateDesiredAcceleration() = 0;

    bool m_isActive{true};

protected:
    iDynTree::VectorDynSize const * m_biasAcceleration; /**< Bias acceleration J \nu. */

    iDynTree::MatrixDynSize const * m_roboticJacobian; /**< Robotic Jacobian in mixed representation. */

    iDynTree::VectorDynSize m_desiredAcceleration; /**< Desired acceleration evaluated by the
                                                      controller. */

    std::unordered_map<std::string, std::shared_ptr<CartesianPID>> m_controllers; /**< Set of
                                                                                     controllers. */
public:

    /**
     * Set bias acceleration
     * @param biasAcceleration bias acceleration \f$ \dot{J} \nu $\f
     */
    void setBiasAcceleration(const iDynTree::VectorDynSize& biasAcceleration){m_biasAcceleration = &biasAcceleration;};

    /**
     * Set the jacobian (robot)
     * @param roboticJacobian standard jacobian used to map the end-effector velocity to the robot velocity
     * (MIXED representation)
     */
    void setRoboticJacobian(const iDynTree::MatrixDynSize& roboticJacobian){m_roboticJacobian = &roboticJacobian;};

    /**
     * Evaluate the constraint jacobian
     */
    void evaluateJacobian(Eigen::SparseMatrix<double>& jacobian) override;

    /**
     * Get the position controller associated to the constraint.
     * @return pointer to the controller.
     */
    std::shared_ptr<LinearPID> positionController();

    /**
     * Get the orientation controller associated to the constraint.
     * @return pointer to the controller.
     */
    std::shared_ptr<RotationalPID> orientationController();

    void activate(){m_isActive = true;};

    void deactivate(){m_isActive = false;};

    /**
     * Evaluate lower and upper bounds.
     */
    void evaluateBounds(Eigen::VectorXd &upperBounds, Eigen::VectorXd &lowerBounds) override;

};

class OneDimensionalConstraint : public GenericCartesianConstraint
{
    /**
     * Evaluate the desired acceleration
     */
    void evaluateDesiredAcceleration() override;

public:
    /**
     * Constructor
     */
    OneDimensionalConstraint();
};

class PositionConstraint : public GenericCartesianConstraint
{
    /**
     * Evaluate the desired acceleration
     */
    void evaluateDesiredAcceleration() override;

public:
    /**
     * Constructor
     */
    PositionConstraint();
};

/**
 * Cartesian Constraint class implements a Cartesian constraint
 */
class CartesianConstraint : public GenericCartesianConstraint
{
    /**
     * Evaluate the desired acceleration
     */
    void evaluateDesiredAcceleration() override;

public:

    /**
     * Constructor
     */
    CartesianConstraint();
};

/**
 * ForceConstraint class allows to obtain a contact force that satisfies the unilateral constraint,
 * the friction cone and the COP position.
 */
class ForceConstraint : public LinearConstraint
{
    bool m_isActive;

    Eigen::MatrixXd m_transform;

    double m_staticFrictionCoefficient; /**< Static linear coefficient of friction */
    double m_numberOfPoints; /**< Number of points in each quadrants for linearizing friction cone */
    double m_torsionalFrictionCoefficient; /**< Torsional coefficient of friction */
    double m_minimalNormalForce; /**< Minimal positive vertical force at contact */

    iDynTree::Vector2 m_footLimitX; /**< Physical size of the foot (x axis) */
    iDynTree::Vector2 m_footLimitY; /**< Physical size of the foot (y axis) */

    bool m_isJacobianEvaluated; /**< True if the Jacobian is evaluated. */
    bool m_areBoundsEvaluated; /**< True if the bounds are evaluated. */

    //todo
    iDynSparseMatrix m_jacobianLeftTrivialized;

    iDynTree::Transform const * m_footToWorldTransform;

public:

    /**
     * Constructor
     * @param numberOfPoints number of points used to approximated the friction cone
     */
    ForceConstraint(const int& numberOfPoints);

    /**
     * Set the static friction cone coefficient
     * @param staticFrictionCoefficient static friction coefficient.
     */
    void setStaticFrictionCoefficient(const double& staticFrictionCoefficient){m_staticFrictionCoefficient = staticFrictionCoefficient;};

    /**
     * Set the torsional friction coefficient
     * @param torsionalFrictionCoefficient torsional friction coefficient.
     */
    void setTorsionalFrictionCoefficient(const double& torsionalFrictionCoefficient){m_torsionalFrictionCoefficient = torsionalFrictionCoefficient;};

    /**
     * Set minimal normal force
     * @param minimalNormalForce minimal normal force. It has to be a positive number
     */
    void setMinimalNormalForce(const double& minimalNormalForce){m_minimalNormalForce = minimalNormalForce;};

    /**
     * Set the size of the foot
     * @param footLimitX vector containing the max and the min X coordinates
     * @param footLimitY vector containing the max and the min y coordinates
     */
    void setFootSize(const iDynTree::Vector2& footLimitX, const iDynTree::Vector2& footLimitY);

    // todo
    void setFootToWorldTransform(const iDynTree::Transform& footToWorldTransform){m_footToWorldTransform = &footToWorldTransform;};

    void activate(){m_isActive = true;};

    void deactivate(){m_isActive = false;};

    /**
     * Evaluate the jacobian
     */
    void evaluateJacobian(Eigen::SparseMatrix<double>& jacobian) override;

    /**
     * Evaluate the lower and upper bounds
     */
    void evaluateBounds(Eigen::VectorXd &upperBounds, Eigen::VectorXd &lowerBounds) override;
};

/**
 * ZMP class allows to obtain a contact force that satisfies the desired ZMP position
 */
class ZMPConstraint : public LinearConstraint
{
    iDynTree::Transform const * m_leftFootToWorldTransform;
    iDynTree::Transform const * m_rightFootToWorldTransform;

    iDynTree::Vector2 m_desiredZMP;
    bool m_areBoundsEvaluated = false;

public:

    ZMPConstraint();

    /**
     * Set the desired ZMP
     * @param zmp desired ZMP
     */
    void setDesiredZMP(const iDynTree::Vector2& zmp){m_desiredZMP = zmp;};

    /**
     * Set the left foot to world transformation
     * @param leftFootToWorldTransform tranformation between the left foot and the world frame world_H_leftFoot
     */
    void setLeftFootToWorldTransform(const iDynTree::Transform& leftFootToWorldTransform){m_leftFootToWorldTransform = &leftFootToWorldTransform;};

    /**
     * Set the right foot to world transformation
     * @param rightFootToWorldTransform tranformation between the right foot and the world frame world_H_rightFoot
     */
    void setRightFootToWorldTransform(const iDynTree::Transform& rightFootToWorldTransform){m_rightFootToWorldTransform = &rightFootToWorldTransform;};

    /**
     * Evaluate the jacobian
     */
    void evaluateJacobian(Eigen::SparseMatrix<double>& jacobian) override;

    /**
     * Evaluate the lower and upper bounds
     */
    void evaluateBounds(Eigen::VectorXd &upperBounds, Eigen::VectorXd &lowerBounds) override;
};

/**
 * Please do not use me! I am not implemented yet!
 */
class LinearMomentumConstraint : public LinearConstraint
{
    double m_robotMass;

    std::shared_ptr<LinearPID> m_controller;

public:

    LinearMomentumConstraint();

    void setRobotMass(const double& robotMass){m_robotMass = robotMass;};

    /**
     * Evaluate the jacobian
     */
    void evaluateJacobian(Eigen::SparseMatrix<double>& jacobian) override;

    /**
     * Evaluate the lower and upper bounds
     */
    void evaluateBounds(Eigen::VectorXd &upperBounds, Eigen::VectorXd &lowerBounds) override;

    std::shared_ptr<LinearPID> controller() {return m_controller;};
};

// todo
// has to be implemented
class AngularMomentumConstraint : public LinearConstraint
{
    std::shared_ptr<LinearPID> m_controller;

    iDynTree::Position const * m_comPosition; /**< . */
    iDynTree::Transform const * m_leftFootToWorldTransform; /**< Left foot to world transformation*/
    iDynTree::Transform const * m_rightFootToWorldTransform; /**< Right foot to world transformation. */

public:

    AngularMomentumConstraint();

    void setCoMPosition(const iDynTree::Position& comPosition){m_comPosition = &comPosition;};

    void setLeftFootToWorldTransform(const iDynTree::Transform& leftFootToWorldTransform){m_leftFootToWorldTransform = &leftFootToWorldTransform;};

    void setRightFootToWorldTransform(const iDynTree::Transform& rightFootToWorldTransform){m_rightFootToWorldTransform = &rightFootToWorldTransform;};

    std::shared_ptr<LinearPID> controller() {return m_controller;};

    /**
     * Evaluate the jacobian
     */
    void evaluateJacobian(Eigen::SparseMatrix<double>& jacobian) override;

    /**
     * Evaluate the lower and upper bounds
     */
    void evaluateBounds(Eigen::VectorXd &upperBounds, Eigen::VectorXd &lowerBounds) override;
};


/**
 *
 */
class SystemDynamicConstraint : public LinearConstraint
{
    iDynTree::MatrixDynSize const * m_massMatrix;
    iDynTree::MatrixDynSize const * m_leftFootJacobian;
    iDynTree::MatrixDynSize const * m_rightFootJacobian;
    iDynTree::VectorDynSize const * m_generalizedBiasForces;

    int m_systemSize;
    iDynSparseMatrix m_selectionMatrix;

public:

    SystemDynamicConstraint(const int& systemSize);

    void setLeftFootJacobian(const iDynTree::MatrixDynSize& leftFootJacobian){m_leftFootJacobian = &leftFootJacobian;};

    void setRightFootJacobian(const iDynTree::MatrixDynSize& rightFootJacobian){m_rightFootJacobian = &rightFootJacobian;};

    void setMassMatrix(const iDynTree::MatrixDynSize& massMatrix){m_massMatrix = &massMatrix;};

    void setGeneralizedBiasForces(const iDynTree::VectorDynSize& generalizedBiasForces){m_generalizedBiasForces = &generalizedBiasForces;};

    /**
     * Evaluate the constraint jacobian
     */
    void evaluateJacobian(Eigen::SparseMatrix<double>& jacobian) override;

    /**
     * Evaluate lower and upper bounds.
     */
    void evaluateBounds(Eigen::VectorXd &upperBounds, Eigen::VectorXd &lowerBounds) override;

};

#endif
