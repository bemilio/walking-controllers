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
 * Matrix block helper.
 * @tparam T the only allowed type are iDynTree::MatrixDynsize and iDynTree::SparseMatrix<>
 */
template <typename T>
struct MatrixBlock
{
    int startingRow; /**< Staring row of the matrix sub-block.*/
    int startingColumn; /**< Staring column of the matrix sub-block.*/

    T matrix; /**< Matrix */

    /**
     * Constructor
     * @param rows number of rows of the matrix
     * @param columns number of columns of the matrix
     */
    MatrixBlock(const int& rows, const int& columns);
};

/**
 * Constraint is an abstract class that embeds the generic constraint whose form is
 * g_l <= g <= g_u
 * @tparam T type matrix for the jacobian matrix
 * @tparam U type matrix for the hessian matrices
 */
template <typename T, typename U>
class Constraint
{
protected:

    bool m_firstTime{true};

    iDynTree::VectorDynSize m_constraint; /**< Vector containing the constraint. */
    iDynTree::VectorDynSize m_lowerBound; /**< Vector containing the lower bound (g_l). */
    iDynTree::VectorDynSize m_upperBound; /**< Vector containing the upper bound (g_u). */

    std::shared_ptr<MatrixBlock<T>> m_jacobian; /**< Jacobian matrix. */
    std::vector<std::shared_ptr<MatrixBlock<U>>> m_hessian; /**< Vector containing the hessian matrices */

    iDynTree::VectorDynSize m_conditionalVariable; /**< Conditional variable */

    /**
     * Set the size of the constraint.
     * @param sizeOfConstraint size of the constraint.
     */
    void setSizeOfConstraint(const int& sizeOfConstraint);

public:

    /**
     * Set the conditional variable
     * @param conditionalVariable conditional variable.
     */
    virtual void setConditionalVariable(const iDynTree::VectorDynSize& conditionalVariable) {m_conditionalVariable = conditionalVariable;};

    /**
     * Evaluate constraint.
     */
    virtual void evaluateConstraint() = 0;

    /**
     * Evaluate Jacobian.
     */
    virtual void evaluateJacobian(Eigen::SparseMatrix<double>& jacobian) = 0;

    /**
     * Evaluate Hessian.
     */
    virtual void evaluateHessian() = 0;

    /**
     * Evaluate lower and upper bounds.
     */
    virtual void evaluateBounds() = 0;

    /**
     * Set the jacobian and hessian starting row and column.
     * @param staringRow staring row of the jacobian sub-block;
     * @param staringColumn staring row of the jacobian sub-block.
     */
    void setSubMatricesStartingPosition(const int& startingRow, const int& startingColumn);

    /**
     * Get the constraint vector.
     * @return constraint vector.
     */
    const iDynTree::VectorDynSize& getConstraint() const {return m_constraint;};

    /**
     * Get the jacobian matrix.
     * @return the Jacobian matrix.
     */
    const std::shared_ptr<MatrixBlock<T>>& getJacobian() const {return m_jacobian;};

    /**
     * Get the vector containing the hessian matrices
     * @return a vector contain the hessian matrices.
     */
    const std::vector<std::shared_ptr<MatrixBlock<U>>>& getHessian() const {return m_hessian;};

    /**
     * Get lower bound g_l.
     * @return lower bound
     */
    const iDynTree::VectorDynSize& getLowerBound() const {return m_lowerBound;};

    /**
     * Get upper bound g_u.
     * @return upper bound
     */
    const iDynTree::VectorDynSize& getUpperBound() const {return m_upperBound;};

    /**
     * Get the number of constraint
     */
    double getNumberOfConstraints() {return m_constraint.size();};
};

/**
 * Linear constraint class. It handles the linear constraints.
 */
template <typename T>
class LinearConstraint : public Constraint<T, iDynSparseMatrix>
{
public:
    /**
     * Evaluate the constraint.
     * Since the constrain is linear the constraint depends only on the Jacobian and on the
     * conditional variable.
     */
    void evaluateConstraint() override;

    /**
     * Since the constraint is linear the hessian matrix is 0
     */
    void evaluateHessian() override;
};

/**
 * GenericCartesianConstraint is an abstract class useful to manage a generic Cartesian constraint
 * i.e. foot position and orientation, CoM position.
 */
class GenericCartesianConstraint : public LinearConstraint<iDynTree::MatrixDynSize>
{
    /**
     * Evaluate the desired acceleration. It depends on the type of constraint (Positional,
     * Rotational)
     */
    virtual void evaluateDesiredAcceleration() = 0;

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

    /**
     * Evaluate lower and upper bounds.
     */
    void evaluateBounds() override;

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
     * @param jacobianCols number of columns of the Jacobian.
     */
    PositionConstraint(const int& jacobianCols);
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
     * @param jacobianCols number of columns of the Jacobian.
     */
    CartesianConstraint(const int& jacobianCols);
};

/**
 * ForceConstraint class allows to obtain a contact force that satisfies the unilateral constraint,
 * the friction cone and the COP position.
 */
class ForceConstraint : public LinearConstraint<iDynTree::MatrixDynSize>
{
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

    void activate();

    void deactivate();

    /**
     * Evaluate the jacobian
     */
    void evaluateJacobian(Eigen::SparseMatrix<double>& jacobian) override;

    /**
     * Evaluate the lower and upper bounds
     */
    void evaluateBounds() override;
};

/**
 * ZMP class allows to obtain a contact force that satisfies the desired ZMP position
 */
class ZMPConstraint : public LinearConstraint<iDynTree::MatrixDynSize>
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
    void evaluateBounds() override;
};

/**
 *
 */
class SystemDynamicConstraint : public LinearConstraint<iDynTree::MatrixDynSize>
{
    iDynTree::MatrixDynSize const * m_massMatrix;
    iDynTree::MatrixDynSize const * m_leftFootJacobian;
    iDynTree::MatrixDynSize const * m_rightFootJacobian;
    iDynTree::VectorDynSize const * m_generalizedBiasForces;

    int m_systemSize;

    iDynSparseMatrix m_selectionMatrix;

public:

    SystemDynamicConstraint(const int& jacobianRows, const int& jacobianCols, const int& systemSize);

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
    void evaluateBounds() override;

};

#include <WalkingConstraint.tpp>

#endif
