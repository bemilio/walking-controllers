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

#include <Utils.hpp>
#include <CartesianPID.hpp>



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
    virtual void evaluateJacobian() = 0;

    /**
     * Evaluate Hessian.
     */
    virtual void evaluateHessian() = 0;

    /**
     * Evaluate lower and upper bounds.
     */
    virtual void evaluateBounds() = 0;

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
    iDynTree::MatrixDynSize m_massMatrix; /**< Mass matrix. */
    iDynTree::VectorDynSize m_generalizedBiasForces; /**< Generalized bias forces
                                                        coriolis + gravitational. */
    iDynTree::VectorDynSize m_biasAcceleration; /**< Bias acceleration J \nu. */

    iDynTree::MatrixDynSize m_roboticJacobian; /**< Robotic Jacobian in mixed representation. */
    iDynTree::MatrixDynSize m_inputMatrix; /**< Input matrix i.e. Selection matrix for the torque. */

    iDynTree::VectorDynSize m_desiredAcceleration; /**< Desired acceleration evaluated by the
                                                      controller. */

    std::unordered_map<std::string, std::shared_ptr<CartesianPID>> m_controllers; /**< Set of
                                                                                     controllers. */
public:

    /**
     * Set the mass matrix
     * @param massMatrix mass matrix (MIXED representation).
     */
    void setMassMatrix(const iDynTree::MatrixDynSize& massMatrix){m_massMatrix = massMatrix;};

    /**
     * Set the generalized bias forces
     * @param generalizedBiasForces coriolis + gravity terms.
     */
    void setGeneralizedBiasForces(const iDynTree::VectorDynSize& generalizedBiasForces) {m_generalizedBiasForces = generalizedBiasForces;};

    /**
     * Set bias acceleration
     * @param biasAcceleration bias acceleration \f$ \dot{J} \nu $\f
     */
    void setBiasAcceleration(const iDynTree::VectorDynSize& biasAcceleration){m_biasAcceleration = biasAcceleration;};

    /**
     * Set the jacobian (robot)
     * @param roboticJacobian standard jacobian used to map the end-effector velocity to the robot velocity
     * (MIXED representation)
     */
    void setRoboticJacobian(const iDynTree::MatrixDynSize& roboticJacobian){m_roboticJacobian = roboticJacobian;};

    /**
     * Set input matrix.
     * @param inputMatrix matrix used to map the input of the system to the dynamic equation
     */
    void setInputMatrix(const iDynTree::MatrixDynSize& inputMatrix){m_inputMatrix = inputMatrix;};

    /**
     * Evaluate the constraint jacobian
     */
    void evaluateJacobian() override;

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
protected:

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
class ForceConstraint : public LinearConstraint<iDynSparseMatrix>
{
    double m_staticFrictionCoefficient; /**< Static linear coefficient of friction */
    double m_numberOfPoints; /**< Number of points in each quadrants for linearizing friction cone */
    double m_torsionalFrictionCoefficient; /**< Torsional coefficient of friction */
    double m_minimalNormalForce; /**< Minimal positive vertical force at contact */

    iDynTree::Vector2 m_footLimitX; /**< Physical size of the foot (x axis) */
    iDynTree::Vector2 m_footLimitY; /**< Physical size of the foot (y axis) */

    bool m_isJacobianEvaluated; /**< True if the Jacobian is evaluated. */
    bool m_areBoundsEvaluated; /**< True if the bounds are evaluated. */

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

    /**
     * Evaluate the jacobian
     */
    void evaluateJacobian() override;

    /**
     * Evaluate the lower and upper bounds
     */
    void evaluateBounds() override;
};

/**
 * ZMPConstraintSingleSupport class manage a nonlinear ZMP single support constraint
 */
class  ZMPConstraintSingleSupport : public Constraint<iDynTree::MatrixDynSize, iDynSparseMatrix>
{
    iDynTree::Transform m_stanceFootToWorldTransform; /**< Stance foot to world transformation */

    double m_stanceFootFZ; /**< Stance foot normal force */
    double m_stanceFootTauX; /**< Stance foot x momentum */
    double m_stanceFootTauY; /**< Stance foot y momentum */

public:

    /**
     * Constructor
     */
    ZMPConstraintSingleSupport();

    /**
     * Set the stance foot to world transformation
     * @param stanceFootToWorldTransform tranformation between the stance foot and the world frame world_H_stanceFoot
     */
    void setStanceFootToWorldTransform(const iDynTree::Transform& stanceFootToWorldTransform){m_stanceFootToWorldTransform = stanceFootToWorldTransform;};

    /**
     * Set the desired ZMP
     * @param zmp desired ZMP
     */
    void setDesiredZMP(const iDynTree::Vector2& zmp);

    /**
     * Set conditional variable.
     * @param conditionalVariable conditional variable
     */
    void setConditionalVariable(const iDynTree::VectorDynSize& conditionalVariable) override;

    /**
     * Evaluate the constraint
     */
    void evaluateConstraint() override;

    /**
     * Evaluate the jacobian
     */
    void evaluateJacobian() override;

    /**
     * Evaluate the hessian
     */
    void evaluateHessian() override;

    /**
     * Evaluate the lower and upper bounds
     */
    void evaluateBounds() override;
};

/**
 * ZMPConstraintDoubleSupport class manage a nonlinear ZMP double support constraint
 */
class ZMPConstraintDoubleSupport : public Constraint<iDynTree::MatrixDynSize, iDynSparseMatrix>
{
    iDynTree::Transform m_leftFootToWorldTransform; /**< Left foot to world tranformation world_H_leftFoot */
    iDynTree::Transform m_rightFootToWorldTransform; /**< Right foot to world tranformation world_H_rightFoot */

    double m_leftFootFZ; /**< Left foot normal force */
    double m_leftFootTauX; /**< Left foot x momentum */
    double m_leftFootTauY; /**< Left foot y momentum */

    double m_rightFootFZ; /**< Right foot normal force */
    double m_rightFootTauX; /**< Right foot x momentum */
    double m_rightFootTauY; /**< Right foot y momentum */

public:

    /**
     * Constructor
     */
    ZMPConstraintDoubleSupport();

    /**
     * Set the left foot to world transformation
     * @param leftFootToWorldTransform tranformation between the left foot and the world frame world_H_leftFoot
     */
    void setLeftFootToWorldTransform(const iDynTree::Transform& leftFootToWorldTransform){m_leftFootToWorldTransform = leftFootToWorldTransform;};

    /**
     * Set the right foot to world transformation
     * @param rightFootToWorldTransform tranformation between the right foot and the world frame world_H_rightFoot
     */
    void setRightFootToWorldTransform(const iDynTree::Transform& rightFootToWorldTransform){m_rightFootToWorldTransform = rightFootToWorldTransform;};

    /**
     * Set conditional variable.
     * @param conditionalVariable conditional variable
     */
    void setConditionalVariable(const iDynTree::VectorDynSize& conditionalVariable) override;

    /**
     * Set the desired ZMP
     * @param zmp desired ZMP
     */
    void setDesiredZMP(const iDynTree::Vector2& zmp);

    /**
     * Evaluate the constraint
     */
    void evaluateConstraint() override;

    /**
     * Evaluate the jacobian
     */
    void evaluateJacobian() override;

    /**
     * Evaluate the hessian
     */
    void evaluateHessian() override;

    /**
     * Evaluate the lower and upper bounds
     */
    void evaluateBounds() override;
};

#include <WalkingConstraint.tpp>

#endif
