/**
 * @file WalkingConstraint.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// iDynTree
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/EigenSparseHelpers.h>

#include <WalkingConstraint.hpp>

void Constraint::setSubMatricesStartingPosition(const int& startingRow, const int& startingColumn)
{
    // set the jacobian starting raw and column
    m_jacobianStartingRow = startingRow;
    m_jacobianStartingColumn = startingColumn;

    // set the hessian staryint row and column.
    // it is important to notice that the m_hessian is a vector containing the hessian matrices
    // (for a nonlinear constraints the number depends on the constraints)
    // we suppose that m_vector has been already populated (notice in general it occurs in the
    // construct of the derived classes)
    m_hessianStartingRow = startingRow;

    // Notice this is not an error. The hessian matrix is quadratic and the blocks
    // different from zero depends on the function between the constraint and the conditional
    // variables
    m_hessianStartingColumn = startingRow;
}

void GenericCartesianConstraint::evaluateJacobian(Eigen::SparseMatrix<double>& jacobian)
{
    if(!m_firstTime)
    {
        for(int i = 0; i < m_roboticJacobian->rows(); i++)
            for(int j = 0; j < m_roboticJacobian->cols(); j++)
                jacobian.coeffRef(m_jacobianStartingRow + i,
                                  m_jacobianStartingColumn + j) = (*m_roboticJacobian)(i,j);
    }
    else
    {
        for(int i = 0; i < m_roboticJacobian->rows(); i++)
            for(int j = 0; j < m_roboticJacobian->cols(); j++)
                jacobian.insert(m_jacobianStartingRow + i,
                                m_jacobianStartingColumn + j) = (*m_roboticJacobian)(i,j);
        m_firstTime = false;
    }
}

std::shared_ptr<LinearPID> GenericCartesianConstraint::positionController()
{
    std::unordered_map<std::string, std::shared_ptr<CartesianPID>>::const_iterator controller;
    controller = m_controllers.find("position_pid");

    if(controller == m_controllers.end())
        return nullptr;

    return std::static_pointer_cast<LinearPID>(controller->second);
}

std::shared_ptr<RotationalPID> GenericCartesianConstraint::orientationController()
{
    std::unordered_map<std::string, std::shared_ptr<CartesianPID>>::const_iterator controller;
    controller = m_controllers.find("orientation_pid");

    if(controller == m_controllers.end())
        return nullptr;

    return std::static_pointer_cast<RotationalPID>(controller->second);
}

void GenericCartesianConstraint::evaluateBounds(Eigen::VectorXd &upperBounds,
                                                Eigen::VectorXd &lowerBounds)
{
    if(m_isActive)
    {
        this->evaluateDesiredAcceleration();
        upperBounds.block(m_jacobianStartingRow, 0, m_sizeOfConstraint, 1) =
            iDynTree::toEigen(m_desiredAcceleration) - iDynTree::toEigen(*m_biasAcceleration);
    }
    else
    {
        upperBounds.block(m_jacobianStartingRow, 0, m_sizeOfConstraint, 1) =
            - iDynTree::toEigen(*m_biasAcceleration);
    }

    lowerBounds.block(m_jacobianStartingRow, 0, m_sizeOfConstraint, 1) =
        upperBounds.block(m_jacobianStartingRow, 0, m_sizeOfConstraint, 1);
}

CartesianConstraint::CartesianConstraint()
{
    // in case of CartesianConstraint the size is 6 (position + orientation)
    m_sizeOfConstraint = 6;

    m_controllers.insert({"position_pid", std::make_shared<LinearPID>()});
    m_controllers.insert({"orientation_pid", std::make_shared<RotationalPID>()});

    m_desiredAcceleration.resize(6);
}

void CartesianConstraint::evaluateDesiredAcceleration()
{
    m_controllers["position_pid"]->evaluateControl();
    iDynTree::toEigen(m_desiredAcceleration).block(0, 0, 3, 1)
        = iDynTree::toEigen(m_controllers["position_pid"]->getControl());

    m_controllers["orientation_pid"]->evaluateControl();
    iDynTree::toEigen(m_desiredAcceleration).block(3, 0, 3, 1)
        = iDynTree::toEigen(m_controllers["orientation_pid"]->getControl());
}

PositionConstraint::PositionConstraint()
{
    m_sizeOfConstraint = 3;

    m_controllers.insert({"position_pid", std::make_shared<LinearPID>()});

    m_desiredAcceleration.resize(3);
}

void PositionConstraint::evaluateDesiredAcceleration()
{
    m_controllers["position_pid"]->evaluateControl();
    iDynTree::toEigen(m_desiredAcceleration)
        = iDynTree::toEigen(m_controllers["position_pid"]->getControl());
}

OneDimensionalConstraint::OneDimensionalConstraint()
{
    // in case of CartesianConstraint the size is 1
    m_sizeOfConstraint = 1;

    m_controllers.insert({"position_pid", std::make_shared<LinearPID>()});

    m_desiredAcceleration.resize(1);
}

void OneDimensionalConstraint::evaluateDesiredAcceleration()
{
    // take only the third element
    m_controllers["position_pid"]->evaluateControl();
    m_desiredAcceleration(0) = m_controllers["position_pid"]->getControl()(2);
}

ForceConstraint::ForceConstraint(const int& numberOfPoints)
    : m_numberOfPoints(numberOfPoints),
      m_isJacobianEvaluated(false),
      m_areBoundsEvaluated(false)
{

    m_transform = Eigen::MatrixXd::Zero(6,6) ;

    // split the friction cone into slices
    double segmentAngle = M_PI/2 / (m_numberOfPoints - 1);
    double numberOfEquationsFrictionCone =  4 * (m_numberOfPoints - 2) + 4;

    // equation used to ensures COP feasibility and unilateral force
    int numberOfEquationsFeasibility = 7;
    int numberOfEquations = numberOfEquationsFrictionCone + numberOfEquationsFeasibility;

    // memory allocation
    setSizeOfConstraint(numberOfEquations);

    m_jacobianLeftTrivialized.resize(numberOfEquations, 6);
}

void ForceConstraint::setFootSize(const iDynTree::Vector2& footLimitX,
                                  const iDynTree::Vector2& footLimitY)
{
    m_footLimitX = footLimitX;
    m_footLimitY = footLimitY;
}

void ForceConstraint::evaluateJacobian(Eigen::SparseMatrix<double>& jacobian)
{
    // since the jacobian is constant it can been evaluated only once
    if(!m_isJacobianEvaluated)
    {
        // split the pi/2 angle into numberOfPoints -1
        double segmentAngle = M_PI/2 / (m_numberOfPoints - 1);
        int numberOfEquationsFrictionCone =  4 * (m_numberOfPoints - 2) + 4;
        int numberOfEquationsFeasibility = 7;
        int numberOfEquation = numberOfEquationsFrictionCone + numberOfEquationsFeasibility;

        // evalyate friction cone constraint
        Eigen::VectorXd angles(numberOfEquationsFrictionCone);
        Eigen::VectorXd pointsX(numberOfEquationsFrictionCone);
        Eigen::VectorXd pointsY(numberOfEquationsFrictionCone);

        for(int i = 0; i < numberOfEquationsFrictionCone; i++)
        {
            angles(i) = i * segmentAngle;
            pointsX(i) = cos(angles(i));
            pointsY(i) = sin(angles(i));
        }

        for(int i = 0; i < numberOfEquationsFrictionCone; i++)
        {
            double firstPointX, firstPointY, secondPointX, secondPointY;
            firstPointX = pointsX(i);
            firstPointY = pointsY(i);

            secondPointX = pointsX((i + 1) % numberOfEquationsFrictionCone);
            secondPointY = pointsY((i + 1) % numberOfEquationsFrictionCone);

            double angularCoefficients;
            angularCoefficients = (secondPointY - firstPointY) / (secondPointX - firstPointX);

            double offset;
            offset = firstPointY - angularCoefficients * firstPointX;

            int inequalityFactor = 1;
            if(angles(i) > M_PI || angles((i + 1) % numberOfEquationsFrictionCone) > M_PI)
                inequalityFactor = -1;

            //  A_ineq(i,:) = inequalityFactor.* [-angularCoefficients, 1, (-offsets*staticFrictionCoefficient), 0, 0, 0];
            m_jacobianLeftTrivialized(i, 0) = -inequalityFactor * angularCoefficients;
            m_jacobianLeftTrivialized(i, 1) = inequalityFactor;
            m_jacobianLeftTrivialized(i, 2) = -inequalityFactor * offset * m_staticFrictionCoefficient;
        }

        // Unilateral constraint and COP position
        m_jacobianLeftTrivialized(numberOfEquationsFrictionCone, 2) = -m_torsionalFrictionCoefficient;
        m_jacobianLeftTrivialized(numberOfEquationsFrictionCone + 1, 2) = -m_torsionalFrictionCoefficient;
        m_jacobianLeftTrivialized(numberOfEquationsFrictionCone + 2, 2) = -1;
        m_jacobianLeftTrivialized(numberOfEquationsFrictionCone + 3, 2) = m_footLimitX(0);
        m_jacobianLeftTrivialized(numberOfEquationsFrictionCone + 4, 2) = -m_footLimitX(1);
        m_jacobianLeftTrivialized(numberOfEquationsFrictionCone + 5, 2) = m_footLimitY(0);
        m_jacobianLeftTrivialized(numberOfEquationsFrictionCone + 6, 2) = -m_footLimitY(1);

        m_jacobianLeftTrivialized(numberOfEquationsFrictionCone + 5, 3) = -1;
        m_jacobianLeftTrivialized(numberOfEquationsFrictionCone + 6, 3) = 1;

        m_jacobianLeftTrivialized(numberOfEquationsFrictionCone + 3, 4) = 1;
        m_jacobianLeftTrivialized(numberOfEquationsFrictionCone + 4, 4) = -1;

        m_jacobianLeftTrivialized(numberOfEquationsFrictionCone, 5) = 1;
        m_jacobianLeftTrivialized(numberOfEquationsFrictionCone + 1, 5) = -1;

        m_isJacobianEvaluated = true;
    }

    m_transform.block(0,0,3,3) = iDynTree::toEigen(m_footToWorldTransform->getRotation().inverse());
    m_transform.block(3,3,3,3) = iDynTree::toEigen(m_footToWorldTransform->getRotation().inverse());

    Eigen::MatrixXd tmp = iDynTree::toEigen(m_jacobianLeftTrivialized) * m_transform;
    if(!m_firstTime)
        for(int i = 0; i < m_jacobianLeftTrivialized.rows(); i++)
            for(int j = 0; j < m_jacobianLeftTrivialized.columns(); j++)
                jacobian.coeffRef(i + m_jacobianStartingRow,
                                  j + m_jacobianStartingColumn) = tmp(i,j);
    else
    {
        for(int i = 0; i < m_jacobianLeftTrivialized.rows(); i++)
            for(int j = 0; j < m_jacobianLeftTrivialized.columns(); j++)
                jacobian.insert(i + m_jacobianStartingRow,
                                j + m_jacobianStartingColumn) = tmp(i,j);
        m_firstTime = false;
    }
}

void ForceConstraint::evaluateBounds(Eigen::VectorXd &upperBounds, Eigen::VectorXd &lowerBounds)
{
    // todo these may be evaluated only once
    double numberOfEquationsFrictionCone =  4 * (m_numberOfPoints - 2) + 4;

    for(int i = 0; i < m_sizeOfConstraint; i++)
    {
        if(i != 2 + numberOfEquationsFrictionCone)
        {
            lowerBounds(i + m_jacobianStartingRow) = -1000;
            upperBounds(i + m_jacobianStartingRow) = 0;
            continue;
        }
        if(m_isActive)
        {
            lowerBounds(i + m_jacobianStartingRow) = -10000;
            upperBounds(i + m_jacobianStartingRow) = -m_minimalNormalForce;
        }
        else
        {
            lowerBounds(i + m_jacobianStartingRow) = 0;
            upperBounds(i + m_jacobianStartingRow) = 0;
        }
    }
    return;
}

ZMPConstraint::ZMPConstraint()
{
    setSizeOfConstraint(2);

    m_areBoundsEvaluated = false;
}

void ZMPConstraint::evaluateJacobian(Eigen::SparseMatrix<double>& jacobian)
{
    double xL = m_leftFootToWorldTransform->getPosition()(0);
    double yL = m_leftFootToWorldTransform->getPosition()(1);

    double xR = m_rightFootToWorldTransform->getPosition()(0);
    double yR = m_rightFootToWorldTransform->getPosition()(1);

    if(m_firstTime)
    {
        jacobian.insert(m_jacobianStartingRow, m_jacobianStartingColumn + 2) = m_desiredZMP(0) - xL;
        jacobian.insert(m_jacobianStartingRow, m_jacobianStartingColumn + 4) = 1;

        jacobian.insert(m_jacobianStartingRow, m_jacobianStartingColumn + 2 + 6) = m_desiredZMP(0) - xR;
        jacobian.insert(m_jacobianStartingRow, m_jacobianStartingColumn + 4 + 6) = 1;

        jacobian.insert(m_jacobianStartingRow + 1, m_jacobianStartingColumn + 2) = m_desiredZMP(1) - yL;
        jacobian.insert(m_jacobianStartingRow + 1, m_jacobianStartingColumn + 3) = -1;

        jacobian.insert(m_jacobianStartingRow + 1, m_jacobianStartingColumn + 2 + 6) = m_desiredZMP(1) - yR;
        jacobian.insert(m_jacobianStartingRow + 1, m_jacobianStartingColumn + 3 + 6) = -1;

        m_firstTime = false;
    }
    else
    {
        jacobian.coeffRef(m_jacobianStartingRow, m_jacobianStartingColumn + 2) = m_desiredZMP(0) - xL;
        jacobian.coeffRef(m_jacobianStartingRow, m_jacobianStartingColumn + 2 + 6) = m_desiredZMP(0) - xR;
        jacobian.coeffRef(m_jacobianStartingRow + 1, m_jacobianStartingColumn + 2) = m_desiredZMP(1) - yL;
        jacobian.coeffRef(m_jacobianStartingRow + 1, m_jacobianStartingColumn + 2 + 6) = m_desiredZMP(1) - yR;
    }
}

void ZMPConstraint::evaluateBounds(Eigen::VectorXd &upperBounds,
                                   Eigen::VectorXd &lowerBounds)
{
    // since the bounds are constant they can been evaluated only once
    if(m_areBoundsEvaluated)
        return;

    for(int i = 0; i < m_sizeOfConstraint; i++)
    {
        lowerBounds(m_jacobianStartingRow + i) = 0;
        upperBounds(m_jacobianStartingRow + i) = 0;
    }

    m_areBoundsEvaluated = true;
}

SystemDynamicConstraint::SystemDynamicConstraint(const int& systemSize)
{
    m_sizeOfConstraint = systemSize + 6;
    m_systemSize = systemSize;

    // this constraints knows its structure (probably this is not the best way to implement it)
    // set the const part of the constraint
    iDynTree::Triplets selectionMatrixTriplets;
    selectionMatrixTriplets.setDiagonalMatrix(6, 0, 1, m_systemSize);
    m_selectionMatrix.resize(m_systemSize + 6, m_systemSize);
    m_selectionMatrix.setFromTriplets(selectionMatrixTriplets);
}

void SystemDynamicConstraint::evaluateJacobian(Eigen::SparseMatrix<double>& jacobian)
{
    if(!m_firstTime)
    {
        for(int i = 0; i < m_sizeOfConstraint; i++)
        {
            int index = i + m_jacobianStartingRow;

            // mass matrix
            for(int j = 0; j < m_systemSize + 6; j++)
                jacobian.coeffRef(index, j) = -(*m_massMatrix)(i, j);

            // left foot
            for(int j = 0; j < 6; j++)
                // transpose
                jacobian.coeffRef(index, j + m_systemSize + 6 + m_systemSize) = (*m_leftFootJacobian)(j, i);

            // right foot
            for(int j = 0; j < 6; j++)
                // transpose
                jacobian.coeffRef(index, j + m_systemSize + 6 + m_systemSize + 6) = (*m_rightFootJacobian)(j, i);
        }
    }
    else
    {
        for(int i = 0; i < m_sizeOfConstraint; i++)
        {
            int index = i + m_jacobianStartingRow;
            // mass matrix
            for(int j = 0; j < m_systemSize + 6; j++)
                jacobian.insert(index, j) = -(*m_massMatrix)(i, j);

            // selection matrix
            for(int j = 0; j < m_selectionMatrix.columns(); j++)
            {
                if(m_selectionMatrix(i, j) != 0)
                    jacobian.insert(index, j + m_systemSize + 6) = m_selectionMatrix(i, j);
            }

            // left foot
            for(int j = 0; j < 6; j++)
                // transpose
                jacobian.insert(index, j + m_systemSize + 6 + m_systemSize) = (*m_leftFootJacobian)(j, i);

            // right foot
            for(int j = 0; j < 6; j++)
                // transpose
                jacobian.insert(index, j + m_systemSize + 6 + m_systemSize + 6) = (*m_rightFootJacobian)(j, i);
        }
        m_firstTime = false;
    }
}


void SystemDynamicConstraint::evaluateBounds(Eigen::VectorXd &upperBounds,
                                             Eigen::VectorXd &lowerBounds)
{
    upperBounds.block(m_jacobianStartingRow, 0, m_sizeOfConstraint, 1) = iDynTree::toEigen(*m_generalizedBiasForces);
    lowerBounds.block(m_jacobianStartingRow, 0, m_sizeOfConstraint, 1) = iDynTree::toEigen(*m_generalizedBiasForces);
}

LinearMomentumConstraint::LinearMomentumConstraint()
{
    m_sizeOfConstraint = 3;

    m_controller = std::make_shared<LinearPID>();
}

void LinearMomentumConstraint::evaluateJacobian(Eigen::SparseMatrix<double>& jacobian)
{
    // the jacobian is constant so it can be evaluated only once
    // notice here we suppose that the Jacobian matrix is not reinitialized
    if(m_firstTime)
    {
        jacobian.insert(m_jacobianStartingRow + 0, m_jacobianStartingColumn + 0) = 1;
        jacobian.insert(m_jacobianStartingRow + 1, m_jacobianStartingColumn + 1) = 1;
        jacobian.insert(m_jacobianStartingRow + 2, m_jacobianStartingColumn + 2) = 1;

        jacobian.insert(m_jacobianStartingRow + 0, m_jacobianStartingColumn + 0 + 6) = 1;
        jacobian.insert(m_jacobianStartingRow + 1, m_jacobianStartingColumn + 1 + 6) = 1;
        jacobian.insert(m_jacobianStartingRow + 2, m_jacobianStartingColumn + 2 + 6) = 1;

        m_firstTime = false;
    }

    return;
}

void LinearMomentumConstraint::evaluateBounds(Eigen::VectorXd &upperBounds,
                                              Eigen::VectorXd &lowerBounds)
{
    iDynTree::Vector3 weight;
    weight.zero();
    weight(2) = m_robotMass * 9.81;

    m_controller->evaluateControl();

    upperBounds.block(m_jacobianStartingRow, 0, 3, 1) = iDynTree::toEigen(m_controller->getControl())
        + iDynTree::toEigen(weight);
    lowerBounds.block(m_jacobianStartingRow, 0, 3, 1) = iDynTree::toEigen(m_controller->getControl())
        + iDynTree::toEigen(weight);
}

AngularMomentumConstraint::AngularMomentumConstraint()
{
    m_sizeOfConstraint = 3;

    m_controller = std::make_shared<LinearPID>();
    // set the desired trajectory (it is constant)
    iDynTree::Vector3 dummy;
    dummy.zero();
    m_controller->setDesiredTrajectory(dummy, dummy, dummy);
}

void AngularMomentumConstraint::evaluateJacobian(Eigen::SparseMatrix<double>& jacobian)
{
    Eigen::Vector3d leftFootToCoMPosition, rightFootToCoMPosition;
    leftFootToCoMPosition = iDynTree::toEigen(m_leftFootToWorldTransform->getPosition())
        - iDynTree::toEigen(*m_comPosition);

    rightFootToCoMPosition = iDynTree::toEigen(m_rightFootToWorldTransform->getPosition())
        - iDynTree::toEigen(*m_comPosition);

    // this can be remove and done by hand
    Eigen::Matrix3d leftFootToCoMPositionSkew, rightFootToCoMPositionSkew;

    leftFootToCoMPositionSkew = iDynTree::skew(leftFootToCoMPosition);
    rightFootToCoMPositionSkew = iDynTree::skew(rightFootToCoMPosition);

    if(m_firstTime)
    {
        // left foot
        jacobian.insert(m_jacobianStartingRow + 0, m_jacobianStartingColumn + 1)
            = leftFootToCoMPositionSkew(0, 1);

        jacobian.insert(m_jacobianStartingRow + 0, m_jacobianStartingColumn + 2)
            = leftFootToCoMPositionSkew(0, 2);

        jacobian.insert(m_jacobianStartingRow + 1, m_jacobianStartingColumn + 0)
            = leftFootToCoMPositionSkew(1, 0);

        jacobian.insert(m_jacobianStartingRow + 1, m_jacobianStartingColumn + 2)
            = leftFootToCoMPositionSkew(1, 2);

        jacobian.insert(m_jacobianStartingRow + 2, m_jacobianStartingColumn + 0)
            = leftFootToCoMPositionSkew(2, 0);

        jacobian.insert(m_jacobianStartingRow + 2, m_jacobianStartingColumn + 1)
            = leftFootToCoMPositionSkew(2, 1);

        jacobian.insert(m_jacobianStartingRow + 0, m_jacobianStartingColumn + 3) = 1;
        jacobian.insert(m_jacobianStartingRow + 1, m_jacobianStartingColumn + 4) = 1;
        jacobian.insert(m_jacobianStartingRow + 2, m_jacobianStartingColumn + 5) = 1;

        // right foot
        jacobian.insert(m_jacobianStartingRow + 0, m_jacobianStartingColumn + 1 + 6)
            = rightFootToCoMPositionSkew(0, 1);

        jacobian.insert(m_jacobianStartingRow + 0, m_jacobianStartingColumn + 2 + 6)
            = rightFootToCoMPositionSkew(0, 2);

        jacobian.insert(m_jacobianStartingRow + 1, m_jacobianStartingColumn + 0 + 6)
            = rightFootToCoMPositionSkew(1, 0);

        jacobian.insert(m_jacobianStartingRow + 1, m_jacobianStartingColumn + 2 + 6)
            = rightFootToCoMPositionSkew(1, 2);

        jacobian.insert(m_jacobianStartingRow + 2, m_jacobianStartingColumn + 0 + 6)
            = rightFootToCoMPositionSkew(2, 0);

        jacobian.insert(m_jacobianStartingRow + 2, m_jacobianStartingColumn + 1 + 6)
            = rightFootToCoMPositionSkew(2, 1);

        jacobian.insert(m_jacobianStartingRow + 0, m_jacobianStartingColumn + 3 + 6) = 1;
        jacobian.insert(m_jacobianStartingRow + 1, m_jacobianStartingColumn + 4 + 6) = 1;
        jacobian.insert(m_jacobianStartingRow + 2, m_jacobianStartingColumn + 5 + 6) = 1;

        m_firstTime = false;
    }
    else
    {
        // left foot
        jacobian.coeffRef(m_jacobianStartingRow + 0, m_jacobianStartingColumn + 1)
            = leftFootToCoMPositionSkew(0, 1);

        jacobian.coeffRef(m_jacobianStartingRow + 0, m_jacobianStartingColumn + 2)
            = leftFootToCoMPositionSkew(0, 2);

        jacobian.coeffRef(m_jacobianStartingRow + 1, m_jacobianStartingColumn + 0)
            = leftFootToCoMPositionSkew(1, 0);

        jacobian.coeffRef(m_jacobianStartingRow + 1, m_jacobianStartingColumn + 2)
            = leftFootToCoMPositionSkew(1, 2);

        jacobian.coeffRef(m_jacobianStartingRow + 2, m_jacobianStartingColumn + 0)
            = leftFootToCoMPositionSkew(2, 0);

        jacobian.coeffRef(m_jacobianStartingRow + 2, m_jacobianStartingColumn + 1)
            = leftFootToCoMPositionSkew(2, 1);

        // right foot
        jacobian.coeffRef(m_jacobianStartingRow + 0, m_jacobianStartingColumn + 1 + 6)
            = rightFootToCoMPositionSkew(0, 1);

        jacobian.coeffRef(m_jacobianStartingRow + 0, m_jacobianStartingColumn + 2 + 6)
            = rightFootToCoMPositionSkew(0, 2);

        jacobian.coeffRef(m_jacobianStartingRow + 1, m_jacobianStartingColumn + 0 + 6)
            = rightFootToCoMPositionSkew(1, 0);

        jacobian.coeffRef(m_jacobianStartingRow + 1, m_jacobianStartingColumn + 2 + 6)
            = rightFootToCoMPositionSkew(1, 2);

        jacobian.coeffRef(m_jacobianStartingRow + 2, m_jacobianStartingColumn + 0 + 6)
            = rightFootToCoMPositionSkew(2, 0);

        jacobian.coeffRef(m_jacobianStartingRow + 2, m_jacobianStartingColumn + 1 + 6)
            = rightFootToCoMPositionSkew(2, 1);
    }
}

void AngularMomentumConstraint::evaluateBounds(Eigen::VectorXd &upperBounds,
                                              Eigen::VectorXd &lowerBounds)
{
    m_controller->evaluateControl();

    upperBounds.block(m_jacobianStartingRow, 0, 3, 1) = iDynTree::toEigen(m_controller->getControl());
    lowerBounds.block(m_jacobianStartingRow, 0, 3, 1) = iDynTree::toEigen(m_controller->getControl());
}

RateOfChangeConstraint::RateOfChangeConstraint(const int& sizeOfTheConstraintVector)
{
    m_sizeOfConstraint = sizeOfTheConstraintVector;
}

void RateOfChangeConstraint::evaluateJacobian(Eigen::SparseMatrix<double>& jacobian)
{
    // the jacobian is constant
    if(!m_firstTime)
        return;

    for(int i = 0; i < m_sizeOfConstraint; i++)
        jacobian.insert(m_jacobianStartingRow + i, m_jacobianStartingColumn + i) = 1;

    m_firstTime = false;
}


void RateOfChangeConstraint::evaluateBounds(Eigen::VectorXd &upperBounds, Eigen::VectorXd &lowerBounds)
{
    for(int i = 0; i < m_sizeOfConstraint; i++)
    {
        lowerBounds(m_jacobianStartingRow + i) = (*m_previousValues)(i) - m_maximumRateOfChange(i);
        upperBounds(m_jacobianStartingRow + i) = (*m_previousValues)(i) + m_maximumRateOfChange(i);
    }
}
