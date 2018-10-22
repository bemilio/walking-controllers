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

void Constraint::setSubMatricesStartingPosition(const int& startingRow,
                                                const int& startingColumn)
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
    this->evaluateDesiredAcceleration();
    upperBounds.block(m_jacobianStartingRow, 0, m_sizeOfConstraint, 1) =
        iDynTree::toEigen(m_desiredAcceleration) - iDynTree::toEigen(*m_biasAcceleration);

    lowerBounds.block(m_jacobianStartingRow, 0, m_sizeOfConstraint, 1) =
        upperBounds.block(m_jacobianStartingRow, 0, m_sizeOfConstraint, 1);
}

CartesianConstraint::CartesianConstraint(const int& jacobianCols)
{
    // in case of CartesianConstraint the size is 6 (position + orientation)
    Constraint::setSizeOfConstraint(6);

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

PositionConstraint::PositionConstraint(const int& jacobianCols)
{
    Constraint::setSizeOfConstraint(3);

    m_controllers.insert({"position_pid", std::make_shared<LinearPID>()});

    m_desiredAcceleration.resize(3);
}

void PositionConstraint::evaluateDesiredAcceleration()
{
    m_controllers["position_pid"]->evaluateControl();
    iDynTree::toEigen(m_desiredAcceleration)
        = iDynTree::toEigen(m_controllers["position_pid"]->getControl());
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

void ForceConstraint::activate()
{
    m_isActive = true;
}

void ForceConstraint::deactivate()
{
    m_isActive = false;
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

// ZMPConstraint::ZMPConstraint()
// {
//     setSizeOfConstraint(2);
//     m_jacobian = std::make_shared<MatrixBlock<iDynTree::MatrixDynSize>>(2, 12);

//     m_areBoundsEvaluated = false;
// }

// void ZMPConstraint::evaluateJacobian(Eigen::SparseMatrix<double>& jacobian)
// {
//     iDynSparseMatrix tmp(2,12);

//     double xL = m_leftFootToWorldTransform->getPosition()(0);
//     double yL = m_leftFootToWorldTransform->getPosition()(1);

//     double xR = m_rightFootToWorldTransform->getPosition()(0);
//     double yR = m_rightFootToWorldTransform->getPosition()(1);

//     iDynTree::Rotation leftFootRotationMatrix = m_rightFootToWorldTransform->getRotation();
//     iDynTree::Rotation rightFootRotationMatrix = m_rightFootToWorldTransform->getRotation();

//     tmp(0,2) = m_desiredZMP(0) - xL;
//     tmp(0,3) = -leftFootRotationMatrix(0,1);
//     tmp(0,4) = leftFootRotationMatrix(0,0);

//     tmp(0,8) = m_desiredZMP(0) - xR;
//     tmp(0,9) = -rightFootRotationMatrix(0,1);
//     tmp(0,10) = rightFootRotationMatrix(0,0);

//     tmp(1,2) = m_desiredZMP(1) - yL;
//     tmp(1,3) = -leftFootRotationMatrix(1,1);
//     tmp(1,4) = leftFootRotationMatrix(1,0);

//     tmp(1,8) = m_desiredZMP(1) - yR;
//     tmp(1,9) = -rightFootRotationMatrix(1,1);
//     tmp(1,10) = rightFootRotationMatrix(1,0);

//     Eigen::MatrixXd transform = Eigen::MatrixXd::Zero(12,12);
//     transform.block(0,0,3,3) = iDynTree::toEigen(leftFootRotationMatrix.inverse());
//     transform.block(3,3,3,3) = iDynTree::toEigen(leftFootRotationMatrix.inverse());
//     transform.block(6,6,3,3) = iDynTree::toEigen(rightFootRotationMatrix.inverse());
//     transform.block(9,9,3,3) = iDynTree::toEigen(rightFootRotationMatrix.inverse());

//     iDynTree::toEigen(m_jacobian->matrix) = iDynTree::toEigen(tmp) * transform;
// }

// void ZMPConstraint::evaluateBounds()
// {
//     // sice the bounds are constant they can been evaluated only once
//     if(m_areBoundsEvaluated)
//         return;

//     m_lowerBound.zero();
//     m_upperBound.zero();

//     m_areBoundsEvaluated = true;
// }

SystemDynamicConstraint::SystemDynamicConstraint(const int& jacobianRows, const int& jacobianCols,
                                                 const int& systemSize)
{
    // memory allocation
    // in case of position constraint the size is 3 only position
    Constraint::setSizeOfConstraint(jacobianRows);

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
        for(int i = 0; i < m_systemSize + 6; i++)
        {
            int index = i + m_jacobianStartingRow;
            // mass matrix
            for(int j = 0; j < m_systemSize + 6; j++)
                jacobian.coeffRef(index, j) = -(*m_massMatrix)(i, j);

            // constatnt
            // // selection matrix
            // for(int j = 0; j < m_selectionMatrix.columns(); j++)
            // {
            //     if(m_selectionMatrix(i, j) != 0)
            //         jacobian.coeffRef(index, j + m_systemSize + 6) = m_selectionMatrix(i, j);
            // }

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
        for(int i = 0; i < m_systemSize + 6; i++)
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
