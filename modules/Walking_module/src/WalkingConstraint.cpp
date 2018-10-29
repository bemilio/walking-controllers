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

#include <qpOASES.hpp>

void GenericCartesianConstraint::evaluateJacobian()
{
    iDynTree::toEigen(m_jacobian->matrix) = iDynTree::toEigen(*m_roboticJacobian) *
        iDynTree::toEigen(*m_massMatrixInverse) * iDynTree::toEigen(*m_inputMatrix);
}

PositionConstraint::PositionConstraint(const int& jacobianCols)
{
    // memory allocation
    // in case of position constraint the size is 3 only position
    Constraint::setSizeOfConstraint(3);

    m_controllers.insert({"position_pid", std::make_shared<LinearPID>()});

    m_desiredAcceleration.resize(3);

    // allocate jacobian rows
    m_jacobian = std::make_shared<MatrixBlock<iDynTree::MatrixDynSize>>(3, jacobianCols);
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

void GenericCartesianConstraint::evaluateBounds()
{
    this->evaluateDesiredAcceleration();
    iDynTree::toEigen(m_upperBound) = iDynTree::toEigen(m_desiredAcceleration)
        - iDynTree::toEigen(*m_biasAcceleration)
        + iDynTree::toEigen(*m_roboticJacobian) * iDynTree::toEigen(*m_massMatrixInverse)
        * iDynTree::toEigen(*m_generalizedBiasForces);

    m_lowerBound = m_upperBound;
}

CartesianConstraint::CartesianConstraint(const int& jacobianCols)
{
    // in case of CartesianConstraint the size is 6 (position + orientation)
    Constraint::setSizeOfConstraint(6);

    m_controllers.insert({"position_pid", std::make_shared<LinearPID>()});
    m_controllers.insert({"orientation_pid", std::make_shared<RotationalPID>()});

    m_desiredAcceleration.resize(6);

    // memory allocation
    m_jacobian = std::make_shared<MatrixBlock<iDynTree::MatrixDynSize>>(6, jacobianCols);
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

    // split the friction cone into slices
    double segmentAngle = M_PI/2 / (m_numberOfPoints - 1);
    double numberOfEquationsFrictionCone =  4 * (m_numberOfPoints - 2) + 4;

    // equation used to ensures COP feasibility and unilateral force
    int numberOfEquationsFeasibility = 7;
    int numberOfEquations = numberOfEquationsFrictionCone + numberOfEquationsFeasibility;

    // memory allocation
    setSizeOfConstraint(numberOfEquations);
    m_jacobian = std::make_shared<MatrixBlock<iDynTree::MatrixDynSize>>(numberOfEquations, 6);

    m_jacobianLeftTrivialized.resize(numberOfEquations, 6);

    for(int i = 0; i < m_lowerBound.size(); i++)
        m_lowerBound(i) = -qpOASES::INFTY;

    m_upperBound.zero();
    m_upperBound(2 + numberOfEquationsFrictionCone) = -m_minimalNormalForce;

}

void ForceConstraint::activate()
{
    double numberOfEquationsFrictionCone =  4 * (m_numberOfPoints - 2) + 4;

    m_lowerBound(2 + numberOfEquationsFrictionCone) = -qpOASES::INFTY;
    m_upperBound(2 + numberOfEquationsFrictionCone) = -m_minimalNormalForce;
}

void ForceConstraint::deactivate()
{
    double numberOfEquationsFrictionCone =  4 * (m_numberOfPoints - 2) + 4;

    m_lowerBound(2 + numberOfEquationsFrictionCone) = 0;
    m_upperBound(2 + numberOfEquationsFrictionCone) = 0;
}

void ForceConstraint::setFootSize(const iDynTree::Vector2& footLimitX,
                                  const iDynTree::Vector2& footLimitY)
{
    m_footLimitX = footLimitX;
    m_footLimitY = footLimitY;
}

void ForceConstraint::evaluateJacobian()
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

    Eigen::MatrixXd transform = Eigen::MatrixXd::Zero(6,6) ;
    transform.block(0,0,3,3) = iDynTree::toEigen(m_footToWorldTransform->getRotation().inverse());
    transform.block(3,3,3,3) = iDynTree::toEigen(m_footToWorldTransform->getRotation().inverse());

    iDynTree::toEigen(m_jacobian->matrix) = iDynTree::toEigen(m_jacobianLeftTrivialized) * transform;
    // iDynTree::toEigen(m_jacobian->matrix) = iDynTree::toEigen(m_jacobianLeftTrivialized);
}

void ForceConstraint::evaluateBounds()
{
    // sice the bounds are constant they can been evaluated only once
    return;
}

ZMPConstraint::ZMPConstraint()
{
    setSizeOfConstraint(2);
    m_jacobian = std::make_shared<MatrixBlock<iDynTree::MatrixDynSize>>(2, 12);

    m_areBoundsEvaluated = false;
}

void ZMPConstraint::evaluateJacobian()
{
    iDynSparseMatrix tmp(2,12);

    double xL = m_leftFootToWorldTransform->getPosition()(0);
    double yL = m_leftFootToWorldTransform->getPosition()(1);

    double xR = m_rightFootToWorldTransform->getPosition()(0);
    double yR = m_rightFootToWorldTransform->getPosition()(1);

    iDynTree::Rotation leftFootRotationMatrix = m_rightFootToWorldTransform->getRotation();
    iDynTree::Rotation rightFootRotationMatrix = m_rightFootToWorldTransform->getRotation();

    tmp(0,2) = m_desiredZMP(0) - xL;
    tmp(0,3) = -leftFootRotationMatrix(0,1);
    tmp(0,4) = leftFootRotationMatrix(0,0);

    tmp(0,8) = m_desiredZMP(0) - xR;
    tmp(0,9) = -rightFootRotationMatrix(0,1);
    tmp(0,10) = rightFootRotationMatrix(0,0);

    tmp(1,2) = m_desiredZMP(1) - yL;
    tmp(1,3) = -leftFootRotationMatrix(1,1);
    tmp(1,4) = leftFootRotationMatrix(1,0);

    tmp(1,8) = m_desiredZMP(1) - yR;
    tmp(1,9) = -rightFootRotationMatrix(1,1);
    tmp(1,10) = rightFootRotationMatrix(1,0);

    Eigen::MatrixXd transform = Eigen::MatrixXd::Zero(12,12);
    transform.block(0,0,3,3) = iDynTree::toEigen(leftFootRotationMatrix.inverse());
    transform.block(3,3,3,3) = iDynTree::toEigen(leftFootRotationMatrix.inverse());
    transform.block(6,6,3,3) = iDynTree::toEigen(rightFootRotationMatrix.inverse());
    transform.block(9,9,3,3) = iDynTree::toEigen(rightFootRotationMatrix.inverse());

    iDynTree::toEigen(m_jacobian->matrix) = iDynTree::toEigen(tmp) * transform;
}

void ZMPConstraint::evaluateBounds()
{
    // sice the bounds are constant they can been evaluated only once
    if(m_areBoundsEvaluated)
        return;

    m_lowerBound.zero();
    m_upperBound.zero();

    m_areBoundsEvaluated = true;
}