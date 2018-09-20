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
    iDynTree::toEigen(m_jacobian->matrix) = iDynTree::toEigen(m_roboticJacobian) *
        iDynTree::toEigen(m_massMatrix).inverse() *
        iDynTree::toEigen(m_inputMatrix);
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
        - iDynTree::toEigen(m_biasAcceleration) +
        iDynTree::toEigen(m_roboticJacobian) * iDynTree::toEigen(m_massMatrix).inverse() *
        iDynTree::toEigen(m_generalizedBiasForces);

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
    transform.block(0,0,3,3) = iDynTree::toEigen(m_footToWorldTransform.getRotation().inverse());
    transform.block(3,3,3,3) = iDynTree::toEigen(m_footToWorldTransform.getRotation().inverse());

    // iDynTree::toEigen(m_jacobian->matrix) = iDynTree::toEigen(m_jacobianLeftTrivialized) * transform;
    iDynTree::toEigen(m_jacobian->matrix) = iDynTree::toEigen(m_jacobianLeftTrivialized);
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

    double xL = m_leftFootToWorldTransform.getPosition()(0);
    double yL = m_leftFootToWorldTransform.getPosition()(1);

    double xR = m_rightFootToWorldTransform.getPosition()(0);
    double yR = m_rightFootToWorldTransform.getPosition()(1);

    iDynTree::Rotation leftFootRotationMatrix = m_leftFootToWorldTransform.getRotation();
    iDynTree::Rotation rightFootRotationMatrix = m_rightFootToWorldTransform.getRotation();

    tmp(0,2) = m_desiredZMP(0) - xL;
    tmp(0,3) = leftFootRotationMatrix(0,1);
    tmp(0,4) = -leftFootRotationMatrix(0,0);

    tmp(0,8) = m_desiredZMP(0) - xR;
    tmp(0,9) = rightFootRotationMatrix(0,1);
    tmp(0,10) = -rightFootRotationMatrix(0,0);

    tmp(0,2) = m_desiredZMP(1) - yL;
    tmp(0,3) = leftFootRotationMatrix(1,1);
    tmp(0,4) = -leftFootRotationMatrix(1,0);

    tmp(0,8) = m_desiredZMP(1) - yR;
    tmp(0,9) = rightFootRotationMatrix(1,1);
    tmp(0,10) = -rightFootRotationMatrix(1,0);

    Eigen::MatrixXd transform = Eigen::MatrixXd::Zero(12,12);
    transform.block(0,0,3,3) = iDynTree::toEigen(leftFootRotationMatrix.inverse());
    transform.block(3,3,3,3) = iDynTree::toEigen(leftFootRotationMatrix.inverse());
    transform.block(6,6,3,3) = iDynTree::toEigen(rightFootRotationMatrix.inverse());
    transform.block(9,9,3,3) = iDynTree::toEigen(rightFootRotationMatrix.inverse());

    // iDynTree::toEigen(m_jacobian->matrix) = iDynTree::toEigen(tmp) * transform;
    iDynTree::toEigen(m_jacobian->matrix) = iDynTree::toEigen(tmp);
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


// ZMPConstraintSingleSupport::ZMPConstraintSingleSupport()
// {
//     setSizeOfConstraint(2);
//     m_jacobian = std::make_shared<MatrixBlock<iDynTree::MatrixDynSize>>(2, 6);

//     m_hessian.push_back(std::make_shared<MatrixBlock<iDynSparseMatrix>>(6,6));
//     m_hessian.push_back(std::make_shared<MatrixBlock<iDynSparseMatrix>>(6,6));
// }

// void ZMPConstraintSingleSupport::setConditionalVariable(const iDynTree::VectorDynSize& conditionalVariable)
// {
//     // we assume that the contact wrenches are structured as follows [f_x f_y f_z tau_x, tau_y, tau_z]
//     int jacobianStartingColumn = m_jacobian->startingColumn;

//     m_stanceFootFZ = conditionalVariable(jacobianStartingColumn + 2);
//     m_stanceFootTauX = conditionalVariable(jacobianStartingColumn + 3);
//     m_stanceFootTauY = conditionalVariable(jacobianStartingColumn + 4);
// }

// void ZMPConstraintSingleSupport::setDesiredZMP(const iDynTree::Vector2 &zmp)
// {
//     m_lowerBound(0) = zmp(0);
//     m_lowerBound(1) = zmp(1);

//     m_upperBound = m_lowerBound;
// }

// void ZMPConstraintSingleSupport::evaluateConstraint()
// {
//     // we assume that the contact wrench is structured as follows [f_x f_y f_z tau_x, tau_y, tau_z]
//     iDynTree::Position zmp;
//     zmp(0) = -m_stanceFootTauY / m_stanceFootFZ;
//     zmp(1) = m_stanceFootTauX / m_stanceFootFZ;
//     zmp(2) = 0.0;

//     zmp = m_stanceFootToWorldTransform * zmp;

//     m_constraint(0) = zmp(0);
//     m_constraint(1) = zmp(1);
// }

// void ZMPConstraintSingleSupport::evaluateJacobian()
// {
//     // zmp jacobian expressed in the stance foot frame
//     iDynSparseMatrix zmpStanceFootJacobian(2,6);
//     zmpStanceFootJacobian(0, 2) = m_stanceFootTauY / std::pow(m_stanceFootFZ, 2);
//     zmpStanceFootJacobian(0, 4) = -1/m_stanceFootFZ;
//     zmpStanceFootJacobian(1, 2) = -m_stanceFootTauX / std::pow(m_stanceFootFZ, 2);
//     zmpStanceFootJacobian(1, 3) = 1/m_stanceFootFZ;

//     // conversion to stance foot frame to world frame
//     iDynTree::toEigen(m_jacobian->matrix) = iDynTree::toEigen(m_stanceFootToWorldTransform.getRotation()).block(0, 0, 2, 2) * iDynTree::toEigen(zmpStanceFootJacobian);
// }

// void ZMPConstraintSingleSupport::evaluateHessian()
// {
//     // evaluate first Hessian
//     iDynTree::Rotation rotationMatrix = m_stanceFootToWorldTransform.getRotation();

//     iDynSparseMatrix zmpStanceFootHessian1(6,6);
//     zmpStanceFootHessian1(2,2) = - 2 * m_stanceFootTauY / std::pow(m_stanceFootFZ, 3);
//     zmpStanceFootHessian1(2,4) = 1 / std::pow(m_stanceFootFZ, 2);
//     zmpStanceFootHessian1(4,2) = 1 / std::pow(m_stanceFootFZ, 2);

//     iDynSparseMatrix zmpStanceFootHessian2(6,6);
//     zmpStanceFootHessian1(2,2) = 2 * m_stanceFootTauX / std::pow(m_stanceFootFZ, 3);
//     zmpStanceFootHessian1(2,3) = -1 / std::pow(m_stanceFootFZ, 2);
//     zmpStanceFootHessian1(3,2) = -1 / std::pow(m_stanceFootFZ, 2);

//     // evaluate first hessian matrix
//     for(int i = 0; i < m_hessian.size(); i++)
//     {
//         Eigen::SparseMatrix<double> tmp = rotationMatrix(i, 0)
//             * iDynTree::toEigen(zmpStanceFootHessian1)
//             + rotationMatrix(i, 1) * iDynTree::toEigen(zmpStanceFootHessian2);

//         m_hessian[i]->matrix = iDynTreeHelper::SparseMatrix::fromEigen(tmp);
//     }
// }

// void ZMPConstraintSingleSupport::evaluateBounds()
// {
//     return;
// }

// ZMPConstraintDoubleSupport::ZMPConstraintDoubleSupport()
// {
//     Constraint::setSizeOfConstraint(2);
//     m_jacobian = std::make_shared<MatrixBlock<iDynTree::MatrixDynSize>>(2, 12);

//     m_hessian.push_back(std::make_shared<MatrixBlock<iDynSparseMatrix>>(12, 12));
//     m_hessian.push_back(std::make_shared<MatrixBlock<iDynSparseMatrix>>(12, 12));
// }

// void ZMPConstraintDoubleSupport::setDesiredZMP(const iDynTree::Vector2 &zmp)
// {
//     m_lowerBound(0) = zmp(0);
//     m_lowerBound(1) = zmp(1);

//     m_upperBound = m_lowerBound;
// }

// void ZMPConstraintDoubleSupport::setConditionalVariable(const iDynTree::VectorDynSize& conditionalVariable)
// {
//     // we assume that the contact wrenches are structured as follows [f_x f_y f_z tau_x, tau_y, tau_z]
//     // and they are stored in a vector containing the left and the right wrench
//     int jacobianStartingColumn = m_jacobian->startingColumn;

//     m_leftFootFZ = conditionalVariable(jacobianStartingColumn + 2);
//     m_leftFootTauX = conditionalVariable(jacobianStartingColumn + 3);
//     m_leftFootTauY = conditionalVariable(jacobianStartingColumn + 4);

//     m_rightFootFZ = conditionalVariable(jacobianStartingColumn + 6 + 2);
//     m_rightFootTauX = conditionalVariable(jacobianStartingColumn + 6 + 3);
//     m_rightFootTauY = conditionalVariable(jacobianStartingColumn + 6 + 4);
// }

// void ZMPConstraintDoubleSupport::evaluateConstraint()
// {
//     // evaluate the local ZMP
//     iDynTree::Position zmpRight, zmpLeft;
//     zmpRight(0) = -m_rightFootTauY / m_rightFootFZ;
//     zmpRight(1) = m_rightFootTauX / m_rightFootFZ;
//     zmpRight(2) = 0.0;

//     zmpLeft(0) = -m_leftFootTauY / m_leftFootFZ;
//     zmpLeft(1) = m_leftFootTauX / m_leftFootFZ;
//     zmpLeft(2) = 0.0;

//     double totalZ = m_leftFootFZ + m_rightFootFZ;

//     zmpLeft = m_leftFootToWorldTransform * zmpLeft;
//     zmpRight = m_leftFootToWorldTransform * zmpRight;

//     // the global zmp is given by a weighted average
//     iDynTree::Position zmp;
//     iDynTree::toEigen(zmp) = (m_leftFootFZ / totalZ) * iDynTree::toEigen(zmpLeft)
//         + (m_rightFootFZ * totalZ) * iDynTree::toEigen(zmpRight);

//     m_constraint(0) = zmp(0);
//     m_constraint(1) = zmp(1);
// }

// void ZMPConstraintDoubleSupport::evaluateJacobian()
// {
//     double totalZ = m_leftFootFZ + m_rightFootFZ;

//     // zmp jacobian expressed in the stance foot frame
//     iDynSparseMatrix leftFootJacobian(2,12);
//     leftFootJacobian (0, 2) = m_leftFootTauY / std::pow(totalZ, 2);
//     leftFootJacobian (0, 4) = -1 / totalZ;
//     leftFootJacobian (0, 8) = m_leftFootTauY / std::pow(totalZ, 2);

//     leftFootJacobian (1, 2) = -m_leftFootTauX / std::pow(totalZ, 2);
//     leftFootJacobian (1, 3) = 1 / totalZ;
//     leftFootJacobian (1, 8) = -m_leftFootTauX / std::pow(totalZ, 2);

//     iDynSparseMatrix rightFootJacobian(2,12);
//     rightFootJacobian (0, 2) = m_rightFootTauY / std::pow(totalZ, 2);
//     rightFootJacobian (0, 8) = m_rightFootTauY / std::pow(totalZ, 2);
//     rightFootJacobian (0, 10) = -1 / totalZ;

//     rightFootJacobian (0, 2) = -m_rightFootTauX / std::pow(totalZ, 2);
//     rightFootJacobian (0, 8) = -m_rightFootTauX / std::pow(totalZ, 2);
//     rightFootJacobian (0, 9) = 1 / totalZ;

//     // conversion to stance foot frame to world frame
//     iDynTree::toEigen(m_jacobian->matrix) = iDynTree::toEigen(m_leftFootToWorldTransform.getRotation()).block(0, 0, 2, 2) * iDynTree::toEigen(leftFootJacobian) +
//         iDynTree::toEigen(m_rightFootToWorldTransform.getRotation()).block(0, 0, 2, 2) * iDynTree::toEigen(rightFootJacobian);
// }

// void ZMPConstraintDoubleSupport::evaluateHessian()
// {
//     double totalZ = m_leftFootFZ + m_rightFootFZ;

//     iDynTree::Rotation leftFootRotationMatrix = m_leftFootToWorldTransform.getRotation();
//     iDynTree::Rotation rightFootRotationMatrix = m_rightFootToWorldTransform.getRotation();


//     // zmp jacobian expressed in the stance foot frame
//     iDynSparseMatrix leftFootHessian1(12,12);
//     leftFootHessian1(2,2) = -2 * m_leftFootTauY / std::pow(totalZ, 3);
//     leftFootHessian1(2,4) = 1 / std::pow(totalZ, 2);
//     leftFootHessian1(2,8) = -2 * m_leftFootTauY / std::pow(totalZ, 3);

//     leftFootHessian1(4,2) = 1 / std::pow(totalZ, 2);
//     leftFootHessian1(4,8) = 1 / std::pow(totalZ, 2);

//     leftFootHessian1(8,2) = -2 * m_leftFootTauY / std::pow(totalZ, 3);
//     leftFootHessian1(8,4) = 1 / std::pow(totalZ, 2);
//     leftFootHessian1(8,8) = -2 * m_leftFootTauY / std::pow(totalZ, 3);

//     iDynSparseMatrix leftFootHessian2(12,12);
//     leftFootHessian1(2,2) = 2 * m_leftFootTauX / std::pow(totalZ, 3);
//     leftFootHessian1(2,3) = -1 / std::pow(totalZ, 2);
//     leftFootHessian1(2,8) = 2 * m_leftFootTauX / std::pow(totalZ, 3);

//     leftFootHessian1(3,2) = -1 / std::pow(totalZ, 2);
//     leftFootHessian1(3,8) = -1 / std::pow(totalZ, 2);

//     leftFootHessian1(8,2) = 2 * m_leftFootTauX / std::pow(totalZ, 3);
//     leftFootHessian1(8,3) = -1 / std::pow(totalZ, 2);
//     leftFootHessian1(8,8) = 2 * m_leftFootTauX / std::pow(totalZ, 3);


//     iDynSparseMatrix rightFootHessian1(12,12);
//     rightFootHessian1(2,2) = -2 * m_rightFootTauY / std::pow(totalZ, 3);
//     rightFootHessian1(2,8) = -2 * m_rightFootTauY / std::pow(totalZ, 3);
//     rightFootHessian1(2,10) = 1 / std::pow(totalZ, 2);

//     rightFootHessian1(8,2) = -2 * m_rightFootTauY / std::pow(totalZ, 3);
//     rightFootHessian1(8,8) = -2 * m_rightFootTauY / std::pow(totalZ, 3);
//     rightFootHessian1(8,10) = 1 / std::pow(totalZ, 2);

//     rightFootHessian1(10,2) = 1 / std::pow(totalZ, 2);
//     rightFootHessian1(10,8) = 1 / std::pow(totalZ, 2);

//     iDynSparseMatrix rightFootHessian2(12,12);
//     rightFootHessian2(2,2) = -2 * m_rightFootTauY / std::pow(totalZ, 3);
//     rightFootHessian2(2,8) = -2 * m_rightFootTauY / std::pow(totalZ, 3);
//     rightFootHessian2(2,10) = 1 / std::pow(totalZ, 2);

//     rightFootHessian2(8,2) = -2 * m_rightFootTauY / std::pow(totalZ, 3);
//     rightFootHessian2(8,8) = -2 * m_rightFootTauY / std::pow(totalZ, 3);
//     rightFootHessian2(8,10) = 1 / std::pow(totalZ, 2);

//     rightFootHessian2(10,2) = 1 / std::pow(totalZ, 2);
//     rightFootHessian2(10,8) = 1 / std::pow(totalZ, 2);

//      for(int i = 0; i < m_hessian.size(); i++)
//     {
//         Eigen::SparseMatrix<double> tmp =
//             leftFootRotationMatrix(i, 0) * iDynTree::toEigen(leftFootHessian1)
//             + leftFootRotationMatrix(i, 1) * iDynTree::toEigen(leftFootHessian2)
//             + rightFootRotationMatrix(i, 0) * iDynTree::toEigen(rightFootHessian1)
//             + rightFootRotationMatrix(i, 1) * iDynTree::toEigen(rightFootHessian2);

//         m_hessian[i]->matrix = iDynTreeHelper::SparseMatrix::fromEigen(tmp);
//     }
// }

// void ZMPConstraintDoubleSupport::evaluateBounds()
// {
//     return;
// }
