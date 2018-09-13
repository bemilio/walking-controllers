/**
 * @file WalkingConstraint.tpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/EigenSparseHelpers.h>

template <typename T>
MatrixBlock<T>::MatrixBlock(const int& rows, const int& columns)
{
    matrix.resize(rows, columns);
}

template <typename T, typename U>
void Constraint<T, U>::setSizeOfConstraint(const int& sizeOfConstraint)
{
    // resize lower and upper bound
    m_constraint.resize(sizeOfConstraint);
    m_upperBound.resize(sizeOfConstraint);
    m_lowerBound.resize(sizeOfConstraint);
}

template <typename T>
void LinearConstraint<T>::evaluateConstraint()
{
    // since the constraint is linear the cost function is just the jacobian times the conditional
    // variable
    int jacobianStartingColumn = this->m_jacobian->startingColumn;
    int jacobianColumns = iDynTree::toEigen(this->m_jacobian->matrix).cols();
    iDynTree::toEigen(this->m_constraint) = iDynTree::toEigen(this->m_jacobian->matrix)
        * iDynTree::toEigen(this->m_conditionalVariable).block(jacobianStartingColumn, 0,
                                                         jacobianColumns, 1);
}

template <typename T>
void LinearConstraint<T>::evaluateHessian()
{
    // since the constraint is linear the hessian matrix is empty
    return;
}
