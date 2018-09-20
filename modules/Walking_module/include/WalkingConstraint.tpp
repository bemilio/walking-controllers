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

template <typename T, typename U>
void Constraint<T, U>::setSubMatricesStartingPosition(const int& startingRow,
                                                      const int& startingColumn)
{
    // set the jacobian starting raw and column
    m_jacobian->startingRow = startingRow;
    m_jacobian->startingColumn = startingColumn;

    // set the hessian staryint row and column.
    // it is important to notice that the m_hessian is a vector containing the hessian matrices
    // (for a nonlinear constraints the number depends on the constraints)
    // we suppose that m_vector has been already populated (notice in general it occurs in the
    // construct of the derived classes)
    for(auto& hessian : m_hessian)
    {
        hessian->startingRow = startingRow;
        // Notice this is not an error. The hessian matrix is quadratic and the blocks
        // different from zero depends on the function between the constraint and the conditional
        // variables
        hessian->startingColumn = startingRow;
    }
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
