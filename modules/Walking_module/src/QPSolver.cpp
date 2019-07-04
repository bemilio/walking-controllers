

// iDynTree
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/EigenSparseHelpers.h>

#include <QPSolver.hpp>
#include <Utils.hpp>

QPSolver::QPSolver(const int& inputSize, const int& numberOfConstraints)
    :m_inputSize(inputSize), m_numberOfConstraints(numberOfConstraints)
{
    // instantiate the solver class
    m_QPSolver = std::make_unique<OsqpEigen::Solver>();

    //set the number of deceision variables of QP problem
    m_QPSolver->data()->setNumberOfVariables(inputSize);

    // set the number of all constraints includes inequality and equality constraints
    m_QPSolver->data()->setNumberOfConstraints(numberOfConstraints);

    m_QPSolver->settings()->setVerbosity(false);
    m_QPSolver->settings()->setPolish(true);

    m_constraintsMatrix.resize(numberOfConstraints, inputSize);
    m_upperBound.resize(numberOfConstraints);
    m_lowerBound.resize(numberOfConstraints);
    m_gradient.resize(inputSize);

    // set the constant elements of the constraint matrix
    m_constraintsMatrix.insert(0, 0) = 1;
    m_constraintsMatrix.insert(1, 1) = 1;

    m_constraintsMatrix.insert(0, 3) = 1;
    m_constraintsMatrix.insert(1, 4) = 1;

    m_constraintsMatrix.insert(2, 0) = 1;
    m_constraintsMatrix.insert(3, 1) = 1;

    m_constraintsMatrix.insert(4, 2) = 1;

    m_hessianMatrix.resize(m_inputSize, m_inputSize);
}

bool QPSolver::setHessianMatrix(const iDynTree::Vector2& zmpWeight, const iDynTree::Vector2& dcmOffsetWeight, const double& sigmaWeight)
{
    m_hessianMatrix.reserve(m_inputSize);

    m_hessianMatrix.insert(0,0) = zmpWeight(0);
    m_hessianMatrix.insert(1,1) = zmpWeight(1);

    m_hessianMatrix.insert(2,2) = sigmaWeight;

    m_hessianMatrix.insert(3,3) = dcmOffsetWeight(0);
    m_hessianMatrix.insert(4,4) = dcmOffsetWeight(1);


    if (m_QPSolver->isInitialized())
    {
        yWarning()<<"[QPslover::setHessianMatrix] The Hessian Matrix should be set just one time! In step adaptation the hessian matrix is constant and just depend on the gains of cost funtion.";
        //        return  false;
    }
    else
    {
        if (!(m_QPSolver->data()->setHessianMatrix(m_hessianMatrix)))
        {
            yError()<<"[QPslover::setHessianMatrix]Unable to set first time the hessian matrix.";
            return false;
        };
    }
    return true;
}

bool QPSolver::setGradientVector(const iDynTree::Vector2& zmpWeight, const iDynTree::Vector2& dcmOffsetWeight, const double& sigmaWeight,
                                 const iDynTree::Vector2& zmpNominal, const iDynTree::Vector2& dcmOffsetNominal, const double& sigmaNominal)
{
    m_gradient.segment(0, 2) = -(iDynTree::toEigen(zmpWeight).asDiagonal() * iDynTree::toEigen(zmpNominal));
    m_gradient(2) = -sigmaWeight * sigmaNominal;
    m_gradient.segment(3, 2)  = -(iDynTree::toEigen(dcmOffsetWeight).asDiagonal() * iDynTree::toEigen(dcmOffsetNominal));

    if(m_QPSolver->isInitialized())
    {
        if(!m_QPSolver->updateGradient<Eigen::Dynamic>(m_gradient))
        {
            yError()<<"[QPSolver::setGradientVector]:unable to update the Gradient Vector";
            return false;
        }
    }
    else
    {
        if(!m_QPSolver->data()->setGradient<Eigen::Dynamic>(m_gradient))
        {
            yError()<<"[QPSolver::setGradientVector]:unable to set the Gradient Vector for the first time";
            return false;
        }
    }
    return true;
}

bool QPSolver::setConstraintsMatrix(const iDynTree::Vector2& currentDcmPosition, const iDynTree::Vector2& currentZmpPosition)
{
    iDynTree::Vector2 temp;
    iDynTree::toEigen(temp) = iDynTree::toEigen(currentZmpPosition) - iDynTree::toEigen(currentDcmPosition);

    m_constraintsMatrix.coeffRef(0, 2) = temp(0);
    m_constraintsMatrix.coeffRef(1, 2) = temp(1);

    if(m_QPSolver->isInitialized())
    {
        if(!m_QPSolver->updateLinearConstraintsMatrix(m_constraintsMatrix)){
            yError()<<"[setConstraintsMatrix] unable to update the linear constraints matrix of QPSolver corresponding to step adaptator!";
            return false;
        }
    }
    else
    {
        if (!m_QPSolver->data()->setLinearConstraintsMatrix(m_constraintsMatrix)) {
            yError()<<"[setConstraintsMatrix] unable to set the the linear constraints matrix of QPSolver corresponding to step adaptator for the first time ";
            return false;
        }
    }
    return true;
}

bool QPSolver::setBoundsVectorOfConstraints(const iDynTree::Vector2& zmpPosition, const iDynTree::Vector2& zmpPositionNominal, const iDynTree::Vector2& zmpPositionTollerance,
                                            const double& stepDuration, const double& stepDurationTollerance, const double& remainingSingleSupportDuration, const double& omega)
{
    // Two equality constraints and three inequality constraint

    // equality constraint
    m_upperBound.segment(0, 2) = iDynTree::toEigen(zmpPosition);
    m_lowerBound.segment(0, 2) = iDynTree::toEigen(zmpPosition);

    m_upperBound.segment(2, 2) = iDynTree::toEigen(zmpPositionNominal) + iDynTree::toEigen(zmpPositionTollerance);
    m_lowerBound.segment(2, 2) = iDynTree::toEigen(zmpPositionNominal) - iDynTree::toEigen(zmpPositionTollerance);

    m_upperBound(4) = std::exp((stepDuration + stepDurationTollerance) * omega);
    m_lowerBound(4) = std::exp((stepDuration - std::min(stepDurationTollerance, remainingSingleSupportDuration) + 0.05) * omega);

    if (m_QPSolver->isInitialized())
    {
        if (!m_QPSolver->updateBounds(m_lowerBound,m_upperBound))
        {
            yError()<<"[setBoundsVectorOfConstraints]Unable to update the bounds of constraints in QP problem in step adaptation";
            return false;
        }
    }
    else
    {
        if (!m_QPSolver->data()->setLowerBound(m_lowerBound))
        {
            yError()<<"[setBoundsVectorOfConstraints] Unable to set the lower bounds of constraints in QP problem in step adaptation ";
            return false;
        }
        if (!m_QPSolver->data()->setUpperBound(m_upperBound))
        {
            yError()<<"[setBoundsVectorOfConstraints] Unable to set the  upper bounds of constraints in QP problem in step adaptation";
            return false;
        }
    }
    return true;
}

bool QPSolver::isInitialized()
{
    return m_QPSolver->isInitialized();
}

bool QPSolver::initialize()
{
    return m_QPSolver->initSolver();
}

bool QPSolver::solve()
{
    if (!m_QPSolver->isInitialized()) {
        yError()<<"[solve in QPSolver.cpp] The solver has not initilialized";
        return false;
    }
    return m_QPSolver->solve();
}

iDynTree::VectorDynSize QPSolver::getSolution()
{
    Eigen::VectorXd solutionEigen = m_QPSolver->getSolution();
    int solutionSize=m_inputSize;
    iDynTree::VectorDynSize solution(solutionSize);
    iDynTree::toEigen(solution)=solutionEigen;
    return solution;
}
