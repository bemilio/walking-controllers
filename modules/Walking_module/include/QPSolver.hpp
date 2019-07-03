

#ifndef QP_SOLVER_HPP
#define QP_SOLVER_HPP

// std
#include <deque>

// iDynTree
#include <iDynTree/Core/SparseMatrix.h>
#include <iDynTree/Core/VectorDynSize.h>

// osqp-eigen
#include <OsqpEigen/OsqpEigen.h>

#include <Utils.hpp>

/**
 * QPSolver class
 */
class QPSolver
{
    /**
     * Pointer to the optimization solver
     */
    std::unique_ptr<OsqpEigen::Solver> m_QPSolver;
    Eigen::SparseMatrix<double> m_constraintsMatrix;
    Eigen::VectorXd m_gradient;
    Eigen::VectorXd m_lowerBound; /**< Lower bound vector. */
    Eigen::VectorXd m_upperBound; /**< Upper bound vector. */

    int m_inputSize; /**< Size of the controlled input vector (2). */
    int m_numberOfConstraints; /**< Size of the controlled input vector (2). */

public:

    /**
     * Constructor.
     * @param numberOfAllConstraints number of equality and inequality constraints!
     * @param inputSize size of the controlled input vector;
     */
    QPSolver(const int& inputSize, const int& numberOfAllConstraints);

    /**
       // TODO
     * Set the hessian matrix.
     * Please do not call this function to update the hessian matrix! It can be set only once.
     * @param hessian hessian matrix.
     * @return true/false in case of success/failure.
     */
    bool setHessianMatrix(const iDynTree::Vector2& zmpWeight, const iDynTree::Vector2& dcmOffsetWeight, const double& sigmaWeight);

    /**
     * Set or update the linear constraints matrix(A) related to equality and inequality constraints(C<Ax<B)
     * If the solver is already set the linear constraints matrix is updated otherwise it is set for
     * the first time.
     * @param currentValuesVector This vector includes the current value of real ZMP, real DCM and delta(the distance that ZMP moves in the SS phase)   ;
     * @return true/false in case of success/failure.
     */
    bool setConstraintsMatrix(const iDynTree::Vector2& currentDcmPosition, const iDynTree::Vector2& currentZmpPosition);

    /**
     * Set or update the gradient
     * @param gainsVector  vector that includes cost function gains
     * @param nominalValuesVector Vector that includes the Desired Value of DCM at the landing moment of foot, StepTiming and next StepPosition and next DCM Offset;
     * @return true/false in case of success/failure.
     */
    bool setGradientVector(const iDynTree::Vector2& zmpWeight, const iDynTree::Vector2& dcmOffsetWeight, const double& sigmaWeight,
                           const iDynTree::Vector2& zmpNominal, const iDynTree::Vector2& dcmOffsetNominal, const double& sigmaNominal);

    /**
     * Set or update the lower and the upper bounds
     * @param nominalValuesVector Vector that includes the Desired Value of DCM at the landing moment of foot, StepTiming and next StepPosition and next DCM Offset;
     * @param currentValuesVector This vector includes the current value of real ZMP, real DCM and delta(the distance that ZMP moves in the SS phase)   ;
     * @param tolerenceOfBounds This vector includes the tolerence between nominal value and the maximum and minimum value constraint    ;
     * @return true/false in case of success/failure.
     */
    bool setBoundsVectorOfConstraints(const iDynTree::Vector2& zmpPosition, const iDynTree::Vector2& zmpPositionNominal,
                                      const iDynTree::Vector2& zmpPositionTollerance,
                                      const double& stepDuration, const double& stepDurationTollerance,
                                      const double& remainingSingleSupportDuration, const double& omega);

    /**
     * Get the state of the solver.
     * @return true if the solver is initialized false otherwise.
     */
    bool isInitialized();

    /**
     * Initialize the solver.
     * @return true/false in case of success/failure.
     */
    bool initialize();

    /**
     * Solve the optimization problem.
     * @return true/false in case of success/failure.
     */
    bool solve();

    /**
     * Get the solver solution
     * @return the entire solution of the solver
     */
    iDynTree::VectorDynSize getSolution();


};

#endif
