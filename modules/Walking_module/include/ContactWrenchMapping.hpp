/**
 * @file ContactWrenchMapping.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_CONTACT_WRENCH_MAPPING_HPP
#define WALKING_CONTACT_WRENCH_MAPPING_HPP

#include <iDynTree/Core/SpatialMomentum.h>

#include <OsqpEigen/OsqpEigen.h>

#include <CartesianPID.hpp>
#include <WalkingConstraint.hpp>

#include <TimeProfiler.hpp>

class ContactWrenchMapping
{
protected:
    bool m_useLinearMomentumConstraint;
    bool m_useLinearMomentumCostFunction;

    bool m_useAngularMomentumConstraint;
    bool m_useAngularMomentumCostFunction;


    //todo
    std::unique_ptr<TimeProfiler> m_profiler; /**< Time profiler. */

    std::unique_ptr<OsqpEigen::Solver> m_optimizer{nullptr}; /**< QP solver. */

    // QP quantities
    Eigen::SparseMatrix<double> m_hessianEigen;
    Eigen::VectorXd m_gradient;
    Eigen::SparseMatrix<double>  m_constraintMatrix;
    Eigen::VectorXd m_upperBound;
    Eigen::VectorXd m_lowerBound;

    // com
    iDynTree::Position m_comPosition ; // used by angular momentum

    virtual void instantiateLinearMomentumConstraint(const yarp::os::Searchable& config) = 0;

    virtual bool instantiateLinearMomentumCostFunction(const yarp::os::Searchable& config) = 0;

    virtual bool instantiateContactForcesConstraint(const yarp::os::Searchable& config) = 0;

    virtual bool instantiateForceRegularizationConstraint(const yarp::os::Searchable& config) = 0;

    virtual bool instantiateAngularMomentumConstraint(const yarp::os::Searchable& config) = 0;

    virtual bool instantiateAngularMomentumCostFunction(const yarp::os::Searchable& config) = 0;


    bool setHessianMatrix();

    bool setGradientVector();

    bool setLinearConstraintMatrix();

    bool setBounds();

    virtual void setNumberOfVariables() = 0;

protected:
    double m_regularizationForceScale;
    double m_regularizationForceOffset;

    std::map<std::string, std::shared_ptr<Constraint>> m_constraints;
    std::map<std::string, std::shared_ptr<CostFunctionElement>> m_costFunctions;

    std::map<std::string, std::unique_ptr<Eigen::SparseMatrix<double>>> m_hessianMatrices;
    std::map<std::string, std::unique_ptr<Eigen::VectorXd>> m_gradientVectors;

    int m_numberOfVariables; /**<Number of variables in the QP problem (# of joints + 12) */
    int m_numberOfConstraints; /**<Number of constraints in the QP problem */
    iDynTree::VectorDynSize m_solution;

public:
    bool initialize(const yarp::os::Searchable& config);

    void setRobotMass(double mass);

    bool setCentroidalTotalMomentum(const iDynTree::SpatialMomentum& linearAngularMomentum);

    virtual void setFeetJacobian(const iDynTree::MatrixDynSize& leftFootJacobian,
                                 const iDynTree::MatrixDynSize& rightFootJacobian) = 0;

    bool setCoMState(const iDynTree::Position& comPosition,
                     const iDynTree::Vector3& comVelocity);

    bool setDesiredCoMTrajectory(const iDynTree::Position& comPosition,
                                 const iDynTree::Vector3& comVelocity,
                                 const iDynTree::Vector3& comAcceleration);

    bool setDesiredVRP(const iDynTree::Vector3& vrp);

    /**
     * Solve the optimization problem.
     * @return true/false in case of success/failure.
     */
    bool solve();

    /**
     * Get the solution of the optimization problem.
     * @param output joint torque
     */
    const iDynTree::VectorDynSize& solution() const;
};


class ContactWrenchMappingDoubleSupport : public ContactWrenchMapping
{
private:

    // feet cartesian
    iDynTree::MatrixDynSize m_leftFootJacobian;
    iDynTree::MatrixDynSize m_rightFootJacobian;
    iDynTree::Transform m_leftFootToWorldTransform;
    iDynTree::Transform m_rightFootToWorldTransform;


    bool instantiateContactForcesConstraint(const yarp::os::Searchable& config) override;

    bool instantiateLinearMomentumCostFunction(const yarp::os::Searchable& config) override;

    bool instantiateForceRegularizationConstraint(const yarp::os::Searchable& config) override;

    bool instantiateAngularMomentumConstraint(const yarp::os::Searchable& config) override;

    bool instantiateAngularMomentumCostFunction(const yarp::os::Searchable& config) override;

    void setNumberOfVariables() override;

    /**
     * Instantiate the Linear momentum constraint.
     * @param config configuration parameters.
     */
    void instantiateLinearMomentumConstraint(const yarp::os::Searchable& config) override;


public:
    void setFeetState(const iDynTree::Transform& leftFootToWorldTransform,
                      const iDynTree::Transform& rightFootToWorldTransform);


    void setFeetJacobian(const iDynTree::MatrixDynSize& leftFootJacobian,
                         const iDynTree::MatrixDynSize& rightFootJacobian) override;

    bool setFeetWeightPercentage(const double &weightInLeft, const double &weightInRight);
};


class ContactWrenchMappingSingleSupport : public ContactWrenchMapping
{
    iDynTree::MatrixDynSize m_stanceFootJacobian;
    iDynTree::Transform m_stanceFootToWorldTransform;


    bool instantiateContactForcesConstraint(const yarp::os::Searchable& config) override;

    bool instantiateLinearMomentumCostFunction(const yarp::os::Searchable& config) override;

    bool instantiateForceRegularizationConstraint(const yarp::os::Searchable& config) override;

    bool instantiateAngularMomentumConstraint(const yarp::os::Searchable& config) override;

    bool instantiateAngularMomentumCostFunction(const yarp::os::Searchable& config) override;


    void setNumberOfVariables() override;

    /**
     * Instantiate the Linear momentum constraint.
     * @param config configuration parameters.
     */
    void instantiateLinearMomentumConstraint(const yarp::os::Searchable& config) override;

public:

    bool setFeetState(const iDynTree::Transform& stanceFootToWorldTransform);

    void setFeetJacobian(const iDynTree::MatrixDynSize& stanceFootJacobian,
                         const iDynTree::MatrixDynSize& swingFootJacobian) override;
};

#endif
