/**
 * @file ContactWrenchMapping.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <iDynTree/yarp/YARPConfigurationsLoader.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/Wrench.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/EigenSparseHelpers.h>

#include <ContactWrenchMapping.hpp>
#include <Utils.hpp>

// #include <EigenMatio/EigenMatio.hpp>

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXd;

double m_mass;

bool ContactWrenchMapping::initialize(const yarp::os::Searchable& config)
{

    // m_profiler = std::make_unique<TimeProfiler>();
    // m_profiler->setPeriod(round(0.1 / 0.01));

    // m_profiler->addTimer("hessian");
    // m_profiler->addTimer("gradient");
    // m_profiler->addTimer("A");
    // m_profiler->addTimer("add A");
    // m_profiler->addTimer("bounds");
    // m_profiler->addTimer("solve");

    // depends on single or double support
    setNumberOfVariables();

    m_numberOfConstraints = 0;

    // results
    m_solution.resize(m_numberOfVariables);

    // check if the config is empty
    if(config.isNull())
    {
        yError() << "[initialize] Empty configuration for Task based torque solver.";
        return false;
    }

    yarp::os::Bottle& linearMomentumOptions = config.findGroup("LINEAR_MOMENTUM");
    instantiateLinearMomentumConstraint(linearMomentumOptions);
    if(!instantiateLinearMomentumCostFunction(linearMomentumOptions))
    {
        yError() << "[initialize] Unable to instantiate the linear momentum cost.";
        return false;
    }

    yarp::os::Bottle& angularMomentumConstraintOptions = config.findGroup("ANGULAR_MOMENTUM");
    if(!instantiateAngularMomentumConstraint(angularMomentumConstraintOptions))
    {
        yError() << "[initialize] Unable to instantiate the Angular Momentum constraint.";
        return false;
    }
    if(!instantiateAngularMomentumCostFunction(angularMomentumConstraintOptions))
    {
        yError() << "[initialize] Unable to instantiate the Angular Momentum constraint.";
        return false;
    }

    yarp::os::Bottle& contactForcesOption = config.findGroup("CONTACT_FORCES");
    if(!instantiateContactForcesConstraint(contactForcesOption))
    {
        yError() << "[initialize] Unable to get the instantiate the force feet constraints.";
        return false;
    }

    yarp::os::Bottle& regularizationForceOption = config.findGroup("REGULARIZATION_FORCE");
    if(!instantiateForceRegularizationConstraint(regularizationForceOption))
    {
        yError() << "[initialize] Unable to get the instantiate the regularization force constraint.";
        return false;
    }


    // resize
    // sparse matrix
    m_hessianEigen.resize(m_numberOfVariables, m_numberOfVariables);
    m_constraintMatrix.resize(m_numberOfConstraints, m_numberOfVariables);

    // dense vectors
    m_gradient = Eigen::VectorXd::Zero(m_numberOfVariables);
    m_lowerBound = Eigen::VectorXd::Zero(m_numberOfConstraints);
    m_upperBound = Eigen::VectorXd::Zero(m_numberOfConstraints);

    // initialize the optimization problem
    m_optimizer = std::make_unique<OsqpEigen::Solver>();
    m_optimizer->data()->setNumberOfVariables(m_numberOfVariables);
    m_optimizer->data()->setNumberOfConstraints(m_numberOfConstraints);

    m_optimizer->settings()->setVerbosity(false);
    m_optimizer->settings()->setLinearSystemSolver(0);

    // set constant element of the cost function
    for(const auto& element: m_costFunctions)
    {
        std::string key = element.first;
        element.second->setHessianConstantElements(*(m_hessianMatrices.at(key)));
        element.second->setGradientConstantElemenets(*(m_gradientVectors.at(key)));
    }

    // set constant element of the constraint
    for(const auto& constraint: m_constraints)
    {
        constraint.second->setJacobianConstantElements(m_constraintMatrix);
        constraint.second->setBoundsConstantElements(m_upperBound, m_lowerBound);
    }

    // print some usefull information
    yInfo() << "Total number of constraints " << m_numberOfConstraints;
    for(const auto& constraint: m_constraints)
        yInfo() << constraint.first << ": " << constraint.second->getNumberOfConstraints()
                << constraint.second->getJacobianStartingRow()
                << constraint.second->getJacobianStartingColumn();

    return true;
}

bool ContactWrenchMapping::setCoMState(const iDynTree::Position& comPosition,
                                        const iDynTree::Vector3& comVelocity)
{
    if(m_useLinearMomentumConstraint)
    {
        // save com desired trajectory
        auto constraint = m_constraints.find("linear_momentum_constraint");
        if(constraint == m_constraints.end())
        {
            yError() << "[setCoMState] unable to find the linear momentum constraint. "
                     << "Please call 'initialize()' method";
            return false;
        }

        auto ptr = std::static_pointer_cast<LinearMomentumConstraint>(constraint->second);
        ptr->setCoMPosition(comPosition);
        ptr->setCoMVelocity(comVelocity);
    }

    if(m_useLinearMomentumCostFunction)
    {
        // save com desired trajectory
        auto costFunction = m_costFunctions.find("linear_momentum_costFunction");
        if(costFunction == m_costFunctions.end())
        {
            yError() << "[setCoMState] unable to find the linear momentum costFunction. "
                     << "Please call 'initialize()' method";
            return false;
        }

        auto ptr = std::static_pointer_cast<LinearMomentumCostFunction>(costFunction->second);
        ptr->setCoMPosition(comPosition);
    }

    if(m_useAngularMomentumConstraint)
    {
        auto constraint = m_constraints.find("angular_momentum_constraint");
        if(constraint == m_constraints.end())
        {
            yError() << "[setCoMState] unable to find the angular momentum constraint. "
                     << "Please call 'initialize()' method";
            return false;
        }

        auto ptr = std::dynamic_pointer_cast<AngularMomentumElement>(constraint->second);
        ptr->setCoMPosition(comPosition);
    }

    if(m_useAngularMomentumCostFunction)
    {
        auto costFunction = m_costFunctions.find("angular_momentum_costFunction");
        if(costFunction == m_costFunctions.end())
        {
            yError() << "[setCoMState] unable to find the angular momentum costFunction. "
                     << "Please call 'initialize()' method";
            return false;
        }

        auto ptr = std::dynamic_pointer_cast<AngularMomentumElement>(costFunction->second);
        ptr->setCoMPosition(comPosition);
    }

    return true;
}

bool ContactWrenchMapping::setDesiredCoMTrajectory(const iDynTree::Position& comPosition,
                                                   const iDynTree::Vector3& comVelocity,
                                                    const iDynTree::Vector3& comAcceleration)
{
    if(m_useLinearMomentumConstraint)
    {
        // save com desired trajectory
        auto constraint = m_constraints.find("linear_momentum_constraint");
        if(constraint == m_constraints.end())
        {
            yError() << "[setCoMState] unable to find the linear momentum constraint. "
                     << "Please call 'initialize()' method";
            return false;
        }

        auto ptr = std::static_pointer_cast<LinearMomentumConstraint>(constraint->second);

        yInfo() << comPosition.toString();
        ptr->setDesiredCoMPosition(comPosition);
        ptr->setDesiredCoMVelocity(comVelocity);
        ptr->setDesiredCoMAcceleration(comAcceleration);
    }
        return true;
}


bool ContactWrenchMapping::setCentroidalTotalMomentum(const iDynTree::SpatialMomentum& centroidalTotalMomentum)
{
    if(m_useAngularMomentumConstraint)
    {
        auto constraint = m_constraints.find("angular_momentum_constraint");
        if(constraint == m_constraints.end())
        {
            yError() << "[setLinearAngularMomentum] unable to find the angular momentum constraint. "
                     << "Please call 'initialize()' method";
            return false;
        }

        auto ptr = std::dynamic_pointer_cast<AngularMomentumElement>(constraint->second);
        ptr->setAngularMomentum(centroidalTotalMomentum.getAngularVec3());
    }

    if(m_useAngularMomentumCostFunction)
    {
        auto costFunction = m_costFunctions.find("angular_momentum_costFunction");
        if(costFunction == m_costFunctions.end())
        {
            yError() << "[setCoMState] unable to find the angular momentum costFunction. "
                     << "Please call 'initialize()' method";
            return false;
        }

        auto ptr = std::dynamic_pointer_cast<AngularMomentumElement>(costFunction->second);
        ptr->setAngularMomentum(centroidalTotalMomentum.getAngularVec3());
    }
    return true;
}


bool ContactWrenchMapping::setDesiredVRP(const iDynTree::Vector3 &vrp)
{
    if(m_useLinearMomentumConstraint)
    {
        auto constraint = m_constraints.find("linear_momentum_constraint");
        if(constraint == m_constraints.end())
        {
            yError() << "[setDesiredVRP] Unable to find the linear_momentum constraint. "
                     << "Please call 'initialize()' method";
            return false;
        }
        auto ptr = std::static_pointer_cast<LinearMomentumConstraint>(constraint->second);
        ptr->setDesiredVRP(vrp);
    }

    if(m_useLinearMomentumCostFunction)
    {
        auto costFunction = m_costFunctions.find("linear_momentum_costFunction");
        if(costFunction == m_costFunctions.end())
        {
            yError() << "[setDesiredVRP] Unable to find the linear_momentum costFunction. "
                     << "Please call 'initialize()' method";
            return false;
        }
        auto ptr = std::static_pointer_cast<LinearMomentumCostFunction>(costFunction->second);
        ptr->setDesiredVRP(vrp);
    }
    return true;
}

bool ContactWrenchMappingDoubleSupport::setFeetWeightPercentage(const double &weightInLeft,
                                                                 const double &weightInRight)
{
    iDynTree::VectorDynSize weightLeft(6), weightRight(6);

    for(int i = 0; i < 6; i++)
    {
        weightLeft(i) = m_regularizationForceScale * std::fabs(weightInLeft)
            + m_regularizationForceOffset;

        weightRight(i) = m_regularizationForceScale * std::fabs(weightInRight)
            + m_regularizationForceOffset;
    }

    auto cost = m_costFunctions.find("regularization_left_force");
    if(cost == m_costFunctions.end())
    {
        yError() << "[setDesiredNeckTrajectory] unable to find the neck trajectory element. "
                 << "Please call 'initialize()' method";
        return false;
    }
    auto ptr = std::static_pointer_cast<InputRegularizationTerm>(cost->second);
    ptr->setWeight(weightLeft);

    cost = m_costFunctions.find("regularization_right_force");
    if(cost == m_costFunctions.end())
    {
        yError() << "[setDesiredNeckTrajectory] unable to find the neck trajectory element. "
                 << "Please call 'initialize()' method";
        return false;
    }
    ptr = std::static_pointer_cast<InputRegularizationTerm>(cost->second);
    ptr->setWeight(weightRight);

    return true;
}

bool ContactWrenchMapping::setHessianMatrix()
{
    std::string key;
    Eigen::SparseMatrix<double> hessianEigen(m_numberOfVariables, m_numberOfVariables);
    for(const auto& element: m_costFunctions)
    {
        key = element.first;

        element.second->evaluateHessian(*(m_hessianMatrices.at(key)));
        hessianEigen += *(m_hessianMatrices.at(key));
    }

    if(m_optimizer->isInitialized())
    {
        if(!m_optimizer->updateHessianMatrix(hessianEigen))
        {
            yError() << "[setHessianMatrix] Unable to update the hessian matrix.";
            return false;
        }
    }
    else
    {
        if(!m_optimizer->data()->setHessianMatrix(hessianEigen))
        {
            yError() << "[setHessianMatrix] Unable to set first time the hessian matrix.";
            return false;
        }
    }

    m_hessianEigen = hessianEigen;

    return true;
}

bool ContactWrenchMapping::setGradientVector()
{
    std::string key;
    m_gradient = MatrixXd::Zero(m_numberOfVariables, 1);
    for(const auto& element: m_costFunctions)
    {
        key = element.first;
        element.second->evaluateGradient(*(m_gradientVectors.at(key)));
        m_gradient += *(m_gradientVectors.at(key));
    }

    if(m_optimizer->isInitialized())
    {
        if(!m_optimizer->updateGradient(m_gradient))
        {
            yError() << "[setGradient] Unable to update the gradient.";
            return false;
        }
    }
    else
    {
        if(!m_optimizer->data()->setGradient(m_gradient))
        {
            yError() << "[setGradient] Unable to set first time the gradient.";
            return false;
        }
    }

    return true;
}

bool ContactWrenchMapping::setLinearConstraintMatrix()
{
    for(const auto& constraint: m_constraints)
    {
        // m_profiler->setInitTime("add A");
        constraint.second->evaluateJacobian(m_constraintMatrix);
        // m_profiler->setEndTime("add A");
    }

    if(m_optimizer->isInitialized())
    {
        if(!m_optimizer->updateLinearConstraintsMatrix(m_constraintMatrix))
        {
            yError() << "[setLinearConstraintsMatrix] Unable to update the constraints matrix.";
            return false;
        }
    }
    else
    {
        if(!m_optimizer->data()->setLinearConstraintsMatrix(m_constraintMatrix))
        {
            yError() << "[setLinearConstraintsMatrix] Unable to set the constraints matrix.";
            return false;
        }
    }

    return true;
}

bool ContactWrenchMapping::setBounds()
{
    for(const auto& constraint: m_constraints)
        constraint.second->evaluateBounds(m_upperBound, m_lowerBound);

    if(m_optimizer->isInitialized())
    {
        if(!m_optimizer->updateBounds(m_lowerBound, m_upperBound))
        {
            yError() << "[setBounds] Unable to update the bounds.";
            return false;
        }
    }
    else
    {
        if(!m_optimizer->data()->setLowerBound(m_lowerBound))
        {
            yError() << "[setBounds] Unable to set the first time the lower bound.";
            return false;
        }

        if(!m_optimizer->data()->setUpperBound(m_upperBound))
        {
            yError() << "[setBounds] Unable to set the first time the upper bound.";
            return false;
        }
    }
    return true;
}

bool ContactWrenchMapping::solve()
{
    // m_profiler->setInitTime("hessian");

    if(!setHessianMatrix())
    {
        yError() << "[solve] Unable to set the hessian matrix.";
        return false;
    }

    // m_profiler->setEndTime("hessian");

    // m_profiler->setInitTime("gradient");

    if(!setGradientVector())
    {
        yError() << "[solve] Unable to set the gradient vector matrix.";
        return false;
    }

    // m_profiler->setEndTime("gradient");

    // m_profiler->setInitTime("A");

    if(!setLinearConstraintMatrix())
    {
        yError() << "[solve] Unable to set the linear constraint matrix.";
        return false;
    }

    // m_profiler->setEndTime("A");

    // m_profiler->setInitTime("bounds");
    if(!setBounds())
    {
        yError() << "[solve] Unable to set the bounds.";
        return false;
    }

    // m_profiler->setEndTime("bounds");

    // m_profiler->setInitTime("solve");
    if(!m_optimizer->isInitialized())
    {
        if(!m_optimizer->initSolver())
        {
            yError() << "[solve] Unable to initialize the solver";
            return false;
        }
    }

    if(!m_optimizer->solve())
    {
        // Eigen::MatioFile file("data.mat");
        // file.write_mat("hessian", Eigen::MatrixXd(m_hessianEigen));
        // file.write_mat("gradient", Eigen::MatrixXd(m_gradient));
        // file.write_mat("constraint", Eigen::MatrixXd(m_constraintMatrix));
        // file.write_mat("lowerBound", Eigen::MatrixXd(m_lowerBound));
        // file.write_mat("upperBound", Eigen::MatrixXd(m_upperBound));
        // file.write_mat("massMatrix", iDynTree::toEigen(m_massMatrix));
        // file.write_mat("comJacobian", iDynTree::toEigen(m_comJacobian));

        // for(const auto& element: m_costFunctions)
        // {
        //     std::string key = element.first;
        //     std::string hessianKey = key + "_hessian";
        //     std::string gradientKey = key + "_gradient";
        //     file.write_mat(hessianKey.c_str(), Eigen::MatrixXd(*(m_hessianMatrices.at(key))));
        //     file.write_mat(gradientKey.c_str(), Eigen::MatrixXd(*(m_gradientVectors.at(key))));
        // }


        yError() << "[solve] Unable to solve the problem.";
        return false;
    }

    iDynTree::toEigen(m_solution) = m_optimizer->getSolution();

    iDynTree::Vector3 totalForce;

    Eigen::Vector3d gravity;
    gravity << 0, 0, 9.81;

    iDynTree::toEigen(totalForce) = iDynTree::toEigen(m_solution).block(0, 0, 3, 0) +
        iDynTree::toEigen(m_solution).block(6, 0, 3, 0);

    yInfo() << "totalForce " << totalForce.toString();
    return true;
}

const iDynTree::VectorDynSize& ContactWrenchMapping::solution() const
{
    return m_solution;
}


bool ContactWrenchMappingDoubleSupport::instantiateContactForcesConstraint(const yarp::os::Searchable& config)
{
    if(config.isNull())
    {
        yError() << "[instantiateFeetConstraint] Empty configuration file.";
        return false;
    }

    double staticFrictionCoefficient;
    if(!YarpHelper::getNumberFromSearchable(config, "staticFrictionCoefficient",
                                            staticFrictionCoefficient))
    {
        yError() << "[initialize] Unable to get the number from searchable.";
        return false;
    }

    int numberOfPoints;
    if(!YarpHelper::getNumberFromSearchable(config, "numberOfPoints", numberOfPoints))
    {
        yError() << "[initialize] Unable to get the number from searchable.";
        return false;
    }

    double torsionalFrictionCoefficient;
    if(!YarpHelper::getNumberFromSearchable(config, "torsionalFrictionCoefficient",
                                            torsionalFrictionCoefficient))
    {
        yError() << "[initialize] Unable to get the number from searchable.";
        return false;
    }

    // feet dimensions
    yarp::os::Value feetDimensions = config.find("foot_size");
    if(feetDimensions.isNull() || !feetDimensions.isList())
    {
        yError() << "Please set the foot_size in the configuration file.";
        return false;
    }

    yarp::os::Bottle *feetDimensionsPointer = feetDimensions.asList();
    if(!feetDimensionsPointer || feetDimensionsPointer->size() != 2)
    {
        yError() << "Error while reading the feet dimensions. Wrong number of elements.";
        return false;
    }

    yarp::os::Value& xLimits = feetDimensionsPointer->get(0);
    if(xLimits.isNull() || !xLimits.isList())
    {
        yError() << "Error while reading the X limits.";
        return false;
    }

    yarp::os::Bottle *xLimitsPtr = xLimits.asList();
    if(!xLimitsPtr || xLimitsPtr->size() != 2)
    {
        yError() << "Error while reading the X limits. Wrong dimensions.";
        return false;
    }

    iDynTree::Vector2 footLimitX;
    footLimitX(0) = xLimitsPtr->get(0).asDouble();
    footLimitX(1) = xLimitsPtr->get(1).asDouble();

    yarp::os::Value& yLimits = feetDimensionsPointer->get(1);
    if(yLimits.isNull() || !yLimits.isList())
    {
        yError() << "Error while reading the Y limits.";
        return false;
    }

    yarp::os::Bottle *yLimitsPtr = yLimits.asList();
    if(!yLimitsPtr || yLimitsPtr->size() != 2)
    {
        yError() << "Error while reading the Y limits. Wrong dimensions.";
        return false;
    }

    iDynTree::Vector2 footLimitY;
    footLimitY(0) = yLimitsPtr->get(0).asDouble();
    footLimitY(1) = yLimitsPtr->get(1).asDouble();

    double minimalNormalForce;
    if(!YarpHelper::getNumberFromSearchable(config, "minimalNormalForce", minimalNormalForce))
    {
        yError() << "[initialize] Unable to get the number from searchable.";
        return false;
    }

    std::shared_ptr<ForceConstraint> ptr;

    // left foot
    ptr = std::make_shared<ForceConstraint>(numberOfPoints, staticFrictionCoefficient,
                                            torsionalFrictionCoefficient, minimalNormalForce,
                                            footLimitX, footLimitY);
    ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 0);
    ptr->setFootToWorldTransform(m_leftFootToWorldTransform);

    m_constraints.insert(std::make_pair("left_force", ptr));
    m_numberOfConstraints += ptr->getNumberOfConstraints();

    // right foot
    ptr = std::make_shared<ForceConstraint>(numberOfPoints, staticFrictionCoefficient,
                                            torsionalFrictionCoefficient, minimalNormalForce,
                                            footLimitX, footLimitY);
    ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 6);
    ptr->setFootToWorldTransform(m_rightFootToWorldTransform);

    m_constraints.insert(std::make_pair("right_force", ptr));
    m_numberOfConstraints += ptr->getNumberOfConstraints();

    return true;
}

bool ContactWrenchMappingDoubleSupport::instantiateForceRegularizationConstraint(const yarp::os::Searchable& config)
{
    yarp::os::Value tempValue;

    if(config.isNull())
    {
        yError() << "[instantiateRegularizationTaskConstraint] Empty configuration torque constraint.";
        return false;
    }

    if(!YarpHelper::getNumberFromSearchable(config, "regularizationForceScale", m_regularizationForceScale))
    {
        yError() << "[instantiateForceRegularizationConstraint] Unable to get regularization force scale";
        return false;
    }

    if(!YarpHelper::getNumberFromSearchable(config, "regularizationForceOffset", m_regularizationForceOffset))
    {
        yError() << "[instantiateForceRegularizationConstraint] Unable to get regularization force offset";
        return false;
    }

    // left foot
    std::shared_ptr<InputRegularizationTerm> ptr;
    ptr = std::make_shared<InputRegularizationTerm>(6);
    ptr->setSubMatricesStartingPosition(0, 0);
    m_costFunctions.insert(std::make_pair("regularization_left_force", ptr));
    m_hessianMatrices.insert(std::make_pair("regularization_left_force",
                                            std::make_unique<Eigen::SparseMatrix<double>>(m_numberOfVariables, m_numberOfVariables)));
    m_gradientVectors.insert(std::make_pair("regularization_left_force",
                                            std::make_unique<Eigen::VectorXd>(Eigen::VectorXd::Zero(m_numberOfVariables))));

    // right foot
    ptr = std::make_shared<InputRegularizationTerm>(6);
    ptr->setSubMatricesStartingPosition(6, 0);
    m_costFunctions.insert(std::make_pair("regularization_right_force", ptr));
    m_hessianMatrices.insert(std::make_pair("regularization_right_force",
                                            std::make_unique<Eigen::SparseMatrix<double>>(m_numberOfVariables, m_numberOfVariables)));
    m_gradientVectors.insert(std::make_pair("regularization_right_force",
                                            std::make_unique<Eigen::VectorXd>(Eigen::VectorXd::Zero(m_numberOfVariables))));
    return true;
}

void ContactWrenchMappingDoubleSupport::instantiateLinearMomentumConstraint(const yarp::os::Searchable& config)
{
    m_useLinearMomentumConstraint = config.check("useAsConstraint", yarp::os::Value("False")).asBool();
    if(m_useLinearMomentumConstraint)
    {

        yarp::os::Value kpValue = config.find("kp");
        iDynTree::VectorDynSize kp(3);
        if(!YarpHelper::yarpListToiDynTreeVectorDynSize(kpValue, kp))
        {
            yError() << "[ContactWrenchMappingSingleSupport::instantiateLinearMomentumCostFunction] Initialization failed while reading kp vector.";
            return;
        }


        yarp::os::Value kdValue = config.find("kd");
        iDynTree::VectorDynSize kd(3);
        if(!YarpHelper::yarpListToiDynTreeVectorDynSize(kdValue, kd))
        {
            yError() << "[ContactWrenchMappingSingleSupport::instantiateLinearMomentumCostFunction] Initialization failed while reading kd vector.";
            return;
        }

        // memory allocation
        auto ptr = std::make_shared<LinearMomentumConstraint>(LinearMomentumConstraint::Type::DOUBLE_SUPPORT);
        // only the forces are used to control the linear momentum
        ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 0);
        ptr->setKd(kd);
        ptr->setKp(kp);

        m_constraints.insert(std::make_pair("linear_momentum_constraint", ptr));
        m_numberOfConstraints += ptr->getNumberOfConstraints();
    }
    return;
}

bool ContactWrenchMappingDoubleSupport::instantiateLinearMomentumCostFunction(const yarp::os::Searchable& config)
{
    m_useLinearMomentumCostFunction = config.check("useAsCostFunction", yarp::os::Value("False")).asBool();
    if(m_useLinearMomentumCostFunction)
    {
        yarp::os::Value tempValue = config.find("weight");
        iDynTree::VectorDynSize weight(3);
        if(!YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, weight))
        {
            yError() << "[ContactWrenchMappingDoubleSupport::instantiateLinearMomentumCostFunction] Initialization failed while reading weight vector.";
            return false;
        }



        // memory allocation
        auto ptr = std::make_shared<LinearMomentumCostFunction>(LinearMomentumCostFunction::Type::DOUBLE_SUPPORT);
        // only the forces are used to control the linear momentum
        ptr->setSubMatricesStartingPosition(0, 0);
        ptr->setWeight(weight);

        m_costFunctions.insert(std::make_pair("linear_momentum_costFunction", ptr));

        m_hessianMatrices.insert(std::make_pair("linear_momentum_costFunction",
                                                std::make_unique<Eigen::SparseMatrix<double>>(m_numberOfVariables, m_numberOfVariables)));
        m_gradientVectors.insert(std::make_pair("linear_momentum_costFunction",
                                                std::make_unique<Eigen::VectorXd>(Eigen::VectorXd::Zero(m_numberOfVariables))));
    }
    return true;
}

void ContactWrenchMapping::setRobotMass(double mass)
{
    if(!m_optimizer->isInitialized())
    {
        auto constraint = m_constraints.find("linear_momentum_constraint");
        if(constraint == m_constraints.end())
        {
            yError() << "[setMassMatrix] unable to find the linear momentum constraint. "
                     << "Please call 'initialize()' method";
            return;
        }

        // TODO remove me
        m_mass = mass;

        auto ptr = std::static_pointer_cast<LinearMomentumConstraint>(constraint->second);
        ptr->setRobotMass(mass);
    }
}

void ContactWrenchMappingDoubleSupport::setNumberOfVariables()
{
    // the optimization variable is given by
    // 1. base + joint acceleration (6 + m_actuatedDOFs)
    // 2. joint torque (m_actuatedDOFs)
    // 3. left and right foot contact wrench (6 + 6)

    m_numberOfVariables =  6 + 6;
}

void ContactWrenchMappingDoubleSupport::setFeetState(const iDynTree::Transform& leftFootToWorldTransform,
                                                      const iDynTree::Transform& rightFootToWorldTransform)
{
    m_leftFootToWorldTransform = leftFootToWorldTransform;
    m_rightFootToWorldTransform = rightFootToWorldTransform;
}

void ContactWrenchMappingDoubleSupport::setFeetJacobian(const iDynTree::MatrixDynSize& leftFootJacobian,
                                                         const iDynTree::MatrixDynSize& rightFootJacobian)
{
    m_rightFootJacobian = rightFootJacobian;
    m_leftFootJacobian = leftFootJacobian;
}

bool ContactWrenchMappingSingleSupport::instantiateContactForcesConstraint(const yarp::os::Searchable& config)
{
    if(config.isNull())
    {
        yError() << "[instantiateFeetConstraint] Empty configuration file.";
        return false;
    }

    double staticFrictionCoefficient;
    if(!YarpHelper::getNumberFromSearchable(config, "staticFrictionCoefficient",
                                            staticFrictionCoefficient))
    {
        yError() << "[initialize] Unable to get the number from searchable.";
        return false;
    }

    int numberOfPoints;
    if(!YarpHelper::getNumberFromSearchable(config, "numberOfPoints", numberOfPoints))
    {
        yError() << "[initialize] Unable to get the number from searchable.";
        return false;
    }

    double torsionalFrictionCoefficient;
    if(!YarpHelper::getNumberFromSearchable(config, "torsionalFrictionCoefficient",
                                            torsionalFrictionCoefficient))
    {
        yError() << "[initialize] Unable to get the number from searchable.";
        return false;
    }

    // feet dimensions
    yarp::os::Value feetDimensions = config.find("foot_size");
    if(feetDimensions.isNull() || !feetDimensions.isList())
    {
        yError() << "Please set the foot_size in the configuration file.";
        return false;
    }

    yarp::os::Bottle *feetDimensionsPointer = feetDimensions.asList();
    if(!feetDimensionsPointer || feetDimensionsPointer->size() != 2)
    {
        yError() << "Error while reading the feet dimensions. Wrong number of elements.";
        return false;
    }

    yarp::os::Value& xLimits = feetDimensionsPointer->get(0);
    if(xLimits.isNull() || !xLimits.isList())
    {
        yError() << "Error while reading the X limits.";
        return false;
    }

    yarp::os::Bottle *xLimitsPtr = xLimits.asList();
    if(!xLimitsPtr || xLimitsPtr->size() != 2)
    {
        yError() << "Error while reading the X limits. Wrong dimensions.";
        return false;
    }

    iDynTree::Vector2 footLimitX;
    footLimitX(0) = xLimitsPtr->get(0).asDouble();
    footLimitX(1) = xLimitsPtr->get(1).asDouble();

    yarp::os::Value& yLimits = feetDimensionsPointer->get(1);
    if(yLimits.isNull() || !yLimits.isList())
    {
        yError() << "Error while reading the Y limits.";
        return false;
    }

    yarp::os::Bottle *yLimitsPtr = yLimits.asList();
    if(!yLimitsPtr || yLimitsPtr->size() != 2)
    {
        yError() << "Error while reading the Y limits. Wrong dimensions.";
        return false;
    }

    iDynTree::Vector2 footLimitY;
    footLimitY(0) = yLimitsPtr->get(0).asDouble();
    footLimitY(1) = yLimitsPtr->get(1).asDouble();

    double minimalNormalForce;
    if(!YarpHelper::getNumberFromSearchable(config, "minimalNormalForce", minimalNormalForce))
    {
        yError() << "[initialize] Unable to get the number from searchable.";
        return false;
    }

    std::shared_ptr<ForceConstraint> ptr;

    // stance foot
    ptr = std::make_shared<ForceConstraint>(numberOfPoints, staticFrictionCoefficient,
                                            torsionalFrictionCoefficient, minimalNormalForce,
                                            footLimitX, footLimitY);

    ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 0);
    ptr->setFootToWorldTransform(m_stanceFootToWorldTransform);

    m_constraints.insert(std::make_pair("stance_force", ptr));
    m_numberOfConstraints += ptr->getNumberOfConstraints();

    return true;
}

bool ContactWrenchMappingSingleSupport::instantiateForceRegularizationConstraint(const yarp::os::Searchable& config)
{
    yarp::os::Value tempValue;

    if(config.isNull())
    {
        yError() << "[instantiateRegularizationTaskConstraint] Empty configuration torque constraint.";
        return false;
    }

    if(!YarpHelper::getNumberFromSearchable(config, "regularizationForceScale", m_regularizationForceScale))
    {
        yError() << "[instantiateForceRegularizationConstraint] Unable to get regularization force scale";
        return false;
    }

    if(!YarpHelper::getNumberFromSearchable(config, "regularizationForceOffset", m_regularizationForceOffset))
    {
        yError() << "[instantiateForceRegularizationConstraint] Unable to get regularization force offset";
        return false;
    }

    // the weight is constant in stance phase
    iDynTree::VectorDynSize weight(6);
    for(int i = 0; i < 6; i++)
        weight(i) = m_regularizationForceScale + m_regularizationForceOffset;

    std::shared_ptr<InputRegularizationTerm> ptr;
    ptr = std::make_shared<InputRegularizationTerm>(6);
    ptr->setSubMatricesStartingPosition(0, 0);
    ptr->setWeight(weight);


    m_costFunctions.insert(std::make_pair("regularization_stance_force", ptr));

    m_hessianMatrices.insert(std::make_pair("regularization_stance_force",
                                            std::make_unique<Eigen::SparseMatrix<double>>(m_numberOfVariables, m_numberOfVariables)));
    m_gradientVectors.insert(std::make_pair("regularization_stance_force",
                                            std::make_unique<Eigen::VectorXd>(Eigen::VectorXd::Zero(m_numberOfVariables))));

    return true;
}

void ContactWrenchMappingSingleSupport::instantiateLinearMomentumConstraint(const yarp::os::Searchable& config)
{
    m_useLinearMomentumConstraint = config.check("useAsConstraint", yarp::os::Value("False")).asBool();
    if(!m_useLinearMomentumConstraint)
    {
        yWarning() << "[ContactWrenchMappingSingleSupport::instantiateLinearMomentumConstraint] The linear momentum will not be used as a constraint";
        return;
    }

    yarp::os::Value kpValue = config.find("kp");
    iDynTree::VectorDynSize kp(3);
    if(!YarpHelper::yarpListToiDynTreeVectorDynSize(kpValue, kp))
    {
        yError() << "[ContactWrenchMappingSingleSupport::instantiateLinearMomentumCostFunction] Initialization failed while reading kp vector.";
        return;
    }


    yarp::os::Value kdValue = config.find("kd");
    iDynTree::VectorDynSize kd(3);
    if(!YarpHelper::yarpListToiDynTreeVectorDynSize(kdValue, kd))
    {
        yError() << "[ContactWrenchMappingSingleSupport::instantiateLinearMomentumCostFunction] Initialization failed while reading kd vector.";
        return;
    }


    if(m_useLinearMomentumConstraint)
    {
        // memory allocation
        auto ptr = std::make_shared<LinearMomentumConstraint>(LinearMomentumConstraint::Type::SINGLE_SUPPORT);
        // only the forces are used to control the linear momentum
        ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 0);
        ptr->setKp(kp);
        ptr->setKd(kd);

        m_constraints.insert(std::make_pair("linear_momentum_constraint", ptr));
        m_numberOfConstraints += ptr->getNumberOfConstraints();
    }
    return;
}

bool ContactWrenchMappingSingleSupport::instantiateLinearMomentumCostFunction(const yarp::os::Searchable& config)
{
    m_useLinearMomentumCostFunction = config.check("useAsCostFunction", yarp::os::Value("False")).asBool();
    if(m_useLinearMomentumCostFunction)
    {
        yarp::os::Value tempValue = config.find("weight");
        iDynTree::VectorDynSize weight(3);
        if(!YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, weight))
        {
            yError() << "[ContactWrenchMappingSingleSupport::instantiateLinearMomentumCostFunction] Initialization failed while reading weight vector.";
            return false;
        }

        // memory allocation
        auto ptr = std::make_shared<LinearMomentumCostFunction>(LinearMomentumCostFunction::Type::SINGLE_SUPPORT);
        // only the forces are used to control the linear momentum
        ptr->setSubMatricesStartingPosition(0, 0);
        ptr->setWeight(weight);

        m_costFunctions.insert(std::make_pair("linear_momentum_costFunction", ptr));

        m_hessianMatrices.insert(std::make_pair("linear_momentum_costFunction",
                                                std::make_unique<Eigen::SparseMatrix<double>>(m_numberOfVariables, m_numberOfVariables)));
        m_gradientVectors.insert(std::make_pair("linear_momentum_costFunction",
                                                std::make_unique<Eigen::VectorXd>(Eigen::VectorXd::Zero(m_numberOfVariables))));
    }
    return true;
}

void ContactWrenchMappingSingleSupport::setNumberOfVariables()
{
    // the optimization variable is given by
    // 1. base + joint acceleration
    // 2. joint torque
    // 3. stance foot contact wrench

    m_numberOfVariables = 6;
}


bool ContactWrenchMappingSingleSupport::setFeetState(const iDynTree::Transform& stanceFootToWorldTransform)
{
    m_stanceFootToWorldTransform = stanceFootToWorldTransform;
    return true;
}

void ContactWrenchMappingSingleSupport::setFeetJacobian(const iDynTree::MatrixDynSize& stanceFootJacobian,
                                                         const iDynTree::MatrixDynSize& swingFootJacobian)
{
    m_stanceFootJacobian = stanceFootJacobian;
}


bool ContactWrenchMappingSingleSupport::instantiateAngularMomentumConstraint(const yarp::os::Searchable& config)
{
    if(config.isNull())
    {
        yInfo() << "[instantiateAngularMomentumConstraint] Empty configuration file. The angular momentum Constraint will not be used";
        m_useAngularMomentumConstraint = false;
        return true;
    }
    m_useAngularMomentumConstraint = true;

    double kp;
    if(!YarpHelper::getNumberFromSearchable(config, "kp", kp))
    {
        yError() << "[instantiateAngularMomentumConstraint] Unable to get proportional gain";
        return false;
    }

    double ki;
    if(!YarpHelper::getNumberFromSearchable(config, "ki", ki))
    {
        yError() << "[instantiateAngularMomentumConstraint] Unable to get proportional gain";
        return false;
    }

    std::shared_ptr<AngularMomentumConstraintSingleSupport> ptr;
    ptr = std::make_shared<AngularMomentumConstraintSingleSupport>();
    ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 0);
    ptr->setKp(kp);
    ptr->setKi(ki);

    ptr->setStanceFootToWorldTransform(m_stanceFootToWorldTransform);

    m_constraints.insert(std::make_pair("angular_momentum_constraint", ptr));
    m_numberOfConstraints += ptr->getNumberOfConstraints();

    return true;
}

bool ContactWrenchMappingDoubleSupport::instantiateAngularMomentumConstraint(const yarp::os::Searchable& config)
{
    if(config.isNull())
    {
        yInfo() << "[instantiateAngularMomentumConstraint] Empty configuration file. The angular momentum Constraint will not be used";
        m_useAngularMomentumConstraint = false;
        return true;
    }
    m_useAngularMomentumConstraint = true;

    double kp;
    if(!YarpHelper::getNumberFromSearchable(config, "kp", kp))
    {
        yError() << "[instantiateAngularMomentumConstraint] Unable to get proportional gain";
        return false;
    }

    double ki;
    if(!YarpHelper::getNumberFromSearchable(config, "ki", ki))
    {
        yError() << "[instantiateAngularMomentumConstraint] Unable to get proportional gain";
        return false;
    }

    std::shared_ptr<AngularMomentumConstraintDoubleSupport> ptr;
    ptr = std::make_shared<AngularMomentumConstraintDoubleSupport>();
    ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 0);
    ptr->setKp(kp);
    ptr->setKi(ki);

    ptr->setLeftFootToWorldTransform(m_leftFootToWorldTransform);
    ptr->setRightFootToWorldTransform(m_rightFootToWorldTransform);

    m_constraints.insert(std::make_pair("angular_momentum_constraint", ptr));
    m_numberOfConstraints += ptr->getNumberOfConstraints();

    return true;
}

bool ContactWrenchMappingSingleSupport::instantiateAngularMomentumCostFunction(const yarp::os::Searchable& config)
{
    m_useAngularMomentumCostFunction = config.check("useAsCostFunction", yarp::os::Value("False")).asBool();
    if(m_useAngularMomentumCostFunction)
    {
        yarp::os::Value tempValue = config.find("weight");
        iDynTree::VectorDynSize weight(3);
        if(!YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, weight))
        {
            yError() << "[ContactWrenchMappingSingleSupport::instantiateAngularMomentumCostFunction] Initialization failed while reading weight vector.";
            return false;
        }

        double kp;
        if(!YarpHelper::getNumberFromSearchable(config, "kp", kp))
        {
            yError() << "[instantiateAngularMomentumConstraint] Unable to get proportional gain";
            return false;
        }

        double ki;
        if(!YarpHelper::getNumberFromSearchable(config, "ki", ki))
        {
            yError() << "[instantiateAngularMomentumConstraint] Unable to get proportional gain";
            return false;
        }

        auto ptr = std::make_shared<AngularMomentumCostFunctionSingleSupport>();
        ptr->setSubMatricesStartingPosition(0, 0);
        ptr->setWeight(weight);
        ptr->setKp(kp);
        ptr->setKi(ki);

        ptr->setStanceFootToWorldTransform(m_stanceFootToWorldTransform);

        m_costFunctions.insert(std::make_pair("angular_momentum_costFunction", ptr));

        m_hessianMatrices.insert(std::make_pair("angular_momentum_costFunction",
                                                std::make_unique<Eigen::SparseMatrix<double>>(m_numberOfVariables, m_numberOfVariables)));
        m_gradientVectors.insert(std::make_pair("angular_momentum_costFunction",
                                                std::make_unique<Eigen::VectorXd>(Eigen::VectorXd::Zero(m_numberOfVariables))));
    }
    return true;
}

bool ContactWrenchMappingDoubleSupport::instantiateAngularMomentumCostFunction(const yarp::os::Searchable& config)
{
    m_useAngularMomentumCostFunction = config.check("useAsCostFunction", yarp::os::Value("False")).asBool();
    if(m_useAngularMomentumCostFunction)
    {
        yarp::os::Value tempValue = config.find("weight");
        iDynTree::VectorDynSize weight(3);
        if(!YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, weight))
        {
            yError() << "[ContactWrenchMappingSingleSupport::instantiateAngularMomentumCostFunction] Initialization failed while reading weight vector.";
            return false;
        }

        double kp;
        if(!YarpHelper::getNumberFromSearchable(config, "kp", kp))
        {
            yError() << "[instantiateAngularMomentumConstraint] Unable to get proportional gain";
            return false;
        }

        double ki;
        if(!YarpHelper::getNumberFromSearchable(config, "ki", ki))
        {
            yError() << "[instantiateAngularMomentumConstraint] Unable to get proportional gain";
            return false;
        }

        auto ptr = std::make_shared<AngularMomentumCostFunctionDoubleSupport>();
        ptr->setSubMatricesStartingPosition(0, 0);
        ptr->setWeight(weight);
        ptr->setKp(kp);
        ptr->setKi(ki);


        ptr->setLeftFootToWorldTransform(m_leftFootToWorldTransform);
        ptr->setRightFootToWorldTransform(m_rightFootToWorldTransform);

        m_costFunctions.insert(std::make_pair("angular_momentum_costFunction", ptr));

        m_hessianMatrices.insert(std::make_pair("angular_momentum_costFunction",
                                                std::make_unique<Eigen::SparseMatrix<double>>(m_numberOfVariables, m_numberOfVariables)));
        m_gradientVectors.insert(std::make_pair("angular_momentum_costFunction",
                                                std::make_unique<Eigen::VectorXd>(Eigen::VectorXd::Zero(m_numberOfVariables))));
    }
    return true;
}
