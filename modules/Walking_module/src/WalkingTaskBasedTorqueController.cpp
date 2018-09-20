/**
 * @file WalkingTaskBasedTorqueController.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <iDynTree/yarp/YARPConfigurationsLoader.h>
#include <iDynTree/Core/Twist.h>

#include <WalkingTaskBasedTorqueController.hpp>
#include <Utils.hpp>

//todo
#include <eigen3/Eigen/Eigenvalues> // header file


typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXd;

bool WalkingTaskBasedTorqueController::instantiateCoMConstraint(const yarp::os::Searchable& config)
{
    if(config.isNull())
    {
        yError() << "[instantiateCoMConstraint] Empty configuration file.";
        return false;
    }

    double kp;
    if(!YarpHelper::getNumberFromSearchable(config, "kp", kp))
    {
        yError() << "[instantiateCoMConstraint] Unable to get proportional gain";
        return false;
    }

    double kd;
    if(!YarpHelper::getNumberFromSearchable(config, "kd", kd))
    {
        yError() << "[instantiateCoMConstraint] Unable to get derivative gain";
        return false;
    }

    // memory allocation
    std::shared_ptr<Constraint<iDynTree::MatrixDynSize, iDynSparseMatrix>> ptr;
    std::shared_ptr<PositionConstraint> ptrCasted;

    ptr = std::make_shared<PositionConstraint>(m_numberOfVariables);
    ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 0);

    ptrCasted = std::static_pointer_cast<PositionConstraint>(ptr);
    ptrCasted->positionController()->setGains(kp, kd);

    m_constraints.insert(std::make_pair("com", ptr));
    m_numberOfConstraints += ptr->getNumberOfConstraints();

    return true;
}

bool WalkingTaskBasedTorqueController::instantiateFeetConstraint(const yarp::os::Searchable& config)
{
    if(config.isNull())
    {
        yError() << "[instantiateFeetConstraint] Empty configuration file.";
        return false;
    }

    double kp;
    if(!YarpHelper::getNumberFromSearchable(config, "kp", kp))
    {
        yError() << "[instantiateFeetConstraint] Unable to get proportional gain";
        return false;
    }

    double kd;
    if(!YarpHelper::getNumberFromSearchable(config, "kd", kd))
    {
        yError() << "[instantiateFeetConstraint] Unable to get derivative gain";
        return false;
    }

    double c0;
    if(!YarpHelper::getNumberFromSearchable(config, "c0", c0))
    {
        yError() << "[instantiateFeetConstraint] Unable to get c0";
        return false;
    }

    double c1;
    if(!YarpHelper::getNumberFromSearchable(config, "c1", c1))
    {
        yError() << "[instantiateFeetConstraint] Unable to get c1";
        return false;
    }

    double c2;
    if(!YarpHelper::getNumberFromSearchable(config, "c2", c2))
    {
        yError() << "[instantiateFeetConstraint] Unable to get c2";
        return false;
    }

    std::shared_ptr<Constraint<iDynTree::MatrixDynSize, iDynSparseMatrix>> ptr;
    std::shared_ptr<CartesianConstraint> ptrCasted;

    // left foot
    ptr = std::make_shared<CartesianConstraint>(m_numberOfVariables);
    ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 0);

    ptrCasted = std::static_pointer_cast<CartesianConstraint>(ptr);
    ptrCasted->positionController()->setGains(kp, kd);
    ptrCasted->orientationController()->setGains(c0, c1, c2);

    m_constraints.insert(std::make_pair("left_foot", ptr));
    m_numberOfConstraints += ptr->getNumberOfConstraints();

    // right foot
    ptr = std::make_shared<CartesianConstraint>(m_numberOfVariables);
    ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 0);

    ptrCasted = std::static_pointer_cast<CartesianConstraint>(ptr);
    ptrCasted->positionController()->setGains(kp, kd);
    ptrCasted->orientationController()->setGains(c0, c1, c2);

    m_constraints.insert(std::make_pair("right_foot", ptr));
    m_numberOfConstraints += ptr->getNumberOfConstraints();

    return true;
}

void WalkingTaskBasedTorqueController::instantiateZMPConstraint()
{
    std::shared_ptr<Constraint<iDynTree::MatrixDynSize, iDynSparseMatrix>> ptr;
    ptr = std::make_shared<ZMPConstraint>();
    ptr->setSubMatricesStartingPosition(m_numberOfConstraints, m_actuatedDOFs);
    m_constraints.insert(std::make_pair("zmp", ptr));
    m_numberOfConstraints += ptr->getNumberOfConstraints();
}

bool WalkingTaskBasedTorqueController::instantiateContactForcesConstraint(const yarp::os::Searchable& config)
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

    std::shared_ptr<Constraint<iDynTree::MatrixDynSize, iDynSparseMatrix>> ptr;
    std::shared_ptr<ForceConstraint> ptrCasted;

    // left foot
    ptr = std::make_shared<ForceConstraint>(numberOfPoints);
    ptr->setSubMatricesStartingPosition(m_numberOfConstraints, m_actuatedDOFs);

    ptrCasted = std::static_pointer_cast<ForceConstraint>(ptr);
    ptrCasted->setStaticFrictionCoefficient(staticFrictionCoefficient);
    ptrCasted->setTorsionalFrictionCoefficient(torsionalFrictionCoefficient);
    ptrCasted->setMinimalNormalForce(minimalNormalForce);
    ptrCasted->setFootSize(footLimitX, footLimitY);

    m_constraints.insert(std::make_pair("left_force", ptr));
    m_numberOfConstraints += ptr->getNumberOfConstraints();

    // right foot
    ptr = std::make_shared<ForceConstraint>(numberOfPoints);
    ptr->setSubMatricesStartingPosition(m_numberOfConstraints, m_actuatedDOFs + 6);

    ptrCasted = std::static_pointer_cast<ForceConstraint>(ptr);
    ptrCasted->setStaticFrictionCoefficient(staticFrictionCoefficient);
    ptrCasted->setTorsionalFrictionCoefficient(torsionalFrictionCoefficient);
    ptrCasted->setMinimalNormalForce(minimalNormalForce);
    ptrCasted->setFootSize(footLimitX, footLimitY);

    m_constraints.insert(std::make_pair("right_force", ptr));
    m_numberOfConstraints += ptr->getNumberOfConstraints();

    return true;
}

bool WalkingTaskBasedTorqueController::instantiateNeckSoftConstraint(const yarp::os::Searchable& config)
{
    yarp::os::Value tempValue;
    m_neckOrientationController = std::make_unique<RotationalPID>();
    if(config.isNull())
    {
        yError() << "[instantiateNeckSoftConstraint] Empty configuration neck soft constraint.";
        return false;
    }

    // get the neck weight
    tempValue = config.find("neckWeight");
    m_neckOrientationWeight.resize(3);
    if(!YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, m_neckOrientationWeight))
    {
        yError() << "[instantiateNeckSoftConstraint] Initialization failed while reading neckWeight.";
        return false;
    }

    double c0, c1, c2;
    if(!YarpHelper::getNumberFromSearchable(config, "c0", c0))
        return false;

    if(!YarpHelper::getNumberFromSearchable(config, "c1", c1))
        return false;

    if(!YarpHelper::getNumberFromSearchable(config, "c2", c2))
        return false;

    m_neckOrientationController->setGains(c0, c1, c2);

    if(!iDynTree::parseRotationMatrix(config, "additional_rotation", m_additionalRotation))
    {
        yError() << "[instantiateNeckSoftConstraint] Unable to set the additional rotation.";
        return false;
    }

    m_neckJacobian.resize(3, m_actuatedDOFs + 6);

    return true;
}

bool WalkingTaskBasedTorqueController::instantiateRegularizationTaskConstraint(const yarp::os::Searchable& config)
{
    yarp::os::Value tempValue;

    if(config.isNull())
    {
        yError() << "[instantiateRegularizationTaskConstraint] Empty configuration joint task constraint.";
        return false;
    }

    m_desiredJointPosition.resize(m_actuatedDOFs);
    m_desiredJointVelocity.resize(m_actuatedDOFs);
    m_desiredJointAcceleration.resize(m_actuatedDOFs);
    m_desiredJointVelocity.zero();
    m_desiredJointAcceleration.zero();
    tempValue = config.find("jointRegularization");
    if(!YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, m_desiredJointPosition))
    {
        yError() << "[initialize] Unable to convert a YARP list to an iDynTree::VectorDynSize, "
                 << "joint regularization";
        return false;
    }

    iDynTree::toEigen(m_regularizationTerm) = iDynTree::toEigen(m_regularizationTerm) *
        iDynTree::deg2rad(1);

    // set the matrix related to the joint regularization
    tempValue = config.find("jointRegularizationWeights");
    m_jointRegularizationWeights.resize(m_actuatedDOFs);
    if(!YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, m_jointRegularizationWeights))
    {
        yError() << "Initialization failed while reading jointRegularizationWeights vector.";
        return false;
    }

    // set the matrix related to the joint regularization
    tempValue = config.find("jointRegularizationProportionalGains");
    m_jointRegularizationProportionalGains.resize(m_actuatedDOFs);
    if(!YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue,
                                                    m_jointRegularizationProportionalGains))
    {
        yError() << "Initialization failed while reading jointRegularizationProportionalGains vector.";
        return false;
    }

    tempValue = config.find("jointRegularizationDerivativeGains");
    m_jointRegularizationDerivativeGains.resize(m_actuatedDOFs);
    if(!YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, m_jointRegularizationDerivativeGains))
    {
        yError() << "Initialization failed while reading jointRegularizationDerivativeGains vector.";
        return false;
    }

    // todo move in a separate class
    m_desiredJointAccelerationController.resize(m_actuatedDOFs);

    return true;
}

void WalkingTaskBasedTorqueController::instantiateConstPartInputMatrix()
{
    // set selection matrix
    iDynTree::Triplets selectionMatrixTriplets;
    selectionMatrixTriplets.setDiagonalMatrix(0, 6, 1, m_actuatedDOFs);
    m_selectionMatrix.resize(m_actuatedDOFs, m_actuatedDOFs + 6);
    m_selectionMatrix.setFromConstTriplets(selectionMatrixTriplets);

    // set constant part of the inputMatrix
    iDynTree::toEigen(m_inputMatrix).block(0, 0, m_actuatedDOFs + 6, m_actuatedDOFs) =
        iDynTree::toEigen(m_selectionMatrix).transpose();
}

bool WalkingTaskBasedTorqueController::initialize(const yarp::os::Searchable& config,
                                                  const int& actuatedDOFs,
                                                  const iDynTree::VectorDynSize& minJointTorque,
                                                  const iDynTree::VectorDynSize& maxJointTorque)
{
    m_actuatedDOFs = actuatedDOFs;
    m_numberOfVariables = m_actuatedDOFs + 6 + 6;
    m_numberOfConstraints = 0;

    // resize matrices
    m_inputMatrix.resize(m_actuatedDOFs + 6, m_actuatedDOFs + 6 + 6);
    m_massMatrix.resize(m_actuatedDOFs + 6, m_actuatedDOFs + 6);
    m_generalizedBiasForces.resize(m_actuatedDOFs + 6);
    m_leftFootJacobian.resize(6, m_actuatedDOFs + 6);
    m_rightFootJacobian.resize(6, m_actuatedDOFs + 6);
    m_minTorqueLimit.resize(m_actuatedDOFs + 6 + 6);
    m_maxTorqueLimit.resize(m_actuatedDOFs + 6 + 6);

    // check if the config is empty
    if(config.isNull())
    {
        yError() << "[initialize] Empty configuration for Task based torque solver.";
        return false;
    }

    // instantiate constraints
    yarp::os::Bottle& comConstraintOptions = config.findGroup("COM");
    if(!instantiateCoMConstraint(comConstraintOptions))
    {
        yError() << "[initialize] Unable to get the instantiate the CoM constraint.";
        return false;
    }

    yarp::os::Bottle& feetConstraintOptions = config.findGroup("FEET");
    if(!instantiateFeetConstraint(feetConstraintOptions))
    {
        yError() << "[initialize] Unable to get the instantiate the feet constraints.";
        return false;
    }

    instantiateZMPConstraint();

    yarp::os::Bottle& contactForcesOption = config.findGroup("CONTACT_FORCES");
    if(!instantiateContactForcesConstraint(contactForcesOption))
    {
        yError() << "[initialize] Unable to get the instantiate the feet constraints.";
        return false;
    }

    // instantiate cost function
    yarp::os::Bottle& neckOrientationOption = config.findGroup("NECK_ORIENTATION");
    if(!instantiateNeckSoftConstraint(neckOrientationOption))
    {
        yError() << "[initialize] Unable to get the instantiate the neck constraint.";
        return false;
    }

    yarp::os::Bottle& regularizationTaskOption = config.findGroup("REGULARIZATION_TASK");
    if(!instantiateRegularizationTaskConstraint(regularizationTaskOption))
    {
        yError() << "[initialize] Unable to get the instantiate the regularization constraint.";
        return false;
    }

    instantiateConstPartInputMatrix();

    m_hessian.resize(m_numberOfVariables * m_numberOfVariables);
    m_gradient.resize(m_numberOfVariables);
    m_constraintMatrix.resize(m_numberOfVariables * m_numberOfConstraints);
    // set all the element of the constraint matrix equal to 0 (note the constraint matrix is block
    // defined)
    std::fill(m_constraintMatrix.begin(), m_constraintMatrix.end(), 0);
    m_upperBound.resize(m_numberOfConstraints);
    m_lowerBound.resize(m_numberOfConstraints);


    for(int i = 0; i<m_actuatedDOFs; i++)
    {
        m_minTorqueLimit(i) = -qpOASES::INFTY;
        m_maxTorqueLimit(i) = qpOASES::INFTY;
    }

    for(int i = m_actuatedDOFs; i<m_numberOfVariables; i++)
    {
        m_minTorqueLimit(i) = -qpOASES::INFTY;
        m_maxTorqueLimit(i) = qpOASES::INFTY;
    }

    // initialize the optimization problem
    m_optimizer = std::make_shared<qpOASES::SQProblem>(m_numberOfVariables, m_numberOfConstraints);
    // m_optimizer->setPrintLevel(qpOASES::PL_LOW);
    m_isFirstTime = true;
    m_isSolutionEvaluated = false;

    return true;
}

bool WalkingTaskBasedTorqueController::setMassMatrix(const iDynTree::MatrixDynSize& massMatrix)
{
    if((massMatrix.rows() != m_actuatedDOFs + 6)
       ||(massMatrix.rows() != m_actuatedDOFs + 6))
    {
        yError() << "[setMassMatrix] The size of the massMatrix is not coherent "
                 << "with the number of the actuated Joint plus six";
        return false;
    }
    m_massMatrix = massMatrix;

    // set mass matrix for each constraint (probably this will be removed)
    std::shared_ptr<GenericCartesianConstraint> ptr;

    auto constraint = m_constraints.find("left_foot");
    if(constraint == m_constraints.end())
    {
        yError() << "[setMassMatrix] unable to find the left foot constraint. "
                 << "Please call 'initialize()' method";
        return false;
    }
    ptr = std::static_pointer_cast<GenericCartesianConstraint>(constraint->second);
    ptr->setMassMatrix(massMatrix);

    constraint = m_constraints.find("right_foot");
    if(constraint == m_constraints.end())
    {
        yError() << "[setMassMatrix] unable to find the right foot constraint. "
                 << "Please call 'initialize()' method";
        return false;
    }
    ptr = std::static_pointer_cast<GenericCartesianConstraint>(constraint->second);
    ptr->setMassMatrix(massMatrix);

    constraint = m_constraints.find("com");
    if(constraint == m_constraints.end())
    {
        yError() << "[setMassMatrix] unable to find the right foot constraint. "
                 << "Please call 'initialize()' method";
        return false;
    }
    ptr = std::static_pointer_cast<GenericCartesianConstraint>(constraint->second);
    ptr->setMassMatrix(massMatrix);

    return true;
}

bool WalkingTaskBasedTorqueController::setGeneralizedBiasForces(const iDynTree::VectorDynSize& generalizedBiasForces)
{
    if(generalizedBiasForces.size() != m_actuatedDOFs + 6)
    {
        yError() << "[setGeneralizedBiasForces] The size of generalizedBiasForces has to be "
                 << m_actuatedDOFs + 6;
        return false;
    }

    m_generalizedBiasForces = generalizedBiasForces;

    // set mass matrix for each constraint (probably this will be removed)
    std::shared_ptr<GenericCartesianConstraint> ptr;

    auto constraint = m_constraints.find("left_foot");
    if(constraint == m_constraints.end())
    {
        yError() << "[setGeneralizedBiasForces] unable to find the left foot constraint. "
                 << "Please call 'initialize()' method";
        return false;
    }
    ptr = std::static_pointer_cast<GenericCartesianConstraint>(constraint->second);
    ptr->setGeneralizedBiasForces(generalizedBiasForces);

    constraint = m_constraints.find("right_foot");
    if(constraint == m_constraints.end())
    {
        yError() << "[setGeneralizedBiasForces] unable to find the right foot constraint. "
                 << "Please call 'initialize()' method";
        return false;
    }
    ptr = std::static_pointer_cast<GenericCartesianConstraint>(constraint->second);
    ptr->setGeneralizedBiasForces(generalizedBiasForces);

    constraint = m_constraints.find("com");
    if(constraint == m_constraints.end())
    {
        yError() << "[setGeneralizedBiasForces] unable to find the com constraint. "
                 << "Please call 'initialize()' method";
        return false;
    }
    ptr = std::static_pointer_cast<GenericCartesianConstraint>(constraint->second);
    ptr->setGeneralizedBiasForces(generalizedBiasForces);

    return true;
}

bool WalkingTaskBasedTorqueController::setDesiredJointTrajectory(const iDynTree::VectorDynSize& desiredJointPosition,
                                                                 const iDynTree::VectorDynSize& desiredJointVelocity,
                                                                 const iDynTree::VectorDynSize& desiredJointAcceleration)
{
    if(desiredJointPosition.size() != m_actuatedDOFs)
    {
        yError() << "[setDesiredJointTrajectory] The size of the jointPosition vector is not coherent"
                 << " with the number of the actuated Joint";
        return false;
    }

    if(desiredJointVelocity.size() != m_actuatedDOFs)
    {
        yError() << "[setDesiredJointTrajectory] The size of the jointVelocity vector is not coherent"
                 << " with the number of the actuated Joint";
        return false;
    }

    if(desiredJointAcceleration.size() != m_actuatedDOFs)
    {
        yError() << "[setDesiredJointTrajectory] The size of the jointVelocity vector is not coherent"
                 << " with the number of the actuated Joint";
        return false;
    }

    m_desiredJointPosition = desiredJointPosition;
    m_desiredJointVelocity = desiredJointVelocity;
    m_desiredJointAcceleration = desiredJointAcceleration;

    return true;
}

bool WalkingTaskBasedTorqueController::setInternalRobotState(const iDynTree::VectorDynSize& jointPosition,
                                                             const iDynTree::VectorDynSize& jointVelocity)
{
    if(jointPosition.size() != m_actuatedDOFs)
    {
        yError() << "[setInternalRobotState] The size of the jointPosition vector is not coherent "
                 << "with the number of the actuated Joint";
        return false;
    }

    if(jointVelocity.size() != m_actuatedDOFs)
    {
        yError() << "[setInternalRobotState] The size of the jointVelocity vector is not coherent "
                 << "with the number of the actuated Joint";
        return false;
    }

    m_jointPosition = jointPosition;
    m_jointVelocity = jointVelocity;

    return true;
}

void WalkingTaskBasedTorqueController::setDesiredNeckTrajectory(const iDynTree::Rotation& desiredNeckOrientation,
                                                                const iDynTree::Vector3& desiredNeckVelocity,
                                                                const iDynTree::Vector3& desiredNeckAcceleration)
{
    // todo are you sure about these equations?
    m_neckOrientationController->setDesiredTrajectory(desiredNeckAcceleration,
                                                      desiredNeckVelocity,
                                                      desiredNeckOrientation * m_additionalRotation);
}

void WalkingTaskBasedTorqueController::setNeckState(const iDynTree::Rotation& neckOrientation,
                                                    const iDynTree::Twist& neckVelocity)
{
    iDynTree::Vector3 neckAngularVelocity;
    iDynTree::toEigen(neckAngularVelocity) = iDynTree::toEigen(neckVelocity.asVector()).block(3,0,3,1);
    m_neckOrientationController->setFeedback(neckAngularVelocity, neckOrientation);
}

bool WalkingTaskBasedTorqueController::setNeckJacobian(const iDynTree::MatrixDynSize& neckJacobian)
{
    if(neckJacobian.rows() != 6)
    {
        yError() << "[setNeckMJacobian] the number of rows has to be equal to 6.";
        return false;
    }
    if(neckJacobian.cols() != m_actuatedDOFs + 6)
    {
        yError() << "[setNeckMJacobian] the number of rows has to be equal to" << m_actuatedDOFs + 6;
        return false;
    }

    iDynTree::toEigen(m_neckJacobian) = iDynTree::toEigen(neckJacobian).block(3, 0, 3,
                                                                              m_actuatedDOFs + 6);

    return true;
}

void WalkingTaskBasedTorqueController::setNeckBiasAcceleration(const iDynTree::Vector6 &neckBiasAcceleration)
{
    // get only the angular part
    iDynTree::toEigen(m_neckBiasAcceleration)
        = iDynTree::toEigen(neckBiasAcceleration).block(3, 0, 3, 1);
}

bool WalkingTaskBasedTorqueController::setDesiredFeetTrajectory(const iDynTree::Transform& leftFootToWorldTransform,
                                                                const iDynTree::Transform& rightFootToWorldTransform,
                                                                const iDynTree::Twist& leftFootTwist,
                                                                const iDynTree::Twist& rightFootTwist,
                                                                const iDynTree::Twist& leftFootAcceleration,
                                                                const iDynTree::Twist& rightFootAcceleration)
{
    std::shared_ptr<CartesianConstraint> ptr;

    // save left foot trajectory
    auto constraint = m_constraints.find("left_foot");
    if(constraint == m_constraints.end())
    {
        yError() << "[setDesiredFeetTrajectory] unable to find the left foot constraint. "
                 << "Please call 'initialize()' method";
        return false;
    }
    ptr = std::static_pointer_cast<CartesianConstraint>(constraint->second);
    ptr->positionController()->setDesiredTrajectory(leftFootAcceleration.getLinearVec3(),
                                                    leftFootTwist.getLinearVec3(),
                                                    leftFootToWorldTransform.getPosition());

    ptr->orientationController()->setDesiredTrajectory(leftFootAcceleration.getAngularVec3(),
                                                       leftFootTwist.getAngularVec3(),
                                                       leftFootToWorldTransform.getRotation());

    // right foot
    constraint = m_constraints.find("right_foot");
    if(constraint == m_constraints.end())
    {
        yError() << "[setDesiredFeetTrajectory] unable to find the right foot constraint. "
                 << "Please call 'initialize()' method";
        return false;
    }
    ptr = std::static_pointer_cast<CartesianConstraint>(constraint->second);
    ptr->positionController()->setDesiredTrajectory(rightFootAcceleration.getLinearVec3(),
                                                    rightFootTwist.getLinearVec3(),
                                                    rightFootToWorldTransform.getPosition());

    ptr->orientationController()->setDesiredTrajectory(rightFootAcceleration.getAngularVec3(),
                                                       rightFootTwist.getAngularVec3(),
                                                       rightFootToWorldTransform.getRotation());
    return true;
}


bool WalkingTaskBasedTorqueController::setFeetState(const iDynTree::Transform& leftFootToWorldTransform,
                                                    const iDynTree::Twist& leftFootTwist,
                                                    const iDynTree::Transform& rightFootToWorldTransform,
                                                    const iDynTree::Twist& rightFootTwist)
{
    std::shared_ptr<CartesianConstraint> ptr;

    // left foot
    auto constraint = m_constraints.find("left_foot");
    if(constraint == m_constraints.end())
    {
        yError() << "[setFeetState] unable to find the left foot constraint. "
                 << "Please call 'initialize()' method";
        return false;
    }
    ptr = std::static_pointer_cast<CartesianConstraint>(constraint->second);
    ptr->positionController()->setFeedback(leftFootTwist.getLinearVec3(),
                                           leftFootToWorldTransform.getPosition());

    ptr->orientationController()->setFeedback(leftFootTwist.getAngularVec3(),
                                              leftFootToWorldTransform.getRotation());

    // right foot
    constraint = m_constraints.find("right_foot");
    if(constraint == m_constraints.end())
    {
        yError() << "[setFeetState] unable to find the right foot constraint. "
                 << "Please call 'initialize()' method";
        return false;
    }
    ptr = std::static_pointer_cast<CartesianConstraint>(constraint->second);
    ptr->positionController()->setFeedback(rightFootTwist.getLinearVec3(),
                                           rightFootToWorldTransform.getPosition());

    ptr->orientationController()->setFeedback(rightFootTwist.getAngularVec3(),
                                              rightFootToWorldTransform.getRotation());

    // Forces
    std::shared_ptr<ForceConstraint> ptrForce;

    // left force
    constraint = m_constraints.find("left_force");
    if(constraint == m_constraints.end())
    {
        yError() << "[setFeetState] unable to find the left force constraint. "
                 << "Please call 'initialize()' method";
        return false;
    }
    ptrForce = std::static_pointer_cast<ForceConstraint>(constraint->second);
    ptrForce->setFootToWorldTransform(leftFootToWorldTransform);

    // right force
    constraint = m_constraints.find("right_force");
    if(constraint == m_constraints.end())
    {
        yError() << "[setFeetState] unable to find the right force constraint. "
                 << "Please call 'initialize()' method";
        return false;
    }
    ptrForce = std::static_pointer_cast<ForceConstraint>(constraint->second);
    ptrForce->setFootToWorldTransform(rightFootToWorldTransform);

    // zmp
    std::shared_ptr<ZMPConstraint> ptrZMP;
    constraint = m_constraints.find("zmp");
    if(constraint == m_constraints.end())
    {
        yError() << "[setFeetState] Unable to find the zmp constraint. "
                 << "Please call 'initialize()' method";
        return false;
    }
    ptrZMP = std::static_pointer_cast<ZMPConstraint>(constraint->second);
    ptrZMP->setLeftFootToWorldTransform(leftFootToWorldTransform);
    ptrZMP->setRightFootToWorldTransform(rightFootToWorldTransform);

    return true;

}

bool WalkingTaskBasedTorqueController::setFeetJacobian(const iDynTree::MatrixDynSize& leftFootJacobian,
                                                       const iDynTree::MatrixDynSize& rightFootJacobian)
{
    // set the feet jacobian
    if(leftFootJacobian.rows() != 6)
    {
        yError() << "[setFeetJacobian] the number of rows has to be equal to 6.";
        return false;
    }
    if(leftFootJacobian.cols() != m_actuatedDOFs + 6)
    {
        yError() << "[setFeetJacobian] the number of columns has to be equal to"
                 << m_actuatedDOFs + 6;
        return false;
    }

    if(rightFootJacobian.rows() != 6)
    {
        yError() << "[setFeetJacobian] the number of rows has to be equal to 6.";
        return false;
    }
    if(rightFootJacobian.cols() != m_actuatedDOFs + 6)
    {
        yError() << "[setFeetJacobian] the number of columns has to be equal to"
                 << m_actuatedDOFs + 6;
        return false;
    }

    // todo (remove?!)
    // set left and right feet jacobians
    std::shared_ptr<GenericCartesianConstraint> ptr;

    auto constraint = m_constraints.find("left_foot");
    if(constraint == m_constraints.end())
    {
        yError() << "[setFeetJacobian] unable to find the left foot constraint. "
                 << "Please call 'initialize()' method";
        return false;
    }
    ptr = std::static_pointer_cast<GenericCartesianConstraint>(constraint->second);
    ptr->setRoboticJacobian(leftFootJacobian);

    constraint = m_constraints.find("right_foot");
    if(constraint == m_constraints.end())
    {
        yError() << "[setFeetJacobian] unable to find the right foot constraint. "
                 << "Please call 'initialize()' method";
        return false;
    }
    ptr = std::static_pointer_cast<GenericCartesianConstraint>(constraint->second);
    ptr->setRoboticJacobian(rightFootJacobian);

    iDynTree::toEigen(m_inputMatrix).block(0, m_actuatedDOFs, m_actuatedDOFs + 6, 6)
        = iDynTree::toEigen(leftFootJacobian).transpose();

    iDynTree::toEigen(m_inputMatrix).block(0, m_actuatedDOFs + 6, m_actuatedDOFs + 6, 6)
        = iDynTree::toEigen(rightFootJacobian).transpose();

    // // set input matrix for all the Cartesian hard constraints (feet, com)
    constraint = m_constraints.find("left_foot");
    ptr = std::static_pointer_cast<GenericCartesianConstraint>(constraint->second);
    ptr->setInputMatrix(m_inputMatrix);

    constraint = m_constraints.find("right_foot");
    ptr = std::static_pointer_cast<GenericCartesianConstraint>(constraint->second);
    ptr->setInputMatrix(m_inputMatrix);

    constraint = m_constraints.find("com");
    ptr = std::static_pointer_cast<GenericCartesianConstraint>(constraint->second);
    ptr->setInputMatrix(m_inputMatrix);

    // set input matrix for all the Cartesian soft constraints (neck orientation)
    // already done before '... .block() = ...'

    return true;
}

bool WalkingTaskBasedTorqueController::setFeetBiasAcceleration(const iDynTree::Vector6 &leftFootBiasAcceleration,
                                                               const iDynTree::Vector6 &rightFootBiasAcceleration)
{
    // todo (remove?!)
    // set left and right feet bias acceleration
    std::shared_ptr<GenericCartesianConstraint> ptr;

    auto constraint = m_constraints.find("left_foot");
    if(constraint == m_constraints.end())
    {
        yError() << "[setFeetJacobian] unable to find the left foot constraint. "
                 << "Please call 'initialize()' method";
        return false;
    }
    ptr = std::static_pointer_cast<GenericCartesianConstraint>(constraint->second);

    iDynTree::VectorDynSize leftFootAccelerationDynVector(leftFootBiasAcceleration.data(), 6);
    ptr->setBiasAcceleration(leftFootAccelerationDynVector);

    constraint = m_constraints.find("right_foot");
    if(constraint == m_constraints.end())
    {
        yError() << "[setFeetJacobian] unable to find the right foot constraint. "
                 << "Please call 'initialize()' method";
        return false;
    }
    ptr = std::static_pointer_cast<GenericCartesianConstraint>(constraint->second);

    iDynTree::VectorDynSize rightFootAccelerationDynVector(rightFootBiasAcceleration.data(), 6);
    ptr->setBiasAcceleration(rightFootAccelerationDynVector);

    return true;
}


bool WalkingTaskBasedTorqueController::setDesiredCoMTrajectory(const iDynTree::Position& comPosition,
                                                               const iDynTree::Vector3& comVelocity,
                                                               const iDynTree::Vector3& comAcceleration)
{
    std::shared_ptr<CartesianConstraint> ptr;

    // save com desired trajectory
    auto constraint = m_constraints.find("com");
    if(constraint == m_constraints.end())
    {
        yError() << "[setDesiredCoMTrajectory] unable to find the com contraint. "
                 << "Please call 'initialize()' method";
        return false;
    }
    ptr = std::static_pointer_cast<CartesianConstraint>(constraint->second);
    ptr->positionController()->setDesiredTrajectory(comAcceleration, comVelocity, comPosition);

    return true;
}

bool WalkingTaskBasedTorqueController::setCoMState(const iDynTree::Position& comPosition,
                                                   const iDynTree::Vector3& comVelocity)
{
    auto constraint = m_constraints.find("com");
    if(constraint == m_constraints.end())
    {
        yError() << "[setCoMState] unable to find the right foot constraint. "
                 << "Please call 'initialize()' method";
        return false;
    }
    auto ptr = std::static_pointer_cast<CartesianConstraint>(constraint->second);
    ptr->positionController()->setFeedback(comVelocity, comPosition);

    return true;
}

bool WalkingTaskBasedTorqueController::setCoMJacobian(const iDynTree::MatrixDynSize& comJacobian)
{
    // set the feet jacobian
    if(comJacobian.rows() != 3)
    {
        yError() << "[setCoMJacobian] the number of rows has to be equal to 3.";
        return false;
    }
    if(comJacobian.cols() != m_actuatedDOFs + 6)
    {
        yError() << "[setCoMJacobian] the number of columns has to be equal to"
                 << m_actuatedDOFs + 6;
        return false;
    }

    auto constraint = m_constraints.find("com");
    if(constraint == m_constraints.end())
    {
        yError() << "[setCoMJacobian] unable to find the left foot constraint. "
                 << "Please call 'initialize()' method";
        return false;
    }
    auto ptr = std::static_pointer_cast<GenericCartesianConstraint>(constraint->second);
    ptr->setRoboticJacobian(comJacobian);

    return true;
}

bool WalkingTaskBasedTorqueController::setCoMBiasAcceleration(const iDynTree::Vector3 &comBiasAcceleration)
{
    std::shared_ptr<GenericCartesianConstraint> ptr;

    auto constraint = m_constraints.find("com");
    if(constraint == m_constraints.end())
    {
        yError() << "[setCoMBiasAcceleration] Unable to find the left foot constraint. "
                 << "Please call 'initialize()' method";
        return false;
    }
    ptr = std::static_pointer_cast<GenericCartesianConstraint>(constraint->second);

    iDynTree::VectorDynSize comBiasAccelerationDynVector(comBiasAcceleration.data(), 3);
    ptr->setBiasAcceleration(comBiasAccelerationDynVector);

    return true;
}

bool WalkingTaskBasedTorqueController::setDesiredZMP(const iDynTree::Vector2 &zmp)
{
    std::shared_ptr<ZMPConstraint> ptr;

    auto constraint = m_constraints.find("zmp");
    if(constraint == m_constraints.end())
    {
        yError() << "[setCoMBiasAcceleration] Unable to find the zmp constraint. "
                 << "Please call 'initialize()' method";
        return false;
    }
    ptr = std::static_pointer_cast<ZMPConstraint>(constraint->second);
    ptr->setDesiredZMP(zmp);

    return true;
}

bool WalkingTaskBasedTorqueController::setFeetState(const bool &leftInContact,
                                                    const bool &rightInContact)
{
    std::shared_ptr<ForceConstraint> ptr;

    auto constraint = m_constraints.find("left_force");
    if(constraint == m_constraints.end())
    {
        yError() << "[setFeetState] Unable to find the zmp constraint. "
                 << "Please call 'initialize()' method";
        return false;
    }
    ptr = std::static_pointer_cast<ForceConstraint>(constraint->second);
    if(leftInContact)
        ptr->activate();
    else
        ptr->deactivate();

    constraint = m_constraints.find("right_force");
    if(constraint == m_constraints.end())
    {
        yError() << "[setFeetState] Unable to find the zmp constraint. "
                 << "Please call 'initialize()' method";
        return false;
    }
    ptr = std::static_pointer_cast<ForceConstraint>(constraint->second);
    if(rightInContact)
        ptr->activate();
    else
        ptr->deactivate();

    return true;
}

bool WalkingTaskBasedTorqueController::setHessianMatrix()
{
    // evaluate the hessian matrix
    MatrixXd massMatrixInverse = iDynTree::toEigen(m_massMatrix).inverse();
    MatrixXd regularizationTaskSubMatrix;
    regularizationTaskSubMatrix = iDynTree::toEigen(m_selectionMatrix)
        * massMatrixInverse * iDynTree::toEigen(m_inputMatrix);

    MatrixXd neckOrientationSubMatrix;
    neckOrientationSubMatrix = iDynTree::toEigen(m_neckJacobian)
        * massMatrixInverse
        * iDynTree::toEigen(m_inputMatrix);

    //todo add neck
    MatrixXd jointRegularizationWeight = iDynTree::toEigen(m_jointRegularizationWeights).asDiagonal();
    Eigen::Map<MatrixXd>(m_hessian.data(), m_numberOfVariables, m_numberOfVariables) =
        regularizationTaskSubMatrix.transpose()
        * iDynTree::toEigen(m_jointRegularizationWeights).asDiagonal()
        * regularizationTaskSubMatrix
        + neckOrientationSubMatrix.transpose()
        * iDynTree::toEigen(m_neckOrientationWeight).asDiagonal()
        * neckOrientationSubMatrix;
    return true;
}

bool WalkingTaskBasedTorqueController::setGradientVector()
{
    // evaluate the gradient vector
    MatrixXd massMatrixInverse = iDynTree::toEigen(m_massMatrix).inverse();
    MatrixXd regularizationTaskSubMatrix;
    regularizationTaskSubMatrix = iDynTree::toEigen(m_selectionMatrix)
        * massMatrixInverse * iDynTree::toEigen(m_inputMatrix);

    iDynTree::toEigen(m_desiredJointAccelerationController)
        = iDynTree::toEigen(m_desiredJointAcceleration)
        + iDynTree::toEigen(m_jointRegularizationDerivativeGains).asDiagonal() *
        (iDynTree::toEigen(m_desiredJointVelocity) - iDynTree::toEigen(m_jointVelocity))
        + iDynTree::toEigen(m_jointRegularizationProportionalGains).asDiagonal() *
        (iDynTree::toEigen(m_desiredJointPosition) - iDynTree::toEigen(m_jointPosition));

    // neck
    MatrixXd neckOrientationSubMatrix;
    neckOrientationSubMatrix = iDynTree::toEigen(m_neckJacobian)
        * massMatrixInverse
        * iDynTree::toEigen(m_inputMatrix);

    m_neckOrientationController->evaluateControl();

    iDynTree::Vector3 tmp = m_neckOrientationController->getControl();

    // evaluate gradient
    // todo important
    Eigen::Map<MatrixXd>(m_gradient.data(), m_numberOfVariables, 1) =
        -regularizationTaskSubMatrix.transpose()
        * iDynTree::toEigen(m_jointRegularizationWeights).asDiagonal()
        * (iDynTree::toEigen(m_desiredJointAccelerationController)
           + iDynTree::toEigen(m_selectionMatrix) * massMatrixInverse
           * iDynTree::toEigen(m_generalizedBiasForces));
    - neckOrientationSubMatrix.transpose()
        * iDynTree::toEigen(m_neckOrientationWeight).asDiagonal()
        * (iDynTree::toEigen(tmp)
           + iDynTree::toEigen(m_neckJacobian) * massMatrixInverse
           * iDynTree::toEigen(m_generalizedBiasForces)
           - iDynTree::toEigen(m_neckBiasAcceleration));
    return true;
}


bool WalkingTaskBasedTorqueController::setLinearConstraintMatrix()
{
    std::shared_ptr<MatrixBlock<iDynTree::MatrixDynSize>> ptr;
    for(const auto& constraint: m_constraints)
    {
        constraint.second->evaluateJacobian();
        ptr = constraint.second->getJacobian();
        Eigen::Map<MatrixXd>(m_constraintMatrix.data(),
                             m_numberOfConstraints,
                             m_numberOfVariables).block(ptr->startingRow, ptr->startingColumn,
                                                        ptr->matrix.rows(), ptr->matrix.cols())
            = iDynTree::toEigen(ptr->matrix);
    }

    return true;
}

bool WalkingTaskBasedTorqueController::setBounds()
{
    for(const auto& constraint: m_constraints)
    {
        if(constraint.first == "com")
        {
            auto ptr = std::static_pointer_cast<PositionConstraint>(constraint.second);
            ptr->evaluateBounds();
        }
        else if(constraint.first == "left_foot"
                || constraint.first == "right_foot")
        {
            auto ptr = std::static_pointer_cast<CartesianConstraint>(constraint.second);
            ptr->evaluateBounds();
        }
        else
            // evaluate bounds
            constraint.second->evaluateBounds();

        int startingRow = constraint.second->getJacobian()->startingRow;
        int constraintSize = constraint.second->getLowerBound().size();

        // set lower bound
        Eigen::Map<MatrixXd>(m_lowerBound.data(), m_numberOfConstraints, 1).block(startingRow, 0,
                                                                                  constraintSize, 1)
            = iDynTree::toEigen(constraint.second->getLowerBound());

        // set upper bound
        Eigen::Map<MatrixXd>(m_upperBound.data(), m_numberOfConstraints, 1).block(startingRow, 0,
                                                                                  constraintSize, 1)
            = iDynTree::toEigen(constraint.second->getUpperBound());
    }

    return true;
}

bool WalkingTaskBasedTorqueController::solve()
{
    if(!setHessianMatrix())
    {
        yError() << "[solve] Unable to set the hessian matrix.";
        return false;
    }

    if(!setGradientVector())
    {
        yError() << "[solve] Unable to set the gradient vector matrix.";
        return false;
    }

    if(!setLinearConstraintMatrix())
    {
        yError() << "[solve] Unable to set the linear constraint matrix.";
        return false;
    }

    if(!setBounds())
    {
        yError() << "[solve] Unable to set the bounds.";
        return false;
    }

    int nWSR = 10000;
    if(!m_isFirstTime)
    {
        if(m_optimizer->hotstart(m_hessian.data(), m_gradient.data(), m_constraintMatrix.data(),
                                 m_minTorqueLimit.data(), m_maxTorqueLimit.data(),
                                 m_upperBound.data(), m_lowerBound.data(), nWSR, 0) != qpOASES::SUCCESSFUL_RETURN)
        {
            yError() << "[solve] Unable to solve the problem.";
            return false;
        }
    }
    else
    {
        if(m_optimizer->init(m_hessian.data(), m_gradient.data(), m_constraintMatrix.data(),
                             0, 0,
                             m_lowerBound.data(), m_upperBound.data(), nWSR, 0) != qpOASES::SUCCESSFUL_RETURN)
        {
            yError() << "[solve] Unable to solve the problem.";
            return false;
        }
        m_isFirstTime = false;
    }

    m_isSolutionEvaluated = true;
    return true;
}

bool WalkingTaskBasedTorqueController::getSolution(iDynTree::VectorDynSize& output)
{
    if(!m_isSolutionEvaluated)
    {
        yError() << "[getSolution] The solution is not evaluated. Please call 'solve()' method.";
        return false;
    }

    if(output.size() != m_actuatedDOFs)
        output.resize(m_actuatedDOFs);

    std::vector<double> result(m_numberOfVariables);

    m_optimizer->getPrimalSolution(result.data());

    for(int i = 0; i < output.size(); i++)
        output(i) = result[i];

    m_isSolutionEvaluated = false;
    return true;
}
