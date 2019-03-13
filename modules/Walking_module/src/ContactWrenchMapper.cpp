/**
 * @file WalkingTaskBasedTorqueController.cpp
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

#include <ContactWrenchMapper.hpp>
#include <Utils.hpp>


bool ContactWrenchMapper::initialize(const yarp::os::Searchable& config,
                                     const int& actutatedDOFs)
{
    // instantiate solvers
    m_singleSupportSolver = std::make_unique<ContactWrenchMappingSingleSupport>();
    m_doubleSupportSolver = std::make_unique<ContactWrenchMappingDoubleSupport>();

    if(!m_singleSupportSolver->initialize(config))
    {
        yError() << "[Initialize] Unable to initialize the single support solver";
        return false;
    }
    if(!m_doubleSupportSolver->initialize(config))
    {
        yError() << "[Initialize] Unable to initialize the double support solver";
        return false;
    }

    m_actuatedDOFs = actutatedDOFs;

    return true;
}

void ContactWrenchMapper::setMass(const double& mass)
{
    if(m_isDoubleSupportPhase)
        m_doubleSupportSolver->setRobotMass(mass);
    else
        m_singleSupportSolver->setRobotMass(mass);
}

void ContactWrenchMapper::setFeetState(const bool &leftInContact, const bool &rightInContact)
{
    m_leftInContact = leftInContact;
    m_rightInContact = rightInContact;

    if(m_leftInContact && m_rightInContact)
    {
        m_isDoubleSupportPhase = true;
        yInfo() << "[setFeetState] Double support phase";
    }
    else
    {
        m_isDoubleSupportPhase = false;
        yInfo() << "[setFeetState] Single support phase";
    }
}

bool ContactWrenchMapper::setCentroidalTotalMomentum(const iDynTree::SpatialMomentum& centroidalTotalMomentum)
{
    if(m_isDoubleSupportPhase)
        return m_doubleSupportSolver->setCentroidalTotalMomentum(centroidalTotalMomentum);
    else
        return m_singleSupportSolver->setCentroidalTotalMomentum(centroidalTotalMomentum);
}

bool ContactWrenchMapper::setFeetState(const iDynTree::Transform& leftFootToWorldTransform,
                                       const iDynTree::Transform& rightFootToWorldTransform)
{
    if(m_isDoubleSupportPhase)
    {
        m_doubleSupportSolver->setFeetState(leftFootToWorldTransform,
                                            rightFootToWorldTransform);
        return true;
    }

    if(m_leftInContact)
    {
        if(!m_singleSupportSolver->setFeetState(leftFootToWorldTransform))
        {
            yInfo() << "[setDesiredFeetTrajectory] Unable to set the foot state (left in contact)";
            return false;
        }
    }
    else
    {
        if(!m_singleSupportSolver->setFeetState(rightFootToWorldTransform))
        {
            yInfo() << "[setDesiredFeetTrajectory] Unable to set the foot state (right in contact)";
            return false;
        }
    }

    return true;
}

bool ContactWrenchMapper::setFeetJacobian(const iDynTree::MatrixDynSize& leftFootJacobian,
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

    if(m_isDoubleSupportPhase)
    {
        m_doubleSupportSolver->setFeetJacobian(leftFootJacobian, rightFootJacobian);
        return true;
    }

    if(m_leftInContact)
        m_singleSupportSolver->setFeetJacobian(leftFootJacobian, rightFootJacobian);
    else
        m_singleSupportSolver->setFeetJacobian(rightFootJacobian, leftFootJacobian);

    return true;
}

bool ContactWrenchMapper::setCoMState(const iDynTree::Position& comPosition,
                                      const iDynTree::Vector3& comVelocity)
{
    if(m_isDoubleSupportPhase)
    {
        if(!m_doubleSupportSolver->setCoMState(comPosition, comVelocity))
        {
            yError() << "[setDesiredCoMTrajectory] Unable to set the CoM state (DS)";
            return false;
        }
    }
    else
    {
        if(!m_singleSupportSolver->setCoMState(comPosition, comVelocity))
        {
            yError() << "[setDesiredCoMTrajectory] Unable to set the CoM state (SS)";
            return false;
        }
    }
    return true;
}

bool ContactWrenchMapper::setDesiredVRP(const iDynTree::Vector3 &vrp)
{
    if(m_isDoubleSupportPhase)
    {
        if(!m_doubleSupportSolver->setDesiredVRP(vrp))
        {
            yError() << "[setDesiredVRP] Unable to set the desired VRP (DS)";
            return false;
        }
    }
    else
    {
        if(!m_singleSupportSolver->setDesiredVRP(vrp))
        {
            yError() << "[setDesiredVRP] Unable to set the desired VRP (SS)";
            return false;
        }
    }
    return true;
}

bool ContactWrenchMapper::setDesiredCoMTrajectory(const iDynTree::Position& comPosition,
                                                  const iDynTree::Vector3& comVelocity,
                                                  const iDynTree::Vector3& comAcceleration)
{
    if(m_isDoubleSupportPhase)
    {
        if(!m_doubleSupportSolver->setDesiredCoMTrajectory(comPosition, comVelocity,comAcceleration))
        {
            yError() << "[setDesiredCoMTrajectory] Unable to set the desired com trajectory (DS)";
            return false;
        }
    }
    else
    {
        if(!m_singleSupportSolver->setDesiredCoMTrajectory(comPosition, comVelocity,comAcceleration))
        {
            yError() << "[setDesiredCoMTrajectory] Unable to set the desired com trajectory (SS)";
            return false;
        }
    }
    return true;
}


bool ContactWrenchMapper::setFeetWeightPercentage(const double &weightInLeft,
                                                  const double &weightInRight)
{
    if(!m_isDoubleSupportPhase)
        return true;

    if(!m_doubleSupportSolver->setFeetWeightPercentage(weightInLeft, weightInRight))
    {
        yError() << "[setFeetWeightPercentage] Unable to set the feet weight percentage";
        return false;
    }
    return true;
}

bool ContactWrenchMapper::solve()
{
    if(m_isDoubleSupportPhase)
    {
        if(!m_doubleSupportSolver->solve())
        {
            yError() << "[solve] Unable to solve the problem (DS)";
            return false;
        }
    }
    else
    {
        if(!m_singleSupportSolver->solve())
        {
            yError() << "[solve] Unable to solve the problem (SS)";
            return false;
        }
    }
    return true;
}

const iDynTree::VectorDynSize& ContactWrenchMapper::solution() const
{
    if(m_isDoubleSupportPhase)
        return m_doubleSupportSolver->solution();
    else
        return m_singleSupportSolver->solution();
}
