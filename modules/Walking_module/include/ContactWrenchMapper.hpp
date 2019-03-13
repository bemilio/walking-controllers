/**
 * @file WalkingTaskBasedTorqueController.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_CONTROL_MAPPER_HPP
#define WALKING_CONTROL_MAPPER_HPP

#include <iDynTree/Core/SpatialMomentum.h>

#include <ContactWrenchMapping.hpp>

class ContactWrenchMapper
{
    bool m_isDoubleSupportPhase{true};

    bool m_leftInContact{true};
    bool m_rightInContact{true};

    std::unique_ptr<ContactWrenchMappingSingleSupport> m_singleSupportSolver;
    std::unique_ptr<ContactWrenchMappingDoubleSupport> m_doubleSupportSolver;

    int m_actuatedDOFs;

public:

    bool initialize(const yarp::os::Searchable& config, const int& actuatedDOFs);

    void setMass(const double& mass);

    void setFeetState(const bool &leftInContact, const bool &rightInContact);

    bool setCentroidalTotalMomentum(const iDynTree::SpatialMomentum& centroidalTotalMomentum);

    bool setFeetState(const iDynTree::Transform& leftFootToWorldTransform,
                      const iDynTree::Transform& rightFootToWorldTransform);

    bool setFeetJacobian(const iDynTree::MatrixDynSize& leftFootJacobian,
                         const iDynTree::MatrixDynSize& rightFootJacobian);

    bool setCoMState(const iDynTree::Position& comPosition,
                     const iDynTree::Vector3& comVelocity);

    bool setDesiredCoMTrajectory(const iDynTree::Position& comPosition,
                                 const iDynTree::Vector3& comVelocity,
                                 const iDynTree::Vector3& comAcceleration);


    bool setFeetWeightPercentage(const double &weightInLeft, const double &weightInRight);

    bool setDesiredVRP(const iDynTree::Vector3& vrp);

    /**
     * Solve the optimization problem.
     * @return true/false in case of success/failure.
     */
    bool solve();

    /**
     * Get the solution of the optimization problem.
     * @return the desired joint torque
     */
    const iDynTree::VectorDynSize& solution() const;

    bool isDoubleSupportPhase();
};

#endif
