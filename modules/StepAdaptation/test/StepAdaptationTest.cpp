/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/Utils.h>
#include <Eigen/Core>
#include "iDynTree/Core/EigenHelpers.h"
#include "iDynTree/Core/MatrixDynSize.h"
#include <cmath>
#include <memory>
#include <iostream>
#include <ctime>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>
#include<StepAdaptator.hpp>
#include <yarp/os/RpcClient.h>
#include <WalkingLogger.hpp>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>



//typedef struct {
//    double initTime = 0.0, endTime = 50.0, dT = 0.01, K = 10, dX = 0.2, dY = 0.0;
//    double maxL = 0.2, minL = 0.05, minW = 0.08, maxAngle = iDynTree::deg2rad(45), minAngle = iDynTree::deg2rad(5);
//    double nominalW = 0.14, maxT = 10, minT = 3, nominalT = 4, timeWeight = 2.5, positionWeight = 1;
//    bool swingLeft = true;
//    double slowWhenTurnGain = 0.5;
//} Configuration;

//bool printSteps(std::deque<Step> leftSteps, std::deque<Step> rightSteps){
//    std::cerr << "Left foot "<< leftSteps.size() << " steps:"<< std::endl;
//    for (auto step : leftSteps){
//        std::cerr << "Position "<< step.position.toString() << std::endl;
//        std::cerr << "Angle "<< iDynTree::rad2deg(step.angle) << std::endl;
//        std::cerr << "Time  "<< step.impactTime << std::endl;
//    }


//    std::cerr << std::endl << "Right foot "<< rightSteps.size() << " steps:" << std::endl;
//    for (auto step : rightSteps){
//        std::cerr << "Position "<< step.position.toString() << std::endl;
//        std::cerr << "Angle "<< iDynTree::rad2deg(step.angle) << std::endl;
//        std::cerr << "Time  "<< step.impactTime << std::endl;
//    }

//    return true;
//}

//bool checkConstraints(std::deque<Step> leftSteps, std::deque<Step> rightSteps, Configuration conf){
//Checking constraints

//    conf.swingLeft = (leftSteps.front().impactTime >= rightSteps.front().impactTime);

//    if (leftSteps.front().impactTime == rightSteps.front().impactTime)
//        leftSteps.pop_front(); //this is a fake step!

//    bool result = true;
//    double distance = 0.0, deltaAngle = 0.0, deltaTime = 0.0, c_theta, s_theta;
//    iDynTree::MatrixDynSize rTranspose(2,2);
//    iDynTree::Vector2 rPl;

//    while (!leftSteps.empty() && !rightSteps.empty()){
//        distance = (iDynTree::toEigen(leftSteps.front().position) - iDynTree::toEigen(rightSteps.front().position)).norm();

//        if (distance > conf.maxL){
//            std::cerr <<"[ERROR] Distance constraint not satisfied" << std::endl;
//            result = false;
//        }

//        deltaAngle = std::abs(leftSteps.front().angle - rightSteps.front().angle);

//        if (deltaAngle > conf.maxAngle){
//            std::cerr <<"[ERROR] Angle constraint not satisfied" << std::endl;
//            result = false;
//        }

//        deltaTime = std::abs(leftSteps.front().impactTime - rightSteps.front().impactTime);

//        if (deltaTime < conf.minT){
//            std::cerr <<"[ERROR] Min time constraint not satisfied" << std::endl;
//            result = false;
//        }

//        c_theta = std::cos(rightSteps.front().angle);
//        s_theta = std::sin(rightSteps.front().angle);

//        rTranspose(0,0) = c_theta;
//        rTranspose(1,0) = -s_theta;
//        rTranspose(0,1) = s_theta;
//        rTranspose(1,1) = c_theta;

//        iDynTree::toEigen(rPl) =
//                iDynTree::toEigen(rTranspose)*(iDynTree::toEigen(leftSteps.front().position) - iDynTree::toEigen(rightSteps.front().position));

//        if (rPl(1) < conf.minW){
//            std::cerr <<"[ERROR] Width constraint not satisfied" << std::endl;
//            result = false;
//        }

//        if(conf.swingLeft)
//            rightSteps.pop_front();
//        else leftSteps.pop_front();

//        conf.swingLeft = !conf.swingLeft;

//    }

//    if(!result)
//        return false;

//    return true;
//}

bool populateDesiredTrajectory(/*UnicyclePlanner& planner,*/ double initTime, double endTime, double dT){

    //    double t = initTime;
    //    iDynTree::Vector2 yDes, yDotDes;
    //    while (t <= endTime){
    //        yDes(0) = 0.01*t;
    //        yDotDes(0) = 0.01;
    //        yDes(1) = 0.5*std::sin(0.1*t);
    //        yDotDes(1) = 0.5*0.1*std::cos(0.1*t);
    //        if(!planner.addDesiredTrajectoryPoint(t,yDes, yDotDes))
    //            return false;
    //        t += dT;
    //    }
    return true;
}

bool plannerTest(){

    //    Configuration conf;
    //    conf.initTime = 0.0;
    //    conf.endTime = 50.0;
    //    conf.dT = 0.01;
    //    conf.K = 10;
    //    conf.dX = 0.2;
    //    conf.dY = 0.0;
    //    conf.maxL = 0.2;
    //    conf.minL = 0.05;
    //    conf.minW = 0.08;
    //    conf.maxAngle = iDynTree::deg2rad(45);
    //    conf.minAngle = iDynTree::deg2rad(5);
    //    conf.nominalW = 0.14;
    //    conf.maxT = 10;
    //    conf.minT = 3;
    //    conf.nominalT = 4;
    //    conf.timeWeight = 2.5;
    //    conf.positionWeight = 1;
    //    conf.swingLeft = true;
    //    conf.slowWhenTurnGain = 0.5;

    //    UnicyclePlanner planner;

    //    //Initialization (some of these calls may be avoided)
    //    iDynTree::assertTrue(planner.setDesiredPersonDistance(conf.dX, conf.dY));
    //    iDynTree::assertTrue(planner.setControllerGain(conf.K));
    //    iDynTree::assertTrue(planner.setMaximumIntegratorStepSize(conf.dT));
    //    iDynTree::assertTrue(planner.setMaxStepLength(conf.maxL));
    //    iDynTree::assertTrue(planner.setWidthSetting(conf.minW, conf.nominalW));
    //    iDynTree::assertTrue(planner.setMaxAngleVariation(conf.maxAngle));
    //    iDynTree::assertTrue(planner.setCostWeights(conf.positionWeight, conf.timeWeight));
    //    iDynTree::assertTrue(planner.setStepTimings(conf.minT, conf.maxT, conf.nominalT));
    //    iDynTree::assertTrue(planner.setPlannerPeriod(conf.dT));
    //    iDynTree::assertTrue(planner.setMinimumAngleForNewSteps(conf.minAngle));
    //    iDynTree::assertTrue(planner.setMinimumStepLength(conf.minL));
    //    iDynTree::assertTrue(planner.setSlowWhenTurnGain(conf.slowWhenTurnGain));

    //    planner.addTerminalStep(true);
    //    planner.startWithLeft(conf.swingLeft);

    //    //Generate desired trajectory
    //    clock_t start = clock();
    //    iDynTree::assertTrue(populateDesiredTrajectory(planner, conf.initTime, conf.endTime, conf.dT));
    //    std::cerr <<"Populating the trajectory took " << (static_cast<double>(clock() - start) / CLOCKS_PER_SEC) << " seconds."<<std::endl;

    //    std::shared_ptr<FootPrint> left, right;
    //    left = std::make_shared<FootPrint>();
    //    right = std::make_shared<FootPrint>();
    //    iDynTree::Vector2 initPosition;
    //    initPosition(0) = 0.3;
    //    initPosition(1) = -0.5;
    //    //left->addStep(initPosition, iDynTree::deg2rad(15), 25); //fake initialization

    //    start = clock();
    //    iDynTree::assertTrue(planner.computeNewSteps(left, right, conf.initTime, conf.endTime));
    //    std::cerr <<"Test Finished in " << (static_cast<double>(clock() - start) / CLOCKS_PER_SEC) << " seconds."<<std::endl;

    //    StepList leftSteps = left->getSteps();
    //    StepList rightSteps = right->getSteps();

    //    std::cerr << "First test." << std::endl;
    //    iDynTree::assertTrue(printSteps(leftSteps, rightSteps));
    //    iDynTree::assertTrue(checkConstraints(leftSteps, rightSteps, conf));



    //    std::cerr << std::endl << "------------------------------------------------------------------" << std::endl;
    //    std::cerr << "Second test." << std::endl;

    //    left->clearSteps();
    //    iDynTree::assertTrue(right->dropPastSteps());
    //    iDynTree::assertTrue(right->numberOfSteps() == 1);
    //    Step lastStep;
    //    iDynTree::assertTrue(right->getLastStep(lastStep));
    //    planner.clearDesiredTrajectory();
    //    iDynTree::Vector2 dummyVector, newDesired;
    //    dummyVector.zero();
    //    newDesired(0) = lastStep.position(0) + 0.5;
    //    newDesired(1) = lastStep.position(1) + 0.5;
    //    iDynTree::assertTrue(planner.addDesiredTrajectoryPoint(lastStep.impactTime+10, newDesired, dummyVector));

    //    iDynTree::assertTrue(planner.computeNewSteps(left, right, lastStep.impactTime, lastStep.impactTime+10));

    //    leftSteps = left->getSteps();
    //    rightSteps = right->getSteps();

    //    iDynTree::assertTrue(printSteps(leftSteps, rightSteps));

    //    iDynTree::assertTrue(checkConstraints(leftSteps, rightSteps, conf));

    return true;
}

int main(int argc, char **argv) {

    std::unique_ptr<WalkingLogger> m_walkingLogger; /**< Pointer to the Walking Logger object. */
    //  iDynTree::assertTrue(plannerTest());
    // prepare and configure the resource finder
    // initialise yarp network
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"[main] Unable to find YARP network";
        return EXIT_FAILURE;
    }

    std::unique_ptr<StepAdaptator> m_stepAdaptator;
    yarp::os::ResourceFinder& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    rf.setDefaultConfigFile("dcmWalkingCoordinator.ini");


    yarp::os::Bottle& generalOptions = rf.findGroup("GENERAL");
    double  m_dT = generalOptions.check("sampling_time", yarp::os::Value(0.016)).asDouble();



    rf.configure(argc, argv);

    m_walkingLogger = std::make_unique<WalkingLogger>();
    yarp::os::Bottle& loggerOptions = rf.findGroup("WALKING_LOGGER");
    if(!m_walkingLogger->configure(loggerOptions, "stepAdaptation"))
    {
        yError() << "[configure] Unable to configure the logger.";
        return false;
    }

    m_walkingLogger->startRecord({"record","foot_pos_x","stepTiming","DCM_offset_x","timed1","timed2","nomNextStep","nomStepTiming","nomDCMOffset","nomLastDCM","omega"});
    // initialize the step adaptation
    m_stepAdaptator = std::make_unique<StepAdaptator>();
    yarp::os::Bottle& stepAdaptatorOptions = rf.findGroup("STEP_ADAPTATOR");
    stepAdaptatorOptions.append(generalOptions);
    iDynTree::VectorFixSize<5> nominalValues;
    iDynTree::Vector3 currentValues;
    double a=0;
    double b=0.1;
    double c;
    double omega=sqrt(9.81/0.6);
    int i=0;
    double stepTiming=0.5;
    double sigma;
    double nextStepPosition=0.5;
    double nominalDCMOffset;
    iDynTree::Vector2 timed;
    timed(0)=0;
    iDynTree::Vector3 leftAdaptedStepParameters;


    for(int var=1;var<=1000;var++){
        i++;

        sigma=exp(omega*stepTiming);

        //nomStepTiming=(jLeftstepList.at(1).impactTime-jRightstepList.at(0).impactTime)/(1+switchOverSwingRatio);
        nominalDCMOffset=0.5/(exp(omega*stepTiming)-1);

        nominalValues(0)=nextStepPosition;
        nominalValues(1)=sigma;
        nominalValues(2)=nominalDCMOffset;
        nominalValues(3)=0;
        nominalValues(4)=omega;


        currentValues(0)=a;
        currentValues(1)=b;
        currentValues(2)=0;

        if(i==230){
            b=b+0.05;
        }

        if (((i+1)%100)==0) {
            a=a+0.5;
            b=b+0.5;
            stepTiming=0.5;
            nextStepPosition=nextStepPosition+0.5;
        }


        //b=b+0.005;
        timed(0)=timed(0)+0.01;
       // stepTiming=stepTiming-0.01;





        if(!m_stepAdaptator->initialize(stepAdaptatorOptions))
        {
            yError() << "[configure] Unable to initialize the step adaptator!";
            return false;
        }



        if(!m_stepAdaptator->RunStepAdaptator(nominalValues,currentValues))
        {
            yError() << "[updateModule] Unable to solve the QP problem of step adaptation.";
            return false;
        }


        if(!m_stepAdaptator->solve())
        {
            yError() << "[updateModule] Unable to solve the QP problem of step adaptation.";
            return false;
        }

        if(!m_stepAdaptator->getControllerOutput(leftAdaptedStepParameters))
        {
            yError() << "[updateModule] Unable to get the step adaptation output.";
            return false;
        }

        m_walkingLogger->sendData(leftAdaptedStepParameters,timed,nominalValues);


        yarp::os::Time::delay(0.01);
    }

    m_walkingLogger->quit();
    return EXIT_SUCCESS;
}
