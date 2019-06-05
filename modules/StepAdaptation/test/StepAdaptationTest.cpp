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



int main(int argc, char **argv) {

    std::unique_ptr<WalkingLogger> m_walkingLogger; /**< Pointer to the Walking Logger object. */

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

    m_walkingLogger->startRecord({"record","foot_pos_x","stepTiming","DCM_offset_x","timed1","timed2","nomNextStep","nomStepTiming","nomDCMOffset","nomLastDCM","omega","var1","var2","var3","var4"});

    // m_walkingLogger->startRecord({"record","dist","steplength","steptime","dcmoffset"});
    // initialize the step adaptation
    m_stepAdaptator = std::make_unique<StepAdaptator>();
    yarp::os::Bottle& stepAdaptatorOptions = rf.findGroup("STEP_ADAPTATOR");
    stepAdaptatorOptions.append(generalOptions);
    iDynTree::VectorFixSize<5> nominalValues;
    iDynTree::Vector3 currentValues;

    double c;
    double omega=sqrt(9.81/0.6);
    int i=0;
    double stepTiming=0.5;
    double stepTiming1=0.5;
    double sigma;
    double nextStepPosition=0.5;
    double nominalDCMOffset;
    iDynTree::Vector2 timed;
    timed(0)=0;
    //iDynTree::Vector3 leftAdaptedStepParameters;
    double alpha=0;
    double initDCM=0;
    double initTimining=0;
    double initiDCMOffset=0;
    double initStepPosition1=0;
    double a;
    double b;
    iDynTree::Vector4 tempp;

    //for (int k=1;k<12;k++){
    //    alpha=alpha+0.011;
    alpha=0;
    a=0;
    b=0.5/(exp(omega*(stepTiming1-0.01))-1);;
    i=0;
    double kk=0;
    nextStepPosition=0.5;
    iDynTree::Vector3 leftAdaptedStepParameters;
    double RStepTiming=stepTiming;


    for (int var2=1;var2<=6;var2++) {
        yInfo()<<var2<<var2;
        RStepTiming=stepTiming1;


        for(int var=1;var<=int(((RStepTiming+(stepTiming1-stepTiming))/0.0100)+0.001);var++){
            i++;
            stepTiming=stepTiming-0.01;
            sigma=exp(omega*stepTiming);

            //nomStepTiming=(jLeftstepList.at(1).impactTime-jRightstepList.at(0).impactTime)/(1+switchOverSwingRatio);
            nominalDCMOffset=0.5/(exp(omega*stepTiming1)-1);

            nominalValues(0)=nextStepPosition;
            nominalValues(1)=sigma;
            nominalValues(2)=nominalDCMOffset;
            nominalValues(3)=0;
            nominalValues(4)=omega;


            currentValues(0)=a;
            currentValues(1)=b;
            currentValues(2)=0;





            if (((var+1)==(int(((RStepTiming+(stepTiming1-stepTiming))/0.0100)+0.001)))) {
                kk=kk+1;
                //a=b-0.1;
                a=a+0.50;
                b=a+nominalDCMOffset;
                stepTiming=0.50;

                nextStepPosition=nextStepPosition+0.5;
                yInfo()<<"milad"<<"ddjdjjjdjj"<<kk;
            }

            timed(0)=timed(0)+0.0100;

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
            double mil=leftAdaptedStepParameters(0)+leftAdaptedStepParameters(2)+(currentValues(0)-currentValues(1))*(leftAdaptedStepParameters(1))-currentValues(0);
            RStepTiming=((log(leftAdaptedStepParameters(1)))/omega);

            tempp(0)=(b-a);/*mil;*/
            tempp(1)=(currentValues(0)-currentValues(1));
            tempp(2)=leftAdaptedStepParameters(1);
            tempp(3)=currentValues(0);
            //        if(i==720){
            //            initDCM=b;
            //            initStepPosition1=leftAdaptedStepParameters(0);
            //            initTimining=leftAdaptedStepParameters(1);
            //            initiDCMOffset=leftAdaptedStepParameters(2);
            //        }

            //        if(i==726){
            //        tempp(0)=b-initDCM;
            //        tempp(1)=leftAdaptedStepParameters(0)-initStepPosition1;
            //        tempp(2)=leftAdaptedStepParameters(1)-initTimining;
            //        tempp(3)=leftAdaptedStepParameters(2)-initiDCMOffset;
            //        yInfo()<<leftAdaptedStepParameters(0)<<initStepPosition1;
            //        }
            b =(b-a)*exp(omega*0.01)+a;
            if(i==112){
                b=b+1*0.18;
            }

            m_walkingLogger->sendData(leftAdaptedStepParameters,timed,nominalValues,tempp);


            yInfo()<<RStepTiming<<i<<var<<int(((RStepTiming+(stepTiming1-stepTiming))/0.0100)+0.001);


            yarp::os::Time::delay(0.01);
        }
    }
    //       m_walkingLogger->sendData(tempp);
    //}

    m_walkingLogger->quit();
    return EXIT_SUCCESS;
}
