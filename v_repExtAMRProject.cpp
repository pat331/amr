// Copyright 2006-2013 Dr. Marc Andreas Freese. All rights reserved.
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
//
// -------------------------------------------------------------------
// This file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
// You are free to use/modify/distribute this file for whatever purpose!
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.0.3 on April 29th 2013

#include <unistd.h>
#include "v_repExtAMRProject.h"
#include "v_repLib.h"
#include <iostream>
#include <vector>
#include <map>
#include <ctime>
#include <fenv.h>
#include <time.h>
#include <fstream>
#include <chrono>
#include <pthread.h>
#include <thread>
#include "Eigen/Core"
#include "Eigen/Dense"

#ifdef _WIN32
    #include <shlwapi.h>
    #pragma comment(lib, "Shlwapi.lib")
#endif /* _WIN32 */

#if defined (__linux) || defined (__APPLE__)
    #include <string.h>
    #include <sys/time.h>
    #define _stricmp(x,y) strcasecmp(x,y)
#endif

#define PLUGIN_VERSION 1
#define EIGEN_INITIALIZE_MATRICES_BY_ZERO
#define PI 3.1415926

LIBRARY vrepLib; // the V-REP library that we will dynamically load and bind

simInt hRobot, hAckerCar, hsteeringLeft, hsteeringRight, hmotorLeft, hmotorRight; // robot handle
simInt hDummy1,hDummy2,hDummy3,hDummy4,hDummyMid1,hDummyMid2,hDummyCentre1,hDummyCentre2;

simFloat pDummyCentre1[3], pDummyCentre2[3];

float xIni1, xIni2, xIni3, xIni4;
float yIni1, yIni2, yIni3, yIni4;
float zIni1, zIni2; // z-component of the robot position (it will be constant during the simulation)
float xFin2, xFin3, xFin4, xFin5;
float yFin2, yFin3, yFin4, yFin5;


float dt; // control timestep (can be modified directly in the V-REP scene)
float trajDur1, trajDur2, trajDur3, trajDur4; // duration of the assigned trajectory

// ray circonference
float r;
float l = 2.5772/2;
float v_limit = 5.0;
float phi = PI/2;


void Initialize(){
	std::cout << "Initializing..." << std::endl;

// Cuboid Handle
	hRobot = simGetObjectHandle("Cuboid");
  hDummy1 = simGetObjectHandle("Dummy1");
  hDummy2 = simGetObjectHandle("Dummy2");
  hDummy3 = simGetObjectHandle("Dummy3");
  hDummy4 = simGetObjectHandle("Dummy4");
  hDummyMid1 = simGetObjectHandle("DummyMid1");
  hDummyMid2 = simGetObjectHandle("DummyMid2");
  hDummyCentre1 = simGetObjectHandle("DummyCentre1");
  hDummyCentre2 = simGetObjectHandle("DummyCentre2");

// Ackeramann Car Robot Handle

  // hAckerCar = simGetObjectHandle("AckermannCar");
  hAckerCar = simGetObjectHandle("Car");
  hsteeringLeft=simGetObjectHandle("nakedCar_steeringLeft");
  hsteeringRight=simGetObjectHandle("nakedCar_steeringRight");
  hmotorLeft=simGetObjectHandle("nakedCar_motorLeft");
  hmotorRight=simGetObjectHandle("nakedCar_motorRight");


	simFloat pRobot[3];
	simGetObjectPosition(hRobot, -1, pRobot);

  // simFloat pAckerCar[3];
  // simGetObjectPosition(hAckerCar, -1, pAckerCar);
	// xIni = pRobot[0];
	// yIni = pRobot[1];
	// zIni = pRobot[2];

  simFloat pDummy1[3],pDummy2[3],pDummy3[3],pDummy4[3],pDummyMid1[3],pDummyMid2[3];
  simGetObjectPosition(hDummy1,-1,pDummy1);
  simGetObjectPosition(hDummy2,-1,pDummy2);
  simGetObjectPosition(hDummy3,-1,pDummy3);
  simGetObjectPosition(hDummy4,-1,pDummy4);
  simGetObjectPosition(hDummyCentre1,-1,pDummyCentre1);
  simGetObjectPosition(hDummyCentre2,-1,pDummyCentre2);

  // ray circonference
  r = sqrtf(pow(pDummy2[0] - pDummy3[0],2) + pow(pDummy2[1] - pDummy3[1],2)) * 1/2;

  // First segment of the trail
  //Init point
  xIni1 = pDummy1[0];
  yIni1 = pDummy1[1];
  zIni1 = pDummy1[2];

  //End point
  xFin2 = pDummy2[0];
  yFin2 = pDummy2[1];


  // First circular part of the trail
  // Init point
  xIni2 = pDummy2[0];
  yIni2 = pDummy2[1];

  //End point
  xFin3 = pDummy3[0];
  yFin3 = pDummy3[1];

  // Second segment of the trail
  //Init point
  xIni3 = pDummy3[0];
  yIni3 = pDummy3[1];


  //End point
  xFin4 = pDummy4[0];
  yFin4 = pDummy4[1];

  // Second circular part of the trail
  // Init point
  xIni4 = pDummy4[0];
  yIni4 = pDummy4[1];

  //End point
  xFin5 = pDummy1[0];
  yFin5 = pDummy1[1];





	dt = (float)simGetSimulationTimeStep();

	trajDur1 = 10.0;
  trajDur2 = 10.0;
  trajDur3 = 10.0;
  trajDur4 = 10.0;
	// xFin = 1.0;
	// yFin = -2.0;

	std::cout << "Initialization Completed" << std::endl;
}


void Execution(){
	// q = (x, y, theta, phi)
	Eigen::Vector4f q_k, q_kp1; // current and next configuration
	// u = (v, omega)
	// Eigen::Vector2f u; // control inputs

  // PROVA CONTROLLO
  float u, u1, u2, vd;

  Eigen::Vector2f v, v_desired, pd, pr;
  Eigen::Matrix2f Kp;
  Kp(0,0)= 1;
  Kp(0,1)=0.0;
  Kp(1,0)=0.0;
  Kp(1,1)= 1;

	simFloat pRobot[3];
  simFloat pAckerCar[3];
	simFloat eRobot[3];
  simFloat eAckerCar[3];

  float norm_v, omega, norm_v_desired;

	// get the current configuration
	simGetObjectPosition(hAckerCar, -1, pAckerCar);
	simGetObjectOrientation(hAckerCar, -1, eAckerCar);
  std::cout << "position inizio"<< pAckerCar[0] <<"\n" <<pAckerCar[1]<<"\n"<< pAckerCar[2]<<"\n" <<std::endl;

  //KINEMATIC MODEL

  q_k << pAckerCar[0], pAckerCar[1], eAckerCar[2], phi;
  // eAckerCar[0]=-PI/2;
  // eAckerCar[2]=-PI/2;
  float theta = eAckerCar[2];
  std::cout << "theta "<<theta <<"\n" <<std::endl;

  //Construct the trajectory
	float t_sim = (float)simGetSimulationTime();

  // Coordinates of point P
  float b = 0.25;
  // float y1 = pAckerCar[0] + l*cos(theta) + b*cos(theta+phi);
  // float y2 = pAckerCar[1] + l*sin(theta) + b*sin(theta+phi);
  float y1 = pAckerCar[0] + l*cos(theta) + b*cos(theta+phi);
  float y2 = pAckerCar[1] + l*sin(theta) + b*sin(theta+phi);

  std::cout << "p Car[0] "<<pAckerCar[0] <<"\n" <<std::endl;
  std::cout << "p Car[1] "<<pAckerCar[1] <<"\n" <<std::endl;
  std::cout << "y1 " << y1 <<"\n" <<std::endl;
  std::cout << "y2 " << y2 <<"\n" <<std::endl;

	float x, y;
	if(t_sim > 0 && t_sim < trajDur1){

    float t1 = t_sim / trajDur1;

    x = t1 * xFin2 + (1 - t1) * xIni1;
		y = t1 * yFin2 + (1 - t1) * yIni1;

    std::cout << "x traj " << x <<"\n" <<std::endl;
    std::cout << "y traj " << y <<"\n" <<std::endl;

    float square_distance = sqrt(pow(xFin2 - xIni1,2)+pow(yFin2 - yIni1,2));

    v_desired(0) = (xFin2 - xIni1)/square_distance;
    v_desired(1) = (yFin2 - yIni1)/square_distance;
    // std::cout << "t sim < trajDur1" <<"\n" <<std::endl;



	}
  // else if(t_sim > trajDur1 && t_sim < trajDur1 + trajDur2){
  //
  //   // traiettoria circolare
  //   float t2 = (t_sim-trajDur1) / trajDur2;
  //   float theta = PI*t2;
  //
  //   x = pDummyCentre1[0] - r*cos(theta);
  //   y = pDummyCentre1[1] - r*sin(theta);
  //
  //   v_desired(0) = PI*r*sin(PI*t2);
  //   v_desired(1) = -PI*r*cos(PI*t2);
  //
  // }
  // else if(t_sim > trajDur1 +trajDur2 && t_sim < trajDur1 + trajDur2 + trajDur3){
  //
  //   float t3 = (t_sim-(trajDur1 + trajDur2)) / trajDur3;
  //
  //   x = t3 * xFin4 + (1 - t3) * xIni3;
	// 	y = t3 * yFin4 + (1 - t3) * yIni3;
  //
  //   v_desired(0) = xFin4 - xIni3;
  //   v_desired(1) = yFin4 - yIni3;
  //
  //
  // }
  // else if (t_sim > trajDur1 +trajDur2 +trajDur3 && t_sim < trajDur1 + trajDur2 + trajDur3 + trajDur4){
  //
  //   float t4 = (t_sim-(trajDur1 + trajDur2 +trajDur3)) / trajDur4;
  //   float theta = PI*t4;
  //
  //   x = pDummyCentre2[0] + r*cos(theta);
  //   y = pDummyCentre2[1] + r*sin(theta);
  //
  //   v_desired(0) = -PI*r*sin(PI*t4);
  //   v_desired(1) = PI*r*cos(PI*t4);
  //
  // }
  else{

    x = xFin2;
    y = yFin2;

    // x = xFin5;
    // y = yFin5;

    v_desired(0) = 0;
    v_desired(1) = 0;

    std::cout << "IL PUNTO È ARRIVATO"<< "\n" <<std::endl;


  }

  // compute the control inputs (via the controller)

  pd(0) = x; // position desired
  pd(1) = y;
  pr(0) = y1; // robot position
  pr(1) = y2;
  // pr(0) = pAckerCar[0];
  // pr(1) = pAckerCar[1];

  // std::cout << "Posizioni"<< "\n" << std::endl;
  // std::cout << "pd(0)"<< pd(0) <<"\n" << std::endl;
  // std::cout << "pd(1)"<< pd(1) <<"\n" << std::endl;
  // std::cout << "pr(0)"<< pr(0) <<"\n" << std::endl;
  // std::cout << "pr(1)"<< pr(1) <<"\n" << std::endl;

  // omega = (norm_v_desired/v_limit) * sqrt(pow(pr(0)-pd(0),2)+pow(pr(1)-pd(1),2));
  // omega = 0;

  // FRONT WHEEL DRIVE

  //controllo per y
  // 0.5 is only one component of the Kp gain matrix
  // u1 = v_desired(0) + (0.5)*(pd(0) - pr(0));
  // u2 = v_desired(1) + (0.5)*(pd(1) - pr(1));
  //
  // // Update of the control point B
  // // [vControlT wControlT]' = Tinv*[u1 u2]'
  //
  // float vControlT = u1*cos(phi + theta) + u2*sin(phi + theta);
  // float wControlT = -(b*u2*cos(theta) - b*u1*sin(theta) - b*u2*cos(2*phi + theta) + b*u1*sin(2*phi + theta) - 2*l*u2*cos(phi + theta) + 2*l*u1*sin(phi + theta))/(2*b*l);
  //
  // y1 = y1 + u1*dt;
  // y2 = y2 + u2*dt;
  //
  // q_kp1(2) = q_k(2) + vControlT*sin(phi)*(1/l)*dt;
	// q_kp1(3) = q_k(3) + wControlT*dt;
  // // q_kp1(2) = q_k(2);
  // // q_kp1(3) = q_k(3);
  //
  // // y1 = pAckerCar[0] + l*cos(theta) + b*cos(theta+phi);
  // // y2 = pAckerCar[1] + l*sin(theta) + b*sin(theta+phi);
  //
  // q_kp1(0) = y1 - l*cos(q_kp1(2)) - b*cos(q_kp1(2)+q_kp1(3));
  // q_kp1(1) = y2 - l*sin(q_kp1(2)) - b*sin(q_kp1(2)+q_kp1(3));

  // // // // // // // // // // // // //

  // REAR WHEEL DRIVE (prova)

  u1 = v_desired(0) + (.1)*(pd(0) - pr(0));
  u2 = v_desired(1) + (.1)*(pd(1) - pr(1));

  // Update of the control point B
  // [vControlT wControlT]' = Tinv*[u1 u2]'

  float theta_dot = u1*cos(phi + theta)*sin(phi) + u2*sin(phi + theta)*(1/l);
  float phi_dot = -(cos(theta+phi)*sin(phi)*(1/l)+sin(theta+phi)*(1/b))*u1-(sin(theta+phi)*sin(phi)*(1/l)-cos(theta+phi)*(1/b))*u2;

  // Euler Integration
  y1 = y1 + u1*dt; // u1 = y1dot
  y2 = y2 + u2*dt; // u2 = y2dot

  // theta_new = theta + theta_dot*dt
  q_kp1(2) = q_k(2) + theta_dot *dt;
  q_kp1(3) = q_k(3) + phi_dot *dt;
  // q_kp1(2) = q_k(2);
  // q_kp1(3) = q_k(3);

  // y1 = pAckerCar[0] + l*cos(theta) + b*cos(theta+phi);
  // y2 = pAckerCar[1] + l*sin(theta) + b*sin(theta+phi);

  // Formula Inversa per trovare la posizione della macchina
  q_kp1(0) = y1 - l*cos(q_kp1(2)) - b*cos(q_kp1(2)+q_kp1(3));
  q_kp1(1) = y2 - l*sin(q_kp1(2)) - b*sin(q_kp1(2)+q_kp1(3));

  // // // // // // // // // // // // //





  // norm_v = sqrt(pow(v(0),2)+pow(v(1),2));
  // std::cout << "norm_v  "<< "\n" << norm_v   <<"\n" <<std::endl;

  //std::cout << "omega"<< omega << std::endl;
  // std::cout << "norm"<< norm_v << std::endl;

  // //FRONT_WHEEL
	// q_kp1(0) = q_k(0) + norm_v*cos(q_k(2))*cos(q_k(3))*dt;
	// q_kp1(1) = q_k(1) + norm_v*sin(q_k(2))*cos(q_k(3))*dt;
	// q_kp1(2) = q_k(2) + norm_v*sin(q_k(3))*(1/l)*dt;
	// q_kp1(3) = q_k(3) + omega*dt;

  //FRONT_WHEEL
  // prova controllo solo per y
  // q_kp1(0) = q_k(0);
  // q_kp1(1) = q_k(1) + vd*sin(q_k(2))*cos(q_k(3))*dt;
  // q_kp1(2) = q_k(2);
  // q_kp1(3) = q_k(3);



  //REAR_WHEEL
  // q_kp1(0) = q_k(0) + norm_v*cos(q_k(2))*dt;
	// q_kp1(1) = q_k(1) + norm_v*sin(q_k(2))*dt;
	// q_kp1(2) = q_k(2) + norm_v*tan(q_k(3))*(1/l)*dt;
	// q_kp1(3) = q_k(3) + omega*dt;

	// set the robot in the new configuration
  pAckerCar[0] = q_kp1(0);
	pAckerCar[1] = q_kp1(1);
	pAckerCar[2] = zIni1;
	eAckerCar[2] = q_kp1(2);
  phi = q_kp1(3);
  //phi=0.0;



  // std::cout << "sampling time"<< dt <<"\n" << std::endl;
  // std::cout << "x"<< q_kp1(0) << "\n" << std::endl;
  // std::cout << "y"<< q_kp1(1) << "\n" <<std::endl;
  // std::cout << "theta"<< q_kp1(2) << "\n" <<std::endl;
  // std::cout << "phi"<< q_kp1(3) << "\n" <<std::endl;
  // std::cout << "omega"<< omega << "\n" <<std::endl;
  // std::cout << "euler fine"<< eAckerCar[0] <<"\n" <<eAckerCar[1]<<"\n"<< eAckerCar[2]<<"\n" <<std::endl;
  std::cout << "position fine"<< pAckerCar[0] <<"\n" <<pAckerCar[1]<<"\n"<< pAckerCar[2]<<"\n" <<std::endl;



	simSetObjectPosition(hAckerCar, -1, pAckerCar);
	simSetObjectOrientation(hAckerCar, -1, eAckerCar);


  // Prova Ackermann Car
  // simSetJointTargetVelocity(hmotorLeft,norm_v);
  // simSetJointTargetVelocity(hmotorRight,norm_v);

  // simSetJointTargetPosition(hsteeringLeft,phi);
  // simSetJointTargetPosition(hsteeringRight,phi);


}

// This is the plugin start routine (called just once, just after the plugin was loaded):

VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer, int reservedInt) {
    // Dynamically load and bind V-REP functions:
    // ******************************************
    // 1. Figure out this plugin's directory:
    char curDirAndFile[1024];
#ifdef _WIN32
    GetModuleFileName(NULL, curDirAndFile, 1023);
    PathRemoveFileSpec(curDirAndFile);
#elif defined (__linux) || defined (__APPLE__)
    getcwd(curDirAndFile, sizeof (curDirAndFile));
#endif
    std::string currentDirAndPath(curDirAndFile);
    // 2. Append the V-REP library's name:
    std::string temp(currentDirAndPath);
#ifdef _WIN32
    temp += "\\v_rep.dll";
#elif defined (__linux)
    temp += "/libv_rep.so";
#elif defined (__APPLE__)
    temp += "/libv_rep.dylib";
#endif /* __linux || __APPLE__ */
    // 3. Load the V-REP library:
    vrepLib = loadVrepLibrary(temp.c_str());
    if (vrepLib == NULL) {
        std::cout << "Error, could not find or correctly load the V-REP library. Cannot start 'PluginSkeleton' plugin.\n";
        return (0); // Means error, V-REP will unload this plugin
    }
    if (getVrepProcAddresses(vrepLib) == 0) {
        std::cout << "Error, could not find all required functions in the V-REP library. Cannot start 'PluginSkeleton' plugin.\n";
        unloadVrepLibrary(vrepLib);
        return (0); // Means error, V-REP will unload this plugin
    }
    // ******************************************

    // Check the version of V-REP:
    // ******************************************
    int vrepVer;
    simGetIntegerParameter(sim_intparam_program_version, &vrepVer);
    if (vrepVer < 20604) // if V-REP version is smaller than 2.06.04
    {
        std::cout << "Sorry, your V-REP copy is somewhat old. Cannot start 'PluginSkeleton' plugin.\n";
        unloadVrepLibrary(vrepLib);
        return (0); // Means error, V-REP will unload this plugin
    }
    // ******************************************

    simLockInterface(1);

    // Here you could handle various initializations
    // Here you could also register custom Lua functions or custom Lua constants
    // etc.

    return (PLUGIN_VERSION); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}

// This is the plugin end routine (called just once, when V-REP is ending, i.e. releasing this plugin):

VREP_DLLEXPORT void v_repEnd() {
    // Here you could handle various clean-up tasks

    unloadVrepLibrary(vrepLib); // release the library
}

// This is the plugin messaging routine (i.e. V-REP calls this function very often, with various messages):

VREP_DLLEXPORT void* v_repMessage(int message, int* auxiliaryData, void* customData, int* replyData) { // This is called quite often. Just watch out for messages/events you want to handle
    // Keep following 6 lines at the beginning and unchanged:
    simLockInterface(1);
    static bool refreshDlgFlag = true;
    int errorModeSaved;
    simGetIntegerParameter(sim_intparam_error_report_mode, &errorModeSaved);
    simSetIntegerParameter(sim_intparam_error_report_mode, sim_api_errormessage_ignore);
    void* retVal = NULL;

    // Here we can intercept many messages from V-REP (actually callbacks). Only the most important messages are listed here.
    // For a complete list of messages that you can intercept/react with, search for "sim_message_eventcallback"-type constants
    // in the V-REP user manual.

    if (message == sim_message_eventcallback_refreshdialogs)
        refreshDlgFlag = true; // V-REP dialogs were refreshed. Maybe a good idea to refresh this plugin's dialog too

    if (message == sim_message_eventcallback_menuitemselected) { // A custom menu bar entry was selected..
        // here you could make a plugin's main dialog visible/invisible
    }

    if (message == sim_message_eventcallback_instancepass) { // This message is sent each time the scene was rendered (well, shortly after) (very often)
        // It is important to always correctly react to events in V-REP. This message is the most convenient way to do so:

        int flags = auxiliaryData[0];
        bool sceneContentChanged = ((flags & (1 + 2 + 4 + 8 + 16 + 32 + 64 + 256)) != 0); // object erased, created, model or scene loaded, und/redo called, instance switched, or object scaled since last sim_message_eventcallback_instancepass message
        bool instanceSwitched = ((flags & 64) != 0);

        if (instanceSwitched) {
            // React to an instance switch here!!
        }

        if (sceneContentChanged) { // we actualize plugin objects for changes in the scene

            //...

            refreshDlgFlag = true; // always a good idea to trigger a refresh of this plugin's dialog here
        }
    }

    if (message == sim_message_eventcallback_mainscriptabouttobecalled) { // The main script is about to be run (only called while a simulation is running (and not paused!))

    }

    if (message == sim_message_eventcallback_simulationabouttostart) { // Simulation is about to start

		// code written here is run OFF-LINE (before the simulation actually starts)
		Initialize();

    }

    if (message == sim_message_eventcallback_simulationended) { // Simulation just ended

		std::cout << "Simulation Concluded" << std::endl;

    }

    if (message == sim_message_eventcallback_moduleopen) { // A script called simOpenModule (by default the main script). Is only called during simulation.
        if ((customData == NULL) || (_stricmp("PluginSkeleton", (char*) customData) == 0)) // is the command also meant for this plugin?
        {
            // we arrive here only at the beginning of a simulation
        }
    }

    if (message == sim_message_eventcallback_modulehandle) { // A script called simHandleModule (by default the main script). Is only called during simulation.

		// code written here is run ON-LINE (the following function is invoked every dt seconds, then MUST complete in dt seconds)
		Execution();

        if ((customData == NULL) || (_stricmp("PluginSkeleton", (char*) customData) == 0)) // is the command also meant for this plugin?
        {
            // we arrive here only while a simulation is running
        }
    }

    if (message == sim_message_eventcallback_moduleclose) { // A script called simCloseModule (by default the main script). Is only called during simulation.
        if ((customData == NULL) || (_stricmp("PluginSkeleton", (char*) customData) == 0)) // is the command also meant for this plugin?
        {
            // we arrive here only at the end of a simulation
        }
    }

    if (message == sim_message_eventcallback_instanceswitch) { // Here the user switched the scene. React to this message in a similar way as you would react to a full
        // scene content change. In this plugin example, we react to an instance switch by reacting to the
        // sim_message_eventcallback_instancepass message and checking the bit 6 (64) of the auxiliaryData[0]
        // (see here above)

    }

    if (message == sim_message_eventcallback_broadcast) { // Here we have a plugin that is broadcasting data (the broadcaster will also receive this data!)

    }

    if (message == sim_message_eventcallback_scenesave) { // The scene is about to be saved. If required do some processing here (e.g. add custom scene data to be serialized with the scene)

    }

    // You can add many more messages to handle here

    if ((message == sim_message_eventcallback_guipass) && refreshDlgFlag) { // handle refresh of the plugin's dialogs
        // ...
        refreshDlgFlag = false;
    }

	// Play button is pressed
    if (simGetSimulationState()==17){

    }
	// Stop button is pressed
	else if(simGetSimulationState()==sim_simulation_stopped){

	}


    // Keep following unchanged:
    simSetIntegerParameter(sim_intparam_error_report_mode, errorModeSaved); // restore previous settings
    simLockInterface(0);
    return retVal;
}
