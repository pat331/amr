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
#include <random>

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
simInt hDummy1,hDummy2,hDummy3,hDummy4,hDummyMid1,hDummyMid2,hDummyCentre1,hDummyCentre2,hDummyEvasive1,hDummyBack1,hDummyEvasive2,hDummyBack2;
simInt hClosestPointFront,hClosestPointRight ;
simInt hCuboid1,hCuboid2,hCuboid3,hCuboid4;

simFloat pDummyCentre1[3], pDummyCentre2[3];
simFloat pCuboid1[3],pCuboid2[3],pCuboid3[3],pCuboid4[3];

float xIni1, xIni2, xIni3, xIni4;
float yIni1, yIni2, yIni3, yIni4;
float zIni1, zIni2, zCar; // z-component of the robot position (it will be constant during the simulation)
float xFin2, xFin3, xFin4, xFin5;
float yFin2, yFin3, yFin4, yFin5;
float xEv1, yEv1, yBack1, yBack2, xEv2, yEv2;
float inflec1, inflec2;
float x_end_ev, y_end_ev;



float dt; // control timestep (can be modified directly in the V-REP scene)
float trajDur1, trajDur2, trajDur3, trajDur4, trajDurEvasive, trajDur1Back, trajDurEvasive2, trajDur2Back; // duration of the assigned trajectory

// ray circonference
float r;
// float l = 2.1/2;
float l = 0.5;
float v_limit = 5.0;
float phi = 0;
float y_m1, y_m2;
float a = 1.0;
float a2 = -1.0;
float half_cuboid1 = 0.25;
float half_cuboid2 = 0.5;
float half_width_car = 0.25;
float x_s = 0.15; // safety distance


float dist_closest_point_front, dist_closest_point_right, min_dist;

//direcrtion = 0 --->osservo con kinect front
//direction = 1 --->osservo con kinect right
int direction = -1;



void Initialize(){
	std::cout << "Initializing..." << std::endl;

	// Cuboid Handle
	//hRobot = simGetObjectHandle("Cuboid");
	hDummy1 = simGetObjectHandle("Dummy1");
	hDummy2 = simGetObjectHandle("Dummy2");
	hDummy3 = simGetObjectHandle("Dummy3");
	hDummy4 = simGetObjectHandle("Dummy4");
	hDummyMid1 = simGetObjectHandle("DummyMid1");
	hDummyMid2 = simGetObjectHandle("DummyMid2");
	hDummyCentre1 = simGetObjectHandle("DummyCentre1");
	hDummyCentre2 = simGetObjectHandle("DummyCentre2");
	hClosestPointFront = simGetObjectHandle("closestDetectedPointFront");
	hClosestPointRight = simGetObjectHandle("closestDetectedPointRight");
	hCuboid1 = simGetObjectHandle("Cuboid1");
	hCuboid2 = simGetObjectHandle("Cuboid2");
	hCuboid3 = simGetObjectHandle("Cuboid3");
	hCuboid4 = simGetObjectHandle("Cuboid4");
	hDummyEvasive1 = simGetObjectHandle("DummyEvasive1");
	hDummyEvasive2 = simGetObjectHandle("DummyEvasive2");
	hDummyBack1 = simGetObjectHandle("DummyBack1");
	hDummyBack2 = simGetObjectHandle("DummyBack2");


	// Ackeramann Car Robot Handle
	hAckerCar = simGetObjectHandle("Car");


	simFloat pDummy1[3],pDummy2[3],pDummy3[3],pDummy4[3],pDummyMid1[3],pDummyMid2[3],pDummyEvasive1[3],pDummyBack1[3],pDummyEvasive2[3],pDummyBack2[3];

	simGetObjectPosition(hDummy1,-1,pDummy1);
	simGetObjectPosition(hDummy2,-1,pDummy2);
	simGetObjectPosition(hDummy3,-1,pDummy3);
	simGetObjectPosition(hDummy4,-1,pDummy4);
	simGetObjectPosition(hDummyCentre1,-1,pDummyCentre1);
	simGetObjectPosition(hDummyCentre2,-1,pDummyCentre2);
	simGetObjectPosition(hCuboid1,-1,pCuboid1);
	simGetObjectPosition(hCuboid2,-1,pCuboid2);
	simGetObjectPosition(hCuboid3,-1,pCuboid3);
	simGetObjectPosition(hCuboid4,-1,pCuboid4);
	simGetObjectPosition(hDummyEvasive1,-1,pDummyEvasive1);
	simGetObjectPosition(hDummyEvasive2,-1,pDummyEvasive2);
	simGetObjectPosition(hDummyBack1,-1,pDummyBack1);
	simGetObjectPosition(hDummyBack2,-1,pDummyBack2);

	//Compute interesting quantities for evasive trajectory
	inflec1 = sqrtf(pow(pDummyEvasive1[0] - pCuboid1[0],2) + pow(pDummyEvasive1[1] - pCuboid1[1],2)) - half_cuboid1 - l;
	y_m1 = half_cuboid1 + half_width_car + x_s;
	inflec2 = -(sqrtf(pow(pDummyEvasive2[0] - pCuboid2[0],2) + pow(pDummyEvasive2[1] - pCuboid2[1],2)) - half_cuboid2 - l);
	// std::cout << "inflect 2:  "<< inflec2 <<"\n" <<std::endl;
	// std::cout << "inflect 1:  "<< inflec1 <<"\n" <<std::endl;

	y_m2 = -(half_cuboid2 + half_width_car + x_s);


	// ray circonference
	r = sqrtf(pow(pDummy2[0] - pDummy3[0],2) + pow(pDummy2[1] - pDummy3[1],2)) * 1/2;
	//std::cout << "Raggio:  "<< r <<"\n" <<std::endl;     Combacia valore che torna è 5.09796, valore ottenuto da path rettilineo è 4.925

	// First segment of the trail
	//Init point
	xIni1 = pDummy1[0];
	yIni1 = pDummy1[1];
	zIni1 = pDummy1[2];
	zCar  = zIni1 + 0.2;

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

	xEv1 = pDummyEvasive1[0];
	yEv1 = pDummyEvasive1[1];

	yBack1 = pDummyBack1[1];

	xEv2 = pDummyEvasive2[0];
	yEv2 = pDummyEvasive2[1];

	yBack2 = pDummyBack2[1];


	dt = (float)simGetSimulationTimeStep();

	trajDur1 = 1.25;
	trajDurEvasive = 1.25;
	trajDur1Back = 2.25;
	// trajDur1 = 1.0;
	// trajDurEvasive = 1.0;
	// trajDur1Back = 1.4;

	// trajDur2 = 5.0;
	trajDur2 = 4.0;

	trajDur3 = 1.25;
	trajDurEvasive2 = 1.25;
	trajDur2Back = 2.25;
	// trajDur3 = 1.0;
	// trajDurEvasive2 = 1.0;
	// trajDur2Back = 1.4;

	// trajDur4 = 5.0;
	trajDur4 = 4.0;

	std::cout << "Initialization Completed" << std::endl;
}


void Execution(){
	// q = (x, y, theta, phi)  // u = (v, omega)

	Eigen::Vector4f q_k, q_kp1; // current and next configuration

	float u, u1, u2, vd;

	Eigen::Vector2f v, v_desired, pd, pr;

	simFloat pRobot[3];
	simFloat pAckerCar[3];
	simFloat eRobot[3];
	simFloat eAckerCar[3];
	simFloat pClosestPointFront[3], pClosestPointRight[3];

	float norm_v, omega, norm_v_desired;


	// get the current configuration
	simGetObjectPosition(hAckerCar, -1, pAckerCar);
	simGetObjectOrientation(hAckerCar, -1, eAckerCar);
	//std::cout << "position inizio"<< pAckerCar[0] <<"\n" <<pAckerCar[1]<<"\n"<< pAckerCar[2]<<"\n" <<std::endl;

	//KINEMATIC MODEL
	q_k << pAckerCar[0], pAckerCar[1], eAckerCar[2], phi;

	float theta = eAckerCar[2];
	float theta_car = theta + PI/2;
	// std::cout << "theta "<<theta <<"\n" <<std::endl;
	// std::cout << "theta car "<<theta_car <<"\n" <<std::endl;

	//we control these two new variable via the variable b
	// float b = 0.55;
	float b = 0.65;
	float y1 = pAckerCar[0] + l*cos(theta_car) + b*cos(theta_car+phi);
	float y2 = pAckerCar[1] + l*sin(theta_car) + b*sin(theta_car+phi);

	//std::cout << "p Car[0] "<<pAckerCar[0] <<"\n" <<std::endl;
	//std::cout << "p Car[1] "<<pAckerCar[1] <<"\n" <<std::endl;
	//std::cout << "Punto b       y1:     " << y1 <<"\n" <<std::endl;
	//std::cout << "Punto b       y2:     " << y2 <<"\n" <<std::endl;

	//CONSTRUCT THE PREDEFINED TRAJECTORY
	float t_sim = (float)simGetSimulationTime();
	float x = 0, y = 0;

	if(t_sim >= 0 && t_sim <= trajDur1){ 								 // TRAETTORIA PEZZO RETTILINEO        -      TRATTO  1

		float t1 = t_sim / trajDur1;

		x = t1 * xEv1 + (1 - t1) * xIni1;

		y = t1 * yEv1 + (1 - t1) * yIni1;

		v_desired(0) = (xEv1 - xIni1)/trajDur1;
		v_desired(1) = (yEv1 - yIni1)/trajDur1;

		std::cout << "Sono nel rettilineo 1" <<"\n" <<std::endl;

     }
	else if (t_sim > trajDur1 && t_sim <= trajDur1 + trajDurEvasive){  //TRAIETTORIA EVASIVE

		float t_ev = (t_sim-trajDur1) / trajDurEvasive;

		//traiettoria lineare per la y
		y = t_ev * yBack1 + (1 - t_ev) * yEv1;
		//evasive trajectory per la x
		x = y_m1 / (1 + exp(-a*y + a*inflec1));

		v_desired(0) = ((yBack1-yEv1)/trajDurEvasive)*(a*y_m1*exp(-a*y+a*inflec1))/pow((exp(-a*y+a*inflec1)+1),2);
		v_desired(1) = (yBack1-yEv1)/trajDurEvasive;

		x_end_ev = y1;
		y_end_ev = y2;

		std::cout << "Sono nell'evasive 1" <<"\n" <<std::endl;

	}
	else if (t_sim > trajDur1 + trajDurEvasive && t_sim <= trajDur1 + trajDurEvasive + trajDur1Back ){ // TRAIETTORIA RIENTRO

		float t_back = (t_sim-(trajDur1 + trajDurEvasive)) / trajDur1Back;

		x = t_back * xFin2 + (1 - t_back) * x_end_ev;

		y = t_back * yFin2 + (1 - t_back) * y_end_ev;

		v_desired(0) = (xFin2 - x_end_ev)/trajDur1Back;
		v_desired(1) = (yFin2 - y_end_ev)/trajDur1Back;

		std::cout << "Sono nel rientro 1" <<"\n" <<std::endl;
	}
	else if(t_sim > trajDur1 + trajDurEvasive + trajDur1Back && t_sim <= trajDur1 + trajDurEvasive + trajDur1Back + trajDur2){				// TRAETTORIA PEZZO CURVILINEO        -      TRATTO  2
		//std::cout << "SONO NELLA TRAETTORIA CIRCOLARE 1" << std::endl;

		// //traiettoria circolare
		float t2 = (t_sim-(trajDur1 + trajDurEvasive + trajDur1Back)) / trajDur2;
		float theta = PI*t2;


		x = pDummyCentre1[0] - r*cos(theta);   //punto sulla traiettoria coordinata x
		y = pDummyCentre1[1] - r*sin(theta);   //punto sulla traiettoria coordinata y

		v_desired(0) =  PI*r*sin(theta)/trajDur2;
		v_desired(1) = -PI*r*cos(theta)/trajDur2;

		std::cout << "Sono nella curva 1" <<"\n" <<std::endl;

	}
	else if(t_sim > trajDur1 + trajDurEvasive + trajDur1Back + trajDur2 && t_sim <= trajDur1 + trajDurEvasive + trajDur1Back + trajDur2 + trajDur3){    // TRAETTORIA PEZZO RETTILINEO        -      TRATTO  3

		float t3 = (t_sim-(trajDur1 + trajDurEvasive + trajDur1Back + trajDur2)) / trajDur3;

		x = t3 * xEv2 + (1 - t3) * xIni3;
		y = t3 * yEv2 + (1 - t3) * yIni3;

		v_desired(0) = (xEv2 - xIni3)/trajDur3;
		v_desired(1) = (yEv2 - yIni3)/trajDur3;

		std::cout << "Sono nel rettilineo 2" <<"\n" <<std::endl;
	}
	else if(t_sim > trajDur1 + trajDurEvasive + trajDur1Back + trajDur2 + trajDur3 && t_sim <= trajDur1 + trajDurEvasive + trajDur1Back + trajDur2 + trajDur3 + trajDurEvasive2){  //TRAIETTORIA EVASIVE 2

		float t_ev = (t_sim-(trajDur1 + trajDurEvasive + trajDur1Back + trajDur2 + trajDur3)) / trajDurEvasive2;

		//traiettoria lineare per la y
		y = t_ev * yBack2 + (1 - t_ev) * yEv2;
		//evasive trajectory per la x
		x = y_m2 / (1 + exp(-a2*y + a2*inflec2));

		v_desired(0) = ((yBack2-yEv2)/trajDurEvasive2)*(a2*y_m2*exp(-a2*y+a2*inflec2))/pow((exp(-a2*y+a2*inflec2)+1),2);
		v_desired(1) = (yBack2-yEv2)/trajDurEvasive2;

		x_end_ev = y1;
		y_end_ev = y2;

		std::cout << "Sono nell'evasive 2" <<"\n" <<std::endl;

	}
	else if(t_sim > trajDur1 + trajDurEvasive + trajDur1Back + trajDur2 + trajDur3 + trajDurEvasive2 && t_sim <= trajDur1 + trajDurEvasive + trajDur1Back + trajDur2 + trajDur3 + trajDurEvasive2 + trajDur2Back){ //TRAIETTORIA RIENTRO

		float t_back = (t_sim-(trajDur1 + trajDurEvasive + trajDur1Back + trajDur2 + trajDur3 + trajDurEvasive2)) / trajDur2Back;

		x = t_back * xFin4 + (1 - t_back) * x_end_ev;

		y = t_back * yFin4 + (1 - t_back) * y_end_ev;

		v_desired(0) = (xFin4 - x_end_ev)/trajDur2Back;
		v_desired(1) = (yFin4 - y_end_ev)/trajDur2Back;

		std::cout << "Sono nel rientro 2" <<"\n" <<std::endl;
	}
	else if (t_sim > trajDur1 + trajDurEvasive + trajDur1Back + trajDur2 + trajDur3 + trajDurEvasive2 + trajDur2Back && t_sim <= trajDur1 + trajDurEvasive + trajDur1Back + trajDur2 + trajDur3 + trajDurEvasive2 + trajDur2Back + trajDur4){   // TRAETTORIA PEZZO CURVILINEO        -      TRATTO  4

		 float t4 = (t_sim-(trajDur1 + trajDurEvasive + trajDur1Back + trajDur2 + trajDur3 + trajDurEvasive2 + trajDur2Back)) / trajDur4;
		 float theta = PI*t4;

		 x = pDummyCentre2[0] + r*cos(theta);
		 y = pDummyCentre2[1] + r*sin(theta);

		 v_desired(0) = -PI*r*sin(PI*t4)/ trajDur4;
		 v_desired(1) = PI*r*cos(PI*t4)/ trajDur4;

		 std::cout << "Sono nella curva 2" <<"\n" <<std::endl;
	}
	else if(t_sim > trajDur1 + trajDurEvasive + trajDur1Back + trajDur2 + trajDur3 + trajDurEvasive2 + trajDur2Back + trajDur4){

		x = xFin5;
		y = yFin5;

		v_desired(0) = 0;
		v_desired(1) = 0;

		std::cout << "IL PUNTO È ARRIVATO"<< "\n" <<std::endl;


	}

	//COMPUTE THE CONTROL INPUTS (via the controller)

	pd(0) = x; // position desired
	pd(1) = y;
	pr(0) = y1; // robot position
	pr(1) = y2;

	// Error in position
	// float pos_error = sqrtf(pow(pd(0) - pr(0),2) + pow(pd(1) - pr(1),2));
	// std::cout << "pos_error "<<pos_error <<"\n" <<std::endl;
	// std::cout << "pd_x "<<pd(0)<<"\n" <<std::endl;
	// std::cout << "pr_x "<<pr(0)<<"\n" <<std::endl;
	// std::cout << "pd_y "<<pd(1)<<"\n" <<std::endl;
	// std::cout << "pr_y "<<pr(1)<<"\n" <<std::endl;


	// Define random generator with Gaussian distribution
	const double mean = 0.0;
	const double stddev = 0.1;
	// construct a trivial random generator engine from a time-based seed:
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator (seed);
	std::normal_distribution<double> distribution (mean,stddev);

	float beta = abs(distribution(generator));

	float vmax = 10;
	float hdb_x = (abs(v_desired(0))/vmax)* sqrtf(pow(pd(0) - pr(0),2) + pow(pd(1) - pr(1),2)) + beta;
	float hdb_y = (abs(v_desired(1))/vmax)* sqrtf(pow(pd(0) - pr(0),2) + pow(pd(1) - pr(1),2)) + beta;

	float radice = sqrtf(pow(pd(0) - pr(0),2) + pow(pd(1) - pr(1),2));

	// std::cout << "hdb_x "<<hdb_x <<"\n" <<std::endl;
	// std::cout << "v_desired on y "<<v_desired(1) <<"\n" <<std::endl;
	// std::cout << " radice "<< radice <<"\n" <<std::endl;
	// std::cout << "hdb_y "<<hdb_y <<"\n" <<std::endl;

  //HDB CONTROL LAWS
	u1 = v_desired(0) + hdb_x * (pd(0) - pr(0));
	u2 = v_desired(1) + hdb_y * (pd(1) - pr(1));

	// std::cout << "u1 "<<u1 <<"\n" <<std::endl;
	// std::cout << "u2 "<<u2 <<"\n" <<std::endl;

	// u1 = v_desired(0) + 1.1 * (pd(0) - pr(0));
	// u2 = v_desired(1) + 1.1 * (pd(1) - pr(1));

	//REAR-WHELL DRIVE
	// float theta_dot = (u1*cos(phi + theta_car)*sin(phi) + u2*sin(phi)*sin(phi + theta_car))*(1/l);
	// float phi_dot = -(cos(theta_car+phi)*sin(phi)*(1/l)+sin(theta_car+phi)*(1/b))*u1-(sin(theta_car+phi)*sin(phi)*(1/l)-cos(theta_car+phi)*(1/b))*u2;

	//FRONT-WHEEL DRIVE
	float v_commanded = (u1*cos(theta_car+phi) + u2*sin(theta_car+phi));
	float omega_commanded = ((u2*(2*l*cos(theta_car+phi) - b*cos(theta_car) + b*cos(theta_car+2*phi)))/(2*b*l)) - ((u1*(2*l*sin(theta_car+phi) - b*sin(theta_car) + b*sin(theta_car+2*phi)))/(2*b*l));

	float theta_dot = (sin(phi)/l)*v_commanded;
	float phi_dot = omega_commanded;

	// std::cout << "theta_dot "<<theta_dot <<"\n" <<std::endl;
	// std::cout << "phi_dot "<<phi_dot <<"\n" <<std::endl;

	// Euler Integration
	y1 = y1 + u1*dt; // u1 = y1dot
	y2 = y2 + u2*dt; // u2 = y2dot

	q_kp1(2) = theta + theta_dot *dt;
	q_kp1(3) = phi + phi_dot *dt;

	theta_car = theta_car + theta_dot *dt;
	phi = phi + phi_dot *dt;


	// y1 = pAckerCar[0] + l*cos(theta) + b*cos(theta+phi);
	// y2 = pAckerCar[1] + l*sin(theta) + b*sin(theta+phi);

	// Formula Inversa per trovare la posizione della macchina
	q_kp1(0) = y1 - l*cos(theta_car) - b*cos(theta_car + phi);
	q_kp1(1) = y2 - l*sin(theta_car) - b*sin(theta_car + phi);


	// set the robot in the new configuration
	pAckerCar[0] = q_kp1(0);
	pAckerCar[1] = q_kp1(1);
	pAckerCar[2] = zCar;
	eAckerCar[2] = q_kp1(2);
	phi = q_kp1(3);


	simSetObjectPosition(hAckerCar, -1, pAckerCar);
	simSetObjectOrientation(hAckerCar, -1, eAckerCar);


	//Take the closest detected point
	simGetObjectPosition(hClosestPointFront, -1, pClosestPointFront);
	// std::cout << "posizione macchina x y z"<< "\n" << pAckerCar[0] <<"\n" <<pAckerCar[1]<<"\n"<< pAckerCar[2]<<"\n" <<std::endl;
	// std::cout << "posizione closest point"<< "\n" << pClosestPointFront[0] <<"\n" <<pClosestPointFront[1]<<"\n"<< pClosestPointFront[2]<<"\n" <<std::endl;
	simGetObjectPosition(hClosestPointRight, -1, pClosestPointRight);

	dist_closest_point_front = sqrtf(pow(pClosestPointFront[0]-pAckerCar[0],2) + pow(pClosestPointFront[1]-pAckerCar[1],2));
	std::cout << "distanza closest point front"<< "\n" << dist_closest_point_front<<"\n"  <<std::endl;
	dist_closest_point_right = sqrtf(pow(pClosestPointRight[0]-pAckerCar[0],2) + pow(pClosestPointRight[1]-pAckerCar[1],2));
	std::cout << "distanza closest point right"<< "\n" << dist_closest_point_right<<"\n"  <<std::endl;

	if(dist_closest_point_front == 0.02 && dist_closest_point_right == 0.02){

		min_dist = 100;
		direction = -1;

		std::cout << "NON STO OSSERVANDO NIENTE"<< "\n" <<std::endl;


	}
	else if(dist_closest_point_front > 0.02){
		//solo se facciamo detection di un cuboid
		//0.2 è la distanza dalla closest point alla car quando viene rimesso alla posizione di default
		min_dist = dist_closest_point_front;
		direction = 0;
		std::cout << "min_dist"<< "\n" << min_dist<<"\n"  <<std::endl;
		std::cout << "STO OSSERVANDO CON KINECT FRONT"<< "\n" <<std::endl;

	}
	else if (dist_closest_point_right > 0.02){

		min_dist = dist_closest_point_right;
		direction = 1;
		std::cout << "min_dist"<< "\n" << min_dist<<"\n"  <<std::endl;
		std::cout << "STO OSSERVANDO CON KINECT RIGHT"<< "\n" <<std::endl;
	}


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
