///////////////////////////////////////////////////
//
//   Demo code about 
//
//     - modeling a complex mechanism (a quarter car model)
//     - using the ChLinkSpring to make spring-damper system
//     - using the ChLinkDistance class to reperesent 
//       long and thin massless rods, whose mass is negligible 
//       for dynamical analysis (as often happens in mechanisms)
//       so they can be modeled as 'distance' constraints
//       instead of making a thin body with small mass and two
//       spherical joints at the end (wihch would be much less
//       efficient from the computational point of view).
//
//	 CHRONO    
//   ------
//   Multibody dinamics engine
//  
// ------------------------------------------------ 
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------ 
///////////////////////////////////////////////////
 
  
 
#include "physics/CHapidll.h" 
#include "physics/CHsystem.h"
#include "physics/CHlinkDistance.h"
#include "irrlicht_interface/CHbodySceneNode.h"
#include "irrlicht_interface/CHbodySceneNodeTools.h" 
#include "irrlicht_interface/CHirrAppInterface.h"
#include "core/CHrealtimeStep.h"

#include <iostream>
#include <vector>
#include <string>
#include <sstream>

#include <irrlicht.h>


// Use the namespace of Chrono

using namespace chrono;
using namespace std;

// Use the main namespaces of Irrlicht
using namespace irr;

using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

vector<char*> bodyTypes;

double SCALEMASS = 1;
double SCALELENGTH = 1;

double getRandomNumber(double min, double max)
{
   // x is in [0,1[
   double x = rand()/static_cast<double>(RAND_MAX);

   // [0,1[ * (max - min) + min is in [min,max[
   double that = min + ( x * (max - min) );

   return that;
}

int generateRockObject(int rockIndex, double rockRadius)
{
	char filename[100];
	sprintf(filename, "../data/humvee/rocks/rock%d.obj", rockIndex);
	ChStreamOutAsciiFile rockFile(filename);

	char filename2[100];
	sprintf(filename2, "../data/humvee/rocks/rockX%d.obj", rockIndex);
	ChStreamOutAsciiFile rockFileX(filename2);

	rockFile << "# OBJ file created by ply_to_obj.c\n#\ng Object001";

	// add vertices
	//1
	ChVector<double> vertex = ChVector<>(-0.57735,  -0.57735,  0.57735);
	ChVector<double> vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " <<vertex.z;
	//2
	vertex = ChVector<>(0.934172,  0.356822,  0);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//3
	vertex = ChVector<>(0.934172,  -0.356822,  0);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//4
	vertex = ChVector<>(-0.934172,  0.356822,  0);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//5
	vertex = ChVector<>(-0.934172,  -0.356822,  0);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//6
	vertex = ChVector<>(0,  0.934172,  0.356822);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//7
	vertex = ChVector<>(0,  0.934172,  -0.356822);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//8
	vertex = ChVector<>(0.356822,  0,  -0.934172);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//9
	vertex = ChVector<>(-0.356822,  0,  -0.934172);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//10
	vertex = ChVector<>(0,  -0.934172,  -0.356822);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//11
	vertex = ChVector<>(0,  -0.934172,  0.356822);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//12
	vertex = ChVector<>(0.356822,  0,  0.934172);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//13
	vertex = ChVector<>(-0.356822,  0,  0.934172);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//14
	vertex = ChVector<>(0.57735,  0.57735,  -0.57735);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//15
	vertex = ChVector<>(0.57735,  0.57735,  0.57735);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//16
	vertex = ChVector<>(-0.57735,  0.57735,  -0.57735);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//17
	vertex = ChVector<>(-0.57735,  0.57735,  0.57735);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//18
	vertex = ChVector<>(0.57735,  -0.57735,  -0.57735);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//19
	vertex = ChVector<>(0.57735,  -0.57735,  0.57735);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//20
	vertex = ChVector<>(-0.57735,  -0.57735,  -0.57735);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;

	double Cz = -1.0;
	// add vertices
	//1
	vertex = ChVector<>(-0.57735,  -0.57735,  0.57735);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFileX << "\nv " << vertex.x << " " << vertex.y << " " << Cz*vertex.z;
	//2
	vertex = ChVector<>(0.934172,  0.356822,  0);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFileX << "\nv " << vertex.x << " " << vertex.y << " " << Cz*vertex.z;
	//3
	vertex = ChVector<>(0.934172,  -0.356822,  0);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFileX << "\nv " << vertex.x << " " << vertex.y << " " << Cz*vertex.z;
	//4
	vertex = ChVector<>(-0.934172,  0.356822,  0);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFileX << "\nv " << vertex.x << " " << vertex.y << " " << Cz*vertex.z;
	//5
	vertex = ChVector<>(-0.934172,  -0.356822,  0);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFileX << "\nv " << vertex.x << " " << vertex.y << " " << Cz*vertex.z;
	//6
	vertex = ChVector<>(0,  0.934172,  0.356822);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFileX << "\nv " << vertex.x << " " << vertex.y << " " << Cz*vertex.z;
	//7
	vertex = ChVector<>(0,  0.934172,  -0.356822);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFileX << "\nv " << vertex.x << " " << vertex.y << " " << Cz*vertex.z;
	//8
	vertex = ChVector<>(0.356822,  0,  -0.934172);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFileX << "\nv " << vertex.x << " " << vertex.y << " " << Cz*vertex.z;
	//9
	vertex = ChVector<>(-0.356822,  0,  -0.934172);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFileX << "\nv " << vertex.x << " " << vertex.y << " " << Cz*vertex.z;
	//10
	vertex = ChVector<>(0,  -0.934172,  -0.356822);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFileX << "\nv " << vertex.x << " " << vertex.y << " " << Cz*vertex.z;
	//11
	vertex = ChVector<>(0,  -0.934172,  0.356822);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFileX << "\nv " << vertex.x << " " << vertex.y << " " << Cz*vertex.z;
	//12
	vertex = ChVector<>(0.356822,  0,  0.934172);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFileX << "\nv " << vertex.x << " " << vertex.y << " " << Cz*vertex.z;
	//13
	vertex = ChVector<>(-0.356822,  0,  0.934172);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFileX << "\nv " << vertex.x << " " << vertex.y << " " << Cz*vertex.z;
	//14
	vertex = ChVector<>(0.57735,  0.57735,  -0.57735);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFileX << "\nv " << vertex.x << " " << vertex.y << " " << Cz*vertex.z;
	//15
	vertex = ChVector<>(0.57735,  0.57735,  0.57735);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFileX << "\nv " << vertex.x << " " << vertex.y << " " << Cz*vertex.z;
	//16
	vertex = ChVector<>(-0.57735,  0.57735,  -0.57735);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFileX << "\nv " << vertex.x << " " << vertex.y << " " << Cz*vertex.z;
	//17
	vertex = ChVector<>(-0.57735,  0.57735,  0.57735);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFileX << "\nv " << vertex.x << " " << vertex.y << " " << Cz*vertex.z;
	//18
	vertex = ChVector<>(0.57735,  -0.57735,  -0.57735);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFileX << "\nv " << vertex.x << " " << vertex.y << " " << Cz*vertex.z;
	//19
	vertex = ChVector<>(0.57735,  -0.57735,  0.57735);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFileX << "\nv " << vertex.x << " " << vertex.y << " " << Cz*vertex.z;
	//20
	vertex = ChVector<>(-0.57735,  -0.57735,  -0.57735);
	vertPert = ChVector<>(getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3),getRandomNumber(-0.3,0.3));
	vertex = rockRadius*(vertex+vertPert);
	rockFileX << "\nv " << vertex.x << " " << vertex.y << " " << Cz*vertex.z;

	// add faces
	rockFile << "\n\nf  19  3  2\nf  12  19  2\nf  15  12  2\nf  8  14  2\nf  18  8  2\nf  3  18  2\nf  20  5  4\nf  9  20  4\nf  16  9  4\nf  13  17  4\nf  1  13  4\nf  5  1  4\nf  7  16  4\nf  6  7  4\nf  17  6  4\nf  6  15  2\nf  7  6  2\nf  14  7  2\nf  10  18  3\nf  11  10  3\nf  19  11  3\nf  11  1  5\nf  10  11  5\nf  20  10  5\nf  20  9  8\nf  10  20  8\nf  18  10  8\nf  9  16  7\nf  8  9  7\nf  14  8  7\nf  12  15  6\nf  13  12  6\nf  17  13  6\nf  13  1  11\nf  12  13  11\nf  19  12  11\n";

	// add faces
	rockFileX << "\n\nf  19  3  2\nf  12  19  2\nf  15  12  2\nf  8  14  2\nf  18  8  2\nf  3  18  2\nf  20  5  4\nf  9  20  4\nf  16  9  4\nf  13  17  4\nf  1  13  4\nf  5  1  4\nf  7  16  4\nf  6  7  4\nf  17  6  4\nf  6  15  2\nf  7  6  2\nf  14  7  2\nf  10  18  3\nf  11  10  3\nf  19  11  3\nf  11  1  5\nf  10  11  5\nf  20  10  5\nf  20  9  8\nf  10  20  8\nf  18  10  8\nf  9  16  7\nf  8  9  7\nf  14  8  7\nf  12  15  6\nf  13  12  6\nf  17  13  6\nf  13  1  11\nf  12  13  11\nf  19  12  11\n";

	return 0;
}

// First of all, define a class for the 'car' (that is, a set of
// bodies and links which are grouped within this class; so it is 
// easier to manage data structures in this example).

class MySimpleCar {
public:
		// THE DATA

	double throttle; // actual value 0...1 of gas throttle.
	double conic_tau; // the transmission ratio of the conic gears at the rear axle
	double gear_tau; // the actual tau of the gear
	double max_motor_torque; // the max torque of the motor [Nm];
	double max_motor_speed;	 // the max rotation speed of the motor [rads/s]

	double tireRadius;	// radius of the tires
	double vehicleLength; // length of the vehicle
	double vehicleWidth; // width of the vehicle
	double wheelSep; // Wheel distance from vehicle body
	double spindleSideDim; // spindle dimension
	bool useSphericalTire; // represent the tires with spheres

		// The parts making the car, as 3d Irrlicht scene nodes, each containing
		// the ChBody object
			// .. truss:
	ChBodySceneNode* truss;
			// .. right front suspension:
	ChBodySceneNode* spindleRF;
	ChBodySceneNode* wheelRF;
	ChSharedPtr<ChLinkLockRevolute> link_revoluteRF;
	ChSharedPtr<ChLinkDistance> link_distRFU1;
	ChSharedPtr<ChLinkDistance> link_distRFU2;
	ChSharedPtr<ChLinkDistance> link_distRFL1;
	ChSharedPtr<ChLinkDistance> link_distRFL2;
	ChSharedPtr<ChLinkSpring>   link_springRF;
	ChSharedPtr<ChLinkDistance> link_distRSTEER;
			// .. left front suspension:
	ChBodySceneNode* spindleLF;
	ChBodySceneNode* wheelLF;
	ChSharedPtr<ChLinkLockRevolute> link_revoluteLF;
	ChSharedPtr<ChLinkDistance> link_distLFU1;
	ChSharedPtr<ChLinkDistance> link_distLFU2;
	ChSharedPtr<ChLinkDistance> link_distLFL1;
	ChSharedPtr<ChLinkDistance> link_distLFL2;
	ChSharedPtr<ChLinkSpring>   link_springLF;
	ChSharedPtr<ChLinkDistance> link_distLSTEER;
			// .. right back suspension:
	ChBodySceneNode* spindleRB;
	ChBodySceneNode* wheelRB;
	ChSharedPtr<ChLinkLockRevolute> link_revoluteRB;
	ChSharedPtr<ChLinkDistance> link_distRBU1;
	ChSharedPtr<ChLinkDistance> link_distRBU2;
	ChSharedPtr<ChLinkDistance> link_distRBL1;
	ChSharedPtr<ChLinkDistance> link_distRBL2;
	ChSharedPtr<ChLinkSpring>   link_springRB;
	ChSharedPtr<ChLinkDistance> link_distRBlat;
	ChSharedPtr<ChLinkEngine>   link_engineL;
			// .. left back suspension:
	ChBodySceneNode* spindleLB;
	ChBodySceneNode* wheelLB;
	ChSharedPtr<ChLinkLockRevolute> link_revoluteLB;
	ChSharedPtr<ChLinkDistance> link_distLBU1;
	ChSharedPtr<ChLinkDistance> link_distLBU2;
	ChSharedPtr<ChLinkDistance> link_distLBL1;
	ChSharedPtr<ChLinkDistance> link_distLBL2;
	ChSharedPtr<ChLinkSpring>   link_springLB;
	ChSharedPtr<ChLinkDistance> link_distLBlat;
	ChSharedPtr<ChLinkEngine>   link_engineR;
		
		// THE FUNCTIONS

		// Build and initialize the car, creating all bodies corresponding to
		// the various parts and adding them to the physical system - also creating
		// and adding constraints to the system.
	MySimpleCar(ChSystem&  my_system,	///< the chrono::engine physical system 
				ISceneManager* msceneManager, ///< the Irrlicht scene manager for 3d shapes
				IVideoDriver* mdriver	///< the Irrlicht video driver
				)
			{
				tireRadius = 0.25;
				vehicleLength = 3;
				vehicleWidth = .01;
				wheelSep = .5;
				spindleSideDim = .01;

				throttle = 0; // initially, gas throttle is 0.
				conic_tau = 0.2;
				gear_tau = 0.3;
				max_motor_torque = 80;//80;
				max_motor_speed = 1100;//800;
				useSphericalTire = true; // represent the tires with spheres

				// --- The car body --- 

				IAnimatedMesh*	humvee_bodyMesh = msceneManager->getMesh("../data/humvee/humvee2.obj");
				truss = (ChBodySceneNode*)addChBodySceneNode(
														&my_system, msceneManager, humvee_bodyMesh,
														1500.0*SCALEMASS,
														ChVector<>(0, 1, 0),
														QUNIT);
				truss->GetBody()->SetInertiaXX(ChVector<>(4.8, 4.5, 1));
				truss->GetBody()->SetBodyFixed(false);
				truss->GetBody()->SetCollide(false);
				truss->addShadowVolumeSceneNode();
				bodyTypes.push_back("chassis");

				//truss = (ChBodySceneNode*)addChBodySceneNode_easyBox(
				//										&my_system, msceneManager,
				//										150.0,
				//										ChVector<>(0, 1 ,0),
				//										QUNIT,
				//										ChVector<>(vehicleWidth, 0.1, vehicleLength) );
				//truss->GetBody()->SetInertiaXX(ChVector<>(4.8, 4.5, 1));
				//truss->GetBody()->SetBodyFixed(false);
				//truss->addShadowVolumeSceneNode();

				// --- Right Front suspension --- 

				// ..the car right-front spindle
				spindleRF = (ChBodySceneNode*)addChBodySceneNode_easyBox(
														&my_system, msceneManager,
														80.0*SCALEMASS,
														ChVector<>(0.5*vehicleWidth+wheelSep-.2, 1 , 0.5*vehicleLength-0.5),
														QUNIT, 
														ChVector<>(0.1, spindleSideDim, spindleSideDim) );
				spindleRF->GetBody()->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
				spindleRF->GetBody()->SetCollide(false);
				bodyTypes.push_back("spindle");

				video::ITexture* cylinderMap = mdriver->getTexture("../data/bluwhite.png");
				// ..the car right-front wheel
				if(useSphericalTire)
				{
					wheelRF = (ChBodySceneNode*)addChBodySceneNode_easySphere(
														&my_system, msceneManager,
														30.0*SCALEMASS,
														ChVector<>(0.5*vehicleWidth+wheelSep, 1 , 0.5*vehicleLength-0.5),
														tireRadius);
					wheelRF->GetBody()->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
					wheelRF->GetBody()->SetCollide(true);
					wheelRF->GetBody()->SetFriction(1.0);
					wheelRF->setMaterialTexture(0,	cylinderMap);
					wheelRF->addShadowVolumeSceneNode();
					bodyTypes.push_back("wheelRF");
				}
				else
				{
					wheelRF = (ChBodySceneNode*)addChBodySceneNode_easyCylinder(
														&my_system, msceneManager,
														30.0*SCALEMASS,
														ChVector<>(0.5*vehicleWidth+wheelSep, 1 , 0.5*vehicleLength-0.5),
														chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Z), 
														ChVector<>(tireRadius*2, 0.3, tireRadius*2) );
					wheelRF->GetBody()->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
					wheelRF->GetBody()->SetCollide(true);
					wheelRF->GetBody()->SetFriction(1.0);
					wheelRF->setMaterialTexture(0,	cylinderMap);
					wheelRF->addShadowVolumeSceneNode();
					bodyTypes.push_back("wheelRF");
				}

				// .. create the revolute joint between the wheel and the spindle
				link_revoluteRF = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute); // right, front, upper, 1
				link_revoluteRF->Initialize(wheelRF->GetBody(), spindleRF->GetBody(), 
					ChCoordsys<>(ChVector<>(0.5*vehicleWidth+wheelSep, 1, 0.5*vehicleLength-0.5) , chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Y)) );
				my_system.AddLink(link_revoluteRF);

				// .. impose distance between two parts (as a massless rod with two spherical joints at the end)
				link_distRFU1 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right, front, upper, 1
				link_distRFU1->Initialize(truss->GetBody(), spindleRF->GetBody(), false, ChVector<>(0.5*vehicleWidth,1.2,0.5*vehicleLength-0.3), ChVector<>(0.5*vehicleWidth+wheelSep-.25,1.2,0.5*vehicleLength-0.5));
				my_system.AddLink(link_distRFU1);
			 
				link_distRFU2 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right, front, upper, 2
				link_distRFU2->Initialize(truss->GetBody(), spindleRF->GetBody(), false, ChVector<>(0.5*vehicleWidth,1.2,0.5*vehicleLength-0.7), ChVector<>(0.5*vehicleWidth+wheelSep-.25,1.2,0.5*vehicleLength-0.5));
				my_system.AddLink(link_distRFU2);

				link_distRFL1 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right, front, lower, 1
				link_distRFL1->Initialize(truss->GetBody(), spindleRF->GetBody(), false, ChVector<>(0.5*vehicleWidth,0.8,0.5*vehicleLength-0.3), ChVector<>(0.5*vehicleWidth+wheelSep-.25,0.8,0.5*vehicleLength-0.5));
				my_system.AddLink(link_distRFL1);

				link_distRFL2 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right, front, lower, 2
				link_distRFL2->Initialize(truss->GetBody(), spindleRF->GetBody(), false, ChVector<>(0.5*vehicleWidth,0.8,0.5*vehicleLength-0.7), ChVector<>(0.5*vehicleWidth+wheelSep-.25,0.8,0.5*vehicleLength-0.5));
				my_system.AddLink(link_distRFL2);
				
				// .. create the spring between the truss and the spindle
				link_springRF = ChSharedPtr<ChLinkSpring>(new ChLinkSpring);
				link_springRF->Initialize(truss->GetBody(), spindleRF->GetBody(), false, ChVector<>(0.5*vehicleWidth,1.2,0.5*vehicleLength-0.5), ChVector<>(0.5*vehicleWidth+wheelSep-.25,0.8,0.5*vehicleLength-0.5));
				link_springRF->Set_SpringK(283000*SCALEMASS/SCALELENGTH);
				link_springRF->Set_SpringR(800*SCALEMASS/SCALELENGTH);
				my_system.AddLink(link_springRF);

				// .. create the rod for steering the wheel
				link_distRSTEER = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right steer
				link_distRSTEER->Initialize(truss->GetBody(), spindleRF->GetBody(), false, ChVector<>(0.5*vehicleWidth,1.21,0.5*vehicleLength-0.1), ChVector<>(0.5*vehicleWidth+wheelSep-.25,1.21,0.5*vehicleLength-0.2));
				my_system.AddLink(link_distRSTEER);


				// --- Left Front suspension --- 

				// ..the car right-front spindle
				spindleLF = (ChBodySceneNode*)addChBodySceneNode_easyBox(
														&my_system, msceneManager,
														80.0*SCALEMASS,
														ChVector<>(-(0.5*vehicleWidth+wheelSep-0.2), 1 , 0.5*vehicleLength-0.5),
														QUNIT, 
														ChVector<>(0.1, spindleSideDim, spindleSideDim) );
				spindleLF->GetBody()->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
				spindleLF->GetBody()->SetCollide(false);
				bodyTypes.push_back("spindle");

				// ..the car left-front wheel
				if(useSphericalTire)
				{
					wheelLF = (ChBodySceneNode*)addChBodySceneNode_easySphere(
															&my_system, msceneManager,
															30.0*SCALEMASS,
															ChVector<>(-(0.5*vehicleWidth+wheelSep), 1 , 0.5*vehicleLength-0.5),
															tireRadius);
					wheelLF->GetBody()->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
					wheelLF->GetBody()->SetCollide(true);
					wheelLF->GetBody()->SetFriction(1.0);
					wheelLF->setMaterialTexture(0,	cylinderMap);
					wheelLF->addShadowVolumeSceneNode();
					bodyTypes.push_back("wheelLF");
				}
				else
				{
					wheelLF = (ChBodySceneNode*)addChBodySceneNode_easyCylinder(
														&my_system, msceneManager,
														30.0*SCALEMASS,
														ChVector<>(-(0.5*vehicleWidth+wheelSep), 1 , 0.5*vehicleLength-0.5),
														chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Z), 
														ChVector<>(tireRadius*2, 0.3, tireRadius*2) );
					wheelLF->GetBody()->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
					wheelLF->GetBody()->SetCollide(true);
					wheelLF->GetBody()->SetFriction(1.0);
					wheelLF->setMaterialTexture(0,	cylinderMap);
					wheelLF->addShadowVolumeSceneNode();
					bodyTypes.push_back("wheelLF");
				}

				// .. create the revolute joint between the wheel and the spindle
				link_revoluteLF = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute); // left, front, upper, 1
				link_revoluteLF->Initialize(wheelLF->GetBody(), spindleLF->GetBody(), 
					ChCoordsys<>(ChVector<>(-(0.5*vehicleWidth+wheelSep), 1, 0.5*vehicleLength-0.5) , chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Y)) );
				my_system.AddLink(link_revoluteLF);

				// .. impose distance between two parts (as a massless rod with two spherical joints at the end)
				link_distLFU1 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // left, front, upper, 1
				link_distLFU1->Initialize(truss->GetBody(), spindleLF->GetBody(), false, ChVector<>(-(0.5*vehicleWidth),1.2,0.5*vehicleLength-0.3), ChVector<>(-(0.5*vehicleWidth+wheelSep-.25),1.2,0.5*vehicleLength-0.5));
				my_system.AddLink(link_distLFU1);
			 
				link_distLFU2 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // left, front, upper, 2
				link_distLFU2->Initialize(truss->GetBody(), spindleLF->GetBody(), false, ChVector<>(-(0.5*vehicleWidth),1.2,0.5*vehicleLength-0.7), ChVector<>(-(0.5*vehicleWidth+wheelSep-.25),1.2,0.5*vehicleLength-0.5));
				my_system.AddLink(link_distLFU2);

				link_distLFL1 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // left, front, lower, 1
				link_distLFL1->Initialize(truss->GetBody(), spindleLF->GetBody(), false, ChVector<>(-(0.5*vehicleWidth),0.8,0.5*vehicleLength-0.3), ChVector<>(-(0.5*vehicleWidth+wheelSep-.25),0.8,0.5*vehicleLength-0.5));
				my_system.AddLink(link_distLFL1);

				link_distLFL2 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // left, front, lower, 2
				link_distLFL2->Initialize(truss->GetBody(), spindleLF->GetBody(), false, ChVector<>(-(0.5*vehicleWidth),0.8,0.5*vehicleLength-0.7), ChVector<>(-(0.5*vehicleWidth+wheelSep-.25),0.8,0.5*vehicleLength-0.5));
				my_system.AddLink(link_distLFL2);
				
				// .. create the spring between the truss and the spindle
				link_springLF = ChSharedPtr<ChLinkSpring>(new ChLinkSpring);
				link_springLF->Initialize(truss->GetBody(), spindleLF->GetBody(), false, ChVector<>(-(0.5*vehicleWidth),1.2,0.5*vehicleLength-0.5), ChVector<>(-(0.5*vehicleWidth+wheelSep-.25),0.8,0.5*vehicleLength-0.5));
				link_springLF->Set_SpringK(283000*SCALEMASS/SCALELENGTH);
				link_springLF->Set_SpringR(800*SCALEMASS/SCALELENGTH);
				my_system.AddLink(link_springLF);

				// .. create the rod for steering the wheel
				link_distLSTEER = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right steer
				link_distLSTEER->Initialize(truss->GetBody(), spindleLF->GetBody(), false, ChVector<>(-(0.5*vehicleWidth),1.21,0.5*vehicleLength-0.1), ChVector<>(-(0.5*vehicleWidth+wheelSep-.25),1.21,0.5*vehicleLength-0.2));
				my_system.AddLink(link_distLSTEER);

				// --- Right Back suspension --- 

				// ..the car right-back spindle
				spindleRB = (ChBodySceneNode*)addChBodySceneNode_easyBox(
														&my_system, msceneManager,
														80.0*SCALEMASS,
														ChVector<>(0.5*vehicleWidth+wheelSep-.2, 1 , -(0.5*vehicleLength-0.5)),
														QUNIT, 
														ChVector<>(0.1, spindleSideDim, spindleSideDim) );
				spindleRB->GetBody()->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
				spindleRB->GetBody()->SetCollide(false);
				bodyTypes.push_back("spindle");

				// ..the car right-back wheel
				cylinderMap = mdriver->getTexture("../data/bluwhite.png");
				if(useSphericalTire)
				{
					wheelRB = (ChBodySceneNode*)addChBodySceneNode_easySphere(
															&my_system, msceneManager,
															30.0*SCALEMASS,
															ChVector<>(.5*vehicleWidth+wheelSep, 1 , -(0.5*vehicleLength-0.5)),
															tireRadius);
					wheelRB->GetBody()->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
					wheelRB->GetBody()->SetCollide(true);
					wheelRB->GetBody()->SetFriction(1.0);
					wheelRB->setMaterialTexture(0,	cylinderMap);
					wheelRB->addShadowVolumeSceneNode();
					bodyTypes.push_back("wheelRB");
				}
				else
				{
					wheelRB = (ChBodySceneNode*)addChBodySceneNode_easyCylinder(
															&my_system, msceneManager,
															30.0*SCALEMASS,
															ChVector<>(.5*vehicleWidth+wheelSep, 1 , -(0.5*vehicleLength-0.5)),
															chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Z), 
															ChVector<>(tireRadius*2, 0.3, tireRadius*2) );
					wheelRB->GetBody()->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
					wheelRB->GetBody()->SetCollide(true);
					wheelRB->GetBody()->SetFriction(1.0);
					wheelRB->setMaterialTexture(0,	cylinderMap);
					wheelRB->addShadowVolumeSceneNode();
					bodyTypes.push_back("wheelRB");
				}

				// .. create the revolute joint between the wheel and the spindle
				link_revoluteRB = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute); // right, back, upper, 1
				link_revoluteRB->Initialize(wheelRB->GetBody(), spindleRB->GetBody(), 
					ChCoordsys<>(ChVector<>(.5*vehicleWidth+wheelSep, 1, -(0.5*vehicleLength-0.5)) , chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Y)) );
				my_system.AddLink(link_revoluteRB);

				// .. create the motor transmission joint between the wheel and the truss (assuming small changes of alignment)
				link_engineR = ChSharedPtr<ChLinkEngine>(new ChLinkEngine); 
				link_engineR->Initialize(wheelRB->GetBody(), truss->GetBody(), 
					ChCoordsys<>(ChVector<>(.5*vehicleWidth+wheelSep, 1, -(0.5*vehicleLength-0.5)) , chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Y)) );
				link_engineR->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_CARDANO); // approx as a double Rzeppa joint
				link_engineR->Set_eng_mode(ChLinkEngine::ENG_MODE_TORQUE);
				my_system.AddLink(link_engineR);

				// .. impose distance between two parts (as a massless rod with two spherical joints at the end)
				link_distRBU1 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right, back, upper, 1
				link_distRBU1->Initialize(truss->GetBody(), spindleRB->GetBody(), false, ChVector<>(.5*vehicleWidth,1.2,-(0.5*vehicleLength-0.3)), ChVector<>(.5*vehicleWidth+wheelSep-.25,1.2,-(0.5*vehicleLength-0.5)));
				my_system.AddLink(link_distRBU1);
			 
				link_distRBU2 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right, back, upper, 2
				link_distRBU2->Initialize(truss->GetBody(), spindleRB->GetBody(), false, ChVector<>(.5*vehicleWidth,1.2,-(0.5*vehicleLength-0.7)), ChVector<>(.5*vehicleWidth+wheelSep-.25,1.2,-(0.5*vehicleLength-0.5)));
				my_system.AddLink(link_distRBU2);

				link_distRBL1 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right, back, lower, 1
				link_distRBL1->Initialize(truss->GetBody(), spindleRB->GetBody(), false, ChVector<>(.5*vehicleWidth,0.8,-(0.5*vehicleLength-0.3)), ChVector<>(.5*vehicleWidth+wheelSep-.25,0.8,-(0.5*vehicleLength-0.5)));
				my_system.AddLink(link_distRBL1);

				link_distRBL2 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right, back, lower, 2
				link_distRBL2->Initialize(truss->GetBody(), spindleRB->GetBody(), false, ChVector<>(.5*vehicleWidth,0.8,-(0.5*vehicleLength-0.7)), ChVector<>(.5*vehicleWidth+wheelSep-.25,0.8,-(0.5*vehicleLength-0.5)));
				my_system.AddLink(link_distRBL2);
				
				// .. create the spring between the truss and the spindle
				link_springRB = ChSharedPtr<ChLinkSpring>(new ChLinkSpring);
				link_springRB->Initialize(truss->GetBody(), spindleRB->GetBody(), false, ChVector<>(.5*vehicleWidth,1.2,-(0.5*vehicleLength-0.5)), ChVector<>(.5*vehicleWidth+wheelSep-.25,0.8,-(0.5*vehicleLength-0.5)));
				link_springRB->Set_SpringK(283000*SCALEMASS/SCALELENGTH);
				link_springRB->Set_SpringR(800*SCALEMASS/SCALELENGTH);
				my_system.AddLink(link_springRB);

				// .. create the rod for avoid the steering of the wheel
				link_distRBlat = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right rod
				link_distRBlat->Initialize(truss->GetBody(), spindleRB->GetBody(), false, ChVector<>(.5*vehicleWidth,1.21,-(0.5*vehicleLength-0.1)), ChVector<>(.5*vehicleWidth+wheelSep-.25,1.21,-(0.5*vehicleLength-0.2)));
				my_system.AddLink(link_distRBlat);


				// --- Left Back suspension --- 

				// ..the car right-back spindle
				spindleLB = (ChBodySceneNode*)addChBodySceneNode_easyBox(
														&my_system, msceneManager,
														80.0*SCALEMASS,
														ChVector<>(-(.5*vehicleWidth+wheelSep-.2), 1 , -(0.5*vehicleLength-0.5)),
														QUNIT, 
														ChVector<>(0.1, spindleSideDim, spindleSideDim) );
				spindleLB->GetBody()->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
				spindleLB->GetBody()->SetCollide(false);
				bodyTypes.push_back("spindle");

				// ..the car left-back wheel
				if(useSphericalTire)
				{
					wheelLB = (ChBodySceneNode*)addChBodySceneNode_easySphere(
															&my_system, msceneManager,
															30.0*SCALEMASS,
															ChVector<>(-(.5*vehicleWidth+wheelSep), 1 , -(0.5*vehicleLength-0.5)),
															tireRadius);
					wheelLB->GetBody()->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
					wheelLB->GetBody()->SetCollide(true);
					wheelLB->GetBody()->SetFriction(1.0);
					wheelLB->setMaterialTexture(0,	cylinderMap);
					wheelLB->addShadowVolumeSceneNode();
					bodyTypes.push_back("wheelLB");
				}
				else
				{
					wheelLB = (ChBodySceneNode*)addChBodySceneNode_easyCylinder(
															&my_system, msceneManager,
															30.0*SCALEMASS,
															ChVector<>(-(.5*vehicleWidth+wheelSep), 1 , -(0.5*vehicleLength-0.5)),
															chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Z), 
															ChVector<>(tireRadius*2, 0.3, tireRadius*2) );
					wheelLB->GetBody()->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
					wheelLB->GetBody()->SetCollide(true);
					wheelLB->GetBody()->SetFriction(1.0);
					wheelLB->setMaterialTexture(0,	cylinderMap);
					wheelLB->addShadowVolumeSceneNode();
					bodyTypes.push_back("wheelLB");
				}

				// .. create the revolute joint between the wheel and the spindle
				link_revoluteLB = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute); // left, back, upper, 1
				link_revoluteLB->Initialize(wheelLB->GetBody(), spindleLB->GetBody(), 
					ChCoordsys<>(ChVector<>(-(.5*vehicleWidth+wheelSep), 1, -(0.5*vehicleLength-0.5)) , chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Y)) );
				my_system.AddLink(link_revoluteLB);

				// .. create the motor transmission joint between the wheel and the truss (assuming small changes of alignment)
				link_engineL = ChSharedPtr<ChLinkEngine>(new ChLinkEngine); 
				link_engineL->Initialize(wheelLB->GetBody(), truss->GetBody(), 
					ChCoordsys<>(ChVector<>(-(.5*vehicleWidth+wheelSep), 1, -(0.5*vehicleLength-0.5)) , chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Y)) );
				link_engineL->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_CARDANO); // approx as a double Rzeppa joint
				link_engineL->Set_eng_mode(ChLinkEngine::ENG_MODE_TORQUE);
				my_system.AddLink(link_engineL);

				// .. impose distance between two parts (as a massless rod with two spherical joints at the end)
				link_distLBU1 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // left, front, upper, 1
				link_distLBU1->Initialize(truss->GetBody(), spindleLB->GetBody(), false, ChVector<>(-(.5*vehicleWidth),1.2,-(0.5*vehicleLength-0.3)), ChVector<>(-(.5*vehicleWidth+wheelSep-.25),1.2,-(0.5*vehicleLength-0.5)));
				my_system.AddLink(link_distLBU1);
			 
				link_distLBU2 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // left, back, upper, 2
				link_distLBU2->Initialize(truss->GetBody(), spindleLB->GetBody(), false, ChVector<>(-(.5*vehicleWidth),1.2,-(0.5*vehicleLength-0.7)), ChVector<>(-(.5*vehicleWidth+wheelSep-.25),1.2,-(0.5*vehicleLength-0.5)));
				my_system.AddLink(link_distLBU2);

				link_distLBL1 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // left, back, lower, 1
				link_distLBL1->Initialize(truss->GetBody(), spindleLB->GetBody(), false, ChVector<>(-(.5*vehicleWidth),0.8,-(0.5*vehicleLength-0.3)), ChVector<>(-(.5*vehicleWidth+wheelSep-.25),0.8,-(0.5*vehicleLength-0.5)));
				my_system.AddLink(link_distLBL1);

				link_distLBL2 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // left, back, lower, 2
				link_distLBL2->Initialize(truss->GetBody(), spindleLB->GetBody(), false, ChVector<>(-(.5*vehicleWidth),0.8,-(0.5*vehicleLength-0.7)), ChVector<>(-(.5*vehicleWidth+wheelSep-.25),0.8,-(0.5*vehicleLength-0.5)));
				my_system.AddLink(link_distLBL2);
				
				// .. create the spring between the truss and the spindle
				link_springLB = ChSharedPtr<ChLinkSpring>(new ChLinkSpring);
				link_springLB->Initialize(truss->GetBody(), spindleLB->GetBody(), false, ChVector<>(-(.5*vehicleWidth),1.2,-(0.5*vehicleLength-0.5)), ChVector<>(-(.5*vehicleWidth+wheelSep-.25),0.8,-(0.5*vehicleLength-0.5)));
				link_springLB->Set_SpringK(283000*SCALEMASS/SCALELENGTH);
				link_springLB->Set_SpringR(800*SCALEMASS/SCALELENGTH);
				my_system.AddLink(link_springLB);

				// .. create the rod for avoid the steering of the wheel
				link_distLBlat = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right 
				link_distLBlat->Initialize(truss->GetBody(), spindleLB->GetBody(), false, ChVector<>(-(.5*vehicleWidth),1.21,-(0.5*vehicleLength-0.1)), ChVector<>(-(.5*vehicleWidth+wheelSep-.25),1.21,-(0.5*vehicleLength-0.2)));
				my_system.AddLink(link_distLBlat);

			}

		// Delete the car object, deleting also all bodies corresponding to
		// the various parts and removing them from the physical system.  Also
		// removes constraints from the system.
	~MySimpleCar()
			{
				ChSystem* mysystem = spindleRF->GetBody()->GetSystem(); // trick to get the system here				
					// When a ChBodySceneNode is removed via ->remove() from Irrlicht 3D scene manager,
					// it is also automatically removed from the ChSystem (the ChSystem::RemoveBody() is
					// automatically called at Irrlicht node deletion - see ChBodySceneNode.h ).

					// For links, just remove them from the ChSystem using ChSystem::RemoveLink()
				mysystem->RemoveLink(link_revoluteRF);				
				mysystem->RemoveLink(link_distRFU1);
				mysystem->RemoveLink(link_distRFU2);
				mysystem->RemoveLink(link_distRFL1);
				mysystem->RemoveLink(link_distRFL2);
				mysystem->RemoveLink(link_springRF);
				mysystem->RemoveLink(link_distRSTEER);

				mysystem->RemoveLink(link_revoluteLF);				
				mysystem->RemoveLink(link_distLFU1);
				mysystem->RemoveLink(link_distLFU2);
				mysystem->RemoveLink(link_distLFL1);
				mysystem->RemoveLink(link_distLFL2);
				mysystem->RemoveLink(link_springLF);
				mysystem->RemoveLink(link_distLSTEER);

				mysystem->RemoveLink(link_revoluteRB);				
				mysystem->RemoveLink(link_distRBU1);
				mysystem->RemoveLink(link_distRBU2);
				mysystem->RemoveLink(link_distRBL1);
				mysystem->RemoveLink(link_distRBL2);
				mysystem->RemoveLink(link_springRB);
				mysystem->RemoveLink(link_distRBlat);
				mysystem->RemoveLink(link_engineR);

				mysystem->RemoveLink(link_revoluteLB);				
				mysystem->RemoveLink(link_distLBU1);
				mysystem->RemoveLink(link_distLBU2);
				mysystem->RemoveLink(link_distLBL1);
				mysystem->RemoveLink(link_distLBL2);
				mysystem->RemoveLink(link_springLB);
				mysystem->RemoveLink(link_distLBlat);
				mysystem->RemoveLink(link_engineL);

				truss->remove();
				spindleRF->remove();
				wheelRF->remove();
				spindleLF->remove();
				wheelLF->remove();
				spindleRB->remove();
				wheelRB->remove();
				spindleLB->remove();
				wheelLB->remove();
			}

		// This can be used, at each time step, to compute the actual value of torque
		// transmitted to the wheels, according to gas throttle / speed / gear value.
		// The following is a very simplified model (the torque curve of the motor is linear
		// and no latency or inertial or clutch effects in gear train are considered.)
	double ComputeWheelTorque()
			{
				// Assume clutch is never used. Given the kinematics of differential,
				// the speed of the engine transmission shaft is the average of the two wheel speeds,
				// multiplied the conic gear transmission ratio inversed:
				double shaftspeed = (1.0/this->conic_tau) * 0.5 *
					(this->link_engineL->Get_mot_rot_dt()+this->link_engineR->Get_mot_rot_dt());
				// The motorspeed is the shaft speed multiplied by gear ratio inversed:
				double motorspeed = (1.0/this->gear_tau)*shaftspeed;
				// The torque depends on speed-torque curve of the motor: here we assume a 
				// very simplified model a bit like in DC motors:
				double motortorque = max_motor_torque - motorspeed*(max_motor_torque/max_motor_speed) ;
				// Motor torque is linearly modulated by throttle gas value:
				motortorque = motortorque *  this->throttle;
				// The torque at motor shaft:
				double shafttorque =  motortorque * (1.0/this->gear_tau);
				// The torque at wheels - for each wheel, given the differential transmission, 
				// it is half of the shaft torque  (multiplied the conic gear transmission ratio)
				double singlewheeltorque = 0.5 * shafttorque * (1.0/this->conic_tau);
				// Set the wheel torque in both 'engine' links, connecting the wheels to the truss;
				if (ChFunction_Const* mfun = dynamic_cast<ChFunction_Const*>(this->link_engineL->Get_tor_funct()))
					mfun->Set_yconst(singlewheeltorque);
				if (ChFunction_Const* mfun = dynamic_cast<ChFunction_Const*>(this->link_engineR->Get_tor_funct()))
					mfun->Set_yconst(singlewheeltorque);
				//debug:print infos on screen:
				   //GetLog() << "motor torque="<< motortorque<< "  speed=" << motorspeed << "  wheel torqe=" << singlewheeltorque <<"\n";
				// If needed, return also the value of wheel torque:
				return singlewheeltorque;
			}
};



// Define a MyEventReceiver class which will be used to manage input
// from the GUI graphical user interface (the interface will
// be created with the basic -yet flexible- platform
// independent toolset of Irrlicht).

class MyEventReceiver : public IEventReceiver
{
public:

	MyEventReceiver(ChSystem* asystem,  
					IrrlichtDevice *adevice,
					MySimpleCar* acar)
			{
				// store pointer to physical system & other stuff so we can tweak them by user keyboard
				msystem = asystem;
				mdevice = adevice;
				mcar    = acar;

				adevice->setEventReceiver(this);

				// ..add a GUI slider to control gas throttle via mouse
				scrollbar_throttle = mdevice->getGUIEnvironment()->addScrollBar(
								true, rect<s32>(10, 85, 150, 100), 0, 100);
				scrollbar_throttle->setMax(100); 
				scrollbar_throttle->setPos(0);
				text_throttle = mdevice->getGUIEnvironment()->addStaticText(
							L"Throttle", rect<s32>(150,85,250,100), false);

				// ..add a GUI slider to control steering via mouse
				scrollbar_steer = mdevice->getGUIEnvironment()->addScrollBar(
								true, rect<s32>(10, 105, 150, 120), 0, 101);
				scrollbar_steer->setMax(100); 
				scrollbar_steer->setPos(50);

				// ..add a GUI text and GUI slider to control the stiffness
				scrollbar_FspringK = mdevice->getGUIEnvironment()->addScrollBar(
								true, rect<s32>(10, 125, 150, 140), 0, 102);
				scrollbar_FspringK->setMax(100); 
				scrollbar_FspringK->setPos(50 + 50.0*(acar->link_springRF->Get_SpringK()-80000.0)/60000.0  );
				text_FspringK = mdevice->getGUIEnvironment()->addStaticText(
								L"Spring K [N/m]:", rect<s32>(150,125,250,140), false);

				// ..add a GUI text and GUI slider to control the damping
				scrollbar_FdamperR = mdevice->getGUIEnvironment()->addScrollBar(
								true, rect<s32>(10, 145, 150, 160), 0, 103);
				scrollbar_FdamperR->setMax(100); 
				scrollbar_FdamperR->setPos(50 + 50.0*(acar->link_springRF->Get_SpringR()-800.0)/800.0  );
				text_FdamperR = mdevice->getGUIEnvironment()->addStaticText(
								L"Damper R [Ns/m]:", rect<s32>(150,145,250,160), false);

				// ..add a GUI text and GUI slider to control the original undeformed spring length
				scrollbar_FspringL = mdevice->getGUIEnvironment()->addScrollBar(
								true, rect<s32>(10, 165, 150, 180), 0, 104);
				scrollbar_FspringL->setMax(100); 
				scrollbar_FspringL->setPos(50 + 50.0*(acar->link_springRF->Get_SpringRestLenght()-0.9)/0.1  );
				text_FspringL = mdevice->getGUIEnvironment()->addStaticText(
								L"Spring L [m]:", rect<s32>(150,165,250,180), false);
			}

	bool OnEvent(const SEvent& event)
			{

				// check if user moved the sliders with mouse..
				if (event.EventType == EET_GUI_EVENT)
				{
					s32 id = event.GUIEvent.Caller->getID();
					IGUIEnvironment* env = mdevice->getGUIEnvironment();

					switch(event.GUIEvent.EventType)
					{
					case EGET_SCROLL_BAR_CHANGED:
							if (id == 101) // id of 'steer' slider..
							{
								s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
								double newsteer = mcar->wheelSep*0.18*( ((double)(pos-50))/50.0 );
								// set the steering, moving horizontally the endpoints of the steer rod endpoint on truss.

								this->mcar->link_distRSTEER->SetEndPoint1Rel(ChVector<>( .5*mcar->vehicleWidth+newsteer,0.21,0.5*mcar->vehicleLength-0.1));
								this->mcar->link_distLSTEER->SetEndPoint1Rel(ChVector<>(-(.5*mcar->vehicleWidth)+newsteer,0.21,0.5*mcar->vehicleLength-0.1));
							}
							if (id == 102) // id of 'spring stiffness' slider..
							{
								s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
								double newstiff = 80000 + 60000*( ((double)(pos-50))/50.0 );
								// set the stiffness of all 4 springs
								//this->mcar->link_springRF->Set_SpringK(newstiff);
								//this->mcar->link_springLF->Set_SpringK(newstiff);
								//this->mcar->link_springRB->Set_SpringK(newstiff);
								//this->mcar->link_springLB->Set_SpringK(newstiff);

								// show stiffness as formatted text in interface screen
								char message[50]; sprintf(message,"Spring K [N/m]: %g",newstiff);
								text_FspringK->setText(core::stringw(message).c_str());
							}
							if (id == 103) // id of 'damping' slider..
							{
								s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
								double newdamping = 800 + 800*( ((double)(pos-50))/50.0 );
								// set the damping of all 4 springs
								//this->mcar->link_springRF->Set_SpringR(newdamping);
								//this->mcar->link_springLF->Set_SpringR(newdamping);
								//this->mcar->link_springRB->Set_SpringR(newdamping);
								//this->mcar->link_springLB->Set_SpringR(newdamping);

								// show stiffness as formatted text in interface screen
								char message[50]; sprintf(message,"Damping R [Ns/m]: %g",newdamping);
								text_FdamperR->setText(core::stringw(message).c_str());
							}
							if (id == 104) // id of 'spring rest length' slider..
							{
								s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
								double newlength = 0.9 + 0.1*( ((double)(pos-50))/50.0 );
								// set the rest length of all 4 springs
								//this->mcar->link_springRF->Set_SpringRestLenght(newlength);
								//this->mcar->link_springLF->Set_SpringRestLenght(newlength);
								//this->mcar->link_springRB->Set_SpringRestLenght(newlength);
								//this->mcar->link_springLB->Set_SpringRestLenght(newlength);

								// show stiffness as formatted text in interface screen
								char message[50]; sprintf(message,"Spring L [m]: %g",newlength);
								text_FspringL->setText(core::stringw(message).c_str());
							}
							if (id == 100) // id of 'throttle' slider..
							{
								s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
								double newthrottle =  ((double)(pos))/100.0 ;
								// Set the throttle value of car (the torque transmitted
								// to wheels depends on throttle, speed, transmission gear, so 
								// it will sent to the link_engineR and link_engineL only when
								// computed by MySimplifiedCar::ComputeWheelTorque(),
								this->mcar->throttle=newthrottle;
							}
					break;
					}
					
				} 

				return false;
			}

private:
	ChSystem*       msystem;
	IrrlichtDevice* mdevice;
	MySimpleCar*    mcar;

	IGUIScrollBar*  scrollbar_steer;
	IGUIStaticText* text_FspringK;
	IGUIScrollBar*  scrollbar_FspringK;
	IGUIStaticText* text_FdamperR;
	IGUIScrollBar*  scrollbar_FdamperR;
	IGUIStaticText* text_FspringL;
	IGUIScrollBar*  scrollbar_FspringL;
	IGUIStaticText* text_throttle;
	IGUIScrollBar*  scrollbar_throttle;
};

int addSlopedRockyTerrain(ChSystem&  my_system,	///< the chrono::engine physical system 
				ISceneManager* msceneManager, ///< the Irrlicht scene manager for 3d shapes
				IVideoDriver* mdriver,	///< the Irrlicht video driver
				chrono::ChVector<> cornerPos1,
				chrono::ChVector<> cornerPos2,
				double bedThickness,
				double bedRockRadius,
				double tumbleRockRadius
				)
{
	// create base of hill
	double bedLength = sqrt(pow((cornerPos2.z-cornerPos1.z),2)+pow((cornerPos2.y-cornerPos1.y),2));
	double bedWidth = abs(cornerPos2.x-cornerPos1.x);
	double slope = -tan((cornerPos2.y-cornerPos1.y)/(cornerPos2.z-cornerPos1.z));
	ChBodySceneNode* terrainBed = (ChBodySceneNode*)addChBodySceneNode_easyBox(&my_system, msceneManager,
											1.0,
											ChVector<>(0.5*(cornerPos2.x+cornerPos1.x),0.5*(cornerPos2.y+cornerPos1.y),0.5*(cornerPos2.z+cornerPos1.z)),
											chrono::Q_from_AngAxis(slope, VECT_X),
											ChVector<>(bedWidth,bedThickness,bedLength) );	
	terrainBed->GetBody()->SetBodyFixed(true);
	terrainBed->GetBody()->SetSfriction(1.0);
	terrainBed->GetBody()->SetKfriction(1.0);
	bodyTypes.push_back("terrain");

	// create large bed rocks on base
	ChBodySceneNode* bedRock;
	ChBodySceneNode* tumbleRock;
	int numRocksAlongLength = bedLength/(1.6*bedRockRadius);
	int numRocksAlongWidth = bedWidth/(1.6*bedRockRadius);
	int numRocks = 0;
	char filename[100];

	double density = 2800*SCALEMASS; // everything is 10 times less mass in this sim
	double volume = 4*CH_C_PI*tumbleRockRadius*tumbleRockRadius*tumbleRockRadius/3.0;
	double mass = density*volume;

	for(int i=0;i<numRocksAlongWidth;i++)
	{
		for(int j=0;j<numRocksAlongLength;j++)
		{
			generateRockObject(numRocks,bedRockRadius*getRandomNumber(.8,1.2));
			sprintf(filename, "../data/humvee/rocks/rock%d.obj", numRocks);
			bedRock = (ChBodySceneNode*)addChBodySceneNode_easyGenericMesh(
												&my_system, msceneManager,
												3.0, 
												cornerPos1+ChVector<>(-1.6*i*bedRockRadius,-1.6*j*bedRockRadius*atan(slope),1.6*j*bedRockRadius),
												QUNIT, 
												filename, 
												false,	// not static 
												false);	// true=convex; false=concave(do convex decomposition of concave mesh)
			bedRock->GetBody()->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
			bedRock->GetBody()->SetCollide(true);
			bedRock->GetBody()->SetBodyFixed(true);
			bedRock->GetBody()->SetFriction(1.0);
			bedRock->addShadowVolumeSceneNode();
			bodyTypes.push_back("rock");
			//bedRock->GetBody()->

			numRocks++;

			generateRockObject(numRocks,tumbleRockRadius*getRandomNumber(.8,1.2));
			sprintf(filename, "../data/humvee/rocks/rock%d.obj", numRocks);
			tumbleRock = (ChBodySceneNode*)addChBodySceneNode_easyGenericMesh(
												&my_system, msceneManager,
												mass, 
												cornerPos1+ChVector<>(-1.6*i*bedRockRadius,cornerPos2.y*1.2,1.6*j*bedRockRadius)+ChVector<>(bedRockRadius,0,bedRockRadius),
												QUNIT, 
												filename, 
												false,	// not static 
												false);	// true=convex; false=concave(do convex decomposition of concave mesh)
			tumbleRock->GetBody()->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
			tumbleRock->GetBody()->SetCollide(true);
			//tumbleRock->GetBody()->SetBodyFixed(true);
			tumbleRock->GetBody()->SetFriction(1.0);
			tumbleRock->addShadowVolumeSceneNode();
			bodyTypes.push_back("rock");

			numRocks++;
		}
	}
	return numRocks;
}

void createWall(ChSystem& mphysicalSystem, ISceneManager* msceneManager, IVideoDriver* driver)
{
	ChBodySceneNode* mrigidBody; 

	ChSharedPtr<ChMaterialSurface> mmaterial(new ChMaterialSurface);
	mmaterial->SetFriction(0.4f);
	mmaterial->SetCompliance (0.00005f);
	mmaterial->SetComplianceT(0.00005f);
	mmaterial->SetDampingF(0.2);

	// Create a bunch of ChronoENGINE rigid bodies (spheres and
	// boxes) which will fall..
	// Bodies are Irrlicht nodes of the special class ChBodySceneNode, 
	// which encapsulates ChBody items).  
	
	video::ITexture* cubeMap   = driver->getTexture("../data/cubetexture_borders.png");
	video::ITexture* sphereMap = driver->getTexture("../data/bluwhite.png");

	double pos_x=0;
	double pos_y=0;
	double pos_z=0;
	double rot_0=0;
	double rot_1=0;
	double rot_2=0;
	double rot_3=0;
	std::string temp_data;
	ifstream ifile("C:/Users/Daniel/Desktop/BrickWall/posData_M113/pos0.txt");

	double scale = .2;
	for (int i = 0; i < 150; i++)  // N. of bricks
	{ 
		getline(ifile,temp_data);
		std::stringstream ss(temp_data);
		ss>>pos_x>>pos_y>>pos_z>>rot_0>>rot_1>>rot_2>>rot_3;
		ChVector<> particle_pos(pos_x,pos_y,pos_z);
		ChQuaternion<> particle_rot(rot_0,rot_1,rot_2,rot_3);

		mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
			&mphysicalSystem, msceneManager,
			10,
			particle_pos*scale,
			particle_rot, 
			ChVector<>(3.96*scale,2*scale,4*scale) );
		//mrigidBody->GetBody()->SetMaterialSurface(mmaterial);
		mrigidBody->setMaterialTexture(0,	cubeMap);
		mrigidBody->addShadowVolumeSceneNode();
		bodyTypes.push_back("brick");
	}

	// Some aesthetics for 3d view..
	mrigidBody->addShadowVolumeSceneNode();
	mrigidBody->setMaterialTexture(0,	sphereMap);
}

//
// This is the program which is executed
//

int main(int argc, char* argv[])
{

	// In CHRONO engine, The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
	// global functions are needed.
	DLL_CreateGlobals();

	// Create the IRRLICHT context (device, etc.)
	IrrlichtDevice* device = createDevice(video::EDT_DIRECT3D9, 
							core::dimension2d<u32>(1680, 1050),	// resolution
							64,									// 32 bit depth 
							false,								// full screen
							false);								// do shadows (might be slow on old PC!)
	//if (device == 0)
	//{
		GetLog() << "Cannot use DirectX - switch to OpenGL \n"; 
		device = createDevice(video::EDT_OPENGL, core::dimension2d<u32>(1680, 1050));
		if (!device) return 1;
	//}

	device->setWindowCaption(L"Modeling a simplified suspension of a car, using a spring-damper");

	IVideoDriver* driver           = device->getVideoDriver();
	ISceneManager*	 msceneManager = device->getSceneManager();
	IGUIEnvironment* guienv        = device->getGUIEnvironment();

 
	// Easy shortcuts to add logo, camera, lights and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo(device);
	ChIrrWizard::add_typical_Sky(device);
	ChIrrWizard::add_typical_Lights(device);
	ChIrrWizard::add_typical_Camera(device, core::vector3df(0,0,-6));
	ICameraSceneNode* camera = msceneManager->getActiveCamera();
	camera->setPosition(core::vector3df(0,0,-6));


    // 
	// HERE YOU CREATE THE MECHANICAL SYSTEM OF CHRONO...
	// 


	// 1- Create a ChronoENGINE physical system: all bodies and constraints
	//    will be handled by this ChSystem object.
	ChSystem my_system;
 
	// 2- Create the rigid bodies of the simpified car suspension mechanical system
	//   maybe setting position/mass/inertias of
	//   their center of mass (COG) etc.
	
	// ..the world
	my_system.Set_G_acc(ChVector<>(0,-9.81/SCALELENGTH,0));
	ChBodySceneNode* my_ground = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&my_system, msceneManager,
											1.0,
											ChVector<>(0,-1,0),
											QUNIT, 
											ChVector<>(200,2,200) );
	my_ground->GetBody()->SetBodyFixed(true);
	my_ground->GetBody()->SetCollide(true);
	my_ground->GetBody()->SetSfriction(1.0);
	my_ground->GetBody()->SetKfriction(1.0);
	video::ITexture* groundMap = driver->getTexture("../data/blu.png");
	my_ground->setMaterialTexture(0,groundMap);
	bodyTypes.push_back("ground");

	//// ..some obstacles on the ground:
	//for (int i=0; i<6; i++)
	//{
	//	ChBodySceneNode* my_obstacle = (ChBodySceneNode*)addChBodySceneNode_easyBox(
	//										&my_system, msceneManager,
	//										3.0,
	//										ChVector<>(20*CHrandom(),2, 20*CHrandom()),
	//										QUNIT,
	//										ChVector<>(1,0.08,0.5) );
	//}
 
	// ..the car (this class - see above - is a 'set' of bodies and links, automatically added at creation)
	MySimpleCar* mycar = new MySimpleCar(my_system, msceneManager, driver);

	ChVector<> cornerPos1 = ChVector<>(10,0,10);
	ChVector<> cornerPos2 = ChVector<>(-10,6,30);
	if(argc==2) cornerPos2 = ChVector<>(-10,6+atof(argv[1]),30);
	int numRocks = 0;
	int slope = tan((cornerPos2.y-cornerPos1.y)/(cornerPos2.z-cornerPos1.z))*180/CH_C_PI;

	//// ADD SLOPED, ROCKY TERRAIN
	//numRocks = addSlopedRockyTerrain(my_system, msceneManager, driver,
	//			cornerPos1,
	//			cornerPos2,
	//			0.1, 0.2, 0.07//0.1, 2, 0.5 //0.1, 0.3, 0.1
	//			);
	//cout << numRocks << endl;
	//// END ADD SLOPED, ROCKY TERRAIN

	//char filename[100];
	//generateRockObject(numRocks,3*getRandomNumber(.8,1.2));
	//sprintf(filename, "../data/humvee/rocks/rock%d.obj", numRocks);
	//ChBodySceneNode* bigRock = (ChBodySceneNode*)addChBodySceneNode_easyGenericMesh(
	//												&my_system, msceneManager,
	//												3.0, 
	//												ChVector<>(-8,0,15),
	//												QUNIT, 
	//												filename, 
	//												true,	// not static 
	//												false);	// true=convex; false=concave(do convex decomposition of concave mesh)
	//bigRock->GetBody()->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
	//bigRock->GetBody()->SetCollide(true);
	//bigRock->GetBody()->SetBodyFixed(true);
	//bigRock->GetBody()->SetFriction(1.0);
	//bigRock->addShadowVolumeSceneNode();
	//bodyTypes.push_back("rock");
	//cout << numRocks << endl;

	//
	// USER INTERFACE
	//
	 
	// Create some graphical-user-interface (GUI) items to show on the screen.
	// This requires an event receiver object -see above.
	MyEventReceiver receiver(&my_system, device, mycar);

	//
	// SETTINGS 
	// 	
	my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);//_MULTITHREAD);
	my_system.SetIterLCPmaxItersSpeed(40); // the higher, the easier to keep the constraints 'mounted'.
 
	//
	// THE SOFT-REAL-TIME CYCLE, SHOWING THE SIMULATION
	//

	// Save the body types
	//ChStreamOutAsciiFile bodyTypesStream("C:/Users/Daniel/Desktop/RockySlope/bodyTypes.txt");

	//for (int i=0;i<bodyTypes.size();i++)
	//{
	//	bodyTypesStream << bodyTypes[i] << ", \n";
	//}

	// This will help choosing an integration step which matches the
	// real-time step of the simulation..
	ChRealtimeStepTimer m_realtime_timer;

	int timeIndex = 0;
	int frameIndex = 0;
	chrono::Vector bodyAngs;
	chrono::Quaternion bodyRot;
	double time = 0;
	double hh = 0.001;

	char velocityFile[100];
	sprintf(velocityFile, "./velocityRockySlope_%d.txt", slope);
	ChStreamOutAsciiFile velocityStream(velocityFile);
	while(device->run()&&time<30)
	{ 
		ChVector<> pos = mycar->truss->GetBody()->GetPos();
		ChVector<> vel = mycar->truss->GetBody()->GetPos_dt();
		velocityStream << time << ", " << pos.x << ", " << pos.y << ", " << pos.z << ", " << vel.x << ", " << vel.y << ", " << vel.z << ", \n";
		cout << time << ", " << pos.x << ", " << pos.y << ", " << pos.z << ", " << vel.x << ", " << vel.y << ", " << vel.z << ", \n";

		//if(time>10) mycar->throttle=0.3;

		//if(timeIndex%2==0)
		//{
		//	char savename[100];
		//	sprintf(savename, "C:/Users/Daniel/Desktop/RockySlope/pos%d.dat", frameIndex);
		//	ChStreamOutAsciiFile saveStream(savename);

		//	std::vector<ChBody*>::iterator abody = my_system.Get_bodylist()->begin();
		//	while (abody != my_system.Get_bodylist()->end())
		//	{
		//		ChBody* bpointer = (*abody);
		//		bodyRot = bpointer->GetRot();
		//		bodyAngs = bodyRot.Q_to_NasaAngles();
		//		//bodyAngs = Quat_to_Angle(4,&bodyRot);

		//		saveStream << bpointer->GetPos().x << ", ";
		//		saveStream << bpointer->GetPos().y << ", ";
		//		saveStream << bpointer->GetPos().z << ", ";
		//		saveStream << bpointer->GetRot().e0 << ", ";
		//		saveStream << bpointer->GetRot().e1 << ", ";
		//		saveStream << bpointer->GetRot().e2 << ", ";
		//		saveStream << bpointer->GetRot().e3 << ", ";
		//		saveStream << bodyAngs.x << ", ";
		//		saveStream << bodyAngs.y << ", ";
		//		saveStream << bodyAngs.z << ", ";
		//		saveStream << "\n";
		//		abody++;
		//	}
		//	frameIndex++;
		//}

		// Irrlicht must prepare frame to draw
		driver->beginScene(true, true, SColor(255,140,161,192));
	
		// Irrlicht now draws simple lines in 3D world representing a 
		// skeleton of the mechanism, in this instant:
		//
		// .. draw solid 3D items (boxes, cylinders, shapes) belonging to Irrlicht scene, if any
		msceneManager->drawAll();

		ChIrrTools::drawAllContactPoints(my_system, driver);

		// .. draw a grid (rotated so that it's horizontal)
		ChIrrTools::drawGrid(driver, 1, 1, 100,100,
			ChCoordsys<>(ChVector<>(0,0.01,0), Q_from_AngX(CH_C_PI_2) ),
			video::SColor(0, 0,0,0), true);

		//for(int i=0;i<pointlist.size();i++) ChIrrTools::drawCircle(driver, 0.002, ChCoordsys<>( pointlist[i], QUNIT));

		// .. draw GUI user interface items (sliders, buttons) belonging to Irrlicht screen, if any
		guienv->drawAll();

		// .. draw the distance constraints (the massless rods) as simplified lines
		std::list<chrono::ChLink*>::iterator iterlink =  my_system.Get_linklist()->begin();
		while(iterlink !=  my_system.Get_linklist()->end())
		{
			if (ChLinkDistance* mylinkdis = ChDynamicCast(ChLinkDistance,(*iterlink)))
				ChIrrTools::drawSegment(driver, 
					mylinkdis->GetEndPoint1Abs(), 
					mylinkdis->GetEndPoint2Abs(),
					video::SColor(255,   0,20,0), true);
			iterlink++;
		}

		// .. draw the spring constraints as simplified spring helix
		iterlink =  my_system.Get_linklist()->begin();
		while(iterlink !=  my_system.Get_linklist()->end())
		{
			if (ChLinkSpring* mylinkspri = ChDynamicCast(ChLinkSpring,(*iterlink)))
				ChIrrTools::drawSpring(driver, 0.03, 
					mylinkspri->GetEndPoint1Abs(),
					mylinkspri->GetEndPoint2Abs(),
					video::SColor(255,   150,20,20),   80,  5,  true);
			iterlink++;
		}
		

		// The torque applied to wheels, using the ChLinkEngine links between 
		// wheels and truss, depends on many parameters (gear, throttle, etc):
		mycar->ComputeWheelTorque();

		// HERE CHRONO INTEGRATION IS PERFORMED: THE 
		// TIME OF THE SIMULATION ADVANCES FOR A SINGLE
		// STEP:
		
		my_system.DoStepDynamics(hh);//m_realtime_timer.SuggestSimulationStep(0.005) );

		// Irrlicht must finish drawing the frame
		driver->endScene(); 

		timeIndex++;
		time += hh;
	}

	if (mycar) delete mycar;	

	// This safely delete every Irrlicht item..
	device->drop();

	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}


