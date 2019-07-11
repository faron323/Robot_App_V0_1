#include "delta.h"

int pulPin[3] = {3,4,5};
int dirPin[3] = {23,25,27};
int enaPin[3] = {22,24,26};
int limPin[3] = {18,19,20};

float degreesPerStep[3] = {1.8, 1.8, 1.8};
float microstep[3] = {2,2,2};

// 90deg = 452.3

// // Delta Arm Constants (in m)
// float sideBase = 0.567; //base equilateral triangle side
// float sidePlatform = 0.076; //platform equilateral triangle side
// float upperLegLength = 0.524; //upper legs length
// float lowerLegLength = 1.244; //lower legs parallelogram length
// float lowerLegWidth = 131; //lower legs parallelogram width
//
// Delta Arm Constants (in m)
float sideBase = 0.260; //base equilateral triangle side
float sidePlatform = 0.130; //platform equilateral triangle side
float upperLegLength = 0.210; //upper legs length
float lowerLegLength = 0.540; //lower legs parallelogram length 3*150 + 95
float lowerLegWidth = 0.095; //lower legs parallelogram width

float angles[3] = {M_PI_2,M_PI_2,M_PI_2}; //initial angles (radians)

DeltaArm delta(&angles, sideBase, sidePlatform, upperLegLength, lowerLegLength, lowerLegWidth);


void setup() {
		Serial.begin(115200);

		delta.initialize(pulPin, dirPin, enaPin, limPin, degreesPerStep, microstep);
		// float anglesTemp[3] = {0*M_PI/180,0*M_PI/180,0*M_PI/180};
		// delta.equalZForwardKinematics(anglesTemp);
		//
		// float anglesTemp1[3] = {10*M_PI/180,10*M_PI/180,10*M_PI/180};
		// delta.equalZForwardKinematics(anglesTemp1);

		delta.moveToPosition(0.3,0.4,-0.49749);
		delta.moveToPosition(0.3,0.4,-0.49749);
		// delay(5000);
		//
		// delta.moveToPosition(-0.5,0,-0.49749);
		// delay(2000);
		//
		// delta.moveToPosition(0,0,-0.49749);
		// delay(2000);
		//
		// delta.moveToPosition(0,-0.5,-0.49749);
		// delay(2000);
		//
		// delta.moveToPosition(0,0,-0.49749);
		// delay(2000);
}

void loop() {
}
