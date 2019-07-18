#include "delta.h"

const int pulPin[3] = {3,4,5};
const int dirPin[3] = {23,25,27};
const int enaPin[3] = {22,24,26};
const int limPin[3] = {18,19,20};

const float degreesPerStep[3] = {1.8, 1.8, 1.8};
const float microstep[3] = {4,4,4};

// Delta Arm Constants (in metres)
const float sideBase = 0.260; //base equilateral triangle side
const float sidePlatform = 0.130; //platform equilateral triangle side
const float upperLegLength = 0.210; //upper legs length
const float lowerLegLength = 0.540; //lower legs parallelogram length
const float lowerLegWidth = 0.095; //lower legs parallelogram width

float angles[3] = {M_PI_2,M_PI_2,M_PI_2}; //initial angles (radians)

void setup() {
		Serial.begin(115200);

		DeltaArm delta(&angles, sideBase, sidePlatform, upperLegLength, lowerLegLength, lowerLegWidth);

		delta.initialize(pulPin, dirPin, enaPin, limPin, degreesPerStep, microstep);

		// float anglesTemp[3] = {90*M_PI/180,90*M_PI/180,110*M_PI/180};
		// float x,y,z;
		// delta.forwardKinematics(anglesTemp, &x, &y, &z);

		// Serial.println("NEW FK POS");
		// Serial.print("x: ");
		// Serial.println(x,5);
		// Serial.print("y: ");
		// Serial.println(y,5);
		// Serial.print("z: ");
		// Serial.println(z,5);

		// delta.calculateVelocity(angles);


		// float angles[3] = {45.6743, 24.5237, 13.43457};
		// float speed[3] = {2000, 2000, 2000};
		// delta.moveToAngles(angles, speed);

		delta.moveToPosition(0,0,-0.49749);
		delay(2000);
		delta.moveToPosition(0.1,0.1,-0.49749);
		// delay(500);
		delta.moveToPosition(0.1,-0.1,-0.49749);
		// delay(500);
		delta.moveToPosition(-0.1,-0.1,-0.49749);
		// delay(500);
		delta.moveToPosition(-0.1,0.1,-0.49749);
		// delay(500);
		delta.moveToPosition(0.1,0.1,-0.49749);
		// delay(500);
		delta.moveToPosition(0.1,-0.1,-0.49749);
		// delay(500);
		delta.moveToPosition(-0.1,-0.1,-0.49749);
		// delay(500);
		delta.moveToPosition(-0.1,0.1,-0.49749);
		// delay(500);
		delta.moveToPosition(0.1,0.1,-0.49749);
		// delay(500);
		delta.moveToPosition(0,0,-0.49749);
}

void loop() {
}
