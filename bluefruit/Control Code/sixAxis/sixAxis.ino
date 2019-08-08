#include "libraries/geometry/Geometry.h"
#include "libraries/EEPROM/EEPROM.h"
#include "libraries/ramp/Ramp.h"
#include "Servo.h"
#include "config.h"
#include "sixAxis.h"

Servo gripper;
KinematicChain roboticArm;

//      theta,        alpha,        d,    a
Link l1(1.74533E-06, -1.570796327, 220, 0);
Link l2(-1.570796327, 0, 0, 220);
Link l3(-1.553343034, 1.570796327, 0, 0);
Link l4(0, -1.570796327, -220, 0);
Link l5(1.74533E-06, 1.570796327, 0, 0);
Link l6(3.141592654, 0, -165, 0);

void setup()
{
		Serial.begin(500000);

		roboticArm.addLink(l1);
		roboticArm.addLink(l2);
		roboticArm.addLink(l3);
		roboticArm.addLink(l4);
		roboticArm.addLink(l5);
		roboticArm.addLink(l6);
		gripper.attach(9);

		for (byte i = 0; i < 6; i++)
		{
				pinMode(pulPin[i], OUTPUT);
				pinMode(dirPin[i], OUTPUT);
				pinMode(limPin[i], INPUT);
				digitalWrite(limPin[i], HIGH);

				if (i < 2)
				{
						digitalWrite(enaPin[i], HIGH);
				}
				else
				{
						digitalWrite(enaPin[i], LOW);
				}
		}
		// roboticArm.initializePins(pulPin, dirPin, enaPin, limPin, degreesPerStep, microstepDivider);
		// roboticArm.setLimits(anglesMin, anglesMax, cartesianMin, cartesianMax);
		// roboticArm.setSpeed(speedCartesian);
		// roboticArm.initializeController(joystickDirection);

		roboticArm.manualControl();

		// Matrix<6> pos = getPositionFromMatrix(roboticArm.forwardKinematics());
		// Matrix<6> point = {0,0,10,0,0,0};
		// for (int i=0; i<6; i++) {
		//      Serial.println(pos(i),20);
		//      point(i) = pos(i) - point(i);
		// }
		//
		// roboticArm.moveToPosition(point);
		//
		// Matrix<6> pos2 = getPositionFromMatrix(roboticArm.forwardKinematics());
		// Matrix<6> point2 = {0,0,-20,0,0,0};
		// for (int i=0; i<6; i++) {
		//      Serial.println(pos2(i),20);
		//      point2(i) = pos2(i) - point2(i);
		// }
		//
		// roboticArm.moveToPosition(point2);
}

void loop()
{
}
