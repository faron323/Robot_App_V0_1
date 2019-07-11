// 6-Axis Robotic Arm Control Code
// 2019 Phillip Ilievski

#include "libraries/geometry/Geometry.h"
#include "libraries/EEPROM/EEPROM.h"
#include "libraries/ramp/Ramp.h"
#include "Servo.h"

const byte numMotors = 5;

Servo gripper;

const int pulPin[] = {3, 4,  5,  6,  7};
const int dirPin[] = {23, 25, 27, 29, 31};
const int enaPin[] = {22, 24, 26, 28, 30};
const int limPin[] = {18, 19, 20, 21, 2};

// Motor Constants
const float degreesPerStep[numMotors] = {0.08823529, 0.27, 0.072, 1.8, 0.38297872};
const int microstepDivider[numMotors] = {8, 8, 16, 16, 16};

// Invert joystick directions
const bool joystickDirection[5] = {0, 1, 1, 0, 0};

// Constraints
const float cartesianMax[numMotors] = {500, 500, 824.941, 500, 500};
const float cartesianMin[numMotors] = {-500, -500, -500, -500, -500};
const int anglesMax[] = {20, 90, 90, 90, 90, 90};
const int anglesMin[] = {-90, -90, -90, -90, -90, -90};
const float anglesCalibrate[6] = {0.0011155378, -90.0000686645, 1.0001525878, -14.3684701919, 0.0279764533, 6.8100128173};

// Kinematic Variables
float anglesDifference[6] = {};
float speedCartesian[6] = {1, 1, 1, 1, 1, 1};
float cartGlobal[6] = {6.719464414, 1.17277E-05, 824.941, -90, 1.0001, -90.0001};
//Calibration for kinematics to be right at initial position
float anglesPrevious[6] = {0.0011155378, -90.0000686645, 1.0001525878, -14.3684701919, 0.0279764533, 6.8100128173};

/*
   Link
   Class containing all link Denavit-Hartenberg parameters.

   Each joint/motor in the arm is classed as a link
 */
class Link
{
public:
float theta, alpha, d, a;

Link(float _theta, float _alpha, float _d, float _a)
		: theta(_theta), alpha(_alpha), d(_d), a(_a)
{
}

// calculates the transformation matrix for the link
Matrix<4,4> getTransformationMatrix()
{
		float cos0 = cos(theta);
		float sin0 = sin(theta);
		float cosa = cos(alpha);
		float sina = sin(alpha);

		Matrix<4,4> matrix = {cos0, -sin0 * cosa, sin0 * sina, a * cos0,
				              sin0, cos0 * cosa, -cos0 * sina, a * sin0,
				              0, sina, cosa, d,
				              0, 0, 0, 1};

		return matrix;
}
};

/*
   KinematicChain
   Class containing all links in the kinematic chain, this preforms all of the
   kinematics and joint move functionality.

   must be initialized with max number of links.
   add links to chain using addLink function.
 */
template <int maxLinks>
class KinematicChain
{
unsigned int _numLinks = 0;

public:
Link *chain[maxLinks];

// Adds a new Link object to the chain.
void addLink(Link &l)
{
		if (_numLinks == maxLinks)
				return;

		chain[_numLinks++] = &l;
}

// Returns the number of links in the chain.
int numLinks()
{
		return _numLinks;
}

/*
   forwardKinematics
   Calculate the Forward Kinematics for the KinematicChain object.
   Joint angles are inherited from each links theta variable.

   Parameters:
      Depth : link depth of calculation (default: maxLinks)
      (i.e.depth of 2 returns the chain transformation matrix at link 2)

      Returns: Transformation matrix at the specified link depth (Matrix<4,4>)
 */
Matrix<4, 4> forwardKinematics(int depth = 6)
{
		Matrix<4, 4> tempMatrix,matrix = {};

		matrix.Fill(0);
		matrix(0, 0) = 1;
		matrix(1, 1) = 1;
		matrix(2, 2) = 1;
		matrix(3, 3) = 1;

		for (byte i = 0; i < depth; i++)
		{
				matrix *= chain[i]->getTransformationMatrix();
		}

		return matrix;
}

/*
   inverseKinematics
   Calculate the Inverse Kinematics for the KinematicChain object.

   Parameters:
      target: The target array {X,Y,Z,Yaw,Pitch,Roll} of the move

      Returns: The joint rotations for each joint in the chain required to reach
               the target, called within moveToPosition function.

   Problems:
      -Works correctly for first move but any further moves return incorrect
       calculations.
      -Angles being incorrectly stored.
      -Miscalculations due to arduino LUT in rot_0_6 possibly produces errors
       in wrist rotations
 */
Matrix<6> inverseKinematics(float target[6])
{
		Matrix<6> theta = {}, returnMatrix;
		Matrix<4, 4> remove_rot_0_6, rot_0_3, rot_0_6;
		Matrix<3, 3> rot_0_5_3;

		for (byte i = 0; i < 3; i++)
		{
				target[i + 3] = target[i + 3] * M_PI / 180; // Convert degrees to radians
		}

		rot_0_6 = getDHMatrix(target);
		float alpha5 = chain[5]->alpha;
		remove_rot_0_6 = {cos(M_PI), sin(M_PI), 0, 0,
				          -sin(M_PI) * cos(alpha5), cos(M_PI) * cos(alpha5), sin(alpha5), 0,
				          sin(M_PI) * sin(alpha5), -cos(M_PI) * sin(alpha5), cos(alpha5), -chain[5]->d,
				          0, 0, 0, 1};
		Matrix<4, 4> rot_0_5 = rot_0_6 * remove_rot_0_6;

		for (byte j = 0; j < 2; j++)
		{
				rot_0_3 = forwardKinematics(3);
				Matrix<3, 3> rot_0_3_transpose = ~rot_0_3.Submatrix(Slice<0, 3>(), Slice<0, 3>());
				Matrix<3, 3> rot_3_6 = rot_0_5.Submatrix(Slice<0, 3>(), Slice<0, 3>()) * rot_0_3_transpose;

				if (target[0] > 0)
				{
						if (target[1] > 0)
						{
								theta(0) = atan(rot_0_5(1, 3) / rot_0_5(0, 3)) * 180 / M_PI;
						}
						else
						{
								theta(0) = atan(rot_0_5(1, 3) / rot_0_5(0, 3)) * 180 / M_PI;
						}
				}
				else
				{
						if (target[1] > 0)
						{
								theta(0) = atan(rot_0_5(1, 3) / rot_0_5(0, 3)) * 180 / M_PI + 180;
						}
						else
						{
								theta(0) = atan(rot_0_5(1, 3) / rot_0_5(0, 3)) * 180 / M_PI - 180;
						}
				}

				float pX = sqrt(sq(abs(rot_0_5(1, 3))) + sq(abs(rot_0_5(0, 3))));
				float pY = rot_0_5(2, 3) - chain[0]->d;
				float pX_a1 = pX - chain[0]->a;
				float paH = sqrt(sq(pY) + sq(pX_a1));
				float A_deg = atan(pY / pX_a1) * 180 / M_PI;
				float B_deg = acos((sq(chain[1]->a) + sq(paH) - sq(chain[3]->d)) / (2 * chain[1]->a * paH)) * 180 / M_PI;
				float C_deg = 180 - acos((sq(chain[3]->d) + sq(chain[1]->a) - sq(paH)) / (2 * abs(chain[3]->d) * chain[1]->a)) * 180 / M_PI;

				if (pX_a1 < 0)
				{
						pX_a1 = -pX_a1;
						paH = sqrt(sq(pY) + sq(pX_a1));
						B_deg = atan(pX_a1 / pY) * 180 / M_PI;
						A_deg = acos((sq(chain[1]->a) + sq(paH) - sq(chain[3]->d)) / (2 * chain[1]->a * paH)) * 180 / M_PI;
						C_deg = 180 - acos((sq(chain[3]->d) + sq(chain[1]->a) - sq(paH)) / (2 * abs(chain[3]->d) * chain[1]->a)) * 180 / M_PI;
						float D_deg = 90 - (A_deg + B_deg);
						theta(1) = -180 + D_deg;
				}
				else
				{
						theta(1) = -(A_deg + B_deg);
				}

				theta(2) = C_deg;

				// Gripper orientaton calculations
				theta(4) = atan2(sqrt(1 - sq(rot_3_6(2, 2))), (rot_3_6(2, 2))) * 180 / M_PI;

				if ((chain[4]->theta) < 0 and theta(4) < 0)
				{
						theta(4) = atan2(-sqrt(1 - sq(rot_3_6(2, 2))), (rot_3_6(2, 2))) * 180 / M_PI;
				}

				theta(3) = atan2(rot_3_6(1, 2), rot_3_6(0, 2)) * 180 / M_PI;

				if (theta(4) > 0)
				{
						theta(3) = atan2(rot_3_6(1, 2), rot_3_6(0, 2)) * 180 / M_PI;

						if (rot_3_6(2, 1) < 0)
						{
								theta(5) = atan2(-rot_3_6(2, 1), rot_3_6(2, 0)) * 180 / M_PI - 180;
						}
						else
						{
								theta(5) = atan2(-rot_3_6(2, 1), rot_3_6(2, 0)) * 180 / M_PI + 180;
						}
				}
				else
				{
						theta(3) = atan2(-rot_3_6(1, 2), -rot_3_6(0, 2)) * 180 / M_PI;

						if (rot_3_6(2, 1) < 0)
						{
								theta(5) = atan2(rot_3_6(2, 1), -rot_3_6(2, 0)) * 180 / M_PI - 180;
						}
						else
						{
								theta(5) = atan2(rot_3_6(2, 1), -rot_3_6(2, 0)) * 180 / M_PI + 180;
						}
				}

				returnMatrix = theta;
				theta(2) -= 90;
				theta(5) += 180;

				for (byte i = 0; i < _numLinks; i++)
				{
						chain[i]->theta = theta(i) * M_PI / 180;
				}

				// Limit gripper rotations to -90->90 degrees
				// if (theta(3) < -90)
				// {
				//      returnMatrix(3) += 180;
				//      returnMatrix(4) = -returnMatrix(4);
				// }
				// else if (theta(3) > 90)
				// {
				//      returnMatrix(3) -= 180;
				//      returnMatrix(4) = -returnMatrix(4);
				// }
		}
		return returnMatrix;
}

/*
   getJointSpeed
   Calculates the speed contribution of each motor to the end point speed
   relative to the chain orientation.

   Parameters:
      Returns: array containing duty cycle speeds for each joint motor in
               microseconds.

   Problems:
      Not tested properly on real arm.
 */
Matrix<6> getJointSpeed()
{
		Matrix<4, 4> tf[6];
		Matrix<3, 1> d_0[6], r_0[6], cross[6];
		Matrix<6, 6> jointVelocity;
		Matrix<6> speedPeriodMicros;
		float speedRadians[6], velocityAngular[6];

		d_0[5] = forwardKinematics(5).Submatrix(Slice<0, 3>(), Slice<3, 4>());

		for (byte i = 0; i < 6; i++)
		{
				tf[i] = forwardKinematics(i);
				d_0[i] = d_0[5] - tf[i].Submatrix(Slice<0, 3>(), Slice<3, 4>());
				r_0[i] = tf[i].Submatrix(Slice<0, 3>(), Slice<2, 3>());
				cross[i] = crossProduct(d_0[i], r_0[i]);
		}

		for (byte j = 0; j < 3; j++)
		{
				for (byte i = 0; i < 6; i++)
				{
						jointVelocity(j, i) = cross[i](j, 0);
						jointVelocity(j + 3, i) = r_0[i](j);
				}
		}

		Invert(jointVelocity);

		for (byte j = 0; j < 6; j++)
		{
				velocityAngular[j] = 0;
				for (byte i = 0; i < 6; i++)
				{
						velocityAngular[j] += jointVelocity(j, i) * speedCartesian[j];
				}
				speedRadians[j] = abs(velocityAngular[j]);
				speedPeriodMicros(j) = 1000 / ((degreesPerStep[j] * microstepDivider[j] * M_PI / 180) / speedRadians[j]) / 2;
		}

		return speedPeriodMicros;
}

/*
   moveToAngle
   Moves each joint to the specified angle at the specified speed.

   Parameters:
      moveDirection: Array of motor rotation directions (0: cw, 1: ccw)
      moveAngles: Array of motor rotation angles (degrees)
      speedPeriodMicros: Array of motor pulse speeds.
 */
void moveToAngle(bool moveDirection[numMotors], float moveAngles[numMotors], Matrix<6> speedPeriodMicros)
{
		long currentMicros[numMotors] = {}, previousMicros[numMotors] = {};
		int maxSteps = 0, stepCount[numMotors] = {};
		int steps[numMotors] = {};
		bool pulToggle[numMotors] = {};
		byte maxStepsIndex = 0;

		for (byte i = 0; i < numMotors; i++)
		{
				digitalWrite(dirPin[i], moveDirection[i]); //Set Motor Direction
				steps[i] = (moveAngles[i] / degreesPerStep[i] * microstepDivider[i]) * 2; //Calculate step count per motor
				// chain[i]->theta = (((steps[i]/2) * degreesPerStep[i]) + anglesPrevious[i])* M_PI/180;

				if (maxSteps < steps[i])
				{
						maxSteps = steps[i]; //calculate step count of longest motor move
						maxStepsIndex = i; //get index of motor with most steps
				}
		}
		// CHECK THIS, IF SPEED OF MOTOR WITH STEPS /= MAXSTEPS IS FASTER THAN MAXSTEPS MOTOR IT WON'T COMPLETE THE MOVE CORRECTLY

		while (maxSteps > stepCount[maxStepsIndex])
		{
				for (byte i = 0; i < numMotors; i++)
				{
						if (stepCount[i] < steps[i])
						{
								currentMicros[i] = micros();
								if (currentMicros[i] - previousMicros[i] >= speedPeriodMicros(i))
								{
										pulToggle[i] = !pulToggle[i];
										digitalWrite(pulPin[i], pulToggle[i]);
										stepCount[i]++;
										previousMicros[i] = currentMicros[i];
								}
						}
				}
		}
}

/*
   moveToPosition
   Move the arm to a cartesian position using inverse kinematics.
   Moves the arm to the input position relative to its current position.

   Parameters:
      targetCartRelative: The target array {X,Y,Z,Yaw,Pitch,Roll} of the move
                          relative to its current positioning.
 */
void moveToPosition(Matrix<6> targetCartRelative)
{
		bool moveDirection[numMotors] = {};
		// k.calibration();

		Serial.println("CART GLOBAL");
		for (byte i = 0; i < 6; i++)
		{
				cartGlobal[i] = targetCartRelative(i);
				Serial.println(cartGlobal[i]);
		}

		Matrix<6> anglesCurrent = inverseKinematics(cartGlobal);

		Serial.println("ANGLES CURRENT");
		for (int i=0; i<6; i++)
		{
				Serial.print(i);
				Serial.print(": ");
				Serial.println(anglesCurrent(i));
		}

		Matrix<6> speedPeriodMicros = getJointSpeed();

		Serial.println("ANGLES DIFFERENCE CALCULATED");
		for (byte i = 0; i < _numLinks; i++)
		{
				anglesDifference[i] = anglesCurrent(i) - anglesPrevious[i];
				anglesPrevious[i] += anglesDifference[i];
				Serial.print(i);
				Serial.print(": ");
				Serial.println(anglesDifference[i]);

				if (anglesDifference[i] > anglesMax[i])
				{
						anglesDifference[i] = anglesMax[i];
						// Serial << i << ": Max rotation limited to " << anglesMax[i] << "\n";
				}
				else if (anglesDifference[i] < anglesMin[i])
				{
						anglesDifference[i] = anglesMin[i];
						// Serial << i << ": Min rotation limited to " << anglesMin[i] << "\n";
				}

				if (anglesDifference[i] < 0)
				{
						moveDirection[i] = 1;
						anglesDifference[i] = abs(anglesDifference[i]);
				}
				else
				{
						moveDirection[i] = 0;
				}
		}

		Matrix<6> moveSpeed = {500, 500, 500, 500, 500};
		moveToAngle(moveDirection, anglesDifference, moveSpeed);
}

/*
   calibrateChain
   Run all of the joints until they hit the limit switches, then back to the
   "home" position.

   Parameters:
      calibrationSpeed: Array containing duty cycle speeds for all motors in
                        microseconds.
 */
void calibrateChain(float calibrationSpeed[numMotors])
{
		long currentMicros[5], previousMicros[5] = {};
		bool calibrationDirection[5] = {0, 1, 0, 0, 1};
		bool limitSwitchStates[5] = {0, 0, 0, 0, 0};
		bool pulToggle[5] = {};
		bool calibrationDone = false;

		// Homing Parameters
		bool homingDirection[5] = {1, 0, 1, 1, 0};
		float homingDegree[5] = {10, 0, 0, 0, 0};

		// Set motor calibration moveDirection
		for (byte i = 0; i < 5; i++)
		{
				digitalWrite(dirPin[i], calibrationDirection[i]);
		}

		// Check if limit switches are toggled before prog start
		for (byte i = 0; i < 5; i++)
		{
				if (digitalRead(limPin[i]) == 1)
				{
						limitSwitchStates[i] = 1;
				}
		}

		// Run to limits
		while (calibrationDone != true)
		{
				calibrationDone = true;
				for (byte i = 0; i < 5; i++)
				{
						if (limitSwitchStates[i] == 0)
						{
								calibrationDone = false;
						}
				}
				for (byte i = 0; i < 5; i++)
				{
						if (digitalRead(limPin[i]) == true)
						{
								limitSwitchStates[i] = true;
								Serial.println(i);
						}
						if (limitSwitchStates[i] == false)
						{
								Serial << pulPin[i] << "\n";
								currentMicros[i] = micros();
								if (currentMicros[i] - previousMicros[i] >= calibrationSpeed[i])
								{
										pulToggle[i] = !pulToggle[i];
										digitalWrite(pulPin[i], pulToggle[i]);
										previousMicros[i] = currentMicros[i];
								}
						}
				}
		}

		// Reverse motors and run to home position
		Serial.println("Returning to home position");
		move(homingDirection, homingDegree, calibrationSpeed);
}


/*
   manualControl
   Control the arm (joint wise) using the PS3 controller on the raspberry pi
   (controller.pyscript) over the serial port.

   Parameters:
      Controller Inputs via the raspberry pi.
      Joysticks: Each axis controls a joint.
      L&R Bumpers: Control the wrist rotation.
      Right Buttons: Set new waypoint on each button.
      Left D-Pad: Read back waypoints set on right buttons.
      Start: Currently runs to position set on X button.
 */
void manualControl()
{
		bool moveDirection[numMotors] = {};
		bool pulToggle[numMotors] = {};
		int stepCount[numMotors] = {};
		int joystickValue[6] = {};
		int joystickSpeed[6] = {};
		int currentSpeed[numMotors] = {};
		int initialSpeed[numMotors] = {};
		long previousMicros[numMotors] = {};
		long currentMicros[numMotors] = {};
		Matrix<4, 6> waypoint = {};
		waypoint.Fill(0);

		int acceleration[numMotors] = {1, 1, 1, 1, 1};
		long finalSpeed[numMotors] = {500, 500, 500, 500, 500};

		for (int i = 0; i < numMotors; i++)
		{
				initialSpeed[i] = finalSpeed[i] + 1000;
		}

		Serial.println("Manual Control Active");
		while (1)
		{
				// Read Controller
				if (Serial.available())
				{
						int data = Serial.read();
						// Serial.println(data);
						// Analog Inputs
						if (data >= 85 && data <= 88) //Read Joystics (X,Y,RX,RY)
						{
								int readData = -1;
								while (readData == -1)
								{
										readData = Serial.read();
								}
								char i = data - 85;
								joystickValue[i] = readData;
								joystickSpeed[i] = abs(joystickValue[i] - 127) * 4;
								if (joystickValue[i] <= 127)
								{
										moveDirection[i] = !joystickDirection[i];
								}
								else
								{
										moveDirection[i] = joystickDirection[i];
								}
								digitalWrite(dirPin[i], moveDirection[i]);
						}
						else if (data >= 89 && data <= 90) //Read Bumpers (LB, RB)
						{
								int readData = -1;
								while (readData == -1)
								{
										readData = Serial.read();
								}
								char i = data - 89;
								joystickValue[4] = readData;

								joystickSpeed[4] = joystickValue[4];
								if (i == 0)
								{
										moveDirection[4] = !joystickDirection[4];
								}
								else
								{
										moveDirection[4] = joystickDirection[4];
								}
								digitalWrite(dirPin[4], moveDirection[4]);
						}
						else if (data >= 0 && data <= 3)
						{
								// Save point (degrees);
								Serial << "Saving Position (" << data << ")\n";
								for (byte i = 0; i < numMotors; i++)
								{
										waypoint(data, i) = stepCount[i] * degreesPerStep[i] / microstepDivider[i];
										Serial << "Axis " << i << ": " << waypoint(data, i) << "\n";
								}
						}
						else if (data >= 4 && data <= 7)
						{
								Serial << "Loading Position (" << data - 4 << ")\n";
								for (byte i = 0; i < numMotors; i++)
								{
										Serial << "Axis " << i << ": " << waypoint(data - 4, i) << "\n";
								}
						}
						else
						{
								// Buttons
								switch (data)
								{
								case 8: //start

										Serial.println("Running to Position Radians (1)");
										float point[6] = {};
										for (int i = 0; i < 6; i++)
										{
												point[i] = (waypoint(1, i) * M_PI / 180);
												// point[i] = chain[i]->theta + (waypoint(1, i) * M_PI / 180);
												Serial << "point " << i << ": " << point[i] << "\n";
										}


										Matrix<4, 4> fk = forwardKinematics();

										Matrix<6> fkPos = getPositionFromMatrix(fk);

										moveToPosition(fkPos);         //angles input incorrect!!!!!!!!!!!!!!
										break;
								case 9: //select
										break;
								case 10: //l_trig
										break;
								case 11: //r_trig
										break;
								case 12: //lb_trig
										break;
								case 13: //rb_trig
										break;
								case 14: //l_stick
										break;
								case 15: //r_stick
										break;
								case 20: //PS Button
										Serial.println("PROGRAM STOPPED");
										for (char i = 0; i < 6; i++)
										{
												joystickSpeed[i] = 0;
										}
										break;
								default:
										break;
								}
						}
				}

				// Set Speeds
				for (int i = 0; i < 5; i++)
				{
						if (joystickSpeed[i] > 10 || joystickSpeed[i] < -10)
						{
								currentMicros[i] = micros();

								if (currentMicros[i] - previousMicros[i] >= initialSpeed[i] - currentSpeed[i])
								{
										previousMicros[i] = currentMicros[i];

										pulToggle[i] = !pulToggle[i];
										digitalWrite(pulPin[i], pulToggle[i]);

										if (moveDirection[i] == !joystickDirection[i])
										{
												stepCount[i]++;
										}
										else
										{
												stepCount[i]--;
										}

										if (currentSpeed[i] < joystickSpeed[i])
										{
												currentSpeed[i] += acceleration[i];
										}
										else if (currentSpeed[i] > joystickSpeed[i])
										{
												currentSpeed[i] = joystickSpeed[i];
										}
								}
						}
				}
		}
}
};


Matrix<4, 4> getDHMatrix(float target[6])
{
		float cosA = cos(target[3]);
		float cosB = cos(target[4]);
		float cosG = cos(target[5]);
		float sinA = sin(target[3]);
		float sinB = sin(target[4]);
		float sinG = sin(target[5]);
		// float cosBsinA = cosB * sinA;
		// float cosAcosB = cosA * cosB;

		Matrix<4, 4> matrix = {
				-(cosA * cosG - cosB * sinA * sinG), cosG * sinA + cosA * cosB * sinG, sinA * sinB, target[0],
				cosB * cosG * sinA + cosA * sinG, cosA * cosB * cosG - sinA * sinG, -cosA * sinB, target[1],
				sinB * sinG, cosG * sinB, -cosB, target[2],
				0, 0, 0, 1
		};
		return matrix;
}

/*
   printMatrix4x4 & printMatrix3x3
   utility to print out Matrix object with a specified decimal point.
 */
void printMatrix4x4(Matrix<4, 4> matrix, int decimals = 10)
{
		for (byte i = 0; i < 4; i++)
		{
				for (byte j = 0; j < 4; j++)
				{
						Serial.print(matrix(i, j), decimals);
						Serial.print(", ");
				}
				Serial.println("");
		}
}

void printMatrix3x3(Matrix<3, 3> matrix, int decimals = 10)
{
		for (byte i = 0; i < 3; i++)
		{
				for (byte j = 0; j < 3; j++)
				{
						Serial.print(matrix(i, j), decimals);
						Serial.print(", ");
				}
				Serial.println("");
		}
}

/*
   getPositionFromMatrix
   Get the position matrix containing (x,y,z,yaw,pitch,roll) from the
   transformation matrix returned by the forwardKinematics function.

   Parameters:
      matrix: Transformation Matrix (4x4)

      Returns: position in {x,y,z,yaw,pitch,roll} format.
 */
Matrix<6> getPositionFromMatrix(Matrix<4, 4> matrix)
{
		float x = matrix(0, 3);
		float y = matrix(1, 3);
		float z = matrix(2, 3);
		float pitch = (180 / M_PI) * atan2(sqrt(sq(matrix(0, 2)) + sq(matrix(1, 2))), -matrix(2, 2));
		float yaw = (180 / M_PI) * atan2(matrix(2, 0) / pitch, matrix(1, 2) / pitch);
		float roll = (180 / M_PI) * atan2(matrix(0, 2) / pitch, matrix(1, 2) / pitch);

		// CONVERT BACK TO DEGREES
		Matrix<6> returnMatrix = {x, y, z, yaw, pitch, roll};
		return returnMatrix;
}

//Calcualte Cross Product of two matricies, called in getJointSpeed.
Matrix<3, 1> crossProduct(Matrix<3, 1> x, Matrix<3, 1> y)
{
		Matrix<3, 1> returnValue;
		returnValue(0, 0) = x(1, 0) * y(2, 0) - x(2, 0) * y(1, 0);
		returnValue(1, 0) = x(2, 0) * y(0, 0) - x(0, 0) * y(2, 0);
		returnValue(2, 0) = x(0, 0) * y(1, 0) - x(1, 0) * y(0, 0);
		return returnValue;
}

// Save link angles to EEPROM, for storing arm positioning when power is turned off.
void savePositionData(KinematicChain<6> k)
{
		// FIX FOR FLOAT VALUES
		// Store Location with EEPROM.write(ADDRESS, VALUE)
		for (int address = 0; address < k.numLinks(); address++)
		{
				Serial.println(k.chain[address]->theta * 100);
				EEPROM.write(address, k.chain[address]->theta * 100);
		}
}

// Retrieve link angles from EEPROM.
Matrix<6> getPositionData(KinematicChain<6> k, byte registerSelect = 0)
{
		Matrix<6> orientation;
		int numLinks = k.numLinks();

		registerSelect = registerSelect * numLinks;
		for (int address = 0 + registerSelect; address < numLinks + registerSelect; address++)
		{
				orientation(address) = EEPROM.read(address) / 100;
		}
		return orientation;
}

KinematicChain<6> roboticArm;
//      theta,        alpha,        d,    a
Link l1(1.74533E-06,  -1.570796327, 220,  0);
Link l2(-1.570796327, 0,            0,    220);
Link l3(-1.553343034, 1.570796327,  0,    0);
Link l4(0,            -1.570796327, -220, 0);
Link l5(1.74533E-06,  1.570796327,  0,    0);
Link l6(3.141592654,  0,            -165, 0);

void setup()
{
		Serial.begin(115200);

		roboticArm.addLink(l1);
		roboticArm.addLink(l2);
		roboticArm.addLink(l3);
		roboticArm.addLink(l4);
		roboticArm.addLink(l5);
		roboticArm.addLink(l6);
		gripper.attach(9);

		for (byte i = 0; i < numMotors; i++)
		{
				pinMode(pulPin[i], OUTPUT);
				pinMode(dirPin[i], OUTPUT);
				pinMode(limPin[i], INPUT);
				digitalWrite(limPin[i], HIGH);

				if(i<2) {
						digitalWrite(enaPin[i], HIGH);
				}else{
						digitalWrite(enaPin[i], LOW);
				}
		}
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
