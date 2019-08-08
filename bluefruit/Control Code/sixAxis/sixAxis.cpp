#include "sixAxis.h"
#include "config.h"

Matrix<4, 4> Link::getTransformationMatrix()
{
		float cos0 = cos(theta);
		float sin0 = sin(theta);
		float cosa = cos(alpha);
		float sina = sin(alpha);

		Matrix<4, 4> matrix = {cos0, -sin0 * cosa, sin0 * sina, a * cos0,
				               sin0, cos0 * cosa, -cos0 * sina, a * sin0,
				               0, sina, cosa, d,
				               0, 0, 0, 1};

		return matrix;
}

void KinematicChain::addLink(Link &l)
{
		if (_numLinks == 6)
				return;

		chain[_numLinks++] = &l;
}

int KinematicChain::numLinks()
{
		return _numLinks;
}

Matrix<4, 4> KinematicChain::forwardKinematics(int depth = 6)
{
		Matrix<4, 4> tempMatrix, matrix = {};

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

Matrix<6> KinematicChain::inverseKinematics(float target[6])
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

Matrix<6> KinematicChain::getJointSpeed()
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

void KinematicChain::moveToAngle(bool moveDirection[6], float moveAngles[6], Matrix<6> speedPeriodMicros)
{
		long currentMicros[6] = {}, previousMicros[6] = {};
		int maxSteps = 0, stepCount[6] = {};
		int steps[6] = {};
		bool pulToggle[6] = {};
		byte maxStepsIndex = 0;

		for (byte i = 0; i < 6; i++)
		{
				digitalWrite(dirPin[i], moveDirection[i]);                        //Set Motor Direction
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
				for (byte i = 0; i < 6; i++)
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

void KinematicChain::moveToPosition(Matrix<6> targetCartRelative)
{
		bool moveDirection[6] = {};
		// k.calibration();

		Serial.println("CART GLOBAL");
		for (byte i = 0; i < 6; i++)
		{
				cartGlobal[i] = targetCartRelative(i);
				Serial.println(cartGlobal[i]);
		}

		Matrix<6> anglesCurrent = inverseKinematics(cartGlobal);

		Serial.println("ANGLES CURRENT");
		for (int i = 0; i < 6; i++)
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

void KinematicChain::manualControl()
{
		bool moveDirection[6] = {};
		bool pulToggle[6] = {};
		int stepCount[6] = {};
		int joystickValue[6] = {};
		int joystickSpeed[6] = {};
		int currentSpeed[6] = {};
		int initialSpeed[6] = {};
		long previousMicros[6] = {};
		long currentMicros[6] = {};
		long previousMillis[6] = {}, currentMillis[6] = {};
		Matrix<4, 6> waypoint = {};
		waypoint.Fill(0);

		long finalSpeed[6] = {20, 50, 20, 50, 60, 0};

		for (int i = 0; i < 6; i++)
		{
				initialSpeed[i] = finalSpeed[i] + 512;
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

								joystickSpeed[5] = 0;
								joystickSpeed[4] = joystickValue[4] * 2;
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
						else if (data >= 83 && data <= 84)
						{
								int readData = -1;
								while (readData == -1)
								{
										readData = Serial.read();
								}
								// DO GRIPPER STUFF HERE
								char i = data - 84;
								joystickValue[5] = readData;
								joystickSpeed[4] = 0;

								joystickSpeed[5] = joystickValue[5] * 2;
								if (i == 0)
								{
										moveDirection[5] = !joystickDirection[4];
								}
								else
								{
										moveDirection[5] = joystickDirection[4];
								}
								digitalWrite(dirPin[5], moveDirection[5]);
						}
						else if (data >= 0 && data <= 3)
						{
								// Save point (degrees);
								Serial << "Saving Position (" << data << ")\n";
								for (byte i = 0; i < 6; i++)
								{
										waypoint(data, i) = stepCount[i] * degreesPerStep[i] / microstepDivider[i];
										Serial << "Axis " << i << ": " << waypoint(data, i) << "\n";
								}
						}
						else if (data >= 4 && data <= 7)
						{
								Serial << "Loading Position (" << data - 4 << ")\n";
								for (byte i = 0; i < 6; i++)
								{
										Serial << "Axis " << i << ": " << waypoint(data - 4, i) << "\n";
								}
						}
						else
						{
								Serial.println(data);
								// Buttons
								if (data == 9)
								{
										Serial.println("IF");
										for (int i = 0; i < 6; i++)
										{
												joystickSpeed[i] = 0;
										}
								}

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

										moveToPosition(fkPos); //angles input incorrect!!!!!!!!!!!!!!
										break;
								case 9: //select
										// for (int i = 0; i < 6; i++)
										// {
										//     joystickSpeed[i] = 0;
										// }
										Serial.println("SELECT");
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
										Serial.println("DEFAULT");
										break;
								}
						}
				}

				// General overhead is 20us, so 40us with 1 motor running -> +20us per additional motor
				// (with step counter disabled)
				for (int i = 0; i < 6; i++)
				{
						if ((joystickSpeed[i] > 30 || joystickSpeed[i] < -30) || pulToggle[i] == 0)
						{
								currentMicros[i] = micros();

								if (currentMicros[i] - previousMicros[i] >= initialSpeed[i] - joystickSpeed[i])
								// if (currentMicros[i] - previousMicros[i] >= 0) //fastest possible pulsing
								{

										digitalWrite(pulPin[i], pulToggle[i]);

										pulToggle[i] = !pulToggle[i];

										// if (pulToggle[i] == 1)
										// {
										//     if (moveDirection[i] == !joystickDirection[i])
										//     {
										//         stepCount[i]++;
										//     }
										//     else
										//     {
										//         stepCount[i]--;
										//     }
										// }
										previousMicros[i] = currentMicros[i];
								}
						}
				}
		}
}

// void KinematicChain::calibrateChain(float calibrationSpeed[6])
// {
//     long currentMicros[5], previousMicros[5] = {};
//     bool calibrationDirection[5] = {0, 1, 0, 0, 1};
//     bool limitSwitchStates[5] = {0, 0, 0, 0, 0};
//     bool pulToggle[5] = {};
//     bool calibrationDone = false;

//     // Homing Parameters
//     bool homingDirection[5] = {1, 0, 1, 1, 0};
//     float homingDegree[5] = {10, 0, 0, 0, 0};

//     // Set motor calibration moveDirection
//     for (byte i = 0; i < 5; i++)
//     {
//         digitalWrite(dirPin[i], calibrationDirection[i]);
//     }

//     // Check if limit switches are toggled before prog start
//     for (byte i = 0; i < 5; i++)
//     {
//         if (digitalRead(limPin[i]) == 1)
//         {
//             limitSwitchStates[i] = 1;
//         }
//     }

//     // Run to limits
//     while (calibrationDone != true)
//     {
//         calibrationDone = true;
//         for (byte i = 0; i < 5; i++)
//         {
//             if (limitSwitchStates[i] == 0)
//             {
//                 calibrationDone = false;
//             }
//         }
//         for (byte i = 0; i < 5; i++)
//         {
//             if (digitalRead(limPin[i]) == true)
//             {
//                 limitSwitchStates[i] = true;
//                 Serial.println(i);
//             }
//             if (limitSwitchStates[i] == false)
//             {
//                 Serial << pulPin[i] << "\n";
//                 currentMicros[i] = micros();
//                 if (currentMicros[i] - previousMicros[i] >= calibrationSpeed[i])
//                 {
//                     pulToggle[i] = !pulToggle[i];
//                     digitalWrite(pulPin[i], pulToggle[i]);
//                     previousMicros[i] = currentMicros[i];
//                 }
//             }
//         }
//     }

//     // Reverse motors and run to home position
//     Serial.println("Returning to home position");
//     move(homingDirection, homingDegree, calibrationSpeed);
// }

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

Matrix<3, 1> crossProduct(Matrix<3, 1> x, Matrix<3, 1> y)
{
		Matrix<3, 1> returnValue;
		returnValue(0, 0) = x(1, 0) * y(2, 0) - x(2, 0) * y(1, 0);
		returnValue(1, 0) = x(2, 0) * y(0, 0) - x(0, 0) * y(2, 0);
		returnValue(2, 0) = x(0, 0) * y(1, 0) - x(1, 0) * y(0, 0);
		return returnValue;
}

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
