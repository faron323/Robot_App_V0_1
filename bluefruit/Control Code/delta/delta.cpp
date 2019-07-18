#include "delta.h"
#include "Arduino.h"

// Delta Kinematics Reference Paper:
// R.L. Williams II, “The Delta Parallel Robot: Kinematics Solutions”
// http://www.ohio.edu/people/williar4/html/pdf/DeltaKin.pdf

DeltaArm::DeltaArm(float (*anglesTemp)[3], float sideBase, float sidePlatform, float upperLegLength, float lowerLegLength, float lowerLegWidth)
{
		angles = anglesTemp;

		for(int i=0; i<3; i++) {
				anglesCurrent[i] = (*anglesTemp)[i];
		}

		Sb = sideBase;
		Sp = sidePlatform;
		L = upperLegLength;
		l = lowerLegLength;
		h = lowerLegWidth;

		Wb = 0.288675*Sb;
		Ub = 0.577350*Sb;
		Wp = 0.288675*Sp;
		Up = 0.577350*Sp;

		a = Wb - Up;
		b = Sp/2 - 0.866025*Wb;
		c = Wp - Wb/2;

		//Initialize x,y,z for use in the calculateVelocity
		forwardKinematics((*angles),&x,&y,&z);
}

void DeltaArm::initialize(int _pulPin[3],int _dirPin[3],int _enaPin[3], int _limPin[3], float _degreesPerStep[3], float _microstep[3])
{
		for (byte i = 0; i < 3; i++)
		{
				pulPin[i] = _pulPin[i];
				dirPin[i] = _dirPin[i];
				enaPin[i] = _enaPin[i];
				limPin[i] = _limPin[i];
				degreesPerStep[i] = _degreesPerStep[i];
				microstep[i] = _microstep[i];

				pinMode(pulPin[i], OUTPUT);
				pinMode(dirPin[i], OUTPUT);
				pinMode(limPin[i], INPUT);
				digitalWrite(limPin[i], HIGH);
				digitalWrite(enaPin[i], HIGH);
		}
}

void DeltaArm::setLimits(float _upperLimits[3], float _lowerLimits[3])
{
		for(byte i=0; i<3; i++) {
				upperLimits[i] = _upperLimits[i];
				lowerLimits[i] = _lowerLimits[i];
		}
}

void DeltaArm::inverseKinematics(float _x, float _y, float _z)
{
		float E[3] = {};
		float F[3] = {};
		float G[3] = {};

		E[0] = 2*L*(_y+a);
		F[0] = 2*_z*L;
		G[0] = sq(_x)+sq(_y)+sq(_z)+sq(a)+sq(L)+2*_y*a-sq(l);

		E[1] = -L*(1.732050*(_x+b)+_y+c);
		F[1] = 2*_z*L;
		G[1] = sq(_x)+sq(_y)+sq(_z)+sq(b)+sq(c)+sq(L)+2*(_x*b+_y*c)-sq(l);

		E[2] = L*(1.732050*(_x-b)-_y-c);
		F[2] = 2*_z*L;
		G[2] = sq(_x)+sq(_y)+sq(_z)+sq(b)+sq(c)+sq(L)+2*(-_x*b+_y*c)-sq(l);

		float Ta[3] = {},Tb[3] = {},T[3] = {};
		float anglesA[3] = {},anglesB[3] = {};

		for(byte i=0; i<3; i++) {
				T[i] = sq(E[i])+sq(F[i])-sq(G[i]);
				Ta[i] = 2*atan2(-F[i]+ sqrt(T[i]), G[i] - E[i]);
				Tb[i] = 2*atan2(-F[i]- sqrt(T[i]), G[i] - E[i]);

				Ta[i] += M_PI_2;
				Tb[i] += M_PI_2;

				if(Ta[i] < 0) {Ta[i] += 2*M_PI;}
				if(Ta[i] >2*M_PI) {Ta[i] -= 2*M_PI;}
				if(Tb[i] < 0) {Tb[i] += 2*M_PI;}
				if(Tb[i] >2*M_PI) {Tb[i] -= 2*M_PI;}

				anglesA[i] = Ta[i];
				anglesB[i] = Tb[i];

				if(anglesA[i] >= upperLimits[i] && anglesA[i] <= lowerLimits[i] )
				{
						(*angles)[i] = anglesA[i];
				}
				else if(anglesB[i] >= upperLimits[i] && anglesB[i] <= lowerLimits[i] )
				{
						(*angles)[i] = anglesB[i];
				}
		}
}

void DeltaArm::forwardKinematics(float _angles[3], float *_x, float *_y, float *_z)
{
		// 3 Spheres FK Method

		float arm1[3]; //arm 1 (_x,_y,_z)
		float arm2[3]; //arm 2 (_x,_y,_z)
		float arm3[3]; //arm 3 (_x,_y,_z)

		// arm1[0] = Wb + sin(_angles[0])*L; //arm 1, _x
		// arm1[1] = 0;                      //arm 1, _y
		// arm1[2] = cos(_angles[0])*L;      //arm 1, _z
		//
		// arm2[0] = (Wb + sin(_angles[1])*L) * cos(2.09333333333); //arm 2, _x
		// arm2[1] = (Wb + sin(_angles[1])*L) * sin(2.09333333333); //arm 2, _y
		// arm2[2] = cos(_angles[1])*L;                             //arm 2, _z
		//
		// arm3[0] = (Wb + sin(_angles[2])*L) * cos(4.18666666667); //arm 3, _x
		// arm3[1] = (Wb + sin(_angles[2])*L) * sin(4.18666666667); //arm 3, _y
		// arm3[2] = cos(_angles[2])*L;                             //arm 3, _z

		arm1[0] = 0;
		arm1[1] = -Wb-L*cos(_angles[0])+Up;
		arm1[2] = -L*sin(_angles[0]);

		arm2[0] = 0.866025*(Wb+L*cos(_angles[1]))-Sp/2;
		arm2[1] = 0.5*(Wb+L*cos(_angles[1]))-Wp;
		arm2[2] = -L*sin(_angles[1]);

		arm3[0] = -0.866025*(Wb+L*cos(_angles[2]))+Sp/2;
		arm3[1] = 0.5*(Wb+L*cos(_angles[2]))-Wp;
		arm3[2] = -L*sin(_angles[2]);

		float a11 = 2*(arm3[0]-arm1[0]);
		float a12 = 2*(arm3[1]-arm1[1]);
		float a13 = 2*(arm3[2]-arm1[2]);

		float a21 = 2*(arm3[0]-arm2[0]);
		float a22 = 2*(arm3[1]-arm2[1]);
		float a23 = 2*(arm3[2]-arm2[2]);

		// Checking for divide by 0 Singularity (Appendix A of Paper)
		if(a13 == 0 || a23 == 0) {
				// If all z's are equal run equalZForwardKinematics instead
				equalZForwardKinematics(_angles, _x, _y, _z);
				return;
		}

		float r1 = l;
		float r2 = l;
		float r3 = l;

		float b1 = sq(r1)-sq(r3)-sq(arm1[0])-sq(arm1[1])-sq(arm1[2])+sq(arm3[0])+sq(arm3[1])+sq(arm3[2]);
		float b2 = sq(r2)-sq(r3)-sq(arm2[0])-sq(arm2[1])-sq(arm2[2])+sq(arm3[0])+sq(arm3[1])+sq(arm3[2]);

		float a1 = a11/a13-a21/a23;
		float a2 = a12/a13-a22/a23;
		float a3 = b2/a23-b1/a13;
		float a4 = -a2/a1;
		float a5 = -a3/a1;
		float a6 = (-a21*a4-a22)/a23;
		float a7 = (b2-a21*a5)/a23;

		float a = sq(a4)+1+sq(a6);
		float b = 2*a4*(a5-arm1[0])-2*arm1[1]+2*a6*(a7-arm1[2]);
		float c = a5*(a5-2*arm1[0])+a7*(a7-2*arm1[2])+sq(arm1[0])+sq(arm1[1])+sq(arm1[2])-sq(r1);
		float w = b*b-4*a*c;

		// Calculate 3 Sphere Intersections
		// Returns 2 Results, Positive & Negative Intersections
		float ya = (-b+sqrt(w))/(2*a);
		float yb = (-b-sqrt(w))/(2*a);

		float xa = a4*(ya)+a5;
		float xb = a4*(yb)+a5;

		float za = a6*(ya)+a7;
		float zb = a6*(yb)+a7;

		// FIND METHOD TO PICK CORRECT SOLUTION

		Serial.println("STANDARD FK:");
		Serial.print("xa: ");
		Serial.println(xa,5);
		Serial.print("ya: ");
		Serial.println(ya,5);
		Serial.print("za: ");
		Serial.println(za,5);
		Serial.print("xb: ");
		Serial.println(xb,5);
		Serial.print("yb: ");
		Serial.println(yb,5);
		Serial.print("zb: ");
		Serial.println(zb,5);

		*_x = xa;
		*_y = ya;
		*_z = za;
}

void DeltaArm::equalZForwardKinematics(float _angles[3], float *_x, float *_y, float *_z)
{
		// Forward Kinematics Function for z1=z2=z3 (all arm _z-axis are equal)

		float arm1[3]; //arm 1 (x1,y1,z1)
		float arm2[3]; //arm 2 (x2,y2,z2)
		float arm3[3]; //arm 3 (x3,y3,z3)

		// arm1[0] = Wb + sin(_angles[0])*L; //x1
		// arm1[1] = 0;                      //y1
		// arm1[2] = cos(_angles[0])*L;      //z1
		//
		// arm2[0] = (Wb + sin(_angles[1])*L) * cos(2.09333333333); //x2
		// arm2[1] = (Wb + sin(_angles[1])*L) * sin(2.09333333333); //y2
		// arm2[2] = cos(_angles[1])*L;                             //z2
		//
		// arm3[0] = (Wb + sin(_angles[2])*L) * cos(4.18666666667); //x3
		// arm3[1] = (Wb + sin(_angles[2])*L) * sin(4.18666666667); //y3
		// arm3[2] = cos(_angles[2])*L;                             //z3

		arm1[0] = 0;
		arm1[1] = -Wb-L*cos(_angles[0]-M_PI_2)+Up;
		arm1[2] = -L*sin(_angles[0]-M_PI_2);

		arm2[0] = 0.866025*(Wb+L*cos(_angles[1]-M_PI_2))-Sp/2;
		arm2[1] = 0.5*(Wb+L*cos(_angles[1]-M_PI_2))-Wp;
		arm2[2] = -L*sin(_angles[1]-M_PI_2);

		arm3[0] = -0.86602*(Wb+L*cos(_angles[2]-M_PI_2))+Sp/2;
		arm3[1] = 0.5*(Wb+L*cos(_angles[2]-M_PI_2))-Wp;
		arm3[2] = -L*sin(_angles[2]-M_PI_2);

		// arm1[0] = 0;
		// arm1[1] = -Wb-L*cos(_angles[0]-M_PI_2);
		// arm1[2] = -L*sin(_angles[0]-M_PI_2);
		//
		// arm2[0] = 0.86602540378*(Wb+L*cos(_angles[1]-M_PI_2));
		// arm2[1] = 0.5*(Wb+L*cos(_angles[1]-M_PI_2));
		// arm2[2] = -L*sin(_angles[1]-M_PI_2);
		//
		// arm3[0] = -0.86602540378*(Wb+L*cos(_angles[2]-M_PI_2));
		// arm3[1] = 0.5*(Wb+L*cos(_angles[2]-M_PI_2));
		// arm3[2] = -L*sin(_angles[2]-M_PI_2);

		//radius of sphere = lower leg length
		float r1 = l;
		float r2 = l;
		float r3 = l;

		//linear equation terms
		float a = 2 * (arm3[0] - arm1[0]);
		float b = 2 * (arm3[1] - arm1[1]);
		float c = sq(r1)-sq(r3)-sq(arm1[0])-sq(arm1[1])+sq(arm3[0])+sq(arm3[1]);
		float d = 2 * (arm3[0] - arm2[0]);
		float e = 2 * (arm3[1] - arm2[1]);
		float f = sq(r2)-sq(r3)-sq(arm2[0])-sq(arm2[1])+sq(arm3[0])+sq(arm3[1]);

		//final _x and _y calculations
		*_x = (c*e-b*f)/(a*e-b*d);
		*_y = (a*f-c*d)/(a*e-b*d);

		//any arms _z term as z1=z2=z3=zn
		float zn = arm3[2];

		//_z calculation terms
		float B = -2*zn;
		float C = sq(zn)-sq(r1)+sq(*_x-arm1[0])+ sq(*_y-arm1[1]);

		//positive and negative _z from quadratic
		float za = (-B + sqrt(sq(B) - 4*C))/2;
		float zb = (-B - sqrt(sq(B) - 4*C))/2;

		if (za>0) {
				*_z = zb;
		} else{
				*_z = za;
		}
}

/*
   void DeltaArm::moveToAngles() (RELATIVE TO CURRENT ANGLES)

   Generates pulses to move DeltaArm to specified angles/direction, at defined
   speeds based on current rotations.Typically called from within moveToPosition
   where angles/direction arrays are calculated by inverseKinematics(), and
   speed is calculated by calculateVelocity().

    Parameters:
      anglesTemp[3]: float, angles to move to (degrees)
      speed[3]: float, length of pulse (milliseconds)
 */
void DeltaArm::moveToAngles(float anglesTemp[3], float speed[3])
{
		long currentMicros[3] = {}, previousMicros[3] = {};
		int maxSteps = 0, stepCounter[3] = {};
		int stepCount[3] = {};
		bool pulToggle[3] = {};
		byte maxStepsIndex = 0;

		for (byte i = 0; i < 3; i++)
		{
				anglesTemp[i] -= (anglesCurrent[i] * 180/M_PI);
				stepCount[i] = (abs(anglesTemp[i]) / (degreesPerStep[i] / microstep[i]));

				Serial.print(i);
				Serial.print(" | Current: ");
				Serial.print(anglesCurrent[i] * 180/M_PI, 5);

				if(anglesTemp[i] <=0) {
						digitalWrite(dirPin[i], 1); //Set Motor Direction
						anglesCurrent[i] -= stepCount[i] * (degreesPerStep[i]/microstep[i]) * M_PI/180;
				}else{
						digitalWrite(dirPin[i], 0); //Set Motor Direction
						anglesCurrent[i] += stepCount[i] * (degreesPerStep[i]/microstep[i]) * M_PI/180;
				}

				Serial.print("\t| Calculated: ");
				Serial.print((*angles)[i] * 180/M_PI, 5);
				Serial.print("\t| Difference: ");
				Serial.print(anglesTemp[i], 5);
				Serial.print("\t| Actual: ");
				Serial.println(anglesCurrent[i] * 180/M_PI, 5);

				stepCount[i] *= 2;
				if (maxSteps < stepCount[i])
				{
						maxSteps = stepCount[i]; //calculate step count of longest motor move
						maxStepsIndex = i; //get index of motor with most steps
				}
		}

		while (maxSteps > stepCounter[maxStepsIndex]) //stop move when motor with most steps is done
		{
				for (byte i = 0; i < 3; i++)
				{
						if (stepCounter[i] < stepCount[i])
						{
								currentMicros[i] = micros();
								if (currentMicros[i] - previousMicros[i] >= speed[i])
								{
										pulToggle[i] = !pulToggle[i];
										digitalWrite(pulPin[i], pulToggle[i]);
										stepCounter[i]++;
										previousMicros[i] = currentMicros[i];
								}
						}
				}
		}
}

/*
   void DeltaArm::moveByAngles()

   Moves arm motors by a specified angle, at a specified speed.

    Parameters:
      anglesTemp[3]: float, angles to move to (degrees)
      speed[3]: float, length of pulse (milliseconds)
 */
void DeltaArm::moveByAngles(float anglesTemp[3], float speed[3])
{
		long currentMicros[3] = {}, previousMicros[3] = {};
		int maxSteps = 0, stepCounter[3] = {};
		int stepCount[3] = {};
		bool pulToggle[3] = {};
		byte maxStepsIndex = 0;

		for (byte i = 0; i < 3; i++)
		{
				if(anglesTemp[i] <=0) {
						digitalWrite(dirPin[i], 1); //Set Motor Direction
				}else{
						digitalWrite(dirPin[i], 0); //Set Motor Direction
				}

				stepCount[i] = (abs(anglesTemp[i]) / (degreesPerStep[i] / microstep[i])) * 2;
				if (maxSteps < stepCount[i])
				{
						maxSteps = stepCount[i]; //calculate step count of longest motor move
						maxStepsIndex = i; //get index of motor with most steps
				}
		}

		while (maxSteps > stepCounter[maxStepsIndex]) //stop move when motor with most steps is done
		{
				for (byte i = 0; i < 3; i++)
				{
						if (stepCounter[i] < stepCount[i])
						{
								currentMicros[i] = micros();
								if (currentMicros[i] - previousMicros[i] >= speed[i])
								{
										pulToggle[i] = !pulToggle[i];
										digitalWrite(pulPin[i], pulToggle[i]);
										stepCounter[i]++;
										previousMicros[i] = currentMicros[i];
								}
						}
				}
		}
}

/*
   void DeltaArm::moveToPosition();

   Calculates DeltaArm move parameters using inverseKinematics() to reach a
   _x,_y,_z position. Motor speeds are set based on calculateVelocity() function.
   Input _x,_y,_z are relative to the global coordinates.

    Parameters:
      _x,_y,_z: float, cartesian position in mm
 */
void DeltaArm::moveToPosition(float _x, float _y, float _z)
{
		float degrees[3] = {};
		float speed[3] = {6000, 6000, 6000};
		bool direction[3] = {};

		inverseKinematics(_x,_y,_z);

		// Calculate Speed HERE

		for(byte i=0; i<3; i++) {
				degrees[i] = (*angles)[i] * 180/M_PI;

				// degrees[i] = (*angles)[i]-anglesCurrent[i] * 180/M_PI;
				//
				//      anglesCurrent[i] = degrees[i] + anglesCurrent[i];
				//      if(degrees[i] <=0) {
				//              direction[i] = 1;
				//              degrees[i] = abs(degrees[i])*(180/M_PI);
				//      }else{
				//              direction[i] = 0;
				//              degrees[i] = abs(degrees[i])*(180/M_PI);
				//      }
		}

		moveToAngles(degrees, speed);
}

void DeltaArm::calculateVelocity(float _angles[3])
{

		x = 2;
		y = 2;
		z = 2;

		Serial.println("POS VEL XYZ");
		Serial.print("x: ");
		Serial.println(x,5);
		Serial.print("y: ");
		Serial.println(y,5);
		Serial.print("z: ");
		Serial.println(z,5);

		// Discrete speed contribution for each motor (rad/s)
		float b11 = L*((y+a)*sin(_angles[0])-z*cos(_angles[0]));
		float b22 = -L*((1.732050*(x+b)+y+c)*sin(_angles[1])+2*z*cos(_angles[1]));
		float b33 = L*((1.732050*(x-b)-y-c)*sin(_angles[2])-2*z*cos(_angles[2]));

		// calculates contribution of each motor to x,y,z, move based on current rotations

		// need to calculate speed based on how long you want move to take
		// say, xyz = 0,0,0 -> 10,0,0, time = 5 sec
		// s=d/t -> 10/5 -> 2mm/s end effector speed


		Serial.println("VELOCITY");
		Serial.print("speed motor 1: ");
		Serial.println(b11,5);
		Serial.print("speed motor 2: ");
		Serial.println(b22,5);
		Serial.print("speed motor 3: ");
		Serial.println(b33,5);
}

// void DeltaArm::calculatePath()
// {
// }

// Calculate path:
// get start and end point
// get increment
// calculate IK and speed over path increment
// interpolate b/w increments


// ENCODE POSITION BASED ON ACTUAL MOVEMENT (DONE)
// MAKE FK CHOOSE EQUAL Z IF NULL CONDITION IS TRIGGERED (DONE)

// SYNC UP FK AND IK FUNCTION READOUTS TO MAKE SURE THEY ALL HAVE THE SAME ROTATION OFFSETS
// forwardKinematics CHOOSE CORRECT A/B OPTION FOR X,Y,Z
// CHECK VELOCITY, DOES IT REQUIRE CALCULATION PER INCREMENT OR PER MOVE?
