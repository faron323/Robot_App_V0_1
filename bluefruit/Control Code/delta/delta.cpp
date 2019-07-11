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

		Wb = 0.28867513459*Sb;
		Ub = 0.57735026919*Sb;
		Wp = 0.28867513459*Sp;
		Up = 0.57735026919*Sp;

		a = Wb - Up;
		b = Sp/2 - 0.86602540378*Wb;
		c = Wp - Wb/2;
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

void DeltaArm::inverseKinematics(float x, float y, float z)
{
		float E[3] = {};
		float F[3] = {};
		float G[3] = {};

		E[0] = 2*L*(y+a);
		F[0] = 2*z*L;
		G[0] = sq(x)+sq(y)+sq(z)+sq(a)+sq(L)+2*y*a-sq(l);

		E[1] = -L*(1.73205080757*(x+b)+y+c);
		F[1] = 2*z*L;
		G[1] = sq(x)+sq(y)+sq(z)+sq(b)+sq(c)+sq(L)+2*(x*b+y*c)-sq(l);

		E[2] = L*(1.73205080757*(x-b)-y-c);
		F[2] = 2*z*L;
		G[2] = sq(x)+sq(y)+sq(z)+sq(b)+sq(c)+sq(L)+2*(-x*b+y*c)-sq(l);

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

				Serial.print(i);
				Serial.print(" Angle A: ");
				Serial.println(anglesA[i]);
				Serial.print(i);
				Serial.print(" Angle B: ");
				Serial.println(anglesB[i]);
				Serial.print("U LIM:");
				Serial.println(upperLimits[i]);
				Serial.print("L LIM:");
				Serial.println(lowerLimits[i]);

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

void DeltaArm::equalZForwardKinematics(float _angles[3])
{
		// Forward Kinematics Function for z1=z2=z3 (all arm z-axis are equal)

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

		arm2[0] = 0.86602540378*(Wb+L*cos(_angles[1]-M_PI_2))-Sp/2;
		arm2[1] = 0.5*(Wb+L*cos(_angles[1]-M_PI_2))-Wp;
		arm2[2] = -L*sin(_angles[1]-M_PI_2);

		arm3[0] = -0.86602540378*(Wb+L*cos(_angles[2]-M_PI_2))+Sp/2;
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

		//final x and y calculations
		float x = (c*e-b*f)/(a*e-b*d);
		float y = (a*f-c*d)/(a*e-b*d);
		float z;

		//any arms z term as z1=z2=z3=zn
		float zn = arm3[2];

		//z calculation terms
		float B = -2*zn;
		float C = sq(zn)-sq(r1)+sq(x-arm1[0])+ sq(y-arm1[1]);

		//positive and negative z from quadratic
		float za = (-B + sqrt(sq(B) - 4*C))/2;
		float zb = (-B - sqrt(sq(B) - 4*C))/2;

		if (za>0) {
				z = zb;
		} else{
				z = za;
		}
		Serial.println("EQUAL ZEROS:");
		Serial.print("x: ");
		Serial.println(x,5);
		Serial.print("y: ");
		Serial.println(y,5);
		Serial.print("za: ");
		Serial.println(za,5);
		Serial.print("zb: ");
		Serial.println(zb,5);
}

void DeltaArm::forwardKinematics(float _angles[3])
{
		// 3 Spheres FK Method

		float arm1[3]; //arm 1 (x,y,z)
		float arm2[3]; //arm 2 (x,y,z)
		float arm3[3]; //arm 3 (x,y,z)

		// arm1[0] = Wb + sin(_angles[0])*L; //arm 1, x
		// arm1[1] = 0;                      //arm 1, y
		// arm1[2] = cos(_angles[0])*L;      //arm 1, z
		//
		// arm2[0] = (Wb + sin(_angles[1])*L) * cos(2.09333333333); //arm 2, x
		// arm2[1] = (Wb + sin(_angles[1])*L) * sin(2.09333333333); //arm 2, y
		// arm2[2] = cos(_angles[1])*L;                             //arm 2, z
		//
		// arm3[0] = (Wb + sin(_angles[2])*L) * cos(4.18666666667); //arm 3, x
		// arm3[1] = (Wb + sin(_angles[2])*L) * sin(4.18666666667); //arm 3, y
		// arm3[2] = cos(_angles[2])*L;                             //arm 3, z

		arm1[0] = 0;
		arm1[1] = -Wb-L*cos(_angles[0])+Up;
		arm1[2] = -L*sin(_angles[0]);

		arm2[0] = 0.86602540378*(Wb+L*cos(_angles[1]))-Sp/2;
		arm2[1] = 0.5*(Wb+L*cos(_angles[1]))-Wp;
		arm2[2] = -L*sin(_angles[1]);

		arm3[0] = -0.86602540378*(Wb+L*cos(_angles[2]))+Sp/2;
		arm3[1] = 0.5*(Wb+L*cos(_angles[2]))-Wp;
		arm3[2] = -L*sin(_angles[2]);

		float a11 = 2*(arm3[0]-arm1[0]);
		float a12 = 2*(arm3[1]-arm1[1]);
		float a13 = 2*(arm3[2]-arm1[2]);

		float a21 = 2*(arm3[0]-arm2[0]);
		float a22 = 2*(arm3[1]-arm2[1]);
		float a23 = 2*(arm3[2]-arm2[2]);

		float r1 = l;
		float r2 = l;
		float r3 = l;

		float b1 = sq(r1)-sq(r3)-sq(arm1[0])-sq(arm1[1])-sq(arm1[2])+sq(arm3[0])+sq(arm3[1])+sq(arm3[2]);
		float b2 = sq(r2)-sq(r3)-sq(arm2[0])-sq(arm2[1])-sq(arm2[2])+sq(arm3[0])+sq(arm3[1])+sq(arm3[2]);

		// float xa = arm1[0];
		// float ya = arm3[1];
		// float za = arm3[2];
		//
		// return;

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
}

void DeltaArm::moveToAngles(float anglesTemp[3], bool direction[3], float speed[3])
{
		long currentMicros[3] = {}, previousMicros[3] = {};
		int maxSteps = 0, stepCounter[3] = {};
		int stepCount[3] = {};
		bool pulToggle[3] = {};
		byte maxStepsIndex = 0;

		for (byte i = 0; i < 3; i++)
		{
				digitalWrite(dirPin[i], direction[i]); //Set Motor Direction
				stepCount[i] = (anglesTemp[i] / (degreesPerStep[i]/microstep[i]));
				if (maxSteps < stepCount[i])
				{
						maxSteps = stepCount[i]; //calculate step count of longest motor move
						maxStepsIndex = i; //get index of motor with most steps
				}
				Serial.print(i);
				Serial.print(" degrees: ");
				Serial.print(anglesTemp[i]);
				Serial.print(", deg/step: ");
				Serial.print(degreesPerStep[i]/microstep[i]);
				Serial.print(", steps: ");
				Serial.println(stepCount[i]);
		}

		while (maxSteps > stepCounter[maxStepsIndex])
		{
				for (byte i = 0; i < 3; i++)
				{
						if (stepCounter[i] < stepCount[i])
						{
								currentMicros[i] = micros();
								if (currentMicros[i] - previousMicros[i] >= speed[i])
								{
										// Serial.println(i);
										pulToggle[i] = !pulToggle[i];
										digitalWrite(pulPin[i], pulToggle[i]);
										stepCounter[i]++;
										previousMicros[i] = currentMicros[i];
								}
						}
				}
		}
}

void DeltaArm::moveToPosition(float x, float y, float z)
{
		float degrees[3] = {};
		float speed[3] = {4000, 4000, 4000};
		bool direction[3] = {};

		inverseKinematics(x,y,z);

		// Calculate Speed HERE

		for(byte i=0; i<3; i++) {
				degrees[i] = (*angles)[i]-anglesCurrent[i];
				Serial.print(i);
				Serial.print(" | Current: ");
				Serial.print(anglesCurrent[i] * 180/M_PI, 10);
				Serial.print(" | Calculated: ");
				Serial.print((*angles)[i] * 180/M_PI, 10);
				Serial.print(" | Difference: ");
				Serial.print(degrees[i] * 180/M_PI, 10);

				anglesCurrent[i] = degrees[i] + anglesCurrent[i];
				if(degrees[i] <=0) {
						direction[i] = 1;
						degrees[i] = abs(degrees[i])*(180/M_PI);
				}else{
						direction[i] = 0;
						degrees[i] = abs(degrees[i])*(180/M_PI);
				}

				Serial.print(" | New: ");
				Serial.println(anglesCurrent[i] * 180/M_PI, 10);
		}

		moveToAngles(degrees, direction, speed);
}
