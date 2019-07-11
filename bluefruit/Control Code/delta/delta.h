#ifndef DELTA_h
#define DELTA_h

class DeltaArm
{
public:
float anglesCurrent[3];

DeltaArm(float (*anglesTemp)[3], float Sb, float Sp, float L, float l, float h);

void initialize(int _pulPins[3],int _dirPins[3],int _enaPins[3], int _limPins[3], float _degreesPerStep[3], float _microstep[3]);
void setLimits(float upperLimits[3], float lowerLimits[3]);
void inverseKinematics(float x, float y, float z);
void equalZForwardKinematics(float _angles[3]);
void forwardKinematics(float _angles[3]);
void moveToAngles(float angles[3], bool direction[3], float speed[3]);
void moveToPosition(float x, float y, float z);


private:
float (*angles)[3];

float a;
float b;
float c;

float Sb;
float Sp;
float L;
float l;
float h;

float Wb;
float Ub;
float Wp;
float Up;

// Limits in radians
float upperLimits[3] = {0,0,0};
float lowerLimits[3] = {3.14159265359,3.14159265359,3.14159265359};

int pulPin[3];
int dirPin[3];
int enaPin[3];
int limPin[3];

float degreesPerStep[3];
float microstep[3];

};
#endif
