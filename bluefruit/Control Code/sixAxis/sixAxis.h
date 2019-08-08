#include "libraries/geometry/Geometry.h"

#ifndef SIXAXIS_H
#define SIXAXIS_H

class Link
{
public:
	float theta, alpha, d, a;

	Link(float _theta, float _alpha, float _d, float _a)
		: theta(_theta), alpha(_alpha), d(_d), a(_a)
	{}

	Matrix<4, 4> getTransformationMatrix();
};


class KinematicChain
{
	unsigned int _numLinks = 0;

public:
	Link *chain[6];
	float speedCartesian[6] = {1, 1, 1, 1, 1, 1};

	void addLink(Link &l);
	int numLinks();
	Matrix<4, 4> forwardKinematics(int depth = 6);
	Matrix<6> inverseKinematics(float target[6]);
	Matrix<6> getJointSpeed();
	void moveToAngle(bool moveDirection[6], float moveAngles[6], Matrix<6> speedPeriodMicros);
	void moveToPosition(Matrix<6> targetCartRelative);
	void calibrateChain(float calibrationSpeed[6]);
	void manualControl();
	

private:

	float anglesDifference[6] = {};
	float cartGlobal[6] = {6.719464414, 1.17277E-05, 824.941, -90, 1.0001, -90.0001};
	//Calibration for kinematics to be right at initial position
	float anglesPrevious[6] = {0.0011155378, -90.0000686645, 1.0001525878, -14.3684701919, 0.0279764533, 6.8100128173};

};

Matrix<4, 4> getDHMatrix(float target[6]);
Matrix<3, 1> crossProduct(Matrix<3, 1> x, Matrix<3, 1> y);
Matrix<6> getPositionFromMatrix(Matrix<4, 4> matrix);
void printMatrix4x4(Matrix<4, 4> matrix, int decimals = 10);
void printMatrix3x3(Matrix<3, 3> matrix, int decimals = 10);


#endif
