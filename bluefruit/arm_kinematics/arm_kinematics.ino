#include <Geometry.h>

class Link {
public:
float theta, alpha, d, a;

Link(float _theta, float _alpha, float _d, float _a) : theta(_theta), alpha(_alpha), d(_d), a(_a) {
}

Matrix<4,4> get_transformation_matrix(float theta_new = NULL){

		if (theta_new == NULL) {
				theta_new = theta;
		}

		float cos0 = cos(theta_new);
		float sin0 = sin(theta_new);
		float cosa = cos(alpha);
		float sina = sin(alpha);

		Matrix<4,4> matrix = {cos0, (-sin0) * cosa, sin0 * sina, a * cos0,
				              sin0, cos0 * cosa, (-cos0) * sina, a * sin0,
				              0, sina, cosa, d,
				              0, 0, 0, 1};

		return matrix;
}
};

template<int max_links> class Kinematic_Chain {
unsigned int _num_links = 0;

public:
Link *chain[max_links];

void add_link(Link &l){
		if(_num_links == max_links)
				return;

		chain[_num_links++] =&l;
}

int num_links(){
		return _num_links;
}

Matrix<4,4> forward_kinematics(int depth=6){
		Matrix<4,4> matrix_TEMP,matrix;

		matrix.Fill(0);
		matrix(0,0) = 1;
		matrix(1,1) = 1;
		matrix(2,2) = 1;
		matrix(3,3) = 1;

		for(int i=0; i<depth; i++) {
				matrix_TEMP = matrix*chain[i]->get_transformation_matrix();
				matrix = matrix_TEMP;
		}
		return matrix;
}

Matrix<6> inverse_kinematics(float target[6]){
		Matrix<6> theta,return_matrix;
		Matrix<4,4> remove_rot_0_6, rot_0_3, rot_0_6;
		Matrix<3,3> rot_0_5_3;

		for(byte i=0; i<3; i++) {
				target[i+3] = target[i+3]*M_PI/180;
		}

		rot_0_6 = DH_matrix(target);
		float alpha5 = chain[5]->alpha;
		remove_rot_0_6 = {-1, 0, 0, 0,
				          -0 * cos(alpha5), -1 * cos(alpha5), sin(alpha5), 0,
				          0 * sin(alpha5), 1 * sin(alpha5), cos(alpha5), -chain[5]->d,
				          0, 0, 0, 1};
		Matrix<4,4> rot_0_5 = rot_0_6 * remove_rot_0_6;

		for(byte j=0; j<2; j++) {

				rot_0_3 = forward_kinematics(3);
				Matrix<3,3> rot_0_3_transpose = ~rot_0_3.Submatrix(Slice<0,3>(),Slice<0,3>());
				Matrix<3,3> rot_3_6 = rot_0_5.Submatrix(Slice<0,3>(),Slice<0,3>())*rot_0_3_transpose;

				if (target[0] > 0) {
						if (target[1] > 0) {
								theta(0) = atan(rot_0_5(1,3) / rot_0_5(0,3)) * 180/M_PI;
						}else{
								theta(0) = atan(rot_0_5(1,3) / rot_0_5(0,3)) * 180/M_PI;
						}
				}else{
						if (target[1] > 0) {
								theta(0) = atan(rot_0_5(1,3) / rot_0_5(0,3)) * 180/M_PI + 180;
						}else{
								theta(0) = atan(rot_0_5(1,3) / rot_0_5(0,3)) * 180/M_PI - 180;
						}
				}

				float pX = sqrt(sq(abs(rot_0_5(1,3))) + sq(abs(rot_0_5(0,3))));
				float pY = rot_0_5(2,3) - chain[0]->d;
				float pX_a1 = pX - chain[0]->a;
				float paH = sqrt(sq(pY) + sq(pX_a1));
				float A_deg = atan(pY / pX_a1) * 180/M_PI;
				float B_deg = acos((sq(chain[1]->a) + sq(paH) - sq(chain[3]->d)) / (2 * chain[1]->a * paH)) * 180/M_PI;
				float C_deg = 180 - acos((sq(chain[3]->d) + sq(chain[1]->a) - sq(paH)) / (2 * abs(chain[3]->d) * chain[1]->a)) * 180/M_PI;

				if (pX_a1 < 0) {
						pX_a1 = -pX_a1;
						paH = sqrt(sq(pY) + sq(pX_a1));
						B_deg = atan(pX_a1 / pY) * 180/M_PI;
						A_deg = acos((sq(chain[1]->a) + sq(paH) - sq(chain[3]->d)) / (2 * chain[1]->a * paH)) * 180/M_PI;
						C_deg = 180 - acos((sq(chain[3]->d) + sq(chain[1]->a) - sq(paH)) / (2 * abs(chain[3]->d) * chain[1]->a)) * 180/M_PI;
						float D_deg = 90 - (A_deg + B_deg);
						theta(1) = -180 + D_deg;
				}else{
						theta(1) = -(A_deg + B_deg);
				}

				theta(2) = C_deg;
				theta(4) = atan2(sqrt(1 - sq(rot_3_6(2,2))), (rot_3_6(2,2))) * 180/M_PI;

				if ((chain[4]->theta) <  0 and theta(3) < 0) {
						theta(4) = -atan2(sqrt(1 - sq(rot_3_6(2,2))), (rot_3_6(2,2))) * 180/M_PI;
				}

				if (theta(4) > 0) {
						theta(3) = atan2(rot_3_6(1,2), rot_3_6(0,2)) * 180/M_PI;
						if (rot_3_6(2,1) < 0) {
								theta(5) = atan2(-rot_3_6(2,1), rot_3_6(2,0)) * 180/M_PI - 180;
						}else{
								theta(5) = atan2(-rot_3_6(2,1), rot_3_6(2,0)) * 180/M_PI + 180;
						}
				}else{
						theta(3) = atan2(-rot_3_6(1,2), -rot_3_6(0,2)) * 180/M_PI;

						if (rot_3_6(2,1) < 0) {
								theta(5) = atan2(rot_3_6(2,1), -rot_3_6(2,0)) * 180/M_PI - 180;
						}else{
								theta(5) = atan2(rot_3_6(2,1), -rot_3_6(2,0)) * 180/M_PI + 180;
						}
				}
				return_matrix = theta;
				theta(2) -= 90;
				theta(5) += 180;
				for (int i=0; i<_num_links; i++) {
						chain[i]->theta = theta(i) * M_PI/180;
				}
		}
		return return_matrix;
}
};

Matrix<4,4> DH_matrix(float target[6]){
		float cosA = cos(target[3]);
		float cosB = cos(target[4]);
		float cosG = cos(target[5]);
		float sinA = sin(target[3]);
		float sinB = sin(target[4]);
		float sinG = sin(target[5]);
		float cosBsinA = cosB*sinA;
		float cosAcosB = cosA*cosB;

		Matrix<4,4> matrix = {-(cosA * cosG - cosBsinA * sinG), cosG * sinA + cosAcosB * sinG, sinB * sinG, target[0],
				              cosBsinA * cosG + cosA * sinG, cosAcosB * cosG - sinA * sinG, cosG * sinB, target[1],
				              sinA * sinB, cosA * sinB, -cosB, target[2],
				              0, 0, 0, 1};
		return matrix;
}
void print_matrix(Matrix<4,4> matrix, int decimals = 10){
		for (byte i=0; i<4; i++) {
				for (byte j=0; j<4; j++) {
						Serial.print(matrix(i,j),decimals);
						Serial.print(", ");
				}
				Serial.println("");
		}
}
void print_matrix3(Matrix<3,3> matrix, int decimals = 10){
		for (byte i=0; i<3; i++) {
				for (byte j=0; j<3; j++) {
						Serial.print(matrix(i,j),decimals);
						Serial.print(", ");
				}
				Serial.println("");
		}
}
Matrix<6> position_from_matrix(Matrix<4,4> matrix){
		float x = matrix(0,3);
		float y = matrix(1,3);
		float z = matrix(2,3);
		float pitch = (180/M_PI) * atan2(sqrt(sq(matrix(0,2))+sq(matrix(1,2))),-matrix(2,2));
		float yaw = (180/M_PI) * atan2(matrix(2,0)/pitch,matrix(1,2)/pitch);
		float roll = (180/M_PI) * atan2(matrix(0,2)/pitch,matrix(1,2)/pitch);
		Matrix<6> return_matrix = {x,y,z,yaw,pitch,roll};
		return return_matrix;
}

void setup(){
		Serial.begin(112500);
		//      theta,          alpha,          d,      a
		Link l1(1.74533E-06,    -1.570796327,   220,    0);
		Link l2(-1.570796327,   0,              0,      220);
		Link l3(-1.553343034,   1.570796327,    0,      0);
		Link l4(0,              -1.570796327,   -220,   0);
		Link l5(1.74533E-06,    1.570796327,    0,      0);
		Link l6(3.141592654,    0,              -165,   0);

		Kinematic_Chain<6> k;
		k.add_link(l1);
		k.add_link(l2);
		k.add_link(l3);
		k.add_link(l4);
		k.add_link(l5);
		k.add_link(l6);

		float target[6] ={6.719464414,10,600,-90,1.0001,-90.0001};

		long current = micros();
		Matrix<6> angles = k.inverse_kinematics(target);
		Serial.print("Runtime: ");
		Serial.print(current);
		Serial.println(" us");
		Serial << angles << "\n";
		long previous = current;

		// Matrix<6> angles = k.inverse_kinematics({{100,-20,400,-90,10,-90.0001}});
		// Serial.print("Runtime: ");
		// Serial.print(current - previous);
		// Serial.println(" us");
		// Serial << angles << "\n";

}

void loop(){

}
