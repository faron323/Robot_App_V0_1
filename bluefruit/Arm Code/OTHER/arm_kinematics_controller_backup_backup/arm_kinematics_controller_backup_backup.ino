#include "Geometry/Geometry.h"
#include <Servo.h>

Servo gripper_servo;

// IO PINS
const int pul_pin[] = {3, 4,  5,  6,  7};
const int dir_pin[] = {23, 25, 27, 29, 31};
const int ena_pin[] = {22, 24, 26, 28, 30};
const int lim_pin[] = {35, 37, 39, 41, 43};
const int cont_pins[] = {18, 19, 20, 21, 2};

const int max_rot[] = {20, 90, 90, 90, 90};
const int min_rot[] = {-90, -90, -90, -90, -90};
const float max_cart[] = {500,500,824.941,500,500};
const float min_cart[] = {-500,-500,-500,-500,-500};
bool calibration_direction[5] = {0,1,0,0,0};

// Gearing Constants
const float gear_ratio[5] = {10.2,       5,    22.5,  1,    4.39024};
const float step_deg[5] =   {0.17647058, 0.36, 0.16,  1.9,  0.410};
const int microstep[5] =    {8,          8,    8,     16,   16};

// OLD DEGS float step_deg[5] = {0.08823529, 0.27, 0.072, 1.8, 0.38297872};

// IK Initializations
float target[6] = {6.719464414,1.17277E-05,824.941,-90,1.0001,-90.0001};
float initial[6] = {0.000101403,-90.00551392,1.010864267,0.055450441,-0.019782341,-0.055450441,}; //CHECK THIS LAST VALUE
float degree[6]; bool dir[5];

float speed_rads[5] = {};
bool stop = false;


class Link {
public:
float theta, alpha, d, a;

Link(float _theta, float _alpha, float _d, float _a)
		: theta(_theta), alpha(_alpha), d(_d), a(_a){
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

		for(byte i=0; i<depth; i++) {
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

				for (byte i=0; i<_num_links; i++) {
						chain[i]->theta = theta(i) * M_PI/180;
				}

				if (theta(3) < -90) {
						return_matrix(3) += 180;
						return_matrix(4) = -return_matrix(4);
				}else if(theta(3)>90) {
						return_matrix(3) -= 180;
						return_matrix(4) = -return_matrix(4);
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

void move_joints(bool dir[5], float degree[5], float speed_rads[5]){
		long current_micros[5]={}, previous_micros[5]={};
		float speed_period_2[5]={};
		int max_steps=0, step_counter[5]={};
		int steps[5]={};
		bool toggle[5]={};
		byte index=0;

		for(byte i=0; i<5; i++) {
				digitalWrite(dir_pin[i], dir[i]);
				steps[i] = (degree[i]/step_deg[i] * microstep[i])*2;
				if (max_steps < steps[i]) {
						max_steps = steps[i];
						index = i;
				}

				// speed_period_2[i] = (( (speed_rads[i])*180/M_PI) / (gear_ratio[i] * microstep[i]))/2;
				// speed_period_2[i] = (1000/(degree[i] / (speed_rads[i]*180/M_PI)));// * gear_ratio[i]/microstep[i]
				// Serial << i << " spd: " << speed_period_2[i] << "\n";
				// Serial << i << " deg: " << degree[i] << "\n";
		}

		while(max_steps > step_counter[index]) {
				for(byte i=0; i<5; i++) {
						// Serial.println(i);
						if (step_counter[i] < steps[i] and digitalRead(lim_pin[i]) == false) {
								current_micros[i] = micros();
								// if (current_micros[i] - previous_micros[i] >= speed_period_2[i]) {
								if (current_micros[i] - previous_micros[i] >= 2000) {
										toggle[i] = !toggle[i];
										digitalWrite(pul_pin[i], toggle[i]);
										step_counter[i]++;
										previous_micros[i] = current_micros[i];
								}
						}
				}
		}
}

void calibration(Kinematic_Chain<6> k){
		long current_micros[5], previous_micros[5] = {};
		// bool calibration_direction[5] = {0,1,0,0,0};
		float calibration_speed[5] = {2000,2000,1000,2000,2000};
		float calibration_theta[5] = {80,-100,100,90,-80};

		bool stop = false;
		bool toggle[5] = {};

		for(byte i=0; i<5; i++) {
				digitalWrite(dir_pin[i], calibration_direction[i]);
		}

		Serial.println("STARTING CALIBRATION");
		while(stop != true) {
				stop = true;
				for(byte i=0; i<5; i++) {
						if (digitalRead(lim_pin[i]) == 0) {
								stop = false;
						}
				}
				for(byte i=0; i<5; i++) {
						if (digitalRead(lim_pin[i]) == false) {
								current_micros[i] = micros();
								if (current_micros[i] - previous_micros[i] >= calibration_speed[i]) {
										toggle[i] = !toggle[i];
										digitalWrite(pul_pin[i], toggle[i]);
										previous_micros[i] = current_micros[i];
								}
						} else{
								Serial.println(i);
						}
				}
		}

		Serial.println("HOMING");
		bool reverse_direction[5] = {1,0,1,1,1};
		float reverse_degree[5] = {70,115,115,90,90};
		float reverse_speed[5] = {2000,2000,1000,2000,2000};
		int max_steps=0, step_counter[5]={};
		int steps[5]={};
		byte index=0;

		for(byte i=0; i<5; i++) {
				digitalWrite(dir_pin[i], reverse_direction[i]);
				steps[i] = (reverse_degree[i]/step_deg[i] * microstep[i])*2;
				if (max_steps < steps[i]) {
						max_steps = steps[i];
						index = i;
				}
		}
		while(max_steps > step_counter[index]) {
				for(byte i=0; i<5; i++) {
						if (step_counter[i] < steps[i]) {
								current_micros[i] = micros();
								if (current_micros[i] - previous_micros[i] >= reverse_speed[i]) {
										toggle[i] = !toggle[i];
										digitalWrite(pul_pin[i], toggle[i]);
										step_counter[i]++;
										previous_micros[i] = current_micros[i];
								}
						}
				}
		}
		// Serial.println("MOVING TO START");
		//
		// float target_diff[6] = {0,0,100,0,0,0};
		// float speed_rads[6] = {2000,2000,2000,2000,2000,2000};
		// for(byte i=0; i<6; i++) {
		//      target[i] = target[i] - target_diff[i];
		// }
		// Matrix<6> angles = k.inverse_kinematics(target);
		//
		// for(byte i=0; i<5; i++) {
		//      degree[i] = angles(i) - initial[i];
		//      initial[i] = initial[i] + degree[i];
		//
		//      if(degree[i]<0) {
		//              dir[i] = 1;
		//              degree[i] = abs(degree[i]);
		//      }else{
		//              dir[i] = 0;
		//      }
		//
		//      if(degree[i] > max_rot[i]) {
		//              degree[i] = max_rot[i];
		//              Serial.print(i);
		//              Serial.print(" Max rot limited to: ");
		//              Serial.println(max_rot[i]);
		//      }else if(degree[i] < min_rot[i]) {
		//              degree[i] = min_rot[i];
		//              Serial.print(i);
		//              Serial.print(" Min rot limited to: ");
		//              Serial.println(min_rot[i]);
		//      }
		// }
		// move_joints(dir, degree, speed_rads);


		Serial.println("CALIBRATION DONE");
}

Matrix<3,1> cross_prod(Matrix<3,1> x, Matrix<3,1> y){
		Matrix<3,1> ret;
		ret(0,0) = x(1,0)*y(2,0) - x(2,0)*y(1,0);
		ret(1,0) = x(2,0)*y(0,0) - x(0,0)*y(2,0);
		ret(2,0) = x(0,0)*y(1,0) - x(1,0)*y(0,0);
		return ret;
}

// Matrix<6,6> velocities(Kinematic_Chain<6> k){
//      Matrix<4,4> tf[6], fk={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
//      Matrix<3,1> d_0[6], r_0[6], cross[6];
//      Matrix<6,6> joint_velocities, inv_joint_velocities;
//
//      tf[0] = k.chain[0]->get_transformation_matrix();
//      for(byte i=1; i<6; i++) {
//              fk = fk*k.chain[i]->get_transformation_matrix();
//              tf[i] = fk;
//      }
//      d_0[5] = tf[5].Submatrix(Slice<0,3>(), Slice<3,4>());
//
//      for (byte i=0; i<6; i++) {
//              d_0[i] = d_0[5] - tf[i].Submatrix(Slice<0,3>(), Slice<3,4>());
//              r_0[i] = tf[i].Submatrix(Slice<0,3>(), Slice<2,3>());
//              cross[i] = cross_prod(d_0[i], r_0[i]);
//      }
//
//      for(byte j=0; j<3; j++) {
//              for(byte i=0; i<6; i++) {
//                      joint_velocities(j,i) = cross[i](j,0);
//                      joint_velocities(j+3,i) = r_0[i](j);
//              }
//      }
//      inv_joint_velocities = joint_velocities.Inverse();
//
//      return inv_joint_velocities;
// }

Matrix<6,6> velocities(Kinematic_Chain<6> k){
		Matrix<4,4> tf[6];
		Matrix<3,1> d_0[6], r_0[6], cross[6];
		Matrix<6,6> joint_velocities, inv_joint_velocities;

		d_0[5] = k.forward_kinematics(5).Submatrix(Slice<0,3>(), Slice<3,4>());

		for (byte i=0; i<6; i++) {
				tf[i] = k.forward_kinematics(i);
				d_0[i] = d_0[5] - tf[i].Submatrix(Slice<0,3>(), Slice<3,4>());
				r_0[i] = tf[i].Submatrix(Slice<0,3>(), Slice<2,3>());
				cross[i] = cross_prod(d_0[i], r_0[i]);
		}

		for(byte j=0; j<3; j++) {
				for(byte i=0; i<6; i++) {
						joint_velocities(j,i) = cross[i](j,0);
						joint_velocities(j+3,i) = r_0[i](j);
				}
		}
		inv_joint_velocities = joint_velocities.Inverse();

		return inv_joint_velocities;
}

Kinematic_Chain<6> k;
//      theta,          alpha,          d,      a
Link l1(1.74533E-06,    -1.570796327,   220,    0);
Link l2(-1.570796327,   0,              0,      220);
Link l3(-1.553343034,   1.570796327,    0,      0);
Link l4(0,              -1.570796327,   -225,   0);
Link l5(1.74533E-06,    1.570796327,    0,      0);
Link l6(3.141592654,    0,              -170,   0);

void setup(){
		// Serial.begin(2000000);
		k.add_link(l1);
		k.add_link(l2);
		k.add_link(l3);
		k.add_link(l4);
		k.add_link(l5);
		k.add_link(l6);

		for(byte i=0; i<5; i++) {
				pinMode(pul_pin[i], OUTPUT);
				pinMode(dir_pin[i], OUTPUT);
				pinMode(ena_pin[i], OUTPUT);
				if(i<2) {
						digitalWrite(ena_pin[i], HIGH);
				}else{
						digitalWrite(ena_pin[i], LOW);
				}
		}

		for(byte i=0; i<5; i++) {
				pinMode(lim_pin[i], INPUT);
				digitalWrite(lim_pin[i], HIGH);
		}

		// Serial.println("PROGRAM START IN:");
		// for(int i=1; i<4; i++) {
		//      Serial.println(i);
		//      delay(1000);
		// }
		Serial.println("START");
		// calibration(k);

		init_joint_controller();
}

void loop(){
		joint_controller_loop(k);
}

// void interrupt_lim_sw(){
//      // ADD TO SETUP -> attachInterrupt(digitalPinToInterrupt(lim_pin[i]), interrupt_lim_sw, RISING);
//      for(int i=0; i<5; i++) {
//              if (digitalRead(lim_pin[i]) == HIGH) {
//                      limit_switches[i] = HIGH;
//              }
//      }
//      for(byte i=0; i<5; i++) {
//              Serial.println(limit_switches[i]);
//      }
// }

// Inverse KINEMATICS
// IK CONSTANTS
// Matrix<6,6> ret_velocities;
// float theta_velocities[6];
// float cart_velocities[6] = {1,1,1,1,1,1};
//
// long current_micros[5]={}, previous_micros[5]={};
// float speed_period_2[5]={};
// int step_counter[5]={};
// int steps[5]={};
// bool toggle[5]={};
// byte index=0;
// IK MAIN LOOP, IK CONTROLLER CODE DOESNT WORK ATM
// while(stop == false) {
//
//
//      int js_read[4] = {};
//      float js_mult[4] = {2,2,2,2};
//      float target_diff[6] = {0,0,0,0,0,0};
//      bool NAN_flag = 0;
//
//      for(byte i=0; i<4; i++) {
//              js_read[i] = analogRead(i);
//              if (js_read[i]>555 or js_read[i]<500) {
//                      // js_mult[i] = abs(js_read[i]-512)/128;
//
//                      if (js_read[i]<512 and target_diff[i] < max_cart[i]) {
//                              target_diff[i] += js_mult[i];
//                      }else if(js_read[i] >= 512 and target_diff[i] > min_cart[i]) {
//                              target_diff[i] -= js_mult[i];
//                      }
//                      // Serial.println(target_diff[i],10);
//                      // Serial << target_diff[i] << "\n";
//                      for(byte i=0; i<6; i++) {
//                              target[i] = target[i] - target_diff[i];
//                      }
//                      Matrix<6> angles = k.inverse_kinematics(target);
//
//                      ret_velocities = velocities(k);
//
//                      for(byte j=0; j<6; j++) {
//                              theta_velocities[j] = 0;
//                              for(byte i=0; i<6; i++) {
//                                      theta_velocities[j] += ret_velocities(j,i) * cart_velocities[j];
//                              }
//
//                              speed_rads[j] = abs(theta_velocities[j]);
//                              // Serial << j << ": ";
//                              // Serial.println(speed[j],10);
//                      }
//
//
//                      for(byte i=0; i<5; i++) {
//                              degree[i] = angles(i) - initial[i];
//                              initial[i] = initial[i] + degree[i];
//
//                              if(degree[i]<0) {
//                                      dir[i] = 1;
//                                      degree[i] = abs(degree[i]);
//                              }else{
//                                      dir[i] = 0;
//                              }
//
//                              if(degree[i] > max_rot[i]) {
//                                      degree[i] = max_rot[i];
//                                      Serial.print(i);
//                                      Serial.print(" Max rot limited to: ");
//                                      Serial.println(max_rot[i]);
//                              }else if(degree[i] < min_rot[i]) {
//                                      degree[i] = min_rot[i];
//                                      Serial.print(i);
//                                      Serial.print(" Min rot limited to: ");
//                                      Serial.println(min_rot[i]);
//                              }
//                      }
//                      move_joints(dir, degree, speed_rads);
//                      // for(byte i=0; i<5; i++) {
//                      //      digitalWrite(dir_pin[i], dir[i]);
//                      //      steps[i] += (degree[i]/step_deg[i] * microstep[i])*2;
//                      //
//                      //      speed_period_2[i] = ((1/(speed_rads[i])) / (gear_ratio[i] * microstep[i]))*2000/2;
//                      //      Serial << i << ": " << speed_period_2[i] << "\n";
//                      // }
//
//                      // if (NAN_flag = 1) {
//                      // Matrix<5> steps = move_joints(dir, degree, speed_initial, speed_target, acceleration);
//                      // for(byte i=0; i<5; i++) {
//                      // k.chain[i]->theta = steps(i)*step_deg[i]*microstep[i];
//                      // }
//                      // Serial.println("HERE");
//              }
//      }
//      // for(byte i=0; i<5; i++) {
//      //      if (steps[i] > 0) {
//      //              current_micros[i] = micros();
//      //              if (current_micros[i] - previous_micros[i] >= speed_period_2[i]) {
//      //                      Serial.println(i);
//      //                      toggle[i] = !toggle[i];
//      //                      digitalWrite(pul_pin[i], toggle[i]);
//      //                      steps[i]--;
//      //                      previous_micros[i] = current_micros[i];
//      //              }
//      //      }
//      // }
// }

int speed_final[5] =   {500, 1000, 200,  800, 800};
int acceleration[5] =  {7,   7,  7,   7,   7};
bool lock_gripper = false;

void init_joint_controller() {
		// Init Joystick Button Interrupts
		for(int i=0; i<3; i++) {
				pinMode(19+i, INPUT);
				digitalWrite(19+i, HIGH);
				attachInterrupt(digitalPinToInterrupt(19+i), interrupt_toggle_joint, HIGH);
		}
		// Init Joystick Pins
		for(byte i=0; i<6; i++) {
				pinMode(91+i, INPUT);
		}
		//Init Gripper Servo
		gripper_servo.attach(9);
		Serial.println("CONTROLLER INIT");
}

void joint_controller_loop(Kinematic_Chain<6> k) {
		Serial.println("CONTROLLER ENABLED");
		bool toggle[5] = {};
		int servo_angle = 0;
		int js_read[6] = {};
		int step_counter[5] = {};
		int speed_current[5] = {};
		int speed_initial[5] = {};
		int speed_joystick[5] = {};
		long current_micros[5] = {};
		long previous_micros[5] = {};
		long current_millis, previous_millis = 0;
		int servo_refresh_rate = 15;
		bool dir[5] = {};
		float degrees[5] = {}, previous_degrees[5] = {};

		// int cw_limit[5] =  {-3000,   -3000,   -6000,   -3000,   -6000};
		// int ccw_limit[5] = {3000,   3000,   3000,   800,   6000};
		long previous = 0;

		for(int i=0; i<5; i++) {
				speed_initial[i] = speed_final[i] + 2048;
		}

		while(1) {

				for(int i=0; i<6; i++) {
						js_read[i] = analogRead(i);
						if (js_read[i] >540 or js_read[i]<494) {
								if (js_read[i]< 512) {
										digitalWrite(dir_pin[i], 0);
										dir[i] = 0;
								}else{
										digitalWrite(dir_pin[i], 1);
										dir[i] = 1;
								}
								speed_joystick[i] = abs(js_read[i]-512)*4;
								current_micros[i] = micros();

								if (current_micros[i] - previous_micros[i] >= speed_initial[i] - speed_current[i]) {
										// if (digitalRead(lim_pin[i]) == false or dir[i] !=  calibration_direction[i]) {
										// Serial.println(i);
										// Serial << i << ": cur: "<< speed_current[i]<< " joy: "<< speed_joystick[i] <<"\n";
										// degrees[i] = ((step_counter[i]/2) * step_deg[i] / microstep[i])  * M_PI/180;
										// k.chain[i]->theta = k.chain[i]->theta + degrees[i] - previous_degrees[i];
										// previous_degrees[i] = degrees[i];

										// long current = millis();
										// Matrix<4,4> fk = k.forward_kinematics();
										// Matrix <6> position = position_from_matrix(fk);
										// for(byte i=0; i<6; i++) {
										//      Serial << position(i) << "\n";
										// }
										// Serial <<"\n";
										// Serial.println(k.chain[i]->theta * 180/M_PI);

										// if((dir[i] == 0 and step_counter[i] > cw_limit[i])or (dir[i] == 1 and step_counter[i] < ccw_limit[i])) {
										toggle[i] = !toggle[i];
										digitalWrite(pul_pin[i], toggle[i]);

										if (speed_current[i] < speed_joystick[i]) {
												speed_current[i] += acceleration[i];
										} else if (speed_current[i] > speed_joystick[i]) {
												speed_current[i] = speed_joystick[i];
										}
										// if (dir[i] == 0) {
										//      step_counter[i]--;
										// }else{
										//      step_counter[i]++;
										// }

										previous_micros[i] = current_micros[i];
										// }
								}
								// }
						}

						// if (i == 5) {
						//      current_millis = millis();
						//      if (current_millis - previous_millis >= servo_refresh_rate) {
						//              if(lock_gripper == false) {
						//                      servo_angle = map(js_read[5], 0, 1023, 110, 20);
						//                      gripper_servo.write(servo_angle);
						//                      // Serial.println(servo_angle);
						//                      previous_millis = current_millis;
						//              }
						//      }
						// }
				}
		}
}

void interrupt_toggle_joint(){
		static unsigned int previous_millis = 0;
		unsigned int current_millis = millis();

		if(current_millis - previous_millis > 300) {
				for(int i=0; i<3; i++) {
						if (digitalRead(19+i) == 0) {
								switch(i) {
								case 0:
										lock_gripper = !lock_gripper;
										Serial.println("GRIPPER");
										break;
								case 1:
										Serial.println(i);
										break;
								default:
										Serial.println(i);
										break;
								}
						}
				}
		}
		previous_millis = current_millis;
}