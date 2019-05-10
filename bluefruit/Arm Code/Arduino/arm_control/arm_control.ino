#include "Geometry/Geometry.h"
#include "EEPROM.h"
#include "Servo.h"

Servo gripper_servo;

const int pul_pin[] = {3, 4,  5,  6,  7};
const int dir_pin[] = {23, 25, 27, 29, 31};
const int ena_pin[] = {22, 24, 26, 28, 30};
const int lim_pin[] = {18, 19, 20, 21, 2};

const float cart_max[6] = {500,500,824.941,500,500};
const float cart_min[6] = {-500,-500,-500,-500,-500};
const int angles_max[] = {20, 90, 90, 90, 90};
const int angles_min[] = {-90, -90, -90, -90, -90};

const float velocities_cartesian[6] = {1,1,1,1,1,1};
const float gear_ratio[5] = {10.2,6.58333,27.0835,1,4.7};
const float step_deg[5] = {0.08823529,0.27,0.072,1.8,0.38297872};
const byte microstep[5] = {8,8,16,16,16};

float angles_initial[6] = {0.000101403,-90.00551392,1.010864267,0.055450441,-0.019782341,-0.055450441,};
float angles[6];
float target[6] = {6.719464414,1.17277E-05,824.941,-90,1.0001,-90.0001};
bool lim_sw_state[] = {0, 0, 0, 0, 0};
bool lock_gripper = false;
bool dir[5];

bool oneshot = true;

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

void move(bool dir[5], float angles[5], Matrix<6> speed_2){
		long current_micros[5]={}, previous_micros[5]={};
		int max_steps=0, step_counter[5]={};
		int steps[5]={};
		bool toggle[5]={};
		byte index=0;

		for(byte i=0; i<5; i++) {
				digitalWrite(dir_pin[i], dir[i]);
				steps[i] = (angles[i]/step_deg[i] * microstep[i])*2;
				if (max_steps < steps[i]) {
						max_steps = steps[i];
						index = i;
				}
		}

		signed char sequence[7] = {1,0,-1,0,-1,0,1};
		byte sequence_count = 0;
		int jerk_increment = 1;
		int max_accel = 1000;

		int move_time_micros = steps[index] * speed_2(index);
		float ramp_percent = 0.3;
		float sustain_percent = 1 - ramp_percent*2;
		int count=0, count_2=0;
		int acceleration;
		// Serial << ramp_percent*steps[index]/3 <<"\n";

		while(max_steps > step_counter[index]) {
				if(count < steps[index]) {
						if (count_2 >= ramp_percent*steps[index]/3 && sequence_count != 3) {
								sequence_count++;
								count_2 = 0;
								// Serial.println("1");
						}else if(count_2 >= sustain_percent*steps[index] && sequence_count == 3) {
								sequence_count++;
								count_2 - 0;
								// Serial.println("2");
						}
						count++;
						count_2++;
						acceleration += sequence[sequence_count];
						Serial << sequence_count <<"\n";
				}

				for(byte i=0; i<5; i++) {
						if (step_counter[i] < steps[i]) {
								current_micros[i] = micros();
								if (current_micros[i] - previous_micros[i] >= speed_2(i)) {
										toggle[i] = !toggle[i];
										digitalWrite(pul_pin[i], toggle[i]);
										step_counter[i]++;
										previous_micros[i] = current_micros[i];
								}
						}
				}
		}
}

Matrix<6> speed(){
		Matrix<4,4> tf[6];
		Matrix<3,1> d_0[6], r_0[6], cross[6];
		Matrix<6,6> joint_velocities, inv_joint_velocities;
		Matrix<6> speed_2;
		float speed_radians[6], velocities_angular[6];

		d_0[5] = forward_kinematics(5).Submatrix(Slice<0,3>(), Slice<3,4>());

		for (byte i=0; i<6; i++) {
				tf[i] = forward_kinematics(i);
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

		Invert(joint_velocities);

		for(byte j=0; j<6; j++) {
				velocities_angular[j] = 0;
				for(byte i=0; i<6; i++) {
						velocities_angular[j] += joint_velocities(j,i) * velocities_cartesian[j];
				}
				speed_radians[j] = abs(velocities_angular[j]);
				speed_2(j) = 1000/((step_deg[j]*microstep[j]*M_PI/180)/speed_radians[j])/2;
		}

		return speed_2;
}

void calibration(){
		Matrix<6> calibration_speed = {2000,2000,800,2000,2000};
		long current_micros[5], previous_micros[5] = {};
		bool calibration_direction[5] = {0,1,0,0,1};
		bool toggle[5] = {};
		bool stop = false;

		// Set motor calibration dir
		for(byte i=0; i<5; i++) {
				digitalWrite(dir_pin[i], calibration_direction[i]);
		}

		// Check if limit switches are toggled before prog start
		for(byte i=0; i<5; i++) {
				if (digitalRead(lim_pin[i]) == 1) {
						lim_sw_state[i] = 1;
				}
		}

		// Run to limits
		while(stop != true) {
				stop = true;
				for(byte i=0; i<5; i++) {
						if (lim_sw_state[i] == 0) {
								stop = false;
						}
				}
				for(byte i=0; i<5; i++) {
						if (digitalRead(lim_pin[i]) == true) {
								lim_sw_state[i] = true;
								Serial.println(i);
						}
						if (lim_sw_state[i] == false) {
								Serial << pul_pin[i] << "\n";
								current_micros[i] = micros();
								if (current_micros[i] - previous_micros[i] >= calibration_speed(i)) {
										toggle[i] = !toggle[i];
										digitalWrite(pul_pin[i], toggle[i]);
										previous_micros[i] = current_micros[i];
								}
						}
				}
		}

		// Reverse motors and run to home position
		Serial.println("Homing Arm");
		bool reverse_direction[5] = {1,0,1,1,0};
		// float reverse_degree[5] = {77,88,100,230,120};
		float reverse_degree[5] = {10,0,0,0,0};
		move(reverse_direction, reverse_degree, calibration_speed);
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

Matrix<3,1> cross_prod(Matrix<3,1> x, Matrix<3,1> y){
		Matrix<3,1> ret;
		ret(0,0) = x(1,0)*y(2,0) - x(2,0)*y(1,0);
		ret(1,0) = x(2,0)*y(0,0) - x(0,0)*y(2,0);
		ret(2,0) = x(0,0)*y(1,0) - x(1,0)*y(0,0);
		return ret;
}

void eeprom_store(Kinematic_Chain<6> k){
		// FIX FOR FLOAT VALUES
		// Store Location with EEPROM.write(ADDRESS, VALUE)
		for(int addr=0; addr<6; addr++) {
				Serial.println(k.chain[addr]->theta*100);
				EEPROM.write(addr, k.chain[addr]->theta*100);
		}
}

Matrix<6> eeprom_get(byte reg_sel=0){
		Matrix<6> orientation;

		reg_sel = reg_sel * 6;
		for(int addr=0+reg_sel; addr<6+reg_sel; addr++) {
				orientation(addr) = EEPROM.read(addr)/100;
		}
		return orientation;
}

void init_controller() {
		// Init Joystick Button Interrupts
		for(int i=0; i<3; i++) {
				pinMode(19+i, INPUT);
				digitalWrite(19+i, HIGH);
				attachInterrupt(digitalPinToInterrupt(19+i), interrupt_toggle_gripper, HIGH);
		}
		// Init Joystick Pins
		for(byte i=0; i<6; i++) {
				pinMode(91+i, INPUT);
		}
		//Init Gripper Servo
		gripper_servo.attach(9);
		Serial.println("CONTROLLER INIT");
}

void manual_control() {
		Serial.println("MANUAL MODE");
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

		int acceleration[5] =  {7, 7, 7, 7, 7};
		int speed_final[5] =   {500, 1000, 200, 800, 800};
		int cw_limit[5] =  {-3000, -3000, -3000, -1800, -6000};
		int ccw_limit[5] = {3000, 3000, 3000, 1800, 6000};
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
										previous_micros[i] = current_micros[i];
										Serial.println(step_counter[i]);

										if((dir[i] == 0 and step_counter[i] > cw_limit[i])or (dir[i] == 1 and step_counter[i] < ccw_limit[i])) {
												toggle[i] = !toggle[i];
												digitalWrite(pul_pin[i], toggle[i]);

												if (speed_current[i] < speed_joystick[i]) {
														speed_current[i] += acceleration[i];
												} else if (speed_current[i] > speed_joystick[i]) {
														speed_current[i] = speed_joystick[i];
												}
												if (dir[i] == 0) {
														step_counter[i]--;
												}else{
														step_counter[i]++;
												}
										}
								}
						}

						if (i == 5) {
								current_millis = millis();
								if (current_millis - previous_millis >= servo_refresh_rate) {
										if(lock_gripper == false) {
												servo_angle = map(js_read[5], 0, 1023, 110, 20);
												gripper_servo.write(servo_angle);
												// Serial.println(servo_angle);
												previous_millis = current_millis;
										}
								}
						}
				}
		}
}

void rpi_recieve(){
// Receive target coordinates from RPi

		if (Serial.available() > 0) {

				char receivedChar = Serial.read();
		}
}

Kinematic_Chain<6> k;
//      theta,          alpha,          d,      a
Link l1(1.74533E-06,    -1.570796327,   220,    0);
Link l2(-1.570796327,   0,              0,      220);
Link l3(-1.553343034,   1.570796327,    0,      0);
Link l4(0,              -1.570796327,   -220,   0);
Link l5(1.74533E-06,    1.570796327,    0,      0);
Link l6(3.141592654,    0,              -165,   0);

void setup(){
		Serial.begin(9600);
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
		Serial.println("Start");
}

void loop(){
		while(oneshot == true) {
				float target_difference[6] = {0,0,-5,0,0,0};

				// k.calibration();

				for(byte i=0; i<6; i++) {
						target[i] += target_difference[i];
				}

				Matrix<6> angles_calculated = k.inverse_kinematics(target);
				Matrix<6> speed_2 = k.speed();

				for(byte i=0; i<6; i++) {
						angles[i] = angles_calculated(i) - angles_initial[i];
						angles_initial[i] += angles[i];

						if(angles[i] > angles_max[i]) {
								angles[i] = angles_max[i];
								Serial << i << ": Max rotation limited to " << angles_max[i] << "\n";
						}else if(angles[i] < angles_min[i]) {
								angles[i] = angles_min[i];
								Serial << i << ": Min rotation limited to " << angles_min[i] << "\n";
						}

						if(angles[i]<0) {
								dir[i] = 1;
								angles[i] = abs(angles[i]);
						}else{
								dir[i] = 0;
						}
				}
				Matrix<6> calibration_speed = {800,800,800,800,800};
				k.move(dir, angles, calibration_speed);
				oneshot = false;
		}
}

void interrupt_toggle_gripper(){
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