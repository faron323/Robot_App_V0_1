#include <Servo.h>

Servo gripper_servo;

//Joint Num:           0   1   2   3   4
const int pul_pin[] = {3,  4,  5,  6,  7};
const int dir_pin[] = {23, 25, 27, 29, 31};
const int ena_pin[] = {22, 24, 26, 28, 30};

int speed_final[5] =   {500, 1000, 200,  1000, 1000};
int acceleration[5] =  {10,   15,   20,   15,   15};
bool lock_gripper = false;

void setup() {
	//Init Controller Button Interrupts
	for(int i=0; i<3; i++) {
		pinMode(19+i, INPUT);
		digitalWrite(19+i, HIGH);
		attachInterrupt(digitalPinToInterrupt(19+i), interrupt_toggle_joint, HIGH);
	}
	//Init Motor Driver Outputs
	for(int i=0; i<5; i++) {
		pinMode(pul_pin[i], OUTPUT);
		pinMode(dir_pin[i], OUTPUT);
		pinMode(ena_pin[i], OUTPUT);
		digitalWrite(ena_pin[i], LOW);
	}
	//Init Gripper Servo
	gripper_servo.attach(9);
	Serial.begin(2000000);
}

void loop() {
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


	for(int i=0; i<5; i++) {
		speed_initial[i] = speed_final[i] + 2048;
	}

	while(1) {

		for(int i=0; i<6; i++) {
			js_read[i] = analogRead(5-i);
			if (js_read[i] >530 or js_read[i]<494) {
				if (js_read[i]< 512) {
					digitalWrite(dir_pin[i], 0);
				}else{
					digitalWrite(dir_pin[i], 1);
				}
//        Serial.println(i);
				speed_joystick[i] = abs(js_read[i]-512)*4;
				current_micros[i] = micros();

				if (current_micros[i] - previous_micros[i] >= speed_initial[i] - speed_current[i]) {

					toggle[i] = !toggle[i];
					digitalWrite(pul_pin[i], toggle[i]);

					if (speed_current[i] < speed_joystick[i]) {
						speed_current[i] += acceleration[i];
					} else if (speed_current[i] > speed_joystick[i]) {
						speed_current[i] = speed_joystick[i];
					}
					if (js_read[i]< 512) {
						step_counter[i]--;
					}else{
						step_counter[i]++;
					}

					previous_micros[i] = current_micros[i];
				}
			}
			if (i == 5) {
				current_millis = millis();
				if (current_millis - previous_millis >= servo_refresh_rate) {
					if(lock_gripper == false) {
						servo_angle = map(js_read[5], 0, 1023, 110, 20);
						gripper_servo.write(servo_angle);
						Serial.println(servo_angle);
						previous_millis = current_millis;
					}
				}
			}
		}
	}
}

void interrupt_toggle_joint(){
	static unsigned int interrupt_previous_millis = 0;
	unsigned int interrupt_millis = millis();

	if(interrupt_millis - interrupt_previous_millis > 300) {
		for(int i=0; i<3; i++) {
			if (digitalRead(19+i) == 0) {
				switch(i) {
				case 0:
//            Serial.println(i);
					lock_gripper = !lock_gripper;
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
	interrupt_previous_millis = interrupt_millis;
}
