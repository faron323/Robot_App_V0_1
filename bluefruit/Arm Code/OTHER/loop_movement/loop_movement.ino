//Joint Num:           0   1   2   3   4
const int pul_pin[] = {3,  4,  5,  6,  7};
const int dir_pin[] = {23, 25, 27, 29, 31};
const int ena_pin[] = {22, 24, 26, 28, 30};

//Direction (0:Clockwise, 1: Anti-CW)
// MODIFY DEGREES TO CHANGE HOW FAR THE ARM WILL ROTATE PER AXIS
// IF LOOP STOPS POWER CYCLE THE ARDUINO

float degree[5] = {40,    40,   50,    45,  0};
bool dir[5] = {0,    1,    0,    0, 0};

int speed_initial[5] = {2000, 2000, 2000,  2000, 2000};
int speed_target[5] =  {1000,  1000,  800,   1000, 1000};
int acceleration[5] =  {1,    1,    1,     1,    1};

bool stop = false;

const float step_deg[5] =   {0.17647058, 0.36, 0.16,  1.9,  0.410};
const int microstep[5] =    {8,          8,    8,     16,   16};

//float step_deg[5] = {0.08823529, 0.27, 0.072, 1.8, 0.38297872
// float step_deg[5] = {0.08823529, 0.18, 0.08, 0.9,  0.205};
// int microstep[5] = {8,8,16,16,16};

void setup() {
//  Serial.begin(2000000);
		for(int i=0; i<5; i++) {
				pinMode(pul_pin[i], OUTPUT);
				pinMode(dir_pin[i], OUTPUT);
				pinMode(ena_pin[i], OUTPUT);
				digitalWrite(ena_pin[i], LOW);
		}
		for(int i=0; i<2; i++) {
				digitalWrite(ena_pin[i], HIGH);
		}
}

void loop() {
		// bool dir[5] = {1,    1,    1,    1,  1};
		// while(stop == false) {
		move_joints(dir, speed_initial, speed_target, acceleration);
		// Serial.print("LOOP");
		delay(3000);
		for(byte i=0; i<5; i++) {
				dir[i] = !dir[i];
		}

		// move_joints(dir, speed_initial, speed_target, acceleration);
		// stop = true;
		// }
}

void move_joints(bool dir[5], int speed_initial[5], int speed_target[5], int acceleration[5]){
		long current_micros[5] = {};
		long previous_micros[5] = {};
		int speed_current[5] = {};
		int step_counter[5] = {};
		int sustain_steps[5] = {};
		int ramp_steps[5] = {};
		bool toggle[5] = {};
		float steps[5] = {};
		float max_steps=0;
		byte index=0;

		for(int i=0; i<5; i++) {
				steps[i] = degree[i]/step_deg[i] * microstep[i];
				if (max_steps < steps[i]) {
						max_steps = steps[i];
						index = i;
				}
		}

		for(int i=0; i<5; i++) {
				//Set Directions
				digitalWrite(dir_pin[i], dir[i]);

				//Calculate Ramping Envelope
				speed_current[i] = speed_initial[i];

				ramp_steps[i] = (speed_initial[i] - speed_target[i]) / acceleration[i];
				sustain_steps[i] = steps[i] - (ramp_steps[i]*2);

				if(ramp_steps[i] >= steps[i]/2) {
						ramp_steps[i] = steps[i]/2-1;
						sustain_steps[i] = steps[i]-ramp_steps[i]*2;
				}
		}

		while(max_steps > step_counter[index]) {

				for(int i=0; i<5; i++) {
						if (step_counter[i] < steps[i]) {
								current_micros[i] = micros();
								if (current_micros[i] - previous_micros[i] >= speed_current[i]) {
										toggle[i] = !toggle[i];
										digitalWrite(pul_pin[i], toggle[i]);

										if(step_counter[i] < ramp_steps[i]) {
												speed_current[i] -= acceleration[i];
										}else if((step_counter[i] >= (ramp_steps[i] + sustain_steps[i])-1)) {
												speed_current[i] += acceleration[i];
										}
										step_counter[i]++;
										previous_micros[i] = current_micros[i];
								}
						}
				}
		}
}