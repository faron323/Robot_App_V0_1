//Joint Num:           0   1   2   3   4
const int pul_pin[] = {3,  4,  5,  6,  7};
const int dir_pin[] = {23, 25, 27, 29, 31};
const int ena_pin[] = {22, 24, 26, 28, 30};

//Direction (0:Clockwise, 1: Anti-CW)

float degree[5] = {0,    0,    0,    0,  45};
bool dir[5] = {1,    1,    1,    1,  1};

int speed_initial[5] = {2500, 1500, 1000,  1500, 1000};
int speed_target[5] =  {900,  500,  500,   1000, 300};
int acceleration[5] =  {2,    2,    2,     1,    2};

bool stop = false;

float step_deg[5] = {0.08823529, 0.27, 0.072, 1.8, 0.38297872};
int microstep[5] = {8,8,2,16,16};

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
		while(stop == false) {
			move_joints(dir, speed_initial, speed_target, acceleration);
      delay(500);
      for(byte i=0; i<5; i++){
        dir[i] = !dir[i];
      }
      
      move_joints(dir, speed_initial, speed_target, acceleration);
				stop = true;
		}
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
