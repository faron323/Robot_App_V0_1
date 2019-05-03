//Joint Num:           0   1   2
const int pul_pin[] = {3,  4,  5};  //pulse output pins
const int dir_pin[] = {23, 25, 27}; //direction output pins
const int ena_pin[] = {22, 24, 26}; //enable output pins

int speed[] =   {2000, 2000, 2000}; // set speed for each motor (lower = faster)
int steps[] =   {1000, 0, 0}; //Set number of steps for each motor

void setup() {
	//Init Motor Driver Outputs
	for(int i=0; i<3; i++) {
		pinMode(pul_pin[i], OUTPUT);
		pinMode(dir_pin[i], OUTPUT);
		pinMode(ena_pin[i], OUTPUT);
		digitalWrite(ena_pin[i], LOW);
	}
	Serial.begin(2000000);
}

void loop() {
	bool toggle[3] = {};
	int step_count[3] = {};
	long current_micros[3] = {};
	long previous_micros[3] = {};

	while(1) {
		for(int i=0; i<3; i++) {
			if(step_count[i] <= steps[i]) {
				// prints step count per motor to terminal
				Serial.print(i);
				Serial.print(": ");
				Serial.print(step_count[i]);
				Serial.print(" / ");
				Serial.print(steps[i]);
				if (current_micros[i] - previous_micros[i] >= speed[i]) {
					toggle[i] = !toggle[i];
					digitalWrite(pul_pin[i], toggle[i]);
					step_count[i]++;
					previous_micros[i] = current_micros[i];
				}
			}
		}
	}
}
