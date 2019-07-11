const int pul_pin = 2;
const int dir_pin = 3;
const int ena_pin = 3;

int speed = 1000;
int steps = 5000;
bool direction = 1;

void setup() {
		Serial.begin(115200);
		//Init Motor Driver Outputs
		pinMode(pul_pin, OUTPUT);
		pinMode(dir_pin, OUTPUT);
		pinMode(ena_pin, OUTPUT);
		digitalWrite(ena_pin, LOW);    //Might have to change LOW to HIGH depending on the drivers your using
}

void loop() {
		bool toggle = false;
		int step_count = 0;
		long current_micros = 0;
		long previous_micros = 0;

		digitalWrite(dir_pin, direction);

		while(1) {
				current_micros = micros();
				if(step_count <= steps) {
						if (current_micros - previous_micros >= speed) {
								toggle = !toggle;
								digitalWrite(pul_pin, toggle);
								Serial.println(step_count);
								step_count++;
								previous_micros = current_micros;
						}
				}
		}
}
