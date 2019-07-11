// //Joint Num:           0   1   2
// const int pul_pin[] = {2,  5,  23};  //pulse output pins
// const int dir_pin[] = {3, 6, 27}; //direction output pins
// const int ena_pin[] = {4, 7, 28}; //enable output pins
//
// int speed[] = {2000, 2000, 2000}; // set speed for each motor (lower = faster)
// int steps[] = {10, 0, 0}; //Set number of steps for each motor
// bool direction[] = {0, 0, 0};
//
// void setup() {
//      Serial.begin(115200);
//      Serial.println("HERE");
//      //Init Motor Driver Outputs
//      for(int i=0; i<3; i++) {
//              pinMode(pul_pin[i], OUTPUT);
//              pinMode(dir_pin[i], OUTPUT);
//              pinMode(ena_pin[i], OUTPUT);
//              digitalWrite(ena_pin[i], LOW);//Might have to change LOW to HIGH depending on the drivers your using
//      }
// }
//
// void loop() {
//      bool toggle[3] = {};
//      int step_count[3] = {};
// long current_micros[3] = {};
//      long previous_micros[3] = {};
//
//      for(int i=0; i<3; i++) {
//              digitalWrite(dir_pin[i], direction[i]);
//      }
//
//      while(1) {
//              for(int i=0; i<3; i++) {
//                      current_micros[i] = micros();
//
//                      if(step_count[i] <= steps[i]) {
//                              if (current_micros[i] - previous_micros[i] >= speed[i]) {
//                                      toggle[i] = !toggle[i];
//                                      digitalWrite(pul_pin[i], toggle[i]);
//                                      Serial.println(step_count[i]);
//                                      step_count[i]++;
//                                      previous_micros[i] = current_micros[i];
//                              }
//                      }
//              }
//      }
// }
