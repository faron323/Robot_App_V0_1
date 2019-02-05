
//Joint Num:           0   1   2   3   4
int pul_pin[] = {3,  4,  5,  6,  7};
int dir_pin[] = {23, 25, 27, 29, 31};
const int ena_pin[] = {22, 24, 26, 28, 30};

int speed_initial[5] = {3000, 4000, 2000,  3000, 3000};
int speed_target[5] =  {800,  800, 800,   3000, 3000};
int acceleration[5] =  {15,    15,    15,     15,    15};

void setup() {
  attachInterrupt(digitalPinToInterrupt(21), interrupt_button_handler, LOW);
  for(int i=0;i<5;i++){
    pinMode(pul_pin[i], OUTPUT);
    pinMode(dir_pin[i], OUTPUT); 
    pinMode(ena_pin[i], OUTPUT);
    digitalWrite(ena_pin[i], LOW);
  }
//  Serial.begin(9600);
}

void loop() {
  int js_read[5] = {};
  int speed_current[5] = {};
  int speed_joystick[5] = {};
  int step_counter[5] = {};
  long current_micros[5] = {}; 
  long previous_micros[5] = {};
  bool toggle[5] = {};

  while(1){
    for(int i=0;i<5;i++){
      js_read[i] = analogRead(i);
      if (js_read[i]< 512){
        digitalWrite(dir_pin[i], 0);
      }else{
        digitalWrite(dir_pin[i], 1);
      }
      speed_joystick[i] = abs(js_read[i]-512)*4;
      
      if (js_read[i] >530 or js_read[i]<494){
        current_micros[i] = micros();
        
        if (current_micros[i] - previous_micros[i] >= speed_initial[i] - speed_current[i]){
          toggle[i] = !toggle[i];
          digitalWrite(pul_pin[i], toggle[i]);

          if (speed_current[i] < speed_joystick[i]){
            speed_current[i] += acceleration[i]; 
          } else if (speed_current[i] > speed_joystick[i]){
            speed_current[i] = speed_joystick[i];
          }
          //max speed current = 2048
          //ramp up to 2048
          
          if (js_read[i]< 512){
            step_counter[i]--;
          }else{
            step_counter[i]++;
          }
          previous_micros[i] = current_micros[i];
        } 
      }
    }
  }
}

void interrupt_button_handler(){
  static unsigned int interrupt_previous_millis = 0;
  static bool state = false;
  unsigned int interrupt_millis = millis();
  
  if(interrupt_millis - interrupt_previous_millis > 200){  
    if (state==true){
      pul_pin[2] = 7;
      dir_pin[2] = 31;
      state == false;
    }else{
      pul_pin[2] = 5;
      dir_pin[2] = 27;
      state == true;
    }
    state = !state;
  }
  interrupt_previous_millis = interrupt_millis;
}
