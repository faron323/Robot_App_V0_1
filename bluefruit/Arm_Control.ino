//Joint Num:           0   1   2   3   4
const int pul_pin[] = {4,  3,  5,  6,  7};
const int dir_pin[] = {25, 23, 27, 29, 31};
const int ena_pin[] = {24, 22, 26, 28, 30};

#define pul_pin_1_right 4
#define dir_pin_1_right 25
#define ena_pin_1_right 24 

//Other motor driver in Joint 1 motor pair (currently being used to control motor 0)
//4,25,24 (pulse, direction, enable)

//Pins for motor 0 Driver (currently disabled because driver connection needs repairs)
//2,21,20

//Motor Control Variables
//Joint Num:            0     1     2      3     4
int steps[5] =         {3000, 2000, 10000, 0,    0};    //Number of Steps
bool dir[5] =          {1,    1,    1,     0,    1};    //Direction (0:Clockwise, 1: Anti-CW)
int speed_initial[5] = {2500, 2000, 1000,  5000, 5000};
int speed_target[5] =  {900,  800,  300,   5000, 5000};
int acceleration[5] =  {1,    1,    1,     1,    1};

bool stop = false;

void setup() {
  for(int i=0;i<5;i++){
    pinMode(pul_pin[i], OUTPUT);
    pinMode(dir_pin[i], OUTPUT); 
    pinMode(ena_pin[i], OUTPUT);
    digitalWrite(ena_pin[i], LOW); 
  }
}

void loop() {
  while(stop == false){
      moveJoints(steps, dir, speed_initial, speed_target, acceleration);
    stop = true;
  }
}

void moveJoints(int steps[5], bool dir[5], int speed_initial[5], int speed_target[5], int acceleration[5]){
  double current_micros[5] = {}; 
  double previous_micros[5] = {};
  int speed_current[5] = {};
  int step_counter[5] = {};
  int sustain_steps[5] = {};
  int ramp_steps[5] = {};
  bool toggle[5] = {};
  
  for(int i=0;i<5;i++){
    //Set Directions
    digitalWrite(dir_pin[i], dir[i]);
    if(i == 1){
      digitalWrite(dir_pin_1_right, dir[i]);
    }

    //Calculate Ramping Envelope
    speed_current[i] = speed_initial[i];

    ramp_steps[i] = (speed_initial[i] - speed_target[i]) / acceleration[i];
    sustain_steps[i] = steps[i] - (ramp_steps[i]*2);
  
    if(ramp_steps[i] >= steps[i]/2){
      ramp_steps[i] = steps[i]/2-1;
      sustain_steps[i] = steps[i]-ramp_steps[i]*2;
    }
  }

  while(1){
    
    for(int i=0;i<5;i++){ 
      if (step_counter[i] < steps[i]){        
        current_micros[i] = micros(); 
        if (current_micros[i] - previous_micros[i] >= speed_current[i]){
          toggle[i] = !toggle[i];
          digitalWrite(pul_pin[i], toggle[i]);
          if(i == 1){
            digitalWrite(pul_pin_1_right, toggle[i]);
          }
          
          if(step_counter[i] < ramp_steps[i]){
            speed_current[i] -= acceleration[i];
          }else if((step_counter[i] >= (ramp_steps[i] + sustain_steps[i])-1)){
            speed_current[i] += acceleration[i];
          }
          
          step_counter[i]++;
          previous_micros[i] = current_micros[i];
        }
      }
    }
  }
}
