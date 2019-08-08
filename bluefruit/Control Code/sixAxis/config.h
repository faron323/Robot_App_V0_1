#ifndef CONFIG_H
#define CONFIG_H

/*
    Six Axis Arm Configuration File
 */

// Pin Layout 24
const int pulPin[] = {22, 24, 26, 28, 30, 32};
const int dirPin[] = {23, 25, 27, 29, 31, 33};
const int enaPin[] = {22, 24, 26, 28, 30, 32};
const int limPin[] = {16, 17, 18, 19, 20, 21};

// Motor Constants
const float degreesPerStep[6] = {0.08823529, 0.27, 0.072, 1.8, 0.38297872, 0};
const int microstepDivider[6] = {8, 8, 16, 16, 16, 16};

// Invert joystick directions
const bool joystickDirection[5] = {1, 1, 1, 1, 0};

// Constraints
const float cartesianMax[6] = {500, 500, 824.941, 500, 500};
const float cartesianMin[6] = {-500, -500, -500, -500, -500};
const float anglesMax[] = {20, 90, 90, 90, 90, 90};
const float anglesMin[] = {-90, -90, -90, -90, -90, -90};

// Initial Angles
const float anglesCalibrate[6] = {0.0011155378, -90.0000686645, 1.0001525878, -14.3684701919, 0.0279764533, 6.8100128173};

#endif