#ifndef functions_h
#define functions_h

double degree = 0;
double target = 0;
double epsilon = pow(10, -6);
double led_angle[24];
int calibration_value[24];
const double angle_coef = 0.785398163397448;

int motors_in2[] = {38, 42, 28, 5};
int motors_in1[] = {36, 40, 30, 3};
int motors_pwm[] = {6, 44, 10, 12};

double central_x = 149;
double central_y = 114;
double front_x = 235; 
double front_y = 120; 

unsigned char Re_buf[8], counter = 0;

bool setTimer(long long timer, int dt) {
    return(millis() - timer < dt);
}

double radian(double angle) {
    angle = angle * PI / 180;
    return angle;
}

double countAngle(double ball_x, double ball_y) {
    double vec_ax = front_x - central_x; 
    double vec_ay = front_y - central_y; 
    double vec_bx = ball_x - central_x; 
    double vec_by = ball_y - central_y; 
    double angle = atan2(vec_by - vec_ay, vec_bx - vec_ay);
    return angle;
}

void runMotor(byte port, short speed) {
    if (speed > 255) 
        speed = 255;
    if (speed < -255)
        speed = -255;
        digitalWrite(motors_in1[port], speed > 0);
        digitalWrite(motors_in2[port], !(speed > 0));
        analogWrite(motors_pwm[port], abs(speed));
}

void moveAngle(double angle, short speed, int u) {
    runMotor(0, speed * sin(angle + angle_coef) + u); 
    runMotor(1, speed * sin(angle - angle_coef) + u);
    runMotor(2, speed * sin(angle + angle_coef) - u);
    runMotor(3, speed * sin(angle - angle_coef) - u);
}

int readChannel(int n, int m) {
    int control_pins[3] = {33, 35, 37};
    int signal[4] = {A1, A3, A5, A7}; 
    int channels[6][3] = { // номера ножек мультиплексора в двоичном коде 
      {0, 0, 0},
      {1, 0, 0},
      {0, 1, 0},
      {1, 1, 0},
      {0, 0, 1},
      {1, 0, 1}
    };
    for (int i = 0; i < 3; ++i) {
        digitalWrite(control_pins[i], channels[n][i]);
    }
    int value = analogRead(signal[m]);
    return value;
}

double updateLed() {
    for (int j = 0; j < 4; ++j) { 
        for (int i = 0; i < 6; ++i) { 
            if (readChannel(i, j) > calibration_value[j * 6 + i])
                return led_angle[j * 6 + i];
        }
    }
    return -1.0;
}

void updateGyro() {
    Serial3.write(0XA5);
    Serial3.write(0X51); //send it for each read
    while (Serial3.available()) {   
        Re_buf[counter] = (unsigned char)Serial3.read();
        if (counter == 0 && Re_buf[0] != 0xAA) return;       
            ++counter;       
        if(counter == 8) {   
            counter = 0;                 
            if (Re_buf[0] == 0xAA && Re_buf[7] == 0x55) {  // data package is correct        
                   degree = (int16_t)(Re_buf[1]<<8|Re_buf[2]) / 100.00;   
            }     
        } 
    }
}

#endif
