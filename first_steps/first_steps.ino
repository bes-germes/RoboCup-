#include "functions.h"
#include <Pixy2.h>

Pixy2 pixy;

long long int timer = 0;
bool line_catched = false;
double angle = 0;
double alpha1 = 0.0;
double alpha2 = 0.0;

short speed = 150;

void setup() {
    Serial.begin(115200);
    Serial3.begin(115200);
    delay(4000);     
    Serial3.write(0XA5);
    Serial3.write(0X54);
    delay(4000);
    Serial3.write(0XA5);
    Serial3.write(0X51);

    pinMode(33, OUTPUT);
    pinMode(35, OUTPUT);
    pinMode(37, OUTPUT);
  
    digitalWrite(33, LOW);
    digitalWrite(35, LOW);
    digitalWrite(37, LOW);

    pinMode(22, INPUT_PULLUP);
    pinMode(23, INPUT_PULLUP);
  
    for (int i = 0; i < 4; ++i) {
        pinMode(motors_pwm[i], OUTPUT);
        pinMode(motors_in1[i], OUTPUT);
        pinMode(motors_in2[i], OUTPUT);
    }

    for (int j = 0; j < 4; ++j) { 
        for (int i = 0; i < 6; ++i) { 
            led_angle[j * 6 + i] = (j * 6 + i) * radian(15) - radian(30);
        }
    }
    
    led_angle[0] += radian(360);
    led_angle[1] += radian(360);

    pixy.init();
    updateGyro();

    target = degree;
}
   
void loop() { 
    pixy.ccc.getBlocks();
    if (pixy.ccc.numBlocks) {
       double ball_x = pixy.ccc.blocks[1].m_x; 
       double ball_y = pixy.ccc.blocks[1].m_y;
       angle = -countAngle(ball_x, ball_y);
       Serial.println(angle);
    }

    if (abs(angle) > 0.4 && abs(angle) < 3.15)
        angle += radian(50) * abs(angle) ;

    int btn1 = digitalRead(23);
    int btn2 = digitalRead(22);

    if (btn1 == 0)
        target = degree;
    
    updateGyro();
    int err = target - degree;
    int u = (int(err) - (int((err)) % 180) * 2);
    int err_old = err;  

    if ((abs(-1.0 - updateLed()) > epsilon) && (abs(alpha1) < epsilon))
        alpha1 = updateLed();
    if ((abs(-1.0 - updateLed()) > epsilon) && (abs(alpha1 - updateLed()) > epsilon)) 
        alpha2 = updateLed();

    if ((abs(alpha1) > epsilon) && (abs(alpha2) > epsilon)) {
        angle = (alpha1 + alpha2) / 2.0 + radian(180);
        alpha1 = 0.0;
        alpha1 = 0.0;
        line_catched = true;
    }
        
    moveAngle(angle, speed, u);
    delay(1);
}
