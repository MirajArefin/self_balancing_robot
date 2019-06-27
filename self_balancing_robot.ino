#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define OUTPUT_READABLE_ACCELGYRO

#define LED_PIN 13
bool blinkState = false;

int rm_forward = 4;
int rm_backward = 3;
int lm_forward = 5;
int lm_backward = 6;
int rm_pwm = 7;
int lm_pwm = 2;

int last_error = 0;
int error = 0;

int motor_speed = 0;
int max_motor_speed = 255;
int target_value = 0;

float Kp = 1;
float Kd = 0;

void setup() {
    MPU6050_setup();
    motor_setup();
}

void loop() {
    PID_controller();
}

int sensorRead(){
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
//        Serial.print("AX : ");
//        Serial.print(ax); Serial.print("\n");

    #endif

    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

    return ax;
}

void MPU6050_setup(){
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(38400);

    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    pinMode(LED_PIN, OUTPUT);
}
void motor_setup(){
  pinMode(rm_pwm, OUTPUT);
  pinMode(lm_pwm, OUTPUT);
  pinMode(rm_forward, OUTPUT);
  pinMode(rm_backward, OUTPUT);
  pinMode(lm_forward, OUTPUT);
  pinMode(lm_backward, OUTPUT);
  
}

int PID_controller(){
  ax = sensorRead();
  Serial.println(ax);
  error = ax - target_value;
  int current_motor_speed = Kp * error + Kd * (error - last_error);
  last_error = error;

  
   
  if (current_motor_speed > 0){
    int lm_speed = motor_speed - current_motor_speed;
    int rm_speed = motor_speed - current_motor_speed;
    
//    Serial.println(lm_speed);
    if (lm_speed > max_motor_speed) lm_speed = max_motor_speed;;
    if (rm_speed > max_motor_speed) rm_speed = max_motor_speed;;

    analogWrite(rm_pwm, rm_speed);
    analogWrite(lm_pwm, lm_speed);
    digitalWrite(rm_forward, HIGH);
    digitalWrite(rm_backward, LOW);
    digitalWrite(lm_forward, HIGH);
    digitalWrite(lm_backward, LOW);
  }
  
  else if (current_motor_speed < 0){
    int lm_speed = motor_speed + current_motor_speed;
    int rm_speed = motor_speed + current_motor_speed;
    
    if (lm_speed > max_motor_speed) lm_speed = max_motor_speed;
    if (rm_speed > max_motor_speed) rm_speed = max_motor_speed;

    analogWrite(rm_pwm, rm_speed);
    analogWrite(lm_pwm, lm_speed);
    digitalWrite(rm_forward, LOW);
    digitalWrite(rm_backward, HIGH);
    digitalWrite(lm_forward, LOW);
    digitalWrite(lm_backward, HIGH); 
  }
  
  
}

