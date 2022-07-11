#include <Wire.h>
#include <KalmanFilter.h>
#include <MPU6050.h>
#include <MsTimer2.h>

int16_t ax, ay, az, gx, gy, gz; // MPU6050 axis data
float   angle_init = -1; // Balance angle

const int OFF_ANGLE      = 40;
const int BALANCED_ANGLE = 0;

/************* Control motor pins *************/
const int MOTOR_RIGHT_1  = 8;
const int MOTOR_RIGHT_2  = 12;
const int PWM_RIGHT      = 10;
const int MOTOR_LEFT_1   = 7;
const int MOTOR_LEFT_2   = 6;
const int PWM_LEFT       = 9;

/************* PID parameters *************/
double kp = 30, ki = 0, kd = 0.6; // anglePWM parameters
double kp_speed = 2.5, ki_speed = 0.01, kd_speed = 0; // speed parameters
int    PD_pwm;  //angle output
float  PI_pwm;
float  pwm = 0;
int    pos_init = 0;
float  v_filter, v_filter_old = 0;
float  e, e_int = 0;

/************* Encoder *************/
const int ENCODER_A = 5;
const int ENCODER_B = 4;
int   pulse_count_A = 0;
int   pulse_count_B = 0;
int   cycle         = 0;
float pos           = 0;
int times = 0;
int new_time = 0;
int valA = 0, valB = 0, flagA = 0, flagB = 0;

MPU6050      mpu6050;
KalmanFilter kf;

void setup() 
{
  //set the control motor’s pin to OUTPUT
  pinMode(MOTOR_RIGHT_1 ,OUTPUT);       
  pinMode(MOTOR_RIGHT_2,OUTPUT);
  pinMode(MOTOR_LEFT_1,OUTPUT);
  pinMode(MOTOR_LEFT_2,OUTPUT);
  pinMode(PWM_LEFT,OUTPUT);
  pinMode(PWM_RIGHT, OUTPUT);

  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);

  // initialize motor values
  digitalWrite(MOTOR_RIGHT_1 , HIGH);       
  digitalWrite(MOTOR_RIGHT_2, LOW);
  digitalWrite(MOTOR_LEFT_1, HIGH);
  digitalWrite(MOTOR_LEFT_2, LOW);
  analogWrite(PWM_LEFT, 0); // range 0 - 255
  analogWrite(PWM_RIGHT, 0); // range 0 - 255

  Wire.begin();                     
  Serial.begin(9600);  // baud rate of 9600             
  delay(1000);
  mpu6050.initialize();  //initialize MPU6050
  delay(500);

  MsTimer2::set(5, interrupt);    // 5 ms per interrupt execution
  MsTimer2::start(); 
}

/**
 * Main loop with a number of serial monitor outputs and
 * encoder counts. 
 */
void loop() 
{
  Serial.print("angle = ");
  Serial.print(kf.angle);
  //Serial.print(" Angle = ");
  //Serial.print(kf.Angle);
  Serial.print(" PD_pwm = ");
  Serial.print(PD_pwm);
  Serial.print(" PI_pwm = ");
  Serial.print(PI_pwm);
  Serial.print(" pwm = ");
  Serial.print(pwm);

  Serial.println("");
  
  //Serial.print("Gyro_x = ");
  //Serial.println(Gyro_x);
  //Serial.print("angle_speed = ");
  //Serial.println(angle_speed);
 
  delay(100);
  
  new_time = times = millis();

  // count pulses from motor encoder
  while((new_time - times) < 100)
  {
    if(digitalRead(ENCODER_A) == HIGH && flagA == 0)
    {
      ++pulse_count_A;
      flagA = 1;
    }
    if(digitalRead(ENCODER_A) == LOW && flagA == 1)
    {
      ++pulse_count_A;
      flagA = 0;
    }

    if(digitalRead(ENCODER_B) == HIGH && flagB == 0)
    {
      ++pulse_count_B;
      flagB = 1;
    }
    if(digitalRead(ENCODER_B) == LOW && flagB == 1)
    {
      ++pulse_count_B;
      flagB = 0;
    }

    new_time = millis();
  }
}

/**
 * Interrupt function captures data from mpu and utilizes Kalman filtering for noise reduction 
 * in determing the car angle, as well as the PI/PD calculations
 */
void interrupt()
{
  sei();  //Allow overall interrupt
  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);     //IIC to get MPU6050 six-axis data ax ay az gx gy gz
  kf.angle_calculate(ax, ay, az, gx, gy, gz, kf.dt, kf.Q_angle, kf.Q_gyro, kf.R_angle, kf.C_0, kf.K1); //get angle and Kalman filtering
  PD_pwm = kp * (kf.angle + angle_init) + kd * kf.angle_speed;  // angle loop of PD control
  angle_PWM();

  if(++cycle >= 8)
  {
    cycle = 0;
    speed_PWM();
  }
}

/**
 * Speed function employs PI control loop 
 */
void speed_PWM()
{
  float v = (pulse_count_A + pulse_count_B) ;
  pulse_count_A = 0;
  pulse_count_B = 0;

  if(pwm < 0)
  {
    v = -v;
  }

  v_filter = (v_filter_old * 0.7) + (v * 0.3);
  v_filter_old = v_filter;

  pos += v_filter;
  pos = constrain(pos, -3550, 3550);

  e_int = (pos_init - pos);
  e = (pos_init - v_filter);
  
  PI_pwm = ki_speed * e_int + kp_speed * e;
}

/**
 * Controls motor speeds and direction based on motor pwms
 */
void angle_PWM()
{
  pwm = constrain(-PD_pwm - PI_pwm, -255, 255);

  if(abs(kf.angle) > OFF_ANGLE)  // stop motors when angle exceeds OFF_ANGLE
  {
    pwm = 0; 
  }

  if(pwm > 0)   
  {
    digitalWrite(MOTOR_LEFT_1, LOW);
    digitalWrite(MOTOR_LEFT_2, HIGH);
    analogWrite(PWM_LEFT, pwm);
    digitalWrite(MOTOR_RIGHT_1, LOW);
    digitalWrite(MOTOR_RIGHT_2, HIGH);
    analogWrite(PWM_RIGHT, pwm);
  }
  else if (pwm < 0)
  {
    digitalWrite(MOTOR_LEFT_1, HIGH);
    digitalWrite(MOTOR_LEFT_2, LOW);
    analogWrite(PWM_LEFT, -pwm);
    digitalWrite(MOTOR_RIGHT_1, HIGH);
    digitalWrite(MOTOR_RIGHT_2, LOW);
    analogWrite(PWM_RIGHT, -pwm);
  }
  else
  {
    analogWrite(PWM_LEFT,  pwm);
    analogWrite(PWM_RIGHT, pwm);
  }
}
