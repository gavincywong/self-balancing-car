#ifndef KalmanFilter_h
#define KalmanFilter_h

#if defined(ARDUINO) && (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif


class KalmanFilter
{
public:
	void Kalman_Filter(double angle_m, double gyro_m);
	void angle_calculate(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, float dt, float Q_angle, float Q_gyro, float R_angle, float C_0, float K1);
	void first_order_filter(float angle_m, float gyro_m);

	float angle_err;
	float q_bias;     //gyroscope drift 
	float accelz = 0;
	float angle;
	float angleY_one;
	float Angle;
	float angle_speed;
	float Pdot[4] = { 0, 0, 0, 0 };
	float P[2][2] = { { 1, 0 }, { 0, 1 } };
	float PCt_0, PCt_1, E;
	
	float K_0, K_1, t_0, t_1;
	float dt = 0.005; //The value of dt is the filter sampling time.
	float Q_angle = 0.001;  //Covariance of gyroscope noise
	float Q_gyro = 0.003;   //Covariance of gyroscope drift noise
	float R_angle = 0.5;    //Covariance of accelerometer
	char C_0 = 1;
	float K1 = 0.05;  // a function containing the Kalman gain is used to calculate the deviation of the optimal estimate.
	float Gyro_x, Gyro_y, Gyro_z; //Angular velocities for gyroscope calculation
  
};

#endif
