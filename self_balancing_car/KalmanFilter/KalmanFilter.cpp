#include "KalmanFilter.h"


void KalmanFilter::Kalman_Filter(double angle_m, double gyro_m)
{
	angle += (gyro_m - q_bias) * dt;  //Prior estimate
	angle_err = angle_m - angle;

	Pdot[0] = Q_angle - P[0][1] - P[1][0];  //Differential of azimuth error covariance
	Pdot[1] = -P[1][1];
	Pdot[2] = -P[1][1];
	Pdot[3] = Q_gyro;

	P[0][0] += Pdot[0] * dt;  //The integral of the covariance differential of the prior estimate error
	P[0][1] += Pdot[1] * dt;
	P[1][0] += Pdot[2] * dt;
	P[1][1] += Pdot[3] * dt;

	//Intermediate variable of matrix multiplication
	PCt_0 = C_0 * P[0][0];
	PCt_1 = C_0 * P[1][0];
	//Denominator
	E = R_angle + C_0 * PCt_0;
	//Gain value
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;

	t_0 = PCt_0;  //Intermediate variable of matrix multiplication
	t_1 = C_0 * P[0][1];

	P[0][0] -= K_0 * t_0;  //Posterior estimation error covariance 
	P[0][1] -= K_0 * t_1;
	P[1][0] -= K_1 * t_0;
	P[1][1] -= K_1 * t_1;

	q_bias += K_1 * angle_err;    //Posterior estimation
	angle_speed = gyro_m - q_bias; //The differential value of the output value; work out the optimal angular velocity
	angle += K_0 * angle_err; //Posterior estimation; work out the optimal angle
}

void KalmanFilter::angle_calculate(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, float dt, float Q_angle, float Q_gyro, float R_angle, float C_0, float K1)
{
	Angle = -atan2(ay, az) * (180 / PI);  //Radial rotation angle calculation formula; negative sign is direction processing
	Gyro_x = -gx / 131;   //The X-axis angular velocity calculated by the gyroscope; the negative sign is the direction processing
	Kalman_Filter(Angle, Gyro_x); //KalmanFilter 
}

void KalmanFilter::first_order_filter(float angle_m, float gyro_m)
{
	angleY_one = K1 * angle_m + (1 - K1) * (angleY_one + gyro_m * dt);
}
