#pragma config(Sensor, S2,     sonarSensorFrontal,         sensorSONAR)
#pragma config(Sensor, S4,     sonarSensorLateral,         sensorSONAR)

#include "common.h"
#include "mindsensors-nxtcam.h"


/************************************************************************************/
// follow_object.c
// Track a blob of the selected color until the robot is "close enough"
/************************************************************************************/

float R = 0.026; // m
float L = 0.128; // m

int setSpeed(float v, float w)
{

	// start the motors so that the robot gets
	// v m/s linear speed and w RADIAN/s angular speed

//	float w_r = (v/R) + (L*w)/(2.0*R);
//	float w_l = (v/R) - (L*w)/(2.0*R);
	float w_r = (L * w + 2 * v)/(2*R);
	float w_l = (2*v - R*w_r)/R;

  // parameters of power/speed transfer
  float mR = 5.5058, mL = 5.5092, nR = 1.4976,	nL = 1.8269;
  //float mR = 5.80117, mL = 5.76965, nR = -0.20796,	nL = 0.138482;
  float motorPowerRight, motorPowerLeft;

  // sets the power for both motors
  motorPowerLeft = mL * w_l + nL;
  motorPowerRight = mR * w_r + nR;

  // checks if calculated power exceeds the motors capacity
  if (motorPowerLeft <= 100 && motorPowerRight <= 100) {
  	hogCPU();
  	motor[motorA] = motorPowerLeft;
  	motor[motorC] = motorPowerRight;
  	releaseCPU();
  	return 1;
	}
	else {

		// too much power
		return 0;
	}

}

void track_wall(){
   int distanciaParaParar = 10;
   int distanciaAlMuro = 6;
   int umbral = 1;
   float v = 0.2;
   float w = 0;
   // Para evitar el esmorramiento contra un muro frontal
   while(SensorValue[sonarSensorFrontal] > distanciaParaParar)
   {
	  if(SensorValue[sonarSensorLateral] > distanciaAlMuro + umbral)
		  w = v/2;
	  if(SensorValue[sonarSensorLateral] < distanciaAlMuro - umbral)
		  w = -(v/2);
      setSpeed(v,w);
   }
}

task main ()
{
	track_wall();
}
