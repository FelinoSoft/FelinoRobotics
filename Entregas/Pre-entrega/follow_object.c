#pragma config(Sensor, S1,     cam,                 sensorI2CCustomFastSkipStates)
//*!!Sensor,    S1,                  cam, sensorI2CCustomFast,      ,            !!*//
// DEFINE CAMERA SENSOR PORT


// CONFIG camera color position
#define BLUE 1
#define RED 0

// CONFIG GOAL PARAMETERS
#define GOAL_COLOR RED
#define AREA_COLOR 100



#include "common.h"
#include "mindsensors-nxtcam.h"


/************************************************************************************/
// follow_object.c
// Track a blob of the selected color until the robot is "close enough"
/************************************************************************************/

float R = 0.026; // m
float L = 0.128; // m

int goalArea = 9500;
int areaError = 500;
float cameraCenter = 85;
float centerError = 5;

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
  motorPowerRight = mR * w_r + nR + 1.15;

  // checks if calculated power exceeds the motors capacity
  if (motorPowerLeft <= 80 && motorPowerRight <= 80) {
  	hogCPU();
  	motor[motorA] = motorPowerLeft;
  	motor[motorC] = motorPowerRight;
  	releaseCPU();
  	return 1;
	}
	else {

		// too much power, sets the power to the maximum possible
		hogCPU();
  	motor[motorA] = 80;
  	motor[motorC] = 80;
  	releaseCPU();
		return 0;
	}

}

void search_ball(){
	  int _nblobs;
	  blob_array _blobs;
	  bool _condensed = true;
		bool searching = true;
		setSpeed(0,1.2);
		while(searching){
			_nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);
			for (int i = 0; i < _nblobs; i++) {
		   if (_blobs[i].colour == GOAL_COLOR){
		  		searching = false;
		 	 }
      }
		}
}

void center_ball(){
  int _nblobs;
  blob_array _blobs;
  bool _condensed = true;
	bool notInCenter = true;
	int blob_center_x = 0;
	int blob_center_y = 0;

	setSpeed(0,1.2);
	while(notInCenter){
		nxtDisplayTextLine(1, "Centering");
		_nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);

		if(_nblobs == 0){
			search_ball();
//			_nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);
//			setSpeed(0,1.2);
		} else{
				NXTCAMgetCenter(_blobs, 0, blob_center_x, blob_center_y);

				if ( abs(blob_center_x - cameraCenter) < centerError ) {
					PlaySoundFile("Woops.rso");
					notInCenter = false;
				}
		}

	}
}

float fw(int blob_center_x){
	float d = blob_center_x - cameraCenter;
	float w = -(d/(cameraCenter))*1.2;
	nxtDisplayTextLine(1, "d: %d - w: %.2f",d,w);
	return w;
}

float fv(int blob_area){
	int dA = abs(goalArea - blob_area);
	float v = 0.2;
	if (dA > 4000) v = 0.4;
	//float v = (1 - (dA/goalArea)) * 0.2;
	nxtDisplayTextLine(2, "a: %d - v: %.2f",dA, v);
	return v;
}

void track_ball(){
	  int _nblobs;
	  blob_array _blobs;
	  bool _condensed = true;
		bool notInFront = true;
		int blob_area = 0;
		int blob_center_x = 0;
		int blob_center_y = 0;
		int v = 0;
		int w = 0;
		while(notInFront){
			_nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);
			if(_nblobs == 0){
				PlaySoundFile("wilhelmA.rso");
				search_ball();
				center_ball();
				//_nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);
			} else{
				NXTCAMgetCenter(_blobs, 0, blob_center_x, blob_center_y);
				blob_area = _blobs[0].size;
				if(abs(blob_area - goalArea) < areaError){
					notInFront = false;
				} else{
					v = fv(blob_area);
					w = fw(blob_center_x);
					setSpeed(0.2,w);
				}
			}
		}
}

void catch_ball()
{
	float armPower = 30;

	// Retracts the arm
//	hogCPU();
//	motor[motorB] = armPower;
//	releaseCPU();

//	Sleep(100);

	// Extends arm to catch the ball

	hogCPU();
	motor[motorB] = -armPower;
	releaseCPU();
	setSpeed(0,0);

	Sleep(500);

	hogCPU();
	motor[motorB] = 0;
	releaseCPU();
}

task main ()
{

  // Initialise the camera
	NXTCAMinit(cam);

	search_ball();
	center_ball();
	track_ball();

	// Catch the ball
	catch_ball();
	PlaySoundFile("wilhelmA.rso");
	Sleep(1500);
}
