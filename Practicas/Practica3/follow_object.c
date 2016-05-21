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


void search_ball(){
	  int _nblobs;
	  blob_array _blobs;
	  bool _condensed = true;
		bool searching = true;
		setSpeed(0,1);
		while(searching){
			_nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);
			for (int i = 0; i < _nblobs; i++) {
		   if (_blobs[i].colour == GOAL_COLOR){
		  		searching = false;
		  		setSpeed(0,0);
		 	 }
      }
		}
}

float fw(int blob_center_x){
	int d = blob_center_x - 85;
	float w = (d/85) * 2;
	nxtDisplayTextLine(1, "d: %d - w: %.2d",d,w);
	return w;
}

float fv(int a){
	int dA = abs(13000 - a);
	float v = (1 - (dA/13000)) * 0.2;
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
				search_ball();
				_nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);
			}
			NXTCAMgetCenter(_blobs, 0, blob_center_x, blob_center_y);
			blob_area = _blobs[0].size;
			if(abs(blob_area - 13000) <500){
				notInFront = false;
				setSpeed(0,0);
			} else{
				v = fv(blob_area);
				w = fw(blob_center_x);
				setSpeed(v,w);
			}
		}
}

task main ()
{

  // Initialise the camera
	NXTCAMinit(cam);
	search_ball();
	track_ball();


//	while (continueTracking) {

		// Get the blobs from the camera into the array
//		 _nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);
//		 nxtDisplayTextLine(1, "%d", _nblobs);

		 // Select blob of COLOR to be tracked
//		 for (int i = 0; i < _nblobs; i++) {
		  // if _blobs[i]
     // if _blobs[i].color == GOAL_COLOR && ...

//     }


		 // Give v and w to the robot according to distance and offset of the blob
		 // If goal reached: continueTracking =0



//	}

	// Catch the ball?


}
