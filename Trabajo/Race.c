#pragma config(Sensor, S1, cam, sensorI2CCustomFastSkipStates)
#pragma config(Sensor, S2, sonarSensorFrontal, sensorSONAR)
#pragma config(Sensor, S3, lightSensor, sensorLightActive)
#pragma config(Sensor, S4, HTGYRO, sensorAnalogInactive)

//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//
#include "mutexLib.c"
#include "positionLib.c"
#include "common.h"
#include "commonAux.c"
#include "hitechnic-gyro.h"
#include "mindsensors-nxtcam.h"
#include "setSpeed.c"
#include "odometry.c"
#include "mapLib.c"
#include "part1.c"
#include "part2.c"
#include "part3.c"

task main()
{
  /* Prepare for the Race */
  // Start gyroscope calibration
  HTGYROstartCal(HTGYRO);

  // Start tasks
  set_position(robot_odometry, 1.4, 1, (PI/2));
  //set_position(robot_odometry, 0, 0.4, -(PI/2));

  StartTask(updateOdometry);
  StartTask(controlSpeed);

  // Move forward until position (0,0)
  //moveForward();
  //set_position(robot_odometry, 0, 0, -(PI/2));

  // Get kind of labyrinth
  B = SensorValue[lightSensor] < 35;

  Pos iniPos;

  // Lets do the eight bitches
  if(B){
  	//start loading map	B
  	StartTask(planPathOnTheRoadTask);

  	// Do trajectory
		doHalfEightLeft();
		//wait for map loaded and set position. Start part 2
		while(!planned){
			// Do nothingu
		}

		// Set position
		cellToPos(iniPos, (X_START_B-1)/2, (Y_START_B-1)/2);
		set_position(robot_odometry, iniPos.x, iniPos.y, -(PI/2));
		doPlanning();

	} else {
		//start loading map A
		StartTask(planPathOnTheRoadTask);
/*
		// Do trajectory
  	doHalfEightRight();

  	//wait for map loaded and set position. Start part 1
  	while(!planned){
			// Do nothingu
		}

		// Set position
		cellToPos(iniPos, (X_START_A-1)/2, (Y_START_A-1)/2);
		set_position(robot_odometry, iniPos.x, iniPos.y, -(PI/2));
		doPlanning();
*/
		// Time to search balls
		startPart3();
  }

  // Stops tasks and closes file handlers
  StopTask(controlSpeed);
  StopTask(updateOdometry);
	Close(hFileHandleOd, nIoResultOd);
	nxtDisplayTextLine(5, "FIN");
}
