#pragma config(Sensor, S2, sonarSensorFrontal, sensorSONAR)
#pragma config(Sensor, S3, lightSensor, sensorLightActive)
#pragma config(Sensor, S4, HTGYRO, sensorAnalogInactive)

//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//
#include "mutexLib.c"
#include "positionLib.c"
#include "common.h"
#include "commonAux.c"
#include "hitechnic-gyro.h"
#include "setSpeed.c"
#include "odometry.c"
#include "mapLib.c"
#include "part1.c"
#include "part2.c"

task main()
{
  /* Prepare for the Race */
  // Start gyroscope calibration
  HTGYROstartCal(HTGYRO);

  // Start tasks
  set_position(robot_odometry, 0, 0.4, -(PI/2));

  StartTask(updateOdometry);
  StartTask(controlSpeed);

  // Move forward until position (0,0)
  moveForward();
  set_position(robot_odometry, 0, 0, -(PI/2));

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
		cellToPos(iniPos, X_START_A, Y_START_A);
		set_position(robot_odometry, iniPos.x, iniPos.y, -(PI/2));
		PlaySoundFile("wilhelmA.rso");
		doPlanning();

	} else {
		//start loading map A
		StartTask(planPathOnTheRoadTask);

		// Do trajectory
  	doHalfEightRight();

  	//wait for map loaded and set position. Start part 1
  	while(!planned){
			// Do nothingu
		}

		// Set position
		cellToPos(iniPos, X_START_B, Y_START_B);
		set_position(robot_odometry, iniPos.x, iniPos.y, -(PI/2));
		PlaySoundFile("wilhelmA.rso");
		doPlanning();
  }

  StopTask(controlSpeed);
  StopTask(updateOdometry);
	Close(hFileHandleOd, nIoResultOd);
	nxtDisplayTextLine(5, "FIN");

  /*
	// Calibrate the gyro, make sure you hold the sensor still
	int x_ini = 1;
	int y_ini = 1;
	int x_end = 15;
	int y_end = 1;

	// Inicializa la malla
	string map_file = "mapa3.txt";
	initConnections();

	if(	loadMap(map_file) ){
	  nxtDisplayTextLine(6, "Mapa loaded ok");
	}else{
	  nxtDisplayTextLine(6, "Mapa NOT loaded");
	}
	HTGYROstartCal(HTGYRO);
	planPath(x_ini, y_ini, x_end, y_end);
	Pos p;
	cellToPos(p,0,0);

	Sleep(1000);

	for(iLoop = 1; iLoop < sizePath; iLoop++){
		go(pathX[iLoop],pathY[iLoop]);

		// Check if hay obstaculo
		if(detectObstacle(pathX[iLoop-1], pathY[iLoop-1])){
			// WTF
			PlaySoundFile("WTF.rso");

			// Okay, vamo a calmarno, voy a eliminar conexion
			deleteConnection(pathX[iLoop-1], pathY[iLoop-1],facingDirection);

			// Ahora replanifico turkey
			planPath(2*pathX[iLoop-1]+1, 2*pathY[iLoop-1]+1, x_end, y_end);

			PlaySoundFile("wilhelmA.rso");

			// Reiniciar bucle
			iLoop = 0;
		}
	}
	*/
}
