#include "commonAux.c"
#include "setSpeed.c"

void doHalfEightRight()
{
  // Destination variables
	float x, y, xFinal, yFinal, theta, thetaFinal;
	float errorTheta = 0.005;
	float errorDist = 0.005;

	// turn 90 degrees on the robot
  float v = 0;
	float w = 1;
	setSpeedBase(v,-w);

	// condicion de parada
	theta = (PI)/2;
	thetaFinal = 0;
	while(abs(theta - thetaFinal) > errorTheta) {
		nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");

  // generate 1st part of trayectory
	v = 0.2;
  w = 0.5;
	setSpeedBase(v,w);

	xFinal = 0;
	yFinal = 0.8;
	thetaFinal = (PI);
	while( euclideanDistance(x,xFinal,y,yFinal) > errorDist &&
				 abs(theta - thetaFinal) > errorTheta) {
		nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");

  // generate 2nd part of trayectory
	setSpeedBase(v,-w);

	xFinal = 0;
	yFinal = 1.6;
	thetaFinal = 0;
	while( euclideanDistance(x,xFinal,y,yFinal) > errorDist &&
				 abs(theta - thetaFinal) > errorTheta) {
	  nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");

	// turn 90 degrees on the robot
  v = 0;
	w = 1;
	// condicion de parada
	theta = 0;
	thetaFinal = (PI)/2;
	setSpeedBase(v,w);
	while(abs(theta - thetaFinal) > errorTheta) {
		nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  	nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");
}

void doHalfEightLeft()
{
  // Destination variables
	float x, y, xFinal, yFinal, theta, thetaFinal;
	float errorTheta = 0.005;
	float errorDist = 0.005;

	// turn 90 degrees on the robot
  float v = 0;
	float w = 1;
	setSpeedBase(v,w);

	// condicion de parada
	theta = (PI)/2;
	thetaFinal = (PI);
	while(abs(theta - thetaFinal) > errorTheta) {
		nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");

  	// generate 1st part of trayectory
	v = 0.2;
  	w = 0.5;
	setSpeedBase(v,-w);

	xFinal = 0;
	yFinal = 0.8;
	thetaFinal = (0);
	while( euclideanDistance(x,xFinal,y,yFinal) > errorDist &&
				 abs(theta - thetaFinal) > errorTheta) {
		nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");

  // generate 2nd part of trayectory

	xFinal = 0;
	yFinal = 1.6;
	thetaFinal = (PI);
	setSpeedBase(v,-w);
	while( euclideanDistance(x,xFinal,y,yFinal) > errorDist &&
				 abs(theta - thetaFinal) > errorTheta) {
	  nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");

  // turn 90 degrees on the robot
  v = 0;
	w = -1;
	// condicion de parada
	theta = (PI)/2;
	thetaFinal = 0;
	setSpeedBase(v,w);
	while(abs(theta - thetaFinal) > errorTheta) {
		nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  	nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");
}

void doHalfEightStraightRight()
{
	// Destination variables
	float x, y, xFinal, yFinal, theta, thetaFinal;
	float errorTheta = 0.005;
	float errorDist = 0.005;

	// turn 90 degrees on the robot
  	float v = 0;
	float w = -1;
	theta = (PI)/2;
	thetaFinal = 0;
	setSpeedBase(v,w);

	// condicion de parada
	while(abs(theta - thetaFinal) > errorTheta) {
		nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");

  	// generate 1st part of trayectory
	v = 0.2;
  	w = 0;
  	xFinal = 0.4;
	yFinal = 0;
	thetaFinal = 0;
	setSpeedBase(v,w);
	while( euclideanDistance(x,xFinal,y,yFinal) > errorDist){
		nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  	nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");

	// turn 90 degrees on the robot
  v = 0;
	w = 1;
	// condicion de parada
	theta = 0;
	thetaFinal = (PI)/2;
	setSpeedBase(v,w);
	while(abs(theta - thetaFinal) > errorTheta) {
		nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  	nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");

  	// generate 2nd part of trayectory
  	v = 0.2;
  	w = 0;
  	// condicion de parada
  	xFinal = 0.4;
	yFinal = 0.8;
	thetaFinal = 0;
	setSpeedBase(v,w);
	while( euclideanDistance(x,xFinal,y,yFinal) > errorDist) {
	  nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");

	// turn 90 degrees on the robot
  v = 0;
	w = 1;
	// condicion de parada
	theta = (PI)/2;
	thetaFinal = (PI);
	setSpeedBase(v,w);

	while(abs(theta - thetaFinal) > errorTheta) {
		nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  	nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");

  // generate 3rd part of trayectory
  	v = 0.2;
  	w = 0;
  	// condicion de parada
  	xFinal = -0.4;
	yFinal = 0.8;
	thetaFinal = (PI);
	setSpeedBase(v,w);
	while( euclideanDistance(x,xFinal,y,yFinal) > errorDist) {
	  nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");

	// turn 90 degrees on the robot
  v = 0;
	w = -1;
	// condicion de parada
	theta = (PI);
	thetaFinal = (PI)/2;
	setSpeedBase(v,w);

	while(abs(theta - thetaFinal) > errorTheta) {
		nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  	nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");

	 // generate 4th part of trayectory
  	v = 0.2;
  	w = 0;
  	// condicion de parada
  	xFinal = -0.4;
	yFinal = 1.6;
	thetaFinal = (PI)/2;
	setSpeedBase(v,w);
	while( euclideanDistance(x,xFinal,y,yFinal) > errorDist) {
	  nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");

	// turn 90 degrees on the robot
  v = 0;
	w = -1;
	// condicion de parada
	theta = (PI)/2;
	thetaFinal = (0);
	setSpeedBase(v,w);

	while(abs(theta - thetaFinal) > errorTheta) {
		nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  	nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");

	 // generate 5th part of trayectory
  	v = 0.2;
  	w = 0;
  	// condicion de parada
  	xFinal = 0;
	yFinal = 1.6;
	thetaFinal = 0;
	setSpeedBase(v,w);
	while( euclideanDistance(x,xFinal,y,yFinal) > errorDist) {
	  nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");

	// turn 90 degrees on the robot
  v = 0;
	w = 1;
	// condicion de parada
	theta = (0);
	thetaFinal = (PI)/2;
	setSpeedBase(v,w);

	while(abs(theta - thetaFinal) > errorTheta) {
		nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  	nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");
}

void doHalfEightStraightLeft()
{
	// Destination variables
	float x, y, xFinal, yFinal, theta, thetaFinal;
	float errorTheta = 0.005;
	float errorDist = 0.005;

	// turn 90 degrees on the robot
  	float v = 0;
	float w = 1;
	theta = (PI)/2;
	thetaFinal = (PI);
	setSpeedBase(v,w);

	// condicion de parada
	while(abs(theta - thetaFinal) > errorTheta) {
		nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");

  	// generate 1st part of trayectory
	v = 0.2;
  	w = 0;
  	xFinal = -0.4;
	yFinal = 0;
	thetaFinal = (PI);
	setSpeedBase(v,w);
	while( euclideanDistance(x,xFinal,y,yFinal) > errorDist){
		nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  	nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");

	// turn 90 degrees on the robot
  v = 0;
	w = -1;
	// condicion de parada
	theta = (PI);
	thetaFinal = (PI)/2;
	setSpeedBase(v,w);
	while(abs(theta - thetaFinal) > errorTheta) {
		nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  	nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");

  	// generate 2nd part of trayectory
  	v = 0.2;
  	w = 0;
  	// condicion de parada
  	xFinal = -0.4;
	yFinal = 0.8;
	thetaFinal = 0;
	setSpeedBase(v,w);
	while( euclideanDistance(x,xFinal,y,yFinal) > errorDist) {
	  nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");

	// turn 90 degrees on the robot
  v = 0;
	w = -1;
	// condicion de parada
	theta = (PI)/2;
	thetaFinal = 0;
	setSpeedBase(v,w);

	while(abs(theta - thetaFinal) > errorTheta) {
		nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  	nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");

  // generate 3rd part of trayectory
  	v = 0.2;
  	w = 0;
  	// condicion de parada
  	xFinal = 0.4;
	yFinal = 0.8;
	thetaFinal = 0;
	setSpeedBase(v,w);
	while( euclideanDistance(x,xFinal,y,yFinal) > errorDist) {
	  nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");

	// turn 90 degrees on the robot
  v = 0;
	w = 1;
	// condicion de parada
	theta = 0;
	thetaFinal = (PI)/2;
	setSpeedBase(v,w);

	while(abs(theta - thetaFinal) > errorTheta) {
		nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  	nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");

	 // generate 4th part of trayectory
  	v = 0.2;
  	w = 0;
  	// condicion de parada
  	xFinal = 0.4;
	yFinal = 1.6;
	thetaFinal = (PI)/2;
	setSpeedBase(v,w);
	while( euclideanDistance(x,xFinal,y,yFinal) > errorDist) {
	  nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");

	// turn 90 degrees on the robot
  v = 0;
	w = 1;
	// condicion de parada
	theta = (PI)/2;
	thetaFinal = (PI);
	setSpeedBase(v,w);

	while(abs(theta - thetaFinal) > errorTheta) {
		nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  	nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");

	 // generate 5th part of trayectory
  	v = 0.2;
  	w = 0;
  	// condicion de parada
  	xFinal = 0;
	yFinal = 1.6;
	thetaFinal = (PI);
	setSpeedBase(v,w);
	while( euclideanDistance(x,xFinal,y,yFinal) > errorDist) {
	  nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");

	// turn 90 degrees on the robot
  v = 0;
	w = -1;
	// condicion de parada
	theta = (PI);
	thetaFinal = (PI)/2;
	setSpeedBase(v,w);

	while(abs(theta - thetaFinal) > errorTheta) {
		nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  	nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");
}
