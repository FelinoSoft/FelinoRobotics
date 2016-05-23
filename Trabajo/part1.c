/*
void doHalfEightRight();
void doHalfEightLeft();
void doHalfEightStraightRight();
void doHalfEightStraightLeft();
*/

#define W_ROTACION 1
#define V_RECTA 0.2

// En V_GIRO y W_GIRO hay que
// mantener una proporcion de
// V/W = R (R = 0.4, radio de curvatura)
#define V_GIRO 0.2
#define W_GIRO 0.5

// correccion para el giro hacia la derecha (del robot)
#define W_GIRO_FIX 0.02

// V_INC = -V_GIRO para desactivar aceleracion
// W_INC = -W_GIRO para desactivar aceleracion
#define V_INC 10
#define W_INC 10

void moveForward()
{
	// Destination variables
	float y, yFinal;
	float errorDist = 0.005;

	AcquireMutex(semaphore_odometry);
	y = robot_odometry.y;
	ReleaseMutex(semaphore_odometry);
	yFinal = 0;
	// Move
	nxtDisplayTextLine(1, "Moving yolo");
	setSpeed(V_RECTA,0,-1,-1);

	while(euclideanDistance(0,0,y,yFinal) > errorDist){
		nxtDisplayTextLine(2, "%2.2f %2.2f", y, yFinal);
		AcquireMutex(semaphore_odometry);
		y = robot_odometry.y;
		ReleaseMutex(semaphore_odometry);
	}

	// Staph
	setSpeed(0,0,-1,-1);


}


void doHalfEightRight()
{
  // Destination variables
	float v, w, x, y, xFinal, yFinal, theta, thetaFinal;
	float errorTheta = 0.005;
	float errorDist = 0.005;

	// turn 90 degrees on the robot
	nxtDisplayTextLine(1, "1");
	setSpeed(0,-W_ROTACION,-1,-1);

	// condicion de parada
	theta = -(PI/2);
	thetaFinal = -(PI);
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

	setSpeedBase(0,0);

  // generate 1st part of trayectory
	nxtDisplayTextLine(1, "2");
	v = V_GIRO;
  w = W_GIRO;
  setSpeed(v,w,
					 -1,
					 -1);

	xFinal = 0;
	yFinal = 0.8;
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

  // generate 2nd part of trayecto
	nxtDisplayTextLine(1, "3");
	w = W_GIRO + W_GIRO_FIX;
	setSpeed(v,-w,
					-1,
					-1);

	xFinal = 0;
	yFinal = 1.6;
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

	// turn 90 degrees on the robot
  v = 0;
	w = W_ROTACION;
	//setSpeedBase(v,w);
	setSpeed(v,w,-1,-1);

	// condicion de parada
	theta = 0;
	thetaFinal = -(PI/2);

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
	float v, w, x, y, xFinal, yFinal, theta, thetaFinal;
	float errorTheta = 0.005;
	float errorDist = 0.005;

	// turn 90 degrees on the robot
	setSpeed(0,W_ROTACION,-1,-1);

	// condicion de parada
	theta = -(PI/2);
	thetaFinal = 0;
	while(abs(theta - thetaFinal) > errorTheta) {
		nxtDisplayTextLine(3, "1-dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
	  nxtDisplayTextLine(5, "Theta: %2.2f", theta);
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");
	setSpeedBase(0,0);

  // generate 1st part of trayectory
	v = V_GIRO;
  w = W_GIRO + W_GIRO_FIX;
  setSpeed(v,-w,
					 -1,
					 -1);

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

  // generate 2nd part of trayecto
	v = V_GIRO;
  w = W_GIRO;
	setSpeed(v,w,
					-1,
					-1);

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
	w = W_ROTACION;
	//setSpeedBase(v,w);
	setSpeed(v,-w,-1,-1);

	// condicion de parada
	theta = 0;
	thetaFinal = -(PI)/2;

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
	float w = -0.5;
	theta = (PI)/2;
	thetaFinal = 0;
	setSpeed(v,w,v/10,w/10);

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
	setSpeed(v,w,v/10,-1);
	while( euclideanDistance(x,xFinal,0,0) > errorDist){
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
	w = 0.5;
	// condicion de parada
	theta = 0;
	thetaFinal = (PI)/2;
	setSpeed(v,w,-1,w/10);
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
	setSpeed(v,w,v/10,-1);
	while( euclideanDistance(0,0,y,yFinal) > errorDist) {
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
	w = 0.5;
	// condicion de parada
	theta = (PI)/2;
	thetaFinal = (PI);
	setSpeed(v,w,-1,w/10);

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
	setSpeed(v,w,v/10,-1);
	while( euclideanDistance(x,xFinal,0,0) > errorDist) {
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
	w = -0.5;
	// condicion de parada
	theta = (PI);
	thetaFinal = (PI)/2;
	setSpeed(v,w,-1,w/10);

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
	setSpeed(v,w,v/10,-1);
	while( euclideanDistance(0,0,y,yFinal) > errorDist) {
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
	w = -0.5;
	// condicion de parada
	theta = (PI)/2;
	thetaFinal = (0);
	setSpeed(v,w,-1,w/10);

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
	setSpeed(v,w,v/10,-1);
	while( euclideanDistance(x,xFinal,0,0) > errorDist) {
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
	w = 0.5;
	// condicion de parada
	theta = (0);
	thetaFinal = (PI)/2;
	setSpeed(v,w,-1,w/10);

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
