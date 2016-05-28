/*
 *  part1.c
 *  Includes functions to do the first part of the race:
 *  the half eight trajectory
 */

#define W_ROTACION 0.8
#define V_RECTA 0.2

// En V_GIRO y W_GIRO hay que
// mantener una proporcion de
// V/W = R (R = 0.4, radio de curvatura)
#define V_GIRO 0.2
#define W_GIRO 0.5

// correccion para el giro hacia la derecha (del robot)
#define W_GIRO_FIX 0.03

// V_INC = -V_GIRO para desactivar aceleracion
// W_INC = -W_GIRO para desactivar aceleracion
#define V_INC 100
#define W_INC 100

bool B = false;	// By default, labyrinth A

/*
 *  Does the half eight starting on A
 */
void doHalfEightRight()
{
  // Destination variables
	float v, w, x, y, xFinal, yFinal, theta, thetaFinal;
	float errorTheta = 0.005;
	float errorDist = 0.005;

	// turn 90 degrees on the robot
	setSpeed(0,-W_ROTACION,-1,-1);

	// condicion de parada
	theta = -(PI/2);
	thetaFinal = -(PI);
	while(abs(theta - thetaFinal) > errorTheta) {
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
		if(theta >= 0){
			theta = theta - (2*PI);
		}
	}
	PlaySoundFile("Woops.rso");

	setSpeedBase(0,0);

  // generate 1st part of trayectory
	v = V_GIRO; //+ V_GIRO/(10);
  w = W_GIRO;// + W_GIRO/(10);
  setSpeed(v,w, -1, -1);
	Sleep(6350);

  // generate 2nd part of trayecto
	v = V_GIRO; //+ V_GIRO/(9.5);
  w = W_GIRO  + W_GIRO_FIX; //+ W_GIRO/(8);
	setSpeed(v,-w,
					-1,
					-1);
	Sleep(6150);
	AcquireMutex(semaphore_odometry);
	x = robot_odometry.x;
	y = robot_odometry.y;
	ReleaseMutex(semaphore_odometry);
	set_position(robot_odometry, x, y, -(PI));
	// turn 90 degrees on the robot
  v = 0;
	w = W_ROTACION;
	//setSpeedBase(v,w);
	setSpeed(v,w,-1,-1);

	// condicion de parada
	theta = 0;
	thetaFinal = -(PI/2);
	AcquireMutex(semaphore_odometry);
	theta = robot_odometry.th;
	ReleaseMutex(semaphore_odometry);
	while(abs(theta - thetaFinal) > errorTheta) {
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");
	setSpeed(0,0,-1,-1);
}

/*
 *  Does the half eight starting on B
 */
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
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
		if(theta > 0){
			setSpeed(0,-W_ROTACION, -1, -1);
		} else {
			setSpeed(0, W_ROTACION, -1, -1);
		}
	}
	PlaySoundFile("Woops.rso");
	setSpeedBase(0,0);

  // generate 1st part of trayectory
	v = V_GIRO;
  w = W_GIRO + W_GIRO_FIX;
  setSpeed(v,-w,
					 -1,
					 -1);
	Sleep(6250);
	PlaySoundFile("Woops.rso");

  // generate 2nd part of trayecto
	v = V_GIRO;
  w = W_GIRO;
	setSpeed(v,w,
					-1,
					-1);
	Sleep(6250);
	PlaySoundFile("Woops.rso");

	// turn 90 degrees on the robot
	AcquireMutex(semaphore_odometry);
	x = robot_odometry.x;
	y = robot_odometry.y;
	ReleaseMutex(semaphore_odometry);
	set_position(robot_odometry, x, y, 0);
  v = 0;
	w = W_ROTACION;
	//setSpeedBase(v,w);
	setSpeed(v,-w,-1,-1);

	// condicion de parada
	theta = 0;
	thetaFinal = -(PI/2);

	while(abs(theta - thetaFinal) > errorTheta) {
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");
	setSpeed(0,0,-1,-1);
}
