/*
void doHalfEightRight();
void doHalfEightLeft();
void doHalfEightStraightRight();
void doHalfEightStraightLeft();
*/

#define W_ROTACION 0.8
#define V_RECTA 0.2

// En V_GIRO y W_GIRO hay que
// mantener una proporcion de
// V/W = R (R = 0.4, radio de curvatura)
#define V_GIRO 0.2
#define W_GIRO 0.5

// correccion para el giro hacia la derecha (del robot)
#define W_GIRO_FIX 0

// V_INC = -V_GIRO para desactivar aceleracion
// W_INC = -W_GIRO para desactivar aceleracion
#define V_INC 100
#define W_INC 100

bool B = false;	// By default, labyrinth A

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
	setSpeed((2*V_RECTA)/3,0,-1,-1);

	while(euclideanDistance(0,0,y,yFinal) > errorDist + 0.005){
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
	v = V_GIRO + V_GIRO/(10);
  w = W_GIRO + W_GIRO/(10);
  setSpeed(v,w,
					 -1,
					 -1);

	xFinal = 0;
	yFinal = 0.8;
	thetaFinal = 0;
	AcquireMutex(semaphore_odometry);
	theta = robot_odometry.th;
	ReleaseMutex(semaphore_odometry);
	while( /*euclideanDistance(x,xFinal,y,yFinal) > errorDist &&*/
				 abs(theta - thetaFinal) > errorTheta) {
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	if(euclideanDistance(x,xFinal,y,yFinal) > errorDist){
		PlaySoundFile("Woops.rso");
	} else {
		PlaySoundFile("wilhelmA.rso");
	}

  // generate 2nd part of trayecto
	v = V_GIRO + V_GIRO/(9.5);
  w = W_GIRO + W_GIRO/(8);
	setSpeed(v,-w,
					-1,
					-1);

	xFinal = 0;
	yFinal = 1.6;
	thetaFinal = -(PI);
	while( euclideanDistance(x,xFinal,y,yFinal) > errorDist &&
				 abs(theta - thetaFinal) > errorTheta) {
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
		if(theta > (PI/2)){
			theta = theta - (2*PI);
		}
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
 * Half eight to the Left
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
	v = V_GIRO + V_GIRO/(9.5);
  w = W_GIRO + W_GIRO/(8);
  setSpeed(v,-w,
					 -1,
					 -1);

	xFinal = 0;
	yFinal = 0.8;
	thetaFinal = -(PI);
	while( euclideanDistance(x,xFinal,y,yFinal) > errorDist &&
				 abs(theta - thetaFinal) > errorTheta) {
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
		if(theta > (PI/2)){
			theta = theta - (2*PI);
		}
	}
	PlaySoundFile("Woops.rso");

  // generate 2nd part of trayecto
	v = V_GIRO + V_GIRO/(10);
  w = W_GIRO + W_GIRO/(10);
	setSpeed(v,w,
					-1,
					-1);

	xFinal = 0;
	yFinal = 1.6;
	thetaFinal = 0;
	AcquireMutex(semaphore_odometry);
	theta = robot_odometry.th;
	ReleaseMutex(semaphore_odometry);
	while( euclideanDistance(x,xFinal,y,yFinal) > errorDist &&
				 abs(theta - thetaFinal) > errorTheta) {
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
