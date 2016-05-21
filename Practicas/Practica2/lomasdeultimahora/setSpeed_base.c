//#pragma config(Sensor,  .... )
//                  PLANTILLA P2
//  Make the robot move at a certain speed (linear and angular), so it can drive along an 8
//  Update the robot internal odometry estimation, so it aproximately "knows" where it is all the time
//
#include "mutexLib.c"
#include "positionLib.c"

// ROBOT PARAMETERS
float R = 0.026; // m
float L = 0.128; // m

TPosition robot_odometry;       // WE SHOULD ACCESS THIS VARIABLE with a "semaphore".
TMutex semaphore_odometry = 0;  // Important to initialize to zero!!! Not acquired.
TFileIOResult nIoResult;
TFileHandle hFileHandle;

// converts degrees to radians
float degToRad(float degrees)
{
	return (degrees * (PI)) /180;
}

// normalizes theta
float normTheta(float theta)
{
	while (theta < 0) {
		theta = 2*(PI) + theta;
	}

	while (theta >= (2*(PI))){
		theta = theta - 2*(PI);
	}

	if (theta > (PI)) {
		theta = theta - (2*(PI));
	}

	return theta;
}

float euclideanDistance(float x1, float x2, float y1, float y2){
	return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}

// FUNCTION!!
int setSpeed(float v, float w)
{

	// start the motors so that the robot gets
  // v m/s linear speed and w RADIAN/s angular speed
  float w_r = (L * w + 2 * v)/(2*R);
  float w_l = (2*v - R*w_r)/R;

  //nxtDisplayTextLine(1, "w_r: %2.3f", w_r);
 	//nxtDisplayTextLine(2, "w_l: %2.3f", w_l);

  // parameters of power/speed transfer
  //float mR = 0.171915, mL = 0.1725, nR = 0.0504,	nL = 0.0116;
  float mR = 5.80117, mL = 5.76965, nR = -0.20796,	nL = 0.138482;
  float motorPowerRight, motorPowerLeft;

  // sets the power for both motors
  motorPowerLeft = mL * w_l + nL;
  motorPowerRight = mR * w_r + nR;

  //nxtDisplayTextLine(3, "mPL: %2.3f", motorPowerLeft);
  //nxtDisplayTextLine(4, "mPR: %2.3f", motorPowerRight);

  // checks if calculated power exceeds the motors capacity
  if (motorPowerLeft <= 100 && motorPowerRight <= 100) {
  	hogCPU();
  	motor[motorA] = motorPowerLeft;
  	motor[motorC] = motorPowerRight;
  	releaseCPU();
  	return 1;
	}
	else {

		// too much water
		return 0;
	}

}

// TASK TO BE LAUNCHED SIMULTANEOUSLY to "main"!!
task updateOdometry(){
  float cycle = 0.005; // we want to update odometry every 0.01 s
  float dSl,dSr,dS,dx,dy,dT;
  float x, y, th;
  string lol, odometryFile;
  int nFileSize = 1000; //1 byte each char...
  odometryFile = "odometria.txt";

  Delete(odometryFile, nIoResult);
	CloseAllHandles(nIoResult);
	hFileHandle = 0;

  while (true){

    float timeAux = nPgmTime;
    float timeAux2;

		// read tachometers, and estimate how many m. each wheel has moved since last update
    // RESET tachometer right after to start including the "moved" degrees turned in next iteration

    // locks the cpu to modify the motors power
    // CPU LOCKED
    hogCPU();
    dSl = nMotorEncoder[motorA];
    dSr = nMotorEncoder[motorC];
    nMotorEncoder[motorA] = 0;
    nMotorEncoder[motorC] = 0;
    releaseCPU();
    // CPU RELEASED

    // calculates odometry
    dSl = R * degToRad(dSl);
    dSr = R * degToRad(dSr);

    dS = (dSr + dSl) / 2;
		dT = (dSr - dSl) / L;
		dx = dS * cos(robot_odometry.th + (dT/2));
		dy = dS * sin(robot_odometry.th + (dT/2));

		x = robot_odometry.x + dx;
		y = robot_odometry.y + dy;
		th = normTheta(robot_odometry.th + dT);

		// updates odometry
		AcquireMutex(semaphore_odometry);
		set_position(robot_odometry, x, y, th);
		ReleaseMutex(semaphore_odometry);

   	// show each step on screen and write in a file
		//nxtDisplayTextLine(2, "ODOMETRY NEW VALUE");
  	//nxtDisplayTextLine(3, "x,y: %2.2f %2.2f", robot_odometry.x,robot_odometry.y);
  	//nxtDisplayTextLine(4, "theta: %2.2f ", robot_odometry.th);

  	// saves odometry in a file
  	/*
  	OpenWrite(hFileHandle, nIoResult, odometryFile, nFileSize);
  	PlaySoundFile("Woops.rso");
    StringFormat(lol, "(%2.2f, %2.2f, %2.2f)\n", x, y, th);
    WriteText(hFileHandle, nIoResult, lol);
		Close(hFileHandle, nIoResult);
		*/

	 	// Wait until cycle is completed
   	timeAux2 = nPgmTime;
   	if ((timeAux2 - timeAux) < (cycle * 1000)) {
   		Sleep( (cycle * 1000) - (timeAux2 - timeAux) );
   	}
	}
}

void ejecutarTrayectoria1()
{
	float x, y, xFinal, yFinal, theta, thetaFinal;
	float errorTheta = 0.005;
	float errorDist = 0.005;

	// turn 90 degrees on the robot
  float v = 0;
	float w = 1;
	setSpeed(v,-w);

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
	setSpeed(v,w);

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
	setSpeed(v,-w);

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

  // generate 3rd part of trayectory
	setSpeed(v,-w);
	xFinal = 0;
	yFinal = 0.8;
	thetaFinal = -(PI);
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

  // generate 4th part of trayectory
	setSpeed(v,w);
	xFinal = 0;
	yFinal = 0;
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
}

void ejecutarTrayectoria2()
{
	float x, y, xFinal, yFinal, theta, thetaFinal;
	float errorTheta = 0.008;
	float errorDist = 0.01;

	// turn 90 degrees on the robot
	float v = 0;
	float w = 1;
	setSpeed(v,w);

	// condicion de parada
	theta = 0;
	thetaFinal = (PI)/2;
	//nxtDisplayTextLine(3, "%2.2f",abs(theta-thetaFinal));
	while(abs(theta - thetaFinal) > errorTheta) {
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");

	// generate 1st part of trayectory
	v = 0.2;
	w = 1.33333;
	setSpeed(v,-w);

	xFinal = 0.1061;
	yFinal = 0.1434;
	thetaFinal = degToRad(17);
	while( euclideanDistance(x,xFinal,y,yFinal) > errorDist &&
				 abs(theta - thetaFinal) > errorTheta) {
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
	setSpeed(v,w);
	errorDist = 0.05;

	xFinal = 0.8711;
	yFinal = 0.3773;
	thetaFinal = degToRad(17);
	while( euclideanDistance(x,xFinal,y,yFinal) > errorDist /*&&
				 /*abs(theta - thetaFinal) > errorTheta*/) {
		nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
		nxtDisplayTextLine(4, "x,y: %2.2f %2.2f", x,y);
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");

  // generate 3rd part of trayectory
	v = 0.2;
	w = 0.5;
	setSpeed(v,-w);
	errorDist = 0.01;

	xFinal = 0.8711;
	yFinal = -0.3877;
	thetaFinal = normTheta(degToRad(-197));
	while( euclideanDistance(x,xFinal,y,yFinal) > errorDist &&
				 abs(theta - thetaFinal) > errorTheta) {
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
	setSpeed(v,w);
	errorDist = 0.05;

	xFinal = 0.1061;
	yFinal = -0.1538;
	thetaFinal = normTheta(degToRad(-197));
	while( euclideanDistance(x,xFinal,y,yFinal) > errorDist/* &&
				 abs(theta - thetaFinal) > errorTheta*/) {
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");

	// generate 5th part of trayectory
	v = 0.2;
	w = 1.33333;
	setSpeed(v,-w);
	errorDist = 0.01;

	xFinal = 0;
	yFinal = 0;
	thetaFinal = (PI)/2;
	while( euclideanDistance(x,xFinal,y,yFinal) > errorDist &&
				 abs(theta - thetaFinal) > errorTheta) {
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	PlaySoundFile("Woops.rso");
}

//the program below uses feedback from encoders to determine how much the robot turns.
task main()
{

  //float v,w; //speeds
  //int radio; //Trajectory R
  //int circunf; // L (semi-dist. between robot wheels)

  // config.

  // reset odometry values and motor encoders.
  // resets odometry
  AcquireMutex(semaphore_odometry);
  set_position(robot_odometry, 0, 0, 0/*(PI)/2*/);
	ReleaseMutex(semaphore_odometry);

  // resets motor encoders
	hogCPU();
  nMotorEncoder[motorA] = 0;
  nMotorEncoder[motorC] = 0;
  releaseCPU();

  StartTask(updateOdometry);

  //ejecutarTrayectoria1();
  ejecutarTrayectoria2();

  StopTask(updateOdometry);
}
