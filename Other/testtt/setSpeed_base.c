//#pragma config(Sensor,  .... )
//
//  Make the robot move at a certain speed (linear and angular), so it can drive along an 8 and an oval
//  Update the robot internal odometry estimation, so it aproximately "knows" where it is all the time
//

/*
 	Autores: Jaime Ruiz-Borau Vizarraga (546751)
					 Alejandro Marquez Ferrer (566400)
					 Alejandro Royo Amondarain (560285)
*/

#include "mutexLib.c"
#include "positionLib.c"

// ROBOT PARAMETERS
float R = 0.026; // m
float L = 0.128; // m

// Odometry string
TFileIOResult nIoResult;
TFileHandle hFileHandle;
long nFileSize                   = 40000; //1 byte each char...

TPosition robot_odometry;       // WE SHOULD ACCESS THIS VARIABLE with a "semaphore".
TMutex semaphore_odometry = 0;  // Important to initialize to zero!!! Not acquired.

// converts degrees to radians
float degToRad(float degrees)
{
	return (degrees * (PI)) /180;
}

// normalizes theta [-pi,pi]
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

// calculates euclidean distance to use it as stop condition
float euclideanDistance(float x1, float x2, float y1, float y2){
	return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}

int setSpeed(float v, float w)
{

	// start the motors so that the robot gets
  // v m/s linear speed and w RADIAN/s angular speed
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

// TASK TO BE LAUNCHED SIMULTANEOUSLY to "main"!!
task updateOdometry(){
  float cycle = 0.01; // we want to update odometry every 0.01 s
  int step = 20;			// we want to write odometry data each 20 steps
  float dSl,dSr,dS,dx,dy,dT;
  float x, y, th;
  string odometryString = "";
  strcat(odometryString, "odometry = ["); // concatenate string2 into string1

  string sFileName = "odometrylog.txt";
  CloseAllHandles(nIoResult);
  //
  // Deletes the file if it already exists
  //
  Delete(sFileName, nIoResult);
  hFileHandle = 0;
  OpenWrite(hFileHandle, nIoResult, sFileName, nFileSize);
  WriteText(hFileHandle, nIoResult, odometryString);
  while (true){
  	// show each step on screen and write in the string
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

		// Write final string into file
		if(step==20){
			step = 0;
			string temp, temp2;
			StringFormat(temp, "%.2f, %.2f,", x, y);
			StringFormat(temp2, "%.2f; \n", th);
			strcat(temp,temp2);
	  	WriteText(hFileHandle, nIoResult, temp);
		}
		step++;

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
		//nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  //nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	nxtDisplayBigTextLine(2, "Tramo 1");

  // generate 1st part of trayectory
	v = 0.2;
  w = 0.5;
	setSpeed(v,w);

	xFinal = 0;
	yFinal = 0.8;
	thetaFinal = (PI);
	while( euclideanDistance(x,xFinal,y,yFinal) > errorDist &&
				 abs(theta - thetaFinal) > errorTheta) {
		//nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  //nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	nxtDisplayBigTextLine(2, "Tramo 2");

  // generate 2nd part of trayectory
	setSpeed(v,-w);

	xFinal = 0;
	yFinal = 1.6;
	thetaFinal = 0;
	while( euclideanDistance(x,xFinal,y,yFinal) > errorDist &&
				 abs(theta - thetaFinal) > errorTheta) {
	  //nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  //nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	nxtDisplayBigTextLine(2, "Tramo 3");

  // generate 3rd part of trayectory
	setSpeed(v,-w);
	xFinal = 0;
	yFinal = 0.8;
	thetaFinal = -(PI);
	while( euclideanDistance(x,xFinal,y,yFinal) > errorDist &&
				 abs(theta - thetaFinal) > errorTheta) {
	  //nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  //nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	nxtDisplayBigTextLine(2, "Tramo 4");

  // generate 4th part of trayectory
	setSpeed(v,w);
	xFinal = 0;
	yFinal = 0;
	thetaFinal = 0;
	while( euclideanDistance(x,xFinal,y,yFinal) > errorDist &&
				 abs(theta - thetaFinal) > errorTheta) {
		//nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  //nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	nxtDisplayBigTextLine(2, "FIN");
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

	while(abs(theta - thetaFinal) > errorTheta) {
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	nxtDisplayBigTextLine(2, "Tramo 1");

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
	nxtDisplayBigTextLine(2, "Tramo 2");

  // generate 2nd part of trayectory
	v = 0.2;
	w = 0;
	setSpeed(v,w);
	errorDist = 0.05;

	xFinal = 0.8711;
	yFinal = 0.3773;
	thetaFinal = degToRad(17);
	while( euclideanDistance(x,xFinal,y,yFinal) > errorDist) {
		nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
		nxtDisplayTextLine(4, "x,y: %2.2f %2.2f", x,y);
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	nxtDisplayBigTextLine(2, "Tramo 3");

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
	nxtDisplayBigTextLine(2, "Tramo 4");

  // generate 4th part of trayectory
	v = 0.2;
	w = 0;
	setSpeed(v,w);
	errorDist = 0.05;

	xFinal = 0.1061;
	yFinal = -0.1538;
	thetaFinal = normTheta(degToRad(-197));
	while( euclideanDistance(x,xFinal,y,yFinal) > errorDist) {
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
	nxtDisplayBigTextLine(2, "Tramo 5");

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
	nxtDisplayBigTextLine(2, "FIN");
}

//the program below uses feedback from encoders to determine how much the robot turns.
task main()
{

	int trayectoria = 1;	// chooses trajectory to run
	float thetaINIT = 0;	// initial theta (different for each trajectory)

	if (trayectoria == 1) {
		thetaINIT = (PI)/2;
	}
	else if (trayectoria == 2) {
		thetaINIT = 0;
  }

	// reset odometry values and motor encoders

  // resets odometry
  AcquireMutex(semaphore_odometry);
  set_position(robot_odometry, 0, 0, thetaINIT);
	ReleaseMutex(semaphore_odometry);

  // resets motor encoders
	hogCPU();
  nMotorEncoder[motorA] = 0;
  nMotorEncoder[motorC] = 0;
  releaseCPU();

  StartTask(updateOdometry);

  // executes the required trajectory
  if (trayectoria == 1) {
		ejecutarTrayectoria1();
	}
	else if (trayectoria == 2) {
		ejecutarTrayectoria2();
  }

  StopTask(updateOdometry);

  Close(hFileHandle, nIoResult);
}
