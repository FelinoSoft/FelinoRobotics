#pragma config(Sensor, S2,     sonarSensorFrontal,         sensorSONAR)
#pragma config(Sensor, S4,     sonarSensorLateral,         sensorSONAR)

#include "common.h"
#include "mindsensors-nxtcam.h"
#include "mutexLib.c"
#include "positionLib.c"


/************************************************************************************/
// follow_wall.c
// Track a blob of the selected color until the robot is "close enough"
/************************************************************************************/

// Odometry string
TFileIOResult nIoResult;
TFileHandle hFileHandle;
long nFileSize                   = 40000; //1 byte each char...

TPosition robot_odometry;       // WE SHOULD ACCESS THIS VARIABLE with a "semaphore".
TMutex semaphore_odometry = 0;  // Important to initialize to zero!!! Not acquired.

float R = 0.026; // m
float L = 0.128; // m

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

// TASK TO BE LAUNCHED SIMULTANEOUSLY to "main"!!
task updateOdometry(){
  float cycle = 0.01; // we want to update odometry every 0.01 s
  int step = 20;            // we want to write odometry data each 20 steps
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

int setSpeed(float v, float w)
{

	// start the motors so that the robot gets
	// v m/s linear speed and w RADIAN/s angular speed
	float w_r = (L * w + 2 * v)/(2*R);
	float w_l = (2*v - R*w_r)/R;

  // parameters of power/speed transfer
  float mR = 5.5058, mL = 5.5092, nR = 1.4976,	nL = 1.8269;
  float motorPowerRight, motorPowerLeft;

  // sets the power for both motors
  motorPowerLeft = mL * w_l + nL;
  motorPowerRight = mR * w_r + nR + 1.15;

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

void track_wall()
{
   float distanciaParaParar = 20;
   float distanciaAlMuro = 20;
   float umbral = 0;
   float v = 0.2;
   float w = 0;

   // Para evitar el esmorramiento contra un muro frontal
   while(SensorValue[sonarSensorFrontal] > distanciaParaParar)
   {

   	AcquireMutex(semaphore_odometry);
   	float x = robot_odometry.x;
   	float y = robot_odometry.y;
   	float th = robot_odometry.th;
   	ReleaseMutex(semaphore_odometry);

   	float sval = SensorValue[sonarSensorLateral];
   	string temp;
   	StringFormat(temp, "%.2f", sval);
   	nxtDisplayBigTextLine(2, temp);
   	string temp2;
   	StringFormat(temp2, "%.2f", th);
   	nxtDisplayBigTextLine(5, temp2);

   	// Oscila a una distancia segura de la pared lateral

   	// Debe girar a la derecha porque se esta alejando
	  if (SensorValue[sonarSensorLateral] > distanciaAlMuro + umbral) {
		  	w = -v;
		  	if(th < -0.2){
		  		w = 0;
		  	}
		}
		// Debe girar a la izquierda porque se esta acercando
		else if(SensorValue[sonarSensorLateral] < distanciaAlMuro - umbral) {
		  	w = v;
		  	if(th > 0.2){
		  		w = 0;
		  	}
		} else {
			if(th > 0.2){
				w = -(v*0.9);
			} else if(th < -0.2){
				w = v*0.9;
			} else{
				w = 0;
			}
		}

    setSpeed(v,w);
   }

  // Gira al encontrarse con un muro de frente
  float theta, thetaFinal;
	float errorTheta = 0.005;

	// turn 90 degrees on the robot
  setSpeed(0,0);

	// condicion de parada
	AcquireMutex(semaphore_odometry);
 	float x = robot_odometry.x;
 	float y = robot_odometry.y;
 	float th = robot_odometry.th;
 	ReleaseMutex(semaphore_odometry);
	theta = th;
	thetaFinal = (PI)/2;
	PlaySoundFile("wilhelmA.rso");
	Sleep(1000);
	v = 0;
	w = 1.5;
	setSpeed(v,w);
	while(abs(theta - thetaFinal) > errorTheta) {
		//nxtDisplayTextLine(3, "dist %2.2f", euclideanDistance(x,xFinal,y,yFinal));
	  //nxtDisplayTextLine(4, "Theta: %2.2f", abs(theta - thetaFinal));
		AcquireMutex(semaphore_odometry);
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
	}
}

task main ()
{

	// resets odometry
  AcquireMutex(semaphore_odometry);
  set_position(robot_odometry, 0, 0, 0);
  ReleaseMutex(semaphore_odometry);

  // resets motor encoders
  hogCPU();
  nMotorEncoder[motorA] = 0;
  nMotorEncoder[motorC] = 0;
  releaseCPU();

	StartTask(updateOdometry);
	track_wall();
	StopTask(updateOdometry);
}
