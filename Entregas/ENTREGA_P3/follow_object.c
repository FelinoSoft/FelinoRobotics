#pragma config(Sensor, S1,     cam,                 sensorI2CCustomFastSkipStates)
//*!!Sensor,    S1,                  cam, sensorI2CCustomFast,      ,            !!*//
// DEFINE CAMERA SENSOR PORT


// CONFIG camera color position
#define BLUE 1
#define RED 0

// CONFIG GOAL PARAMETERS
#define GOAL_COLOR RED
#define AREA_COLOR 100

#include "mutexLib.c"
#include "positionLib.c"

#include "common.h"
#include "mindsensors-nxtcam.h"

/************************************************************************************/
// follow_object.c
// Track a blob of the selected color until the robot is "close enough"
/************************************************************************************/

// Odometry string
TFileIOResult nIoResult;
TFileHandle hFileHandle;
long nFileSize                   = 10000; //1 byte each char...

TPosition robot_odometry;       // WE SHOULD ACCESS THIS VARIABLE with a "semaphore".
TMutex semaphore_odometry = 0;  // Important to initialize to zero!!! Not acquired.

float R = 0.026; // m
float L = 0.128; // m
int lastSeen = 1;

int goalArea = 11000;
int areaError = 500;
float cameraCenter = 85;
float centerError = 5;

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
  motorPowerRight = mR * w_r + nR + 1.15;

  // checks if calculated power exceeds the motors capacity
  if (motorPowerLeft <= 80 && motorPowerRight <= 80) {
  	hogCPU();
  	motor[motorA] = motorPowerLeft;
  	motor[motorC] = motorPowerRight;
  	releaseCPU();
  	return 1;
	}
	else {

		// too much power, sets the power to the maximum possible
		hogCPU();
  	motor[motorA] = 80;
  	motor[motorC] = 80;
  	releaseCPU();
		return 0;
	}

}

/* El robot busca la pelota en el sitio dando vueltas sobre si mismo*/
void search_ball(){
	  int _nblobs;
	  blob_array _blobs;
	  bool _condensed = true;
		bool searching = true;
		setSpeed(0,1.2*lastSeen);

		// Mientras la camara no detecte un blob del color de
		// la pelota, el robot sigue girando
		while(searching){
			_nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);
			for (int i = 0; i < _nblobs; i++) {

				// Si se encuentra la pelota se ha terminado de buscar
		   	if (_blobs[i].colour == GOAL_COLOR){
		   		searching = false;
		 	 	}
      }
		}
}

/* El robot ha encontrado la pelota, pero tiene que girar hasta encontrarla
 en el centro de la camara */
void center_ball(){
  int _nblobs;
  blob_array _blobs;
  bool _condensed = true;
	bool notInCenter = true;
	int blob_center_x = 0;
	int blob_center_y = 0;

	// Mientras el robot no haya enfilado la pelota hay que seguir girando en la
	// direccion de la ultima posicion conocida de la pelota
	setSpeed(0,1.2*lastSeen);
	while(notInCenter){
		nxtDisplayTextLine(1, "Centering");
		_nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);

		// Si la camara no detecta ningun blob, hay que volver a buscar la pelota
		if(_nblobs == 0){
			search_ball();
		} else{

			// Comprueba si la pelota se encuentra aproximadamente en el centro de
			// la camara
			NXTCAMgetCenter(_blobs, 0, blob_center_x, blob_center_y);
			if ( abs(blob_center_x - cameraCenter) < centerError ) {
				notInCenter = false;
			}
		}
	}
}

// Devuelve la velocidad angular correspondiente al giro a realizar
// para mantener la pelota centrada
float fw(int blob_center_x){
	float d = cameraCenter - blob_center_x;
	if(d >= 0){
		lastSeen = 1;
	} else{
		lastSeen = -1;
	}
	float w = (d/(cameraCenter))*2;
	nxtDisplayTextLine(1, "d: %d - w: %.2f",d,w);
	return w;
}

// Devuelve la velocidad lineal correspondiente al seguimiento de la
// pelota en funcion de la distancia entre el robot y la pelota
float fv(int blob_area){

	float dA = sqrt(goalArea) - sqrt(blob_area);
	float v = (dA / 349.60) + 0.05;
	nxtDisplayTextLine(2, "v: %.2f",v);
	return v;
}

// Realiza el seguimiento de la pelota corrigiendo las velocidades lineal
// y angular en funcion de lo descentrada que esta la pelota respecto a la
// camara y la distancia respecto a la misma
void track_ball(){
	  int _nblobs;
	  blob_array _blobs;
	  bool _condensed = true;
		bool notInFront = true;
		int blob_area = 0;
		int blob_center_x = 0;
		int blob_center_y = 0;
		float v = 0;
		float w = 0;

		while(notInFront){
			_nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);
			if(_nblobs == 0){
				search_ball();
				center_ball();
			} else{
				NXTCAMgetCenter(_blobs, 0, blob_center_x, blob_center_y);
				blob_area = _blobs[0].size;

				// Comprueba si la pelota esta lo suficientemente cerca como
				// para atraparla
				if(abs(blob_area - goalArea) < areaError){
					notInFront = false;
				} else{
					v = fv(blob_area);
					w = fw(blob_center_x);
					setSpeed(v,w);
				}
			}
		}
}

// Baja el brazo retractil para atrapar la pelota
void catch_ball()
{
	float armPower = 30;

	// Extends arm to catch the ball
	hogCPU();
	motor[motorB] = -armPower;
	releaseCPU();
	setSpeed(0,0);
	PlaySoundFile("wilhelmA.rso");
	Sleep(500);

	hogCPU();
	motor[motorB] = 0;
	releaseCPU();
}

// Comprueba si tiene la pelota atrapada bajo el brazo retractil
bool check_catched(){
	bool catched = true;
	setSpeed(0.2,0);
	Sleep(1000);
	int _nblobs;
	blob_array _blobs;
	bool _condensed = true;
	_nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);

	if(_nblobs == 0){
			PlaySoundFile("WTF.rso");
			hogCPU();
			motor[motorB] = 30;
			releaseCPU();
			setSpeed(0,0);
			Sleep(500);

			hogCPU();
			motor[motorB] = 0;
			releaseCPU();
			catched = false;
	}
	return catched;
}

task main ()
{

	// Initialize the camera
	NXTCAMinit(cam);

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

	// Retracts the arm
	hogCPU();
	motor[motorB] = 30;
	releaseCPU();
	Sleep(500);

	hogCPU();
	motor[motorB] = 0;
	releaseCPU();

	// Mientras no haya atrapado la pelota, el robot sigue
	// buscandola, centrandola, siguiendola e intentando atraparla
	bool catched = false;
	while (!catched){
		search_ball();
		center_ball();
		track_ball();

		// Catch the ball
		catch_ball();
		catched = check_catched();
	}

	StopTask(updateOdometry);

	Sleep(1500);
}
