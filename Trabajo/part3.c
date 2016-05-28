/*
 *  part3.c
 *  Includes functions to do the third part of the race:
 *  the catching red balls area
 */

// CONFIG camera color position
#define BLUE 2
#define GREEN 1
#define RED 0

// CONFIG GOAL PARAMETERS
#define GOAL_COLOR RED
#define EXIT_COLOR GREEN
#define AREA_COLOR 100

// Config center of ball area
#define X_CENTER_A 2
#define X_CENTER_B 0.8
#define Y_CENTER 2

// Config positions of cardboards
#define X_CARDBOARD_A 1.8
#define X_CARDBOARD_B 0.6
#define Y_CARDBOARD 3

/* Variables */
int areaMin = 100;
int areaMed = 200;

int lastSeen = 1;
int goalArea = 4500;
int areaError = 500;
float cameraCenter = 85;
float centerError = 5;
bool finished = false;

// Custom structs
typedef struct
{
	float a;
	float b;
} Vec;

/*
 * Returns the angular speed equivalent to the turn that the robot has to make
 * to keep the ball centered
 */
float fw(int blob_center_x){
	float d = cameraCenter - blob_center_x;
	if(d >= 0){
		lastSeen = 1;
	} else{
		lastSeen = -1;
	}
	float w = (d/(cameraCenter))*2;
	return w;
}

/*
 *  Goes to the center of the room
 */
void goToCenter(){
	float errorTheta = 0.005;

	// Obtain position
	float x, y, theta, thetaFinal;
	AcquireMutex(semaphore_odometry);
	x = robot_odometry.x;
	y = robot_odometry.y;
	theta = robot_odometry.th;
	ReleaseMutex(semaphore_odometry);
	setSpeed(0,0,-1,-1);

	// Go to center
		setSpeed(V_RECTA, 0, -1, -1);

		while(y < Y_CENTER){
			// Get info
			// Position info
			AcquireMutex(semaphore_odometry);
			x = robot_odometry.x;
			y = robot_odometry.y;
			theta = robot_odometry.th;
			ReleaseMutex(semaphore_odometry);
		} // End while

		setSpeed(0,0,-1,-1);
}

/*
 *  Searches the ball
 */
void search_ball(){

		PlaySoundFile("Woops.rso");
	  int _nblobs;
	  blob_array _blobs;
	  bool _condensed = true;
		bool searching = true;
		setSpeed(0,1.2*lastSeen,-1,-1);

		// Mientras la camara no detecte un blob del color de
		// la pelota, el robot sigue girando
		while(searching){
			_nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);
			for (int i = 0; i < _nblobs; i++) {

				// Si se encuentra la pelota se ha terminado de buscar
		   	if (_blobs[i].colour == GOAL_COLOR && _blobs[i].size > areaMin){
		   		searching = false;
		 	 	}
      }
		}
}

/*
 *  Moves the robot to align its center to the ball center
 */
void center_ball(){
	PlaySoundFile("wilhelmA.rso");
  int _nblobs;
  blob_array _blobs;
  bool _condensed = true;
	bool notInCenter = true;
	int blob_center_x = 0;
	int blob_center_y = 0;
	bool ball_found = false;

	// Mientras el robot no haya enfilado la pelota hay que seguir girando en la
	// direccion de la ultima posicion conocida de la pelota
	setSpeed(0,1.2*lastSeen,-1,-1);
	while(notInCenter){
		_nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);

		// Si la camara no detecta ningun blob, hay que volver a buscar la pelota
		if(_nblobs == 0){
			search_ball();
		} else{

			// Comprueba si la pelota se encuentra aproximadamente en el centro de
			// la camara
			int i = 0;
			while(!ball_found && i < _nblobs){
				if(_blobs[i].colour == GOAL_COLOR && _blobs[i].size > areaMin){
					ball_found = true;
				} else{
					i++;
				}
			}

			if(ball_found){
				NXTCAMgetCenter(_blobs, i, blob_center_x, blob_center_y);
				if ( abs(blob_center_x - cameraCenter) < centerError ) {
					notInCenter = false;
				}
			} else{
				search_ball();
			}
		}
	}
}

/*
 * Returns the lineal speed equivalent to the turn that the robot has to make
 * to keep the ball centered
 */
float fv(int blob_area){

	float dA = sqrt(goalArea) - sqrt(blob_area);
	float v = (dA / 349.60) + 0.05;
	return v;
}

/*
 *  Goes after the ball found
 */
void track_ball(){
		PlaySoundFile("WTF.rso");
	  int _nblobs;
	  blob_array _blobs;
	  bool _condensed = true;
		bool notInFront = true;
		int blob_area = 0;
		int blob_center_x = 0;
		int blob_center_y = 0;
		float v = 0;
		float w = 0;
		bool ball_found = false;

		while(notInFront){
			_nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);
			if(_nblobs == 0){
				search_ball();
				center_ball();
			} else{

				int i = 0;
				while(i < _nblobs && !ball_found){
					if(_blobs[i].colour == GOAL_COLOR){
						ball_found = true;
					} else{
						i++;
					}
				}

				if(ball_found){
						NXTCAMgetCenter(_blobs, i, blob_center_x, blob_center_y);
						blob_area = _blobs[i].size;

						// Comprueba si la pelota esta lo suficientemente cerca como
						// para atraparla
						if(abs(blob_area - goalArea) < areaError){
							notInFront = false;
						} else{
							v = fv(blob_area);
							w = fw(blob_center_x);
							setSpeed(v,w,-1,-1);
						}
				} else{
					search_ball();
					center_ball();
				}
			}
		}
}

/*
 *  Pulls down the arm to catch the ball
 */
void catch_ball()
{
	float armPower = 30;

	// Extends arm to catch the ball
	hogCPU();
	motor[motorB] = -armPower;
	releaseCPU();
	setSpeed(0,0,-1,-1);
	PlaySoundFile("wilhelmA.rso");
	AcquireMutex(semaphore_odometry);
  float posX = robot_odometry.x;
  float posY = robot_odometry.y;
  float theta = robot_odometry.th;
  ReleaseMutex(semaphore_odometry);
	drawRobot(posX * 1000, posY * 1000, theta);
	Sleep(500);

	hogCPU();
	motor[motorB] = 0;
	releaseCPU();
}

/*
 *  Check if it has the ball catched
 */
bool check_catched(){
	bool catched = true;
	bool red_found = false;
	int _nblobs;
	blob_array _blobs;
	bool _condensed = true;
	_nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);


	if(_nblobs == 0){
		catched = false;
	   		PlaySoundFile("WTF.rso");
					hogCPU();
					motor[motorB] = 30;
					releaseCPU();
					setSpeed(0,0,-1,-1);
					Sleep(500);

					hogCPU();
					motor[motorB] = 0;
					releaseCPU();
					catched = false;
			  }
		else{
			red_found = false;
			int i = 0;
			while(i < _nblobs && !red_found){

				/* Comprueba si hay blobs rojos */
		   		if (_blobs[i].size > areaMin && _blobs[i].colour == GOAL_COLOR){
		   			PlaySoundFile("Woops.rso");
		   			red_found = true;
		 		}
		 		else{
		 			i++;
		 		}
		  } // End while

		  if(!red_found){
	   		catched = false;
	   		PlaySoundFile("WTF.rso");
					hogCPU();
					motor[motorB] = 30;
					releaseCPU();
					setSpeed(0,0,-1,-1);
					Sleep(500);

					hogCPU();
					motor[motorB] = 0;
					releaseCPU();
					catched = false;
			  }
		}
		return catched;
}

/*
 *  Searches the nearest and the correct exit to finish the race
 */
void searchNearestExit(){

	// Girar hasta estar orientados hacia arriba (PI/2)

	float errorTheta = 0.005;
	float errorDist = 0.05;
	// Obtain position
	float x, y, theta, thetaFinal;
	AcquireMutex(semaphore_odometry);
	x = robot_odometry.x;
	y = robot_odometry.y;
	theta = robot_odometry.th;
	ReleaseMutex(semaphore_odometry);

	thetaFinal = (PI/2);
	float v, w;
	while(abs(theta - thetaFinal) > errorTheta) {
		// If theta no es 90 grados
		if(theta >= -(PI/2) && theta < (PI)/2){
			// Gire a la izquierda
			v = 0;
    	w = W_ROTACION;
		} else{
			// Gire a la derecha
			v = 0;
			w =-W_ROTACION;
		}
		// Check if our orientation is correct
		AcquireMutex(semaphore_odometry);
    theta = robot_odometry.th;
    ReleaseMutex(semaphore_odometry);
    setSpeed(v,w,-1,-1);
	}
	setSpeed(0,0,-1,-1);

	float yBase = 2.6;
	AcquireMutex(semaphore_odometry);
	x = robot_odometry.x;
	y = robot_odometry.y;
	theta = robot_odometry.th;
	ReleaseMutex(semaphore_odometry);

	// Retroceder hasta alcanzar la yBase
	if(y > yBase) {
		while(abs(y - yBase) > errorDist){
			setSpeed(-V_RECTA,0,-1,-1);
			AcquireMutex(semaphore_odometry);
			y = robot_odometry.y;
			ReleaseMutex(semaphore_odometry);
		}
		setSpeed(0,0,-1,-1);
	} else {
	// O avanzar hasta alcanzar la yBase
		while(abs(y - yBase) > errorDist){
			setSpeed(V_RECTA,0,-1,-1);
			AcquireMutex(semaphore_odometry);
			y = robot_odometry.y;
			ReleaseMutex(semaphore_odometry);
		}
		setSpeed(0,0,-1,-1);
	}

	AcquireMutex(semaphore_odometry);
	x = robot_odometry.x;
	y = robot_odometry.y;
	theta = robot_odometry.th;
	ReleaseMutex(semaphore_odometry);
	float xBase;
	if(B) {
		xBase = 0.8;
	} else{
		xBase = 2;
	}

	bool verdeALaIzquierda = false;
	/* Comprobar donde estamos en el espacio de las bolas */
	float wRotTemp;
	if(x < xBase){
		wRotTemp = -W_ROTACION;
	} else {
		wRotTemp = W_ROTACION;
	}

	bool sabemosPorDondeSalir = false;
	setSpeed(0,wRotTemp,-1,-1);
	blob_array _blobs;
	memset(_blobs, 0, sizeof(blob_array));
	bool _condensed = true;
	int _nblobs;
	int greenPosition = -2;
	int bluePosition = -2;
	int dirtG;
	int dirtB;
	while(!sabemosPorDondeSalir){
		_nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);
		greenPosition = -2;
	  bluePosition = -2;
		if(_nblobs == 0){
			setSpeed(0,wRotTemp,-1,-1);
		} else{
			NXTCAMgetAverageCenter(_blobs, 3, GREEN, greenPosition, dirtG);
			NXTCAMgetAverageCenter(_blobs, 3, BLUE, bluePosition, dirtB);

		  if(greenPosition > 0  && bluePosition> 0 && greenPosition != bluePosition){
		  	// Hemos encontrado los dos
		  	if(greenPosition < bluePosition){
		  		verdeALaIzquierda = true;
		  	} else {
		  		verdeALaIzquierda = false;
		  	}
		  	setSpeed(0,0,-1,-1);
		  	sabemosPorDondeSalir = true;
		  } else{
		  	// No hemos encontrado los dos, seguimos girando
		  	setSpeed(0,wRotTemp,-1,-1);
		  } // End if
		} // End else
	}
	/* Ir a la pared */
	// Go recto madafaka
	/* Calcular por donde hay que salir en funcion del circuito y el verde */
	if(B && verdeALaIzquierda){
		thetaFinal = PI;
		facingDirection = 6;
		setSpeed(0,W_ROTACION,-1,-1);
	} else if(B && !verdeALaIzquierda) {
		thetaFinal = 0;
		facingDirection = 2;
		setSpeed(0,-W_ROTACION,-1,-1);
	} else if(!B && verdeALaIzquierda) {
		thetaFinal = 0;
		facingDirection = 2;
		setSpeed(0,-W_ROTACION,-1,-1);
	} else {
		thetaFinal = PI;
		facingDirection = 6;
		setSpeed(0,W_ROTACION,-1,-1);
	}

	AcquireMutex(semaphore_odometry);
	x = robot_odometry.x;
	y = robot_odometry.y;
	theta = robot_odometry.th;
	ReleaseMutex(semaphore_odometry);

	while(abs(theta - thetaFinal) > errorTheta) {
		AcquireMutex(semaphore_odometry);
		x = robot_odometry.x;
		y = robot_odometry.y;
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
		if(theta < 0 && theta < -(PI/2) ){
			theta = theta + (2*PI);
		}
	}
	setSpeed(0,0,-1,-1);

	// Go recto madafaka
	setSpeed(V_RECTA, 0, -1, -1);

	while(SensorValue[sonarSensorFrontal] > distanceToDetect+15){
		// Seguir recto bitch
	}
	setSpeed(0,0,-1,-1);
	thetaRecalibrated = false;
	StartTask(recalTheta);
	while(!thetaRecalibrated){
		// Do nothing
	}
	thetaRecalibrated = false;

	thetaFinal = (PI/2);
	AcquireMutex(semaphore_odometry);
	x = robot_odometry.x;
	ReleaseMutex(semaphore_odometry);
	if(x > xBase){
		setSpeed(0,W_ROTACION,-1,-1);
	} else{
		setSpeed(0,-W_ROTACION,-1,-1);
	}

	AcquireMutex(semaphore_odometry);
	x = robot_odometry.x;
	y = robot_odometry.y;
	theta = robot_odometry.th;
	ReleaseMutex(semaphore_odometry);

	while(abs(theta - thetaFinal) > errorTheta) {
		AcquireMutex(semaphore_odometry);
		theta = robot_odometry.th;
		ReleaseMutex(semaphore_odometry);
		if(theta < 0 && theta < -(PI/2) ){
			theta = theta + (2*PI);
		}
	}

	setSpeed(0,0,-1,-1);

	// GO RECTO YOLO
	setSpeed(V_RECTA, 0, -1, -1);
	float yMeta = 3.6;
	AcquireMutex(semaphore_odometry);
	x = robot_odometry.x;
	y = robot_odometry.y;
	theta = robot_odometry.th;
	ReleaseMutex(semaphore_odometry);

	while(abs(y - yMeta) > errorDist){
		AcquireMutex(semaphore_odometry);
		y = robot_odometry.y;
		ReleaseMutex(semaphore_odometry);
	}
	setSpeed(0,0,-1,-1);
	finished = true;
	AcquireMutex(semaphore_odometry);
  float posX = robot_odometry.x;
  float posY = robot_odometry.y - 0.6;
  theta = robot_odometry.th;
  ReleaseMutex(semaphore_odometry);
	drawRobot(posX * 1000, posY * 1000, theta);
}

/*
 *  Contains the main behaviour on part 3
 */
void startPart3(){
	// Mientras no haya atrapado la pelota, el robot sigue
	// buscandola, centrandola, siguiendola e intentando atraparla
	goToCenter();
	bool catched = false;
	while (!finished){
		while(!catched){
			search_ball();
			center_ball();
			track_ball();

			// Catch the ball
			catch_ball();
			catched = check_catched();
		}
		searchNearestExit();
	}

	/* Esperar mucho tiempo */
}
