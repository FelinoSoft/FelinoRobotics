// CONFIG camera color position
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

int lastSeen = 1;
int goalArea = 5500;
int areaError = 500;
float cameraCenter = 85;
float centerError = 5;
bool esMiPrimeritaVezNoMas = true;

typedef struct
{
	float a;
	float b;
} Vec;

void carToPol(Vec dest, Vec p1, Vec p2){
	dest.a = euclideanDistance(p1.a, p2.a, p1.b, p2.b);
	dest.b = atan2(p2.a - p1.a, p2.b - p1.b);
}


/* El robot busca la pelota en el sitio dando vueltas sobre si mismo*/
void search_object(int color){

	// Define variables
	bool searching = true;
	float errorTheta = 0.005;
	float errorDist = 0.05;
	int _nblobs;
  blob_array _blobs;
  bool _condensed = true;

	// Obtain position
	float x, y, theta, thetaFinal;
	AcquireMutex(semaphore_odometry);
	x = robot_odometry.x;
	y = robot_odometry.y;
	theta = robot_odometry.th;
	ReleaseMutex(semaphore_odometry);

	// Check if its not the first time we search the ball
	if(!esMiPrimeritaVezNoMas){
		// Do a barrel roll before going to the center

		setSpeed(0, W_ROTACION*lastSeen, -1, -1);

		thetaFinal = theta - lastSeen*errorTheta*2;
		thetaFinal = normTheta(thetaFinal);
		while(searching && abs(theta - thetaFinal) > errorTheta){
			// Get info

			// Camera info
			_nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);

			// Position info
			AcquireMutex(semaphore_odometry);
			x = robot_odometry.x;
			y = robot_odometry.y;
			theta = robot_odometry.th;
			ReleaseMutex(semaphore_odometry);

			// Do stuff
			for (int i = 0; i < _nblobs; i++) {

				// Si se encuentra la pelota se ha terminado de buscar
		   	if (_blobs[i].colour == color){
		   		searching = false;
		 		}
	   	}
		}
	}

	// Obtain angle to turn
	Vec robotVec;
	robotVec.a = x;
	robotVec.b = y;
	Vec centerVec;
	if(B){
		centerVec.a = X_CENTER_B;
	} else {
		centerVec.a = X_CENTER_A;
	}
	centerVec.b = Y_CENTER;

	Vec polarVec;
	carToPol(polarVec, robotVec, centerVec);

	if(searching) {

		// Mientras la camara no detecte un blob del color de
		// la pelota, el robot sigue girando
		// condicion de parada
		thetaFinal = (PI/2) - polarVec.b;
		if(thetaFinal > (PI/2)){
			// Must go left
			setSpeed(0, W_ROTACION, -1, -1);
		} else {
			// Must go right
			setSpeed(0, -W_ROTACION, -1, -1);
		}
		while(searching && abs(theta - thetaFinal) > errorTheta){
			// Get info

			// Camera info
			_nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);

			// Position info
			AcquireMutex(semaphore_odometry);
			x = robot_odometry.x;
			y = robot_odometry.y;
			theta = robot_odometry.th;
			ReleaseMutex(semaphore_odometry);

			// Do stuff
			for (int i = 0; i < _nblobs; i++) {
				// Si se encuentra la pelota se ha terminado de buscar
		   	if (_blobs[i].colour == color){
		   		searching = false;
		 		}
	   	}
		}
		setSpeedBase(0,0);
	}

	// Si no la ha encontrado, sigo searching mientras voy al centro
	if(searching) {
		// Go to center
		setSpeed(V_RECTA, 0, -1, -1);

		while(searching && euclideanDistance(x,centerVec.a,y,centerVec.b) > errorDist){
			// Get info

			// Camera info
			_nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);

			// Position info
			AcquireMutex(semaphore_odometry);
			x = robot_odometry.x;
			y = robot_odometry.y;
			theta = robot_odometry.th;
			ReleaseMutex(semaphore_odometry);

			// Do stuff
			for (int i = 0; i < _nblobs; i++) {
				// Si se encuentra la pelota se ha terminado de buscar
		   	if (_blobs[i].colour == color){
		   		searching = false;
		 		}
	   	} // End for
		} // End while

		if(searching) {
			// Dar vueltas sobre uno mismo buscando la bola
			setSpeed(0, W_ROTACION, -1, -1);

			while(searching){
				// Get info

				// Camera info
				_nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);

				// Do stuff
				for (int i = 0; i < _nblobs; i++) {
					PlaySoundFile("wilhelmA.rso");

					// Si se encuentra la pelota se ha terminado de buscar
			   	if (_blobs[i].colour == color){
			   		searching = false;
			 		}
		   	} // End for
			} // End while
		}
	} // End if

}

void center_object(int color){
  int _nblobs;
  blob_array _blobs;
  bool _condensed = true;
	bool notInCenter = true;
	bool red_found = false;
	int blob_center_x = 0;
	int blob_center_y = 0;

	// Mientras el robot no haya enfilado la pelota hay que seguir girando en la
	// direccion de la ultima posicion conocida de la pelota
	setSpeed(0,1.2*lastSeen,-1,-1);
	while(notInCenter){
		nxtDisplayTextLine(1, "Centering");
		_nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);

		if(_nblobs == 0){
			search_object(color);
		} else{
			red_found = false;
			int i = 0;
			while(i < _nblobs && !red_found){

				/* Comprueba si hay blobs rojos */
		   	if (_blobs[i].colour == color){
		   		red_found = true;
		 		}
		 		else{
		 			i++;
		 		}
		  } // End while

		  if(red_found){
	   		NXTCAMgetCenter(_blobs, i, blob_center_x, blob_center_y);
				if ( abs(blob_center_x - cameraCenter) < centerError ) {
					notInCenter = false;
				}
			} else{
				search_object(color);
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
void track_object(int color){
  int _nblobs;
  blob_array _blobs;
  bool _condensed = true;
	bool notInFront = true;
	bool red_found = false;
	int blob_area = 0;
	int blob_center_x = 0;
	int blob_center_y = 0;
	float v = 0;
	float w = 0;

	while(notInFront){
		_nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);
		if(_nblobs == 0){
			search_object(color);
			center_object(color);
		} else{
			red_found = false;
			int i = 0;
			while(i < _nblobs && !red_found){

				/* Comprueba si hay blobs rojos */
		   	if (_blobs[i].colour == color){
		   		red_found = true;
		 		}
		 		else{
		 			i++;
		 		}
		  } // End while

		  if(red_found){
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
				search_object(color);
				center_object(color);
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
	setSpeed(0,0,-1,-1);
	PlaySoundFile("wilhelmA.rso");
	Sleep(500);

	hogCPU();
	motor[motorB] = 0;
	releaseCPU();
}

// Comprueba si tiene la pelota atrapada bajo el brazo retractil
bool check_catched(){
	bool catched = true;
	bool red_found = false;
	int _nblobs;
	blob_array _blobs;
	bool _condensed = true;
	_nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);


	if(_nblobs == 0){
		catched = false;
	} else{
			red_found = false;
			int i = 0;
			while(i < _nblobs && !red_found){

				/* Comprueba si hay blobs rojos */
		   	if (_blobs[i].colour == GOAL_COLOR){
		   		red_found = true;
		 		}
		 		else{
		 			i++;
		 		}
		  } // End while

		  if(!red_found){
	   		catched = false;
		  }
		}
		return catched;



	/*
			PlaySoundFile("WTF.rso");
			hogCPU();
			motor[motorB] = 30;
			releaseCPU();
			setSpeed(0,0,-1,-1);
			Sleep(500);

			hogCPU();
			motor[motorB] = 0;
			releaseCPU();
			catched = false;*/
}

void searchNearestExit(){

	search_object(EXIT_COLOR);
	center_object(EXIT_COLOR);
	track_object(EXIT_COLOR);
	setSpeed(0,0,-1,-1);
	Sleep(10000);

	// TODO
	PlaySoundFile("wilhelmA.rso");
}

void exitLab(){
	// TODO
	PlaySoundFile("Woops.rso");
}

void startPart3(){
	// Mientras no haya atrapado la pelota, el robot sigue
	// buscandola, centrandola, siguiendola e intentando atraparla
	while(!camStarted){
		// Do nothing
	}
	bool finished = false;
	bool catched = false;
	while (!finished){
		while(!catched){
			search_object(GOAL_COLOR);
			center_object(GOAL_COLOR);
			track_object(GOAL_COLOR);

			// Catch the ball
			catch_ball();
			catched = check_catched();
		}
		searchNearestExit();
		exitLab();
	}
}
