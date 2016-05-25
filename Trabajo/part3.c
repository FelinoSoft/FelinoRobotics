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

int areaMin = 100;
int areaMed = 200;

int lastSeen = 1;
int goalArea = 5500;
int areaError = 500;
float cameraCenter = 85;
float centerError = 5;
bool esMiPrimeritaVezNoMas = true;

bool finished = false;

typedef struct
{
	float a;
	float b;
} Vec;

//void center_object(int color);

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
			nxtDisplayTextLine(1, "Search g2 %d", color);
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
		   		if (_blobs[i].size > areaMin && _blobs[i].colour == color){
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
			nxtDisplayTextLine(1, "Search g %d", color);
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
		   		if (_blobs[i].size > areaMin && _blobs[i].colour == color){
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
			nxtDisplayTextLine(1, "Search r %d", color);
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
	}
	if(searching) {
		// Dar vueltas sobre uno mismo buscando la bola
		setSpeed(0, W_ROTACION, -1, -1);

		while(searching){
			// Get info

			// Camera info
			_nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);

			// Do stuff
			for (int i = 0; i < _nblobs; i++) {

				// Si se encuentra la pelota se ha terminado de buscar
	   			if (_blobs[i].size > areaMin && _blobs[i].colour == color){
		   			searching = false;
		 		}
	   	} // End for
		} // End while
	} // End if
}// End search object

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
void small_tracking(){

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
		nxtDisplayTextLine(1, "Sm Tracking %d", 0);
		_nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);
		if(_nblobs == 0){
			search_object(GOAL_COLOR);
			//center_object(GOAL_COLOR);
		} else{
			red_found = false;
			int i = 0;
			while(i < _nblobs && !red_found){

				/* Comprueba si hay blobs rojos */
		   		if (_blobs[i].size > areaMin && _blobs[i].colour == GOAL_COLOR){
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
				if(abs(blob_area - 300) < areaError){
					notInFront = false;
				} else{
					v = fv(blob_area);
					w = fw(blob_center_x);
					setSpeed(v,w,-1,-1);
				}
			} else{
				search_object(GOAL_COLOR);
					//center_object(GOAL_COLOR);
			}
		}
	}
}

/*
void center_object(int color){
  int _nblobs;
  blob_array _blobs;
  bool _condensed = true;
	bool notInCenter = true;
	bool red_found = false;
	bool small_tracked = false;
	int blob_center_x = 0;
	int blob_center_y = 0;

	// Mientras el robot no haya enfilado la pelota hay que seguir girando en la
	// direccion de la ultima posicion conocida de la pelota
	setSpeed(0,1.2*lastSeen,-1,-1);
	while(notInCenter){
		nxtDisplayTextLine(1, "Centering %d", color);
		_nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);

		if(_nblobs == 0){
			search_object(color);
		} else{
			red_found = false;
			int i = 0;
			while(i < _nblobs && !red_found){

				// Comprueba si hay blobs rojos
		   	if (_blobs[i].size > areaMin && _blobs[i].colour == color){
		   			red_found = true;
		 		} else if(_blobs[i].size > areaMin && _blobs[i].size < areaMed &&
		 			_blobs[i].colour == color){
		 				small_tracking();
		 				small_tracked = true;
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
			} else if(!small_tracked){
				search_object(color);
			}
		}
	}
}*/



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
		nxtDisplayTextLine(1, "Tracking %d", color);
		_nblobs = NXTCAMgetBlobs(cam, _blobs, _condensed);
		if(_nblobs == 0){
			search_object(color);
			/*
			if(color == 0){
				center_object(color);
			}*/
		} else{
			red_found = false;
			int i = 0;
			while(i < _nblobs && !red_found){

				/* Comprueba si hay blobs rojos */
		   		if (_blobs[i].size > areaMin && _blobs[i].colour == color){
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
				/*
				if(color == 0){
					center_object(color);
				}*/
			} // End else red found
		} // End else n blobs == 0
	} // End while
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

	float yBase = y;
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
		  	nxtDisplayTextLine(1, "%d", greenPosition);
		  	nxtDisplayTextLine(2, "%d", bluePosition);
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

	while(SensorValue[sonarSensorFrontal] > distanceToDetect*2){
		// Seguir recto bitch
	}
	setSpeed(0,0,-1,-1);
	thetaRecalibrated = false;
	StartTask(recalTheta);
	while(!thetaRecalibrated){
		// Do nothing
	}

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
	float yMeta = 3.2;
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
  float posY = robot_odometry.y;
  theta = robot_odometry.th;
  ReleaseMutex(semaphore_odometry);
	drawRobot(posX * 1000, posY * 1000, theta);
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
			//center_object(GOAL_COLOR);
			track_object(GOAL_COLOR);

			// Catch the ball
			catch_ball();
			catched = true;/*check_catched(); TODO*/
		}
		searchNearestExit();
	}

	/* Esperar mucho tiempo */
}
