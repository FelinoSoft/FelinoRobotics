#define X_START_A  3
#define Y_START_A  7
#define X_START_B  11
#define Y_START_B  7
#define X_END_A  7
#define Y_END_A  7
#define X_END_B 7
#define Y_END_B 7

bool B = false;	// By default, labyrinth A
bool planned = false;
bool camStarted = false;

void doPlanning()
{
	int x_end;
	int y_end;
	// Check type of labyrinth
	if(B){
		// Labyrinth B
		x_end = X_END_B;
		y_end = Y_END_B;
	} else {
		// Labyrinth A, this is the default one
		x_end = X_END_A;
		y_end = Y_END_A;
	}

	for(iLoop = 1; iLoop < sizePath; iLoop++){
		go(pathX[iLoop],pathY[iLoop]);

		// Check if hay obstaculo
		AcquireMutex(semaphore_odometry);
    float posX = robot_odometry.x;
    float posY = robot_odometry.y;
    ReleaseMutex(semaphore_odometry);

    Pos temp1;
    cellToPos(temp1, pathX[iLoop - 1], pathY[iLoop - 1]);
    Pos temp2;
    cellToPos(temp2, pathX[iLoop], pathY[iLoop]);
    bool thereIsObstacle = false;
    int index;

    float distToOrigin = euclideanDistance(posX, temp1.x, posY, temp1.y);
    float distToDestin = euclideanDistance(posX, temp2.x, posY, temp2.y);

    nxtDisplayTextLine(1, "ORI %d %d", pathX[iLoop - 1], pathY[iLoop - 1]);
    nxtDisplayTextLine(2, "DES %d %d", pathX[iLoop], pathY[iLoop]);
    nxtDisplayTextLine(3, "dist %2.2f %2.2f", distToOrigin, distToDestin);

		// If its nearer from iLoop - 1
    if(distToOrigin < distToDestin){
    	// Check wall from iLoop - 1
    	if(detectObstacle(pathX[iLoop - 1], pathY[iLoop - 1])){
    		thereIsObstacle = true;
    		index = iLoop - 1;
    	}
    } else {
			// Check wall from iLoop
    	if(detectObstacle(pathX[iLoop], pathY[iLoop])){
    		thereIsObstacle = true;
    		index = iLoop;
    	}
  	}

		if(thereIsObstacle){
			// WTF
			PlaySoundFile("WTF.rso");

			// Okay, vamo a calmarno, voy a eliminar conexion
			deleteConnection(pathX[index], pathY[index],facingDirection);
			if(facingDirection == 0){
				deleteConnection(pathX[index], pathY[index], facingDirection + 1);
				deleteConnection(pathX[index], pathY[index], 7);
			} else {
				deleteConnection(pathX[index], pathY[index], facingDirection + 1);
				deleteConnection(pathX[index], pathY[index], facingDirection - 1);
			}

			// Ahora replanifico turkey
			planPath((2*pathX[index])+1, (2*pathY[index])+1, x_end, y_end);

			PlaySoundFile("wilhelmA.rso");

			// Reiniciar bucle
			iLoop = 0;
		}
	}
}

void solveLabyrinth(bool type)
{
	// Common variables
	int x_ini;
	int y_ini;
	int x_end;
	int y_end;
	string map_file;

	// Check type of labyrinth
	if(type){
		// Labyrinth B
		x_ini = X_START_B;
		y_ini = Y_START_B;
		x_end = X_END_B;
		y_end = Y_END_B;
		map_file = "mapaB.txt";
	} else {
		// Labyrinth A, this is the default one
		x_ini = X_START_A;
		y_ini = Y_START_A;
		x_end = X_END_A;
		y_end = Y_END_A;
		map_file = "mapaA.txt";
	}

	// Inicializa la malla
	initConnections();

	if(loadMap(map_file)){
	  nxtDisplayTextLine(6, "Mapa loaded ok");
	} else {
	  nxtDisplayTextLine(6, "Mapa NOT loaded");
	}

	planPath(x_ini, y_ini, x_end, y_end);
	planned = true;

	doPlanning();
}

// Takes control of the speed
task planPathOnTheRoadTask()
{
  // Common variables
	int x_ini;
	int y_ini;
	int x_end;
	int y_end;
	string map_file;

	// Check type of labyrinth
	if(B){
		// Labyrinth B
		x_ini = X_START_B;
		y_ini = Y_START_B;
		x_end = X_END_B;
		y_end = Y_END_B;
		map_file = "mapaB.txt";
	} else {
		// Labyrinth A, this is the default one
		x_ini = X_START_A;
		y_ini = Y_START_A;
		x_end = X_END_A;
		y_end = Y_END_A;
		map_file = "mapaA.txt";
	}

	// Inicializa la malla
	initConnections();

	if(loadMap(map_file)){
	  nxtDisplayTextLine(6, "Mapa loaded ok");
	} else {
	  nxtDisplayTextLine(6, "Mapa NOT loaded");
	}

	planPath(x_ini, y_ini, x_end, y_end);

	planned = true;

	PlaySoundFile("wilhelmA.rso");

	// Initialize the camera
	NXTCAMinit(cam);
	camStarted = true;

	PlaySoundFile("WTF.rso");
}
