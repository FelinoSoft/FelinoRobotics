#define X_START_A  3
#define Y_START_A  7
#define X_START_B  11
#define Y_START_B  7
#define X_END_A  7
#define Y_END_A  7
#define X_END_B 7
#define Y_END_B 7

bool planned = false;
bool camStarted = false;
bool doRecalibration = true;
bool doDrawing = true;
bool thetaRecalibrated = false;

float get5SonarValues()
{
	float value = SensorValue[sonarSensorFrontal];
	value = value + SensorValue[sonarSensorFrontal];
	value = value + SensorValue[sonarSensorFrontal];
	value = value + SensorValue[sonarSensorFrontal];
	value = value + SensorValue[sonarSensorFrontal];
	return value/5;
}

task recalTheta()
{
	wait1Msec(100);
	float minValue = get5SonarValues();
	setSpeed(0, W_ROTACION/3, -1, -1);
	wait1Msec(100);
	float medida_actual = get5SonarValues();

	// Girar a la izquierda hasta que la medida actual sea mayor que la previa
	while(minValue >= medida_actual){
		minValue = medida_actual;
		wait1Msec(100);
		medida_actual = get5SonarValues();
	}
	setSpeed(0,0,-1,-1);

	setSpeed(0, -(W_ROTACION/3), -1, -1);
	wait1Msec(100);
	medida_actual = get5SonarValues();
	// Girar a la derecha hasta que la medida actual sea mayor que la previa
	while(minValue >= medida_actual){
		minValue = medida_actual;
		wait1Msec(100);
		medida_actual = get5SonarValues();
	}
	setSpeed(0,0,-1,-1);
	AcquireMutex(semaphore_odometry);
  float posX = robot_odometry.x;
  float posY = robot_odometry.y;
	float th = robot_odometry.th;
  ReleaseMutex(semaphore_odometry);

	if(facingDirection == 0){
		set_position(robot_odometry, posX, posY, (PI/2));
	} else if(facingDirection == 2){
		set_position(robot_odometry, posX, posY, 0);
	} else if(facingDirection == 4){
		set_position(robot_odometry, posX, posY, -(PI/2));
	} else if(facingDirection == 6){
		if(th < 0){
			set_position(robot_odometry, posX, posY, -(PI));
		} else {
			set_position(robot_odometry, posX, posY, PI);
		}
	}
	thetaRecalibrated = true;
}

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

			if(doRecalibration){
				//StartTask(recalTheta);
				thetaRecalibrated = true;
			}
			reDrawMap();
			// Ahora replanifico turkey
			planPath((2*pathX[index])+1, (2*pathY[index])+1, x_end, y_end);

			PlaySoundFile("wilhelmA.rso");

			if(doDrawing) {
				doReDrawMap = true;
			}

			if(doRecalibration){
				while(!thetaRecalibrated){
					// Do nothing
				}
				thetaRecalibrated = false;
			}

			// Reiniciar bucle
			iLoop = 0;
		}
	}
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
	if(alVuelo){
  	wait1Msec(1);
  }

	if(loadMap(map_file)){
	  nxtDisplayTextLine(6, "Mapa loaded ok");
	} else {
	  nxtDisplayTextLine(6, "Mapa NOT loaded");
	}

	if(alVuelo){
  	wait1Msec(1);
  }
	planPath(x_ini, y_ini, x_end, y_end);
	if(alVuelo){
  	wait1Msec(1);
  }
	planned = true;

	PlaySoundFile("wilhelmA.rso");
	alVuelo = false;
}
