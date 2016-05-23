#include "common.h"
#include "mapLib.c"

#define X_START_A  3
#define Y_START_A  7
#define X_START_B  -1
#define Y_START_B  -1
#define X_END_A  7
#define Y_END_A  7
#define X_END_B -1
#define Y_END_B -1

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
		map_file = "mapB.txt";
	} else {
		// Labyrinth A, this is the default one
		x_ini = X_START_A;
		y_ini = Y_START_A;
		x_end = X_END_A;
		y_end = Y_END_A;
		map_file = "mapA.txt";
	}

	// Inicializa la malla
	initConnections();

	if(loadMap(map_file)){
	  nxtDisplayTextLine(6, "Mapa loaded ok");
	} else {
	  nxtDisplayTextLine(6, "Mapa NOT loaded");
	}

	planPath(x_ini, y_ini, x_end, y_end);

	for(iLoop = 1; iLoop < sizePath; iLoop++){
		go(pathX[iLoop],pathY[iLoop]);

		// Check if hay obstaculo
		if(detectObstacle(pathX[iLoop-1], pathY[iLoop-1])){
			// WTF
			PlaySoundFile("WTF.rso");

			// Okay, vamo a calmarno, voy a eliminar conexion
			deleteConnection(pathX[iLoop-1], pathY[iLoop-1],facingDirection);

			// Ahora replanifico turkey
			planPath(2*pathX[iLoop-1]+1, 2*pathY[iLoop-1]+1, x_end, y_end);

			PlaySoundFile("wilhelmA.rso");

			// Reiniciar bucle
			iLoop = 0;
		}
	}
}

// Takes control of the speed
task planPathOnTheRoadTask()
{
  // TODO
}
