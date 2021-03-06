#pragma config(Sensor, S2,     sonarSensorFrontal,         sensorSONAR)
#pragma config(Sensor, S4,     HTGYRO,              sensorAnalogInactive)

/*
 *  mapLib.c
 *  Contains functions for dealing with maps and path
 *  calculations in unknown environments for the robot
 */

/* Variables */
// File handling variables
TFileIOResult nIoResult;
TFileHandle hFileHandleGrid;
long nFileSize                   = 5000; //1 byte each char..

// Global variables
int sizeX;
int sizeY;
int sizeCell;
int pixPerX;
int pixPerY;

// Max number of cells to use
#define MAX_X 20
#define MAX_Y 20

// Connection matrix and path
bool connectionsMatrix[2*MAX_X+1][2*MAX_Y+1];
int pathX[MAX_X];
int pathY[MAX_X];
int sizePath;

// Other useful variables
float distanceToDetect = 20;
int facingDirection = 2;
int iLoop;
bool alVuelo = true;

// Custom structs
typedef struct
{
   short grid[2*MAX_X + 1][2*MAX_Y + 1];

} Grid;

typedef struct
{
	float x;
	float y;
} Pos;

typedef struct
{
	short x;
	short y;
} Cell;

// Grid variable used for storing the map
Grid grid;

/*
 *  Initializes the connections matrix
 */
void initConnections(){
   for(int i=0; i<2*MAX_X+1; ++i){
      for (int j=0; j<2*MAX_Y+1; ++j){
        connectionsMatrix[i][j]=false;
      }
   }
}

/*
 *  Converts from map's cells to connection's matrix coordinates given a
 *  neighborhood index
 */
void cell2connCoord(int cellX, int cellY, int numNeigh, int & connX, int & connY){
  connX=2*cellX+1;
  connY=2*cellY+1;
  switch(numNeigh){
    case 0: connY++; break;
    case 1: connY++; connX++; break;
    case 2: connX++;break;
    case 3: connY--; connX++; break;
    case 4: connY--; break;
    case 5: connY--; connX--; break;
    case 6: connX--; break;
    case 7: connY++; connX--; break;
  }
}

/*
 *  Opens a connection in the connection matrix
 */
void setConnection(int cellX, int cellY, int numNeigh){
  int connX, connY; // coordinates in the connection matrix
  // from coordinates in the grid of cells to coordinates in the connection matrix
  cell2connCoord(cellX, cellY, numNeigh, connX, connY);
  connectionsMatrix[connX][connY]=true;
}

/*
 *  Closes a connection in the connection matrix
 */
void deleteConnection(int cellX, int cellY, int numNeigh){
  int connX, connY; // coordinates in the connection matrix
  // from coordinates in the grid of cells to coordinates in the connection matrix
  cell2connCoord(cellX, cellY, numNeigh, connX, connY);
  connectionsMatrix[connX][connY]=false;
}

/*
 *  Returns the value of the given cell in the connections matrix
 */
bool isConnected(int cellX, int cellY, int numNeigh){
   int connX, connY; // coordinates in the connection matrix
  // from coordinates in the grid of cells to coordinates in the connection matrix
  cell2connCoord(cellX, cellY, numNeigh, connX, connY);
  return(connectionsMatrix[connX][connY]);

}

/*
 *  Reads the header of the map file
 */
bool readLineHeader(TFileHandle hFileHandle,TFileIOResult nIoResult, int & dimX, int & dimY, int &dimCell)
{
  //unsigned ans;
  //short ans;
  int ind = 1;
  //float aux = 0.1;
  bool eol = false;
  bool endfile = false;
  char onechar;
  char linechar[40];
  int num = 0;
  int indNum=0;
  int numbersRead[3];

  // read header
	while(!eol){
	  ReadByte(hFileHandle, nIoResult, onechar);
	  if ( nIoResult == 0 ){ // all ok
        if (onechar==13) // end of line
        {
          linechar[ind-1]=0;
          eol=true;
        }
        else{
          if (onechar=='\n'){ // line jump
            //skip
          }
          else{
            linechar[ind-1]=onechar;
            if (onechar==' '){
              numbersRead[indNum]=num;
              num=0;
              indNum++;
            }else{
              num = 10*num + (onechar - '0' );
            }
            ind++;
          }
        }
     }
     else{
          if (nIoResult==ioRsltEndOfFile){
            eol=true;
            endfile=true;

          }else{
            //nxtDisplayTextLine(1, "PROBLEM READING map");
          }
      }
   }
   // from char to string
   //StringFromChars(linestring,linechar);
   if (numbersRead[indNum]!=num && num!=0){
      numbersRead[indNum]=num;
   }

   dimX = numbersRead[0];
   dimY = numbersRead[1];
   dimCell = numbersRead[2];

   /*nxtDisplayTextLine(3, "%d %d ", dimX, dimY);
   nxtDisplayTextLine(4, "%d ", dimCell);
   wait10Msec(300);*/

   return endfile;
}

/*
 *  Reads next line from the map file
 */
bool readNextLine(TFileHandle hFileHandle,TFileIOResult & nIoResult, int & mapRow)
{
  //short ans;

  int ind = 0; 									// pointer to keep all text read in vector linechar
  char linechar[(MAX_X*2+1)*3]; // how long do we expect the lines...
  string linestring;
  char onechar;

  bool eol = false;
  bool endfile = false;
  int mapCol=0;

  // read header
	while(!eol){
	  ReadByte(hFileHandle, nIoResult, onechar);
	  if ( nIoResult == 0 ){ 			// all ok
        if (onechar==13) 				// end of line
        {
          linechar[ind]=0;
          eol=true;
        }
        else{
          if (onechar=='\n'){ 	// line jump
            //skip
          }
          else{
            linechar[ind]=onechar;
            if (onechar==' '){
              //numbersRead[indNum]=num;
              //num=0;
              //indNum++;
            }else{
              if (onechar=='1'){
                //nxtDisplayTextLine(3, " %d %d", mapCol,mapRow);
                connectionsMatrix[mapCol][mapRow]=true;
              }
              // else { false} // by default is false
              mapCol++;

            }
            ind++;
          }
        }
     }
     else{
          if (nIoResult==ioRsltEndOfFile){
            eol=true;
            endfile=true;

          }else{
            //nxtDisplayTextLine(1, "PROBLEM READING map");
          }
      }
   }

   // jump to next row
   mapRow--;
   if (mapRow<0){
      // STOP READING, map is full
      endfile=true;
   }

   // from char to string
   StringFromChars(linestring,linechar);
   /*if (numbersRead[indNum]!=num && num!=0){
      numbersRead[indNum]=num;
   }*/

   //nxtDisplayTextLine(3, "%s ", linestring);

   /*for(int j=2; j<=indNum; ++j){
      setConnection(numbersRead[0], numbersRead[1], numbersRead[j]);
      nxtDisplayTextLine(4, "%d connection open", numbersRead[j]);
      //wait10Msec(200);
   }*/
   return endfile;
}

/*
 * Loads map from file
 */
bool loadMap(string mapFileName)
{
     bool loadingOk=false;
     //int dimConectionX,dimConectionY;
     int mapRow; // last row from connection matrix

     string line="";
     bool eof = false;
     TFileIOResult nIoResult;
     TFileHandle hFileHandle;
     int nFileSize = 0; // it is filled when we open file

     Close(hFileHandle, nIoResult);
     hFileHandle = 0;
	   //nxtDrawLine(_x+2, _y, _x-2, _y);

	   OpenRead(hFileHandle, nIoResult, mapFileName, nFileSize);
	   if( nIoResult ==0 ){
	       //nxtDisplayTextLine(1, "OPEN OK: %d", nFileSize);

	       //StringFromChars(sToString, FromChars)
         //Converts an array of bytes to a string value.  You MUST end your char array with a char value of zero!

	        // read first line
          eof = readLineHeader(hFileHandle,nIoResult, sizeX, sizeY, sizeCell);
          //nxtDisplayTextLine(2, "%s", line);
          mapRow=2*sizeY;
          // read rest of data
          while(!eof){
            eof = readNextLine(hFileHandle,nIoResult, mapRow);
            //eof = readNextCellConnections(hFileHandle,nIoResult);
            //nxtDisplayTextLine(2, "%s", line);
            if(alVuelo){
            	wait1Msec(1);
            }
	        }
	        loadingOk=true;
	        Close(hFileHandle, nIoResult);
     }
     else{
           loadingOk=false;
           //nxtDisplayTextLine(1, "PROBLEM OPENING file");
     }

	   return loadingOk;
}


/*
 *  Convert from robot odometry coordinates (in mm) to cell coordinates
 */
void pos2cell(float x_mm, float y_mm, int & x_cell, int & y_cell){

  x_cell =  (int) x_mm/sizeCell;

  y_cell = (int) y_mm/sizeCell;

}

/*
 *  Calculates the wavefront for the NF1 path calculation
 */
void calcWaveFront(Grid g, int x, int y, int value){
	if(alVuelo){
  	wait1Msec(1);
  }

	if(x >= 0 && y >= 0 && x < 2*sizeX+1 && y < 2*sizeY+1 && y < 8){
		if(g.grid[x][y] != -1 && (g.grid[x][y] > value || g.grid[x][y] == -2)){

			//Celda no es un obstaculo, y es o una celda ya visitada pero con mayor coste,
			//o una celda no visitada

			//Asignamos valor
			g.grid[x][y] = value;

			//Expandimos onda
			calcWaveFront(g,x-1,y,value + 1);
			calcWaveFront(g,x+1,y,value + 1);
			calcWaveFront(g,x,y-1,value + 1);
			calcWaveFront(g,x,y+1,value + 1);
		}
	}
}

/*
 *  Returns the contrary direction given a neighbor index
 */
int getInverse(int neighbor){
	if(neighbor == 0) {
		// We are coming from the right
		return 1;
	} else if(neighbor == 1) {
		// We are coming from the left
		return 0;
	} else if(neighbor == 2) {
		// We are coming from the top
		return 3;
	} else if(neighbor == 3) {
		// We are coming from the bottom
		return 2;
	} else{
		return -1;
	}
}

/*
 *  Calculates best path for reaching the goal using NF1 algorithm
 */
void makePath(int x_ini, int y_ini, int x_end, int y_end, Grid grid){
	int n = 0;
	if(x_ini % 2 != 0 && y_ini % 2 !=0){
		pathX[n] = (x_ini - 1) / 2;
		pathY[n] = (y_ini - 1) / 2;
	}

	int previousNeighbor = -1;
	int neighborX[4] = {1,-1,0,0};
	int neighborY[4] = {0,0,1,-1};
	int previousBestX = x_ini;
	int previousBestY = y_ini;
	int indexMove = -1;

	while(previousBestX != x_end || previousBestY != y_end){
		int bestX = previousBestX;
		int bestY = previousBestY;

		if(indexMove < 0){
			for(int i = 0; i < 4; i++){
				int candidatoX = previousBestX + neighborX[i];
				int candidatoY = previousBestY + neighborY[i];
				int valorCandidato = grid.grid[candidatoX][candidatoY];
				int valorPath = grid.grid[previousBestX][previousBestY];

				if(valorCandidato != -1 && valorCandidato < valorPath){
					// If we have not previous neighbor, store bests as usual
					if(previousNeighbor < 0){
						bestX = candidatoX;
						bestY = candidatoY;
						indexMove = i;
					} else {
						// If we have previous neighbor, store bests only if the
						// direction chosen is not the inverse of the previous
						// neighbor
						if(i != getInverse(previousNeighbor)){
							bestX = candidatoX;
							bestY = candidatoY;
							indexMove = i;
						}
					}
				}
			}
		} else{
			// Jump
			bestX = bestX + neighborX[indexMove];
			bestY = bestY + neighborY[indexMove];
		}

		if((bestX / 2)*2 != bestX && (bestY / 2)*2 != bestY){
			previousNeighbor = indexMove;
			indexMove = -1;
			n = n + 1;
			pathX[n] = (bestX - 1) / 2;
			pathY[n] = (bestY - 1) / 2;
		}
		previousBestX = bestX;
		previousBestY = bestY;
  }

  sizePath = n + 1;
}

/*
 *  Store in pathX and pathY respectively the coordinates of all the cells
 *  that the robot has to cross to reach the goal.
 */
void planPath(int x_ini, int y_ini, int x_end,int y_end){

	int i;
	int j;
	for(i = 0; i < 2*sizeX+1; i++){
		for(j = 0; j < 2*sizeY+1; j++){
			// Hay conexion
			if(connectionsMatrix[i][j]){
				grid.grid[i][j] = -2;
			} else {
				grid.grid[i][j] = -1;
			}
		}
	}

	if(alVuelo){
  	wait1Msec(1);
  }
	grid.grid[x_end][y_end] = 0;

	calcWaveFront(grid,x_end-1,y_end,1);
	calcWaveFront(grid,x_end+1,y_end,1);
	calcWaveFront(grid,x_end,y_end-1,1);
	calcWaveFront(grid,x_end,y_end+1,1);

	if(alVuelo){
  	wait1Msec(1);
  }
	//INICIALIZARRRR FICHERRROOOS
	string sFileName = "grid.txt";
	string sFileName2 = "path.txt";
	hFileHandleGrid = 0;
  Close(hFileHandleGrid, nIoResult);
	if(alVuelo){
  	wait1Msec(1);
  }
  // Deletes the file if it already exists
  Delete(sFileName, nIoResult);
  OpenWrite(hFileHandleGrid, nIoResult, sFileName, nFileSize);
	if(alVuelo){
		wait1Msec(1);
	}
  // Escribe la matriz resultante del algoritmo NF1 en un fichero
  for(int i = 2*sizeY; i >= 0;i--){
  	for(int j = 0; j < 2*sizeX+1;j++){
  		string temp, temp2;
  		if(grid.grid[j][i] > -1 && grid.grid[j][i] < 10){
  			StringFormat(temp, " %d ", grid.grid[j][i]);
  		} else {
  		  StringFormat(temp, "%d ", grid.grid[j][i]);
  	  }
  		WriteText(hFileHandleGrid,nIoResult,temp);
  		if(j == 2*sizeX){
  			StringFormat(temp2, "\n");
  			WriteText(hFileHandleGrid,nIoResult,temp2);
  		}
  	}
	}
	Close(hFileHandleGrid, nIoResult);
	if(alVuelo){
  	wait1Msec(1);
  }
	Delete(sFileName2, nIoResult);
	hFileHandleGrid = 2;
  OpenWrite(hFileHandleGrid, nIoResult, sFileName2, nFileSize);

	makePath(x_ini, y_ini, x_end, y_end, grid);


	for(int i = 0; i < sizePath; i++){
		string p;
		StringFormat(p, "%d, %d\n", pathX[i], pathY[i]);
		WriteText(hFileHandleGrid,nIoResult,p);
	}
	if(alVuelo){
  	wait1Msec(1);
  }
	Close(hFileHandleGrid, nIoResult);
}

/*
 *  Converts from cell coordinates to robot coordinates (x,y)
 */
void cellToPos(Pos position, int cellX, int cellY){
	position.x = cellX * 0.4 + 0.2;
	position.y = cellY * 0.4 + 0.2;
}

/*
 *  Converts from robot coordinates to cell coordinates
 */
void posToCell(Cell cell, float posX, float posY){
	cell.x = posX/0.4;
	cell.y = posY/0.4;
}

/*
 *  Returns true if given a position the robot detects an obstacle
 */
bool detectObstacle(int posx, int posy){
	// Epic
	return SensorValue[sonarSensorFrontal] < distanceToDetect && isConnected(posx,posy,facingDirection);
}

/*
 *  Draws the robot given a position
 */
void drawRobot(float x_mm, float y_mm, float ang_rad){
  int cellx,celly;
  int pixX,pixY;
  float ang_grad;
  int th;

  pos2cell(x_mm, y_mm, cellx,celly);

  pixX=cellx*pixPerX+pixPerX/2;
  pixY=celly*pixPerY+pixPerY/2;
  nxtFillEllipse(pixX-1, pixY+1, pixX+1, pixY-1); //nxtFillEllipse(Left, Top, Right, Bottom);

  //normalizeAngle(ang_rad);
  ang_grad=radiansToDegrees(ang_rad);
  if (ang_grad<0){ ang_grad=ang_grad+360;}
  th=(ang_grad+22.5)/45;
  while(th>7){th=th-8;}

	//paint orientation
	if(th==0)		    { nxtDrawLine(pixX,pixY,pixX+2,pixY);		}
	else if(th==1)	{ nxtDrawLine(pixX,pixY,pixX+2,pixY+2);	}
	else if(th==2)	{ nxtDrawLine(pixX,pixY,pixX,pixY+2);	  }
	else if(th==3)	{ nxtDrawLine(pixX,pixY,pixX-2,pixY+2);	}
	else if(th==4)	{ nxtDrawLine(pixX,pixY,pixX-2,pixY);		}
	else if(th==5)	{ nxtDrawLine(pixX,pixY,pixX-2,pixY-2);	}
	else if(th==6)	{ nxtDrawLine(pixX,pixY,pixX,pixY-2);		}
	else if(th==7)	{ nxtDrawLine(pixX,pixY,pixX+2,pixY-2);	}
}

/*
 *  Go from current position to the middle of the given cell
 */
void go(int cellX, int cellY){

	// obtains current position (odometry)
 	AcquireMutex(semaphore_odometry);
  float posX = robot_odometry.x;
  float posY = robot_odometry.y;
  float theta = robot_odometry.th;
  ReleaseMutex(semaphore_odometry);
	drawRobot(posX * 1000, posY * 1000, theta);
  // obtains the cell where the robot is
  Cell cell;
  posToCell(cell, posX, posY);
	short robotX = cell.x;
  short robotY = cell.y;

  Pos origin;
  cellToPos(origin, pathX[iLoop - 1], pathY[iLoop - 1]);
  Pos destination;
  cellToPos(destination, pathX[iLoop], pathY[iLoop]);

  float v;
  float w;
  float wgiro = 1.2;
  float thetaFinal;
  float errorTheta = 0.01;
  float umbralcito = 0.02;
  float errorDist = 0.01;
  bool ygriega = false;
  int div = 6;

	// checks the direction to go
	if (cellY > robotY) {

		// Going up
    thetaFinal = (PI)/2;
    facingDirection = 0;

		while(abs(theta - thetaFinal) > errorTheta) {
			// If theta no es 90 grados
			if(theta >= -(PI/2) && theta < (PI)/2){
				// Gire a la izquierda
				v = 0;
	    	w = wgiro;
			} else{
				// Gire a la derecha
				v = 0;
				w =-wgiro;
			}
			// Check if our orientation is correct
			AcquireMutex(semaphore_odometry);
      theta = robot_odometry.th;
      ReleaseMutex(semaphore_odometry);
      setSpeed(v,w,-1,-1);
		}
		ygriega = true;
	}
	else if (cellX > robotX) {

		// Going right
    thetaFinal = 0;
    facingDirection = 2;

		while(abs(theta - thetaFinal) > errorTheta) {
			// If theta no es 0 grados
			if(theta < 0){
				// Gire a la izquierda
				v = 0;
		  	w = wgiro;
			} else{
				// Gire a la derecha
				v = 0;
				w =-wgiro;
			}
			// Check if our orientation is correct
			AcquireMutex(semaphore_odometry);
      theta = robot_odometry.th;
      ReleaseMutex(semaphore_odometry);
      setSpeed(v,w,-1,-1);
		}

	}
	else if (cellY < robotY) {

		// Going down
    thetaFinal = -(PI/2);
    facingDirection = 4;

    // If theta no es -90 grados
		if(theta >= (PI/2) || theta < -(PI/2)){
			// Gire a la izquierda
			v = 0;
    	w = wgiro;
		} else{
			// Gire a la derecha
			v = 0;
			w =-wgiro;
		}
		while(abs(theta - thetaFinal) > errorTheta) {
			// Check if our orientation is correct
			AcquireMutex(semaphore_odometry);
      theta = robot_odometry.th;
      ReleaseMutex(semaphore_odometry);
      setSpeed(v,w,-1,-1);
		}
		ygriega = true;
	}
	else if (cellX < robotX) {

		// Going left
    thetaFinal = PI;
    facingDirection = 6;
    float fixTheta = theta;

    if(theta < 0){
    	fixTheta += 2*PI;
    }
		while(abs(fixTheta - thetaFinal) > errorTheta) {
			// If theta no es 180 grados
			if(theta > 0){
				// Gire a la izquierda
				v = 0;
	    	w = wgiro;
			} else{
				// Gire a la derecha
				v = 0;
				w =-wgiro;
			}
			// Check if our orientation is correct
			AcquireMutex(semaphore_odometry);
      theta = robot_odometry.th;
      ReleaseMutex(semaphore_odometry);
      fixTheta = theta;
      if(theta < 0){
	    	fixTheta += 2*PI;
	    }
      setSpeed(v,w,-1,-1);
		}

	}

	// the robot takes the next step

	// Check for obstacles
	bool obstacleDetected = false;

	while(euclideanDistance(posX,destination.x,posY,destination.y) > errorDist && !obstacleDetected){
      AcquireMutex(semaphore_odometry);
      posX = robot_odometry.x;
      posY = robot_odometry.y;
      theta = robot_odometry.th;
      ReleaseMutex(semaphore_odometry);

      // If its nearer from iLoop - 1
      if(euclideanDistance(posX,origin.x,posY,origin.y) <
      					euclideanDistance(posX, destination.x, posY, destination.y)){
      	// Check wall from iLoop - 1
      	obstacleDetected = detectObstacle(pathX[iLoop - 1], pathY[iLoop - 1]);
      } else {
				// Check wall from iLoop
      	obstacleDetected = detectObstacle(pathX[iLoop], pathY[iLoop]);
    	}


      if(abs(theta - thetaFinal) > umbralcito) {
      	if(thetaFinal == PI){ // Giro a la izquierda FIXME
					if(theta > 0){
						// Gire a la izquierda
			    	w = wgiro/(div-1);
					} else{
						// Gire a la derecha
						w =-wgiro/(div);
					}
				} else if(thetaFinal == -(PI/2)){ // Giro hacia abajo
			    // If theta no es -90 grados
					if(theta >= (PI/2) || theta < -(PI/2)){
						// Gire a la izquierda
			    	w = wgiro/(div-1);
					} else{
						// Gire a la derecha
						w =-wgiro/(div);
					}
				} else if(thetaFinal == 0){ // Giro a la derecha
					// If theta no es 0 grados
					if(theta < 0){
						// Gire a la izquierda
				  	w = wgiro/(div-1);
					} else{
						// Gire a la derecha
						w =-wgiro/(div);
					}
				} else if(thetaFinal == (PI)/2){ // Giro para arriba
					// If theta no es 90 grados
					if(theta >= -(PI/2) && theta < (PI)/2){
						// Gire a la izquierda
			    	w = wgiro/(div-1);
					} else{
						// Gire a la derecha
						w =-wgiro/(div);
					}
				}
			}

			setSpeed(0.1,w,-1,-1);
      		if(ygriega){
				posX = destination.x;
			} else{
				posY = destination.y;
			}
  }
  if(obstacleDetected){
  	setSpeed(0,0,-1,-1);
  }
}
