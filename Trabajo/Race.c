#pragma config(Sensor, S2,     sonarSensorFrontal,         sensorSONAR)
#pragma config(Sensor, S4,     HTGYRO,              sensorAnalogInactive)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//
#include "mutexLib.c"
#include "positionLib.c"
#include "common.h"
#include "hitechnic-gyro.h"

TFileIOResult nIoResult;
TFileHandle hFileHandle;
long nFileSize                   = 5000; //1 byte each char..

// variables globales de las dimensiones del mapa
int sizeX;
int sizeY;
int sizeCell;
int pixPerX;
int pixPerY;

// DEFINIR EL MAXIMO NUMERO DE CELDAS QUE PODEMOS UTILIZAR
#define MAX_X 10
#define MAX_Y 10

bool connectionsMatrix[2*MAX_X+1][2*MAX_Y+1];
int pathX[MAX_X*MAX_Y];
int pathY[MAX_X*MAX_Y];
int sizePath;

float distanceToDetect = 20;
int facingDirection = 2;
int iLoop;

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


// Odometry file
TFileIOResult nIoResultOd;
TFileHandle hFileHandleOd;
long nFileSizeOd = 10000; 			//1 byte each char...

TPosition robot_odometry;       // WE SHOULD ACCESS THIS VARIABLE with a "semaphore".
TMutex semaphore_odometry = 0;  // IMPORTANT to initialize to zero. Not acquired.

float R = 0.026; 								// m
float L = 0.128; 								// m


// Converts degrees to radians
float degToRad(float degrees)
{
    return (degrees * (PI)) /180;
}

float radToDeg(float rads)
{
		return ((rads * 180)) / (PI);
}

// Normalizes theta [-pi,pi]
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

// Updates the odometry
task updateOdometry(){

  float cycle = 0.01; 										// we want to update odometry every 0.01 s
  int step = 20;            							// we want to write odometry data each 20 steps
  float dSl,dSr,dS,dx,dy,dT;
  float x, y, th;
  string odometryString = "";
  strcat(odometryString, "odometry = ["); // concatenate string2 into string1

  string sFileName = "odometrylog.txt";
  CloseAllHandles(nIoResultOd);
  //
  // Deletes the file if it already exists
  //
  Delete(sFileName, nIoResultOd);
  hFileHandleOd = 0;
  OpenWrite(hFileHandleOd, nIoResultOd, sFileName, nFileSizeOd);
  WriteText(hFileHandleOd, nIoResultOd, odometryString);
  float timeAux = 0;
  float timeAux2;
  float timeAuxOld = 0;
  float mseconds;
  float rotSpeed;
	//errorGyro = normTheta( degToRad(errorGyro) * (-1) );

  while (true){
    // show each step on screen and write in the string
		timeAuxOld = timeAux;
    timeAux = nPgmTime;
    mseconds = (timeAux - timeAuxOld);

    // GIROSCOPO
    // Reads gyro value
    rotSpeed = HTGYROreadRot(HTGYRO);
    if(abs(rotSpeed) <= 4){
    		rotSpeed = 0;
  	}
  	else {
  		rotSpeed = rotSpeed - 0.78;
  	}

    rotSpeed = normTheta( degToRad(rotSpeed) * (-1) );
    dT = rotSpeed * (mseconds / 1000);

    timeAux2 = 0;

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

  	dSl = R * degToRad(dSl);
    dSr = R * degToRad(dSr);
    dS = (dSr + dSl) / 2;

		dx = dS * cos(robot_odometry.th + (dT/2));
	  dy = dS * sin(robot_odometry.th + (dT/2));

    x = robot_odometry.x + dx;
    y = robot_odometry.y + dy;
    th = normTheta(robot_odometry.th + dT);

    // Updates odometry
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
    		WriteText(hFileHandleOd, nIoResultOd, temp);
    }
    step++;

    // Wait until cycle is completed
    timeAux2 = nPgmTime;
    if ((timeAux2 - timeAux) < (cycle * 1000)) {
        Sleep( (cycle * 1000) - (timeAux2 - timeAux) );
    }
  }
}

// Sets speed to motors
int setSpeed(float v, float w)
{

  // start the motors so that the robot gets
  // v m/s linear speed and w RADIAN/s angular speed
    float w_r = (L * w + 2 * v)/(2*R);
    float w_l = (2*v - R*w_r)/R;

  // parameters of power/speed transfer
  float mR = 5.5058, mL = 5.5092, nR = 1.4976,  nL = 1.8269;
  //float mR = 5.80117, mL = 5.76965, nR = -0.20796,    nL = 0.138482;
  float motorPowerRight, motorPowerLeft;

  // sets the power for both motors
  if(v == 0 && w == 0){
  		motorPowerLeft = 0;
 			motorPowerRight = 0;
	} else{
			motorPowerLeft = mL * w_l + nL;
  		motorPowerRight = mR * w_r + nR + 1.15;
	}


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


void initConnections(){
     for(int i=0; i<2*MAX_X+1; ++i){
        for (int j=0; j<2*MAX_Y+1; ++j){
          connectionsMatrix[i][j]=false;
        }
     }

}

/* pasar de una celda(cellX,cellY) y un indice de vecindad numNeigh,
al correspondiente punto en la matriz de conexiones (connX,connY) */
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

void setConnection(int cellX, int cellY, int numNeigh){
  int connX, connY; // coordinates in the connection matrix
  // from coordinates in the grid of cells to coordinates in the connection matrix
  cell2connCoord(cellX, cellY, numNeigh, connX, connY);
  connectionsMatrix[connX][connY]=true;
}

void deleteConnection(int cellX, int cellY, int numNeigh){
  int connX, connY; // coordinates in the connection matrix
  // from coordinates in the grid of cells to coordinates in the connection matrix
  cell2connCoord(cellX, cellY, numNeigh, connX, connY);
  connectionsMatrix[connX][connY]=false;
}

bool isConnected(int cellX, int cellY, int numNeigh){
   int connX, connY; // coordinates in the connection matrix
  // from coordinates in the grid of cells to coordinates in the connection matrix
  cell2connCoord(cellX, cellY, numNeigh, connX, connY);
  return(connectionsMatrix[connX][connY]);

}


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
              nxtDisplayTextLine(1, "PROBLEM READING map");
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
                  nxtDisplayTextLine(3, " %d %d", mapCol,mapRow);
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
              nxtDisplayTextLine(1, "PROBLEM READING map");
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

     nxtDisplayTextLine(3, "%s ", linestring);

     /*for(int j=2; j<=indNum; ++j){
        setConnection(numbersRead[0], numbersRead[1], numbersRead[j]);
        nxtDisplayTextLine(4, "%d connection open", numbersRead[j]);
        //wait10Msec(200);
     }*/
     return endfile;
	}

////////////////////////////////////////////////////////////////
// load map from a txt to the occupancy and connection matrix
// FILL GLOBAL VARIABLES dimX dimY cellSize
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

     CloseAllHandles(nIoResult);
     hFileHandle = 0;
	   //nxtDrawLine(_x+2, _y, _x-2, _y);

	   OpenRead(hFileHandle, nIoResult, mapFileName, nFileSize);
	   if( nIoResult ==0 ){
	       nxtDisplayTextLine(1, "OPEN OK: %d", nFileSize);

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
	        }
	        loadingOk=true;
	        Close(hFileHandle, nIoResult);
     }
     else{
           loadingOk=false;
           nxtDisplayTextLine(1, "PROBLEM OPENING file");
     }

	   return loadingOk;
}


// DRAW map and robot


void drawMap(){
  int i,j,cx,cy;

  eraseDisplay(); // L_B: (0,0); T_T: (99,63)
  //nxtDrawRect(_l, _t, _r, _b);
  pixPerX=100/sizeX;
  pixPerY=64/sizeY;

  nxtDrawRect(0,sizeY*pixPerY,sizeX*pixPerX,0);

  //nxtDrawLine(xPos, yPos, xPosTo, yPosTo);
  //i=cellX*sizeY+cellY;

  // check "vertical" walls
  for (i=2; i<2*sizeX; i=i+2){
    for (j=1; j< 2*sizeY; j=j+2){
      if (connectionsMatrix[i][j]==false){
          // paint "right" wall from cell (i/2-1, j2-1)
          cx=(i-1)/2;
          cy=(j-1)/2;
          nxtDrawLine((cx+1)*pixPerX, cy*pixPerY, (cx+1)*pixPerX, (cy+1)*pixPerY);
      }
    }
  }


  // check "horizontal" walls
  for (j=2; j<2*sizeY; j=j+2){
    for (i=1; i< 2*sizeX; i=i+2){
      if (connectionsMatrix[i][j]==false){
          // paint "top" wall from cell (i-1)/2, (j-1)/2)
          cx=(i-1)/2;
          cy=(j-1)/2;
          nxtDrawLine((cx)*pixPerX, (cy+1)*pixPerY, (cx+1)*pixPerX, (cy+1)*pixPerY);
      }
    }
  }

}

/* Convert from robot odometry coordinates (in mm) to cell coordinates */
void pos2cell(float x_mm, float y_mm, int & x_cell, int & y_cell){

  x_cell =  (int) x_mm/sizeCell;

  y_cell = (int) y_mm/sizeCell;

}

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

void calcWaveFront(Grid g, int x, int y, int value){
	if(x >= 0 && y >= 0 && x < 2*sizeX+1 && y < 2*sizeY+1){
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

/* FUNCIONES A IMPLEMENTAR */

// Make path
void makePath(int x_ini, int y_ini, int x_end, int y_end, Grid grid){
	int n = 0;
	if(x_ini % 2 != 0 && y_ini % 2 !=0){
		pathX[n] = (x_ini - 1) / 2;
		pathY[n] = (y_ini - 1) / 2;
	}

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
						bestX = candidatoX;
						bestY = candidatoY;
						indexMove = i;
				}
			}
		} else{
			// Jump
			bestX = bestX + neighborX[indexMove];
			bestY = bestY + neighborY[indexMove];
		}

		if(bestX % 2 != 0 && bestY % 2 !=0){
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

void planPath(int x_ini, int y_ini, int x_end,int y_end){
// Store in pathX and pathY respectively the coordinates of all the cells that the robot has to cross to reach the goal.

  // NF1

	// Inicializar matriz
	Grid grid;

	//Falta darle valores
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

	grid.grid[x_end][y_end] = 0;

	calcWaveFront(grid,x_end-1,y_end,1);
	calcWaveFront(grid,x_end+1,y_end,1);
	calcWaveFront(grid,x_end,y_end-1,1);
	calcWaveFront(grid,x_end,y_end+1,1);

	//INICIALIZARRRR FICHERRROOOS
	string sFileName = "grid.txt";
  CloseAllHandles(nIoResult);

  // Deletes the file if it already exists
  Delete(sFileName, nIoResult);

  hFileHandle = 0;
  OpenWrite(hFileHandle, nIoResult, sFileName, nFileSize);

  // Escribe la matriz resultante del algoritmo NF1 en un fichero

  for(int i = 2*sizeY; i >= 0;i--){
  	for(int j = 0; j < 2*sizeX+1;j++){
  		string temp, temp2;
  		if(grid.grid[j][i] > -1 && grid.grid[j][i] < 10){
  			StringFormat(temp, " %d ", grid.grid[j][i]);
  		} else {
  		  StringFormat(temp, "%d ", grid.grid[j][i]);
  	  }
  		WriteText(hFileHandle,nIoResult,temp);
  		if(j == 2*sizeX){
  			StringFormat(temp2, "\n");
  			WriteText(hFileHandle,nIoResult,temp2);
  		}
  	}
	}



	makePath(x_ini, y_ini, x_end, y_end, grid);


	for(int i = 0; i < sizePath; i++){
		string p;
		StringFormat(p, "%d, %d\n", pathX[i], pathY[i]);
		WriteText(hFileHandle,nIoResult,p);
	}

	CloseAllHandles(nIoResult);
}

void cellToPos(Pos position, int cellX, int cellY){
	position.x = cellX * 0.4 + 0.2;
	position.y = cellY * 0.4 + 0.2;
}

void posToCell(Cell cell, float posX, float posY){
	cell.x = posX/0.4;
	cell.y = posY/0.4;
}

float euclideanDistance(float x1, float x2, float y1, float y2){
    return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}

// DetectObstacle
bool detectObstacle(int posx, int posy){
	// Epic
	return SensorValue[sonarSensorFrontal] < distanceToDetect && isConnected(posx,posy,facingDirection);
}

// go from CURRENT position (odometry) to middle of cell (cellX, cellY)
void go(int cellX, int cellY){

	// obtains current position (odometry)
 	AcquireMutex(semaphore_odometry);
  float posX = robot_odometry.x;
  float posY = robot_odometry.y;
  float theta = robot_odometry.th;
  ReleaseMutex(semaphore_odometry);

  // obtains the cell where the robot is
  Cell cell;
  posToCell(cell, posX, posY);
  nxtDisplayTextLine(1, "CellI %d %d", cell.x,cell.y);
  nxtDisplayTextLine(2, "CellOb %d %d", cellX,cellY);
	short robotX = cell.x;

  short robotY = cell.y;
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
      setSpeed(v,w);
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
      setSpeed(v,w);
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
      setSpeed(v,w);
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
      setSpeed(v,w);
		}

	}

	Pos p;
	cellToPos(p,cellX,cellY);
	nxtDisplayTextLine(3, "PosR %.2f %.2f", posX,posY);
	nxtDisplayTextLine(4, "PosF %.2f %.2f", p.x,p.y);

	// the robot takes the next step

	// Check for obstacles
	bool obstacleDetected = false;

	while(euclideanDistance(posX,p.x,posY,p.y) > errorDist && !obstacleDetected){
			obstacleDetected = detectObstacle(pathX[iLoop-1], pathY[iLoop-1]);
      AcquireMutex(semaphore_odometry);
      posX = robot_odometry.x;
      posY = robot_odometry.y;
      theta = robot_odometry.th;
      ReleaseMutex(semaphore_odometry);


      if(abs(theta - thetaFinal) > umbralcito) {
      	if(thetaFinal == PI){ // Giro a la izquierda FIXME
					if(theta > 0){
						// Gire a la izquierda
			    	w = wgiro/div;
					} else{
						// Gire a la derecha
						w =-wgiro/div;
					}
				} else if(thetaFinal == -(PI/2)){ // Giro hacia abajo
			    // If theta no es -90 grados
					if(theta >= (PI/2) || theta < -(PI/2)){
						// Gire a la izquierda
			    	w = wgiro/div;
					} else{
						// Gire a la derecha
						w =-wgiro/div;
					}
				} else if(thetaFinal == 0){ // Giro a la derecha
					// If theta no es 0 grados
					if(theta < 0){
						// Gire a la izquierda
				  	w = wgiro/div;
					} else{
						// Gire a la derecha
						w =-wgiro/div;
					}
				} else if(thetaFinal == (PI)/2){ // Giro para arriba
					// If theta no es 90 grados
					if(theta >= -(PI/2) && theta < (PI)/2){
						// Gire a la izquierda
			    	w = wgiro/div;
					} else{
						// Gire a la derecha
						w =-wgiro/div;
					}
				}
			}

      nxtDisplayTextLine(1, "CellI %d %d", cell.x,cell.y);
  		nxtDisplayTextLine(2, "CellOb %d %d", cellX,cellY);
      nxtDisplayTextLine(3, "PosR %.2f %.2f", posX,posY);
			nxtDisplayTextLine(4, "PosF %.2f %.2f", p.x,p.y);
			nxtDisplayTextLine(6, "F: %d", facingDirection);
			//float w = (thetaFinal - theta)*10;
			//w = w - 0.9;
			nxtDisplayTextLine(5, "w: %.2f", w);
			setSpeed(0.1,w);
      if(ygriega){
				posX = p.x;
			} else{
				posY = p.y;
			}
  }
  if(obstacleDetected){
  	setSpeed(0,0);
  }
}

task main()
{
  /* Prepare for the Race */

  // Start gyroscope calibration
  HTGYROstartCal(HTGYRO);

  // Start tasks
  // TODO Set Speed Task
  // Odometry Task
  set_position(robot_odometry, p.x, p.y, 0);
  StartTask(updateOdometry);

		// Calibrate the gyro, make sure you hold the sensor still
		int x_ini = 1;
		int y_ini = 1;
		int x_end = 15;
		int y_end = 1;

		// Inicializa la malla
		string map_file = "mapa3.txt";
		initConnections();

		if(	loadMap(map_file) ){
		  nxtDisplayTextLine(6, "Mapa loaded ok");
		}else{
		  nxtDisplayTextLine(6, "Mapa NOT loaded");
		}
		HTGYROstartCal(HTGYRO);
		planPath(x_ini, y_ini, x_end, y_end);
		Pos p;
		cellToPos(p,0,0);


		Sleep(1000);

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
		StopTask(updateOdometry);
 		Close(hFileHandleOd, nIoResultOd);
		nxtDisplayTextLine(5, "FIN");
}
