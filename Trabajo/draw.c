int pixPerX;
int pixPerY;


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