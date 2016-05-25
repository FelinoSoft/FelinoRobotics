bool doReDrawMap = false;

// DRAW map for the first time
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

// Draw map another time
void reDrawMap(){
  int i,j,cx,cy;

  // Dont erase display
  //eraseDisplay(); // L_B: (0,0); T_T: (99,63)
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
