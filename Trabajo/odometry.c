#pragma config(Sensor, S4,     HTGYRO,              sensorAnalogInactive)
/*
 *  odometry.c
 *  Manages odometry of the robot
 */


// Odometry file
TFileIOResult nIoResultOd;
TFileHandle hFileHandleOd;
long nFileSizeOd = 10000; 			//1 byte each char...

/*
 *  Updates the odometry (INITIALIZE robot_odometry BEFORE USING IT)
 */
task updateOdometry(){

  float cycle = 0.01; 								// we want to update odometry every 0.01 s
  int step = 20;            					// we want to write odometry data each 20 steps
  float dSl,dSr,dS,dx,dy,dT;
  float x, y, th;
  string odometryString = "";
  strcat(odometryString, "odometry = ["); // concatenate string2 into string1

  string sFileName = "odometrylog.txt";
  hFileHandleOd = 5;
  Close(hFileHandleOd, nIoResultOd);
  //
  // Deletes the file if it already exists
  //
  Delete(sFileName, nIoResultOd);
  OpenWrite(hFileHandleOd, nIoResultOd, sFileName, nFileSizeOd);
  WriteText(hFileHandleOd, nIoResultOd, odometryString);
  float timeAux = 0;
  float timeAux2;
  float timeAuxOld = 0;
  float mseconds;
  float rotSpeed;

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

    // read tachometers, and estimate how many m. each wheel has moved since
    // last update
    // RESET tachometer right after to start including the "moved" degrees
    // turned in next iteration
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
