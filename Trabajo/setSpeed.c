/*
 *  setSpeed.c
 *  Manages speed control of the Robot
 *  controlSpeed task must be active while calling to
 *  setSpeed to control the speed of the Robot
 *  You can also call to classic setSpeedBase for manual speedControl
 */

// Semaphore for accesing speed variables
TMutex access_speed = 0;  // Important to initialize to zero!!! Not acquired.

// Speed parameters
float objV = 0;   // Objective V speed
float objW = 0;   // Objective W speed
float curV = 0;   // Current V speed
float curW = 0;   // Current W speed
float incV = 0;  // Value for modifying the current V speed
float incW = 0;  // Value for modifying the current W speed

// Sets speed variables for automatic adjustement
void setSpeed(float oV, float oW, float iV, float iW)
{
  AcquireMutex(access_speed);
  objV = oV;
  objW = oW;
  incV = iV;
  incW = iW;
  ReleaseMutex(access_speed);
  wait1Msec(1); // To force timeslice to end and give other threads time
}

// Sets speed to motors
int setSpeedBase(float v, float w)
{
  // Start the motors so that the robot gets
  // v m/s linear speed and w RADIAN/s angular speed
  float w_r = (L * w + 2 * v)/(2*R);
  float w_l = (2*v - R*w_r)/R;

  // Parameters of power/speed transfer
  float mR = 5.5058, mL = 5.5092, nR = 1.4976,  nL = 1.8269;
  //float mR = 5.80117, mL = 5.76965, nR = -0.20796,    nL = 0.138482;
  float motorPowerRight, motorPowerLeft;

  // Sets the power for both motors
  if(v == 0 && w == 0){
  		motorPowerLeft = 0;
 			motorPowerRight = 0;
	} else{
			motorPowerLeft = mL * w_l + nL;
  		motorPowerRight = mR * w_r + nR + 1.15;
	}

	// Set current speed with semaphore
    AcquireMutex(access_speed);
    curV = v;
    curW = w;
    ReleaseMutex(access_speed);

  // Checks if calculated power exceeds the motors capacity
  if (motorPowerLeft <= 80 && motorPowerRight <= 80) {
    hogCPU();
    motor[motorA] = motorPowerLeft;
    motor[motorC] = motorPowerRight;
    releaseCPU();
    return 1;
  } else {
    // Too much power, sets the power to the maximum possible
    hogCPU();
    motor[motorA] = 80;
    motor[motorC] = 80;
    releaseCPU();
    return 0;
  }
}

// Takes control of the speed
task controlSpeed()
{
  // Cycle variables
  float cycle = 0.01; 			// We want to apply speed changes every 0.01 s
  float timeAux;
  float timeAux2;
  float finalV;
  float finalW;
  float finalIncV;
  float finalIncW;
  // Copy of speed variables accessed through semaphore
  float _objV, _curV, _incV, _objW, _curW, _incW = 0;

  // Main loop
  while (true){
    // Cycle control
    timeAux = nPgmTime;
    timeAux2 = 0;

    // Obtain real variables with semaphore
    AcquireMutex(access_speed);
    _objV = objV;
    _curV = curV;
    _incV = incV;
    _objW = objW;
    _curW = curW;
    _incW = incW;
    ReleaseMutex(access_speed);

    // Manage V speed
    if(_objV != _curV){
      // V speed needs adjustement
      if(_incV <= 0){
        // Instant change
        finalV = _objV;
      } else{
        // Incremental variable sign adjustement
        if(_objV > _curV){
          // Needs positive adjustement
          finalIncV = _incV;
        } else {
          // Needs negative adjustement
          finalIncV = (-1)*_incV;
        }

        // Incremental change
        finalV = _curV + finalIncV;
      }
    } else{
      // V speed doesn't need adjustement
      finalV = _curV;
    }

    // Manage W speed
    if(_objW != _curW){
      // W speed needs adjustement
      if(_incW <= 0){
        // Instant change
        finalW = _objW;
      } else{
        // Incremental variable sign adjustement
        if(_objW > _curW){
          // Needs positive adjustement
          finalIncW = _incW;
        } else {
          // Needs negative adjustement
          finalIncW = (-1)*_incW;
        }

        // Incremental change
        finalW = _curW + finalIncW;
        nxtDisplayTextLine(2, "%2.2f", finalW);
      }
    } else{
      // W speed doesn't need adjustement
      finalW = _curW;
    }

    // Adjust speed
    setSpeedBase(finalV, finalW);

    // Wait until cycle is completed
    timeAux2 = nPgmTime;
    if ((timeAux2 - timeAux) < (cycle * 1000)) {
        Sleep( (cycle * 1000) - (timeAux2 - timeAux) );
    }
  }
}
