/*
 *  commonAux.c
 *  Manages common variables of the Robot
 *  such as its position and the odometry semaphore.
 *  It does also contain auxiliary common functions
 */

/* Common variables */
// Robot parameters
float R = 0.026; 			// m
float L = 0.128; 			// m

// Odometry
TPosition robot_odometry;       // WE SHOULD ACCESS THIS VARIABLE with a "semaphore".
TMutex semaphore_odometry = 0;  // Important to initialize to zero!!! Not acquired.

/* Common functions */

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
    while (theta < -(2*(PI))) {
        theta = 2*(PI) + theta;
    }

    while (theta >= (2*(PI))){
        theta = theta - 2*(PI);
    }

    if (theta > (PI)) {
        theta = theta - (2*(PI));
    }
    if (theta < -(PI)) {
    	theta = theta + (2*(PI));
    }

    return theta;
}

float euclideanDistance(float x1, float x2, float y1, float y2){
    return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}
