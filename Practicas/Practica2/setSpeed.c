#pragma platform(NXT)

//Constants
const double L = 0.126;
const double R = 0.026;

//Relacion potencia - w
const double MD = ;
const double MI = ;

void setSpeed(double v, double w){

		//Calculamos la velocidad de las ruedas
		double wd = (L * w + 2 * v)/(2*R);
		double wi = (2*v - R*wd)/R;

		//Establecemos la potencia de cada rueda
		motor[motorA] = MI * wi;
		motor[motorC] = MD * wd;
}

task main()
{
		setSpeed();
}
