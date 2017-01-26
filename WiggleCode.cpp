/*
 * WiggleCode.cpp
 *
 *  Created on: Feb 15, 2016
 *      Author: Robot
 */
float TargetAcquired(long double error, double p, double d){//error is 320.0-Miku.
	static double derr{0}, preverror{0};
	float output=p*error+d*derr/.0333333;
	derr=error-preverror;
	return (output/10);
}



