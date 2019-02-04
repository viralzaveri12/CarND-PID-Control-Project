#include "PID.h"

using namespace std;

 /*
  * Constructor
  */
PID::PID() {}

 /*
  * Destructor
  */
PID::~PID() {}

 /*
  * Initilizes PID parameters
  */
void PID::Init(double Kp, double Ki, double Kd) {
	
	PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;
	
    p_error = 0.0;
	d_error = 0.0;
	i_error = 0.0;
}

 /*
  * Update error based on the current cte (cross track error)
  */
void PID::UpdateError(double cte) {
	
	double prev_cte;

	prev_cte = p_error;
	p_error  = cte;
	i_error += cte;
	d_error  = cte - prev_cte;
}

 /*
  * Compute and return total error
  */
double PID::TotalError() {
	
	return Kp*p_error + Ki*i_error + Kd*d_error;
}

