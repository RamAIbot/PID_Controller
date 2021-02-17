#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
   Kp = Kp_;
   Ki = Ki_;
   Kd = Kd_;
   
   p_error=0;
   i_error=0;
   d_error=0;

}

void PID::UpdateError(double cte,double prev_cte,double cte_sum) {
  /**
   * TODO: Update PID errors based on cte.
   */
   p_error = -Kp * cte;
   d_error = -Kd * (cte - prev_cte);
   i_error = -Ki * cte_sum;
	
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return (p_error+d_error+i_error);  // TODO: Add your total error calc here!
}
