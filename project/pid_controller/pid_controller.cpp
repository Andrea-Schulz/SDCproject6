/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   //Initialize PID coefficients (and errors, if needed)
   Kp = Kpi;
   Ki = Kii;
   Kd = Kdi;
   output_lim_max = output_lim_maxi;
   output_lim_min = output_lim_mini;
   cte_curr = 0.0;

}


void PID::UpdateError(double cte) {
   cte_prev = cte_curr;
   cte_curr = cte;
   cte_int += cte * delta_time;

}

double PID::TotalError() {
   //Calculate and return the total error
   //The code should return a value in the interval [output_lim_mini, output_lim_maxi]

    double control = -Kp*cte_curr - Kd*(cte_curr-cte_prev)/delta_time - Ki*cte_int;
    if(control > output_lim_max){
      control = output_lim_max;
    }else if(control < output_lim_min){
      control = output_lim_max;
    }
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   //Update the delta time with new value
   delta_time = new_delta_time;

}