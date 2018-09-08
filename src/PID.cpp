#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;

    PID::p_error = 0.0;
    PID::i_error = 0.0;
    PID::d_error = 0.0;
    PID::last_p_error = 0.0;

}

void PID::UpdateError(double cte) {
    PID::p_error = cte;
    PID::i_error += cte;
   
    if (PID::last_p_error != 0){
        PID::d_error = cte - PID::last_p_error;
    }
}

double PID::TotalError() {

    double pid = 0.0;
    pid = -PID::Kp * PID::p_error;
    pid = pid - PID::Ki * PID::i_error;
    pid = pid - PID::Kd * PID::d_error;
    return pid;

}

