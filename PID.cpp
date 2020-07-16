#include <Arduino.h>
#include "PID.h"

template <typename T> int sgn(T val);

PID::PID(double *actual, double *goal,
         double kp, double ki, double kd,
         double max_output, double min_output) {
    this->actual = actual;
    this->goal = goal;
    this->output = nullptr;

    this->kp = kp;
    this->ki = ki;
    this->kd = kd;

    this->max_output = max_output;
    this->min_output = min_output;

    prev_error = 0;
    error_sum = 0;

    prev_time = 0;
    sample_time = 10;
}

PID::PID(double *actual, double *goal,
         double kp, double ki, double kd,
         double max_output, double min_output,
         unsigned long sample_time) {
    this->actual = actual;
    this->goal = goal;
    this->output = nullptr;

    this->kp = kp;
    this->ki = ki;
    this->kd = kd;

    this->max_output = max_output;
    this->min_output = min_output;

    this->sample_time = sample_time;
    
    prev_error = 0;
    error_sum = 0;

    prev_time = 0;
}

double PID::calculate(unsigned long curr_time) {
    double error = (*goal) - (*actual);

    double p = kp * error;
    
    if ((error == 0) || (sgn(error) != sgn(prev_error))) {
        error_sum = 0;
    } else {
        error_sum += error;
    }

    double i = ki * error_sum;

    double d = kd * ((error - prev_error) / (curr_time - prev_time));

    double output = p + i + d;
    if (output > max_output) {
        output = max_output;
    } else if (output < min_output) {
        output = min_output;
    }

    prev_error = error;
    prev_time = curr_time;

    return output;
}

void PID::execute() {
    unsigned long curr_time = millis();
    unsigned long dt = curr_time - prev_time;

    if (dt >= sample_time) {
        *output =  calculate(curr_time);
    }
}

double PID::get_kp() {
    return kp;
}

double PID::get_ki() {
    return ki;
}

double PID::get_kd() {
    return kd;
}

double PID::get_max() {
    return max_output;
}

double PID::get_min() {
    return min_output;
}

unsigned long PID::get_sample_time() {
    return sample_time;
}

void PID::set_p(double kp) {
    this->kp = kp;
}

void PID::set_i(double ki) {
    this->ki = ki;
}

void PID::set_d(double kd) {
    this->kd = kd;
}

void PID::set_max(double max_output) {
    this->max_output = max_output;
}

void PID::set_min(double min_output) {
    this->min_output = min_output;
}

void PID::set_sample_time(unsigned long sample_time) {
    this->sample_time = sample_time;
}

void PID::set_output(double *output) {
    this->output = output;
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
