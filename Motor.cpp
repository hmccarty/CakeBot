#include "Motor.h"

Motor::Motor(byte forward, byte reverse,
             byte clk, byte dt,
             double Kp, double Ki, double Kd,
             double min_output, double max_output,
             unsigned long sample_time) 
  : m_enc(clk, dt), m_pid(&curr_velocity, &pwm_output, &wanted_velocity, Kp, Ki, Kd, DIRECT)
{
    this->forward = forward;
    this->reverse = reverse;

    pwm_output = 0;
    this->min_output = min_output;
    this->max_output = max_output;

    prev_position = 0;
    curr_position = 0;
    wanted_position = 0; 

    curr_velocity = 0;
    wanted_velocity = 0;

    prev_time = 0;
    this->sample_time = sample_time;
}

void Motor::setup() {
    pinMode(forward, OUTPUT);
    pinMode(reverse, OUTPUT);

    m_pid.SetMode(AUTOMATIC);
    m_pid.SetOutputLimits(min_output, max_output);
}

void Motor::reset_encoder() {
    m_enc.readAndReset();
}

void Motor::execute() {
  unsigned long curr_time = millis();
  unsigned long time_change = (curr_time - prev_time);
  curr_position = (double) m_enc.read();

  if (time_change >= sample_time) {
    curr_velocity = ((curr_position - prev_position) / (double) time_change);

    prev_position = curr_position;
    prev_time = curr_time;
  }

  m_pid.Compute();
  set_duty_cycle(pwm_output);
}

void Motor::set_duty_cycle(double pwm) {
  if (abs(pwm) < MIN_PWM) {
    pwm = 0;
  }

  if (pwm > 0) {
    analogWrite(forward, pwm);
    analogWrite(reverse, 0);
  } else {
    analogWrite(forward, 0);
    analogWrite(reverse, -pwm);
  }
}

long Motor::get_position() {
    return m_enc.read();
}

double Motor::get_velocity() {
    return curr_velocity;
}

void Motor::set_position(double position) {
  wanted_position = position;
}

double Motor::get_wanted() {
  return wanted_velocity;
}

double Motor::get_max() {
  return max_output;
}

double Motor::get_min() {
  return min_output;
}

void Motor::set_velocity(double wanted_velocity) {
  this->wanted_velocity = wanted_velocity;
}

void Motor::set_pid(double Kp, double Ki, double Kd) {
  m_pid.SetTunings(Kp, Kd, Ki);
}

void Motor::set_limits(double max_output, double min_output) {
  this->max_output = max_output;
  this->min_output = min_output;
}
