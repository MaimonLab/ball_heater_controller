#ifndef BallHeater_H
#define BallHeater_H

#include "Arduino.h"
#include <Servo.h>
#include <PID_v1.h>
// #include "DataLogger.h"

// Control Modes
#define STANDBY 0
#define LOCAL_CONTROL 1
#define REMOTE_CONTROL 2
#define MANUAL_TEST 3

#define HIGH_TEMP_ERROR 6
#define NO_TEMP_ERROR 7

class BallHeater
{
public:
    BallHeater();
    // BallHeater(byte heatPin, byte coolPin );
    // void init(void);
    void init(void);

    void tick(void);
    void check_power_status(void);
    void set_pwm(float pwm_goal);
    void set_cooling_mode(byte cool_mode);
    void set_target_temp(float target_temp);

    void set_control_mode(byte mode);
    void set_pid_params(double kp, double ki, double kd);

    void read_temps(void);

    float read_thermistor(byte pin);
    float get_pwm_float();
    float get_target_temp();
    float get_heater_temp();
    float get_aux_therm_temp();

    double get_pid_kp();
    double get_pid_ki();
    double get_pid_kd();
    byte get_control_mode();

private:
    void check_read_time();

    uint32_t _last_log = 0;
    uint32_t _last_temp_read = 0;

    byte _heat_pin = 9;

    const byte _aux_therm = A2;
    const byte _main_therm = A3;
    const int _stale_time = 1000;

    unsigned long _pid_update; // last time pid output calculated (ms)

    float _pwm_float = 0;
    unsigned int _pwm_level = 0;

    const unsigned int _pwm_max = 0x07FF; // 11-bit resolution.  7812 Hz PWM

    byte _controller_mode = 0;

    double _target_temp = 25;
    float _heater_temp;
    float _aux_therm_temp;

    double _pid_input;
    double _pid_output;
    double _kp = 120, _ki = 0.2, _kd = 0.1;
    PID _pid = PID(&_pid_input, &_pid_output, &_target_temp,
                   _kp, _ki, _kd, DIRECT);
};

#endif
