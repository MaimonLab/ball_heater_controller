#include "Arduino.h"
#include "BallHeater.h"

// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
#define NUMSAMPLES 5 // thermistor samples
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3988
#define SERIESRESISTOR 10000

#define LOG_INTERVAL 1000

#define ECHO_TO_SERIAL 0

#define HEATER_MAX_PERCENT 90
#define MAX_HEATER_TEMP 60

// Default Constructor
//  BallHeater::BallHeater(){}

/**
    Constructor for the BallHeater class.

    Initializes the hardware peripherals involved.
*/
BallHeater::BallHeater()
{
}

/*-------------------- init -------------------------
      Initializes the aspects of the BallHeater instance.
 --------------------------------------------------------*/
void BallHeater::init()
{
    //    analogReadResolution(12);
    // data_logger = DataLogger("pwm,hot_temp,cold_temp,therm_3,air_temp,humid");

    // set some PID parameters
    _pid.SetOutputLimits(0, HEATER_MAX_PERCENT);
    // _pid.SetSampleTime(1000); // 1 sec.

    delay(200); // make sure things are settled before starting.
    pinMode(_heat_pin, OUTPUT);

    // Stop Timer/Counter1
    TCCR1A = 0; // Timer/Counter1 Control Register A
    TCCR1B = 0; // Timer/Counter1 Control Register B
    TIMSK1 = 0; // Timer/Counter1 Interrupt Mask Register
    TIFR1 = 0;  // Timer/Counter1 Interrupt Flag Register
    ICR1 = _pwm_max;
    OCR1A = 0; // Default to 0% PWM
    OCR1B = 0; // Default to 0% PWM

    // Set clock prescale to 1 for maximum PWM frequency
    TCCR1B |= (1 << CS10);

    // Set to Timer/Counter1 to Waveform Generation Mode 14: Fast PWM with TOP set by ICR1
    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM13) | (1 << WGM12);

    // Enable Fast PWM on Pin 9: Set OC1A at BOTTOM and clear OC1A on OCR1A compare
    TCCR1A |= (1 << COM1A1);
    pinMode(9, OUTPUT);
}

/*-------------------- tick -------------------------
    Updates the drive PWM level and recalculates PID if needed
 ---------------------------------------------------------*/
void BallHeater::tick()
{
    // digitalWrite(6, HIGH);

    // read temps if measuremnts stale.
    this->check_read_time();

    // check if heater is overtemp and shut down +
    // go into error mode until dropped.
    // TODO: add recovery from error mode and alerts of error mode.
    // Currently just turns pwm off, and locks up...
    if (_heater_temp > MAX_HEATER_TEMP)
    {
        this->set_control_mode(HIGH_TEMP_ERROR);
        this->set_pwm(0);
    }

    // Run pid / update pwm if neccesary
    _pid_input = _heater_temp;
    _pid.Compute();
    if (_controller_mode == REMOTE_CONTROL || _controller_mode == LOCAL_CONTROL)
    {
        this->set_pwm(_pid_output);
    }
    else if (_controller_mode == MANUAL_TEST)
    {
        _pid_output = _pwm_float; // keep pid updated with current sitch.
    }
    else
    {
        this->set_pwm(0);
    }
}

/*-------------------- set_pwm -------------------------
    Sets PWM level for the heater. Must be a float
    between 0 and 100, function then converts to correct integer
    pwm level.
 ---------------------------------------------------------*/
void BallHeater::set_pwm(float pwm_goal)
{
    pwm_goal = constrain(pwm_goal, 0, HEATER_MAX_PERCENT);
    _pwm_float = pwm_goal;
    _pwm_level = (pwm_goal / 100) * _pwm_max;
    OCR1A = _pwm_level;
}

/*-------------------- set_control_mode -------------------------
    Sets the controller mode to ether local control mode, remote control mode, or manual test mode.
    In manual mode the heater pwm float is adjusted directly
    whereas in local or remote mode the PID controller controls it
    based on the target_temp.
        STANDBY 0
        LOCAL_CONTROL 1
        REMOTE_CONTROL 2
        MANUAL_TEST 3

        HIGH_TEMP_ERROR 6
 ---------------------------------------------------------*/
void BallHeater::set_control_mode(byte mode)
{
    _controller_mode = mode;

    if (_controller_mode == LOCAL_CONTROL || _controller_mode == REMOTE_CONTROL)
    {
        _pid.SetMode(AUTOMATIC);
    }
    else
        _pid.SetMode(MANUAL);
    if (_controller_mode == STANDBY || _controller_mode == HIGH_TEMP_ERROR)
    {
        this->set_pwm(0);
    }
}

/*-------------------- set_pid_params -------------------------
   Sets the PID controller params to those given.
 ---------------------------------------------------------*/
void BallHeater::set_pid_params(double kp, double ki, double kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _pid.SetTunings(kp, ki, kd);
}

/*-------------------- Getters -------------------------
       Several simple functions for returning relavent private
       variables. Sensor related functions check to make sure
       the measurements are recent first.
 ---------------------------------------------------------*/
float BallHeater::get_pwm_float()
{
    return _pwm_float;
}

float BallHeater::get_target_temp()
{
    return _target_temp;
}

float BallHeater::get_heater_temp()
{
    this->check_read_time();
    return _heater_temp;
}

float BallHeater::get_aux_therm_temp()
{
    this->check_read_time();
    return _aux_therm_temp;
}

double BallHeater::get_pid_kp()
{
    _kp = _pid.GetKp();
    return _kp;
}

double BallHeater::get_pid_ki()
{
    _ki = _pid.GetKi();
    return _ki;
}

double BallHeater::get_pid_kd()
{
    _kd = _pid.GetKd();
    return _kd;
}

byte BallHeater::get_control_mode()
{
    return _controller_mode;
}

/*-------------------- check_read_time  -------------------------
       Checks if the temps have been read recently and reads if not.
 ---------------------------------------------------------*/
void BallHeater::check_read_time()
{
    if (millis() - _last_temp_read > _stale_time)
        this->read_temps();
}

/*-------------------- read_temps -------------------------
      Reads the temperatures from the various sensors and updates their variables.
 ---------------------------------------------------------*/
void BallHeater::read_temps(void)
{
    // digitalWrite(13, HIGH);
    _heater_temp = read_thermistor(_main_therm);
    _aux_therm_temp = read_thermistor(_aux_therm);
#if ECHO_TO_SERIAL
    Serial.print("heater: ");
    Serial.print(_heater_temp);
    Serial.print(", aux: ");
    Serial.print(_aux_therm_temp);
#endif
    _last_temp_read = millis();
    // digitalWrite(13, LOW);
}

/*-------------------- read_thermistor -------------------------
      Reads a thermistor on specified pin several times and returns
      an average temperature in degrees C
 ---------------------------------------------------------*/
float BallHeater::read_thermistor(byte pin)
{
    uint8_t i;
    float average;
    int samples[NUMSAMPLES];

    // take N samples in a row,
    for (i = 0; i < NUMSAMPLES; i++)
    {
        samples[i] = analogRead(pin);
        //   delay(10);
    }

    // average all the samples out
    average = 0;
    for (i = 0; i < NUMSAMPLES; i++)
    {
        average += samples[i];
    }
    average /= NUMSAMPLES;

    // convert the value to resistance
    average = 1023 / average - 1;
    average = SERIESRESISTOR / average;
    // Serial.print("Thermistor resistance ");
    // Serial.println(average);

    float steinhart;
    steinhart = average / THERMISTORNOMINAL;          // (R/Ro)
    steinhart = log(steinhart);                       // ln(R/Ro)
    steinhart /= BCOEFFICIENT;                        // 1/B * ln(R/Ro)
    steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart;                      // Invert
    steinhart -= 273.15;                              // convert to C
#if ECHO_TO_SERIAL
    Serial.print("temp: ");
    Serial.println(steinhart);
#endif
    return steinhart;
}

void BallHeater::set_target_temp(float target_temp)
{
    _target_temp = target_temp;
}
