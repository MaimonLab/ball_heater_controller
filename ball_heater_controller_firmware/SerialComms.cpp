#include "SerialComms.h"

SerialComms::SerialComms(BallHeater *ball_heater)
{
    _ball_heater = ball_heater;
    _command_buffer_ptr = _command_buffer;
}

/*-------------------- tick -------------------------
      Reads available serial data into buffer and processes
      and executes if a full command has been recieved.
 ---------------------------------------------------------*/
void SerialComms::tick()
{
    // digitalWrite(8, HIGH);
    // read bytes available into buffer.
    while (Serial.available() > 0)
    {

        if (_command_write_pos == _command_read_pos - 1)
        {
            // Do not overwrite unread commands
            break;
        }

        // put sock messages in command buffer
        byte inByte = (byte)Serial.read();
        *(_command_buffer_ptr + _command_write_pos) = inByte;
        _command_write_pos++;

        if (_command_write_pos >= COMMAND_BUFFER_SIZE)
        {
            // Wrap around command buffer
            _command_write_pos = 0;
        }
    }
    this->get_serial_commands();
    // digitalWrite(8, LOW);
}

/*-------------------- get_serial_commands -------------------------
      Reads the received serial data off of _command_buffer and checks integrity.
      Fires command if full command has been received an passed integrity checks.
      Echos back original command in this case.
 ---------------------------------------------------------*/

void SerialComms::get_serial_commands()
{
    while (_command_write_pos != _command_read_pos && !_command_ready)
    {
        // check for command timeout.
        if (_command_verification != 0 &&
            ((millis() - _command_start_time) > COMMAND_READ_TIMEOUT))
        {
            _command_verification = 0;
            this->reset();
        }

        // Haven't read all command bytes and a full command hasn't been read
        bool shouldClear = false;
        switch (_command_verification)
        {
        // Pre payload verification
        case 0:
            if (254 == (uint8_t) * (_command_buffer_ptr + _command_read_pos))
            {
                _command_verification = 1;
                _command_start_time = millis();
                // Serial.println("case0");
            }
            else
            {
                shouldClear = true;
            }
            break;
        // Pre payload verification
        case 1:
            if (237 == (uint8_t) * (_command_buffer_ptr + _command_read_pos))
            {
                _command_verification = 2;
            }
            else
            {
                shouldClear = true;
            }
            break;

        // Get Command Code
        case 2:
            if (this->verify_command_code((uint8_t) * (_command_buffer_ptr + _command_read_pos)))
            {
                if (_num_arg_bytes > 0)
                    _command_verification = 3;
                else
                    _command_verification = 4;
            }
            else
            {
                shouldClear = true;
            }
            break;

        case 3:
            // Serial.println("case3");

            // Get all the arument inputs
            if (_placed_args_bytes < _num_arg_bytes)
            {
                _command_args[_placed_args_bytes] = (uint8_t) * (_command_buffer_ptr + _command_read_pos);
                _placed_args_bytes++;
            }
            if (_placed_args_bytes == _num_arg_bytes)
            {
                _command_verification = 4;
            }
            break;
        // Post payload verification
        case 4:
            if (190 == (uint8_t) * (_command_buffer_ptr + _command_read_pos))
            {
                _command_verification = 5;
            }
            else
            {
                shouldClear = true;
            }
            break;
        // Post payload verification
        case 5:
            if (173 == (uint8_t) * (_command_buffer_ptr + _command_read_pos))
            {
                _command_ready = true;
                _command_verification = 0;
            }
            else
            {
                shouldClear = true;
            }
            break;

        default:
            shouldClear = true;
            break;
        }
        // Advance Read position and wrap if needed.
        _command_read_pos++;
        if (_command_read_pos >= COMMAND_BUFFER_SIZE)
            _command_read_pos = 0;

        // If error, clear socket command object
        if (shouldClear)
        {
            _command_verification = 0;
            this->reset();
        }
    }
    if (_command_ready)
    {
        // Serial.println("firing command");
        int retval = (this->*_commmand_func)(_command_args);
        // TODO: check retval and do something.

        this->echo_command();
        this->reset();
    }
}

/*-------------------- verify_command_code -------------------------
      Verifies that the command code is a valid number and sets
      the relavent parameters.
 ---------------------------------------------------------*/
bool SerialComms::verify_command_code(uint8_t code)
{
    if (code < NUMCOMMANDS)
    {
        _current_command_code = code;
        _commmand_func = _func_map[code];
        _num_arg_bytes = _num_arg_bytes_map[code];
        return true;
    }
    else
        return false;
}

/*-------------------- reset -------------------------
       Resets the command verification system in case of error.
 ---------------------------------------------------------*/
void SerialComms::reset()
{
    _num_arg_bytes = 0;
    _placed_args_bytes = 0;
    _command_ready = false;

    for (int i = 0; i < COMMAND_ARG_BUF_SIZE; i++)
    {
        _command_args[i] = NULL;
    }
}

/*-------------------- echo_command -------------------------
      Echos the received command and argbytes over serial for verification.
 ---------------------------------------------------------*/
void SerialComms::echo_command()
{
    Serial.write(0xFE);
    Serial.write(0xED);
    Serial.write(_current_command_code);
    Serial.write(_command_args, _placed_args_bytes);
    Serial.write(0xBE);
    Serial.write(0xAD);
}

/*-------------------- send_status -------------------------
       Return the current temperatures and setpoints via serial.
 ---------------------------------------------------------*/
int SerialComms::send_status(byte *in_byte)
{
    write_float(_ball_heater->get_target_temp());
    write_float(_ball_heater->get_pwm_float());
    write_float(_ball_heater->get_heater_temp());
    write_float(_ball_heater->get_aux_therm_temp());
    Serial.write(_ball_heater->get_control_mode());
    return 0;
}

/*-------------------- write_float -------------------------
      converts a float to binary and sends it on serial.
 ---------------------------------------------------------*/
void SerialComms::write_float(float to_send)
{
    byte *binver = (byte *)&to_send;
    Serial.write(binver, 4);
}

/*-------------------- send_status_header -------------------------
      Returns a comma separated string of the status variables over serial,
      and a list of variable types.
 ---------------------------------------------------------*/
int SerialComms::send_status_header(byte *in_byte)
{
    Serial.println("target_temp,pwm_percent,"
                   "heater_temp,aux_therm_temp,control_mode");
    Serial.println("float,float,float,float,uint_8");
    return 0;
}

/*-------------------- send_pid_params -------------------------
       Sends the PID params (cast to float) back over serial.
 ---------------------------------------------------------*/
int SerialComms::send_pid_params(byte *in_byte)
{
    write_float((float)_ball_heater->get_pid_kp());
    write_float((float)_ball_heater->get_pid_ki());
    write_float((float)_ball_heater->get_pid_kd());
    return 0;
}

/*-------------------- set_pid_params -------------------------
     Sets the three parameters of the PID controller after decoding from bytes.
 ---------------------------------------------------------*/
int SerialComms::set_pid_params(byte *in_byte)
{
    float kp = ByteToType::BytesToFloat(in_byte);
    float ki = ByteToType::BytesToFloat(in_byte + 4);
    float kd = ByteToType::BytesToFloat(in_byte + 8);
    _ball_heater->set_pid_params(kp, ki, kd);
    return 0;
}

/*-------------------- set_target_temp -------------------------
      Set the target temp on the BallHeater to the given value.
 ---------------------------------------------------------*/
int SerialComms::set_target_temp(byte *in_byte)
{
    float target_temp = ByteToType::BytesToFloat((in_byte));
    _ball_heater->set_target_temp(target_temp);
    if (_ball_heater->get_control_mode() == STANDBY || _ball_heater->get_control_mode() == LOCAL_CONTROL)
        _ball_heater->set_control_mode(REMOTE_CONTROL);
    return 0;
}

/*-------------------- set_heater_pwm_manual -------------------------
      If in manual mode, set the ball_heater pwm float directly. Takes float 0-100
 ---------------------------------------------------------*/
int SerialComms::set_heater_pwm_manual(byte *in_byte)
{
    if (_ball_heater->get_control_mode() != MANUAL_TEST)
        return 1;
    float pwm_float = ByteToType::BytesToFloat(in_byte);
    _ball_heater->set_pwm(pwm_float);
    return 0;
}

/*-------------------- set_controller_mode -------------------------
     Sets the controller mode to ether manual or automatic, cool or heat.
     In manual mode the peliter pwm float is adjusted directly
     whereas in automatic mode the PID controller controlls it
      based on the target_temp.
    STANDBY 0
    LOCAL_CONTROL 1
    REMOTE_CONTROL 2
    MANUAL_TEST 3
 ---------------------------------------------------------*/
int SerialComms::set_controller_mode(byte *in_byte)
{
    byte mode = in_byte[0];
    _ball_heater->set_control_mode(mode);
    return 0;
}
