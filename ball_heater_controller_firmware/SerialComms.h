#ifndef SerialComms_H
#define SerialComms_H

#include "Arduino.h"
#include "BallHeater.h"
#include "ByteToType.h"

#define COMMAND_BUFFER_SIZE 112
#define COMMAND_ARG_BUF_SIZE 20
#define COMMAND_READ_TIMEOUT 1000

#define NUMCOMMANDS 7

class SerialComms
{
public:
    SerialComms(BallHeater *ball_heater);
    void tick();

private:
    bool verify_command_code(uint8_t code);
    void get_serial_commands();
    void reset();

    // Command functions
    void echo_command();
    int send_status(byte *in_byte);
    void write_float(float to_send);
    int send_status_header(byte *in_byte);
    int send_pid_params(byte *in_byte);
    int set_pid_params(byte *in_byte);
    int set_target_temp(byte *in_byte);
    int set_heater_pwm_manual(byte *in_byte);
    int set_controller_mode(byte *in_byte);

    BallHeater *_ball_heater;

    // Command reading
    byte _command_buffer[COMMAND_BUFFER_SIZE];
    uint8_t _command_read_pos = 0;
    uint8_t _command_write_pos = 0;
    byte *_command_buffer_ptr;
    uint8_t _command_verification = 0;
    uint32_t _command_start_time = 0;

    // command processing / execution
    byte _command_args[COMMAND_ARG_BUF_SIZE];
    uint8_t _current_command_code;
    // Number of bytes in the argument that is associated with a command
    uint8_t _num_arg_bytes;
    // Number of recieved bytes for a current command
    uint8_t _placed_args_bytes = 0;
    bool _command_ready = false;

    // function to get fired when executed
    int (SerialComms::*_commmand_func)(byte *);

    int (SerialComms::*_func_map[NUMCOMMANDS])(byte *input) = {
        &SerialComms::send_status,
        &SerialComms::send_status_header,
        &SerialComms::set_target_temp,
        &SerialComms::send_pid_params,
        &SerialComms::set_pid_params,
        &SerialComms::set_controller_mode,
        &SerialComms::set_heater_pwm_manual};

    int _num_arg_bytes_map[NUMCOMMANDS] = {
        0,  // send_status -- 0
        0,  // send_status_header -- 1
        4,  // set_target_temp -- 2
        0,  // send_pid_params -- 3
        12, // set_pid_params -- 4
        1,  // set_controller_mode -- 5
        4   // set_heater_pwm_manual -- 6
    };
};

#endif
