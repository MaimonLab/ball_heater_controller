#include "Wire.h"
// #include "Adafruit_LiquidCrystal.h"
#include <Servo.h>
#include <Wire.h>
#include <Rotary.h>
// #include <hd44780.h> // main hd44780 header
// #include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header
#include "BallHeater.h"
#include "SerialComms.h"
#include <PID_v1.h>

#include <SerLCD.h> //Click here to get the library: http://librarymanager/All#SparkFun_SerLCD

// Pin Defines

// PIN	Assignment
// 0
// 1
// 2    rotary encoder B
// 3    rotary encoder A
// 4	rotary encoder switch (active low)
// 5
// 6
// 7    Expansion port J3
// 8    Expansion port J4
// 9    heater output
// 10
// 11
// 12
// 13
// A0
// A1
// A2   aux Thermistor
// A3   Thermistor main
// A4
// A5

#define ROTARY_1 3
#define ROTARY_2 2

#define TEMP_STEP 0.5F;

#define WIRECLOCK 400000L // tell hd44780 example to use this i2c clock rate

// hd44780_I2Cexp lcd; // declare lcd object: auto locate & auto config expander chip
SerLCD lcd; // Initialize the library with default I2C address 0x72

Rotary rotary = Rotary(ROTARY_1, ROTARY_2);

BallHeater ball_heater;
SerialComms serial_comms = SerialComms(&ball_heater);
int state = 0;
unsigned long timestamp = 0;

// Screen Buffer
char line0[17];
char line1[17];

void setup()
{
    // pinMode(9, OUTPUT);
    pinMode(2, INPUT_PULLUP);
    pinMode(3, INPUT_PULLUP);
    Serial.begin(115200);
    Wire.begin();
    // lcd.begin(16, 2);
    lcd.begin(Wire);

    lcd.setBacklight(255, 255, 255); // Set backlight to bright white
    lcd.setContrast(0);              // Set contrast. Lower to 0 for higher contrast.

    sprintf(line0, "Ball Heater");
    sprintf(line1, "V 0.1 by Jazz");
    update_display(1);
    delay(3000);

    //    Wire.setClock(WIRECLOCK);
    ball_heater.init();
    ball_heater.set_control_mode(LOCAL_CONTROL);
    //   Serial.begin(9600);
    //   delay(3000);

    // ball_heater.set_pwm(0.6);
}

/*-------------------- check_encoder -------------------------
      Checks if the rotary encoder has been turned and uptades the values
      accordingly.
 ---------------------------------------------------------*/
void check_encoder(void)
{

    unsigned char result = rotary.process();
    if (result == DIR_NONE)
        return;

    if (ball_heater.get_control_mode() == LOCAL_CONTROL)
    {
        float current_goal_temp = ball_heater.get_target_temp();

        if (result == DIR_CW)
        {
            current_goal_temp += TEMP_STEP;
        }
        else if (result == DIR_CCW)
        {
            current_goal_temp -= TEMP_STEP;
        }
        ball_heater.set_target_temp(current_goal_temp);
    }

    else if (ball_heater.get_control_mode() == MANUAL_TEST)
    {
        float current_pwm = ball_heater.get_pwm_float();
        float new_pwm;
        float max_pwm = 100;
        float min_pwm = 0;

        if (result == DIR_CW)
        {
            new_pwm = min(current_pwm + 2, max_pwm);
            ball_heater.set_pwm(new_pwm);
        }
        else if (result == DIR_CCW)
        {
            new_pwm = max(current_pwm - 2, min_pwm);
            ball_heater.set_pwm(new_pwm);
        }
    }
}

/*-------------------- mySetCursor -------------------------
      Moves cursor with given delay instead of the hardcoded
      50ms from the library.
 ---------------------------------------------------------*/
void mySetCursor(byte col, byte row, int delay_ms)
{
    int row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    // kepp variables in bounds
    // Explicitly cast numeric literals to type byte to avoid ESP32 and ESP8266 compile errors
    row = max((byte)0, row);              // row cannot be less than 0
    row = min(row, (byte)(MAX_ROWS - 1)); // row cannot be greater than max rows

    // send the commandset
    lcd.beginTransmission();
    lcd.transmit(SPECIAL_COMMAND); // Send special command character
    lcd.transmit(LCD_SETDDRAMADDR | (col + row_offsets[row]));
    lcd.endTransmission();
    delay(delay_ms);
} // setCursor

/*-------------------- update_display -------------------------
      Updates the display with current text buffers if the time is right.
 ---------------------------------------------------------*/
void update_display(int interval)
{
    // digitalWrite(3, HIGH);
    if (millis() % interval == 0)
    {
        lcd.setCursor(0, 0);
        // mySetCursor(0, 0, 5);
        lcd.print(line0);
        // lcd.clear();
        // lcd.print(strcat(line0, line1));
        lcd.setCursor(0, 1);
        // mySetCursor(0, 1, 5);
        lcd.print(line1);
    }
    // digitalWrite(3, LOW);
}

String get_control_mode_string(byte mode)
{
    switch (mode)
    {
    case LOCAL_CONTROL:
        return "Local";
    case REMOTE_CONTROL:
        return "Remote";
    case MANUAL_TEST:
        return "Manual";
    default:
        return "Error";
    }
}

unsigned long last_time = 0;
char heater_temp_string[5];
char target_temp_string[5];

void loop()
{
    check_encoder();
    ball_heater.tick();
    serial_comms.tick();
    String control_mode_string = get_control_mode_string(ball_heater.get_control_mode());
    float target_temp = ball_heater.get_target_temp();
    float heater_temp = ball_heater.get_heater_temp();
    dtostrf(heater_temp, 4, 1, heater_temp_string);
    dtostrf(target_temp, 4, 1, target_temp_string);

    sprintf(line0, "%s. %3d  pwm",
            control_mode_string.c_str(),
            round(ball_heater.get_pwm_float()));

    sprintf(line1, "g:%s cur:%s", target_temp_string, heater_temp_string);

    update_display(200);
    if (millis() - last_time > 1000)
    {
        // Serial.println(ball_heater.get_control_mode());
        // Serial.println(heater_temp);
        // Serial.println(round(heater_temp));
        // Serial.println(heater_temp_string);
        // Serial.println(line0);
        // Serial.println(line1);
        // last_time = millis();
    }
}
