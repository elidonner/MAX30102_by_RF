/********************************************************
 *
 * Project: MAXREFDES117#
 * Filename: RD117_ARDUINO.ino
 * Description: This module contains the Main application for the MAXREFDES117 example program.
 *
 * Revision History:
 *\n 1-18-2016 Rev 01.00 GL Initial release.
 *\n 12-22-2017 Rev 02.00 Significantlly modified by Robert Fraczkiewicz
 *\n 08-22-2018 Rev 02.01 Added conditional compilation of the code related to ADALOGGER SD card operations
 *
 * --------------------------------------------------------------------
 *
 * This code follows the following naming conventions:
 *
 * char              ch_pmod_value
 * char (array)      s_pmod_s_string[16]
 * float             f_pmod_value
 * int32_t           n_pmod_value
 * int32_t (array)   an_pmod_value[16]
 * int16_t           w_pmod_value
 * int16_t (array)   aw_pmod_value[16]
 * uint16_t          uw_pmod_value
 * uint16_t (array)  auw_pmod_value[16]
 * uint8_t           uch_pmod_value
 * uint8_t (array)   auch_pmod_buffer[16]
 * uint32_t          un_pmod_value
 * int32_t *         pn_pmod_value
 *
 * ------------------------------------------------------------------------- */

#include <Arduino.h>
#include <SPI.h>
#include "algorithm_by_RF.h"
#include "max30102.h"

// ADDED BY ELIDONNER
#include "PinNames.h"
#include "encode.h"

// #define DEBUG // Uncomment for debug output to the Serial stream
// #define CALIBRATE //uncomment to see calibration mode
#define DELAY_PROBLEM // uncomment to see issue with delay

#ifndef DEBUG
#define TO_TXT // Uncomment for printing to txt file, only possible without debug
#endif


// Interrupt pin
const byte oxiInt = D3; // pin connected to MAX30102 INT

uint32_t elapsedTime, timeStart;

uint32_t aun_ir_buffer[BUFFER_SIZE];  // infrared LED sensor data
uint32_t aun_red_buffer[BUFFER_SIZE]; // red LED sensor data
float old_n_spo2;                     // Previous SPO2 value
uint8_t uch_dummy, k;

float n_spo2, ratio, correl; // SPO2 value
int8_t ch_spo2_valid;        // indicator to show if the SPO2 calculation is valid
int32_t n_heart_rate;        // heart rate value
int8_t ch_hr_valid;          // indicator to show if the heart rate calculation is valid
int32_t i;
char hr_str[10];

// ELIDONNER ADDED FOR CALIBRATION:
// bounds for saturation limit (could be adjusted)
// meant to get largest swings in ADC readings without oversaturating
#define MAX_SAT 260000
#define MIN_SAT 200000
#define NO_PERSON 100000 // if IR is below this, nothing is present in front of sensor
// LED min and max brightness
#define MIN_BRIGHTNESS 10
#define MAX_BRIGHTNESS 255
// helpful definitions for determining which LED we are calibrating
#define IR 1
#define RED 0
// our target value for the LED raw values
const int setpoint = MAX_SAT - ((MAX_SAT - MIN_SAT) / 2);

// proportional gain, this value found empirically to give us reasonable control values
double Kp = 0.0001;

// flags for calibration
bool irCalibrated = false;
bool redCalibrated = false;
bool onPerson = false;

// led intensities
int irIntensity = 80;
int redIntensity = 80;

// DUMMY Buffer, normally there is a buffer, used to store 4096 bytes of data (one page of flash storage)
// Our data includes IMU readings, pulse ox readings, time stamps, etc
// HERE it is just a counter to determine when we use dummy delay
int dummy_buffer_cnt = 0;

void setup()
{

  pinMode(oxiInt, INPUT); // pin D10 connects to the interrupt output pin of the MAX30102

  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  maxim_max30102_init(); // initialize the MAX30102
  old_n_spo2 = 0.0;

  timeStart = millis();
}

// Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every ST seconds
void loop()
{

  // calibrate the leds
  #ifdef CALIBRATE
  if (!check_calibration())
  {
    calibrate_leds();
  }
  else
  {
    // buffer length of BUFFER_SIZE stores ST seconds of samples running at FS sps
    // read BUFFER_SIZE samples, and determine the signal range
    for (i = 0; i < BUFFER_SIZE; i++)
    {
      read_value(i);
      // if the values we read weren't calibrated, breakout so we can recalculate
      if (!check_calibration())
      {
        calibrate_leds();
        break;
      }
    }
  }
#else
    for (i = 0; i < BUFFER_SIZE; i++)
    {
      read_value(i);
    }
#endif


  // calculate heart rate and SpO2 after BUFFER_SIZE samples (ST seconds of samples) using Robert's method
  rf_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid, &ratio, &correl);
  elapsedTime = millis() - timeStart;
#ifdef TO_TXT
  Serial.println(encode_data((unsigned int) elapsedTime, time_stamp_e));
#endif
}

/**
 * @brief proportional feedback control for leds
 *
 * @param reading raw led value
 * @return int control value
 */
int P_Control(int reading)
{
  // error from the reading and target value
  double error = setpoint - reading;

  // proportional gain, this value found empirically to give us reasonable control values
  double Kp = 0.0001;

  int control_signal = Kp * error;
  return control_signal;
}

/**
 * @brief calibrate both LEDs
 *
 */
void calibrate_leds()
{
  int cnt = 0;
  long unsigned int redReading; // local variable to store the redLed reading
  long unsigned int irReading;  // local variable to store the irLed reading

// print that we have begun a calibration
#ifdef DEBUG
  Serial.print("calibration started");
#endif

  // Keep calibrating the LEDs while they remain uncalibrated
  while (cnt < 100)
  {
    // read values in
    read_value(cnt);
    irReading = aun_ir_buffer[cnt];
    redReading = aun_red_buffer[cnt];

    // if the reading is outside the saturation limits, reset the count
    // otherwise increment the count
    cnt = check_calibration() ? cnt + 1 : 0;

    // call calibration on each LED with this value
    calibrate(redReading, redCalibrated, redIntensity, RED);
    calibrate(irReading, irCalibrated, irIntensity, IR);
  }

  // calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  rf_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid, &ratio, &correl);
}

/**
 * @brief helper function for adjusting pulse amplitude of single led
 *
 * @param reading
 * @param calibrated
 */
void calibrate(int reading, bool &calibrated, int &intensity, int type)
{
  // first check if the this led is already calibrated
  // avoids us "overcalibrating" led if we are just calibrating one LED
  if (!calibrated)
  {
    // call proportional control function to get amount of control
    intensity = int(constrain(intensity + P_Control(reading), MIN_BRIGHTNESS, MAX_BRIGHTNESS));

    // set this new pulse amplitude
    setPulseAmplitude(intensity, type);
  }

#ifdef DEBUG
  if (type == IR)
  {
    Serial.print("IR: ");
  }
  else
  {
    Serial.print("RED: ");
  }
  Serial.print("Intensity = ");
  Serial.print(intensity, DEC);
  Serial.print(", value = ");
  Serial.println(reading, DEC);
#endif

#ifdef TO_TXT
  if (type == IR)
  {
    Serial.println(encode_data((unsigned int)intensity, ir_amplitude_e));
  }
  else
  {
    Serial.println(encode_data((unsigned int)intensity, red_amplitude_e));
  }
#endif
}

/**
 * @brief read a single fifo val
 *
 * @param index to read at
 * @return int value read
 */
void read_value(int index)
{
  long unsigned int red;
  long unsigned int ir;
  while (digitalRead(D3) == 1)
    ;                                                                          // wait until the interrupt pin asserts
  maxim_max30102_read_fifo((aun_red_buffer + index), (aun_ir_buffer + index)); // read from MAX30102 FIFO
  red = aun_red_buffer[index];
  ir = aun_ir_buffer[index];

#ifdef DEBUG
  Serial.print("Index: ");
  Serial.print(index, DEC);
  Serial.print(", Red: ");
  Serial.print(red, DEC);
  Serial.print(", IR: ");
  Serial.println(ir, DEC);
#endif
#ifdef TO_TXT
  Serial.println(encode_data((unsigned int) red, red_e));
  Serial.println(encode_data((unsigned int) ir, ir_e));
#endif

  // check first that the led is on the person
  //  Check if there is a person, uses the IR led since it isn't as affected by skin tone or ambient light
  onPerson = personDetected(ir);

  check_calibration(red, ir);

#ifdef DELAY_PROBLEM
  dummy_buffer_cnt++;
  if (dummy_buffer_cnt >= 256)
  {
    delay(125);

#ifdef DEBUG
    Serial.println("\n\"Writing\" data to storage\n");
#endif

    dummy_buffer_cnt = 0;
  }
#endif
}

/**
 * @brief Check if an LED is calibrated based on raw reading
 *
 * @param reading of photodiode
 * @param calibrated flag we are checking
 * @return true if calibrated
 * @return false if not calibrated
 */
bool check_calibration(int reading, bool &calibrated, int &intensity)
{
  // if a person is present
  if (onPerson)
  {
    // if we are within the bounds we are calibrated
    if (reading <= MAX_SAT && reading >= MIN_SAT)
    {
      calibrated = true;
    }
    /**
     * test edge cases
     */
    // if the intensity is min, but we are fully saturated, go ahead and say we are calibrated
    else if (intensity == MIN_BRIGHTNESS && reading > MAX_SAT)
    {
      calibrated = true;
    }
    // if the intensity is max, but we aren't saturated, go ahead and say we are calibrated
    else if (intensity == MAX_BRIGHTNESS && reading < MIN_SAT)
    {
      calibrated = true;
    }

    // OTHERWISE we are uncalibrated
    else
    {
      calibrated = false;
    }
  }
  // if a person isn't present, then the LED is not calibrated
  else
  {
    calibrated = false;
  }

  return calibrated;
}

/**
 * @brief Check if the calibration of both LEDS, overloads check_calibration for single LED
 *
 * @param reading of photodiode
 * @return true if calibrated
 * @return false if not calibrated
 */
bool check_calibration(int redReading, int irReading)
{
  // check this way so that it goes through checking calibration of both
  bool red = check_calibration(redReading, redCalibrated, redIntensity);
  bool ir = check_calibration(irReading, irCalibrated, irIntensity);

  return (red && ir);
}

/**
 * @brief Simple flag check
 *
 * @return true if calibrated
 * @return false if not calibrated
 */
bool check_calibration()
{
  return (redCalibrated && irCalibrated);
}

/**
 * @brief Set the Pulse Amplitude of given led type
 *
 * @param brightness to set led to
 * @param type of led
 */
void setPulseAmplitude(byte brightness, int type)
{
  // set the LED pulse amplitude by calling function in MAX3010x class

  // cast the brightness value to unsigned int to store it
  if (type == IR)
  {
    maxim_max30102_write_reg(REG_LED2_PA, brightness);
#ifdef DEBUG
    Serial.print("IR Amplitude: ");
    Serial.println(brightness);
#endif
  }
  else if (type == RED)
  {
    maxim_max30102_write_reg(REG_LED1_PA, brightness);
#ifdef DEBUG
    Serial.print("Red Amplitude: ");
    Serial.println(brightness);
#endif
  }
}

bool personDetected(int reading)
{
  // Check if there is a person, uses the IR led since it isn't as affected by skin tone or ambient light
  bool personPresent = false;

  // if we are below the NO_PERSON limit (found empirically), then nothing is detected in front of sensor
  if (irIntensity == MAX_BRIGHTNESS && reading < NO_PERSON)
  {
    personPresent = false;
#ifdef DEBUG
    Serial.println("No person detected");
#endif
  }
  else
  {
    personPresent = true;
  }

  return personPresent;
}