/**
 * encode.h
 * Header for encoding pulse oximeter data.
 *
 * Saves space when writing to flash memory by writing data with a 5 bit flag
 * to 32 bit ints.
 * This is possible since all data is less than 27 bits
 * Altrumed
 */
#ifndef ENCODE_H
#define ENCODE_H

#include "Arduino.h"

/**
 * @brief The different encoding flags
 * the values are 5 bit flags
 * they get bit shifted to the beginning of a 32 bit int
 * i.e. red_e = "00001" = "00001 000 00000000 00000000 00000000" = 134217728
 * ...  ir_e  = "00010" = "00010 000 00000000 00000000 00000000" = 268435456
 * 
 */
typedef enum{
    time_stamp_e = 0b00000,
    red_e = 0b00001,
    ir_e = 0b00010,
    off_person_e = 0b00011,
    red_amplitude_e = 0b00100,
    ir_amplitude_e = 0b00101
} encode_type;

/**
 * @brief Encode the given data with the encode_type
 * Performs a bitwise or to combine the 32 bit ints into one int
 * 
 * @param value of data
 * @param type of data
 * @return unsigned int encoded data
 */
unsigned int encode_data(unsigned int value, encode_type type);

/**
 * @brief Function is overloaded to handle floats
 * Multiplies float values by 10,000, then converts them to ints
 * 
 * @param value 
 * @param type 
 * @return unsigned int 
 */
unsigned int encode_data(float value, encode_type type);
#endif //ENCODE_H