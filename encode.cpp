/**
 * DOVE_algorithm.cpp
 * Header for led on pulse_oximeter class.
 *
 * Altrumed
 */
#include "encode.h"

unsigned int encode_data(unsigned int value, encode_type type)
{
    //Cast the encode type to a 32 bit int
    unsigned int casted_type = (unsigned int) type;
    //shift the 5 bit encode type to the start of the 32 bit int
    casted_type <<= (32-5);
    //return a bitwise or of value and the encode_type 32 bit int
    return (value | casted_type);
}

unsigned int encode_data(float value, encode_type type)
{
    //Overloaded function to accept float values

    //cast value to 32 bit int to avoid "ambiguity"
    unsigned int ans;
    //make sure the value is less than 27 bits
    if (value < 67108864)
    {
        //convert the value to an int by multiplying by 10,000
        value = value > 0 ? value*10000 : value*(-10000);
        if (value < 67108864)
        {
            //cast the float value to an int
            ans = static_cast<unsigned int>(value);

            //now call the overloaded function
            return encode_data(ans, type);
        }
    }
    ans = 999;
    return encode_data(ans, type);
}