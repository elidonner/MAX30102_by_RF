#ORDER OF THIS LIST IS IMPORTANT
data_types = ["time_stamp",
"red",
"ir",
"off_person",
"red_amplitude",
"ir_amplitude"
]

data_size = 32 #bits
flag_size = 5 #bits
bit_shift = data_size - flag_size
five_bit_mask = (0b11111)<<(bit_shift)
# 0000 0111 1111 1111 1111 1111 1111 1111
data_mask = 0b00000111111111111111111111111111


def encode_data(value, type):
    '''
    Encode the given data
    '''
    #get the index of the type, then bitwise shift to start of 32 bit int
    flag = (data_types.index(type)) << (bit_shift)
    #return an or of this flag and the data value
    return (value | flag)


def decode_data(value):
    try:
        #get the five bit flag by bitwise & with five bit mask
        #then a bitwise shift
        flag = (value & five_bit_mask) >> (bit_shift)
        decode_type = data_types[flag]
        
        #get the value with bitwise & with data_mask
        value = value & data_mask

        #convert accelerometer values back to floats
        if decode_type in ["x", "y", "z", "roll", "pitch", "yaw","spo2"]:
            value = value / 10000
        if decode_type in ["nx", "ny", "nz", "nroll", "npitch", "nyaw"]:
            value = value / -10000

        #convert time to seconds
        if decode_type == "time_stamp":
            value = value / 1000
    except:
        value = 0
        flag = 11111
        decode_type = "unkown"

    return [value, decode_type, flag]