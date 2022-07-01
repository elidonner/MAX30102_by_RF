'''
This simple file is the parent for creating our five bit flags for data we would like to store
'''

#ORDER OF THIS LIST IS IMPORTANT
data_types = ["time_stamp",
"red",
"ir",
"off_person",
"red_amplitude",
"ir_amplitude"
]

def main():

    #first give the encode_types a five bit flag, print it
    encode_types = {}
    i = 0
    for data in data_types:
        binary_string = bin(i).replace("0b", "")
        while len(binary_string) < 5:
            binary_string = "0" + binary_string
        encode_types[data] = binary_string
        i += 1

    print(encode_types)

    #now set it up in the format we want for our c++ encode.cpp file
    for key, value in encode_types.items():
        # value = hex(int(value,2))
        print (key, "_e = 0b", value,",",sep="")


if __name__ == '__main__':
    main()