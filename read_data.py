##############
# Script listens to serial port and writes contents into a file
##############
# requires pySerial to be installed
import serial
from datetime import datetime

# serial_port = '/dev/tty.usbmodem14101'
serial_port = 'COM9'
baud_rate = 115200  # In arduino, Serial.begin(baud_rate)
write_to_file_path = "YOUR_FILE_PATH.txt"
ser = serial.Serial(serial_port, baud_rate)


def sendToArduino(sendStr):
    sendStr = bytes(sendStr, 'utf-8')
    ser.write(sendStr)


def rec_serial():

    output_file = open(write_to_file_path, "w")
    output_file.write("Begin\n")
    while True:
        line = ser.readline()
        # ser.readline returns a binary, convert to string
        line = line.decode("utf-8")
        output_file.write(line)
        if line.strip() == "Done":
            break


def main():

    rec_serial()
    


if __name__ == '__main__':
    main()
