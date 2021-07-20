import serial

import pandas as pd
import Wav
print("füz")
#Serial port initialization
ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=1000000,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=None)
ser.flushInput()

print("connected to: " + ser.portstr)

#Send start command
charToSend = 's' #start command
ser.write(charToSend.encode())
print("Connection established!!")

#Read from serial
newfile = open('txt_file.txt', "w")
while (1):
    try:
        for c in ser.read(122):  #Read 122 bytes
            print(c)
            newfile.write("%s\n" % str(c))
            ser.flush()

    # IMPORTANT: Stop acquisition pressing Ctrl+C if you want to store data in a CSV file
    except KeyboardInterrupt:  #It stores received data in the new file NewData. It overwrites!!!
        break
ser.close()

df = pd.read_csv('txt_file.txt', sep= " ")
df.to_csv('/home/maschine/data/csv_file.csv', index= False)
print(df.shape)
print('DONE')
Wav.make_wav()
print("see wav file in data folder")
