import serial
import datetime
ser = serial.Serial('COM3', timeout=None, baudrate=9800)

import csv
import time
csvfile = open('test1.csv', 'w', newline='')
spamwriter = csv.writer(csvfile)
try:
    while True:
        data_raw = ser.readline()
        formatted = (data_raw).decode('ascii').split(",")
        print(formatted)
        spamwriter.writerow([*formatted[:-1], datetime.datetime.now().isoformat()])
except KeyboardInterrupt:
    pass