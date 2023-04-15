import serial

ser = serial.Serial('COM5', timeout=None, baudrate=9800) 
import csv
csvfile = open('test.csv', 'w', newline='')
spamwriter = csv.writer(csvfile)
try:
    while True:
        data_raw = ser.readline()
        formatted = (data_raw).decode('ascii').split(",")

        print(formatted[:-1])
        spamwriter.writerow(formatted[: -1])
except KeyboardInterrupt:
    pass
