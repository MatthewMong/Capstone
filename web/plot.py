import matplotlib.pyplot as plt 
import numpy as np 
import pandas as pd 
import os 
import glob 
import sys 
import getopt

def main(argv):
    path = os.getcwd() #default path is working directory 
    try:
        opts, args = getopt.getopt(argv, "h:p:", 
                        ["path"])
    except:
        print("Usage: python3 plot.py -p <PATH>")
        sys.exit(3)

    for opt, arg in opts:
        if opt == "-h":
            print("___.py -p <path>")
            sys.exit()
        if opt in ("-p", "--path"):
            path = arg
    return path
   
if __name__ == "__main__":
    path = main(sys.argv[1:]) 

    df = pd.DataFrame(pd.read_csv(path))
    df_edited = df
    df_edited.columns = ['raw_acc_x', 'raw_acc_y', 'raw_acc_z', 'raw_gyr_x', 'raw_gyr_y', 'raw_gyr_z','raw_pressure','time']
    df_edited['time'] = df_edited['time']/1000000
    df = df_edited
        
    fig, axs = plt.subplots(2, 1)
    axs[0].plot(df['time'],df['raw_acc_x'] ,df['time'], df['raw_acc_y'],df['time'],df['raw_acc_z'])
    axs[0].set_xlabel('Time(s)')
    axs[0].set_ylabel('Acceleration (m/s^2)')
    axs[0].grid(True)
    
    axs[1].plot(df['time'],df['raw_gyr_x'],df['time'], df['raw_gyr_y'],df['time'],df['raw_gyr_z'] )
    axs[1].set_xlabel('Time(s)')
    axs[1].set_ylabel('Gyroscope (degrees)')
    axs[1].grid(True)
    
    plt.show()