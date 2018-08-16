# -*- coding: utf-8 -*-
"""
Script for live collection and display of Doppler returns for the 
MIT "Build a Small Radar" course (summer 2018 edition).  Be sure to scroll down 
to the bottom of the script and set:
(a) your serial port before execution
(b) your sample frequency corresponding to the code on the Arduino 
Use Ctrl-C to stop collecting data (you may have to close the display first).

    Copyright (C) 2018  Daniel A. LeMaster

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

"""

import serial
import time
import numpy as np
import matplotlib.pyplot as plt

c = 299e6 # speed of light (m/s)
fc = 2440e6 # radar frequency (Hz)
    
def livedoppler(serialport,cpi=0.5,loopduration=60,maxspeed=30,Fs = 2000):
    """Live collection, calculation, and waterfall display of Doppler returns. 
    
    Parameters
    ----------
    serialport :
        name of the serial port connected to the radar (string)
    loopduration : 
        amount of time to buffer Doppler returns (s). Old returns are dropped after
        the collection time exceeds the loopduration. Default value is 60 s.
    maxspeed : 
        maximum speed (m/s) to plot on the waterfall and retain for later use.
        Default value is 30 m/s.
    Fs :
        Arduino sampling rate (Hz).  This will differ depending on which version
        of the Arduino software that you use.  Set this to whatever value was used
        in the Matlab doppler processor.  Default is 2000 Hz.
        
    Returns
    -------
    doppler :
        array of doppler values collected over the most recent loop duration.
        Rows are indexed by time. Columns are index by speed. (dB)
    
    Notes:
    -------
    Collection is halted by pressing Ctrl-C.  You may also have to close the
    display window before pressing Ctrl-C.  The Doppler data will then be 
    returned as an array.    
    """      
    
    #set up the serial port
    ser = serial.Serial(serialport, baudrate=500000,bytesize=8,parity='N',stopbits=1,timeout=0)       
    time.sleep(.1)
    

    #setup the waterfall plot
    fig, ax = plt.subplots()
    plt.ylabel('time (s)')
    plt.xlabel('speed (m/s)')
    
    #preliminaty calculations
    windowsize = int(cpi*Fs) #number of samples in one processing window
    han = np.hanning(windowsize) #Han window to suppress Fourier transform sidelobes
    speedaxis = (np.fft.fftfreq(windowsize, d=1.0/Fs)*c/fc)[0:int(windowsize/2)] #scale frequency to speed in m/s
    doppler = np.zeros((int(loopduration/cpi),int(windowsize/2))) #initialize array to store doppler values
    doppler = doppler[:,speedaxis <= maxspeed] #crop based on maxspeed     
    speedaxis = speedaxis[speedaxis <= maxspeed] #crop based on maxspeed        
    ext = [0.0, maxspeed, -1.0*loopduration, 0.0] #set boundaries for the waterfall 
        
    rawbin = list() #list to hold raw data from the Arduino 
    print('Press Ctrl-C to stop collecting data.')
    ser.reset_input_buffer()   
    try:
        while True:
            cbyte = ser.read()
            if cbyte: #skip ahead if string is empty
                rawbin.append(cbyte) #append raw Arduino data to list
            if len(rawbin) >= 2*windowsize: #process Doppler data once a full "window" is collected
                    ser.reset_input_buffer()
                    #convert raw Arduino data into 12-bit integers, scale to 3.3V range
                    raw8bit = [ord(idx) for idx in rawbin]
                    raw12bit = 256*np.asarray(raw8bit[0::2])+np.asarray(raw8bit[1::2])    
                    data = raw12bit*3.3/(2**12)-3.3/2
                    #transform the windowed data into doppler returns
                    doppler = np.roll(doppler,1,axis=0)
                    doppler[0,:] = 20*np.log10(np.abs(np.fft.fft(han*data))[0:speedaxis.shape[0]])
                    #plot the result
                    ax.imshow(doppler, extent = ext,cmap='jet')
                    plt.pause(0.05)
                    plt.show()
                    rawbin = list()
                     
                    
    except KeyboardInterrupt: #stops collection when Ctrl-C is pressed
        pass
    
    print('Collection complete.')
    ser.close() #close the serial port   
    
    return doppler     
    

serialport = '/dev/ttyACM0'  #in Windows, this will be 'COM1', etc.    
doppler = livedoppler(serialport,cpi=0.5,loopduration=60,maxspeed=30,Fs = 2000)