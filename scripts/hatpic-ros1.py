#!/usr/bin/env python
"""
File: hatpic-ros1.py
Authors: Julien Mellet, and Simon Le berre
Date: 2024-05-26
Description: A Python script to use the hatpic device to send some commands through ROS1.
"""

__author__ = "Julien Mellet, and Simon Le Berre"
__copyright__ = "Copyright 2024, Hatpic"
__credits__ = ["Julien Mellet", "Simon Le Berre"]
__license__ = "GPL"
__maintainer__ = "Julien Mellet"
__email__ = "julien.mellet@unina.it"
__status__ = "Production"

import serial
import time

hatpic = serial.Serial(port='/dev/ttyUSB0',  baudrate=115200, timeout=1)

data_a = 0
data_b = 0
data_c = 0
data_d = 0

def write(x):
    hatpic.write(bytes(x,'utf-8'))
    time.sleep(0.05)

def read():
    return classification(get_data())

def get_data():
    data_list = [b'a', b'b', b'c', b'd' ,b'0', b'-', b'1',b'2', b'3', b'4', b'5', b'6', b'7', b'8', b'9']
    #try:
    data = hatpic.read(1)
    start_timing = time.time()
    while data == None:
        data = hatpic.read(1)
        #condition = True   # add tempo mesurement after 5 secondes on sort si pas de data
        time.sleep(0.05)
        t = time.time()
        if t-start_timing > 2:
            print('no data received: :(')
            break
    
    if data:
        if (isinstance(data, bytes)):
            if (data.decode('utf-8')) == "i":
                trame =  "i"
                while (data.decode('utf-8')) != "o":
                    data = hatpic.read(1)
                    if (isinstance(data, bytes)):
                        if data in data_list:
                            trame = trame + data.decode('utf-8')
                        else:
                            pass
                    else:
                        pass
                
                trame = trame +"o"
                return trame
    #except:
    #    print("Error 1: get_data")
    #    return None
    
def extraction_value(chaine, debut, fin):
    #print(chaine)
    debut_index = chaine.find(debut)
    fin_index   = chaine.find(fin)
    if debut_index != -1 and fin_index != -1:
        return chaine[debut_index + len(debut):fin_index]
    else:
        return -1

def classification(trame):
    #print(trame)
    #global data_a, data_b, data_c, data_d
    if trame != None:
        if trame[0:2] == 'ia':
            data_a = int(extraction_value(trame, 'a', 'b'))
            data_b = int(extraction_value(trame, 'b', 'c'))
            data_c = int(extraction_value(trame, 'c', 'd'))
            data_d = int(extraction_value(trame, 'd', 'o'))
            data_hatpic = [data_a, data_b, data_c, data_d]
            return data_hatpic

while True:
    value  = write('ia100b2000c0d23o')
    #value  = write('10')
    #time.sleep(0.)
    data2 = read()
    if data2 != None:
        print(data2)
