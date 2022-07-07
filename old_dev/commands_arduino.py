# code par Pierre Mirouze
# FabriqExpo Exploradôme de Vitry
# code d'envoi des coordonnées à l'arduino

import serial
import time



class arm_controller:

    def __init__(self, serialport='/dev/ttyACM0'):

        self.ser = serial.Serial(serialport, 9600)
    
        self.wait_forready()


    def decodestr(self, inputstr):   
        inputstr=inputstr.decode("utf-8")
        inputstr=inputstr.replace("\r","")
        inputstr=inputstr.replace("\n","")
        inputstr=inputstr.replace("'b","")
        return inputstr

    def wait_forready(self):
        print(self.decodestr(self.ser.readline()))
        while True:
            str1=self.decodestr(self.ser.readline())
            print(str1)
            if str1=="ready":
                break

    def move(self, inputarr):


        #format : <x,y,z,bool_move,bool_open,delayms,type_int> = <23,56,89,1,1,3456,3> {17}
        #X: 7.00 Y: 8.00 Z: 9.00 bool_move: 1.00 bool_open: 0.00 delay_ms: 10.00 move_type: 1.00

        #bool_move contrôle un mouvement linéaire
        
        inputs="<"+str(x)+","+str(y)">"
        inputs=inputs.encode("utf-8")

        self.ser.write(inputs)
        while True:
            str1=self.decodestr(self.ser.readline())
            print(str1),
            if str1=="done":
                break

