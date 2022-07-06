# code par Pierre Mirouze depuis un projet de pacogarcia3
# FabriqExpo Exploradôme de Vitry
# code d'envoi des coordonnées à l'arduino

import serial
import time



class arm_controller:

    def __init__(self, serialport='/dev/ttyACM1',LOW_Z=-20.5, HOVER_Z=-16.5,DROP_Z=-18.5, MID_Z=-15, HIGH_Z=-6):

        self.ser = serial.Serial(serialport, 9600)
    
        self.wait_forready()

        self.LOW_Z=LOW_Z  #coordonnée la plus basse atteignable
        self.HOVER_Z=HOVER_Z #Z pendant translation
        self.DROP_Z=DROP_Z #Z avant de lâcher
        self.MID_Z=MID_Z #Z intermédiaire (repos)
        self.HIGH_Z=HIGH_Z #Z le plus haut atteignable


        self.DELAY=0


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

    def move_untildone(self, inputarr):


        #format : <x,y,z,bool_move,bool_open,delayms,type_int> = <23,56,89,1,1,3456,3> {17}
        #X: 7.00 Y: 8.00 Z: 9.00 bool_move: 1.00 bool_open: 0.00 delay_ms: 10.00 move_type: 1.00

        #bool_move contrôle un mouvement linéaire
        
        inputs="<"+str(inputarr[0])+","+str(inputarr[1])+","+str(inputarr[2])+","+str(inputarr[3])+","+str(inputarr[4])+","+str(inputarr[5])+">"
        inputs=inputs.encode("utf-8")

        self.ser.write(inputs)
        while True:
            str1=self.decodestr(self.ser.readline())
            print(str1),
            if str1=="done":
                break


    def move_toXYZ(self, x, y, z):
        self.move_untildone([x,y,z,1,1,self.DELAY,1])
        
    def move_and_pickup(self, x, y):

        
        #montée du bras
        self.move_untildone([x,0,self.HIGH_Z,1,1,self.DELAY,1])
        #ouvrir
        self.move_untildone([x,0,self.HIGH_Z,1,1,self.DELAY,1])
        #mmouvement z
        self.move_untildone([x,0,self.HIGH_Z,1,1,self.DELAY,1])
        #descente du bras
        self.move_untildone([x,y,self.HOVER_Z,0,1,self.DELAY,1])
        self.move_untildone([x,y,self.LOW_Z,1,1,self.DELAY,1])
        #fermer
        self.move_untildone([x,y,self.LOW_Z,1,0,self.DELAY*20,1])
        #montée du bras
        self.move_untildone([x,y,self.HOVER_Z,1,0,self.DELAY,1])
        self.move_untildone([x,0,self.HIGH_Z,0,0,self.DELAY,1])


    def transport_and_drop(self, x, y):

        #monter
        self.move_untildone([x,y,self.MID_Z,1,0,self.DELAY,1])
        #baisser
        self.move_untildone([x,y,self.DROP_Z,0,0,self.DELAY,1])
        #lâcher
        self.move_untildone([x,y,self.DROP_Z,1,1,self.DELAY*10,1])
        #monter
        self.move_untildone([x,0,self.HIGH_Z,0,1,self.DELAY,1])



    def move_home(self):

        self.move_untildone([0,0,self.HIGH_Z,1,1,self.DELAY,1])



    def move_home_plane(self):

        #bouger au 0 pince fermée
        self.move_untildone([0,0,self.LOW_Z,1,0,self.DELAY,1])


    def move_x(self,x):

        self.move_untildone([x,0,self.HIGH_Z,1,0,self.DELAY,1])



    def move_clearcamera(self):

        self.move_untildone([40,0,self.HIGH_Z,0,1,self.DELAY,1])


    def test_arm(self):

        #home
        self.move_home()

        #libérer vue cam
        self.move_clearcamera()

        #home
        self.move_home()

        self.move_and_pickup(5, 5)

        self.transport_and_drop(15,0)

        #home
        self.move_home()

