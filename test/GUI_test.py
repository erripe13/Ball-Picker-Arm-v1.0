from Tkinter import *
import tkMessageBox
import time
import serial

#+++++++++++++Variables+++++++++++++++++++++
ser = serial.Serial('/dev/ttyACM0', 9600)


#++++++++++++++++Functions+++++++++++++++++++++++


def move_it():
    #this function sends the command of joint angles to the arduino to move the servos to the desired positions

    command = str(base.get()) + ',' + str(shoulder.get()) +   ',' + str(elbow.get() ) + 'd'
    print command
    ser.write(command)
    #if not in the commandline can print verification to a notification window
    #tkMessageBox.showinfo( "Hello Python", "message sent")


#++++++++++++++++++++The GUI++++++++++++++++++++++
root = Tk()

#++++++++++++++++++++Drive Motors++++++++++++++++++

motorControl = Frame(root)
motorControl.pack()

forwardFrame = Frame(motorControl)
forwardFrame.pack()

backFrame = Frame(motorControl)
backFrame.pack (side = BOTTOM)

speedControl = Frame(root)
speedControl.pack()


#+++++++++++++++++ARM+++++++++++++++++++++++++
# The scroll bars
armControl = Frame(root)
armControl.pack( )

armLabel = Label(armControl, text = "Arm Componets", bg = "red", padx = 100)
armLabel.pack()

#++++++++++++++++++++++++BASE+++++++++++++++++++++++++++

baseLabel = Label(armControl, text = "rotation", bg = "green", padx = 100)
baseLabel.pack()
base = Scale(armControl, from_= 1, to = 175, length = 300, orient = HORIZONTAL)
base.pack()

#++++++++++++++++++++++++Shoulder+++++++++++++++++++++++++
shoulderLabel = Label(armControl, text = "servo1", bg = "green", padx = 100)
shoulderLabel.pack()
shoulder = Scale(armControl, from_= 10, to = 70, length = 300, orient = HORIZONTAL)
shoulder.pack()

#++++++++++++++++++++++ELBOW++++++++++++++++++++++++++++
elbowLabel = Label(armControl, text = "servo2", bg = "green", padx = 100)
elbowLabel.pack()
elbow = Scale(armControl, from_= 145, to = 180, length = 300, orient = HORIZONTAL)
elbow.pack()

#Send
armGoBut = Button(armControl, text ="Envoyer", height = 2, width = 5, command = move_it  )
armGoBut.pack()

#++++++++++++++++++++++++++++Gripper+++++++++++++++++++

gripperFrame = Frame(root)
gripperFrame.pack()


gripperLabel = Label(gripperFrame, text = "Gripper", bg = "green", padx = 20)
gripperLabel.pack()


gripOpenBut = Button(gripperFrame, text ="Open", height = 2, width = 5, command = open_gripper  )
gripOpenBut.pack(side = RIGHT)

gripCloseBut = Button(gripperFrame, text ="Close", height = 2, width = 5,command = close_gripper )
gripCloseBut.pack(side = RIGHT)

#+++++++++++++++++++++++++++Read Values+++++++++++++++++

root.mainloop(