# code par Pierre Mirouze
# FabriqExpo Exploradôme de Vitry
# suivi de la balle avec OpenCV, extraction des coordonnées, envoi de l'ordre de mouvement au bras robotisé

# import des paquets nécessaires
from collections import deque
from imutils.video import VideoStream
import numpy as np
import cv2
import imutils
import time
import RPi.GPIO as GPIO
import serial

#config du switch de debug
GPIO.setmode(GPIO.BOARD)
buttonPin = 36
GPIO.setwarnings(False) # Ignorer warning
GPIO.setup(buttonPin, GPIO.IN)


# limites HSV de la couleur de la balle, à définir avec hsv_define.py
HSVLower = (15, 112, 143) #(H low, S low, V low)
HSVUpper = (45, 255, 255) #(H up, S up, V up)


pts = deque(maxlen=50) #longueur de la traînée rouge

# attribution flux webcam
vs = VideoStream(src=0).start()
# attente démarrage cam
time.sleep(1.0)

#déclaration de variables
xpast=0
ypast=0
counter=0

#fonction de décodage du port série
def decodestr(inputstr):   
        inputstr=inputstr.decode("utf-8")
        inputstr=inputstr.replace("\r","")
        inputstr=inputstr.replace("\n","")
        inputstr=inputstr.replace("'b","")
        return inputstr


# main
with serial.Serial("/dev/ttyACM0", 9600, timeout=1) as arduino:
	time.sleep(0.1) #wait for serial to open
	if arduino.isOpen():
		print("{} connected!".format(arduino.port))
		while arduino.inWaiting()==0: pass
		while True :
			if  arduino.inWaiting()>0: 
				answer=arduino.readline()
				answer=decodestr(answer)
				print(answer)
				arduino.flushInput()
				if answer== 'ready' :
					while True:
						# définir frame comme flux video
						frame = vs.read()
						# fin de la boucle si pas de flux
						if frame is None:
							break
						# réduction de la résolution, flou, et conversion HSV
						frame = frame[20:450, 60:600]
						#affichage des 4 angles
						calib = cv2.circle(frame,(84,38), 5,(0,0,255),-1)
						calib = cv2.circle(calib,(449,36),5,(0,0,255),-1)
						calib = cv2.circle(calib,(34,419),5,(0,0,255),-1)
						calib = cv2.circle(calib,(517,415),5,(0,0,255),-1) 
						#déclaration des points du trapèze à rectifier
						imgPts = np.float32([[84,38],[449,36],[34,419],[517,415]])
						objPoints = np.float32([[0,0],[1000,0],[0,1000],[1000,1000]])
						matrix = cv2.getPerspectiveTransform(imgPts,objPoints)
						frame = cv2.warpPerspective(frame,matrix,(1000,1000))
						blurred = cv2.GaussianBlur(frame, (11, 11), 0)
						hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
						# génération d'un masque sur la balle
						mask = cv2.inRange(hsv, greenLower, HSVUpper)
						mask = cv2.erode(mask, None, iterations=2)
						mask = cv2.dilate(mask, None, iterations=2)
						# trouver contours de la sortie du filtre et coordonnées de son centre
						cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
						cnts = imutils.grab_contours(cnts)
						center = None
						# si un contour est trouvé :
						if len(cnts) > 0:
							c = max(cnts, key=cv2.contourArea)
							((x, y), radius) = cv2.minEnclosingCircle(c)
							M = cv2.moments(c)
							center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
							# préciser le rayon minimal pour la détection (en pixels)
							if radius > 10:
								# dessiner le cercle et mettre à jour la liste des dernières coordonnées (pour le trail)
								cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
								cv2.circle(frame, center, 5, (0, 0, 255), -1)			
								#conversion en centimètres
								x=((x/1000)*64)+1
								y=((y/1000)*64)-0.4
								x=round(x,1)
								y=round(y,1)
								cv2.putText(frame,"x,y: "+str(x)+","+str(y),(20,20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),2)
								#si la balle arrête de se déplacer, démarrer la procédure de saisie
								if (xpast>=x-0.25 and xpast<=x+0.25 and ypast>=y-0.25 and ypast<=y+0.25):
									cv2.putText(frame,"LE ROBOT VA ATTRAPER LA BALLE",(20,60),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
									counter=counter+1
									#si la balle est immobile depuis 15 images, aller l'attraper
									if (counter>=15) :
										inputs="<"+str(x)+","+str(y)+">"
										inputs=inputs.encode("utf-8")
										arduino.write(inputs) #envoi des coordonnées à l'arduino
										print("inputs :", inputs)
										while arduino.inWaiting()==0: pass
										while True:
											if  arduino.inWaiting()>0: 
												answer=arduino.readline()
												answer=decodestr(answer)
												print(answer)
												arduino.flushInput()
												if answer=='done' : #si le bras a terminé le mouvement
													break
										counter=0
								xpast=x
								ypast=y
						# màj des données
						pts.appendleft(center)
						# fonction pour le trail rouge
						for i in range(1, len(pts)):
							# si pas de données, sauter
							if pts[i - 1] is None or pts[i] is None:
								continue
							# calculer l'épaisseur de la ligne en fonction du temps et dessiner
							thickness = int(np.sqrt(10 / float(i + 1)) * 2.5)
							cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)
							# afficher la sortie video traitée assemblée
						if GPIO.input(buttonPin) == GPIO.HIGH:
							#cv2.namedWindow("pré-calib", cv2.WND_PROP_FULLSCREEN)	
							#cv2.setWindowProperty("pré-calib",cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)
							cv2.imshow("pré-calib", calib)
						elif GPIO.input(buttonPin) == GPIO.LOW:
							cv2.namedWindow("Retour tracking", cv2.WND_PROP_FULLSCREEN)	
							cv2.setWindowProperty("Retour tracking",cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)
							cv2.imshow("Retour tracking", frame)
						key = cv2.waitKey(1) & 0xFF
						# arrêt si Q est pressé
						if key == ord("q"):
							break
					# arrêt de la camera
					vs.stop()
					# fermer les fenetres
					cv2.destroyAllWindows()
