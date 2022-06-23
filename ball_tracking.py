# code par Pierre Mirouze depuis la base de connaissance d'Adrian Rosebrock
# FabriqExpo Exploradôme de Vitry
# suivi de la balle OpenCV et extraction des coordonnées

# import des paquets nécessaires
from collections import deque
from imutils.video import VideoStream
import numpy as np
import cv2
import imutils
import time

# limites HSV couleur balle, à définir avec hsv_define.py
greenLower = (15, 112, 143)
greenUpper = (45, 255, 255)
pts = deque(maxlen=50)

# attribution flux webcam
vs = VideoStream(src=0).start()
# attente démarrage cam
time.sleep(2.0)

# main
while True:
	# définir frame comme flux video
	frame = vs.read()
	#frame = frame[1] if args.get("video", False) else frame
	# fin de la boucle si pas de flux
	if frame is None:
		break
	# réduction de la résolution, flou, et conversion HSV
	frame = imutils.resize(frame, width=600)
	blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
	# génération d'un masque sur la balle
	mask = cv2.inRange(hsv, greenLower, greenUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)
	# trouver contours de la sortie du filtre et coordonnées de son centre
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
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
			cv2.circle(frame, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
			cv2.circle(frame, center, 5, (0, 0, 255), -1)
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
	cv2.putText(frame,"x,y: "+str(x)+","+str(y),(20,20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF
	# arrêt si Q est pressé
	if key == ord("q"):
		break
# arrêt de la camera
vs.stop()

# fermer les fenetres
cv2.destroyAllWindows()
