#  Bras Robotisé attrapeur de balles

Ce projet est un bras robotisé classique à deux axes + une pince, destiné à attraper une balle grâce à une vision basée sur OpenCV.
Ce repository contient l'ensemble du code source ainsi que la CAO du shield Arduino.

<img src="https://user-images.githubusercontent.com/52047867/178235491-6a052895-79d3-4b22-a54e-cb9cacc90fcf.jpg" width="350">

## Principe et matériel

Ce projet est basé sur un **Raspberry Pi 4** (2GB RAM) muni de sa **Pi Caméra v2**, exécutant le code Python pour la détection de la balle, et transmettant par USB à l'**Arduino** les coordonnées de cette balle. L'Arduino est chargée de commander le **moteur pas-à-pas** de translation et les deux **servomoteurs** de commande du bras robotisé de manière à ce qu'il attrape la balle en question.

Un Shield Arduino conçu pour ce projet, dont la CAO est disponible dans le dossier [Shield PCB](https://github.com/erripe13/Ball-Picker-Arm-v1.0/tree/main/Shield%20PCB).  Il permet de connecter à l'Arduino, mais surtout d'alimenter, tous les périphériques du projet (hormis la caméra, l'interrupteur DEBUG, et le ruban led qui sont directement connectés au Raspberry Pi) à savoir :
 - Capteur fin de course de translation (calibration de l'axe X)
 - Capteur de température (régulation de la ventilation) (5V)
 - Ventilateurs (12V input, sortie variable par pwm)
 - Servomoteur fort couple n°1 (6V)
 - Servomoteur fort couple n°2 (6V)
 - Servomoteur de la pince (5V)
 - Driver du moteur pas-à-pas (12V)

Ce Shield comporte aussi deux circuits de coupure d'alimentation des moteurs par deux MOSFET (n et p, IRL510 et IRF4905) en montage high-side. Cela permet de "reposer" les servomoteurs quand ils ne servent pas pour éviter la surchauffe. Un MOSFET-N est également utilisé en montage low-side pour réguler l'alimentation des ventilateurs. Les détails et la schématique de cette carte sont disponibles dans le dossier du projet KiCad précédemment cité.

## Rôle des programmes - arborescence du dépôt

À la racine sont disponibles les programmes Python destinés à être exécutés sur le Raspberry Pi. 

 - *ball_tracking* est le programme principal, il remplit les fonctions suivantes :
	 - détection de la balle par filtrage HSV puis détection de contours
	 - compensation de la perspective dûe à l'orientation non-zénitale de la caméra
	 - extraction des coordonnées réelles de la balle
	 - envoi des coordonnées à l'Arduino par USB une fois la balle immobile
 - *hsv_define* permet de définir manuellement les seuils (bas et haut) de chaque paramètre du filtre HSV (cf. tuto plus bas dans ce document)
 - *NeoPixel* est exécuté au démarrage du système et allume en blanc le ruban LED connecté au GPIO18 du Raspberry

Dans le dossier arduino_runrobot on trouve le programme unique exécuté par l'Arduino

 - *arm_arduino.ino* permet d'effectuer les tâches suivantes :
	 - réception de décodage des coordonnées de la balle
	 - pilotage des moteurs pour se déplacer et aller attraper la balle

## Procédure de calibration

### Calibration du trapèze de détection - correction de la perspective

La calibration de la correction de perspective a été réduite au plus simple. Pour y accéder, il faut démarrer le système avec le switch DEBUG activé. On dispose ainsi d'un affichage de la vidéo avant traitement :
L'objectif est de régler manuellement la position de la caméra pour faire coincider les points rouges avec les quatre angles du plateau de jeu. 

<img src="https://user-images.githubusercontent.com/52047867/178235724-72e4d138-2fbe-4a8f-b678-352255694053.jpg" width="350">

Une fois cette tâche réalisée on peut basculer le bouton debug en position normale et observer l'image après son traitement, par conséquent rectifiée.

<img src="https://user-images.githubusercontent.com/52047867/178235735-25601584-9a38-4b6e-b2c6-fcfab519fc51.jpg" width="350">

Si on souhaite modifier les coordonnées de ces points (attention : ce n'est pas nécessaire dans la situation du modèle actuel du robot) il s'agit des lignes 70 à 75 du programme *ball_tracking* :

    #affichage des points rouges
    calib = cv2.circle(frame,(84,38), 5,(0,0,255),-1)
    calib = cv2.circle(calib,(449,36),5,(0,0,255),-1)
    calib = cv2.circle(calib,(34,419),5,(0,0,255),-1)
    calib = cv2.circle(calib,(517,415),5,(0,0,255),-1)
    #déclaration des points du trapèze à rectifier (identiques au points rouges, respecter l'ordre)
    imgPts = np.float32([[84,38],[449,36],[34,419],[517,415]])


### Calibration du filtre de couleur  - détection de la balle

Si la balle n'est pas correctement détectée, où qu'on souhaite utiliser une autre couleur de balle, il est nécessaire de trouver les 6 seuils de réglage du filtre.
Il faut exécuter le programme *hsv_define*. On y observe le flux caméra sans filtrage, puis une autre fenêtre avec 6 sliders et la sortie du filtre juste en-dessous.

Il faut d'abord cherche les 3 meilleurs paramètres LOW dans l'ordre HSV pour que la sortie du filtre soit la plus propre possible, puis ensuite diminuer les 3 paramètres HIGH pour nettoyer au mieux le reste de pollution. La sortie du filtre doit afficher en blanc la balle, quasiment sans aucune autre perturbation. Une fois ces 6 paramètres trouvés, il faut les renseigner dans le programme _ball_tracking_ au lignes 23 et 24.

<img src="https://user-images.githubusercontent.com/52047867/178241426-f36855b2-244e-4145-b5ba-fc0305012950.jpg" width="350"><img src="https://user-images.githubusercontent.com/52047867/178241441-9e0ddcb8-bc6c-4012-8825-5191c1054a69.jpg" width="350">

    HSVLower = (15, 112, 143) #(H low, S low, V low)
    HSVUpper = (45, 255, 255) #(H up, S up, V up)
