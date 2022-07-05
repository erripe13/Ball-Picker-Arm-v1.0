# Bras Robotisé FabriqExpo v1
- Le rôle de chaque fichier est exposé dans les trois premières lignes de commentaire.
- Environnement Hardware vérifié comme étant fonctionnel : Raspberry Pi 4B 2GB RAM, Arduino Uno -> 1 pas-à-pas et 4 servo
- Environnement Software vérifié comme étant fonctionnel : Python 3.9.12, Raspberry Pi OS 32-bit 11 "Bullseye" 05/2002
- Dépendences soft Python : opencv-python, imutils, numpy, time, collections, serial, argparse, glob, picamera, os
- Dépendences soft Arduino : Servo.h, math.h

1. La camera doit d'abord être calibrée (instructions à venir)
2. La perspective doit ensuite être calibrée (instructions à venir)
3. Les paramètres du filtre HSV de détection de la balle doivent enfin être définis grâce au script hsv_define.py
4. Le projet s'exécute en démarrant le script main.py qui gère toutes les sous-exécutions.
