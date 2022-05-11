# Bras Robotisé FabriqExpo v1
- Le rôle de chaque fichier est exposé dans les trois premières lignes de commentaire.
- Le projet a été développé pour Raspberry Pi 4B 2GB RAM, connecté en USB à une Arduino Uno chargée de la commande d'un moteur pas-à-pas et de 4 servo.
- La camera doit d'abord être calibrée avec un damier à l'aide du script initial_camera_calibration.py
- La perspective doit ensuite être calibrée à l'aide du script initial_perspective_calibration.py
- Les paramètres du filtre HSV de détection de la balle doivent enfin être définis grâce au script hsv_define.py
- Le projet s'exécute en démarrant le script main.py qui gère toutes les sous-exécutions.
