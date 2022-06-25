#https://docs.opencv.org/3.3.0/dc/dbb/tutorial_py_calibration.html
# code par Pierre Mirouze depuis un projet de pacogarcia3
# FabriqExpo Exploradôme de Vitry
# calibration de la déformation de la caméra (damier)

import numpy as np
import cv2
import glob
import time

workingdir="/home/pi/Desktop/Captures/"
savedir="camera_data/"

# critère d'arrêt
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# liste des points
objp = np.zeros((7*7,3), np.float32)

#2.5cm par carré du damier dans l'axe horizontal
objp[:,:2] = np.mgrid[0:7,0:7].T.reshape(-1,2)*2.5

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('calibration_images/*.jpg')

win_name="Verify"
cv2.namedWindow(win_name, cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty(win_name,cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)

print("getting images")
for fname in images:
    img = cv2.imread(fname)
    print(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


    # trouver le damier
    ret, corners = cv2.findChessboardCorners(gray, (7,7), None)
    # corréler les points du damier avec la topographie de l'image
    if ret == True: 
        objpoints.append(objp)
        corners2=cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)
        # dessiner les lignes du damier
        cv2.drawChessboardCorners(img, (7,7), corners2, ret)
        cv2.imshow(win_name, img)
        cv2.waitKey(500)

    img1=img
    
cv2.destroyAllWindows()

print(">==> Starting calibration")
ret, cam_mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

#print(ret)
print("Camera Matrix")
print(cam_mtx)
np.save(savedir+'cam_mtx.npy', cam_mtx)

print("Distortion Coeff")
print(dist)
np.save(savedir+'dist.npy', dist)

print("r vecs")
print(rvecs[2])

print("t Vecs")
print(tvecs[2])

print(">==> Calibration ended")


h,  w = img1.shape[:2]
print("Image Width, Height")
print(w, h)
newcam_mtx, roi=cv2.getOptimalNewCameraMatrix(cam_mtx, dist, (w,h), 1, (w,h))

print("Region of Interest")
print(roi)
np.save(savedir+'roi.npy', roi)

print("New Camera Matrix")
np.save(savedir+'newcam_mtx.npy', newcam_mtx)
print(np.load(savedir+'newcam_mtx.npy'))

inverse = np.linalg.inv(newcam_mtx)
print("Inverse New Camera Matrix")
print(inverse)


# rectifier l'image
undst = cv2.undistort(img1, cam_mtx, dist, None, newcam_mtx)

cv2.imshow('img1', img1)
cv2.waitKey(2000)
cv2.destroyAllWindows()
cv2.imshow('img1', undst)
cv2.waitKey(2000)
cv2.destroyAllWindows()
