import sys

import cv2 as cv
import numpy as np
np.set_printoptions(suppress=True)

with np.load('params.npz') as X:
    mtx, dist = [X[i] for i in ('mtx', 'dist')]


def draw(img, corners: np.array, imgpts):
    corner = tuple(corners[0].astype(int).ravel())
    img = cv.line(img, corner, tuple(imgpts[0].astype(int).ravel()), (255, 0, 0), 5)
    img = cv.line(img, corner, tuple(imgpts[1].astype(int).ravel()), (0, 255, 0), 5)
    img = cv.line(img, corner, tuple(imgpts[2].astype(int).ravel()), (0, 0, 255), 5)
    return img


criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((9 * 15, 3), np.float32)
objp[:, :2] = np.mgrid[0:15, 0:9].T.reshape(-1, 2)
axis = np.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]]).reshape(-1, 3)

img = None
cap = cv.VideoCapture(int(sys.argv[1]))
k = 0
try:
    while(k != 27):
        hasFrame, img = cap.read()
        if not hasFrame:
            continue

        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        ret, corners = cv.findChessboardCorners(gray, (15, 9), None)
        if ret:
            corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            ret, rvecs, tvecs = cv.solvePnP(objp, corners2, mtx, dist)
            imgpts, jac = cv.projectPoints(axis, rvecs, tvecs, mtx, dist)
            img = draw(img, corners2, imgpts)

        cv.imshow('img', img)
        k = cv.waitKey(1)
finally:
    cv.destroyAllWindows()
