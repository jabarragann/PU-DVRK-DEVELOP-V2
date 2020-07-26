#!/usr/bin/env python



import cv2
import numpy as np 
import sys

if __name__ == '__main__':

	print(cv2.__version__)	


	cap = cv2.VideoCapture(int(sys.argv[1]))
	cv2.namedWindow('frame',cv2.WINDOW_NORMAL)

	while True:
		ret, img = cap.read()

		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

		ret, corners = cv2.findChessboardCorners(gray,(9,6), None)

		if ret == True:
			img = cv2.drawChessboardCorners(img,(9,6),corners, ret)


		cv2.imshow('frame', img)

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	cap.release()
	cv2.destroyAllWindows()