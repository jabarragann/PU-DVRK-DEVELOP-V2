#!/usr/bin/env python

import cv2
import numpy as np

if __name__ == "__main__":

   # read image through command line
   img = cv2.imread("./mask1.jpg")

   # convert the image to grayscale
   gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

   # convert the grayscale image to binary image
   ret,thresh = cv2.threshold(gray_image,80,255,0)

   #Morphological operations to reduce noise
   kernel_size = 7
   kernel = np.ones((kernel_size, kernel_size),np.uint8)
   thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
   thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

   # find contours in the binary image
   contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
   print(len(contours))

   # cv2.imshow("grey", thresh)
   # cv2.waitKey(0)

   for c in contours:
      # calculate moments for each contour
      M = cv2.moments(c)

      if M["m00"] == 0:
         print("problems with one contour")
         continue

      # calculate x,y coordinate of center
      cX = int(M["m10"] / M["m00"])
      cY = int(M["m01"] / M["m00"])
      cv2.circle(img, (cX, cY), 5, (255, 255, 255), -1)
      cv2.putText(img, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

      print("pixel locations: ",cX,cY)

   # display the image
   cv2.imshow("Image", img)
   cv2.waitKey(0)

   cv2.destroyAllWindows()