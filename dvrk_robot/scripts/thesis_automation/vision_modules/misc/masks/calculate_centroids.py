#!/usr/bin/env python

import cv2
import numpy as np

if __name__ == "__main__":

   # read image through command line
   img = cv2.imread("./mask4.jpg")

   # convert the image to grayscale
   gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

   # convert the grayscale image to binary image
   ret,thresh = cv2.threshold(gray_image,80,255,0)

   #Morphological operations to reduce noise
   kernel_size = 7
   kernel = np.ones((kernel_size, kernel_size),np.uint8)
   thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=2)
   thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)

   # find contours in the binary image
   contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
   print(len(contours))

   # cv2.imshow("grey", thresh)
   # cv2.waitKey(0)
   
   cv2.drawContours(img, contours, -1, (0,255,0), 3)

   centroids_list = []
   for c in contours:
      # calculate moments for each contour
      M = cv2.moments(c)
      area = cv2.contourArea(c)
      
      if M["m00"] == 0:
         print("problems with one contour")
         continue

      # calculate x,y coordinate of center
      cX = int(M["m10"] / M["m00"])
      cY = int(M["m01"] / M["m00"])
      cv2.circle(img, (cX, cY), 5, (255, 255, 255), -1)
      cv2.putText(img, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

      centroids_list.append([area,(cX,cY)])
      print("pixel locations: ",cX,cY, "area",area )

   #Sorted list by area
   print("Sorted centroids by area")
   centroids_list.sort(reverse=True,key=lambda x:x[0])
   print(centroids_list)

   #Fake contour
   my_cont=[[200,300],[380,301],[400,400], [250,350],]
   my_cont = np.array(my_cont).reshape((-1,1,2)).astype(np.int32)
   pt_to_test =  (309, 340)
   # pt_to_test = (50,50)
   cv2.drawContours(img, [my_cont], -1, (0,0,255), 3)
   cv2.circle(img, pt_to_test, 5, (255, 255, 255), -1)
   dist1 = cv2.pointPolygonTest(my_cont, pt_to_test, True)
   print(dist1)

   #Valid area 
   valid_area =  [[43,340],[140,143],[214,54],[384,116],[515,194],[530,291],[456,457],[63,454]]
   valid_area = np.array(valid_area).reshape((-1,1,2)).astype(np.int32)
   cv2.drawContours(img, [valid_area], -1, (0,0,255), 3)

   # display the image
   thresh = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
   final = np.hstack((img, thresh))
   cv2.imshow("Image", final)
   cv2.waitKey(0)

   cv2.destroyAllWindows()