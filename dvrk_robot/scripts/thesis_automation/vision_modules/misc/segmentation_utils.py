import sys
sys.path.append("/home/isat/juanantonio/repos/MasterThesisComputerVision/")

import json
import numpy as np
import scipy.misc
import torch
import os
import cv2
from PIL import Image
# from src.fcn import VGGNet, FCNs
from fcn import VGGNet, FCNs
from torch import nn
import pickle 

def parse_label():
    f = json.load(open(label_colors_file, "r"))

    for idx, cl in enumerate(f):
        label = cl["name"]
        color = cl["color"]
        print(label, color)
        label2color[label] = color
        color2label[tuple(color)] = label
        label2index[label] = idx
        index2label[idx] = label

def labels_colors_dicts(path):
	label_colors_file = path
	label2color = {}
	color2label = {}
	label2index = {}
	index2label = {}

	f = json.load(open(label_colors_file, "r"))

	for idx, cl in enumerate(f):
		label = cl["name"]
		color = cl["color"]
		print(label, color)
		label2color[label] = color
		color2label[tuple(color)] = label
		label2index[label] = idx
		index2label[idx] = label

	final_dict = {'label2color':label2color,'color2label':color2label,
				'label2index':label2index,'index2label':index2label}

	return final_dict

def calculate_centroids(img):
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

   final_results = []
   for c in contours:
      # calculate moments for each contour
      M = cv2.moments(c)

      if M["m00"] == 0:
         print("problems with one contour")
         continue

      # calculate x,y coordinate of center
      cX = int(M["m10"] / M["m00"])
      cY = int(M["m01"] / M["m00"])
      
      final_results.append([cX, cY])
      # cv2.circle(img, (cX, cY), 5, (255, 255, 255), -1)
      # cv2.putText(img, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
      # print("pixel locations: ",cX,cY)

   # display the image
   # cv2.imshow("Image", img)
   # cv2.waitKey(0)
   # cv2.destroyAllWindows()

   return np.array(final_results)

def test_img(img,model, labels_dict, n_class=5):
	index2label = labels_dict['index2label']
	label2color = labels_dict['label2color']

	

	h,w,_= img.shape
	assert w == 640 and h == 480, 'Error with input shape'

	# h, w, c = img.shape[0], img.shape[1], img.shape[2]
	val_h = h
	val_w = w
	#resize image
	# img = scipy.misc.imresize(img, (val_h, val_w), interp='bilinear', mode=None)
	# img = cv2.resize(img, (val_h, val_w), interpolation=cv2.INTER_AREA)
	# img = img.resize((val_w,val_h)) #PILLOW images resize methods requires a tuple with the (width,height). Different from cv2.resize


	# img = np.array(img) ##Convert PIL imaged to numpy a array
	# img = img[:, :, ::-1]  ## # switch to BGR
	orig_img = np.copy(img)

	#Debug
	# cv2.imshow('test',img)
	# cv2.waitKey(0)

	img = np.transpose(img, (2, 0, 1)) / 255.
	means = np.array([103.939, 116.779, 123.68]) / 255.

	img[0] -= means[0]
	img[1] -= means[1]
	img[2] -= means[2]

	inputs = torch.from_numpy(img.copy()).float()
	inputs = torch.unsqueeze(inputs, 0).cpu() #changed from cuda to cpu
	output = model(inputs)
	output = output.data.cpu().numpy()

	N, _, h, w = output.shape
	assert (N == 1)
	pred = output.transpose(0, 2, 3, 1).reshape(-1, n_class).argmax(axis=1).reshape(h, w)


	pred_img = np.zeros((val_h, val_w, 3), dtype=np.float32)
	# for cls in range(n_class):
	for cls in range(0,1):
		pred_inds = pred == cls
		label = index2label[cls]
		color = label2color[label]
		pred_img[pred_inds] = color


	pred_img = pred_img.astype(np.uint8)
	pred_img = cv2.cvtColor(pred_img, cv2.COLOR_BGR2RGB)  #Change to BGR color space

	# finalFrame = np.hstack((orig_img, pred_img))
	finalFrame = 0.6*orig_img + 0.4*pred_img
	finalFrame = finalFrame.astype(np.uint8)

	return finalFrame, pred_img
	

	# #Show image
	# cv2.namedWindow('Frame prediction', cv2.WINDOW_NORMAL)
	# cv2.namedWindow('mask',cv2.WINDOW_NORMAL)
	# cv2.imshow("mask", pred_img)
	# cv2.imshow("Frame prediction", finalFrame)
	# cv2.waitKey(0)
	# cv2.destroyAllWindows()