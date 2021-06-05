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


def create_model(path):
    n_class =5 
    python2_state_dict = pickle.load(open(path,"rb"))

    vgg_model = VGGNet(requires_grad=True, remove_fc=True)
    model = FCNs(pretrained_net=vgg_model, n_class=n_class)
    model = nn.DataParallel(model, device_ids=[0])
    model.load_state_dict(python2_state_dict)

    if torch.cuda.is_available():
        model = model.cuda()

    return model

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

class centroid_module():
    def __init__(self):
        self.previous_centroid = None


    def is_in_neiborhood_of_previous(self,new_centroid):
        if self.previous_centroid is None:
            return False

        cx, cy = self.previous_centroid
        #create boundary
        size_b = 60
        neiborhood = [(cx-size_b,cy),(cx,cy+size_b),(cx+size_b,cy),(cx,cy-size_b),]
        neiborhood = np.array(neiborhood).reshape((-1,1,2)).astype(np.int32)

        dist1 = cv2.pointPolygonTest(neiborhood, new_centroid, True)
        if dist1 > 0: #If point inside legal area
            return True
        else:
            return False    
        
    def calculate_percentage_of_blood(self,mask):
        m = mask.copy()
        gray_image = cv2.cvtColor(m, cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray_image,80,255,0)
        #cv2.imshow('img',thresh)
        #cv2.waitKey(0)
        thresh = thresh/255
        #print(thresh.shape)
        #print(thresh.sum())
        #print(640*480)
        thresh = thresh.sum()
        return thresh/(640*480)

    def calculate_centroids(self,img):
        #Valid area for autonomy
        valid_area = [(528, 464),
                        (194, 468),
                        (108, 464),
                        (89, 228),
                        (92, 26),
                        (351, 11),
                        (503, 27),
                        (521, 135),
                        (528, 420),
                        (452, 473)]
        
        valid_area = np.array(valid_area).reshape((-1,1,2)).astype(np.int32)

        # convert the image to grayscale
        gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # convert the grayscale image to binary image
        ret,thresh = cv2.threshold(gray_image,80,255,0)
        #Morphological operations to reduce noise
        kernel_size = 7
        kernel = np.ones((kernel_size, kernel_size),np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=3)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=3)

        # find contours in the binary image
        contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        print(len(contours))

        final_results = []
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

            #If centroid is in the neiborhood of previous centroid skip it
            if self.is_in_neiborhood_of_previous((cX,cY)):
                print("centroid not included",(cX,cY))
                continue

            dist1 = cv2.pointPolygonTest(valid_area, (cX,cY), True)
            if dist1 > 0: #If point inside legal area
                final_results.append([area,[cX, cY]])   

        #Sort by the area
        final_results.sort(reverse=True,key=lambda x:x[0])
        final_results = [final_results[i][1] for i in range(len(final_results)) ]

        if len(final_results) > 0:
            self.previous_centroid = final_results[0]
        
        # print(final_results)
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
    if torch.cuda.is_available():
        inputs = torch.unsqueeze(inputs, 0).cuda() #changed from cuda to cpu
    else:
        inputs = torch.unsqueeze(inputs, 0).cpu()

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
