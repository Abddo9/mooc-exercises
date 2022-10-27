#!/usr/bin/env python
# coding: utf-8

# In[127]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

import cv2
import numpy as np


def get_steer_matrix_left_lane_markings(shape):
    """
        Args:
            shape: The shape of the steer matrix (tuple of ints)
        Return:
            steer_matrix_left_lane: The steering (angular rate) matrix for Braitenberg-like control 
                                    using the masked left lane markings (numpy.ndarray)
    """
    
    steer_matrix_left_lane = np.zeros(shape)
    for j in range(int(shape[1]*2/3)):
        for i in range(60,shape[0]):
            steer_matrix_left_lane[shape[0]-1-i,shape[1]-1-j] = -0.000001/np.exp(j/100) 
    #for j in range(int(shape[1]*2/3)):
        #steer_matrix_left_lane[:,shape[1]-1-j] = - 0.000001/np.exp(j/100) #10**134/np.exp(j)
    #steer_matrix_left_lane[:,int(shape[1]/4):shape[1]-1] = -.001

    return steer_matrix_left_lane


# In[130]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK


def get_steer_matrix_right_lane_markings(shape):
    """
        Args:
            shape: The shape of the steer matrix (tuple of ints)
        Return:
            steer_matrix_right_lane: The steering (angular rate) matrix for Braitenberg-like control 
                                     using the masked right lane markings (numpy.ndarray)
    """
    
    steer_matrix_right_lane = np.zeros(shape)
    for j in range(int(shape[1]*2/3)):
        for i in range(60,shape[0]):
            steer_matrix_right_lane[shape[0]-1-i,j] = 0.0000001/np.exp(j/100) 
    #for j in range(int(shape[1]*2/3)):
        #steer_matrix_right_lane[:,j] = 0.000001/np.exp(j/100) #10**134/np.exp(j)
    #steer_matrix_right_lane[:,0:int(shape[1]*3/4)+1] = 0.001

    return steer_matrix_right_lane


# In[124]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

import cv2
import numpy as np


def detect_lane_markings(image):
    """
        Args:
            image: An image from the robot's camera in the BGR color space (numpy.ndarray)
        Return:
            left_masked_img:   Masked image for the dashed-yellow line (numpy.ndarray)
            right_masked_img:  Masked image for the solid-white line (numpy.ndarray)
    """
    
    h, w, _ = image.shape
    
    # Convert it to gray
    img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    sigma = 3 
    # Smooth the image using a Gaussian kernel
    img_gaussian_filter = cv2.GaussianBlur(img,(0,0), sigma)
    
    # Compute the gradient
    sobelx = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,1,0)
    sobely = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,0,1)
    
    # Compute the magnitude of the gradients
    Gmag = np.sqrt(sobelx*sobelx + sobely*sobely)
    
    # Remove small gradients
    threshold = np.array([80])
    mask_mag = (Gmag > threshold)
    
    # Divide the image to teo parts
    mask_left = np.ones(sobelx.shape)
    mask_left[:,int(np.floor(w/2)):w + 1] = 0
    mask_right = np.ones(sobelx.shape)
    mask_right[:,0:int(np.floor(w/2))] = 0
    
    # try it also with all ones
    mask_ground = np.ones((h,w));
    #mask_ground[0:160,0:625] = 0;
    
    mask_sobelx_pos = (sobelx > 0)
    mask_sobelx_neg = (sobelx < 0)
    mask_sobely_pos = (sobely > 0)
    mask_sobely_neg = (sobely < 0)
    
    h_ratio = 179/360
    s_ratio = 255/100
    v_ratio = 255/100
    
    yellow_lower_hsv = np.array([20,100,100])  # after trying a lot, I found the specific range online  
    yellow_upper_hsv = np.array([30,255,255])  
    white_lower_hsv = np.array([0,3*s_ratio,55*v_ratio])         # this is by trying different values
    white_upper_hsv = np.array([320*h_ratio,20*s_ratio,100*v_ratio])  
    
    # Convert the image to HSV for any color-based filtering
    imghsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask_white = cv2.inRange(imghsv, white_lower_hsv, white_upper_hsv)
    mask_yellow = cv2.inRange(imghsv, yellow_lower_hsv, yellow_upper_hsv)
    
    mask_left_edge = mask_ground * mask_left * mask_mag * mask_sobelx_neg * mask_sobely_neg * mask_yellow
    mask_right_edge = mask_ground * mask_right * mask_mag * mask_sobelx_pos * mask_sobely_neg * mask_white
    
    return (mask_left_edge, mask_right_edge)

