import cv2
import numpy as np
video = 'visibility_calculation'
vidcap = cv2.VideoCapture(video+'.mp4')
success,image = vidcap.read()
count = 0
cv2.imwrite(video+'.jpg', image)     # save frame as JPEG file      
