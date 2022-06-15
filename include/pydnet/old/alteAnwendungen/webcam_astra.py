#
# MIT License
#
# Copyright (c) 2018 Matteo Poggi m.poggi@unibo.it
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


import tensorflow as tf
import sys
import os
import argparse
import time
import datetime
from utils import *
from pydnet import *

from primesense import openni2
from primesense import _openni2 as c_api
import numpy as np
import cv2

import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as plt3D

openni2_dist="/home/stephan/turtlebot2_ros_melodic/catkin_ws/src/astra_camera/include/openni2_redist/x64/"

# forces tensorflow to run on CPU
os.environ['CUDA_VISIBLE_DEVICES'] = '-1'

parser = argparse.ArgumentParser(description='Argument parser')

""" Arguments related to network architecture"""
parser.add_argument('--width', dest='width', type=int, default=512, help='width of input images')
parser.add_argument('--height', dest='height', type=int, default=256, help='height of input images')
parser.add_argument('--resolution', dest='resolution', type=int, default=1, help='resolution [1:H, 2:Q, 3:E]')
parser.add_argument('--checkpoint_dir', dest='checkpoint_dir', type=str, default='checkpoint/IROS18/pydnet', help='checkpoint directory')

args = parser.parse_args()

def main(_):

  with tf.Graph().as_default():
    height = args.height
    width = args.width
    placeholders = {'im0':tf.placeholder(tf.float32,[None, None, None, 3], name='im0')}

    with tf.variable_scope("model") as scope:
      model = pydnet(placeholders)

    init = tf.group(tf.global_variables_initializer(),
                   tf.local_variables_initializer())

    loader = tf.train.Saver()
    saver = tf.train.Saver()
    
    openni2.initialize(openni2_dist)
    if (openni2.is_initialized()):
        print("OpenNI2 initialized")
    else:
        raise ValueError("OpenNI2 failed to initialize!!")
    
    ## Register the device
    dev = openni2.Device.open_any()
    
    ## Create the streams stream
    rgb_stream = dev.create_color_stream()
    depth_stream = dev.create_depth_stream()
    
    ## Configure the depth_stream -- changes automatically based on bus speed
    #print 'Depth video mode info', depth_stream.get_video_mode() # Checks depth video configuration
    depth_stream.set_video_mode(c_api.OniVideoMode(pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_1_MM, resolutionX=640, resolutionY=480, fps=30))
    
    ## Check and configure the mirroring -- default is True
    ## Note: I disable mirroring
    # print 'Mirroring info1', depth_stream.get_mirroring_enabled()
    depth_stream.set_mirroring_enabled(False)
    rgb_stream.set_mirroring_enabled(False)
    
    ## More infor on streams depth_ and rgb_
    #help(depth_stream)
    
    
    ## Start the streams
    rgb_stream.start()
    depth_stream.start()
    
    ## Synchronize the streams
    dev.set_depth_color_sync_enabled(True) # synchronize the streams
    
    ## IMPORTANT: ALIGN DEPTH2RGB (depth wrapped to match rgb stream)
    dev.set_image_registration_mode(openni2.IMAGE_REGISTRATION_DEPTH_TO_COLOR)
    
    def get_rgb():
        """
        Returns numpy 3L ndarray to represent the rgb image.
        """
        bgr   = np.fromstring(rgb_stream.read_frame().get_buffer_as_uint8(),dtype=np.uint8).reshape(480,640,3)#.reshape(240,320,3)
        rgb   = cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
        return rgb
    #get_rgb


    def get_depth():
        """
        Returns numpy ndarrays representing the raw and ranged depth images.
        Outputs:
            dmap:= distancemap in mm, 1L ndarray, dtype=uint16, min=0, max=2**12-1
            d4d := depth for dislay, 3L ndarray, dtype=uint8, min=0, max=255    
        Note1: 
            fromstring is faster than asarray or frombuffer
        Note2:     
            .reshape(120,160) #smaller image for faster response 
                    OMAP/ARM default video configuration
            .reshape(240,320) # Used to MATCH RGB Image (OMAP/ARM)
                    Requires .set_video_mode
        """
        dmap = np.fromstring(depth_stream.read_frame().get_buffer_as_uint16(),dtype=np.uint16).reshape(480,640)  # Works & It's FAST
        d4d = np.uint8(dmap.astype(float) *255/ 2**12-1) # Correct the range. Depth images are 12bits
        d4d = 255 - cv2.cvtColor(d4d,cv2.COLOR_GRAY2RGB)
        return dmap, d4d
    #get_depth
    
    #cam = cv2.VideoCapture(0)

    with tf.Session() as sess:
        sess.run(init)
        loader.restore(sess, args.checkpoint_dir)

        
        while True:
          """
          for i in range(4):
            cam.grab()
          ret_val, img = cam.read()
          """
          
          img = get_rgb()
          #DEPTH (For comparison)
          dmm,d4d = get_depth()
          d4d_color = cv2.resize(d4d, (width, height))#.astype(np.float32) / 255.
          #d4d_color = cv2.resize(d4d, (640, 320))#.astype(np.float32) / 255.
          #d4d_color = cv2.applyColorMap(d4d_color, cv2.COLORMAP_INFERNO)
         
          
          img = cv2.resize(img, (width, height)).astype(np.float32) / 255.
          img = np.expand_dims(img, 0)
          start = time.time()
          disp = sess.run(model.results[args.resolution-1], feed_dict={placeholders['im0']: img})
          end = time.time()

          disp_color = applyColorMap(disp[0,:,:,0]*20, 'gist_gray')*255.
          dist_color = applyColorMap(disp[0,:,:,1]*20, 'gist_gray')*255.
          
          img_disp = img[0] * 255.
          #print(np.array(img_disp).shape, np.array(dist_color).shape, np.array(d4d_color).shape) 
          #toShow = (np.concatenate((img_disp, disp_color,  dist_color, d4d_color), 0)).astype(np.uint8)
          #toShow = (np.concatenate((img_disp, disp_color, d4d_color), 0)).astype(np.uint8)
          toShow = (np.concatenate((img_disp, dist_color, d4d_color), 0)).astype(np.uint8)
          #toShow = (np.concatenate((img_disp, disp_color, dist_color), 0)).astype(np.uint8)




          toShow = cv2.resize(toShow, (width/2, height))
          
          cv2.imshow('pydnet', toShow)
          
          dmm_pc = dmm/1000.0 #Scaling from mm to m
          orig_shape = np.shape(dmm_pc)
          
          network_scaler = 255.0#/100.0
          x_disp = range(0,np.shape(disp[0,:,:,0])[0])
          y_disp = range(0,np.shape(disp[0,:,:,0])[1])
          z_disp = disp[0,:,:,0] * network_scaler
          
          x_dist = range(0,np.shape(disp[0,:,:,1])[0])
          y_dist = range(0,np.shape(disp[0,:,:,1])[1])
          z_dist = disp[0,:,:,1] * network_scaler
          
          x_dm = range(0,np.shape(dmm)[0])
          y_dm = range(0,np.shape(dmm)[1])
          z_dm = dmm / 1000.0    #scale to m
          
          #print(z_disp)
          #print(z_dist)
          #print(z_dm)
          
          
          
          #print(np.shape(dmm_pc), np.shape(d4d), np.shape(disp))
          
          #print(disp[0, :,:,0])
          #print(disp[0, :,:,1])
          #disp_pc = cv2.resize(disp[0,:,:,0], (orig_shape)) *20.*255./100. #TODO: Check Factor
          
          #err = dmm_pc - disp_pc    
          
          
          k = cv2.waitKey(1)         
          if k == 1048603 or k == 27:
              np.set_printoptions(threshold=sys.maxsize)
              f = open("./testfile.txt", "w")
              #f.write(np.array2string(z_dm))
              #f.write(np.array2string(z_disp))
              f.write(np.array2string(z_dist))
              f.close()
              break  # esc to quit
          if k == 1048688:
              cv2.waitKey(0) # 'p' to pause
            

          print("Time: " + str(end - start))
          del img
          del disp
          del toShow
          
        # Release resources   
        #cam.release()        
        cv2.destroyAllWindows()
        rgb_stream.stop()
        depth_stream.stop()
        openni2.unload()
        print ("Terminated")

if __name__ == '__main__':
    tf.app.run()
