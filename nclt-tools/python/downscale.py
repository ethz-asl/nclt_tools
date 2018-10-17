"""
Demonstrating how to undistort images.

Reads in the given calibration file, parses it, and uses it to undistort the given
image. Then display both the original and undistorted images.

To use:

    python undistort.py image calibration_file
"""

import numpy as np
import cv2
import matplotlib.pyplot as plt
import argparse
import re
import IPython
import os
import csv

start_end_times_us = {
  #'2012-01-08': (1326031433535532, 1326031967545537),
  #'2012-01-15': (1326654469953068, 1326655069978983),
  #'2012-01-22': (1327251905811462, 1327252497878984),
  #'2012-02-02': (1328212051476852, 1328212453471751),
  #'2012-02-04': (1328393682247239, 1328394208160485),
  #'2012-02-05': (1328449685371162, 1328450205367313),
  #'2012-02-12': (1329069344143600, 1329069926199849),
  #'2012-02-18': (1329601461802048, 1329602019798243),
  #'2012-02-19': (1329676290214232, 1329676860246575),
  #'2012-03-17': (1331989931467924, 1331990435511798),
  #'2012-03-25': (1332701879650421, 1332702451733989),
  #'2012-03-31': (1333216660220586, 1333217258266406),
  #'2012-04-29': (1335705547785012, 1335706165816232),
  #'2012-05-11': (1336760625227772, 1336761163223778),
  #'2012-05-26': (1338076324050166, 1338076886096364),
  #'2012-06-15': (1339760531033684, 1339761071023474),
  #'2012-08-04': (1344082664305715, 1344083162360426),
  #'2012-08-20': (1345505758743350, 1345506290715394),
  #'2012-09-28': (1348867829415324, 1348868311331502),
  #'2012-10-28': (1351447257386290, 1351447733309056),
  #'2012-11-16': (1353100254746174, 1353100786747252),
  '2012-11-04': (1352043741677010, 1352044391570802),
  #'2012-11-17': (1353175976858135, 1353176632776230),
  #'2012-12-01': (1354400544482626, 1354401076426224),
  #'2013-02-23': (1361647468373810, 1361648026267947),
  #'2013-04-05': (1365178967560658, 1365179387483361)
  }
  
datasets_to_process = set(['2012-11-04'])

class Undistort(object):

    def __init__(self, fin, scale=1.0, fmask=None):
        self.fin = fin
        # read in distort
        with open(fin, 'r') as f:
            #chunks = f.readline().rstrip().split(' ')
            header = f.readline().rstrip()
            chunks = re.sub(r'[^0-9,]', '', header).split(',')
            self.mapu = np.zeros((int(chunks[1]),int(chunks[0])),
                    dtype=np.float32)
            self.mapv = np.zeros((int(chunks[1]),int(chunks[0])),
                    dtype=np.float32)
            for line in f.readlines():
                chunks = line.rstrip().split(' ')
                self.mapu[int(chunks[0]),int(chunks[1])] = float(chunks[3])
                self.mapv[int(chunks[0]),int(chunks[1])] = float(chunks[2])
        # generate a mask
        self.mask = np.ones(self.mapu.shape, dtype=np.uint8)
        self.mask = cv2.remap(self.mask, self.mapu, self.mapv, cv2.INTER_LINEAR)
        kernel = np.ones((30,30),np.uint8)
        self.mask = cv2.erode(self.mask, kernel, iterations=1)

    """
    Optionally, define a mask
    """
    def set_mask(fmask):
        # add in the additional mask passed in as fmask
        if fmask:
            mask = cv2.cvtColor(cv2.imread(fmask), cv2.COLOR_BGR2GRAY)
            self.mask = self.mask & mask
        new_shape = (int(self.mask.shape[1]*scale), int(self.mask.shape[0]*scale))
        self.mask = cv2.resize(self.mask, new_shape,
                               interpolation=cv2.INTER_CUBIC)
        #plt.figure(1)
        #plt.imshow(self.mask, cmap='gray')
        #plt.show()

    """
    Use OpenCV to undistorted the given image
    """
    def undistort(self, img):
        return cv2.resize(cv2.remap(img, self.mapu, self.mapv, cv2.INTER_LINEAR),
                          (self.mask.shape[1], self.mask.shape[0]),
                          interpolation=cv2.INTER_CUBIC)
              
def process(image_folder, undistort_map):
  for root, dirs, files in os.walk(image_folder):
    base, c_folder = os.path.split(root)
    images_folder, lb3_folder = os.path.split(base)
    images_folder, date_folder = os.path.split(images_folder)
    print 'Date ' + date_folder

    print 'Got ', len(files), ' images to process. Date ' + date_folder
    skipped = 0
    processed = 0
    for f in files:
      f_name, ext = os.path.splitext(f)
      
      processed += 1
      image_filename = root + '/' + f

      out_directory = base + "/../Undistorted-Downscaled-SE/" + c_folder
      #print 'Image output directory: ', out_directory
      out_image_filename = out_directory + '/' + f_name + ".png"
      if not os.path.exists(out_directory):
        os.makedirs(out_directory)

      if True or not os.path.exists(out_image_filename):
        im = cv2.imread(image_filename)
        im_gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        im_gray_downscaled = cv2.resize(im_gray, (im_gray.shape[1] / 2, im_gray.shape[0] / 2))
        cv2.imwrite(out_image_filename, im_gray_downscaled) 
      
      if processed % 100 == 0:
        print 'Processed ', str(processed), ' images, skipped ' + str(skipped) + ' images.'
    

def main():
    parser = argparse.ArgumentParser(description="Undistort images")
    #parser.add_argument('image_folders_path',  type=str, help='image_folders_path')
    #parser.add_argument('u_maps', type=str, help='undistortion maps')

    images_folder = '../images'
    umaps_folder = '../U2D_ALL_1616X1232' #U2D_Cam5_1616X1232.txt
    #args = parser.parse_args()

    num_cams = 6
    for cam_index in range(num_cams):
      print 'Processing cam ', cam_index
      cam_folder = 'Cam' + str(cam_index)

      u_map = umaps_folder + '/U2D_Cam' +str(cam_index) + '_1616X1232.txt'
      for name in os.listdir(images_folder):
          if os.path.isdir(os.path.join(images_folder, name)):
              if name in datasets_to_process:
                image_path = os.path.join(images_folder, name, 'lb3/Undistorted-WS', cam_folder)
                print 'Processing directory ' + image_path
                process(image_path, u_map)

if __name__ == "__main__":
    main()
