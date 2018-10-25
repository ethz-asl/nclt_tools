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
    undistort = Undistort(undistort_map)
    print 'Loaded camera calibration'

    for root, dirs, files in os.walk(image_folder):
      base, c_folder = os.path.split(root)
      images_folder, lb3_folder = os.path.split(base)
      images_folder, date_folder = os.path.split(images_folder)
      print 'Date ' + date_folder

    print 'Got ', len(files), ' images to process. Date ' + date_folder + '.'
    skipped_undist = 0
    skipped_ds = 0
    processed = 0
    for f in files:
      f_name, ext = os.path.splitext(f)
     
      processed += 1
      image_filename = root + '/' + f

      out_directory = base + "/Undistorted-All/" + c_folder
      out_directory_ds = base + "/Undistorted-Downscaled-All/" + c_folder

      #print 'Image output directory: ', out_directory
      out_image_filename = out_directory + '/' + f_name + ".png"
      out_image_filename_ds = out_directory_ds + '/' + f_name + ".png"
      if not os.path.exists(out_directory):
        os.makedirs(out_directory)
      if not os.path.exists(out_directory_ds):
        os.makedirs(out_directory_ds)

      if not os.path.exists(out_image_filename) or not os.path.exists(out_image_filename_ds):
        im = cv2.imread(image_filename)
        im_undistorted = undistort.undistort(im)
        im_gray = cv2.cvtColor(im_undistorted, cv2.COLOR_BGR2GRAY)
        
        if not os.path.exists(out_image_filename):
          cv2.imwrite(out_image_filename, im_gray)
        else:
          skipped_undist += 1
  
        if not os.path.exists(out_image_filename_ds):
          im_gray_downscaled = cv2.resize(im_gray, (im_gray.shape[1] / 2, im_gray.shape[0] / 2))
          cv2.imwrite(out_image_filename_ds, im_gray_downscaled)
        else:
          skipped_ds += 1
      else:
        skipped_undist += 1
        skipped_ds += 1

      if processed % 100 == 0:
        print 'Processed ', str(processed), ' images, skipped undist ' + str(skipped_undist) + ', ds: ', str(skipped_ds), '.'
      

def main():
    parser = argparse.ArgumentParser(description="Undistort images")
    #parser.add_argument('image_folders_path',  type=str, help='image_folders_path')
    #parser.add_argument('u_maps', type=str, help='undistortion maps')

    images_folder = '/mnt/ASL_public_datasets/umich/umich/images'
    umaps_folder = '/mnt/ASL_public_datasets/umich/umich/U2D_ALL_1616X1232' #U2D_Cam5_1616X1232.txt
    #args = parser.parse_args()

    for name in os.listdir(images_folder):
      if name == '2012-01-08':
        continue

      num_cams = 6
      for cam_index in range(num_cams):
        print 'Processing cam ', cam_index
        cam_folder = 'Cam' + str(cam_index)

        u_map = umaps_folder + '/U2D_Cam' +str(cam_index) + '_1616X1232.txt'

        if os.path.isdir(os.path.join(images_folder, name)):
            image_path = os.path.join(images_folder, name, 'lb3', cam_folder)
            print 'Processing directory ' + image_path
            process(image_path, u_map)

if __name__ == "__main__":
    main()
