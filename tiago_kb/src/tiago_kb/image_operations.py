#!/usr/bin/env python

import cv2
import numpy as np

def binaryImage(img, threshold):
	result = np.zeros( img.shape )
	result[img>threshold] = 1
	return result

def normalizeImage(img):
    v_min = np.min(img)
    v_max = np.max(img)
    result = np.subtract(img, v_min)
    if abs(v_max-v_min) > 0.000001:
        result = np.multiply( result, 255.0/(v_max-v_min) )
    return result

class OperationEnlarge:
    def __init__(self, radius, enlarge_value, border_value=None):
        assert enlarge_value == 0 or enlarge_value == 1
        self.radius = radius
        self.border_value = border_value
        self.enlarge_value = enlarge_value
        self.kernel = np.zeros( (radius*2+1, radius*2+1) )
        cv2.circle(self.kernel, (radius, radius), radius, 1, thickness=-1)
        self.sum_kernel = np.sum( self.kernel )

    def compute(self, in_data):
        img = in_data['img'].astype(float)
        assert type(img) is np.ndarray

        if self.border_value is None:
            img_corr = cv2.filter2D(img, -1, self.kernel, borderType=cv2.BORDER_REPLICATE)
        else:
            img_border = cv2.copyMakeBorder(img, 1, 1, 1, 1, borderType=cv2.BORDER_CONSTANT, value=self.border_value)
            img_corr = cv2.filter2D(img_border, -1, self.kernel, borderType=cv2.BORDER_REPLICATE)[1:-1,1:-1]
            assert img_corr.shape[0] == img.shape[0] and img_corr.shape[1] == img.shape[1]

        if self.enlarge_value == 1:
            result = np.zeros( img_corr.shape, dtype=int )
            result[ img_corr < -0.5 ] = 1
            result[ img_corr > 0.5 ] = 1
        else:
            result = np.ones( img_corr.shape, dtype=int )
            result[ img_corr < self.sum_kernel-0.5 ] = 0
            result[ img_corr > self.sum_kernel+0.5 ] = 0

        return {'img' : result}
