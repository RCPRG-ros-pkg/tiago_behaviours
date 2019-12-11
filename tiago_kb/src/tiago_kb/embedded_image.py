#!/usr/bin/env python

import copy
import PyKDL
import numpy as np
import cv2

import transforms

def overlayImage(src, dst, pos, dst_shape=None):
    assert (not dst is None and dst_shape is None) or (dst is None and not dst_shape is None)
    src_sx = src.shape[1]
    src_sy = src.shape[0]

    if dst_shape is None:
        dst_shape = dst.shape
    dst_sx = dst_shape[1]
    dst_sy = dst_shape[0]

    src_x_min = 0
    src_x_max = src_sx

    src_y_min = 0
    src_y_max = src_sy

    dst_x_min = int(pos[0])
    dst_x_max = dst_x_min + src_sx

    dst_y_min = int(pos[1])
    dst_y_max = dst_y_min + src_sy

    if dst_x_min < 0:
        src_x_min = -dst_x_min
        dst_x_min = 0

    if dst_y_min < 0:
        src_y_min = -dst_y_min
        dst_y_min = 0

    if dst_x_max > dst_sx:
        src_x_max = src_x_max - (dst_x_max-dst_sx)
        dst_x_max = dst_sx

    if dst_y_max > dst_sy:
        src_y_max = src_y_max - (dst_y_max-dst_sy)
        dst_y_max = dst_sy

    if src_x_min >= src_x_max or src_y_min >= src_y_max:
        if dst is None:
            return np.zeros( dst_shape )
        else:
            return np.copy(dst)

    if dst is None:
        result = np.zeros( dst_shape )
    else:
        result = np.copy(dst)
    if len(src.shape) == 3:
        result[dst_y_min:dst_y_max, dst_x_min:dst_x_max, :] = src[src_y_min:src_y_max, src_x_min:src_x_max, :]
    else:
        result[dst_y_min:dst_y_max, dst_x_min:dst_x_max] = src[src_y_min:src_y_max, src_x_min:src_x_max]

    return result

# An EmbeddedImage is embedded in its own frame
class EmbeddedImage:
    def __init__(self, img, origin, res):
        self.__img__ = img
        # Position of origin of the image in the image frame [m]
        self.__origin__ = origin
        # Resolution in meters per pixel [m/px]
        self.__res__ = res

    def getResolution(self):
        return self.__res__

    def getOrigin(self):
        return self.__origin__

    def getImage(self):
        return self.__img__

    def getScaled(self, scale):
        img = cv2.resize(self.__img__, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
        return EmbeddedImage(img, self.__origin__, self.__res__/scale)

    def getTransformed(self, tf):
        rot = tf[1]
        px_center = ( int(-self.__origin__[0]/self.__res__), int(-self.__origin__[1]/self.__res__) )
        M_rot = cv2.getRotationMatrix2D( px_center, np.rad2deg(-rot), 1 )
        img_rot = cv2.warpAffine(self.__img__.astype(float), M_rot,
                                    (self.__img__.shape[1], self.__img__.shape[0]))
        origin_new = ( self.__origin__[0] + tf[0][0], self.__origin__[1] + tf[0][1] )
        return EmbeddedImage(img_rot, origin_new, self.__res__)

    def copy(self):
        return EmbeddedImage( self.__img__.copy(), copy.copy(self.__origin__), copy.copy(self.__res__) )

    def emptyCopy(self):
        return EmbeddedImage( np.zeros(self.__img__.shape), copy.copy(self.__origin__), copy.copy(self.__res__) )

    def getValueAt(self, pt):
        ix = int( float(pt[0]-self.__origin__[0]) / self.__res__ )
        iy = int( float(pt[1]-self.__origin__[1]) / self.__res__ )
        if ix < 0 or ix >= self.__img__.shape[1] or\
                iy < 0 or iy >= self.__img__.shape[0]:
            return None
        return self.__img__[iy, ix]

def addI(im1, im2):
    assert isinstance(im1, EmbeddedImage)
    assert isinstance(im2, EmbeddedImage)

    res1 = im1.getResolution()
    scale = im2.getResolution() / res1
    im2 = im2.getScaled( scale )

    # Now, both images have the same resolution
    assert abs(1.0 - im2.getResolution()/res1) < 0.0001

    orig1 = im1.getOrigin()
    orig2 = im2.getOrigin()
    pos_px = (float(orig2[0]-orig1[0])/res1, float(orig2[1]-orig1[1])/res1)
    im2_mv = overlayImage(im2.getImage(), None, pos_px, dst_shape=im1.getImage().shape)
    return EmbeddedImage( im1.getImage() + im2_mv, im1.getOrigin(), res1)

def multI(im, factor):
    assert isinstance(im, EmbeddedImage)
    return EmbeddedImage( im.getImage() * factor, im.getOrigin(), im.getResolution())

def int_tuple(pt):
    return (int(pt[0]), int(pt[1]))

def drawLine(im, pt1, pt2, color, thickness):
    assert isinstance(im, EmbeddedImage)
    res = im.getResolution()
    orig = im.getOrigin()
    pt1_px = ( (pt1[0]-orig[0])/res, (pt1[1]-orig[1])/res )
    pt2_px = ( (pt2[0]-orig[0])/res, (pt2[1]-orig[1])/res )
    cv2.line( im.getImage(), int_tuple(pt1_px), int_tuple(pt2_px), color, thickness=thickness )

def drawCircle(im, pt, radius, color, thickness):
    assert isinstance(im, EmbeddedImage)
    res = im.getResolution()
    orig = im.getOrigin()
    pt_px = ( (pt[0]-orig[0])/res, (pt[1]-orig[1])/res )
    cv2.circle( im.getImage(), int_tuple(pt_px), radius, color, thickness=thickness )

def test_EmbeddedImage(output_path):
    img1 = np.zeros( (500, 500) )
    cv2.circle(img1, (100,200), 20, 255, thickness=-1)
    cv2.line(img1, (100,100), (120,300), 255, thickness=5)
    cv2.line(img1, (0,0), (500,500), 255, thickness=2)
    im1 = EmbeddedImage(img1, (-2, -2), 0.01)

    img2 = np.zeros( (250, 250) )
    cv2.circle(img2, (50,100), 10, 255, thickness=-1)
    cv2.line(img2, (50,50), (60,150), 255, thickness=2)
    cv2.line(img2, (50,50), (150,60), 255, thickness=1)
    im2 = EmbeddedImage(img2, (-2, -2), 0.02)

    im3 = EmbeddedImage(img2, (-3, -3), 0.02)

    i_add1 = multI( addI(im1, im2), 0.5)
    i_add2 = multI( addI(im2, im1), 0.5)
    i_add3 = multI( addI(im3, im2), 0.5)

    cv2.imwrite(output_path + '/im1.png', im1.getImage())
    cv2.imwrite(output_path + '/im2.png', im2.getImage())

    cv2.imwrite(output_path + '/i_add1.png', i_add1.getImage())
    cv2.imwrite(output_path + '/i_add2.png', i_add2.getImage())
    cv2.imwrite(output_path + '/i_add3.png', i_add3.getImage())

    tf = ( (0.3, 0.1), np.deg2rad(25.0) )
    origin = (-4, -2)
    points = [ (0.5,0.5), (0.7, 2), (2,2), (2.5,1.5), (-origin[0], -origin[1]) ]
    im3 = EmbeddedImage(np.zeros( (400, 400) ), origin, 0.02)
    im4 = EmbeddedImage(np.zeros( (400, 400) ), origin, 0.02)
    for i in range(len(points)-1):
        #cv2.line( img, points[i], points[i+1], 255, thickness=4 )
        pt1 = ( points[i][0] + origin[0], points[i][1] + origin[1] )
        pt2 = ( points[i+1][0] + origin[0], points[i+1][1] + origin[1] )
        tf_pt1 = transforms.transformPoint(pt1, tf)
        tf_pt2 = transforms.transformPoint(pt2, tf)
        #cv2.line( img2, int_tuple(tf_pt1), int_tuple(tf_pt2), 255, thickness=4 )
        drawLine(im3, pt1, pt2, 255, 4)
        drawLine(im4, tf_pt1, tf_pt2, 255, 4)

    im3_tf = im3.getTransformed(tf)
    print 'im3 origin:', im3.getOrigin()
    print 'im3_tf origin:', im3_tf.getOrigin()
    cv2.imwrite(output_path + '/im3.png', im3.getImage())
    cv2.imwrite(output_path + '/im4.png', im4.getImage())
    cv2.imwrite(output_path + '/im3_tf.png', im3_tf.getImage())

    i_add4 = multI( addI(im3_tf, im4), 0.5)
    cv2.imwrite(output_path + '/im3_tf_im4.png', i_add4.getImage())
