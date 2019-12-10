#!/usr/bin/env python

import os
import math
import PyKDL
import numpy as np
import xml.dom.minidom as minidom
import cv2
import yaml

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

def transformPoint(pt, tf):
    fr = PyKDL.Frame( PyKDL.Rotation.RotZ(tf[1]), PyKDL.Vector(tf[0][0], tf[0][1], 0) )
    tf_pt = fr * PyKDL.Vector(pt[0], pt[1], 0)
    return (tf_pt.x(), tf_pt.y())

def transformVector(vec, tf):
    rotM = PyKDL.Rotation.RotZ(tf[1])
    tf_n = rotM * PyKDL.Vector(vec[0], vec[1], 0)
    return (tf_n.x(), tf_n.y())

def rad2deg(angle_rad):
    return 180.0 * angle_rad / math.pi

def deg2rad(angle_rad):
    return math.pi * angle_rad / 180.0

def parsePointStr(point_str):
    fields = point_str.strip().split()
    assert len(fields) == 2
    return ( float(fields[0]), float(fields[1]) )

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
        M_rot = cv2.getRotationMatrix2D( px_center, rad2deg(-rot), 1 )
        img_rot = cv2.warpAffine(self.__img__.astype(float), M_rot,
                                    (self.__img__.shape[1], self.__img__.shape[0]))
        origin_new = ( self.__origin__[0] + tf[0][0], self.__origin__[1] + tf[0][1] )
        return EmbeddedImage(img_rot, origin_new, self.__res__)

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

def drawPointPlace(img, pl, color):
    assert isinstance(img, EmbeddedImage)
    assert pl.getType() == 'point'
    pt = pl.getPt()
    n = pl.getN()
    res = img.getResolution()
    pt2 = (pt[0] + 15.0*res*n[0], pt[1] + 15.0*res*n[1])

    drawCircle(img, pt, 10, color, 1)
    drawLine(img, pt, pt2, color, 1)

class VolumetricPlace:
    def __init__(self, pl_id, name, img, origin, res):
        self.__pl_id__  = pl_id
        self.__eimg__ = EmbeddedImage(img, origin, res)
        self.__name__  = name

    def getType(self):
        return 'volumetric'

    def getId(self):
        return self.__pl_id__

    def getName(self):
        return self.__name__

    def getEmbeddedImage(self):
        return self.__eimg__

    def getTransformed(self, tf):
        eimg = self.__eimg__.getTransformed(tf)
        return VolumetricPlace(self.__pl_id__, self.__name__, eimg.getImage(),
                                eimg.getOrigin(), eimg.getResolution())

class PointPlace:
    def __init__(self, pl_id, name, pt, n):
        self.__pl_id__  = pl_id
        self.__name__  = name
        self.__pt__ = pt
        self.__n__ = n

    def getType(self):
        return 'point'

    def getId(self):
        return self.__pl_id__

    def getName(self):
        return self.__name__

    def getPt(self):
        return self.__pt__

    def getN(self):
        return self.__n__

    def getTransformed(self, tf):
        tf_pt = transformPoint(self.__pt__, tf)
        tf_n = transformVector(self.__n__, tf)
        return PointPlace( self.__pl_id__, self.__name__, tf_pt, tf_n)

class MapContext:
    def __init__(self, name, resolution, origin, map_img):
        self.__name__ = name
        self.__map__ = EmbeddedImage(map_img, origin, resolution)
        self.__volumetric_mask__ = None
        self.__volumetric_places__ = []
        self.__point_places__ = []

    def getName(self):
        return self.__name__

    def setVolumetricMask(self, img):
        if self.__volumetric_mask__ is None:
            assert img.shape == self.__map__.getImage().shape
            self.__volumetric_mask__ = EmbeddedImage(img, self.__map__.getOrigin(),
                                        self.__map__.getResolution() )
        else:
            raise Exception('Method setVolumetricMask executed twice for the same MapContext object.')

    def getMap(self):
        return self.__map__

    def getVolumetricMask(self):
        return self.__volumetric_mask__

    def getPlaceById(self, pl_id):
        for pl in self.__volumetric_places__:
            if pl.getId() == pl_id:
                return pl
        for pl in self.__point_places__:
            if pl.getId() == pl_id:
                return pl
        return None

    def getPlaceByName(self, name):
        for pl in self.__volumetric_places__:
            if pl.getName() == name:
                return pl
        for pl in self.__point_places__:
            if pl.getName() == name:
                return pl
        return None

    def addVolumetricPlace(self, pl_id, name, img):
        if not self.getPlaceById( pl_id ) is None:
            raise Exception('Two places with the same id: "' + pl_id + '"')
        if not self.getPlaceByName( name ) is None:
            raise Exception('Two places with the same name: "' + name + '"')
        assert img.shape == self.__map__.getImage().shape
        self.__volumetric_places__.append(
                VolumetricPlace(pl_id, name, img, self.__map__.getOrigin(), self.__map__.getResolution()) )

    def addPointPlace(self, pl_id, name, position, front_vec):
        if not self.getPlaceById( pl_id ) is None:
            raise Exception('Two places with the same id: "' + pl_id + '"')
        if not self.getPlaceByName( name ) is None:
            raise Exception('Two places with the same name: "' + name + '"')
        self.__point_places__.append( PointPlace(pl_id, name, position, front_vec) )

class PlacesXml:
    def __init__(self, filename):
        xml_str = None
        with open(filename, "r") as f:
            xml_str = f.read()
        dom = minidom.parseString(xml_str)

        head_tail = os.path.split(filename)
        self.__cwd__ = head_tail[0]
        assert self.__cwd__

        self.__map_contexts__ = []
        self.__map_transforms__ = []
        for n in dom.childNodes:
            if n.nodeType == n.ELEMENT_NODE:
                if n.tagName == "places":
                    self.parsePlaces( n )

    def parsePlaces(self, xml):
        assert xml.tagName == "places"

        for n in xml.childNodes:
            if n.nodeType == n.ELEMENT_NODE:
                if n.tagName == "map_context":
                    self.parseMapContext( n )
                elif n.tagName == "map_transform":
                    self.parseMapTransform( n )
                else:
                    raise Exception('Unknown xml tag "' + n.tagName + '" within "places" node.')

    def parseMapContext(self, xml):
        assert xml.tagName == "map_context"

        name_str = xml.getAttribute("name")
        map_param_filename_str = xml.getAttribute("map_param_filename")
        map_filename_str = xml.getAttribute("map_filename")

        map_img = cv2.imread(self.__cwd__ + '/' + map_filename_str, cv2.IMREAD_GRAYSCALE)
        map_img = np.flipud(map_img)
        with open(self.__cwd__ + '/' + map_param_filename_str, 'r') as yaml_file:
            map_file_yaml = yaml.safe_load(yaml_file)
            resolution = map_file_yaml['resolution']
            origin = ( map_file_yaml['origin'][0], map_file_yaml['origin'][1] )

        mc = MapContext(name_str, resolution, origin, map_img)

        for n in xml.childNodes:
            if n.nodeType == n.ELEMENT_NODE:
                if n.tagName == "volumetric_mask":
                    self.parseVolumetricMask( n, mc )
                elif n.tagName == "volumetric_place":
                    self.parseVolumetricPlace( n, mc )
                elif n.tagName == "point_place":
                    self.parsePointPlace( n, mc )
                else:
                    raise Exception('Unknown xml tag "' + n.tagName + '" within "places" node.')
        self.__map_contexts__.append(mc)

    def parseVolumetricMask( self, xml, mc ):
        # <volumetric_mask filename="img/places_mask.png" />
        assert xml.tagName == "volumetric_mask"
        filename_str = xml.getAttribute("filename")
        img = cv2.imread(self.__cwd__ + '/' + filename_str, cv2.IMREAD_GRAYSCALE)
        img = np.flipud(img)
        mc.setVolumetricMask(img)

    def parseVolumetricPlace( self, xml, mc ):
        # <volumetric_place id="warsztat" name="warsztat" filename="img/warsztat.png" />
        assert xml.tagName == "volumetric_place"
        id_str = xml.getAttribute("id")
        name_str = xml.getAttribute("name")
        filename_str = xml.getAttribute("filename")

        img = cv2.imread(self.__cwd__ + '/' + filename_str, cv2.IMREAD_GRAYSCALE)
        img = np.flipud(img)
        mc.addVolumetricPlace(id_str, name_str, img)

    def parsePointPlace( self, xml, mc ):
        # <point_place id="fotel" name="fotel" position="-0.64 0.34" front_vec="0 -1" />
        assert xml.tagName == "point_place"
        id_str = xml.getAttribute("id")
        name_str = xml.getAttribute("name")
        position_str = xml.getAttribute("position")
        front_vec_str = xml.getAttribute("front_vec")

        pos = parsePointStr( position_str )
        vec_n = parsePointStr( front_vec_str )
        mc.addPointPlace(id_str, name_str, pos, vec_n)

    def parseMapTransform(self, xml):
        # <map_transform map1="sim" map2="real" translation="-1.87296056747 -0.53914129734" rotation="-0.0821140241595" />
        assert xml.tagName == "map_transform"

        map1_str = xml.getAttribute("map1")
        map2_str = xml.getAttribute("map2")
        translation_str = xml.getAttribute("translation")
        rotation_str = xml.getAttribute("rotation")

        transl = parsePointStr( translation_str )
        rot = float(rotation_str)

        self.__map_transforms__.append( (map1_str, map2_str, transl, rot) )

    def getMapsTransform(self, map2, map1):
        for mt in self.__map_transforms__:
            transl = mt[2]
            rot = mt[3]
            if mt[0] == map1 and mt[1] == map2:
                return transl, rot
            elif mt[0] == map2 and mt[1] == map1:
                fr = PyKDL.Frame( PyKDL.Rotation.RotZ(rot), PyKDL.Vector(transl[0], transl[1], 0) )
                fr_inv = fr.Inverse()
                return (fr_inv.p.x(), fr_inv.p.y()), -rot
        raise Exception('No such transform is available: "' + map1 + '", "' + map2 + '"')

    def getPlaceById(self, pl_id, dest_map_context_name):
        for mc in self.__map_contexts__:
            pl = mc.getPlaceById( pl_id )
            if pl is None:
                continue
            if mc.getName() == dest_map_context_name:
                return pl
            else:
                tf = self.getMapsTransform(mc.getName(), dest_map_context_name)
                return pl.getTransformed(tf)
        raise Exception( 'Place id not found: "' + pl_id + '"')

    def getPlaceByName(self, name, dest_map_context_name):
        for mc in self.__map_contexts__:
            pl = mc.getPlaceByName( name )
            if pl is None:
                continue
            if mc.getName() == dest_map_context_name:
                return pl
            else:
                tf = self.getMapsTransform(mc.getName(), dest_map_context_name)
                return pl.getTransformed(tf)
        raise Exception( 'Place name not found: "' + name + '"')

    def getMask(self, dest_map_context_name):
        for mc in self.__map_contexts__:
            vol_mask = mc.getVolumetricMask()
            if vol_mask is None:
                continue
            if mc.getName() == dest_map_context_name:
                return vol_mask
            else:
                tf = self.getMapsTransform(mc.getName(), dest_map_context_name)
                return vol_mask.getTransformed(tf)
        raise Exception( 'Volumetric mask not found')

    def getMapContext(self, mc_name):
        for mc in self.__map_contexts__:
            if mc.getName() == mc_name:
                return mc
        raise Exception('Map context not found: "' + mc_name + '"')

#
# Tests
#

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

    tf = ( (0.3, 0.1), deg2rad(25.0) )
    origin = (-4, -2)
    points = [ (0.5,0.5), (0.7, 2), (2,2), (2.5,1.5), (-origin[0], -origin[1]) ]
    im3 = EmbeddedImage(np.zeros( (400, 400) ), origin, 0.02)
    im4 = EmbeddedImage(np.zeros( (400, 400) ), origin, 0.02)
    for i in range(len(points)-1):
        #cv2.line( img, points[i], points[i+1], 255, thickness=4 )
        pt1 = ( points[i][0] + origin[0], points[i][1] + origin[1] )
        pt2 = ( points[i+1][0] + origin[0], points[i+1][1] + origin[1] )
        tf_pt1 = transformPoint(pt1, tf)
        tf_pt2 = transformPoint(pt2, tf)
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

def test_PlacesXML(output_path, places_xml_filename):
    print 'Parsing file "' + places_xml_filename + '"'

    places = PlacesXml(places_xml_filename)

    if False:
        tf_S_R = places.getMapsTransform('sim', 'real')
        tf_R_S = places.getMapsTransform('real', 'sim')
        print 'tf sim -> real', tf_S_R
        print 'tf real -> sim', tf_R_S
        tf1 = PyKDL.Frame( PyKDL.Rotation.RotZ(tf_S_R[1]), PyKDL.Vector(tf_S_R[0][0], tf_S_R[0][1], 0) )
        tf2 = PyKDL.Frame( PyKDL.Rotation.RotZ(tf_R_S[1]), PyKDL.Vector(tf_R_S[0][0], tf_R_S[0][1], 0) )
        print tf1*tf2

    for mc_name in ['sim', 'real']:
        pl_kuchnia = places.getPlaceById('kuchnia', mc_name)
        pl_fotel = places.getPlaceById('fotel', mc_name)
        pl_drzwi = places.getPlaceById('drzwi', mc_name)
        pl_korytarz_a = places.getPlaceById('korytarz_a', mc_name)

        eimg_kuchnia = pl_kuchnia.getEmbeddedImage()
        eimg_korytarz_a = pl_korytarz_a.getEmbeddedImage()

        # Draw places over the map
        eimg_map = places.getMapContext(mc_name).getMap()

        i_add1 = multI( addI(eimg_map, addI(eimg_kuchnia, eimg_korytarz_a)), 0.3)
        i_add2 = multI( addI(eimg_kuchnia, addI(eimg_map, eimg_korytarz_a)), 0.3)
        i_add3 = multI( addI(eimg_korytarz_a, addI(eimg_kuchnia, eimg_map)), 0.3)

        eimg_mask = places.getMask(mc_name)
        i_add4 = multI( addI(eimg_mask, eimg_map), 0.3)
        i_add5 = multI( addI(eimg_map, eimg_mask), 0.3)

        for im in [i_add1, i_add2, i_add3]:
            drawPointPlace(im, pl_fotel, 255)
            drawPointPlace(im, pl_drzwi, 255)

        cv2.imwrite(output_path + '/' + mc_name + '_map1.png', i_add1.getImage())
        cv2.imwrite(output_path + '/' + mc_name + '_map2.png', i_add2.getImage())
        cv2.imwrite(output_path + '/' + mc_name + '_map3.png', i_add3.getImage())
        cv2.imwrite(output_path + '/' + mc_name + '_map4.png', i_add4.getImage())
        cv2.imwrite(output_path + '/' + mc_name + '_map5.png', i_add5.getImage())
