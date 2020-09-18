#!/usr/bin/env python

import os
import math
import PyKDL
import numpy as np
import xml.dom.minidom as minidom
import cv2
import yaml

import skfmm

import embedded_image as ei
import transforms
import image_operations as iop

dbg_output_path = '/home/dseredyn/tiago_public_ws/img'

def drawPointPlace(img, pl, color):
    assert isinstance(img, ei.EmbeddedImage)
    assert pl.getType() == 'point'
    pt = pl.getPt()
    n = pl.getN()
    res = img.getResolution()
    pt2 = (pt[0] + 15.0*res*n[0], pt[1] + 15.0*res*n[1])

    ei.drawCircle(img, pt, 10, color, 1)
    ei.drawLine(img, pt, pt2, color, 1)

def parsePointStr(point_str):
    fields = point_str.strip().split()
    assert len(fields) == 2
    return ( float(fields[0]), float(fields[1]) )

def str2bool(v):
  return v.lower() in ("yes", "true", "t", "1")

class VolumetricPlace:
    def __init__(self, pl_id, name, img, origin, res):
        self.__pl_id__  = pl_id
        self.__eimg__ = ei.EmbeddedImage(img, origin, res)
        self.__name__  = name

    def update(self, pl_id, name, img, origin, res):
        self.__pl_id__  = pl_id
        self.__eimg__ = ei.EmbeddedImage(img, origin, res)
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
    def __init__(self, pl_id, name, pt, n, face_place):
        self.__pl_id__  = pl_id
        self.__name__  = name
        self.__pt__ = pt
        self.__n__ = n
        self.__face_place__ = face_place

    def update(self, pl_id, name, pt, n, face_place):
        self.__pl_id__  = pl_id
        self.__name__  = name
        self.__pt__ = pt
        self.__n__ = n
        self.__face_place__ = face_place 

    def getType(self):
        return 'point'

    def isDestinationFace(self):
        return self.__face_place__

    def getId(self):
        return self.__pl_id__

    def getName(self):
        return self.__name__

    def getPt(self):
        return self.__pt__

    def getN(self):
        return self.__n__

    def getTransformed(self, tf):
        tf_pt = transforms.transformPoint(self.__pt__, tf)
        tf_n = transforms.transformVector(self.__n__, tf)
        return PointPlace( self.__pl_id__, self.__name__, tf_pt, tf_n, self.__face_place__)

class MapContext:
    def __init__(self, name, resolution, origin, map_img):
        self.__name__ = name
        self.__map__ = ei.EmbeddedImage(map_img, origin, resolution)
        self.__volumetric_mask__ = None
        self.__volumetric_places__ = []
        self.__point_places__ = []

    def getName(self):
        return self.__name__

    def setVolumetricMask(self, img):
        if self.__volumetric_mask__ is None:
            assert img.shape == self.__map__.getImage().shape
            self.__volumetric_mask__ = ei.EmbeddedImage(img, self.__map__.getOrigin(),
                                        self.__map__.getResolution() )
        else:
            raise Exception('Method setVolumetricMask executed twice for the same MapContext object.')

    def getMap(self):
        return self.__map__

    def getVolumetricMask(self):
        return self.__volumetric_mask__

    def getVolumetricPlacesIds(self):
        result = []
        for pl in self.__volumetric_places__:
            result.append( pl.getId() )
        return result

    def getPointPlacesIds(self):
        result = []
        for pl in self.__point_places__:
            result.append( pl.getId() )
        return result

    def getPlacesIds(self):
        return self.getPointPlacesIds() + self.getVolumetricPlacesIds()

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
    
        self.__volumetric_places__.append(
                VolumetricPlace(pl_id, name, img, self.__map__.getOrigin(), self.__map__.getResolution()) )

    def addPointPlace(self, pl_id, name, position, front_vec, face_place):
        if not self.getPlaceById( pl_id ) is None:
            raise Exception('Two places with the same id: "' + pl_id + '"')
        if not self.getPlaceByName( name ) is None:
            raise Exception('Two places with the same name: "' + name + '"')
        self.__point_places__.append( PointPlace(pl_id, name, position, front_vec, face_place) )

    def updatePointPlace(self, pl_id, name,position, front_vec):
        if self.getPlaceById( pl_id ) is None:
            raise Exception('There are no such place. Cannot remove it. Place_id: "' + pl_id + '"')
        for pl in self.__point_places__:
            if pl.getId() == pl_id:
                face_place = pl.isDestinationFace()
                pl.update(pl_id, name, position, front_vec, face_place)

    def updateVolumetricPlace(self, pl_id, name, img):
        assert img.shape == self.__map__.getImage().shape
        if self.getPlaceById( pl_id ) is None:
            raise Exception('There are no such place. Cannot remove it. Place_id: "' + pl_id + '"')
        for pl in self.__volumetric_places__:
            if pl.getId() == pl_id:
                pl.update(pl_id, name, img, self.__map__.getOrigin(), self.__map__.getResolution()) 

class KBPlaces:
    def __init__(self):
        self.__map_contexts__ = []
        self.__map_transforms__ = []

    def addMapContext(self, mc):
        self.__map_contexts__.append( mc )

    def addMapTransform(self, mt):
        self.__map_transforms__.append( mt )

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

    def getVolumetricPlacesIds(self):
        result = []
        for mc in self.__map_contexts__:
            result = result + mc.getVolumetricPlacesIds()
        return result

    def getPointPlacesIds(self):
        result = []
        for mc in self.__map_contexts__:
            result = result + mc.getPointPlacesIds()
        return result

    def getPlacesIds(self):
        return self.getPointPlacesIds() + self.getVolumetricPlacesIds()

    def whatIsAt(self, pt, mc_name):
        pl_id_list = self.getVolumetricPlacesIds()
        for pl_id in pl_id_list:
            pl = self.getPlaceById(pl_id, mc_name)
            assert pl.getType() == 'volumetric'
            if pl.getEmbeddedImage().getValueAt(pt) > 0.5:
                return pl_id
        return None

    def getClosestPointOfPlace(self, pt_start, pl_id, mc_name, dbg_output_path=None):
        pl = self.getPlaceById(pl_id, mc_name)
        mask = self.getMask(mc_name)

        res = mask.getResolution()
        orig = mask.getOrigin()

        # Shrink the mask
        radius_m = 0.4
        radius_px = int( radius_m / res)
        op_enl = iop.OperationEnlarge(radius_px, 0)
        img_mask = iop.binaryImage( mask.getImage(), 0.5 )
        img_mask_shrinked = op_enl.compute({'img':img_mask})['img']

        # Shrink the place
        radius_m = 0.1
        radius_px = int( radius_m / res)
        op_enl2 = iop.OperationEnlarge(radius_px, 0)
        img_pl = iop.binaryImage( pl.getEmbeddedImage().getImage(), 0.5)
        img_pl_shrinked = op_enl2.compute({'img':img_pl})['img']

        if not dbg_output_path is None:
            cv2.imwrite( dbg_output_path + '/mask.png', iop.normalizeImage(img_mask) )
            cv2.imwrite( dbg_output_path + '/mask_shrinked.png', iop.normalizeImage(img_mask_shrinked) )

        phi_emb = mask.emptyCopy()
        ei.drawCircle(phi_emb, pt_start, int( 0.1 / res), 1, -1)
        phi = 1-2*phi_emb.getImage()

        if not dbg_output_path is None:
            cv2.imwrite( dbg_output_path + '/phi.png', iop.normalizeImage(phi) )

        phi = np.ma.MaskedArray(phi, 1-img_mask_shrinked)
        start_pt_dist = skfmm.distance(phi, dx=1e-2).data
        start_pt_dist_max = np.max(start_pt_dist)
        start_pt_dist[img_mask_shrinked==0] = start_pt_dist_max+1

        if not dbg_output_path is None:
            cv2.imwrite( dbg_output_path + '/start_pt_dist.png', iop.normalizeImage(start_pt_dist) )

        pl_dist_img = np.multiply( start_pt_dist, img_pl_shrinked )
        pl_dist_img[img_mask_shrinked==0] = start_pt_dist_max+1
        pl_dist_img[img_pl_shrinked==0] = start_pt_dist_max+1

        if not dbg_output_path is None:
            cv2.imwrite( dbg_output_path + '/pl_dist_img.png', iop.normalizeImage(pl_dist_img) )

        iy,ix = np.unravel_index(np.argmin(pl_dist_img, axis=None), pl_dist_img.shape)
        cv2.circle(img_mask, (ix,iy), 5, 2, thickness=1)

        if not dbg_output_path is None:
            cv2.imwrite( dbg_output_path + '/img_mask_goal.png', iop.normalizeImage(img_mask) )

        return ( ix * res + orig[0], iy * res + orig[1])

class PlacesXmlParser:
    def __init__(self, filename):
        xml_str = None
        with open(filename, "r") as f:
            xml_str = f.read()
        dom = minidom.parseString(xml_str)

        head_tail = os.path.split(filename)
        self.__cwd__ = head_tail[0]
        assert self.__cwd__

        self.kb_places = KBPlaces()

        for n in dom.childNodes:
            if n.nodeType == n.ELEMENT_NODE:
                if n.tagName == "places":
                    self.parsePlaces( n )

    def getKB(self):
        return self.kb_places

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
        self.kb_places.addMapContext( mc )

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
        name_str = xml.getAttribute("name").encode('utf-8').decode('utf-8')
        print 'parseVolumetricPlace', id_str, name_str
        filename_str = xml.getAttribute("filename")

        img = cv2.imread(self.__cwd__ + '/' + filename_str, cv2.IMREAD_GRAYSCALE)
        img = np.flipud(img)
        mc.addVolumetricPlace(id_str, name_str, img)

    def parsePointPlace( self, xml, mc ):
        # <point_place id="fotel" name="fotel" position="-0.64 0.34" front_vec="0 -1" />
        assert xml.tagName == "point_place"
        id_str = xml.getAttribute("id")
        name_str = xml.getAttribute("name").encode('utf-8').decode('utf-8')
        position_str = xml.getAttribute("position")
        front_vec_str = xml.getAttribute("front_vec")
        face_place_str = xml.getAttribute("face_place")

        pos = parsePointStr( position_str )
        vec_n = parsePointStr( front_vec_str )
        vec_n_len = math.sqrt(vec_n[0]**2 + vec_n[1]**2)
        vec_n = (vec_n[0]/vec_n_len, vec_n[1]/vec_n_len)
        face_place = str2bool(face_place_str)
        mc.addPointPlace(id_str, name_str, pos, vec_n, face_place)

    def parseMapTransform(self, xml):
        # <map_transform map1="sim" map2="real" translation="-1.87296056747 -0.53914129734" rotation="-0.0821140241595" />
        assert xml.tagName == "map_transform"

        map1_str = xml.getAttribute("map1")
        map2_str = xml.getAttribute("map2")
        translation_str = xml.getAttribute("translation")
        rotation_str = xml.getAttribute("rotation")

        transl = parsePointStr( translation_str )
        rot = float(rotation_str)

        self.kb_places.addMapTransform( (map1_str, map2_str, transl, rot) )

#
# Tests
#

def test_PlacesXML(output_path, places_xml_filename):
    print 'Parsing file "' + places_xml_filename + '"'

    places = PlacesXmlParser(places_xml_filename).getKB()

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
        pl_korytarz_a = places.getPlaceById('korytarz', mc_name)

        eimg_kuchnia = pl_kuchnia.getEmbeddedImage()
        eimg_korytarz_a = pl_korytarz_a.getEmbeddedImage()

        # Draw places over the map
        eimg_map = places.getMapContext(mc_name).getMap()

        i_add1 = ei.multI( ei.addI(eimg_map, ei.addI(eimg_kuchnia, eimg_korytarz_a)), 0.3)
        i_add2 = ei.multI( ei.addI(eimg_kuchnia, ei.addI(eimg_map, eimg_korytarz_a)), 0.3)
        i_add3 = ei.multI( ei.addI(eimg_korytarz_a, ei.addI(eimg_kuchnia, eimg_map)), 0.3)

        eimg_mask = places.getMask(mc_name)
        i_add4 = ei.multI( ei.addI(eimg_mask, eimg_map), 0.3)
        i_add5 = ei.multI( ei.addI(eimg_map, eimg_mask), 0.3)

        for im in [i_add1, i_add2, i_add3]:
            drawPointPlace(im, pl_fotel, 255)
            drawPointPlace(im, pl_drzwi, 255)

        cv2.imwrite(output_path + '/' + mc_name + '_map1.png', i_add1.getImage())
        cv2.imwrite(output_path + '/' + mc_name + '_map2.png', i_add2.getImage())
        cv2.imwrite(output_path + '/' + mc_name + '_map3.png', i_add3.getImage())
        cv2.imwrite(output_path + '/' + mc_name + '_map4.png', i_add4.getImage())
        cv2.imwrite(output_path + '/' + mc_name + '_map5.png', i_add5.getImage())

    test_places = [
        ('korytarz', (2.9, 2.3)),
        ('przesmyk', (3.65, 6.5)),
        ('kuchnia', (2.9, -0.6)),
        ('salon', (0.0, 0.0)),
        ('pokoj', (3.0, 5.0)),
        ('warsztat', (2.0, 8.5)),
        ]

    '''
    tf = places.getMapsTransform('sim', 'real')
    for px in np.linspace(-10, 10, 11):
        for py in np.linspace(-10, 10, 11):
            pt = (px, py)
            pt_tf = transforms.transformPoint( pt, tf )
            result_place_id_sim = places.whatIsAt( pt, 'sim' )
            result_place_id_real = places.whatIsAt( pt_tf, 'real' )
            #print px, py, result_place_id_sim, result_place_id_real
            assert result_place_id_sim == result_place_id_real
    '''

    # Lookup in the 'sim' map context
    for place_id, place_pos in test_places:
        result_place_id = places.whatIsAt(place_pos, 'sim')
        #print result_place_id, place_id
        assert result_place_id == place_id

    place_id = 'kuchnia'
    closest_pt = places.getClosestPointOfPlace( (0, 0), place_id, 'sim')
    print 'closest_pt to place "' + place_id + '": ' + str(closest_pt)


    id_list = places.getPlacesIds()
    for place_id in id_list:
        pl = places.getPlaceById(place_id, 'sim')
        name = pl.getName()
        print type(name), name
